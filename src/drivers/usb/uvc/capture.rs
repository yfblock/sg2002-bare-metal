//! Isoch IN 抓帧与帧组装：按 FID 翻转/EOF 判帧界，MJPEG 负载按 SOI/EOI 校验。

use crate::drivers::usb::dwc2;
use crate::drivers::usb::dwc2::{DMA_OFF_UVC_BULK, UVC_BULK_DMA_CAP};
use crate::drivers::usb::error::{UsbError, UsbResult};

use super::session::UvcCamera;

/// 单微帧 RX 工作区大小：HS Isoch 单微帧最多 1024 字节(mult=1),4096 已够;
/// 其余缓冲全部留给拼接 JPEG。
pub const UVC_WORK_AREA_BYTES: usize = 4096;
pub const UVC_ASSEMBLED_JPEG_DMA_OFF: usize = DMA_OFF_UVC_BULK + UVC_WORK_AREA_BYTES;

/// UVC 数据包(等时 IN 负载)的视图:`bLength@0` + `bInfo@1` + 负载 `@hlen..`。
struct UvcPacket<'a> {
    pkt: &'a [u8],
    hlen: usize,
}

impl<'a> UvcPacket<'a> {
    /// 头合法时构造;`bLength` 缺失 / <2 / 超出包长返回 `None`。
    fn new(pkt: &'a [u8]) -> Option<Self> {
        let hlen = *pkt.first()? as usize;
        if hlen < 2 || hlen > pkt.len() {
            return None;
        }
        Some(Self { pkt, hlen })
    }

    /// `bInfo` bit1:EOF(帧结束标记)。
    fn eof(&self) -> bool {
        self.pkt[1] & 0x02 != 0
    }

    /// `bInfo` bit0:FID(帧 ID,逐帧翻转)。
    fn fid(&self) -> u8 {
        self.pkt[1] & 0x01
    }

    /// 去头后的 MJPEG 负载切片。
    fn payload(&self) -> &'a [u8] {
        &self.pkt[self.hlen..]
    }
}

/// 跨 capture 持久化的「上次 EOF 帧的 FID」。
/// 0xFF = 还没抓过;其它值 = 0/1。后续 capture 直接以已知 FID 起步,
/// 免去等到下一次完整翻转的一个帧周期。
static LAST_EOF_FID: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0xFF);

/// 已累积 JPEG 的末 2 字节是否为 EOI(`ff d9`)——帧完整性判定。
fn tail_is_eoi(jpeg_len: usize) -> bool {
    dwc2::dma_rx_slice(UVC_ASSEMBLED_JPEG_DMA_OFF + jpeg_len.saturating_sub(2), 2)
        .is_some_and(|t| t == [0xff, 0xd9])
}

/// 追加 payload 到帧缓冲。首笔须以 SOI(`ff d8`) 开头(padding 包跳过);
/// 溢出返回 Err。
fn append(
    jpeg_len: &mut usize,
    saw_data: &mut bool,
    payload: &[u8],
    jpeg_cap: usize,
) -> UsbResult<()> {
    if payload.is_empty() {
        return Ok(());
    }
    if !*saw_data && !payload.starts_with(&[0xff, 0xd8]) {
        return Ok(()); // padding 包:跳过
    }
    if *jpeg_len + payload.len() > jpeg_cap {
        return Err(UsbError::Hardware("video assemble overflow"));
    }
    dwc2::dma_write_at(UVC_ASSEMBLED_JPEG_DMA_OFF + *jpeg_len, payload)?;
    *jpeg_len += payload.len();
    *saw_data = true;
    Ok(())
}

impl UvcCamera {
    /// 抓一帧(视频负载组装至 [`UVC_ASSEMBLED_JPEG_DMA_OFF`])。
    ///
    /// 每次 `read_uframe` 返回的整个数据就是**一个完整的 UVC 数据包**
    /// (带 12 字节头)。SG2002 DWC2 只支持 mult=1 等时。
    ///
    /// **残帧陷阱**(0c45:64ab 等廉价 webcam):帧间会插入"元数据帧"——
    /// 带 SOI 但无 EOI(典型 1008 字节),FID 翻转/EOF 均合法。所有
    /// "帧结束"判定都要求 EOI(`ff d9`)真实存在。
    pub fn capture_frame(&self) -> UsbResult<usize> {
        let iso = dwc2::IsochInEp::new(self.control_ep.dev(), self.sel.ep_num, self.sel.mps_raw);
        let jpeg_cap = UVC_BULK_DMA_CAP.saturating_sub(UVC_WORK_AREA_BYTES);
        let mut transfers = 0u32;
        let mut data_transfers = 0u32;
        let work_off = DMA_OFF_UVC_BULK;

        const MAX_UFRAMES: u32 = 80_000;
        let init_fid = LAST_EOF_FID.load(core::sync::atomic::Ordering::Relaxed);
        let mut jpeg_len = 0usize;

        // 外层:每次进入 = 从干净状态开始等一个完整帧。
        // 残帧(垃圾)→ continue 回来重新等;正常帧 → return。
        'frame: for _ in 0..4 {
            // ── 等 FID 翻转(本阶段只有 prev_fid,零解引用)──
            let mut prev_fid = (init_fid <= 1).then_some(init_fid);
            let first = 'wait: {
                for _ in 0..MAX_UFRAMES {
                    transfers += 1;
                    let actual = iso.read_uframe(work_off)?;
                    if actual == 0 {
                        continue;
                    }
                    data_transfers += 1;
                    let slice = dwc2::dma_rx_slice(work_off, actual)
                        .ok_or(UsbError::Hardware("dma view"))?;
                    let Some(p) = UvcPacket::new(slice) else {
                        continue;
                    };
                    match prev_fid {
                        None => prev_fid = Some(p.fid()),
                        Some(prev) if prev != p.fid() => break 'wait p,
                        _ => {}
                    }
                }
                break 'frame; // 等翻转也超时
            };

            // ── 攒帧(本阶段三个本地变量,零解引用)──
            let mut saw_data = false;
            let mut fid = first.fid();
            jpeg_len = 0;

            // 首包处理(FID 已翻转,直接追加)
            {
                let p = &first;
                if p.fid() != fid { /* 首包 fid == fid,不会进 */ }
                append(&mut jpeg_len, &mut saw_data, p.payload(), jpeg_cap)?;
                if p.eof() {
                    if saw_data && tail_is_eoi(jpeg_len) {
                        LAST_EOF_FID.store(fid, core::sync::atomic::Ordering::Relaxed);
                        return Ok(jpeg_len);
                    }
                    continue 'frame;
                }
            }

            // 后续包逐个读
            for _ in 0..MAX_UFRAMES {
                transfers += 1;
                let actual = iso.read_uframe(work_off)?;
                if actual == 0 {
                    continue;
                }
                data_transfers += 1;
                let slice =
                    dwc2::dma_rx_slice(work_off, actual).ok_or(UsbError::Hardware("dma view"))?;
                let Some(p) = UvcPacket::new(slice) else {
                    continue;
                };

                // FID 翻转:上一帧可能完整(EOI 在)?
                if p.fid() != fid {
                    if saw_data && tail_is_eoi(jpeg_len) {
                        LAST_EOF_FID.store(fid, core::sync::atomic::Ordering::Relaxed);
                        return Ok(jpeg_len);
                    }
                    // 残帧:丢弃重开
                    (jpeg_len, fid, saw_data) = (0, p.fid(), false);
                }

                append(&mut jpeg_len, &mut saw_data, p.payload(), jpeg_cap)?;

                // EOF + EOI = 帧完整;EOF 无 EOI = 残帧
                if p.eof() {
                    if saw_data && tail_is_eoi(jpeg_len) {
                        LAST_EOF_FID.store(fid, core::sync::atomic::Ordering::Relaxed);
                        return Ok(jpeg_len);
                    }
                    // 残帧:丢弃,回外层等干净帧
                    continue 'frame;
                }
            }
        }

        log::info!(
            "UVC: capture timeout after {} uframes ({} data; {} bytes assembled)",
            transfers,
            data_transfers,
            jpeg_len,
        );
        Err(UsbError::Timeout)
    }
}
