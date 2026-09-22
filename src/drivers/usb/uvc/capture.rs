//! Isoch IN 抓帧与帧组装：等 FID 翻转锁定帧头，逐包追加 payload，
//! EOF/FID 翻转时以 EOI(`ff d9`)校验帧完整性；残帧丢弃重试。

use crate::drivers::usb::dwc2;
use crate::drivers::usb::dwc2::{IsochInEp, DMA_OFF_UVC_BULK, UVC_BULK_DMA_CAP};
use crate::drivers::usb::error::{UsbError, UsbResult};
use core::sync::atomic::Ordering;

use super::session::UvcCamera;

/// 单微帧 RX 工作区大小：HS Isoch 单微帧最多 1024 字节(mult=1),4096 已够;
/// 其余缓冲全部留给拼接 JPEG。
pub const UVC_WORK_AREA_BYTES: usize = 4096;
pub const UVC_ASSEMBLED_JPEG_DMA_OFF: usize = DMA_OFF_UVC_BULK + UVC_WORK_AREA_BYTES;

/// 上限次数(等翻转 + 攒帧各 80k 微帧,共 ~160ms × 2)。

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

    /// 本包是否属于 FID 为 `fid` 的帧(同帧 true,翻转 false)。
    fn is_fid(&self, fid: u8) -> bool {
        self.fid() == fid
    }
}

/// 跨 capture 持久化的「上次完整帧的 FID」。0xFF = 还没抓过。
static LAST_EOF_FID: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0xFF);

/// 已累积 JPEG 的末 2 字节是否为 EOI(`ff d9`)——帧完整性判定。
fn tail_is_eoi(jpeg_len: usize) -> bool {
    dwc2::dma_rx_slice(UVC_ASSEMBLED_JPEG_DMA_OFF + jpeg_len.saturating_sub(2), 2)
        .is_some_and(|t| t == [0xff, 0xd9])
}

/// 追加 payload 到帧缓冲。首笔须以 SOI(`ff d8`) 开头(padding 包跳过)。
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
    /// **残帧陷阱**(0c45:64ab 等廉价 webcam):帧间插入"元数据帧"——带
    /// SOI 但无 EOI(典型 1008 字节),FID 翻转/EOF 均合法。所有帧结束
    /// 判定都要求 EOI 真实存在,否则丢弃重试。
    pub fn capture_frame(&self) -> UsbResult<usize> {
        let iso = IsochInEp::new(self.control_ep.dev(), self.sel.ep_num, self.sel.mps_raw);
        let jpeg_cap = UVC_BULK_DMA_CAP.saturating_sub(UVC_WORK_AREA_BYTES);
        let work_off = DMA_OFF_UVC_BULK;
        let init_fid = LAST_EOF_FID.load(core::sync::atomic::Ordering::Relaxed);
        let init_fid = (init_fid <= 1).then_some(init_fid);

        // 每轮 = 等一个完整帧;残帧(垃圾)→ 重试;4 轮未成 → 超时。
        for _ in 0..4 {
            match Self::try_frame(&iso, work_off, init_fid, jpeg_cap) {
                Ok(len) => return Ok(len),
                Err(UsbError::Timeout) => break, // 等翻转或读包超时,不再重试
                Err(e) => return Err(e),         // 硬错误(STALL/溢出)上抛
            }
        }
        log::info!("UVC: capture timeout");
        Err(UsbError::Timeout)
    }

    /// 等一个完整帧:等 FID 翻转 → 攒帧 → EOI 校验。
    /// 残帧或超时 → Err(Timeout)(调用方决定重试);硬错误 → Err 上抛。
    fn try_frame(
        iso: &IsochInEp,
        work_off: usize,
        init_fid: Option<u8>,
        jpeg_cap: usize,
    ) -> UsbResult<usize> {
        // ── 阶段1:等 FID 翻转──
        let mut prev_fid = init_fid;
        let first = loop {
            let Some(p) = UvcPacket::new(iso.read_payload(work_off)?) else {
                continue; // 头非法:跳过,读下一个包
            };
            match prev_fid {
                None => prev_fid = Some(p.fid()),
                Some(prev) if prev != p.fid() => break p, // 翻转!本包即首包
                _ => {}
            }
        };

        // ── 阶段2:攒帧──
        let frame_fid = first.fid();
        let mut jpeg_len = 0usize;
        let mut saw_data = false;
        let mut fid = frame_fid;

        // 首包:刚翻转,直接追加(不进循环,消 first_done flag)。
        append(&mut jpeg_len, &mut saw_data, first.payload(), jpeg_cap)?;
        if first.eof() {
            if saw_data && tail_is_eoi(jpeg_len) {
                LAST_EOF_FID.store(frame_fid, Ordering::Relaxed);
                return Ok(jpeg_len);
            }
            return Err(UsbError::Timeout); // 单包残帧
        }

        // 后续包:逐个读、追加、判帧结束。
        loop {
            let Some(p) = UvcPacket::new(iso.read_payload(work_off)?) else {
                continue; // 头非法:跳过,读下一个包
            };

            // FID 翻转:上一帧可能完整(EOI 在)?
            if !p.is_fid(fid) {
                if saw_data && tail_is_eoi(jpeg_len) {
                    LAST_EOF_FID.store(frame_fid, Ordering::Relaxed);
                    return Ok(jpeg_len);
                }
                // 残帧:丢弃重开
                (jpeg_len, fid, saw_data) = (0, p.fid(), false);
            }

            append(&mut jpeg_len, &mut saw_data, p.payload(), jpeg_cap)?;

            // EOF + EOI = 帧完整;EOF 无 EOI = 残帧
            if p.eof() {
                if saw_data && tail_is_eoi(jpeg_len) {
                    LAST_EOF_FID.store(frame_fid, Ordering::Relaxed);
                    return Ok(jpeg_len);
                }
                return Err(UsbError::Timeout); // 残帧:视为本轮超时,让调用方重试
            }
        }
    }
}
