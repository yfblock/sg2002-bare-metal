//! Isoch IN 抓帧与帧组装：按 FID 翻转/EOF 判帧界，MJPEG 负载按 SOI/EOI 校验；
//! 含微帧级调试 trace。

use crate::drivers::usb::dwc2;
use crate::drivers::usb::dwc2::{DMA_OFF_UVC_BULK, UVC_BULK_DMA_CAP};
use crate::drivers::usb::error::{UsbError, UsbResult};

use super::session::UvcCamera;

/// 单微帧 RX 工作区大小：HS Isoch 单微帧最多 1024×3=3072 字节，4096 已够用；
/// 其余缓冲全部留给拼接 JPEG。
pub const UVC_WORK_AREA_BYTES: usize = 4096;
pub const UVC_ASSEMBLED_JPEG_DMA_OFF: usize = DMA_OFF_UVC_BULK + UVC_WORK_AREA_BYTES;

/// UVC 数据包(等时 IN 负载)的视图:`bLength@0` + `bInfo@1` + 负载 `@hlen..`。
/// 构造即校验头长度;字段访问器自带位语义文档。
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

enum FrameState {
    /// 等待首次 FID 翻转（丢弃当前不完整帧的尾巴）。
    WaitFirstSwitch { last_fid: Option<u8> },
    /// 已锁定 frame_fid，开始累积；遇 EOF 或 fid 翻转都视为帧结束。
    Capturing { frame_fid: u8, saw_data: bool },
}

/// 跨 capture 持久化的「上次 EOF 帧的 FID」。
/// 0xFF = 还没抓过；其它值 = 0/1。后续 capture 直接以 `WaitFirstSwitch { last_fid: Some(..) }`
/// 开始，免去等到下一次完整翻转的 ~半~一个帧周期。
static LAST_EOF_FID: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0xFF);

fn process_packet(
    pkt: &[u8],
    state: &mut FrameState,
    jpeg_len: &mut usize,
    jpeg_cap: usize,
) -> UsbResult<bool> {
    let Some(p) = UvcPacket::new(pkt) else {
        return Ok(false); // 头非法:丢弃
    };
    if let FrameState::WaitFirstSwitch { last_fid } = state {
        match *last_fid {
            // 还没见过任何 FID：记住当前值，等下一次翻转。
            None => {
                *last_fid = Some(p.fid());
                return Ok(false);
            }
            // FID 翻转 = 新帧开始：转入 Capturing，本包继续处理。
            Some(prev) if prev != p.fid() => {
                *state = FrameState::Capturing {
                    frame_fid: p.fid(),
                    saw_data: false,
                };
            }
            _ => return Ok(false),
        }
    }
    process_packet_capturing(state, &p, jpeg_len, jpeg_cap)
}

/// 检查已累积 JPEG 的末 2 字节是否为 EOI(`ff d9`)；DMA 读失败视为不是。
fn tail_is_eoi(len: usize) -> bool {
    len >= 2
        && dwc2::dma_rx_slice(UVC_ASSEMBLED_JPEG_DMA_OFF + len - 2, 2)
            .map(|t| t == [0xff, 0xd9])
            .unwrap_or(false)
}

/// Capturing 状态的单包 MJPEG 帧组装;仅由 [`process_packet`] 在确认状态后调用。
fn process_packet_capturing(
    state: &mut FrameState,
    p: &UvcPacket<'_>,
    jpeg_len: &mut usize,
    jpeg_cap: usize,
) -> UsbResult<bool> {
    let FrameState::Capturing {
        frame_fid,
        saw_data,
    } = state
    else {
        unreachable!()
    };
    let payload = p.payload();
    if p.fid() != *frame_fid {
        // 廉价 webcam 的"元数据帧"陷阱:带 SOI 但无 EOI,FID 也会翻转——
        // 要求 EOI(`ff d9`)真实存在才认为帧完整,否则丢弃重新开始。
        if *saw_data && tail_is_eoi(*jpeg_len) {
            return Ok(true);
        }
        // 残帧(无 EOI):丢弃累积,当前 packet 作为新帧首包。
        *jpeg_len = 0;
        *frame_fid = p.fid();
        *saw_data = false;
    }
    if !payload.is_empty() {
        // 首次累积须以 SOI(`ff d8`) 开头:摄像头帧间有 padding packet(同 FID
        // 但无 SOI),直接累积会产出首字节非 ff d8 的截断帧——跳过等真 SOI。
        if !*saw_data && !payload.starts_with(&[0xff, 0xd8]) {
            return Ok(false);
        }
        if jpeg_len.checked_add(payload.len()).unwrap_or(usize::MAX) > jpeg_cap {
            return Err(UsbError::Hardware("video assemble overflow"));
        }
        dwc2::dma_write_at(UVC_ASSEMBLED_JPEG_DMA_OFF + *jpeg_len, payload)?;
        *jpeg_len += payload.len();
        *saw_data = true;
    }
    if p.eof() {
        // EOF 同样校验 EOI:metadata 帧带合法 EOF 标记但 JPEG 仅有 SOI。
        if *saw_data && tail_is_eoi(*jpeg_len) {
            return Ok(true);
        }
        // 残帧(带 EOF 但无 EOI):丢弃累积,回到 WaitFirstSwitch 干净开始。
        *jpeg_len = 0;
        *saw_data = false;
        *state = FrameState::WaitFirstSwitch {
            last_fid: Some(p.fid()),
        };
        return Ok(false);
    }
    Ok(false)
}

impl UvcCamera {
    /// 抓一帧（视频负载组装至 [`UVC_ASSEMBLED_JPEG_DMA_OFF`]）。
    ///
    /// **关键**：等时模式下 `mult=1` 时，每次 `IsochInEp::read_uframe` 返回的整个数据（最多 mps 字节）就是
    /// **一个完整的 USB 包 = 一个 UVC 数据包**（带 12 字节头），**不可再切分**。
    pub fn capture_frame(&self) -> UsbResult<usize> {
        let iso = dwc2::IsochInEp::new(self.control_ep.dev(), self.sel.ep_num, self.sel.mps_raw);
        let mps_low = dwc2::wmax_mps(self.sel.mps_raw).max(1) as usize;
        let mult = dwc2::wmax_mult(self.sel.mps_raw).clamp(1, 3) as usize;
        let jpeg_cap = UVC_BULK_DMA_CAP.saturating_sub(UVC_WORK_AREA_BYTES);
        let mut jpeg_len = 0usize;
        let mut transfers = 0u32;
        let mut data_transfers = 0u32;
        let work_off = DMA_OFF_UVC_BULK;
        let prev_eof_fid = LAST_EOF_FID.load(core::sync::atomic::Ordering::Relaxed);
        let mut state = FrameState::WaitFirstSwitch {
            last_fid: if prev_eof_fid <= 1 {
                Some(prev_eof_fid)
            } else {
                None
            },
        };
        const MAX_UFRAMES: u32 = 80_000;
        for _ in 0..MAX_UFRAMES {
            transfers = transfers.wrapping_add(1);
            let actual = iso.read_uframe(work_off)?;
            if actual == 0 {
                continue;
            }
            data_transfers = data_transfers.wrapping_add(1);
            let slice =
                dwc2::dma_rx_slice(work_off, actual).ok_or(UsbError::Hardware("dma view"))?;

            let eof = if mult == 1 {
                process_packet(slice, &mut state, &mut jpeg_len, jpeg_cap)?
            } else {
                // mult>1：一次 read_uframe 的 DMA 数据里可能含多个 USB 包，按 mps 切开逐包处理。
                let mut hit_eof = false;
                for pkt in slice.chunks(mps_low) {
                    if process_packet(pkt, &mut state, &mut jpeg_len, jpeg_cap)? {
                        hit_eof = true;
                        break;
                    }
                }
                hit_eof
            };
            if eof {
                if let FrameState::Capturing { frame_fid, .. } = state {
                    LAST_EOF_FID.store(frame_fid, core::sync::atomic::Ordering::Relaxed);
                }
                return Ok(jpeg_len);
            }
        }
        log::info!(
            "UVC: capture timeout after {} uframes ({} data; {} bytes assembled, mult={})",
            transfers,
            data_transfers,
            jpeg_len,
            mult
        );
        Err(UsbError::Timeout)
    }
}
