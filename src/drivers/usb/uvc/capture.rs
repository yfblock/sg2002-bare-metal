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
    /// 已锁定 frame_fid，开始累积;`jpeg_len` 随状态走(不再独立传参,
    /// 消除满屏 `*jpeg_len` 解引用)。遇 EOF 或 fid 翻转都视为帧结束。
    Capturing {
        frame_fid: u8,
        saw_data: bool,
        jpeg_len: usize,
    },
}

impl FrameState {
    /// 处理一个 UVC 包;命中帧结束返回 true。
    fn process_packet(&mut self, p: &UvcPacket<'_>, jpeg_cap: usize) -> UsbResult<bool> {
        match self {
            FrameState::WaitFirstSwitch { last_fid } => match *last_fid {
                // 还没见过任何 FID:记住当前值,等下一次翻转。
                None => {
                    *last_fid = Some(p.fid());
                    Ok(false)
                }
                // 同一 FID:还在帧尾残留里,跳过。
                Some(prev) if prev == p.fid() => Ok(false),
                // FID 翻转 = 新帧开始:转入 Capturing,本包继续处理。
                Some(_) => {
                    *self = FrameState::Capturing {
                        frame_fid: p.fid(),
                        saw_data: false,
                        jpeg_len: 0,
                    };
                    self.process_packet_capturing(p, jpeg_cap)
                }
            },
            FrameState::Capturing { .. } => self.process_packet_capturing(p, jpeg_cap),
        }
    }

    // process_packet_capturing 的原文,改为方法
    /// Capturing 状态的单包 MJPEG 帧组装;仅由 [`Self::process_packet`] 在确认状态后调用。
    ///
    /// **残帧陷阱**(0c45:64ab 等廉价 webcam):帧间会插入"元数据帧"——带 SOI
    /// 但无 EOI(典型 1008 字节),FID 也会翻转、EOF 标记也可能合法。所有
    /// "帧结束"判定都要求 EOI(`ff d9`)真实存在,否则丢弃重新开始。
    fn process_packet_capturing(&mut self, p: &UvcPacket<'_>, jpeg_cap: usize) -> UsbResult<bool> {
        // 解构为本地可变引用——后续代码不再满屏 *xxx
        let Self::Capturing {
            frame_fid: fid,
            saw_data,
            jpeg_len,
        } = self
        else {
            unreachable!()
        };

        // FID 翻转:上一帧可能完整(EOI 存在)→返回;或残帧→丢弃重开。
        if p.fid() != *fid {
            if *saw_data && Self::tail_is_eoi(*jpeg_len) {
                return Ok(true); // 帧完整
            }
            (*jpeg_len, *fid, *saw_data) = (0, p.fid(), false); // 重开
        }

        let payload = p.payload();
        if !payload.is_empty() {
            // 首次累积须以 SOI(ff d8) 开头(padding packet 跳过)。
            if !*saw_data && !payload.starts_with(&[0xff, 0xd8]) {
                return Ok(false);
            }
            if *jpeg_len + payload.len() > jpeg_cap {
                return Err(UsbError::Hardware("video assemble overflow"));
            }
            dwc2::dma_write_at(UVC_ASSEMBLED_JPEG_DMA_OFF + *jpeg_len, payload)?;
            *jpeg_len += payload.len();
            *saw_data = true;
        }

        // EOF:帧结束的正式信号,但残帧也可能带 EOF——EOI 说了算。
        if p.eof() && *saw_data && Self::tail_is_eoi(*jpeg_len) {
            return Ok(true); // 帧完整
        }
        if p.eof() {
            // 残帧(带 EOF 但无 EOI):丢弃,回 WaitFirstSwitch 干净开始。
            *self = FrameState::WaitFirstSwitch {
                last_fid: Some(p.fid()),
            };
        }
        Ok(false)
    }

    /// 已累积 JPEG 的末 2 字节是否为 EOI(`ff d9`)——帧完整性判定。
    /// DMA 读失败(含 len<2 经 saturating_sub 落在窗口外的场景)视为不是。
    fn tail_is_eoi(jpeg_len: usize) -> bool {
        dwc2::dma_rx_slice(UVC_ASSEMBLED_JPEG_DMA_OFF + jpeg_len.saturating_sub(2), 2)
            .is_some_and(|t| t == [0xff, 0xd9])
    }
}

/// 跨 capture 持久化的「上次 EOF 帧的 FID」。
/// 0xFF = 还没抓过；其它值 = 0/1。后续 capture 直接以 `WaitFirstSwitch { last_fid: Some(..) }`
/// 开始，免去等到下一次完整翻转的 ~半~一个帧周期。
static LAST_EOF_FID: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0xFF);

impl UvcCamera {
    /// 抓一帧（视频负载组装至 [`UVC_ASSEMBLED_JPEG_DMA_OFF`]）。
    ///
    /// 每次 `read_uframe` 返回的整个数据就是**一个完整的 UVC 数据包**(带 12 字节头),
    /// 不可再切分。SG2002 DWC2 只支持 mult=1 等时。
    fn process_packets(slice: &[u8], state: &mut FrameState, jpeg_cap: usize) -> UsbResult<bool> {
        match UvcPacket::new(slice) {
            Some(p) => state.process_packet(&p, jpeg_cap),
            None => Ok(false),
        }
    }

    pub fn capture_frame(&self) -> UsbResult<usize> {
        let iso = dwc2::IsochInEp::new(self.control_ep.dev(), self.sel.ep_num, self.sel.mps_raw);
        let jpeg_cap = UVC_BULK_DMA_CAP.saturating_sub(UVC_WORK_AREA_BYTES);
        let mut transfers = 0u32;
        let mut data_transfers = 0u32;
        let work_off = DMA_OFF_UVC_BULK;
        let prev_eof_fid = LAST_EOF_FID.load(core::sync::atomic::Ordering::Relaxed);
        let mut state = FrameState::WaitFirstSwitch {
            last_fid: (prev_eof_fid <= 1).then_some(prev_eof_fid),
        };
        const MAX_UFRAMES: u32 = 80_000;
        for _ in 0..MAX_UFRAMES {
            transfers += 1;
            let actual = iso.read_uframe(work_off)?;
            if actual == 0 {
                continue;
            }
            data_transfers += 1;
            let slice =
                dwc2::dma_rx_slice(work_off, actual).ok_or(UsbError::Hardware("dma view"))?;

            if Self::process_packets(slice, &mut state, jpeg_cap)? {
                if let FrameState::Capturing {
                    frame_fid,
                    jpeg_len,
                    ..
                } = state
                {
                    LAST_EOF_FID.store(frame_fid, core::sync::atomic::Ordering::Relaxed);
                    return Ok(jpeg_len);
                }
            }
        }
        let jpeg_len = match state {
            FrameState::Capturing { jpeg_len, .. } => jpeg_len,
            _ => 0,
        };
        log::info!(
            "UVC: capture timeout after {} uframes ({} data; {} bytes assembled)",
            transfers,
            data_transfers,
            jpeg_len,
        );
        Err(UsbError::Timeout)
    }
}
