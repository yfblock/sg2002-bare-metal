//! 等时（Isochronous）IN：`IsochInEp` 端点句柄 —— 在**下一微帧**调度通道 1，
//! 支持 HS 高带宽（mult 1..=3）。

use tock_registers::interfaces::Readable;

use crate::arch::cache;
use crate::drivers::usb::error::{UsbError, UsbResult};
use super::regs::HCINT;
use super::channel::{Channel, next_uframe_oddfrm};
use super::regs::{HCCHAR, HCTSIZ};
use super::dma::{dma_ptr, UVC_BULK_DMA_CAP};
use tock_registers::fields::FieldValue;

/// `wMaxPacketSize` 原始值 → 低 11 位（每事务最大字节数）。
#[inline]
pub fn wmax_mps(mps_raw: u16) -> u32 {
    u32::from(mps_raw & 0x7FF)
}

/// `wMaxPacketSize` 原始值 → mult（高带宽事务数，1..=3）。
#[inline]
pub fn wmax_mult(mps_raw: u16) -> u32 {
    (u32::from((mps_raw >> 11) & 0x3)) + 1
}

/// 每微帧总吞吐 = mps × mult。
#[inline]
pub fn wmax_payload_per_uframe(mps_raw: u16) -> u32 {
    wmax_mps(mps_raw).saturating_mul(wmax_mult(mps_raw))
}

/// 等时（Isochronous）IN 视频端点句柄：绑定设备地址 / 端点号 / `wMaxPacketSize`。
#[derive(Clone, Copy)]
pub struct IsochInEp {
    dev: u32,
    ep_num: u32,
    mps_raw: u16,
}

impl IsochInEp {
    /// 绑定一条 Isoch IN 端点。
    ///
    /// # 参数
    /// - `dev`：设备 USB 地址。
    /// - `ep_num`：端点号（`bEndpointAddress & 0x0F`）。
    /// - `mps_raw`：端点描述符中的 `wMaxPacketSize` 原值（低 11 位为每事务字节数，
    ///   bit12..11 为高带宽倍数减一）。
    #[inline]
    pub fn new(dev: u32, ep_num: u8, mps_raw: u16) -> Self {
        Self { dev, ep_num: u32::from(ep_num), mps_raw }
    }

    /// 本端点的 HCCHAR(IN 方向;mult 高带宽事务数)。
    fn hcchar(&self, mps: u32, mult: u32) -> FieldValue<u32, HCCHAR::Register> {
        HCCHAR::MPS.val(mps)
            + HCCHAR::EPNUM.val(self.ep_num)
            + HCCHAR::DEVADDR.val(self.dev)
            + HCCHAR::EPTYPE::Isochronous
            + HCCHAR::MC.val(mult.clamp(1, 3))
            // 等时 IN:方向恒 IN(视频流)
            + HCCHAR::EPDIR::SET
    }

    /// 在 **下一微帧** 启动一次通道，最多接收 `mult` 个 USB 事务（每个 ≤ `mps` 字节）。
    ///
    /// 返回本次实际收到的字节数（0 表示设备本微帧无数据 / 0-byte 包）。
    ///
    /// **PID 编码（DWC2）**：单事务 DATA0；双事务 DATA1；三事务 DATA2。
    /// **MC**：写入 `HCCHAR.MC` = `mult`。
    /// **ODDFRM**：根据 `HFNUM` 选择下个微帧的奇偶。
    ///
    /// # 参数
    /// - `dma_off`：本微帧接收缓冲在内部 DMA 窗口中的起始偏移。
    pub fn read_uframe(&self, dma_off: usize) -> UsbResult<usize> {
        let mps = wmax_mps(self.mps_raw);
        let mult = wmax_mult(self.mps_raw);
        if mps == 0 || mult > 3 {
            return Err(UsbError::Protocol("bad isoch mps_raw"));
        }
        let xfersize = mps.saturating_mul(mult);
        if (xfersize as usize) > UVC_BULK_DMA_CAP {
            return Err(UsbError::Protocol("isoch xfer > dma cap"));
        }
        let pid = match mult {
            3 => HCTSIZ::PID::Data2,
            2 => HCTSIZ::PID::Data1,
            _ => HCTSIZ::PID::Data0,
        };
        let pktcnt = mult;

        unsafe {
            let tsiz = pid + HCTSIZ::PKTCNT.val(pktcnt) + HCTSIZ::XFERSIZE.val(xfersize);
            let oddfrm = next_uframe_oddfrm();

            let ch = Channel::VIDEO;
            ch.arm(
                tsiz,
                (self.hcchar(mps, mult) + oddfrm + HCCHAR::CHENA::SET).value,
                dma_off as u32,
            )?;

            let st = ch.wait_halted()?;
            if st.is_set(HCINT::STALL) {
                return Err(UsbError::Stall);
            }
            if st.is_set(HCINT::AHBERR) {
                return Err(UsbError::Hardware("AHBERR on isoch"));
            }
            if st.is_set(HCINT::FRMOVRN)
                || st.is_set(HCINT::XACTERR)
                || st.is_set(HCINT::BBLERR)
                || st.is_set(HCINT::DATATGLERR)
                || st.is_set(HCINT::NYET)
                || st.is_set(HCINT::NAK)
            {
                return Ok(0);
            }
            if !st.is_set(HCINT::XFERCOMPL) {
                return Ok(0);
            }
            let rem = ch.chan_regs().hctsiz.read(HCTSIZ::XFERSIZE);
            let actual = xfersize.saturating_sub(rem) as usize;
            if actual > 0 {
                cache::dcache_invalidate_after_dma(dma_ptr().add(dma_off), actual);
            }
            Ok(actual)
        }
    }
}
