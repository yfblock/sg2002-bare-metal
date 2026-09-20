//! 主机通道原语：启停/等待 halt、NAK/XACT 有界重试、USB ISR，以及 HFNUM 时间辅助。
//!
//! 通道约定：**0 = EP0 控制**，**1 = Isoch 视频**。

use core::sync::atomic::{AtomicBool, Ordering};
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::LocalRegisterCopy;

use super::regs::{Dwc2HostChannel, GINTSTS, HCCHAR, HCINT, HCTSIZ, HFNUM};
use crate::drivers::usb;
use crate::drivers::usb::error::{UsbError, UsbResult};
use tock_registers::fields::FieldValue;

/// `HCINT` 快照（通道 halt 时读出的中断原因位，供上层区分 XFERCOMPL / NAK / STALL 等）。
pub(crate) type HcintSnapshot = LocalRegisterCopy<u32, HCINT::Register>;

/// 主机通道句柄：绑定通道索引。约定 **0 = EP0 控制**、**1 = Isoch 视频**
/// ——「一条端点 ↔ 一个硬件通道」由端点句柄（[`super::Ep0`] / [`super::IsochInEp`]）
/// 通过 [`Channel::CONTROL`] / [`Channel::VIDEO`] 选定。
#[derive(Clone, Copy)]
pub struct Channel(u32);

/// HCINT 写 1 清除：清完整 11 位（含 ACK/NYET 等）。
pub(crate) const HCINT_ALL_W1C: u32 = 0x7FF;

/// 每个通道的「传输完成」flag，由 USB ISR (`handle_usb_irq`) 置位，
/// [`Channel::wait_halted`] 每轮检查一次。下标 = 通道号（0=EP0, 1=Isoch）。
static CH_DONE: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

/// USB ISR 被调用的次数（诊断用：判断中断是否真的到达小核）。
static USB_ISR_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// 取走 USB ISR 计数（swap 清零）。
pub fn take_usb_isr_count() -> u32 {
    USB_ISR_COUNT.swap(0, Ordering::Relaxed)
}

/// USB 中断处理：由 trap handler 调用。
///
/// DWC2 中断链路：通道完成 → `HCINT.CHHLTD` → `HAINT` → `GINTSTS.HCHINT`
/// → PLIC source 30 → M-mode trap。本函数清 `HCINT` 并设 `CH_DONE` 唤醒等待者。
pub fn handle_usb_irq() {
    USB_ISR_COUNT.fetch_add(1, Ordering::Relaxed);
    let dwc2 = usb::dwc2_regs();
    if !dwc2.gintsts.is_set(GINTSTS::HCHINT) {
        return;
    }
    // HAINT：每 bit 对应一个通道的中断状态。
    let haint = dwc2.haint.get();
    for ch in 0..2u32 {
        if haint & (1 << ch) == 0 {
            continue;
        }
        let chan = usb::dwc2_channel(ch);
        let hcint = chan.hcint.extract();
        // 清掉本通道所有中断位（W1C）
        chan.hcint.set(hcint.get());
        if hcint.is_set(HCINT::CHHLTD) {
            CH_DONE[ch as usize].store(true, Ordering::Release);
        }
    }
}

pub(crate) fn spin_delay(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

pub(crate) fn usb_bus_fence_before_dma() {
    riscv::asm::fence();
}

impl Channel {
    /// EP0 控制传输通道。
    pub const CONTROL: Channel = Channel(0);
    /// Isoch 视频传输通道（与控制分离）。
    pub const VIDEO: Channel = Channel(1);

    /// 通道寄存器视图。
    #[inline]
    pub(crate) fn chan_regs(&self) -> &'static Dwc2HostChannel {
        usb::dwc2_channel(self.0)
    }

    /// 等通道空闲（`CHENA` 自清）。
    pub(crate) fn wait_disabled(&self) -> UsbResult<()> {
        let chan = self.chan_regs();
        for _ in 0..2_000_000u32 {
            if !chan.hcchar.is_set(HCCHAR::CHENA) {
                return Ok(());
            }
            spin_delay(8);
        }
        Err(UsbError::Timeout)
    }

    /// 等通道 halt。中断 flag 优先，`spin_delay` 轮询兜底。
    ///
    /// 两条路径都留着是因为 PLIC source 30 在 C906L 上触发率很低——
    /// 实测每 100 帧约 69 次 ISR，而同期有 7700 次通道传输，覆盖率不到 1%。
    /// 中断链路本身是正确的（`HCINTMSK` 已编程、无误触发），只是不足以替代轮询。
    pub(crate) fn wait_halted(&self) -> UsbResult<HcintSnapshot> {
        let chan = self.chan_regs();
        let idx = self.0 as usize;
        for _ in 0..8_000_000u32 {
            // 中断 flag（USB ISR 置 CH_DONE）优先，轮询 CHHLTD 兜底；
            // HCINT 的 W1C 只在返回路径写回一次。
            let done = CH_DONE[idx].swap(false, Ordering::AcqRel);
            let hi = chan.hcint.extract();
            if done || hi.is_set(HCINT::CHHLTD) {
                chan.hcint.set(hi.get());
                return Ok(hi);
            }
            spin_delay(8);
        }
        Err(UsbError::Timeout)
    }

    /// EP0 上对 NAK / XACTERR 做有限次重试；STALL 立即返回。
    pub(crate) unsafe fn xfer(
        &self,
        hcchar: FieldValue<u32, HCCHAR::Register>,
        hctsiz: u32,
        dma_off: u32,
    ) -> UsbResult<HcintSnapshot> {
        let chan = self.chan_regs();
        let dmap = super::dma::dma_phys(dma_off as usize);

        // EP0 control 上：NAK = 设备未就绪，自动重试；XACTERR = CRC/PID/babble，
        // 在 reset 解除后总线还可能不稳定，也允许少量重试。STALL 立即返回。
        const NAK_RETRIES: u32 = 64;
        const XACT_RETRIES: u32 = 8;
        let mut xact_left = XACT_RETRIES;
        let hc_value = (hcchar + HCCHAR::CHENA::SET).value;
        for attempt in 0..=NAK_RETRIES {
            self.wait_disabled()?;
            // wait_disabled 已保证 CHENA 自清（通道停止），无需再发 CHDIS halt。
            chan.hcsplt.set(0);
            chan.hcint.set(HCINT_ALL_W1C);
            chan.hcintmsk
                .set((HCINT::CHHLTD::SET + HCINT::XFERCOMPL::SET).value);
            chan.hctsiz.set(hctsiz);
            usb_bus_fence_before_dma();
            chan.hcdma.set(dmap);
            usb_bus_fence_before_dma();
            chan.hcchar.set(hc_value);
            let st = self.wait_halted()?;
            if st.is_set(HCINT::STALL) {
                return Err(UsbError::Stall);
            }
            if st.is_set(HCINT::XACTERR) {
                if xact_left == 0 {
                    log::info!("USB-XACT EXHAUSTED ch={} hcchar={:#010x} hctsiz={:#010x} dma={:#010x} hcint={:#010x}",
                    self.0, hc_value, hctsiz, dmap, st.get());
                    return Err(UsbError::Protocol("ch xfer error (XACT)"));
                }
                xact_left -= 1;
                // XACTERR 退避 1ms（让 D+/D- 稳定再试）。
                crate::arch::time::delay(core::time::Duration::from_millis(1));
                continue;
            }
            if st.is_set(HCINT::NAK) {
                if attempt == NAK_RETRIES {
                    log::info!("USB-NAK EXHAUSTED ch={} hcchar={:#010x} hctsiz={:#010x} dma={:#010x} hcint={:#010x}",
                    self.0, hc_value, hctsiz, dmap, st.get());
                    return Err(UsbError::Protocol("ch xfer NAK exhausted"));
                }
                // Synopsys 建议 NAK 后等待 ~1 ms 再重试（HSEOF）。
                crate::arch::time::delay(core::time::Duration::from_millis(1));
                continue;
            }
            if !st.is_set(HCINT::XFERCOMPL) {
                log::info!("USB-CHHLTD-NO-XFER ch={} hcchar={:#010x} hctsiz={:#010x} dma={:#010x} hcint={:#010x}",
                self.0, hc_value, hctsiz, dmap, st.get());
                return Err(UsbError::Protocol("CHHLTD without XFERCOMPL"));
            }
            return Ok(st);
        }
        unreachable!()
    }
}

pub(crate) fn hcchar_control(
    dev: u32,
    ep: u32,
    mps: u32,
    dir_in: bool,
) -> FieldValue<u32, HCCHAR::Register> {
    let mut field = HCCHAR::MPS.val(mps)
        + HCCHAR::EPNUM.val(ep)
        + HCCHAR::DEVADDR.val(dev)
        + HCCHAR::EPTYPE::Control;
    if dir_in {
        field = field + HCCHAR::EPDIR::SET;
    }
    field
}

pub(crate) fn hcchar_isoch(
    dev: u32,
    ep: u32,
    mps: u32,
    mult: u32,
    dir_in: bool,
) -> FieldValue<u32, HCCHAR::Register> {
    let mut field = HCCHAR::MPS.val(mps)
        + HCCHAR::EPNUM.val(ep)
        + HCCHAR::DEVADDR.val(dev)
        + HCCHAR::EPTYPE::Isochronous
        + HCCHAR::MC.val(mult.clamp(1, 3));
    if dir_in {
        field = field + HCCHAR::EPDIR::SET;
    }
    field
}

/// 读 HFNUM 决定下个微帧奇偶；若当前帧 LSB=0（偶），下一帧为奇 -> 设 ODDFRM；反之清 0。
#[inline]
pub(crate) fn next_uframe_oddfrm() -> FieldValue<u32, HCCHAR::Register> {
    let fr = usb::dwc2_regs().hfnum.read(HFNUM::FRNUM);
    if (fr & 1) == 0 {
        HCCHAR::ODDFRM::SET
    } else {
        HCCHAR::ODDFRM::CLEAR
    }
}

pub(crate) fn hctsiz(pid: FieldValue<u32, HCTSIZ::Register>, pktcnt: u32, xfersize: u32) -> u32 {
    (pid + HCTSIZ::PKTCNT.val(pktcnt) + HCTSIZ::XFERSIZE.val(xfersize)).value
}

/// `SET_ADDRESS` 后延时，满足 USB 2.0 在下一事务前使用新地址的要求
/// （设备侧恢复，Linux 主机栈常用 ~10ms，这里给 50ms 富余；
/// 原迭代计数版按 1GHz 校准，25MHz 上实测 ~27s 纯属过杀）。
pub fn usb_post_set_address_delay() {
    crate::arch::time::delay(core::time::Duration::from_millis(50));
}
