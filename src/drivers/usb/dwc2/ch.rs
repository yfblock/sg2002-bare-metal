//! 主机通道原语：启停/等待 halt、NAK/XACT 有界重试、USB ISR，以及 HFNUM 时间辅助。
//!
//! 通道约定：**0 = EP0 控制**，**1 = Isoch 视频**。

use core::sync::atomic::{AtomicBool, Ordering};
use tock_registers::interfaces::{ReadWriteable, Readable, Writeable};
use tock_registers::LocalRegisterCopy;

use super::regs::{Dwc2HostChannel, Dwc2Regs, GINTSTS, HCCHAR, HCINT, HCTSIZ, HFNUM};
use crate::drivers::usb;
use crate::drivers::usb::error::{UsbError, UsbResult};
use tock_registers::fields::FieldValue;

/// `HCINT` 快照（通道 halt 时读出的中断原因位，供上层区分 XFERCOMPL / NAK / STALL 等）。
pub(crate) type HcintSnapshot = LocalRegisterCopy<u32, HCINT::Register>;

#[inline]
pub(crate) fn regs() -> &'static Dwc2Regs {
    usb::dwc2_regs()
}

#[inline]
pub(crate) fn channel(ch: u32) -> &'static Dwc2HostChannel {
    usb::dwc2_channel(ch)
}

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
    let r = regs();
    if !r.gintsts.is_set(GINTSTS::HCHINT) {
        return;
    }
    // HAINT：每 bit 对应一个通道的中断状态。
    let haint = r.haint.get();
    for ch in 0..2u32 {
        if haint & (1 << ch) == 0 {
            continue;
        }
        let c = channel(ch);
        let hcint = c.hcint.extract();
        // 清掉本通道所有中断位（W1C）
        c.hcint.set(hcint.get());
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
    pub(crate) fn regs(&self) -> &'static Dwc2HostChannel {
        channel(self.0)
    }

    /// 等通道空闲（`CHENA` 自清）。
    pub(crate) fn wait_disabled(&self) -> UsbResult<()> {
        let c = self.regs();
        for _ in 0..2_000_000u32 {
            if !c.hcchar.is_set(HCCHAR::CHENA) {
                return Ok(());
            }
            spin_delay(8);
        }
        Err(UsbError::Timeout)
    }

    /// 若通道仍忙，按 Linux `dwc2_hc_halt` 同时置 `CHENA|CHDIS` 请求停止。
    pub(crate) fn halt(&self) {
        let c = self.regs();
        if !c.hcchar.is_set(HCCHAR::CHENA) {
            return;
        }
        c.hcchar.modify(HCCHAR::CHENA::SET + HCCHAR::CHDIS::SET);
        for _ in 0..500_000u32 {
            if !c.hcchar.is_set(HCCHAR::CHENA) {
                return;
            }
            spin_delay(8);
        }
    }

    /// 等通道 halt。中断 flag 优先，`spin_delay` 轮询兜底。
    ///
    /// 两条路径都留着是因为 PLIC source 30 在 C906L 上触发率很低——
    /// 实测每 100 帧约 69 次 ISR，而同期有 7700 次通道传输，覆盖率不到 1%。
    /// 中断链路本身是正确的（`HCINTMSK` 已编程、无误触发），只是不足以替代轮询。
    pub(crate) fn wait_halted(&self) -> UsbResult<HcintSnapshot> {
        let c = self.regs();
        let idx = self.0 as usize;
        for _ in 0..8_000_000u32 {
            // 中断路径：USB ISR 设了 CH_DONE
            if CH_DONE[idx].swap(false, Ordering::AcqRel) {
                let hi = c.hcint.extract();
                c.hcint.set(hi.get());
                return Ok(hi);
            }
            // 轮询兜底。实测 PLIC source 30 覆盖率不足 1%，绝大多数传输走这里。
            let hi = c.hcint.extract();
            if hi.is_set(HCINT::CHHLTD) {
                c.hcint.set(hi.get());
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
        let c = self.regs();
        let dmap = super::dma::dma_phys(dma_off as usize);

        // EP0 control 上：NAK = 设备未就绪，自动重试；XACTERR = CRC/PID/babble，
        // 在 reset 解除后总线还可能不稳定，也允许少量重试。STALL 立即返回。
        const NAK_RETRIES: u32 = 64;
        const XACT_RETRIES: u32 = 8;
        let mut xact_left = XACT_RETRIES;
        let hc_value = (hcchar + HCCHAR::CHENA::SET).value;
        for attempt in 0..=NAK_RETRIES {
            self.wait_disabled()?;
            self.halt();
            c.hcsplt.set(0);
            c.hcint.set(HCINT_ALL_W1C);
            c.hcintmsk
                .set((HCINT::CHHLTD::SET + HCINT::XFERCOMPL::SET).value);
            c.hctsiz.set(hctsiz);
            usb_bus_fence_before_dma();
            c.hcdma.set(dmap);
            usb_bus_fence_before_dma();
            c.hcchar.set(hc_value);
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
                // XACTERR 退避更久（让 D+/D- 稳定再试），约 1ms。
                spin_delay(2_000_000);
                continue;
            }
            if st.is_set(HCINT::NAK) {
                if attempt == NAK_RETRIES {
                    log::info!("USB-NAK EXHAUSTED ch={} hcchar={:#010x} hctsiz={:#010x} dma={:#010x} hcint={:#010x}",
                    self.0, hc_value, hctsiz, dmap, st.get());
                    return Err(UsbError::Protocol("ch xfer NAK exhausted"));
                }
                // Synopsys 建议 NAK 后等待 ~1 ms 再重试（HSEOF），这里用粗粒度 spin。
                spin_delay(200_000);
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
    let mut v = HCCHAR::MPS.val(mps & 0x7ff)
        + HCCHAR::EPNUM.val(ep & 0xf)
        + HCCHAR::DEVADDR.val(dev & 0x7f)
        + HCCHAR::EPTYPE::Control;
    if dir_in {
        v = v + HCCHAR::EPDIR::SET;
    }
    v
}

pub(crate) fn hcchar_isoch(
    dev: u32,
    ep: u32,
    mps: u32,
    mult: u32,
    dir_in: bool,
) -> FieldValue<u32, HCCHAR::Register> {
    let mut v = HCCHAR::MPS.val(mps & 0x7ff)
        + HCCHAR::EPNUM.val(ep & 0xf)
        + HCCHAR::DEVADDR.val(dev & 0x7f)
        + HCCHAR::EPTYPE::Isochronous
        + HCCHAR::MC.val(mult.clamp(1, 3) & 0x3);
    if dir_in {
        v = v + HCCHAR::EPDIR::SET;
    }
    v
}

/// 读 HFNUM 决定下个微帧奇偶；若当前帧 LSB=0（偶），下一帧为奇 -> 设 ODDFRM；反之清 0。
#[inline]
pub(crate) fn next_uframe_oddfrm() -> FieldValue<u32, HCCHAR::Register> {
    let fr = regs().hfnum.read(HFNUM::FRNUM);
    if (fr & 1) == 0 {
        HCCHAR::ODDFRM::SET
    } else {
        HCCHAR::ODDFRM::CLEAR
    }
}

pub(crate) fn hctsiz(pid: FieldValue<u32, HCTSIZ::Register>, pktcnt: u32, xfersize: u32) -> u32 {
    (pid + HCTSIZ::PKTCNT.val(pktcnt) + HCTSIZ::XFERSIZE.val(xfersize)).value
}

/// 计算 `HCTSIZ.PKTCNT`：按 `mps` 分包后的包数（至少为 1）///
/// # 参数
/// - `mps`：端点最大包长（字节），为 0 时按 1 包处理。
/// - `nbytes`：本段传输总字节数。
pub(crate) fn pktcnt_for(mps: u32, nbytes: u32) -> u32 {
    if mps == 0 {
        return 1;
    }
    nbytes.div_ceil(mps)
}

/// `SET_ADDRESS` 后延时，满足 USB 2.0 在下一事务前使用新地址的要求
/// （设备侧恢复，Linux 主机栈常用 ~10ms，这里给 50ms 富余；
/// 原迭代计数版按 1GHz 校准，25MHz 上实测 ~27s 纯属过杀）。
pub fn usb_post_set_address_delay() {
    crate::arch::time::delay(core::time::Duration::from_millis(50));
}

/// Hub 下游端口 `PORT_RESET` 后给设备恢复时间（TDRSTR 后的 TRSTRCY 稳定，100ms 富余）。
pub fn usb_post_hub_port_reset_delay() {
    crate::arch::time::delay(core::time::Duration::from_millis(100));
}
