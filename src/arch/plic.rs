//! 小核 PLIC(0x70000000):使用 `riscv_plic` crate 管理中断源。
//!
//! C906L 小核用 context 0(M-mode hart 0),启用 source 61(邮箱)+ 30(USB)。

use core::num::NonZeroU32;
use core::ptr::NonNull;
use riscv_plic::{Plic, PLICRegs};

/// 小核邮箱中断 PLIC source(intr_conf.h: MBOX_INT_C906_2ND = 61)。
pub const MBOX_IRQ_SRC: u32 = 61;
/// USB (DWC2) 中断 PLIC source(dts `usb@04340000 interrupts=<0x1e>` = 30)。
pub const USB_IRQ_SRC: u32 = 30;

/// C906L 的 M-mode context 编号(hart 0 的 M-mode)。
const CTX: usize = 0;

/// PLIC MMIO 基址(编译期常量,恒有效)。
const PLIC_BASE: *mut PLICRegs = 0x7000_0000 as *mut PLICRegs;

/// 取 PLIC 视图(`Plic::new` 是 const 构造,零开销,无需运行时注册)。
#[inline]
fn plic() -> Plic {
    // SAFETY: 基址为编译期常量,本板恒有效;裸机单核无并发访问。
    unsafe { Plic::new(NonNull::new_unchecked(PLIC_BASE)) }
}

/// 初始化:threshold=0、source 61/30 优先级=1 并使能。
pub unsafe fn init() {
    let mut plic = plic();
    plic.set_threshold(CTX, 0);

    let mbox = NonZeroU32::new(MBOX_IRQ_SRC).unwrap();
    plic.set_priority(mbox, 1);
    plic.enable(mbox, CTX);

    let usb = NonZeroU32::new(USB_IRQ_SRC).unwrap();
    plic.set_priority(usb, 1);
    plic.enable(usb, CTX);
}

/// Claim(读 claim 寄存器获取中断 source ID;0 = 无 pending)。
pub fn claim() -> u32 {
    plic().claim(CTX).map(|v| v.get()).unwrap_or(0)
}

/// Complete(写 source ID 完成中断处理)。
pub fn complete(src: u32) {
    if let Some(nz) = NonZeroU32::new(src) {
        plic().complete(CTX, nz);
    }
}
