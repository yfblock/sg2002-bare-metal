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

/// PLIC 单例(裸机单核,`Plic` 自带 `unsafe impl Sync`)。
static mut PLIC: Option<Plic> = None;

/// 初始化:threshold=0、source 61/30 优先级=1 并使能。
pub unsafe fn init() {
    let base = NonNull::new(0x7000_0000 as *mut PLICRegs).unwrap();
    let mut plic = Plic::new(base);
    plic.set_threshold(CTX, 0);

    let mbox = NonZeroU32::new(MBOX_IRQ_SRC).unwrap();
    plic.set_priority(mbox, 1);
    plic.enable(mbox, CTX);

    let usb = NonZeroU32::new(USB_IRQ_SRC).unwrap();
    plic.set_priority(usb, 1);
    plic.enable(usb, CTX);

    PLIC = Some(plic);
}

/// Claim(读 claim 寄存器获取中断 source ID;0 = 无 pending)。
pub fn claim() -> u32 {
    unsafe {
        match &mut PLIC {
            Some(p) => p.claim(CTX).map(|v| v.get()).unwrap_or(0),
            None => 0,
        }
    }
}

/// Complete(写 source ID 完成中断处理)。
pub fn complete(src: u32) {
    if let Some(nz) = NonZeroU32::new(src) {
        unsafe {
            if let Some(p) = &mut PLIC {
                p.complete(CTX, nz);
            }
        }
    }
}
