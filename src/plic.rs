//! C906L 小核 PLIC（0x70000000）：启用 source 61（邮箱）和 30（USB）。
//! PLIC context 0 = M-mode hart 0。

const PLIC_BASE: usize = 0x7000_0000;

/// 小核邮箱中断 PLIC source（intr_conf.h: MBOX_INT_C906_2ND = 61）。
pub const MBOX_IRQ_SRC: u32 = 61;
/// USB (DWC2) 中断 PLIC source（dts `usb@04340000 interrupts=<0x1e>` = 30）。
pub const USB_IRQ_SRC: u32 = 30;
/// 设置 source 61（邮箱）和 30（USB）的优先级、threshold=0、enable。
pub unsafe fn init() {
    // threshold = 0（接受所有优先级）
    core::ptr::write_volatile((PLIC_BASE + 0x200000) as *mut u32, 0);

    // source 61（邮箱）：优先级 1，ENABLE2 bit 29
    core::ptr::write_volatile((PLIC_BASE + 61 * 4) as *mut u32, 1);
    let en2 = (PLIC_BASE + 0x2004) as *mut u32;
    let old = core::ptr::read_volatile(en2);
    core::ptr::write_volatile(en2, old | (1 << 29));

    // source 30（USB）：优先级 1，ENABLE0 bit 30
    core::ptr::write_volatile((PLIC_BASE + 30 * 4) as *mut u32, 1);
    let en0 = (PLIC_BASE + 0x2000) as *mut u32;
    let old = core::ptr::read_volatile(en0);
    core::ptr::write_volatile(en0, old | (1 << 30));
}

/// Claim PLIC（读 claim 寄存器获取中断 source ID）。
pub fn claim() -> u32 {
    unsafe { core::ptr::read_volatile((PLIC_BASE + 0x200004) as *const u32) }
}

/// Complete PLIC（写 source ID 完成）。
pub fn complete(src: u32) {
    unsafe { core::ptr::write_volatile((PLIC_BASE + 0x200004) as *mut u32, src) };
}
