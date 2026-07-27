//! C906L 小核 PLIC（0x70000000）设置：启用 source 61（MBOX_INT_C906_2ND）。
//! PLIC context 0 = M-mode hart 0。

const PLIC_BASE: usize = 0x7000_0000;

/// 小核邮箱中断 PLIC source（intr_conf.h: MBOX_INT_C906_2ND = 61）。
pub const MBOX_IRQ_SRC: u32 = 61;

/// 初始化 PLIC：设置 source 61 优先级、threshold=0、enable。
pub unsafe fn init() {
    // 优先级（source 61 @ PLIC_BASE + 61*4）
    core::ptr::write_volatile((PLIC_BASE + 61 * 4) as *mut u32, 1);
    // threshold = 0（接受所有优先级）
    core::ptr::write_volatile((PLIC_BASE + 0x200000) as *mut u32, 0);
    // enable source 61（ENABLE2 = +0x2004, bit 29 = 61-32）
    let en2 = (PLIC_BASE + 0x2004) as *mut u32;
    let old = core::ptr::read_volatile(en2);
    core::ptr::write_volatile(en2, old | (1 << 29));
}

/// Claim PLIC（读 claim 寄存器获取中断 source ID）。
pub fn claim() -> u32 {
    unsafe { core::ptr::read_volatile((PLIC_BASE + 0x200004) as *const u32) }
}

/// Complete PLIC（写 source ID 完成）。
pub fn complete(src: u32) {
    unsafe { core::ptr::write_volatile((PLIC_BASE + 0x200004) as *mut u32, src) };
}
