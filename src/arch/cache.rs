//! D-Cache 维护（DMA 一致性契约）。riscv64 + T-Head C906 自定义缓存指令。
//!
//! C906 使用非标缓存指令（`dcache.cva/iva`，`.insn` 编码），与标准
//! Zicbom 编码空间重叠——同时启用会因 binutils 解码歧义直接 `compile_error!`。
//! USB/JPU 的 DMA 缓冲都靠这里的 clean/invalidate 保证与 CPU 视图一致。

const CACHE_LINE: usize = 64;

// 与标准 RISC-V Zicbom 缓存指令冲突时给出明确报错——zicbom 与 C906 自定义指令编码空间重叠，
// 同时打开会导致 binutils 解码歧义。
#[cfg(target_feature = "zicbom")]
compile_error!("RISC-V `zicbom` 标准缓存指令与 C906 自定义编码冲突，请关闭 zicbom。");

// 按行精细维护：dcache.cva / dcache.iva（C906 非标指令）

#[inline(always)]
unsafe fn dcache_cva(va: usize) {
    unsafe {
        core::arch::asm!(".insn i 0x0b, 0, x0, {0}, 0x025", in(reg) va);
    }
}

#[inline(always)]
unsafe fn dcache_iva(va: usize) {
    unsafe {
        core::arch::asm!(".insn i 0x0b, 0, x0, {0}, 0x026", in(reg) va);
    }
}

/// 把 `[start, start+size)` 之内的所有缓存行 **clean**（写回到内存），
/// 用于 CPU 写完 DMA 描述符 / TX 数据后、把寄存器交给 DMA 之前。
#[inline]
pub fn dcache_clean_range(start: usize, size: usize) {
    if size == 0 {
        return;
    }
    let mut addr = start & !(CACHE_LINE - 1);
    let end = start + size;
    while addr < end {
        unsafe { dcache_cva(addr) };
        addr += CACHE_LINE;
    }
    riscv::asm::fence();
}

/// 把 `[start, start+size)` 之内的所有缓存行 **invalidate**（丢掉脏数据，
/// 强制下次读回内存），用于 DMA 写完 RX 帧、CPU 读取之前。
#[inline]
pub fn dcache_invalidate_range(start: usize, size: usize) {
    if size == 0 {
        return;
    }
    let mut addr = start & !(CACHE_LINE - 1);
    let end = start + size;
    while addr < end {
        unsafe { dcache_iva(addr) };
        addr += CACHE_LINE;
    }
    riscv::asm::fence();
}
