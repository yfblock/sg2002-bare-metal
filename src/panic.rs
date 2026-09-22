//! Panic 处理：静默 `wfi` 死循环（小核不发 UART，避免与大核控制台冲突）。

use core::panic::PanicInfo;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // 位置裸写 0x0190_0420/24(ctx[3].aux 高半字,心跳只用低 4B):
    // panic 中不可依赖 ipc/UART(带锁设施),write_volatile 是唯一可靠出口。
    if let Some(l) = info.location() {
        let f = l.file().as_bytes();
        let mut f4 = 0u32;
        for i in 0..4 {
            f4 |= (f.get(i).copied().unwrap_or(0) as u32) << (8 * i);
        }
        unsafe {
            core::ptr::write_volatile(0x0190_0420 as *mut u32, l.line());
            core::ptr::write_volatile(0x0190_0424 as *mut u32, f4);
        }
    }
    // 写 frame_count 错误标记（0xFFFF_FFFF），大核的 panic 探测器读该字段发现小核 panic。
    crate::ipc::write(0xFFFF_FFFF, 0, 0);
    loop {
        riscv::asm::wfi();
    }
}
