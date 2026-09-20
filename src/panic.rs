//! Panic 处理：静默 `wfi` 死循环（小核不发 UART，避免与大核控制台冲突）。

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // 写 frame_count 错误标记（0xFFFF_FFFF），大核的 panic 探测器读该字段发现小核 panic。
    crate::ipc::write(0xFFFF_FFFF, 0, 0);
    loop {
        riscv::asm::wfi();
    }
}
