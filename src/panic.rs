//! Panic 处理：静默 `wfi` 死循环（小核不发 UART，避免与大核控制台冲突）。

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // 写一个错误标记到邮箱 magic，便于大核发现小核 panic。
    crate::ipc::write(0xFFFF_FFFF, 0, 0);
    loop {
        riscv::asm::wfi();
    }
}
