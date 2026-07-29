//! 小核运行时控制标志，由大核经邮箱控制消息设置。
//!
//! 控制消息格式：`0xF0_49_<cmd>_<arg>`（`0x49` = 'I'，取自 IVE 调试通道）。
//! 解析在 `trap.rs::handle_mailbox_irq`。
//!
//! | cmd | 含义 |
//! |-----|------|
//! | 0x50 | arg=1 暂停采集流水线，arg=0 恢复 |
//! | 0x56 | 设 IVE 输入 `fmt_sel` = arg |
//! | 0x57 | arg=1 静音串口输出，arg=0 恢复 |
//!
//! 为什么需要「暂停」：大核要比对 IVE 输出的 RGB 与软件参考，
//! 必须保证读 YUV 和读 RGB 时缓冲内容属于**同一帧**。
//! 小核持续覆盖单缓冲的话，比对结果没有意义。
//!
//! 为什么需要「静音」：UART0 与大核共用，小核每 100 帧一行的汇总
//! 会把大核的表格输出冲散。

use core::sync::atomic::{AtomicBool, Ordering};

static PAUSED: AtomicBool = AtomicBool::new(false);
static MUTED: AtomicBool = AtomicBool::new(false);

/// 流水线是否已暂停。
#[inline]
pub fn paused() -> bool {
    PAUSED.load(Ordering::Acquire)
}

pub fn set_paused(v: bool) {
    PAUSED.store(v, Ordering::Release);
}

/// 串口是否静音。
#[inline]
pub fn muted() -> bool {
    MUTED.load(Ordering::Relaxed)
}

pub fn set_muted(v: bool) {
    MUTED.store(v, Ordering::Relaxed);
}
