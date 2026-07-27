//! 小工具：忙等延时。

/// 粗粒度忙等（约 `cycles` 次自旋）。用于任务间限速，无精确计时语义。
pub(crate) fn busy_wait(cycles: u64) {
    for _ in 0..cycles {
        core::hint::spin_loop();
    }
}
