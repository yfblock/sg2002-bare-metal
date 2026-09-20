//! SG2002 JPU（JPEG Processing Unit）纯 Rust 驱动。
//!
//! 对照 U-Boot CVitek 驱动实现（`drivers/jpeg/`），在裸机上以轮询方式完成
//! Baseline JPEG 硬件解码，输出 YUV420 planar。

mod decoder;
mod header;
pub mod mem;
pub mod regs;

pub use decoder::JpuDecoder;

mod session;
pub use session::{decode_to_shared, reset_count};

/// `UnsafeCell` + `Sync`：裸机单核安全持有可变全局。
pub(crate) struct SyncUnsafeCell<T>(pub(crate) core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for SyncUnsafeCell<T> {}
impl<T> SyncUnsafeCell<T> {
    pub(crate) const fn new(value: T) -> Self { Self(core::cell::UnsafeCell::new(value)) }

    pub(crate) fn with_mut<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        // SAFETY: 裸机单核上下文,无并发访问。
        unsafe { f(&mut *self.0.get()) }
    }
}

/// 解码分步计时：`decode()` 每走一步累计耗时，由 \[FPS\] 报告输出。
pub mod trace {
    use core::sync::atomic::{AtomicU64, Ordering};
    use core::time::Duration;

    /// 步号（仅保留 \[FPS\] 报告实际读取的 5 步）。
    pub mod step {
        pub const COPY_STREAM: u32 = 3;
        pub const CLEAN_STREAM: u32 = 4;
        pub const INV_FRAME: u32 = 8;
        pub const POLL: u32 = 14;
        pub const INV_AFTER: u32 = 15;
    }

    /// 各步累计耗时（纳秒——Duration 的原子存储基元），下标见 `step`。
    /// 只累加不打印;u64 纳秒容量 ~584 年,累加窗口内不溢出。
    static STEP_NANOS: [AtomicU64; 17] = [const { AtomicU64::new(0) }; 17];

    /// 取走并清零某步的累计耗时。
    pub fn take_step_time(step: u32) -> Duration {
        STEP_NANOS
            .get(step as usize)
            .map(|a| Duration::from_nanos(a.swap(0, Ordering::Relaxed)))
            .unwrap_or(Duration::ZERO)
    }

    /// 记一步耗时：把「上次 mark 到现在」累加到 `step`。
    /// ticks → Duration 统一经 `arch::time::elapsed_since`。
    #[inline]
    pub(crate) fn mark_timed(step: u32) {
        static LAST: AtomicU64 = AtomicU64::new(0);
        let now = crate::arch::time::rdtime();
        let prev = LAST.swap(now, Ordering::Relaxed);
        if prev != 0 && now > prev {
            let dt = crate::arch::time::elapsed_since(prev);
            if let Some(a) = STEP_NANOS.get(step as usize) {
                a.fetch_add(dt.as_nanos() as u64, Ordering::Relaxed);
            }
        }
    }
}
