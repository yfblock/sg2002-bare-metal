//! SG2002 JPU（JPEG Processing Unit）纯 Rust 驱动。
//!
//! 对照 U-Boot CVitek 驱动实现（`drivers/jpeg/`），在裸机上以轮询方式完成
//! Baseline JPEG 硬件解码，输出 YUV420 planar。

mod decoder;
mod header;
pub mod mem;
pub mod regs;

pub use decoder::JpuDecoder;

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

/// 解码分步计时：`decode()` 每走一步累计耗时（ticks），由 [FPS] 报告输出。
pub mod trace {
    use core::sync::atomic::Ordering;

    /// 步号（仅保留 [FPS] 报告实际读取的 5 步）。
    pub mod step {
        pub const COPY_STREAM: u32 = 3;
        pub const CLEAN_STREAM: u32 = 4;
        pub const INV_FRAME: u32 = 8;
        pub const POLL: u32 = 14;
        pub const INV_AFTER: u32 = 15;
    }

    /// 各步累计耗时（rdtime ticks），下标见 `step`。只累加不打印。
    pub static STEP_TICKS: [core::sync::atomic::AtomicU32; 17] =
        [const { core::sync::atomic::AtomicU32::new(0) }; 17];

    #[inline]
    fn now_ticks() -> u64 {
        crate::arch::time::rdtime()
    }

    /// 取走并清零某步的累计 ticks。
    pub fn take_step_ticks(step: u32) -> u32 {
        STEP_TICKS
            .get(step as usize)
            .map(|a| a.swap(0, Ordering::Relaxed))
            .unwrap_or(0)
    }

    /// 记一步耗时：把「上次 mark 到现在」累加到 `step`。
    #[inline]
    pub(crate) fn mark_timed(step: u32) {
        use core::sync::atomic::AtomicU64;
        static LAST: AtomicU64 = AtomicU64::new(0);
        let now = now_ticks();
        let prev = LAST.swap(now, Ordering::Relaxed);
        if prev != 0 && now > prev {
            if let Some(a) = STEP_TICKS.get(step as usize) {
                a.fetch_add((now - prev) as u32, Ordering::Relaxed);
            }
        }
    }
}
