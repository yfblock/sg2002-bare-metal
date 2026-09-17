//! SG2002 JPU（JPEG Processing Unit）纯 Rust 驱动。
//!
//! 对照 U-Boot CVitek 驱动实现（`drivers/jpeg/`），在裸机上以轮询方式完成
//! Baseline JPEG 硬件解码，输出 YUV420 planar。

mod decoder;
mod header;
pub mod mem;
pub mod regs;

pub use decoder::JpuDecoder;

/// 解码分步计时：`decode()` 每走一步累计耗时（ticks），由 [FPS] 报告输出。
pub mod trace {
    use core::sync::atomic::Ordering;

    /// 步号：见 `decoder::decode()` 里的调用点。
    pub mod step {
        pub const ENTER: u32 = 1;
        pub const PARSE_HEADER: u32 = 2;
        pub const COPY_STREAM: u32 = 3;
        pub const CLEAN_STREAM: u32 = 4;
        pub const FRAME_LAYOUT: u32 = 5;
        pub const FREE_FRAME: u32 = 6;
        pub const ALLOC_FRAME: u32 = 7;
        pub const INV_FRAME: u32 = 8;
        pub const CFG_STREAM_REGS: u32 = 9;
        pub const HUFF: u32 = 10;
        pub const QUANT: u32 = 11;
        pub const GRAM: u32 = 12;
        pub const START_DECODE: u32 = 13;
        pub const POLL: u32 = 14;
        pub const INV_AFTER: u32 = 15;
        pub const DONE: u32 = 16;
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
