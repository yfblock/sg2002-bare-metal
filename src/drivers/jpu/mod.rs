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
pub use session::{decode_to_shared, take_reset_count};

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
