//! 小核运行统计块（共享 DRAM，大核只读）。
//!
//! 为什么需要它：主循环里 `uvc_capture_one_frame` 的 `Err` 分支原本是静默的
//! （不 notify、不打日志），一旦抓帧持续失败就表现为 `frame_count` 冻结而串口
//! 毫无输出，根本分不清是卡在 USB 抓帧还是 JPU 解码。UART0 又和大核共用，不能
//! 靠小核刷日志。所以把计数器放共享 DRAM，由大核 `/dev/cvi-mailbox` 探针读出。
//!
//! 位置：`0x8FFFE040`，在邮箱（`0x8FFFE000`，32B）之后的空隙里，互不重叠。

/// 统计块物理地址（邮箱之后的空隙）。
pub const STATS_PA: usize = 0x8FFF_E040;
/// 有效标志 "STAT"。
pub const STATS_MAGIC: u32 = 0x5354_4154;

/// 布局须与大核 `cvi_mailbox.rs` 的 `SmallCoreStats` 一致。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Stats {
    pub magic: u32,
    /// 主循环进入次数（卡住时它不再增长 → 卡在循环体内某个调用里）。
    pub loop_iters: u32,
    /// `uvc_capture_one_frame` 成功 / 失败次数。
    pub cap_ok: u32,
    pub cap_err: u32,
    /// JPU 解码成功 / 失败次数、累计复位次数。
    pub jpu_ok: u32,
    pub jpu_err: u32,
    pub jpu_reset: u32,
    /// 循环里最后到达的阶段（见 `stage`），配合 loop_iters 定位卡点。
    pub stage: u32,
    /// JPU `decode()` 内部步号（sg200x_bsp::jpu::trace::step），由 BSP 直接写。
    pub jpu_trace: u32,
    /// `rdtime` 低 32 位，每轮更新。用来标定 timebase 频率（大核两次探针间隔已知）。
    pub time_lo: u32,
    /// JPU 轮询轮次 / 轮询时刻（`rdtime` 低 32 位），由 BSP 的 `mark_poll` 直接写。
    /// 主循环被 JPU 堵住时，只有这两个字段还在动——用来区分"死循环"和"慢循环"。
    pub poll_count: u32,
    pub poll_time: u32,
}

/// `jpu_trace` 字段的物理地址——交给 BSP 的 trace 钩子直接写。
pub const TRACE_PA: usize = STATS_PA + 8 * 4;

/// 阶段标记：卡住时 `stage` 停在哪一档就说明卡在那一步。
pub mod stage {
    /// 即将调用 `uvc_capture_one_frame`。
    pub const CAPTURE: u32 = 1;
    /// 抓帧返回，即将取 DMA 视图。
    pub const GOT_FRAME: u32 = 2;
    /// 即将调用 JPU 解码。
    pub const DECODE: u32 = 3;
    /// 解码返回，即将写邮箱。
    pub const NOTIFY: u32 = 4;
    /// 一轮走完。
    pub const DONE: u32 = 5;
}

#[inline]
fn ptr() -> *mut u32 {
    STATS_PA as *mut u32
}

/// 把统计块写回 DRAM。理由同 `mailbox::flush_mailbox`——带 cache 的 identity
/// 映射下，不显式 clean 大核就读不到（之前靠 JPU 的大范围 invalidate 侥幸生效）。
#[inline]
fn flush() {
    sg200x_bsp::utils::cache::dcache_clean_range(STATS_PA, 12 * 4);
}

/// 清零并打上 magic。
pub fn init() {
    unsafe {
        let p = ptr();
        for i in 1..12 {
            core::ptr::write_volatile(p.add(i), 0);
        }
        core::ptr::write_volatile(p.add(0), STATS_MAGIC);
    }
    flush();
}

/// 逐字段写，避免整结构 RMW（和邮箱同理，别互相覆盖）。
macro_rules! bump {
    ($name:ident, $idx:expr) => {
        #[inline]
        pub fn $name() {
            unsafe {
                let p = ptr().add($idx);
                core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
            }
            flush();
        }
    };
}

bump!(inc_loop, 1);
bump!(inc_cap_ok, 2);
bump!(inc_cap_err, 3);
bump!(inc_jpu_ok, 4);
bump!(inc_jpu_err, 5);
bump!(inc_jpu_reset, 6);

/// `rdtime` 计数频率（SG2002 = 25MHz，实测 25.005MHz）。
pub const TIMEBASE_HZ: u64 = 25_000_000;

/// 读 64 位 `rdtime`。
#[inline]
pub fn rdtime() -> u64 {
    let t: usize;
    unsafe { core::arch::asm!("rdtime {0}", out(reg) t, options(nomem, nostack)) };
    t as u64
}

/// 记录 `rdtime` 低 32 位（标定 timebase 用）。
#[inline]
pub fn set_time_lo() {
    let t: usize;
    unsafe { core::arch::asm!("rdtime {0}", out(reg) t, options(nomem, nostack)) };
    unsafe { core::ptr::write_volatile(ptr().add(9), t as u32) };
    flush();
}

/// 记录当前阶段。
#[inline]
pub fn set_stage(s: u32) {
    unsafe { core::ptr::write_volatile(ptr().add(7), s) };
    flush();
}
