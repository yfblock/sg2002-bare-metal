//! 时间与延时：`rdtime`（CLINT 硬件定时器，两核共享）+ `delay(Duration)` + `Deadline`。
//!
//! SG2002 的 timebase = 25 MHz（实测 25.005 MHz，25 ticks/µs）；大核 C906B(1GHz)
//! 和小核 C906L(25MHz) 读同一个 mtime，跨核时间戳可直接比较。
//! 唯一的延时入口是 [`delay`]（接受 [`core::time::Duration`]），精确计时与 CPU 频率无关。

use core::time::Duration;

/// `rdtime`（mtime）计数频率。SG2002 为 25 MHz——实测 25.005 MHz
/// （小核连续采样 `rdtime`，300,333,869 ticks / 12.011 s）。
pub const TIMEBASE_HZ: u64 = 25_000_000;

/// 读 64 位 `rdtime`（CLINT mtime，M-mode / S-mode 都可读，C906 已验证）。
///
/// 实现 = `riscv::register::time::read64()`（`csrr time`，与 `rdtime` 伪指令同义）。
#[inline]
pub fn rdtime() -> u64 {
    riscv::register::time::read64()
}

/// 把 [`Duration`] 换算成 rdtime ticks（u128 中间量防溢出；40ns/tick 的精度上限）。
#[inline]
fn duration_to_ticks(d: Duration) -> u64 {
    (d.as_nanos() * (TIMEBASE_HZ as u128) / 1_000_000_000u128) as u64
}

/// 阻塞精确延时。基于 rdtime 硬件定时器，与 CPU 频率无关；
/// wrapping 比较天然容忍计数器回绕。
pub fn delay(d: Duration) {
    let target = rdtime().wrapping_add(duration_to_ticks(d));
    while (rdtime().wrapping_sub(target) as i64) < 0 {
        core::hint::spin_loop();
    }
}

/// 距某次 `rdtime()` 采样已过去的时长（wrapping 差值，容忍计数器回绕）。
///
/// 超时轮询惯用法：循环前 `let t0 = rdtime();`，循环里
/// `if elapsed_since(t0) >= Duration::from_millis(10) { ... }`。
#[inline]
pub fn elapsed_since(t0: u64) -> Duration {
    let ticks = rdtime().wrapping_sub(t0);
    let nanos = (ticks as u128) * 1_000_000_000u128 / TIMEBASE_HZ as u128;
    Duration::from_nanos(u64::try_from(nanos).unwrap_or(u64::MAX))
}
