//! 时间与延时：`rdtime`（CLINT 硬件定时器，两核共享）+ `delay(Duration)`。
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

// CLINT mtimer 心跳（wedge 观测）
//
// 每 10ms 一拍：心跳 +1、采样被中断 PC，写入邮箱 ctx 的两个协议空闲字
// （MMIO 非缓存，大核 mmio_read32 实时可见，无需 cache 维护）：
// - `0x0190_041C` ctx[3].aux = 心跳计数（turn 只用 ctx[3] 低 4B，高 4B 空闲）
// - `0x0190_040C` ctx[1].aux = 最近一次被时钟打断的 PC（B2S 协议不用 aux）
//
// 判读：小核完全活 → 心跳持续走；心跳冻结 → 核心/总线级 wedge（mtimer 都进
// 不来）；心跳走但业务死 → 主循环/特定 IRQ 路径问题。

/// CLINT mtimecmp（SG2002 CLINT @ 0x7400_0000,标准布局 +0x4000）。
/// 经 32 位桥,按 RV32 惯用高低字对写,不赌 64-bit 单写能否解码。
const CLINT_MTIMECMP_LO: usize = 0x7400_4000;
const CLINT_MTIMECMP_HI: usize = 0x7400_4004;
/// 心跳周期：10ms。
const HEARTBEAT_INTERVAL_TICKS: u64 = TIMEBASE_HZ / 100;
const HEARTBEAT_PA: usize = 0x0190_041C;
const LAST_PC_PA: usize = 0x0190_040C;

/// 启动心跳（`trap::init_interrupts` 尾部调用：先装 cmp 再开 MTIE，避免风暴）。
pub fn init_heartbeat() {
    unsafe {
        // 观测字清零:ctx RAM 上电为垃圾值,不清则 hb 从乱数起跳、判读依赖增量
        core::ptr::write_volatile(HEARTBEAT_PA as *mut u32, 0);
        core::ptr::write_volatile(LAST_PC_PA as *mut u32, 0);
        rearm();
        riscv::register::mie::set_mtimer();
        // 看门狗自愈:此后的喂狗职责归心跳 ISR(总线 wedge → 停跳 → 复位)
        crate::drivers::wdt::start();
    }
}

/// 重装比较器：mtime + 周期（绝对值，防迟到补拍风暴）。高低字分写。
#[inline]
fn rearm() {
    let next = rdtime().wrapping_add(HEARTBEAT_INTERVAL_TICKS);
    unsafe {
        core::ptr::write_volatile(CLINT_MTIMECMP_LO as *mut u32, next as u32);
        core::ptr::write_volatile(CLINT_MTIMECMP_HI as *mut u32, (next >> 32) as u32);
    }
}

/// mtimer ISR：喂狗 → 心跳 +1 → 采样被中断 PC → 重装下一拍（绝对值，防迟到补拍风暴）。
/// 喂狗放最前：wedge 时心跳即喂狗，停跳后 WDT ≈0.34s 复位整片。
pub fn heartbeat_tick() {
    crate::drivers::wdt::kick();
    unsafe {
        let hb = core::ptr::read_volatile(HEARTBEAT_PA as *const u32);
        core::ptr::write_volatile(HEARTBEAT_PA as *mut u32, hb.wrapping_add(1));
        let pc = riscv::register::mepc::read();
        core::ptr::write_volatile(LAST_PC_PA as *mut u32, pc as u32);
    }
    rearm();
}
