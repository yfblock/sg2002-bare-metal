//! 跨核 UART 控制台：DW 8250 寄存器访问 + 打印辅助 + Dekker 跨核行锁。
//!
//! UART0（0x04140000，CVITEK 集成的 DW APB 8250，**32-bit 寄存器步长**）已由
//! U-Boot 初始化为 115200——这里只轮询 TX，不配波特率/FIFO、不做 RX。
//!
//! # 跨核 UART 行锁
//! 两核共用 UART0,并发打印会字节级交错。锁字放在**邮箱控制器的空闲
//! context slot 2**(0x0190_0410,MMIO 天然非缓存,两核直读直写一致)。
//! 策略:主循环 `print` 短自旋后**强制夺取**(防上电遗留脏值死锁);
//! **ISR 上下文用 `line_lock_try`,拿不到就丢行,绝不等待**。

use core::fmt::Write as _;
use tock_registers::{register_bitfields, register_structs};
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::registers::{ReadOnly, WriteOnly};

register_bitfields![u32,
    /// LSR（+0x14）：线路状态。
    pub LSR [
        /// Transmit Holding Register Empty（可写下一字节）。
        THRE OFFSET(5) NUMBITS(1) [],
    ],
];

register_structs! {
    /// DW 8250 寄存器映射（仅 TX 用到的两个；DLAB=0 视图）。
    pub Dw8250Uart {
        /// 发送保持寄存器（写）。
        (0x00 => pub thr: WriteOnly<u32>),
        (0x04 => _reserved04: [u32; 4]),
        (0x14 => pub lsr: ReadOnly<u32, LSR::Register>),
        (0x18 => @END),
    }
}

/// UART0 MMIO 基址。
const UART0_BASE: usize = 0x0414_0000;

/// 取 UART0 寄存器视图（基址为编译期常量，恒有效）。
#[inline]
fn regs() -> &'static Dw8250Uart {
    unsafe { &*(UART0_BASE as *const Dw8250Uart) }
}

/// 阻塞发送一个字节（轮询 LSR.THRE）。`\n` 不自动加 `\r`——由 [`print`] 处理。
#[inline]
pub(crate) fn uart_putc(c: u8) {
    while !regs().lsr.is_set(LSR::THRE) {}
    regs().thr.set(c as u32);
}

// ---- 跨核行锁(Dekker,纯 load/store)----
// 锁变量在邮箱 context slot2/3(MMIO 非缓存,两核直读直写一致)。
// 注:曾试 amoswap.w —— 小核(M-mode)可用,但大核(S-mode)对该 MMIO 段
// 触发异常(go 后首条打印即挂),故改 Dekker 两进程互斥(不依赖 AMO):
//   flag_small @ slot2 低4B(只小核写)/ flag_big @ slot2 高4B(只大核写)
//   turn       @ slot3 低4B(双方写,竞争时决定谁让行)
// 两侧实现必须同款(大核侧 tools/bigcore-bm/src/main.rs)。
use crate::drivers::mailbox::{CTX_SLOT2, CTX_SLOT3};
const ME: usize = 1; // 小核 = 1;大核 = 0
const LOCK_SPIN_LIMIT: u32 = 2_000_000; // 自旋上限;超时强闯(防上电遗留脏值)

#[inline]
fn flag_addr(cpu: usize) -> usize {
    CTX_SLOT2 + cpu * 4
}

/// 本核持有标记(main 与 ISR 同核共用一套 Dekker 旗字)。ISR 不得对 main
/// 已持有的锁做「升旗-判让-降旗」——那会解掉被打断的 main 手里的锁,
/// 让两核同时认为持锁。ISR 见此标记直接放弃回显。
static LOCK_HELD: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// 尝试获锁一次(适合 ISR:失败立即返回 false,绝不等待)
pub(crate) fn line_lock_try() -> bool {
    use core::sync::atomic::Ordering;
    if LOCK_HELD.load(Ordering::Acquire) {
        // 本核 main 正持锁(打印中被本核 ISR 打断):绝不嵌套、绝不动旗字
        return false;
    }
    let ok = unsafe {
        core::ptr::write_volatile(flag_addr(ME) as *mut u32, 1);
        riscv::asm::fence();
        let other = core::ptr::read_volatile(flag_addr(1 - ME) as *const u32);
        if other == 1 {
            core::ptr::write_volatile(flag_addr(ME) as *mut u32, 0);
            return false;
        }
        true
    };
    if ok {
        LOCK_HELD.store(true, Ordering::Release);
    }
    ok
}

/// 主循环获锁:Dekker 完整让行 + 超时强闯兜底
pub(crate) fn line_lock_wait() {
    use core::sync::atomic::Ordering;
    unsafe {
        core::ptr::write_volatile(flag_addr(ME) as *mut u32, 1);
        riscv::asm::fence();
        let mut spins = 0u32;
        while core::ptr::read_volatile(flag_addr(1 - ME) as *const u32) == 1 {
            let turn = core::ptr::read_volatile(CTX_SLOT3 as *const u32);
            if turn as usize != ME {
                // 对方持有 turn:让行(降旗等 turn),再升旗重试
                core::ptr::write_volatile(flag_addr(ME) as *mut u32, 0);
                riscv::asm::fence();
                while core::ptr::read_volatile(CTX_SLOT3 as *const u32) as usize != ME
                    && spins < LOCK_SPIN_LIMIT
                {
                    spins += 1;
                }
                core::ptr::write_volatile(flag_addr(ME) as *mut u32, 1);
                riscv::asm::fence();
            }
            spins += 1;
            if spins > LOCK_SPIN_LIMIT {
                break; // 强闯:宁可偶尔乱码,不能死等
            }
        }
    }
    LOCK_HELD.store(true, Ordering::Release);
}

/// 放锁:把 turn 让给对方,再降旗
pub(crate) fn line_unlock() {
    use core::sync::atomic::Ordering;
    LOCK_HELD.store(false, Ordering::Release);
    unsafe {
        riscv::asm::fence();
        core::ptr::write_volatile(CTX_SLOT3 as *mut u32, (1 - ME) as u32);
        core::ptr::write_volatile(flag_addr(ME) as *mut u32, 0);
        riscv::asm::fence();
    }
}

/// 无锁直打(调用方必须已持锁):供 ISR 在单次锁窗口内拼多段输出
pub(crate) fn print_nolock(s: &str) {
    for &b in s.as_bytes() {
        if b == b'\n' {
            uart_putc(b'\r');
        }
        uart_putc(b);
    }
}

/// 打印字符串；`\n` 自动转成 `\r\n`（串口终端需要）。持跨核行锁。
pub(crate) fn print(s: &str) {
    if crate::ipc::muted() {
        return;
    }
    line_lock_wait();
    print_nolock(s);
    line_unlock();
}

/// `core::fmt::Write` 端点:持锁上下文内把格式化字节直写 UART
/// (`\n` → `\r\n` 由 [`print_nolock`] 处理)。
struct UartFmt;

impl core::fmt::Write for UartFmt {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        print_nolock(s);
        Ok(())
    }
}

/// 无锁格式化打印(调用方必须已持锁;ISR/FAULT 路径用,不受 mute 门控)。
pub(crate) fn print_fmt_nolock(args: core::fmt::Arguments<'_>) {
    let _ = UartFmt.write_fmt(args);
}

/// 格式化打印(`format_args!`);`\n` 自动转 `\r\n`。持跨核行锁(整段一次),受 mute 门控。
pub(crate) fn print_fmt(args: core::fmt::Arguments<'_>) {
    if crate::ipc::muted() {
        return;
    }
    line_lock_wait();
    let _ = UartFmt.write_fmt(args);
    line_unlock();
}
