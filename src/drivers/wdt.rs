//! DW APB 看门狗（SG2002 dts `cv-wd@0x3010000`，compatible `snps,dw-wdt`）。
//!
//! 用途：总线/互连级 wedge（两核同灭、mtimer 都进不来、无 FAULT）时把
//! 「静默死机等人断电」变成整片自愈复位——小核 mtimer 心跳每 10ms 喂狗，
//! 心跳停 = 喂狗停 = 复位重启（2026-09-20 遥测定位的间歇性双核挂死对策）。
//! 注意复位是全片性质：b2sbm 双核测试期间触发会让测试 FAIL 并重启板子，
//! 但板子回到可继续操作的干净状态，而不是 wedged 死机。

use tock_registers::{register_bitfields, register_structs};
use tock_registers::interfaces::Writeable;
use tock_registers::registers::{ReadWrite, WriteOnly};

register_bitfields![u32,
    /// WDT_CR（0x00）：控制。
    pub CR [
        /// 看门狗使能。
        EN OFFSET(0) NUMBITS(1) [],
        /// 响应模式：0 = 硬件复位，1 = 先中断再复位。取 0（直复位）。
        RMOD OFFSET(1) NUMBITS(1) [],
        /// 复位脉冲长度（00=2 pclk … 11=256 pclk），默认即可。
        RPL OFFSET(2) NUMBITS(2) [],
    ],
    /// WDT_TORR（0x04）：超时档位，周期 = (2^(8+TORR) + 1) 个 pclk。
    pub TORR [
        TIMEOUT OFFSET(0) NUMBITS(4) [],
    ],
];

register_structs! {
    /// DW APB WDT 寄存器映射（使能/档位/喂狗）。
    pub DwApbWdt {
        (0x00 => pub cr: ReadWrite<u32, CR::Register>),
        (0x04 => pub torr: ReadWrite<u32, TORR::Register>),
        (0x08 => _reserved: [u32; 1]),
        (0x0c => pub crr: WriteOnly<u32>),
        (0x10 => @END),
    }
}

/// WDT MMIO 基址（SG2002 dts `cv-wd@0x3010000`）。
use crate::platform::WDT_BASE;
/// 喂狗魔法值（DW WDT 规定写 0x76 重装计数器）。
const KICK_MAGIC: u32 = 0x76;

/// 取 WDT 寄存器视图（基址为编译期常量，恒有效）。
#[inline]
fn wdt_regs() -> &'static DwApbWdt {
    unsafe { &*(WDT_BASE as *const DwApbWdt) }
}

/// 启动看门狗：最大档 TORR=15（2^23+1 周期；pclk 25MHz 下 ≈0.34s），
/// 硬件复位模式（RMOD=0）。由 `arch::time::init_heartbeat` 调用，
/// 此后 mtimer 心跳负责喂狗。
pub fn start() {
    let wdt = wdt_regs();
    wdt.torr.write(TORR::TIMEOUT.val(15));
    wdt.cr.write(CR::EN::SET);
    kick();
}

/// 喂狗（写 CRR=0x76 重装）。心跳每 10ms 调用；wedge 时心跳停 → 复位。
#[inline]
pub fn kick() {
    wdt_regs().crr.set(KICK_MAGIC);
}
