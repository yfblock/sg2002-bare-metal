//! 板级(SG2002):跨核 UART 控制台(打印 + Dekker 行锁)、USB PHY/时钟/VBUS/pinmux 初始化。
//!
//! 分层:`crate::arch` = 核级(ISA/trap/cache/时间);`crate::drivers` = 设备驱动;
//! 本模块 = 板级(UART 控制台含 DW 8250 寄存器访问 + 跨核行锁,单消费者不拆驱动层)。

pub mod uart;
pub mod platform;

pub use platform::platform_init;
