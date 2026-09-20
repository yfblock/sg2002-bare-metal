//! 从 sg200x-bsp 迁移的硬件驱动(小核固件自包含,不依赖外部 crate)。
//!
//! 模块与 sg200x-bsp 一一对应,只保留 bare-metal 实际用到的部分:
//! soc(基址)、pinmux、gpio、mailbox(硬件邮箱)、USB 主机栈(DWC2+UVC)、JPU、IVE。
//! (cache/延时等架构原语在 [`crate::arch`];UART 控制台含寄存器访问在
//! [`crate::logger`]——单消费者,不设独立驱动层。)

pub mod wdt;
pub mod pinmux;
pub mod gpio;
pub mod mailbox;
pub mod usb;
pub mod jpu;
pub mod ive;
