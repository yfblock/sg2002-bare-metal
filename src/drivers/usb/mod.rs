//! USB 子系统：基于 Synopsys **DWC2** 的 **主机**栈；UVC 类协议在 [`uvc`]。
//!
//! 子模块：
//! - [`dwc2`]：DWC2 控制器（寄存器/bring-up/EP0 与等时传输）。
//! - [`enumerate`]：根口连接检查 + 拓扑扫描入口。
//! - [`topology`]：Hub 描述符解析与端口递归枚举。
//!
//! 单板裸机、identity 映射（VA=PA）：控制器/PHY MMIO 基址直接取
//! [`crate::platform`] 的地址表常量，无需运行时配置。
//!
//! # 公共子模块
//!
//! - [`error`]：[`error::UsbError`] / [`error::UsbResult`]。
//! - [`setup`]：标准 SETUP 字节数组；**类专用** SETUP 见 [`uvc`]。
//!
//! # 寄存器视图
//!
//! - [`dwc2_regs`]：DWC2 全局寄存器视图（基址 = [`crate::platform::DWC2_BASE`]）。
//!
//! DMA 与 CPU 视图一致性由 [`crate::arch::cache`] 的 clean / invalidate 辅助完成。
//!
//! 设备（外设）模式在孪生树 sg200x-bsp（feature `device-mode`）；本树仅主机。

use crate::drivers::usb::dwc2::regs::Dwc2Regs;

pub mod error;
pub mod hub;
pub mod setup;

pub mod device;
pub mod dwc2;
pub mod topology;
pub mod uvc;


// DWC2 寄存器一律走 [`dwc2::regs`] 的 `tock-registers` 访问器。

/// 取 DWC2 全局寄存器视图（基址为编译期常量，恒有效）。
#[inline]
pub fn dwc2_regs() -> &'static Dwc2Regs {
    unsafe { &*(crate::platform::DWC2_BASE as *const Dwc2Regs) }
}

