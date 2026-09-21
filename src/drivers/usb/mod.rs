//! USB 子系统：基于 Synopsys **DWC2** 的 **主机**栈；UVC 类协议在 [`uvc`]。
//!
//! 子模块：
//! - [`dwc2`]：DWC2 控制器（寄存器/bring-up/EP0 与等时传输）。
//! - [`hub`]：Hub 抽象 + 树遍历/分派（Linux hub.c 模型）。
//! - [`root`]：根 hub（DWC2 根口）+ 总线入口 `enumerate_bus`。
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
pub mod root;
pub mod setup;

pub mod device;
pub mod dwc2;
pub mod uvc;

use device::DeviceDriver;

/// 已注册类驱动（顺序即优先级;组装点在本模块——驱动住在各自模块,
/// device 与 uvc 互不依赖）。SAFETY: 编译期定死、运行期只读;单核。
pub(crate) static DRIVERS: &[&dyn DeviceDriver] = &[&uvc::UvcCameraDriver];

// DWC2 寄存器一律走 [`dwc2::regs`] 的 `tock-registers` 访问器。

/// 取 DWC2 全局寄存器视图（基址为编译期常量，恒有效）。
#[inline]
pub fn dwc2_regs() -> &'static Dwc2Regs {
    unsafe { &*(crate::platform::DWC2_BASE as *const Dwc2Regs) }
}
