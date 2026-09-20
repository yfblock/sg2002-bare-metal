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
//! - [`dwc2_channel`]：主机通道寄存器块（**0** = EP0 控制、**1** = Isoch 视频）。
//! - [`cv182x_phy_regs`]：CV182x（SG2002）片内 USB2 PHY 视图。
//!
//! DMA 与 CPU 视图一致性由 [`crate::arch::cache`] 的 clean / invalidate 辅助完成。
//!
//! 设备（外设）模式在孪生树 sg200x-bsp（feature `device-mode`）；本树仅主机。

use crate::drivers::usb::dwc2::regs::{Dwc2HostChannel, Dwc2Regs, DWC2_MAX_HOST_CHANNELS};
use crate::drivers::usb::dwc2::regs::Cv182xUsb2Phy;

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

/// 取第 `ch` 号主机通道寄存器块。
///
/// # 参数
/// - `ch`：主机通道索引；本栈约定 **0** 为 EP0 控制、**1** 为 Isoch 视频。
///
/// # Panics
/// `ch` 超出 IP 支持数量时 panic（编程错误，通道号本栈写死为 0/1）。
#[inline]
pub fn dwc2_channel(ch: u32) -> &'static Dwc2HostChannel {
    let idx = ch as usize;
    assert!(idx < DWC2_MAX_HOST_CHANNELS, "invalid DWC2 host channel index");
    &dwc2_regs().hc[idx]
}

/// 取 CV182x 片内 USB2 PHY 寄存器视图（基址为编译期常量，恒有效）。
#[inline]
pub fn cv182x_phy_regs() -> &'static Cv182xUsb2Phy {
    unsafe { &*(crate::platform::CV182X_USB2_PHY_BASE as *const Cv182xUsb2Phy) }
}

