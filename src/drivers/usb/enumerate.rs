//! USB 主机枚举入口：初始化 DWC2 后委托 [`super::topology`] 做树遍历。

use crate::drivers::usb::dwc2;
use crate::drivers::usb::device::UsbDevice;
use crate::drivers::usb::error::UsbResult;
use crate::drivers::usb::hub::RootHub;
use crate::drivers::usb::topology;

/// 初始化主机并枚举摄像头：树遍历整条总线,返回被类驱动接管的设备。
pub fn enumerate_camera() -> UsbResult<UsbDevice> {
    dwc2::dwc2_host_init()?;
    topology::enumerate_bus(&RootHub)
}
