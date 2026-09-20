//! USB 主机枚举入口：初始化 DWC2 后经 [`super::hub`] 统一抽象处理根口连接/复位，
//! 再委托 [`super::topology`] 做 Hub 检测与递归端口遍历。

use core::time::Duration;

use crate::drivers::usb::dwc2;
use crate::drivers::usb::error::{UsbError, UsbResult};
use crate::drivers::usb::hub::{Hub, PortSpeed, RootHub};
use crate::drivers::usb::topology;
use crate::drivers::usb::device::UsbDevice;

/// 初始化主机并枚举摄像头：经 hub **递归遍历整条总线**，对每台设备做
/// `SET_ADDRESS`/`SET_CONFIGURATION`，返回扫描到的 UVC 摄像头。
pub fn enumerate_camera() -> UsbResult<UsbDevice> {
    dwc2::dwc2_host_init()?;
    let root = RootHub;
    if !root.wait_connect(1, Duration::from_secs(5)) {
        return Err(UsbError::Hardware(
            "root port CONNSTS=0: no device (enable VBUS e.g. GPIOB6 / cable / PHY)",
        ));
    }
    root.connect_reset_sequence(1)?;
    let speed = PortSpeed::from_status(root.port_status_w0(1)?);
    topology::enumerate_bus(speed)
}
