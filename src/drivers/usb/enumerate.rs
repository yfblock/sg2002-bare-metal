//! USB 主机枚举入口：初始化 DWC2 后委托 [`super::topology`] 做 Hub 检测与递归端口遍历。

use core::time::Duration;
use tock_registers::interfaces::Readable;

use crate::drivers::usb::dwc2::{self, regs::HPRT0};
use crate::drivers::usb::error::{UsbError, UsbResult};
use crate::drivers::usb::topology::{self, UvcEnumerated};

/// 初始化主机并做拓扑扫描。
pub fn enumerate_topology_only() -> UsbResult<UvcEnumerated> {
    dwc2::dwc2_host_init()?;
    check_root_device_connected()?;
    dwc2::dwc2_host_root_bus_reset_pulse()?;
    topology::enumerate_bus_print_tree_only()
}

/// 轮询 `HPRT0.CONNSTS`，直到根口报告已连接设备或超时。
fn check_root_device_connected() -> UsbResult<()> {
    let t0 = crate::arch::time::rdtime();
    let timeout = Duration::from_secs(5);

    loop {
        if dwc2::hprt0().is_set(HPRT0::CONNSTS) {
            return Ok(());
        }
        if crate::arch::time::elapsed_since(t0) >= timeout {
            break;
        }
        core::hint::spin_loop();
    }

    Err(UsbError::Hardware(
        "HPRT0 CONNSTS=0: no device on root port (enable VBUS e.g. GPIOB6 / cable / PHY)",
    ))
}
