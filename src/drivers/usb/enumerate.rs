//! USB 主机枚举入口：初始化 DWC2 后委托 [`super::topology`] 做 Hub 检测与递归端口遍历。

use crate::drivers::usb::error::{UsbError, UsbResult};
use tock_registers::interfaces::Readable;

use crate::drivers::usb::dwc2::{self, regs::HPRT0};
use crate::drivers::usb::topology::{self, TopologyScanExtras};

/// 初始化主机并做拓扑扫描。
///
/// 会打印复位前后 `HPRT0` 调试信息。
///
/// # 返回值
/// - [`TopologyScanExtras`]：枚举过程中发现的 UVC 设备线索（可能为 `None`）。
pub fn enumerate_topology_only() -> UsbResult<TopologyScanExtras> {
    dwc2::dwc2_host_init()?;
    check_root_device_connected()?;
    let p = dwc2::hprt0();
    log::debug!("USB-DBG pre-reset HPRT0={:#010x} CONNSTS={} ENABLE={} SPD={} (0=HS 1=FS 2=LS)",
        p.get(),
        p.is_set(HPRT0::CONNSTS),
        p.is_set(HPRT0::ENA),
        p.read(HPRT0::SPD),);
    dwc2::debug_dump_root_port_hw("pre-reset");
    dwc2::dwc2_host_root_bus_reset_pulse()?;
    let p = dwc2::hprt0();
    log::debug!("USB-DBG post-reset HPRT0={:#010x} CONNSTS={} ENABLE={} SPD={} (0=HS 1=FS 2=LS)",
        p.get(),
        p.is_set(HPRT0::CONNSTS),
        p.is_set(HPRT0::ENA),
        p.read(HPRT0::SPD),);
    dwc2::debug_dump_root_port_hw("post-reset");
    topology::enumerate_bus_print_tree_only()
}

/// 轮询 `HPRT0.CONNSTS`，直到根口报告已连接设备或超时。
///
/// 超时常见于未供电 / 无线缆 / PHY 未切到 host；日志中会打印 `HPRT0` 快照。
fn check_root_device_connected() -> UsbResult<()> {
    const SPIN_PER_TRY: u32 = 256;
    /// 轮询次数（粗粒度）；慢速 Hub/上电后可等到 CONNSTS。
    const TRIES: u32 = 400_000;

    for t in 0..TRIES {
        if dwc2::hprt0().is_set(HPRT0::CONNSTS) {
            if t > 0 {
                log::debug!("USB-DBG root connect after {} polls HPRT0={:#010x}",
                    t, dwc2::hprt0().get());
            }
            return Ok(());
        }
        for _ in 0..SPIN_PER_TRY {
            core::hint::spin_loop();
        }
    }

    let p = dwc2::hprt0();
    log::debug!("USB-DBG no root connect: HPRT0={:#010x} PWR={} CONNSTS={} LNSTS={}",
        p.get(),
        p.is_set(HPRT0::PWR),
        p.is_set(HPRT0::CONNSTS),
        p.read(HPRT0::LNSTS),);
    dwc2::debug_dump_root_port_hw("no root connect");
    Err(UsbError::Hardware(
        "HPRT0 CONNSTS=0: no device on root port (enable VBUS e.g. GPIOB6 / cable / PHY)",
    ))
}
