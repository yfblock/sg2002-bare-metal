//! USB 总线树遍历：hub 为节点、端口为边——根口与 hub 端口走同一条
//! [`Hub::enumerate_child`](super::hub::Hub::enumerate_child) 路径取出子设备,
//! 经 [`super::device`] 类驱动注册表接管并沿返回值上抛。
//!
//! 三层分工:`enumerate_bus`(入口/根口 bring-up/收结果)→ `dispatch_device`
//! (单台设备分派 hub/功能)→ `walk_hub_ports`(遍历 hub 端口并递归下探)。

use core::time::Duration;

use super::device::{UsbDevice, DRIVERS};
use super::hub::{DeviceHub, Hub, RootHub};
use crate::drivers::usb::error::{UsbError, UsbResult};

/// 拓扑日志缩进（每级 2 空格，封顶 12 级）。
#[inline]
fn topo_indent(depth: u8) -> &'static str {
    const SPACES: &str = "                        ";
    &SPACES[..2 * (depth as usize).min(SPACES.len() / 2)]
}

macro_rules! topo_log {
    ($depth:expr, $($tt:tt)*) => {
        ::log::info!(
            target: "sg200x_bsp::usb::topology",
            "{}{}",
            topo_indent($depth),
            format_args!($($tt)*)
        )
    };
}

/// 分派一台已枚举的设备:
///
/// - **Hub** → 转 [`walk_hub_ports`] 递归下探;
/// - **功能设备** → 类驱动注册表匹配,被接管则 `Some` 上抛(先到先得)。
fn dispatch_device(depth: u8, dev: UsbDevice, next_addr: &mut u8) -> UsbResult<Option<UsbDevice>> {
    topo_log!(depth, "[USB] dev VID={:04x} PID={:04x} dev_class={:02x}",
        dev.vid, dev.pid, dev.dev_class);

    if dev.is_hub() {
        let hub_addr = dev.ep0.dev() as u8;
        topo_log!(depth, "[USB]   -> Hub addr={}", hub_addr);
        return walk_hub_ports(depth, &DeviceHub::new(&dev.ep0)?, next_addr);
    }

    // 功能设备:注册表顺序即优先级,首个匹配者胜出;驱动无状态。
    topo_log!(depth, "[USB]   -> function addr={} first_ifc_class={:02x}",
        dev.ep0.dev(), dev.iface_class);
    match DRIVERS.iter().find(|d| d.matches(&dev)) {
        Some(driver) => {
            topo_log!(depth, "[USB]   -> driver \"{}\" took addr={}",
                driver.name(), dev.ep0.dev());
            Ok(Some(dev))
        }
        None => Ok(None),
    }
}

/// 遍历一台 hub 的全部下游端口:供电 → 等稳定 → 逐口取子设备
/// ([`Hub::enumerate_child`])并递归 [`dispatch_device`];返回子树被接管
/// 的设备(多台先到先得)。单口失败只记日志跳过,不中断整树。
fn walk_hub_ports(
    depth: u8,
    hub_dev: &DeviceHub,
    next_addr: &mut u8,
) -> UsbResult<Option<UsbDevice>> {
    let nports = hub_dev.nports();
    topo_log!(depth, "[USB]   -> Hub descriptor: {} downstream port(s), PwrOn2PwrGood={} ms",
        nports, hub_dev.pwr_good_ms().max(20)); // 上电稳定时间给 ≥20ms 富余

    // ① 给所有下游端口供电(USB 2.0 §11.11.1:hub 端口默认 PowerOff)
    for port in 1..=nports {
        if let Err(e) = hub_dev.port_power(port) {
            topo_log!(depth, "[USB]   -> port {} POWER fail: {:?}", port, e);
        }
    }
    // ② 等 PwrOn2PwrGood + 100ms 让下游设备 VBUS 稳定 + 自检
    crate::arch::time::delay(hub_dev.pwr_good() + Duration::from_millis(100));

    let mut claimed: Option<UsbDevice> = None;
    for port in 1..=nports {
        let Some(child) = hub_dev.enumerate_child(port, next_addr)? else {
            continue;
        };
        let sub = dispatch_device(depth.saturating_add(1), child, next_addr)?;
        claimed = claimed.or(sub); // 多台候选先到先得
    }
    Ok(claimed)
}

/// 树遍历整条总线：根口等连接 → 取根口子设备 → 分派;返回被类驱动接管的
/// 设备;根口无设备/无人接管 = `Err`。
pub fn enumerate_bus(root: &RootHub) -> UsbResult<UsbDevice> {
    log::info!("[USB] topology: recursive hub scan (QEMU may insert virtual usb-hub on single root port)");

    if !root.wait_connect(1, Duration::from_secs(5)) {
        return Err(UsbError::Hardware(
            "no device on root port (enable VBUS e.g. GPIOB6 / cable / PHY)",
        ));
    }
    let mut next_addr: u8 = 1;
    let child = root
        .enumerate_child(1, &mut next_addr)?
        .ok_or(UsbError::Protocol("root port child not enabled"))?;
    log::info!("[USB] topology: scan finished.");
    dispatch_device(0, child, &mut next_addr)?
        .ok_or(UsbError::Protocol("no device claimed by any class driver"))
}
