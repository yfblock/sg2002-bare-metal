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
/// - **功能设备** → 类驱动注册表匹配,被接管则上抛;无人接管 =
///   `Err(NotPresent)`(空枝软信号)。
fn dispatch_device(depth: u8, dev: UsbDevice, next_addr: &mut u8) -> UsbResult<UsbDevice> {
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
            Ok(dev)
        }
        None => Err(UsbError::NotPresent),
    }
}

/// 遍历一台 hub 的全部下游端口:供电 → 等稳定 → 逐口取子设备
/// ([`Hub::enumerate_child`])并递归 [`dispatch_device`];返回子树被接管
/// 的设备(多台先到先得)。单口 NotPresent 只跳过不中断;子树无人
/// 接管 = `Err(NotPresent)`。
fn walk_hub_ports(
    depth: u8,
    hub_dev: &DeviceHub,
    next_addr: &mut u8,
) -> UsbResult<UsbDevice> {
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
        let child = match hub_dev.enumerate_child(port, next_addr) {
            Ok(d) => d,
            Err(UsbError::NotPresent) => continue, // 空口/端口级失败:跳过
            Err(e) => return Err(e),               // 硬失败:中断整树
        };
        match dispatch_device(depth.saturating_add(1), child, next_addr) {
            Ok(d) => {
                if claimed.is_none() {
                    claimed = Some(d); // 多台候选先到先得
                }
            }
            Err(UsbError::NotPresent) => {} // 子树无人接管:继续扫其他口
            Err(e) => return Err(e),
        }
    }
    claimed.ok_or(UsbError::NotPresent)
}

/// 树遍历整条总线：根口上电 → 等连接 → 取根口子设备 → 分派;返回被类驱动
/// 接管的设备;根口无设备/无人接管 = `Err`。「上电→等连→取子」与
/// [`walk_hub_ports`] 同形。
pub fn enumerate_bus(root: &RootHub) -> UsbResult<UsbDevice> {
    log::info!("[USB] topology: recursive hub scan (QEMU may insert virtual usb-hub on single root port)");

    root.port_power(1)?; // HPRT0.PWR(controller bring-up 不再代劳)
    if !root.wait_connect(1, Duration::from_secs(5)) {
        return Err(UsbError::Hardware(
            "no device on root port (enable VBUS e.g. GPIOB6 / cable / PHY)",
        ));
    }
    let mut next_addr: u8 = 1;
    let child = root.enumerate_child(1, &mut next_addr).map_err(|e| match e {
        UsbError::NotPresent => UsbError::Protocol("root port child not enabled"),
        e => e,
    })?;
    log::info!("[USB] topology: scan finished.");
    dispatch_device(0, child, &mut next_addr).map_err(|e| match e {
        UsbError::NotPresent => UsbError::Protocol("no device claimed by any class driver"),
        e => e,
    })
}
