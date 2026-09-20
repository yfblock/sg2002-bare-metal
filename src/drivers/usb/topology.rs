//! USB 总线枚举器：递归遍历 hub 树，逐设备 `SET_ADDRESS`/`SET_CONFIGURATION`，
//! 经 [`super::device`] 类驱动注册表接管功能设备并沿返回值上抛。
//!
//! 三层分工：`enumerate_bus`（入口/收结果）→ `visit_default_depth`（枚举单台
//! 设备并分派 hub/功能）→ `walk_hub_ports`（遍历 hub 端口并递归下探）。
//! 端口操作机制在 [`super::hub`]，设备身份与驱动在 [`super::device`]。

use super::device::{self, UsbDevice};
use super::hub::{DeviceHub, PortSpeed, Hub, W0_CONNECTION, W0_ENABLE};
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

/// 在默认地址 **0** 上枚举一台设备并分派：
///
/// - **Hub** → 转 [`walk_hub_ports`] 递归下探；
/// - **功能设备** → 类驱动注册表匹配，被接管则作为 `Some` 上抛（先到先得）。
///
/// `parent_hub==0 && port_on_hub==0` 表示根口直连（仅影响日志）。`next_addr`
/// 为共享地址分配游标，递归全程穿过。
fn visit_default_depth(
    depth: u8,
    parent_hub: u8,
    port_on_hub: u8,
    speed: PortSpeed,
    next_addr: &mut u8,
) -> UsbResult<Option<UsbDevice>> {
    let dev = device::enumerate_device(speed, next_addr)?;

    if parent_hub == 0 && port_on_hub == 0 {
        topo_log!(depth, "[USB] root dev@0 VID={:04x} PID={:04x} dev_class={:02x}",
            dev.vid, dev.pid, dev.dev_class);
    } else {
        topo_log!(depth, "[USB] dev@0 (hub {} port {}) VID={:04x} PID={:04x} dev_class={:02x}",
            parent_hub, port_on_hub, dev.vid, dev.pid, dev.dev_class);
    }

    if dev.is_hub() {
        let hub_addr = dev.ep0.dev() as u8;
        topo_log!(depth, "[USB]   -> Hub enumerated addr={}", hub_addr);
        return walk_hub_ports(depth, &DeviceHub::new(&dev.ep0)?, hub_addr, next_addr);
    }

    // 功能设备:注册表顺序即优先级,首个匹配者胜出;驱动无状态。
    topo_log!(depth, "[USB]   -> function addr={} first_ifc_class={:02x}",
        dev.ep0.dev(), dev.iface_class);
    match device::DRIVERS.iter().find(|d| d.matches(&dev)) {
        Some(driver) => {
            topo_log!(depth, "[USB]   -> driver \"{}\" took addr={}",
                driver.name(), dev.ep0.dev());
            Ok(Some(dev))
        }
        None => Ok(None),
    }
}

/// 遍历一台 hub 的全部下游端口:供电 → 等稳定 → 逐口「扫连接 → 复位 → 查使能」,
/// 对连接且使能的端口递归 [`visit_default_depth`];返回子树被接管的设备
/// (多台先到先得)。单口失败只记日志跳过,不中断整树。
fn walk_hub_ports(
    depth: u8,
    hub_dev: &DeviceHub,
    hub_addr: u8,
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
    crate::arch::time::delay(hub_dev.pwr_good() + core::time::Duration::from_millis(100));

    let mut claimed: Option<UsbDevice> = None;
    for port in 1..=nports {
        // ③ 扫连接
        let status = match hub_dev.port_status_w0(port) {
            Ok(s) => s,
            Err(e) => {
                topo_log!(depth, "[USB]   -> port {} GET_PORT_STATUS: {:?}", port, e);
                continue;
            }
        };
        let conn = status & W0_CONNECTION != 0;
        topo_log!(depth, "[USB]   -> port {} wPortStatus={:#06x} {}",
            port, status, if conn { "CONNECTED" } else { "empty" });
        if !conn {
            continue;
        }

        // ④ 清连接变化 → 复位并等稳定(TDRSTR/TRSTRCY) → 清复位变化
        if let Err(e) = hub_dev.connect_reset_sequence(port) {
            topo_log!(depth, "[USB]   -> port {} reset sequence: {:?}", port, e);
            continue;
        }

        // ⑤ 复位后必须 PORT_ENABLE=1,否则该口复位失败
        let after = match hub_dev.port_status_w0(port) {
            Ok(s) => s,
            Err(e) => {
                topo_log!(depth, "[USB]   -> port {} after-reset GET_PORT_STATUS: {:?}", port, e);
                continue;
            }
        };
        let enabled = after & W0_ENABLE != 0;
        let child_speed = PortSpeed::from_status(after);
        topo_log!(depth, "[USB]   -> port {} after-reset wPortStatus={:#06x} ENABLED={} SPD={}",
            port, after, enabled, child_speed.as_str());
        if !enabled {
            continue;
        }

        let sub = visit_default_depth(depth.saturating_add(1), hub_addr, port, child_speed, next_addr)?;
        claimed = claimed.or(sub); // 多台候选先到先得
    }
    Ok(claimed)
}

/// 递归枚举整条总线，返回被类驱动接管的设备；无人接管 = `Err(Protocol)`。
pub fn enumerate_bus(root_speed: PortSpeed) -> UsbResult<UsbDevice> {
    log::info!("[USB] topology: recursive hub scan (QEMU may insert virtual usb-hub on single root port)");

    let mut next_addr: u8 = 1;
    let visit = visit_default_depth(0, 0, 0, root_speed, &mut next_addr);
    log::info!("[USB] topology: scan finished.");
    visit?.ok_or(UsbError::Protocol("no device claimed by any class driver"))
}
