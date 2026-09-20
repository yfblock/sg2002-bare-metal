//! USB 总线拓扑：检测 **Hub**（含 QEMU 插入的虚拟 `usb-hub`）、读 Hub 描述符与端口状态，**递归**枚举下游设备并打印。
//! 返回 [`super::device::UsbDevice`]（UVC 驱动接管的摄像头;未找到 = Err）。
//!
//! 与 [`super::enumerate`] 配合：在 `dwc2_host_init` 之后由 `enumerate_camera()` 调用。

use super::hub::Hub;
use crate::drivers::usb::error::{UsbError, UsbResult};

/// 拓扑日志缩进（每级 2 空格）。
#[inline]
fn topo_indent(depth: u8) -> &'static str {
    const T: [&str; 12] = [
        "",
        "  ",
        "    ",
        "      ",
        "        ",
        "          ",
        "            ",
        "              ",
        "                ",
        "                  ",
        "                    ",
        "                      ",
    ];
    T[(depth as usize).min(T.len() - 1)]
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

/// 读配置描述符前 64 字节，返回首个 **INTERFACE** 描述符的 `bInterfaceClass`（无则 0）。
/// 在默认地址 **0** 上枚举一台设备：`SET_ADDRESS` → `SET_CONFIGURATION` → 打印信息。
///
/// - 若为 **Hub**：分配地址、读 Hub 描述符、给各端口上电、`PORT_RESET` 后递归
///   [`visit_default_depth`]（仅支持下游 **HS** 设备，FS/LS 会跳过并打日志）。
/// - 若为 **功能设备**：被类驱动接管则作为返回值上抛(多台先到先得)。
///
/// `parent_hub==0` 且 `port_on_hub==0` 表示根口直连。`next_addr` 为共享
/// 地址分配游标(递归全程穿过);返回本子树被接管的设备(无则 None)。
fn visit_default_depth(
    depth: u8,
    parent_hub: u8,
    port_on_hub: u8,
    speed: super::hub::PortSpeed,
    next_addr: &mut u8,
) -> UsbResult<Option<super::device::UsbDevice>> {
    let dev = super::device::enumerate_device(speed, next_addr)?;

    if parent_hub == 0 && port_on_hub == 0 {
        topo_log!(
            depth,
            "[USB] root dev@0 VID={:04x} PID={:04x} dev_class={:02x}",
            dev.vid, dev.pid, dev.dev_class
        );
    } else {
        topo_log!(
            depth,
            "[USB] dev@0 (hub {} port {}) VID={:04x} PID={:04x} dev_class={:02x}",
            parent_hub, port_on_hub, dev.vid, dev.pid, dev.dev_class
        );
    }

    if dev.is_hub() {
        let hub_addr = dev.ep0.dev() as u8;
        topo_log!(depth, "[USB]   -> Hub enumerated addr={}", hub_addr);
        return walk_hub_ports(depth, &super::hub::DeviceHub::new(&dev.ep0)?, hub_addr, next_addr);
    }

    // 普通功能设备:类驱动注册表分发(顺序即优先级,首个匹配者胜出;
    // 驱动无状态,接管结果沿返回值上抛)。
    topo_log!(
        depth,
        "[USB]   -> function addr={} first_ifc_class={:02x}",
        dev.ep0.dev(),
        dev.iface_class
    );
    if let Some(driver) = super::device::DRIVERS.iter().find(|d| d.matches(&dev)) {
        topo_log!(depth, "[USB]   -> driver \"{}\" took addr={}",
            driver.name(), dev.ep0.dev());
        return Ok(Some(dev));
    }

    Ok(None)
}

/// 遍历一台 hub 的全部下游端口:供电 → 等稳定 → 逐口扫连接/复位/使能,
/// 对连接且使能的端口递归 [`visit_default_depth`];返回子树被接管的设备
/// (多台先到先得)。单口失败只记日志跳过,不中断整树。
fn walk_hub_ports(
    depth: u8,
    hub_dev: &super::hub::DeviceHub,
    hub_addr: u8,
    next_addr: &mut u8,
) -> UsbResult<Option<super::device::UsbDevice>> {
    let nports = hub_dev.nports();
    let pwr_good_ms = hub_dev.pwr_good_ms().max(20); // 给 ≥20ms 富余
    topo_log!(depth, "[USB]   -> Hub descriptor: {} downstream port(s), PwrOn2PwrGood={} ms",
        nports, pwr_good_ms);

    // ① 给所有下游端口供电(USB 2.0 §11.11.1:hub 端口默认 PowerOff)
    for port in 1..=nports {
        if let Err(e) = hub_dev.port_power(port) {
            topo_log!(depth, "[USB]   -> port {} POWER fail: {:?}", port, e);
        }
    }
    // ② 等 PwrOn2PwrGood + 100ms 让下游设备 VBUS 稳定 + 自检
    crate::arch::time::delay(hub_dev.pwr_good() + core::time::Duration::from_millis(100));

    let mut claimed: Option<super::device::UsbDevice> = None;
    for port in 1..=nports {
        // ③ 扫连接
        let status = match hub_dev.port_status_w0(port) {
            Ok(s) => s,
            Err(e) => {
                topo_log!(depth, "[USB]   -> port {} GET_PORT_STATUS: {:?}", port, e);
                continue;
            }
        };
        let conn = status & super::hub::W0_CONNECTION != 0;
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
        let enabled = after & super::hub::W0_ENABLE != 0;
        let child_speed = super::hub::PortSpeed::from_status(after);
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

/// 递归枚举整条总线并打印拓扑。
///
/// # 返回值
/// 扫描到的 UVC 摄像头；拓扑中无 Video 类设备时返回 `Err(Protocol)`。
pub fn enumerate_bus(root_speed: super::hub::PortSpeed) -> UsbResult<super::device::UsbDevice> {
    log::info!("[USB] topology: recursive hub scan (QEMU may insert virtual usb-hub on single root port)");

    let mut next_addr: u8 = 1;
    let visit = visit_default_depth(0, 0, 0, root_speed, &mut next_addr);
    log::info!("[USB] topology: scan finished.");
    match visit? {
        Some(cam) => Ok(cam),
        None => Err(UsbError::Protocol("no device claimed by any class driver")),
    }
}
