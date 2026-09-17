//! USB 总线拓扑：检测 **Hub**（含 QEMU 插入的虚拟 `usb-hub`）、读 Hub 描述符与端口状态，**递归**枚举下游设备并打印。
//!
//! 与 [`super::enumerate`] 配合：在 `dwc2_host_init` 之后由 `enumerate_topology_only()` 调用；
//! 返回 [`TopologyScanExtras`]（UVC 摄像头候选）。MSC 候选扫描已随 Bulk/MSC 路径移除，
//! 需要时见 sg200x-bsp 的 `topology.rs`。

use crate::drivers::usb::error::{UsbError, UsbResult};
use crate::drivers::usb::dwc2;
use crate::drivers::usb::setup;

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
    T.get(depth as usize).copied().unwrap_or("                      ")
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

/// USB `bDeviceClass`：Hub。
const USB_CLASS_HUB: u8 = 0x09;
/// QEMU 默认 `usb-hub`（插在根口与首个外设之间）VID/PID。
const QEMU_USB_HUB_VID: u16 = 0x0409;
const QEMU_USB_HUB_PID: u16 = 0x55aa;
/// 接口类：Video。
const USB_CLASS_VIDEO: u8 = 0x0e;

const MAX_USB_ADDR: u8 = 127;

/// 拓扑扫描附带的设备线索（UVC 摄像头）。
#[derive(Clone, Copy, Debug, Default)]
pub struct TopologyScanExtras {
    /// 枚举到的首个 **Video(0x0e)** 类功能设备（多为 UVC 摄像头）。
    pub uvc: Option<UvcEnumerated>,
}

#[derive(Clone, Copy, Debug)]
pub struct UvcEnumerated {
    pub addr: u8,
    pub ep0_mps: u32,
}

const MAX_HUB_PORTS: u8 = 16;

#[derive(Debug, Clone, Copy)]
struct ScanState {
    next_free_addr: u8,
    extras: TopologyScanExtras,
}

impl ScanState {
    const fn new() -> Self {
        Self {
            next_free_addr: 1,
            extras: TopologyScanExtras {
                uvc: None,
            },
        }
    }

    fn take_addr(&mut self) -> UsbResult<u8> {
        let a = self.next_free_addr;
        if a >= MAX_USB_ADDR {
            return Err(UsbError::Protocol("usb address space full"));
        }
        self.next_free_addr = self.next_free_addr.saturating_add(1);
        Ok(a)
    }
}

#[inline]
fn is_hub_device(class: u8, vid: u16, pid: u16) -> bool {
    class == USB_CLASS_HUB || (vid == QEMU_USB_HUB_VID && pid == QEMU_USB_HUB_PID)
}

/// 读配置描述符前 64 字节，返回首个 **INTERFACE** 描述符的 `bInterfaceClass`（无则 0）。
fn first_interface_class(ep: &dwc2::Ep0) -> UsbResult<u8> {
    let mut buf = [0u8; 64];
    ep.read(setup::get_descriptor_configuration(0, 64), &mut buf)?;
    let mut i: usize = 0;
    while i + 2 <= buf.len() {
        let bl = buf[i] as usize;
        if bl < 2 {
            break;
        }
        let ty = buf[i + 1];
        if ty == 4 && i + 6 <= buf.len() {
            return Ok(buf[i + 5]);
        }
        i = i.saturating_add(bl);
    }
    Ok(0)
}

/// Hub 描述符关键字段：端口数、`bPwrOn2PwrGood`（2ms 单位的端口上电稳定时间）。
struct HubInfo {
    nports: u8,
    pwr_on_2_pwr_good_ms: u32,
}

fn hub_info(ep: &dwc2::Ep0) -> UsbResult<HubInfo> {
    let mut buf = [0u8; 64];
    ep.read(setup::get_descriptor_hub(64), &mut buf)?;
    if buf[0] < 7 || buf[1] != setup::USB_DT_HUB {
        return Err(UsbError::Protocol("invalid hub descriptor"));
    }
    let nports = buf[2].min(MAX_HUB_PORTS);
    let pwr_on = u32::from(buf[5]).saturating_mul(2);
    Ok(HubInfo {
        nports,
        pwr_on_2_pwr_good_ms: pwr_on,
    })
}

fn hub_port_status_w0(ep: &dwc2::Ep0, port: u16) -> UsbResult<u16> {
    let mut buf = [0u8; 4];
    ep.read(setup::hub_get_port_status(port), &mut buf)?;
    Ok(u16::from_le_bytes([buf[0], buf[1]]))
}

/// 粗粒度毫秒延迟，仅用于 hub 端口上电后稳定（`bPwrOn2PwrGood`）+ 端口 reset 等待。
/// 1ms ≈ 250_000 spin-loop 在当前 sg2002 配置下经验值（与 ep0 内部 `spin_delay`
/// 校准的同一档参数：`spin_delay(20_000_000)` ≈ 80ms）。
fn spin_delay_ms(ms: u32) {
    // 2026-09-16:原 250K 迭代/ms 按 1GHz 校准,25MHz 上膨胀 50 倍(VBUS 等待 10s!)
    // 改用 rdtime 硬件定时器精确延时
    crate::arch::time::delay(core::time::Duration::from_millis(ms as u64));
}

/// USB 2.0 hub 端口速度位（`wPortStatus[10:9]`）→ 文字描述。
fn port_speed_str(status: u16) -> &'static str {
    let ls = (status >> 9) & 1;
    let hs = (status >> 10) & 1;
    match (hs, ls) {
        (1, _) => "HS",
        (_, 1) => "LS",
        _ => "FS",
    }
}

/// 在默认地址 **0** 上枚举一台设备：`SET_ADDRESS` → `SET_CONFIGURATION` → 打印信息。
///
/// - 若为 **Hub**：分配地址、读 Hub 描述符、给各端口上电、`PORT_RESET` 后递归
///   [`visit_default_depth`]（仅支持下游 **HS** 设备，FS/LS 会跳过并打日志）。
/// - 若为 **功能设备**：把 UVC 候选写入 [`TopologyScanExtras`]。
///
/// `parent_hub==0` 且 `port_on_hub==0` 表示根口直连。
fn visit_default_depth(
    depth: u8,
    parent_hub: u8,
    port_on_hub: u8,
    st: &mut ScanState,
) -> UsbResult<()> {
    let (vid, pid, ep0_mps, dev_class) = dwc2::Ep0::probe_default_addr()?;

    if parent_hub == 0 && port_on_hub == 0 {
        topo_log!(
            depth,
            "[USB] root dev@0 VID={:04x} PID={:04x} dev_class={:02x}",
            vid,
            pid,
            dev_class
        );
    } else {
        topo_log!(
            depth,
            "[USB] dev@0 (hub {} port {}) VID={:04x} PID={:04x} dev_class={:02x}",
            parent_hub,
            port_on_hub,
            vid,
            pid,
            dev_class
        );
    }

    if is_hub_device(dev_class, vid, pid) {
        let hub_addr = st.take_addr()?;
        dwc2::Ep0::set_address(hub_addr, ep0_mps)?;
        dwc2::usb_post_set_address_delay();
        let hub = dwc2::Ep0::new(u32::from(hub_addr), ep0_mps);
        hub.set_configuration(1)?;

        topo_log!(depth, "[USB]   -> Hub enumerated addr={} ep0_mps={}",
            hub_addr, ep0_mps);

        let info = hub_info(&hub)?;
        let nports = info.nports;
        let pwr_good_ms = info.pwr_on_2_pwr_good_ms.max(20); // 给 ≥20ms 富余
        topo_log!(depth, "[USB]   -> Hub descriptor: {} downstream port(s), PwrOn2PwrGood={} ms",
            nports, pwr_good_ms);

        // ① 给所有下游端口供电：USB 2.0 spec §11.11.1：hub 上电后端口默认 PowerOff，
        //    必须由 host 显式 SET_PORT_FEATURE(PORT_POWER) 才会给下游 VBUS。
        for port in 1..=nports {
            if let Err(e) = hub.hub_set_port_feature(u16::from(port), setup::HUB_PORT_FEATURE_POWER) {
                topo_log!(depth, "[USB]   -> port {} POWER fail: {:?}", port, e);
            }
        }
        // ② 等 PwrOn2PwrGood + 100ms 让下游设备 VBUS 稳定 + 自检
        spin_delay_ms(pwr_good_ms.saturating_add(100));

        for port in 1..=nports {
            let status = match hub_port_status_w0(&hub, u16::from(port)) {
                Ok(s) => s,
                Err(e) => {
                    topo_log!(
                        depth,
                        "[USB]   -> port {} GET_PORT_STATUS: {:?}",
                        port,
                        e
                    );
                    continue;
                }
            };
            let conn = status & 1 != 0;
            topo_log!(
                depth,
                "[USB]   -> port {} wPortStatus={:#06x} {}",
                port,
                status,
                if conn { "CONNECTED" } else { "empty" }
            );
            if !conn {
                continue;
            }

            // ③ 清 C_PORT_CONNECTION（连接变化位），再 PORT_RESET
            let _ = hub.hub_clear_port_feature(u16::from(port), setup::HUB_PORT_FEATURE_C_CONNECTION);

            if let Err(e) = hub.hub_set_port_feature(u16::from(port), setup::HUB_PORT_FEATURE_RESET) {
                topo_log!(depth, "[USB]   -> port {} RESET fail: {:?}", port, e);
                continue;
            }
            // USB 2.0 §7.1.7.5：TDRSTR ≥ 50ms；hub 完成 reset 后会自动置 C_PORT_RESET。
            dwc2::usb_post_hub_port_reset_delay();

            // ④ 读端口状态：必须 PORT_ENABLE=1，否则 reset 失败
            let after = match hub_port_status_w0(&hub, u16::from(port)) {
                Ok(s) => s,
                Err(e) => {
                    topo_log!(
                        depth,
                        "[USB]   -> port {} after-reset GET_PORT_STATUS: {:?}",
                        port,
                        e
                    );
                    continue;
                }
            };
            let _ = hub.hub_clear_port_feature(u16::from(port), setup::HUB_PORT_FEATURE_C_RESET);
            let enabled = (after >> 1) & 1 != 0;
            let speed = port_speed_str(after);
            topo_log!(
                depth,
                "[USB]   -> port {} after-reset wPortStatus={:#06x} ENABLED={} SPD={}",
                port,
                after,
                enabled,
                speed
            );
            if !enabled {
                continue;
            }

            // ⑤ 速度提示：HS hub 下若挂 FS/LS 设备需要 split transaction
            //    （HCSPLT 编程），当前 host 通道未实现，无法访问 EP0 → 跳过。
            if speed != "HS" {
                topo_log!(
                    depth,
                    "[USB]   -> port {} 设备非 HS（{}），HS hub 下 FS/LS 设备需要 split transaction，当前驱动暂不支持，跳过此端口枚举",
                    port,
                    speed
                );
                continue;
            }

            visit_default_depth(depth.saturating_add(1), hub_addr, port, st)?;
        }
        return Ok(());
    }

    // 普通功能设备
    let fn_addr = st.take_addr()?;
    dwc2::Ep0::set_address(fn_addr, ep0_mps)?;
    dwc2::usb_post_set_address_delay();
    let fun = dwc2::Ep0::new(u32::from(fn_addr), ep0_mps);
    fun.set_configuration(1)?;

    let iface_class = first_interface_class(&fun).unwrap_or(0);
    topo_log!(
        depth,
        "[USB]   -> function addr={} ep0_mps={} first_ifc_class={:02x}",
        fn_addr,
        ep0_mps,
        iface_class
    );

    if iface_class == USB_CLASS_VIDEO && st.extras.uvc.is_none() {
        st.extras.uvc = Some(UvcEnumerated {
            addr: fn_addr,
            ep0_mps,
        });
        topo_log!(depth, "[USB]   -> Video class device (UVC candidate) addr={}",
            fn_addr);
    }

    Ok(())
}

/// 仅递归枚举并打印拓扑。
///
/// # 返回值
/// [`TopologyScanExtras`]：发现的 UVC 候选（可能为 `None`）。
pub fn enumerate_bus_print_tree_only() -> UsbResult<TopologyScanExtras> {
    log::info!("[USB] topology: recursive hub scan (QEMU may insert virtual usb-hub on single root port)");

    let mut st = ScanState::new();
    let visit = visit_default_depth(0, 0, 0, &mut st);
    log::info!("[USB] topology: scan finished.");
    visit?;
    Ok(st.extras)
}
