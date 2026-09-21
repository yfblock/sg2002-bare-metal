//! USB 设备抽象与类驱动注册表：设备身份（[`UsbDevice`]）、公共枚举序列
//! （[`enumerate_device`]）、按类接管功能设备的 [`DeviceDriver`] 注册表。
//!
//! 与 [`super::hub`] 的端口抽象互补：Hub 抽象统一「上游端口怎么操作」，
//! 本模块统一「端口后面的设备是什么、谁接管」。

use super::dwc2::ControlEp;
use super::error::{UsbError, UsbResult};
use super::hub::PortSpeed;
use super::setup::StdRequest;

/// QEMU 默认 `usb-hub`（插在根口与外设之间）VID/PID。
const QEMU_USB_HUB_VID: u16 = 0x0409;
const QEMU_USB_HUB_PID: u16 = 0x55aa;

/// USB 地址上限（7 位寻址）。
const MAX_USB_ADDR: u8 = 127;
/// `bDeviceClass`：Hub。
pub const USB_CLASS_HUB: u8 = 0x09;
/// 接口类：Video（UVC 驱动的匹配条件）。
pub const USB_CLASS_VIDEO: u8 = 0x0E;
/// `bDescriptorType`：接口描述符（首接口类扫描用）。
pub const USB_DT_INTERFACE: u8 = 4;

/// 一台已枚举设备：身份 + 控制端点句柄（枚举后身份不再散架为元组）。
#[derive(Clone, Copy)]
pub struct UsbDevice {
    pub control_ep: ControlEp,
    pub vid: u16,
    pub pid: u16,
    pub dev_class: u8,
    /// 首接口类（功能设备的分类依据；0 = 未知）。
    pub iface_class: u8,
    /// 上游端口速度。
    pub speed: PortSpeed,
}

impl UsbDevice {
    /// 是否 hub（类码 0x09 或 QEMU 虚拟 hub VID:PID）。
    pub fn is_hub(&self) -> bool {
        self.dev_class == USB_CLASS_HUB
            || (self.vid == QEMU_USB_HUB_VID && self.pid == QEMU_USB_HUB_PID)
    }
}

/// 公共枚举序列（在默认地址 0 上）：探测 → `SET_ADDRESS` → `SET_CONFIGURATION`
/// → 读首接口类，产出完整设备身份。hub 与功能设备共用。
pub(crate) fn enumerate_device(speed: PortSpeed, next_addr: &mut u8) -> UsbResult<UsbDevice> {
    let (vid, pid, control_ep_mps, dev_class) = ControlEp::probe_default_addr()?;
    let addr = *next_addr;
    if addr >= MAX_USB_ADDR {
        return Err(UsbError::Protocol("usb address space full"));
    }
    *next_addr = addr.saturating_add(1);
    // 默认地址句柄上发 SET_ADDRESS,成功后句柄自身迁移到新地址。
    let mut control_ep = ControlEp::new(control_ep_mps);
    control_ep.set_address(addr)?;
    // SET_ADDRESS 后恢复延时:USB 2.0 要求下一事务前用新地址;Linux 主机栈
    // 常用 ~10ms,这里给 50ms 富余(原迭代计数版按 1GHz 校准,25MHz 上
    // 实测 ~27s 纯属过杀)。
    crate::arch::time::delay(core::time::Duration::from_millis(50));
    control_ep.set_configuration(1)?;
    let iface_class = first_interface_class(&control_ep).unwrap_or(0);
    Ok(UsbDevice {
        control_ep,
        vid,
        pid,
        dev_class,
        iface_class,
        speed,
    })
}

/// 读配置描述符首接口的 `bInterfaceClass`。
fn first_interface_class(ep: &ControlEp) -> UsbResult<u8> {
    let mut buf = [0u8; 64];
    ep.read(StdRequest::get_descriptor_configuration(0, 64), &mut buf)?;
    let mut i: usize = 0;
    while i + 2 <= buf.len() {
        let bl = buf[i] as usize;
        if bl < 2 {
            break;
        }
        let ty = buf[i + 1];
        if ty == USB_DT_INTERFACE && i + 6 <= buf.len() {
            return Ok(buf[i + 5]);
        }
        i = i.saturating_add(bl);
    }
    Ok(0)
}

/// 类驱动：声明对已枚举功能设备的匹配条件。
///
/// 注册表 [`DRIVERS`] 顺序即优先级;首个 `matches` 的驱动胜出,
/// 接管设备沿遍历返回值上抛——驱动无状态、无副作用。
/// (将来驱动需要接管动作/类初始化时再扩 `probe`,需求拉动。)
pub trait DeviceDriver: Sync {
    fn name(&self) -> &'static str;
    /// 匹配判定（接口类/设备类/VID:PID）。
    fn matches(&self, dev: &UsbDevice) -> bool;
}

/// UVC 摄像头驱动：Video(0x0e) 类功能设备。
struct UvcCameraDriver;

impl DeviceDriver for UvcCameraDriver {
    fn name(&self) -> &'static str {
        "uvc-camera"
    }

    fn matches(&self, dev: &UsbDevice) -> bool {
        dev.iface_class == USB_CLASS_VIDEO
    }
}

/// 已注册类驱动（顺序即优先级）。
/// SAFETY: 注册表编译期定死、运行期只读;裸机单核无并发访问。
pub(crate) static DRIVERS: &[&dyn DeviceDriver] = &[&UvcCameraDriver];

// SAFETY 补充见上
