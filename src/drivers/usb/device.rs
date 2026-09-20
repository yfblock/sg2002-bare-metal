//! USB 设备抽象与类驱动注册表：设备身份（[`UsbDevice`]）、公共枚举序列
//! （[`enumerate_device`]）、按类接管功能设备的 [`DeviceDriver`] 注册表。
//!
//! 与 [`super::hub`] 的端口抽象互补：Hub 抽象统一「上游端口怎么操作」，
//! 本模块统一「端口后面的设备是什么、谁接管」。

use super::dwc2::{self, Ep0};
use super::error::{UsbError, UsbResult};
use super::hub::PortSpeed;
use super::setup;

/// USB Hub 类码（`bDeviceClass`）。
const USB_CLASS_HUB: u8 = 0x09;
/// QEMU 默认 `usb-hub`（插在根口与外设之间）VID/PID。
const QEMU_USB_HUB_VID: u16 = 0x0409;
const QEMU_USB_HUB_PID: u16 = 0x55aa;

/// USB 地址上限（7 位寻址）。
const MAX_USB_ADDR: u8 = 127;

/// 一台已枚举设备：身份 + 控制端点句柄（枚举后身份不再散架为元组）。
#[derive(Clone, Copy)]
pub struct UsbDevice {
    pub ep0: Ep0,
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

/// 枚举状态：USB 地址分配器 + 各驱动接管的设备。
pub(crate) struct ScanState {
    next_free_addr: u8,
    /// UVC 驱动接管的首台摄像头。
    pub(crate) uvc: Option<UsbDevice>,
}

impl ScanState {
    pub(crate) const fn new() -> Self {
        Self {
            next_free_addr: 1,
            uvc: None,
        }
    }

    /// 分配下一个 USB 设备地址（单调递增，耗尽报错）。
    pub(crate) fn take_addr(&mut self) -> UsbResult<u8> {
        let addr = self.next_free_addr;
        if addr >= MAX_USB_ADDR {
            return Err(UsbError::Protocol("usb address space full"));
        }
        self.next_free_addr = self.next_free_addr.saturating_add(1);
        Ok(addr)
    }
}

/// 公共枚举序列（在默认地址 0 上）：探测 → `SET_ADDRESS` → `SET_CONFIGURATION`
/// → 读首接口类，产出完整设备身份。hub 与功能设备共用。
pub(crate) fn enumerate_device(speed: PortSpeed, st: &mut ScanState) -> UsbResult<UsbDevice> {
    let (vid, pid, ep0_mps, dev_class) = Ep0::probe_default_addr()?;
    let addr = st.take_addr()?;
    Ep0::set_address(addr, ep0_mps)?;
    dwc2::usb_post_set_address_delay();
    let ep0 = Ep0::new(u32::from(addr), ep0_mps);
    ep0.set_configuration(1)?;
    let iface_class = first_interface_class(&ep0).unwrap_or(0);
    Ok(UsbDevice {
        ep0,
        vid,
        pid,
        dev_class,
        iface_class,
        speed,
    })
}

/// 读配置描述符首接口的 `bInterfaceClass`。
fn first_interface_class(ep: &Ep0) -> UsbResult<u8> {
    let mut buf = [0u8; 64];
    ep.read(setup::get_descriptor_configuration(0, 64), &mut buf)?;
    let mut i: usize = 0;
    while i + 2 <= buf.len() {
        let bl = buf[i] as usize;
        if bl < 2 {
            break;
        }
        let ty = buf[i + 1];
        if ty == setup::USB_DT_INTERFACE && i + 6 <= buf.len() {
            return Ok(buf[i + 5]);
        }
        i = i.saturating_add(bl);
    }
    Ok(0)
}

/// 类驱动：对已枚举的功能设备做匹配与接管。
///
/// 注册表 [`DRIVERS`] 顺序即优先级；首个 `matches` 的驱动 `probe` 接管。
pub trait DeviceDriver: Sync {
    fn name(&self) -> &'static str;
    /// 匹配判定（接口类/设备类/VID:PID）。
    fn matches(&self, dev: &UsbDevice) -> bool;
    /// 接管设备（记录候选等）；`Err` 中断总线遍历。
    fn probe(&self, dev: &UsbDevice, st: &mut ScanState) -> UsbResult<()>;
}

/// UVC 摄像头驱动：首个 Video(0x0e) 类功能设备胜出。
struct UvcCameraDriver;

impl DeviceDriver for UvcCameraDriver {
    fn name(&self) -> &'static str {
        "uvc-camera"
    }

    fn matches(&self, dev: &UsbDevice) -> bool {
        dev.iface_class == setup::USB_CLASS_VIDEO
    }

    fn probe(&self, dev: &UsbDevice, st: &mut ScanState) -> UsbResult<()> {
        if st.uvc.is_none() {
            st.uvc = Some(*dev);
        }
        Ok(())
    }
}

/// 已注册类驱动（顺序即优先级）。
/// SAFETY: 注册表编译期定死、运行期只读;裸机单核无并发访问。
pub(crate) static DRIVERS: &[&dyn DeviceDriver] = &[&UvcCameraDriver];

// SAFETY 补充见上
