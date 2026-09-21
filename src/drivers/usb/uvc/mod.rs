//! USB Video Class（UVC）：配置描述符解析、`PROBE`/`COMMIT`、**Isoch IN** 抓一帧。
//!
//! 修复要点：视频端点在无数据时大量 **NAK**，须在主机侧重试
//! （见 [`crate::drivers::usb::dwc2::IsochInEp::read_uframe`]）。
//!
//! 子模块：
//! - [`setup`]：UVC 类专用 SETUP 包构造（VS PROBE/COMMIT、VC 实体控制）。
//! - [`descriptor`]：配置描述符读取与解析（VS 流/格式/端点、VC 实体）。
//! - [`control`]：摄像头 VC/PU 控制（白平衡、曝光等）。
//! - [`stream`]：`PROBE`/`COMMIT` 协商与流启停。
//! - [`capture`]：Isoch IN 抓帧与 MJPEG 帧组装。

pub mod capture;
pub mod control;
pub mod descriptor;
pub mod session;
pub mod setup;
pub mod stream;

// 重导出仅保留跨模块消费项:会话门面 + DMA 偏移(main 取 JPEG 切片用) + 选流偏好。
pub use capture::UVC_ASSEMBLED_JPEG_DMA_OFF;
pub use descriptor::UvcPrefs;
pub use session::UvcCamera;

// ---- UVC 驱动(注册进 usb::DRIVERS;设备类型留在本模块) ----

use core::cell::SyncUnsafeCell;
use core::time::Duration;

use super::device::{DeviceDriver, UsbDevice, USB_CLASS_VIDEO};
use super::error::UsbResult;

/// 选流偏好槽(main 在总线枚举前写入;默认 640×480@30fps)。
static PREFS: SyncUnsafeCell<UvcPrefs> = SyncUnsafeCell::new(UvcPrefs {
    frame_w: 640,
    frame_h: 480,
    frame_interval: Duration::from_nanos(33_333_300), // 33.33ms ≈ 30fps
});

/// 设置选流偏好;须在 `RootHub::enumerate_bus` 之前调用。
pub fn set_prefs(prefs: UvcPrefs) {
    // SAFETY: 裸机单核,枚举前由 main 写入一次。
    unsafe { *PREFS.get() = prefs };
}

/// 会话槽:probe 造好放这里,main 取走(单相机固件)。
static CAMERA: SyncUnsafeCell<Option<UvcCamera>> = SyncUnsafeCell::new(None);

/// 取走 probe 建立的摄像头会话(若无相机返回 None)。
pub fn take_camera() -> Option<UvcCamera> {
    // SAFETY: 裸机单核;枚举结束后 main 取一次。
    unsafe { (*CAMERA.get()).take() }
}

/// UVC 摄像头驱动:Video(0x0e) 类功能设备。
pub struct UvcCameraDriver;

impl DeviceDriver for UvcCameraDriver {
    fn name(&self) -> &'static str {
        "uvc-camera"
    }

    fn matches(&self, dev: &UsbDevice) -> bool {
        dev.iface_class == USB_CLASS_VIDEO
    }

    fn probe(&self, dev: UsbDevice) -> UsbResult<()> {
        log::info!(
            "[USB] camera VID={:04x} PID={:04x} addr={} speed={}",
            dev.vid,
            dev.pid,
            dev.control_ep.dev(),
            dev.speed.as_str()
        );
        let camera = UvcCamera::open(dev, unsafe { &*PREFS.get() })?;
        // SAFETY: 裸机单核,枚举流程单写、main 单读。
        unsafe { *CAMERA.get() = Some(camera) };
        Ok(())
    }
}
