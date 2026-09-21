//! UVC 会话：从 [`UsbDevice`] 建立可抓帧的摄像头会话，封装全部协议编排
//! （读配置描述符 → 解析流/控制实体 → 相机调校 → PROBE/COMMIT → 启动
//! → warmup），应用层不再接触 `ep0`/`sel` 细节。
//!
//! 与 [`super::device`] 类驱动的分工：`UvcCameraDriver::probe` 在拓扑遍历
//! **中途**只做轻量匹配记录（枚举时做重 I/O 初始化会连累整树遍历）；
//! 本模块的 [`open`] 在遍历**结束后**做重初始化——匹配 ≠ 就绪。

use crate::drivers::usb::device::UsbDevice;
use crate::drivers::usb::error::UsbResult;

use super::descriptor::{self, UvcStreamSelection};
use crate::drivers::usb::dwc2::Ep0;

/// 已建立的 UVC 摄像头会话：设备 + 选定的流参数。
///
/// 抓帧走 [`capture_frame`]；构造见 [`open`]。
pub struct UvcCamera {
    pub(crate) ep0: Ep0,
    pub(crate) sel: UvcStreamSelection,
    /// VC 实体 ID 与 bmControls(控制请求的寻址材料;运行期调参可用)。
    pub(crate) entities: descriptor::UvcControlEntities,
}

/// 打开摄像头会话（读配置描述符 → 解析 → 调校 → PROBE/COMMIT → 启动）。
///
/// # 参数
/// - `dev`：枚举得到的 UVC 设备——**按值移交**：会话消费设备(取其 EP0),
///   调用方此后不再使用该句柄。
/// - `prefs`：选流偏好（帧尺寸/帧率）。
pub fn open(dev: UsbDevice, prefs: &descriptor::UvcPrefs) -> UsbResult<UvcCamera> {
    let ep0 = dev.ep0;

    // 配置描述符 → 有效切片(wTotalLength 截断)
    let cfg_buf = ep0.get_configuration_descriptor(1)?;
    let cfg_total = u16::from_le_bytes([cfg_buf[2], cfg_buf[3]]) as usize;
    let cfg = &cfg_buf[..cfg_total.min(cfg_buf.len())];

    let sel = descriptor::parse_uvc_video_stream(cfg, cfg_total, prefs)?;
    let entities = descriptor::parse_uvc_control_entities(cfg, cfg_total).unwrap_or_default();

    // 早构造:后续步骤(调校/开流/warmup)全部是会话方法。
    let mut camera = UvcCamera { ep0, sel, entities };
    // 相机调校(自动白平衡/50Hz/AE;失败不阻塞——按出厂默认继续)
    let _ = camera.init_controls();
    camera.start_stream()?;
    let _ = camera.capture_frame(); // warmup:丢弃首帧
    Ok(camera)
}
