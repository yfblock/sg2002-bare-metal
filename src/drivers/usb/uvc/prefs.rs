//! 选流/抓帧的原子偏好开关与 setter。
//!
//! 各项须在 [`crate::drivers::usb::uvc::parse_uvc_video_stream`] 之前设置，
//! 解析与 alt 回选时读取。

pub static PREFERRED_MAX_PIXELS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// 设置 [`PREFERRED_MAX_PIXELS`]。必须在 [`crate::drivers::usb::uvc::parse_uvc_video_stream`] 之前调用。
pub fn set_preferred_max_pixels(p: u32) {
    PREFERRED_MAX_PIXELS.store(p, core::sync::atomic::Ordering::Relaxed);
}

/// 上层可设置"首选精确帧尺寸"（宽）。与 [`PREFERRED_FRAME_H`] 同时非 0 时，
/// [`crate::drivers::usb::uvc::parse_uvc_video_stream`] 的 `rank()` 对精确匹配该尺寸的 frame 给最高分。
/// 0 表示不启用精确尺寸偏好（退回 [`PREFERRED_MAX_PIXELS`] 逻辑）。
///
/// 典型：JPU 1 MiB DMA pool 把可硬件解码的分辨率限制在 ~640×480，超出会
/// `jpu_alloc` 失败；故 camera+JPU 路径应 `set_preferred_frame_size(640, 480)`。
pub static PREFERRED_FRAME_W: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);
/// 首选精确帧尺寸（高），见 [`PREFERRED_FRAME_W`]。
pub static PREFERRED_FRAME_H: core::sync::atomic::AtomicU16 =
    core::sync::atomic::AtomicU16::new(0);

/// 设置首选精确帧尺寸。必须在 [`crate::drivers::usb::uvc::parse_uvc_video_stream`] 之前调用。
/// `(0, 0)` 关闭精确尺寸偏好。
pub fn set_preferred_frame_size(w: u16, h: u16) {
    PREFERRED_FRAME_W.store(w, core::sync::atomic::Ordering::Relaxed);
    PREFERRED_FRAME_H.store(h, core::sync::atomic::Ordering::Relaxed);
}

/// 上层可设置"首选帧间隔"（100ns 单位，UVC `dwFrameInterval`）。非 0 时
/// [`crate::drivers::usb::uvc::parse_uvc_video_stream`] 对每个 frame 描述符从其可用 interval 集合中选**最接近**
/// 此值的那个（而非默认的最小间隔=最高 fps）。0 表示沿用最小间隔。
///
/// 典型：`333_333` ≈ 30 fps。30fps 常给廉价 webcam 更多曝光/ISP 余量。
pub static PREFERRED_FRAME_INTERVAL: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// 设置 [`PREFERRED_FRAME_INTERVAL`]。必须在 [`crate::drivers::usb::uvc::parse_uvc_video_stream`] 之前调用。
pub fn set_preferred_frame_interval(iv: u32) {
    PREFERRED_FRAME_INTERVAL.store(iv, core::sync::atomic::Ordering::Relaxed);
}
