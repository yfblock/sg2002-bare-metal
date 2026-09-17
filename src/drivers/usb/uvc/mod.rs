//! USB Video Class（UVC）：配置描述符解析、`PROBE`/`COMMIT`、**Isoch IN** 抓一帧。
//!
//! 修复要点：视频端点在无数据时大量 **NAK**，须在主机侧重试
//! （见 [`crate::drivers::usb::dwc2::IsochInEp::read_uframe`]）。
//!
//! 子模块：
//! - [`setup`]：UVC 类专用 SETUP 包构造（VS PROBE/COMMIT、VC 实体控制）。
//! - [`prefs`]：选流/抓帧的原子偏好开关与 setter。
//! - [`descriptor`]：配置描述符读取与解析（VS 流/格式/端点、VC 实体）。
//! - [`control`]：摄像头 VC/PU 控制（白平衡、曝光等）。
//! - [`stream`]：`PROBE`/`COMMIT` 协商与流启停。
//! - [`capture`]：Isoch IN 抓帧与 MJPEG/Uncompressed 帧组装。

pub mod setup;
pub mod prefs;
pub mod descriptor;
pub mod control;
pub mod stream;
pub mod capture;

// 重导出保持完整 `usb::uvc::X` 公开 API。bin crate 里未被本 crate
// 使用会报 unused_imports（如 uvc_session 会话层专用项），与 dwc2/mod.rs 的
// 重导出同构，这里统一豁免。
#[allow(unused_imports)]
pub use capture::{
    reset_frame_continuity, take_frame_bytes, uvc_capture_one_frame, LAST_EOF_FID,
    UVC_ASSEMBLED_JPEG_DMA_OFF, UVC_WORK_AREA_BYTES,
};
pub use control::{uvc_init_camera_controls, UvcImageTuning};
#[allow(unused_imports)]
pub use descriptor::{
    parse_uvc_control_entities, parse_uvc_video_stream, read_configuration_descriptor,
    UvcControlEntities, UvcStreamSelection,
};
#[allow(unused_imports)]
pub use prefs::{
    set_preferred_frame_interval, set_preferred_frame_size, set_preferred_max_pixels,
    FRAME_DEBUG, PREFERRED_FRAME_H, PREFERRED_FRAME_INTERVAL, PREFERRED_FRAME_W,
    PREFERRED_MAX_PIXELS,
};
#[allow(unused_imports)]
pub use stream::uvc_start_video_stream;
