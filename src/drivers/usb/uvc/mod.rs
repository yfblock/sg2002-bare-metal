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

pub mod setup;
pub mod descriptor;
pub mod control;
pub mod session;
pub mod stream;
pub mod capture;

// 重导出仅保留跨模块消费项:会话门面 + DMA 偏移(main 取 JPEG 切片用) + 选流偏好。
pub use capture::UVC_ASSEMBLED_JPEG_DMA_OFF;
pub use descriptor::UvcPrefs;
pub use session::{open, UvcCamera};
