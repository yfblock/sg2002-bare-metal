//! Synopsys DesignWare USB 2.0 OTG (DWC2) **主机**控制器：寄存器、bring-up、
//! EP0/Isoch 传输调度与共用 DMA 窗。
//!
//! 约定：**通道 0** 专用于 EP0 控制传输；**通道 1** 专用于 Isoch 视频（与部分 IP
//! 在单通道上复用控制+批量时的异常行为隔离）。
//!
//! 子模块：
//! - [`regs`]：`tock-registers` 寄存器/位域/主机通道/PHY 结构。
//! - [`controller`]：上电、软复位、Force Host、FIFO、`HPRT0` 根口操作。
//! - [`ch`]：主机通道原语（启停/等待/NAK-XACT 重试）+ USB ISR + HFNUM 时间。
//! - [`dma`]：内部 DMA 窗（EP0 小缓冲 + UVC 等时大区）。
//! - [`control`]：EP0 控制传输（标准请求 + Hub 端口请求包装）。
//! - [`isoch`]：等时 IN 端点（`IsochInEp`，下一微帧调度，高带宽 mult 支持）。

pub mod regs;
pub mod controller;
pub mod ch;
pub mod control;
pub mod dma;
pub mod isoch;

pub use controller::{dwc2_host_init, dwc2_host_root_bus_reset_pulse};
pub use ch::{handle_usb_irq, take_usb_isr_count, usb_post_set_address_delay};
pub use control::Ep0;
pub use dma::{dma_rx_slice, dma_write_at, DMA_OFF_UVC_BULK, UVC_BULK_DMA_CAP};
pub use isoch::{wmax_mps, wmax_mult, wmax_payload_per_uframe, IsochInEp};
