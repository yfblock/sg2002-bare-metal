//! 内部 DMA 窗：前 1KiB 为 EP0/控制小缓冲，其后是 UVC 等时大区
//! （单微帧 RX 工作区 + 组装 MJPEG/像素帧）。须物理连续。

use crate::drivers::usb::error::{UsbError, UsbResult};

/// EP0 SETUP 包 + 小数据缓冲区的区域大小（DMA 窗口头部的 1KiB）。
pub(crate) const EP0_REGION_BYTES: usize = 1024;

/// UVC 等时传输使用的 DMA 区起始偏移（紧跟在 1KiB EP0 工作区之后）。
pub const DMA_OFF_UVC_BULK: usize = EP0_REGION_BYTES;
/// UVC 视频缓冲容量；前 `UVC_WORK_AREA_BYTES` 用作单微帧 RX 工作区，其余拼接 JPEG。
/// 720p MJPEG 单帧典型 100-300KB，需要 ≥320KB 的 JPEG 区。
pub const UVC_BULK_DMA_CAP: usize = 384 * 1024;

/// 整个 `DmaBuf` 大小（供边界检查）。
const DMA_BUF_TOTAL: usize = EP0_REGION_BYTES + UVC_BULK_DMA_CAP;
/// EP0 SETUP 包固定落在窗口头部。
pub(crate) const OFF_EP0: usize = 0;
/// EP0 小缓冲读（Hub 描述符、配置前缀、`GET_PORT_STATUS`），与 UVC 等时大区错开。
pub(crate) const DMA_OFF_SMALL_IO: usize = 256;

#[repr(C, align(256))]
struct DmaBuf {
    bytes: [u8; EP0_REGION_BYTES],
    uvc_bulk: [u8; 384 * 1024],
}

static mut DMA_BUF: DmaBuf = DmaBuf {
    bytes: [0; EP0_REGION_BYTES],
    uvc_bulk: [0; 384 * 1024],
};

/// DMA 工作区基址（`static mut` 仅经裸指针访问，避免 `static_mut_refs`）。
#[inline]
pub(crate) fn dma_ptr() -> *mut u8 {
    core::ptr::addr_of_mut!(DMA_BUF).cast::<u8>()
}

/// 安全的只读视图，供 UVC 等解析刚完成的 DMA 数据（**仅**在传输/cache invalidate 之后调用）。
///
/// # 参数
/// - `off`：相对内部 DMA 窗口起始的字节偏移。
/// - `len`：要暴露的连续字节长度。
#[inline]
pub fn dma_rx_slice(off: usize, len: usize) -> Option<&'static [u8]> {
    if len == 0 || off.checked_add(len)? > DMA_BUF_TOTAL {
        return None;
    }
    Some(unsafe { core::slice::from_raw_parts(dma_ptr().add(off), len) })
}

/// 将数据写入内部 DMA 窗口（CPU 写，供 UVC 组装 JPEG 等）。
///
/// # 参数
/// - `off`：相对 DMA 窗口起始的偏移。
/// - `src`：要拷贝进去的源数据。
pub fn dma_write_at(off: usize, src: &[u8]) -> UsbResult<()> {
    let end = off
        .checked_add(src.len())
        .ok_or(UsbError::Protocol("dma write overflow"))?;
    if end > DMA_BUF_TOTAL {
        return Err(UsbError::Protocol("dma write out of buf"));
    }
    unsafe {
        core::ptr::copy_nonoverlapping(src.as_ptr(), dma_ptr().add(off), src.len());
    }
    Ok(())
}
