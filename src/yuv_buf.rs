//! 大小核共享 YUV/RGB 帧缓冲。
//!
//! JPU 解码 YUV420 → IVE 硬件转 RGB888 → 大核通过 mmap 读 RGB。
//!
//! 内存布局（rtos_region 2MB 内）：
//! ```text
//!   小核镜像    0x8FE00000  ~552KB
//!   YUV 单缓冲  0x8FE88000  460800 B (640x480 YUV420)
//!   JPU pool    0x8FEF8800  256KB (stream_buf only)
//!   RGB 输出    0x8FF38800  921600 B (640x480 RGB888 planar)
//!   spare       0x8FFE0800  ~129KB → mailbox 0x90040000 / stats 0x90040040
//! ```

/// YUV420 帧缓冲物理地址（JPU DMA 直写）。
pub const YUV_BUF_PA: usize = 0x8FE8_8000;
/// YUV 帧缓冲容量。摄像头实际发 YUV422(fmt=1)，JPU frame_size=614400。
pub const YUV_BUF_SIZE: usize = 614400;

/// RGB888 planar 输出缓冲物理地址（IVE CSC 输出）。
/// R/G/B 三个平面各 640×480 = 307200，共 921600。
pub const RGB_BUF_PA: usize = 0x8FF5_E000;

/// 取 YUV 的 Y/U/V 平面地址。
///
/// **按 YUV422 planar 计算**：摄像头 MJPEG 是 4:2:2 采样（JPU 报 `fmt=1`，
/// `frame_size = w*h*2`），色度平面是 `(w/2) × h`，**不是** YUV420 的
/// `(w/2) × (h/2)`。之前按 420 算，V 平面起始地址少了 `(w/2)*(h/2)` 字节，
/// 会让 IVE 把 U 平面的下半段当成 V 读。
#[inline]
pub fn yuv_planes(pa: usize, w: u32, h: u32) -> (usize, usize, usize) {
    let y = pa;
    let u = pa + (w * h) as usize;
    let v = u + ((w / 2) * h) as usize;
    (y, u, v)
}

/// 取 RGB 的 R/G/B 平面地址。
#[inline]
pub fn rgb_planes(pa: usize, w: u32, h: u32) -> (usize, usize, usize) {
    let plane = (w * h) as usize;
    (pa, pa + plane, pa + 2 * plane)
}
