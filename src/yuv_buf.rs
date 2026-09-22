//! 大小核共享 YUV/RGB 帧缓冲。
//!
//! JPU 解码 YUV420 → IVE 硬件转 RGB888 → 大核通过 mmap 读 RGB。
//!
//! 布局常量在 `crate::platform` 的「预留 rtos 区共享布局」段(单一事实来源)。

/// YUV 三平面 + 步长。4:2:2 planar:色度平面 `(w/2) × h`,stride_c = w/2。
pub struct YuvPlanes {
    pub y: usize,
    pub u: usize,
    pub v: usize,
    pub stride_y: usize,
    pub stride_c: usize,
}

/// RGB888 planar 三平面(共享步长 w)。
pub struct RgbPlanes {
    pub r: usize,
    pub g: usize,
    pub b: usize,
    pub stride: usize,
}

/// 取 YUV 的 Y/U/V 平面。
///
/// **按 YUV422 planar 计算**：摄像头 MJPEG 是 4:2:2 采样（JPU 报 `fmt=1`，
/// `frame_size = w*h*2`），色度平面是 `(w/2) × h`，**不是** YUV420 的
/// `(w/2) × (h/2)`。之前按 420 算，V 平面起始地址少了 `(w/2)*(h/2)` 字节，
/// 会让 IVE 把 U 平面的下半段当成 V 读。
#[inline]
#[allow(dead_code)]
pub fn yuv_planes(pa: usize, w: u32, h: u32) -> YuvPlanes {
    YuvPlanes {
        y: pa,
        u: pa + (w * h) as usize,
        v: pa + (w * h) as usize + ((w / 2) * h) as usize,
        stride_y: w as usize,
        stride_c: (w / 2) as usize,
    }
}

/// 取 RGB 的 R/G/B 平面地址。
#[inline]
#[allow(dead_code)]
pub fn rgb_planes(pa: usize, w: u32, h: u32) -> RgbPlanes {
    RgbPlanes {
        r: pa,
        g: pa + (w * h) as usize,
        b: pa + 2 * (w * h) as usize,
        stride: w as usize,
    }
}
