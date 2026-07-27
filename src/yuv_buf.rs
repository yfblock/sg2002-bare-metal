//! 大小核共享 YUV 帧缓冲（单缓冲）。
//!
//! 小核抓 MJPEG → JPU 解码 → YUV420 写到 0x8FE90000（rtos_region 内，512KB）。
//! 大核通过 /dev/cvi-yuv 读 YUV 数据（带 seqlock 一致性重检防撕裂）。
//! 邮箱 0x8FFFE000 放帧元信息（含 YUV 大小/宽高）。
//!
//! 单缓冲：rtos_region 仅 2MB，容纳不下小核镜像+双缓冲 YUV+JPU pool。大核 read_at
//! 读邮箱 frame_count 拷 YUV 后重检——若推进≥2 说明被覆盖则重试。

use core::ptr::copy_nonoverlapping;

/// YUV 帧缓冲物理地址（rtos_region 内，小核镜像 stack 之上）。
pub const YUV_BUF_PA: usize = 0x8FE8_8000;
/// YUV 缓冲最大大小。本相机 MJPEG 为 YUV422(614400)，但大核侧 yuv-fps 工具按
/// YUV420(460800) 读取；压力测试关注通信链路（FPS/帧计数），故裁到 460800 兼容工具。
/// （完整 YUV422 传输可后续放大缓冲或大核侧改读 yuv_size。）
pub const YUV_BUF_MAX: usize = 460800;

/// 把 YUV 数据拷到共享缓冲区（小核 identity 映射，直接写 PA）。
pub fn write_yuv(src: &[u8]) {
    let n = src.len().min(YUV_BUF_MAX);
    unsafe {
        let dst = YUV_BUF_PA as *mut u8;
        copy_nonoverlapping(src.as_ptr(), dst, n);
    }
}
