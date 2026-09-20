//! cvirtos — SG2002 C906L 小核 M-mode 裸机固件
//!
//! 流水线:USB 抓 MJPEG → JPU 硬解 YUV → IVE CSC(未生效)→ 邮箱通知大核
//!
//! # 模块结构
//!
//! ```text
//! src/
//! ├── main.rs       入口:初始化 → 主循环(capture → decode+IVE → notify)
//! ├── arch/         架构层(RV64 M-mode C906L)
//! │   └── asm.S     _start 启动块 + _trap_entry 上下文块(mod.rs 里 global_asm! 引入)
//! │   ├── trap      M-mode trap 入口/分发 + mtvec/mie 初始化
//! │   ├── plic      PLIC 中断控制器(M-mode ctx)
//! │   ├── cache     D-cache 维护(DMA 一致性)
//! │   └── time      rdtime/delay(Duration)/elapsed_since(25MHz timebase)
//! ├── ipc.rs        大小核通信协议(DRAM 邮箱 ABI/帧通知/B2S 消息处理/pause-mute)
//! ├── logger.rs     跨核 UART 控制台(DW8250 + 打印 + Dekker 行锁)+ log 门面
//! ├── panic.rs      panic handler
//! ├── platform.rs   板级(SG2002):SoC MMIO 地址表 + rtos 区布局 + USB 平台初始化
//! ├── yuv_buf.rs    共享 YUV/RGB 平面几何(地址见 platform)
//! └── drivers/      从 sg200x-bsp 迁移的硬件驱动
//!     ├── mailbox   cvi 硬件邮箱控制器(门铃/认领/跨核锁槽)
//!     ├── wdt        DW APB 看门狗(wedge 自愈复位)
//!     ├── pinmux    引脚复用(USB VBUS)
//!     ├── gpio      GPIO 输出
//!     ├── usb/      USB 主机栈(DWC2 + UVC 协议)
//!     ├── jpu/      JPU 硬件解码
//!     └── ive/      IVE 硬件 CSC(未跑通,保留)
//! ```

#![feature(sync_unsafe_cell)] // core::cell::SyncUnsafeCell(nightly 钉死于 rust-toolchain.toml)
#![no_std]
#![recursion_limit = "512"]
#![no_main]

// 架构层(RV64 M-mode)
mod arch;

// 应用层
mod yuv_buf;
mod logger;
mod panic;

// 大小核通信
mod ipc;

// 小核硬件
mod platform;

// 硬件驱动(从 sg200x-bsp 迁移)
mod drivers;

use crate::drivers::usb::{dwc2::{self, Ep0}, uvc};
use crate::drivers::usb::enumerate_camera;

#[no_mangle]
pub(crate) extern "C" fn rust_main() -> ! {
    // trap 入口 + 基础初始化
    unsafe { arch::trap::init_mtvec() };
    logger::init();
    logger::print("=== C906L UVC+JPU start ===\n");
    ipc::write(0, 0, 0);
    platform::platform_init();
    unsafe { arch::trap::init_interrupts() };

    // UVC 初始化(同步,一次性)
    let cam = enumerate_camera().expect("enum");
    let ep0 = Ep0::new(u32::from(cam.addr), cam.ep0_mps);

    let cfg_buf = uvc::read_configuration_descriptor(&ep0, 1).expect("read cfg");
    let cfg_total = u16::from_le_bytes([cfg_buf[2], cfg_buf[3]]) as usize;
    let cfg = &cfg_buf[..cfg_total.min(cfg_buf.len())];
    let prefs = uvc::UvcPrefs {
        frame_w: 640,
        frame_h: 480,
        frame_interval: 333_333, // ≈30fps:给廉价 webcam 更多曝光/ISP 余量
    };
    let mut sel = uvc::parse_uvc_video_stream(cfg, cfg_total, &prefs).expect("parse stream");

    if let Some(ent) = uvc::parse_uvc_control_entities(cfg, cfg_total) {
        let _ = uvc::uvc_init_camera_controls(&ep0, &ent);
    }
    uvc::uvc_start_video_stream(&ep0, &mut sel).expect("start stream");
    let _ = uvc::uvc_capture_one_frame(&ep0, &sel); // warmup

    // 进入主循环(永不返回)
    pipeline_loop(&ep0, &sel)
}

/// FPS 报告间隔(帧数)
const FPS_REPORT_EVERY: u32 = 100;

/// 采集/处理 统计(单核,不需要原子)。各阶段耗时为 Duration 累计。
struct PipelineStats {
    frames: u32,
    cap: core::time::Duration,
    dec: core::time::Duration,
    ive: core::time::Duration,
    byte_acc: u64,
    fps_mark_frame: u32,
    fps_mark_time: u64,
}

impl PipelineStats {
    fn new() -> Self {
        Self {
            frames: 0,
            cap: core::time::Duration::ZERO,
            dec: core::time::Duration::ZERO,
            ive: core::time::Duration::ZERO,
            byte_acc: 0,
            fps_mark_frame: 0,
            fps_mark_time: crate::arch::time::rdtime(),
        }
    }
}

/// 主循环:capture → JPU decode → IVE CSC → mailbox notify。
///
/// `ep0`/`sel` 由 rust_main 的初始化阶段产生。
fn pipeline_loop(ep0: &Ep0, sel: &uvc::UvcStreamSelection) -> ! {
    let mut st = PipelineStats::new();

    loop {
        if ipc::paused() {
            core::hint::spin_loop();
            continue;
        }

        let t_cap0 = crate::arch::time::rdtime();
        match uvc::uvc_capture_one_frame(ep0, sel) {
            Ok(n) => {
                st.frames = st.frames.wrapping_add(1);
                st.cap += crate::arch::time::elapsed_since(t_cap0);
                st.byte_acc += n as u64;

                // decode + notify
                decode_and_notify(n, &mut st);

                // FPS 报告
                if st.frames.wrapping_sub(st.fps_mark_frame) >= FPS_REPORT_EVERY {
                    report_fps(&mut st);
                }
            }
            Err(_) => {
            }
        }
    }
}

/// YUV 缺失时的兜底通知:只上行 MJPEG 本身(dims 按协商几何 640×480 编码)。
fn notify_mjpeg_only(frame_count: u32, jpeg_len: usize) {
    let flags = ipc::FLAG_SOI | ipc::FLAG_EOI
        | ipc::encode_dims(640, 480);
    ipc::notify(frame_count, jpeg_len as u32, flags);
}

/// JPU 解码 → IVE CSC → 邮箱 notify(单帧处理)。
///
/// 取不到 DMA 帧数据与解码失败共用同一条兜底路径(只通知 MJPEG)。
fn decode_and_notify(jpeg_len: usize, st: &mut PipelineStats) {
    let jpeg = match dwc2::dma_rx_slice(uvc::UVC_ASSEMBLED_JPEG_DMA_OFF, jpeg_len) {
        Some(s) => s,
        None => return notify_mjpeg_only(st.frames, jpeg_len),
    };

    let t_dec0 = crate::arch::time::rdtime();
    let decoded = crate::drivers::jpu::decode_to_shared(jpeg);
    st.dec += crate::arch::time::elapsed_since(t_dec0);

    let (w, h, len) = match decoded {
        Ok(r) => r,
        Err(_) => return notify_mjpeg_only(st.frames, jpeg_len),
    };

    // IVE 硬件 CSC
    let yuv = yuv_buf::yuv_planes(platform::YUV_BUF_PA, w, h);
    let rgb = yuv_buf::rgb_planes(platform::RGB_BUF_PA, w, h);
    let t_ive0 = crate::arch::time::rdtime();
    if let Err(_e) = crate::drivers::ive::csc(&yuv, &rgb, w, h) {}
    st.ive += crate::arch::time::elapsed_since(t_ive0);

    let reported = len.min(platform::YUV_BUF_SIZE);
    let flags = ipc::FLAG_SOI | ipc::FLAG_EOI
        | ipc::FLAG_YUV_READY
        | ipc::encode_dims(w, h);
    ipc::notify(st.frames, reported as u32, flags);
}

/// 每 N 帧打印一次 FPS 统计。
fn report_fps(st: &mut PipelineStats) {
    let now = crate::arch::time::rdtime();
    let dt = now.wrapping_sub(st.fps_mark_time);
    let frames = st.frames.wrapping_sub(st.fps_mark_frame) as u64;
    let fps_x100 = if dt > 0 {
        frames * crate::arch::time::TIMEBASE_HZ * 100 / dt
    } else { 0 };

    let kbps = if dt > 0 { st.byte_acc * crate::arch::time::TIMEBASE_HZ / dt / 1024 } else { 0 };
    // Duration → 每帧均值(µs);被统计的 Duration 在各实参处显式可见
    let per_frame = |d: core::time::Duration| (d / (frames.max(1) as u32)).as_micros() as u64;
    logger::print_fmt(format_args!(
        "[FPS] frames={} fps={}.{:02} bytes/frame={} KB/s={} \
         us{{cap={} dec={} ive={} hb={}}} usbisr={} jpu_err={}\n",
        st.frames,
        fps_x100 / 100,
        fps_x100 % 100,
        st.byte_acc / frames.max(1),
        kbps,
        per_frame(st.cap),
        per_frame(st.dec),
        per_frame(st.ive),
        // 心跳观测字(MMIO 直读):main 视角验证 mtimer 是否真的在走
        unsafe { core::ptr::read_volatile(0x0190_041C as *const u32) },
        crate::drivers::usb::dwc2::take_usb_isr_count(),
        crate::drivers::jpu::take_reset_count(),
    ));

    st.cap = core::time::Duration::ZERO;
    st.dec = core::time::Duration::ZERO;
    st.ive = core::time::Duration::ZERO;
    st.byte_acc = 0;
    st.fps_mark_frame = st.frames;
    st.fps_mark_time = now;
}
