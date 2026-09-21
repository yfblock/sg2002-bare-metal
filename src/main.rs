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

mod arch;
mod drivers;
mod ipc;
mod logger;
mod panic;
mod platform;
mod yuv_buf;

use core::time::Duration;

use crate::drivers::usb::hub::RootHub;
use crate::drivers::usb::{dwc2, uvc};

const FPS_REPORT_EVERY: u32 = 100;

#[no_mangle]
pub(crate) extern "C" fn rust_main() -> ! {
    // trap 入口 + 基础初始化
    arch::trap::init_mtvec();
    logger::init();
    logger::print("=== C906L UVC+JPU start ===\n");
    ipc::write(0, 0, 0);
    platform::platform_init();
    arch::trap::init_interrupts();

    // UVC 初始化(同步,一次性):主机 bring-up → 树遍历(驱动各自 probe
    // 并存好自己的设备)→ main 取用。
    dwc2::dwc2_host_init().expect("host init");
    uvc::set_prefs(uvc::UvcPrefs {
        frame_w: 640,
        frame_h: 480,
        // 33.3333ms ≈ 30fps:给廉价 webcam 更多曝光/ISP 余量。
        frame_interval: Duration::from_nanos(33_333_300),
    });
    RootHub.enumerate_bus().expect("enum");
    let camera = uvc::take_camera().expect("open camera");

    // 进入主循环(永不返回)
    pipeline_loop(&camera)
}

/// 主循环:capture → JPU decode → IVE CSC → mailbox notify。
///
/// `control_ep`/`sel` 由 rust_main 的初始化阶段产生。
fn pipeline_loop(camera: &uvc::UvcCamera) -> ! {
    let mut frames: u32 = 0;
    let mut fps_mark_frame = 0u32;
    let mut fps_mark_time = crate::arch::time::rdtime();

    loop {
        if ipc::paused() {
            core::hint::spin_loop();
            continue;
        }

        match camera.capture_frame() {
            Ok(n) => {
                frames = frames.wrapping_add(1);

                // decode + notify
                decode_and_notify(n, frames);

                // FPS 报告
                if frames.wrapping_sub(fps_mark_frame) >= FPS_REPORT_EVERY {
                    report_fps(frames, &mut fps_mark_frame, &mut fps_mark_time);
                }
            }
            Err(_) => {}
        }
    }
}

/// YUV 缺失时的兜底通知:只上行 MJPEG 本身(dims 按协商几何 640×480 编码)。
fn notify_mjpeg_only(frame_count: u32, jpeg_len: usize) {
    let flags = ipc::FLAG_SOI | ipc::FLAG_EOI | ipc::encode_dims(640, 480);
    ipc::notify(frame_count, jpeg_len as u32, flags);
}

/// JPU 解码 → IVE CSC → 邮箱 notify(单帧处理)。
///
/// 取不到 DMA 帧数据与解码失败共用同一条兜底路径(只通知 MJPEG)。
fn decode_and_notify(jpeg_len: usize, frames: u32) {
    let jpeg = match dwc2::dma_rx_slice(uvc::UVC_ASSEMBLED_JPEG_DMA_OFF, jpeg_len) {
        Some(s) => s,
        None => return notify_mjpeg_only(frames, jpeg_len),
    };

    let decoded = crate::drivers::jpu::decode_to_shared(jpeg);
    let (w, h, len) = match decoded {
        Ok(r) => r,
        Err(_) => return notify_mjpeg_only(frames, jpeg_len),
    };

    // IVE 硬件 CSC
    let yuv = yuv_buf::yuv_planes(platform::YUV_BUF_PA, w, h);
    let rgb = yuv_buf::rgb_planes(platform::RGB_BUF_PA, w, h);
    if let Err(_e) = crate::drivers::ive::csc(&yuv, &rgb, w, h) {}

    let reported = len.min(platform::YUV_BUF_SIZE);
    let flags = ipc::FLAG_SOI | ipc::FLAG_EOI | ipc::FLAG_YUV_READY | ipc::encode_dims(w, h);
    ipc::notify(frames, reported as u32, flags);
}

/// 每 N 帧打印一次帧数/FPS(usbisr/jpu_err 计数器在此 drain)。
fn report_fps(frames: u32, fps_mark_frame: &mut u32, fps_mark_time: &mut u64) {
    let now = crate::arch::time::rdtime();
    let dt = now.wrapping_sub(*fps_mark_time);
    let n = frames.wrapping_sub(*fps_mark_frame) as u64;
    let fps_x100 = if dt > 0 {
        n * crate::arch::time::TIMEBASE_HZ * 100 / dt
    } else {
        0
    };
    logger::print_fmt(format_args!(
        "[FPS] frames={} fps={}.{:02} jpu_err={} pc={:#010x}\n",
        frames,
        fps_x100 / 100,
        fps_x100 % 100,
        crate::drivers::jpu::take_reset_count(),
        crate::arch::time::last_sampled_pc(),
    ));

    *fps_mark_frame = frames;
    *fps_mark_time = now;
}
