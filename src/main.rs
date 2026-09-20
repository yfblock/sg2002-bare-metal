//! cvirtos — SG2002 C906L 小核 M-mode 裸机固件
//!
//! 流水线:USB 抓 MJPEG → JPU 硬解 YUV → IVE CSC(未生效)→ 邮箱通知大核
//!
//! # 模块结构
//!
//! ```text
//! src/
//! ├── main.rs       入口:初始化 → 主循环(capture → decode+IVE → notify)
//! ├── jpu.rs        JPU 解码封装(使用 drivers/jpu)
//! ├── arch/         架构层(RV64 M-mode C906L)
//! │   └── asm.S     _start 启动块 + _trap_entry 上下文块(mod.rs 里 global_asm! 引入)
//! │   ├── trap      M-mode trap 入口/分发 + mtvec/mie 初始化
//! │   ├── plic      PLIC 中断控制器(M-mode ctx)
//! │   ├── cache     D-cache 维护(DMA 一致性)
//! │   └── time      rdtime/delay(Duration)/elapsed_since(25MHz timebase)
//! ├── ipc.rs        大小核通信协议(DRAM 邮箱 ABI/帧通知/B2S 消息处理/pause-mute)
//! ├── platform/     板级(SG2002):跨核 UART 控制台(打印+Dekker 行锁)、USB 平台初始化
//! │   ├── uart      DW8250 寄存器访问 + 打印辅助 + 跨核行锁
//! │   └── platform  USB PHY/时钟/VBUS/pinmux 初始化
//! ├── yuv_buf.rs    共享 YUV/RGB 缓冲布局
//! ├── logger.rs     log → UART 路由
//! ├── panic.rs      panic handler
//! └── drivers/      从 sg200x-bsp 迁移的硬件驱动
//!     ├── soc       MMIO 基址常量
//!     ├── mailbox   cvi 硬件邮箱控制器(门铃/认领/跨核锁槽)
//!     ├── pinmux    引脚复用(USB VBUS)
//!     ├── gpio      GPIO 输出
//!     ├── usb/      USB 主机栈(DWC2 + UVC 协议)
//!     ├── jpu/      JPU 硬件解码
//!     └── ive/      IVE 硬件 CSC(未跑通,保留)
//! ```

#![no_std]
#![recursion_limit = "512"]
#![no_main]

// ---- 架构层(RV64 M-mode) ----
mod arch;

// ---- 应用层 ----
mod jpu;
mod yuv_buf;
mod logger;
mod panic;

// ---- 大小核通信 ----
mod ipc;

// ---- 小核硬件 ----
mod platform;

// ---- 硬件驱动(从 sg200x-bsp 迁移)----
mod drivers;

use crate::drivers::usb::{dwc2::{self, Ep0}, uvc};
use crate::drivers::usb::enumerate_topology_only;

#[no_mangle]
pub(crate) extern "C" fn rust_main() -> ! {
    // ---- trap 入口 + 基础初始化 ----
    unsafe { arch::trap::init_mtvec() };
    logger::init();
    platform::uart::print("=== C906L UVC+JPU start ===\n");
    ipc::write(0, 0, 0);
    platform::platform_init();
    unsafe { arch::trap::init_interrupts() };

    // ---- UVC 初始化(同步,一次性)----
    uvc::set_preferred_frame_size(640, 480);
    uvc::set_preferred_max_pixels(640 * 480);
    uvc::set_preferred_frame_interval(333_333);

    let cam = enumerate_topology_only().expect("enum");
    let ep0 = Ep0::new(u32::from(cam.addr), cam.ep0_mps);

    let cfg_buf = uvc::read_configuration_descriptor(&ep0, 1).expect("read cfg");
    let cfg_total = u16::from_le_bytes([cfg_buf[2], cfg_buf[3]]) as usize;
    let cfg = &cfg_buf[..cfg_total.min(cfg_buf.len())];
    let mut sel = uvc::parse_uvc_video_stream(cfg, cfg_total).expect("parse stream");

    if let Some(ent) = uvc::parse_uvc_control_entities(cfg, cfg_total) {
        let tune = uvc::UvcImageTuning {
            ae_priority: None,   // 自动曝光,不限制帧率
            ..Default::default()
        };
        let _ = uvc::uvc_init_camera_controls(&ep0, &ent, &tune);
    }
    uvc::uvc_start_video_stream(&ep0, &mut sel).expect("start stream");
    let _ = uvc::uvc_capture_one_frame(&ep0, &sel); // warmup

    // ---- 进入主循环(永不返回)----
    pipeline_loop(&ep0, &sel)
}

/// FPS 报告间隔(帧数)
const FPS_REPORT_EVERY: u32 = 100;

/// 采集/处理 统计(单核,不需要原子)
struct PipelineStats {
    tick_cap: u64,
    tick_dec: u64,
    tick_ive: u64,
    byte_acc: u64,
    fps_mark_frame: u32,
    fps_mark_time: u64,
}

impl PipelineStats {
    fn new() -> Self {
        Self {
            tick_cap: 0,
            tick_dec: 0,
            tick_ive: 0,
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
    let mut frame_count: u32 = 0;
    let mut st = PipelineStats::new();

    loop {
        if ipc::paused() {
            core::hint::spin_loop();
            continue;
        }

        let t_cap0 = crate::arch::time::rdtime();
        match uvc::uvc_capture_one_frame(ep0, sel) {
            Ok(n) => {
                frame_count = frame_count.wrapping_add(1);
                st.tick_cap += crate::arch::time::rdtime().wrapping_sub(t_cap0);
                st.byte_acc += uvc::take_frame_bytes().max(n as u32) as u64;

                // ---- decode + notify ----
                decode_and_notify(n, frame_count, &mut st);

                // ---- FPS 报告 ----
                if frame_count.wrapping_sub(st.fps_mark_frame) >= FPS_REPORT_EVERY {
                    report_fps(frame_count, &mut st);
                }
            }
            Err(_) => {
            }
        }
    }
}

/// JPU 解码 → IVE CSC → 邮箱 notify(单帧处理)。
fn decode_and_notify(jpeg_len: usize, frame_count: u32, st: &mut PipelineStats) {
    let jpeg = match dwc2::dma_rx_slice(uvc::UVC_ASSEMBLED_JPEG_DMA_OFF, jpeg_len) {
        Some(s) => s,
        None => {
            let flags = ipc::FLAG_SOI | ipc::FLAG_EOI
                | ipc::encode_dims(640, 480);
            ipc::notify(frame_count, jpeg_len as u32, flags);
            return;
        }
    };

    let t_dec0 = crate::arch::time::rdtime();
    let decoded = jpu::decode_to_shared(jpeg);
    st.tick_dec += crate::arch::time::rdtime().wrapping_sub(t_dec0);

    match decoded {
        Ok((w, h, len)) => {

            // IVE 硬件 CSC
            let (y_pa, u_pa, v_pa) = yuv_buf::yuv_planes(yuv_buf::YUV_BUF_PA, w, h);
            let (r_pa, g_pa, b_pa) = yuv_buf::rgb_planes(yuv_buf::RGB_BUF_PA, w, h);
            let t_ive0 = crate::arch::time::rdtime();
            if let Err(_e) = crate::drivers::ive::csc_yuv420_to_rgb888(
                y_pa, u_pa, v_pa, w, w / 2,
                r_pa, g_pa, b_pa, w, w, h,
            ) {
            }
            st.tick_ive += crate::arch::time::rdtime().wrapping_sub(t_ive0);

            let reported = len.min(yuv_buf::YUV_BUF_SIZE);
            let flags = ipc::FLAG_SOI | ipc::FLAG_EOI
                | ipc::FLAG_YUV_READY
                | ipc::encode_dims(w, h);
            ipc::notify(frame_count, reported as u32, flags);
        }
        Err(_) => {
            let flags = ipc::FLAG_SOI | ipc::FLAG_EOI
                | ipc::encode_dims(640, 480);
            ipc::notify(frame_count, jpeg_len as u32, flags);
        }
    }
}

/// 每 N 帧打印一次 FPS 统计。
fn report_fps(frame_count: u32, st: &mut PipelineStats) {
    let now = crate::arch::time::rdtime();
    let dt = now.wrapping_sub(st.fps_mark_time);
    let frames = frame_count.wrapping_sub(st.fps_mark_frame) as u64;
    let fps_x100 = if dt > 0 {
        frames * crate::arch::time::TIMEBASE_HZ * 100 / dt
    } else { 0 };

    platform::uart::print("[FPS] frames=");
    platform::uart::print_dec(frame_count as u64);
    platform::uart::print(" fps=");
    platform::uart::print_dec(fps_x100 / 100);
    platform::uart::print(".");
    let frac = fps_x100 % 100;
    if frac < 10 { platform::uart::print("0"); }
    platform::uart::print_dec(frac);
    platform::uart::print(" bytes/frame=");
    platform::uart::print_dec(st.byte_acc / frames.max(1));
    let kbps = if dt > 0 { st.byte_acc * crate::arch::time::TIMEBASE_HZ / dt / 1024 } else { 0 };
    platform::uart::print(" KB/s=");
    platform::uart::print_dec(kbps);
    let us = |t: u64| t * 1_000_000 / crate::arch::time::TIMEBASE_HZ / frames.max(1);
    platform::uart::print(" us{cap=");
    platform::uart::print_dec(us(st.tick_cap));
    platform::uart::print(" dec=");
    platform::uart::print_dec(us(st.tick_dec));
    platform::uart::print(" ive=");
    platform::uart::print_dec(us(st.tick_ive));
    // 心跳观测字(MMIO 直读):main 视角验证 mtimer 是否真的在走
    platform::uart::print(" hb=");
    platform::uart::print_dec(unsafe { core::ptr::read_volatile(0x0190_041C as *const u32) } as u64);
    platform::uart::print("} jpu{inv1=");
    platform::uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::INV_FRAME) as u64));
    platform::uart::print(" poll=");
    platform::uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::POLL) as u64));
    platform::uart::print(" inv2=");
    platform::uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::INV_AFTER) as u64));
    platform::uart::print(" cpy=");
    platform::uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::COPY_STREAM) as u64));
    platform::uart::print(" cln=");
    platform::uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::CLEAN_STREAM) as u64));
    platform::uart::print("}");
    platform::uart::print(" usbisr=");
    platform::uart::print_dec(crate::drivers::usb::dwc2::take_usb_isr_count() as u64);
    platform::uart::print(" jpu_err=");
    platform::uart::print_dec(jpu::reset_count() as u64);
    platform::uart::print("\n");

    st.tick_cap = 0;
    st.tick_dec = 0;
    st.tick_ive = 0;
    st.byte_acc = 0;
    st.fps_mark_frame = frame_count;
    st.fps_mark_time = now;
}
