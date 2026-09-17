//! 小核任务:采集/解码/通知 主循环(从 main.rs 提取)。
//!
//! 结构:单核协作式,capture → decode+IVE → notify 串行。
//! 由 `rust_main` 初始化后调用 `pipeline_loop` 进入无限循环。

use crate::platform::uart;
use crate::ipc;
use crate::jpu;
use crate::yuv_buf;

use crate::drivers::usb::uvc;
use crate::drivers::usb::dwc2;

/// FPS 报告间隔(帧数)
const FPS_REPORT_EVERY: u32 = 100;

/// 采集/处理 统计(单核,不需要原子)
pub struct PipelineStats {
    pub tick_cap: u64,
    pub tick_dec: u64,
    pub tick_ive: u64,
    pub byte_acc: u64,
    pub fps_mark_frame: u32,
    pub fps_mark_time: u64,
}

impl PipelineStats {
    pub fn new() -> Self {
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
/// `ep0`/`sel` 由 main.rs 的初始化阶段产生。
pub fn pipeline_loop(
    ep0: &crate::drivers::usb::dwc2::Ep0,
    sel: &uvc::UvcStreamSelection,
) -> ! {
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

            let reported = len.min(yuv_buf::YUV_BUF_MAX);
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

    uart::print("[FPS] frames=");
    uart::print_dec(frame_count as u64);
    uart::print(" fps=");
    uart::print_dec(fps_x100 / 100);
    uart::print(".");
    let frac = fps_x100 % 100;
    if frac < 10 { uart::print("0"); }
    uart::print_dec(frac);
    uart::print(" bytes/frame=");
    uart::print_dec(st.byte_acc / frames.max(1));
    let kbps = if dt > 0 { st.byte_acc * crate::arch::time::TIMEBASE_HZ / dt / 1024 } else { 0 };
    uart::print(" KB/s=");
    uart::print_dec(kbps);
    let us = |t: u64| t / 25 / frames.max(1);
    uart::print(" us{cap=");
    uart::print_dec(us(st.tick_cap));
    uart::print(" dec=");
    uart::print_dec(us(st.tick_dec));
    uart::print(" ive=");
    uart::print_dec(us(st.tick_ive));
    uart::print(" wyuv=");
    uart::print_dec(us(jpu::take_write_ticks()));
    uart::print("} jpu{inv1=");
    uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::INV_FRAME) as u64));
    uart::print(" poll=");
    uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::POLL) as u64));
    uart::print(" inv2=");
    uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::INV_AFTER) as u64));
    uart::print(" cpy=");
    uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::COPY_STREAM) as u64));
    uart::print(" cln=");
    uart::print_dec(us(crate::drivers::jpu::trace::take_step_ticks(
        crate::drivers::jpu::trace::step::CLEAN_STREAM) as u64));
    uart::print("}");
    uart::print(" usbisr=");
    uart::print_dec(crate::drivers::usb::dwc2::take_usb_isr_count() as u64);
    uart::print(" jpu_err=");
    uart::print_dec(jpu::reset_count() as u64);
    uart::print("\n");

    st.tick_cap = 0;
    st.tick_dec = 0;
    st.tick_ive = 0;
    st.byte_acc = 0;
    st.fps_mark_frame = frame_count;
    st.fps_mark_time = now;
}
