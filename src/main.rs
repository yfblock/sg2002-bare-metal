#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(static_mut_refs)]

mod control;
mod mailbox;
mod platform;
mod plic;
mod trap;
mod uart;
mod util;
mod panic;
mod logger;
mod jpu;
mod stats;
mod yuv_buf;

use sg200x_bsp::usb::class::uvc;
use sg200x_bsp::usb::host::{self, dwc2};

core::arch::global_asm!(
    ".section .text.boot,\"ax\"",
    ".global _start",
    ".type _start,@function",
"_start:",
    "   csrw mie, zero",
    "   la sp, __stack_top",
    "   la t0, __bss_start",
    "   la t1, __bss_end",
    "1: bgeu t0, t1, 2f",
    "   sd zero, 0(t0)",
    "   addi t0, t0, 8",
    "   j 1b",
    "2: call {main}",
    "3: wfi",
    "   j 3b",
    main = sym rust_main,
);

#[no_mangle]
extern "C" fn rust_main() -> ! {
    unsafe {
        let entry = (trap::_trap_entry as *const ()) as usize;
        core::arch::asm!("csrw mtvec, {0}", in(reg) entry, options(nostack, preserves_flags));
    }
    logger::init();
    uart::print("=== C906L UVC+JPU start ===\n");
    mailbox::write(0, 0, 0);
    stats::init();
    jpu::init_trace();
    platform::platform_init();
    unsafe { trap::init(); }
    uvc::set_preferred_frame_size(640, 480);

    uvc::set_preferred_max_pixels(640 * 480);
    uvc::set_preferred_frame_interval(333_333);
    let extras = host::enumerate_topology_only().expect("enum");
    let cam = extras.uvc.expect("no UVC camera");
    let dev = u32::from(cam.addr);
    let ep0 = cam.ep0_mps;
    let cfg_buf = uvc::read_configuration_descriptor(dev, ep0, 1).expect("read cfg");
    let cfg_total = u16::from_le_bytes([cfg_buf[2], cfg_buf[3]]) as usize;
    let cfg = &cfg_buf[..cfg_total.min(cfg_buf.len())];
    let mut sel = uvc::parse_uvc_video_stream(cfg, cfg_total).expect("parse stream");
    if let Some(ent) = uvc::parse_uvc_control_entities(cfg, cfg_total) {
        // AE priority=0：要求摄像头保持恒定帧率，不许为了曝光降帧率。
        // 默认值 1 会让这台声称 60fps 的摄像头实际只给 16.7fps。
        let tune = uvc::UvcImageTuning {
            // 自动曝光：不限制帧率，让摄像头按光照自由调整曝光时间。
            // 弱光下帧率会从 60fps 降低（AE Priority=1 允许）。
            ae_priority: None,
            ..Default::default()
        };
        let _ = uvc::uvc_init_camera_controls(dev, ep0, &ent, &tune);
    }
    uvc::uvc_start_video_stream(dev, ep0, &mut sel).expect("start stream");
    let _ = uvc::uvc_capture_one_frame(dev, ep0, &sel);

    let mut frame_count: u32 = 0;
    // 自测帧率：小核独占 UART 时（仅启小核、不 bootm 大核）每 100 帧打一次实测 fps。
    // 双核同时跑时 logger 设 Off，这里的 uart::print 仍会输出但量很小（每 ~6s 一行）。
    const UNCOMPRESSED: bool = false;
    const CAPTURE_ONLY: bool = false;
    const FPS_REPORT_EVERY: u32 = 100;
    let mut byte_acc: u64 = 0;
    let (mut tick_cap, mut tick_dec, mut tick_ive) = (0u64, 0u64, 0u64);
    let mut fps_mark_frame: u32 = 0;
    let mut fps_mark_time: u64 = stats::rdtime();
    loop {
        // 大核可经邮箱暂停流水线，让共享缓冲定格在同一帧供比对。
        if control::paused() {
            core::hint::spin_loop();
            continue;
        }
        stats::inc_loop();
        stats::set_time_lo();
        stats::set_stage(stats::stage::CAPTURE);
        let t_cap0 = stats::rdtime();
        match uvc::uvc_capture_one_frame(dev, ep0, &sel) {
            Ok(n) => {
                stats::inc_cap_ok();
                stats::set_stage(stats::stage::GOT_FRAME);
                frame_count = frame_count.wrapping_add(1);
                // 本帧实到字节数（uncompressed 时可能超过组装区，只计数不写入）。
                tick_cap += stats::rdtime().wrapping_sub(t_cap0);
                byte_acc += uvc::take_frame_bytes().max(n as u32) as u64;
                if CAPTURE_ONLY {
                    stats::inc_jpu_ok();
                    stats::set_stage(stats::stage::DONE);
                } else if UNCOMPRESSED {
                    stats::inc_jpu_ok();
                    stats::set_stage(stats::stage::NOTIFY);
                    let flags = mailbox::FLAG_SOI | mailbox::FLAG_EOI
                        | mailbox::encode_dims(640, 480);
                    mailbox::notify(frame_count, n as u32, flags);
                    stats::set_stage(stats::stage::DONE);
                } else {
                // 取本帧 MJPEG 字节（sg200x-bsp 在 DMA_BUF 组装好的 JPEG）。
                let jpeg = match dwc2::dma_rx_slice(uvc::UVC_ASSEMBLED_JPEG_DMA_OFF, n) {
                    Some(s) => s,
                    None => {
                        let flags = mailbox::FLAG_SOI | mailbox::FLAG_EOI
                            | mailbox::encode_dims(640, 480);
                        mailbox::notify(frame_count, n as u32, flags);
                        continue;
                    }
                };
                stats::set_stage(stats::stage::DECODE);
                let t_dec0 = stats::rdtime();
                let decoded = jpu::decode_to_shared(jpeg);
                tick_dec += stats::rdtime().wrapping_sub(t_dec0);
                stats::set_stage(stats::stage::NOTIFY);
                match decoded {
                    Ok((w, h, len)) => {
                        stats::inc_jpu_ok();
                        // IVE 硬件 CSC: YUV422 → RGB888 (640x480)
                        let (y_pa, u_pa, v_pa) = yuv_buf::yuv_planes(yuv_buf::YUV_BUF_PA, w, h);
                        let (r_pa, g_pa, b_pa) = yuv_buf::rgb_planes(yuv_buf::RGB_BUF_PA, w, h);
                        let t_ive0 = stats::rdtime();
                        if let Err(_e) = sg200x_bsp::ive::csc_yuv420_to_rgb888(
                            y_pa, u_pa, v_pa, w, w / 2,
                            r_pa, g_pa, b_pa, w, w, h,
                        ) {
                            stats::inc_jpu_err();
                        }
                        tick_ive += stats::rdtime().wrapping_sub(t_ive0);
                        let reported = len.min(yuv_buf::YUV_BUF_MAX);
                        let flags = mailbox::FLAG_SOI | mailbox::FLAG_EOI
                            | mailbox::FLAG_YUV_READY
                            | mailbox::encode_dims(w, h)
;
                        mailbox::notify(frame_count, reported as u32, flags);
                    }
                    Err(_) => {
                        stats::inc_jpu_err();
                        let flags = mailbox::FLAG_SOI | mailbox::FLAG_EOI
                            | mailbox::encode_dims(640, 480);
                        mailbox::notify(frame_count, n as u32, flags);
                    }
                }
                stats::set_stage(stats::stage::DONE);
                }
                if frame_count.wrapping_sub(fps_mark_frame) >= FPS_REPORT_EVERY {
                    let now = stats::rdtime();
                    let dt = now.wrapping_sub(fps_mark_time);
                    let frames = frame_count.wrapping_sub(fps_mark_frame) as u64;
                    // fps*100，避免浮点
                    let fps_x100 = if dt > 0 {
                        frames * stats::TIMEBASE_HZ * 100 / dt
                    } else {
                        0
                    };
                    uart::print("[FPS] frames=");
                    uart::print_dec(frame_count as u64);
                    uart::print(" fps=");
                    uart::print_dec(fps_x100 / 100);
                    uart::print(".");
                    let frac = fps_x100 % 100;
                    if frac < 10 {
                        uart::print("0");
                    }
                    uart::print_dec(frac);
                    uart::print(" bytes/frame=");
                    uart::print_dec(byte_acc / frames.max(1));
                    // 吞吐 KB/s = byte_acc * TIMEBASE / dt / 1024
                    let kbps = if dt > 0 {
                        byte_acc * stats::TIMEBASE_HZ / dt / 1024
                    } else {
                        0
                    };
                    uart::print(" KB/s=");
                    uart::print_dec(kbps);
                    // 采集统计：uf/loop = 每次传输占掉几个微帧（125us），>1 即跟不上
                    let (loops, data, ufs) = uvc::take_capture_stats();
                    uart::print(" loops/f=");
                    uart::print_dec((loops / frames.max(1) as u32) as u64);
                    uart::print(" data/f=");
                    uart::print_dec((data / frames.max(1) as u32) as u64);
                    uart::print(" uf/loop_x10=");
                    uart::print_dec((ufs as u64 * 10) / (loops.max(1) as u64));
                    // 各阶段每帧耗时（微秒）。rdtime 25MHz -> ticks/25 = us
                    let us = |t: u64| t / 25 / frames.max(1);
                    uart::print(" us{cap=");
                    uart::print_dec(us(tick_cap));
                    uart::print(" dec=");
                    uart::print_dec(us(tick_dec));
                    uart::print(" ive=");
                    uart::print_dec(us(tick_ive));
                    uart::print(" wyuv=");
                    uart::print_dec(us(jpu::take_write_ticks()));
                    uart::print("} jpu{inv1=");
                    uart::print_dec(us(sg200x_bsp::jpu::trace::take_step_ticks(
                        sg200x_bsp::jpu::trace::step::INV_FRAME) as u64));
                    uart::print(" poll=");
                    uart::print_dec(us(sg200x_bsp::jpu::trace::take_step_ticks(
                        sg200x_bsp::jpu::trace::step::POLL) as u64));
                    uart::print(" inv2=");
                    uart::print_dec(us(sg200x_bsp::jpu::trace::take_step_ticks(
                        sg200x_bsp::jpu::trace::step::INV_AFTER) as u64));
                    uart::print(" cpy=");
                    uart::print_dec(us(sg200x_bsp::jpu::trace::take_step_ticks(
                        sg200x_bsp::jpu::trace::step::COPY_STREAM) as u64));
                    uart::print(" cln=");
                    uart::print_dec(us(sg200x_bsp::jpu::trace::take_step_ticks(
                        sg200x_bsp::jpu::trace::step::CLEAN_STREAM) as u64));
                    uart::print("}");
                    tick_cap = 0;
                    tick_dec = 0;
                    tick_ive = 0;
                    uart::print(" usbisr=");
                    uart::print_dec(sg200x_bsp::usb::host::dwc2::ep0::take_usb_isr_count() as u64);
                    uart::print(" jpu_err=");
                    uart::print_dec(jpu::reset_count() as u64);
                    uart::print("\n");
                    byte_acc = 0;
                    fps_mark_frame = frame_count;
                    fps_mark_time = now;
                }
            }
            // 抓帧失败：以前这里是空的 `{}`，既不 notify 也不计数，一旦持续失败
            // 就表现为 frame_count 冻结且串口毫无输出，完全无法定位。至少要计数。
            Err(_) => {
                stats::inc_cap_err();
                stats::set_stage(stats::stage::DONE);
            }
        }
    }
}
