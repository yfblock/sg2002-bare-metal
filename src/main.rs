#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(static_mut_refs)]

mod mailbox;
mod platform;
mod plic;
mod trap;
mod uart;
mod util;
mod panic;
mod logger;
mod jpu;
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
    platform::platform_init();
    // 尽早启用 M-mode 外部中断 + PLIC source 61（MBOX_INT_C906_2ND）：
    // UVC 枚举要几十秒，放在它后面会把开机初期大核发来的消息全丢掉。
    // 小核收 61、大核收 101，两条线互不干扰（PLIC 每个 source 独立 pending）。
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
        let _ = uvc::uvc_init_camera_controls(dev, ep0, &ent, &uvc::UvcImageTuning::default());
    }
    uvc::uvc_start_video_stream(dev, ep0, &mut sel).expect("start stream");
    let _ = uvc::uvc_capture_one_frame(dev, ep0, &sel);
    let mut frame_count: u32 = 0;
    loop {
        match uvc::uvc_capture_one_frame(dev, ep0, &sel) {
            Ok(n) => {
                frame_count = frame_count.wrapping_add(1);
                // 取本帧 MJPEG 字节（sg200x-bsp 在 DMA_BUF 组装好的 JPEG）。
                let jpeg = match dwc2::dma_rx_slice(uvc::UVC_ASSEMBLED_JPEG_DMA_OFF, n) {
                    Some(s) => s,
                    None => {
                        // 取不到 DMA 视图：只通知 MJPEG 大小，不置 YUV_READY。
                        let flags = mailbox::FLAG_SOI | mailbox::FLAG_EOI
                            | mailbox::encode_dims(640, 480);
                        mailbox::notify(frame_count, n as u32, flags);
                        continue;
                    }
                };
                match jpu::decode_to_shared(jpeg) {
                    Ok((w, h, len)) => {
                        // 报告的 yuv_size 受共享缓冲容量裁剪（write_yuv 同样裁剪）。
                        let reported = len.min(yuv_buf::YUV_BUF_MAX);
                        let flags = mailbox::FLAG_SOI | mailbox::FLAG_EOI
                            | mailbox::FLAG_YUV_READY
                            | mailbox::encode_dims(w, h);
                        mailbox::notify(frame_count, reported as u32, flags);
                    }
                    Err(_) => {
                        // 解码失败（JPU 已在 decode_to_shared 内复位重建）。
                        // 仅通知 MJPEG 大小，不置 YUV_READY；下一帧重试。
                        let flags = mailbox::FLAG_SOI | mailbox::FLAG_EOI
                            | mailbox::encode_dims(640, 480);
                        mailbox::notify(frame_count, n as u32, flags);
                    }
                }
            }
            Err(_) => {}
        }
    }
}
