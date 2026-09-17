//! cvirtos — SG2002 C906L 小核 M-mode 裸机固件
//!
//! 流水线:USB 抓 MJPEG → JPU 硬解 YUV → IVE CSC(未生效)→ 邮箱通知大核
//!
//! # 模块结构
//!
//! ```text
//! src/
//! ├── main.rs       入口:初始化 → task::pipeline_loop
//! ├── task.rs       主循环(capture → decode+IVE → notify)
//! ├── jpu.rs        JPU 解码封装(使用 drivers/jpu)
//! ├── arch/         架构层(RV64 M-mode C906L)
//! │   └── asm.S     _start 启动块 + _trap_entry 上下文块(mod.rs 里 global_asm! 引入)
//! │   ├── trap      M-mode trap 入口/分发 + mtvec/mie 初始化
//! │   ├── plic      PLIC 中断控制器(M-mode ctx)
//! │   ├── cache     D-cache 维护(DMA 一致性)
//! │   ├── time      rdtime/delay(Duration)/elapsed_since(25MHz timebase)
//! │   └── sync      fence/wfi 原语
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
#![allow(static_mut_refs)]

// ---- 架构层(RV64 M-mode) ----
mod arch;

// ---- 应用层 ----
mod task;
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

use crate::drivers::usb::{dwc2::Ep0, uvc};
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

    let extras = enumerate_topology_only().expect("enum");
    let cam = extras.uvc.expect("no UVC camera");
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
    task::pipeline_loop(&ep0, &sel)
}
