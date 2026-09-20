//! SG2002 板级：SoC MMIO **地址表** + USB 平台初始化（时钟 / PHY / VBUS / pinmux）。
//!
//! 全部外设物理基址集中在本模块（按物理地址升序），各驱动 `use crate::platform`
//! 取用——BSP 风格单一事实来源；核内中断控制器（CLINT/PLIC）归 arch 层。
//! 核内中断控制器 CLINT/PLIC 的地址在 arch 层(使用方唯一)。另带 USB 平台初始化(与 arceos `usb_camera` / StarryOS `cvi_usb_camera` 等价,
//! 裸机 identity 映射 VA=PA 无需运行时安装）与 TOP/CLKGEN/IOBLK 寄存器视图。
use tock_registers::interfaces::{ReadWriteable, Writeable};
use tock_registers::{register_bitfields, register_structs};
use tock_registers::registers::ReadWrite;

use crate::drivers::gpio::GPIO;
use crate::drivers::pinmux;

// SoC MMIO 地址表（物理基址,升序）
/// cvi 硬件邮箱控制器。
pub const HW_MBOX_BASE: usize = 0x0190_0000;
/// TOP 模块（系统顶层控制寄存器）。
pub const TOP_BASE: usize = 0x0300_0000;
/// FMUX（引脚功能复用）。
pub const FMUX_BASE: usize = 0x0300_1000;
/// IOBLK Group1（USB_VBUS_DET pad 配置块）。
pub const IOBLK_G1_BASE: usize = 0x0300_1800;
/// 时钟发生器（`clock-controller`，`cvitek,cv181x-clk`）。
pub const CLKGEN_BASE: usize = 0x0300_2000;
/// CV182x 片内 USB2 PHY（DTS `usb@04340000` 第二段 `reg`）。
pub const CV182X_USB2_PHY_BASE: usize = 0x0300_6000;
/// DW APB 看门狗（dts `cv-wd@0x3010000`）。
pub const WDT_BASE: usize = 0x0301_0000;
/// GPIO1 (GPIOB)，Active Domain。
pub const GPIO1_BASE: usize = 0x0302_1000;
/// UART0（DW APB 8250,32-bit 步长;U-Boot 已初始化）。
pub const UART0_BASE: usize = 0x0414_0000;
/// DWC2 USB OTG 控制器（DTS `usb@04340000` 第一段 `reg`）。
pub const DWC2_BASE: usize = 0x0434_0000;
/// IVE 智能视觉引擎。
pub const IVE_BASE: usize = 0x0A0A_0000;
/// JPU JPEG 编解码器。
pub const JPU_REG_BASE: usize = 0x0B00_0000;
/// mtimer 心跳计数（10ms/拍,借邮箱 ctx[3].aux——turn 只用低 4B）。
pub const HEARTBEAT_PA: usize = 0x0190_041C;
/// 最近被时钟打断的 PC 采样（借邮箱 ctx[1].aux,B2S 协议不使用 aux）。
pub const LAST_PC_PA: usize = 0x0190_040C;

// 预留 rtos 区共享布局（镜像加载/入口 0x8FE00000 由 memory.ld 定;区段 [0x8FE00000, 0x90000000) dtb rtos_region,
// 大核不分配;见 memory.ld 与 yuv_buf.rs 平面几何说明）
/// YUV 单缓冲（JPU DMA 直写;640×480 YUV422 = 614400B）。
pub const YUV_BUF_PA: usize = 0x8FE8_8000;
/// YUV 缓冲容量（摄像头实际发 YUV422,frame_size = w*h*2）。
pub const YUV_BUF_SIZE: usize = 614400;
/// JPU DMA 内存池（stream_buf;YUV 缓冲之后避免重叠）。
pub const JPU_POOL_PA: usize = 0x8FF1_E000;
pub const JPU_POOL_SIZE: usize = 0x0004_0000; // 256KB
/// RGB888 planar 输出（IVE CSC 输出;三平面各 307200B）。
pub const RGB_BUF_PA: usize = 0x8FF5_E000;

// 模块常量
const VBUS_GPIO_PIN: u8 = 6;
const VBUS_GPIO_ACTIVE_HIGH: bool = true;

// TOP 块寄存器视图（与 JPU 时钟复位共享,自 drivers/soc.rs 收集至此）
register_bitfields![u32,
    /// TOP 复位寄存器（bit4: JPEG 复位释放; bit11: USB 复位）。
    pub TOP_RST [
        JPEG OFFSET(4) NUMBITS(1) [],
        USB OFFSET(11) NUMBITS(1) [],
    ],
    /// TOP USB pin 模式（bit0: device/host 模式, bit6/7: PHY ID pad）。
    pub TOP_USB_PIN [
        MODE OFFSET(0) NUMBITS(1) [],
        PHY_ID OFFSET(6) NUMBITS(2) [],
    ],
    /// TOP ECO 寄存器（bit7: USB eco）。
    pub TOP_ECO [
        USB OFFSET(7) NUMBITS(1) [],
    ],
];

register_structs! {
    /// TOP 级控制（复位/USB pin/ECO）。
    pub TopRegs {
        (0x000 => _reserved000: [u32; 18]),
        (0x048 => pub usb_pin: ReadWrite<u32, TOP_USB_PIN::Register>),
        (0x04c => _reserved04c: [u32; 26]),
        (0x0b4 => pub eco: ReadWrite<u32, TOP_ECO::Register>),
        (0x0b8 => _reserved0b8: [u32; 2004]),
        (0x2008 => pub clk_jpeg: ReadWrite<u32>),
        (0x200c => _reserved200c: [u32; 1021]),
        (0x3000 => pub rst: ReadWrite<u32, TOP_RST::Register>),
        (0x3004 => @END),
    }
}

/// TOP 级控制视图（基址 `TOP_BASE`,编译期常量恒有效）。
#[inline]
pub fn top() -> &'static TopRegs {
    unsafe { &*(TOP_BASE as *const TopRegs) }
}

// USB 平台专用寄存器视图
register_bitfields![u32,
    /// CLKGEN 时钟使能寄存器 1（bit28-31: USB 相关时钟门控）。
    pub CLKGEN_EN1 [
        USB_CLK OFFSET(28) NUMBITS(4) [],
    ],
    /// CLKGEN 时钟使能寄存器 2。
    pub CLKGEN_EN2 [
        EN OFFSET(0) NUMBITS(1) [],
    ],
    /// CLKGEN bypass 0（bit17/18: USB PHY bypass）。
    pub CLKGEN_BYP0 [
        USB_BYPASS OFFSET(17) NUMBITS(2) [],
    ],
    /// IOBLK pad 驱动能力（bits[7:5],7 = 最强档）。
    pub IOBLK_DRV [
        DS OFFSET(5) NUMBITS(3) [],
    ],
];

register_structs! {
    /// CLKGEN 时钟生成器。
    pub ClkgenRegs {
        (0x000 => _reserved000),
        (0x004 => pub en1: ReadWrite<u32, CLKGEN_EN1::Register>),
        (0x008 => pub en2: ReadWrite<u32, CLKGEN_EN2::Register>),
        (0x00c => _reserved00c: [u32; 9]),
        (0x030 => pub byp0: ReadWrite<u32, CLKGEN_BYP0::Register>),
        (0x034 => @END),
    }
}

register_structs! {
    /// IOBLK Group1 pad 配置。
    pub IoblkG1Regs {
        (0x000 => _reserved000: [u32; 8]),
        (0x020 => pub usb_vbus_det: ReadWrite<u32, IOBLK_DRV::Register>),
        (0x024 => @END),
    }
}

/// CLKGEN 视图（编译期常量恒有效）。
#[inline]
fn clkgen() -> &'static ClkgenRegs {
    unsafe { &*(CLKGEN_BASE as *const ClkgenRegs) }
}

/// IOBLK Group1 视图。
#[inline]
fn ioblk_g1() -> &'static IoblkG1Regs {
    unsafe { &*(IOBLK_G1_BASE as *const IoblkG1Regs) }
}

/// 一次性平台初始化：上电 USB 时钟/PHY/VBUS、配 pinmux。
pub fn platform_init() {
    unsafe {
        enable_usb_clocks_cv181x();
    }
    unsafe {
        cvitek_usb_top_host_bringup();
    }
    pinmux_usb_vbus_det_gpio_output_prep();
    enable_usb_vbus_gpio();
    crate::arch::time::delay(core::time::Duration::from_millis(200));
}

unsafe fn enable_usb_clocks_cv181x() {
    let clk = clkgen();
    clk.en1.modify(CLKGEN_EN1::USB_CLK.val(0xF));
    clk.en2.modify(CLKGEN_EN2::EN::SET);
    clk.byp0.modify(CLKGEN_BYP0::USB_BYPASS.val(0));
}

/// PHY ID pad toggle workaround：先写 device 再写 host。
unsafe fn cvitek_usb_top_host_bringup() {
    let top = top();
    // USB 复位脉冲：拉低 → 释放
    top.rst.modify(TOP_RST::USB::CLEAR);
    crate::arch::time::delay(core::time::Duration::from_micros(50));
    top.rst.modify(TOP_RST::USB::SET);
    crate::arch::time::delay(core::time::Duration::from_micros(50));

    // PHY_ID=11(device)→11 保持,再切 PHY_ID=01(host);MODE 位均为 host 驱动
    top.usb_pin.modify(TOP_USB_PIN::MODE::SET + TOP_USB_PIN::PHY_ID.val(0b11));
    crate::arch::time::delay(core::time::Duration::from_millis(1));
    top.usb_pin.modify(TOP_USB_PIN::MODE::SET + TOP_USB_PIN::PHY_ID.val(0b01));
    crate::arch::time::delay(core::time::Duration::from_millis(1));

    top.eco.modify(TOP_ECO::USB::SET);
}

fn pinmux_usb_vbus_det_gpio_output_prep() {
    // 复用 USB_VBUS_DET 引脚为 XGPIOB[6](identity 映射,FMUX 寄存器视图直接取)
    pinmux::pinmux_regs()
        .usb_vbus_det
        .write(pinmux::FSEL::VAL::XGPIOB_6);
    // IOBLK G1:USB_VBUS_DET pad 驱动能力拉满(bits[7:5]=7,7=最强档)
    ioblk_g1().usb_vbus_det.modify(IOBLK_DRV::DS.val(7));
}

fn enable_usb_vbus_gpio() {
    let gpio = unsafe { GPIO::new(GPIO1_BASE) };
    let pin = gpio.pin(VBUS_GPIO_PIN);
    pin.set_output_direction();
    pin.set(VBUS_GPIO_ACTIVE_HIGH);
}
