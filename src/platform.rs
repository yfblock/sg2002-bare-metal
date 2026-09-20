//! USB 平台初始化：时钟 / PHY / VBUS / pinmux。
//!
//! 与 arceos `usb_camera` / StarryOS `cvi_usb_camera` 的平台初始化等价，但裸机无 MMU
//! （identity 映射，VA=PA）：DWC2/PHY 的 MMIO 基址由 USB 栈直接取
//! `crate::drivers::soc` 常量，无需运行时安装。
//!
//! USB 平台**专用**的寄存器视图（CLKGEN 时钟门控 / IOBLK pad 驱动）收集在本模块；
//! TOP 块与 JPU 时钟复位共享（`soc::top()`），视图留在 `drivers::soc`——驱动层不
//! 反向依赖板级层。
use tock_registers::interfaces::{ReadWriteable, Writeable};
use tock_registers::{register_bitfields, register_structs};
use tock_registers::registers::ReadWrite;

use crate::drivers::gpio::{GPIO, GPIO1_BASE};
use crate::drivers::pinmux;
use crate::drivers::soc;

// 模块常量
const VBUS_GPIO_PIN: u8 = 6;
const VBUS_GPIO_ACTIVE_HIGH: bool = true;
/// IOBLK Group1 基址（USB_VBUS_DET pad 配置块）。
const IOBLK_G1_BASE: usize = 0x0300_1800;

// USB 平台专用寄存器视图（自 drivers/soc.rs 收集至此,单消费者）
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

/// CLKGEN 视图（基址 `soc::CLKGEN_BASE`,编译期常量恒有效）。
#[inline]
fn clkgen() -> &'static ClkgenRegs {
    unsafe { &*(soc::CLKGEN_BASE as *const ClkgenRegs) }
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
    let top = soc::top();
    // USB 复位脉冲：拉低 → 释放
    top.rst.modify(soc::TOP_RST::USB::CLEAR);
    crate::arch::time::delay(core::time::Duration::from_micros(50));
    top.rst.modify(soc::TOP_RST::USB::SET);
    crate::arch::time::delay(core::time::Duration::from_micros(50));

    // PHY_ID=11(device)→11 保持,再切 PHY_ID=01(host);MODE 位均为 host 驱动
    top.usb_pin.modify(soc::TOP_USB_PIN::MODE::SET + soc::TOP_USB_PIN::PHY_ID.val(0b11));
    crate::arch::time::delay(core::time::Duration::from_millis(1));
    top.usb_pin.modify(soc::TOP_USB_PIN::MODE::SET + soc::TOP_USB_PIN::PHY_ID.val(0b01));
    crate::arch::time::delay(core::time::Duration::from_millis(1));

    top.eco.modify(soc::TOP_ECO::USB::SET);
}

fn pinmux_usb_vbus_det_gpio_output_prep() {
    // 复用 USB_VBUS_DET 引脚为 XGPIOB[6](identity 映射,FMUX 寄存器视图直接取)
    pinmux::regs()
        .usb_vbus_det
        .write(pinmux::FSEL::VAL::XGPIOB_6);
    // IOBLK G1:USB_VBUS_DET pad 驱动能力拉满(bits[7:5]=7,7=最强档)
    ioblk_g1().usb_vbus_det.modify(IOBLK_DRV::DS.val(7));
}

fn enable_usb_vbus_gpio() {
    let gpio = unsafe { GPIO::new(GPIO1_BASE) };
    gpio.pin(VBUS_GPIO_PIN).set_output_direction();
    gpio.pin(VBUS_GPIO_PIN).set(VBUS_GPIO_ACTIVE_HIGH);
}
