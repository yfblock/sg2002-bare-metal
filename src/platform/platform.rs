//! USB 平台初始化：时钟 / PHY / VBUS / pinmux。
//!
//! 与 arceos `usb_camera` / StarryOS `cvi_usb_camera` 的平台初始化等价，但裸机无 MMU
//! （identity 映射，VA=PA）：DWC2/PHY 的 MMIO 基址由 USB 栈直接取
//! `crate::drivers::soc` 常量，无需运行时安装。

use core::ptr::{read_volatile, write_volatile};

use crate::drivers::gpio::{GPIO, GPIO1_BASE};
use tock_registers::interfaces::Writeable;

use crate::drivers::pinmux;
use crate::drivers::soc::{CLKGEN_BASE, TOP_BASE};

const IOBLK_G1_PADDR: usize = 0x0300_1800;
const IOBLK_G1_USB_VBS_DET_OFF: usize = 0x020;
const VBUS_GPIO_PIN: u8 = 6;
const VBUS_GPIO_ACTIVE_HIGH: bool = true;

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
    let b = CLKGEN_BASE;
    let en1 = (b + 0x004) as *mut u32;
    let en2 = (b + 0x008) as *mut u32;
    let byp0 = (b + 0x030) as *mut u32;
    unsafe {
        let v1 = read_volatile(en1);
        let v2 = read_volatile(en2);
        let byp = read_volatile(byp0);
        write_volatile(en1, v1 | (0xFu32 << 28));
        write_volatile(en2, v2 | 1u32);
        write_volatile(byp0, byp & !((1u32 << 17) | (1u32 << 18)));
    }
}

/// PHY ID pad toggle workaround：先写 device 再写 host。
unsafe fn cvitek_usb_top_host_bringup() {
    let top = TOP_BASE;
    let rst = (top + 0x3000) as *mut u32;
    unsafe {
        let v = read_volatile(rst);
        write_volatile(rst, v & !(1 << 11));
        crate::arch::time::delay(core::time::Duration::from_micros(50));
        write_volatile(rst, v | (1 << 11));
        crate::arch::time::delay(core::time::Duration::from_micros(50));

        let usb_pin = (top + 0x48) as *mut u32;
        let x = read_volatile(usb_pin);
        let dev_mode = (x & !0xC0u32) | 0xC0u32 | 0x01u32;
        write_volatile(usb_pin, dev_mode);
        crate::arch::time::delay(core::time::Duration::from_millis(1));
        let host_mode = (x & !0xC0u32) | 0x40u32 | 0x01u32;
        write_volatile(usb_pin, host_mode);
        crate::arch::time::delay(core::time::Duration::from_millis(1));

        let eco = (top + 0xB4) as *mut u32;
        write_volatile(eco, read_volatile(eco) | 0x80);
    }
}

fn pinmux_usb_vbus_det_gpio_output_prep() {
    // 复用 USB_VBUS_DET 引脚为 XGPIOB[6](identity 映射,FMUX 寄存器视图直接取)
    pinmux::regs()
        .usb_vbus_det
        .write(pinmux::FSEL::VAL::XGPIOB_6);
    // IOBLK G1:USB_VBUS_DET pad 驱动能力拉满(bits[7:5]=7,7=最强档)
    let r = (IOBLK_G1_PADDR + IOBLK_G1_USB_VBS_DET_OFF) as *mut u32;
    unsafe {
        let v = read_volatile(r);
        write_volatile(r, v | (7 << 5));
    }
}

fn enable_usb_vbus_gpio() {
    let gpio = unsafe { GPIO::new(GPIO1_BASE) };
    gpio.pin(VBUS_GPIO_PIN).set_output_direction();
    gpio.pin(VBUS_GPIO_PIN).set(VBUS_GPIO_ACTIVE_HIGH);
}
