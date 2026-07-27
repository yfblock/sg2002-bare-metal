//! USB 平台初始化：时钟 / PHY / VBUS / pinmux / DWC2 与 PHY MMIO 基址 / DMA 地址转换。
//!
//! 与 arceos `usb_camera` / StarryOS `cvi_usb_camera` 的平台初始化等价，但裸机无 MMU
//! （identity 映射，VA=PA），故直接用 sg200x-bsp `soc` 里的物理基址，无需 phys_to_virt。

use core::ptr::{read_volatile, write_volatile};

use sg200x_bsp::gpio::{Direction, GPIO, GPIO1_BASE};
use sg200x_bsp::pinmux::{FMUX_BASE, FMUX_USB_VBUS_DET, IOBLK_BASE, IOBLK_GRTC_BASE, Pinmux};
use sg200x_bsp::soc::{CLKGEN_BASE, CV182X_USB2_PHY_BASE, DWC2_BASE, TOP_BASE};
use sg200x_bsp::usb;
use tock_registers::interfaces::Writeable;

const IOBLK_G1_PADDR: usize = 0x0300_1800;
const IOBLK_G1_USB_VBS_DET_OFF: usize = 0x020;
const VBUS_GPIO_PIN: u8 = 6;
const VBUS_GPIO_ACTIVE_HIGH: bool = true;

/// 一次性平台初始化：上电 USB 时钟/PHY/VBUS、配 pinmux、安装 DWC2/PHY 基址与 DMA 转换。
pub fn platform_init() {
    unsafe {
        enable_usb_clocks_cv181x();
    }
    unsafe {
        cvitek_usb_top_host_bringup();
    }
    pinmux_usb_vbus_det_gpio_output_prep();
    enable_usb_vbus_gpio();
    spin_udelay(200_000);

    // identity 映射：VA = PA。DMA 缓冲（sg200x-bsp 的 DMA_BUF，位于本镜像 .bss @ 0x880xxxxx）
    // 的 VA 即 PA，HCDMA 直接写 VA 低 32 位即可。
    usb::set_dwc2_base_virt(DWC2_BASE);
    usb::set_cv182x_phy_base_virt(CV182X_USB2_PHY_BASE);
    usb::set_usb_dma_to_phys_fn(None);
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
        spin_udelay(50);
        write_volatile(rst, v | (1 << 11));
        spin_udelay(50);

        let usb_pin = (top + 0x48) as *mut u32;
        let x = read_volatile(usb_pin);
        let dev_mode = (x & !0xC0u32) | 0xC0u32 | 0x01u32;
        write_volatile(usb_pin, dev_mode);
        spin_udelay(1_000);
        let host_mode = (x & !0xC0u32) | 0x40u32 | 0x01u32;
        write_volatile(usb_pin, host_mode);
        spin_udelay(1_000);

        let eco = (top + 0xB4) as *mut u32;
        write_volatile(eco, read_volatile(eco) | 0x80);
    }
}

fn pinmux_usb_vbus_det_gpio_output_prep() {
    // identity：FMUX_BASE/IOBLK_BASE/IOBLK_GRTC_BASE 直接当 VA
    let pinmux = unsafe { Pinmux::new(FMUX_BASE, IOBLK_BASE, IOBLK_GRTC_BASE) };
    pinmux
        .fmux()
        .usb_vbus_det
        .write(FMUX_USB_VBUS_DET::FSEL::XGPIOB_6);
    let r = (IOBLK_G1_PADDR + IOBLK_G1_USB_VBS_DET_OFF) as *mut u32;
    unsafe {
        let v = read_volatile(r);
        write_volatile(r, v | (7 << 5));
    }
}

fn enable_usb_vbus_gpio() {
    let gpio = unsafe { GPIO::new(GPIO1_BASE) };
    gpio.pin(VBUS_GPIO_PIN).set_direction(Direction::Output);
    gpio.pin(VBUS_GPIO_PIN).set(VBUS_GPIO_ACTIVE_HIGH);
}

/// 粗粒度延时（约 us 微秒级，非精确）。
fn spin_udelay(us: u32) {
    for _ in 0..us.saturating_mul(64) {
        core::hint::spin_loop();
    }
}
