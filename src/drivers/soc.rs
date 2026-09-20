//! SG2002 / CV181x SoC 外设 MMIO **物理基址** 一览。
//!
//! 常量按物理地址升序排列。各驱动模块通过 `// (moved) pub use crate::drivers::soc::…` 保持原有路径的兼容性；
//! 新代码请优先使用本模块中的常量。

// =============================================================================
// 0x020B_xxxx — 多核 / 安全子系统
// =============================================================================

// =============================================================================
// 0x0300_xxxx — 系统 / 时钟 / 复位 / 引脚复用 / IOBLK / USB PHY
// =============================================================================

/// TOP 模块（系统顶层控制寄存器）
pub const TOP_BASE: usize = 0x0300_0000;

/// FMUX（引脚功能复用）
pub const FMUX_BASE: usize = 0x0300_1000;

/// 时钟发生器（`clock-controller`，`cvitek,cv181x-clk`）
pub const CLKGEN_BASE: usize = 0x0300_2000;

/// CV182x 片内 USB2 PHY（DTS `usb@04340000` 第二段 `reg`，物理基址见本常量）
pub const CV182X_USB2_PHY_BASE: usize = 0x0300_6000;

// =============================================================================
// 0x0302_xxxx — GPIO
// =============================================================================

/// GPIO1 (GPIOB)，Active Domain
pub const GPIO1_BASE: usize = 0x0302_1000;

// =============================================================================
// 0x043x_xxxx — SD/MMC / USB (DWC2)
// =============================================================================

/// DWC2 USB OTG 控制器物理基址（DTS `usb@04340000` 第一段 `reg`）
pub const DWC2_BASE: usize = 0x0434_0000;

// =============================================================================
// 0x0502_xxxx — No-die / RTC 域
// =============================================================================

// ==== tock-registers 视图（本驱动使用到的时钟/复位/pinmux 块）====
use tock_registers::{register_bitfields, register_structs, registers::ReadWrite};

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


/// TOP 级控制视图。
#[inline]
pub fn top() -> &'static TopRegs {
    unsafe { &*(TOP_BASE as *const TopRegs) }
}

register_structs! {
    /// IOBLK Group1 pad 配置。
    pub IoblkG1Regs {
        (0x000 => _reserved000: [u32; 8]),
        (0x020 => pub usb_vbus_det: ReadWrite<u32>),
        (0x024 => @END),
    }
}

