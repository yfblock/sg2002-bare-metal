//! GPIO 输出引脚控制（DW APB GPIO，`tock-registers` 封装，精简版：只留输出）。
//!
//! 从 sg200x-bsp/src/gpio.rs 精简而来（原 541 行 → 本文件，去掉中断/debounce）。
//! 寄存器布局：+0x000 数据寄存器 DR（写 bit N = 设 pin N 电平）、
//! +0x004 方向寄存器 DDR（1 = 输出）。基址经 [`GPIO::new`] 注入——
//! 现固件只用 GPIO1（`platform::GPIO1_BASE`）驱动 USB VBUS。

use tock_registers::interfaces::ReadWriteable;
use tock_registers::registers::ReadWrite;
use tock_registers::{register_bitfields, register_structs};

register_bitfields![u32,
    /// DR/DDR 共用布局：bit N = pin N（全 32 引脚单字段掩码）。
    pub PIN [
        /// 引脚位（bit N = pin N）。
        VAL OFFSET(0) NUMBITS(32) [],
    ],
];

register_structs! {
    /// DW APB GPIO 寄存器映射（本固件只用 DR/DDR）。
    pub GpioRegs {
        (0x00 => pub dr: ReadWrite<u32, PIN::Register>),
        (0x04 => pub ddr: ReadWrite<u32, PIN::Register>),
        (0x08 => @END),
    }
}

/// GPIO 驱动实例
pub struct GPIO {
    regs: &'static GpioRegs,
}

impl GPIO {
    /// 创建 GPIO 实例（基址由调用方传入，如 `platform::GPIO1_BASE`）
    pub unsafe fn new(base: usize) -> Self {
        Self {
            regs: &*(base as *const GpioRegs),
        }
    }

    /// 获取指定引脚的句柄
    pub fn pin(&self, num: u8) -> Pin {
        Pin {
            regs: self.regs,
            num,
        }
    }
}

/// 单个 GPIO 引脚
pub struct Pin {
    regs: &'static GpioRegs,
    num: u8,
}

impl Pin {
    /// 设置方向为输出
    pub fn set_output_direction(&self) {
        self.regs.ddr.modify(PIN::VAL.val(1u32 << self.num));
    }

    /// 设置输出电平
    pub fn set(&self, high: bool) {
        self.regs
            .dr
            .modify(PIN::VAL.val(if high { 1u32 << self.num } else { 0 }));
    }
}
