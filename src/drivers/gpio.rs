//! GPIO 控制(精简版:只保留 GPIO1 的输出引脚操作)。
//!
//! 从 sg200x-bsp/src/gpio.rs 精简而来(原 541 行 → 本文件)。
//! 原版支持 4 个 GPIO 实例 + 中断 + debounce,这里只留最基本的输出设置。
//!
//! 寄存器(DW APB GPIO):
//!   +0x000 数据寄存器(写 bit N = 设 pin N 电平)
//!   +0x004 方向寄存器(1 = 输出)
//!
//! 用法(与原版 API 兼容):
//! ```ignore
//! let gpio = unsafe { GPIO::new(GPIO1_BASE) };
//! gpio.pin(6).set_direction(Direction::Output);
//! gpio.pin(6).set(true);
//! ```

use core::ptr::{read_volatile, write_volatile};

// 重导出 soc 常量(保持 platform.rs 导入路径兼容)
pub use crate::drivers::soc::GPIO1_BASE;

/// GPIO 驱动实例
pub struct GPIO {
    base: usize,
}

impl GPIO {
    /// 创建 GPIO 实例(基址见 soc.rs 的 GPIO0~3_BASE)
    pub unsafe fn new(base: usize) -> Self {
        Self { base }
    }

    /// 获取指定引脚的句柄
    pub fn pin(&self, num: u8) -> Pin {
        Pin { base: self.base, num }
    }
}

/// 单个 GPIO 引脚
pub struct Pin {
    base: usize,
    num: u8,
}

impl Pin {
    /// 设置方向
    pub fn set_output_direction(&self) {
        let ddr = (self.base + 0x004) as *mut u32;
        unsafe {
            let mask = 1u32 << self.num;
            write_volatile(ddr, read_volatile(ddr) | mask);
        }
    }

    /// 设置输出电平
    pub fn set(&self, high: bool) {
        let dr = (self.base + 0x000) as *mut u32;
        unsafe {
            let mask = 1u32 << self.num;
            let v = read_volatile(dr);
            let v = if high { v | mask } else { v & !mask };
            write_volatile(dr, v);
        }
    }
}
