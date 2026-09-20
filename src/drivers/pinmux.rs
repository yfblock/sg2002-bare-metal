//! 引脚复用 FMUX（精简版：只配置 USB_VBUS_DET 引脚），`tock-registers` 封装。
//!
//! `usb_vbus_det`（0xFC）的 FSEL 字段（bits `[2:0]`）决定该引脚复用为
//! `USB_VBUS_DET` 功能还是 GPIO——USB host 模式需要 VBUS 检测走 GPIO 轮询，
//! 故选 `XGPIOB_6`（platform.rs 再配 GPIO1_6 输出拉高）。

use tock_registers::{register_bitfields, register_structs};
use tock_registers::registers::ReadWrite;

use crate::platform::FMUX_BASE;

register_bitfields![u32,
    /// FMUX 功能选择字段（bits [2:0]，每引脚一个寄存器）。
    pub FSEL [
        VAL OFFSET(0) NUMBITS(3) [
            /// 复用为 USB_VBUS_DET 检测功能。
            USB_VBUS_DET = 0,
            /// 复用为 XGPIOB[6]（USB host 的 VBUS 控制）。
            XGPIOB_6 = 3,
        ],
    ]
];

register_structs! {
    /// FMUX 寄存器映射（本固件只用到 0xFC 的 USB_VBUS_DET）。
    pub FmuxRegs {
        (0x00 => _reserved: [u32; 63]),
        (0xfc => pub usb_vbus_det: ReadWrite<u32, FSEL::Register>),
        (0x100 => @END),
    }
}

/// 取 FMUX 寄存器视图（基址为编译期常量，恒有效）。
#[inline]
pub(crate) fn pinmux_regs() -> &'static FmuxRegs {
    unsafe { &*(FMUX_BASE as *const FmuxRegs) }
}
