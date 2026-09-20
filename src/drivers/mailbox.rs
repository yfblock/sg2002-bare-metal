//! cvi 硬件邮箱控制器驱动（0x0190_0000），`tock-registers` 封装。
//!
//! 寄存器布局（cvi_mailbox.h / osdrv rtos_cmdqu.c）：
//! - `+0x00/04/08`：`cpu_mbox_en[0/1/2]`——决定 `mbox_set` 门铃打到哪个核
//!   （索引=接收方 CPU 编号：0=CA53，1=C906B 大核，2=C906L 小核）。
//! - `+0x10 + cpu*16`：各 CPU 的中断 bank（`int_clr` W1C 清 pending /
//!   `int_mask` / `int_val` pending 位图，bit n = slot n）。
//! - `+0x60`：全局 `mbox_set`（敲铃）。
//! - `+0x400`：context buffer，每 slot 8 字节（低 4B payload，高 4B 辅助）。
//!
//! 两个方向必须用不同 slot：`mbox_set` 是全局寄存器，同 slot 会互相触发+覆盖 payload。
//!
//! 本驱动只做寄存器原语；DRAM 邮箱协议（0x9004_0000 共享内存 ABI + 帧通知 +
//! B2S 控制消息）在 [`crate::ipc`]。

use tock_registers::register_structs;
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::registers::{ReadOnly, ReadWrite, WriteOnly};

/// 控制器 MMIO 基址。
pub const HW_MBOX_BASE: usize = 0x0190_0000;

/// CPU 编号（cvi_mailbox.h）：0=CA53，1=C906B 大核，2=C906L 小核。
pub const CPU_BIG: usize = 1;
pub const CPU_SMALL: usize = 2;

/// 小核→大核 slot（大核 PLIC source 101，dts `rtos_cmdqu interrupts=<101>`）。
pub const SLOT_S2B: usize = 0;
/// 大核→小核 slot（小核 PLIC source 61）。
pub const SLOT_B2S: usize = 1;

/// 跨核 UART 行锁借用的 context slot2（双 flag：低 4B=小核、高 4B=大核）。
/// Dekker 协议两侧必须同款（小核 platform/uart 与大核 tools/bigcore-bm）。
pub const CTX_SLOT2: usize = HW_MBOX_BASE + 0x400 + 2 * 8;
/// 跨核 UART 行锁的 turn 字（slot3 低 4B）。
pub const CTX_SLOT3: usize = HW_MBOX_BASE + 0x400 + 3 * 8;

register_structs! {
    /// context buffer 单个 slot（8 字节：低 4B payload，高 4B 辅助）。
    pub MboxCtxSlot {
        (0x00 => pub payload: ReadWrite<u32>),
        (0x04 => pub aux: ReadWrite<u32>),
        (0x08 => @END),
    }
}

register_structs! {
    /// 单个 CPU 的中断 bank（基址 = 0x10 + cpu*16）。
    pub MboxCpuBank {
        (0x00 => pub int_clr: WriteOnly<u32>),    // W1C 清 pending
        (0x04 => pub int_mask: ReadWrite<u32>),
        (0x08 => pub int_val: ReadOnly<u32>),     // pending 位图(bit n = slot n)
        (0x0c => _reserved0c),
        (0x10 => @END),
    }
}

register_structs! {
    /// cvi 硬件邮箱控制器完整寄存器映射（仅声明本驱动用到的字段）。
    pub MboxRegs {
        /// `cpu_mbox_en[0]`（CA53，本固件未用）。
        (0x00 => _reserved00),
        /// `cpu_mbox_en[1]`（C906B 大核）。
        (0x04 => pub cpu1_en: ReadWrite<u32>),
        /// `cpu_mbox_en[2]`（C906L 小核）。
        (0x08 => pub cpu2_en: ReadWrite<u32>),
        (0x0c => _reserved0c),
        /// 各 CPU 的中断 bank（索引=CPU 编号）。
        (0x10 => pub banks: [MboxCpuBank; 3]),
        (0x40 => _reserved40: [u32; 8]),
        /// 全局敲铃（由 `cpu_mbox_en[cpu]` 决定谁收）。
        (0x60 => pub mbox_set: WriteOnly<u32>),
        (0x64 => _reserved64: [u32; 231]),
        /// context buffer（slot 0-3：0/1 消息载荷，2/3 借给跨核行锁）。
        (0x400 => pub ctx: [MboxCtxSlot; 4]),
        (0x420 => @END),
    }
}

/// 取控制器寄存器视图（基址为编译期常量，恒有效）。
#[inline]
fn regs() -> &'static MboxRegs {
    unsafe { &*(HW_MBOX_BASE as *const MboxRegs) }
}

/// 小核→大核门铃：把 `(payload0, payload1)` 写入 slot0 并敲铃中断大核。
pub unsafe fn doorbell_big(payload0: u32, payload1: u32) {
    let slot = &regs().ctx[SLOT_S2B];
    slot.payload.set(payload0);
    slot.aux.set(payload1);

    let big = &regs().banks[CPU_BIG];
    big.int_clr.set(1 << SLOT_S2B);
    // 显式解除 int_mask（reset 默认应为 0；写 0 排除“被 mask 掉所以不上 PLIC”的可能）。
    big.int_mask.set(0);

    let en = &regs().cpu1_en;
    // 绝对写:cpu1_en 只有 slot0(S2B)一个使用者,不 RMW(同 claim_b2s 的理由)。
    en.set(1 << SLOT_S2B);

    regs().mbox_set.set(1 << SLOT_S2B);
}

/// 大核→小核：认领一条消息（由小核邮箱 ISR 调用）。
///
/// 清 pending + 关对应 en bit（让控制器 deassert，不清会被 level-triggered PLIC
/// 无限重投）；仅当 slot1 有事件时返回其 payload（并清 slot 防下次读到旧值）。
/// 小核 ISR 消费后关 en bit 的副作用，同时被大核用作「消费确认」证据
/// （tools/b2s-comm-test-bm 轮询 `cpu_mbox_en[2]` bit 清零）。
pub unsafe fn claim_b2s() -> Option<u32> {
    let small = &regs().banks[CPU_SMALL];
    let int_val = small.int_val.get();
    if int_val & (1 << SLOT_B2S) == 0 {
        return None;
    }
    // ① 先取 payload:大核把「en 清零」当消费确认,一旦清 en 它就可能覆写
    //    ctx slot1 发下一轮——payload 必须在此之前读走。
    let slot = &regs().ctx[SLOT_B2S];
    let msg = slot.payload.get();
    slot.payload.set(0);
    // 注意:ctx[1].aux(0x0190_040C)已被 mtimer 心跳借用为 PC 采样,这里不再清零。
    // ② 再清 pending。绝对写 bit1,不清其他位(int_val 可能含新到期的位)。
    small.int_clr.set(1 << SLOT_B2S);
    // ③ 最后清 en。绝对写 0:cpu2_en 只有 slot1(B2S)一个使用者,而大核
    //    b2s_send 对同一字 RMW 置位——跨核 RMW 交错会把刚清的位复活,
    //    大核的消费确认(b2s_consumed 轮询该位)从此永假,双侧卡死。
    regs().cpu2_en.set(0);
    Some(msg)
}
