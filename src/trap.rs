//! Trap 入口 + 中断处理：处理来自大核的 PLIC source 61（邮箱中断）。
//!
//! trap 入口保存全部寄存器 → 调 `rust_trap_handler` → 恢复 → mret。
//! 处理器：claim PLIC → 如果 source 61 → 读 HW 邮箱消息（大核发来的）→ 写 DRAM 邮箱回复 → complete。

use core::ptr::{read_volatile, write_volatile};

use crate::plic;
use crate::mailbox;
use crate::uart;

const HW_MBOX_BASE: usize = 0x0190_0000;
const HW_MBOX_CTX: usize = HW_MBOX_BASE + 0x400;
const RECEIVE_CPU: usize = 2; // C906L 小核自己——邮箱寄存器索引 = 接收方 CPU
/// 大核→小核用 slot 1（小核→大核用 slot 0，见 mailbox.rs SLOT）。
/// 两个方向必须错开 slot：`mbox_set` 是全局寄存器，同 slot 会互相触发+覆盖 payload。
const SLOT_B2S: usize = 1;

/// RV64: Machine External Interrupt = bit63 + code 11 = 0x8000_0000_0000_000b。
const CAUSE_M_EXTERNAL: usize = 0x8000_0000_0000_000b;
/// RV64: Load access fault = code 5。
const CAUSE_LOAD_ACCESS: usize = 5;
/// RV64: Store access fault = code 7。
const CAUSE_STORE_ACCESS: usize = 7;

// ============================ trap 入口（asm） ============================

core::arch::global_asm!(
    ".section .text.trap,\"ax\"",
    ".align 2",
    ".global _trap_entry",
    ".type _trap_entry,@function",
"_trap_entry:",
    "   addi sp, sp, -256",
    "   sd ra,   0(sp)",
    "   sd gp,   8(sp)",
    "   sd tp,  16(sp)",
    "   sd t0,  24(sp)",
    "   sd t1,  32(sp)",
    "   sd t2,  40(sp)",
    "   sd s0,  48(sp)",
    "   sd s1,  56(sp)",
    "   sd a0,  64(sp)",
    "   sd a1,  72(sp)",
    "   sd a2,  80(sp)",
    "   sd a3,  88(sp)",
    "   sd a4,  96(sp)",
    "   sd a5, 104(sp)",
    "   sd a6, 112(sp)",
    "   sd a7, 120(sp)",
    "   sd s2, 128(sp)",
    "   sd s3, 136(sp)",
    "   sd s4, 144(sp)",
    "   sd s5, 152(sp)",
    "   sd s6, 160(sp)",
    "   sd s7, 168(sp)",
    "   sd s8, 176(sp)",
    "   sd s9, 184(sp)",
    "   sd s10,192(sp)",
    "   sd s11,200(sp)",
    "   sd t3, 208(sp)",
    "   sd t4, 216(sp)",
    "   sd t5, 224(sp)",
    "   sd t6, 232(sp)",
    "   csrr t0, mepc",
    "   sd t0, 240(sp)",
    "   csrr t0, mstatus",
    "   sd t0, 248(sp)",
    "   csrr a0, mcause",
    "   mv  a1, sp",
    "   call {handler}",
    "   mv  sp, a0",
    "   ld t0, 248(sp)",
    "   csrw mstatus, t0",
    "   ld t0, 240(sp)",
    "   csrw mepc, t0",
    "   ld ra,   0(sp)",
    "   ld gp,   8(sp)",
    "   ld tp,  16(sp)",
    "   ld t0,  24(sp)",
    "   ld t1,  32(sp)",
    "   ld t2,  40(sp)",
    "   ld s0,  48(sp)",
    "   ld s1,  56(sp)",
    "   ld a0,  64(sp)",
    "   ld a1,  72(sp)",
    "   ld a2,  80(sp)",
    "   ld a3,  88(sp)",
    "   ld a4,  96(sp)",
    "   ld a5, 104(sp)",
    "   ld a6, 112(sp)",
    "   ld a7, 120(sp)",
    "   ld s2, 128(sp)",
    "   ld s3, 136(sp)",
    "   ld s4, 144(sp)",
    "   ld s5, 152(sp)",
    "   ld s6, 160(sp)",
    "   ld s7, 168(sp)",
    "   ld s8, 176(sp)",
    "   ld s9, 184(sp)",
    "   ld s10,192(sp)",
    "   ld s11,200(sp)",
    "   ld t3, 208(sp)",
    "   ld t4, 216(sp)",
    "   ld t5, 224(sp)",
    "   ld t6, 232(sp)",
    "   addi sp, sp, 256",
    "   mret",
    handler = sym rust_trap_handler,
);

extern "C" {
    pub fn _trap_entry();
}

/// 安装 trap 向量 + 初始化 PLIC + 开启外部中断。
pub unsafe fn init() {
    let entry = (_trap_entry as *const ()) as usize;
    core::arch::asm!("csrw mtvec, {0}", in(reg) entry, options(nostack, preserves_flags));
    plic::init();
    // MIE.MEIE (bit 11) = 外部中断使能
    let meie: usize = 1 << 11;
    core::arch::asm!("csrs mie, {0}", in(reg) meie, options(nostack, preserves_flags));
    // mstatus.MIE (bit 3) = 全局中断使能
    let mie: usize = 1 << 3;
    core::arch::asm!("csrs mstatus, {0}", in(reg) mie, options(nostack, preserves_flags));
}

/// Trap handler：处理 PLIC 外部中断 + 定时器中断 + 访问异常。
#[no_mangle]
extern "C" fn rust_trap_handler(mcause: usize, cur_sp: usize) -> usize {
    if mcause == CAUSE_M_EXTERNAL {
        let src = plic::claim();
        if src == plic::MBOX_IRQ_SRC {
            handle_mailbox_irq();
        } else if src == plic::USB_IRQ_SRC {
            sg200x_bsp::usb::host::dwc2::ep0::handle_usb_irq();
        }
        plic::complete(src);
    } else if mcause == CAUSE_LOAD_ACCESS || mcause == CAUSE_STORE_ACCESS {
        // 访问异常：打印 mepc + mtval（故障地址），然后死循环
        let mepc: usize;
        let mtval: usize;
        unsafe {
            core::arch::asm!("csrr {0}, mepc", out(reg) mepc);
            core::arch::asm!("csrr {0}, mtval", out(reg) mtval);
        }
        uart::print("\n!!! C906L FAULT: mcause=");
        uart::print_hex(mcause as u64);
        uart::print(" mepc=");
        uart::print_hex(mepc as u64);
        uart::print(" mtval=");
        uart::print_hex(mtval as u64);
        uart::print(" !!!\n");
        loop {
            unsafe { core::arch::asm!("wfi") };
        }
    }
    cur_sp
}

/// 处理邮箱中断：读大核发来的消息 → 回写 DRAM 邮箱 reply 字段 → 清 HW 邮箱。
fn handle_mailbox_irq() {
    unsafe {
        // 读 HW 邮箱 context slot 1（大核发来的 4B 消息）
        let msg = read_volatile((HW_MBOX_CTX + SLOT_B2S * 8) as *const u32);

        // 清 HW 邮箱中断：清掉 cpu_mbox_set[2] 上所有 pending bit + 关对应 en bit，
        // 让邮箱控制器 deassert。不清就会被 level-triggered PLIC 无限重投。
        let int_val = read_volatile((HW_MBOX_BASE + 0x10 + RECEIVE_CPU * 16 + 8) as *const u32);
        if int_val != 0 {
            write_volatile((HW_MBOX_BASE + 0x10 + RECEIVE_CPU * 16) as *mut u32, int_val);
            let en = (HW_MBOX_BASE + RECEIVE_CPU * 4) as *mut u32;
            let old = read_volatile(en);
            write_volatile(en, old & !int_val);
        }

        if int_val & (1 << SLOT_B2S) != 0 {
            // 清掉 context slot，避免下次读到旧值
            write_volatile((HW_MBOX_CTX + SLOT_B2S * 8) as *mut u64, 0);

            // 控制消息 0xF0_49_<cmd>_<arg>，见 control.rs。
            if msg & 0xFFFF_0000 == 0xF049_0000 {
                let cmd = (msg >> 8) & 0xFF;
                let arg = msg & 0xFF;
                match cmd {
                    0x50 => crate::control::set_paused(arg != 0),
                    0x56 => sg200x_bsp::ive::set_input_fmt(arg),
                    0x57 => crate::control::set_muted(arg != 0),
                    _ => {}
                }
            } else {
                uart::print("[MB-RX] big->small msg=");
                uart::print_hex(msg as u64);
                uart::print("\n");
            }
            // 回写 DRAM 邮箱 reply 字段，大核读回即证明往返成功
            mailbox::write_reply(msg);
        }
    }
}
