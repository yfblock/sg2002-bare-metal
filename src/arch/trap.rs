//! M-mode trap：入口汇编（保存/恢复全部寄存器 → `rust_trap_handler` → `mret`）、
//! `mtvec`/`mie`/`mstatus` 初始化，以及中断分发（PLIC 邮箱/USB）与访问异常打印。
//!
//! 邮箱消息的业务处理在 [`crate::ipc::handle_mailbox_irq`]，
//! USB 中断在 [`crate::drivers::usb::dwc2::handle_usb_irq`]——
//! 本层只做架构分发（与旧 comm/trap.rs 同构，不做回调注册）。

use riscv::interrupt::{Exception, Interrupt, Trap};
use riscv::register::mcause::Mcause;
use riscv::register::mtvec::{self, Mtvec};

use super::plic;
use crate::logger;

// trap 入口上下文块在 asm.S(_trap_entry,经 asm.rs 的 global_asm! 引入)。
extern "C" {
    fn _trap_entry();
}

/// 仅安装 trap 向量（Direct 模式）。在一切初始化**之前**调用，保证任何异常
/// 都有入口可去（fault 打印依赖 UART 已被 U-Boot 配好，可用）。
pub unsafe fn init_mtvec() {
    mtvec::write(Mtvec::new(
        _trap_entry as *const () as usize,
        mtvec::TrapMode::Direct,
    ));
}

/// 完整中断初始化：初始化 PLIC + 开启外部中断 + 启动 mtimer 心跳
/// （trap 向量已由 `rust_main` 早期调用 [`init_mtvec`] 安装，此处不重复）。
/// 在平台初始化完成后调用（过早开中断可能在业务未就绪时收到邮箱消息）。
pub unsafe fn init_interrupts() {
    plic::init();
    // MIE.MEIE (bit 11) = 外部中断使能；mstatus.MIE (bit 3) = 全局中断使能
    riscv::register::mie::set_mext();
    riscv::register::mie::set_mtimer();
    riscv::register::mstatus::set_mie();
    super::time::init_heartbeat();
}

/// Trap handler：处理 PLIC 外部中断 + 访问异常。
///
/// `Mcause` 为 `#[repr(C)]` 单 `usize` 字段，按 psABI 走 a0 传参，
/// 与入口汇编 `csrr a0, mcause` 的约定一致。
#[no_mangle]
extern "C" fn rust_trap_handler(mcause: Mcause, cur_sp: usize) -> usize {
    match mcause.cause().try_into() {
        Ok(Trap::Interrupt(Interrupt::MachineExternal)) => {
            // Machine External Interrupt → PLIC claim/分发/complete
            let src = plic::claim();
            match src {
                plic::MBOX_IRQ_SRC => crate::ipc::handle_mailbox_irq(),
                plic::USB_IRQ_SRC => {
                    crate::drivers::usb::dwc2::handle_usb_irq();
                }
                _ => {}
            }
            plic::complete(src);
        }
        Ok(Trap::Interrupt(Interrupt::MachineTimer)) => {
            crate::arch::time::heartbeat_tick();
        }
        Ok(Trap::Exception(Exception::LoadFault | Exception::StoreFault)) => {
            // 访问异常：打印 mepc + mtval（故障地址），然后死循环
            let mepc = riscv::register::mepc::read();
            let mtval = riscv::register::mtval::read();
            // 故障信息优先于锁:try 一次,拿不到也硬打(乱码可忍,丢 fault 不可忍)
            let _lk = logger::line_lock_try();
            logger::print_fmt_nolock(format_args!(
                "\n!!! C906L FAULT: mcause={:#x} mepc={:#x} mtval={:#x} !!!\n",
                mcause.bits() as u64,
                mepc as u64,
                mtval as u64
            ));
            if _lk {
                logger::line_unlock();
            }
            loop {
                riscv::asm::wfi();
            }
        }
        _ => {}
    }
    cur_sp
}
