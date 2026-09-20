//! 大小核通信协议：DRAM 邮箱（0x9004_0000）共享内存 ABI + 帧通知 + B2S 消息处理
//! + 大核下发的运行时控制（pause/mute）。
//!
//! DRAM 邮箱用 `tock-registers` 寄存器视图封装（[`MailboxRegs`]）。注意它不是
//! MMIO 而是**共享 DRAM**：访问走 cache，靠 [`flush_mailbox`] 的 clean 保证
//! 对大核可见（写序 + volatile 由 tock 访问器保证）。
//!
//! 硬件邮箱控制器（门铃/认领）在 [`crate::drivers::mailbox`]；ISR 由
//! [`crate::arch::trap`] 分发到本模块的 [`handle_mailbox_irq`]。

use core::sync::atomic::{AtomicBool, Ordering};
use tock_registers::register_structs;
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::registers::ReadWrite;

use crate::drivers::mailbox as hw;

// ---------------------------------------------------------------------------
// DRAM 邮箱（32 字节，tock-registers 视图）
// ---------------------------------------------------------------------------

register_structs! {
    /// DRAM 邮箱寄存器视图：偏移 0-15 小核→大核（帧信息），16-31 大核→小核（回复）。
    ///
    /// `flags` 不用位域类型——height(bits 8-19) 与 `FLAG_YUV_READY`(bit15) 在协议里
    /// 本就重叠（见 [`FLAG_SLOT`] 注释），编码统一走 [`encode_dims`] 等 u32 辅助。
    pub MailboxRegs {
        (0x00 => pub magic: ReadWrite<u32>),
        (0x04 => pub frame_count: ReadWrite<u32>),
        (0x08 => pub yuv_size: ReadWrite<u32>),   // 0 = 此帧无 YUV，只有 MJPEG
        /// bit0 SOI, bit1 EOI, bit15 YUV ready, bits 20-31/8-19 = width/height
        /// （经 [`encode_dims`] 编码）。
        /// 位布局注意：height 占 bits 8-19，h=480(0x1E0)<<8 = 0x1E000 会让
        /// bit14/bit15 恒为 1——所以 bit2 之外不能再塞别的标志位
        /// （曾预留作双缓冲 slot 索引，特性未启用，已删）。
        (0x0c => pub flags: ReadWrite<u32>),
        (0x10 => pub reply_magic: ReadWrite<u32>),
        (0x14 => pub reply_data: ReadWrite<u32>),
        (0x18 => pub reply_seq: ReadWrite<u32>),
        (0x1c => _pad: u32),                      // ABI 保留字,不访问
        (0x20 => @END),
    }
}

/// 取 DRAM 邮箱视图（identity 映射，基址为编译期常量，恒有效）。
#[inline]
fn regs() -> &'static MailboxRegs {
    unsafe { &*(MAILBOX_PA as *const MailboxRegs) }
}

pub const MAILBOX_PA: usize = 0x9004_0000;
pub const MAILBOX_MAGIC: u32 = 0xC906_C906;
pub const REPLY_MAGIC: u32 = 0x52504C59;

/// flags 位定义
pub const FLAG_SOI: u32 = 1 << 0;
pub const FLAG_EOI: u32 = 1 << 1;
pub const FLAG_YUV_READY: u32 = 1 << 15;

/// 把 width/height 编码到 flags 的高位（解码在读者侧，见 b2s-comm-test）。
pub fn encode_dims(w: u32, h: u32) -> u32 {
    ((w & 0xFFF) << 20) | ((h & 0xFFF) << 8)
}

/// 把 DRAM 邮箱那几行 cache 写回内存。
///
/// **必须显式做**：小核是带 cache 的 identity 映射，寄存器访问器的 volatile 只
/// 保证不被编译器优化掉，不保证出 cache。之前能工作纯属侥幸——JPU 每帧两次
/// `dcache_invalidate_range`(614400 B) 远大于 L1 D-cache，顺带把邮箱的脏行
/// 也挤回了 DRAM；一旦去掉那两次 invalidate，大核就只能读到全 0。
#[inline]
fn flush_mailbox() {
    crate::arch::cache::dcache_clean_range(
        MAILBOX_PA,
        core::mem::size_of::<MailboxRegs>(),
    );
}

/// 整结构写入邮箱（启动存活标记 / panic 标记用；帧信息走 [`notify`]）。
pub fn write(frame_count: u32, yuv_size: u32, flags: u32) {
    let r = regs();
    r.magic.set(MAILBOX_MAGIC);
    r.frame_count.set(frame_count);
    r.yuv_size.set(yuv_size);
    r.flags.set(flags);
    r.reply_magic.set(0);
    r.reply_data.set(0);
    r.reply_seq.set(0);
    flush_mailbox();
}

/// 写帧信息 + 触发硬件邮箱中断通知大核。
///
/// 只写 offset 0..16（小核→大核那一半），**不要**整结构 RMW：
/// reply 字段由中断上下文的 `write_reply` 写，整结构读改写会在
/// "读 current → 写回" 之间把 ISR 刚写的 reply 覆盖掉（13 FPS 下必然丢）。
pub fn notify(frame_count: u32, yuv_size: u32, flags: u32) {
    let r = regs();
    r.frame_count.set(frame_count);
    r.yuv_size.set(yuv_size);
    r.flags.set(flags);
    // magic 最后写：大核以 magic 作为"这块内容有效"的判据
    r.magic.set(MAILBOX_MAGIC);

    flush_mailbox();

    // context[0] 同步一份实时帧计数（MMIO 非缓存，大核轮询它判帧流活性）。
    unsafe { hw::doorbell_big(frame_count, MAILBOX_MAGIC) };
}

/// 写回复（小核 ISR 收到大核消息后调用）。
///
/// 只碰 offset 16..32（大核→小核那一半），和 `notify` 写的 0..16 完全不重叠，
/// 所以中断上下文和主循环并发写也不会互相覆盖。
pub fn write_reply(msg: u32) {
    let r = regs();
    let seq = r.reply_seq.get();
    r.reply_data.set(msg);
    r.reply_seq.set(seq.wrapping_add(1));
    // magic 最后写，作为有效标志
    r.reply_magic.set(REPLY_MAGIC);
    flush_mailbox();
}

// ---------------------------------------------------------------------------
// 大核下发的运行时控制（B2S 控制消息 0xF0_49_<cmd>_<arg> 的状态侧）
// ---------------------------------------------------------------------------

static PAUSED: AtomicBool = AtomicBool::new(false);
static MUTED: AtomicBool = AtomicBool::new(false);

/// 流水线是否已暂停。
#[inline]
pub fn paused() -> bool {
    PAUSED.load(Ordering::Acquire)
}

pub fn set_paused(v: bool) {
    PAUSED.store(v, Ordering::Release);
}

/// 串口是否静音。
#[inline]
pub fn muted() -> bool {
    MUTED.load(Ordering::Relaxed)
}

pub fn set_muted(v: bool) {
    MUTED.store(v, Ordering::Relaxed);
}

// ---------------------------------------------------------------------------
// B2S 消息处理（邮箱 ISR，由 arch::trap 分发）
// ---------------------------------------------------------------------------

/// 处理邮箱中断：认领大核消息 → 控制命令 / 回显 → 回写 DRAM 邮箱 reply 字段。
pub fn handle_mailbox_irq() {
    // 排水:一轮 claim 的窗口内若大核又敲铃,级别触发的 PLIC 会再次进来;
    // 这里顺带就地消费,省一次 trap 进出(大核 2s 一发,循环至多两圈)。
    while let Some(msg) = unsafe { hw::claim_b2s() } {
        // 控制消息 0xF0_49_<cmd>_<arg>（'I' = IVE 调试通道）。
        if msg & 0xFFFF_0000 == 0xF049_0000 {
            let cmd = (msg >> 8) & 0xFF;
            let arg = msg & 0xFF;
            match cmd {
                0x50 => set_paused(arg != 0),
                0x56 => crate::drivers::ive::set_input_fmt(arg),
                0x57 => set_muted(arg != 0),
                _ => {}
            }
        } else if !muted() {
            // ISR 上下文:try 锁一次,拿不到就丢这行回显(绝不等待,防 ISR 延迟);
            // 三段输出拼在一次锁窗口内,保证 [MB-RX] 行不被大核打断
            use crate::logger;
            if logger::line_lock_try() {
                logger::print_fmt_nolock(format_args!("[MB-RX] big->small msg={:#x}\n", msg));
                logger::line_unlock();
            }
        }
        // 回写 DRAM 邮箱 reply 字段，大核读回即证明往返成功
        write_reply(msg);
    }
}
