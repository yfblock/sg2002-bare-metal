//! 大小核通信邮箱（32 字节）。
//!
//! 偏移 0-15：小核 → 大核（帧信息：magic/frame_count/yuv_size/flags+dimensions）。
//! 偏移 16-31：大核 → 小核回复（reply_magic/reply_data/reply_seq/_pad）。

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Mailbox {
    // 0-15: 小核 → 大核
    pub magic: u32,
    pub frame_count: u32,
    pub yuv_size: u32,   // YUV 数据字节数（0 = 此帧无 YUV，只有 MJPEG）
    pub flags: u32,      // bit0: SOI ok, bit1: EOI ok, bit15: YUV ready, bit16-31: width<<16|height
    // 16-31: 大核 → 小核回复
    pub reply_magic: u32,
    pub reply_data: u32,
    pub reply_seq: u32,
    pub _pad: u32,
}

pub const MAILBOX_PA: usize = 0x9004_0000;
pub const MAILBOX_MAGIC: u32 = 0xC906_C906;
pub const REPLY_MAGIC: u32 = 0x52504C59;

/// flags 位定义
pub const FLAG_SOI: u32 = 1 << 0;
pub const FLAG_EOI: u32 = 1 << 1;
pub const FLAG_YUV_READY: u32 = 1 << 15;
/// bit2：双缓冲 slot 索引（0/1）。
///
/// 注意：**不能用 bit14**——`encode_dims` 的 height 字段占 bits 8-19，
/// h=480(0x1E0) << 8 = 0x1E000，bit14 恰好为 1，会覆盖 slot 位。
/// bit15 同理被 height 覆盖（但 FLAG_YUV_READY 恰好一直为 true，没暴露问题）。
pub const FLAG_SLOT: u32 = 1 << 2;

/// 把 width/height 编码到 flags 的高位。
pub fn encode_dims(w: u32, h: u32) -> u32 {
    ((w & 0xFFF) << 20) | ((h & 0xFFF) << 8)
}

/// 从 flags 解码 width/height。
pub fn decode_dims(flags: u32) -> (u32, u32) {
    ((flags >> 20) & 0xFFF, (flags >> 8) & 0xFFF)
}

/// 把 slot 索引（0/1）编码到 flags bit2。
pub fn encode_slot(slot: u32) -> u32 {
    (slot & 1) << 2
}

/// 从 flags 解码 slot 索引。
pub fn decode_slot(flags: u32) -> u32 {
    (flags >> 2) & 1
}

/// 把 DRAM 邮箱那几行 cache 写回内存。
///
/// **必须显式做**：小核是带 cache 的 identity 映射，`write_volatile` 只保证不被
/// 编译器优化掉，不保证出 cache。之前能工作纯属侥幸——JPU 每帧两次
/// `dcache_invalidate_range`(614400 B) 远大于 L1 D-cache，顺带把邮箱的脏行
/// 也挤回了 DRAM；一旦去掉那两次 invalidate，大核就只能读到全 0。
#[inline]
fn flush_mailbox() {
    sg200x_bsp::utils::cache::dcache_clean_range(
        MAILBOX_PA,
        core::mem::size_of::<Mailbox>(),
    );
}

const HW_MBOX_BASE: usize = 0x0190_0000;
const HW_MBOX_CONTEXT: usize = HW_MBOX_BASE + 0x400;
/// 接收方 CPU 编号（cvi_mailbox.h）：0=CA53, 1=C906B(大核), 2=C906L(小核)。
/// 邮箱寄存器索引用的是**接收方**编号——要中断大核就写 cpu_mbox_en[1]，
/// 对应 PLIC source 101（dts `rtos_cmdqu interrupts=<101>`, riscv,ndev=101）。
/// 参考 osdrv rtos_cmdqu.c `rtos_cmdqu_send()`: SEND_TO_CPU 索引 = 接收方。
const TARGET_CPU: usize = 1; // C906B 大核
/// 小核→大核用 slot 0；大核→小核用 slot 1（见 trap.rs SLOT_B2S）。
///
/// `mbox_set` 是**全局**寄存器，由 `cpu_mbox_en[cpu]` 决定谁收。两个方向必须
/// 用不同 slot：否则 A 方向置的 en bit 会让 B 方向的 mbox_set 也打到自己，
/// 而且 0x400 的 context buffer 同一个 slot 会被对方覆盖。
pub const SLOT: usize = 0;

/// 写帧信息 + 触发 HW 邮箱中断通知大核。
///
/// 只写 offset 0..16（小核→大核那一半），**不要**整结构 RMW：
/// reply 字段由中断上下文的 `write_reply` 写，整结构读改写会在
/// "读 current → 写回" 之间把 ISR 刚写的 reply 覆盖掉（13 FPS 下必然丢）。
pub fn notify(frame_count: u32, yuv_size: u32, flags: u32) {
    unsafe {
        let p = MAILBOX_PA as *mut u32;
        core::ptr::write_volatile(p.add(1), frame_count);
        core::ptr::write_volatile(p.add(2), yuv_size);
        core::ptr::write_volatile(p.add(3), flags);
        // magic 最后写：大核以 magic 作为"这块内容有效"的判据
        core::ptr::write_volatile(p.add(0), MAILBOX_MAGIC);

        flush_mailbox();

        let ctx = (HW_MBOX_CONTEXT + SLOT * 8) as *mut u32;
        core::ptr::write_volatile(ctx.add(0), frame_count);
        core::ptr::write_volatile(ctx.add(1), MAILBOX_MAGIC);

        let int_clr = (HW_MBOX_BASE + 0x10 + TARGET_CPU * 16) as *mut u32;
        core::ptr::write_volatile(int_clr, 1 << SLOT);

        // 显式解除 int_mask（reset 默认应为 0；写 0 排除“被 mask 掉所以不上 PLIC”的可能）。
        let int_mask = (HW_MBOX_BASE + 0x10 + TARGET_CPU * 16 + 4) as *mut u32;
        core::ptr::write_volatile(int_mask, 0);

        let en = (HW_MBOX_BASE + TARGET_CPU * 4) as *mut u32;
        let old = core::ptr::read_volatile(en);
        core::ptr::write_volatile(en, old | (1 << SLOT));

        let mbox_set = (HW_MBOX_BASE + 0x60) as *mut u32;
        core::ptr::write_volatile(mbox_set, 1 << SLOT);
    }
}

/// 写回复（小核 ISR 收到大核消息后调用）。
///
/// 只碰 offset 16..32（大核→小核那一半），和 `notify` 写的 0..16 完全不重叠，
/// 所以中断上下文和主循环并发写也不会互相覆盖。
pub fn write_reply(msg: u32) {
    unsafe {
        let p = MAILBOX_PA as *mut u32;
        let seq = core::ptr::read_volatile(p.add(6));
        core::ptr::write_volatile(p.add(5), msg);
        core::ptr::write_volatile(p.add(6), seq.wrapping_add(1));
        // magic 最后写，作为有效标志
        core::ptr::write_volatile(p.add(4), REPLY_MAGIC);
    }
    flush_mailbox();
}

/// 初始化存活标记。
pub fn write(frame_count: u32, yuv_size: u32, flags: u32) {
    let mb = Mailbox {
        magic: MAILBOX_MAGIC,
        frame_count,
        yuv_size,
        flags,
        reply_magic: 0,
        reply_data: 0,
        reply_seq: 0,
        _pad: 0,
    };
    unsafe { core::ptr::write_volatile(MAILBOX_PA as *mut Mailbox, mb) };
    flush_mailbox();
}
