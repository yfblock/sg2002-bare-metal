//! b2s-comm-test —— 大小核通信测试(大核侧用户态程序,跑在 StarryOS 上)。
//!
//! 测三条通路,全部按《docs/大小核通信协议.md》:
//!   1. S2B(小核→大核):DRAM 邮箱前半区 —— magic/帧计数单调/尺寸合理
//!   2. B2S(大核→小核):write /dev/cvi-mailbox 4B → 小核 ISR 回写 reply 半区
//!      校验 reply_data == 发送值、reply_seq 随发送次数增长(往返闭环)
//!   3. YUV 零拷贝(尽力而为):mmap /dev/cvi-yuv,抽头尾字节看数据在动
//!
//! 用法:b2s-comm-test [时长秒,默认 10] [B2S 发送间隔秒,默认 2]
//! 退出码:0 = 全部通过;1 = 有失败项。
//!
//! 构建:bash ../build.sh(静态 musl,无解释器依赖)

use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::os::unix::io::AsRawFd;
use std::time::{Duration, Instant};

/// DRAM 邮箱结构 @0x9004_0000(见协议 §3;经 /dev/cvi-mailbox 暴露)
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Mailbox {
    // 0-15:小核 → 大核
    magic: u32,
    frame_count: u32,
    yuv_size: u32,
    flags: u32,
    // 16-31:大核 → 小核回复
    reply_magic: u32,
    reply_data: u32,
    reply_seq: u32,
    _pad: u32,
}

const MAGIC: u32 = 0xC906_C906;
const PANIC_MAGIC: u32 = 0xFFFF_FFFF;
const FLAG_YUV_READY: u32 = 1 << 15;
const FLAG_SOI: u32 = 1 << 0;
const FLAG_EOI: u32 = 1 << 1;
const YUV_SLOT_SIZE: usize = 0x96000; // 640×480 YUV422 = 614400

fn decode_dims(flags: u32) -> (u32, u32) {
    // bits 20-31 = width, bits 8-19 = height(协议 §3 flags 位域)
    ((flags >> 20) & 0xFFF, (flags >> 8) & 0xFFF)
}

/// 读一次邮箱结构(seek(0) + read_exact 走 read_at 语义,拿 32B 二进制而非诊断流)
fn read_mailbox(f: &mut File) -> Option<Mailbox> {
    let mut buf = [0u8; 32];
    f.seek(SeekFrom::Start(0)).ok()?;
    f.read_exact(&mut buf).ok()?;
    Some(Mailbox {
        magic: u32::from_le_bytes(buf[0..4].try_into().unwrap()),
        frame_count: u32::from_le_bytes(buf[4..8].try_into().unwrap()),
        yuv_size: u32::from_le_bytes(buf[8..12].try_into().unwrap()),
        flags: u32::from_le_bytes(buf[12..16].try_into().unwrap()),
        reply_magic: u32::from_le_bytes(buf[16..20].try_into().unwrap()),
        reply_data: u32::from_le_bytes(buf[20..24].try_into().unwrap()),
        reply_seq: u32::from_le_bytes(buf[24..28].try_into().unwrap()),
        _pad: 0,
    })
}

// ---- mmap(Linux RISC-V 通用常量,链接 musl libc) ----
const PROT_READ: i32 = 1;
const MAP_SHARED: i32 = 1;
const MAP_FAILED: isize = -1;

extern "C" {
    fn mmap(
        addr: *mut core::ffi::c_void,
        len: usize,
        prot: i32,
        flags: i32,
        fd: i32,
        offset: i64,
    ) -> *mut core::ffi::c_void;
}

/// 抽查 YUV 缓冲首尾各 4 字节,返回 (head, tail),用于零拷贝数据活性检查
fn peek_yuv(ptr: *mut u8, yuv_size: u32) -> (u32, u32) {
    let sz = (yuv_size as usize).min(YUV_SLOT_SIZE);
    unsafe {
        let head = ptr::read_u32(ptr);
        let tail_off = sz.saturating_sub(4);
        let tail = ptr::read_u32(ptr.add(tail_off));
        (head, tail)
    }
}

mod ptr {
    pub unsafe fn read_u32(p: *mut u8) -> u32 {
        let b = [p.read(), p.add(1).read(), p.add(2).read(), p.add(3).read()];
        u32::from_le_bytes(b)
    }
}

fn main() {
    let duration = std::env::args()
        .nth(1)
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(10);
    let tx_interval = std::env::args()
        .nth(2)
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(2);

    println!("== b2s-comm-test: 大小核通信测试(dual-core comm test) ==");
    println!("时长 {}s,B2S 发送间隔 {}s", duration, tx_interval);

    // ---------- 打开设备 ----------
    let mut mb_file = OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/cvi-mailbox")
        .expect("open /dev/cvi-mailbox(StarryOS 未启动或驱动未注册?)");

    let yuv_file = File::open("/dev/cvi-yuv").ok();
    if yuv_file.is_none() {
        println!("[warn] /dev/cvi-yuv 打开失败,YUV 零拷贝检查将跳过");
    }
    let yuv_map = yuv_file.as_ref().and_then(|f| {
        let p = unsafe {
            mmap(
                std::ptr::null_mut(),
                YUV_SLOT_SIZE,
                PROT_READ,
                MAP_SHARED,
                f.as_raw_fd(),
                0,
            )
        };
        if p as isize == MAP_FAILED {
            println!("[warn] mmap /dev/cvi-yuv 失败,零拷贝检查跳过");
            None
        } else {
            Some(p as *mut u8)
        }
    });

    // ---------- 等第一帧 ----------
    println!("[S2B] 等待小核第一帧(blocking poll)...");
    let first = loop {
        if let Some(mb) = read_mailbox(&mut mb_file) {
            if mb.magic == MAGIC && mb.frame_count > 0 {
                break mb;
            }
            if mb.magic == PANIC_MAGIC {
                println!("[FAIL] 小核已 panic(magic=0xFFFFFFFF)");
                std::process::exit(1);
            }
            if mb.magic != MAGIC {
                println!(
                    "[FAIL] magic={:#010x} != 期望 {:#010x}:小核未运行?",
                    mb.magic, MAGIC
                );
                std::process::exit(1);
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    };
    let (w, h) = decode_dims(first.flags);
    println!(
        "[S2B] 第一帧:count={} yuv={}B {}x{} YUV_READY={} SOI={} EOI={}",
        first.frame_count,
        first.yuv_size,
        w,
        h,
        first.flags & FLAG_YUV_READY != 0,
        first.flags & FLAG_SOI != 0,
        first.flags & FLAG_EOI != 0,
    );

    // ---------- 主测试循环 ----------
    let start = Instant::now();
    let mut last_fc = first.frame_count;
    let mut s2b_frames: u32 = 0; // frame_count 推进次数
    let mut s2b_errors: u32 = 0; // 帧计数回退/尺寸异常
    let mut b2s_sent: u32 = 0;
    let mut b2s_roundtrip_ok: u32 = 0;
    let mut b2s_roundtrip_fail: u32 = 0;
    let mut yuv_peek_changed: u32 = 0;
    let mut last_peek: Option<(u32, u32)> = None;

    while start.elapsed() < Duration::from_secs(duration) {
        // --- S2B:读邮箱,校验帧元信息 ---
        if let Some(mb) = read_mailbox(&mut mb_file) {
            if mb.magic == MAGIC {
                if mb.frame_count > last_fc {
                    s2b_frames += mb.frame_count - last_fc;
                    last_fc = mb.frame_count;
                    let (w2, h2) = decode_dims(mb.flags);
                    if mb.yuv_size == 0
                        || mb.yuv_size as usize > YUV_SLOT_SIZE
                        || w2 != w
                        || h2 != h
                    {
                        s2b_errors += 1;
                    }
                    // --- YUV 零拷贝抽查 ---
                    if let (Some(p), true) = (yuv_map, mb.flags & FLAG_YUV_READY != 0) {
                        let peek = peek_yuv(p, mb.yuv_size);
                        if last_peek != Some(peek) {
                            yuv_peek_changed += 1;
                        }
                        last_peek = Some(peek);
                    }
                } else if mb.frame_count < last_fc {
                    s2b_errors += 1; // 帧计数回退
                }
            }
        }

        // --- B2S:按间隔发送递增消息,校验 reply 半区 ---
        let elapsed = start.elapsed().as_secs();
        if elapsed > 0 && elapsed % tx_interval == 0 && (elapsed as u32 / tx_interval as u32) > b2s_sent {
            let msg = 0xA5A5_0000u32 | b2s_sent;
            match mb_file.write(&msg.to_le_bytes()) {
                Ok(4) => {
                    b2s_sent += 1;
                    // 小核 ISR 回写需要一点时间
                    std::thread::sleep(Duration::from_millis(200));
                    if let Some(mb) = read_mailbox(&mut mb_file) {
                        if mb.reply_data == msg && mb.reply_seq >= b2s_sent {
                            b2s_roundtrip_ok += 1;
                            println!(
                                "[B2S] 往返 #{:<3} 发 {:#010x} → reply_data={:#010x} seq={} ✓",
                                b2s_sent, msg, mb.reply_data, mb.reply_seq
                            );
                        } else {
                            b2s_roundtrip_fail += 1;
                            println!(
                                "[B2S] 往返 #{:<3} 发 {:#010x} → reply_data={:#010x} seq={} ✗",
                                b2s_sent, msg, mb.reply_data, mb.reply_seq
                            );
                        }
                    }
                }
                Ok(n) => {
                    println!("[B2S] 短写: {} 字节", n);
                    b2s_roundtrip_fail += 1;
                }
                Err(e) => {
                    println!("[B2S] write 失败: {}", e);
                    b2s_roundtrip_fail += 1;
                }
            }
        }

        std::thread::sleep(Duration::from_millis(50));
    }

    // ---------- 汇总 ----------
    let secs = start.elapsed().as_secs_f64();
    let fps = s2b_frames as f64 / secs;
    println!("\n== 结果汇总({:.1}s) ==", secs);
    println!(
        "S2B 帧流      : {} 帧(≈{:.1} fps),元数据异常 {} 次 {}",
        s2b_frames,
        fps,
        s2b_errors,
        if s2b_errors == 0 { "✓" } else { "✗" }
    );
    println!(
        "B2S 往返      : {}/{} 成功 {}",
        b2s_roundtrip_ok,
        b2s_sent,
        if b2s_roundtrip_fail == 0 { "✓" } else { "✗" }
    );
    if yuv_map.is_some() {
        println!(
            "YUV 零拷贝    : 内容变化 {} 次 {}",
            yuv_peek_changed,
            if yuv_peek_changed > 0 { "✓" } else { "✗(数据没动?)" }
        );
    } else {
        println!("YUV 零拷贝    : 跳过(设备不可用)");
    }

    let pass = s2b_frames > 0
        && s2b_errors == 0
        && b2s_roundtrip_fail == 0
        && b2s_roundtrip_ok > 0
        && (yuv_map.is_none() || yuv_peek_changed > 0);
    println!("总体          : {}", if pass { "PASS" } else { "FAIL" });
    std::process::exit(if pass { 0 } else { 1 });
}
