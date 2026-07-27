//! 小核 JPU 解码封装：把抓到的 MJPEG 经 sg200x-bsp 的 `JpuDecoder` 解码成
//! YUV420，写入共享 DRAM 的指定 slot。
//!
//! 关键点：
//! - DMA pool 放在**预留 rtos 区**固定物理地址 `0x8FF00000`（896 KiB），**不**用
//!   .bss 静态缓冲——1 MiB 静态 pool 会让小核 .bss 膨胀到 ~2.4 MiB，清 bss 时会
//!   踩到 U-Boot 堆（U-Boot 启动小核后还要 malloc 加载 starryos.uimg）→ U-Boot
//!   崩溃复位整片 SoC。预留区是大核 dtb 不碰的 DRAM，小核 identity 映射可直接用。
//! - 用 `new_at_no_vd_remap_with_pool`：设 JPU 时钟/复位/VC + 软复位，但**不设
//!   VD_REMAP**——VD_REMAP 会把 32 位 DMA 地址扩成 40 位，超出 256MB DDR 范围。
//!   且不写 `TOP_DDR_ADDR_MODE_OFF`（那会改大核 DDR 映射导致大核崩溃）。
//! - **wedge 自恢复**：已知 JPU 连续解码 ~94 帧后超时 wedged 且超时后不复位。
//!   解码返回 Err 时，drop 旧 decoder（释放 stream/frame buf）并重建——重建会重跑
//!   `hardware_init_at_no_vd_remap`（含软复位 `wait_sw_reset_done_at`），下一帧可续跑。
//!
//! 预留区 [0x8FE00000, 0x90000000)（2MB）布局：
//!   YUV slot0 [0x8FE00000, 0x8FE80000) 512K
//!   YUV slot1 [0x8FE80000, 0x8FF00000) 512K
//!   JPU pool  [0x8FF00000, 0x8FFE0000) 896K
//!   (gap)     [0x8FFE0000, 0x8FFFE000) 64K
//!   mailbox   [0x8FFFE000, 0x8FFFE020) 32B

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering};

use sg200x_bsp::jpu::{JpuDecoder, regs::{JPU_REG_BASE, VC_REG_BASE}};
use sg200x_bsp::soc::TOP_BASE;

use crate::uart;
use crate::yuv_buf;

/// JPU DMA 内存池物理地址（rtos_region 内，YUV 缓冲之后，mailbox 之前）。
const JPU_POOL_PA: usize = 0x8FF1_E000;
/// JPU DMA 内存池大小。stream_buf(256K) + frame_buf(640×480 YUV422≈614K) ≈ 864K，
/// 896K (0xE0000) 留 2 页余量；[0x8FF1E000, 0x8FFFE000)，mailbox 紧随其后。
const JPU_POOL_SIZE: usize = 0x000E_0000;

/// 裸机单核：JPU 仅在主循环访问，邮箱 ISR 不触碰 JPU。用 `UnsafeCell` 持有单例。
struct SyncUnsafeCell<T>(UnsafeCell<T>);
unsafe impl<T> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    const fn new(value: T) -> Self {
        Self(UnsafeCell::new(value))
    }
}

static DECODER: SyncUnsafeCell<Option<JpuDecoder>> = SyncUnsafeCell::new(None);

/// JPU 复位次数（wedge 自恢复计数），供日志节流与压力测试观测。
static RESET_COUNT: AtomicU32 = AtomicU32::new(0);

fn identity_dma(v: usize) -> usize {
    v
}

/// 创建一个新 decoder（首次调用 + wedge 恢复时用）。
fn create_decoder() -> Result<JpuDecoder, &'static str> {
    // SAFETY: 小核 identity 映射（VA=PA），pool 在预留 rtos 区（普通 DRAM，JPU DMA
    // 可达，32 位地址不需 VD_REMAP）；JPU/TOP/VC 为物理 MMIO 基址，identity 下直访。
    unsafe {
        JpuDecoder::new_at_no_vd_remap_with_pool(
            JPU_REG_BASE,
            TOP_BASE,
            VC_REG_BASE,
            identity_dma,
            JPU_POOL_PA,
            JPU_POOL_SIZE,
        )
    }
}

/// 把 MJPEG 解码成 YUV420 并写入共享 DRAM（单缓冲 0x8FE90000）。
///
/// 成功返回 `(width, height, yuv_len)`。失败时 JPU 已被复位重建，返回 `Err`；
/// 调用方应跳过本帧 YUV（只通知 MJPEG），下一帧重试。
pub fn decode_to_shared(jpeg: &[u8]) -> Result<(u32, u32, usize), &'static str> {
    let cell = unsafe { &mut *DECODER.0.get() };
    if cell.is_none() {
        match create_decoder() {
            Ok(d) => {
                uart::print("[JPU] decoder initialized\n");
                *cell = Some(d);
            }
            Err(e) => {
                uart::print("[JPU] init failed: ");
                uart::print(e);
                uart::print("\n");
                return Err(e);
            }
        }
    }

    let decoder = cell.as_mut().expect("decoder present");
    match decoder.decode(jpeg) {
        Ok(result) => {
            let len = result.yuv_data.len();
            yuv_buf::write_yuv(result.yuv_data);
            Ok((result.width, result.height, len))
        }
        Err(e) => {
            // wedge / 解码错误：drop 旧 decoder 并重建（重跑硬件 init + 软复位）。
            let n = RESET_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
            // 节流：第 1、每 16 次打印一次，避免冲掉大核串口。
            if n == 1 || n % 16 == 0 {
                uart::print("[JPU] decode err=");
                uart::print(e);
                uart::print(" reset#");
                uart::print_hex(n as u64);
                uart::print("\n");
            }
            *cell = None; // drop 旧 decoder（释放 stream/frame buf）
            match create_decoder() {
                Ok(d) => *cell = Some(d),
                Err(re_err) => {
                    uart::print("[JPU] re-init failed: ");
                    uart::print(re_err);
                    uart::print("\n");
                }
            }
            Err(e)
        }
    }
}

/// 读取累计复位次数（供观测）。
pub fn reset_count() -> u32 {
    RESET_COUNT.load(Ordering::Relaxed)
}
