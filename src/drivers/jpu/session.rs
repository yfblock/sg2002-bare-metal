//! JPU 解码会话：decoder 单例 + 预留区 pool + wedge 自恢复。
//!
//! - DMA pool 用**预留 rtos 区**固定物理地址（`platform::JPU_POOL_PA`）,不用
//!   .bss 静态缓冲——大静态 pool 会让清 bss 踩到 U-Boot 堆(小核启动后 U-Boot
//!   还要 malloc 加载大核镜像)→ 整片复位。
//! - `new_at_no_vd_remap_with_pool`:设时钟/复位/VC + 软复位但**不设 VD_REMAP**
//!   (32 位 DMA 地址扩 40 位会超 256MB DDR;改 DDR 映射会崩大核)。
//! - **wedge 自恢复**:解码 Err 时 drop 旧 decoder 重建(重跑硬件 init + 软复位),
//!   下一帧续跑。日志直走 logger 控制台而非 log 门面——wedge 诊断需在
//!   `LevelFilter::Off` 下依然可见。

use core::sync::atomic::{AtomicU32, Ordering};

use super::JpuDecoder;
use super::SyncUnsafeCell;
use crate::logger;
use crate::platform::{JPU_POOL_PA, JPU_POOL_SIZE, YUV_BUF_PA, YUV_BUF_SIZE};

static DECODER: SyncUnsafeCell<Option<JpuDecoder>> = SyncUnsafeCell::new(None);

/// JPU 复位次数（wedge 自恢复计数），供日志节流与压力测试观测。
static RESET_COUNT: AtomicU32 = AtomicU32::new(0);

/// 创建一个新 decoder（首次调用 + wedge 恢复时用）。
fn create_decoder() -> Result<JpuDecoder, &'static str> {
    // SAFETY: 小核 identity 映射（VA=PA），pool 在预留 rtos 区（普通 DRAM，JPU DMA
    // 可达，32 位地址不需 VD_REMAP）；JPU/TOP/VC 为物理 MMIO 基址，identity 下直访。
    unsafe {
        let mut decoder = JpuDecoder::new_at_no_vd_remap_with_pool(
            JPU_POOL_PA,
            JPU_POOL_SIZE,
        )?;
        // 不在这里固定 output_buffer —— 由 set_output_slot() 每帧交替指向 slot 0/1。
        decoder.set_cpu_reads_output(false);
        Ok(decoder)
    }
}

/// 把 MJPEG 解码成 YUV422 并写入指定 slot 的共享 DRAM。
///
/// `slot` = 0/1，决定 JPU DMA 写入哪个双缓冲 slot。
/// 成功返回 `(width, height, yuv_len)`。失败时 JPU 已被复位重建，返回 `Err`；
/// 调用方应跳过本帧 YUV（只通知 MJPEG），下一帧重试。
pub fn decode_to_shared(jpeg: &[u8]) -> Result<(u32, u32, usize), &'static str> {
    let cell = unsafe { &mut *DECODER.0.get() };
    if cell.is_none() {
        match create_decoder() {
            Ok(new_decoder) => {
                logger::print("[JPU] decoder initialized\n");
                *cell = Some(new_decoder);
            }
            Err(e) => {
                logger::print("[JPU] init failed: ");
                logger::print(e);
                logger::print("\n");
                return Err(e);
            }
        }
    }

    let decoder = cell.as_mut().expect("decoder present");
    unsafe { decoder.set_output_buffer(YUV_BUF_PA, YUV_BUF_SIZE) };
    match decoder.decode(jpeg) {
        Ok(result) => {
            // 数据已由 JPU 直接 DMA 进共享缓冲，无需再搬。
            Ok((result.width, result.height, result.yuv_data.len()))
        }
        Err(e) => {
            // wedge / 解码错误：drop 旧 decoder 并重建（重跑硬件 init + 软复位）。
            let resets = RESET_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
            // 节流：第 1、每 16 次打印一次，避免冲掉大核串口。
            if resets == 1 || resets % 16 == 0 {
                logger::print_fmt(format_args!("[JPU] decode err={e} reset#{resets:#x}\n"));
            }
            *cell = None; // drop 旧 decoder（释放 stream/frame buf）
            match create_decoder() {
                Ok(new_decoder) => *cell = Some(new_decoder),
                Err(re_err) => {
                    logger::print("[JPU] re-init failed: ");
                    logger::print(re_err);
                    logger::print("\n");
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
