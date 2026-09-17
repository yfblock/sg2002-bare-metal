//! 架构层：RISC-V RV64 M-mode（C906L）相关的全部底层原语。
//!
//! 业务模块（ipc / drivers / platform / stats / task）只经 `arch::` 使用架构能力：
//! - [`asm.S`](self) 的引入（`_start` 启动块 + `_trap_entry` 上下文块，见下方
//!   `global_asm!`）。
//! - [`trap`]：M-mode trap 分发 + `mtvec`/`mie`/`mstatus` 初始化。
//! - [`plic`]：RISC-V PLIC 中断控制器（source 61 邮箱 / 30 USB）。
//! - [`cache`]：D-cache 维护（C906 自定义 `dcache.cva/iva/ciall` 指令；DMA 一致性契约）。
//! - [`time`]：`rdtime` 硬件定时器（25 MHz timebase）、`delay(Duration)`、`elapsed_since`。
//!
//! fence / `wfi` 等通用指令直接用 `riscv::asm`（crate 封装），不设本地模块。
//!
//! 注意：C906 自定义缓存指令与标准 `zicbom` 编码冲突（见 [`cache`]），不可同时启用。

// 全局汇编(asm.S: _start 启动块 + _trap_entry trap 上下文块)。
// 被 call 的 rust_main / rust_trap_handler 均为 #[no_mangle] extern "C",按名引用。
core::arch::global_asm!(include_str!("asm.S"));

pub mod cache;
pub mod plic;
pub mod time;
pub mod trap;
