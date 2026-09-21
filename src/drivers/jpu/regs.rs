//! JPU MMIO 寄存器（`tock-registers`）与平台 bring-up 辅助函数。

use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite},
};

use core::time::Duration;

use crate::arch::time::{delay, elapsed_since, rdtime};
use crate::platform::{JPU_REG_BASE, VC_REG_BASE};

// 模块常量
/// JPU 子系统时钟使能值（TOP+0x2008;Cvitek 厂商 SDK 值,位语义无公开
/// 文档——0x3300 = 两个 2-bit 字段各 0b11,按家族惯例为时钟源选择+使能,
/// 仅整值经真机验证:不写则 JPU 不工作,写入后 16.5fps 稳定解码）。
const CLK_JPEG_ENABLE: u32 = 0x3300;

const JPU_WARMUP_BBC_BASE: u32 = 0x8026_C000;

/// JPEG 像素格式（写入 MCU/DPB 相关寄存器）
pub const FORMAT_420: u32 = 0;
pub const FORMAT_422: u32 = 1;
pub const FORMAT_224: u32 = 2;
pub const FORMAT_444: u32 = 3;
pub const FORMAT_400: u32 = 4;

/// DMA 池几何（mem.rs 页位图分配器的容量上界与页大小）。
pub const STREAM_BUF_SIZE: usize = 0x40000;
pub const JPU_DRAM_PHYSICAL_SIZE: usize = 0x0010_0000;
pub const VMEM_PAGE_SIZE: usize = 16 * 1024;

/// 霍夫曼表上传相位/地址（HUFF_CTRL.PHASE 与 HUFF_ADDR 的阶段基址;见 decoder.rs 上传序列）。
pub const HUFF_PHASE_MIN: u32 = 0x003;
pub const HUFF_PHASE_MAX: u32 = 0x403;
pub const HUFF_PHASE_PTR: u32 = 0x803;
pub const HUFF_PHASE_VAL: u32 = 0xC03;
pub const HUFF_ADDR_MAX: u32 = 0x440;
pub const HUFF_ADDR_PTR: u32 = 0x880;

pub const QMAT_PHASE_Y: u32 = 0x03;
pub const QMAT_PHASE_CB: u32 = 0x43;
pub const QMAT_PHASE_CR: u32 = 0x83;

register_bitfields![u32,
    /// VC（Video Codec）子块使能寄存器（bit0-4 = 各子块使能）。
    pub VC_ENABLE [
        BLOCKS OFFSET(0) NUMBITS(5) [],
    ],
];

register_structs! {
    /// VC 子块控制。
    pub VcRegs {
        (0x00 => pub enable: ReadWrite<u32, VC_ENABLE::Register>),
        (0x04 => @END),
    }
}

/// 使能 VC（Video Codec）子块——JPU 所属的电源/时钟门控域
/// （`platform::VC_REG_BASE`,bit0-4 各对应一个编解码子块,0x1F = 全开）。
/// 不开它 JPU 寄存器不响应。写后读回一次,保证使能落盘再继续 JPU 操作。
fn enable_vc_subblocks() {
    let vc = unsafe { &*(VC_REG_BASE as *const VcRegs) };
    vc.enable.modify(VC_ENABLE::BLOCKS.val(0x1F));
    let _ = vc.enable.get();
}

register_bitfields! [
    u32,

    pub VALUE32 [
        VAL OFFSET(0) NUMBITS(32) []
    ],

    pub MJPEG_PIC_START [
        START_PIC OFFSET(0) NUMBITS(1) [],
        START_INIT OFFSET(1) NUMBITS(1) [],
    ],

    pub MJPEG_PIC_STATUS [
        DONE OFFSET(0) NUMBITS(1) [],
        ERROR OFFSET(1) NUMBITS(1) [],
    ],

    pub MJPEG_PIC_CTRL [
        USER_HUFF_TAB OFFSET(6) NUMBITS(1) [],
        HUFF_DC_IDX OFFSET(7) NUMBITS(3) [],
        HUFF_AC_IDX OFFSET(10) NUMBITS(3) [],
    ],

    pub MJPEG_PIC_SIZE [
        HEIGHT OFFSET(0) NUMBITS(16) [],
        WIDTH OFFSET(16) NUMBITS(16) [],
    ],

    pub MJPEG_BBC_STRM_CTRL [
        PAGES OFFSET(0) NUMBITS(31) [],
        END_FLAG OFFSET(31) NUMBITS(1) [],
    ],

    pub MJPEG_BBC_BUSY [
        BUSY OFFSET(0) NUMBITS(1) [],
    ],

    pub MJPEG_HUFF_CTRL [
        PHASE OFFSET(0) NUMBITS(12) [],
    ],

    pub MJPEG_QMAT_CTRL [
        PHASE OFFSET(0) NUMBITS(8) [],
    ],
];

register_structs! {
    pub JpuRegisters {
        (0x000 => pub pic_start: ReadWrite<u32, MJPEG_PIC_START::Register>),
        (0x004 => pub pic_status: ReadWrite<u32, MJPEG_PIC_STATUS::Register>),
        (0x008 => pub pic_errmb: ReadWrite<u32, VALUE32::Register>),
        (0x00C => _reserved_pic_setmb),
        (0x010 => pub pic_ctrl: ReadWrite<u32, MJPEG_PIC_CTRL::Register>),
        (0x014 => pub pic_size: ReadWrite<u32, MJPEG_PIC_SIZE::Register>),
        (0x018 => pub mcu_info: ReadWrite<u32, VALUE32::Register>),
        (0x01C => pub rot_info: ReadWrite<u32, VALUE32::Register>),
        (0x020 => pub scl_info: ReadWrite<u32, VALUE32::Register>),
        (0x024 => _reserved_if_info),
        (0x028 => pub clp_info: ReadWrite<u32, VALUE32::Register>),
        (0x02C => pub op_info: ReadWrite<u32, VALUE32::Register>),
        (0x030 => pub dpb_config: ReadWrite<u32, VALUE32::Register>),
        (0x034 => pub dpb_base_y: ReadWrite<u32, VALUE32::Register>),
        (0x038 => pub dpb_base_cb: ReadWrite<u32, VALUE32::Register>),
        (0x03C => pub dpb_base_cr: ReadWrite<u32, VALUE32::Register>),
        (0x040 => _reserved_dpb_extra: [u8; 0x24]),
        (0x064 => pub dpb_ystride: ReadWrite<u32, VALUE32::Register>),
        (0x068 => pub dpb_cstride: ReadWrite<u32, VALUE32::Register>),
        (0x06C => _reserved_wresp: [u8; 0x14]),
        (0x080 => pub huff_ctrl: ReadWrite<u32, MJPEG_HUFF_CTRL::Register>),
        (0x084 => pub huff_addr: ReadWrite<u32, VALUE32::Register>),
        (0x088 => pub huff_data: ReadWrite<u32, VALUE32::Register>),
        (0x08C => _reserved_huff_pad),
        (0x090 => pub qmat_ctrl: ReadWrite<u32, MJPEG_QMAT_CTRL::Register>),
        (0x094 => _reserved_qmat_addr),
        (0x098 => pub qmat_data: ReadWrite<u32, VALUE32::Register>),
        (0x09C => _reserved_coef: [u8; 0x14]),
        (0x0B0 => pub rst_intval: ReadWrite<u32, VALUE32::Register>),
        (0x0B4 => pub rst_index: ReadWrite<u32, VALUE32::Register>),
        (0x0B8 => pub rst_count: ReadWrite<u32, VALUE32::Register>),
        (0x0BC => _reserved_rst_pad: [u8; 0x34]),
        (0x0F0 => pub dpcm_diff_y: ReadWrite<u32, VALUE32::Register>),
        (0x0F4 => pub dpcm_diff_cb: ReadWrite<u32, VALUE32::Register>),
        (0x0F8 => pub dpcm_diff_cr: ReadWrite<u32, VALUE32::Register>),
        (0x0FC => _reserved_dpcm_pad),
        (0x100 => pub gbu_ctrl: ReadWrite<u32, VALUE32::Register>),
        (0x104 => _reserved_gbu_mid: [u8; 0x10]),
        (0x114 => pub gbu_wd_ptr: ReadWrite<u32, VALUE32::Register>),
        (0x118 => pub gbu_tt_cnt: ReadWrite<u32, VALUE32::Register>),
        (0x11C => pub gbu_tt_cnt_h: ReadWrite<u32, VALUE32::Register>),
        (0x120 => _reserved_gbu_pbit: [u8; 0x20]),
        (0x140 => pub gbu_bbsr: ReadWrite<u32, VALUE32::Register>),
        (0x144 => pub gbu_bber: ReadWrite<u32, VALUE32::Register>),
        (0x148 => pub gbu_bbir: ReadWrite<u32, VALUE32::Register>),
        (0x14C => pub gbu_bbhr: ReadWrite<u32, VALUE32::Register>),
        (0x150 => _reserved_gbu_tail: [u8; 0x10]),
        (0x160 => pub gbu_ff_rptr: ReadWrite<u32, VALUE32::Register>),
        (0x164 => _reserved_bbc_gap: [u8; 0xA4]),
        (0x208 => pub bbc_end_addr: ReadWrite<u32, VALUE32::Register>),
        (0x20C => pub bbc_wr_ptr: ReadWrite<u32, VALUE32::Register>),
        (0x210 => pub bbc_rd_ptr: ReadWrite<u32, VALUE32::Register>),
        (0x214 => pub bbc_ext_addr: ReadWrite<u32, VALUE32::Register>),
        (0x218 => pub bbc_int_addr: ReadWrite<u32, VALUE32::Register>),
        (0x21C => pub bbc_data_cnt: ReadWrite<u32, VALUE32::Register>),
        (0x220 => pub bbc_command: ReadWrite<u32, VALUE32::Register>),
        (0x224 => pub bbc_busy: ReadOnly<u32, MJPEG_BBC_BUSY::Register>),
        (0x228 => pub bbc_ctrl: ReadWrite<u32, VALUE32::Register>),
        (0x22C => pub bbc_cur_pos: ReadWrite<u32, VALUE32::Register>),
        (0x230 => pub bbc_bas_addr: ReadWrite<u32, VALUE32::Register>),
        (0x234 => pub bbc_strm_ctrl: ReadWrite<u32, MJPEG_BBC_STRM_CTRL::Register>),
        (0x238 => @END),
    }
}

/// 取 JPU 寄存器视图（基址为编译期常量 [`crate::platform::JPU_REG_BASE`]，
/// 小核 identity 映射下恒有效，无需参数）。
#[inline]
pub fn jpu_regs() -> &'static JpuRegisters {
    unsafe { &*(JPU_REG_BASE as *const JpuRegisters) }
}

/// 设时钟/复位/VC，并完成 JPU 软复位；但**不设 VD_REMAP**。
/// 适用于小核（C906L）：VD_REMAP 是 8-bit 字段（bit24-31，表示 addr\[39:32\]），
/// 设为 1 会让 32 位 DMA 地址变成 (1<<32)|addr，超出 256MB DDR 范围。
pub fn hardware_init() {
    let top = crate::platform::top();
    // JPEG 时钟使能 + 复位释放(TOP 寄存器,soc.rs TopRegs 视图)
    top.clk_jpeg.set(top.clk_jpeg.get() | CLK_JPEG_ENABLE); // RMW: 只置时钟位
    top.rst.modify(crate::platform::TOP_RST::JPEG::SET);
    enable_vc_subblocks();

    let regs = jpu_regs();
    let _ = regs.pic_status.get();
    regs.bbc_bas_addr
        .write(VALUE32::VAL.val(JPU_WARMUP_BBC_BASE));
    let _ = regs.bbc_bas_addr.get();

    wait_sw_reset_done();
}

/// 给 JPEG 块打一次**真正的复位脉冲**，然后重新初始化（不含 VD_REMAP / DDR mode）。
///
/// 为什么需要它：常规 init 只做 `v | TOP_RST_JPEG_RELEASE_BIT`，
/// 也就是只"释放"复位——对一个已经在跑（或已经挂死）的块是**空操作**，
/// 从来没有把复位真正拉低过。所以 JPU 挂死后，无论重跑 `hardware_init_*`
/// 还是重建整个 `JpuDecoder`，硬件状态都不会被清掉（实测挂死后连续 280+ 帧
/// 全部超时失败）。必须 assert → 等待 → deassert 才能把块拉回干净状态。
///
/// 只动 JPEG 的时钟/复位位和 VC 使能，**不写** `TOP_DDR_ADDR_MODE_OFF`
/// （那会改 DDR 地址映射把大核搞崩）。
pub fn hard_reset() {
    let top = crate::platform::top();
    // 1) 拉低复位(清 JPEG release 位)
    top.rst.modify(crate::platform::TOP_RST::JPEG::CLEAR);
    delay(Duration::from_millis(1));
    // 2) 时钟使能后再释放复位
    top.clk_jpeg.set(CLK_JPEG_ENABLE);
    top.rst.modify(crate::platform::TOP_RST::JPEG::SET);
    delay(Duration::from_millis(1));
    // 3) VC 子块重新使能
    enable_vc_subblocks();
}

/// 等待软复位完成。按时间设上限（10ms 足够）——次数上限的实际时长取决于主频
/// 和循环开销，在小核上会长到不可接受，见 [`crate::arch::time`]。
fn wait_sw_reset_done() {
    let regs = jpu_regs();
    regs.pic_start.write(MJPEG_PIC_START::START_INIT::SET);
    let t0 = rdtime();
    let timeout = Duration::from_millis(10);
    while elapsed_since(t0) < timeout {
        if !regs.pic_start.is_set(MJPEG_PIC_START::START_INIT) {
            return;
        }
        core::hint::spin_loop();
    }
}

/// 等待 BBC 空闲。同上，按时间设上限。
pub fn wait_bbc_idle() {
    let regs = jpu_regs();
    let t0 = rdtime();
    let timeout = Duration::from_millis(10);
    while elapsed_since(t0) < timeout {
        if !regs.bbc_busy.is_set(MJPEG_BBC_BUSY::BUSY) {
            return;
        }
        core::hint::spin_loop();
    }
}
