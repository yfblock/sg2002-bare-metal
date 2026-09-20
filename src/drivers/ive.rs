//! IVE (Image Video Engine) 硬件 CSC（YUV→RGB）驱动，`tock-registers` 封装。
//!
//! SG2002 的 IVE 在 0x0A0A0000，有专用 CSC 硬件，支持内存到内存转换。
//! 小核可访问（已验证）。CSC 在 FILTEROP 块里实现，通过直接写寄存器启动。
//!
//! 参考：osdrv/interdrv/v2/ive/hal/mars/cvi_ive_platform.c 的 _cvi_ive_csc()。
//!
//! 寄存器块偏移（相对 IVE base 0x0A0A0000）：
//!   IVE_TOP    @ +0x0000
//!   IMG_IN     @ +0x0400
//!   FILTEROP   @ +0x2000（含 ODMA @ +0x120 与 CSC 系数 @ +0x198）
//!
//! 状态：**输出未验证**（`fmt_sel` 等编码逆向自 osdrv，无数据手册），
//! 硬件链路本身可跑通不挂死；保留供大核比对工具继续调。

use core::sync::atomic::{AtomicU32, Ordering};
use core::time::Duration;
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::registers::{ReadOnly, ReadWrite};
use tock_registers::{register_bitfields, register_structs};

use crate::arch::time::{elapsed_since, rdtime};

/// IVE 寄存器基址。
use crate::platform::IVE_BASE;

register_bitfields![u32,
    /// TOP +0x04：软复位 / 启动。
    pub TopCtrl1 [
        SOFTRST OFFSET(0) NUMBITS(1) [],
        /// 写 1 启动一次转换（frame valid）。
        FMT_VLD_FG OFFSET(4) NUMBITS(1) [],
    ],
    /// TOP +0x08：图像尺寸（-1 编码）。
    pub TopSize [
        W_M1 OFFSET(0) NUMBITS(13) [],
        H_M1 OFFSET(16) NUMBITS(13) [],
    ],
    /// TOP +0x10：top 使能。
    pub TopEnable [
        IMG_IN OFFSET(0) NUMBITS(1) [],
        CSC OFFSET(3) NUMBITS(1) [],
        FILTEROP OFFSET(15) NUMBITS(1) [],
    ],
    /// IMG_IN +0x00：输入配置。
    pub ImgInCtrl [
        SRC_SEL OFFSET(0) NUMBITS(2) [DRAM = 2],
        /// 输入格式（逆向枚举，无数据手册；可经 [`set_input_fmt`] 运行时覆盖扫描）。
        FMT_SEL OFFSET(4) NUMBITS(4) [YUV420P = 0],
        BURST OFFSET(8) NUMBITS(4) [],
        /// bit16：逆向 osdrv 时的未名字段，原样置 1（含义待考）。
        UNDOC16 OFFSET(16) NUMBITS(1) [],
    ],
    /// IMG_IN +0x14：shadow 寄存器更新方式。
    pub ImgUpd [
        /// 1 = 立即更新（不等 vsync）。
        SHRD_SEL OFFSET(2) NUMBITS(1) [],
    ],
    /// IMG_IN +0x68：IP 清理/状态。
    pub ImgInIp [
        IP_CLR_W1T OFFSET(18) NUMBITS(1) [],
    ],
    /// FILTEROP +0x10：工作模式。
    pub FopMode [
        /// 1 = FILTER3CH（CSC 走这里）。
        MODE OFFSET(0) NUMBITS(4) [CSC = 1],
    ],
    /// FILTEROP +0x194：系数更新。
    pub CoefUpd [
        /// 1 = 使用软件写入的 CSC 系数。
        COEFF_SW_UPDATE OFFSET(16) NUMBITS(1) [],
    ],
    /// FILTEROP +0x1C8：CSC 使能。
    pub CscCtrl [
        /// 系数表模式（0 = BT601 limited YUV2RGB）。
        ENMODE OFFSET(0) NUMBITS(4) [],
        ENABLE OFFSET(4) NUMBITS(1) [],
    ],
    /// FILTEROP ODMA +0x120：输出 DMA 配置。
    pub OdmaCtrl [
        DMA_BLEN OFFSET(0) NUMBITS(1) [],
        /// 输出格式（逆向枚举）。
        FMT_SEL OFFSET(8) NUMBITS(4) [RGB888_PLANAR = 2],
        DMA_EN OFFSET(12) NUMBITS(1) [],
    ],
];

register_structs! {
    /// IVE 完整寄存器映射（仅声明本驱动用到的字段，其余区段留作 reserved）。
    pub IveRegs {
        (0x000 => _reserved000),
        /* ---- IVE_TOP ---- */
        (0x004 => pub top_ctrl1: ReadWrite<u32, TopCtrl1::Register>),
        (0x008 => pub top_size: ReadWrite<u32, TopSize::Register>),
        (0x00c => _reserved00c),
        (0x010 => pub top_enable: ReadWrite<u32, TopEnable::Register>),
        (0x014 => _reserved014: [u32; 31]),
        /// frame done 状态（任意 done bit 置位 = 完成）。
        (0x090 => pub top_status: ReadOnly<u32>),
        (0x094 => pub top_int_en: ReadWrite<u32>),
        (0x098 => pub top_int_st: ReadWrite<u32>),
        (0x09c => _reserved09c: [u32; 217]),
        /* ---- IMG_IN ---- */
        (0x400 => pub img_in_ctrl: ReadWrite<u32, ImgInCtrl::Register>),
        (0x404 => _reserved404),
        (0x408 => pub img_in_size: ReadWrite<u32, TopSize::Register>),
        (0x40c => pub y_pitch: ReadWrite<u32>),
        (0x410 => pub c_pitch: ReadWrite<u32>),
        (0x414 => pub img_upd: ReadWrite<u32, ImgUpd::Register>),
        (0x418 => _reserved418: [u32; 3]),
        (0x424 => pub y_base_lo: ReadWrite<u32>),
        (0x428 => pub y_base_hi: ReadWrite<u32>),
        (0x42c => pub u_base_lo: ReadWrite<u32>),
        (0x430 => pub u_base_hi: ReadWrite<u32>),
        (0x434 => pub v_base_lo: ReadWrite<u32>),
        (0x438 => pub v_base_hi: ReadWrite<u32>),
        (0x43c => _reserved43c: [u32; 11]),
        (0x468 => pub img_in_ip: ReadWrite<u32, ImgInIp::Register>),
        (0x46c => _reserved46c: [u32; 1769]),
        /* ---- FILTEROP ---- */
        (0x2010 => pub fop_mode: ReadWrite<u32, FopMode::Register>),
        /// 3ch_en / op_y_wdma_en（本驱动全清零）。
        (0x2014 => pub fop_en: ReadWrite<u32>),
        (0x2018 => _reserved2018: [u32; 66]),
        /* ---- FILTEROP ODMA（输出 DMA）---- */
        (0x2120 => pub odma_ctrl: ReadWrite<u32, OdmaCtrl::Register>),
        (0x2124 => pub r_base_lo: ReadWrite<u32>),
        (0x2128 => pub r_base_hi: ReadWrite<u32>),
        (0x212c => pub g_base_lo: ReadWrite<u32>),
        (0x2130 => pub g_base_hi: ReadWrite<u32>),
        (0x2134 => pub b_base_lo: ReadWrite<u32>),
        (0x2138 => pub b_base_hi: ReadWrite<u32>),
        (0x213c => pub out_y_pitch: ReadWrite<u32>),
        (0x2140 => pub out_c_pitch: ReadWrite<u32>),
        (0x2144 => _reserved2144),
        (0x2148 => pub out_w_m1: ReadWrite<u32, TopSize::Register>),
        (0x214c => pub out_h_m1: ReadWrite<u32, TopSize::Register>),
        (0x2150 => _reserved2150: [u32; 17]),
        /* ---- FILTEROP CSC ---- */
        (0x2194 => pub coef_upd: ReadWrite<u32, CoefUpd::Register>),
        /// 12 个 19-bit 系数（{c00,c01,c02,off0, c10.., c20..}）。
        (0x2198 => pub csc_c00: ReadWrite<u32>),
        (0x219c => pub csc_c01: ReadWrite<u32>),
        (0x21a0 => pub csc_c02: ReadWrite<u32>),
        (0x21a4 => pub csc_off0: ReadWrite<u32>),
        (0x21a8 => pub csc_c10: ReadWrite<u32>),
        (0x21ac => pub csc_c11: ReadWrite<u32>),
        (0x21b0 => pub csc_c12: ReadWrite<u32>),
        (0x21b4 => pub csc_off1: ReadWrite<u32>),
        (0x21b8 => pub csc_c20: ReadWrite<u32>),
        (0x21bc => pub csc_c21: ReadWrite<u32>),
        (0x21c0 => pub csc_c22: ReadWrite<u32>),
        (0x21c4 => pub csc_off2: ReadWrite<u32>),
        (0x21c8 => pub csc_ctrl: ReadWrite<u32, CscCtrl::Register>),
        (0x21cc => @END),
    }
}

/// 取 IVE 寄存器视图（基址为编译期常量，恒有效）。
#[inline]
fn ive_regs() -> &'static IveRegs {
    unsafe { &*(IVE_BASE as *const IveRegs) }
}

/// BT.601 limited range YUV→RGB 系数（Video BT601 YUV2RGB, mode 0）。
/// 布局：{c00,c01,c02,off0, c10,c11,c12,off1, c20,c21,c22,off2}
/// 来自 cvi_ive_platform.c coef_BT601_to_GBR_16_235。
const CSC_COEF_BT601_LIMIT: [u32; 12] = [
    1024, 0, 1404, 179188, 1024, 344, 715, 136040, 1024, 1774, 0, 226505,
];

/// IMG_IN 的 `fmt_sel` 覆盖值。
///
/// 硬件格式枚举没有可用的数据手册，`FMT_YUV420P = 0` 是逆向 osdrv 得到的，
/// 摄像头实际输出是 YUV422，对应的编码未知。做成运行时可写，
/// 配合大核的比对工具扫描候选值——哪个值让 IVE 输出与软件 CSC 参考一致，
/// 哪个就是对的。`u32::MAX` 表示"用默认值"。
static FMT_SEL_OVERRIDE: AtomicU32 = AtomicU32::new(u32::MAX);

/// 设置 IMG_IN `fmt_sel` 覆盖值（`u32::MAX` = 恢复默认）。
pub fn set_input_fmt(fmt: u32) {
    FMT_SEL_OVERRIDE.store(fmt, Ordering::Relaxed);
}

/// 读当前生效的 `fmt_sel`（模块内供 `csc_yuv420_to_rgb888` 使用）。
fn input_fmt() -> u32 {
    match FMT_SEL_OVERRIDE.load(Ordering::Relaxed) {
        u32::MAX => ImgInCtrl::FMT_SEL::YUV420P.value,
        v => v & 0xF,
    }
}

#[inline]
fn write_base(lo: &ReadWrite<u32>, hi: &ReadWrite<u32>, pa: u64) {
    lo.set(pa as u32);
    hi.set((pa >> 32) as u32);
}

/// 执行一次 YUV420→RGB888 CSC 转换（阻塞直到完成或超时）。
///
/// # 参数
/// - `y_pa`, `u_pa`, `v_pa`: 输入 YUV420 planar 的 Y/U/V 物理地址
/// - `y_pitch`, `c_pitch`: 输入 Y/C 的 stride（字节）
/// - `r_pa`, `g_pa`, `b_pa`: 输出 RGB888 planar 的 R/G/B 物理地址
/// - `r_pitch`: 输出 RGB 的 stride（字节，RGB planar 的 G/B pitch 相同）
/// - `width`, `height`: 图像尺寸
pub fn csc_yuv420_to_rgb888(
    y_pa: usize, u_pa: usize, v_pa: usize,
    y_pitch: u32, c_pitch: u32,
    r_pa: usize, g_pa: usize, b_pa: usize,
    r_pitch: u32,
    width: u32, height: u32,
) -> Result<(), &'static str> {
    let reg = ive_regs();
    let wm1 = width.saturating_sub(1);
    let hm1 = height.saturating_sub(1);

    // 1. 软复位 IVE
    reg.top_ctrl1.write(TopCtrl1::SOFTRST::SET);
    crate::arch::time::delay(Duration::from_micros(10));
    reg.top_ctrl1.write(TopCtrl1::SOFTRST::CLEAR);

    // 2. 图像尺寸
    reg.top_size.write(TopSize::W_M1.val(wm1) + TopSize::H_M1.val(hm1));

    // 3. FILTEROP 配置为 CSC，清 3ch/WDMA 使能
    reg.fop_mode.write(FopMode::MODE::CSC);
    reg.fop_en.set(0);

    // 4. CSC 系数（12 个 19-bit，软件系数）
    let coefs: [&ReadWrite<u32>; 12] = [
        &reg.csc_c00, &reg.csc_c01, &reg.csc_c02, &reg.csc_off0,
        &reg.csc_c10, &reg.csc_c11, &reg.csc_c12, &reg.csc_off1,
        &reg.csc_c20, &reg.csc_c21, &reg.csc_c22, &reg.csc_off2,
    ];
    for (reg, &c) in coefs.iter().zip(CSC_COEF_BT601_LIMIT.iter()) {
        reg.set(c & 0x7FFFF);
    }
    reg.coef_upd.write(CoefUpd::COEFF_SW_UPDATE::SET);

    // 5. CSC 使能 + 模式（mode 0 = BT601 limited YUV2RGB）
    reg.csc_ctrl.write(CscCtrl::ENABLE::SET + CscCtrl::ENMODE.val(0));

    // 6. 清 IMG_IN IP
    reg.img_in_ip.write(ImgInIp::IP_CLR_W1T::SET);
    crate::arch::time::delay(Duration::from_micros(10));
    reg.img_in_ip.set(0);

    // 7. 输入图像配置
    reg.y_pitch.set(y_pitch);
    reg.c_pitch.set(c_pitch);
    reg.img_in_size.write(TopSize::W_M1.val(wm1) + TopSize::H_M1.val(hm1));
    reg.img_in_ctrl.write(
        ImgInCtrl::SRC_SEL::DRAM
            + ImgInCtrl::FMT_SEL.val(input_fmt())
            + ImgInCtrl::BURST.val(8)
            + ImgInCtrl::UNDOC16::SET,
    );
    write_base(&reg.y_base_lo, &reg.y_base_hi, y_pa as u64);
    write_base(&reg.u_base_lo, &reg.u_base_hi, u_pa as u64);
    write_base(&reg.v_base_lo, &reg.v_base_hi, v_pa as u64);
    reg.img_upd.write(ImgUpd::SHRD_SEL::SET);

    // 8. 使能 FILTEROP top
    reg.top_enable.write(TopEnable::IMG_IN::SET + TopEnable::CSC::SET + TopEnable::FILTEROP::SET);

    // 9. 输出 DMA 配置
    reg.out_y_pitch.set(r_pitch);
    reg.out_c_pitch.set(r_pitch);
    reg.out_w_m1.write(TopSize::W_M1.val(wm1));
    reg.out_h_m1.write(TopSize::H_M1.val(hm1));
    write_base(&reg.r_base_lo, &reg.r_base_hi, r_pa as u64);
    write_base(&reg.g_base_lo, &reg.g_base_hi, g_pa as u64);
    write_base(&reg.b_base_lo, &reg.b_base_hi, b_pa as u64);
    reg.odma_ctrl.write(
        OdmaCtrl::FMT_SEL::RGB888_PLANAR + OdmaCtrl::DMA_BLEN.val(1) + OdmaCtrl::DMA_EN::SET,
    );

    // 10. 清中断状态/使能（不用中断），fmt_vld_fg = 1 启动。
    // （旧代码还向 RO 的 top_status 写 0，无操作意义，已略。）
    reg.top_int_st.set(0);
    reg.top_int_en.set(0);
    reg.top_ctrl1.write(TopCtrl1::FMT_VLD_FG::SET);

    // 11. 轮询完成（top_status 任意 done bit 置位即完成；上限 1s）
    let t0 = rdtime();
    let timeout = Duration::from_millis(1_000);
    loop {
        if reg.top_status.get() != 0 {
            return Ok(());
        }
        if elapsed_since(t0) >= timeout {
            return Err("IVE CSC timeout");
        }
    }
}
