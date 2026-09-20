//! DWC2 寄存器布局（Synopsys DesignWare OTG 2.0），使用 `tock-registers` 提供
//! 类型化 MMIO 访问。布局对齐 Linux `drivers/usb/dwc2/hw.h`。

use tock_registers::{
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite},
};

/// GHWCFG2.NUM_HOST_CHAN 字段理论上限;本驱动仅用通道 0/1。
pub const DWC2_MAX_HOST_CHANNELS: usize = 16;

register_bitfields![u32,
    /// GSNPSID（Synopsys 产品识别）:高 16 位产品魔数（0x4f54 = "OTG"）,
    /// 低 16 位核心版本号（Linux 以版本号分界软复位序列/GDFIFOCFG 配置）。
    pub GSNPSID [
        PRODUCT_ID OFFSET(16) NUMBITS(16) [],
        VERSION OFFSET(0) NUMBITS(16) [],
    ],

    /// AHB 总线配置（DMA、突发长度、全局中断使能）。
    pub GAHBCFG [
        GLBL_INTR_EN OFFSET(0) NUMBITS(1) [],
        HBSTLEN OFFSET(1) NUMBITS(4) [
            Incr16 = 7,
        ],
        DMA_EN OFFSET(5) NUMBITS(1) [],
    ],
    /// USB 接口配置（TOUTCAL、UTMI 宽度、Force Host/Device）。
    pub GUSBCFG [
        TOUTCAL OFFSET(0) NUMBITS(3) [],
        PHYIF16 OFFSET(3) NUMBITS(1) [],
        ULPI_UTMI_SEL OFFSET(4) NUMBITS(1) [],
        FORCEHOSTMODE OFFSET(29) NUMBITS(1) [],
    ],
    /// 复位与 FIFO flush。
    pub GRSTCTL [
        CSFTRST OFFSET(0) NUMBITS(1) [],
        RXFFLSH OFFSET(4) NUMBITS(1) [],
        TXFFLSH OFFSET(5) NUMBITS(1) [],
        TXFNUM OFFSET(6) NUMBITS(5) [],
        CSFTRST_DONE OFFSET(29) NUMBITS(1) [],
        AHBIDLE OFFSET(31) NUMBITS(1) [],
    ],
    /// 全局中断状态（W1C 位：USBSUSP/USBRST/ENUMDONE/ISOOUTDROP/EOPF/RSTDET/WKUPINT 等）。
    pub GINTSTS [
        CURMODE_HOST OFFSET(0) NUMBITS(1) [],
        HCHINT OFFSET(25) NUMBITS(1) [],
    ],
    /// 全局中断掩码（位定义与 [`GINTSTS`] 一一对应）。
    pub GINTMSK [
        HCHINT OFFSET(25) NUMBITS(1) [],
    ],
    /// OTG 控制寄存器（`dr_mode=otg` 时主机会话 override 用）。
    pub GOTGCTL [
        VBVALOEN OFFSET(2) NUMBITS(1) [],
        VBVALOVAL OFFSET(3) NUMBITS(1) [],
        AVALOEN OFFSET(4) NUMBITS(1) [],
        AVALOVAL OFFSET(5) NUMBITS(1) [],
        DBNCE_FLTR_BYPASS OFFSET(15) NUMBITS(1) [],
    ],
    /// 硬件配置 2（包含 ARCH、Host channel 数）。
    pub GHWCFG2 [
        ARCH OFFSET(3) NUMBITS(2) [],
        NUM_HOST_CHAN OFFSET(14) NUMBITS(4) [],
    ],
    /// 硬件配置 3（DFIFO 总深度）。
    pub GHWCFG3 [
        DFIFO_DEPTH OFFSET(16) NUMBITS(16) [],
    ],
    /// 硬件配置 4（专用 FIFO 标志、UTMI PHY 数据宽度）。
    pub GHWCFG4 [
        DED_FIFO_EN OFFSET(25) NUMBITS(1) [],
        UTMI_PHY_DATA_WIDTH OFFSET(14) NUMBITS(2) [],
    ],
    /// 动态 FIFO 配置（EP info base）。
    pub GDFIFOCFG [
        EPINFOBASE OFFSET(16) NUMBITS(16) [],
    ],
    /// RX FIFO 深度。
    pub GRXFSIZ [
        RXFDEP OFFSET(0) NUMBITS(16) [],
    ],
    /// 非 periodic TX FIFO 起始地址 + 深度。
    pub GNPTXFSIZ [
        NPTXFSTADDR OFFSET(0) NUMBITS(16) [],
        NPTXFDEP OFFSET(16) NUMBITS(16) [],
    ],
    /// Periodic TX FIFO 起始地址 + 深度。
    pub HPTXFSIZ [
        PTXFSTADDR OFFSET(0) NUMBITS(16) [],
        PTXFDEP OFFSET(16) NUMBITS(16) [],
    ],
    /// 主机配置（FS/LS、PHY 时钟）。
    pub HCFG [
        FSLSPCLKSEL OFFSET(0) NUMBITS(2) [],
        FSLSSUPP OFFSET(2) NUMBITS(1) [],
    ],
    /// 主机端口控制状态（HPRT0）。
    ///
    /// **写回陷阱**：以下位是 R/W1C——读到的值 1 表示"当前生效/事件 pending"，
    /// 但**写 1 的含义是"清除/禁用"**。RMW 时必须先屏蔽，否则会把读到的
    /// ENA=1 写回去 = 禁用端口。掩码见 [`HPRT0_W1C_MASK`]。
    ///   bit1 CONNDET / bit2 ENA（写 1=disable!）/ bit3 ENACHG / bit5 OVRCURCHG
    pub HPRT0 [
        CONNSTS OFFSET(0) NUMBITS(1) [],
        CONNDET OFFSET(1) NUMBITS(1) [],
        ENA OFFSET(2) NUMBITS(1) [],
        ENACHG OFFSET(3) NUMBITS(1) [],
        OVRCURCHG OFFSET(5) NUMBITS(1) [],
        RST OFFSET(8) NUMBITS(1) [],
        PWR OFFSET(12) NUMBITS(1) [],
    ],
    /// 主机帧编号（HFNUM）。
    pub HFNUM [
        FRNUM OFFSET(0) NUMBITS(16) [],
    ],
    /// 主机通道字符（HCCHAR）：MPS、EP、方向、类型、设备地址、奇偶帧、CHENA/CHDIS。
    pub HCCHAR [
        MPS OFFSET(0) NUMBITS(11) [],
        EPNUM OFFSET(11) NUMBITS(4) [],
        EPDIR OFFSET(15) NUMBITS(1) [],
        EPTYPE OFFSET(18) NUMBITS(2) [
            Control = 0,
            Isochronous = 1,
        ],
        MC OFFSET(20) NUMBITS(2) [],
        DEVADDR OFFSET(22) NUMBITS(7) [],
        ODDFRM OFFSET(29) NUMBITS(1) [],
        CHDIS OFFSET(30) NUMBITS(1) [],
        CHENA OFFSET(31) NUMBITS(1) [],
    ],
    /// 主机通道中断（HCINT）。
    pub HCINT [
        XFERCOMPL OFFSET(0) NUMBITS(1) [],
        CHHLTD OFFSET(1) NUMBITS(1) [],
        AHBERR OFFSET(2) NUMBITS(1) [],
        STALL OFFSET(3) NUMBITS(1) [],
        NAK OFFSET(4) NUMBITS(1) [],
        NYET OFFSET(6) NUMBITS(1) [],
        XACTERR OFFSET(7) NUMBITS(1) [],
        BBLERR OFFSET(8) NUMBITS(1) [],
        FRMOVRN OFFSET(9) NUMBITS(1) [],
        DATATGLERR OFFSET(10) NUMBITS(1) [],
    ],
    /// 主机通道传输大小（HCTSIZ）。
    pub HCTSIZ [
        XFERSIZE OFFSET(0) NUMBITS(19) [],
        PKTCNT OFFSET(19) NUMBITS(10) [],
        PID OFFSET(29) NUMBITS(2) [
            Data0 = 0,
            Data2 = 1,
            Data1 = 2,
            Setup = 3,
        ],
    ],
];

register_structs! {
    /// 单个主机通道寄存器块（占 0x20 字节，基址 = `Dwc2Regs.hc_base + n * 0x20`）。
    pub Dwc2HostChannel {
        (0x00 => pub hcchar: ReadWrite<u32, HCCHAR::Register>),
        (0x04 => pub hcsplt: ReadWrite<u32>),
        (0x08 => pub hcint: ReadWrite<u32, HCINT::Register>),
        (0x0c => pub hcintmsk: ReadWrite<u32, HCINT::Register>),
        (0x10 => pub hctsiz: ReadWrite<u32, HCTSIZ::Register>),
        (0x14 => pub hcdma: ReadWrite<u32>),
        (0x18 => _reserved18: [u32; 2]),
        (0x20 => @END),
    }
}

register_structs! {
    /// 完整 DWC2 寄存器映射（仅声明本驱动使用到的字段；其余区段留作 reserved）。
    pub Dwc2Regs {
        (0x000 => pub gotgctl: ReadWrite<u32, GOTGCTL::Register>),
        (0x004 => _reserved004),
        (0x008 => pub gahbcfg: ReadWrite<u32, GAHBCFG::Register>),
        (0x00c => pub gusbcfg: ReadWrite<u32, GUSBCFG::Register>),
        (0x010 => pub grstctl: ReadWrite<u32, GRSTCTL::Register>),
        (0x014 => pub gintsts: ReadWrite<u32, GINTSTS::Register>),
        (0x018 => pub gintmsk: ReadWrite<u32, GINTMSK::Register>),
        (0x01c => _reserved01c: [u32; 2]),
        (0x024 => pub grxfsiz: ReadWrite<u32, GRXFSIZ::Register>),
        (0x028 => pub gnptxfsiz: ReadWrite<u32, GNPTXFSIZ::Register>),
        (0x02c => _reserved02c),
        (0x040 => pub gsnpsid: ReadOnly<u32, GSNPSID::Register>),
        (0x044 => _reserved044),
        (0x048 => pub ghwcfg2: ReadOnly<u32, GHWCFG2::Register>),
        (0x04c => pub ghwcfg3: ReadOnly<u32, GHWCFG3::Register>),
        (0x050 => pub ghwcfg4: ReadOnly<u32, GHWCFG4::Register>),
        (0x054 => _reserved054),
        (0x05c => pub gdfifocfg: ReadWrite<u32, GDFIFOCFG::Register>),
        (0x060 => _reserved060),
        (0x100 => pub hptxfsiz: ReadWrite<u32, HPTXFSIZ::Register>),
        (0x104 => _reserved104),
        (0x400 => pub hcfg: ReadWrite<u32, HCFG::Register>),
        (0x404 => _reserved404),
        (0x408 => pub hfnum: ReadOnly<u32, HFNUM::Register>),
        (0x40c => _reserved40c: [u32; 2]),
        /// Host All Channels Interrupt：bit n = 通道 n 有中断 pending（RO）。
        (0x414 => pub haint: ReadOnly<u32>),
        (0x418 => pub haintmsk: ReadWrite<u32>),
        (0x41c => _reserved41c),
        (0x440 => pub hprt0: ReadWrite<u32, HPRT0::Register>),
        (0x444 => _reserved444),
        (0x500 => pub hc: [Dwc2HostChannel; DWC2_MAX_HOST_CHANNELS]),
        (0x700 => _reserved700),
        (0xe00 => pub pcgctl: ReadWrite<u32>),
        (0xe04 => @END),
    }
}

register_structs! {
    /// CV182x 片内 USB2 PHY MMIO（DTS `usb@04340000` 第二段 `reg`，物理基址 [`crate::platform::CV182X_USB2_PHY_BASE`]）。
    /// 字段名对齐 vendor Linux `drivers/usb/dwc2/platform.c` 中的 `REGxxx` 宏。
    pub Cv182xUsb2Phy {
        (0x000 => _reserved000),
        (0x004 => _reserved004),
        (0x008 => _reserved008),
        (0x00c => _reserved00c),
        (0x010 => _reserved010),
        (0x014 => pub reg014: ReadWrite<u32>),
        (0x018 => _reserved018),
        (0x01c => _reserved01c),
        (0x020 => _reserved020),
        (0x024 => _reserved024),
        (0x028 => _reserved028),
        (0x02c => _reserved02c),
        (0x030 => _reserved030),
        (0x034 => _reserved034),
        (0x038 => _reserved038),
        (0x03c => _reserved03c),
        (0x040 => _reserved040),
        (0x044 => _reserved044),
        (0x048 => _reserved048),
        (0x04c => _reserved04c),
        (0x050 => _reserved050),
        (0x054 => _reserved054),
        (0x058 => @END),
    }
}
