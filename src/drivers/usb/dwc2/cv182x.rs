//! CV182x/SG2002 的 DWC2 **SoC 专属参数**（对齐 Linux `dwc2_set_cv182x_params` +
//! `dwc2_config_fifos` 路径，见
//! [Sipeed LicheeRV-Nano `params.c`](https://github.com/sipeed/LicheeRV-Nano-Build/blob/d4003f15b35d43ad4842f427050ab2bba0114fa5/linux_5.10/drivers/usb/dwc2/params.c#L217)）。
//!
//! 与 [`super::controller`] 的分工：controller 保留通用 bring-up 序列
//! （软复位/Force Host/FIFO flush/根口上电），本文件只装该 SoC 的旋钮——
//! 动态 FIFO 布局、UTMI 宽度、GAHB DMA、片内 PHY 的 UTMI_OVERRIDE 释放。

use tock_registers::interfaces::{ReadWriteable, Readable, Writeable};

use crate::drivers::usb;
use super::channel::spin_delay;
use super::regs::{
    GAHBCFG, GDFIFOCFG, GHWCFG2, GHWCFG3, GHWCFG4, GNPTXFSIZ, GRXFSIZ, GSNPSID, GUSBCFG,
    HPTXFSIZ, Cv182xUsb2Phy,
};
use crate::drivers::usb::error::UsbResult;

/// GDFIFOCFG 配置分界（Linux `core.h`/`hcd.c`:版本 ≥ 2.91a 时写 GDFIFOCFG）。
const DWC2_CORE_REV_2_91A: u32 = 0x291a;

/// 动态 FIFO：优先采用设备树常用值；超出 `GHWCFG3.DFIFO_DEPTH` 总深度时按
/// Linux `dwc2_calculate_dynamic_fifo` 收缩（主机通道数 = 1 + `GHWCFG2.NUM_HOST_CHAN`）。
pub(crate) fn init_host_fifos_cv182x() -> UsbResult<()> {
    let dwc2 = usb::dwc2_regs();
    let total = dwc2.ghwcfg3.read(GHWCFG3::DFIFO_DEPTH);
    let hc = 1 + dwc2.ghwcfg2.read(GHWCFG2::NUM_HOST_CHAN);
    let mut rx: u32 = 536;
    let mut nptx: u32 = 32;
    let mut ptx: u32 = 768;

    if rx.saturating_add(nptx).saturating_add(ptx) > total {
        rx = 516 + hc;
        nptx = 256;
        ptx = 768;
    }
    let sum = rx.saturating_add(nptx).saturating_add(ptx);
    if sum > total {
        ptx = total.saturating_sub(rx).saturating_sub(nptx);
    }

    dwc2.grxfsiz.write(GRXFSIZ::RXFDEP.val(rx));
    dwc2.gnptxfsiz
        .write(GNPTXFSIZ::NPTXFDEP.val(nptx) + GNPTXFSIZ::NPTXFSTADDR.val(rx));
    dwc2.hptxfsiz
        .write(HPTXFSIZ::PTXFDEP.val(ptx) + HPTXFSIZ::PTXFSTADDR.val(rx + nptx));

    let ded = dwc2.ghwcfg4.is_set(GHWCFG4::DED_FIFO_EN);
    if ded && dwc2.gsnpsid.read(GSNPSID::VERSION) >= DWC2_CORE_REV_2_91A {
        let epbase = rx.wrapping_add(nptx).wrapping_add(ptx);
        dwc2.gdfifocfg.modify(GDFIFOCFG::EPINFOBASE.val(epbase));
    }

    Ok(())
}

/// UTMI 数据宽度按 `GHWCFG4.UTMI_PHY_DATA_WIDTH` 自适配；HS 超时校准；
/// 保持 `FORCEHOSTMODE`（与 controller 的 `force_host_mode` 一致）。
///
/// **PHYIF16 设错是 chirp 失败的关键根因之一**：cv182x 的 PHY 实测为 8-bit UTMI
/// （vendor U-Boot `usb_gusbcfg = 0x40081400` 中 bit3 = 0 = 8-bit；
/// vendor Linux 也未显式设 PHYIF16）。如果 IP 报告 8-bit-only 或 programmable，
/// **必须** 把 PHYIF16 清零，否则 DWC2 与 PHY 的 UTMI 总线宽度不匹配，
/// chirp K/J 信号无法被正确解码，HPRT0.SPD 永远停在 FS。
pub(crate) fn init_gusbcfg_cv182x_utmi16_hs() {
    let dwc2 = usb::dwc2_regs();
    let utmi_w = dwc2.ghwcfg4.read(GHWCFG4::UTMI_PHY_DATA_WIDTH);
    let want_16bit = utmi_w == 1; // 16-bit only 时才必须 PHYIF16=1
    log::debug!("USB-DBG GHWCFG4.UTMI_PHY_DATA_WIDTH={utmi_w} (0=8 only, 1=16 only, 2=programmable) => PHYIF16={}",
        if want_16bit { 1 } else { 0 });
    let mut field = GUSBCFG::FORCEHOSTMODE::SET
        + GUSBCFG::ULPI_UTMI_SEL::CLEAR
        + GUSBCFG::TOUTCAL.val(0x7);
    if want_16bit {
        field += GUSBCFG::PHYIF16::SET;
    } else {
        field += GUSBCFG::PHYIF16::CLEAR;
    }
    dwc2.gusbcfg.modify(field);
}

pub(crate) fn init_gahb_dma_cv182x() {
    let dwc2 = usb::dwc2_regs();
    let arch = dwc2.ghwcfg2.read(GHWCFG2::ARCH);
    dwc2.gahbcfg.modify(
        GAHBCFG::HBSTLEN::Incr16 + GAHBCFG::GLBL_INTR_EN::SET,
    );
    if arch == 2 {
        dwc2.gahbcfg.modify(GAHBCFG::DMA_EN::SET);
    }
}

/// 与厂商 Linux `platform.c` host 路径对齐：**不设 `UTMI_OVERRIDE`**。
///
/// DWC2 在 host 模式下通过 UTMI 接口自行驱动 `dp_pulldown` / `dm_pulldown` 信号；
/// 若 `UTMI_OVERRIDE`=1，PHY 忽略 DWC2 的 UTMI 信号，可能干扰控制器的连接检测。
///
/// 写 `REG014=0` 将控制权还给 DWC2（vendor kernel host 路径不碰 `REG014`；
/// `utmi_chgdet_prepare`/`utmi_reset` 仅在 `CONFIG_USB_DWC2_PERIPHERAL` 充电检测里使用）。
pub(crate) fn cv182x_usb2_phy_host_clear_utmi_override() {
    // SAFETY: PHY 基址为编译期常量,视图恒有效。
    let phy = unsafe { &*(crate::platform::CV182X_USB2_PHY_BASE as *const Cv182xUsb2Phy) };
    let old = phy.reg014.get();
    phy.reg014.set(0);
    spin_delay(200_000);
    let now = phy.reg014.get();
    log::debug!("USB-DBG REG014 {:#06x}->{:#06x} (UTMI_OVERRIDE cleared, DWC2 drives pulldowns)",
        old, now);
}
