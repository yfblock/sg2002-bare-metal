//! Synopsys DWC2：探测、主机模式 bring-up（M1）、后续通道传输（M2+）。
//!
//! 寄存器名与位定义对齐 Linux `drivers/usb/dwc2/hw.h`（DesignWare OTG 2.0），通过
//! [`super::regs`] 中的 `tock-registers` 结构访问。
//!
//! 主机初始化对齐 CV182x/SG2002 路径（Linux
//! `dwc2_set_cv182x_params` + `dwc2_core_host_init` / `dwc2_config_fifos`：UTMI 16-bit、HS、动态 FIFO、
//! `GDFIFOCFG`、`PCGCTL`、`TOUTCAL`），见
//! [Sipeed LicheeRV-Nano `params.c`](https://github.com/sipeed/LicheeRV-Nano-Build/blob/d4003f15b35d43ad4842f427050ab2bba0114fa5/linux_5.10/drivers/usb/dwc2/params.c#L217)。

use tock_registers::interfaces::{ReadWriteable, Readable, Writeable};

use crate::drivers::usb::error::{UsbError, UsbResult};
use core::time::Duration;

use crate::arch::time::delay;
use crate::drivers::usb;
/// GDFIFOCFG 配置分界（Linux `core.h`/`hcd.c`:版本 ≥ 2.91a 时写 GDFIFOCFG）。
const DWC2_CORE_REV_2_91A: u32 = 0x291a;
/// 软复位序列分界：见 Linux `dwc2_core_reset()`（≥ 4.20a 用 `CSFTRST_DONE`，不再傻等 `CSFTRST` 自清）。
const DWC2_CORE_REV_4_20A: u32 = 0x420a;
use super::ch::{poll_until, spin_delay};
use tock_registers::registers::ReadWrite;
use super::regs::{
    GSNPSID,
    GAHBCFG, GDFIFOCFG, GHWCFG2, GHWCFG3, GHWCFG4, GINTMSK, GINTSTS, GOTGCTL, GRXFSIZ, GNPTXFSIZ, HPTXFSIZ, GRSTCTL,
    GUSBCFG, HCFG, HPRT0,
};

/// `dwc2_host_init` 内超时（`wait_ahb_idle` / 软复位 / FIFO flush）时转储；与 EP0 的 `USB-TOUT ch_*` 区分。
fn dbg_dwc2_init_timeout(phase: &'static str) {
    let dwc2 = usb::dwc2_regs();
    let grst = dwc2.grstctl.get();
    let gint = dwc2.gintsts.get();
    let gahb = dwc2.gahbcfg.get();
    let hprt = dwc2.hprt0.get();
    let ahb_idle = dwc2.grstctl.is_set(GRSTCTL::AHBIDLE);
    let csftrst = dwc2.grstctl.is_set(GRSTCTL::CSFTRST);
    let rst_done = dwc2.grstctl.is_set(GRSTCTL::CSFTRST_DONE);
    let rx_flush = dwc2.grstctl.is_set(GRSTCTL::RXFFLSH);
    let tx_flush = dwc2.grstctl.is_set(GRSTCTL::TXFFLSH);
    log::warn!("USB-TOUT dwc2-init [{}] GRSTCTL={:#010x} AHBIDLE={} CSFTRST={} CSFTRST_DONE={} RXFFLSH={} TXFFLSH={}",
        phase, grst, ahb_idle, csftrst, rst_done, rx_flush, tx_flush);
    log::warn!("USB-TOUT dwc2-init [{}] GINTSTS={:#010x} GAHBCFG={:#010x} HPRT0={:#010x}",
        phase, gint, gahb, hprt);
}

// Linux `core.h`：`snpsid >= 0x4f54291a` 时配置 `GDFIFOCFG`（`hcd.c`）。

fn wait_ahb_idle() -> UsbResult<()> {
    if poll_until(3_000_000, 32, || usb::dwc2_regs().grstctl.is_set(GRSTCTL::AHBIDLE)) {
        return Ok(());
    }
    dbg_dwc2_init_timeout("wait_ahb_idle");
    Err(UsbError::Timeout)
}

fn core_soft_reset() -> UsbResult<()> {
    wait_ahb_idle()?;
    let dwc2 = usb::dwc2_regs();
    let new_rst_seq = dwc2.gsnpsid.read(GSNPSID::VERSION) >= DWC2_CORE_REV_4_20A;

    dwc2.grstctl.modify(GRSTCTL::CSFTRST::SET);

    if !new_rst_seq && poll_until(3_000_000, 32, || !dwc2.grstctl.is_set(GRSTCTL::CSFTRST)) {
        spin_delay(4096);
        return Ok(());
    }
    if new_rst_seq && poll_until(3_000_000, 32, || dwc2.grstctl.is_set(GRSTCTL::CSFTRST_DONE)) {
        // Linux `dwc2_core_reset`：Core ≥ 4.20a 时等 `CSFTRST_DONE`，再清 `CSFTRST` 并置位 `CSFTRST_DONE`。
        dwc2.grstctl
            .modify(GRSTCTL::CSFTRST::CLEAR + GRSTCTL::CSFTRST_DONE::SET);
        spin_delay(4096);
        return Ok(());
    }
    dbg_dwc2_init_timeout(if new_rst_seq { "core_soft_reset CSFTRST_DONE" } else { "core_soft_reset CSFTRST (legacy)" });
    Err(UsbError::Timeout)
}

fn force_host_mode() -> UsbResult<()> {
    let dwc2 = usb::dwc2_regs();
    dwc2.gusbcfg.modify(GUSBCFG::FORCEHOSTMODE::SET);
    spin_delay(100_000);
    if poll_until(500_000, 32, || dwc2.gintsts.is_set(GINTSTS::CURMODE_HOST)) {
        return Ok(());
    }
    Err(UsbError::Hardware("CURMODE_HOST not set after FORCEHOSTMODE"))
}

/// 取根端口寄存器视图（HPRT0）。
#[inline]
pub fn hprt0() -> &'static ReadWrite<u32, HPRT0::Register> {
    &usb::dwc2_regs().hprt0
}

/// 设/清 HPRT0 的 `PWR` 与 `RST`（本驱动仅需写这两个普通字段）。
///
/// 写前做两件事，均不可省：
/// 1. 屏蔽 W1C 位——`ENA` 等位读回的 1 表示"已使能"，写 1 的含义却是
///    "禁用/清除"，不清掉会误禁用端口（`HPRT0_W1C_MASK`）；
/// 2. 清掉目标字段位——否则 `rst=false` 这种 CLEAR 语义写不进去。
fn hprt0_port(pwr: bool, rst: bool) {
    usb::dwc2_regs().hprt0.modify(
        HPRT0::PWR.val(pwr as u32)
            + HPRT0::RST.val(rst as u32)
            // W1C 位显式加入并置 0:modify() 的 RMW 会把它们从读回值中清掉
            // (写 0 到 W1C = 安全无操作),防止误清 pending 状态。
            + HPRT0::CONNDET.val(0)
            + HPRT0::ENA.val(0)
            + HPRT0::ENACHG.val(0)
            + HPRT0::OVRCURCHG.val(0)
    );
}

fn port_reset_pulse() {
    // USB 2.0 spec TDRSTR (root hub reset) min = 50ms（实测 cv182x 的 PHY chirp K/J
    // 必须在 PRTRST 期间完成，不够长 chirp 不会发生，HPRT0.SPD 只能停在 FS）。
    // 这里给到 ≥60ms 留余量，并保留 PWR；CONNDET 若是 pending 先写 1 清掉。
    if hprt0().is_set(HPRT0::CONNDET) {
        hprt0().modify(HPRT0::CONNDET::SET); // W1C:写 1 清 pending
    }
    hprt0_port(true, true); // 保留 PWR,拉 PRTRST
    delay(Duration::from_millis(60)); // PRTRST 60ms
    hprt0_port(true, false); // 解 PRTRST
    // TRSTRCY：reset 解除到首次 SETUP 之间 ≥10ms，慢 U 盘需 50–100ms 让 PHY 完成
    // chirp K-J-K-J + 内部 controller 启动。这里给 ~80ms 保守余量。
    delay(Duration::from_millis(80)); // TRSTRCY 80ms
}

/// 在已检测到设备连接后发出 **USB 总线复位**（应在 `CONNSTS==1` 之后调用，符合主机枚举顺序）。
///
/// 会先对 `CONNDET` 做写 1 清除（若置位），再拉 `PRTRST`。
pub fn dwc2_host_root_bus_reset_pulse() {
    port_reset_pulse();
}

// CV182x / SG2002 主机（Linux `dwc2_set_cv182x_params` + `dwc2_core_host_init`）

fn wait_grstctl_handshake(field: tock_registers::fields::Field<u32, GRSTCTL::Register>, set: bool) -> UsbResult<()> {
    let dwc2 = usb::dwc2_regs();
    if poll_until(3_000_000, 8, || dwc2.grstctl.is_set(field) == set) {
        spin_delay(64);
        return Ok(());
    }
    dbg_dwc2_init_timeout("wait_grstctl handshake");
    Err(UsbError::Timeout)
}

fn flush_rx_fifo_host() -> UsbResult<()> {
    wait_ahb_idle()?;
    usb::dwc2_regs().grstctl.write(GRSTCTL::RXFFLSH::SET);
    wait_grstctl_handshake(GRSTCTL::RXFFLSH, false)?;
    spin_delay(2_000);
    Ok(())
}

fn flush_tx_fifo_host_all() -> UsbResult<()> {
    wait_ahb_idle()?;
    usb::dwc2_regs()
        .grstctl
        .write(GRSTCTL::TXFFLSH::SET + GRSTCTL::TXFNUM.val(0x10));
    wait_grstctl_handshake(GRSTCTL::TXFFLSH, false)?;
    spin_delay(2_000);
    Ok(())
}

/// 动态 FIFO：优先采用设备树常用值；超出 `GHWCFG3.DFIFO_DEPTH` 总深度时按
/// Linux `dwc2_calculate_dynamic_fifo` 收缩（主机通道数 = 1 + `GHWCFG2.NUM_HOST_CHAN`）。
fn init_host_fifos_cv182x() -> UsbResult<()> {
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

/// `dr_mode=otg` 时常用：使能 override 并置位 A-session / VBUS valid，否则根口可能无电气活动。
fn init_gotgctl_otg_host_session_overrides() {
    usb::dwc2_regs().gotgctl.modify(
        GOTGCTL::DBNCE_FLTR_BYPASS::SET
            + GOTGCTL::AVALOEN::SET
            + GOTGCTL::AVALOVAL::SET
            + GOTGCTL::VBVALOEN::SET
            + GOTGCTL::VBVALOVAL::SET,
    );
    spin_delay(200_000);
}

/// UTMI 数据宽度按 `GHWCFG4.UTMI_PHY_DATA_WIDTH` 自适配；HS 超时校准；
/// 保持 `FORCEHOSTMODE`（与 [`force_host_mode`] 一致）。
///
/// **PHYIF16 设错是 chirp 失败的关键根因之一**：cv182x 的 PHY 实测为 8-bit UTMI
/// （vendor U-Boot `usb_gusbcfg = 0x40081400` 中 bit3 = 0 = 8-bit；
/// vendor Linux 也未显式设 PHYIF16）。如果 IP 报告 8-bit-only 或 programmable，
/// **必须** 把 PHYIF16 清零，否则 DWC2 与 PHY 的 UTMI 总线宽度不匹配，
/// chirp K/J 信号无法被正确解码，HPRT0.SPD 永远停在 FS。
fn init_gusbcfg_cv182x_utmi16_hs() {
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

fn init_gahb_dma_cv182x() {
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
fn cv182x_usb2_phy_host_clear_utmi_override() {
    let phy = usb::cv182x_phy_regs();
    let old = phy.reg014.get();
    phy.reg014.set(0);
    spin_delay(200_000);
    let now = phy.reg014.get();
    log::debug!("USB-DBG REG014 {:#06x}->{:#06x} (UTMI_OVERRIDE cleared, DWC2 drives pulldowns)",
        old, now);
}

/// M1：软复位、强制 Host、FIFO、GAHB、HCFG、根口上电（及 CV182x PHY 下拉）。
///
/// **不在此处** 发 USB 总线复位：应在确认 [`hprt0`] 的 `CONNSTS` 后调用 [`dwc2_host_root_bus_reset_pulse`]。
///
/// 成功返回 Ok，不保证已有设备连接；请读 [`hprt0`] 的 `CONNSTS`。
pub fn dwc2_host_init() -> UsbResult<()> {
    let dwc2 = usb::dwc2_regs();
    dwc2.gintmsk.set(0);
    dwc2.gintsts.set(0xFFFF_FFFF);

    core_soft_reset()?;
    force_host_mode()?;
    core_soft_reset()?;

    init_gotgctl_otg_host_session_overrides();
    init_gusbcfg_cv182x_utmi16_hs();
    dwc2.pcgctl.set(0);
    init_gahb_dma_cv182x();
    // Linux 在 HS 下不置 HCFG_FSLSSUPP（RPi/全速演示才需要 FSLS）。
    dwc2.hcfg.modify(HCFG::FSLSSUPP::CLEAR + HCFG::FSLSPCLKSEL.val(0));
    init_host_fifos_cv182x()?;
    flush_tx_fifo_host_all()?;
    flush_rx_fifo_host()?;

    dwc2.haintmsk.set((1 << 0) | (1 << 1));
    dwc2.gintmsk.modify(GINTMSK::HCHINT::SET);

    dwc2.gintsts.set(0xFFFF_FFFF);

    // 根口上电（PWR=1，不拉 RST）
    hprt0_port(true, false);

    cv182x_usb2_phy_host_clear_utmi_override();

    Ok(())
}
