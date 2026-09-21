//! Synopsys DWC2：探测、主机模式 bring-up（M1）、后续通道传输（M2+）。
//!
//! 寄存器名与位定义对齐 Linux `drivers/usb/dwc2/hw.h`（DesignWare OTG 2.0），通过
//! [`super::regs`] 中的 `tock-registers` 结构访问。
//!
//! 主机初始化对齐 CV182x/SG2002 路径；SoC 专属旋钮（UTMI 宽度/动态 FIFO/
//! GAHB DMA/PHY UTMI_OVERRIDE）在 [`super::cv182x`]。

use tock_registers::interfaces::{ReadWriteable, Readable, Writeable};

use crate::drivers::usb::error::{UsbError, UsbResult};
use core::time::Duration;

use crate::arch::time::delay;
use crate::drivers::usb;
/// 软复位序列分界：见 Linux `dwc2_core_reset()`（≥ 4.20a 用 `CSFTRST_DONE`，不再傻等 `CSFTRST` 自清）。
const DWC2_CORE_REV_4_20A: u32 = 0x420a;
use super::ch::{poll_until, spin_delay};
use super::regs::{
    GSNPSID, GINTMSK, GINTSTS, GOTGCTL, GRSTCTL,
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

/// 设/清 HPRT0 的 `PWR` 与 `RST`（本驱动仅需写这两个普通字段）。
///
/// 写前做两件事，均不可省：
/// 1. 屏蔽 W1C 位——`ENA` 等位读回的 1 表示"已使能"，写 1 的含义却是
///    "禁用/清除"，不清掉会误禁用端口（`HPRT0_W1C_MASK`）；
/// 2. 清掉目标字段位——否则 `rst=false` 这种 CLEAR 语义写不进去。
pub fn hprt0_port(pwr: bool, rst: bool) {
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

/// 在已检测到设备连接后发出 **USB 总线复位**（应在 `CONNSTS==1` 之后调用，
/// 符合主机枚举顺序）。CONNDET 的 W1C 在调用方
/// [`RootHub::clear_connection_change`](crate::drivers::usb::hub::RootHub) 完成。
pub fn port_reset_pulse() {
    // USB 2.0 spec TDRSTR (root hub reset) min = 50ms（实测 cv182x 的 PHY chirp K/J
    // 必须在 PRTRST 期间完成，不够长 chirp 不会发生，HPRT0.SPD 只能停在 FS）。
    // 这里给到 ≥60ms 留余量，并保留 PWR。
    hprt0_port(true, true); // 保留 PWR,拉 PRTRST
    delay(Duration::from_millis(60)); // PRTRST 60ms
    hprt0_port(true, false); // 解 PRTRST
    // TRSTRCY：reset 解除到首次 SETUP 之间 ≥10ms，慢 U 盘需 50–100ms 让 PHY 完成
    // chirp K-J-K-J + 内部 controller 启动。这里给 ~80ms 保守余量。
    delay(Duration::from_millis(80)); // TRSTRCY 80ms
}


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

/// M1：软复位、强制 Host、FIFO、GAHB、HCFG（及 CV182x PHY 下拉）。
/// 根口上电(`HPRT0.PWR`)不在此——由编排层经 `RootHub::port_power` 完成。
///
/// **不在此处** 发 USB 总线复位：应在确认 HPRT0 的 `CONNSTS` 后调用 [`port_reset_pulse`]。
///
/// 成功返回 Ok，不保证已有设备连接；请读 HPRT0 的 `CONNSTS`。
pub fn dwc2_host_init() -> UsbResult<()> {
    let dwc2 = usb::dwc2_regs();
    dwc2.gintmsk.set(0);
    dwc2.gintsts.set(0xFFFF_FFFF);

    core_soft_reset()?;
    force_host_mode()?;
    core_soft_reset()?;

    init_gotgctl_otg_host_session_overrides();
    super::cv182x::init_gusbcfg_cv182x_utmi16_hs();
    dwc2.pcgctl.set(0);
    super::cv182x::init_gahb_dma_cv182x();
    // Linux 在 HS 下不置 HCFG_FSLSSUPP（RPi/全速演示才需要 FSLS）。
    dwc2.hcfg.modify(HCFG::FSLSSUPP::CLEAR + HCFG::FSLSPCLKSEL.val(0));
    super::cv182x::init_host_fifos_cv182x()?;
    flush_tx_fifo_host_all()?;
    flush_rx_fifo_host()?;

    dwc2.haintmsk.set((1 << 0) | (1 << 1));
    dwc2.gintmsk.modify(GINTMSK::HCHINT::SET);

    dwc2.gintsts.set(0xFFFF_FFFF);

    super::cv182x::cv182x_usb2_phy_host_clear_utmi_override();

    Ok(())
}
