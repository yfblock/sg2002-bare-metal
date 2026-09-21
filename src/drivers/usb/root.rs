//! 根 hub：DWC2 控制器自身(单端口、寄存器固定地址)。端口操作经
//! [`Hub`] trait 与外部 hub 无差别对接;根口专属的总线入口
//! [`RootHub::enumerate_bus`] 也在本模块。

use core::time::Duration;

use tock_registers::interfaces::{ReadWriteable, Readable};

use super::dwc2::{self, regs::HPRT0};
use super::error::{UsbError, UsbResult};
use super::hub::{Hub, W0_CONNECTION, W0_ENABLE, W0_RESET};

/// 根 hub：DWC2 控制器自身（单端口、寄存器固定地址——本类型仅作 trait
/// 分发标记，无字段；根口操作实现委托 `dwc2` 模块）。
pub struct RootHub;

impl Hub for RootHub {
    fn nports(&self) -> u8 {
        1
    }

    fn port_power(&self, _port: u8) -> UsbResult<()> {
        dwc2::hprt0_port(true, false); // W1C 安全写:置 PWR,不拉 RST
        Ok(())
    }

    fn port_status_w0(&self, _port: u8) -> UsbResult<u16> {
        let p = &super::dwc2_regs().hprt0;
        let mut w0 = 0u16;
        if p.is_set(HPRT0::CONNSTS) {
            w0 |= W0_CONNECTION;
        }
        if p.is_set(HPRT0::ENA) {
            w0 |= W0_ENABLE;
        }
        if p.is_set(HPRT0::RST) {
            w0 |= W0_RESET;
        }
        // SPD[18:17](Synopsys:00=HS,01=FS,10=LS)→ wPortStatus[10:9](00=FS,01=LS,10=HS):
        // 两种编码顺序相反,须重映射而非平移。
        let spd = (p.read(HPRT0::SPD) as u16) & 3;
        w0 |= match spd {
            0 => 2, // HS
            1 => 0, // FS
            _ => 1, // LS
        } << 9;
        Ok(w0)
    }

    fn reset_port(&self, _port: u8) -> UsbResult<()> {
        Ok(dwc2::port_reset_pulse())
    }

    fn clear_connection_change(&self, _port: u8) -> UsbResult<()> {
        // CONNDET 为连接/断开事件的沿位,W1C 清 pending——与外部 hub 的
        // CLEAR_FEATURE(C_PORT_CONNECTION) 同义。
        let p = &super::dwc2_regs().hprt0;
        if p.is_set(HPRT0::CONNDET) {
            p.modify(HPRT0::CONNDET::SET);
        }
        Ok(())
    }

    fn clear_reset_change(&self, _port: u8) -> UsbResult<()> {
        // 诚实 no-op:HPRT0 无 C_PORT_RESET 对应位——PRTRST 是控制位(写 0 释放),
        // 复位完成由 ENA=1 电平观察,无沿位可清(ENACHG 是 enable 语义,不冒充)。
        Ok(())
    }

    fn pwr_good(&self) -> Duration {
        Duration::ZERO
    }
}

impl RootHub {
    /// 树遍历整条总线：根口上电 → 等连接 → 取根口子设备 → 分派;被认领的
    /// 设备由各驱动自存,应用层向驱动取用。根口无设备/子设备未使能 = `Err`。
    pub fn enumerate_bus(&self) -> UsbResult<()> {
        log::info!(
            "[USB] bus: recursive hub scan (QEMU may insert virtual usb-hub on single root port)"
        );

        self.port_power(1)?; // HPRT0.PWR(controller bring-up 不代劳)
        if !self.wait_connect(1, Duration::from_secs(5)) {
            return Err(UsbError::Hardware(
                "no device on root port (enable VBUS e.g. GPIOB6 / cable / PHY)",
            ));
        }
        let mut next_addr: u8 = 1;
        let child = self
            .enumerate_child(1, &mut next_addr)
            .map_err(|e| match e {
                UsbError::NotPresent => UsbError::Protocol("root port child not enabled"),
                e => e,
            })?;
        log::info!("[USB] bus: scan finished.");
        child.dispatch(&mut next_addr)
    }
}
