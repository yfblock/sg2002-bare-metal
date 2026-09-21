//! 统一 Hub 端口抽象：根 hub（DWC2 `HPRT0`）与外部 hub（USB hub 类请求经 EP0）
//! 的端口操作归一到 [`Hub`] trait，枚举序列（[`wait_connect`] /
//! [`connect_reset_sequence`]）对两种上游无差别——多态调用点真实存在。
//!
//! 状态统一为 USB 2.0 §11.24.2 `wPortStatus` word0 布局（根口现场转换，
//! Linux `dwc2_hcd_hub_control` 同款做法）。总线枚举（[`enumerate_bus`] /
//! [`Hub::walk_subtree`]/[`dispatch_device`]）也在本模块——Linux hub.c 模型:
//! hub 驱动拥有树遍历与设备分派。

use core::time::Duration;

use tock_registers::interfaces::{Readable, ReadWriteable};

use super::device::{enumerate_device, UsbDevice, DRIVERS};
use super::dwc2::{self, regs::HPRT0};
use super::error::{UsbError, UsbResult};

// ---- 总线遍历(topology 并入;Linux hub.c 模型:hub 驱动拥有枚举) ----

/// Hub 端口特性：`PORT_RESET`。
const HUB_PORT_FEATURE_RESET: u16 = 4;
/// Hub 端口特性：`PORT_POWER`（hub 上电后端口电源默认关闭，必须先打开）。
const HUB_PORT_FEATURE_POWER: u16 = 8;
/// Hub 端口特性：`C_PORT_CONNECTION`（连接变化位，CLEAR 用）。
const HUB_PORT_FEATURE_C_CONNECTION: u16 = 16;
/// Hub 端口特性：`C_PORT_RESET`。
const HUB_PORT_FEATURE_C_RESET: u16 = 20;
/// Hub 类描述符类型（`GET_DESCRIPTOR(Hub)` 的 wValue 高字节）。
const USB_DT_HUB: u8 = 0x29;

/// `wPortStatus[0]`：当前连接。
pub const W0_CONNECTION: u16 = 1 << 0;
/// `wPortStatus[1]`：端口已使能。
pub const W0_ENABLE: u16 = 1 << 1;
/// `wPortStatus[4]`：复位进行中。
pub const W0_RESET: u16 = 1 << 4;

/// 统一 Hub：端口号取值 `1..=nports`。
pub trait Hub {
    /// 下游端口数。
    fn nports(&self) -> u8;
    /// 端口上电（外部 hub `SET_FEATURE(PORT_POWER)`；根口写 `HPRT0.PWR`，
    /// 经 [`dwc2::hprt0_port`] 的 W1C 安全写）。
    fn port_power(&self, port: u8) -> UsbResult<()>;
    /// 读端口状态 word0（统一布局；根口 `HPRT0` 现场转换）。
    fn port_status_w0(&self, port: u8) -> UsbResult<u16>;
    /// 复位端口并**等待稳定**（根口 `PRTRST` 脉冲含自带时序；外部 hub
    /// `SET_FEATURE(PORT_RESET)` + `TDRSTR`/`TRSTRCY` 等待）。
    fn reset_port(&self, port: u8) -> UsbResult<()>;
    /// 清 CONNECTION 变化位（根口 `HPRT0.CONNDET` W1C；外部
    /// `CLEAR_FEATURE(C_PORT_CONNECTION)`）。
    fn clear_connection_change(&self, port: u8) -> UsbResult<()>;
    /// 清 RESET 变化位（外部 `CLEAR_FEATURE(C_PORT_RESET)`）。
    fn clear_reset_change(&self, port: u8) -> UsbResult<()>;
    /// 端口上电稳定时间（外部 hub 描述符 `bPwrOn2PwrGood`；根口常驻供电，0）。
    fn pwr_good(&self) -> Duration;

    /// 轮询等待端口报告连接（按时上限，命中返回 true）。
    fn wait_connect(&self, port: u8, timeout: Duration) -> bool {
        let t0 = crate::arch::time::rdtime();
        loop {
            if let Ok(w0) = self.port_status_w0(port) {
                if w0 & W0_CONNECTION != 0 {
                    return true;
                }
            }
            if crate::arch::time::elapsed_since(t0) >= timeout {
                return false;
            }
            core::hint::spin_loop();
        }
    }

    /// 端口「清连接变化 → 复位并等稳定 → 清复位变化」统一序列，
    /// 对根口与外部 hub 无差别。
    fn connect_reset_sequence(&self, port: u8) -> UsbResult<()> {
        self.clear_connection_change(port)?;
        self.reset_port(port)?;
        self.clear_reset_change(port)?;
        Ok(())
    }

    /// **树节点语义**：枚举端口后面的设备,返回树此枝的子节点。
    ///
    /// 序列:查连接 → [`Self::connect_reset_sequence`] → 复位后须 `ENABLE`
    /// → 在默认地址 0 上完成 `SET_ADDRESS`/`SET_CONFIGURATION`,产出
    /// [`UsbDevice`](设备身份,含本端口速度)。空口/未使能/端口级失败 =
    /// `Err(NotPresent)`(空枝软信号,只记日志);其余 `Err` 为硬失败。
    ///
    /// 根口与外部 hub 端口走同一实现——树遍历(topology)对两者无差别。
    fn enumerate_child(&self, port: u8, next_addr: &mut u8) -> UsbResult<UsbDevice> {
        let w0 = match self.port_status_w0(port) {
            Ok(s) => s,
            Err(e) => {
                log::info!(target: "sg200x_bsp::usb::hub", "[USB] port {} GET_PORT_STATUS: {:?}", port, e);
                return Err(UsbError::NotPresent);
            }
        };
        if w0 & W0_CONNECTION == 0 {
            log::info!(target: "sg200x_bsp::usb::hub", "[USB] port {} empty (w0={:#06x})", port, w0);
            return Err(UsbError::NotPresent);
        }
        if let Err(e) = self.connect_reset_sequence(port) {
            log::warn!(target: "sg200x_bsp::usb::hub", "[USB] port {} reset sequence: {:?}", port, e);
            return Err(UsbError::NotPresent);
        }
        let after = match self.port_status_w0(port) {
            Ok(s) => s,
            Err(e) => {
                log::info!(target: "sg200x_bsp::usb::hub", "[USB] port {} after-reset status: {:?}", port, e);
                return Err(UsbError::NotPresent);
            }
        };
        if after & W0_ENABLE == 0 {
            log::info!(target: "sg200x_bsp::usb::hub", "[USB] port {} reset done but not enabled (w0={:#06x})", port, after);
            return Err(UsbError::NotPresent);
        }
        let speed = PortSpeed::from_status(after);
        log::info!(target: "sg200x_bsp::usb::hub", "[USB] port {} enabled w0={:#06x} SPD={}",
            port, after, speed.as_str());
        Ok(enumerate_device(speed, next_addr)?)
    }

    /// **树遍历**(Linux hub.c 模型):供电 → 等稳定 → 逐口取子设备
    /// ([`Self::enumerate_child`])并递归 [`dispatch_device`];返回子树被
    /// 接管的设备(多台先到先得)。单口 NotPresent 只跳过不中断;子树
    /// 无人接管 = `Err(NotPresent)`。
    fn walk_subtree(&self, next_addr: &mut u8) -> UsbResult<UsbDevice> {
        let nports = self.nports();
        // ① 给所有下游端口供电(USB 2.0 §11.11.1:hub 端口默认 PowerOff)
        for port in 1..=nports {
            if let Err(e) = self.port_power(port) {
                log::info!(target: "sg200x_bsp::usb::hub", "[USB] port {} POWER fail: {:?}", port, e);
            }
        }
        // ② 等 PwrOn2PwrGood + 100ms 让下游设备 VBUS 稳定 + 自检
        crate::arch::time::delay(self.pwr_good() + Duration::from_millis(100));

        let mut claimed: Option<UsbDevice> = None;
        for port in 1..=nports {
            let child = match self.enumerate_child(port, next_addr) {
                Ok(d) => d,
                Err(UsbError::NotPresent) => continue, // 空口/端口级失败:跳过
                Err(e) => return Err(e),               // 硬失败:中断整树
            };
            match dispatch_device(child, next_addr) {
                Ok(d) => {
                    if claimed.is_none() {
                        claimed = Some(d); // 多台候选先到先得
                    }
                }
                Err(UsbError::NotPresent) => {} // 子树无人接管:继续扫其他口
                Err(e) => return Err(e),
            }
        }
        claimed.ok_or(UsbError::NotPresent)
    }
}

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

/// Hub 描述符 `bNbrPorts` 上限（防描述符异常值撑爆遍历）。
const MAX_HUB_PORTS: u8 = 16;

/// 外部 hub：绑定已寻址的 [`dwc2::Ep0`] 与其描述符信息。
pub struct DeviceHub<'a> {
    ep0: &'a dwc2::Ep0,
    nports: u8,
    /// `bPwrOn2PwrGood` 已换算的毫秒数。
    pwr_on_pwr_good_ms: u32,
}

impl<'a> DeviceHub<'a> {
    /// `GET_DESCRIPTOR(Hub)` 读描述符并绑定。
    pub fn new(ep0: &'a dwc2::Ep0) -> UsbResult<Self> {
        let mut buf = [0u8; 64];
        ep0.read(hub_setup(HubRequest::GetHubDescriptor(64)), &mut buf)?;
        if buf[0] < 7 || buf[1] != USB_DT_HUB {
            return Err(UsbError::Protocol("invalid hub descriptor"));
        }
        Ok(Self {
            ep0,
            nports: buf[2].min(MAX_HUB_PORTS),
            pwr_on_pwr_good_ms: u32::from(buf[5]).saturating_mul(2),
        })
    }
}

impl Hub for DeviceHub<'_> {
    fn nports(&self) -> u8 {
        self.nports
    }

    fn port_power(&self, port: u8) -> UsbResult<()> {
        // USB 2.0 §11.11.1：hub 上电后端口默认 PowerOff，必须显式
        // SET_PORT_FEATURE(PORT_POWER) 才会给下游 VBUS。
        self.ep0.write_no_data(hub_setup(HubRequest::SetPortFeature {
            port: u16::from(port),
            feature: HUB_PORT_FEATURE_POWER,
        }))
    }

    fn port_status_w0(&self, port: u8) -> UsbResult<u16> {
        let mut buf = [0u8; 4];
        self.ep0.read(hub_setup(HubRequest::GetPortStatus(u16::from(port))), &mut buf)?;
        Ok(u16::from_le_bytes([buf[0], buf[1]]))
    }

    fn reset_port(&self, port: u8) -> UsbResult<()> {
        self.ep0.write_no_data(hub_setup(HubRequest::SetPortFeature {
            port: u16::from(port),
            feature: HUB_PORT_FEATURE_RESET,
        }))?;
        // USB 2.0 §7.1.7.5：TDRSTR ≥ 50ms，hub 完成后自动置 C_PORT_RESET；
        // TRSTRCY（复位解除到首次事务）一并等待。
        crate::arch::time::delay(Duration::from_millis(100));
        Ok(())
    }

    fn clear_connection_change(&self, port: u8) -> UsbResult<()> {
        self.ep0.write_no_data(hub_setup(HubRequest::ClearPortFeature {
            port: u16::from(port),
            feature: HUB_PORT_FEATURE_C_CONNECTION,
        }))
    }

    fn clear_reset_change(&self, port: u8) -> UsbResult<()> {
        self.ep0.write_no_data(hub_setup(HubRequest::ClearPortFeature {
            port: u16::from(port),
            feature: HUB_PORT_FEATURE_C_RESET,
        }))
    }

    fn pwr_good(&self) -> Duration {
        Duration::from_millis(u64::from(self.pwr_on_pwr_good_ms))
    }
}

/// USB 2.0 hub 端口速度位（`wPortStatus[10:9]`，§11.24.2.1）：
/// 00=full-speed, 01=low-speed, 10=high-speed。
#[derive(Clone, Copy)]
pub enum PortSpeed {
    Hs,
    Fs,
    Ls,
}

impl PortSpeed {
    pub fn from_status(status: u16) -> Self {
        match (status >> 9) & 3 {
            0 => PortSpeed::Fs,
            1 => PortSpeed::Ls,
            _ => PortSpeed::Hs,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            PortSpeed::Hs => "HS",
            PortSpeed::Fs => "FS",
            PortSpeed::Ls => "LS",
        }
    }
}



// ---- Hub 类 SETUP 构造（归本模块;与标准/UVC 构造同为纯函数）----

/// Hub 类控制请求;由 [`hub_setup`] 构造 8 字节 SETUP 包。
pub(crate) enum HubRequest {
    /// `SET_PORT_FEATURE`（`bmRequestType=0x23`，`bRequest=SET_FEATURE`）。
    SetPortFeature { port: u16, feature: u16 },
    /// `CLEAR_PORT_FEATURE`（清 `C_PORT_CONNECTION`/`C_PORT_RESET` 等变化位）。
    ClearPortFeature { port: u16, feature: u16 },
    /// `GET_PORT_STATUS`（数据阶段固定 4 字节 `wPortStatus`/`wPortChange`）;
    /// 参数 = 下游端口号。
    GetPortStatus(u16),
    /// `GET_DESCRIPTOR(Hub)` — 在 Hub **已 SET_CONFIGURATION** 后读取其描述符;
    /// 参数 = `wLength`。
    GetHubDescriptor(u16),
}

/// 按 [`HubRequest`] 构造 8 字节 SETUP 包。
#[inline]
pub(crate) fn hub_setup(req: HubRequest) -> [u8; 8] {
    match req {
        HubRequest::SetPortFeature { port, feature } => {
            let [fl, fh] = feature.to_le_bytes();
            let [pl, ph] = port.to_le_bytes();
            [0x23, 0x03, fl, fh, pl, ph, 0, 0]
        }
        HubRequest::ClearPortFeature { port, feature } => {
            let [fl, fh] = feature.to_le_bytes();
            let [pl, ph] = port.to_le_bytes();
            [0x23, 0x01, fl, fh, pl, ph, 0, 0]
        }
        HubRequest::GetPortStatus(port) => {
            let [pl, ph] = port.to_le_bytes();
            [0xA3, 0x00, 0, 0, pl, ph, 4, 0]
        }
        HubRequest::GetHubDescriptor(w_length) => {
            let [ll, lh] = w_length.to_le_bytes();
            [0xA0, 0x06, 0x00, USB_DT_HUB, 0x00, 0x00, ll, lh]
        }
    }
}

// ---- 总线遍历(topology 并入;Linux hub.c 模型:hub 驱动拥有枚举) ----

/// 分派一台已枚举的设备:
///
/// - **Hub** → [`Hub::walk_subtree`] 递归下探;
/// - **功能设备** → 类驱动注册表匹配,被接管则上抛;无人接管 =
///   `Err(NotPresent)`(空枝软信号)。
fn dispatch_device(dev: UsbDevice, next_addr: &mut u8) -> UsbResult<UsbDevice> {
    log::info!(target: "sg200x_bsp::usb::hub", "[USB] dev VID={:04x} PID={:04x} dev_class={:02x}",
        dev.vid, dev.pid, dev.dev_class);

    if dev.is_hub() {
        log::info!(target: "sg200x_bsp::usb::hub", "[USB]   -> Hub addr={}", dev.ep0.dev() as u8);
        return DeviceHub::new(&dev.ep0)?.walk_subtree(next_addr);
    }

    // 功能设备:注册表顺序即优先级,首个匹配者胜出;驱动无状态。
    log::info!(target: "sg200x_bsp::usb::hub", "[USB]   -> function addr={} first_ifc_class={:02x}",
        dev.ep0.dev(), dev.iface_class);
    match DRIVERS.iter().find(|d| d.matches(&dev)) {
        Some(driver) => {
            log::info!(target: "sg200x_bsp::usb::hub", "[USB]   -> driver \"{}\" took addr={}",
                driver.name(), dev.ep0.dev());
            Ok(dev)
        }
        None => Err(UsbError::NotPresent),
    }
}

/// 树遍历整条总线：根口上电 → 等连接 → 取根口子设备 → 分派;返回被类驱动
/// 接管的设备;根口无设备/无人接管 = `Err`(根级把 NotPresent 升格为带
/// 上下文的硬错误)。
pub fn enumerate_bus(root: &RootHub) -> UsbResult<UsbDevice> {
    log::info!("[USB] bus: recursive hub scan (QEMU may insert virtual usb-hub on single root port)");

    root.port_power(1)?; // HPRT0.PWR(controller bring-up 不代劳)
    if !root.wait_connect(1, Duration::from_secs(5)) {
        return Err(UsbError::Hardware(
            "no device on root port (enable VBUS e.g. GPIOB6 / cable / PHY)",
        ));
    }
    let mut next_addr: u8 = 1;
    let child = root.enumerate_child(1, &mut next_addr).map_err(|e| match e {
        UsbError::NotPresent => UsbError::Protocol("root port child not enabled"),
        e => e,
    })?;
    log::info!("[USB] bus: scan finished.");
    dispatch_device(child, &mut next_addr).map_err(|e| match e {
        UsbError::NotPresent => UsbError::Protocol("no device claimed by any class driver"),
        e => e,
    })
}
