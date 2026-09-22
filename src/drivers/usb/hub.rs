//! 统一 Hub 端口抽象：根 hub（[`super::root::RootHub`],DWC2 `HPRT0`）与
//! 外部 hub（USB hub 类请求经 EP0）的端口操作归一到 [`Hub`] trait,
//! 枚举序列（[`Hub::wait_connect`] / [`Hub::connect_reset_sequence`]）对两种
//! 上游无差别——多态调用点真实存在。
//!
//! 状态统一为 USB 2.0 §11.24.2 `wPortStatus` word0 布局。树遍历与设备分派
//! （[`Hub::walk_subtree`]/[`UsbDevice::dispatch`]）也在本模块——Linux hub.c
//! 模型:hub 驱动拥有树遍历与设备分派;根口专属入口在 [`super::root`]。

use core::time::Duration;

use super::device::UsbDevice;
use super::dwc2::ControlEp;
use super::error::{UsbError, UsbResult};
use super::DRIVERS;

/// 本模块日志 target(单一权威点)。
const LOG_TARGET: &str = "sg200x_bsp::usb::hub";

/// Hub 类描述符类型（`GET_DESCRIPTOR(Hub)` 的 wValue 高字节）。
const USB_DT_HUB: u8 = 0x29;

/// Hub 端口特性选择子(USB 2.0 Table 11-17;每次请求取一个,非位掩码)。
#[derive(Clone, Copy)]
pub(crate) enum PortFeature {
    /// `PORT_RESET`(=4):端口复位。
    Reset,
    /// `PORT_POWER`(=5):端口供电(hub 端口默认 PowerOff,必须显式打开)。
    Power,
    /// `C_PORT_CONNECTION`(=16):连接变化位,CLEAR 用。
    ConnectionChange,
    /// `C_PORT_RESET`(=20):复位变化位。
    ResetChange,
}

impl PortFeature {
    fn code(self) -> u16 {
        match self {
            PortFeature::Reset => 4,
            PortFeature::Power => 8, // 规范值 5;但本板 hub 收到 5 会断电重上电切断在线相机,8(实测值)被忽略而端口保持常供电
            PortFeature::ConnectionChange => 16,
            PortFeature::ResetChange => 20,
        }
    }
}

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
                log::info!(target: LOG_TARGET, "[USB] port {} GET_PORT_STATUS: {:?}", port, e);
                return Err(UsbError::NotPresent);
            }
        };
        if w0 & W0_CONNECTION == 0 {
            log::info!(target: LOG_TARGET, "[USB] port {} empty (w0={:#06x})", port, w0);
            return Err(UsbError::NotPresent);
        }
        if let Err(e) = self.connect_reset_sequence(port) {
            log::warn!(target: LOG_TARGET, "[USB] port {} reset sequence: {:?}", port, e);
            return Err(UsbError::NotPresent);
        }
        let after = match self.port_status_w0(port) {
            Ok(s) => s,
            Err(e) => {
                log::info!(target: LOG_TARGET, "[USB] port {} after-reset status: {:?}", port, e);
                return Err(UsbError::NotPresent);
            }
        };
        if after & W0_ENABLE == 0 {
            log::info!(target: LOG_TARGET, "[USB] port {} reset done but not enabled (w0={:#06x})", port, after);
            return Err(UsbError::NotPresent);
        }
        let speed = PortSpeed::from_status(after);
        log::info!(target: LOG_TARGET, "[USB] port {} enabled w0={:#06x} SPD={}",
            port, after, speed.as_str());
        Ok(UsbDevice::enumerate(speed, next_addr)?)
    }

    /// **树遍历**(Linux hub.c 模型):供电 → 等稳定 → 逐口取子设备
    /// ([`Self::enumerate_child`])并递归 [`UsbDevice::dispatch`];被认领的
    /// 设备由各驱动自存(不沿返回值上抛)。单口 NotPresent 只跳过不中断;
    /// 硬失败中断整树。
    fn walk_subtree(&self, next_addr: &mut u8) -> UsbResult<()> {
        let nports = self.nports();
        // ① 给无连接的端口供电(已有连接的跳过:常供电 hub 对已连接
        // 端口重发 PORT_POWER 会断电重上电,切断在线设备)。
        for port in 1..=nports {
            let connected = self
                .port_status_w0(port)
                .is_ok_and(|w0| w0 & W0_CONNECTION != 0);
            if connected {
                continue;
            }
            if let Err(e) = self.port_power(port) {
                log::info!(target: LOG_TARGET, "[USB] port {} POWER fail: {:?}", port, e);
            }
        }
        // ② 等 PwrOn2PwrGood + 100ms 让下游 VBUS 稳定。
        crate::arch::time::delay(self.pwr_good() + Duration::from_millis(100));

        for port in 1..=nports {
            let child = match self.enumerate_child(port, next_addr) {
                Ok(d) => d,
                Err(UsbError::NotPresent) => continue, // 空口/端口级失败:跳过
                Err(e) => return Err(e),               // 硬失败:中断整树
            };
            child.dispatch(next_addr)?;
        }
        Ok(())
    }
}

/// Hub 描述符 `bNbrPorts` 上限（防描述符异常值撑爆遍历）。
const MAX_HUB_PORTS: u8 = 16;

/// 外部 hub：持有已寻址设备的 [`dwc2::ControlEp`]（Copy 值，随本句柄走）与其
/// 描述符信息。
pub struct DeviceHub {
    control_ep: ControlEp,
    nports: u8,
    /// `bPwrOn2PwrGood` 已换算的毫秒数。
    pwr_on_pwr_good_ms: u32,
}

impl DeviceHub {
    /// `GET_DESCRIPTOR(Hub)` 读描述符并绑定。
    pub fn new(control_ep: ControlEp) -> UsbResult<Self> {
        let mut buf = [0u8; 64];
        control_ep.read(HubRequest::get_hub_descriptor(64), &mut buf)?;
        if buf[0] < 7 || buf[1] != USB_DT_HUB {
            return Err(UsbError::Protocol("invalid hub descriptor"));
        }
        Ok(Self {
            control_ep,
            nports: buf[2].min(MAX_HUB_PORTS),
            pwr_on_pwr_good_ms: buf[5] as u32 * 2,
        })
    }
}

impl UsbDevice {
    /// 分派一台已枚举的设备(按值消费,决定自身去向):
    ///
    /// - **Hub** → [`Hub::walk_subtree`] 递归下探;
    /// - **功能设备** → 首个匹配的类驱动 `probe`(设备由驱动自存,
    ///   不沿返回值上抛);无人认领只记日志(设备被忽略)。
    pub(crate) fn dispatch(self, next_addr: &mut u8) -> UsbResult<()> {
        log::info!(target: LOG_TARGET, "[USB] dev VID={:04x} PID={:04x} dev_class={:02x}",
        self.vid, self.pid, self.dev_class);

        if self.is_hub() {
            log::info!(target: LOG_TARGET, "[USB]   -> Hub addr={}", self.control_ep.dev() as u8);
            return DeviceHub::new(self.control_ep)?.walk_subtree(next_addr);
        }

        // 功能设备:注册表顺序即优先级,首个匹配者胜出。
        log::info!(target: LOG_TARGET, "[USB]   -> function addr={} first_ifc_class={:02x}",
            self.control_ep.dev(), self.iface_class);
        match DRIVERS.iter().find(|d| d.matches(&self)) {
            Some(driver) => {
                log::info!(target: LOG_TARGET, "[USB]   -> driver \"{}\" took addr={}",
                    driver.name(), self.control_ep.dev());
                driver.probe(self)
            }
            None => {
                log::info!(target: LOG_TARGET, "[USB]   -> no driver, ignored");
                Ok(())
            }
        }
    }
}

impl Hub for DeviceHub {
    fn nports(&self) -> u8 {
        self.nports
    }

    fn port_power(&self, port: u8) -> UsbResult<()> {
        // USB 2.0 §11.11.1：hub 上电后端口默认 PowerOff，必须显式
        // SET_PORT_FEATURE(PORT_POWER) 才会给下游 VBUS。
        self.control_ep.write_no_data(HubRequest::set_port_feature(
            port as u16,
            PortFeature::Power,
        ))
    }

    fn port_status_w0(&self, port: u8) -> UsbResult<u16> {
        let mut buf = [0u8; 4];
        self.control_ep
            .read(HubRequest::get_port_status(port as u16), &mut buf)?;
        Ok(u16::from_le_bytes([buf[0], buf[1]]))
    }

    fn reset_port(&self, port: u8) -> UsbResult<()> {
        self.control_ep.write_no_data(HubRequest::set_port_feature(
            port as u16,
            PortFeature::Reset,
        ))?;
        // USB 2.0 §7.1.7.5：TDRSTR ≥ 50ms，hub 完成后自动置 C_PORT_RESET；
        // TRSTRCY（复位解除到首次事务）一并等待。
        crate::arch::time::delay(Duration::from_millis(100));
        Ok(())
    }

    fn clear_connection_change(&self, port: u8) -> UsbResult<()> {
        self.control_ep
            .write_no_data(HubRequest::clear_port_feature(
                port as u16,
                PortFeature::ConnectionChange,
            ))
    }

    fn clear_reset_change(&self, port: u8) -> UsbResult<()> {
        self.control_ep
            .write_no_data(HubRequest::clear_port_feature(
                port as u16,
                PortFeature::ResetChange,
            ))
    }

    fn pwr_good(&self) -> Duration {
        Duration::from_millis(self.pwr_on_pwr_good_ms as u64)
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

/// Hub 类请求构造器(命名空间;每函数直接产出 8 字节 SETUP 包)。
pub(crate) struct HubRequest;

impl HubRequest {
    /// `SET_PORT_FEATURE`（`bmRequestType=0x23`，`bRequest=SET_FEATURE`）。
    #[inline]
    pub(crate) fn set_port_feature(port: u16, feature: PortFeature) -> [u8; 8] {
        let [fl, fh] = feature.code().to_le_bytes();
        let [pl, ph] = port.to_le_bytes();
        [0x23, 0x03, fl, fh, pl, ph, 0, 0]
    }

    /// `CLEAR_PORT_FEATURE`（清 `C_PORT_CONNECTION`/`C_PORT_RESET` 等变化位）。
    #[inline]
    pub(crate) fn clear_port_feature(port: u16, feature: PortFeature) -> [u8; 8] {
        let [fl, fh] = feature.code().to_le_bytes();
        let [pl, ph] = port.to_le_bytes();
        [0x23, 0x01, fl, fh, pl, ph, 0, 0]
    }

    /// `GET_PORT_STATUS`（数据阶段固定 4 字节 `wPortStatus`/`wPortChange`）;
    /// 参数 = 下游端口号。
    #[inline]
    pub(crate) fn get_port_status(port: u16) -> [u8; 8] {
        let [pl, ph] = port.to_le_bytes();
        [0xA3, 0x00, 0, 0, pl, ph, 4, 0]
    }

    /// `GET_DESCRIPTOR(Hub)` — 在 Hub **已 SET_CONFIGURATION** 后读取其描述符;
    /// 参数 = `wLength`。
    #[inline]
    pub(crate) fn get_hub_descriptor(w_length: u16) -> [u8; 8] {
        let [ll, lh] = w_length.to_le_bytes();
        [0xA0, 0x06, 0x00, USB_DT_HUB, 0x00, 0x00, ll, lh]
    }
}
