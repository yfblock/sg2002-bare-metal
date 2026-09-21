//! USB 标准 SETUP 数据包构造（小端 8 字节）。
//!
//! 数组顺序与总线上 **SETUP PID** 后紧跟的 8 字节一致:`bmRequestType`、
//! `bRequest`、`wValue`、`wIndex`、`wLength`。类专用构造见各自模块
//! (hub 在 [`super::hub`],UVC 在 [`super::uvc::setup`])。

/// USB 标准设备请求;由 [`std_setup`] 构造 8 字节 SETUP 包。
pub enum StdRequest {
    /// `GET_DESCRIPTOR(Device)`（`wLength` 固定 18 = 整份设备描述符）。
    GetDescriptorDevice,
    /// `SET_ADDRESS`;`addr` 合法范围 **1..=127**（0 为默认地址）。
    SetAddress { addr: u8 },
    /// `SET_CONFIGURATION`;`cfg` = `bConfigurationValue`（非 0 激活）。
    SetConfiguration { cfg: u8 },
    /// `GET_DESCRIPTOR(Configuration)` — 对**已分配地址**的设备使用。
    /// 可先读 9 字节头再按 `wTotalLength` 读全。
    GetDescriptorConfiguration { cfg_index: u8, w_length: u16 },
    /// `SET_INTERFACE`（选接口备用设置,UVC 开流用）。
    SetInterface { alt: u8, interface: u8 },
}

/// 按 [`StdRequest`] 构造 8 字节 SETUP 包。
#[inline]
pub fn std_setup(req: StdRequest) -> [u8; 8] {
    match req {
        StdRequest::GetDescriptorDevice => [
            0x80, // bmRequestType: Dir IN, Type Standard, Recipient Device
            6,    // GET_DESCRIPTOR
            0x00, 0x01, // wValue: DEVICE(high) index 0(low)
            0x00, 0x00,
            18, 0, // wLength
        ],
        StdRequest::SetAddress { addr } => [
            0x00,
            5, // SET_ADDRESS;wValue = addr
            addr, 0,
            0, 0,
            0, 0,
        ],
        StdRequest::SetConfiguration { cfg } => [
            0x00,
            9, // SET_CONFIGURATION;wValue = cfg
            cfg, 0,
            0, 0,
            0, 0,
        ],
        StdRequest::GetDescriptorConfiguration { cfg_index, w_length } => {
            // USB 规范:wValue 高字节 = 描述符类型(2=CONFIGURATION),低字节 = 索引。
            let [vl, vh] = (2 << 8 | u16::from(cfg_index)).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0x80, 6, vl, vh, 0x00, 0x00, ll, lh]
        }
        StdRequest::SetInterface { alt, interface } => [
            0x01, // bmRequestType: Dir OUT, Type Standard, Recipient Interface
            0x0B, // SET_INTERFACE;wValue = alt, wIndex = interface
            alt, 0,
            interface, 0,
            0, 0,
        ],
    }
}
