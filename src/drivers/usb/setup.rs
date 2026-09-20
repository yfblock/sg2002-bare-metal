//! USB 标准 SETUP 数据包构造（小端 8 字节）。
//!
//! 数组顺序与总线上 **SETUP PID** 后紧跟的 8 字节一致:`bmRequestType`、
//! `bRequest`、`wValue`、`wIndex`、`wLength`。类专用构造见各自模块
//! (hub 在 [`super::hub`],UVC 在 [`super::uvc::setup`])。

/// `wLength`/`wValue`/`wIndex` 的 16 位小端拆分。USB 多字节字段一律小端。
#[inline]
fn wle(v: u16) -> [u8; 2] {
    v.to_le_bytes()
}

/// 构造标准 `GET_DESCRIPTOR(Device)`（`wLength` 固定 18 = 整份设备描述符）。
#[inline]
pub fn get_descriptor_device() -> [u8; 8] {
    [
        0x80, // bmRequestType: Dir IN, Type Standard, Recipient Device
        6,    // GET_DESCRIPTOR
        0x00,
        0x01, // wValue: DEVICE(high) index 0(low)
        0x00,
        0x00,
        wle(18)[0], wle(18)[1],
    ]
}

/// 构造 `SET_ADDRESS`。`addr` 合法范围 **1..=127**（0 为默认地址）。
#[inline]
pub fn set_address(addr: u8) -> [u8; 8] {
    [
        0x00,
        5, // SET_ADDRESS;wValue = addr
        addr, 0,
        0, 0,
        0, 0,
    ]
}

/// 构造 `SET_CONFIGURATION`。`cfg` = `bConfigurationValue`（非 0 激活）。
#[inline]
pub fn set_configuration(cfg: u8) -> [u8; 8] {
    [
        0x00,
        9, // SET_CONFIGURATION;wValue = cfg
        cfg, 0,
        0, 0,
        0, 0,
    ]
}

/// 构造 `GET_DESCRIPTOR(Configuration)` — 对**已分配地址**的设备使用。
/// 可先读 9 字节头再按 `wTotalLength` 读全。
#[inline]
pub fn get_descriptor_configuration(cfg_index: u8, w_length: u16) -> [u8; 8] {
    // USB 规范:wValue 高字节 = 描述符类型(2=CONFIGURATION),低字节 = 索引。
    let wvalue = 2 << 8 | u16::from(cfg_index);
    let [vl, vh] = wle(wvalue);
    let [ll, lh] = wle(w_length);
    [0x80, 6, vl, vh, 0x00, 0x00, ll, lh]
}

/// 构造 `SET_INTERFACE`（选接口备用设置,UVC 开流用）。
#[inline]
pub fn set_interface(alt: u8, interface: u8) -> [u8; 8] {
    [
        0x01, // bmRequestType: Dir OUT, Type Standard, Recipient Interface
        0x0B, // SET_INTERFACE;wValue = alt, wIndex = interface
        alt, 0,
        interface, 0,
        0, 0,
    ]
}
