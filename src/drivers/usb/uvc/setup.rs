//! UVC 类专用 SETUP 包构造：VideoStreaming（PROBE/COMMIT）与 VideoControl
//! （实体控制）的 `SET_CUR` / `GET_CUR` / `GET_MAX` / `GET_DEF`。
//! Streaming 变体的 `wIndex = interface`;Control 变体的
//! `wIndex = (entity_id<<8) | interface`。

/// UVC 类控制请求构造器(命名空间;每函数直接产出 8 字节 SETUP 包)。
pub struct UvcRequest;

impl UvcRequest {
    /// VideoStreaming `SET_CUR`（写 PROBE/COMMIT 结构;`wValue = selector<<8`）。
    #[inline]
    pub fn set_cur_streaming(interface: u8, selector: u8, w_length: u16) -> [u8; 8] {
        let [vl, vh] = ((selector as u16) << 8).to_le_bytes();
        let [ll, lh] = w_length.to_le_bytes();
        [0x21, 0x01, vl, vh, interface, 0, ll, lh]
    }

    /// VideoStreaming `GET_CUR`（读 PROBE/COMMIT 结构）。
    #[inline]
    pub fn get_cur_streaming(interface: u8, selector: u8, w_length: u16) -> [u8; 8] {
        let [vl, vh] = ((selector as u16) << 8).to_le_bytes();
        let [ll, lh] = w_length.to_le_bytes();
        [0xA1, 0x81, vl, vh, interface, 0, ll, lh]
    }

    /// VideoStreaming `GET_MAX`（最大探测结构长度）。
    #[inline]
    pub fn get_max_streaming(interface: u8, selector: u8, w_length: u16) -> [u8; 8] {
        let [vl, vh] = ((selector as u16) << 8).to_le_bytes();
        let [ll, lh] = w_length.to_le_bytes();
        [0xA1, 0x83, vl, vh, interface, 0, ll, lh]
    }

    /// VideoControl `SET_CUR`（写实体控制值,如白平衡/曝光模式）。
    #[inline]
    pub fn set_cur_control(interface: u8, entity_id: u8, selector: u8, w_length: u16) -> [u8; 8] {
        let [vl, vh] = ((selector as u16) << 8).to_le_bytes();
        let [il, ih] = (((entity_id as u16) << 8) | interface as u16).to_le_bytes();
        let [ll, lh] = w_length.to_le_bytes();
        [0x21, 0x01, vl, vh, il, ih, ll, lh]
    }

    /// VideoControl `GET_CUR`（读实体当前值）。
    #[inline]
    pub fn get_cur_control(interface: u8, entity_id: u8, selector: u8, w_length: u16) -> [u8; 8] {
        let [vl, vh] = ((selector as u16) << 8).to_le_bytes();
        let [il, ih] = (((entity_id as u16) << 8) | interface as u16).to_le_bytes();
        let [ll, lh] = w_length.to_le_bytes();
        [0xA1, 0x81, vl, vh, il, ih, ll, lh]
    }

    /// VideoControl `GET_DEF`：摄像头出厂默认值。
    #[inline]
    pub fn get_def_control(interface: u8, entity_id: u8, selector: u8, w_length: u16) -> [u8; 8] {
        let [vl, vh] = ((selector as u16) << 8).to_le_bytes();
        let [il, ih] = (((entity_id as u16) << 8) | interface as u16).to_le_bytes();
        let [ll, lh] = w_length.to_le_bytes();
        [0xA1, 0x87, vl, vh, il, ih, ll, lh]
    }
}
