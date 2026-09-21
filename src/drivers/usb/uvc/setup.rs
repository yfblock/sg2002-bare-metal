//! UVC 类专用 SETUP 包构造：VideoStreaming（PROBE/COMMIT 带宽协商）与
//! VideoControl（实体控制）的 `SET_CUR` / `GET_CUR` / `GET_MAX` / `GET_DEF`。

/// UVC 类控制请求;由 [`uvc_setup`] 构造 8 字节 SETUP 包。
///
/// Streaming 变体作用于 VideoStreaming 接口（`wIndex = interface`），
/// Control 变体作用于 VideoControl 实体（`wIndex = (entity_id<<8) | interface`）。
pub(crate) enum UvcRequest {
    /// VideoStreaming `SET_CUR`（写 PROBE/COMMIT 结构；`wValue = selector<<8`）。
    SetCurStreaming { interface: u8, selector: u8, w_length: u16 },
    /// VideoStreaming `GET_CUR`（读 PROBE/COMMIT 结构）。
    GetCurStreaming { interface: u8, selector: u8, w_length: u16 },
    /// VideoStreaming `GET_MAX`（最大探测结构长度）。
    GetMaxStreaming { interface: u8, selector: u8, w_length: u16 },
    /// VideoControl `SET_CUR`（写实体控制值，如白平衡/曝光模式）。
    SetCurControl { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
    /// VideoControl `GET_CUR`（读实体当前值）。
    GetCurControl { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
    /// VideoControl `GET_DEF`：摄像头出厂默认值。
    GetDefControl { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
}

/// 按 [`UvcRequest`] 构造 8 字节 SETUP 包。
#[inline]
pub(crate) fn uvc_setup(req: UvcRequest) -> [u8; 8] {
    match req {
        UvcRequest::SetCurStreaming { interface, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0x21, 0x01, vl, vh, interface, 0, ll, lh]
        }
        UvcRequest::GetCurStreaming { interface, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x81, vl, vh, interface, 0, ll, lh]
        }
        UvcRequest::GetMaxStreaming { interface, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x83, vl, vh, interface, 0, ll, lh]
        }
        UvcRequest::SetCurControl { interface, entity_id, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [il, ih] = ((u16::from(entity_id) << 8) | u16::from(interface)).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0x21, 0x01, vl, vh, il, ih, ll, lh]
        }
        UvcRequest::GetCurControl { interface, entity_id, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [il, ih] = ((u16::from(entity_id) << 8) | u16::from(interface)).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x81, vl, vh, il, ih, ll, lh]
        }
        UvcRequest::GetDefControl { interface, entity_id, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [il, ih] = ((u16::from(entity_id) << 8) | u16::from(interface)).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x87, vl, vh, il, ih, ll, lh]
        }
    }
}
