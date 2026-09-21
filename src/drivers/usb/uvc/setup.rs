//! UVC 类专用 SETUP 包构造：Video Streaming（PROBE/COMMIT）与 Video Control
//! （实体控制）的 `SET_CUR` / `GET_CUR` / `GET_MAX` / `GET_DEF`。

/// UVC 类控制请求;由 [`uvc_setup`] 构造 8 字节 SETUP 包。
pub(crate) enum UvcRequest {
    /// Video Streaming `SET_CUR`（`wValue = selector<<8`）。
    SetCurVs { interface: u8, selector: u8, w_length: u16 },
    /// Video Streaming `GET_CUR`（探测/提交等）。
    GetCurVs { interface: u8, selector: u8, w_length: u16 },
    /// Video Streaming `GET_MAX`（最大探测结构长度）。
    GetMaxVs { interface: u8, selector: u8, w_length: u16 },
    /// Video Control `SET_CUR`（`wIndex = (entity_id<<8) | interface`）。
    SetCurVc { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
    /// Video Control `GET_CUR`。
    GetCurVc { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
    /// Video Control `GET_DEF`：摄像头出厂默认值。
    GetDefVc { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
}

/// 按 [`UvcRequest`] 构造 8 字节 SETUP 包。
#[inline]
pub(crate) fn uvc_setup(req: UvcRequest) -> [u8; 8] {
    match req {
        UvcRequest::SetCurVs { interface, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0x21, 0x01, vl, vh, interface, 0, ll, lh]
        }
        UvcRequest::GetCurVs { interface, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x81, vl, vh, interface, 0, ll, lh]
        }
        UvcRequest::GetMaxVs { interface, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x83, vl, vh, interface, 0, ll, lh]
        }
        UvcRequest::SetCurVc { interface, entity_id, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [il, ih] = ((u16::from(entity_id) << 8) | u16::from(interface)).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0x21, 0x01, vl, vh, il, ih, ll, lh]
        }
        UvcRequest::GetCurVc { interface, entity_id, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [il, ih] = ((u16::from(entity_id) << 8) | u16::from(interface)).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x81, vl, vh, il, ih, ll, lh]
        }
        UvcRequest::GetDefVc { interface, entity_id, selector, w_length } => {
            let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
            let [il, ih] = ((u16::from(entity_id) << 8) | u16::from(interface)).to_le_bytes();
            let [ll, lh] = w_length.to_le_bytes();
            [0xA1, 0x87, vl, vh, il, ih, ll, lh]
        }
    }
}
