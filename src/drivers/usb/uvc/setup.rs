//! UVC 类专用 SETUP 包构造：VideoStreaming（PROBE/COMMIT 带宽协商）与
//! VideoControl（实体控制）的 `SET_CUR` / `GET_CUR` / `GET_MAX` / `GET_DEF`。

/// UVC VideoStreaming 接口请求（`wIndex = interface`，`wValue = selector<<8`）。
pub(crate) enum VideoStreamingRequest {
    /// `SET_CUR`（写 PROBE/COMMIT 结构）。
    SetCur { interface: u8, selector: u8, w_length: u16 },
    /// `GET_CUR`（读 PROBE/COMMIT 结构）。
    GetCur { interface: u8, selector: u8, w_length: u16 },
    /// `GET_MAX`（最大探测结构长度）。
    GetMax { interface: u8, selector: u8, w_length: u16 },
}

/// UVC VideoControl 实体请求（`wIndex = (entity_id<<8) | interface`）。
pub(crate) enum VideoControlRequest {
    /// `SET_CUR`（写实体控制值，如白平衡/曝光模式）。
    SetCur { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
    /// `GET_CUR`（读实体当前值）。
    GetCur { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
    /// `GET_DEF`：摄像头出厂默认值。
    GetDef { interface: u8, entity_id: u8, selector: u8, w_length: u16 },
}

/// 按 [`VideoStreamingRequest`] 构造 8 字节 SETUP 包。
#[inline]
pub(crate) fn video_streaming_setup(req: VideoStreamingRequest) -> [u8; 8] {
    let (bm_req_type, b_request, interface, selector, w_length) = match req {
        VideoStreamingRequest::SetCur { interface, selector, w_length } => {
            (0x21, 0x01, interface, selector, w_length)
        }
        VideoStreamingRequest::GetCur { interface, selector, w_length } => {
            (0xA1, 0x81, interface, selector, w_length)
        }
        VideoStreamingRequest::GetMax { interface, selector, w_length } => {
            (0xA1, 0x83, interface, selector, w_length)
        }
    };
    let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
    let [ll, lh] = w_length.to_le_bytes();
    [bm_req_type, b_request, vl, vh, interface, 0, ll, lh]
}

/// 按 [`VideoControlRequest`] 构造 8 字节 SETUP 包。
#[inline]
pub(crate) fn video_control_setup(req: VideoControlRequest) -> [u8; 8] {
    let (bm_req_type, b_request, interface, entity_id, selector, w_length) = match req {
        VideoControlRequest::SetCur { interface, entity_id, selector, w_length } => {
            (0x21, 0x01, interface, entity_id, selector, w_length)
        }
        VideoControlRequest::GetCur { interface, entity_id, selector, w_length } => {
            (0xA1, 0x81, interface, entity_id, selector, w_length)
        }
        VideoControlRequest::GetDef { interface, entity_id, selector, w_length } => {
            (0xA1, 0x87, interface, entity_id, selector, w_length)
        }
    };
    let [vl, vh] = (u16::from(selector) << 8).to_le_bytes();
    let [il, ih] = ((u16::from(entity_id) << 8) | u16::from(interface)).to_le_bytes();
    let [ll, lh] = w_length.to_le_bytes();
    [bm_req_type, b_request, vl, vh, il, ih, ll, lh]
}
