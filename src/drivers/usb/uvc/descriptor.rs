//! UVC 配置描述符读取与解析：VS 流（格式/帧/等时端点候选，含选流打分与
//! interval 选择）与 VC 实体（CameraTerminal / ProcessingUnit）。
//! `read_configuration_descriptor` 经 EP0 读回整份配置描述符，其余为纯解析。
//! UVC `dwFrameInterval` 线上为 100ns tick 的 u32;域内统一 [`Duration`],
//! 仅在边界换算。

use core::time::Duration;

use crate::drivers::usb::error::{UsbError, UsbResult};
use crate::drivers::usb::dwc2;
use crate::drivers::usb::device;
use crate::drivers::usb::setup::{std_setup, StdRequest};

const USB_DT_ENDPOINT: u8 = 5;
const CS_INTERFACE: u8 = 0x24;

const VS_FORMAT_MJPEG: u8 = 0x06;
const VS_FRAME_MJPEG: u8 = 0x07;
const VS_FORMAT_UNCOMPRESSED: u8 = 0x04;
const VS_FRAME_UNCOMPRESSED: u8 = 0x05;

const USB_SUBCLASS_VIDEO_STREAMING: u8 = 0x02;
const USB_SUBCLASS_VIDEO_CONTROL: u8 = 0x01;

// VideoControl class-specific interface descriptor subtypes
const VC_INPUT_TERMINAL: u8 = 0x02;
const VC_PROCESSING_UNIT: u8 = 0x05;

/// `wTerminalType = 0x0201` 表示 ITT_CAMERA（CameraTerminal）。
const ITT_CAMERA: u16 = 0x0201;

/// UVC `dwFrameInterval`(100ns tick) → 域内 [`Duration`]。
#[inline]
fn from_uvc_ticks(t: u32) -> Duration {
    Duration::from_nanos(u64::from(t) * 100)
}

/// [`Duration`] → UVC `dwFrameInterval` u32(写 PROBE/COMMIT 负载用)。
#[inline]
pub(crate) fn to_uvc_ticks(d: Duration) -> u32 {
    (d.as_nanos() / 100) as u32
}

const ENDPOINT_ATTR_ISOCH: u8 = 1;

// 视频流参数

/// 解析得到的 VS 流参数(仅 Isoch;Bulk 已删除)。
#[derive(Clone, Debug)]
pub struct UvcStreamSelection {
    pub vs_interface: u8,
    pub alt_setting: u8,
    pub ep_num: u8,
    /// `wMaxPacketSize` 原始值（含 HS 带宽倍增位）。
    pub mps_raw: u16,
    pub format_index: u8,
    pub frame_index: u8,
    pub frame_interval: Duration,
    /// 选定格式是否为 MJPEG（用于上层判断输出是否为 JPEG）。
    pub is_mjpeg: bool,
    pub frame_w: u16,
    pub frame_h: u16,
    /// PROBE/COMMIT 协商后设备使用的 `dwMaxPayloadTransferSize`（单微帧字节数）。
    /// 由 [`crate::drivers::usb::uvc::uvc_start_video_stream`] 在协商后填充，用于 capture 切包。
    pub negotiated_payload_size: u32,
    /// 同一个 ep_num 下的所有 Isoch alt 候选 `(alt, mps_raw)`，按描述符出现顺序记录（未排序）。
    /// PROBE 协商后用 [`reselect_isoch_alt_for_payload`] 回选最匹配的 alt，避免出现
    /// "alt=1 但 negotiated_payload=3060" 这种带宽不够的 mismatch。
    pub isoch_alts_count: u8,
    pub isoch_alts: [(u8, u16); 8],
}

/// 根据偏好 interval 从某 frame 描述符的可用 interval 集合中选最接近的值。
///
/// 返回 0 表示未设偏好（调用方沿用 `min_ival`）；入参/候选/返回均为 100ns
/// tick(`dwFrameInterval` 原始值)。`i` 为该 VS_FRAME 描述符在 `cfg` 中的起始
/// 偏移，`bl` 为其 `bLength`；`ival_type`>0 为离散列表（其后跟 `ival_type` 个 u32），
/// `ival_type==0` 为连续区间（`dwMinFrameInterval`@26 / `dwMaxFrameInterval`@30）。
fn choose_frame_interval(
    cfg: &[u8],
    i: usize,
    bl: usize,
    dflt_ival: u32,
    ival_type: u8,
    pref: u32,
) -> u32 {
    if pref == 0 {
        return 0;
    }
    let best_for = |v: u32, cur_best: Option<u32>| -> Option<u32> {
        if v == 0 {
            return cur_best;
        }
        match cur_best {
            None => Some(v),
            Some(b) => {
                let d_new = if v >= pref { v - pref } else { pref - v };
                let d_old = if b >= pref { b - pref } else { pref - b };
                if d_new < d_old { Some(v) } else { Some(b) }
            }
        }
    };
    let mut best: Option<u32> = None;
    if ival_type == 0 {
        if bl >= 38 {
            let lo = u32::from_le_bytes([cfg[i + 26], cfg[i + 27], cfg[i + 28], cfg[i + 29]]);
            let hi = u32::from_le_bytes([cfg[i + 30], cfg[i + 31], cfg[i + 32], cfg[i + 33]]);
            if lo > 0 && hi > 0 {
                let ival = pref.clamp(lo, hi);
                best = best_for(ival, best);
            }
        }
        best = best_for(dflt_ival, best);
    } else {
        let count = ival_type as usize;
        let mut pos = i + 26;
        for _ in 0..count {
            if pos + 4 > i + bl {
                break;
            }
            let ival = u32::from_le_bytes([cfg[pos], cfg[pos + 1], cfg[pos + 2], cfg[pos + 3]]);
            best = best_for(ival, best);
            pos += 4;
        }
        best = best_for(dflt_ival, best);
    }
    best.unwrap_or(0)
}

/// 通过 EP0 读取完整配置描述符（按首 9 字节里的 `wTotalLength`，最大 4096）。
pub fn read_configuration_descriptor(ep: &dwc2::Ep0, cfg_index: u8) -> UsbResult<[u8; 4096]> {
    let mut hdr = [0u8; 9];
    ep.read(
        std_setup(StdRequest::GetDescriptorConfiguration { cfg_index, w_length: 9 }),
        &mut hdr,
    )?;
    if hdr[1] != device::USB_DT_CONFIGURATION {
        return Err(UsbError::Protocol("not a configuration descriptor"));
    }
    let total = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;
    if total > 4096 {
        return Err(UsbError::Protocol("configuration descriptor too large (>4096)"));
    }
    let mut buf = [0u8; 4096];
    ep.read(
        std_setup(StdRequest::GetDescriptorConfiguration {
            cfg_index,
            w_length: total as u16,
        }),
        &mut buf[..total],
    )?;
    Ok(buf)
}

/// 解析 VS 接口：优先选择 **MJPEG 格式**；若无 MJPEG 则使用未压缩 (YUY2/NV12) 格式。
/// 端点选择 **Isoch IN**(取带宽最高的 alt)。
///
/// 同时把所有 VS 候选打到串口，便于诊断。
/// 选流偏好。
///
/// - `frame_w`/`frame_h`：精确匹配该尺寸的 frame 得最高分；其余按"≤ 偏好面积
///   越接近越好、超出倒扣"打分。典型：JPU DMA pool 把可硬件解码的分辨率限制在
///   ~640×480，超出 `jpu_alloc` 失败
/// - `frame_interval`:非 [`Duration::ZERO`] 时从各 frame 的可用 interval 中
///   选**最接近**值而非默认最小间隔(最高 fps)。典型 30fps ≈ 33.33ms——
///   给廉价 webcam 更多曝光/ISP 余量
pub struct UvcPrefs {
    pub frame_w: u16,
    pub frame_h: u16,
    pub frame_interval: Duration,
}

pub fn parse_uvc_video_stream(cfg: &[u8], cfg_total: usize, prefs: &UvcPrefs) -> UsbResult<UvcStreamSelection> {
    let len = cfg_total.min(cfg.len());
    if len < 12 {
        return Err(UsbError::Protocol("cfg too short"));
    }

    let mut i = usize::from(cfg[0]);
    if i >= len {
        return Err(UsbError::Protocol("bad cfg bLength"));
    }

    let mut cur_ifc_class = 0u8;
    let mut cur_ifc_sub = 0u8;
    let mut cur_ifc_num = 0u8;
    let mut cur_alt = 0u8;

    let mut best_isoch: Option<(u8, u8, u16, u8)> = None;
    // 同一 ep_num 的所有 Isoch alt 候选，用于 PROBE 协商后回选合适带宽。
    let mut isoch_alts: [(u8, u16); 8] = [(0u8, 0u16); 8];
    let mut isoch_alts_count: usize = 0;

    let mut mjpeg_pick: Option<(u8, u8, u16, u16, u32)> = None;
    let mut uncomp_pick: Option<(u8, u8, u16, u16, u32)> = None;
    let mut cur_fmt_ix_for_frame = 0u8;

    while i + 2 <= len {
        let bl = cfg[i] as usize;
        if bl < 2 || i + bl > len {
            break;
        }
        let ty = cfg[i + 1];

        if ty == device::USB_DT_INTERFACE && bl >= 9 {
            cur_ifc_num = cfg[i + 2];
            cur_alt = cfg[i + 3];
            cur_ifc_class = cfg[i + 5];
            cur_ifc_sub = cfg[i + 6];
        } else if ty == CS_INTERFACE
            && cur_ifc_class == device::USB_CLASS_VIDEO
            && cur_ifc_sub == USB_SUBCLASS_VIDEO_STREAMING
        {
            let st = cfg.get(i + 2).copied().unwrap_or(0);
            if (st == VS_FORMAT_MJPEG || st == VS_FORMAT_UNCOMPRESSED) && bl >= 4 {
                cur_fmt_ix_for_frame = cfg[i + 3];
                log::info!("UVC: VS-fmt if={cur_ifc_num} alt={cur_alt} ix={} subtype={:#04x} ({})",
                    cur_fmt_ix_for_frame, st,
                    if st == VS_FORMAT_MJPEG { "MJPEG" } else { "Uncompressed" });
            }
            if (st == VS_FRAME_MJPEG || st == VS_FRAME_UNCOMPRESSED) && bl >= 26 {
                let frame_ix = cfg[i + 3];
                let w = u16::from_le_bytes([cfg[i + 5], cfg[i + 6]]);
                let h = u16::from_le_bytes([cfg[i + 7], cfg[i + 8]]);
                let dflt_ival = u32::from_le_bytes([cfg[i + 21], cfg[i + 22], cfg[i + 23], cfg[i + 24]]);
                let ival_type = cfg[i + 25];
                let mut min_ival = dflt_ival;
                if ival_type == 0 && bl >= 38 {
                    let dw_min = u32::from_le_bytes([cfg[i + 26], cfg[i + 27], cfg[i + 28], cfg[i + 29]]);
                    if dw_min > 0 { min_ival = dw_min; }
                } else if ival_type > 0 {
                    let count = ival_type as usize;
                    let mut pos = i + 26;
                    for _ in 0..count {
                        if pos + 4 > i + bl { break; }
                        let ival = u32::from_le_bytes([cfg[pos], cfg[pos + 1], cfg[pos + 2], cfg[pos + 3]]);
                        if ival > 0 && ival < min_ival { min_ival = ival; }
                        pos += 4;
                    }
                }
                // dwFrameInterval 是 **100ns** 单位，故 fps = 1e7/iv，fps*100 = 1e9/iv。
                // 原来写的是 1e8/iv，所有帧率标注都小了 10 倍（"6.00 fps" 实为 60fps）。
                let fps_x100 = if dflt_ival > 0 { 1_000_000_000_u32 / dflt_ival.max(1) } else { 0 };
                let fps_min_x100 = if min_ival > 0 { 1_000_000_000_u32 / min_ival.max(1) } else { 0 };
                log::info!("UVC: VS-frame fmt_ix={cur_fmt_ix_for_frame} frame_ix={frame_ix} {}x{} iv_dflt={dflt_ival} ({}.{:02} fps) iv_min={min_ival} ({}.{:02} fps) ival_type={ival_type}",
                    w, h,
                    fps_x100 / 100, fps_x100 % 100,
                    fps_min_x100 / 100, fps_min_x100 % 100);
                // 把该 frame 支持的 interval **全列出来**——只看 dflt/min 无法判断
                // "某个目标帧率到底可选不可选"（离散表只有一档时，任何偏好都是空操作）。
                if ival_type > 0 {
                    let mut pos = i + 26;
                    for k in 0..ival_type as usize {
                        if pos + 4 > i + bl {
                            break;
                        }
                        let ival = u32::from_le_bytes([cfg[pos], cfg[pos + 1], cfg[pos + 2], cfg[pos + 3]]);
                        let fps = if ival > 0 { 1_000_000_000_u32 / ival } else { 0 };
                        log::info!("UVC:   ival[{}] = {} ({}.{:02} fps)", k, ival, fps / 100, fps % 100);
                        pos += 4;
                    }
                } else if bl >= 38 {
                    let dw_min = u32::from_le_bytes([cfg[i + 26], cfg[i + 27], cfg[i + 28], cfg[i + 29]]);
                    let dw_max = u32::from_le_bytes([cfg[i + 30], cfg[i + 31], cfg[i + 32], cfg[i + 33]]);
                    let dw_step = u32::from_le_bytes([cfg[i + 34], cfg[i + 35], cfg[i + 36], cfg[i + 37]]);
                    let fmin = if dw_max > 0 { 1_000_000_000_u32 / dw_max } else { 0 };
                    let fmax = if dw_min > 0 { 1_000_000_000_u32 / dw_min } else { 0 };
                    log::info!("UVC:   ival continuous: min={dw_min} max={dw_max} step={dw_step} => {}.{:02}..{}.{:02} fps",
                        fmin / 100, fmin % 100, fmax / 100, fmax % 100);
                }
                // 选定本 frame 描述符实际使用的 interval：
                // 设了 PREFERRED_FRAME_INTERVAL 时选最接近它的可用值；否则沿用最小（最高 fps）。
                let chosen_ival = choose_frame_interval(cfg, i, bl, dflt_ival, ival_type, to_uvc_ticks(prefs.frame_interval));
                let dflt_ival = if chosen_ival > 0 { chosen_ival } else if min_ival > 0 { min_ival } else { dflt_ival };
                let pick = (cur_fmt_ix_for_frame, frame_ix, w, h, dflt_ival);
                let is_mjpeg = st == VS_FRAME_MJPEG;
                let rank = |(_, _, pw, ph, _): (u8, u8, u16, u16, u32)| -> i32 {
                    let w = i32::from(pw);
                    let h = i32::from(ph);
                    let area = w * h;
                    let pref_w = i32::from(prefs.frame_w);
                    let pref_h = i32::from(prefs.frame_h);
                    // ① 精确尺寸优先：与偏好 frame_w/h 完全一致的 frame 得最高分。
                    if w == pref_w && h == pref_h {
                        return 2_000_000;
                    }
                    // ② 非精确匹配：≤ 偏好面积越接近越好；超出按超出量倒扣。
                    let pref_area = pref_w.saturating_mul(pref_h);
                    if area <= pref_area {
                        pref_area - area
                    } else {
                        -(area - pref_area)
                    }
                };
                if is_mjpeg {
                    let pick_better = match mjpeg_pick {
                        None => true,
                        Some(prev) => rank(pick) > rank(prev),
                    };
                    if pick_better { mjpeg_pick = Some(pick); }
                } else {
                    let pick_better = match uncomp_pick {
                        None => true,
                        Some(prev) => rank(pick) > rank(prev),
                    };
                    if pick_better { uncomp_pick = Some(pick); }
                }
            }
        } else if ty == USB_DT_ENDPOINT
            && cur_ifc_class == device::USB_CLASS_VIDEO
            && cur_ifc_sub == USB_SUBCLASS_VIDEO_STREAMING
        {
            let ep_addr = cfg[i + 2];
            let attr = cfg[i + 3];
            let mps_raw = u16::from_le_bytes([cfg[i + 4], cfg[i + 5]]);
            let mps = dwc2::wmax_mps(mps_raw);
            let mult = dwc2::wmax_mult(mps_raw);
            let xfer = attr & 0x03;
            if (ep_addr & 0x80) == 0 {
                i += bl;
                continue;
            }
            let ep_num = ep_addr & 0x0F;
            let total = u32::from(mps) * u32::from(mult);
            log::info!("UVC: VS-cand if={cur_ifc_num} alt={cur_alt} ep={ep_num} kind={} mps={mps} mult={mult} total={total}/uframe mps_raw={mps_raw:#06x}",
                if xfer == ENDPOINT_ATTR_ISOCH { "Isoch" } else { "Other" });
            if xfer == ENDPOINT_ATTR_ISOCH {
                let tak = (cur_alt, ep_num, mps_raw, cur_ifc_num);
                // 沿用旧逻辑给 best_isoch 一个"初始猜测"（mult=1 优先），但真正的 alt
                // 由 PROBE 之后 [`reselect_isoch_alt_for_payload`] 重新选定。
                let payload = dwc2::wmax_payload_per_uframe(mps_raw);
                best_isoch = Some(match best_isoch {
                    None => tak,
                    Some(b) => {
                        // (mult==1, payload) 字典序：mult=1 候选优先（DWC2 兼容），
                        // 同档比每微帧总吞吐。
                        if (mult == 1, payload)
                            > (dwc2::wmax_mult(b.2) == 1, dwc2::wmax_payload_per_uframe(b.2))
                        {
                            tak
                        } else {
                            b
                        }
                    }
                });
                if isoch_alts_count < isoch_alts.len() {
                    isoch_alts[isoch_alts_count] = (cur_alt, mps_raw);
                    isoch_alts_count += 1;
                }
            }
        }

        i += bl;
    }

    let Some((alt, epn, mps_raw, vs_if)) = best_isoch else {
        return Err(UsbError::NotImplemented);
    };

    // 格式优先级：MJPEG 优先（带宽小；JPU 解码 MJPEG，Uncompressed 只作兜底）。
    let (fmt_ix, frame_ix, frame_w, frame_h, interval, is_mjpeg) = match (mjpeg_pick, uncomp_pick) {
        (Some((fi, frix, w, h, iv)), _) => (fi, frix, w, h, iv, true),
        (None, Some((fi, frix, w, h, iv))) => (fi, frix, w, h, iv, false),
        (None, None) => return Err(UsbError::Protocol("no VS format/frame")),
    };

    log::info!("UVC: SEL if={vs_if} alt={alt} ep={epn} mps_raw={:#06x} fmt_ix={fmt_ix} frame_ix={frame_ix} {}x{} iv={interval} mjpeg={is_mjpeg}",
        mps_raw, frame_w, frame_h);

    Ok(UvcStreamSelection {
        vs_interface: vs_if,
        alt_setting: alt,
        ep_num: epn,
        mps_raw,
        format_index: fmt_ix,
        frame_index: frame_ix,
        frame_interval: from_uvc_ticks(interval),
        is_mjpeg,
        frame_w,
        frame_h,
        negotiated_payload_size: 0,
        isoch_alts_count: isoch_alts_count as u8,
        isoch_alts,
    })
}

/// 根据 PROBE/COMMIT 协商出的 `payload_per_uframe`，从所有 Isoch alt 候选中挑出
/// **总带宽 ≥ payload** 且**最小**的那一个；找不到则取带宽最大的。
///
/// 找到后更新 `sel.alt_setting` 和 `sel.mps_raw`。
///
/// **DWC2 兼容性**：SG2002 等低端 DWC2 不可靠支持 HS 高带宽 Isoch（mult > 1），
/// 传输能完成但数据内容错误。因此只考虑 mult=1 的候选；若设备协商的 payload
/// 超过 mult=1 最大带宽，仍选最大 mult=1 alt——摄像头会自适应降低每微帧吞吐，
/// 帧传输耗时更长但数据正确。
pub(crate) fn reselect_isoch_alt_for_payload(sel: &mut UvcStreamSelection) {
    if sel.isoch_alts_count == 0 {
        return;
    }
    let need = sel.negotiated_payload_size;
    if need == 0 {
        return;
    }
    let alts = &sel.isoch_alts[..sel.isoch_alts_count as usize];
    let mut best_fit: Option<(u8, u16, u32)> = None;
    let mut best_max: Option<(u8, u16, u32)> = None;
    // **DWC2 兼容性**：SG2002 等低端 DWC2 不可靠支持 HS 高带宽 Isoch（mult > 1），
    // 传输能完成但数据内容错误——只考虑 mult=1 候选；若设备协商的 payload 超过
    // mult=1 最大带宽，仍选最大 alt，摄像头会自适应降低每微帧吞吐（帧传输
    // 耗时更长但数据正确）。
    for &(alt, mps_raw) in alts {
        let mps = dwc2::wmax_mps(mps_raw);
        let mult = dwc2::wmax_mult(mps_raw);
        if mult > 1 {
            continue;
        }
        let total = mps * mult;
        if total >= need {
            let pick = (alt, mps_raw, total);
            best_fit = Some(match best_fit {
                None => pick,
                Some(p) if p.2 > total => pick,
                Some(p) => p,
            });
        }
        let pick = (alt, mps_raw, total);
        best_max = Some(match best_max {
            None => pick,
            Some(p) if p.2 < total => pick,
            Some(p) => p,
        });
    }
    let (new_alt, new_mps_raw, new_total) = best_fit
        .or(best_max)
        .unwrap_or((sel.alt_setting, sel.mps_raw, 0));
    if new_alt != sel.alt_setting || new_mps_raw != sel.mps_raw {
        log::info!("UVC: re-select Isoch alt {} (mps_raw={:#06x}, {} B/uframe) -> alt {} (mps_raw={:#06x}, {} B/uframe) for payload={}",
            sel.alt_setting, sel.mps_raw,
            dwc2::wmax_payload_per_uframe(sel.mps_raw),
            new_alt, new_mps_raw, new_total, need);
        sel.alt_setting = new_alt;
        sel.mps_raw = new_mps_raw;
    }
}

// VC 实体

/// `parse_uvc_control_entities` 的输出：UVC VideoControl 接口及其下的实体 ID/支持位。
#[derive(Clone, Debug, Default)]
pub struct UvcControlEntities {
    pub vc_interface: u8,
    /// CameraTerminal（输入终端，wTerminalType=0x0201）的 entity ID。
    pub camera_terminal_id: Option<u8>,
    /// CameraTerminal `bmControls` 位掩码（最多 24 位，UVC 1.5）。
    pub ct_controls: u32,
    /// ProcessingUnit 的 entity ID。
    pub processing_unit_id: Option<u8>,
    /// ProcessingUnit `bmControls` 位掩码（最多 24 位）。
    pub pu_controls: u32,
}

/// 解析配置描述符，找出 VideoControl 接口下的 CameraTerminal/ProcessingUnit 实体 ID
/// 与各自的 `bmControls`，用于后续 SET_CUR 控制（自动白平衡 / 自动曝光等）。
pub fn parse_uvc_control_entities(cfg: &[u8], cfg_total: usize) -> Option<UvcControlEntities> {
    let len = cfg_total.min(cfg.len());
    if len < 12 {
        return None;
    }
    let mut i = usize::from(cfg[0]);
    if i >= len {
        return None;
    }

    let mut cur_ifc_class = 0u8;
    let mut cur_ifc_sub = 0u8;
    let mut cur_ifc_num;
    let mut out = UvcControlEntities::default();
    let mut found_vc = false;

    while i + 2 <= len {
        let bl = cfg[i] as usize;
        if bl < 2 || i + bl > len {
            break;
        }
        let ty = cfg[i + 1];

        if ty == device::USB_DT_INTERFACE && bl >= 9 {
            cur_ifc_num = cfg[i + 2];
            cur_ifc_class = cfg[i + 5];
            cur_ifc_sub = cfg[i + 6];
            if cur_ifc_class == device::USB_CLASS_VIDEO && cur_ifc_sub == USB_SUBCLASS_VIDEO_CONTROL {
                out.vc_interface = cur_ifc_num;
                found_vc = true;
            }
        } else if ty == CS_INTERFACE
            && cur_ifc_class == device::USB_CLASS_VIDEO
            && cur_ifc_sub == USB_SUBCLASS_VIDEO_CONTROL
            && bl >= 3
        {
            let st = cfg[i + 2];
            match st {
                VC_INPUT_TERMINAL
                    // bLength=15+x，bUnitID@3, wTerminalType@4..6, bAssocTerm@6,
                    // 后续 wObjectiveFocalLengthMin/Max + wOcularFocalLength + bControlSize@14, bmControls@15..
                    if bl >= 15 => {
                        let id = cfg[i + 3];
                        let tt = u16::from_le_bytes([cfg[i + 4], cfg[i + 5]]);
                        if tt == ITT_CAMERA {
                            out.camera_terminal_id = Some(id);
                            let csize = cfg[i + 14] as usize;
                            let cmax = csize.min(bl.saturating_sub(15)).min(4);
                            let mut bm = 0u32;
                            for k in 0..cmax {
                                bm |= u32::from(cfg[i + 15 + k]) << (8 * k);
                            }
                            out.ct_controls = bm;
                        }
                    }
                VC_PROCESSING_UNIT
                    // bLength=10+n，bUnitID@3, bSourceID@4, wMaxMultiplier@5..7, bControlSize@7, bmControls@8..
                    if bl >= 9 => {
                        let id = cfg[i + 3];
                        let csize = cfg[i + 7] as usize;
                        let cmax = csize.min(bl.saturating_sub(8)).min(4);
                        let mut bm = 0u32;
                        for k in 0..cmax {
                            bm |= u32::from(cfg[i + 8 + k]) << (8 * k);
                        }
                        out.processing_unit_id = Some(id);
                        out.pu_controls = bm;
                    }
                _ => {}
            }
        }

        i += bl;
    }

    if found_vc { Some(out) } else { None }
}
