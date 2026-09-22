//! UVC 配置描述符读取与解析：VS 流（格式/帧/等时端点候选，含选流打分与
//! interval 选择）与 VC 实体（CameraTerminal / ProcessingUnit）。
//! 配置描述符经 [`ControlEp::get_configuration_descriptor`](crate::drivers::usb::dwc2::ControlEp)
//! 读回,其余为纯解析。
//! UVC `dwFrameInterval` 线上为 100ns tick 的 u32;域内统一 [`Duration`],
//! 仅在边界换算。

use core::time::Duration;

use crate::drivers::usb::device;
use crate::drivers::usb::dwc2;
use crate::drivers::usb::error::{UsbError, UsbResult};

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
    Duration::from_nanos(t as u64 * 100)
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
/// tick(`dwFrameInterval` 原始值)。`ival_type`>0 为离散列表（其后跟
/// `ival_type` 个 u32），`ival_type==0` 为连续区间（`dwMinFrameInterval`@26 /
/// `dwMaxFrameInterval`@30）。
fn choose_frame_interval(d: Desc, dflt_ival: u32, ival_type: u8, pref: u32) -> u32 {
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
                if d_new < d_old {
                    Some(v)
                } else {
                    Some(b)
                }
            }
        }
    };
    let mut best: Option<u32> = None;
    if ival_type == 0 {
        if d.len >= 38 {
            if let (Some(lo), Some(hi)) = (d.u32le(26), d.u32le(30)) {
                if lo > 0 && hi > 0 {
                    best = best_for(pref.clamp(lo, hi), best);
                }
            }
        }
        best = best_for(dflt_ival, best);
    } else {
        for k in 0..ival_type as usize {
            let Some(ival) = d.u32le(26 + 4 * k) else {
                break;
            };
            best = best_for(ival, best);
        }
        best = best_for(dflt_ival, best);
    }
    best.unwrap_or(0)
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

/// 越界检查的描述符读取视图:字段偏移相对描述符起点,长度为描述符自声明的
/// `bLength`;越界读取得 `None`——取代散落各处的 `bl >= N` 前置判断,
/// `from_le_bytes` 只在本视图出现。
#[derive(Clone, Copy)]
struct Desc<'a> {
    buf: &'a [u8],
    base: usize,
    len: usize,
}

impl<'a> Desc<'a> {
    /// 在 `buf[base]` 处构造视图;`end` 为扫描上限(`wTotalLength` 截断)。
    /// `bLength` 缺失 / <2 / 越出 `end` 返回 `None`。
    fn new(buf: &'a [u8], base: usize, end: usize) -> Option<Self> {
        let bl = *buf.get(base)? as usize;
        if bl < 2 || base + bl > end {
            return None;
        }
        Some(Desc { buf, base, len: bl })
    }

    /// 描述符类型 `bDescriptorType`(@1;构造已保证存在)。
    fn ty(&self) -> u8 {
        self.buf[self.base + 1]
    }

    fn u8(&self, off: usize) -> Option<u8> {
        (off < self.len).then(|| self.buf[self.base + off])
    }

    fn u16le(&self, off: usize) -> Option<u16> {
        (off + 2 <= self.len)
            .then(|| u16::from_le_bytes([self.buf[self.base + off], self.buf[self.base + off + 1]]))
    }

    fn u32le(&self, off: usize) -> Option<u32> {
        (off + 4 <= self.len).then(|| {
            u32::from_le_bytes([
                self.buf[self.base + off],
                self.buf[self.base + off + 1],
                self.buf[self.base + off + 2],
                self.buf[self.base + off + 3],
            ])
        })
    }
}

/// 单个 VS_FRAME 帧档位(打分/选择的最小单元;`ival` 为 100ns tick)。
#[derive(Clone, Copy)]
struct FramePick {
    fmt_ix: u8,
    frame_ix: u8,
    w: u16,
    h: u16,
    ival: u32,
    is_mjpeg: bool,
}

/// VS 等时 IN 端点候选累积器。
struct IsochCandidates {
    /// `(alt, ep, mps_raw, if)` 初始猜测:mult=1 优先、同档比每微帧吞吐;
    /// 真正的 alt 由 PROBE 后 [`reselect_isoch_alt_for_payload`] 回选。
    best: Option<(u8, u8, u16, u8)>,
    /// 同一 ep 的全部 Isoch alt `(alt, mps_raw)`,按描述符出现顺序(未排序)。
    alts: [(u8, u16); 8],
    count: usize,
}

/// 解析一个 VS_FRAME 描述符:读字段、算 min/default interval、打 fps 与
/// interval 全表诊断日志、按偏好选定 interval。非 FRAME 子类型/长度不足
/// 返回 `None`。
fn parse_vs_frame(d: Desc, st: u8, fmt_ix: u8, pref_ticks: u32) -> Option<FramePick> {
    if !(st == VS_FRAME_MJPEG || st == VS_FRAME_UNCOMPRESSED) {
        return None;
    }
    let frame_ix = d.u8(3)?;
    let w = d.u16le(5)?;
    let h = d.u16le(7)?;
    let dflt_ival = d.u32le(21)?;
    let ival_type = d.u8(25)?;
    let mut min_ival = dflt_ival;
    if ival_type == 0 && d.len >= 38 {
        if let Some(dw_min) = d.u32le(26) {
            if dw_min > 0 {
                min_ival = dw_min;
            }
        }
    } else if ival_type > 0 {
        for k in 0..ival_type as usize {
            let Some(ival) = d.u32le(26 + 4 * k) else {
                break;
            };
            if ival > 0 && ival < min_ival {
                min_ival = ival;
            }
        }
    }
    // dwFrameInterval 是 **100ns** 单位，故 fps = 1e7/iv，fps*100 = 1e9/iv。
    // 原来写的是 1e8/iv，所有帧率标注都小了 10 倍（"6.00 fps" 实为 60fps）。
    let fps_x100 = if dflt_ival > 0 {
        1_000_000_000_u32 / dflt_ival.max(1)
    } else {
        0
    };
    let fps_min_x100 = if min_ival > 0 {
        1_000_000_000_u32 / min_ival.max(1)
    } else {
        0
    };
    log::info!("UVC: VS-frame fmt_ix={fmt_ix} frame_ix={frame_ix} {w}x{h} iv_dflt={dflt_ival} ({}.{:02} fps) iv_min={min_ival} ({}.{:02} fps) ival_type={ival_type}",
        fps_x100 / 100, fps_x100 % 100,
        fps_min_x100 / 100, fps_min_x100 % 100);
    // 把该 frame 支持的 interval **全列出来**——只看 dflt/min 无法判断
    // "某个目标帧率到底可选不可选"（离散表只有一档时，任何偏好都是空操作）。
    if ival_type > 0 {
        for k in 0..ival_type as usize {
            let Some(ival) = d.u32le(26 + 4 * k) else {
                break;
            };
            let fps = if ival > 0 {
                1_000_000_000_u32 / ival
            } else {
                0
            };
            log::info!(
                "UVC:   ival[{}] = {} ({}.{:02} fps)",
                k,
                ival,
                fps / 100,
                fps % 100
            );
        }
    } else if d.len >= 38 {
        let dw_min = d.u32le(26).unwrap_or(0);
        let dw_max = d.u32le(30).unwrap_or(0);
        let dw_step = d.u32le(34).unwrap_or(0);
        let fmin = if dw_max > 0 {
            1_000_000_000_u32 / dw_max
        } else {
            0
        };
        let fmax = if dw_min > 0 {
            1_000_000_000_u32 / dw_min
        } else {
            0
        };
        log::info!("UVC:   ival continuous: min={dw_min} max={dw_max} step={dw_step} => {}.{:02}..{}.{:02} fps",
            fmin / 100, fmin % 100, fmax / 100, fmax % 100);
    }
    // 选定本 frame 描述符实际使用的 interval：
    // 设了偏好 interval 时选最接近它的可用值；否则沿用最小（最高 fps）。
    let chosen_ival = choose_frame_interval(d, dflt_ival, ival_type, pref_ticks);
    let ival = if chosen_ival > 0 {
        chosen_ival
    } else if min_ival > 0 {
        min_ival
    } else {
        dflt_ival
    };
    Some(FramePick {
        fmt_ix,
        frame_ix,
        w,
        h,
        ival,
        is_mjpeg: st == VS_FRAME_MJPEG,
    })
}

/// 帧尺寸打分:与偏好精确一致得最高;否则 ≤ 偏好面积越接近越好、超出按
/// 超出量倒扣。
fn frame_rank(p: &FramePick, prefs: &UvcPrefs) -> i32 {
    let w = p.w as i32;
    let h = p.h as i32;
    let area = w * h;
    let pref_w = prefs.frame_w as i32;
    let pref_h = prefs.frame_h as i32;
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
}

impl IsochCandidates {
    /// 考察一个 VS 接口下的端点描述符:IN 等时则记入初始猜测与 alt 全量表。
    /// 字段不足(畸形描述符)静默跳过。
    fn consider(&mut self, d: Desc, cur_alt: u8, cur_ifc_num: u8) -> Option<()> {
        let ep_addr = d.u8(2)?;
        let attr = d.u8(3)?;
        let mps_raw = d.u16le(4)?;
        let mps = dwc2::wmax_mps(mps_raw);
        let xfer = attr & 0x03;
        if (ep_addr & 0x80) == 0 {
            return Some(()); // 只关心 IN
        }
        let ep_num = ep_addr & 0x0F;
        log::info!("UVC: VS-cand if={cur_ifc_num} alt={cur_alt} ep={ep_num} kind={} mps={mps} total={mps}/uframe mps_raw={mps_raw:#06x}",
            if xfer == ENDPOINT_ATTR_ISOCH { "Isoch" } else { "Other" });
        if xfer == ENDPOINT_ATTR_ISOCH {
            let tak = (cur_alt, ep_num, mps_raw, cur_ifc_num);
            let payload = dwc2::wmax_payload_per_uframe(mps_raw);
            // payload 最大者优先(mult 恒为 1:reselect 只选 mult=1 alt)。
            self.best = Some(match self.best {
                None => tak,
                Some(b) => {
                    if payload > dwc2::wmax_payload_per_uframe(b.2) {
                        tak
                    } else {
                        b
                    }
                }
            });
            if self.count < self.alts.len() {
                self.alts[self.count] = (cur_alt, mps_raw);
                self.count += 1;
            }
        }
        Some(())
    }
}

pub(crate) fn parse_uvc_video_stream(
    cfg: &[u8],
    cfg_total: usize,
    prefs: &UvcPrefs,
) -> UsbResult<UvcStreamSelection> {
    let len = cfg_total.min(cfg.len());
    if len < 12 {
        return Err(UsbError::Protocol("cfg too short"));
    }

    let mut i = cfg[0] as usize;
    if i >= len {
        return Err(UsbError::Protocol("bad cfg bLength"));
    }

    let pref_ticks = to_uvc_ticks(prefs.frame_interval);
    let mut cur_ifc_class = 0u8;
    let mut cur_ifc_sub = 0u8;
    let mut cur_ifc_num = 0u8;
    let mut cur_alt = 0u8;

    let mut isoch = IsochCandidates {
        best: None,
        alts: [(0, 0); 8],
        count: 0,
    };
    let mut mjpeg_pick: Option<FramePick> = None;
    let mut uncomp_pick: Option<FramePick> = None;
    let mut cur_fmt_ix = 0u8;

    while i + 2 <= len {
        let Some(d) = Desc::new(cfg, i, len) else {
            break;
        };
        let ty = d.ty();

        if ty == device::USB_DT_INTERFACE && d.len >= 9 {
            cur_ifc_num = d.u8(2).unwrap_or(cur_ifc_num);
            cur_alt = d.u8(3).unwrap_or(cur_alt);
            cur_ifc_class = d.u8(5).unwrap_or(cur_ifc_class);
            cur_ifc_sub = d.u8(6).unwrap_or(cur_ifc_sub);
        } else if ty == CS_INTERFACE
            && cur_ifc_class == device::USB_CLASS_VIDEO
            && cur_ifc_sub == USB_SUBCLASS_VIDEO_STREAMING
        {
            let st = d.u8(2).unwrap_or(0);
            if st == VS_FORMAT_MJPEG || st == VS_FORMAT_UNCOMPRESSED {
                if let Some(ix) = d.u8(3) {
                    cur_fmt_ix = ix;
                    log::info!("UVC: VS-fmt if={cur_ifc_num} alt={cur_alt} ix={cur_fmt_ix} subtype={st:#06x} ({})",
                        if st == VS_FORMAT_MJPEG { "MJPEG" } else { "Uncompressed" });
                }
            }
            if let Some(pick) = parse_vs_frame(d, st, cur_fmt_ix, pref_ticks) {
                let beats = match if pick.is_mjpeg {
                    &mjpeg_pick
                } else {
                    &uncomp_pick
                } {
                    None => true,
                    Some(prev) => frame_rank(&pick, prefs) > frame_rank(prev, prefs),
                };
                if beats {
                    if pick.is_mjpeg {
                        mjpeg_pick = Some(pick);
                    } else {
                        uncomp_pick = Some(pick);
                    }
                }
            }
        } else if ty == USB_DT_ENDPOINT
            && cur_ifc_class == device::USB_CLASS_VIDEO
            && cur_ifc_sub == USB_SUBCLASS_VIDEO_STREAMING
        {
            let _ = isoch.consider(d, cur_alt, cur_ifc_num);
        }

        i += d.len;
    }

    let Some((alt, epn, mps_raw, vs_if)) = isoch.best else {
        return Err(UsbError::NotImplemented);
    };

    // 格式优先级：MJPEG 优先（带宽小；JPU 解码 MJPEG，Uncompressed 只作兜底）。
    let pick = match (mjpeg_pick, uncomp_pick) {
        (Some(p), _) => p,
        (None, Some(p)) => p,
        (None, None) => return Err(UsbError::Protocol("no VS format/frame")),
    };

    log::info!("UVC: SEL if={vs_if} alt={alt} ep={epn} mps_raw={mps_raw:#06x} fmt_ix={} frame_ix={} {}x{} iv={} mjpeg={}",
        pick.fmt_ix, pick.frame_ix, pick.w, pick.h, pick.ival, pick.is_mjpeg);

    Ok(UvcStreamSelection {
        vs_interface: vs_if,
        alt_setting: alt,
        ep_num: epn,
        mps_raw,
        format_index: pick.fmt_ix,
        frame_index: pick.frame_ix,
        frame_interval: from_uvc_ticks(pick.ival),
        is_mjpeg: pick.is_mjpeg,
        frame_w: pick.w,
        frame_h: pick.h,
        negotiated_payload_size: 0,
        isoch_alts_count: isoch.count as u8,
        isoch_alts: isoch.alts,
    })
}

/// 根据 PROBE/COMMIT 协商出的 `payload_per_uframe`，从所有 Isoch alt 候选中挑出
/// **总带宽 ≥ payload** 且**最小**的那一个；找不到则取带宽最大的。
///
/// 找到后更新 `self.alt_setting` 和 `self.mps_raw`。
///
/// **DWC2 兼容性**：SG2002 等低端 DWC2 不可靠支持 HS 高带宽 Isoch（mult > 1），
/// 传输能完成但数据内容错误。因此只考虑 mult=1 的候选；若设备协商的 payload
/// 超过 mult=1 最大带宽，仍选最大 mult=1 alt——摄像头会自适应降低每微帧吞吐，
/// 帧传输耗时更长但数据正确。
impl UvcStreamSelection {
    pub(crate) fn reselect_isoch_alt_for_payload(&mut self) {
        if self.isoch_alts_count == 0 {
            return;
        }
        let need = self.negotiated_payload_size;
        if need == 0 {
            return;
        }
        let alts = &self.isoch_alts[..self.isoch_alts_count as usize];
        let mut best_fit: Option<(u8, u16, u32)> = None;
        let mut best_max: Option<(u8, u16, u32)> = None;
        // SG2002 DWC2 只支持 mult=1:total = mps。
        // 若设备协商的 payload 超过最大带宽,仍选最大 alt——摄像头自适应。
        for &(alt, mps_raw) in alts {
            let total = dwc2::wmax_mps(mps_raw);
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
        let (new_alt, new_mps_raw, new_total) =
            best_fit
                .or(best_max)
                .unwrap_or((self.alt_setting, self.mps_raw, 0));
        if new_alt != self.alt_setting || new_mps_raw != self.mps_raw {
            log::info!("UVC: re-select Isoch alt {} (mps_raw={:#06x}, {} B/uframe) -> alt {} (mps_raw={:#06x}, {} B/uframe) for payload={}",
                self.alt_setting, self.mps_raw,
                dwc2::wmax_payload_per_uframe(self.mps_raw),
                new_alt, new_mps_raw, new_total, need);
            self.alt_setting = new_alt;
            self.mps_raw = new_mps_raw;
        }
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
pub(crate) fn parse_uvc_control_entities(
    cfg: &[u8],
    cfg_total: usize,
) -> Option<UvcControlEntities> {
    let len = cfg_total.min(cfg.len());
    if len < 12 {
        return None;
    }
    let mut i = cfg[0] as usize;
    if i >= len {
        return None;
    }

    let mut cur_ifc_class = 0u8;
    let mut cur_ifc_sub = 0u8;
    let mut cur_ifc_num;
    let mut out = UvcControlEntities::default();
    let mut found_vc = false;

    while i + 2 <= len {
        let Some(d) = Desc::new(cfg, i, len) else {
            break;
        };
        let ty = d.ty();

        if ty == device::USB_DT_INTERFACE && d.len >= 9 {
            cur_ifc_num = d.u8(2).unwrap_or(0);
            cur_ifc_class = d.u8(5).unwrap_or(cur_ifc_class);
            cur_ifc_sub = d.u8(6).unwrap_or(cur_ifc_sub);
            if cur_ifc_class == device::USB_CLASS_VIDEO && cur_ifc_sub == USB_SUBCLASS_VIDEO_CONTROL
            {
                out.vc_interface = cur_ifc_num;
                found_vc = true;
            }
        } else if ty == CS_INTERFACE
            && cur_ifc_class == device::USB_CLASS_VIDEO
            && cur_ifc_sub == USB_SUBCLASS_VIDEO_CONTROL
            && d.len >= 3
        {
            let st = d.u8(2).unwrap_or(0);
            match st {
                VC_INPUT_TERMINAL
                    // bLength=15+x，bUnitID@3, wTerminalType@4..6, bAssocTerm@6,
                    // 后续 wObjectiveFocalLengthMin/Max + wOcularFocalLength + bControlSize@14, bmControls@15..
                    if d.len >= 15 =>
                {
                    let id = d.u8(3).unwrap_or(0);
                    let tt = d.u16le(4).unwrap_or(0);
                    if tt == ITT_CAMERA {
                        out.camera_terminal_id = Some(id);
                        let csize = d.u8(14).unwrap_or(0) as usize;
                        let cmax = csize.min(d.len.saturating_sub(15)).min(4);
                        let mut bm = 0u32;
                        for k in 0..cmax {
                            if let Some(b) = d.u8(15 + k) {
                                bm |= (b as u32) << (8 * k);
                            }
                        }
                        out.ct_controls = bm;
                    }
                }
                VC_PROCESSING_UNIT
                    // bLength=10+n，bUnitID@3, bSourceID@4, wMaxMultiplier@5..7, bControlSize@7, bmControls@8..
                    if d.len >= 9 =>
                {
                    let id = d.u8(3).unwrap_or(0);
                    let csize = d.u8(7).unwrap_or(0) as usize;
                    let cmax = csize.min(d.len.saturating_sub(8)).min(4);
                    let mut bm = 0u32;
                    for k in 0..cmax {
                        if let Some(b) = d.u8(8 + k) {
                            bm |= (b as u32) << (8 * k);
                        }
                    }
                    out.processing_unit_id = Some(id);
                    out.pu_controls = bm;
                }
                _ => {}
            }
        }

        i += d.len;
    }

    if found_vc {
        Some(out)
    } else {
        None
    }
}
