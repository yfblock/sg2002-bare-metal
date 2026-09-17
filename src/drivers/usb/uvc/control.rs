//! 摄像头 VideoControl 实体控制：ProcessingUnit（亮度/对比度/白平衡…）与
//! CameraTerminal（自动曝光/聚焦）。按 `bmControls` 逐项决定是否发送；
//! 不支持就跳过，STALL 只记日志、不致命。

use crate::drivers::usb::error::UsbResult;
use crate::drivers::usb::dwc2;

use super::descriptor::UvcControlEntities;
use super::setup::{uvc_get_cur_vc, uvc_get_def_vc, uvc_set_cur_vc};

// ProcessingUnit selectors (wValue MSB)
#[allow(dead_code)] const PU_BACKLIGHT_COMPENSATION: u8 = 0x01;
#[allow(dead_code)] const PU_BRIGHTNESS_CONTROL: u8 = 0x02;
#[allow(dead_code)] const PU_CONTRAST_CONTROL: u8 = 0x03;
#[allow(dead_code)] const PU_GAIN_CONTROL: u8 = 0x04;
#[allow(dead_code)] const PU_HUE_CONTROL: u8 = 0x06;
#[allow(dead_code)] const PU_SATURATION_CONTROL: u8 = 0x07;
#[allow(dead_code)] const PU_SHARPNESS_CONTROL: u8 = 0x08;
const PU_WHITE_BALANCE_TEMPERATURE_CONTROL: u8 = 0x0A;
const PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL: u8 = 0x0B;
const PU_HUE_AUTO_CONTROL: u8 = 0x10;
const PU_POWER_LINE_FREQUENCY_CONTROL: u8 = 0x05;

// CameraTerminal selectors
const CT_AE_MODE_CONTROL: u8 = 0x02;
const CT_AE_PRIORITY_CONTROL: u8 = 0x03;
#[allow(dead_code)] const CT_EXPOSURE_TIME_ABSOLUTE_CONTROL: u8 = 0x04;
const CT_FOCUS_AUTO_CONTROL: u8 = 0x08;

/// 仅在控制传输出现 STALL 时返回 false，其它错误则当成"不支持"忽略。
fn try_set_cur_u8(
    ep: &dwc2::Ep0,
    vc_if: u8,
    entity: u8,
    selector: u8,
    value: u8,
) -> bool {
    let setup = uvc_set_cur_vc(vc_if, entity, selector, 1);
    let buf = [value];
    ep.write(setup, &buf).is_ok()
}

#[allow(dead_code)]
fn try_get_cur_u8(
    ep: &dwc2::Ep0,
    vc_if: u8,
    entity: u8,
    selector: u8,
) -> Option<u8> {
    let setup = uvc_get_cur_vc(vc_if, entity, selector, 1);
    let mut buf = [0u8; 1];
    if ep.read(setup, &mut buf).is_ok() {
        Some(buf[0])
    } else {
        None
    }
}

#[allow(dead_code)]
fn try_get_cur_u16(
    ep: &dwc2::Ep0,
    vc_if: u8,
    entity: u8,
    selector: u8,
) -> Option<u16> {
    let setup = uvc_get_cur_vc(vc_if, entity, selector, 2);
    let mut buf = [0u8; 2];
    if ep.read(setup, &mut buf).is_ok() {
        Some(u16::from_le_bytes(buf))
    } else {
        None
    }
}

fn try_get_def_u8(
    ep: &dwc2::Ep0,
    vc_if: u8,
    entity: u8,
    selector: u8,
) -> Option<u8> {
    let setup = uvc_get_def_vc(vc_if, entity, selector, 1);
    let mut buf = [0u8; 1];
    if ep.read(setup, &mut buf).is_ok() {
        Some(buf[0])
    } else {
        None
    }
}

fn try_get_def_u16(
    ep: &dwc2::Ep0,
    vc_if: u8,
    entity: u8,
    selector: u8,
) -> Option<u16> {
    let setup = uvc_get_def_vc(vc_if, entity, selector, 2);
    let mut buf = [0u8; 2];
    if ep.read(setup, &mut buf).is_ok() {
        Some(u16::from_le_bytes(buf))
    } else {
        None
    }
}

fn try_set_cur_u16(
    ep: &dwc2::Ep0,
    vc_if: u8,
    entity: u8,
    selector: u8,
    value: u16,
) -> bool {
    let setup = uvc_set_cur_vc(vc_if, entity, selector, 2);
    let buf = value.to_le_bytes();
    ep.write(setup, &buf).is_ok()
}

/// ProcessingUnit 单项控制描述（selector / 宽度 / 可选覆盖值）。
struct PuCtrl {
    bit: u32,
    selector: u8,
    width: u8,
    name: &'static str,
    override_val: Option<u16>,
}

/// 把 ProcessingUnit 的 1/2 字节控制项设到 `override_val`；为 `None` 则用 `GET_DEF`。
/// 只有 `bmControls` 标记支持的 selector 才会发送。
fn pu_apply_one(ep: &dwc2::Ep0, vc_if: u8, pu: u8, bm: u32, ctrl: PuCtrl) {
    if (bm & (1u32 << ctrl.bit)) == 0 {
        return;
    }
    let want_src = ctrl.override_val.map(|_| "override").unwrap_or("def");
    match ctrl.width {
        1 => {
            let cur = try_get_cur_u8(ep, vc_if, pu, ctrl.selector);
            let want = match ctrl.override_val {
                Some(v) => Some(v as u8),
                None => try_get_def_u8(ep, vc_if, pu, ctrl.selector),
            };
            match (cur, want) {
                (Some(c), Some(d)) if c != d => {
                    let ok = try_set_cur_u8(ep, vc_if, pu, ctrl.selector, d);
                    log::info!(
                        "UVC: PU.{} {c} -> {d} ({want_src}, {})",
                        ctrl.name,
                        if ok { "ok" } else { "set 失败" }
                    );
                }
                (None, _) => log::warn!("UVC: PU.{} GET_CUR 失败", ctrl.name),
                _ => {}
            }
        }
        2 => {
            let cur = try_get_cur_u16(ep, vc_if, pu, ctrl.selector);
            let want = match ctrl.override_val {
                Some(v) => Some(v),
                None => try_get_def_u16(ep, vc_if, pu, ctrl.selector),
            };
            match (cur, want) {
                (Some(c), Some(d)) if c != d => {
                    let ok = try_set_cur_u16(ep, vc_if, pu, ctrl.selector, d);
                    log::info!(
                        "UVC: PU.{} {c} -> {d} ({want_src}, {})",
                        ctrl.name,
                        if ok { "ok" } else { "set 失败" }
                    );
                }
                (None, _) => log::warn!("UVC: PU.{} GET_CUR 失败", ctrl.name),
                _ => {}
            }
        }
        _ => {}
    }
}

/// 上层可选的图像调节覆盖。每一项 `Some(v)` 表示用 `v` SET_CUR 覆盖摄像头出厂默认；
/// `None` 表示沿用 `GET_DEF` 出厂默认。
///
/// 使用例：把 `brightness=Some(96)` 让画面比 def 的 128 暗一些。
#[derive(Clone, Copy, Debug, Default)]
pub struct UvcImageTuning {
    pub brightness: Option<u16>,
    pub contrast: Option<u16>,
    pub hue: Option<u16>,
    pub saturation: Option<u16>,
    pub sharpness: Option<u16>,
    pub gamma: Option<u16>,
    pub backlight: Option<u16>,
    pub gain: Option<u16>,
    /// `Some(K)` = 关 Auto WB、用手动色温 K（典型 2800–6500）。`None` = 开 Auto WB。
    pub white_balance_temp_k: Option<u16>,
    /// 0=Disabled, 1=50Hz, 2=60Hz；`None` 时按 50Hz 设置。
    pub power_line_freq: Option<u8>,
    /// `CT_AE_PRIORITY_CONTROL`：**0 = 帧率必须恒定**，1 = 允许自动曝光降帧率换曝光。
    ///
    /// `None` 沿用历史行为（1）。注意设 1 时摄像头可能远低于描述符声称的帧率——
    /// 实测本机 0c45:64ab 声称 60fps，AE priority=1 时只给 16.7fps。
    /// 需要稳定高帧率就设 `Some(0)`（弱光下画面会变暗）。
    pub ae_priority: Option<u8>,
    /// 曝光时间上限(100µs 单位)。`Some(v)` = 设置 CT_EXPOSURE_TIME_ABSOLUTE(部分
    /// 摄像头在 AE 自动模式下也接受此值作为上限)。`None` = 不设。
    /// 例:Some(300) = 最大 30ms(对应 30fps 帧周期的 90%)。
    pub exposure_time_max: Option<u32>,
}

/// 摄像头初始化常用控制：**自动白平衡**、**自动曝光**、**关闭手动 Hue**、**Power-line 50Hz**、
/// **Focus-Auto**。每一项都按 `bmControls` 决定是否发送，不支持就跳过；STALL 也只是日志，不致命。
///
/// `tune` 中 `Some(v)` 字段会用 `v` 覆盖出厂 def，`None` 字段则沿用 def。
///
/// `0c45:64ab` 等 SunplusIT/Sonix 摄像头出厂在某些场景下默认白平衡是手动模式，
/// 这是图像偏色（偏蓝/偏紫）的最常见原因。
pub fn uvc_init_camera_controls(
    ep: &dwc2::Ep0,
    ent: &UvcControlEntities,
    tune: &UvcImageTuning,
) -> UsbResult<()> {
    log::info!(
        "UVC: VC if={} CT={:?} (bm={:#010x}) PU={:?} (bm={:#010x})",
        ent.vc_interface,
        ent.camera_terminal_id,
        ent.ct_controls,
        ent.processing_unit_id,
        ent.pu_controls
    );

    if let Some(pu) = ent.processing_unit_id {
        let vc_if = ent.vc_interface;
        let bm = ent.pu_controls;

        // ① 图像调节参数：tune.* 为 Some 则覆盖；None 则用 GET_DEF。
        // PU bmControls 位定义（UVC 1.5）：
        //   D0=Brightness D1=Contrast D2=Hue D3=Saturation D4=Sharpness
        //   D5=Gamma D6=WB Temp D8=Backlight D9=Gain D10=PowerLineFreq
        //   D11=Hue Auto D12=WB Temp Auto
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 0, selector: PU_BRIGHTNESS_CONTROL, width: 2, name: "Brightness", override_val: tune.brightness });
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 1, selector: PU_CONTRAST_CONTROL, width: 2, name: "Contrast", override_val: tune.contrast });
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 2, selector: PU_HUE_CONTROL, width: 2, name: "Hue", override_val: tune.hue });
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 3, selector: PU_SATURATION_CONTROL, width: 2, name: "Saturation", override_val: tune.saturation });
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 4, selector: PU_SHARPNESS_CONTROL, width: 2, name: "Sharpness", override_val: tune.sharpness });
        // PU_GAMMA selector = 0x09
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 5, selector: 0x09, width: 2, name: "Gamma", override_val: tune.gamma });
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 8, selector: PU_BACKLIGHT_COMPENSATION, width: 2, name: "Backlight", override_val: tune.backlight });
        pu_apply_one(ep, vc_if, pu, bm, PuCtrl { bit: 9, selector: PU_GAIN_CONTROL, width: 2, name: "Gain", override_val: tune.gain });

        // ② 白平衡
        match tune.white_balance_temp_k {
            // 用户指定手动色温 → 关 Auto，写手动 K
            Some(k) if (bm & (1 << 6)) != 0 => {
                if (bm & (1 << 12)) != 0 {
                    let _ = try_set_cur_u8(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL, 0);
                }
                let _ = try_set_cur_u16(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL, k);
                log::info!("UVC: PU.WB = {k}K (manual)");
            }
            // 默认走 Auto WB（若支持 D12）
            _ => {
                if (bm & (1 << 12)) != 0 {
                    let _ = try_set_cur_u8(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL, 0);
                    if (bm & (1 << 6)) != 0 {
                        if let Some(d) = try_get_def_u16(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL) {
                            let _ = try_set_cur_u16(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL, d);
                        }
                    }
                    let _ = try_set_cur_u8(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL, 1);
                    let cur_t = try_get_cur_u16(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL).unwrap_or(0);
                    log::info!("UVC: PU.WB = Auto (cur {cur_t}K)");
                } else if (bm & (1 << 6)) != 0 {
                    let val = try_get_def_u16(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL).unwrap_or(4500);
                    let _ = try_set_cur_u16(ep, vc_if, pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL, val);
                    log::info!("UVC: PU.WB = {val}K (no-auto)");
                }
            }
        }

        if (bm & (1 << 11)) != 0 {
            let _ = try_set_cur_u8(ep, vc_if, pu, PU_HUE_AUTO_CONTROL, 1);
        }

        if (bm & (1 << 10)) != 0 {
            let plf = tune.power_line_freq.unwrap_or(1);
            let _ = try_set_cur_u8(ep, vc_if, pu, PU_POWER_LINE_FREQUENCY_CONTROL, plf);
        }
    }

    if let Some(ct) = ent.camera_terminal_id {
        // CT bmControls：D1=AE Mode, D2=AE Priority, D17=Focus Auto
        if (ent.ct_controls & (1 << 1)) != 0 {
            // AE Mode 是位掩码（UVC 1.5）：0x01=Manual, 0x02=Auto,
            // 0x04=Shutter Priority, 0x08=Aperture Priority。
            // 廉价 webcam 多数只接受 0x08（光圈优先=自动曝光），先试 0x02 失败则降级。
            let mut applied = 0u8;
            for &mode in &[0x02u8, 0x08, 0x04] {
                if try_set_cur_u8(ep, ent.vc_interface, ct, CT_AE_MODE_CONTROL, mode) {
                    applied = mode;
                    break;
                }
            }
            log::info!("UVC: CT.AeMode = {applied:#04x}");

            if (ent.ct_controls & (1 << 2)) != 0 {
                let prio = tune.ae_priority.unwrap_or(1);
                if try_set_cur_u8(
                    ep,
                    ent.vc_interface,
                    ct,
                    CT_AE_PRIORITY_CONTROL,
                    prio,
                ) {
                    log::info!("UVC: AE priority = {}", prio);
                }
                // 曝光时间上限:限制 AE 的最大曝光,防止帧率下降
                if let Some(max_100us) = tune.exposure_time_max {
                    let setup = uvc_set_cur_vc(ent.vc_interface, ct, CT_EXPOSURE_TIME_ABSOLUTE_CONTROL, 4);
                    let buf = max_100us.to_le_bytes();
                    if ep.write(setup, &buf).is_ok() {
                        log::info!("UVC: exposure time max = {} ({}ms)", max_100us, max_100us / 10);
                    } else {
                        log::debug!("UVC: exposure time max not supported (auto mode may ignore it)");
                    }
                    log::info!("UVC: CT.AePriority = {prio} (0=帧率恒定, 1=允许降帧率)");
                }
            }
        }

        if (ent.ct_controls & (1 << 17)) != 0 {
            let _ = try_set_cur_u8(ep, ent.vc_interface, ct, CT_FOCUS_AUTO_CONTROL, 1);
        }
    }

    Ok(())
}
