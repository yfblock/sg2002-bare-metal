//! 摄像头 VideoControl 实体控制：ProcessingUnit（亮度/对比度/白平衡…）与
//! CameraTerminal（自动曝光/聚焦）。按 `bmControls` 逐项决定是否发送；
//! 不支持就跳过，STALL 只记日志、不致命。

use crate::drivers::usb::error::UsbResult;

use super::session::UvcCamera;
use super::setup::UvcRequest;

// ProcessingUnit selectors (wValue MSB)
const PU_BACKLIGHT_COMPENSATION: u8 = 0x01;
const PU_BRIGHTNESS_CONTROL: u8 = 0x02;
const PU_CONTRAST_CONTROL: u8 = 0x03;
const PU_GAIN_CONTROL: u8 = 0x04;
const PU_HUE_CONTROL: u8 = 0x06;
const PU_SATURATION_CONTROL: u8 = 0x07;
const PU_SHARPNESS_CONTROL: u8 = 0x08;
const PU_GAMMA_CONTROL: u8 = 0x09;
const PU_WHITE_BALANCE_TEMPERATURE_CONTROL: u8 = 0x0A;
const PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL: u8 = 0x0B;
const PU_HUE_AUTO_CONTROL: u8 = 0x10;
const PU_POWER_LINE_FREQUENCY_CONTROL: u8 = 0x05;

// CameraTerminal selectors
const CT_AE_MODE_CONTROL: u8 = 0x02;
const CT_AE_PRIORITY_CONTROL: u8 = 0x03;
const CT_FOCUS_AUTO_CONTROL: u8 = 0x08;

/// ProcessingUnit 单项 2 字节控制描述（selector / 名称）。
struct PuCtrl {
    bit: u32,
    selector: u8,
    name: &'static str,
}

impl UvcCamera {
    /// 控制传输出现**任何**错误（含 STALL）都返回 false，调用方按“不支持”降级/忽略。
    fn try_set_cur_u8(&self, entity: u8, selector: u8, value: u8) -> bool {
        let setup = UvcRequest::SetCurControl {
            interface: self.entities.vc_interface,
            entity_id: entity,
            selector,
            w_length: 1,
        }
        .build();
        let buf = [value];
        self.control_ep.write(setup, &buf).is_ok()
    }

    fn try_get_cur_u16(&self, entity: u8, selector: u8) -> Option<u16> {
        let setup = UvcRequest::GetCurControl {
            interface: self.entities.vc_interface,
            entity_id: entity,
            selector,
            w_length: 2,
        }
        .build();
        let mut buf = [0u8; 2];
        if self.control_ep.read(setup, &mut buf).is_ok() {
            Some(u16::from_le_bytes(buf))
        } else {
            None
        }
    }

    fn try_get_def_u16(&self, entity: u8, selector: u8) -> Option<u16> {
        let setup = UvcRequest::GetDefControl {
            interface: self.entities.vc_interface,
            entity_id: entity,
            selector,
            w_length: 2,
        }
        .build();
        let mut buf = [0u8; 2];
        if self.control_ep.read(setup, &mut buf).is_ok() {
            Some(u16::from_le_bytes(buf))
        } else {
            None
        }
    }

    fn try_set_cur_u16(&self, entity: u8, selector: u8, value: u16) -> bool {
        let setup = UvcRequest::SetCurControl {
            interface: self.entities.vc_interface,
            entity_id: entity,
            selector,
            w_length: 2,
        }
        .build();
        let buf = value.to_le_bytes();
        self.control_ep.write(setup, &buf).is_ok()
    }

    /// 把 ProcessingUnit 的 2 字节控制项设为 `GET_DEF` 出厂默认值。
    /// 只有 `bmControls` 标记支持的 selector 才会发送。
    fn pu_apply_one(&self, pu: u8, bm: u32, ctrl: PuCtrl) {
        if (bm & (1u32 << ctrl.bit)) == 0 {
            return;
        }
        let cur = self.try_get_cur_u16(pu, ctrl.selector);
        let want = self.try_get_def_u16(pu, ctrl.selector);
        match (cur, want) {
            (Some(c), Some(d)) if c != d => {
                let ok = self.try_set_cur_u16(pu, ctrl.selector, d);
                log::info!(
                    "UVC: PU.{} {c} -> {d} (def, {})",
                    ctrl.name,
                    if ok { "ok" } else { "set 失败" }
                );
            }
            (None, _) => log::warn!("UVC: PU.{} GET_CUR 失败", ctrl.name),
            _ => {}
        }
    }

    /// 摄像头初始化常用控制：**自动白平衡**、**自动曝光**、**关闭手动 Hue**、**Power-line 50Hz**、
    /// **Focus-Auto**。每一项都按 `bmControls` 决定是否发送，不支持就跳过；STALL 也只是日志，不致命。
    ///
    /// `0c45:64ab` 等 SunplusIT/Sonix 摄像头出厂在某些场景下默认白平衡是手动模式，
    /// 这是图像偏色（偏蓝/偏紫）的最常见原因。
    pub(crate) fn init_controls(&self) -> UsbResult<()> {
        let ent = &self.entities;
        log::info!(
            "UVC: VC if={} CT={:?} (bm={:#010x}) PU={:?} (bm={:#010x})",
            ent.vc_interface,
            ent.camera_terminal_id,
            ent.ct_controls,
            ent.processing_unit_id,
            ent.pu_controls
        );

        if let Some(pu) = ent.processing_unit_id {
            let bm = ent.pu_controls;

            // ① 图像调节参数：各项统一设为 GET_DEF 出厂默认。
            // PU bmControls 位定义（UVC 1.5）：
            //   D0=Brightness D1=Contrast D2=Hue D3=Saturation D4=Sharpness
            //   D5=Gamma D6=WB Temp D8=Backlight D9=Gain D10=PowerLineFreq
            //   D11=Hue Auto D12=WB Temp Auto
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 0,
                    selector: PU_BRIGHTNESS_CONTROL,
                    name: "Brightness",
                },
            );
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 1,
                    selector: PU_CONTRAST_CONTROL,
                    name: "Contrast",
                },
            );
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 2,
                    selector: PU_HUE_CONTROL,
                    name: "Hue",
                },
            );
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 3,
                    selector: PU_SATURATION_CONTROL,
                    name: "Saturation",
                },
            );
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 4,
                    selector: PU_SHARPNESS_CONTROL,
                    name: "Sharpness",
                },
            );
            // PU_GAMMA selector = 0x09
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 5,
                    selector: PU_GAMMA_CONTROL,
                    name: "Gamma",
                },
            );
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 8,
                    selector: PU_BACKLIGHT_COMPENSATION,
                    name: "Backlight",
                },
            );
            self.pu_apply_one(
                pu,
                bm,
                PuCtrl {
                    bit: 9,
                    selector: PU_GAIN_CONTROL,
                    name: "Gain",
                },
            );

            // ② 白平衡：默认走 Auto WB（若支持 D12）
            if (bm & (1 << 12)) != 0 {
                let _ = self.try_set_cur_u8(pu, PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL, 0);
                if (bm & (1 << 6)) != 0 {
                    if let Some(d) = self.try_get_def_u16(pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL)
                    {
                        let _ = self.try_set_cur_u16(pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL, d);
                    }
                }
                let _ = self.try_set_cur_u8(pu, PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL, 1);
                let cur_t = self
                    .try_get_cur_u16(pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL)
                    .unwrap_or(0);
                log::info!("UVC: PU.WB = Auto (cur {cur_t}K)");
            } else if (bm & (1 << 6)) != 0 {
                let val = self
                    .try_get_def_u16(pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL)
                    .unwrap_or(4500);
                let _ = self.try_set_cur_u16(pu, PU_WHITE_BALANCE_TEMPERATURE_CONTROL, val);
                log::info!("UVC: PU.WB = {val}K (no-auto)");
            }

            if (bm & (1 << 11)) != 0 {
                let _ = self.try_set_cur_u8(pu, PU_HUE_AUTO_CONTROL, 1);
            }

            // ③ Power-line 频率固定按 50Hz 抑制。
            if (bm & (1 << 10)) != 0 {
                let _ = self.try_set_cur_u8(pu, PU_POWER_LINE_FREQUENCY_CONTROL, 1);
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
                    if self.try_set_cur_u8(ct, CT_AE_MODE_CONTROL, mode) {
                        applied = mode;
                        break;
                    }
                }
                log::info!("UVC: CT.AeMode = {applied:#04x}");

                // AE priority 固定 1（允许自动曝光降帧率换曝光）。
                if (ent.ct_controls & (1 << 2)) != 0
                    && self.try_set_cur_u8(ct, CT_AE_PRIORITY_CONTROL, 1)
                {
                    log::info!("UVC: AE priority = 1");
                }
            }

            if (ent.ct_controls & (1 << 17)) != 0 {
                let _ = self.try_set_cur_u8(ct, CT_FOCUS_AUTO_CONTROL, 1);
            }
        }

        Ok(())
    }
}
