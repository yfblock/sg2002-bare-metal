//! VS 流协商：`PROBE` → `GET_CUR` → `COMMIT` → `SET_INTERFACE`。

use crate::drivers::usb::error::UsbResult;
use crate::drivers::usb::dwc2;
use crate::drivers::usb::setup::{std_setup, StdRequest};

use super::descriptor::{reselect_isoch_alt_for_payload, UvcStreamSelection};
use super::setup::{uvc_setup, UvcRequest};

const VS_PROBE_CONTROL: u8 = 0x01;
const VS_COMMIT_CONTROL: u8 = 0x02;

const UVC_PROBE_COMMIT_LEN: usize = 34;

fn build_probe_commit_payload(sel: &UvcStreamSelection) -> [u8; UVC_PROBE_COMMIT_LEN] {
    let mut buf = [0u8; UVC_PROBE_COMMIT_LEN];
    // 实测 0c45:64ab 等廉价 webcam 不论我们 wCompQuality 设几（1/47/10000），
    // 一旦 bmHint.D3=1 锁定 wCompQuality 就会切到"低质量量化表"（吐 ~21K），
    // 反而比不锁定（吐 ~26K）糟糕。所以这里只锁定 frame interval (D0=1)，
    // 不锁定 wCompQuality (D3=0)，让设备用出厂默认 quality。
    buf[0] = 0x01;
    buf[1] = 0x00;
    buf[2] = sel.format_index;
    buf[3] = sel.frame_index;
    buf[4..8].copy_from_slice(&sel.frame_interval.to_le_bytes());
    let w = u32::from(sel.frame_w.max(640));
    let h = u32::from(sel.frame_h.max(480));
    let est = if sel.is_mjpeg { w.saturating_mul(h) } else { w.saturating_mul(h).saturating_mul(2) };
    buf[18..22].copy_from_slice(&est.to_le_bytes());
    let pkt_total = dwc2::wmax_payload_per_uframe(sel.mps_raw);
    buf[22..26].copy_from_slice(&pkt_total.to_le_bytes());
    buf
}

fn dump_probe(prefix: &str, p: &[u8]) {
    if p.len() < 26 { return; }
    let bm_hint = u16::from_le_bytes([p[0], p[1]]);
    let fmt_ix = p[2];
    let frame_ix = p[3];
    let interval = u32::from_le_bytes([p[4], p[5], p[6], p[7]]);
    let key_frm = u16::from_le_bytes([p[8], p[9]]);
    let pframe = u16::from_le_bytes([p[10], p[11]]);
    let comp_q = u16::from_le_bytes([p[12], p[13]]);
    let comp_w = u16::from_le_bytes([p[14], p[15]]);
    let delay = u16::from_le_bytes([p[16], p[17]]);
    let max_video = u32::from_le_bytes([p[18], p[19], p[20], p[21]]);
    let max_pkt = u32::from_le_bytes([p[22], p[23], p[24], p[25]]);
    log::info!("UVC: {prefix} bmHint={bm_hint:#06x} fmt={fmt_ix} frame={frame_ix} iv={interval} keyFrm={key_frm} pFrm={pframe} compQ={comp_q} compW={comp_w} delay={delay} dwMaxVideoFrameSize={max_video} dwMaxPayloadTransferSize={max_pkt}");
}

/// `PROBE` → `GET_CUR` → `COMMIT` → `SET_INTERFACE`。
///
/// 协商后会更新 `sel.negotiated_payload_size`，并依据
/// 协商出的 `dwMaxPayloadTransferSize` **重新选择最匹配的 alt setting**（避免 mps 切包错位）。
pub fn uvc_start_video_stream(ep: &dwc2::Ep0, sel: &mut UvcStreamSelection) -> UsbResult<()> {
    let _ = ep.write_no_data(std_setup(StdRequest::SetInterface { alt: 0, interface: sel.vs_interface }));

    let probe_init = build_probe_commit_payload(sel);
    dump_probe("PROBE.SET", &probe_init);

    ep.write(
        uvc_setup(UvcRequest::SetCurStreaming {
            interface: sel.vs_interface,
            selector: VS_PROBE_CONTROL,
            w_length: UVC_PROBE_COMMIT_LEN as u16,
        }),
        &probe_init,
    )?;

    let mut probe_max = [0u8; UVC_PROBE_COMMIT_LEN];
    if ep
        .read(
            uvc_setup(UvcRequest::GetMaxStreaming {
                interface: sel.vs_interface,
                selector: VS_PROBE_CONTROL,
                w_length: UVC_PROBE_COMMIT_LEN as u16,
            }),
            &mut probe_max,
        )
        .is_ok()
    {
        dump_probe("PROBE.MAX", &probe_max);
    }

    let mut probe = [0u8; UVC_PROBE_COMMIT_LEN];
    ep.read(
        uvc_setup(UvcRequest::GetCurStreaming {
            interface: sel.vs_interface,
            selector: VS_PROBE_CONTROL,
            w_length: UVC_PROBE_COMMIT_LEN as u16,
        }),
        &mut probe,
    )?;
    dump_probe("PROBE.CUR", &probe);

    sel.negotiated_payload_size = u32::from_le_bytes([probe[22], probe[23], probe[24], probe[25]]);
    let negotiated_frame_size = u32::from_le_bytes([probe[18], probe[19], probe[20], probe[21]]);

    // 根据协商出的 dwMaxPayloadTransferSize 重新选 Isoch alt。
    reselect_isoch_alt_for_payload(sel);

    // 若 reselect 降级到了更低带宽的 alt（例如 mult=1），须把 COMMIT 中的
    // dwMaxPayloadTransferSize 压到该 alt 的实际每微帧吞吐，否则摄像头
    // 按 3060 B 分包、主机只收 1020 B/uframe 会导致数据截断。
    let alt_mps = dwc2::wmax_payload_per_uframe(sel.mps_raw);
    if alt_mps > 0 && alt_mps < sel.negotiated_payload_size {
        log::info!("UVC: clamping COMMIT dwMaxPayloadTransferSize {} -> {} to match alt bandwidth",
            sel.negotiated_payload_size, alt_mps);
        sel.negotiated_payload_size = alt_mps;
        probe[22..26].copy_from_slice(&alt_mps.to_le_bytes());
    }

    ep.write(
        uvc_setup(UvcRequest::SetCurStreaming {
            interface: sel.vs_interface,
            selector: VS_COMMIT_CONTROL,
            w_length: UVC_PROBE_COMMIT_LEN as u16,
        }),
        &probe,
    )?;

    ep.write_no_data(std_setup(StdRequest::SetInterface { alt: sel.alt_setting, interface: sel.vs_interface }))?;

    log::info!("UVC: streaming armed if={} alt={} negotiated_payload={} frame_size={}",
        sel.vs_interface, sel.alt_setting, sel.negotiated_payload_size, negotiated_frame_size);

    Ok(())
}

