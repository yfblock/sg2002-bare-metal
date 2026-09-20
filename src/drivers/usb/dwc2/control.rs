//! EP0 控制传输：`Ep0` 端点句柄（绑定设备地址 + EP0 包长）+ 标准请求
//! （SET_ADDRESS / SET_CONFIGURATION / 设备描述符）与 Hub 端口请求包装，全部走通道 0。

use super::ch::{hcchar_control, hctsiz, Channel};
use super::dma::{dma_ptr, DMA_OFF_SMALL_IO, OFF_EP0};
use super::regs::HCTSIZ;
use crate::arch::cache;
use crate::drivers::usb::error::{UsbError, UsbResult};
use crate::drivers::usb::setup;

/// EP0 控制端点句柄：把控制传输的公共前置参数（设备地址、EP0 最大包长）
/// 绑定为端点自身状态。
#[derive(Clone, Copy)]
pub struct Ep0 {
    dev: u32,
    mps: u32,
}

impl Ep0 {
    /// 绑定一台已寻址设备的 EP0。
    ///
    /// # 参数
    /// - `dev`：设备 USB 地址（7 位数值）。
    /// - `ep0_mps`：该设备 EP0 最大包长（8/16/32/64）。
    #[inline]
    pub fn new(dev: u32, ep0_mps: u32) -> Self {
        Self { dev, mps: ep0_mps }
    }

    /// 设备 USB 地址。
    #[inline]
    pub fn dev(&self) -> u32 {
        self.dev
    }

    /// 枚举首步：在默认地址 **0** 上读 `GET_DESCRIPTOR(DEVICE, 18)`。
    ///
    /// # 返回值
    /// `(vid, pid, ep0_mps, b_device_class)`，均在设备描述符前 18 字节内解析。
    pub fn probe_default_addr() -> UsbResult<(u16, u16, u32, u8)> {
        unsafe {
            let wlen: u16 = 18;
            let setup_pkt = setup::get_descriptor_device();
            core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
            cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

            let mut hc = hcchar_control(0, 0, 64, false);
            Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Setup, 1, 8), OFF_EP0 as u32)?;

            hc = hcchar_control(0, 0, 64, true);
            Channel::CONTROL.xfer(
                hc,
                hctsiz(HCTSIZ::PID::Data1, 1, wlen as u32),
                OFF_EP0 as u32,
            )?;
            cache::dcache_invalidate_after_dma(dma_ptr().add(OFF_EP0), wlen as usize);

            let sl = core::slice::from_raw_parts(dma_ptr().add(OFF_EP0), wlen as usize);
            if sl.len() < 12 {
                return Err(UsbError::Protocol("short descriptor"));
            }
            let vid = u16::from_le_bytes([sl[8], sl[9]]);
            let pid = u16::from_le_bytes([sl[10], sl[11]]);
            let ep0_mps = normalize_ep0_mps(sl[7]);
            let b_device_class = sl[4];

            hc = hcchar_control(0, 0, 64, false);
            Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Data1, 1, 0), OFF_EP0 as u32)?;

            Ok((vid, pid, ep0_mps, b_device_class))
        }
    }

    /// 在默认地址 0 上发送 `SET_ADDRESS`（须在 [`Ep0::new`] 建立句柄之前）。
    ///
    /// # 参数
    /// - `addr`：设备新地址，合法 **1..=127**。
    /// - `ep0_mps`：地址 0 阶段使用的 EP0 MPS（枚举首步常用 64）。
    pub fn set_address(addr: u8, ep0_mps: u32) -> UsbResult<()> {
        write_no_data_raw(0, setup::set_address(addr), ep0_mps)
    }

    /// 对已寻址设备发送 `SET_CONFIGURATION`。
    ///
    /// # 参数
    /// - `cfg`：`bConfigurationValue`（通常非 0 表示激活配置）。
    pub fn set_configuration(&self, cfg: u8) -> UsbResult<()> {
        self.write_no_data(setup::set_configuration(cfg))
    }

    /// 对 **已寻址** Hub 发送 `SET_PORT_FEATURE`（无数据阶段）。
    ///
    /// # 参数
    /// - `port`：下游端口号（从 1 开始）。
    /// - `feature`：Hub 端口特性选择子（如 [`setup::HUB_PORT_FEATURE_POWER`]）。
    pub fn hub_set_port_feature(&self, port: u16, feature: u16) -> UsbResult<()> {
        self.write_no_data(setup::hub_set_port_feature(port, feature))
    }

    /// 对 **已寻址** Hub 发送 `CLEAR_PORT_FEATURE`（清除 `C_PORT_*` 等变化位）。
    pub fn hub_clear_port_feature(&self, port: u16, feature: u16) -> UsbResult<()> {
        self.write_no_data(setup::hub_clear_port_feature(port, feature))
    }

    /// 无数据阶段控制传输：`SETUP` + `STATUS` IN（零长度）。
    pub fn write_no_data(&self, setup_pkt: [u8; 8]) -> UsbResult<()> {
        write_no_data_raw(self.dev, setup_pkt, self.mps)
    }

    /// 控制读：SETUP + 若干 IN 数据包（DATA1/DATA0 交替）+ STATUS OUT。
    ///
    /// 适用于 Hub 描述符、配置前缀、`GET_PORT_STATUS`、UVC 配置描述符 / PROBE-COMMIT 等。
    ///
    /// # 参数
    /// - `setup_pkt`：8 字节 SETUP（`wLength` 应等于 `out.len()` 的期望读长）。
    /// - `out`：接收缓冲区，长度须与 SETUP 中 `wLength` 一致且在 `(0,4096]`。
    pub fn read(&self, setup_pkt: [u8; 8], out: &mut [u8]) -> UsbResult<()> {
        if out.is_empty() || out.len() > 4096 {
            return Err(UsbError::Protocol("bad ep0 read len"));
        }
        let total = out.len() as u32;
        let (dev, mps) = (self.dev, self.mps);
        unsafe {
            core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
            cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

            let mut hc = hcchar_control(dev, 0, mps, false);
            Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Setup, 1, 8), OFF_EP0 as u32)?;

            let mut left = total;
            let mut out_off: usize = 0;
            let mut data1 = true;
            while left > 0 {
                let chunk = left.min(mps);
                hc = hcchar_control(dev, 0, mps, true);
                let pid = if data1 {
                    HCTSIZ::PID::Data1
                } else {
                    HCTSIZ::PID::Data0
                };
                Channel::CONTROL.xfer(hc, hctsiz(pid, 1, chunk), DMA_OFF_SMALL_IO as u32)?;
                cache::dcache_invalidate_after_dma(dma_ptr().add(DMA_OFF_SMALL_IO), chunk as usize);
                core::ptr::copy_nonoverlapping(
                    dma_ptr().add(DMA_OFF_SMALL_IO),
                    out.as_mut_ptr().add(out_off),
                    chunk as usize,
                );
                out_off += chunk as usize;
                left -= chunk;
                data1 = !data1;
            }

            hc = hcchar_control(dev, 0, mps, false);
            Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Data1, 1, 0), OFF_EP0 as u32)?;
            Ok(())
        }
    }

    /// 控制写：`SETUP` + `DATA` OUT（可多包）+ `STATUS` IN（ZLP）。
    ///
    /// # 参数
    /// - `setup_pkt`：8 字节 SETUP（`wLength` 应等于 `data.len()`）。
    /// - `data`：OUT 数据阶段负载（最大 4096 字节）。
    pub fn write(&self, setup_pkt: [u8; 8], data: &[u8]) -> UsbResult<()> {
        if data.len() > 4096 {
            return Err(UsbError::Protocol("bad ep0 write data len"));
        }
        let (dev, mps) = (self.dev, self.mps);
        unsafe {
            core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
            cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

            let mut hc = hcchar_control(dev, 0, mps, false);
            Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Setup, 1, 8), OFF_EP0 as u32)?;

            let mut left = data.len() as u32;
            let mut src: usize = 0;
            let mut data1 = true;
            while left > 0 {
                let chunk = left.min(mps);
                core::ptr::copy_nonoverlapping(
                    data.as_ptr().add(src),
                    dma_ptr().add(DMA_OFF_SMALL_IO),
                    chunk as usize,
                );
                cache::dcache_clean_for_dma(dma_ptr().add(DMA_OFF_SMALL_IO), chunk as usize);
                hc = hcchar_control(dev, 0, mps, false);
                let pid = if data1 {
                    HCTSIZ::PID::Data1
                } else {
                    HCTSIZ::PID::Data0
                };
                Channel::CONTROL.xfer(hc, hctsiz(pid, 1, chunk), DMA_OFF_SMALL_IO as u32)?;
                src += chunk as usize;
                left -= chunk;
                data1 = !data1;
            }

            hc = hcchar_control(dev, 0, mps, true);
            Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Data1, 1, 0), OFF_EP0 as u32)?;
            Ok(())
        }
    }
}

#[inline]
fn normalize_ep0_mps(b: u8) -> u32 {
    match b {
        8 | 16 | 32 | 64 => b as u32,
        _ => 8,
    }
}

/// 无数据阶段控制传输的内部实现（供 `Ep0::set_address` 的地址 0 特例使用）。
fn write_no_data_raw(dev: u32, setup_pkt: [u8; 8], mps: u32) -> UsbResult<()> {
    unsafe {
        core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
        cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);

        let hc = hcchar_control(dev, 0, mps, false);
        Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Setup, 1, 8), OFF_EP0 as u32)?;

        let hc = hcchar_control(dev, 0, mps, true);
        Channel::CONTROL.xfer(hc, hctsiz(HCTSIZ::PID::Data1, 1, 0), OFF_EP0 as u32)?;
        Ok(())
    }
}
