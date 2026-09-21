//! EP0 控制传输：`Ep0` 端点句柄（绑定设备地址 + EP0 包长）+ 标准请求
//! （SET_ADDRESS / SET_CONFIGURATION / 设备描述符）与 Hub 端口请求包装，全部走通道 0。

use super::ch::Channel;
use super::dma::{dma_ptr, DMA_OFF_SMALL_IO, OFF_EP0};
use super::regs::{HCCHAR, HCTSIZ};
use tock_registers::fields::FieldValue;
use crate::arch::cache;
use crate::drivers::usb::error::{UsbError, UsbResult};
use crate::drivers::usb::setup::{std_setup, StdRequest};

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

    /// 本端点的 HCCHAR(EP 恒 0;方向由 dir_in 定)。
    fn hcchar(&self, dir_in: bool) -> FieldValue<u32, HCCHAR::Register> {
        let mut field = HCCHAR::MPS.val(self.mps)
            + HCCHAR::EPNUM.val(0)
            + HCCHAR::DEVADDR.val(self.dev)
            + HCCHAR::EPTYPE::Control;
        if dir_in {
            field = field + HCCHAR::EPDIR::SET;
        }
        field
    }

    /// SETUP 阶段:8 字节请求拷入 EP0 DMA 窗,clean 后发出。
    fn setup_stage(&self, setup_pkt: &[u8; 8]) -> UsbResult<()> {
        unsafe {
            core::ptr::copy_nonoverlapping(setup_pkt.as_ptr(), dma_ptr().add(OFF_EP0), 8);
            cache::dcache_clean_for_dma(dma_ptr().add(OFF_EP0), 8);
            Channel::CONTROL.xfer(
                self.hcchar(false),
                HCTSIZ::PID::Setup + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(8),
                OFF_EP0 as u32,
            )?;
        }
        Ok(())
    }

    /// STATUS 零长阶段;`dir_in` 与数据阶段方向相反。
    fn status_stage(&self, dir_in: bool) -> UsbResult<()> {
        unsafe {
            Channel::CONTROL.xfer(
                self.hcchar(dir_in),
                HCTSIZ::PID::Data1 + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(0),
                OFF_EP0 as u32,
            )?;
        }
        Ok(())
    }

    /// 枚举首步：在默认地址 **0** 上读 `GET_DESCRIPTOR(DEVICE, 18)`。
    ///
    /// # 返回值
    /// `(vid, pid, ep0_mps, b_device_class)`，均在设备描述符前 18 字节内解析。
    pub fn probe_default_addr() -> UsbResult<(u16, u16, u32, u8)> {
        // 默认地址 0 阶段的临时句柄(MPS 固定 64,USB 2.0 枚举惯例)。
        let ep0 = Ep0 { dev: 0, mps: 64 };
        unsafe {
            let wlen: u16 = 18;
            ep0.setup_stage(&std_setup(StdRequest::GetDescriptorDevice))?;

            Channel::CONTROL.xfer(
                ep0.hcchar(true),
                HCTSIZ::PID::Data1 + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(wlen as u32),
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

            ep0.status_stage(false)?;

            Ok((vid, pid, ep0_mps, b_device_class))
        }
    }

    /// 在默认地址 0 上发送 `SET_ADDRESS`（须在 [`Ep0::new`] 建立句柄之前）。
    ///
    /// # 参数
    /// - `addr`：设备新地址，合法 **1..=127**。
    /// - `ep0_mps`：地址 0 阶段使用的 EP0 MPS（枚举首步常用 64）。
    pub fn set_address(addr: u8, ep0_mps: u32) -> UsbResult<()> {
        Ep0 { dev: 0, mps: ep0_mps }.write_no_data(std_setup(StdRequest::SetAddress { addr }))
    }

    /// 对已寻址设备发送 `SET_CONFIGURATION`。
    ///
    /// # 参数
    /// - `cfg`：`bConfigurationValue`（通常非 0 表示激活配置）。
    pub fn set_configuration(&self, cfg: u8) -> UsbResult<()> {
        self.write_no_data(std_setup(StdRequest::SetConfiguration { cfg }))
    }

    /// `GET_DESCRIPTOR(Configuration)` 标准两段式:先读 9 字节头取
    /// `wTotalLength`,再读全量(上限 4096)。纯标准请求组合,归本类型。
    pub fn get_configuration_descriptor(&self, cfg_index: u8) -> UsbResult<[u8; 4096]> {
        const USB_DT_CONFIGURATION: u8 = 2;
        let mut hdr = [0u8; 9];
        self.read(
            std_setup(StdRequest::GetDescriptorConfiguration { cfg_index, w_length: 9 }),
            &mut hdr,
        )?;
        if hdr[1] != USB_DT_CONFIGURATION {
            return Err(UsbError::Protocol("not a configuration descriptor"));
        }
        let total = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;
        if total > 4096 {
            return Err(UsbError::Protocol("configuration descriptor too large (>4096)"));
        }
        let mut buf = [0u8; 4096];
        self.read(
            std_setup(StdRequest::GetDescriptorConfiguration {
                cfg_index,
                w_length: total as u16,
            }),
            &mut buf[..total],
        )?;
        Ok(buf)
    }

    /// 无数据阶段控制传输：`SETUP` + `STATUS` IN（零长度）。
    pub fn write_no_data(&self, setup_pkt: [u8; 8]) -> UsbResult<()> {
        self.setup_stage(&setup_pkt)?;
        self.status_stage(true)
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
        self.setup_stage(&setup_pkt)?;
        unsafe {
            let mut left = total;
            let mut out_off: usize = 0;
            let mut data1 = true;
            while left > 0 {
                let chunk = left.min(self.mps);
                let hc = self.hcchar(true);
                let pid = if data1 {
                    HCTSIZ::PID::Data1
                } else {
                    HCTSIZ::PID::Data0
                };
                Channel::CONTROL.xfer(hc, pid + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(chunk), DMA_OFF_SMALL_IO as u32)?;
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

            self.status_stage(false)
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
        self.setup_stage(&setup_pkt)?;
        unsafe {
            let mut left = data.len() as u32;
            let mut src: usize = 0;
            let mut data1 = true;
            while left > 0 {
                let chunk = left.min(self.mps);
                core::ptr::copy_nonoverlapping(
                    data.as_ptr().add(src),
                    dma_ptr().add(DMA_OFF_SMALL_IO),
                    chunk as usize,
                );
                cache::dcache_clean_for_dma(dma_ptr().add(DMA_OFF_SMALL_IO), chunk as usize);
                let hc = self.hcchar(false);
                let pid = if data1 {
                    HCTSIZ::PID::Data1
                } else {
                    HCTSIZ::PID::Data0
                };
                Channel::CONTROL.xfer(hc, pid + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(chunk), DMA_OFF_SMALL_IO as u32)?;
                src += chunk as usize;
                left -= chunk;
                data1 = !data1;
            }

            self.status_stage(true)
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

