//! EP0 控制传输：`ControlEp` 端点句柄（绑定设备地址 + EP0 包长）+ 标准请求
//! （SET_ADDRESS / SET_CONFIGURATION / 设备描述符）与 Hub 端口请求包装，全部走通道 0。

use super::channel::Channel;
use super::dma::{dma_ptr, dma_rx_slice, DMA_OFF_SMALL_IO, OFF_EP0};
use super::regs::{HCCHAR, HCTSIZ};
use crate::arch::cache;
use crate::drivers::usb::error::{UsbError, UsbResult};
use crate::drivers::usb::setup::StdRequest;
use tock_registers::fields::FieldValue;

/// EP0 控制端点句柄：把控制传输的公共前置参数（设备地址、EP0 最大包长）
/// 绑定为端点自身状态。
#[derive(Clone, Copy)]
pub struct ControlEp {
    dev: u32,
    mps: u32,
}

/// 设备描述符(USB 2.0 §9.6.1,固定 18 字节)的字段视图:偏移与字段名
/// 同址、小端显式、构造即校验长度。避免裸指针 reinterpret 的 packed
/// 引用 UB 与目标字节序假设。
struct DeviceDescriptor<'a> {
    sl: &'a [u8],
}

impl<'a> DeviceDescriptor<'a> {
    fn new(sl: &'a [u8]) -> Option<Self> {
        (sl.len() >= 18).then_some(Self { sl })
    }

    /// `bDeviceClass`@4。
    fn device_class(&self) -> u8 {
        self.sl[4]
    }

    /// `bMaxPacketSize0`@7。
    fn max_packet_size0(&self) -> u8 {
        self.sl[7]
    }

    /// `idVendor`@8..10(LE)。
    fn vendor_id(&self) -> u16 {
        u16::from_le_bytes([self.sl[8], self.sl[9]])
    }

    /// `idProduct`@10..12(LE)。
    fn product_id(&self) -> u16 {
        u16::from_le_bytes([self.sl[10], self.sl[11]])
    }
}

impl ControlEp {
    /// 绑定一台已寻址设备的 EP0。
    ///
    /// # 参数
    /// - `dev`：设备 USB 地址（7 位数值）。
    /// - `control_ep_mps`：该设备 EP0 最大包长（8/16/32/64）。
    #[inline]
    pub fn new(dev: u32, control_ep_mps: u32) -> Self {
        Self {
            dev,
            mps: control_ep_mps,
        }
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
    fn setup_stage(&self, setup_packet: &[u8; 8]) -> UsbResult<()> {
        unsafe {
            core::ptr::copy_nonoverlapping(setup_packet.as_ptr(), dma_ptr().add(OFF_EP0), 8);
            cache::dcache_clean_range(dma_ptr() as usize + OFF_EP0, 8);
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
    /// `(vid, pid, control_ep_mps, b_device_class)`，均在设备描述符前 18 字节内解析。
    pub fn probe_default_addr() -> UsbResult<(u16, u16, u32, u8)> {
        // 默认地址 0 阶段的临时句柄(MPS 固定 64,USB 2.0 枚举惯例)。
        let control_ep = ControlEp { dev: 0, mps: 64 };
        // wLength = 18:设备描述符规范全长。
        let w_length: u16 = 18;
        control_ep.setup_stage(&StdRequest::get_descriptor_device())?;

        unsafe {
            Channel::CONTROL.xfer(
                control_ep.hcchar(true),
                HCTSIZ::PID::Data1 + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(w_length as u32),
                OFF_EP0 as u32,
            )?;
        }
        cache::dcache_invalidate_range(dma_ptr() as usize + OFF_EP0, w_length as usize);

        let descriptor = DeviceDescriptor::new(
            dma_rx_slice(OFF_EP0, w_length as usize).ok_or(UsbError::Hardware("dma view"))?,
        )
        .ok_or(UsbError::Protocol("short descriptor"))?;
        let ep0_max_packet_size = normalize_ep0_mps(descriptor.max_packet_size0());

        control_ep.status_stage(false)?;

        Ok((
            descriptor.vendor_id(),
            descriptor.product_id(),
            ep0_max_packet_size,
            descriptor.device_class(),
        ))
    }

    /// 在**默认地址句柄**上发送 `SET_ADDRESS`,并把本句柄迁移到新地址。
    ///
    /// 前置条件:`self` 处于默认地址(`dev==0`)——枚举序列中 probe 之后、
    /// 正式使用之前;成功后 `self` 即新地址句柄,调用方无需重建。
    ///
    /// # 参数
    /// - `addr`：设备新地址，合法 **1..=127**。
    pub fn set_address(&mut self, addr: u8) -> UsbResult<()> {
        if self.dev != 0 {
            return Err(UsbError::Protocol("set_address on non-default address"));
        }
        self.write_no_data(StdRequest::set_address(addr))?;
        self.dev = addr as u32;
        Ok(())
    }

    /// 对已寻址设备发送 `SET_CONFIGURATION`。
    ///
    /// # 参数
    /// - `cfg`：`bConfigurationValue`（通常非 0 表示激活配置）。
    pub fn set_configuration(&self, cfg: u8) -> UsbResult<()> {
        self.write_no_data(StdRequest::set_configuration(cfg))
    }

    /// 对已寻址设备发送 `SET_INTERFACE`(选接口备用设置,UVC 开流切带宽档)。
    pub fn set_interface(&self, alt: u8, interface: u8) -> UsbResult<()> {
        self.write_no_data(StdRequest::set_interface(alt, interface))
    }

    /// `GET_DESCRIPTOR(Configuration)` 标准两段式:先读 9 字节头取
    /// `wTotalLength`,再读全量(上限 4096)。纯标准请求组合,归本类型。
    pub fn get_configuration_descriptor(&self, cfg_index: u8) -> UsbResult<[u8; 4096]> {
        const USB_DT_CONFIGURATION: u8 = 2;
        let mut hdr = [0u8; 9];
        self.read(
            StdRequest::get_descriptor_configuration(cfg_index, 9),
            &mut hdr,
        )?;
        if hdr[1] != USB_DT_CONFIGURATION {
            return Err(UsbError::Protocol("not a configuration descriptor"));
        }
        let total = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;
        if total > 4096 {
            return Err(UsbError::Protocol(
                "configuration descriptor too large (>4096)",
            ));
        }
        let mut buf = [0u8; 4096];
        self.read(
            StdRequest::get_descriptor_configuration(cfg_index, total as u16),
            &mut buf[..total],
        )?;
        Ok(buf)
    }

    /// 无数据阶段控制传输：`SETUP` + `STATUS` IN（零长度）。
    pub fn write_no_data(&self, setup_packet: [u8; 8]) -> UsbResult<()> {
        self.setup_stage(&setup_packet)?;
        self.status_stage(true)
    }

    /// 控制读：SETUP + 若干 IN 数据包（DATA1/DATA0 交替）+ STATUS OUT。
    ///
    /// 适用于 Hub 描述符、配置前缀、`GET_PORT_STATUS`、UVC 配置描述符 / PROBE-COMMIT 等。
    ///
    /// # 参数
    /// - `setup_packet`：8 字节 SETUP（`wLength` 应等于 `out.len()` 的期望读长）。
    /// - `out`：接收缓冲区，长度须与 SETUP 中 `wLength` 一致且在 `(0,4096]`。
    pub fn read(&self, setup_packet: [u8; 8], out: &mut [u8]) -> UsbResult<()> {
        if out.is_empty() || out.len() > 4096 {
            return Err(UsbError::Protocol("bad control_ep read len"));
        }
        self.setup_stage(&setup_packet)?;
        let hc = self.hcchar(true); // IN 数据段:管道方向恒定,循环外组装
                                    // 控制传输数据阶段:首包 DATA1,随后 DATA1/DATA0 交替(数据切换)。
        let mut data1 = true;
        for out_chunk in out.chunks_mut(self.mps as usize) {
            let pid = if data1 {
                HCTSIZ::PID::Data1
            } else {
                HCTSIZ::PID::Data0
            };
            // SAFETY: DMA 源为窗口小 IO 区(偏移界内);CONTROL 通道由单核
            // 调用方独占;invalidate 后读回的是设备刚写的数据。
            unsafe {
                Channel::CONTROL.xfer(
                    hc,
                    pid + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(out_chunk.len() as u32),
                    DMA_OFF_SMALL_IO as u32,
                )?;
                let dma_src = dma_ptr().add(DMA_OFF_SMALL_IO);
                cache::dcache_invalidate_range(dma_src as usize, out_chunk.len());
                core::ptr::copy_nonoverlapping(dma_src, out_chunk.as_mut_ptr(), out_chunk.len());
            }
            data1 = !data1;
        }

        self.status_stage(false)
    }

    /// 控制写：`SETUP` + `DATA` OUT（可多包）+ `STATUS` IN（ZLP）。
    ///
    /// # 参数
    /// - `setup_packet`：8 字节 SETUP（`wLength` 应等于 `data.len()`）。
    /// - `data`：OUT 数据阶段负载（最大 4096 字节）。
    pub fn write(&self, setup_packet: [u8; 8], data: &[u8]) -> UsbResult<()> {
        if data.len() > 4096 {
            return Err(UsbError::Protocol("bad control_ep write data len"));
        }
        self.setup_stage(&setup_packet)?;
        let hc = self.hcchar(false); // OUT 数据段:管道方向恒定,循环外组装
                                     // 控制传输数据阶段:首包 DATA1,随后 DATA1/DATA0 交替(数据切换)。
        let mut data1 = true;
        for chunk in data.chunks(self.mps as usize) {
            let pid = if data1 {
                HCTSIZ::PID::Data1
            } else {
                HCTSIZ::PID::Data0
            };
            // SAFETY: 拷贝目标为窗口小 IO 区(偏移界内),与源切片不重叠;
            // CONTROL 通道由单核调用方独占。
            unsafe {
                let dma_dst = dma_ptr().add(DMA_OFF_SMALL_IO);
                core::ptr::copy_nonoverlapping(chunk.as_ptr(), dma_dst, chunk.len());
                cache::dcache_clean_range(dma_dst as usize, chunk.len());
                Channel::CONTROL.xfer(
                    hc,
                    pid + HCTSIZ::PKTCNT.val(1) + HCTSIZ::XFERSIZE.val(chunk.len() as u32),
                    DMA_OFF_SMALL_IO as u32,
                )?;
            }
            data1 = !data1;
        }

        self.status_stage(true)
    }
}

#[inline]
fn normalize_ep0_mps(b: u8) -> u32 {
    match b {
        8 | 16 | 32 | 64 => b as u32,
        _ => 8,
    }
}
