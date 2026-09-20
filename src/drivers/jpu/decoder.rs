//! JPU 硬件 JPEG 解码器（Baseline，轮询模式）。

use super::header::{JpegHeaderInfo, parse_jpeg_header};
use super::mem::{PhysBuffer, copy_to_phys, jpu_free};
use super::regs::{
    HUFF_ADDR_MAX, HUFF_ADDR_PTR, HUFF_PHASE_MAX, HUFF_PHASE_MIN, HUFF_PHASE_PTR, HUFF_PHASE_VAL,
    MJPEG_BBC_STRM_CTRL, MJPEG_HUFF_CTRL, MJPEG_PIC_CTRL, MJPEG_PIC_SIZE, MJPEG_PIC_START,
    MJPEG_PIC_STATUS, MJPEG_QMAT_CTRL, QMAT_PHASE_CB, QMAT_PHASE_CR, QMAT_PHASE_Y,
    STREAM_BUF_SIZE, VALUE32, jpu_regs, wait_bbc_idle,
    FORMAT_400, FORMAT_420, FORMAT_422, FORMAT_224, FORMAT_444,
};
use crate::arch::cache::dcache_clean_range;
use core::time::Duration;

use crate::arch::time::{elapsed_since, rdtime};
use tock_registers::interfaces::{Readable, Writeable};

/// 解码结果：YUV420 planar，数据位于固定输出 DMA 帧缓冲（至下次 decode 有效）。
pub struct DecodeResult {
    pub width: u32,
    pub height: u32,
    /// 输出帧字节数（YUV 平面总大小）。
    pub frame_size: usize,
}

/// JPU 解码器实例（持有 stream DMA 缓冲 + 固定输出帧缓冲）。
pub struct JpuDecoder {
    stream_buf: PhysBuffer,
    /// 调用方指定的固定输出缓冲（物理地址，不属于内部 pool）：`decode()` 让 JPU
    /// **直接 DMA 到这里**，省掉一次整帧 memcpy。CPU 从不通过 cache 读它，
    /// 故不做输出侧 dcache 维护——不会有脏行写回覆盖 DMA 数据。
    frame_buf: PhysBuffer,
}

impl JpuDecoder {
    /// 创建解码器，用 `hardware_init`（设时钟/复位/VC/软复位，但不设 VD_REMAP）。
    /// 适用于小核（C906L）：VD_REMAP 会把 32 位 DMA 地址扩展到 40 位，超出 DDR 范围。
    /// DMA pool 用外部地址（绕过静态 DMA_BUFFER 在预留区的问题）。
    /// 输出帧固定 DMA 到 `[out_pa, out_pa+out_size)`。
    ///
    /// # Safety
    /// 调用方须保证 pool 区与 `[out_pa, out_pa+out_size)` 均为有效、独占、
    /// JPU DMA 可达的物理内存。
    pub unsafe fn new_with_pool(
        dma_pool_base: usize,
        dma_pool_size: usize,
        out_pa: usize,
        out_size: usize,
    ) -> Result<Self, &'static str> {
        let mut decoder = Self {
            stream_buf: PhysBuffer { addr: 0, size: 0 },
            frame_buf: PhysBuffer { addr: out_pa, size: out_size },
        };
        super::mem::init_jpu_memory_with(dma_pool_base, dma_pool_size);
        super::regs::hardware_init();
        decoder.stream_buf = super::mem::jpu_alloc(STREAM_BUF_SIZE)
            .ok_or("Failed to allocate stream buffer")?;
        Ok(decoder)
    }

    pub fn decode(&mut self, jpeg_data: &[u8]) -> Result<DecodeResult, &'static str> {

        let header_info = parse_jpeg_header(jpeg_data)?;

        let copy_len = jpeg_data.len().min(self.stream_buf.size);
        copy_to_phys(self.stream_buf, &jpeg_data[..copy_len]);
        dcache_clean_range(self.stream_buf.addr, copy_len);

        let (frame_size, layout) = frame_layout(&header_info)?;

        // 外部输出缓冲：不分配也不释放，直接 DMA 过去。
        if frame_size > self.frame_buf.size {
            log::warn!(
                "[JPU] output buf too small: frame_size={} out.size={} {}x{} fmt={}",
                frame_size, self.frame_buf.size, header_info.width, header_info.height, header_info.format
            );
            return Err("output buffer too small");
        }

        configure_stream_regs(
            &self.stream_buf,
            copy_len,
            &header_info,
            layout,
        );

        upload_huff_tables(&header_info)?;
        upload_quant_tables(&header_info)?;

        let stream_dma = self.stream_buf.addr; // identity(VA=PA)
        gram_setup(stream_dma, &header_info)?;

        let frame_dma = self.frame_buf.addr; // identity(VA=PA)
        start_decode(frame_dma, &header_info, layout)?;
        if let Err(e) = poll_decode_done() {
            // 挂死/出错后必须真正复位 JPEG 块（assert→deassert 脉冲），否则
            // 后续每帧都会再等满一个超时。
            super::regs::hard_reset();
            return Err(e);
        }

        let result = DecodeResult {
            width: header_info.width,
            height: header_info.height,
            frame_size,
        };
        Ok(result)
    }
}

impl Drop for JpuDecoder {
    fn drop(&mut self) {
        if !self.stream_buf.is_empty() {
            jpu_free(self.stream_buf);
        }
        // frame_buf 是外部固定输出缓冲，不属于内部 pool，不能 free。
    }
}

#[derive(Clone, Copy)]
struct FrameLayout {
    aligned_width: u32,
    aligned_height: u32,
    stride_y: u32,
    stride_c: u32,
    luma_size: usize,
    chroma_size: usize,
    mcu_block_num: u32,
    comp_info: u32,
    bus_req_num: u32,
}

/// `op_info` 寄存器的 AHB 总线突发请求 beat 数——厂商驱动按格式数据密度推荐:
/// 420(SPARSE) < 422/224(MEDIUM) < 444/400(DENSE);纯性能参数,不影响正确性。
const BUS_REQ_NUM_SPARSE: u32 = 2;
const BUS_REQ_NUM_MEDIUM: u32 = 3;
const BUS_REQ_NUM_DENSE: u32 = 4;

fn frame_layout(header: &JpegHeaderInfo) -> Result<(usize, FrameLayout), &'static str> {
    let aligned_width = match header.format {
        FORMAT_420 | FORMAT_422 => header.width.div_ceil(16) * 16,
        _ => header.width.div_ceil(8) * 8,
    };
    let aligned_height = match header.format {
        FORMAT_420 | FORMAT_224 => header.height.div_ceil(16) * 16,
        _ => header.height.div_ceil(8) * 8,
    };
    let stride_y = aligned_width;
    let stride_c = match header.format {
        FORMAT_420 | FORMAT_422 => aligned_width / 2,
        FORMAT_400 => 0,
        _ => aligned_width,
    };

    let luma_size = (stride_y * aligned_height) as usize;
    let chroma_size = match header.format {
        FORMAT_420 => (stride_c * aligned_height / 2) as usize,
        FORMAT_422 | FORMAT_224 => luma_size / 2,
        FORMAT_444 => luma_size,
        FORMAT_400 => 0,
        _ => (stride_c * aligned_height / 2) as usize,
    };

    // comp_info:每分量采样因子编为 (h<<2)|v,nibble 位序 [11:8]=Y [7:4]=Cb [3:0]=Cr;
    // mcu_block_num:每 MCU 的 8×8 块数 = Σ hᵢ·vᵢ。
    // 两者均由 SOF 采样因子直接推导,与厂商驱动真值表逐字节一致,不再按格式查表。
    let mut comp_info = 0u32;
    let mut mcu_block_num = 0u32;
    for (i, &(h, v)) in header.sampling.iter().enumerate().take(header.num_components as usize) {
        comp_info |= (u32::from(h) << 2 | u32::from(v)) << (8 - 4 * i);
        mcu_block_num += u32::from(h) * u32::from(v);
    }
    let bus_req_num = match header.format {
        FORMAT_420 => BUS_REQ_NUM_SPARSE,
        FORMAT_422 | FORMAT_224 => BUS_REQ_NUM_MEDIUM,
        FORMAT_444 | FORMAT_400 => BUS_REQ_NUM_DENSE,
        _ => BUS_REQ_NUM_SPARSE,
    };

    Ok((
        luma_size + chroma_size * 2,
        FrameLayout {
            aligned_width,
            aligned_height,
            stride_y,
            stride_c,
            luma_size,
            chroma_size,
            mcu_block_num,
            comp_info,
            bus_req_num,
        },
    ))
}

fn configure_stream_regs(stream_buf: &PhysBuffer,
    copy_len: usize,
    header: &JpegHeaderInfo,
    layout: FrameLayout,
) {
    let jpu = jpu_regs();
    let stream_phys = stream_buf.addr as u32; // identity(VA=PA)
    let stream_end = (stream_buf.addr + copy_len) as u32;

    jpu.bbc_bas_addr.write(VALUE32::VAL.val(stream_phys));
    jpu.bbc_end_addr.write(VALUE32::VAL.val(stream_end));
    jpu.bbc_rd_ptr.write(VALUE32::VAL.val(stream_phys));
    jpu.bbc_wr_ptr.write(VALUE32::VAL.val(stream_end));

    let strm_pages = copy_len.div_ceil(256);
    jpu.bbc_strm_ctrl.set(
        (MJPEG_BBC_STRM_CTRL::END_FLAG::SET + MJPEG_BBC_STRM_CTRL::PAGES.val(strm_pages as u32))
            .into(),
    );

    jpu.gbu_tt_cnt.write(VALUE32::VAL.val(0));
    jpu.gbu_tt_cnt_h.write(VALUE32::VAL.val(0));
    jpu.pic_errmb.write(VALUE32::VAL.val(0));

    let mut huff_dc_idx = 0u32;
    let mut huff_ac_idx = 0u32;
    for i in 0..3 {
        huff_dc_idx = (huff_dc_idx << 1) | header.dc_huff_tbl[i] as u32;
        huff_ac_idx = (huff_ac_idx << 1) | header.ac_huff_tbl[i] as u32;
    }
    jpu.pic_ctrl.set(
        (MJPEG_PIC_CTRL::HUFF_DC_IDX.val(huff_dc_idx)
            + MJPEG_PIC_CTRL::HUFF_AC_IDX.val(huff_ac_idx)
            + MJPEG_PIC_CTRL::USER_HUFF_TAB::SET)
            .into(),
    );

    jpu.pic_size.write(
        MJPEG_PIC_SIZE::WIDTH.val(layout.aligned_width)
            + MJPEG_PIC_SIZE::HEIGHT.val(layout.aligned_height),
    );
    jpu.rot_info.write(VALUE32::VAL.val(0));
    jpu.mcu_info.write(VALUE32::VAL.val((layout.mcu_block_num << 16) | (header.num_components << 12) | layout.comp_info));
    jpu.dpb_config.write(VALUE32::VAL.val(0));
    jpu.rst_intval.write(VALUE32::VAL.val(header.restart_interval));
    jpu.scl_info.write(VALUE32::VAL.val(0));
    jpu.op_info.write(VALUE32::VAL.val(layout.bus_req_num));
}

fn upload_huff_tables(header: &JpegHeaderInfo) -> Result<(), &'static str> {
    let jpu = jpu_regs();

    jpu.huff_ctrl
        .write(MJPEG_HUFF_CTRL::PHASE.val(HUFF_PHASE_MIN));
    for table_idx in [0, 2, 1, 3] {
        for j in 0..16 {
            let huff_data = header.huff_tables[table_idx].min_codes[j];
            let temp = sign_extend_16(huff_data);
            jpu.huff_data.write(VALUE32::VAL.val(((temp & 0xFFFF) << 16) | huff_data));
        }
    }

    jpu.huff_ctrl
        .write(MJPEG_HUFF_CTRL::PHASE.val(HUFF_PHASE_MAX));
    jpu.huff_addr.write(VALUE32::VAL.val(HUFF_ADDR_MAX));
    for table_idx in [0, 2, 1, 3] {
        for j in 0..16 {
            let huff_data = header.huff_tables[table_idx].max_codes[j];
            let temp = sign_extend_16(huff_data);
            jpu.huff_data.write(VALUE32::VAL.val(((temp & 0xFFFF) << 16) | huff_data));
        }
    }

    jpu.huff_ctrl
        .write(MJPEG_HUFF_CTRL::PHASE.val(HUFF_PHASE_PTR));
    jpu.huff_addr.write(VALUE32::VAL.val(HUFF_ADDR_PTR));
    for table_idx in [0, 2, 1, 3] {
        for j in 0..16 {
            let huff_data = header.huff_tables[table_idx].ptrs[j] as u32;
            let temp = sign_extend_8(huff_data);
            jpu.huff_data.write(VALUE32::VAL.val(((temp & 0xFFFFFF) << 8) | huff_data));
        }
    }

    jpu.huff_ctrl
        .write(MJPEG_HUFF_CTRL::PHASE.val(HUFF_PHASE_VAL));
    for &table_idx in &[0, 2, 1, 3] {
        let is_dc = table_idx == 0 || table_idx == 2;
        let max_count = if is_dc { 12 } else { 162 };
        let bits_len = if is_dc { 12 } else { 16 };
        let count: usize = header.huff_tables[table_idx].bits[..bits_len]
            .iter()
            .map(|&b| b as usize)
            .sum();

        for j in 0..count.min(header.huff_tables[table_idx].num_values) {
            let val = header.huff_tables[table_idx].values[j] as u32;
            let temp = sign_extend_8(val);
            jpu.huff_data.write(VALUE32::VAL.val(((temp & 0xFFFFFF) << 8) | val));
        }
        for _ in count..max_count {
            jpu.huff_data.write(VALUE32::VAL.val(0xFFFF_FFFF));
        }
    }

    jpu.huff_ctrl.write(MJPEG_HUFF_CTRL::PHASE.val(0));
    Ok(())
}

/// 负系数时 JPU 寄存器高位的全 1 填充(16-bit 系数 / 8-bit 系数装 24-bit 字段)
const NEG_FILL_16: u32 = 0xFFFF;
const NEG_FILL_24: u32 = 0xFFFFFF;

/// 16-bit 系数的负值填充:最高位为 1 时高位全 1(T.81 F.2.2 EXTEND 语义)。
fn sign_extend_16(huff_data: u32) -> u32 {
    if huff_data & 0x8000 != 0 {
        NEG_FILL_16
    } else {
        0
    }
}

/// 8-bit 系数的负值填充:最高位为 1 时 24-bit 字段高位全 1。
fn sign_extend_8(huff_data: u32) -> u32 {
    if huff_data & 0x80 != 0 {
        NEG_FILL_24
    } else {
        0
    }
}

fn upload_quant_tables(header: &JpegHeaderInfo) -> Result<(), &'static str> {
    let jpu = jpu_regs();
    let qmat_phases = [QMAT_PHASE_Y, QMAT_PHASE_CB, QMAT_PHASE_CR];
    let comp_count = (header.num_components as usize).min(3);
    for (comp_idx, &phase) in qmat_phases.iter().enumerate().take(comp_count) {
        let table_idx = header.quant_tbl[comp_idx];
        if table_idx >= 4 || table_idx >= header.quant_table_count {
            continue;
        }

        jpu.qmat_ctrl.write(MJPEG_QMAT_CTRL::PHASE.val(phase));
        for j in 0..64 {
            jpu.qmat_data.write(VALUE32::VAL.val(header.quant_tables[table_idx].values[j] as u32));
        }
        jpu.qmat_ctrl.write(MJPEG_QMAT_CTRL::PHASE.val(0));
    }
    Ok(())
}

fn gram_setup(stream_phys: usize, header: &JpegHeaderInfo) -> Result<(), &'static str> {
    let jpu = jpu_regs();
    let ecs_offset = header.ecs_offset;
    let page_ptr = ecs_offset >> 8;
    let mut word_ptr = (ecs_offset & 0xF0) >> 2;
    let bit_ptr = (ecs_offset & 0xF) << 3;

    if page_ptr & 1 != 0 {
        word_ptr += 64;
    }
    if word_ptr & 1 != 0 {
        word_ptr -= 1;
    }

    for i in 0..2 {
        let cur_page = page_ptr + i;
        jpu.bbc_cur_pos.write(VALUE32::VAL.val(cur_page as u32));
        jpu.bbc_ext_addr.write(VALUE32::VAL.val((stream_phys as u32) + ((cur_page as u32) << 8)));
        jpu.bbc_int_addr.write(VALUE32::VAL.val(((cur_page & 1) as u32) << 6));
        jpu.bbc_data_cnt.write(VALUE32::VAL.val(256 / 4));
        jpu.bbc_command.write(VALUE32::VAL.val(0));
        wait_bbc_idle();
    }

    jpu.bbc_cur_pos.write(VALUE32::VAL.val((page_ptr + 2) as u32));
    jpu.bbc_ctrl.write(VALUE32::VAL.val(1));

    jpu.gbu_wd_ptr.write(VALUE32::VAL.val(word_ptr as u32));
    jpu.gbu_bbsr.write(VALUE32::VAL.val(0));
    jpu.gbu_bber.write(VALUE32::VAL.val(((256 / 4) * 2) - 1));

    if page_ptr & 1 != 0 {
        jpu.gbu_bbir.write(VALUE32::VAL.val(0));
        jpu.gbu_bbhr.write(VALUE32::VAL.val(0));
    } else {
        jpu.gbu_bbir.write(VALUE32::VAL.val(256 / 4));
        jpu.gbu_bbhr.write(VALUE32::VAL.val(256 / 4));
    }

    jpu.gbu_ctrl.write(VALUE32::VAL.val(4));
    jpu.gbu_ff_rptr.write(VALUE32::VAL.val(bit_ptr as u32));
    Ok(())
}

fn start_decode(frame_phys: usize,
    header: &JpegHeaderInfo,
    layout: FrameLayout,
) -> Result<(), &'static str> {
    let jpu = jpu_regs();
    jpu.rst_index.write(VALUE32::VAL.val(0));
    jpu.rst_count.write(VALUE32::VAL.val(0));
    jpu.dpcm_diff_y.write(VALUE32::VAL.val(0));
    jpu.dpcm_diff_cb.write(VALUE32::VAL.val(0));
    jpu.dpcm_diff_cr.write(VALUE32::VAL.val(0));

    let bit_ptr = (header.ecs_offset & 0xF) << 3;
    jpu.gbu_ff_rptr.write(VALUE32::VAL.val(bit_ptr as u32));
    jpu.gbu_ctrl.write(VALUE32::VAL.val(3));

    jpu.dpb_base_y.write(VALUE32::VAL.val(frame_phys as u32));
    let cb_phys = frame_phys + layout.luma_size;
    jpu.dpb_base_cb.write(VALUE32::VAL.val(cb_phys as u32));
    let cr_phys = cb_phys + layout.chroma_size;
    jpu.dpb_base_cr.write(VALUE32::VAL.val(cr_phys as u32));

    jpu.dpb_ystride.write(VALUE32::VAL.val(layout.stride_y));
    jpu.dpb_cstride.write(VALUE32::VAL.val(layout.stride_c));
    jpu.clp_info.write(VALUE32::VAL.val(0));

    // W1C:写 1 清 DONE/ERROR,清上一帧残留状态再启动
    jpu.pic_status.write(MJPEG_PIC_STATUS::DONE::SET + MJPEG_PIC_STATUS::ERROR::SET);
    jpu.pic_start.write(MJPEG_PIC_START::START_PIC::SET);
    Ok(())
}

/// 单帧解码的等待上限。640×480 baseline MJPEG 正常 1~5ms 完成，200ms 已极宽松。
///
/// **必须按时间而非轮询次数判定**：原来只有 `MAX_POLLS` 次数上限，而每轮内层
/// 1000 次 `spin_loop` 在小核 C906L 上要 ~461us（实测 ~2168 轮/秒），500k 轮实际
/// 要 **230 秒**才超时。JPU 一挂，小核就被这个循环堵 230 秒，`frame_count` 冻结，
/// 外部看起来像彻底死机——而不是预期的"2 秒后超时并复位"。
const DECODE_TIMEOUT_MS: u64 = 200;

fn poll_decode_done() -> Result<(), &'static str> {
    let jpu = jpu_regs();
    let t0 = rdtime();
    let timeout = Duration::from_millis(DECODE_TIMEOUT_MS);

    loop {
        if jpu.pic_status.is_set(MJPEG_PIC_STATUS::DONE) {
            // W1C:写 1 清 DONE/ERROR
            jpu.pic_status.write(MJPEG_PIC_STATUS::DONE::SET + MJPEG_PIC_STATUS::ERROR::SET);
            return Ok(());
        }

        if jpu.pic_status.is_set(MJPEG_PIC_STATUS::ERROR) {
            let status = jpu.pic_status.get();
            let err_mb = jpu.pic_errmb.get();
            log::warn!(
                "[JPU] Error! status=0x{:x}, err_mb=0x{:x}",
                status,
                err_mb
            );
            // W1C:写 1 清 DONE/ERROR
            jpu.pic_status.write(MJPEG_PIC_STATUS::DONE::SET + MJPEG_PIC_STATUS::ERROR::SET);
            return Err("JPU decode error");
        }

        // 轮询间隔别太大：这里每轮的开销直接决定超时判定的粒度。
        for _ in 0..64 {
            core::hint::spin_loop();
        }
        if elapsed_since(t0) >= timeout {
            let status = jpu.pic_status.get();
            log::warn!("[JPU] Timeout! status=0x{:x}", status);
            return Err("JPU decode timeout");
        }
    }
}
