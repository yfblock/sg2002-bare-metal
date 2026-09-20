//! JPEG 头解析（SOF / DHT / DQT / SOS）。
//!
//! 线格式常量遵循 ITU-T T.81：
//! - 标记码见 Table B.1
//! - 段结构：`FF xx` + 2 字节大端长度（值含长度字段自身、不含标记）
//! - SOF 组件条目 3 字节：ID | (H<<4)|V | 量化表号
//! - DHT 表：Tc/Th 字节 + 16 字节码长计数 + 值序列
//! - DQT 表：Pq/Tq 字节 + 64 项系数（8-bit 或 16-bit）

use super::regs::{FORMAT_400, FORMAT_420, FORMAT_422, FORMAT_224, FORMAT_444};

// ---------------------------------------------------------------------------
// JPEG 标记码（T.81 Table B.1）
// ---------------------------------------------------------------------------

const MARKER_PREFIX: u8 = 0xFF;
/// 填充字节:标记后的 0x00 是数据字节 0xFF 的转义,不是标记
const BYTE_STUFFING: u8 = 0x00;
const MARKER_SOI: u8 = 0xD8;
const MARKER_EOI: u8 = 0xD9;
const MARKER_SOF0: u8 = 0xC0; // Baseline DCT
const MARKER_SOF2: u8 = 0xC2; // Progressive DCT(仅识别;JPU 只解 baseline)
const MARKER_DHT: u8 = 0xC4;
const MARKER_SOS: u8 = 0xDA;
const MARKER_DQT: u8 = 0xDB;
const MARKER_DRI: u8 = 0xDD;
/// 0xC0 起的其余段均带长度字段,整体跳过
const MARKER_LEN_SEGMENT_MIN: u8 = 0xC0;

/// JPEG 标记类型——解析器的唯一类型判别入口,线字节经 [`Marker::from_byte`] 归类。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Marker {
    /// FF FF 填充序列:仅前移 1 字节(第二个 FF 可能是真标记的前缀)
    Fill,
    /// FF 00:数据字节 0xFF 的转义
    Stuffing,
    /// 图像开始
    Soi,
    /// 图像结束
    Eoi,
    /// Baseline DCT 帧
    Sof0,
    /// Progressive DCT 帧(仅识别;JPU 只解 baseline)
    Sof2,
    /// 霍夫曼表
    Dht,
    /// 扫描起点,其后即熵编码数据
    Sos,
    /// 量化表
    Dqt,
    /// 重启间隔
    Dri,
    /// 0xC0 起其余带长度字段的段,整体跳过(RSTn 在 header 区不出现,也同落此分支)
    OtherWithLength,
    /// 其余无长度字段的标记(如 TEM),跳过标记对
    Other,
}

impl Marker {
    /// 线字节 → 类型。线值常量只在本函数出现一次。
    fn from_byte(b: u8) -> Self {
        match b {
            MARKER_PREFIX => Marker::Fill,
            BYTE_STUFFING => Marker::Stuffing,
            MARKER_SOI => Marker::Soi,
            MARKER_EOI => Marker::Eoi,
            MARKER_SOF0 => Marker::Sof0,
            MARKER_SOF2 => Marker::Sof2,
            MARKER_DHT => Marker::Dht,
            MARKER_SOS => Marker::Sos,
            MARKER_DQT => Marker::Dqt,
            MARKER_DRI => Marker::Dri,
            MARKER_LEN_SEGMENT_MIN..=0xFE => Marker::OtherWithLength,
            _ => Marker::Other,
        }
    }
}

// ---------------------------------------------------------------------------
// 段结构布局(相对 0xFF 所在位置 i 的字节偏移)
// ---------------------------------------------------------------------------

/// 标记占 2 字节(FF + 码)
const MARKER_SIZE: usize = 2;
/// 段长度字段在 i+2 起,大端 u16
const SEG_LENGTH_OFF: usize = 2;

const SOF_HEIGHT_OFF: usize = 5;
const SOF_WIDTH_OFF: usize = 7;
const SOF_NCOMP_OFF: usize = 9;
/// SOF 组件条目区起点(i+10)
const SOF_COMPS_OFF: usize = 10;
/// SOF 组件条目 3 字节:ID | (H<<4)|V | 量化表号
const COMP_ENTRY_SIZE: usize = 3;
const COMP_SAMPLING_OFF: usize = 1;
const COMP_QUANT_OFF: usize = 2;
/// JPEG 基线最多 3 分量(Y/Cb/Cr)
const NUM_JPEG_COMPS: usize = 3;

/// SOS:Ns 在 i+4,其后每扫描分量 2 字节(ID | (Td<<4)|Ta)
const SOS_NCOMP_OFF: usize = 4;
const SOS_COMPS_OFF: usize = 5;
const SCAN_COMP_ENTRY_SIZE: usize = 2;
const SCAN_COMP_TABLES_OFF: usize = 1;

/// DRI:重启间隔 Ri 在 i+4 起,大端 u16
const DRI_INTERVAL_OFF: usize = 4;

// ---------------------------------------------------------------------------
// DHT / DQT 表结构
// ---------------------------------------------------------------------------

/// 霍夫曼码长档数(1..16 bit)
const HUFF_LENGTH_COUNT: usize = 16;
/// 单张霍夫曼表头 = Tc/Th(1 字节) + 码长计数(16 字节)
const DHT_TABLE_HDR_SIZE: usize = 1 + HUFF_LENGTH_COUNT;
/// 量化表 8×8 = 64 系数;16-bit 模式每项 2 字节
const DQT_TABLE_ENTRIES: usize = 64;

/// 空 code-length 档的哨兵:该长度无码
const HUFF_NO_CODE: u32 = 0xFFFF;
/// 无效霍夫曼值指针哨兵
const HUFF_NO_PTR: u8 = 0xFF;
/// 负系数时 JPU 寄存器高位的全 1 填充(16-bit 系数 / 8-bit 系数装 24-bit 字段)
const NEG_FILL_16: u32 = 0xFFFF;
const NEG_FILL_24: u32 = 0xFFFFFF;

/// 霍夫曼表槽的 JPU 排布:slot bit1 = Th 低位、bit0 = Tc 低位(厂商驱动同款映射)
fn huff_slot(tc: u8, th: u8) -> usize {
    (((th & 1) << 1) | (tc & 1)) as usize
}

fn high_nibble(b: u8) -> u8 {
    b >> 4
}

fn low_nibble(b: u8) -> u8 {
    b & 0x0F
}

/// JPEG 段内多字节字段均为大端(长度、宽高、重启间隔、16-bit 量化系数)
fn be16(data: &[u8], off: usize) -> usize {
    ((data[off] as usize) << 8) | data[off + 1] as usize
}

/// 读段长度字段(i+2 起大端 u16,值含自身 2 字节、不含标记)。
/// None = 缓冲在长度字段处被截断。检查与读取相邻,保证 be16 不越界。
fn seg_length(data: &[u8], i: usize) -> Option<usize> {
    let off = i + SEG_LENGTH_OFF;
    (off + 1 < data.len()).then(|| be16(data, off))
}

pub struct JpegHeaderInfo {
    pub width: u32,
    pub height: u32,
    pub num_components: u32,
    /// SOF0 各分量采样因子 (h, v)，顺序 Y/Cb/Cr；1 分量时只有 [0] 有效。
    pub sampling: [(u8, u8); 3],
    pub format: u32,
    pub ecs_offset: usize,
    pub restart_interval: u32,
    pub dc_huff_tbl: [usize; 3],
    pub ac_huff_tbl: [usize; 3],
    pub quant_tbl: [usize; 3],
    pub huff_tables: [HuffTable; 4],
    pub quant_tables: [QuantTable; 4],
    pub quant_table_count: usize,
}

pub struct HuffTable {
    pub bits: [u8; 16],
    pub values: [u8; 256],
    pub num_values: usize,
    pub min_codes: [u32; 16],
    pub max_codes: [u32; 16],
    pub ptrs: [u8; 16],
}

impl HuffTable {
    pub fn new() -> Self {
        Self {
            bits: [0; HUFF_LENGTH_COUNT],
            values: [0; 256],
            num_values: 0,
            min_codes: [HUFF_NO_CODE; HUFF_LENGTH_COUNT],
            max_codes: [HUFF_NO_CODE; HUFF_LENGTH_COUNT],
            ptrs: [HUFF_NO_PTR; HUFF_LENGTH_COUNT],
        }
    }

    /// 16-bit 系数的负值填充:最高位为 1 时高位全 1(T.81 F.2.2 EXTEND 语义)。
    pub fn sign_extend_16(huff_data: u32) -> u32 {
        if huff_data & 0x8000 != 0 {
            NEG_FILL_16
        } else {
            0
        }
    }

    /// 8-bit 系数的负值填充:最高位为 1 时 24-bit 字段高位全 1。
    pub fn sign_extend_8(huff_data: u32) -> u32 {
        if huff_data & 0x80 != 0 {
            NEG_FILL_24
        } else {
            0
        }
    }

    pub fn generate(&mut self) {
        let mut ptr_cnt: usize = 0;
        let mut huff_code: u32 = 0;
        let mut data_flag = false;

        for i in 0..HUFF_LENGTH_COUNT {
            if self.bits[i] != 0 {
                self.ptrs[i] = ptr_cnt as u8;
                ptr_cnt += self.bits[i] as usize;
                self.min_codes[i] = huff_code;
                self.max_codes[i] = huff_code + (self.bits[i] as u32 - 1);
                data_flag = true;
            } else {
                self.ptrs[i] = HUFF_NO_PTR;
                self.min_codes[i] = HUFF_NO_CODE;
                self.max_codes[i] = HUFF_NO_CODE;
            }

            if data_flag {
                if self.bits[i] == 0 {
                    huff_code <<= 1;
                } else {
                    huff_code = (self.max_codes[i] + 1) << 1;
                }
            }
        }
    }
}

pub struct QuantTable {
    pub values: [u16; 64],
}

impl QuantTable {
    pub fn new() -> Self {
        Self {
            values: [0; DQT_TABLE_ENTRIES],
        }
    }
}

impl JpegHeaderInfo {
    pub fn new() -> Self {
        Self {
            width: 0,
            height: 0,
            num_components: 0,
            // 默认 format=FORMAT_420 对应的采样因子
            sampling: [(2, 2), (1, 1), (1, 1)],
            format: FORMAT_420,
            ecs_offset: 0,
            restart_interval: 0,
            dc_huff_tbl: [0; NUM_JPEG_COMPS],
            ac_huff_tbl: [0; NUM_JPEG_COMPS],
            quant_tbl: [0; NUM_JPEG_COMPS],
            huff_tables: [HuffTable::new(), HuffTable::new(), HuffTable::new(), HuffTable::new()],
            quant_tables: [
                QuantTable::new(),
                QuantTable::new(),
                QuantTable::new(),
                QuantTable::new(),
            ],
            quant_table_count: 0,
        }
    }
}

pub fn parse_jpeg_header(data: &[u8]) -> Result<JpegHeaderInfo, &'static str> {
    let mut i = 0;
    let mut header_info = JpegHeaderInfo::new();

    while i < data.len().saturating_sub(1) {
        if data[i] != MARKER_PREFIX {
            i += 1;
            continue;
        }

        match Marker::from_byte(data[i + 1]) {
            // 填充/无长度标记:整体跳过(Fill 例外——仅前移 1 字节,第二个 FF 可能是真标记)
            Marker::Fill => {
                i += 1;
                continue;
            }
            Marker::Stuffing | Marker::Soi | Marker::Other => {
                i += MARKER_SIZE;
                continue;
            }
            Marker::Eoi => break,
            Marker::Sof0 | Marker::Sof2 => {
                // 边界与原逻辑逐字节一致:需读到首个组件 ID 字节(i+10)
                if i + SOF_COMPS_OFF >= data.len() {
                    return Err("SOF too short");
                }

                header_info.height = be16(data, i + SOF_HEIGHT_OFF) as u32;
                header_info.width = be16(data, i + SOF_WIDTH_OFF) as u32;
                header_info.num_components = data[i + SOF_NCOMP_OFF] as u32;

                if header_info.num_components == NUM_JPEG_COMPS as u32 {
                    let comps = i + SOF_COMPS_OFF;
                    // 需容纳 NUM_JPEG_COMPS 个完整组件条目
                    if comps + NUM_JPEG_COMPS * COMP_ENTRY_SIZE <= data.len() {
                        for c in 0..NUM_JPEG_COMPS {
                            let entry = comps + c * COMP_ENTRY_SIZE;
                            let hv = data[entry + COMP_SAMPLING_OFF];
                            header_info.sampling[c] = (high_nibble(hv), low_nibble(hv));
                            header_info.quant_tbl[c] = data[entry + COMP_QUANT_OFF] as usize;
                        }

                        let (h1, v1) = header_info.sampling[0];
                        let (h2, v2) = header_info.sampling[1];
                        header_info.format = if h1 == 2 && v1 == 2 && h2 == 1 && v2 == 1 {
                            FORMAT_420
                        } else if h1 == 2 && v1 == 1 {
                            FORMAT_422
                        } else if h1 == 1 && v1 == 2 {
                            FORMAT_224
                        } else {
                            FORMAT_444
                        };
                    }
                } else {
                    // 灰度:仅 Y,采样 (1,1)
                    header_info.sampling = [(1, 1), (1, 1), (1, 1)];
                    header_info.format = FORMAT_400;
                }

                // i+10 < len 已保证长度字段必在界内(i+3 < i+10);None 分支为防御性保留
                let Some(length) = seg_length(data, i) else {
                    i += 1;
                    continue;
                };
                i += MARKER_SIZE + length;
                continue;
            }
            Marker::Dht => {
                let Some(length) = seg_length(data, i) else {
                    i += 1;
                    continue;
                };
                parse_dht(data, i + MARKER_SIZE + SEG_LENGTH_OFF, i + MARKER_SIZE + length, &mut header_info)?;
                i += MARKER_SIZE + length;
                continue;
            }
            Marker::Sos => {
                let Some(sos_length) = seg_length(data, i) else {
                    i += 1;
                    continue;
                };

                if i + SOS_COMPS_OFF < data.len() {
                    let num_scan_components = data[i + SOS_NCOMP_OFF] as usize;
                    let mut comp_offset = i + SOS_COMPS_OFF;
                    for comp_idx in 0..num_scan_components.min(NUM_JPEG_COMPS) {
                        if comp_offset + SCAN_COMP_ENTRY_SIZE <= data.len() {
                            let tables = data[comp_offset + SCAN_COMP_TABLES_OFF];
                            header_info.dc_huff_tbl[comp_idx] = high_nibble(tables) as usize;
                            header_info.ac_huff_tbl[comp_idx] = low_nibble(tables) as usize;
                            comp_offset += SCAN_COMP_ENTRY_SIZE;
                        }
                    }
                }

                header_info.ecs_offset = i + MARKER_SIZE + sos_length;
                return Ok(header_info);
            }
            Marker::Dqt => {
                let Some(length) = seg_length(data, i) else {
                    i += 1;
                    continue;
                };
                parse_dqt(data, i + MARKER_SIZE + SEG_LENGTH_OFF, i + MARKER_SIZE + length, &mut header_info)?;
                i += MARKER_SIZE + length;
                continue;
            }
            Marker::Dri => {
                if i + DRI_INTERVAL_OFF + 2 <= data.len() {
                    header_info.restart_interval = be16(data, i + DRI_INTERVAL_OFF) as u32;
                }
                let Some(length) = seg_length(data, i) else {
                    i += 1;
                    continue;
                };
                i += MARKER_SIZE + length;
                continue;
            }
            Marker::OtherWithLength => {
                let Some(length) = seg_length(data, i) else {
                    i += MARKER_SIZE;
                    continue;
                };
                i += MARKER_SIZE + length;
                continue;
            }
        }
    }

    Err("SOS not found")
}

fn parse_dht(
    data: &[u8],
    start: usize,
    end: usize,
    header_info: &mut JpegHeaderInfo,
) -> Result<(), &'static str> {
    let mut offset = start;

    while offset < end && offset + 1 < data.len() {
        let tc_th = data[offset];
        let tc = high_nibble(tc_th);
        let th = low_nibble(tc_th);
        let table_idx = huff_slot(tc, th);

        let mut num_values = 0;
        for j in 0..HUFF_LENGTH_COUNT {
            if offset + 1 + j < data.len() {
                header_info.huff_tables[table_idx].bits[j] = data[offset + 1 + j];
                num_values += data[offset + 1 + j] as usize;
            }
        }

        for j in 0..num_values {
            if offset + DHT_TABLE_HDR_SIZE + j < data.len() {
                header_info.huff_tables[table_idx].values[j] = data[offset + DHT_TABLE_HDR_SIZE + j];
            }
        }
        header_info.huff_tables[table_idx].num_values = num_values;
        header_info.huff_tables[table_idx].generate();

        offset += DHT_TABLE_HDR_SIZE + num_values;
    }

    Ok(())
}

fn parse_dqt(
    data: &[u8],
    start: usize,
    end: usize,
    header_info: &mut JpegHeaderInfo,
) -> Result<(), &'static str> {
    let mut offset = start;

    while offset < end && offset + 1 < data.len() {
        let pq_tq = data[offset];
        let tq: usize = low_nibble(pq_tq) as usize;

        if high_nibble(pq_tq) == 0 {
            // Pq=0:8-bit 系数,每项 1 字节
            for j in 0..DQT_TABLE_ENTRIES {
                if offset + 1 + j < data.len() {
                    header_info.quant_tables[tq].values[j] = data[offset + 1 + j] as u16;
                }
            }
            offset += 1 + DQT_TABLE_ENTRIES;
        } else {
            // Pq=1:16-bit 系数,每项 2 字节大端
            for j in 0..DQT_TABLE_ENTRIES {
                if offset + 1 + j * 2 + 1 < data.len() {
                    header_info.quant_tables[tq].values[j] =
                        ((data[offset + 1 + j * 2] as u16) << 8) | (data[offset + 1 + j * 2 + 1] as u16);
                }
            }
            offset += 1 + DQT_TABLE_ENTRIES * 2;
        }

        if tq >= header_info.quant_table_count {
            header_info.quant_table_count = tq + 1;
        }
    }

    Ok(())
}
