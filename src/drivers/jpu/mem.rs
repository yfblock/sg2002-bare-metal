//! JPU DMA 物理内存分配器。
//!
//! 在静态对齐缓冲上实现 16 KiB 页位图分配；所有 `unsafe` 集中在本模块。

use core::cell::SyncUnsafeCell;

/// 位图字数:pool 尺寸在 init 时钳制到 `JPU_DRAM_PHYSICAL_SIZE`,页数上限
/// `JPU_DRAM_PHYSICAL_SIZE / VMEM_PAGE_SIZE`(1 MiB / 16 KiB = 64 页 = 1 字),
/// 由此静态定长,alloc/free 无需再查位图越界。
const BITMAP_WORDS: usize =
    JPU_DRAM_PHYSICAL_SIZE.div_ceil(VMEM_PAGE_SIZE).div_ceil(u64::BITS as usize);
use super::regs::{JPU_DRAM_PHYSICAL_SIZE, VMEM_PAGE_SIZE};

/// 由 JPU 内存池分配的物理地址区间。
#[derive(Clone, Copy, Debug)]
pub struct PhysBuffer {
    pub addr: usize,
    pub size: usize,
}

impl PhysBuffer {
    pub fn is_empty(&self) -> bool {
        self.addr == 0 || self.size == 0
    }
}

struct JpuMemoryPool {
    base_addr: usize,
    size: usize,
    num_pages: usize,
    bitmap: [u64; BITMAP_WORDS],
}

impl JpuMemoryPool {
    const fn new() -> Self {
        Self {
            base_addr: 0,
            size: 0,
            num_pages: 0,
            bitmap: [0; BITMAP_WORDS],
        }
    }

    /// 重置为全空闲位图（重复 init 用：重建 decoder 前旧实例已 Drop、
    /// 全部页已 free，重跑等价于干净初值）。
    fn init(&mut self, base: usize, size: usize) {
        self.base_addr = (base + VMEM_PAGE_SIZE - 1) & !(VMEM_PAGE_SIZE - 1);
        self.size = size & !(VMEM_PAGE_SIZE - 1);
        self.num_pages = self.size / VMEM_PAGE_SIZE;
        for word in &mut self.bitmap {
            *word = u64::MAX;
        }
    }

    fn alloc(&mut self, size: usize) -> Option<PhysBuffer> {
        let npages = size.div_ceil(VMEM_PAGE_SIZE);
        let mut consecutive = 0usize;
        let mut start_page = 0usize;

        for page_idx in 0..self.num_pages {
            let word_idx = page_idx / 64;
            let bit_idx = page_idx % 64;

            if self.bitmap[word_idx] & (1 << bit_idx) != 0 {
                if consecutive == 0 {
                    start_page = page_idx;
                }
                consecutive += 1;
                if consecutive >= npages {
                    for i in 0..npages {
                        let page = start_page + i;
                        self.bitmap[page / 64] &= !(1 << (page % 64));
                    }
                    let addr = self.base_addr + start_page * VMEM_PAGE_SIZE;
                    return Some(PhysBuffer {
                        addr,
                        size: npages * VMEM_PAGE_SIZE,
                    });
                }
            } else {
                consecutive = 0;
            }
        }
        None
    }

    fn free(&mut self, buf: PhysBuffer) {
        if buf.is_empty() || buf.addr < self.base_addr || buf.addr >= self.base_addr + self.size {
            return;
        }
        let start_page = (buf.addr - self.base_addr) / VMEM_PAGE_SIZE;
        let npages = buf.size.div_ceil(VMEM_PAGE_SIZE);
        for i in 0..npages {
            let page = start_page + i;
            if page >= self.num_pages {
                break;
            }
            self.bitmap[page / 64] |= 1 << (page % 64);
        }
    }
}

static MEM_STATE: SyncUnsafeCell<JpuMemoryPool> = SyncUnsafeCell::new(JpuMemoryPool::new());

/// 用外部缓冲区初始化 DMA 内存池（绕过静态 DMA_BUFFER）。
/// 用于小核（C906L）等需要把 DMA pool 放在普通 DRAM（非预留区）的场景：
/// 预留区的 JPU DMA 地址映射可能不正确（DDR 控制器/总线防火墙限制）。
///
/// # Safety
/// 调用方须保证 `[base, base+JPU_DRAM_PHYSICAL_SIZE)` 是有效的、独占的物理内存，
/// 且 JPU DMA 引擎能正确访问该地址范围。
pub unsafe fn init_jpu_memory_with(base: usize, size: usize) {
    // 未 init 时 num_pages=0,alloc 自然返回 None;重复 init 见 pool.init 注释。
    // SAFETY: 裸机单核上下文,池状态仅主循环访问。
    let pool = unsafe { &mut *MEM_STATE.get() };
    pool.init(base, size.min(JPU_DRAM_PHYSICAL_SIZE));
}

pub fn jpu_alloc(size: usize) -> Option<PhysBuffer> {
    // SAFETY: 同上。
    let pool = unsafe { &mut *MEM_STATE.get() };
    pool.alloc(size)
}

pub fn jpu_free(buf: PhysBuffer) {
    if buf.is_empty() {
        return;
    }
    // SAFETY: 同上。
    let pool = unsafe { &mut *MEM_STATE.get() };
    pool.free(buf);
}

/// 将 JPEG bitstream 拷贝到已分配的 stream 物理缓冲。
pub fn copy_to_phys(buf: PhysBuffer, src: &[u8]) {
    let len = src.len().min(buf.size);
    if len == 0 {
        return;
    }
    let dst = phys_slice_mut(buf.addr, len);
    dst.copy_from_slice(&src[..len]);
}

fn phys_slice_mut(addr: usize, len: usize) -> &'static mut [u8] {
    // SAFETY: `addr`/`len` 来自本模块分配且尚未 free;仅写入 stream 缓冲。
    unsafe { core::slice::from_raw_parts_mut(addr as *mut u8, len) }
}
