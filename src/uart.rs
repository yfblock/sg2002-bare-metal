//! UART0 驱动 + 打印辅助。
//!
//! UART0 = 0x04140000，DesignWare 8250，**32-bit 寄存器步长**（与 cvitek `dw_regs`
//! 的 `uint32_t` 字段一致）：THR@+0x00，LSR@+0x14（THRE=0x20）。U-Boot 已把
//! UART0 初始化为 115200 控制台，小核直接写 THR 即可输出，无需再配波特率。

use core::ptr::{read_volatile, write_volatile};

const UART0_BASE: usize = 0x0414_0000;
const UART_THR: usize = 0x00; // 发送保持寄存器（写）
const UART_LSR: usize = 0x14; // Line Status Register
const LSR_THRE: u32 = 0x20; // Transmit Holding Register Empty

/// 阻塞发送一个字节（轮询 LSR.THRE）。`\n` 不自动加 `\r`——由 [`print`] 处理。
#[inline]
pub(crate) fn uart_putc(c: u8) {
    unsafe {
        let lsr = (UART0_BASE + UART_LSR) as *mut u32;
        while read_volatile(lsr) & LSR_THRE == 0 {}
        write_volatile((UART0_BASE + UART_THR) as *mut u32, c as u32);
    }
}

/// 打印字符串；`\n` 自动转成 `\r\n`（串口终端需要）。
pub(crate) fn print(s: &str) {
    for &b in s.as_bytes() {
        if b == b'\n' {
            uart_putc(b'\r');
        }
        uart_putc(b);
    }
}

/// 打印 `0x` 前缀的 64 位十六进制。
pub(crate) fn print_hex(v: u64) {
    print("0x");
    let mut started = false;
    for shift in (0..64).step_by(4).rev() {
        let nib = ((v >> shift) & 0xf) as u8;
        if nib != 0 || started || shift == 0 {
            uart_putc(if nib < 10 { b'0' + nib } else { b'a' + nib - 10 });
            started = true;
        }
    }
}

/// 打印 64 位十进制。
pub(crate) fn print_dec(n: u64) {
    let mut buf = [0u8; 20];
    let mut i = buf.len();
    let mut n = n;
    if n == 0 {
        i -= 1;
        buf[i] = b'0';
    }
    while n > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    let s = core::str::from_utf8(&buf[i..]).unwrap_or("?");
    print(s);
}
