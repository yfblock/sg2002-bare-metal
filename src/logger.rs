//! 把 sg200x-bsp 的 `log::info!/warn!` 路由到 UART0（诊断用）。
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

struct UartLogger;
static LOGGER: UartLogger = UartLogger;
static INIT: AtomicBool = AtomicBool::new(false);

pub fn init() {
    if INIT.swap(true, Ordering::SeqCst) {
        return;
    }
    let _ = log::set_logger(&LOGGER);
    // UART0 与大核 StarryOS 共用：小核刷日志会把大核 boot 输出冲掉（互相截断）。
    // 调试大核时用 Off 让大核独占串口；要看小核 UVC/JPU 日志时临时改回 Info。
    log::set_max_level(log::LevelFilter::Off);
}

impl log::Log for UartLogger {
    fn enabled(&self, m: &log::Metadata) -> bool {
        m.level() <= log::Level::Info
    }
    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let mut w = UartWriter;
        let _ = writeln!(w, "[{}] {}", record.level(), record.args());
    }
    fn flush(&self) {}
}

struct UartWriter;
impl core::fmt::Write for UartWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        crate::platform::uart::print(s);
        Ok(())
    }
}
