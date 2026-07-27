# bare-metal (cvirtos) — SG2002 C906L 小核 M-mode 裸机 RTOS + sg200x-bsp UVC 抓图

跑在 SG2002 **C906L 小核**上、**M-mode** 的 Rust 裸机 RTOS，**复用 sg200x-bsp 的 USB/UVC
协议栈**从 USB 摄像头抓 MJPEG 图像。大核 C906B 仍跑 U-Boot，小核独立驱动 DWC2 + 摄像头。

## 关键点

- **sg200x-bsp 是 no_std**，可直接被裸机引用；USB/UVC 代码原样在 C906L M-mode 工作。
- 小核与大核**共享 SoC 地址空间**，裸机无 MMU（identity 映射，VA=PA），故 `set_dwc2_base_virt`
  等直接传物理基址，`set_usb_dma_to_phys_fn(None)`（VA=PA）。
- DMA 缓冲（sg200x-bsp 的 `DMA_BUF`，~385 KiB）落在本镜像 `.bss` @ 0x880xxxxx，DWC2 32-bit
  HCDMA 直接写该地址（在 DRAM 内，DMA 可达）。
- UVC 走**轮询**（sg200x-bsp 的 ep0.rs 不依赖中断），故无需 trap/中断。

## 架构

- **目标**：`riscv64gc-unknown-none-elf`（与 sg200x-bsp 一致，C906L 是 rv64imafdc）。
- **加载/入口**：0x88000000，`_start`（global_asm）关 mie/设栈/清 bss/调 `rust_main`。
- **UART**：UART0 @ 0x04140000（DW 8250，32-bit 步长）；U-Boot 已初始化，直接写 THR。
- **logger**（`logger.rs`）：把 sg200x-bsp 的 `log::info!/warn!` 路由到 UART，看枚举过程。
- **平台初始化**（`platform.rs`）：时钟/PHY/VBUS/pinmux（参考 cvitek `phy-cv1800-usb.c`），
  identity 映射下直接用 `soc` 物理基址。
- **UVC**：`UvcSession::open`（枚举→PROBE/COMMIT→warmup）+ `capture_recovering`（抓帧，
  带错误恢复）。偏好 640×480 MJPEG。
- bin ~66 KiB；.bss ~393 KiB（含 DMA_BUF）+ 64 KiB 栈，全部在 2 MiB 区内。

## 文件

- `src/main.rs` — 入口：`_start` + `rust_main`（platform_init → `UvcSession::open` → 抓帧循环，
  打印每帧尺寸 + SOI/EOI 校验 + 首 8 字节 hex）。
- `src/platform.rs` — USB 平台初始化（时钟/PHY/VBUS/pinmux/DWC2 基址/DMA 转换）。
- `src/logger.rs` — `log::Log` → UART。
- `src/uart.rs` / `src/util.rs` / `src/panic.rs` — UART、忙等、panic。
- `memory.ld` / `.cargo/config.toml` / `build.sh` — 链接、构建。

## 构建 + 真机

```bash
./build.sh                                   # → cvirtos.bin（~66KB，entry=0x88000000）
cp cvirtos.bin /srv/tftp/cvirtos.bin
# /srv/tftp/cmd.txt: bootcmd=tftp 0x88000000 cvirtos.bin; cvi_reset_c906l 0x88000000
sg2002-ctl 0; sleep 2; sg2002-ctl 1; tio /dev/ttyUSB0 -b 115200
```

串口（开头几行 U-Boot 与小核共用 UART0 会短暂交错）：
```
=== C906L UVC capture via sg200x-bsp (SG2002 small core, M-mode) ===
initializing USB platform (clocks/PHY/VBUS)...
  clocks / phy bringup / pinmux / vbus / bases set
platform init done
[UVC] opening session...
[INFO] [USB] root dev@0 VID=1a40 PID=0101 dev_class=09        ← hub
[INFO]   [USB] dev@0 (hub 1 port 1) VID=0c45 PID=64ab ...      ← 摄像头
[INFO] UVC-session: open addr=2 VID=0c45 PID=64ab ep0_mps=64
[INFO] UVC: SEL ... 640x480 ... mjpeg=true
[INFO] UVC: streaming armed ...
[UVC] open ok: 640x480 dev=2
[frame 1] 19087B  SOI=ok EOI=ok head=ff d8 ff e0 00 21 41 56
[frame 2] ...
...
[frame 122] 73245B  SOI=ok EOI=ok head=ff d8 ff e0 00 21 41 56
```
连续抓到 100+ 帧，每帧 `SOI=ok`(ff d8) `EOI=ok`(ff d9)——合法 MJPEG。

## 备注

- 摄像头经一个 USB hub（1a40:0101）接到 DWC2；sg200x-bsp 的递归 hub 枚举正确处理。
- 与 SD rootfs 无关：只走 U-Boot TFTP + `cvi_reset_c906l`。
- 小核未启用 cache（identity + uncached），sg200x-bsp 的 cache 维护指令在 uncached 下无害，
  DMA 与 CPU 一致性自然满足。
- git 历史另有「抢占式定时器调度器」版（trap.rs/sched.rs）和「async 协作式」版；本版为
  UVC 抓图聚焦，单任务轮询（未启用调度器/中断）。可在此基础上把抓帧放进一个抢占式任务。
