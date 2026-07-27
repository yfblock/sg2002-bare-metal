#!/usr/bin/env bash
# 构建 C906L 小核 M-mode 裸机 RTOS → cvirtos.bin
# 产物：cvirtos.bin（raw binary，load/entry = 0x88000000）
# 部署：cp cvirtos.bin /srv/tftp/cvirtos.bin
#       （cmd.txt: tftp 0x88000000 cvirtos.bin; cvi_reset_c906l 0x88000000）
set -euo pipefail
cd "$(dirname "$0")"

TARGET=riscv64gc-unknown-none-elf

if ! rustup target list --installed 2>/dev/null | grep -q "^${TARGET}$"; then
  echo "rust target $TARGET 未安装，执行: rustup target add $TARGET" >&2
  rustup target add "$TARGET"
fi

cargo build --release

ELF="target/${TARGET}/release/cvirtos"
rust-objcopy -O binary "$ELF" cvirtos.bin

echo
echo "built: cvirtos.bin ($(stat -c %s cvirtos.bin) bytes)"
rust-objdump -f "$ELF" | grep -i "start address"
echo "部署: cp cvirtos.bin /srv/tftp/cvirtos.bin"
