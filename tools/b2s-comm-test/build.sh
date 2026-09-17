#!/usr/bin/env bash
# 构建 b2s-comm-test(大核用户态测试程序,静态 musl,无解释器依赖)
# 产物:target/riscv64gc-unknown-linux-musl/release/b2s-comm-test
# 部署:加入安装器文件表(/tmp/recover-install/build.py 的 FILES)重跑安装,
#       或放进任何 extent 版 rootfs 的 /bin/。
set -euo pipefail
cd "$(dirname "$0")"

TARGET=riscv64gc-unknown-linux-musl
XT=${XUANTIE_DIR:-$HOME/Code/tpu-tennis/toolchains/xuantie-v3.4.0}

rustup target list --installed | grep -q "^${TARGET}$" || rustup target add "$TARGET"

# 静态链接(+crt-static):不受 rootfs 解释器布局影响;链接器用 Xuantie 包装版
RUSTFLAGS="-C target-feature=+crt-static -C linker=$XT/bin/riscv64-unknown-linux-musl-gcc" \
    cargo build --release --target "$TARGET"

BIN="target/${TARGET}/release/b2s-comm-test"
echo
echo "built: $BIN ($(stat -c %s "$BIN") bytes)"
file "$BIN" | head -1
