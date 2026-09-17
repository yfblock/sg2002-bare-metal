# cvirtos — SG2002 C906L 小核 M-mode 裸机固件
#
# 目标:
#   make / make build   构建 cvirtos.bin(cargo release + objcopy)
#   make deploy         部署到 tftp 目录(板子 U-Boot 取的路径,默认 /srv/tftp)
#   make test           一键真机回归:构建 → 部署 → boardctl run b2sbm
#                       (米家冷启动 → 小核 cvirtos + 大核 b2s-comm-test-bm,
#                        断言 S2B 帧流/B2S 往返/YUV/总体 PASS,测完自动断电)
#   make doc            生成文档(cargo doc --no-deps)
#   make clean          清理构建产物
#
# 可覆盖变量:
#   OBJCOPY / OBJDUMP  默认 rust-objcopy / rust-objdump(cargo-binutils)
#   TFTP_DIR           部署目录,默认 /srv/tftp
#
# 前置(一次性,已配好):boardctl 板卡配置 ~/.config/boardctl/boards/sg2002.toml
# 的 run.b2sbm 目标,与 /srv/tftp/b2sbm-boot.scr 启动脚本。

TARGET   := riscv64gc-unknown-none-elf
ELF      := target/$(TARGET)/release/cvirtos
BIN      := cvirtos.bin
TFTP_DIR ?= /srv/tftp
TEST_LOG := target/b2sbm-last.log
# cargo-binutils 装在 ~/.cargo/bin;某些非登录 shell 不带它,统一前置(可被外部 PATH 覆盖时仍生效)
export PATH := $(HOME)/.cargo/bin:$(PATH)

# objcopy/objdump 用 cargo-binutils 的 rust-xxx(环境变量可覆盖)
OBJCOPY ?= rust-objcopy
OBJDUMP ?= rust-objdump

.PHONY: all build deploy test doc clean

all: build

build:
	@rustup target list --installed 2>/dev/null | grep -q "^$(TARGET)$$" \
		|| { echo "rust target $(TARGET) 未安装,执行 rustup target add"; rustup target add $(TARGET); }
	cargo build --release
	$(OBJCOPY) -O binary $(ELF) $(BIN)
	@echo "built: $(BIN) ($$(stat -c %s $(BIN)) bytes, entry 0x8FE00000)"
	$(OBJDUMP) -f $(ELF) | grep -i "start address"

deploy: build
	cp $(BIN) $(TFTP_DIR)/$(BIN)
	@echo "deployed: $(TFTP_DIR)/$(BIN) ($$(stat -c %s $(TFTP_DIR)/$(BIN)) bytes)"

test: deploy
	@echo "==== 真机回归 boardctl run b2sbm(冷启动,断言 PASS,自动断电) ===="
	@mkdir -p target
	@boardctl run b2sbm > $(TEST_LOG) 2>&1; st=$$?; cat $(TEST_LOG); \
		if [ $$st -eq 0 ]; then \
			echo "==== ALL PASS:build + deploy + 真机回归 ===="; \
		else \
			echo "==== FAIL:完整日志见 $(TEST_LOG) ===="; \
		fi; exit $$st

doc:
	cargo doc --no-deps

clean:
	cargo clean
	rm -f $(BIN)
