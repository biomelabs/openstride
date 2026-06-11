PROJECT ?= openstride
BOARD ?= xiao_nrf54l15/nrf54l15/cpuapp
APP ?= .

# per-board build dir to avoid stale CMake cache when switching boards
BUILD_PARENT ?= build
BUILD_DIR ?= $(BUILD_PARENT)/$(subst /,_,$(BOARD))

BOARD_ID_PATTERN ?= XIAO.nRF

# pass ANT=0 to exclude ant-nrfconnect/sdk-ant
ANT ?= 1

UF2_PATH = $(BUILD_DIR)/$(PROJECT)/zephyr/zephyr.uf2
MERGED_HEX = $(BUILD_DIR)/merged.hex

# optional CMSIS-DAP probe serial (e.g. PROBE=06E4B471)
PROBE ?=
# pyOCD target name for erase/flash/recover targets
TARGET ?= nrf54l

.PHONY: all clean erase flash recover debug setup teardown unit

all:
	uv run west build -b $(BOARD) -d $(BUILD_DIR) $(APP)

clean:
	rm -rf $(BUILD_PARENT)

# mass-erase only, unlocks write-protection (e.g. APPROTECT) without flashing
erase:
	uv run python scripts/recovery.py --mode erase \
		$(if $(PROBE),--probe $(PROBE),) --target $(TARGET)

# flash without erase, builds first
flash: all
	uv run python scripts/recovery.py --mode flash --firmware $(MERGED_HEX) \
		$(if $(PROBE),--probe $(PROBE),) --target $(TARGET)

# full recovery (mass-erase then flash), builds first
recover: all
	uv run python scripts/recovery.py --mode factory --firmware $(MERGED_HEX) \
		$(if $(PROBE),--probe $(PROBE),) --target $(TARGET)

copyfw:
	@if [ ! -f "$(UF2_PATH)" ]; then \
		echo "ERROR: $(UF2_PATH) not found."; \
		echo "       Run `make all` first."; \
		exit 1; \
	fi; \
	UF2_MOUNT=$$(find /mnt -maxdepth 2 -name "INFO_UF2.TXT" 2>/dev/null | head -1 | sed 's|/INFO_UF2.TXT||'); \
	if [ -z "$$UF2_MOUNT" ]; then \
		echo "ERROR: No UF2 drive found under `/mnt`."; \
		echo "       Double-tap reset to enter bootloader mode."; \
		exit 1; \
	fi; \
	BOARD_LINE=$$(grep -i "Board-ID" "$$UF2_MOUNT/INFO_UF2.TXT" 2>/dev/null | head -1); \
	if ! echo "$$BOARD_LINE" | grep -qiE "$(BOARD_ID_PATTERN)"; then \
		echo "ERROR: Expected a board ID like '$(BOARD_ID_PATTERN)', got: '$$BOARD_LINE'"; \
		echo "       Override with: make copyfw BOARD_ID_PATTERN=<pattern>"; \
		exit 1; \
	fi; \
	echo "Board:  $$BOARD_LINE"; \
	echo "Target: $$UF2_MOUNT"; \
	cp "$(UF2_PATH)" "$$UF2_MOUNT/zephyr.uf2" && \
	echo "Done."

debug:
	uv run west debug --runner openocd -d $(BUILD_DIR)

setup:
	uv sync
	@if [ ! -d ../.west ]; then uv run west init -l .; fi
	@if [ "$(ANT)" = "1" ]; then \
		uv run west update --group-filter +ant; \
	else \
		uv run west update; \
	fi
	uv run west zephyr-export
	@if [ "$$(uname -s)" = "Darwin" ]; then \
		uv run west sdk install -t arm-zephyr-eabi; \
	else \
		uv run west sdk install -t arm-zephyr-eabi x86_64-zephyr-elf; \
	fi

unit:
	uv run west twister -T tests/unit --inline-logs -O results

teardown:
	rm -rf vendor/ ../.west/ .venv/
