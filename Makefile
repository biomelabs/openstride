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

.PHONY: all clean flash debug setup teardown unit

all:
	uv run west build -b $(BOARD) -d $(BUILD_DIR) $(APP)

clean:
	rm -rf $(BUILD_PARENT)

flash:
	uv run west flash --runner openocd -d $(BUILD_DIR)

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

unit:
	uv run west twister -T tests/unit --inline-logs -O results

teardown:
	rm -rf nrf/ vendor/ ../.west/ .venv/
