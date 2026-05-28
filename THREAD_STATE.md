# Thread State (Quick Handoff)

## TL;DR
- Firmware and app version are aligned at `1.0.0`.
- `main.ino` builds cleanly for `uno_r4_wifi` and `uno_r4_minima`.
- Firmware upload to `COM6` (`uno_r4_wifi`) succeeded.
- Firmware now exposes version via Modbus registers; `main.py` checks and warns on mismatch.
- Use `_buildcheck/pio_main/platformio.ini` plus the explicit PlatformIO exe path on this PC.

## Current Status
- Firmware (`ino/main/main.ino`) and desktop app (`main.py`) are aligned to version `1.0.0`.
- Firmware was built successfully for both UNO R4 targets (`uno_r4_wifi`, `uno_r4_minima`).
- Firmware was uploaded successfully to `COM6` (UNO R4 WiFi).

## Key Functional Changes Completed
- Removed loop-time baseline refresh in firmware so hold-restore baseline is not constantly rewritten.
  - Removed `capture_restore_channels_from_tmp()` call from `loop()`.
  - Kept baseline capture in `Start()` only.
  - Removed redundant setup-time capture call so baseline is established at start.
- Added firmware version source-of-truth in `main.ino`:
  - `FW_VERSION_MAJOR`, `FW_VERSION_MINOR`, `FW_VERSION_PATCH` = `1`, `0`, `0`.
  - `firmware_version_text()` returns `"1.0.0"`.
- Exposed firmware version over Modbus as readback registers:
  - `fw_version_major`, `fw_version_minor`, `fw_version_patch` appended in `regs_t`.
  - `refresh_version_registers()` updates these each loop after `slave.loop(...)` to keep them read-only from host side.
- Added Python-side version check in `main.py`:
  - `EXPECTED_FIRMWARE_VERSION = "1.0.0"`.
  - Reads firmware version after connect/start and warns if mismatch or unreadable.
- Earlier Modbus robustness fixes in `main.py` were applied:
  - Proper exception-frame handling for read/write responses.
  - Write echo validation.
  - Start-in-progress guard and safer shutdown behavior.

## Important Technical Notes
- On UNO R4, `word` is 16-bit (`uint16_t`), so the raw register array must match struct size.
- `regs_t.raw` is set to `word raw[16+MAX_COUNT];` (includes 3 new version registers and 32-bit fields split into 2 words).
- Python version register base logic:
  - `version_reg = REG_CHANNEL0 + max_count + 6`
  - reads 3 words => `major.minor.patch`

## Build/Upload Setup Discovered on This PC
- `platformio` is **not** on PATH in this shell.
- Working executable:
  - `C:\Users\I Love Judy\.platformio\penv\Scripts\platformio.exe`
- Use these environment variables when invoking:
  - `PLATFORMIO_CORE_DIR=d:\Coding\Python\Modbus\.pio-core`
  - `PYTHONIOENCODING=utf-8`
- Build project config used for this sketch:
  - `_buildcheck/pio_main/platformio.ini`
  - `src_dir = ../../ino/main`
  - `lib_dir = ../../ino/lib`
  - Envs: `uno_r4_wifi`, `uno_r4_minima`

## Known Good Commands
```powershell
$env:PLATFORMIO_CORE_DIR='d:\Coding\Python\Modbus\.pio-core'
$env:PYTHONIOENCODING='utf-8'
& 'C:\Users\I Love Judy\.platformio\penv\Scripts\platformio.exe' run -d _buildcheck/pio_main -e uno_r4_wifi
& 'C:\Users\I Love Judy\.platformio\penv\Scripts\platformio.exe' run -d _buildcheck/pio_main -e uno_r4_minima
& 'C:\Users\I Love Judy\.platformio\penv\Scripts\platformio.exe' run -d _buildcheck/pio_main -e uno_r4_wifi -t upload --upload-port COM6
```

## Device/Port Info Observed
- `COM6` detected as:
  - `USB Serial Device (COM6)`
  - `VID:PID=2341:1002`
  - mapped/used as UNO R4 WiFi target.

## Last Verified Outcomes
- `main.py` compiles (`python -m py_compile main.py`).
- PlatformIO build results:
  - `uno_r4_wifi`: SUCCESS
  - `uno_r4_minima`: SUCCESS
- Upload result:
  - `uno_r4_wifi` to `COM6`: SUCCESS
