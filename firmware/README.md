# ESP32 firmware (R-1.1.6 + patches)

Builds the ESP32 motion-controller firmware for the RiderPi from the public reference
re-implementation plus local patches. Does **not** flash.

```
firmware/build.sh [rider2-firmware|rider2-trained|rider2-rl-only]   # default: rider2-firmware
PIO=~/.platformio/penv/Scripts/platformio.exe firmware/build.sh      # use an existing PlatformIO
```

Runs on Linux/macOS and on Windows in Git Bash (Python ≥ 3.8 as `python3` or `python`, override with `PYTHON=`).

| Item | Value |
|---|---|
| Reference | [LuwuDynamics/xgorider2_rl](https://github.com/LuwuDynamics/xgorider2_rl), commit `f534986` (`REF_COMMIT` to override) |
| Build system | PlatformIO 6.1 in `firmware/build/venv` (or `PIO=`), `espressif32@6.9.0`, Arduino 3.20017, `esp32dev`, 4 MB DIO |
| Env on RiderPi | `rider2-firmware` (classic PID balance) |
| Output | `firmware/out/app0.bin`, `app0.bin.sha256`, `app0.build.txt` (ref commit, env, patch hashes, PlatformIO version) |
| Work dir | `firmware/build/` (clone, venv, `.pio`), gitignored |
| First run | downloads toolchain/framework (~1 GB), ~2 min build on a PC |

## Patches (`firmware/patches/`)

| Patch | Files | Change |
|---|---|---|
| 0001-wheel-registers | `command_protocol.h/.cpp`, `main.cpp` | read reg `0x0c` (16 B): wheel 11/21 ticks, wheel 11/21 raw angle 0..1023 (int32 LE); read reg `0x0d` (24 B): roll, pitch, yaw (float LE) + wheel 11/21 ticks (int32 LE) + `t_ms` (uint32 LE), one snapshot |
| 0002-action-254-disarm | `main.cpp` | action `0x3e = 254` → `emergencyStop()` (latched like `!stop`); 255 = arm unchanged; 0 does not stop |
| 0003-led-brightness-half | `board_support.cpp` | LED scale 129/256 → 64/256 |
| 0004-standup | `main.cpp` | action `0x3e = 253` / `!standup`: arm + 3 s start window at \|pitch\| < 40°, \|roll\| < 10° (stand up from lying); raw classic wheel PID until upright (\|pitch\| < 8°, \|roll\| < 5° for ~160 ms); not upright after 2 s → latched tilt stop; `!stop`/254/calibration abort; `!status` `"standup"` = idle/requested/active |

Wheel 11 = left, 21 = right, 1024 ticks/rev, both positive forward. Host side: `xgo-drv` auto-detects 0x0d/0x0c ([../docs/driver.md](../docs/driver.md)).

Size check (same toolchain): patches 0001–0003 add 288 bytes (unpatched 1 242 864 → patched 1 243 152 bytes);
the binaries built earlier on Windows show the same +288 (1 243 824 → 1 244 112).

Flashed on the RiderPi since 2026-11-04: 0001–0004. Windows build (PlatformIO 6.1.19) is bit-identical
to that binary: 1 245 136 bytes, SHA-256 `0e5c0887e1d70ad6071ace20870bcaa887a8104941ee27e2e150a34849ef1031`.

## Flash (manual, after review)

```
python -m esptool --port <PORT> --baud 115200 read-flash 0 0x400000 backup.bin   # full backup first
python -m esptool --port <PORT> --baud 115200 write-flash 0x10000 firmware/out/app0.bin
```

| Rule | Reason |
|---|---|
| Only app0 (`0x10000`) | partition table identical to stock Rider2 |
| Never write `0x310000–0x32ffff` | leg zero points (`0x310000`), BLE name (`0x320000`), pitch trim (`0x321000`) |
| Never write spiffs | |
| USB port changes after resets (COM3 ↔ COM4) | check port before each esptool call |
| After flash: disarmed (`reason: startup`) | action 255 (web: Aufstehen) to balance; wheels start once upright |

Restore stock R-1.1.3: write app0 from the full backup (`esp32_app_app0_0x10000.bin`) to `0x10000`.
