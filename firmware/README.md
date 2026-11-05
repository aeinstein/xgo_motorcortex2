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
| 0005-pitch-yaw-coupling | `control/rider_leg_imu.cpp` | remove the rotation about the vertical from the tilt rates: pitch rate += yaw rate × tan(roll), roll rate −= yaw rate × tan(pitch) (angles clamped ±30°). Balancing with ~2° roll / ~1° pitch, a 40 °/s turn leaked ~1.5 °/s into the pitch gyro → 2–3° pitch error → Rider rolled back on left turns, forward on right turns (up to 0.3 m/s, measured 2026-11-05). Pitch-only correction was not enough at 80 °/s (roll estimate drifts ±2.5°). Unchanged without turning. Host test `firmware/tests/test_imu_yaw_coupling.cpp` (±40/80 °/s: up to 5.3° → ≤ 0.03°) |
| 0006-accel-freeze-while-turning | `control/rider_leg_imu.cpp` | while \|yaw rate\| > 25 °/s the filtered accelerometer angles are held: the IMU is not on the turning axis, centripetal acceleration (~1.4° apparent tilt at 85 °/s) pulled the fused pitch → Rider rolled forward on fast turns in both directions. Standing and straight driving unchanged (balance point incl. the uncalibrated pitch-gyro offset stays). Host test with lever arm ±0.11 m: 0005 alone ≤ 0.53°, with 0006 ≤ 0.03° |
| 0007-diag-register | `command_protocol.h/.cpp`, `main.cpp`, `motion_controller.h` | read reg `0x0e` (32 B, LE, refreshed every 10 ms): `u16 ms`, `i16` wheel speed 11/21 (mm/s, controller input), controller output 11/21, bus output 11/21, balance output, pitch target ×100 (°), pitch + bias ×100, hold correction ×10, `u8` hold mode, `u8` flags (bit0 driving, bit1 lift guard, bit2 turn hold), `i16` wheel-centre position (mm, wraps), hold position (mm), filtered forward ×10, filtered turn ×10. On the Pi: `echo "0x0e 32" > /proc/XGORider/raw_reg; cat /proc/XGORider/raw_reg` |
| 0008-turn-hold | `control/rider_motion.h/.cpp`, `motion_controller.h/.cpp`, `main.cpp` | position hold also while turning on the spot, on the wheel centre (both encoders; stock uses wheel 11 + leg servo 12 and stops holding outside turn byte 113..143), same effective gain (675 per m of real travel). Runtime switch, default **off**: action `0x3e = 241` on / `240` off, console `!turnhold on/off`. Host test `firmware/tests/test_turn_hold.cpp` |

Wheel 11 = left, 21 = right, 1024 ticks/rev, both positive forward. Host side: `xgo-drv` auto-detects 0x0d/0x0c ([../docs/driver.md](../docs/driver.md)).

Size check (same toolchain): patches 0001–0003 add 288 bytes (unpatched 1 242 864 → patched 1 243 152 bytes);
the binaries built earlier on Windows show the same +288 (1 243 824 → 1 244 112).

Flashed on the RiderPi since 2026-11-05 19:49: 0001–0008, 1 247 216 bytes, SHA-256 `c0d7441da7726ff228f6b3ee0a4a217c04fdf12d1419919a8aec4abbad47752f` (turn hold off by default; register 0x0e verified). 2026-11-05 19:28: 0001–0006, 1 245 920 bytes, SHA-256 `a8cb2107e9451d3308ef3faa37dd81a1f3cb51fe6fe83466d516c2449bb09318`. Turn test: 0.15 −1 / +11 cm, 0.30 +11 / +27 cm (left / right) – 0006 shows no measurable gain over 0005 alone. Before: 2026-11-05 18:52: 0001–0004 + first 0005 (pitch only), 1 245 824 bytes, SHA-256 `665c3d3a…` (full backup before: `debug_firmware/esp32_backup_vor0005_20261105_184307.bin`, SHA-256 `51645336…`; its app0 = the 0001–0004 image below). Turn test: 40 °/s fixed (1–2 cm instead of 20–40 cm), 80 °/s still 16–40 cm forward. 0005 (pitch + roll) without 0006, flashed 2026-11-05 19:05: 1 245 888 bytes, SHA-256 `3ccdcb29ebc40d9d878d9b10051f408ac8952b5c92db4489e139f13b6c77a803`. Turn test (3 s on the spot): 0.15 −4 / +9 cm, 0.30 +6 / +26 cm (left / right); before 0005: 20–40 cm. 2026-11-04 … 2026-11-05: 0001–0004.
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
