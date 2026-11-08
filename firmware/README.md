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
| 0005-pitch-yaw-coupling | `control/rider_leg_imu.cpp` | remove the rotation about the vertical from the tilt rates: pitch rate += yaw rate × tan(roll), roll rate −= yaw rate × tan(pitch) (angles clamped ±30°). Balancing with ~2° roll / ~1° pitch, a 40 °/s turn leaked ~1.5 °/s into the pitch gyro → 2–3° pitch error → Rider rolled back on left turns, forward on right turns (up to 0.3 m/s, measured 2026-11-05). Pitch-only correction was not enough at 80 °/s (roll estimate drifts ±2.5°). Unchanged without turning. Revised 2026-11-07: the tilt comes from the filtered accelerometer angles (frozen by 0006), not the fused estimate. The fused pitch sits ~2° high because the pitch gyro bias (~−1.2 °/s) is never calibrated; that made the roll estimate drift while turning (+1.3° left, −0.6° right) and gz·tan(roll) spoiled pitch (right turn at 80 °/s: 3° error, reg 0x10). In addition, 2.4 % of the yaw rate shows up in the pitch gyro (cross-axis, measured 2.3–2.5 % at ±80 °/s) and is subtracted. Host test `firmware/tests/test_imu_yaw_coupling.cpp` (±40/80 °/s: up to 5.3° → ≤ 0.03°) |
| 0006-accel-freeze-while-turning | `control/rider_leg_imu.cpp` | while \|yaw rate\| > 25 °/s the filtered accelerometer angles are held: the IMU is not on the turning axis, centripetal acceleration (~1.4° apparent tilt at 85 °/s) pulled the fused pitch → Rider rolled forward on fast turns in both directions. Standing and straight driving unchanged (balance point incl. the uncalibrated pitch-gyro offset stays). Host test with lever arm ±0.11 m: 0005 alone ≤ 0.53°, with 0006 ≤ 0.03° |
| 0007-diag-register | `command_protocol.h/.cpp`, `main.cpp`, `motion_controller.h` | read reg `0x0e` (32 B, LE, refreshed every 10 ms): `u16 ms`, `i16` wheel speed 11/21 (mm/s, controller input), controller output 11/21, bus output 11/21, balance output, pitch target ×100 (°), pitch + bias ×100, hold correction ×10, `u8` hold mode, `u8` flags (bit0 driving, bit1 lift guard, bit2 turn hold), `i16` wheel-centre position (mm, wraps), hold position (mm), filtered forward ×10, filtered turn ×10. On the Pi: `echo "0x0e 32" > /proc/XGORider/raw_reg; cat /proc/XGORider/raw_reg` |
| 0008-turn-hold | `control/rider_motion.h/.cpp`, `motion_controller.h/.cpp`, `main.cpp` | position hold also while turning on the spot, on the wheel centre (both encoders; stock uses wheel 11 + leg servo 12 and stops holding outside turn byte 113..143), same effective gain (675 per m of real travel). Runtime switch, default **off**: action `0x3e = 241` on / `240` off, console `!turnhold on/off`. Host test `firmware/tests/test_turn_hold.cpp` |
| 0009-yaw-bias-tracking | `control/rider_leg_imu.h/.cpp`, `command_protocol.h/.cpp`, `main.cpp` | stock measures the gyro-z bias once (0.5 s after boot) and integrates yaw only above 2 °/s; the bias then moves with temperature and stays invisible (measured 0.48 °/s off, 31° heading error in 75 s). `trackYawBias` compares 1 s windows of the gyro with a reference and follows the bias: while balancing the encoder difference (τ 15 s, only below 5–6 °/s, so fast turns with wheel slip do not count), while not driving standing still (τ 5 s, only if \|rate\| < 1.5 °/s and the accelerometer spread is < 0.5 m/s²). Yaw dead band 2 → 0.3 °/s. Read reg `0x0f` (12 B, LE): `i16` bias (m°/s), boot bias (m°/s), last error (m°/s), `u16` updates, `u8` reference (0 none, 1 wheels, 2 still), `u8` window open, `i16` yaw rate ×100. Host test `firmware/tests/test_yaw_bias.cpp` |
| 0010-turn-hold-default-on | `main.cpp` | 0008 on by default (2026-11-05 turn test: wheel centre ≤ 2.3 cm per 3 s turn, without hold up to 29 cm); action `240` still turns it off until the next start |
| 0011-no-yaw-deadband | `control/rider_leg_imu.h/.cpp` | yaw integrated at every rate (no dead band; stock 2 °/s, 0009 0.3 °/s), only after the boot calibration (bias is a partial sum before). Bias comes from 0009. Host test: 0.1 °/s for 60 s counted, error = boot-calibration residual only |
| 0012-keep-roll-trim | `control/rider_leg_imu.h/.cpp` | stock lets the roll target decay back to the roll command (0.95/tick) once leveling (`0x61`) is switched off; the Rider then stays level only while the leg servos sit in their dead zone (~2.6°), and every height change brought back ~2° roll (RiderPi 2026-11-07). Now the difference found by leveling (target − command, ±10°) is kept as a trim, also across height changes; reset with the leg state (stand-up, fall, disarm). Never leveled → bit-identical to stock. Host test `firmware/tests/test_roll_trim.cpp` |
| 0013-imu-diag-register | `control/rider_leg_imu.h/.cpp`, `command_protocol.h/.cpp`, `main.cpp` | diagnostics only, fusion unchanged: read reg `0x10` (32 B, LE, refreshed every 10 ms): `u16 ms`, `i16` raw roll gyro ×100 (°/s), filtered roll rate ×100, accel z ×100 (m/s²), accel pitch this sample ×100 (°), filtered accel pitch ×100, pitch ×100, raw pitch gyro ×100 (°/s), 0005 term gz·tan(roll) ×100, filtered pitch rate ×100, yaw rate ×10, accel roll ×100, filtered accel roll ×100, roll ×100, 0005 term −gz·tan(pitch) ×100, `u8` flags (bit0 0006 freezes, bit1 calibrated), `u8` sample counter. Host test `firmware/tests/test_imu_diag.cpp` |
| 0014-imu-stream | `imu_stream.h` (new), `command_protocol.h/.cpp`, `main.cpp` | IMU/wheel data stream instead of polling. Write reg `0x11` = rate in Hz (0 off, max 250; off after reset and after 5 s without bytes from the Pi on aux). Packets in the usual frame, type `0x14`, reg `0x11`, 34 B; payload LE: `u16 seq`, `u32 t_us` (micros of the IMU read), `float` roll/pitch/yaw (°), `s32` ticks wheel 11/21. Selected by measurement time (IMU is read every ~3 ms on the RiderPi, not 2 ms); control task only fills a lock-free ring, `loop()` sends and drops when the UART FIFO is full (gap in `seq`). Tested over USB 2026-11-08: 100/50/250 Hz → 102/51/250 packets/s, no gaps, no checksum errors; Pi driver unaffected. Aux path (Pi) needs driver support. Host test `firmware/tests/test_imu_stream.cpp` |

Wheel 11 = left, 21 = right, 1024 ticks/rev, both positive forward. Host side: `xgo-drv` auto-detects 0x0d/0x0c ([../docs/driver.md](../docs/driver.md)).

Size check (same toolchain): patches 0001–0003 add 288 bytes (unpatched 1 242 864 → patched 1 243 152 bytes);
the binaries built earlier on Windows show the same +288 (1 243 824 → 1 244 112).

Flashed on the RiderPi since 2026-11-08: 0001–0014, 1 250 080 bytes, SHA-256 `3cb084a8b4af637f128a7dbd63f274942360affde9f1c2cd4b5181593f74f86e`. 2026-11-07 ~17:15: 0001–0013, 0005 cross-axis 2.4 %, 1 249 040 bytes, SHA-256 `af4066033630c5a97e3d90d4d22b5bf1831e62596e7bcb0a17884d1d267591c8`. Band test 17:17 (360° left and back right, band near camera height): IMU pitch vs. band ≤0.25° from ~1 s after each turn (before: up to 2.3°); during right turns reg 0x10 shows pitch +1.2…+1.5° vs. standing and roll −0.6…−1.1° vs. accel ~0 (truth during the turn not measured). Before: 0005 with 1.9 % (turn test 17:05: pitch gyro residual ≤0.44 °/s, roll residual ≤0.45 °/s), SHA-256 `b42ea886b0e8fc010d37079d20389084664701f46f9398e68dc69ebfe59da5ed`; lying: roll gyro −0.06 °/s, pitch gyro −1.16 °/s. 2026-11-07 ~17:00: revised 0005, 1 248 960 bytes, SHA-256 `012a4c79e0b861491c0f37e7fd145b5482667ccec04c400eca05e2fa41375496`; turn test 16:55 (±78 °/s, reg 0x10): mean pitch during the turn vs. standing −0.84° right / −0.15° left (old 0005: −1.9° / −0.5°), 0.5–2.5 s after −0.25° / −0.58°; roll estimate still drifts while turning (+1.1° left, −0.6° right), no longer affects pitch. 2026-11-07 ~16:20: 0001–0013 (old 0005), 1 248 976 bytes, SHA-256 `e63c8ae049dd460d20701f417f86dc00077b6fab6869432ef640246e8eec68ab` (no backup; register 0x10 verified, lying: raw pitch gyro −1.16 °/s, never calibrated). 2026-11-07 15:53: 0001–0012, 1 248 560 bytes, SHA-256 `27841a5e0df02a636745ee2219fb0a2601656c503eb8e054c7f2ce6d7596787d` (full backup before: `debug_firmware/esp32_backup_vor0012_20261107_154426.bin`, SHA-256 `406b7f6e…`); after flash state 253, IMU 50 Hz, no serial timeouts; roll trim across height changes not yet checked on the Rider. 2026-11-05 21:23: 0001–0010, 1 248 576 bytes, SHA-256 `40dc57b8e88c4e19f7074c7f5658e94c1aff287cd0c4bbcf252cf5061c99d634` (registers 0x0e and 0x0f verified). Drive test 21:30: 60 s run ended IMU +13.4°, wheels +15.4° (before: −3.5° / +27.7°); wheel centre per 3 s turn −0.1 / +1.7 / +1.3 / +2.8 cm. Bias tracking while balancing got only one window every ~4 s (samples above 6 °/s discard the window); a real bias error while balancing was not exercised. 2026-11-05 19:49: 0001–0008, 1 247 216 bytes, SHA-256 `c0d7441da7726ff228f6b3ee0a4a217c04fdf12d1419919a8aec4abbad47752f` (turn hold off by default; register 0x0e verified). 2026-11-05 19:28: 0001–0006, 1 245 920 bytes, SHA-256 `a8cb2107e9451d3308ef3faa37dd81a1f3cb51fe6fe83466d516c2449bb09318`. Turn test: 0.15 −1 / +11 cm, 0.30 +11 / +27 cm (left / right) – 0006 shows no measurable gain over 0005 alone. Before: 2026-11-05 18:52: 0001–0004 + first 0005 (pitch only), 1 245 824 bytes, SHA-256 `665c3d3a…` (full backup before: `debug_firmware/esp32_backup_vor0005_20261105_184307.bin`, SHA-256 `51645336…`; its app0 = the 0001–0004 image below). Turn test: 40 °/s fixed (1–2 cm instead of 20–40 cm), 80 °/s still 16–40 cm forward. 0005 (pitch + roll) without 0006, flashed 2026-11-05 19:05: 1 245 888 bytes, SHA-256 `3ccdcb29ebc40d9d878d9b10051f408ac8952b5c92db4489e139f13b6c77a803`. Turn test (3 s on the spot): 0.15 −4 / +9 cm, 0.30 +6 / +26 cm (left / right); before 0005: 20–40 cm. 2026-11-04 … 2026-11-05: 0001–0004.
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
