# ESP32 firmware (R-1.1.6 + patches)

Builds the ESP32 motion-controller firmware for the RiderPi from the public reference
re-implementation plus 8 topic patches. Does **not** flash.

```
firmware/build.sh [rider2-firmware|rider2-trained|rider2-rl-only]   # default: rider2-firmware
PIO=~/.platformio/penv/Scripts/platformio.exe firmware/build.sh      # use an existing PlatformIO
firmware/tests/run.sh [filter]                                       # host tests (g++, after build.sh)
```

| Item | Value |
|---|---|
| Reference | [LuwuDynamics/xgorider2_rl](https://github.com/LuwuDynamics/xgorider2_rl), commit `f534986` (`REF_COMMIT` to override) |
| Build system | PlatformIO 6.1 in `firmware/build/venv` (or `PIO=`), `espressif32@6.9.0`, Arduino 3.20017, `esp32dev`, 4 MB DIO |
| Env on RiderPi | `rider2-firmware` (classic PID balance) |
| Output | `firmware/out/app0.bin`, `app0.bin.sha256`, `app0.build.txt` (ref commit, env, patch hashes, PlatformIO version) |
| Work dir | `firmware/build/` (clone, venv, `.pio`), gitignored; builds are reproducible (same SHA) |
| Hosts | Linux/macOS, Windows in Git Bash (Python ≥ 3.8, `PYTHON=` to override); host tests need g++ (GPU host) |

## Patches (`firmware/patches/`, applied in order)

Each patch builds on its own on top of the previous ones.

| Patch | Content | Registers / actions | Host tests |
|---|---|---|---|
| 01-ble | Own BLE service `RiderKerk` instead of the vendor one (fff0/fff1/fff2, name from flash `0x320000`, trace stream `0xb3`): characteristics CMD (binary frames), CON (text console), PI (tunnel to the Pi), IP. No pairing. Console output goes to the channel the line came from ([../docs/ble.md](../docs/ble.md)) | tunnel: frames type `0x15` / write `0x17`; `0x18` = IPv4 of the Pi (written by the Pi, 0.0.0.0 after 10 s) | `test_ble_tunnel` |
| 02-pi-interface | Read `0x0d` (24 B): roll, pitch, yaw (float) + wheel 11/21 ticks (int32) + `t_ms`. IMU/wheel stream: write `0x11` = rate in Hz (≤ 250, off after 5 s without bytes from the Pi), packets type `0x14`, 26 B (`imu_stream.h`) | `0x0d`, `0x11` | `test_imu_stream` |
| 03-diagnostics | `0x0e` (32 B): controller values (layout `main.cpp` `fillDiag`). `0x15` (32 B): running sums of the raw IMU samples (`tools/rawdiag.py`). `0x19` (32 B): lift-guard log that survives stops (`lift_log.h`, `fillLiftDiag`; Pi: `/proc/XGORider/lift`), console `!liftguard status` | `0x0e`, `0x15`, `0x19` | `test_raw_diag`, `test_lift_log` |
| 04-attitude | Attitude as unit vector "up" in body axes, rotated by the gyro, pulled to the accelerometer with 8 s (weight by magnitude and rate, v·yaw rate subtracted). Bias of all three gyros from the boot calibration, which only counts at standstill (span < 1.5 °/s, < 0.5 m/s²; after 20 rejected windows as stock); roll/pitch bias tracked (40 s), yaw bias tracked from wheels/standstill. Yaw integrated without dead band from the rate about the vertical. Pitch to the outside is the true pitch; controller and guards get pitch + 2.05° (the stored pitch trim was learned with the stock ~2° error) | — | `test_lagefusion_replay`, `test_imu_yaw_coupling`, `test_pitch_fusion`, `test_yaw_bias`, `test_gyro_boot_cal` |
| 05-actions | 254 = latched stop; 253 / `!standup` = stand up from lying (3 s window, raw wheel PID until upright, 2 s timeout); 237 / `!restart` = ESP32 restart when not balancing (also during low battery); stored actions as table `kActions` (1–6 as stock, new ones 7–230); low-battery state 250 clears after 5 s ≥ 15 % (→ 253); no self-start after a fall (state 0 stays until 255/253); stock `automatic_lean` removed (never set) | actions 254, 253, 237 | `test_action_table`, `test_battery_recover` |
| 06-leds | Global LED brightness n/256 (default 32); spot circling in the user colours while the gyro calibrates (0.4 s per turn); no roll/height wave colours, user colours shown whenever the firmware has no picture of its own, pink again on arming; all LEDs blue while the Pi shuts down | `0x12`, `!led N`; actions 234 on / 233 off, `!shutdown on/off` | `test_led_calibrating` |
| 07-balance | Position hold also while turning on the spot (wheel centre, 675/m); roll trim from leveling kept across height changes; roll held at 0 by the legs while balancing without roll command (also straight); byte path: speed integral as floor (30 % of the command, ≤ 3°); lean into fast turns: roll target −atan(v·ω/g), ≤ 6°, leg margins, low-pass 0.08 s, target changes fed forward to the legs (default **off**) | actions 232 on / 231 off, `!lean on/off`; `0x0e` byte 23 bit 4 | `test_turn_hold`, `test_roll_trim`, `test_drive_integral`, `test_roll_hold`, `test_curve_lean` |
| 08-velocity | Drive frame in real units instead of bytes: write `0x1a` (13 B: v mm/s, a, jerk, w 0.1 °/s, alpha, turn jerk; flag bit0 = acceleration mode, bit1 = curvature mode: field w = κ 0.001 1/m, w = κ·v_target, v/a/jerk limited by w_max/alpha/turn jerk over |κ| so the path stays on the circle, κ follows with |dκ/dt| ≤ alpha/|v|; bit2 = course target: field w = Δψ 0.1° relative to the generator angle at reception, turns with w_max/alpha/turn jerk without overshoot (stopping-distance look-ahead), arrived at w = 0 and |rest| < 0.5°, then the heading PID keeps holding the target while course frames arrive (also standing; stock logic dropped it: 2–4 % short), with an integral (ki 0.04 per call, deadband 0.3°) only while holding still, because P alone stays below the friction threshold under ~3° (1.4–2.2° left); bit4 = toggle bit, only a changed toggle sets a new target, repeats only feed the watchdog; bit3 = distance with stop: 15-byte frame, byte 13 u16 remaining distance in mm, every frame resets it (no toggle), it shrinks by the measured wheel distance, the speed axis only steps towards v if the stopping distance still fits into rest − lag (lag = ∫(v_target − v_measured), decays while standing), reached at ≤ 5 mm; with bit1 the arc length counts; bit5 = absolute course target: 5-byte frame (flags 0x20 | i16 ψ_goal 0.01° in the firmware heading, mod 360 | u16 w_max 0.1 °/s, 0 = 0x1b), rest = wrap180(ψ_goal − heading PID target) every tick (shorter way, exactly opposite turns right), every frame only moves the goal, the generator keeps its state (no toggle, changed goal or w_max taken over smoothly), alpha/turn jerk from `0x1b` (18 B), speed axis brakes to 0 with a/jerk of the last frame, arrived as bit2, reopens only at |rest| > 1°; bit1+bit2, bit4 without bit2, bit3 without 15 bytes or with bit0/bit2, bit5 not alone in 5 bytes, bit5 without 18-byte limits and unknown bits reject the frame), limits + watchdog `0x1b` (14 or 18 B: v_max, w_max, timeout, unit µm/s per wheel unit, bytes 8–13 ignored, 14/16 alpha °/s² and turn jerk °/s³ of the absolute course target; older firmware rejects 18 B, read capability bit4 first; written by xgonav at start, frames are rejected without them), read `0x1a` (12 B state; byte 1 = capabilities, bit0 curvature, bit1 course target, bit2 distance, bit4 absolute course target; flag bit4 = curvature mode, bit5 = course target active, bit6 = course reached, bit7 = distance reached). Jerk-limited set-point generator; speed control: stock P 0.11 on target − measured (0.14 while holding) plus (kp 0.15 − 0.11) on the error clamped to ±40 units + integral ki 0.06 on the error clamped to ±40 units, i_max 8° (anti-windup, from the end of the ramp, not during it; carries offset and slope) + feed-forward from the target only: atan(a_target/g) − kf·v_target (kf 0.05 ° per unit, the pitch the rider needs to hold a speed), gains in the firmware, folded into the decaying integral when the frame ends; heading target advances by exactly w·dt. The last written path wins (byte, neutral, stop, action ≤ 230, 253, arming end the frame). `0xa0/0xa1` removed (fixed 210 mm/s / 300 °/s). Layouts: `velocity_cmd.h` | `0x1a`, `0x1b`; `!vel …` (also `!vel kappa V K`, `!vel turn DEG`, `!vel dist V S`), `!velctl KI FF IMAX [KP [KF]]`, `!turnctl KP KD FF [KI_HOLD]` (RAM only) | `test_velocity_cmd` (with `XGO_VECTORS=1`: test vectors `tests/vectors/*.csv` for the Pi rebuild), `test_velocity_ctrl`, `test_turn_ctrl` |

Wheel 11 = left, 21 = right, 1024 ticks/rev, both positive forward. Host side: [../docs/driver.md](../docs/driver.md).

## State on the Rider

| Item | Value |
|---|---|
| Flashed | 2026-11-25 ~14:05: old series 0001–0035 (= patches 01–08 before the dead code was removed), SHA-256 `ef2a2894…`, 1 267 760 B; fallback `out/app0_0035neu.bin` |
| Built, **not flashed** | patches 01–08: SHA-256 `97d8cf13…`, 1 266 128 B. Differs from the flashed one only by removed dead code (see Holzwege: 0x0c, 0x0f, 0x10, 0x16, 0x17 read, switches 241/240, 239/238, 236/235); behaviour as with the switches on |
| Driven | velocity frame (speed −4…+11 %, turn rate 1 %), attitude fusion, roll hold |
| Never triggered on the Rider (host tests only) | low-battery recovery, lift log after a lift, no self-start after a fall |

## Open

| Item | Note |
|---|---|
| Stand-up fails sometimes | state 251 (actuator fault, left leg holds) 0.2 s after start, 3 of 4 tries on 2026-11-17; cause open; 237 (restart) clears it |
| Control loop 260 instead of 282 Hz while balancing | attitude maths in double; float would help |
| Curve lean sign | before switching it on (232): drive a curve, check that roll goes to the inside (IMU stream roll vs. yaw rate) |
| BLE | CMD and CON over BLE, `wlan connect`, throughput not verified |

## Flash (manual, after review)

```
python -m esptool --port <PORT> --baud 115200 write-flash 0x10000 firmware/out/app0.bin
```

| Rule | Reason |
|---|---|
| Only app0 (`0x10000`) | partition table identical to stock Rider2 |
| Never write `0x310000–0x32ffff` | leg zero points (`0x310000`), vendor BLE name (`0x320000`, unused), pitch trim (`0x321000`) |
| Never write spiffs | |
| No full backup (`read-flash`) before flashing | enough backups exist (~30) |
| USB port changes after resets (COM3 ↔ COM4) | check port before each esptool call |
| After flash: disarmed (`reason: startup`) | action 255 or 253 to balance |

Restore stock R-1.1.3: write app0 from the full backup (`esp32_app_app0_0x10000.bin`) to `0x10000`.

## Holzwege

Dead ends of the old series 0001–0039 (2026-11-04 … 11-25), one line each. The old patch files are in git history before the consolidation.

| Tried | Why dropped |
|---|---|
| LED scale fixed at 64/256 (0003) | replaced by a runtime multiplier (`0x12`) |
| Subtract the yaw rate from the tilt rates via tan(angle) (0005), freeze the accelerometer angles above 25 °/s (0006), unfiltered accel pitch as a switch (0017) | worked around the never calibrated pitch gyro (~2° pitch error); 0006 showed no measurable gain; all replaced by the vector fusion (04) |
| Yaw dead band 0.3 °/s (0009) | slow turns were lost; no dead band at all now |
| Turn hold switch, default off (0008/0010) | always on; switch had no user |
| Speed integral on the full error (first 0015) | wound up at every start, speed swung with ~5 s period; now only a floor of 30 % |
| Switches for drive integral (239/238), roll hold (236/235), turn hold (241/240) | nobody on the Pi used them |
| Register `0x0c` (wheel ticks + raw angle) | driver reads `0x0d`/stream since 2026-11-17 |
| Register `0x0f` (yaw-bias diagnostics), `0x10` (fusion diagnostics, 0013) | no reader left after the fusion was done |
| Register `0x16` / `!balance` (balance offset at runtime) | the Pi never wrote it; constant 2.05° |
| Reading `0x17` (BLE connected) | unused; the tunnel frames report connect/disconnect |
| Stock speed limits `0xa0/0xa1` | fixed 210 mm/s / 300 °/s, real units come with the velocity frame |
| Velocity frame: feed-forward in firmware units | 3.9 × too small; now mm/s² via the unit from `0x1b` |
| Velocity frame: scaling the stock P path (factor 0.55, 0.42) | 0.42 was worse; the rider needs a speed-proportional pitch, now −kv·v_measured |
| Velocity frame: kv on the target speed | braked after the ramp while the actual speed was still low |
| Velocity frame: kp 0.11, ki 0.1 (integral from the ramp end) | forward slow pendulum ~4 s, 0.74–1.43 × (2026-11-25); ki 0.06 alone: 1.33–1.36 ×; kp 0.15 + ki 0.06 (RAM test): +46 forward 1.09 ×, no undershoot |
| Velocity frame: integral ki 0.05, ±20 units, only 0.6 s after the ramp | forward missed ~1.5° pitch: 0.62–0.78 × target, 90 % after 3.1–3.6 s, then 1.4–1.5 × (2026-11-25, backward fine) |
| Velocity frame: kp 0.3 within ±20 units | limit cycle 2.5 Hz while driving (target −35: measured −84…+2, pitch target ±10°; kv build: ±3, ±1°) (2026-11-25) |
| Velocity frame: kp 0.3 on the full speed error | standing-up swing (wheels ±190 units) gave 57° pitch target, fell over (2026-11-25); now extra gain only within ±20 units |
| Velocity frame: −kv·v_measured (fixed pitch need per speed) | a model: offset/slope not in it, reverse 1.3–1.4 × target, forward 1.03–1.07 × (2026-11-25); now only target − measured |
| Distance stop: lag as ∫(v_target − v_measured) since start | summed the start-up shortfall and the steady speed offset, which the rider never makes up: stopped 5–6 cm short forward, 9 cm long reverse (runde9, ede94cd8); now lag = 0.25 s × filtered v_measured |
| Velocity frame: integral during the ramp | wound up during acceleration; now only 0.6 s after the ramp |
| Stock `automatic_lean` | never set by any register or console |
| Vendor BLE (fff0/fff1/fff2, name sector, trace stream `0xb3`, `!name`) | own service `RiderKerk` |
