# Kernel driver `xgo-drv`

| Item | Value |
|---|---|
| Source | `kernel_module/`, `constants.h` |
| Base | [aeinstein/xgo_motorcortex](https://github.com/aeinstein/xgo_motorcortex) `c3c888c` + patches 0001–0004 + wheel/raw_reg extensions |
| UART | `/dev/ttyAMA0`, 115200 8N1, to ESP32 |
| Tested on | RiderPi CM4, kernel 6.6.74+rpt-rpi-v8, firmware R-1.1.3 and R-1.1.6 |

```
make -C kernel_module && sudo make -C kernel_module install
sudo rmmod xgo-drv; sudo modprobe xgo-drv
cat /proc/XGORider/serial_stats; cat /proc/XGORider/imu
```

## Patches

| Patch | Change |
|---|---|
| 0001 | memory bugs in proc handlers fixed; `settings/motion_timeout_ms` (kernel watchdog, default 0 = off) |
| 0002 | synchronous reads (`completion` instead of `msleep(200)`); IMU thread → `imu`, `imu_log`, `serial_stats`; UART raw mode in `rc.local` |
| 0003 | `settings/force_yaw`: target = heading at stand-up/enable, pulsed correction, pauses during `speed_z` turns. Built, not tested on Rider; xgonav disables it |
| 0004 | `roll` = roll angle in degrees (reg 0x36); old function (roll balance, 0x61) → `roll_balance` |
| later | wheel encoders (0x0c / combined 0x0d) → `wheels`, `wheels_log`, `settings/wheel_hz`; `raw_reg` diagnostics; `calibration` sends bytes 0/1 (was ASCII, inverted); `pitch_calibration` |
| stream | IMU/wheel stream of firmware patch 0014 (2026-11-08): `settings/imu_stream_hz`, frames type 0x14 → `imu_log`/`wheels_log` with the firmware's sample time; polling only as fallback |

## IMU / wheel read path (auto-detected at load)

| Mode | Register | Content | Firmware |
|---|---|---|---|
| 3 | 0x0d | roll, pitch, yaw (float) + left/right ticks (int32) + t_ms (uint32), 24 B | R-1.1.6 + patch |
| 2 | 0x65 | accel[3] + roll/pitch/yaw, 6 floats, 24 B | R-1.1.6 |
| 1 | 0x62.. | 12-byte block roll/pitch/yaw | if supported |
| 0 | 0x62, 0x63, 0x64 | single registers | all |

- Detection compares against fresh single-register reads (roll/pitch within 2°, yaw within 10°).
- Wheels via 0x0c (16 B: ticks L/R, pos L/R) every k-th IMU cycle, k = `imu_hz / wheel_hz`, if 0x0c answers.
- Timestamp: CLOCK_MONOTONIC, end of reply minus transfer time ((n + 8) × 10 bits at 115200). Firmware 0014: a 0x0d reply comes from a snapshot renewed every 10 ms, so the value can be up to 10 ms older than this.

## IMU/wheel stream (firmware patch 0014)

| Item | Value |
|---|---|
| Enable | `settings/imu_stream_hz` > 0: the IMU thread writes register 0x11 = rate every second (firmware stops after 5 s without a byte from the Pi and after an ESP restart); 0 writes 0 once |
| Frame | `55 00 22 14 11` + 26 B + checksum `00 AA`: seq u16, t_us u32 (`micros()` right after the IMU read), roll/pitch/yaw float, ticks wheel 11/21 s32 (little endian) |
| Rates | firmware: 0..250 Hz, divider of its IMU read (~333 Hz on the RiderPi), intervals at 100 Hz 9–12 ms |
| Time | offset = receive time − 34 B transfer (2.95 ms) − t_us; minimum taken immediately when lower, window minimum every ~2 s (follows ESP clock drift); t_us unwrapped (71 min overflow), ESP restart resets the offset |
| Logs | each frame → one `imu_log` and one `wheels_log` sample (`fw_ms` = t_us / 1000) |
| Polling | skipped while frames arrive (< 200 ms old), otherwise as before (firmware without stream, stream lost) |
| Stats | `serial_stats`: `stream`, `stream_gaps` (seq gaps = frames dropped by the firmware), `stream_hz`, `stream_active`, `stream_offset_us` |
| Userspace | xgonav `[imu] stream_hz` writes the setting at start; `imu_log` format unchanged |

## `/proc/XGORider`

| File | Mode | Format / range | Register |
|---|---|---|---|
| `speed_x` | w | u8 0..255, 128 = stop; arms motion watchdog | 0x30 |
| `speed_z` | w | u8 0..255, 128 = stop; arms watchdog; ≠128 pauses `force_yaw` | 0x32 |
| `height` | rw | u8 0..255 (web UI min 43); read: last written value, empty until first write; not reset when the firmware stands up at 128 | 0x35 |
| `roll` | rw | int −20..20 °, mapped linearly to 0..255; read: last written value, empty until first write | 0x36 |
| `roll_balance` | w | 0/1 | 0x61 |
| `action` | w | u8; R-1.1.6+patch: 1..6 actions, 254 disarm/emergency stop, 255 arm, 0 = no action | 0x3e |
| `leds/0`..`leds/3` | w | `#RRGGBB` | 0x69..0x6c |
| `battery` | r | % | 0x01 |
| `state` | r | firmware state, 0 = fallen, 1 = balancing | |
| `yaw` | r | integer degrees | |
| `buttons` | r | integer | |
| `imu` | r | `seq t_ns roll pitch yaw` (millidegrees), newest sample | |
| `imu_log` | r | last 128 samples, oldest first | |
| `wheels` | r | `seq t_ns ticks_left ticks_right fw_ms` (fw_ms only with 0x0d) | |
| `wheels_log` | r | last 64 samples | |
| `serial_stats` | r | `frames bad timeouts unsolicited imu_seq imu_hz imu_block wheels wheel_seq wheel_hz stream stream_gaps stream_hz stream_active stream_offset_us` (one per line) | |
| `raw_reg` | rw, root | write `"0x0b 16"` (addr, len), read `"0x0b 16: 00 11 .."`; read-only diagnostics | any |

### `/proc/XGORider/settings`

| File | Default | Range / effect |
|---|---|---|
| `motion_timeout_ms` | 0 (off) | 0..60000; no `speed_x`/`speed_z` write for this long → stop |
| `imu_hz` | 50 | 0..200, 0 = off |
| `imu_stream_hz` | 0 | 0..250; > 0 = firmware stream (0014), polling only as fallback |
| `wheel_hz` | 25 | 0..200, 0 = off; effective ≤ `imu_hz`, as integer divider |
| `force_yaw` | 0 | 0/1 |
| `low_batt_level` | 8 | 0..100 % |
| `shutdown_on_low_batt` | 1 | battery < level → `orderly_poweroff` |
| `sleep_ms_on_loop` | 200 | main loop sleep (battery, state, yaw polling) |
| `verbose` | 0 | bitmask: 1 serial, 2 gpio, 4 proc, 8 main |
| `calibration` | — | write 1 = start leg calibration (motors limp), 0 = **save** zero points to flash |
| `pitch_calibration` | — | write 1: firmware balances in place, stores pitch trim (reg 0x09); Rider must balance |

## UART raw mode (required)

`deploy/rc.local`:
```
stty -F /dev/ttyAMA0 115200 cs8 -cstopb -parenb -brkint -imaxbel -opost -onlcr -isig -icanon -iexten \
  -echo -echoe -echok -echoctl -echoke -ixon -ixoff -icrnl -inlcr -igncr -istrip min 0 time 1
```
- Without `-ixon -icrnl`: a random 0x13 (XOFF) byte in an ESP32 reply stops the transmitter; all serial traffic hangs (fast IMU polling: after ~1 s).
- Live fix without reload: `termios.tcflow(fd, TCOON)` + `tcsetattr(..., TCSANOW, ...)` from Python; `stty` itself hangs (waits for output drain).

## ESP32 protocol assumptions

- Replies carry the requested address.
- Writes are not acknowledged.
- Internal update rate of firmware attitude values unknown.
- `serial_stats` counts timeouts and unsolicited frames.

## Userspace test

```
driver-test/run.sh kernel_module [seconds=6] [imu_hz=50] [--block] [--faults] [--wheels --combo] [--stream[=HZ] | --stream-off]
```
Compiles the real `xgo-serial.c` and IMU code against `kshim.h`, runs it against `esp32_emu.py`.
Result: 49 Hz at target 50 (block mode: 97 Hz at 100), pitch error 0.06° median, parser recovers from drops, noise bytes, bad checksums.
Stream (emulator with its own ESP clock, 40 ppm, 0–8 ms send delay): 100 Hz without gaps, timestamp error −0.25 ms median (polling 0x0d: −1.3 ms), 0 unsolicited; with `--faults` gaps counted, polling only for the other registers; `--stream-off` falls back to polling.
