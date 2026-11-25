# Kernel driver `xgo-drv`

| Item | Value |
|---|---|
| Source | `kernel_module/`, `constants.h` |
| Base | [aeinstein/xgo_motorcortex](https://github.com/aeinstein/xgo_motorcortex) `c3c888c` + patches 0001, 0002, 0004 + wheel/raw_reg extensions + stream |
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
| 0003 | `settings/force_yaw` (pulsed heading hold): **removed 2026-11-17**, was never tested on the Rider. xgonav writes `force_yaw = 0` only if an old driver still has the entry |
| 0004 | `roll` = roll angle in degrees (reg 0x36); old function (roll balance, 0x61) → `roll_balance` |
| later | wheel encoders (combined register 0x0d) → `wheels`, `wheels_log`; `raw_reg` diagnostics; `calibration` sends bytes 0/1 (was ASCII, inverted); `pitch_calibration` |
| stream | IMU/wheel stream of firmware patch 0014 (2026-11-08): `settings/imu_stream_hz`, frames type 0x14 → `imu_log`/`wheels_log` with the firmware's sample time; polling only as fallback |
| 2026-11-17 | read paths for firmware without firmware patch 0001 removed (IMU modes 0/1/2, wheel register 0x0c, `settings/wheel_hz`); `yaw` from the stream; `force_yaw` removed; dead code removed (`xgo-lcd.c`, `xgo-lcd.h`). **Not installed on the Rider** |
| 2026-11-25 | drive frame of firmware patches 0029–0032: `vel`, `vel_limits` (text ↔ bytes in `xgo-vel.h`, shared with the driver test); installed on the Rider 2026-11-25, `vel_limits` with 7 fields (14 bytes) since firmware 0035 |

## IMU / wheel read path (auto-detected at load)

| Path | Register | Content | Firmware |
|---|---|---|---|
| Stream | type 0x14 (see below) | roll, pitch, yaw + wheel ticks with the firmware's sample time | R-1.1.6 + firmware patch 0014 |
| Polling (only without stream) | 0x0d | roll, pitch, yaw (float) + left/right ticks (int32) + t_ms (uint32), 24 B, at `imu_hz` | R-1.1.6 + firmware patch 0001 |

- Detection of 0x0d at load compares against fresh single-register reads (roll/pitch within 2°, yaw within 10°).
- Firmware without 0x0d: IMU and wheels only via the stream, warning in the kernel log. The older read paths (single registers 0x62–0x64, 12-byte block from 0x62, 0x65, wheels via 0x0c) were removed 2026-11-17.
- `serial_stats`: `imu_block` = 3 (0x0d can be polled) or 0; `wheels` = wheel ticks available (0x0d or stream seen).
- Timestamp: CLOCK_MONOTONIC, end of reply minus transfer time ((n + 8) × 10 bits at 115200). Firmware 0014: a 0x0d reply comes from a snapshot renewed every 10 ms, so the value can be up to 10 ms older than this.

## IMU/wheel stream (firmware patch 0014)

| Item | Value |
|---|---|
| Enable | `settings/imu_stream_hz` > 0: the IMU thread writes register 0x11 = rate every second (firmware stops after 5 s without a byte from the Pi and after an ESP restart); 0 writes 0 once |
| Frame | `55 00 22 14 11` + 26 B + checksum `00 AA`: seq u16, t_us u32 (`micros()` right after the IMU read), roll/pitch/yaw float, ticks wheel 11/21 s32 (little endian) |
| Rates | firmware: 0..250 Hz, divider of its IMU read (~333 Hz on the RiderPi), intervals at 100 Hz 9–12 ms |
| Time | offset = receive time − 34 B transfer (2.95 ms) − t_us; minimum taken immediately when lower, window minimum every ~2 s (follows ESP clock drift); t_us unwrapped (71 min overflow), ESP restart resets the offset |
| Logs | each frame → one `imu_log` and one `wheels_log` sample (`fw_ms` = t_us / 1000) |
| Polling | skipped while the stream is active: last frame younger than max(200 ms, 2.5 send intervals) (fixed 200 ms until 2026-11-17); otherwise 0x0d (firmware without stream, stream lost). Emulator, `--wheels --combo --stream=5`, 20 s: 11 polled samples between 100 stream frames (start-up before the first frame only), 17–18 before |
| Heading | `/proc/XGORider/yaw` is set from every frame (whole degrees, truncated like register 0x68, only while balancing); register 0x68 is polled only without stream (saves ~5 requests/s) |
| Stats | `serial_stats`: `stream`, `stream_gaps` (seq gaps = frames dropped by the firmware), `stream_hz`, `stream_active`, `stream_offset_us` |
| Userspace | xgonav `[imu] stream_hz` writes the setting at start; `imu_log` format unchanged |

## `/proc/XGORider`

| File | Mode | Format / range | Register |
|---|---|---|---|
| `speed_x` | w | u8 0..255, 128 = stop; arms motion watchdog | 0x30 |
| `speed_z` | w | u8 0..255, 128 = stop; arms watchdog | 0x32 |
| `vel` | rw | write one drive frame (firmware 0029–0032): `V A J W ALPHA JW` = mm/s, mm/s² (≥ 0), mm/s³, °/s (one decimal), °/s², °/s³ (0 = no limit), or `acc A J W ALPHA JW` = acceleration mode (A signed, up to ±v_max); arms the motion watchdog (it writes 128 to 0x30/0x32, which also ends the frame path). Read = firmware state, fresh from the ESP32: `active accel_mode watchdog_fired limits_set v w v_max w_max timeout_ms` (e.g. `1 0 0 1 250 -12.3 500 90.0 300`); old firmware: `Fehler <n>`. Without `vel_limits` the firmware drops every frame | 0x1a |
| `vel_limits` | rw | write `V_MAX W_MAX TIMEOUT_MS UNIT_UM KI_E4 IMAX_CENTI KV_E4` → 14 bytes (firmware 0035): v_max in firmware speed units, °/s with one decimal, firmware watchdog ms (0 = off), µm/s per firmware unit, integral gain × 10000, its limit in 0.01° (both 0 = console value), pitch feed-forward per unit of measured speed × 10000 (0 = off); xgonav writes it at start and every 2 s. Read = last written text | 0x1b |
| `height` | rw | u8 0..255 (web UI min 43); read: last written value, empty until first write; not reset when the firmware stands up at 128 | 0x35 |
| `roll` | rw | int −20..20 °, mapped linearly to 0..255; read: last written value, empty until first write | 0x36 |
| `roll_balance` | w | 0/1 | 0x61 |
| `action` | w | u8; R-1.1.6+patch: 1..6 actions, 237 restart the ESP32 (firmware patch 0018, refused while balancing), 253 stand up from lying, 254 disarm/emergency stop, 255 arm, 0 = no action | 0x3e |
| `leds/0`..`leds/3` | w | `#RRGGBB` | 0x69..0x6c |
| `battery` | r | % | 0x01 |
| `state` | r | firmware state: 1 = balancing, 254 = lying (set down by action 254, LEDs orange), 0 = fallen (tilt, LEDs red; since firmware 0027 stays until a command, also upright), 253 = ready (disarmed, starts when upright), 250 = low battery, 251 = actuator fault, 252 = other fault; names in `robot.FW_ZUSTAND` | |
| `yaw` | r | integer degrees, only updated while balancing; from the stream, without stream from register 0x68 every `sleep_ms_on_loop` | |
| `buttons` | r | bit mask, 1 = pressed: A = 1 (GPIO 17), B = 2 (22), C = 4 (23), D = 8 (24); polled every `sleep_ms_on_loop`. **A+B together = `orderly_poweroff`** (driver). C+D held 3 s = Wi-Fi setup ([wifi-setup.md](wifi-setup.md)) | |
| `imu` | r | `seq t_ns roll pitch yaw` (millidegrees), newest sample | |
| `imu_log` | r | last 128 samples, oldest first | |
| `wheels` | r | `seq t_ns ticks_left ticks_right fw_ms` (fw_ms only with 0x0d) | |
| `wheels_log` | r | last 64 samples | |
| `serial_stats` | r | `frames bad timeouts unsolicited imu_seq imu_hz imu_block wheels wheel_seq stream stream_gaps stream_hz stream_active stream_offset_us tunnel_rx tunnel_drop tunnel_tx pi_ip` (one per line) | |
| `raw_reg` | rw, root | write `"0x0b 16"` (addr, len), read `"0x0b 16: 00 11 .."`; read-only diagnostics | any |
| `lift` | r | lift-guard log of firmware patch 0027, read from the ESP32 on every open, format as `raw_reg` (`"0x19 32: .."`, layout `firmware/README.md`); old firmware: `Fehler <n>`. xgonav reads it once after a fall (`robot.lift_text`) | 0x19 |
| `ble` | rw, root | BLE tunnel of firmware patch 0025 ([ble.md](ble.md)). Blocking read (or `poll`) = one record: byte 0 kind (0 data, 1 BLE connected, 2 disconnected) + ≤ 23 bytes, buffer ≥ 24 bytes; returns 0 when the module unloads. Write = bytes to the BLE client, sent in 23-byte chunks every 4 ms. Reader: `xgo-ble` | 0x17, frames type 0x15 |

### `/proc/XGORider/settings`

| File | Default | Range / effect |
|---|---|---|
| `motion_timeout_ms` | 0 (off) | 0..60000; no `speed_x`/`speed_z` write for this long → stop |
| `imu_hz` | 50 | 0..200, 0 = off; polling rate of 0x0d, used only without stream |
| `imu_stream_hz` | 0 | 0..250; > 0 = firmware stream (0014), polling only as fallback |
| `led_brightness` | −1 (never written) | 0..255: global LED multiplier n/256 of firmware patch 0016 (reg 0x12; firmware default 32, former fixed value 64). Sent once on write; the firmware forgets it on an ESP restart, the file then still shows the last written value. Older firmware ignores it |
| `ip_dev` | `wlan0` | network interface (name ≤ 8 characters) whose first IPv4 the driver reports to the ESP32 for BLE (firmware patch 0025, reg 0x18): on change and every 3 s from the main loop; `0.0.0.0` without address. Older firmware ignores it |
| `low_batt_level` | 8 | 0..100 % |
| `shutdown_on_low_batt` | 1 | battery < level → `orderly_poweroff` |
| `sleep_ms_on_loop` | 200 | main loop sleep (battery, state; yaw polling only without stream) |
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
driver-test/run.sh kernel_module [seconds=6] [imu_hz=50] [--wheels --combo] [--stream[=HZ] | --stream-off] [--tunnel] [--faults]
```
`--tunnel` (firmware 0025): the emulator sends data through the BLE tunnel (frames type 0x15), the harness echoes it via register 0x17; checked complete and in order next to polling and stream (2026-11-20 with `--stream --faults`: 892 bytes in 45 records, none dropped).
Compiles the real `xgo-serial.c` and IMU code against `kshim.h`, runs it against `esp32_emu.py`. Without `--wheels --combo` the emulator has no 0x0d and the driver does not poll IMU/wheels (stream only).
Result (polling 0x0d): 49 Hz at target 50, pitch error 0.06° median, parser recovers from drops, noise bytes, bad checksums.
Stream (emulator with its own ESP clock, 40 ppm, 0–8 ms send delay): 100 Hz without gaps, timestamp error −0.25 ms median (polling 0x0d: −1.3 ms), 0 unsolicited; with `--faults` gaps counted, polling only for the other registers; `--stream-off` falls back to polling.
