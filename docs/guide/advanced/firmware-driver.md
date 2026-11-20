# Firmware and driver

## Kernel driver `xgo-drv`

| Item | Value |
|---|---|
| Source | `kernel_module/` (upstream [aeinstein/xgo_motorcortex](https://github.com/aeinstein/xgo_motorcortex) + patches) |
| Build | `make -C kernel_module` on the Rider (kernel headers) |
| Interface | `/proc/XGORider/*`: drive, height, action, IMU, `imu_log`, wheels, LEDs, settings |
| Test without hardware | `driver-test/run.sh kernel_module 6 50 --wheels --combo --faults` (driver code in userspace against an ESP32 emulator) |
| Reload | `sudo rmmod xgo-drv; sudo modprobe xgo-drv` |

Update order: xgonav first, then the driver. Reference: [Driver](../../driver.md).

## ESP32 firmware

| Item | Value |
|---|---|
| Base | R-1.1.6 from `LuwuDynamics/xgorider2_rl` |
| Patches | `firmware/patches/` (wheel registers, turn hold, gyro bias, IMU stream, nick fusion, …) |
| Build | `firmware/build.sh [rider2-firmware]` → `firmware/out/app0.bin` (PlatformIO, does **not** flash) |
| Host tests | `g++` against the patched sources, see [Testing](../../testing.md) |

!!! danger "Flashing"
    Only the app partition `app0` at `0x10000`. **Never** write `0x310000–0x32ffff` (leg zero points, BLE name, pitch trim) or spiffs.

LED colours and patterns: [LEDs](../../LED.md).
