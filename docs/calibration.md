# Calibration

Run on the Rider in `/opt/xgonav` with `venv/bin/python`. Results go into `/opt/xgonav/config.toml`
by hand. Order matters: FOV first.

Positioning helpers (balancing Rider):
```
python tools/nudge.py turn <deg>     # IMU-controlled, + = left
python tools/nudge.py move <s>       # open loop, + = forward
```

## 0. Horizontal FOV — `[camera] hfov_deg`

| Item | Value |
|---|---|
| Default | 62.2° (RPi camera v1) — wrong for RiderPi |
| RiderPi measured | ~91° (f ≈ 314 px at 640 px; board ~300 px, 16 checkerboard images 297–316 px) |
| Method | board of known width at known distance, or checkerboard + `cv2.calibrateCamera` |
| Wrong FOV effect | `solve` returned 54 mm camera height instead of measured 128 mm |

## 1. Turn direction and scale — `[robot] vyaw_limit_dps`

```
python tools/calibrate.py turn       # Rider turns 3 s on the spot
```
Checks sign, prints `vyaw_limit_dps`. Measured via ego-motion → needs step 0.

## 2. Camera height and tilt — `[camera] mount_offset_mm`, `tilt_deg`, `[imu] pitch_ref_deg`

| Item | Value |
|---|---|
| Height, direct measurement | lens centre above floor; RiderPi: 128 mm at body height 85 mm → `mount_offset_mm = 43` |
| Tilt | from wall base line |
| RiderPi | `tilt_deg = 1.39` at raw IMU pitch -2.7° (R-1.1.3, board + 2 wall measurements, spread 0.25°) |
| R-1.1.6 | `pitch_ref_deg = -0.5`, set visually on the horizon line; wall measurement open |

```
python tools/calibrate.py wall --cols 0.45:0.55 --debug wand.jpg   # Rider facing a wall, ~1 m
python tools/calibrate.py wall ...                                 # second position, ~2 m
python tools/calibrate.py solve <row1>:<m1> <row2>:<m2>
```
- Distance = wheel contact point → wall base, measured **after** standing up (Rider rolls ~20 cm forward).
- Check the debug image: shadow at the wall base → floor model measures the shadow edge.
- `tilt_deg` and `pitch_ref_deg` are a pair; enter both.
- Long exposure allowed for this (`exposure_us = 33000`).

## 3. Lens shading — `[freespace] shading_file`

```
python tools/calibrate.py shading [shading.json]
```
| Item | Value |
|---|---|
| Target | uniformly white self-luminous surface filling the frame (laptop, white fullscreen, max brightness; focus irrelevant) |
| Browser page | `data:text/html,<body style="background:#fff;cursor:none">` |
| Rejected if | coverage < 75 % of frame, or overexposed |
| Config | `shading_file = "shading.json"`, `shading_max_gain` (RiderPi 5.0) |
| Missing file | runs uncorrected (log warning); RiderPi then sees edges as obstacles |

## 4. Wheel odometry — `[odom] scale_left`, `scale_right`

```
python3 tools/calibrate_odom.py fahrt [--v 0.12] [--secs 8] [--out /tmp/odomcal.json]
python3 tools/calibrate_odom.py rechne --dist-cm <tape> [--ref kamera|imu|mittel]
```
| Item | Value |
|---|---|
| Precondition | xgonav running, pipeline on, mode idle, Rider balancing, free space ahead (distance + 30 cm) |
| Drive | ~1 m at v = 0.12 via manual mode |
| Reference | heading from camera (ego-motion) and IMU; distance from tape measure |
| RiderPi (2026-11-03) | 215 cm tape, wheels 215.3 cm → `scale_left = 0.9905`, `scale_right = 1.0068` |
| Base values | 60 mm wheel, 1024 ticks/rev (hand-measured 1029/1043), confirmed 29.5 cm on tape |
| Ctrl+C | always writes standstill and mode idle |

## 5. Forward scale (command fallback)

Assumption: 1 firmware unit = 1 m/s (`vx_limit`). Used for the map only when wheel odometry is unavailable.
RiderPi at v = 0.08: ~12.5 cm/s (assumption: 12 cm/s).

## 6. Reverse gain — `[robot] reverse_gain`

| Command (2 s) | Distance |
|---|---|
| u8 147 (v = +0.15) | 86 cm forward |
| u8 109 (v = −0.15) | 4 cm backward |
| u8 90 (v = −0.15, `reverse_gain = 2.0`) | ≈ forward distance (by eye) |

Backward command: Rider leans 2–3°, overshoots forward when stopping.

## Locked: leg calibration

Web buttons for leg zero points (`settings/calibration`) and pitch trim (`settings/pitch_calibration`)
are removed since 2026-11-02: saving with slack legs overwrote the leg zero points in ESP32 flash.
