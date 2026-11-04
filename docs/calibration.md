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
| Default | 88.5° (RiderPi); RPi camera v1 would be 62.2° |
| RiderPi measured | 88.5° (f = 328.3 px at 640 px) from folding-rule measurement (section 2a); ±1 mm camera height ≈ ±0.55°. Earlier checkerboard on a display (297–316 px) was disturbed by reflections |
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

## 2a. Folding-rule method (2026-11-04, no wall, no display)

| Item | Value |
|---|---|
| Setup | rule A along the camera axis, rule B across (2 m, centred) at 0.5 / 1.0 / 1.5 / 2.0 m; distance wheel contact point → front edge of B |
| Per position | 40 raw frames (`/frame?raw=1`) averaged + IMU pitch; Rider resting still (disarmed, leaning back) is ideal — no sway |
| Model | row(D) = cy + f·tan(atan(h / (D + 8 mm + e)) − τ), cy = 240; fit e, τ and f (with measured h) |
| Degeneracy | rows alone only fix f·h → measure camera height h directly |
| RiderPi | h = 100 mm (measured), rows 338.2 / 303.3 / 292.3 / 286.3 → f = 328.3 px (hfov 88.5°), horizon row 269.8 = 5.18° up at IMU pitch −4.43°, residuals ≤ 0.24 px |
| Result | `hfov_deg = 88.5`, `tilt_deg = 5.18`, `pitch_ref_deg = -4.43`; `mount_offset_mm` needs camera height while balancing at known body height (still 43) |

## 3. Lens shading — `[freespace] shading_file`

```
python tools/calibrate.py shading [shading.json]
```
| Item | Value |
|---|---|
| Target | uniformly white self-luminous surface filling the frame (laptop, white fullscreen, max brightness; focus irrelevant) |
| Browser page | `data:text/html,<body style="background:#fff;cursor:none">` |
| Rejected if | coverage < 75 % of frame, or overexposed |
| Camera setup | same as xgonav from `config.toml`: `sensor_mode`, `scaler_crop`, `fps` (auto exposure). Before 2026-11-04 the tool used the 640x480 sensor mode without crop → mismatched file |
| Config | `shading_file = "shading.json"`, `shading_max_gain` (RiderPi 5.0) |
| Missing file | runs uncorrected (log warning); with `method = "farbe"` the RiderPi then sees dark lower corners as obstacles |
| `method = "kanten"` | boundary from the uncorrected image; shading only affects the color model (bottom strip, `beide`) |
| RiderPi status | ISP (libcamera ALSC) already corrects most shading. 2026-11-04 recalibrations (`shading-neu*.json`, last one centered) helped the color model less than the old file in dark scenes → old `shading.json` kept |

## 4. Wheel odometry — `[odom] scale_left`, `scale_right`, `track_m`

```
python3 tools/calibrate_odom.py messen [--dreh-grad 360] [--strecke-cm 30] [--hin-her 2] [--nur-vor]
python3 tools/calibrate_odom.py rechne [--ref gerade|kamera|imu|mittel] [--dist-cm <tape>] [--spur-fest]
```
| Item | Value |
|---|---|
| Precondition | xgonav running, pipeline on, mode idle, Rider balancing |
| Start | tool waits; align Rider, then press „Hand“ in the web UI (no drive key). Heading is held by IMU from then on |
| Sequence | turn +360° / −360° in place → back to start heading → 2× 30 cm forward and back (distance by wheel odometry) |
| Space | strecke + ~15 cm ahead, turning room only |
| Abort | web „Idle“/stop (tool watches mode) or Ctrl+C; always writes standstill and mode idle |
| Model | per segment `kr·sr − kl·sl = track·θ_ref` (path need not be straight); turns fix `track_m`, drives fix kl/kr difference |
| Scale | mean of current kl/kr from config, or `--dist-cm` for the first forward segment (straight drive only, `--nur-vor`) |
| Heading ref | drives: `gerade` (θ = 0, confirm straight by eye; default), or camera/IMU; turns: IMU |
| RiderPi (2026-11-04) | 212.5 cm tape, straight by eye (θ = 0), raw 210.95/210.68 cm → `scale_left = 1.0073`, `scale_right = 1.0087` |
| RiderPi (2026-11-03) | 0.9905/1.0068 — tire was seated wrong, superseded |
| Pitfalls | camera heading biased during forward drive (−8.9° on 2 m straight); firmware yaw has 2°/s dead zone; ESP gyro bias measured only at boot — restart ESP lying still if Rider rotates while standing |
| Base values | 60 mm wheel, 1024 ticks/rev (hand-measured 1029/1043), confirmed 29.5 cm on tape |

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
