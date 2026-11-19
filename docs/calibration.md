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
| Default | 91.8° (RiderPi, f = 310 px at 640 px); RPi camera v1 would be 62.2° |
| RiderPi measured (2026-11-07) | turns on the spot against IMU heading (IMU = 99.6–99.8 % of true angle over full turns): rotation fit with fixed IMU yaw 304–310 px, ORB pairs 25–45° apart 310–313 px in three recordings → 91.8°. Lens distortion negligible (k1 −0.010…+0.003, ≤ 2 px at the edge, ~4 px in the corner) |
| Earlier values | 88.5° (f 328.3 px) from the folding rule (section 2a) with measured h = 100 mm: that method only fixes f·h, the 5.6 % gap means h was ~106 mm. Checkerboard on a display gave 297–316 px |
| Driving direction (2026-11-08) | focus of expansion while driving straight: 16 segments ≥ 1.5 m from 8 drives (45 m), rotation per frame pair estimated, FOE shared per segment → column 333.8 (MAD 16, SE ~4; ≥ 2.5 m: 332.2). Camera looks ~2.4° left of the driving direction → `[camera] lens_yaw_deg = 2.40` (formerly `cx_offset_px = 13`), rotated out in the undistortion table. Mounting roll 1.25° → `lens_roll_deg` (formerly `[imu] roll_ref_deg`). A second Rider only needs its own lens block (distortion, roll, yaw); pitch stays `tilt_deg`. Simulator check ±3 px. Roll not measured (stripe board: ~1.5°) |
| Check yaw/roll (2026-11-18) | folding rule on the floor under the camera along the driving direction must lie on the web overlay "Image centre" (column W/2); a floor joint at right angles to it must be level. Confirmed with `lens_yaw_deg = 2.40`, `lens_roll_deg = 1.25` (Rider lying). Use one rule segment only (the hinges kink ~1°); re-align after the Rider stood up |
| Wrong FOV effect | all angles from the image (ego yaw/pitch, tracker shift, object bearing) scale with f: with 88.5° they were ~5 % too small |
| Floor distances | depend on f·h: after changing f, re-measure camera height (`mount_offset_mm`), else distances shrink ~6 % |
| Method | turn ≥ 360° on the spot and compare image rotation with IMU heading, or checkerboard + `cv2.calibrateCamera` |

## 1. Turn direction and scale — `[robot] vyaw_limit_dps`

```
python tools/calibrate.py turn       # Rider turns 3 s on the spot
```
Checks sign, prints `vyaw_limit_dps`. Measured via ego-motion → needs step 0.

## 2. Camera height and tilt — `[camera] mount_offset_mm`, `tilt_deg`, `[imu] pitch_ref_deg`

| Item | Value |
|---|---|
| Height, direct measurement | lens centre above floor while balancing, at two or more height set-points → `cam_h_u8 = [a, b]` (mm = a + b·u8) |
| RiderPi (2026-11-07) | u8 106 / 150 / 200: wheel axle 31 mm (2026-11-08 exact: wheel diameter 60 mm, hub 30 mm above the floor), leg pivot 87 / 91 / 98 mm, lens centre 122 / 125 / 132–133 mm → `cam_h_u8 = [109.4, 0.1126]` (residuals ≤ 1.3 mm). Camera sits rigidly 35 mm above the leg pivot |
| Why not body height + offset | the body height in mm (60–120 over u8 0–255) rises 22 mm from u8 106 to 200, the real body only 11 mm; the old `mount_offset_mm = 43` (128 mm at u8 106) put the camera 18 mm too high at u8 200 |
| Floor check (2026-11-07) | tape front edges 0.48 / 0.98 m from wheel contact, raw frames at u8 106 and 200, camera 36 mm ahead of contact (CAD): horizon off −0.05° / +0.08°, f·h too large by factor 1/0.926 / 1/0.921 (distances ~7.5 % too far) |
| Grid check (2026-11-10) | printed grid (`tools/pruefmuster.py`, 50 mm, 950 mm across the sheet joints) flat on the floor, balancing, 39 raw frames each: lens height = distance of a floor point from the grid's vanishing line ÷ lateral px/mm at that point (no camera model, no distance needed) → 124–126 mm at u8 128, 131–133 mm at u8 205 (spread < 1 mm); horizon row from the grid 254 / 251 vs. 255.5 / 248.5 in xgonav. Grid on the wall at 0.40 / 0.75 m: f 307–313 px from the 350 mm shift, pixels square within 0.5 %, distortion −3.6 px at r 220 px, about −10 px in the corners (not a single k1). Row check with the whole grid in view (u8 205, wheel contact 310 mm before the first line, 9 cross lines 0.31–0.71 m, pitch free per frame): `ground_dist` with f 310 and h 132.5 mm fits the centre column within 0.7 px and puts the horizon at row 247.1 (xgonav at that moment 247.0–247.8); with h × 0.921 the residuals are −3.7…+2.2 px and the horizon lands at 253.9. Towards the right image edge the fitted f·h rises to +4 % (Rider not square to the grid or distortion, not separated). Sets where the wheel position was only estimated to ±2 mm near the first line gave ±4–8 % and are not used |
| RiderPi value | `cam_h_u8 = [109.4, 0.1126]` = the direct measurement, confirmed by the grid check. The factor 0.921 from the tape check (used 2026-11-07 to 2026-11-10) is dropped; why the tape check differed is open |
| CAD check | `docs/Rider-Pi.STEP` with the leg linkage of `tools/step_to_model.py`: lens 122.6 mm at u8 106 when the floor is taken at the tyre (radius 31.5 mm), rise 12.4 mm to u8 200 |
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
| Result | `hfov_deg = 88.5`, `tilt_deg = 5.18`, `pitch_ref_deg = -4.43`; `mount_offset_mm` needs camera height while balancing at known body height (still 43). Superseded for f by section 0 (91.8°); f·h from this fit stays valid |

## 2b. Standing horizon per body height — removed 2026-11-17

`[horizon]` (stand reference per stand-up, horizon row per body height; `xgonav/horizon.py`, `tools/horizon_replay.py`, web card *Horizont*) is removed; it was off on the RiderPi since 2026-11-12. The pitch is the pure IMU pitch of the frame (2c) with `pitch_ref_deg` + `tilt_deg` (section 2).

## 2c. Time offset frame → IMU — `[imu] time_offset_ms`, `pitch_sign`

Both are fixed in the defaults since 2026-11-17 (-1, 10.5); the online estimate (`ImuSync`) runs only with 0 / None (simulator).

Every consumer (geometry, floor model roll, map heading, tilt, overlay, web) uses one IMU value per frame: `imu.imu_frame` at frame time (`SensorTimestamp`) + `time_offset_ms`, linearly interpolated in the `imu_log` history. Measure the offset on the PC from a raw recording (web button *Aufzeichnen*, needs pitch motion such as height changes):

```bash
python tools/imu_versatz.py ~/fahrt/<recording> [--fenster idx,idx,... --halb 48]
```

| Date | Result |
|---|---|
| 2026-11-17 | recording 20261117-161237 (height changes), 8 sections of 4 s: +9.9..+12.0 ms, all together **+10.5 ms**, sign −1, correlation 0.99; slope 1.13 (image pitch 13 % larger than IMU pitch: focal length or IMU scale, open) |

Web: the line under the camera image shows the IMU value of each frame (pitch, roll, heading, sample age, pitch rate image vs. IMU) and counts frames without a valid IMU value (red).
Drift check: *Bild am IMU-Nick festhalten* (camera bar) shifts each frame in the browser by `row_stand − row_imu` (normal pose against the pure IMU pitch; black border, no load on the pipeline); the cyan line is then fixed. Scene moving against it = IMU or time-sync drift.

## 3. Lens shading — removed 2026-11-18

Only the classic colour floor model used `shading.json`. FloorNet works on the uncorrected (lens-undistorted) frame; `tools/calibrate.py shading` and `[freespace] shading_file` are gone.

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
| RiderPi (2026-11-19) | both 1.0080 (mean kept): dead-reckoning suggestion over 66 raw runs, wheels against IMU heading, no left/right difference, track 0.0903 |
| RiderPi (2026-11-03) | 0.9905/1.0068 — tire was seated wrong, superseded |
| Pitfalls | camera heading biased during forward drive (−8.9° on 2 m straight); firmware yaw has 2°/s dead zone; ESP gyro bias measured only at boot — restart ESP lying still if Rider rotates while standing |
| Base values | 60 mm wheel, 1024 ticks/rev (hand-measured 1029/1043), confirmed 29.5 cm on tape |

## 5. Forward scale (command fallback)

`robot.vx_limit`: 1.8 m/s per firmware unit on the RiderPi (2026-11-05). Used by the path check and follow prediction, and for the map only when wheel odometry is unavailable.
RiderPi at v = 0.08: ~12.5 cm/s (assumption: 12 cm/s).

## 6. Drive gains — `[robot] reverse_gain`, `manual_forward_gain`

With firmware patches up to 0010 both directions respond alike (register 0x0e, 2026-11-05: wheel speed per target 2.67 forward, 2.65 reverse). Sent 0.12: 26 cm/s, sent 0.24: 50 cm/s. RiderPi uses 2.0 for both; forward only in manual mode. Table below: 2026-11-02.

| Command (2 s) | Distance |
|---|---|
| u8 147 (v = +0.15) | 86 cm forward |
| u8 109 (v = −0.15) | 4 cm backward |
| u8 90 (v = −0.15, `reverse_gain = 2.0`) | ≈ forward distance (by eye) |

Backward command: Rider leans 2–3°, overshoots forward when stopping.

## Locked: leg calibration

Web buttons for leg zero points (`settings/calibration`) and pitch trim (`settings/pitch_calibration`)
are removed since 2026-11-02: saving with slack legs overwrote the leg zero points in ESP32 flash.
