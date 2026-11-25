# Configuration

## Precedence

1. Defaults: dataclasses in `xgonav/config.py`.
2. TOML file (`--config`), e.g. `/opt/xgonav/config.toml` from `deploy/config.example.toml`. Only list deviations.
3. CLI flags.

Unknown section or key in TOML → `KeyError` at load. New parameters go into the dataclass first.

## CLI (`python -m xgonav`)

| Flag | Effect |
|---|---|
| `--config FILE` | TOML overrides |
| `--mode idle\|follow\|explore\|approach\|home\|pet` | `behavior.mode` |
| `--target CLASS` | `behavior.target_class` (COCO, e.g. `person`, `chair`) |
| `--source picam\|<index>\|<file>` | `camera.source` |
| `--robot proc\|sim` | default: `proc` if source is `picam`, else `sim` |
| `--model FILE` | `detector.model` |
| `--no-detector` | `detector.enabled = false` |
| `--web-port N` | `output.web_port` (0 = off) |
| `--no-lcd` | `output.lcd_fifo = ""` |
| `--record FILE.mp4` | record overlay video |
| `--record-raw DIR` | raw recording switched on from the start, to `DIR` instead of `output.record_dir` (same as the web button, see `docs/web-interface.md`): JPEGs + `meta.jsonl` (for `tools/replay_raw.py`), only while balancing, each recording in a new subdirectory `DIR/YYYYMMDD-HHMMSS` |
| `--duration S` | exit after S seconds |
| `--fast` | video file as fast as possible |
| `-v` | debug log |

## Parameters

Default = `config.py` = the RiderPi values (`/opt/xgonav/config.toml` without `/opt` paths). (sim: …) = `Config.neutral()` (`NEUTRAL` in `config.py`), used by `tests/sim_run.py` and the tests; since 2026-11-23 it differs only where the simulator cannot model the hardware (camera, IMU auto sync, wheel scale, levelling, detector rate, FloorNet, display, CPU pinning) — all drive, turn, behaviour, map and timing values are the Rider's.

**Speeds (since 2026-11-23):** every drive and turn value of a mode (`home.v`, `explore.v`, `pet.drive_v`, `behavior.search_w`, …) and the hand control are **factors 0..1 of `robot.max_v` / `robot.max_w`**. Sent = factor × `tempo` × `max_v` (hand control: stick × `tempo` × `max_v`); capped at ±1 (firmware range). Values below are factors; absolute = factor × 0.5 on the RiderPi.

RiderPi = `deploy/config.example.toml` where it still differs (blank = default). [K] = device-specific, calibrate.

### top level
| Key | Default | RiderPi | Note |
|---|---|---|---|
| `control_hz` | 24.0 | | control steps per second, at the frame nearest to the beat (threshold `1/control_hz − 0.5/fps`), so at most the frame rate: 24 = every frame. Without image processing (`manual`) by the clock. Until 2026-11-17: 20, which gave only every 2nd frame (~12 Hz) on the Rider and ~20 Hz in the simulator (20 fps). 24 Hz **not installed, not driven** |

### `[robot]`
| Key | Default | RiderPi | Unit / note |
|---|---|---|---|
| `max_v` | 0.5 | | maximum drive value (share of the firmware range); all speeds are factors of it. RiderPi 0.15 → 0.5 on 2026-11-13 (user), **never driven above 0.24 absolute**. Sent 0.12 ≈ 26 cm/s, 0.24 ≈ 50 cm/s |
| `max_w` | 0.5 | | maximum turn value; 0.25 ≈ 90 °/s (firmware scale assumed); RiderPi 0.30 → 0.5 on 2026-11-13 (user) |
| `tempo` | 1.0 | | start value of the speed slider: every drive and turn command of every mode is multiplied by it after the `max_v`/`max_w` clamp (`Robot.drive`); at runtime `POST /tempo?f=0.05..1` (web slider), until restart. Firmware actions run at their own speed |
| `level_pulse_s` | 3.0 | | length of one leveling pulse (roll command 0, firmware `roll_balance` on, then off) |
| `level_after_standup` | true (sim: false) | | one pulse after each stand-up, 2.5 s after it; web button „Neu ausrichten“ triggers the same pulse |
| `level_while_driving` | true (sim: false) | | firmware leveling (0x61) on after `level_drive_delay_s` (1.0) of straight driving (\|w\| ≤ `level_drive_w_max` 0.06, roll command 0), off when turning, stopping or rolling; trim stays (patch 0012) |
| `vx_limit` | 1.8 | | m/s per firmware unit (measured 2026-11-05: command 0.15 → ~27 cm/s); path check, follow prediction, odometry fallback |
| `vyaw_limit_dps` | 360 | | °/s at u8 0/255 |
| `vel_frame` | true | | drive frame of firmware patch 08-velocity (`xgonav/fahrrahmen.py`): used when the driver has `/proc/XGORider/vel` and the firmware answers reg 0x1a, else bytes; if the firmware did not answer at start (flashing, ESP restart), xgonav asks again every 5 s and switches over. Sends target m/s (normalised × `vx_limit`) and °/s (× `vyaw_limit_dps`) with acceleration and jerk; the firmware ramps (S-curve) and controls speed and turn rate. (Pi ramp, `speed_ctrl` and `turn_min_w` removed 2026-11-25; the byte fallback sends targets unramped); `robot.command` is the firmware's setpoint, recomputed on the Pi. Simulator follows it with 0.15 s / 0.1 s lag (assumption until measured) |
| `vel_kappa` | true | | curvature mode (ARBEITSPLAN.md item 5, stage 1): path-follower commands carry `Command.kappa` and go out as `kap` frames (κ in 0.001 1/m); the firmware sets w = κ · v_target and limits v/a/jerk instead of w, so the arc holds while accelerating and braking. Only if the firmware reports capability bit0 (reg 0x1a byte 1); otherwise turn-rate frames and a log warning. A stop after an arc brakes on the same arc. Driven 2026-11-25 (κ 2 by hand), on since |
| `vel_kurs` | true | | course target (ARBEITSPLAN.md item 5, stage 2): turning on the spot to a heading (path follower turn/align, `POST /turn`) goes out as `kurs` frames (Δψ in 0.1°, toggle bit = new target); the firmware turns with braking look-ahead and holds the heading. While a target runs, the mode's turn limit is sent as w_max in `vel_limits`. A new target only when none runs or the running one has arrived and > 15° remain (never while the firmware turns: the heading lags > 20° at 90 °/s); `POST /turn` keeps sending the target for `behavior.turn_settle_s` (2 s) after arrival so the firmware holds it (81cb3335). Only if the firmware reports capability bit1 and the driver has 13-field `vel`; otherwise turn-rate frames. Driven 2026-11-25 (±90°/±180° within ±1.4°), on since |
| `vel_strecke` | true | | distance with stop (ARBEITSPLAN.md item 5, stage 3): commands with `Command.strecke` (remaining metres, recomputed by the caller from odometry each step, e.g. `/drive?strecke=M`) go out as 15-byte `dist`/`kapdist` frames (rest in mm, rounded to 5 mm); the firmware brakes so it stops there (look-ahead plus wheel lag). Only if the firmware reports capability bit2 and the driver has 14-field `vel`. The rest only shrinks within one run (a larger one would wake the firmware after its stop). Used by hand control and by `home` for the final straight to the start and the short correction (driven backwards when the start lies ≤ 0.3 m behind). Driven 2026-11-25 (within ±3.3 cm on 0.5–1 m) |
| `fw_speed_unit_mm` | 3.97 | | mm/s per firmware speed unit (measured 2026-11-25: 3.95 / 3.90 at 152 / 246 mm/s with the nominal 60 mm wheel; 3.97 since the distance runs: the firmware's wheel distance was 1.6–2.0 % short of the ticks with the tape-measured odometry scale 1.008); all drive values to the firmware (v, a, jerk, v_max) are divided by it. Measure: `tools/fw_einheit.py` ([calibration.md](calibration.md)) |
| `vel_accel_mps2` / `vel_decel_mps2` / `vel_jerk_mps3` | 0.5 / 1.0 / 2.5 | | faster in the same direction / slower or reversing; cautious start values, to be measured (0.5 m/s² ≈ 2.9° lean) |
| `vel_brake_mps2` / `vel_brake_jerk_mps3` | 1.5 / 10 | | `drive(brake=True)` |
| `vel_alpha_dps2` / `vel_jerk_w_dps3` | 360 / 1800 | | turning |
| `vel_timeout_ms` / `vel_limits_s` | 500 / 2.0 | | firmware watchdog (no frame → ramp to 0; above `keepalive_s`); `vel_limits` (v_max = `max_v` × `vx_limit`, w_max = `max_w` × `vyaw_limit_dps`) rewritten this often (an ESP restart loses it) |
| `vel_ki` / `vel_i_max_deg` / `vel_kv` | 0.03 / 6.0 / 0.025 | | **no effect** since 2026-11-25 (firmware 08-velocity, e93e525): the firmware controls speed on target − measured only, with fixed kp 0.3, ki 0.15, i_max 8°, and ignores these fields. Still sent in `vel_limits` (part of the 0x1b format) |
| `keepalive_s` | 0.3 | | s |
| `stall_timeout_s` | 0.5 | | s, userspace watchdog |
| `kernel_watchdog_ms` | 1000 | 1000 | `settings/motion_timeout_ms` |
| `height_min_mm` / `height_max_mm` | 60 / 120 | | scale mm → u8 (0..255) |
| `height_limit_u8` | 205 | | highest setpoint (≈ 108 mm; firmware clamps at 200 ≈ 107 mm); see `limitations.md` |
| `led_file` | `/var/lib/xgonav/led_brightness` | | LED brightness n/256 stored by the web UI; `deploy/rc.local` writes it at boot |
| `led_default` | 20 | | used while `led_file` is missing |
| `led_start_s` | 0.2 | | on start: 255 for this long (sign that xgonav runs), then `led_file`; 0 = set only. One-off, no per-frame cost |

### `[camera]`
| Key | Default | RiderPi | Note |
|---|---|---|---|
| `source` | `picam` | `picam` | |
| `width` × `height` | 640 × 480 | | |
| `fps` | 24 | 24 | |
| `sensor_mode` | `1296x972` (sim: `""`) | | pure 2×2 binning; 640x480 mode runs 26.5 instead of 25 fps |
| `scaler_crop` | `[16, 0, 2560, 1920]` (sim: `[]`) | | keeps FOV/tilt of 640x480 mode |
| `exposure_us` | 20000 (sim: 4000) | | 0 = auto; runtime-adjustable from web |
| `analogue_gain` | 16.0 (sim: 0.0) | | 1..63.9, 0 = auto (auto max 8) |
| `gain_switch` | true (sim: false) | | at `analogue_gain`: too bright (> `gain_bright_frac` 0.05 of pixels ≥ `gain_sat_level` 250, or mean > `gain_bright_mean` 120) → auto; auto at its limit (≥ `gain_auto_max` 8 − 0.5) and mean < `gain_dark_mean` 70 → `analogue_gain`; `gain_dwell_s` 2, `gain_hold_s` 5; a manual web value pauses it (`xgonav/verstaerkung.py`) |
| `contrast`, `saturation`, `brightness` | 1.0, 1.0, 0.0 | | ISP contrast/saturation/brightness of the camera (web sliders 0.5..4 resp. −0.5..0.5 at runtime, `POST /bild`; hue only in the browser, libcamera has no hue control). Idea: overdrive at the source for FloorNet, compensate in the browser (CSS filter). FloorNet is trained on 1.0: other values are a different input until it is trained on them. Per image in `meta.jsonl` `cam` |
| `hfov_deg` | 91.8 | 91.8 [K] | f = 310 px @ 640, from turns vs IMU heading (calibration.md §0) |
| `lens_knots_px`, `lens_g_px`, `lens_center` | 18 knots, centre (322.5, 232.5) (sim: empty = off) | | Lens distortion, removed right after the camera (`camera.lens_maps`, one remap per frame: `native/linsefast.cpp` 5.1 ms on one core in service (with mounting roll/yaw in the table; plus 1.5 ms copy out of the uncached camera buffer), bit-identical to `cv2.remap`, which needs 7 ms wall / 17 ms CPU on the CM4). Model: measured pixel → ideal pixel, radial `g(r)` in px at the knots. From ~67 000 straight edges of 12 drives (2026-11-18): ≤ 2 px up to r 340 px, up to 18 px in the corners. Raw recordings are undistorted from then on (`cam.entzerrt` in `meta.jsonl`) |
| `lens_roll_deg`, `lens_yaw_deg` | 1.25, 2.40 (sim: 0) | | Mounting roll and yaw, rotated out in the same table about W/2, H/2 (no cost per frame). Afterwards the image is level at IMU roll 0 and the driving direction is column W/2. Roll: raw IMU roll at which the image is level (three corridor scans 2026-11-17, formerly `[imu] roll_ref_deg`); yaw: atan(13 / 309.6), focus of expansion at column 333 (formerly `cx_offset_px`). Pixels without source are filled from the border (~26 columns on the right). `meta.jsonl` `cam.entzerrt` holds `camera.lens_id` (older recordings: `true` = radial only) |
| `lens_native` | true | | Remap in the C++ core (self-check against `cv2.remap` at start); false or core missing: `cv2.remap` |
| `capture_native` | true | | Capture via libcamera directly (`native/camfast.cpp`, `libxgocam.so`): frame undistorted straight from the DMA buffer, same stream settings as picamera2 (RGB888, SMPTE170M, noise reduction fast, raw stream for the sensor mode). Needs `lens_native` (or no lens model) and the same self-check; false, library missing or open fails: picamera2 |
| `mount_offset_mm` | 43.0 (sim: 45.0) | | camera height = body height + offset; only used without `cam_h_u8` |
| `cam_h_u8` | `[109.4, 0.1126]` (sim: `[]`) | | camera height = a + b·u8 (height set-point): measured lens centre while balancing, confirmed by the floor grid 2026-11-10 (calibration.md §2) |
| `cam_fwd_mm` | 36.0 (sim: 0.0) | | lens ahead of the wheel axle; map origin is the axle, floor points are entered this far ahead (CAD 36.9, floor check 34/41, pattern 25–35). Simulator renders the camera there too |
| `tilt_deg` | 5.18 (sim: 0.0) | | pair with `imu.pitch_ref_deg` |
| `frame_clock_corr` | 1.0 | | 640x480 mode: ~1.061 |
| `loop_file` / `realtime_file` | true / true | | video replay |

### `[imu]`
| Key | Default | RiderPi | Note |
|---|---|---|---|
| `enabled` | true | true | needs `imu_log` |
| `imu_hz` | 50 | 50 | written to `settings/imu_hz` |
| `stream_hz` | 100 (sim: 0) | | > 0: written to `settings/imu_stream_hz` → firmware stream (patch 02-pi-interface) with the firmware's sample time; driver without it: warning, polling |
| `idle_hz` | 0 | 0 | While the Rider is set down (not balancing): stream rate and read rate of the IMU thread; back to `stream_hz` and `poll_s` on stand-up. Saves ~6 % of one core on the CM4. `0` = never switch. Sleep mode still turns the stream off |
| `pitch_sign` | -1 (sim: 0) | | +1/-1 fixed; 0 = auto from image correlation (`ImuSync`: once per second, valid only after the first motion). Fixed since 2026-11-17: the online estimate no longer runs; no IMU pitch for a frame → log warning, `pitch_src` = `"bild"` |
| `yaw_scale_left`, `yaw_scale_right` | 1.0028, 1.0043 (sim: 1.0) | | Factor on heading changes of the firmware yaw (gyro scale error), separate for left (+) and right turns. Measured 2026-11-18 on the spot against a floor mark (slow): 10 × 360° right 3560° real vs. 3551.3° IMU, 10 × left 3556° vs. 3548.4°. 2026-11-23 fast (w 0.5, as in explore), with those factors applied: 6 × right 2160° vs. 2156.1° (+0.18 %), 6 × left 2158° vs. 2156.6° (+0.07 %); drift from image pairs of explore runs (`tools/drift_kurve.py`) points the same way. Recorded IMU samples are already corrected |
| `yaw_jump_dps` | 500 | | heading change faster than this, or across a gap > 0.5 s (ESP restart starts at 0), is discarded; count in status `odom.imu_spruenge` |
| `time_offset_ms` | 10.5 (sim: None) | | IMU time = frame time + this. None = auto. 10.5: `tools/imu_versatz.py` on a raw recording (2026-11-17, 8 sections +9.9..+12.0 ms, correlation 0.99). Pitch for the geometry is the pure IMU pitch at this time (no image share since 2026-11-17) |
| `pitch_ref_deg` | -5.0 (sim: 0.0) | | raw IMU pitch at camera calibration (-4.43; R-1.1.3: -2.7) plus 0.2° from a corridor of 2.04 m drivable width (three start scans 2026-11-17 gave 1.98 m); -2° for the attitude fusion of firmware patch 04-attitude (2026-11-18: pitch from gravity, pitch gyro bias calibrated); -5.0 since 2026-11-19 (corridor run 20261119-122328 gave 1.53 / 1.67 m with -6.23, 2.04 m at +1.4° / +1.0°). Only `tilt_deg + pitch_ref_deg` matters |
| `use_roll` | true (sim: false) | | FloorNet boundary is rotated by the IMU roll per frame before it reaches maps and safety (`freespace.roll_columns`); the mounting roll (1.25°) is already rotated out of the image (`[camera] lens_roll_deg`). Without it one corridor wall lay at 0.7 or 1.2 m depending on the image edge (wall 33–48 cm thick in the start scan, 10–14 cm with it; three scans 2026-11-17) |
| `min_corr` | 0.6 | | auto sync only: accepted above |
| `sync_range_ms` / `sync_step_ms` | 100 / 4 | | auto sync only |
| `poll_s` | 0.01 | | read `imu_log` this often; only new lines are parsed (RiderPi: one read 185 µs for 128 lines). At 0.05 the newest sample was up to ~50 ms older than the frame and the pitch was held, not interpolated |

### Removed 2026-11-23

Ignored with a warning: `pet.turn_min_w`, `dunkel.turn_min_w`, `robot.turn_min_w` (no lower turn bound since 2026-11-25, the firmware controls small turn rates); `robot.speed_ctrl*`, `robot.accel_v`, `robot.accel_w` (the firmware ramps and controls speed, drive frame); `robot.max_sent` (cap is `max_v`/`max_w`, sent at most ±1); `robot.manual_forward_gain`, `robot.reverse_gain` (workarounds for the old firmware; hand control = `max_v` × `tempo` × 1).

### Removed 2026-11-17

Must not be in `config.toml` any more (`KeyError` at load):

| Key | Was |
|---|---|
| `[horizon]` (whole section) | standing horizon (`xgonav/horizon.py`) |
| `grid.turn_settle_s`, `grid.turn_settle_dps` | no map entries after a fast turn (was off) |
| `explore.weak_wall_l`, `explore.pass_turn_max_deg` | corridor workarounds (were off) |
| `reloc.scan_extra_deg`, `reloc.max_shift_m`, `reloc.tries` | no user in the code |

### `[ego]`
| Key | Default |
|---|---|
| `proc_width` | 320 |
| `max_features` / `min_features` | 150 / 60 |
| `lk_win` / `lk_levels` / `lk_iters` | 11 / 3 / 10 |
| `lk_fb_check` | true |
| `ransac_px` | 2.0 |
| `native` | true | use C++ core `xgonav/native/libxgofast.so` if built (`make -C xgonav/native`): same result (max. difference 1e-13°, 6227 frames), one image pyramid per frame instead of four; gray+downscale, sharpness and `goodFeaturesToTrack` re-implemented bit-identically (checked against OpenCV at startup, else OpenCV); CM4 offline 7.5 → 4.2 ms/frame (p95 6.1 ms), live 9.9 → 6.6 ms before the re-implementations. Missing/stale `.so` → Python |

### `[detector]`
| Key | Default | RiderPi |
|---|---|---|
| `model` | `models/yolo11n_256_ncnn` | NCNN export dir (`model.ncnn.param/.bin`, `metadata.yaml`) or `.onnx`; CM4: NCNN 96 ms vs onnxruntime 117 ms per frame, identical boxes |
| `fallback_model` | `models/yolo11n_256.onnx` | used if `model` or the `ncnn` package is missing |
| `input_size` | 256 | |
| `conf` / `iou` | 0.35 / 0.45 | |
| `threads` | 2 | 2 |
| `max_hz` | 5.0 (sim: 0.0) | |
| `classes` | `["person", "chair", "couch", "dining table", "potted plant", "bed", "dog", "cat"]` (sim: `[]`) | selectable targets for `follow`/`approach` (web dropdown); `[]` = all COCO. Recognition only, never obstacles |
| `sharpness_rel` / `max_wait_s` | 0.6 / 0.25 | |

### `[tracker]`
`iou_match` 0.25, `max_lost_s` 1.5, `min_hits` 2.

### `[freespace]`
Floor boundary from FloorNet only (`xgonav/freespace.py`, `docs/floornet.md`). There is no fallback: without the network (file missing, C++ core missing or self-test failed, runtime error, invalid rows) the Rider is **blind**: `safety()` stops every mode except `manual`, `explore`/`approach`/`home`/`patrol` are refused, `/status` `fs_src` = `"blind"`. The classic floor model (`farbe`, `kanten`, lens shading) was removed on 2026-11-18. Its keys in an old `config.toml` log a warning and are ignored; `shadow_net` is read as `net`.

| Key | Default | RiderPi | Note |
|---|---|---|---|
| `enabled` | true | | false → only `idle`, `follow`, `manual` allowed |
| `threaded` | true (sim: false) | | network in its own thread (`FreeSpaceWorker`); false = in the frame loop |
| `width` | 160 | | columns of the working resolution (maps, sectors) |
| `sectors` | 5 | | |
| `restart_hold_s` | 1.0 | | no boundary taken for this long after pipeline start, standing up, exposure/gain change |
| `net` | `models/floornet.bin` (sim: `""`) | | FloorNet weights (`tools/floornet/export.py`). The simulator injects `sim.SimFloorNet` (boundary from ground truth) instead |
| `net_outlier_px` | `25.0` | `25.0` | FloorNet decoder, spike removal: a column further than this many full-frame rows from the median of the 7 columns around it takes the best bin at that median, if enough probability mass lies there. `0` = plain best bin. See `docs/floornet.md` |
| `net_outlier_ratio` | `0.3` | `0.3` | Mass (5 bins) required at the median, relative to the mass at the column's own peak |
| `net_dir` | `models/floornet` | | folder of selectable FloorNet weights (`*.bin`); web *Bodenmodell* switches by repointing the symlink `net` and reloading the net in-process (`POST /floornet`). `deploy/install.sh` creates folder and symlink (repo net as `p11a.bin`). `""` = not selectable |
| `net_smooth_n` | 3 | | boundary smoothed over time: the last n outputs are mapped onto the current frame (map pose, pitch) and the median per column counts; never averaged. `3` removes a spike seen in one frame, a new obstacle appears one network frame (83 ms) later. `0`/`1` = off; without a map pose the output passes unchanged |

### `[map]` (short-term local map)
`max_age_s` 4.0, `confirm` 3, `confirm_radius_m` 0.08, `radius_m` 0.18 (path check: a confirmed point closer than this to the arc brakes), `radius_tight_m` 0.08 (same while the path follower drives a tight spot, `home.tight_m`; Rider half width 5.8 cm per CAD); moves with the dead-reckoning step (`[odom]`). `native` true: moving, entering (ground geometry, clear seen floor, thin, forget), confirming and the arc check in the C++ core (`xgonav/native/navfast.cpp`) if built; same result as numpy (CM4, 2000 points: arc check after a new observation 5.8 → 0.4 ms, confirmed points for the planner 94 → 0.6 ms).

### `[behavior]`
| Key | Default | RiderPi |
|---|---|---|
| `mode` | idle | idle |
| `target_class` | person | person |
| `follow_dist_m` | 0.9 | |
| `approach_dist_m` | 0.5 | 0.5 |
| `dist_deadband` | 0.1 | |
| `kp_turn` / `kd_turn` | 2.5 / 0.25 | |
| `kp_dist` / `ff_dist` | 0.35 / 0.8 | |
| `cruise_v` / `backup_v` | 0.30 / 0.20 | | factors of `max_v` (absolute 0.15 / 0.10) |
| `search_w` | 0.14 | | factor of `max_w` (absolute 0.07) |
| `turn_w` (explore: turn away) | 0.42 | | factor of `max_w` (absolute 0.21) |
| `search_timeout_s` | 10.0 | | search without room map (turning in place), then wait |
| `lost_seek` | true | | follow, target lost: drive to its last known position via the planner, then look around; needs room map + confirmed pose |
| `lost_seek_max_m` / `lost_arrive_m` | 4.0 / 0.25 | | cap of the distance used for the last position / arrived there |
| `lost_vel_s` / `lost_lead_s` / `lost_lead_max_m` / `lost_lead_min_mps` | 1.0 / 1.5 / 1.5 / 0.15 | | expected position instead of the last seen one (2026-11-23, also when following in `pet`): target speed in the room map over the last `lost_vel_s`; faster than `lost_lead_min_mps` → last position + speed × `lost_lead_s`, at most `lost_lead_max_m` further. No path there → last seen position. Shown in the room map (orange ring) and `/status` `erwartet` |
| `lost_look_deg` / `lost_look_w` | 360 / 0.20 | | look around at the last position: angle, turn rate (factor of `max_w`) |
| `blocked_free` / `slow_free` | 0.22 / 0.55 | |
| `ttc_avoid_s` / `ttc_stop_s` | 1.2 / 0.8 | |
| `push_v` / `push_s` / `push_n` / `push_free_m` | 0.90 (factor, absolute 0.45) / 1.2 / 2 / 0.4 | push: forward commanded, < 2 cm progress for 2.5 s (1 s after a push) and FloorNet ≥ `push_free_m` free ahead → drive `push_v` for `push_s`; at most `push_n` times until 0.3 m progress, then back up as before. Path following, `home`, `patrol`, and the reactive escape of `explore`. 0 = off. Not driven (carpet groove 2026-11-17) |
| `kante_v` / `kante_fenster_s` / `kante_s` | 0.15 / 0.5 / 0.5 | | fast push (2026-11-23): wheels ≥ `kante_v` m/s within `kante_fenster_s`, then < 0.03 m/s while forward was commanded (joint, edge) and < 2 cm progress for `kante_s` → push, even if `safety()`/arc check have lowered the command meanwhile (joint in front of a step: free space and arc check cut v, the 2.5 s rule never fired). `kante_v` 0 = off. Simulator only (scene `fuge`) |
| `push_rest_m` / `push_min_m` / `push_max_m` / `push_brake_s` | 0.05 / 0.03 / 0.15 / 0.25 | | every push drives at most min(FloorNet free ahead, local-map clearance) − `push_rest_m`, capped at `push_max_m`; no push below `push_min_m`; ends with a full brake once distance + wheel speed × `push_brake_s` reaches it. A push skips free space/TTC in `safety()` and the arc-check distance (keep-out zones still apply) |
| `height_cruise_mm` / `height_search_mm` / `height_fast_mm` | 85 / 100 / 80 | |

### `[output]`
`web_host` 127.0.0.1, `web_port` 8088 (0 = off), `web_fps` 10, `jpeg_native` true (browser JPEG in the hardware encoder `jpeg_dev` = `/dev/video31`, fallback `cv2.imencode`), `lcd_fifo` `""` = off (sim: `/tmp/lcd0`), `lcd_fps` 15, `record_dir` `~/fahrt` (raw recording), `record_min_free_mb` 500 (below: recording switches itself off), `record_jpeg_q_hw` 95 (raw-recording JPEG in the hardware encoder; RiderPi: 4.6 instead of 8.6 ms CPU per image, mean deviation 0.70 vs 0.63, 51 vs 41 KB; above 95 only larger), `record_jpeg_q_cv` 92 (libjpeg fallback), `record_map_s` 2.0 (room map PNG every n s into `karte/`, 0 = off), `record_map_px` 480, `record_map_view_m` 6.0. `meta.jsonl` names the encoder per image (`"jpeg": "hw95"`/`"cv92"`; missing before 2026-11-19 = libjpeg q92).

### `[emotion]`
`enabled` true, `dir` `/opt/xgonav/emotions`, `fifo` `/tmp/lcd0`, `gap_s` 5, `lost_s` 2, `battery_low` 20 %, `battery_repeat_s` 300.
Sounds: `sound` true, `sound_dir` `/opt/xgonav/sounds`, `sound_found` `freude.wav`, `sound_lost` `trauer.wav`, `sound_device` `""` (ALSA default).

### `[pet]` (mode `pet`, see [display-emotions.md](display-emotions.md))
All keys with comments: `PetCfg` in `xgonav/config.py`.

| Key | Default | Note |
|---|---|---|
| `retreat_m` | 0 (off) | jerk backwards per scare, cap 0.30 m, RiderPi test 0.12. Blind, needs wheels |
| `creep_m` | 0 (off) | toward a cat per encounter, cap 1.0 m, test 0.4. Needs wheels, floor model, local map |
| `seek_m` | −1 = no limit | person gone: drive toward the last position, per loss. Needs wheels, floor model, local map |
| `roam_m`, `roam_radius_m` | −1, −1 = no limit | nobody around: roam in legs per phase without encounter (ends at `doze_s`); radius around the stand-up spot |
| `look` | true | look around (turning only) after a person is gone and when idle |
| `roam_after_s`, `roam_repeat_s`, `roam_leg_m`, `roam_free_min`, `roam_battery` | 30, 45, 0.4, 0.55, 40 % | |
| `seek_stop_m`, `drive_v`, `look_w`, `look_step_deg`, `look_steps`, `look_pause_s` | 0.4, 0.24, 0.48, 90, 4, 1.5 | factors (absolute 0.12 / 0.24); faster since 2026-11-13 (user); `turn_w` 0.80, `wiggle_w` 0.20, `creep_v` 0.12, `retreat_v` 0.20 (factors) |
| `hear`, `hear_min_db`, `hear_front_deg`, `hear_turns`, `hear_listen_s`, `hear_repeat_s`, `hear_side_deg` | true, 8, 20, 3, 3.5, 6, 45 | hearing ([display-emotions.md](display-emotions.md#hearing-sound-direction-audiomd)); needs `[ortung]` with `pet` in `modes` |
| `hear_startle_db`, `hear_quiet_s` | 20, 8 | sudden loud sound after silence |
| `call_events`, `call_m`, `call_repeat_s` | 3, −1 = no limit, 3 m per call, 45 | calling: drive toward a voice from the front, never more than `seek_m` |
| `hear_seek` | true | heard, nobody in view: search instead of giving up (direction known: drive there like calling; unknown: look around once, listening `hear_listen_s` per view); lockout `call_repeat_s` |
| `seek_home` | false (2026-11-22) | search after hearing over, nobody found: back to the map start like `home` (start image check), only with an anchor there; false: stays and listens |
| `call_openings`, `call_louder_db` | 4, 3.0 | call search via the openings of the room map (needs map + confirmed pose); 0 = old straight search (calling goals with a wall ahead go the other way) |
| `call_dir_s_per_deg`, `call_min_m` | 0.1, 0.3 | call search after he turned toward the sound: openings off the heard direction cost this much s per degree (90° = 9 s); openings with a shorter path than `call_min_m` do not count (2026-11-22) |
| `lost_home` | false | the same after the search for a vanished person; false: stays there and listens |
| `follow`, `follow_m`, `follow_repeat_s` | true, −1 = no limit, 3 | any person in view (not lying/sitting) → exactly the routine of mode `follow` (`behaviors.Follow`: `behavior.follow_dist_m`, speed, backwards up to `backup_v`, heights, search at the last place); follow waits → pet again; lockout from the end (2026-11-22; `follow_trust`, `follow_dist_m`, `follow_start_m`, `follow_hold_s`, `follow_settle_s`, `follow_block_s`, `follow_lost_s` removed, warning) |
| `wide_babble_s` | 6 | person box wider than tall (lying/sitting): no following, turns toward her, trust grows, babbles every 6 s with head tilt |
| `eyes_fix_s`, `eyes_fix_lid` | 1.5, 2 | eyes narrow to slits once a person is fixated (0 s = off) |
| `come_events`, `come_m`, `come_stop_m`, `come_repeat_s` | 1, −1 = no limit, 0.4 (2026-11-22, was 0.8), 8 | come here: drive toward a visible person who speaks (also after a call search found someone); no planner path → straight toward her; at most 30 s |
| `talk`, `talk_min_events`, `talk_pause_s`, `talk_repeat_s` | true, 2, 1.6, 7 | talking back |
| `parrot`, `parrot_p`, `parrot_repeat_s`, `parrot_delay_s`, `parrot_listen_s` | true, 0.5, 30, 0.2, 3 | voice samples: play instead of babbling, min. gap, delay before parroting a confirmed/recognized sample, capture while person gone ≤ s ([audio.md](audio.md#voice-samples-xgonavstimmepy-mode-pet)) |
| `faint` | true | hard scare: set down, wake up, stand up by itself; on at the RiderPi since 2026-11-13 |
| `faint_level` | 0.8 | scare level that triggers a faint |
| `gestures` | true | false: only eyes and sounds |
| `zone_far_m`, `zone_mid_m`, `zone_close_m`, `kneel_max_m` | 3.0, 1.5, 0.5, 1.9 | person zones |
| `startle_mps`, `startle_heavy_mps`, `startle_max_m` | 1.0, 1.8, 1.6 | approach speed = min(foot-point line fit, measured box-width growth × distance / `startle_vs`); widths of boxes cut at the left/right image edge are ignored; **not tuned on real footage** |
| `startle_vs` | 1.0 | < 1 trusts the box growth more |
| `pulse_margin` | 0.85 | budgets are time pulses (budget / speed × margin), wheels are only the upper limit: after a stop command it coasts ~0.9 s × speed |
| `dog_fear_m`, `dog_close_m`, `cat_stop_m` | 2.0, 0.5, 0.8 | |
| `nap_s`, `eyes_open_s`, `clear_s`, `nap_max_s` | 10, 3, 2.5, 60 | eyes shut, opening, nobody in view, give up |
| `duck_mm`, `roll_deg` | 75, 8 | raised to minimum + 10 mm; cap 12° |
| `active_hz`, `doze_hz` | 4, 1 | detector rate in this mode |
| `doze_s`, `timeout_s` | 180, 600 | nobody around: doze; set down and `idle` |
| `gaze_dir` | `/opt/xgonav/eyes_gaze` | eye atlas |

### `[ruhe]` (sleep mode, see [web-interface.md](web-interface.md#sleep-mode-ruhe))
| Key | Default | Meaning |
|---|---|---|
| `tick_s` | 1.0 | main loop and `robot` thread wake-up period in sleep mode |
| `sleep_ms_on_loop` | 1000 | driver main loop period in sleep mode (normally 200) |

### `[ortung]` (sound direction, see [audio.md](audio.md#sound-direction-xgonavortungpy))
| Key | Default | Meaning |
|---|---|---|
| `enabled`, `modes` | true, `idle` `pet` | only with `--robot proc`; never in sleep mode |
| `device`, `rate`, `block` | `hw:CARD=wm8960soundcard,DEV=0`, 48000, 1024 | `arecord` capture; block = power of two |
| `mic_dist_m`, `tau_offset_us`, `sign` | 0.0038, −2.2, 1 | effective microphone spacing and fixed channel offset (calibration 2026-11-13, balancing); −1 swaps left/right |
| `f_lo`, `f_hi` | 300, 2000 | band in Hz; above 2 kHz the channels are not phase-true |
| `tau_margin`, `steer_steps`, `phat_beta` | 1.5, 20, 0.8 | search range × spacing, delays per side, 1 = phase only |
| `on_db`, `off_db`, `min_level` | 9, 3, 0 | gate opens / stays open above the noise floor; absolute floor (0 = off) |
| `noise_up`, `noise_down`, `warm_blocks` | 0.001, 0.2, 25 | noise floor tracking per block; blocks after start without events |
| `window_blocks`, `min_blocks`, `hang_blocks`, `min_interval_s` | 47, 24, 12, 0 | blocks per estimate (1 s); shorter sounds dropped; pauses bridged; pause after an estimate |
| `min_quality`, `nachhall_s`, `retry_s` | 0.4, 0.3, 10 | drop below; deaf after own sounds; retry when `arecord` ends |

### `[stimme]` (voice samples in `pet`, see [audio.md](audio.md#voice-samples-xgonavstimmepy-mode-pet))
| Key | Default | Meaning |
|---|---|---|
| `enabled`, `dir` | true, `/opt/xgonav/stimmen` | needs `[ortung]` and sounds (`aplay`) |
| `max_confirmed`, `pool_n`, `kandidat_max_min` | 10, 30, 1 | confirmed samples (when full, a new one replaces the least heard), unconfirmed kept, minutes to get confirmed |
| `vad_on_db`, `vad_off_db`, `vad_hang_s`, `vad_pre_s`, `min_s`, `max_s` | 6, 4, 0.6, 0.3, 0.3, 3 | utterance detection on the ortung level (pre-roll, hang-over) |
| `trim_db`, `laut_db` | 12, 10 | features only within trim_db of the loudest frame; voicing counted within laut_db |
| `voiced_min`, `flat_max`, `per_min` | 0.4, 0.3, 0.5 | share of voiced frames; voiced = flatness below and pitch strength above |
| `match_dist`, `dtw_band`, `len_ratio` | 8, 0.25, 1.6 | DTW match (labelled recording on the Rider: same phrase 5.1–8.2, different ≥ 8.8) |
| `gate_db`, `knee_db` | 12, 6 | noise gate before the filter |
| `pitch_st`, `speed`, `band_lo`, `band_hi`, `vibrato_hz`, `vibrato_ms`, `drive`, `peak_db` | 12, 1.0, 1000, 5000, 25, 0.3, 2, −3 | budgie filter: semitones higher at the same tempo, extra tempo factor (> 1 = higher and faster), band, vibrato, drive, peak |

### `[odom]`
| Key | Default | RiderPi |
|---|---|---|
| `enabled` / `source` | true / `auto` | `auto` = wheels if fresh, else command × `vx_limit` |
| `wheel_diameter_m` | 0.060 | |
| `track_m` | 0.090 | | Turn test 2026-11-18: wheel heading 0.0–0.4 % low, within slip scatter, unchanged |
| `ticks_per_rev` | 1024 | |
| `scale_left` / `scale_right` | 1.0080 / 1.0080 [K] (sim: 1.0 / 1.0) | | since 2026-11-19 equal (mean of the 2026-11-04 tape run): wheels against IMU heading in 66 raw runs (`tools/odom_replay.py`) showed no difference between the wheels (±0.2 % per run) |
| `poll_hz` / `max_age_s` / `max_jump_ticks` | 25 / 0.3 / 2048 | |
| `read_idle` | `false` | `false` | `false`: while the Rider is set down (not balancing) the wheel thread reads nothing; the pose stands still even if he is rolled by hand. Reading restarts on stand-up without the travel in between. `true` = always read |
| `contra_window_s`, `contra_deg`, `contra_frac` | 1.0, 4.0, 0.10 | | dead reckoning (`odomfusion.py`): wheel heading differs from IMU heading by more than `contra_deg` + `contra_frac` × turn within the window → slip (heading stays IMU, distance from the wheel matching the IMU turn) |
| `cam_err_deg`, `cam_err_deg_m`, `imu_doubt_s` | 1.0, 5.0, 5.0 | | the camera judges a contradiction while its possible error (`cam_err_deg` + `cam_err_deg_m` × distance in the window) is below a quarter of it; if it sides with the wheels, heading comes from the wheels for `imu_doubt_s` and the window is made up. 67 raw runs: wheels minus IMU per 1 s p95 ~2°, p99 ~3.5°; IMU heading swings 40–140° when the Rider tips over |
| `calib_seg_s`, `calib_n` | 2.0, 300 | | suggestion for `scale_left`/`scale_right`/`track_m` from slip-free segments against the IMU heading, status `odom.vorschlag`, display only; 0 = off |

### `[grid]` (room map)
| Key | Default |
|---|---|
| `enabled` | true |
| `resolution_m` / `size_m` | 0.05 / 48.0 (16 until 2026-11-17, then 24: a straight corridor hit the edge 12 m from the start and explore turned back; sim 16; a smaller saved map is loaded centred, a larger one ignored) |
| `min_range_m` / `max_range_m` | 0.15 / 2.5 (hits only up to here, distance along the ray since 2026-11-23; 1.5 until 2026-11-24) |
| `side_range_m` | 1.0 (since 2026-11-24): hits and free rays only inside the ellipse `max_range_m`/`free_range_m` ahead × this sideways (camera frame); range along bearing β = 1/√(cos²β/R² + sin²β/side²): 2.5 m ahead, ~1.3 m at the image edge (46°), never more than 1 m to the side; 0 = off |
| `confirm_dr_m` / `confirm_cells` | 0 = off (since 2026-11-14, one sighting is enough with FloorNet; 0.2 before) / 2: with > 0 a hit counts only once the same spot (± cells) was hit from ranges at least this far apart; until then it is a candidate (orange in `/map.png`). Reflections in a glossy floor and seams move with the Rider and are never confirmed; turning on the spot alone confirms nothing |
| `mark_blocking` | true | `LocalMap` points the path check brakes for are entered as confirmed and occupied (what stops the Rider is on the map); free rays remove them again if the spot is later seen free |
| `brake_block_s` | 300 | full brake while already in tight mode (`[map] radius_tight_m`): the spot does not fit, planner and goal search block the hit points plus `brake_block_r_m` this long; free rays do not reopen it, only time. In memory only. 0 = off |
| `brake_block_r_m` | 0.18 | radius of that block (= `[map] radius_m`) |
| `brake_block_max_mps` | 0.25 | only if the wheels were at most this fast: a full brake from cruising speed in tight mode means "too fast", not "does not fit" (it closed the 25 cm door in sim `spalt` once the follower braked earlier) |
| `free_range_m` | 2.5 (1.5 until 2026-11-24, RiderPi 1.1 from 2026-11-06 to 2026-11-14); free rays up to here (along the ray, limited sideways by `side_range_m`) |
| `column_step` | 2 |
| `l_free` / `l_occ` | -0.4 / 0.5 (two hits to `l_show`) |
| `l_min` / `l_max` / `l_show` | -3.0 / 3.5 / 0.7 |
| `free_margin_m` / `occ_free_factor` | 0.10 / 0.5 |
| `hit_band_quad` | 0.05: a hit is entered as a band ± this × d² along the ray (1 m ±5 cm, 1.5 m ±11 cm), not one cell. Ranges from the image scatter ±0.3 m at 1–1.5 m (drive 20261118-135118, wall behind chairs): single cells rarely got two hits and free rays of other frames removed them. 0.08 merged chair and table legs. Since 2026-11-18, not driven |
| `view_m` | 6.0 |
| `threaded` | true |
| `native` | true: one observation in one call in the C++ core (`xgonav/native/gridfast.cpp`) if built: rays from the floor boundary, free rays, hits with `confirm_dr_m`; same map as numpy (CM4 1.64 → 0.18 ms per observation). Also `wall_dist` (`navfast.cpp`) |
| `file` | `""` (off); RiderPi example: `/opt/xgonav/raumkarte.npz` |
| `save_every_s` | 60.0 (only if changed; also on exit) |

### `[reloc]` (pose state, map matching)

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `false` | pose state: unknown after set-down, fall, restart; map locked until confirmed in the web UI (`docs/mapping.md`) |
| `blind_s` | 2.0 | balancing this long without fresh wheel data: pose unknown |
| `track` | `false` | running map matching while driving: map pose follows the room map (`docs/mapping.md`, Map matching). Independent of `enabled` |
| `every_s` / `min_move_m` / `min_turn_deg` | 2.0 / 0.2 / 20 | match at most this often and only after this much travel or turn |
| `max_step_m` / `max_step_deg` / `gain` | 0.08 / 3 / 0.6 | correction per match |
| `hold_m` / `hold_deg` | 0.10 / 3 | remaining correction above this: nothing is entered into the map |
| `ref_age_s` / `ref_every_s` | 12 / 10 | while mapping: match against a map state at least this old; new state this often |
| the rest | | windows, weights, quality limits, global search (`xgonav/config.py`, `RelocCfg`) |

### `[startbild]` (start image check, `xgonav/startbild.py`)

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | `home` ends via a pre-point, a straight run onto the start and a measurement against the image taken at the start; the room-map pose is corrected. Driven once on the Rider (2026-11-19) |
| `vor_m` / `vor_tol_m` | 0.40 / 0.08 | pre-point this far before the start on the start heading; arrival tolerance there |
| `gerade_v` | 0.16 | speed of the straight run, factor of `max_v` (~10 cm/s) |
| `tiefe_schritt_m` / `basen` | 0.04 / 0.1…0.3 | keep a grey image every 4 cm of the straight run; depth images at these distances before the end |
| `features` | 3000 | SIFT features per image |
| `min_inl` | 30 | fewer consistent matches: no correction (offline outliers all < 27) |
| `max_m` / `max_deg` | 0.5 / 15 | larger correction counts as a wrong measurement |
| `max_lage_deg` / `agree_m` | 1.0 / 0.05 | pitch/roll residual of the anchor; free 6-DOF cross-check must lie this close |
| `nach_m` | 0.04 | remaining distance after the correction above which `home` drives once more |
| `end_deg` / `end_s` | 2.0 / 8.0 | final turn to the start heading after the check: tolerance (instead of `[home] align_tol_deg` 10°); stops when it turns past the heading or after `end_s` |
| `timeout_s` | 20 | wait this long for the measurement |
| `anker_m` / `anker_deg` | 0.03 / 5.0 | anchor image is taken while the Rider stands this close to the start (once per start point); the anchor keeps its actual pose, so the limit only means "at the start" (1 cm / 1° was too strict after standing up) |

### `[home]` (return to start)
| Key | Default | Meaning |
|---|---|---|
| `inflate_m` | 0.23 | normal clearance to occupied cells; also margin of no-go zones and the cleared disc around the pose |
| `clear_cost` | 4.0 | extra cost near obstacles (fades out at 2 × `inflate_m`) |
| `squeeze_m` / `squeeze_cost` | 0.09 / 20 | cells closer than `inflate_m` are not blocked but cost this much more; blocked only below `squeeze_m` (cell centre distance; 0.09 = direct neighbours at 5 cm cells, a 25 cm gap stays open in any grid position). The path squeezes through only if there is no other way. 0 = off (gaps from ~0.45 m, as until 2026-11-11) |
| `tight_m` | 0.21 | tight spot: occupied map cell closer than this to the pose or to the path up to 0.45 m ahead (0 = off) |
| `tight_lookahead_m` / `tight_v` | 0.2 / 0.6 | tight spot: lookahead, speed factor |
| `tight_turn_enter_deg` / `tight_turn_exit_deg` | 30 / 10 | tight spot: turn in place thresholds |
| `tight_center` / `tight_center_max_deg` | true / 30 | tight spot: aim at the middle between the nearest confirmed `LocalMap` points left and right ahead, if within this angle of the path direction (status „Lücke“) |
| `unknown_cost` | 3.0 | extra cost for unknown cells |
| `margin_m` | 1.5 | search area = pose, start, driven path + margin |
| `lookahead_m` | 0.5 | pure pursuit |
| `glatt_r_m` / `glatt_step_m` | 0.3 / 0.05 | corners of a newly planned path become circular arcs (`planner.glaetten`; home, patrol, the chosen explore path): radius halved up to 3× where the planner's blocked cells require it, else the corner stays. 0 = corners as before 2026-11-25 (work plan point 3: drive curves instead of turning in place) |
| `kurve_a_mps2` / `kurve_vorschau_m` | 0.3 / 1.5 | curve speed (`PathFollower.kurve_v`): in every corner of the path up to `kurve_vorschau_m` ahead at most √(a / curvature), braking before it with `end_decel_mps2` (≈ 0.3 m/s on a 0.3 m arc). Status `Kurve in …m`. 0 = off |
| `arrive_m` | 0.15 | goal radius |
| `end_decel_mps2` / `end_lag_s` / `end_min_v` | 0.25 / 1.0 / 0.15 | braking before the path end (every drive that stops there, also `home`/`patrol`; `explore` only without a planned next goal): speed ≤ √(2·a·s), s = remaining path length minus wheel speed × `end_lag_s` (the Rider applies a set-point only after ~0.7–1 s), at least `end_min_v` × `max_v`. Full `explore` speed starts braking ~1.8 m before the end (before 2026-11-23: 0.45 × straight-line distance, from ~0.9 m; `home` did not brake) |
| `free_slow` / `free_rest_m` | true / 0.10 | same braking profile before the first local-map obstacle along the path and on the arc of the drive command (same arc and radius as the arc check, up to 2.5 m; s minus `free_rest_m`) and before the first sharp corner (> `turn_enter_deg`; with pass-through also into the next path). Status `frei …m` when it limits. Before: only the arc check (look-ahead = braking distance + 0.15 m, ~0.85 m at 0.65 m/s) reacted, with a full brake (RiderPi 2026-11-24). The arc check now derives its allowed speed from the faster of command and wheels, so an early lowered command no longer forces a full brake (sim `spalt`: that closed the door via brake spots) |
| `replan_s` | 2.0 | replan interval (also after 3 s without progress) |
| `keep_factor` | 1.2 | keep the current path while free and ≤ factor × new path + 0.3 m (not after getting stuck) |
| `use_localmap` | true | confirmed `LocalMap` points (same as the path check) are obstacles for planning |
| `v` | 0.30 | speed, factor of `max_v` (`patrol`, path following in the `follow` search; `explore` uses `explore.v`, every planned path in `pet` uses `pet.drive_v`) |
| `start_v` | 0.45 | "Zum Start": mode `home` and the drive home in `pet`, +50 % (2026-11-22); final straight run `startbild.gerade_v` |
| `kp_turn` | 1.5 | 1/s, turn rate per heading error (follow uses `behavior.kp_turn`) |
| `turn_w_max` | 0.50 | cap for turning in place (path follower, final alignment), factor of `max_w` (absolute 0.25); 1.0 = `robot.max_w`. Turning at 0.5 rolled the Rider 7° and upset IMU pitch and pose (2026-11-17) |
| `turn_enter_deg` / `turn_exit_deg` | 75 / 25 | turn in place above enter until below exit; otherwise drive the curve (speed ≥ 30 %) |
| `align_heading` / `align_tol_deg` | true / 10 | turn to start heading at goal |
| `max_fail` | 3 | consecutive failed plans → stop |
| `native` | true | A*, path shortening, the path follower's target point and the loops of the frontier search (`explore`: wavefront, scoring of the groups) in the C++ core (`xgonav/native/navfast.cpp`) if built; same result as Python, GIL released during the call. CM4 on the saved room map: `plan()` 113 → 7 ms, `next_goal()` 137 → 14 ms (median) |

### `[cpu]`
Default (RiderPi): `main_cpus [0,1]`, `detector_cpus [2,3]`, `render_cpus [0,1,2,3]`, `freespace_cpus [2,3]`. Empty = no pinning (sim).
