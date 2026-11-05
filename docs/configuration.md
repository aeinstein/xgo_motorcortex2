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
| `--mode idle\|follow\|explore\|approach\|home` | `behavior.mode` |
| `--target CLASS` | `behavior.target_class` (COCO, e.g. `person`, `chair`) |
| `--source picam\|<index>\|<file>` | `camera.source` |
| `--robot proc\|sim` | default: `proc` if source is `picam`, else `sim` |
| `--model FILE` | `detector.model` |
| `--no-detector` | `detector.enabled = false` |
| `--web-port N` | `output.web_port` (0 = off) |
| `--no-lcd` | `output.lcd_fifo = ""` |
| `--record FILE.mp4` | record overlay video |
| `--record-raw DIR` | raw JPEGs + `meta.jsonl` (for `tools/replay_raw.py`) |
| `--duration S` | exit after S seconds |
| `--fast` | video file as fast as possible |
| `-v` | debug log |

## Parameters

Default = `config.py`. RiderPi = `deploy/config.example.toml` (blank = default). [K] = device-specific, calibrate.

### top level
| Key | Default | RiderPi |
|---|---|---|
| `control_hz` | 20.0 | |

### `[robot]`
| Key | Default | RiderPi | Unit / note |
|---|---|---|---|
| `max_v` | 0.25 | 0.15 | normalised; 0.15 ≈ 23 cm/s |
| `max_w` | 0.25 | 0.15 | normalised; 0.25 ≈ 90 °/s |
| `reverse_gain` | 1.0 | 2.0 [K] | multiplier for negative v |
| `vx_limit` | 1.5 | | firmware units at u8 0/255 (used for map only, uncalibrated) |
| `vyaw_limit_dps` | 360 | | °/s at u8 0/255 |
| `accel_v` / `accel_w` | 0.8 / 2.0 | | per second |
| `keepalive_s` | 0.3 | | s |
| `stall_timeout_s` | 0.5 | | s, userspace watchdog |
| `kernel_watchdog_ms` | 1000 | 1000 | `settings/motion_timeout_ms` |
| `height_min_mm` / `height_max_mm` | 60 / 120 | | |

### `[camera]`
| Key | Default | RiderPi | Note |
|---|---|---|---|
| `source` | `picam` | `picam` | |
| `width` × `height` | 640 × 480 | | |
| `fps` | 24 | 24 | |
| `sensor_mode` | `""` | `1296x972` | pure 2×2 binning; 640x480 mode runs 26.5 instead of 25 fps |
| `scaler_crop` | `[]` | `[16, 0, 2560, 1920]` | keeps FOV/tilt/shading of 640x480 mode |
| `exposure_us` | 4000 | 20000 | 0 = auto; runtime-adjustable from web |
| `analogue_gain` | 0.0 | 16.0 | 1..16, 0 = auto (auto max 8) |
| `hfov_deg` | 88.5 | 88.5 [K] | |
| `mount_offset_mm` | 45 | 43 [K] | camera height = body height + offset |
| `tilt_deg` | 0.0 | 1.39 [K] | pair with `imu.pitch_ref_deg` |
| `frame_clock_corr` | 1.0 | | 640x480 mode: ~1.061 |
| `loop_file` / `realtime_file` | true / true | | video replay |

### `[imu]`
| Key | Default | RiderPi | Note |
|---|---|---|---|
| `enabled` | true | true | needs `imu_log` |
| `imu_hz` | 50 | 50 | written to `settings/imu_hz` |
| `pitch_sign` | 0 | | 0 = auto from image correlation (RiderPi: -1) |
| `time_offset_ms` | None | | None = auto (RiderPi: +4..+8 ms) |
| `pitch_ref_deg` | 0.0 | -0.5 [K] | raw IMU pitch at camera calibration (R-1.1.3: -2.7) |
| `min_corr` | 0.6 | | sync accepted above |
| `sync_range_ms` / `sync_step_ms` | 100 / 4 | | |

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
| `max_hz` | 0 (no limit) | 5.0 |
| `classes` | `[]` (all COCO) | `person` + floor furniture (2026-11-05, as example config): `chair`, `couch`, `dining table`, `potted plant`, `bed`, `dog`, `cat` — non-target boxes block their columns (thin chair legs are invisible to `kanten`) |
| `sharpness_rel` / `max_wait_s` | 0.6 / 0.25 | |

### `[tracker]`
`iou_match` 0.25, `max_lost_s` 1.5, `min_hits` 2.

### `[freespace]`
| Key | Default | RiderPi | Note |
|---|---|---|---|
| `enabled` | true | | false → only `idle`, `follow`, `manual` allowed |
| `threaded` | false | true | |
| `width` | 160 | | columns |
| `sample_rows` / `sample_cols` / `sample_bottom` | 0.18 / 0.30 / 0.0 | 0.34 / 0.5 / | floor reference patch |
| `sectors` | 5 | | |
| `floor_thresh` | 0.08 | | |
| `edge_weight` | 0.4 | | |
| `min_obst_rows` | 6 | | |
| `min_obst_height_m` | 0.04 | | |
| `shading_file` | `""` | `shading.json` [K] | |
| `shading_max_gain` | 8.0 | 5.0 | |
| `min_known_frac` | 0.2 | | columns below → unknown |
| `bg_model` / `shading_max_gain_bg` | false / 8.0 | false | edge model from blue+green where red is unmeasurable; full width, but false obstacles in dark lower corners |
| `min_learn_v` | 12 | | no learning from (nearly) black samples |
| `restart_hold_s` | 1.0 | | relearn after pipeline start, standing up, exposure/gain change |
| `method` | `farbe` | `kanten` | floor boundary: `farbe` = color histogram; `kanten` = first step edge from the bottom with a differing patch above (uncorrected image, insensitive to shading and gray-on-gray walls); `beide` = `kanten`, color model where no edge is found |
| `edge_thin` / `edge_k` | true / 3 | | remove thin lines (joints) first; rows per side of the step |
| `edge_chroma_w` | 2.0 | | weight of LAB a/b vs. L |
| `edge_step_thr` / `edge_seg_thr` | 35 / 28 | | min step; min deviation of the patch above (min obstacle height) from the floor below |
| `edge_log_s` | 16 | | step on log brightness `ln(1+L/s)` (ratio instead of difference): dark walls/furniture in dark scenes; 0 = linear |
| `edge_glare` | true | | lamp glare on glossy floor → column unknown (NaN) instead of obstacle |
| `glare_v` / `glare_px` / `glare_dl` / `glare_spread` | 250 / 2 / 8 / 8 | | glare = brighter above the step (≥ `glare_dl`, linear L) and ≥ `glare_px` saturated pixels between step and horizon−12 rows; grown to bright neighbours with similar step row; not at the image border (lamp-lit wall looks the same) |
| `native` | true | | C++ core (`xgonav/native/floorfast.cpp`) if built: `kanten`/`beide` edge scan and the 4:1 downscale (NEON, only if bit-identical to `cv2.resize` at startup); same result as numpy |
| `edge_fallback_free` | 0.98 | | `beide`: column counts as "nothing found" from this free fraction |
| `horizon_wall` | false | | `farbe` only: floor color continuing above the horizon = wall, base at first strong edge (false alarms in the simulator from pitch errors) |
| `horizon_margin_rows` / `horizon_band_rows` / `horizon_floor_frac` / `wall_edge_thr` | 2 / 6 / 0.6 / 8 | | horizon rule band and edge threshold |

Replay of three evening drives (`tools/replay_raw.py`, 2026-11-04), false alarms: `farbe` 25 / 35 / 53 %, `kanten` 0.2 / 2.4 / 16 %.

2026-11-05 (`edge_log_s` 16, `edge_glare`): replay 0 / 0 / 54 % — follow11 now sees the kitchen cabinet side 0.16 m beside the path and the 0.33 m gray wall it missed before; the replay counts these as false alarms (person far). Manual test cases (19 frames): 17 OK (before 13). Cost +0.2 ms per call on a PC.

### `[map]` (short-term local map)
`max_age_s` 4.0, `confirm` 1 (RiderPi 3), `confirm_radius_m` 0.08.

### `[behavior]`
| Key | Default | RiderPi |
|---|---|---|
| `mode` | idle | idle |
| `target_class` | person | person |
| `follow_dist_m` | 1.2 | 0.9 |
| `approach_dist_m` | 0.5 | 0.5 |
| `dist_deadband` | 0.15 | 0.10 |
| `kp_turn` / `kd_turn` | 2.5 / 0.25 | |
| `kp_dist` / `ff_dist` | 0.35 / 0.8 | |
| `cruise_v` / `backup_v` | 0.15 / 0.10 | 0.15 / |
| `search_w` | 0.12 | 0.07 |
| `search_timeout_s` | 20 | 10 |
| `blocked_free` / `slow_free` | 0.22 / 0.55 | |
| `ttc_avoid_s` / `ttc_stop_s` | 1.2 / 0.8 | |
| `height_cruise_mm` / `height_search_mm` / `height_fast_mm` | 85 / 115 / 70 | 85 / 100 / 80 |

### `[output]`
`web_host` 127.0.0.1, `web_port` 8088 (0 = off), `web_fps` 10, `lcd_fifo` `/tmp/lcd0` ("" = off), `lcd_fps` 15.

### `[emotion]`
`enabled` true, `dir` `/opt/xgonav/emotions`, `fifo` `/tmp/lcd0`, `gap_s` 5, `lost_s` 2, `battery_low` 20 %, `battery_repeat_s` 300.
Sounds: `sound` true, `sound_dir` `/opt/xgonav/sounds`, `sound_found` `freude.wav`, `sound_lost` `trauer.wav`, `sound_device` `""` (ALSA default).

### `[odom]`
| Key | Default | RiderPi |
|---|---|---|
| `enabled` / `source` | true / `auto` | `auto` = wheels if fresh, else command × `vx_limit` |
| `wheel_diameter_m` | 0.060 | |
| `track_m` | 0.090 | |
| `ticks_per_rev` | 1024 | |
| `scale_left` / `scale_right` | 1.0 / 1.0 | 1.0073 / 1.0087 [K] |
| `poll_hz` / `max_age_s` / `max_jump_ticks` | 25 / 0.3 / 2048 | |
| `yaw_source` | `räder` | heading for explore + local map; `kamera` = ego-motion only |

### `[grid]` (room map)
| Key | Default |
|---|---|
| `enabled` | true |
| `resolution_m` / `size_m` | 0.05 / 16.0 |
| `min_range_m` / `max_range_m` | 0.15 / 2.5 |
| `column_step` | 2 |
| `l_free` / `l_occ` | -0.4 / 1.2 |
| `l_min` / `l_max` / `l_show` | -3.0 / 3.5 / 0.7 |
| `free_margin_m` / `occ_free_factor` | 0.10 / 0.5 |
| `view_m` | 6.0 |
| `yaw_source` | `räder` (wheels) or `kamera` |
| `threaded` | true |
| `native` | true: enter rays in the C++ core (`xgonav/native/gridfast.cpp`) if built; same map as numpy (CM4 1.5 → 0.1 ms per observation) |
| `file` | `""` (off); RiderPi example: `/opt/xgonav/raumkarte.npz` |
| `save_every_s` | 60.0 (only if changed; also on exit) |

### `[home]` (return to start)
| Key | Default | Meaning |
|---|---|---|
| `inflate_m` | 0.18 | occupied cells inflated, blocked for planning |
| `clear_cost` | 4.0 | extra cost near obstacles (fades out at 2 × `inflate_m`) |
| `unknown_cost` | 3.0 | extra cost for unknown cells |
| `margin_m` | 1.5 | search area = pose, start, driven path + margin |
| `lookahead_m` | 0.35 | pure pursuit |
| `arrive_m` | 0.15 | goal radius |
| `replan_s` | 2.0 | replan interval (also after 3 s without progress) |
| `v` | 0.15 | normalised speed |
| `align_heading` / `align_tol_deg` | true / 10 | turn to start heading at goal |
| `max_fail` | 3 | consecutive failed plans → stop |

### `[cpu]`
Empty = no pinning. RiderPi: `main_cpus [0,1]`, `detector_cpus [2,3]`, `render_cpus [0,1,2,3]`, `freespace_cpus [2,3]`.
