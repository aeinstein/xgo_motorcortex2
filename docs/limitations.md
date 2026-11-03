# Limitations and measured facts

Single device: RiderPi, CM4 2 GB, kernel 6.6.74+rpt-rpi-v8, OV5647 wide-angle, firmware R-1.1.3 → R-1.1.6.

## Hardware test status

| Item | Status |
|---|---|
| Driver patches 0001/0002, IMU ~49 Hz, kernel watchdog | tested |
| Wheel encoders (0x0d/0x0c), odometry scale | tested (tape measure, 215 cm) |
| Camera geometry, lens shading | measured |
| Manual driving (web UI) | tested |
| `follow` | tested 2026-11-02 (runs 3–8, `max_v` 0.10–0.15); works, path check blocked in some runs |
| `explore`, `approach` | **not tested** |
| IMU sync while balancing | not verified |
| Patch 0003 (`force_yaw`) | not tested |
| Room map | display only, no driving decisions |

## Camera

| Item | Value |
|---|---|
| HFOV | ~91° (f ≈ 314 px @ 640 px); default/simulator still 62.2° |
| Distortion | ≤ 4 px in x 40–571 (corners not measured) |
| Height | 128 mm at body height 85 mm (`mount_offset_mm = 43`) |
| Tilt | 1.39° at IMU pitch −2.7° (R-1.1.3) |
| Lens shading | edge: blue/green to ~20 %, red to 1–9 % (from ~250 px off centre) |
| Measurable floor | middle ~6/8 of width from 18–25 cm; outer eighths and lower corners blind |
| Low light | 4 ms at 15–50 lux: gain 8 maxed, mean brightness 3/255 (auto 33 ms: 20–30/255); RiderPi config: 20 ms, gain 16 |
| 640x480 sensor mode | runs 26.5 instead of 25 fps (driver pixel rate ~6 % low); use 1296x972 + crop |

## CPU (CM4, 640×480)

| Configuration | Main loop |
|---|---|
| no detector, 30 fps | ~12 ms/frame |
| YOLO11n@256, 2 threads (~145 ms/inference, ~7 Hz) | ~24 ms |
| 3 threads or 320 px | exceeds 33 ms budget |
| floor model threaded, pinning (2026-11-02) | mean 12.6 ms, p95 17.1 ms, CPU 232–243 % |
| pipeline stopped | service CPU ~4 % |

Largest cost: optical flow (`ego.lk_*`); prebuilt LK pyramids cannot be passed via the Python binding.

## Firmware behaviour

| Item | Value |
|---|---|
| IMU heading | firmware integrates yaw only at \|rate\| > 2°/s; 60 s balancing idle: IMU +2.5°, image −2.6°. Firmware heading hold (reg 0x0A) and `force_yaw` cannot see this drift. xgonav uses `vis_yaw` |
| Turn overshoot | after stop command 10–17° further; commands within u8 128 ± 6 ignored. `tools/nudge.py` turns in two stages |
| Reverse | much weaker than forward (u8 109: 4 cm vs u8 147: 86 cm in 2 s) → `reverse_gain = 2.0` |
| Balancing drift | slow drift in position and heading |
| Firmware units | 1 unit ≈ 1 m/s assumed (`vx_limit`), map fallback only |
| Stand-up | Rider rolls ~20 cm forward |

## Perception

| Item | Value |
|---|---|
| Floor model | needs fairly uniform floor; patterned carpet / hard tile joints → phantom obstacles. Laminate OK with `edge_weight 0.4`, `min_obst_rows 6` (0 false blocks in 40 raw images). Fails on some floors / at desk (open) |
| Follow false alarms | replay `raw_follow8`: path blocked in ~49 % of control steps without real obstacle |
| Without IMU / sync | slow pitch unobservable: 1° ≈ 40 % distance error at 2 m; size fusion compensates. Status `pitch_src` shows source |
| Follow distance (sim) | ~1.5 m without / ~1.4 m with IMU at target 1.2 m (deadband + lag) |
| Explore (sim) | occasional contact in 40 s runs (2 of 5); 30 s slow test passes; cause not analysed |
| Ego-motion | spikes up to ~80 ms per frame |
| Without free space (`freespace.enabled = false`) | only `idle`, `manual`, `follow`; `explore`/`approach` → HTTP 409 |

## Other

- Web UI has no authentication.
- Room map drifts (no markers).
- Leg calibration buttons removed (saving with slack legs overwrote flash zero points, 2026-11-02).
- Battery: driver powers off below 8 % (`low_batt_level`); full image processing draws more than the charging dock supplies.
