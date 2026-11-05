# Limitations and measured facts

Single device: RiderPi, CM4 2 GB, kernel 6.6.74+rpt-rpi-v8, OV5647 wide-angle, firmware R-1.1.3 → R-1.1.6 (+ patches 0001–0005 since 2026-11-05).

## Hardware test status

| Item | Status |
|---|---|
| Driver patches 0001/0002, IMU ~49 Hz, kernel watchdog | tested |
| Wheel encoders (0x0d/0x0c), odometry scale | tested (tape measure, 215 cm) |
| Wheel heading vs IMU | 2026-11-05, 184 s / 15 m / 25 turns: slope 1.000, 3° apart at the end (track 0.090 m fits) |
| Camera geometry, lens shading | measured |
| Manual driving (web UI) | tested |
| `follow` | tested 2026-11-02 (runs 3–8, `max_v` 0.10–0.15); works, path check blocked in some runs |
| `explore` | short manual tries 2026-11-04/05, no recording; stood in front of obstacles without turning away (fixed, `path_blocked`) |
| `home` | 2026-11-05, 3 tries: did not reach the start. Stop/turn/start cycles and changing plans (45 s without progress), then stuck 1.3 m before the start at a gap the path check considers too narrow; afterwards changed (path keeping, `LocalMap` obstacles in planning, curve driving), **not retested** |
| `approach` | **not tested** |
| IMU sync while balancing | not verified |
| Patch 0003 (`force_yaw`) | not tested |
| Room map | used only by `home`; drifts (no markers); loading assumes the Rider stands at the start point |

## Camera

| Item | Value |
|---|---|
| HFOV | 88.5° (f ≈ 328 px @ 640 px, folding-rule measurement); default and simulator use it |
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
| `kanten` + NCNN detector + ego/floor optimisations (2026-11-05, idle, Rider lying) | mean ~12 ms, p95 ~20 ms; ego 9.9 ms/frame (was 12.0); detector ~133 ms (onnxruntime ~170); floor model ~17 ms; CPU ~182 % |
| + ego-motion C++ core (2026-11-05) | mean ~9.4 ms, p95 ~15 ms; ego 6.6 ms/frame; CPU ~167 % |
| + floor model C++ core: edge scan, NEON 4:1 downscale (2026-11-05) | floor model 11.4 → 5.2 ms/call offline (same result), live in its thread ~12 → ~10 ms; main loop unaffected (own thread). Daylight ~8.5 / p95 14 ms, CPU ~158 %; evening, gain 16: ~13 / p95 24 ms, ego 9–10 ms, CPU ~180 % |
| + ego-motion shortcuts (gray, sharpness, `goodFeaturesToTrack`) + room map C++ (2026-11-05) | ego offline 5.5 → 4.2 ms/frame, p95 7–11 → 6.1 ms (feature refill 7.8 → 2.9 ms); room map 2.0 → 0.7 ms/observation; both bit-identical. Live (evening, lying): mean 6.5 ms, p95 11.4 ms, ego 5.2 ms, CPU ~158 % |
| pipeline stopped | service CPU ~4 % |

Largest cost: optical flow (`ego.lk_*`). The Python binding does not accept prebuilt LK pyramids (also OpenCV 5); the C++ core (`xgonav/native`) builds one pyramid per frame and reuses it.

## Firmware behaviour

| Item | Value |
|---|---|
| Camera heading | `vis_yaw` drifts during forward drive: depth differences leak into the rotation model (2 m straight: −8.9° / +3°; left vs right image half −0.1° / −7.9°). Analysis: `tools/ego_debug.py` |
| IMU heading | firmware integrates yaw only at \|rate\| > 2°/s; 60 s balancing idle: IMU +2.5°, image −2.6°. Firmware heading hold (reg 0x0A) and `force_yaw` cannot see this drift. xgonav uses wheel heading (`odom.yaw_source`) |
| Turn overshoot | after stop command 10–17° further; commands within u8 128 ± 6 ignored. `tools/nudge.py` turns in two stages |
| Reverse | much weaker than forward (u8 109: 4 cm vs u8 147: 86 cm in 2 s) → `reverse_gain = 2.0` |
| Balancing drift | slow drift in position and heading |
| Firmware units | 1 unit ≈ 1 m/s assumed (`vx_limit`), map fallback only |
| Stand-up | Rider rolls ~20 cm forward |
| Rolling while turning | R-1.1.6 integrates the tilt gyros only; balancing with ~2° roll / ~1° pitch, turning leaks into them → pitch estimate +2–3° (left) / −5–9° (right) → rolled back on left turns, forward on right turns (up to 0.3 m/s, 20–40 cm per 3 s turn). Firmware patch 0005 (flashed 2026-11-05) removes the rotation about the vertical from pitch and roll rate. Turn test (3 s on the spot, centre displacement): 0.15 (~37 °/s) −4 / +9 cm; 0.30 (~85 °/s) +6 cm left, +26 cm right (first pitch-only version +16 / +40 cm). Remaining at fast turns: pitch estimate sinks 1–3° in both directions, consistent with centripetal acceleration on the accelerometer (IMU ~11 cm from the turning axis) plus ~0.3 % gyro cross-coupling. Patch 0006 (flashed 2026-11-05 19:28, holds the accelerometer angles while |yaw rate| > 25 °/s) changed nothing measurable: 0.15 −1 / +11 cm, 0.30 +11 / +27 cm (left / right) – centripetal acceleration is not the main cause; right turns still push forward (cause open: per-wheel motor/speed asymmetry or gyro cross-coupling, needs firmware trace). Turning also lags (~0.3 s, 10–17° overshoot); `sim_run.py --dyn` models lag, dead band and a short roll-back |

## Perception

| Item | Value |
|---|---|
| Floor model | needs fairly uniform floor; patterned carpet / hard tile joints → phantom obstacles. Laminate OK with `edge_weight 0.4`, `min_obst_rows 6` (0 false blocks in 40 raw images). Fails on some floors / at desk (open) |
| Follow false alarms | replay `raw_follow8`: path blocked in ~49 % of control steps without real obstacle |
| `kanten` remaining errors (2026-11-05, 36 frames reviewed) | reflections of furniture in glossy floor put the boundary 10–20 cm too close; diffuse light pool edge in a dark room (follow9 304) seen as obstacle; thin chair/table legs invisible (use detector `classes`) |
| Carpet | the carpet edge is a real obstacle (Rider cannot climb it); `kanten` detects it |
| Path check radius | `LocalMap` radius 0.18 m: a wall 0.16 m beside the path blocks driving along it (follow11 kitchen), although the Rider passed there without contact. Points already inside the radius only block when the path gets closer (escape rule) |
| Monocular depth (YOLO26n-depth, offline test) | ~0.7 s/frame on CM4 at 320 px (estimated), sees carpet as flat, fooled by glossy floor, no wall/floor separation beyond ~0.7 m at 12.8 cm camera height |
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
