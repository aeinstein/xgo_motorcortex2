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
| IMU heading | firmware integrates yaw only at \|rate\| > 2°/s; 60 s balancing idle: IMU +2.5°, image −2.6°. Firmware heading hold (reg 0x0A) and `force_yaw` cannot see this drift. Gyro-z bias is measured once at boot: 2026-11-05 45 min after boot 0.48 °/s off (slope against wheels 0.998); 75 s turn test ended IMU −3.5°, wheels +27.7°, seen ~35° left. Patch 0009 (flashed 2026-11-05) tracks the bias and lowers the dead band to 0.3 °/s: 60 s drive test IMU +13.4°, wheels +15.4°. One stop from ~50 cm/s reverse twisted the Rider ~9° right within 0.5 s; the next run stopped within 2° in both directions (not reproduced, cause unknown). xgonav uses wheel heading (`odom.yaw_source`) |
| Turn overshoot | after stop command 10–17° further; commands within u8 128 ± 6 ignored. `tools/nudge.py` turns in two stages |
| Reverse | 2026-11-02 much weaker than forward (u8 109: 4 cm vs u8 147: 86 cm in 2 s). 2026-11-05 with patches up to 0010: symmetric (sent 0.12 forward 0.70 m in 3 s, sent 0.24 reverse 1.30 m in 3 s). `reverse_gain = 2.0` always, `manual_forward_gain = 2.0` manual only. Sent 0.24, 3 s (2026-11-05 21:50): forward 1.09 m, peak 0.49 m/s, reverse 1.39 m, peak 0.53 m/s; same top speed, forward needs ~1 s longer to reach it (cause not examined); heading within 1° forward, 3° reverse; stops in ~0.6 s, pitch ±8°. Sent 0.30 (full stick) not driven yet. Autonomous modes stay at factor 1 forward: simulator (`--dyn`, ~54 cm/s, 8 runs each) 1 collision and home arrived 1 of 4, factor 1: 0 collisions, 4 of 4; free-space thresholds would need retuning |
| Balancing drift | slow drift in position and heading |
| Roll while standing | +2.1 to +3.2° (+ = right side down), stays with the Rider when it turns (floor share ~0.6° over 173° heading, 8 rest phases); lying +0.4 to +1.0° (once −0.1°). Height test 2026-11-05: 75 mm 2.22 / 2.55°, 115 mm 2.21 / 2.24°, standard 2.73 / 2.31°; 8 ticks of leg-zero error would give 0.77° between the heights, so it explains ~0.5° at most. Rest: the body is really tilted (user: straight at roll −2°, IMU then +0.49°, close to the lying value, so ~0.5° is sensor share). Leveling test 2026-11-05: leveling on reached −0.04° in 1.1 s without oscillation; after one turn and with leveling off, same leg command, roll stayed +0.13° (before +2.46°), so it is a state, not a fixed length difference. Hysteresis test 2026-11-05: with roll command 0 the body stays at +2.10° after stand-up, +0.72° after −3°, +3.35° after +3°: dead zone or play in the leg servos, ~2.6° wide (servo dead zone vs mechanical play not separated). A 3 s leveling pulse left +0.51°; afterwards +0.54° after a left turn, +0.60° after a right turn, +0.82° after 2 s forward, +1.16° after 2 s reverse (creeps with driving, long-term not tested). `robot.level_after_standup` runs one such pulse after each stand-up, the web button „Neu ausrichten“ (`POST /level`) another. Firmware leveling (0x61) is off by default |
| Firmware units | 1 unit ≈ 1 m/s assumed (`vx_limit`), map fallback only |
| Stand-up | Rider rolls ~20 cm forward |
| Rolling while turning | R-1.1.6 integrates the tilt gyros only; balancing with ~2° roll / ~1° pitch, turning leaks into them → pitch estimate +2–3° (left) / −5–9° (right) → rolled back on left turns, forward on right turns (up to 0.3 m/s, 20–40 cm per 3 s turn). Firmware patch 0005 (flashed 2026-11-05) removes the rotation about the vertical from pitch and roll rate. Turn test (3 s on the spot, centre displacement): 0.15 (~37 °/s) −4 / +9 cm; 0.30 (~85 °/s) +6 cm left, +26 cm right (first pitch-only version +16 / +40 cm). Remaining at fast turns: pitch estimate sinks 1–3° in both directions, consistent with centripetal acceleration on the accelerometer (IMU ~11 cm from the turning axis) plus ~0.3 % gyro cross-coupling. Patch 0006 (flashed 2026-11-05 19:28, holds the accelerometer angles while |yaw rate| > 25 °/s) changed nothing measurable: 0.15 −1 / +11 cm, 0.30 +11 / +27 cm (left / right) – centripetal acceleration is not the main cause; right turns still push forward. Register 0x0e trace 2026-11-05 20:36: stock switches position hold off while turning, only the speed loop brakes; a small forward speed asks for lean-back, measured pitch stays ~1° behind the target, balance output keeps pushing forward (origin of that 1° not fully resolved). Patch 0008 (flashed) holds the wheel centre also while turning: 0.15 −0.2 / +1.8 cm, 0.30 +2.0 / +2.3 cm (left / right) against −3.6 / +9.5 and +15 / +29 cm without; rate unchanged; patch 0010 (flashed 2026-11-05) makes it the default. Turning also lags (~0.3 s, 10–17° overshoot); `sim_run.py --dyn` models lag, dead band and a short roll-back |

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
