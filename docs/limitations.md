# Limitations and measured facts

Single device: RiderPi, CM4 2 GB, kernel 6.6.74+rpt-rpi-v8, OV5647 wide-angle, firmware R-1.1.3 → R-1.1.6 (+ patches 0001–0013 since 2026-11-07).

## Hardware test status

| Item | Status |
|---|---|
| Driver patches 0001/0002, IMU ~49 Hz, kernel watchdog | tested |
| Wheel encoders (0x0d/0x0c), odometry scale | tested (tape measure, 215 cm) |
| Wheel heading vs IMU | 2026-11-05, 184 s / 15 m / 25 turns: slope 1.000, 3° apart at the end (track 0.090 m fits) |
| Camera geometry, lens shading | measured |
| Manual driving (web UI) | tested |
| `follow` | tested 2026-11-02 (runs 3–8, `max_v` 0.10–0.15); works, path check blocked in some runs |
| `explore` | frontier mapping, three recorded RiderPi runs 2026-11-06, see below. Map not yet usable: ends too early |
| `home` | 2026-11-05, 3 tries: did not reach the start. Stop/turn/start cycles and changing plans (45 s without progress), then stuck 1.3 m before the start at a gap the path check considers too narrow; afterwards changed (path keeping, `LocalMap` obstacles in planning, curve driving), **not retested** |
| `approach` | **not tested** |
| `pet` | simulator and unit tests only, **not run on the Rider**. Approach thresholds not tuned on real footage; cats and dogs from 12.8 cm camera height untested. A person standing < 1.9 m has no head in the image, so "kneeling" is inferred from the box top. Reverse is blind. Auto stand-up relies on IMU, wheels, `/proc/XGORider/state` and an empty image; the lying camera view is limited, floor or steps in front are not checked. A direct `echo 254 > action` without `POST /stop` is not noticed |
| FloorNet | shadow mode only, makes no decisions; preset goal against hand lines missed ([floornet.md](floornet.md)) |
| IMU sync while balancing | not verified |
| Patch 0003 (`force_yaw`) | not tested |
| Room map | used by `home` and `explore`; drifts (no markers); glossy floors mirror window sills and furniture edges as obstacles at d = D·h_cam/(h_edge + h_cam), RiderPi suppresses them with `confirm_dr_m` (only after the Rider moved); rug edges stay obstacles; loading assumes the Rider stands at the start point. The boundary row is entered as reported (no row correction since 2026-11-11); its error at a wall base is not measured |

## System software

| Item | Value |
|---|---|
| Kernel | 6.6.74, pinned (`deploy/apt-pin-kernel`). `apt upgrade` 2026-11-05 aborted at 6.12.115 (`wm8960-soundcard` does not build); rolled back, reboot verified |
| Python packages of the service | user `aeinstein`: OpenCV 4.10, numpy 1.26 from `~/.local`; root sees system 4.6, 1.24. The C++ core links system OpenCV 4.6 |
| libcamera | 0.5.2 since 2026-11-05: via xgonav 24.0 fps, loop 8.9 / p95 19.3 ms (lying, evening). Image quality not compared with 0.4 |
| `pykms` | pip package `rpi-kms` in `~/.local` broke with the new `libkms++` (undefined symbol, service crash loop); moved aside, system `python3-kms++` is used |

## Camera

| Item | Value |
|---|---|
| HFOV | 91.8° (f ≈ 310 px @ 640 px, from turns vs IMU heading 2026-11-07; the earlier 88.5° made image angles ~5 % too small); default and simulator use it. Camera height re-measured while balancing (`cam_h_u8`); the tape check at 0.48 / 0.98 m (2026-11-07) showed f·h 7.9 % too large and was corrected by height × 0.921 until 2026-11-10; the floor grid then confirmed the measured height, factor dropped, floor distances with it not driven yet |
| Distortion | ≤ 4 px in x 40–571 (corners not measured) |
| Height | 128 mm at body height 85 mm (`mount_offset_mm = 43`) |
| Tilt | 1.39° at IMU pitch −2.7° (R-1.1.3) |
| Lens shading | edge: blue/green to ~20 %, red to 1–9 % (from ~250 px off centre) |
| Measurable floor | middle ~6/8 of width from 18–25 cm; outer eighths and lower corners blind |
| Low light | 4 ms at 15–50 lux: gain 8 maxed, mean brightness 3/255 (auto 33 ms: 20–30/255); RiderPi config: 20 ms, gain 16 |
| Analogue gain range | driver register 16..1023 = 1..63.94×; libcamera `camera_controls` shows 1..16 before start, 1..63.94 after |
| Gain above 8× (measured 2026-11-06, 5 ms, one scene, 16 frames, green raw) | signal and temporal noise both scale with gain up to 63.9×; SNR stays ~4–5 from 8× on, so only brighter, and saturates sooner (20 ms, daylight: 16 % of pixels at 16×, 45 % at 63.9×) |
| Missing raw codes | exactly 16× every 2nd code, exactly 32× every 4th; 15×, 17×, 24×, 31×, 33×, 48× none; cause unknown |
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
| Firmware units | 1 unit ≈ 1.8 m/s at command 0.12–0.24 (`vx_limit`, 2026-11-05) |
| Stand-up | Rider rolls ~20 cm forward |
| Rolling while turning | R-1.1.6 integrates the tilt gyros only; balancing with ~2° roll / ~1° pitch, turning leaks into them → pitch estimate +2–3° (left) / −5–9° (right) → rolled back on left turns, forward on right turns (up to 0.3 m/s, 20–40 cm per 3 s turn). Firmware patch 0005 (flashed 2026-11-05) removes the rotation about the vertical from pitch and roll rate. Turn test (3 s on the spot, centre displacement): 0.15 (~37 °/s) −4 / +9 cm; 0.30 (~85 °/s) +6 cm left, +26 cm right (first pitch-only version +16 / +40 cm). Remaining at fast turns: pitch estimate sinks 1–3° in both directions, consistent with centripetal acceleration on the accelerometer (IMU ~11 cm from the turning axis) plus ~0.3 % gyro cross-coupling. Patch 0006 (flashed 2026-11-05 19:28, holds the accelerometer angles while |yaw rate| > 25 °/s) changed nothing measurable: 0.15 −1 / +11 cm, 0.30 +11 / +27 cm (left / right) – centripetal acceleration is not the main cause; right turns still push forward. Register 0x0e trace 2026-11-05 20:36: stock switches position hold off while turning, only the speed loop brakes; a small forward speed asks for lean-back, measured pitch stays ~1° behind the target, balance output keeps pushing forward (origin of that 1° not fully resolved). Patch 0008 (flashed) holds the wheel centre also while turning: 0.15 −0.2 / +1.8 cm, 0.30 +2.0 / +2.3 cm (left / right) against −3.6 / +9.5 and +15 / +29 cm without; rate unchanged; patch 0010 (flashed 2026-11-05) makes it the default. Turning also lags (~0.3 s, 10–17° overshoot); `sim_run.py --dyn` models lag, dead band and a short roll-back |

## Perception

| Item | Value |
|---|---|
| Floor model | needs fairly uniform floor; patterned carpet / hard tile joints → phantom obstacles. Laminate OK with `edge_weight 0.4`, `min_obst_rows 6` (0 false blocks in 40 raw images). Fails on some floors / at desk (open) |
| Follow false alarms | replay `raw_follow8`: path blocked in ~49 % of control steps without real obstacle |
| `kanten` remaining errors (2026-11-05, 36 frames reviewed) | reflections of furniture in glossy floor put the boundary 10–20 cm too close; diffuse light pool edge in a dark room (follow9 304) seen as obstacle; thin chair/table legs invisible (use detector `classes`) |
| Carpet | the carpet edge is a real obstacle (Rider cannot climb it); `kanten` detects it |
| Path check radius | `LocalMap` radius 0.18 m: a wall 0.16 m beside the path blocks driving along it (follow11 kitchen), although the Rider passed there without contact. Points already inside the radius only block when the path gets closer (escape rule). In a tight spot of `home`/`explore`/`patrol` the radius is `map.radius_tight_m` 0.08 m: 2.2 cm beyond the Rider's half width, **not driven** |
| Narrow gaps | planner and `explore` take gaps from 25 cm when there is no other way (`home.squeeze_m` 0.09, tight-spot steering). Simulator only (scene `spalt`, collision radius 7 cm instead of 12 cm); the 5 cm map sometimes draws such a gap too narrow and it is skipped. The Rider rolls back on every start and overshoots turns by ~15°: turning in place in front of a gap is untested. More goals at the end of `explore` (holes in mapped walls look like gaps) |
| Monocular depth (YOLO26n-depth, offline test) | ~0.7 s/frame on CM4 at 320 px (estimated), sees carpet as flat, fooled by glossy floor, no wall/floor separation beyond ~0.7 m at 12.8 cm camera height |
| Without IMU / sync | slow pitch unobservable: 1° ≈ 40 % distance error at 2 m; size fusion compensates. Status `pitch_src` shows source |
| Follow distance (sim) | ~1.5 m without / ~1.4 m with IMU at target 1.2 m (deadband + lag) |
| Explore (sim) | frontier version: 0 collisions in 10 runs to the end; an earlier version once hit a box corner and wheel slip then ruined the pose (1.7 m). Contact is not detected |
| Ego-motion | spikes up to ~80 ms per frame |
| Without free space (`freespace.enabled = false`) | only `idle`, `manual`, `follow`; `explore`/`approach` → HTTP 409 |

## Other

- Web UI has no authentication.
- Room map drifts (no markers).
- Web attitude model: legs show the last written `height`/`roll` setpoints, not measured positions (firmware does not report them). State 254 (Absetzen) shows legs at the firmware minimum (45 mm extension) as an assumption. Other firmware moves (limp legs, leveling, stand-up, falls) are not shown; unknown after driver load (assumes 128).
- Body height and camera geometry: xgonav reads `/proc/XGORider/height` every 0.2 s and follows new writes, also from the web (before 2026-11-07 it ignored them: drive `unten3` ran at camera height 151 mm while the geometry assumed 128 mm, distances ~18 % too far). While the Rider is down the firmware zeroes its height every tick and stands up at its centre 128 (≈ 90 mm); `/proc` still shows the old value, so xgonav assumes 90 mm after each stand-up until the next write (behaviors send their height again). It is the setpoint, not the leg position (legs need ~1 s). A web write of the same value as before the stand-up is not detected.
- Body height limited to u8 205 (web UI, API, xgonav; provisional). The firmware drives only 55..200 (`main.cpp`, range seen in training), so 201..205 act as 200 (≈ 107 mm). Setpoint 235 on 2026-11-06: firmware stop, state 252 (not tilt, fall, low battery or servo loss; IMU, control gap > 30 ms or invalid control output; cause open, single event). 205 held ~2 s, 225 ~1 s. Firmware itself accepts 0..255; direct writes to `/proc/XGORider/height` are not limited.
- Leg calibration buttons removed (saving with slack legs overwrote flash zero points, 2026-11-02).
- Battery: driver powers off below 8 % (`low_batt_level`); full image processing draws more than the charging dock supplies.

## `explore` on the RiderPi (2026-11-06)

| Item | Finding |
|---|---|
| **No cliff protection** | Run 1 nearly drove down a step, stopped by hand about 50 cm before it. The frontier goal lay ~20 cm behind the step, the map had it as free. The floor model saw the step only from about 1 m. Block stairs physically; do not run unattended |
| Flat objects | A yellow tape patch and the edge of a chair mat on the floor count as obstacles |
| Stand-up roll | The Rider rolled ~24 cm forward (one run); the start point is set after that, so it is not where the Rider lay |
| Early end | Runs 2 and 3 reported "Karte fertig" with open areas left: occupied specks plus `inflate_m` cut the map even on driven ground (run 2), last open spot 0.49 m from an old goal inside the 0.5 m avoid radius (run 3). Changed afterwards (driven ground, `avoid_r_m` 0.3), **not driven since** |
| Timing | Control step grew during the run (8 → 46 ms mean, 27 ms CPU), raw recording running; `LocalMap.confirmed_points` compares every point with every point (up to 2900 points). Result now cached between floor updates, the planner's copy runs in its thread. Run 4, same recording: control step 8–12 ms mean, main loop median 19 ms, p95 49 ms (before 33 and 97 ms). Planner median 215 ms, peak 761 ms |
| Heading | Run 6, 32 m, photo with A4 scale: pose 75 cm wrong with wheel heading (slip while turning, wheels and IMU differ by up to 37°). Same run re-integrated with IMU heading: about 6 cm, walls single and straight. One run, strip taken as start. `grid.yaw_source = "imu"` since then, not driven yet |
| `patrol` | since 2026-11-06, simulator only, **not driven**. No alarm yet |
| Drift with IMU heading | Run 9, 11.9 m, measured at the end: 195 mm to the left, heading looked right (pose said 4 cm, 7°). Larger than the 6 cm the re-integration of run 6 suggested |
| Way back | Run 4 ended 1.7 m from the start: the direct way led along a chair and a chair mat. Abort now only after `give_up_s` 20 s without approach. Run 5 ended 0.89 m from the start with "kein Weg" while driving freely: the room map alone had a straight way, so the local map closed it. `plan()` now falls back without it, not driven since |
| Contact | Only inferred from stationary wheels under command, see `docs/mapping.md`; no sensor |
