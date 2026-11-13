# Architecture

## Per-frame data flow (`App._step()`, `xgonav/app.py`)

```
camera.FrameSource ─► egomotion.EgoMotion ─► tracker.Tracker ◄── detector.DetectorWorker (thread)
                            │                       │
                            ├─► imu.ImuSync (complementary filter IMU + image → pitch_dev)
                            ├─► freespace.FreeSpaceEstimator (opt. thread) ─► localmap.LocalMap
                            ├─► wheels (odometry) ─► gridmap.GridMap (thread, display only)
                            ▼
              behaviors.World ─► Behavior.step() ─► behaviors.safety() ─► App._arc_check() ─► robot.Robot.drive()
```

## Clock rates

| Loop | Rate | Does |
|---|---|---|
| Image loop | camera fps (RiderPi config: 25) | ego-motion, tracker update, free space, snapshot |
| Control step | `control_hz` = 20 Hz | build `World`, run behaviour, safety, `drive()` |
| Detector | `detector.max_hz` (RiderPi: 5 Hz), only in `follow`/`approach`/`pet` | YOLO on sharp frames, own thread; recognition only, never obstacles |
| IMU / wheels | 50 Hz (`imu.imu_hz`) / `wheel_hz` | kernel thread, read from `/proc/XGORider/imu_log`, `wheels_log` |

## Rules

- Only the newest frame counts. `FrameSource.get()` drops old frames. No queues.
- Ego-motion is removed everywhere: per frame a homography `A = K·Rᵀ·K⁻¹` (rotation model: yaw/pitch/roll + expansion), applied to all tracks; delayed detections are shifted with `EgoMotion.transform_between(a, b)`; horizon line and ground-point distance use `pitch_dev`.
- Display reads snapshots only: `App.snapshot` is replaced per frame (never mutated), signalled via `snap_cond`. Web, LCD and recorders (`--record`, `--record-raw`) read from it; `render()` draws the overlay on demand.

## Modules

| Module | Role |
|---|---|
| `camera.py` | `PiCameraSource` (picamera2), `OpenCVSource` (index/file) |
| `egomotion.py` | LK optical flow + RANSAC → rotation model, sharpness, time-to-contact; C++ core `native/egofast.cpp` (ctypes, optional) |
| `imu.py` | `ProcImu`, `ImuSync`: sign/time offset by correlation, complementary filter |
| `detector.py` | `DetectorWorker`: YOLO via NCNN (`YoloNcnn`) or onnxruntime (`YoloOnnx`), fallback model; disabled if none is usable |
| `tracker.py` | Tracks, ego-motion compensation, latency compensation, Kalman distance/velocity |
| `geometry.py` | Camera geometry, ground-plane distance |
| `freespace.py`, `shading.py` | Floor colour model → free/blocked per image column; lens-shading correction; edge scan and downscale optionally in C++ (`native/floorfast.cpp`) |
| `fastlib.py` | loads optional C++ core `native/libxgofast.so` (ctypes), ABI check per function group |
| `navfast.py` | ctypes binding of `native/navfast.cpp`: A* + path shortening, wavefront, frontier group scoring, local map (move, enter, confirm, arc check), wall distance, path-follower target; used by `planner.py`, `frontier.py`, `localmap.py`, `gridmap.py`, `behaviors.py`, each keeps an equivalent Python path |
| `floornet.py` | self-trained floor network in the C++ core (`native/floornet.cpp`); runs in the freespace thread: shadow mode (no decisions) or, with `primary = "net"`, alone instead of `method` ([floornet.md](floornet.md)) |
| `localmap.py` | Short-term obstacle memory in robot frame (`map.max_age_s` = 4 s); move, enter, confirm and arc check optionally in C++ (`native/navfast.cpp`) |
| `wheels.py` | Wheel odometry from `wheels_log` |
| `gridmap.py` | Room occupancy grid in world frame, start point, save/load; entering an observation (rays, free rays, hits) optionally in C++ (`native/gridfast.cpp`) |
| `planner.py` | A* on the room map for `home`, own thread (`PlannerWorker`); A* and shortening optionally in C++ (`native/navfast.cpp`) |
| `behaviors.py` | `idle`, `manual`, `follow`, `explore` (`MapExplore`, reactive `Explore` without room map), `approach`, `home`; `PathFollower`, `safety()` |
| `pet.py` | mode `pet`: `PetBrain` (logic only), `Pet` (behavior); reacts to people, cats, dogs. Control step only, nothing per frame |
| `frontier.py` | mapping targets for `explore`, runs in the planner thread; loops optionally in C++ (`native/navfast.cpp`) |
| `robot.py` | `ProcRobot` (`/proc/XGORider`), `SimRobot`; ramp, keepalive, watchdog |
| `web.py` | Local HTTP/WebSocket API (`127.0.0.1:8088`) |
| `lcd.py` | Overlay to `/tmp/lcd0` (Pi only, lazy import) |
| `emotion.py` | Automatic emotions on the display |
| `affinity.py` | CPU pinning |
| `sim.py` | Simulator (`SimWorld`, `FakeDetector`, `SimImu`) |

## Swappable backends

| Role | Real | Simulator/test |
|---|---|---|
| Image source | `camera.PiCameraSource`, `camera.OpenCVSource` | `sim.SimWorld` |
| Robot | `robot.ProcRobot` | `robot.SimRobot` |
| Detector | `detector.DetectorWorker` | `sim.FakeDetector` (ground truth, 180 ms latency, 5 Hz) |
| IMU | `imu.ProcImu` | `sim.SimImu` |

`--robot` default: `proc` if `camera.source = "picam"`, else `sim`. LCD and emotions only with `proc`.

## Safety layers (in order, last word wins)

| Layer | Action |
|---|---|
| `behaviors.safety()` | not balancing → stop; centre free space < `blocked_free`·0.6 → no forward; < `slow_free` → scale v (≥ 0.25); TTC < `ttc_stop_s` (0.8 s) → no forward |
| `App._arc_check()` | planned arc (forward and reverse) checked against local map |
| `Robot` | clamp to `max_v`/`max_w`, ramp (`accel_v`, `accel_w`), keepalive 0.3 s, userspace watchdog `stall_timeout_s` 0.5 s |
| Kernel | `settings/motion_timeout_ms` (xgonav sets `robot.kernel_watchdog_ms` = 1000) |

`manual` mode skips floor model and arc check; fall detection and watchdogs stay active.

## Robot convention

- `drive(v, w)`: normalised [-1, 1]; v > 0 forward, w > 0 turn left.
- `robot.norm_to_u8`: `u8 = 255/(2L)·(v + L)`, 128 = standstill.
- Firmware **holds** setpoints; stops must be written explicitly.
- `robot.reverse_gain` multiplies negative v before sending, `robot.manual_forward_gain` positive v in manual mode only (RiderPi: 2.0 each).
- `robot.speed_ctrl`: `App._speed_tick` (control step) compares wheel speed with the target, `robot.SpeedCtrl` integrates the difference, `Robot._send` adds it after the gains. Firmware speed loop is P only, so the Rider runs slower on carpet without it.

## CPU (RiderPi config, `[cpu]`)

| Thread | Cores |
|---|---|
| Main loop, camera, OpenCV workers | 0–1 |
| YOLO (NCNN: thread + OpenMP pool on 2–3; onnxruntime: caller on 2, pool on 3) | 2–3 |
| Free-space worker, grid worker | 2–3 |
| Web/LCD rendering, raw recording | 0–3 |
