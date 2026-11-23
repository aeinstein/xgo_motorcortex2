# Architecture

## Per-frame data flow (`App._step()`, `xgonav/app.py`)

```
camera.FrameSource ─► egomotion.EgoMotion ─► tracker.Tracker ◄── detector.DetectorWorker (thread)
                            │                       │
                            ├─► imu.imu_frame (IMU at frame time, one value per frame → pitch_dev, roll, heading)
                            ├─► freespace.FreeSpaceWorker: FloorNet (thread) ─► localmap.LocalMap
                            ├─► wheels (odometry) ─► gridmap.GridMap (thread, display only)
                            ▼
              behaviors.World ─► Behavior.step() ─► behaviors.safety() ─► App._arc_check() ─► robot.Robot.drive()
```

## Clock rates

| Loop | Rate | Does |
|---|---|---|
| Image loop | camera fps (`camera.fps` = 24) | ego-motion, tracker update, free space, snapshot |
| Control step | `control_hz` = 24 Hz (sim: 20), at the frame nearest to the beat (threshold `1/control_hz − 0.5/fps`), so at most the frame rate: RiderPi every frame. Until 2026-11-17 `control_hz` 20 gave only every 2nd frame (~12 Hz) on the Rider: the 50 ms threshold was reached only at the second frame (41.7 ms apart). 24 Hz **not installed, not driven** | build `World`, run behaviour, safety, `drive()` |
| Detector | `detector.max_hz` (RiderPi: 5 Hz), only in `follow`/`approach`/`pet` | YOLO on sharp frames, own thread; recognition only, never obstacles |
| IMU / wheels | firmware stream `imu.stream_hz` (100 Hz); without stream 50 Hz polling of 0x0d (`imu.imu_hz`) | kernel thread, read from `/proc/XGORider/imu_log`, `wheels_log` |

## Rules

- Only the newest frame counts. `FrameSource.get()` drops old frames. No queues.
- Ego-motion is removed everywhere: per frame a homography `A = K·Rᵀ·K⁻¹` (rotation model: yaw/pitch/roll + expansion), applied to all tracks; delayed detections are shifted with `EgoMotion.transform_between(a, b)`; horizon line and ground-point distance use `pitch_dev`.
- Display reads snapshots only: `App.snapshot` is replaced per frame (never mutated), signalled via `snap_cond`. Web, LCD and recorders (`--record`, `--record-raw`) read from it; `render()` draws the overlay on demand.

## Modules

| Module | Role |
|---|---|
| `camera.py` | `PiCameraSource`: libcamera directly via `native/camfast.cpp` (`libxgocam.so`, undistorts straight from the DMA buffer, pixels never pass through Python), fallback picamera2; `OpenCVSource` (index/file) |
| `egomotion.py` | LK optical flow + RANSAC → rotation model, sharpness, time-to-contact; C++ core `native/egofast.cpp` (ctypes, optional) |
| `imu.py` | `ProcImu`, `ImuSync`: sign/time offset by correlation, only when not fixed in `[imu]` (default: fixed, simulator: auto); `imu_frame`: pitch, roll, heading at frame time + offset, used by geometry, floor model, maps, tilt, overlay and web (`Snapshot.imu`); image pitch only as comparison (`Snapshot.nickrate`) |
| `detector.py` | `DetectorWorker`: YOLO via NCNN (`YoloNcnn`) or onnxruntime (`YoloOnnx`), fallback model; disabled if none is usable |
| `tracker.py` | Tracks, ego-motion compensation, latency compensation, Kalman distance/velocity |
| `geometry.py` | Camera geometry, ground-plane distance |
| `freespace.py` | FloorNet boundary rows → free fraction per column and sectors, rolled by the camera roll; `FreeSpaceWorker` runs the network in its own thread. No fallback: without the network the Rider is blind (`safety()` stops) |
| `jpeg.py` | JPEG for the browser view in the Pi hardware encoder (`native/jpegfast.cpp`, `/dev/video31`, self-check at first frame), fallback `cv2.imencode`; raw recording (`rawrec.py`) uses its own instance at q95 (fallback libjpeg q92), encoder per image in `meta.jsonl` |
| `fastlib.py` | loads optional C++ core `native/libxgofast.so` and camera library `native/libxgocam.so` (ctypes), ABI check per function group |
| `navfast.py` | ctypes binding of `native/navfast.cpp`: A* + path shortening, wavefront, frontier group scoring, local map (move, enter, confirm, arc check), wall distance, path-follower target; used by `planner.py`, `frontier.py`, `localmap.py`, `gridmap.py`, `behaviors.py`, each keeps an equivalent Python path |
| `floornet.py` | self-trained floor network in the C++ core (`native/floornet.cpp`), the only source of the floor boundary; `ZeilenGlaetter` smooths it over time ([floornet.md](floornet.md)). Simulator: `sim.SimFloorNet` (ground truth) |
| `localmap.py` | Short-term obstacle memory in robot frame (`map.max_age_s` = 4 s); move, enter, confirm and arc check optionally in C++ (`native/navfast.cpp`) |
| `wheels.py` | Wheel odometry from `wheels_log` |
| `gridmap.py` | Room occupancy grid in world frame, start point, save/load; entering an observation (rays, free rays, hits) optionally in C++ (`native/gridfast.cpp`) |
| `startbild.py` | start image check: anchor image at the start, measurement of the pose against it at the end of `home`, own thread |
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
| Floor boundary | `floornet.FloorNet` | `sim.SimFloorNet` (ground truth of the rendered frame; `sim.Phantoms` adds false alarms) |

`--robot` default: `proc` if `camera.source = "picam"`, else `sim`. LCD and emotions only with `proc`.

## Safety layers (in order, last word wins)

| Layer | Action |
|---|---|
| `behaviors.safety()` | not balancing → stop; blind (no FloorNet) → stop; centre free space < `blocked_free`·0.6 → no forward; < `slow_free` → scale v (≥ 0.25); TTC < `ttc_stop_s` (0.8 s) → no forward; free space and TTC skipped for a push of the path follower (length limited, see `push_*`) |
| `App._arc_check()` | planned arc (forward and reverse) checked against local map; speed = max(command, wheels over 0.2 s), look-ahead ≥ braking distance + 0.15 m (min. 1.2 s); free length < `0.6·v + 0.08 + 0.8·max(0, v − 0.3)` → v scaled, or v = 0 without ramp if the wheels are > 0.05 m/s faster than the scaled value (Rider follows smaller commands only after ~1–2 s, brakes on 0 in ~0.3 s; 2026-11-23, simulator only); a push of the path follower only checks keep-out zones |
| `Robot` | clamp to `max_v`/`max_w`, ramp (`accel_v`, `accel_w`; skipped for a full brake from the arc check), keepalive 0.3 s, userspace watchdog `stall_timeout_s` 0.5 s |
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
