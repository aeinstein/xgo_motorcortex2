# Odometry and maps

## Odometry (`xgonav/wheels.py`, `App._odom_ds`)

| Quantity | Source |
|---|---|
| Distance | wheel encoders (`/proc/XGORider/wheels_log`) if fresh (`odom.max_age_s` 0.3 s), else command × `robot.vx_limit` × dt |
| Heading (local map, safety, explore) | `odom.yaw_source`: `räder` (wheels, fallback camera) or `kamera`; camera heading is biased during forward drive |
| Heading (room map) | `grid.yaw_source`: `räder` (wheels, fallback camera) or `kamera` |

Encoders: 1024 ticks/rev, 60 mm wheel, track 90 mm, left = 11, right = 21, both positive forward.
int32 overflow and ESP32 restarts handled (`max_jump_ticks` 2048). Firmware R-1.1.3 has no encoders → command fallback.
Status JSON: `odom.src` = `räder` | `befehl`, `odom.m` = total distance.

## Short-term local map (`xgonav/localmap.py`)

| Item | Value |
|---|---|
| Frame | robot (x forward, y left, m) |
| Input | floor-boundary points from the free-space estimator |
| Motion | heading from ego-motion, distance from odometry |
| Lifetime | `map.max_age_s` = 4 s |
| Range | 2.5 m |
| Confirmation | point blocks only with ≥ `map.confirm` observations within `confirm_radius_m` (RiderPi: 3 within 0.08 m) |
| Use | `App._arc_check()`: each command checked as an arc, forward and reverse |
| Purpose | obstacles that left the ±31° (sim) / ±45° (RiderPi) field of view |

## Room map (`xgonav/gridmap.py`)

| Item | Value |
|---|---|
| Type | occupancy grid, log-odds |
| Frame | world, origin = start pose, x forward, y left at start |
| Size | 16 m × 16 m, 0.05 m cells, start in the centre |
| Rays | every 2nd of 160 floor-model columns |
| Free | up to boundary − 0.10 m; `l_free` −0.4; occupied cells reduced only ×0.5 |
| Occupied | at boundary, `l_occ` +1.2 |
| Clamp | −3.0 .. +3.5; shown as free/occupied beyond ±0.7 |
| Range | 0.15–2.5 m; "free to horizon" columns mark free up to 2.5 m only |
| Unknown | columns without measurable floor (lens shading) stay unknown |
| Thread | `GridWorker` (cores = `cpu.freespace_cpus`), pose at capture time passed along |
| Use | display; path planning for mode `home` (not for other modes) |
| Drift | no correction (no markers); drifts over time |

### Rendering (`/map.png`)

| Colour | Meaning |
|---|---|
| grey | unknown |
| light | free |
| dark | occupied |
| green line | path |
| cyan line | planned path (`home`) |
| blue circle + line | start point and start heading |
| red triangle | Rider pose |
| blue bar | 1 m scale |

Parameters: `px` 120..960 (default 480), `view` 1..16 m (default `grid.view_m` 6).
Reset: web „Zurücksetzen“ or `POST /map/reset` (empty map, pose 0, start 0).
Status JSON `grid`: `pose [x m, y m, yaw °]`, `start [x m, y m, yaw °]`, `zum_start_m`, `belegt_m2`, `frei_m2`, `updates`, `ms`, `verworfen`; top level `plan_ms`.

### Start point

| Item | Value |
|---|---|
| Default | map origin (pose at xgonav start) |
| Set | web „Start hier“ or `POST /map/start` → start = current pose (x, y, heading) |
| Reset | `/map/reset` → 0 |

### Persistence

| Item | Value |
|---|---|
| File | `grid.file` (npz: log-odds float16, resolution, size, start, driven path); empty = off |
| Save | every `grid.save_every_s` (60 s) if changed, in the `GridWorker` thread; on exit; web „Speichern“ / `POST /map/save` |
| Write | `<file>.tmp` + rename (atomic) |
| Load | at xgonav start if the file exists; resolution/size mismatch → ignored with warning |
| Pose after load | **= start point**. No relocalisation: place the Rider on the start point (same heading) before starting xgonav |

### Return to start (`--mode home`, `xgonav/planner.py`)

| Item | Value |
|---|---|
| Planner | A* (8-neighbour) on map cells, search area = pose, start, driven path + `home.margin_m` |
| Blocked | occupied cells (`> l_show`) inflated by `home.inflate_m` 0.18 m; cleared within that radius around pose and goal (drift) |
| Cost | 1 + `clear_cost` near obstacles + `unknown_cost` for unknown cells; line-of-sight shortcut only if not more expensive |
| Thread | `PlannerWorker` (cores = `cpu.freespace_cpus`); replan every `replan_s` 2 s and after 3 s without 5 cm progress |
| Path keeping | current path kept while free and ≤ `keep_factor` × new + 0.3 m; dropped after getting stuck |
| Local obstacles | confirmed `LocalMap` points within 1.5 m (what `_arc_check` brakes for) are blocked for planning (`use_localmap`) |
| Following | pure pursuit, lookahead 0.5 m, `home.kp_turn` 1.5/s; turn in place only above 75° error until below 25°, otherwise drive the curve at ≥ 30 % speed |
| Goal | within `arrive_m` 0.15 m, then turn to start heading (`align_heading`), status „am Start angekommen“, stands |
| Failure | `max_fail` 3 consecutive plans without path → stop, „kein Weg zum Start“ |
| Safety | `safety()` and `_arc_check()` apply as in `explore` |
| Requires | `freespace.enabled`, `grid.enabled`, running pipeline |
| Accuracy | limited by map drift: arrives at the *map* start; real error = pose error |

### Simulator quality (`tests/sim_run.py explore`)

Reported per run: `praezision_15cm` (occupied cells within 15 cm of a true wall/box), `abdeckung_10cm` (wall/box points ≤ 2 m from the driven path with an occupied cell within 10 cm), `pose_fehler_m`, `kurs_fehler_deg`.
Example 15 s explore: precision 0.62, coverage 0.23, pose error 0.10 m, heading error 6.2°.
