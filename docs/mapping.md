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
| Frame | robot (x forward, y left, m), origin = wheel axle; floor points shifted forward by `camera.cam_fwd_mm` |
| Input | floor-boundary points from the free-space estimator |
| Motion | distance from wheel odometry; heading per `map.yaw_source`: wheels (else ego-motion) or, RiderPi, the firmware heading from `imu_log` (same source as the room map) |
| Lifetime | `map.max_age_s` = 4 s |
| Range | 2.5 m |
| Confirmation | point blocks only with ≥ `map.confirm` observations within `confirm_radius_m` (RiderPi: 3 within 0.08 m) |
| Use | `App._arc_check()`: each command checked as an arc, forward and reverse |
| Purpose | obstacles that left the ±31° (sim) / ±45° (RiderPi) field of view |

## Room map (`xgonav/gridmap.py`)

| Item | Value |
|---|---|
| Type | occupancy grid, log-odds |
| Frame | world, origin = start pose (wheel axle), x forward, y left at start; rays start `camera.cam_fwd_mm` ahead of the pose |
| Size | 16 m × 16 m, 0.05 m cells, start in the centre |
| Rays | every 2nd of 160 floor-model columns |
| Free | up to boundary − (0.10 m + 0.05 × d²); `l_free` −0.4; occupied cells reduced only ×0.5 |
| Occupied | at boundary, `l_occ` +0.5: two hits until shown as occupied |
| Clamp | −3.0 .. +3.5; shown as free/occupied beyond ±0.7 |
| Range | hits 0.15–`max_range_m` 1.25 m, free rays to `free_range_m` 1.5 m, RiderPi 1.1 m (one image row at 2 m is ~40 cm; a step was seen as floor from > 1 m on 2026-11-06, simulator with 1.1: floor known 0.43–0.83, occupied precision 0.84–0.92); "free to horizon" columns mark free up to 1.5 m only |
| Row correction | `row_bias` (working rows, + = farther): default 1.5 from the simulator (boundary below the true one); RiderPi −1.0 (tape at 0.48 m, 2026-11-07: boundary ~1 row above the edge). Room map only, the local map uses the raw boundary. Wall base not checked yet |
| Pitch, pose | those of the captured image (`FreeSpace.pitch_dev`, `.pose`), not of the later frame the result arrives in |
| Filter | only while balancing and below `max_turn_dps` 40 °/s |
| Driven path | `driven_free_r_m` 0.10 m around the path counts as free |
| Unknown | columns without measurable floor (lens shading) stay unknown |
| Thread | `GridWorker` (cores = `cpu.freespace_cpus`), pose at capture time passed along |
| Use | display; path planning for modes `home` and `explore` |
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

Parameters: `px` 120..960 (default 480), `view` 1..16 m (default `grid.view_m` 6), `cx`, `cy` centre in map metres (default Rider pose).
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

### Route and patrol (`xgonav/route.py`, mode `patrol`)

| Item | Value |
|---|---|
| Map lock | set when `explore` ends or `patrol` starts, `POST /map/lock`; locked: observations and the driven path do not change the map, the pose still moves. `explore` refuses a locked map |
| Map id | `map_id` in the npz; a route file with another id is ignored |
| Route file | `<grid.file>.route.json`: points, `loop`, zones, written atomically together with the map |
| Point check | on known floor, outside `inflate_m`, reachable from the start, outside zones; otherwise the route is rejected with a message |
| No-go zones | polygons in map coordinates. Planner and frontier never enter them plus `inflate_m`, `_arc_check` stops an arc that would come within 0.18 m (moving away stays allowed). All modes except manual. They drift with the pose: **no replacement for a physical barrier** |
| Patrol | points in order, `pause_s` 1.5 s each; stuck: replan, then back up and skip the point; contact, `max_fail` plans without a way or `point_timeout_s` 90 s: skip. No point reached in a lap, or end of a non-loop route: to the start |
| Near enough | stuck within `home.near_m` 0.5 m of a target counts as arrived: the 0.18 m clearance can make a free target unreachable |
| Start | only standing and with user confirmation that the Rider is at the start point: the pose jumps to the start, which removes drift. Battery ≥ 40 %. Back at 30 %, set down at 15 % (action 254) |
| Status JSON | `patrol`: `zustand`, `punkt`, `von`, `runde`, `erreicht`, `ausgelassen`, `ende`; `grid`: `gesperrt`, `map_id`, `route`, `zonen` |

### Mapping a room (`--mode explore`, `xgonav/frontier.py`)

| Item | Value |
|---|---|
| Start | one full turn at `explore.scan_w` |
| Frontier | reachable free cell next to unknown; groups ≥ `min_cluster_m` 0.4 m. Reachability from the room map only, the local map only shapes the path |
| Choice | unknown area within `gain_radius_m` / (drive time + turn time + `base_s`). The goal stays until reached or gone, no switching by score |
| Arrival | stops `standoff_m` before the frontier, turns towards it, `settle_s`; no new goal within `avoid_r_m` 0.3 m of the spot |
| Driven ground | cells within `driven_free_r_m` of the driven path stay passable for the planner despite occupied specks and `inflate_m`; obstacles the local map sees now still block |
| Contact | command > 0.08 but < 2 cm movement in 2.5 s (normal start-up takes up to 1.8 s): treated as contact, back up and block the goal at once. Wheel slip hides it |
| Stuck | no 5 cm progress or 10° turn in 3 s: replan without the old path; again, or `goal_timeout_s`: back up 0.8 s, block the goal. No goal left: `wander_s` reactive driving, at most `max_wander` times, then stop |
| End | nothing reachable, < `min_growth_m2` new floor in `no_growth_s`, `max_s`, or `min_battery`. Then save and drive to the start (`return_home`) |
| Status JSON | `explore`: `zustand`, `stellen`, `gesperrt`, `befreit`, `ziel`, `ende` |
| Without room map | `explore` drives reactively (`Explore`), as `approach` does while searching |

Measuring: `tools/map_eval.py record` stores simulator observations, `eval` replays them through `GridMap` in seconds, with `--set grid.x=a,b` sweeps and `--rays` (boundary error per distance).

### Simulator quality (`tests/sim_run.py explore`)

Reported per run: `praezision_15cm` (occupied cells within 15 cm of a true wall/box), `abdeckung_10cm` (wall/box points ≤ 2 m from the driven path with an occupied cell within 10 cm), `pose_fehler_m`, `kurs_fehler_deg`.
Also `waende_10cm` (all walls), `boden_bekannt` (true floor known as free), `frei_aussen_m2` (free cells outside the room).

| 2026-11-05 (`--wheels --kanten --fs-thread --dyn --rider`) | Reactive, 90 s, before | Frontier, to „Karte fertig“ |
|---|---|---|
| Occupied cells ≤ 15 cm from truth | 0.46 | 0.95–0.97 |
| Walls ≤ 10 cm | 0.36 | 0.87–0.94 |
| Floor known | 0.67 (after fixes) | 0.89–0.92 |
| Free outside the room | ~9 m² | 0.13–0.37 m² |
| Duration, distance | never finished | 150–250 s, 27–44 m |
| Collisions | 0 of 8 | 0 of 6 (one room, two rooms) |

Largest single cause: distances were computed from the pitch of a later frame (threaded floor model). Boundary error at 0.5–0.75 m, p10/p90: −0.19/+0.17 m before, −0.11/−0.02 m after.
With 8 phantom spots (`--phantoms 8`, two rooms): 0 collisions in 4 runs, 3 complete, 1 ended early and reported an unreached spot.

2026-11-06, `free_range_m` 1.1, driven ground passable, `avoid_r_m` 0.3 (one room, two rooms, with and without 8 phantoms): 12 of 12 to „Karte fertig“, 0 collisions, 142–204 s, 24–36 m, floor known 0.78–0.88, walls 0.77–0.94, occupied precision 0.86–0.94, free outside 0.01–0.07 m².

Tried and dropped 2026-11-06: weak hits beyond `max_range_m` (up to 1.75–2.25 m, +0.1–0.2 per image). RiderPi recordings: obstacles in the map 3–13 s earlier (median), little added. Simulator (18 runs): occupied precision 0.64–0.84, and in the two-room scene 7 of 9 runs saw at most 0.69 of the floor, 5 of them about 0.40; 0 collisions.
