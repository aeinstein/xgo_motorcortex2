# Odometry and maps

## Odometry (`xgonav/wheels.py`, `App._odom_ds`)

| Quantity | Source |
|---|---|
| Distance | wheel encoders (`/proc/XGORider/wheels_log`) if fresh (`odom.max_age_s` 0.3 s), else command × `robot.vx_limit` × dt |
| Heading (local map, safety, explore) | `odom.yaw_source`: `räder` (wheels, fallback camera) or `kamera`; camera heading is biased during forward drive |
| Heading (room map) | `grid.yaw_source`: `räder` (wheels, fallback camera), `kamera` or, RiderPi, `imu` (firmware heading from `imu_log`, fallback `/proc` yaw, then wheels) |

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
| Confirmation | `confirm_dr_m` (RiderPi 0.2 m, ± `confirm_cells` 2): occupied only after hits on the same spot from two ranges ≥ 0.2 m apart. RiderPi 2026-11-08: the window sill mirrored in the tiles was entered at 0.56 m from the start and 0.82 m from 0.72 m further on (d = D·h_cam/(h_edge + h_cam)), i.e. a second wall. Simulator explore 120 s: walls 82 % → 73 %, precision 0.90 → 0.93, 0 collisions. Tried and dropped the same day: hits only up to 0.7 m (walls 25 %, explore ended early) and a half-resolution edge band (the 4 cm test compares a mean over the segment, so resolution hardly matters; tape at 0.98 m unchanged) |
| Braking | points the path check brakes for count as confirmed and occupied at once (`grid.mark_blocking`). RiderPi 2026-11-11: the Rider braked for obstacles the map never showed (1382 unconfirmed cells, 1192 occupied) |
| Pitch, pose | pitch of the captured image (`FreeSpace.pitch_dev`); pose (`.pose`) as of the last control step before that image was processed, not of the later frame the result arrives in. Not interpolated to the capture time: while turning the pose is older than the image by an unmeasured amount (40 °/s × 50 ms = 2°) |
| Filter | only while balancing and below `max_turn_dps` 40 °/s; the rate is the wheel heading (`odom.yaw_source`) at the control step in which the result arrives, not at capture |
| Driven path | `driven_free_r_m` 0.10 m around the path counts as free |
| Unknown | columns without measurable floor (lens shading) stay unknown |
| Thread | `GridWorker` (cores = `cpu.freespace_cpus`), the pose handed to the floor model is passed along |
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
| Pose after load | **= start point**. No relocalisation: place the Rider on the start point (same heading) before starting xgonav. With `[reloc] enabled`: pose = pose at the last save, state "unknown" until confirmed |

### Return to start (`--mode home`, `xgonav/planner.py`)

| Item | Value |
|---|---|
| Planner | A* (8-neighbour) on map cells, search area = pose, start, driven path + `home.margin_m` |
| Blocked | occupied cells (`> l_show`) inflated by `home.squeeze_m` 0.09 m (direct neighbours); cells within `home.inflate_m` 0.23 m cost `squeeze_cost` 20 more (not on the driven path), so the path squeezes through only if there is no other way. Cleared within `inflate_m` around pose and goal (drift) |
| Narrowest gap | 25 cm clear width (Rider 11.5 cm wide per CAD): planner needs ≥ 3 free cells in the middle, path check in a tight spot `map.radius_tight_m` 0.08 m needs ≥ 16 cm between confirmed points. Simulator scene `spalt` 2026-11-11: passed without contact, ≤ 2 cm off centre, when the map showed the gap open; in 1 of 6 runs the map drew it 10 cm wide and it was skipped. **Not driven on the Rider** |
| Cost | 1 + `clear_cost` near obstacles + `unknown_cost` for unknown cells; line-of-sight shortcut only if not more expensive (maximum and sum) |
| Thread | `PlannerWorker` (cores = `cpu.freespace_cpus`); replan every `replan_s` 2 s and after 3 s without 5 cm progress |
| Path keeping | current path kept while free and ≤ `keep_factor` × new + 0.3 m; dropped after getting stuck |
| Local obstacles | confirmed `LocalMap` points within 1.5 m (what `_arc_check` brakes for) are blocked for planning (`use_localmap`) |
| Following | pure pursuit, lookahead 0.5 m, `home.kp_turn` 1.5/s; turn in place only above 75° error until below 25°, otherwise drive the curve at ≥ 30 % speed |
| Tight spot | occupied map cell within `tight_m` 0.21 m of the pose or of the path up to 0.45 m ahead (normal paths keep 0.23 m): lookahead 0.2 m, speed × 0.6, turn in place above 30° until below 10°, path check with `map.radius_tight_m` 0.08 m instead of 0.18 m, and aim at the middle between the nearest confirmed `LocalMap` points left and right ahead (the room map places a narrow gap centimetres off). Cost on the Pi: four 11 × 11 cell windows per control step while following a path |
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
| No-go zones | polygons in map coordinates. Planner and frontier never enter them plus `inflate_m`, `_arc_check` stops an arc that would come within `map.radius_m` 0.18 m (moving away stays allowed). All modes except manual. They drift with the pose: **no replacement for a physical barrier** |
| Patrol | points in order, `pause_s` 1.5 s each; stuck: replan, then back up and skip the point; contact, `max_fail` plans without a way or `point_timeout_s` 90 s: skip. No point reached in a lap, or end of a non-loop route: to the start |
| Near enough | stuck within `home.near_m` 0.5 m of a target counts as arrived: the path check clearance can make a free target unreachable |
| Start | only standing and with user confirmation that the Rider is at the start point: the pose jumps to the start, which removes drift. Battery ≥ 40 %. Back at 30 %, set down at 15 % (action 254) |
| Status JSON | `patrol`: `zustand`, `punkt`, `von`, `runde`, `erreicht`, `ausgelassen`, `ende`; `grid`: `gesperrt`, `map_id`, `route`, `zonen` |

### Mapping a room (`--mode explore`, `xgonav/frontier.py`)

| Item | Value |
|---|---|
| Start | one full turn at `explore.scan_w` |
| Speed | `explore.v` 0.20 (≈ 36 cm/s at `vx_limit` 1.8) to goals and back to the start (`home.v` 0.15); exceeds `robot.max_v` in mode `explore` only. Tight spots × `home.tight_v`, wander at `behavior.cruise_v`. Slows down towards the path end unless the next goal is already planned. RiderPi 2026-11-11: two runs at 0.30, no contact |
| Frontier | reachable free cell next to unknown; groups ≥ `min_cluster_m` 0.4 m. Only when none of those is left: groups close to obstacles (within `home.inflate_m`, e.g. the mouth of a 25 cm gap, 3–5 cells) ≥ `squeeze_cluster_m` 0.15 m with ≥ `min_unknown_m2` 0.05 m² of connected unknown behind. Reachability from the room map only, the local map only shapes the path |
| Choice | nearest first: smallest drive time + turn time; unknown area within `gain_radius_m` counts as at most `gain_bonus_s` 3 s less (full from `gain_ref_m2` 2 m²). The goal stays until reached or gone and moves with its group, no switching by score. Until 2026-11-11 area / time: on the RiderPi four trips of 3.5–9.6 m across known floor |
| Near the goal | within `roll_m` 1.0 m: replan every `roll_every_s` 0.4 s, alternating between following the receding frontier and planning the goal after this one (from the goal, without its frontier). On arrival with the frontier in view (± `pass_view_deg` 40°) and that plan ≤ `pass_age_s` 3 s old: drives on without stopping |
| Tight-only goals | at most `tight_max` 3 per run (holes in drawn walls look like gaps) |
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

## Pose state (`[reloc] enabled`, default off)

Built 2026-11-11, simulator and unit tests only; not installed on the Rider.

| Item | Behaviour |
|---|---|
| Pose becomes unknown | xgonav start with a loaded map; balancing -> not balancing (set down, fall, rest mode); balancing > `blind_s` (2 s) without fresh wheel data; web "Pose verwerfen". Never on an empty map |
| While unknown | nothing is entered into the map, driven path not recorded, driven ground not cleared; pose keeps integrating (display, hollow triangle). `home`, `patrol`, `explore` refuse to start and end if running. `manual`, `follow`, `approach`, `pet`, `idle` stay allowed |
| Valid again | web "Steht am Startpunkt" (pose jumps to the start point), "Pose stimmt" (shown pose kept), patrol start with confirmation, `/map/reset`. Only while balancing: standing up rolls ~24 cm |
| Not done | keep-out zones are checked at the wrong place while the pose is unknown (web shows a note); buttons are not greyed out, xgonav answers 409 |
| Cost on the Pi | two comparisons per control step |

## Map matching (`xgonav/reloc.py`, offline only so far)

Matching itself is not wired into the app yet; only the pose state below is. Tools: `tools/map_replay.py` (variant `abgleich=1`), `tools/reloc_eval.py`, `tools/map_eval.py reloc` (`docs/testing.md`).

| Part | What |
|---|---|
| Scan | floor boundary points of the last seconds in the robot frame (`ScanPuffer`), same rays as `GridMap.add_freespace`; hits weighted 1 / (1 + (d / 0.8 m)²), samples on the free rays |
| Field | from a map state: closeness to the nearest wall (sigma 0.10 m); hits on known free floor and free samples through walls cost |
| `abgleich_lokal` | window ±0.30 m, ±8° around the odometry pose, one `cv2.matchTemplate` per heading. Corridor (pose slides along one direction): corrects across and heading only |
| `suche_global` | whole map, 3° / 0.10 m, best 8 refined; accepted only if unique |
| "unsure" | too little wall, too little known area, does not fit, slides, ambiguous, peak at the window border |
| While mapping | reference = map state at least 12 s old; nothing is entered while the remaining correction exceeds 0.10 m / 3° |

Measured 2026-11-11 (PC; boundary from FloorNet `floornet_s3.bin`, 8 raw recordings, simulator with drift 3 % distance, 2 % turn, 0.1 °/s):

| Case | Result |
|---|---|
| Simulator, second drive against the finished map of the first | mean pose error 0.56 -> 0.25 m (`raum`), 0.78 -> 0.13 m (`zwei`) |
| Simulator, global search, 15 s pieces of the second drive | 4 right, 0 wrong, 18 unsure of 22 (square rooms: corners look alike); 30 s pieces in `raum`: 7 right, 0 wrong, 5 unsure |
| Real recordings, matching while mapping | cells between double walls 1949 -> 1532 over 8 drives (explore6: 472 -> 286, end pose moved 0.76 m; measured drift of that drive 0.75 m); part of the drop is observations not entered |
| Real recordings, global search, piece cut out | 5 right, 9 wrong, 98 unsure of 112: explore sees most places once, the rest of the map does not know the piece |
| Real recordings, global search, piece kept in the map (upper bound) | 32 right, 4 wrong (all explore6, wheel heading), 76 unsure |
| Time on the PC | field 2 ms, local match 3 ms, global search 42-75 ms; not measured on the Pi |

Open: two drives through the same room in one map frame (real test of the global search), timing on the Pi.
