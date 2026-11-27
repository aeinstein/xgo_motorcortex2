# Odometry and maps

## Odometry (`xgonav/wheels.py`, `App._odom_ds`)

| Quantity | Source |
|---|---|
| Distance | wheel encoders (`/proc/XGORider/wheels_log`) if fresh (`odom.max_age_s` 0.3 s), else command × `robot.vx_limit` × dt |
| Heading (behaviors, local map, room map) | one dead-reckoning step (`xgonav/odomfusion.py`, since 2026-11-19): IMU heading at frame time, else wheels, else ego-motion; wheels checked against IMU (`[odom] contra_*`), contradiction = slip unless the camera, when its possible error is small against the contradiction, sides with the wheels (then wheels for `imu_doubt_s`, window made up); camera heading is biased during forward drive, never a witness there. Status `odom` (`kurs`, `strecke`, `abw_deg`, `schlupf`, `widersprueche`, `imu_zweifel`, `vorschlag`) |

Encoders: 1024 ticks/rev, 60 mm wheel, track 90 mm, left = 11, right = 21, both positive forward.
int32 overflow and ESP32 restarts handled (`max_jump_ticks` 2048). Firmware R-1.1.3 has no encoders → command fallback.
Status JSON: `odom.src` = `räder` | `befehl`, `odom.m` = total distance.

## Short-term local map (`xgonav/localmap.py`)

| Item | Value |
|---|---|
| Frame | robot (x forward, y left, m), origin = wheel axle; floor points shifted forward by `camera.cam_fwd_mm` |
| Input | floor-boundary points from the free-space estimator |
| Motion | distance from wheel odometry; heading from the dead-reckoning step (same as the room map); during slip the distance comes from the wheel matching the IMU turn |
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
| Occupied | at boundary ± `hit_band_quad` × d² along the ray (0.05; sim 0 = one cell), `l_occ` +0.5: two hits until shown as occupied |
| Clamp | −3.0 .. +3.5; shown as free/occupied beyond ±0.7 |
| Range | since 2026-11-24 an ellipse in the camera frame: 2.5 m ahead (`max_range_m`, `free_range_m`), at most 1 m sideways (`side_range_m`), ~1.3 m along the ray at the image edge; before: hits 0.15–`max_range_m` 1.5 m (1.25 until 2026-11-14), free rays to `free_range_m` 1.5 m (RiderPi 1.1 m from 2026-11-06 to 2026-11-14) (one image row at 2 m is ~40 cm; a step was seen as floor from > 1 m on 2026-11-06, simulator with 1.1: floor known 0.43–0.83, occupied precision 0.84–0.92); "free to horizon" columns mark free up to 1.5 m only; both limits are distances along the ray since 2026-11-23 (before: forward distance, so up to 1.44× farther at the image edge) |
| Confirmation | off since 2026-11-14 (`confirm_dr_m` 0: with FloorNet one sighting is enough; not driven). Before: `confirm_dr_m` (RiderPi 0.2 m, ± `confirm_cells` 2): occupied only after hits on the same spot from two ranges ≥ 0.2 m apart. RiderPi 2026-11-08: the window sill mirrored in the tiles was entered at 0.56 m from the start and 0.82 m from 0.72 m further on (d = D·h_cam/(h_edge + h_cam)), i.e. a second wall. Simulator explore 120 s: walls 82 % → 73 %, precision 0.90 → 0.93, 0 collisions. Tried and dropped the same day: hits only up to 0.7 m (walls 25 %, explore ended early) and a half-resolution edge band (the 4 cm test compares a mean over the segment, so resolution hardly matters; tape at 0.98 m unchanged) |
| Braking | points the path check brakes for count as confirmed and occupied at once (`grid.mark_blocking`). RiderPi 2026-11-11: the Rider braked for obstacles the map never showed (1382 unconfirmed cells, 1192 occupied). FloorNet's free rays erase them again (chair legs, cabinet edges: floor visible behind) |
| Brake spots | full brake already in tight mode: hit points + `grid.brake_block_r_m` 0.18 m blocked for planner and goal search for `grid.brake_block_s` 300 s, not erased by free rays; replan at once. Not on every full brake: in front of the 25 cm door (sim `spalt`) it brakes with the normal radius first and then passes in tight mode; storing those cells closed the door (known floor 43 instead of 82 %). Red circles in `/map.png`, count `grid.bremsstellen` in `/status` |
| Pitch, pose | pitch of the captured image (`FreeSpace.pitch_dev`); pose (`.pose`) as of the last control step before that image was processed, not of the later frame the result arrives in. Not interpolated to the capture time: while turning the pose is older than the image by an unmeasured amount (40 °/s × 50 ms = 2°) |
| Filter | only while balancing, below `max_turn_dps` 60 °/s (40 until 2026-11-23: curves in explore at ~50 °/s left passed frontiers unknown, they were visited twice; offline 60 adds 13–15 % observations, double-wall cells +2–3 %; the pause after a fast turn, `turn_settle_s`, was removed 2026-11-17; offline still `tools/map_replay.py` variant `ruhe=S`); the rate is the dead-reckoning heading at the control step in which the result arrives, not at capture |
| Driven path | `driven_free_r_m` 0.10 m around the path counts as free |
| Unknown | columns without measurable floor (`known_from`; FloorNet reports none) stay unknown |
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
| orange ring with cross | expected position of a lost target (`follow`, following in `pet`): where it should have gone (`behavior.lost_lead_s`) |
| blue circle + line | start point and start heading |
| red triangle | Rider pose |
| yellow dots | short-term map (`localmap.py`) points, drawn only, never entered; gone once that map forgets them (`map.max_age_s`) or the pipeline stops |
| orange-red dots | of those, the confirmed ones (`map.confirm`): the arc check brakes for them |
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
| Blocked | occupied cells (`> l_show`) inflated by `home.squeeze_m` 0.09 m (direct neighbours); cells within `home.inflate_m` 0.23 m cost `squeeze_cost` 20 more (not on the driven path), so the path squeezes through only if there is no other way. Cleared within `inflate_m` around pose and goal (drift), except the confirmed local-map points themselves (since 2026-11-24: he had planned through a table leg 10 cm ahead). Local-map points block every cell whose centre is closer than `home.merk_r_m` 0.08 m, from their true coordinates (`planner.merk_maske`, like the arc check in tight spots), not their cell plus `squeeze_m`: on the 5 cm grid a 15 cm gap between chair and table legs looked like 20 cm. Now 15 cm is closed, 25 cm open, wherever the gap lies on the grid |
| Narrowest gap | 25 cm clear width (Rider 11.5 cm wide per CAD): planner needs ≥ 3 free cells in the middle, path check in a tight spot `map.radius_tight_m` 0.08 m needs ≥ 16 cm between confirmed points. Simulator scene `spalt` 2026-11-11: passed without contact, ≤ 2 cm off centre, when the map showed the gap open; in 1 of 6 runs the map drew it 10 cm wide and it was skipped. **Not driven on the Rider** |
| Cost | 1 + `clear_cost` near obstacles + `unknown_cost` for unknown cells; line-of-sight shortcut only if not more expensive (maximum and sum) |
| Thread | `PlannerWorker` (cores = `cpu.freespace_cpus`); replan every `replan_s` 2 s and after 3 s without 5 cm progress |
| Path keeping | current path kept while free and ≤ `keep_factor` × new + 0.3 m; dropped after getting stuck |
| Local obstacles | confirmed `LocalMap` points within `localmap_reach_m` 2.0 m (1.0 until 2026-11-24: he planned from 2 m towards a passage camera and local map already saw closed) (what `_arc_check` brakes for) are blocked for planning (`use_localmap`). Explore (since 2026-11-23): the best of the top 4 frontier groups that is reachable with them wins; a group reachable only without them (room map alone, e.g. through chair legs only the local map knows) is taken last and counts as tight (`tight_max`), but only if that path does not cross a local-map point (`explore.ziel_sicht`, since 2026-11-24) |
| Following | pure pursuit, lookahead 0.5 m, `home.kp_turn` 1.5/s; turn in place only above 75° error until below 25°, otherwise drive the curve at ≥ 30 % speed |
| Tight spot | occupied map cell within `tight_m` 0.21 m of the pose or of the path up to 0.45 m ahead (normal paths keep 0.23 m): lookahead 0.2 m, speed × 0.6, turn in place above 30° until below 10°, path check with `map.radius_tight_m` 0.08 m instead of 0.18 m, and aim at the middle between the nearest confirmed `LocalMap` points left and right ahead (the room map places a narrow gap centimetres off), unless the path turns a corner within `tight_center_knick_m` 0.5 m (since 2026-11-24: he aimed straight at the gap middle into a cabinet while the path went left round its corner). Cost on the Pi: four 11 × 11 cell windows per control step while following a path |
| Goal | within `arrive_m` 0.15 m, then turn to start heading (`align_heading`), status „am Start angekommen“, stands |
| Failure | `max_fail` 3 consecutive plans without path → stop, „kein Weg zum Start“ |
| Safety | `safety()` and `_arc_check()` apply as in `explore` |
| Requires | `freespace.enabled`, `grid.enabled`, running pipeline |
| Accuracy | limited by map drift: arrives at the *map* start; real error = pose error. With `[startbild]` the pose is measured against the start image at the end (below) |

### Start image check (`xgonav/startbild.py`, `[startbild]`)

| Item | Value |
|---|---|
| Anchor | grey image + SIFT features while the Rider stands at the start point (≤ 3 cm, ≤ 5°, not turning); once per start point, only after the roll calibration after stand-up (`Robot.levelled`); dropped when the start is set anew, on set-down/fall (may be carried) and when the pose becomes unknown (2026-11-22) |
| Start point | set on every stand-up (once standing still 1 s after the roll calibration) and set-down, in every mode, never on a mode change (`App._start_tick`, 2026-11-22); not with a locked map or unknown pose; "Start hier" in the web as before |
| Ending of `home` | path to a pre-point 0.40 m before the start → turn to start heading → straight onto the start (grey image every 4 cm) → stand 0.5 s → measure → if > 4 cm left: turn to the start, drive (once) → turn to the start heading within `end_deg` 2° (stop on overshoot or after 8 s) |
| Measurement | own thread on `cpu.render_cpus`: end image vs. images 0.1–0.3 m earlier, triangulated with the odometry of the straight run (mm-exact over decimetres); anchor matched to these 3D points; anchor pose solved for x, y, heading plus small pitch/roll residuals, camera height fixed (124 mm measured); cross-check free 6-DOF PnP |
| Camera model | undistorted image, f from hfov, principal point image centre, tilt + `pitch_dev`, IMU roll (without it the 3D points were rotated by the end roll, +2.3° in run 20261119-144940), lens `cam_fwd` ahead of the axle |
| Gate | ≥ `min_inl` 30 consistent matches, correction ≤ 0.5 m / 15°, pitch/roll residual ≤ 1°, 6-DOF within 5 cm; otherwise no correction, status says why |
| Correction | `GridMap.shift_pose(k)` with k ∘ measured anchor pose = anchor pose at the start; behaviour heading shifted by the same angle |
| Web | colour copy of the anchor frame (once per anchor), `GET /startbild.jpg` (JPEG once per anchor, on request), status `startbild.bild` = anchor number (0 = none); shown below the room map, hidden without anchor |
| Offline | `tools/start_abgleich.py` (same code on recordings). Run 20261119-144940 (37 m): end measured 8.0 cm forward / 10.2 cm left / −4.8°, user with folding rule 5 / 11 cm, odometry −3 / +2 cm. Short pairs of the same run (odometry nearly exact): accepted ones within 1.8 cm |
| Simulator | `tests/sim_run.py home 120 --wheels --rider --yaw-scale 1.01 --set startbild.enabled=True`: true distance to the start at the end 6.5 cm (without: 22 cm), map heading error 0.9° (3.5°) |
| Rider | run 2026-11-19 161006: measured 19 cm / −5.1°, 131 matches, 11.4 s on the CM4; end ~1 cm ahead of the start (user), but turned −9.5° (old 10° tolerance) → `end_deg` |
| Cost on the Pi | nothing while driving; ~1 ms grey image for the anchor and every 4 cm of the straight run; measurement SIFT on 1 + 5 images while standing (0.25 s on the PC, CM4 not measured) |

### Route and patrol (`xgonav/route.py`, mode `patrol`)

| Item | Value |
|---|---|
| Map lock | set when `explore` ends or `patrol` starts, `POST /map/lock`; locked: observations and the driven path do not change the map, the pose still moves. `explore` refuses a locked map |
| Map id | `map_id` in the npz; a route file with another id is ignored |
| Route file | `<grid.file>.route.json`: points, `loop`, zones, written atomically together with the map |
| Point check | on known floor, outside `inflate_m`, reachable from the start, outside zones; otherwise the route is rejected with a message |
| No-go zones | polygons in map coordinates. Planner and frontier never enter them plus `inflate_m`, `_arc_check` stops an arc that would come within `map.radius_m` 0.18 m (moving away stays allowed). All modes except manual. They drift with the pose: **no replacement for a physical barrier** |
| Dark zones | `GridMap.dark_zones`: rectangles set by fear of the dark during explore (`docs/display-emotions.md`), same effect as no-go zones via `GridMap.all_zones()`, expire after `[dunkel] zone_s`, never saved, not editable; violet in the map image, `dunkelzonen` in the map stats |
| Patrol | points in order, `pause_s` 1.5 s each; stuck: replan, then back up and skip the point; contact, `max_fail` plans without a way or `point_timeout_s` 90 s: skip. No point reached in a lap, or end of a non-loop route: to the start |
| Near enough | stuck within `home.near_m` 0.5 m of a target counts as arrived: the path check clearance can make a free target unreachable |
| Start | only standing and with user confirmation that the Rider is at the start point: the pose jumps to the start, which removes drift. Battery ≥ 40 %. Back at 30 %, set down at 15 % (action 254) |
| Status JSON | `patrol`: `zustand`, `punkt`, `von`, `runde`, `erreicht`, `ausgelassen`, `ende`; `grid`: `gesperrt`, `map_id`, `route`, `zonen` |

### Mapping a room (`--mode explore`, `xgonav/frontier.py`)

| Item | Value |
|---|---|
| Start | one full turn at `explore.scan_w` |
| Speed | `explore.v` 0.50 × `max_v` × tempo (since 2026-11-27, before 0.80: wheels up to 1.07 m/s; RiderPi 0.25 absolute, ≈ 45 cm/s at `vx_limit` 1.8); accelerating and slowing down × `explore.accel_faktor` 0.5 (full brake of the path check unchanged); turning on the spot (turn value and heading-target rate) × `explore.dreh_faktor` 0.5, steering while driving unchanged to goals and back to the start (`home.v` 0.30); factors of `robot.max_v` like every speed (since 2026-11-23, before explore could exceed `max_v`). Tight spots × `home.tight_v`, wander at `behavior.cruise_v`. Slows down towards the path end unless the next goal is already planned. RiderPi 2026-11-11: two runs at 0.30, no contact |
| Frontier | reachable free cell next to unknown; groups ≥ `min_cluster_m` 0.4 m. Only when none of those is left: groups close to obstacles (within `home.inflate_m`, e.g. the mouth of a 25 cm gap, 3–5 cells) ≥ `squeeze_cluster_m` 0.15 m with ≥ `min_unknown_m2` 0.05 m² of connected unknown behind. Reachability from the room map only, the local map only shapes the path. Since 2026-11-24 (RiderPi: goal behind a bin only the local map knew): the goal must be visible from its stop point (straight line free in room map and local map, `ziel_sicht`), and the path to it uses known floor only (`weg_bekannt`; otherwise it led through another opening into the unknown and to the spot from behind). The fallback path without the local map must pass every confirmed local-map point by `merk_abstand_m` 0.08 m (= `map.radius_tight_m`; it had planned into a 10 cm slot between a box and a door, where the arc check braked). Cells within that distance of confirmed local-map points do not count as unknown, so a hole in a drawn wall that the local map sees as wall is no frontier (since 2026-11-24, RiderPi: corner of a niche). Cost on the Pi: ≤ 8 lines per goal search in the planner thread, nothing per frame |
| Choice | nearest first: smallest drive time + turn time (bearing to the spot / `turn_dps`; 15 since 2026-11-14 so that straight ahead wins: 90° weigh like 2.1 m of driving, 40 before and in the simulator); unknown area within `gain_radius_m` counts as at most `gain_bonus_s` 3 s less (full from `gain_ref_m2` 2 m²). The goal stays until reached or gone and moves with its group, no switching by score. Until 2026-11-11 area / time: on the RiderPi four trips of 3.5–9.6 m across known floor |
| Near the goal | within `roll_m` 1.0 m: replan every `roll_every_s` 0.4 s, alternating between following the receding frontier and planning the goal after this one (from the goal, without its frontier). On arrival with the frontier in view (± `pass_view_deg` 40°) and that plan ≤ `pass_age_s` 3 s old: drives on without stopping. Every goal change is logged (`Erkunden: ...`) |
| Tight-only goals | compete with the others at `tight_penalty_s` 3 s extra (since 2026-11-15; before and in the simulator, -1: only when no goal with clearance is left, which made him turn around for a goal behind him while a tight spot lay straight ahead); at most `tight_max` 10 per run (3 until 2026-11-15 and in the simulator: holes in drawn walls look like gaps; RiderPi 2026-11-15 left 10 spots behind tight places with 3) |
| Speed | path follower brakes by the `home.end_*` profile before the first local-map obstacle along the path or on the command arc (up to 2.5 m) and before sharp corners (`home.free_slow`), not only by the arc check |
| Arrival | stops `standoff_m` before the frontier, turns towards it, `settle_s`; no new goal within `avoid_r_m` 0.3 m of the spot |
| Driven ground | cells within `driven_free_r_m` of the driven path stay passable for the planner despite occupied specks and `inflate_m`; obstacles the local map sees now still block |
| Contact | command > 0.08 but < 2 cm movement in 2.5 s (normal start-up takes up to 1.8 s): treated as contact, back up and block the goal at once. Wheel slip hides it |
| Stuck | no 5 cm progress or 10° turn in 3 s: replan without the old path; again, or `goal_timeout_s`: back up 0.8 s, block the goal. Stuck within `gap_block_s` 2 s after driving in a tight spot or gap (follower "eng"/"Lücke"): block at once (since 2026-11-23; replanning led through the same gap between chair legs that only the local map knew). Then the confirmed local-map points within `gap_mark_r_m` 0.6 m are entered as occupied in the room map (like `grid.mark_blocking`, own thread). No goal left: `wander_s` reactive driving, at most `max_wander` times, then stop |
| End | nothing reachable, < `min_growth_m2` new floor in `no_growth_s`, `max_s`, or `min_battery`. Then save and drive to the start (`return_home`). Arrived there after a way home of at least `recheck_min_m` 1 m, it searches once more (new floor was seen on the way; at most `recheck_max` 3 times per run, 0 = off) and only then reports the map as done |
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

## Map matching (`xgonav/reloc.py`)

Running matching is wired into the app with `[reloc] track = true` (default on since 2026-11-12; RiderPi: off since 2026-11-14): `reloc.Nachfuehrung` runs in the `GridWorker` thread for every observation entered into the room map, never in the image loop. A correction shifts the map pose rigidly (`GridMap.shift_pose`, at most `max_step_m` 0.08 m / `max_step_deg` 3° per match) and the poses of queued observations. Locked map: the map itself is the reference; while mapping: a map state ≥ `ref_age_s` old. Status: `grid.abgleich` (`abgleiche`, `angenommen`, `korrektur_m`, `korrektur_deg`, `haelt`, `zuletzt`, `unsicher`, `ms_max`). The global search (start pose) is still offline only. **Not driven.** Tools (same class): `tools/map_replay.py` (variant `abgleich=1`), `tools/reloc_eval.py`, `tools/map_eval.py reloc` (`docs/testing.md`).

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

Known weakness: in front of a single wall the position along the wall is not observable; in a synthetic test the match was accepted there (not flagged as sliding) and pulled the pose up to 0.18 m along the wall until a corner came into view.

Open: two drives through the same room in one map frame (real test of the global search), timing on the Pi, a RiderPi drive with `track`.
