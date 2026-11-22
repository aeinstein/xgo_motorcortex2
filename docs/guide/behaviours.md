# Behaviours

A mode is picked in the web UI (mode bar) or with `--mode` on the command line. Every mode except `manual` needs a running pipeline and a working FloorNet; without it the Rider is **blind** and stands still.

## Modes

| Mode (web, EN / DE) | `--mode` | What it does | Driven on the Rider |
|---|---|---|---|
| Idle | `idle` | watches only, no driving; sound direction active | yes |
| Manual / Hand | `manual` | D-pad / keys; skips floor model and path check, fall detection and watchdogs stay | yes |
| Follow / Folgen | `follow` | follows the chosen target class (person by default) at a set distance | yes (2026-11-02, `max_v` 0.10–0.15) |
| Approach / Anfahren | `approach` | searches the target class, drives to ~0.55 m in front and stops | **no**, simulator only |
| Explore / Erkunden | `explore` | maps the room: full turn, then drives to the borders between known floor and unknown (frontiers), returns to the start | three runs 2026-11-06, map not usable yet; **no fall protection** |
| Home / Zum Start | `home` | plans a path over the room map back to the start point; at the end measures the pose against the start image | yes (2026-11-19: end ~1 cm from start) |
| Patrol / Streife | `patrol` | drives the route drawn on the room map, lap after lap; back to start at 30 % battery | **no** |
| Pet / Haustier | `pet` | reacts to people, cats and dogs with eyes, sounds and gestures; can search, roam and listen | **no**, simulator and unit tests only |

Details: [Odometry and maps](../mapping.md) (explore, home, patrol), [Display and emotions](../display-emotions.md) (pet), [Limitations](../limitations.md).

!!! danger "Stairs"
    The floor model sees a step too late. `explore`, `approach`, `patrol` and `pet` with driving enabled must **never run near stairs or edges**.

## Safety layers

Every drive command passes these layers; the last one has the final word.

| Layer | Effect |
|---|---|
| `safety()` | not balancing → stop; blind → stop; little free space ahead → slow down / no forward; time-to-contact < 0.8 s → no forward |
| Path check (`_arc_check`) | planned arc (forward and reverse) against the local map |
| `Robot` | clamp to `max_v` / `max_w`, ramp, keepalive, userspace watchdog 0.5 s |
| Kernel watchdog | `motion_timeout_ms` 1000: stops if xgonav stops writing |

Detections (YOLO) are only targets, never obstacles: obstacles come from FloorNet alone.

## Follow

| Item | Value |
|---|---|
| Target | class from the dropdown (`GET /detector`); detector runs only in follow, approach, pet |
| Distance | from the foot point of the box and the camera geometry |
| Lost target, with room map and confirmed pose | planner drives to the last known position (pose + distance + bearing at last sighting), there turns 360° (`lost_look_deg`), then waits; target back in view ends the search. Simulator only |
| Lost target, otherwise | turns towards the side it was last seen, waits after `search_timeout_s` |

## Explore → Home → Patrol

```
Explore (map the room) → map locked → draw route + no-go zones → Patrol (drive the route) → Home
```

| Step | Note |
|---|---|
| Explore | ends when nothing reachable is left, saves the map and drives to the start |
| Map lock | set after explore; drives no longer change the map |
| Patrol start | only standing **at the start point** (confirmation dialog), battery ≥ 40 % |
| Home | arrives at the *map* start; the start image check corrects the pose at the end |

## Pet

| Situation | Reaction |
|---|---|
| person far / near / very near | notices, turns towards, cute, shy |
| trusted person walks away (`follow_trust`) | follows like `follow` (same servo), stops when close |
| person gone | searches the last position (planner), looks around, then listens; sound only counts after that |
| nobody for 30 s | roams (looks around, short legs) |
| sound from the side | turns towards it and listens; calling makes it come |
| dog close | whimpers, ducks; optional faint (sets down, stands up again by itself) |

Defaults stay on the spot unless driving budgets are set (`[pet] seek_m`, `roam_m`, `call_m`). `retreat_m`, `creep_m` and `faint` only with someone watching, never near stairs.
