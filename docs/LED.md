# XGO Rider — LED meaning (firmware view)

Applies to firmware **R-1.1.6** (`Rider2-recovered-full`). Sources: `board_support.cpp`
(`renderLeds`/`renderFaultLeds`), `led_status.h` (`ledFaultFor`), mode mapping in `main.cpp`.
Stock firmware R-1.1.3 may differ (not analysed).

## Hardware
- **4 RGB LEDs** (WS2812/NeoPixel) on **GPIO27** (RMT channel 0), index 0..3.
- Brightness: every output is multiplied by n/256. Stock ~0.5 (129), patch 0003 fixed 64, patch 0016: n settable at runtime (reg `0x12`, `!led <n>`, Pi: `settings/led_brightness`), default 32, not stored across an ESP restart. Pi side stores the web value in `/var/lib/xgonav/led_brightness` (default 20 % = 51); `rc.local` sets it at boot, xgonav on start flashes 255 for 0.2 s, then sets it. "Full" colour values are usually 200–255.

## Priority
Per frame `renderFaultLeds` is tried first, otherwise `renderLeds`:

1. **Fault colour** is shown **unless** `robot_mode==1` (balancing) or `robot_mode==250` (low battery), or there is no fault.
2. Otherwise normal display (driving direction, wave animations, user LEDs, low-battery blink).

Fault colour depends on `last_stop_reason` (`ledFaultFor(driving,last_stop)`), normal display on
`mode` and drive/action values. All fields are in `!status` (`mode`, `reason`, `last_stop_reason`, `driving`).

## Fault / stop colours (Rider NOT standing, servos usually released)

| Colour | Pattern | Meaning (LedFault ← stop reason) |
|---|---|---|
| **Red, steady** | constant | **Tipped over / tilt fault**. Also `mode 0`. |
| **Red, fast blink** (~5 Hz) | 100 ms on/off | **Free fall detected** — stopped and servos released *before* impact. |
| **Red, slow blink** (~1 Hz) | 500 ms on/off | **Other fault**: IMU, actuator, timing, invalid control output, policy or calibration fault. Servos released. |
| **Amber** (R200 G70 B0), steady | constant | **Manually stopped** — `!stop`, action 0x3e=254, switch or app. |

## Low battery

| Colour | Pattern | Meaning |
|---|---|---|
| **Red, blinking** (battery timing) | on/off with `low_blink` | **Battery empty / shutdown imminent** (`mode 250`, `stop_required`). Own blink timing, not the fault patterns above. |

## Normal operation (balancing, `mode 1`)

| Colour | When | Meaning |
|---|---|---|
| **User LEDs** (default: rainbow palette) | standing/balancing, not driving | Idle. Colours via `!rgb` / registers 0x69–0x6c. |
| **Green, direction-dependent** | driving (\|forward\|>5 or \|turn\|>5) | Forward → LEDs 0&3, backward → 1&2, left/right turn → two LEDs each. |

## Action animations (during an action, `mode 1`)

| Colour | When | Meaning |
|---|---|---|
| **Blue blinking** (~1 Hz) | `pitch_trim_stage==2` | **Pitch trim** running (balancing in place). |
| **Blue↔cyan gradient** (animated) | roll wave active | Roll "wave" action; hue follows roll setpoint. |
| **Green↔blue tone** | height wave active | Height "wave" action; hue follows height. |

## Mode → situation (`!status.mode`)

| mode | Situation | LED |
|---:|---|---|
| 1 | balancing / driving | user LEDs, green direction, or wave animation |
| 0 | tilt/fall fault, "stranded", runaway protection | red (tilt = steady, fall = fast blink) |
| 250 | battery empty / shutdown | red blinking (battery timing) |
| 251 | actuator fault / flash stop | red slow blink (Other) |
| 252 | policy/control fault | red slow blink (Other) |
| 253 | after flash / ready but disarmed / neutral | no fault colour (LEDs unchanged / user LEDs) |
| 254 | manual stop | amber |
| 255 | leg calibration running | (calibration mode) |

## Cheat sheet
- **Red** = problem/stopped (pattern gives cause: steady = tip, fast = fall, slow = fault, battery timing = battery).
- **Amber** = deliberately stopped.
- **Green** = driving (direction).
- **Blue** = trim / roll wave.
- **Colourful, steady** = idle (user LEDs).

Read state live: `!status` → `mode`, `driving`, `reason`, `last_stop_reason`. Set LEDs (visible only when idle):
`!rgb <index 0..3> <r> <g> <b>` or registers 0x69–0x6c (`/proc/XGORider/leds/0..3`).
