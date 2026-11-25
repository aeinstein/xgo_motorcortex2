# XGO Rider — LED meaning (firmware view)

Applies to firmware **R-1.1.6** (`Rider2-recovered-full`). Sources: `board_support.cpp`
(`renderLeds`/`renderFaultLeds`), `led_status.h` (`ledFaultFor`), mode mapping in `main.cpp`.
Stock firmware R-1.1.3 may differ (not analysed).

## Hardware
- **4 RGB LEDs** (WS2812/NeoPixel) on **GPIO27** (RMT channel 0), index 0..3.
- Brightness: every output is multiplied by n/256. Stock ~0.5 (129); patch 06-leds: n settable at runtime (reg `0x12`, `!led <n>`, Pi: `settings/led_brightness`), default 32, not stored across an ESP restart. Pi side stores the web value in `/var/lib/xgonav/led_brightness` (default 20 % = 51); `rc.local` sets it at boot, xgonav on start flashes 255 for 0.2 s, then sets it. "Full" colour values are usually 200–255.

## Priority
Per frame `renderFaultLeds` is tried first, otherwise `renderLeds`:

1. **Fault colour** is shown **unless** `robot_mode==1` (balancing) or `robot_mode==250` (low battery), or there is no fault.
2. Otherwise normal display (low-battery blink, gyro calibration, pitch trim, driving direction).
3. Otherwise the **user LEDs** (patch 06-leds), balancing or not; redrawn when they or the brightness change.

LED effects are the client's business: the firmware shows states only. The colour gradients of the roll/height wave (actions 1, 2, 5, 6, registers 0x39/0x82) were removed with patch 06-leds.

Fault colour depends on `last_stop_reason` (`ledFaultFor(driving,last_stop)`), normal display on
`mode` and drive/action values. All fields are in `!status` (`mode`, `reason`, `last_stop_reason`, `driving`).

## Power-on / ESP reset

| Colour | Pattern | Meaning |
|---|---|---|
| **Pink** (R192 G0 B63), circling | bright spot runs 0 → 1 → 2 → 3 (front left, rear left, rear right, front right), 0.4 s per turn, the others at 12 % | **Gyro boot calibration running** (patch 06-leds, `!ImuState::calibrated`): keep the Rider still. ~1.5 s after power-on at standstill, up to ~15 s when moved, forever while `\|roll\| ≥ 4°`; again after an IMU fusion restart (then in the current user colours). Low battery, mode 0 and fault colours go first. |
| **Pink** (R192 G0 B63), steady, all 4 | constant | **Powered on / ESP reset / armed, gyro calibrated.** `begin()` and `arm()` (action 255, stand-up 253, `!arm`) set the user LEDs to palette index 234; arming overwrites colours written by the client (patch 06-leds). Shown whenever no fault/stop colour, low battery, driving (green) or trim display is due. |

## Fault / stop colours (Rider NOT standing, servos usually released)

| Colour | Pattern | Meaning (LedFault ← stop reason) |
|---|---|---|
| **Red, steady** | constant | **Tipped over / tilt fault**. Also `mode 0`, upright too. Patch 05-actions: no self-start from mode 0 (stock: balanced again after ~500 upright control ticks while still red); only a command arms it (255, stand-up 253), then pink. |
| **Red, fast blink** (~5 Hz) | 100 ms on/off | **Free fall detected** — stopped and servos released *before* impact. |
| **Red, slow blink** (~1 Hz) | 500 ms on/off | **Other fault**: IMU, actuator, timing, invalid control output, policy or calibration fault. Servos released. |
| **Amber** (R200 G70 B0), steady | constant | **Manually stopped** — `!stop`, action 0x3e=254, switch or app. |

## Pi shutdown (patch 06-leds)

| Colour | Pattern | Meaning |
|---|---|---|
| **Blue** (R0 G0 B200), steady, all 4 | constant | **Pi shut down** (action 234, set by `xgo-shutdown-led.service` on Pi shutdown/reboot). Before every other display, nothing else changes; cleared by action 233 (xgonav on start) or ESP restart. |

## Low battery

| Colour | Pattern | Meaning |
|---|---|---|
| **Red, blinking** (battery timing) | on/off with `low_blink` | **Battery empty / shutdown imminent** (`mode 250`, `stop_required`). Own blink timing, not the fault patterns above. Stock: latched until power-off, all commands rejected. Patch 05-actions: clears after 5 s at ≥ 15 % (then mode 253), action 237 (ESP restart) accepted meanwhile. |

## Normal operation (balancing, `mode 1`)

| Colour | When | Meaning |
|---|---|---|
| **User LEDs** (default: pink, palette 234, all 4 the same) | standing/balancing, not driving | Idle. Colours via `!rgb` / registers 0x69–0x6c. |
| **Green, direction-dependent** | driving (\|forward\|>5 or \|turn\|>5) | Forward → LEDs 0&3, backward → 1&2, left/right turn → two LEDs each. |

## Pitch trim (`mode 1`)

| Colour | When | Meaning |
|---|---|---|
| **Blue blinking** (~1 Hz) | `pitch_trim_stage==2` | **Pitch trim** running (balancing in place). |

## Mode → situation (`!status.mode`)

| mode | Situation | LED |
|---:|---|---|
| 1 | balancing / driving | user LEDs, green direction |
| 0 | tilt/fall fault, "stranded", runaway protection; stays until a command (patch 05-actions) | red (tilt = steady, fall = fast blink) |
| 250 | battery empty / shutdown | red blinking (battery timing) |
| 251 | actuator fault / flash stop | red slow blink (Other) |
| 252 | policy/control fault | red slow blink (Other) |
| 253 | after flash / ready but disarmed / neutral | fault colour of the current guard reason (e.g. red while tipped), otherwise user LEDs (pink after power-on and after arming) |
| 254 | manual stop | amber |
| 255 | leg calibration running | (calibration mode) |

## Cheat sheet
- **Red** = problem/stopped (pattern gives cause: steady = tip, fast = fall, slow = fault, battery timing = battery). Does not balance by itself (patch 05-actions).
- **Amber** = deliberately stopped.
- **Green** = driving (direction).
- **Blue** = pitch trim (blinking); steady blue = Pi shut down (patch 06-leds).
- **Pink circling** = gyro calibrating, keep still.
- **Pink** = powered on / armed (mode 253 balances by itself once upright), or idle with default user LEDs.
- **Other steady colour** = idle (user LEDs set via `!rgb` / `leds/0..3`).

Read state live: `!status` → `mode`, `driving`, `reason`, `last_stop_reason`. Set LEDs (visible whenever no state display is due, also while lying; reset to pink by arming):
`!rgb <index 0..3> <r> <g> <b>` or registers 0x69–0x6c (`/proc/XGORider/leds/0..3`).
