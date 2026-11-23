# MCP server (`web/mcp.php`)

Lets a multimodal LLM look at the world through the Rider: camera image, room map, status, turning, driving.

| Item | Value |
|---|---|
| URL | `https://<rider>/mcp.php` |
| Transport | MCP Streamable HTTP, JSON-RPC 2.0 per `POST`, answer as `application/json`; no SSE stream, no session (`GET` → 405) |
| Protocol versions | 2025-06-18, 2025-03-26, 2024-11-05 |
| Methods | `initialize`, `ping`, `tools/list`, `tools/call`; notifications → 202 |
| Stack | nginx → php-fpm pool `xgonav` → `web/mcp.php` → xgonav local API `127.0.0.1:8088` and `/proc/XGORider` |
| Auth | none by default (LAN only); optional bearer token. `Content-Type: application/json` required (415 otherwise: blocks form posts from foreign web pages) |
| Language | tool names, descriptions and results are German |
| Tested | on the Rider 2026-11-20 (lying, sleep mode, level `schauen`): `initialize`, `tools/list`, `status`, `karte`, `erkennung`, `emotion`, `kamera`, refusals of `bild` (pipeline off) and of the movement tools. `tests/test_mcp.py` (fake xgonav, needs `php`): 10 passed on the Rider 2026-11-23 (PHP 8.2, temporary venv with pytest); no tool has moved the Rider yet. `fahren` refuses until xgonav has been restarted with `sicher=1` |

## Permission (`/etc/xgonav/mcp.json`)

```json
{"stufe": "schauen", "token": ""}
```

| `stufe` | Tools allowed |
|---|---|
| `schauen` (default, also without the file) | look only: nothing moves |
| `drehen` | + stand up, set down, turn on the spot, body height, roll |
| `fahren` | + drive, modes, map, route, firmware actions |

- `token` not empty: requests need `Authorization: Bearer <token>` (401 otherwise).
- Read on every request; no restart needed. `deploy/install.sh` creates the file with `schauen` and keeps an existing one.
- A tool above the level answers with `isError` and a text telling the LLM to ask the owner.

## Tools

| Tool | Level | Arguments | Effect |
|---|---|---|---|
| `status` | schauen | `voll` | short JSON: mode, message, balancing, battery, heading, pitch, camera height, free space per sector, map pose, tracks, last sound, level; `voll`: xgonav `/status` verbatim plus the short form |
| `bild` | schauen | `overlay` | camera JPEG 640×480 (`/frame?raw=1`, newer than the last status frame) + frame number, heading, pitch, free space; `overlay`: with floor boundary etc. Error while the pipeline is stopped (no stale image) |
| `karte` | schauen | `ausschnitt_m`, `px`, `mitte_x`, `mitte_y` | room map PNG (`/map.png`) + pose, start, shown section, route and no-go zones |
| `erkennung` | schauen | | xgonav `/detector` |
| `bildverarbeitung` | schauen | `an` | pipeline on (waits for frames, ≤ 20 s) / off |
| `ruhemodus` | schauen (wake) / drehen (sleep) | `an` | `false`: wake (`xgo-ruhe off`, xgonav `/ruhe?on=0`; stays lying, pipeline stays off); `true`: sleep mode as the web button (set down, pipeline off, sleep eyes, `xgo-ruhe on`) |
| `kamera` | schauen | `belichtung_us`, `verstaerkung` | exposure / gain (0 = auto); without arguments: current values |
| `leds` | schauen | `farben` | up to 4 colours `#rrggbb` → `/proc/XGORider/leds/<i>` |
| `emotion` | schauen | `name` | display animation; without name: list |
| `stopp` | schauen | | xgonav `/stop`: standstill, mode idle; aborts a running MCP movement |
| `aufstehen` | drehen | | `/stop`, action 253, waits for balancing (≤ 8 s) + 6 s; needs pipeline, battery ≥ 20 % |
| `absetzen` | drehen | | `/stop`, action 254, waits until lying |
| `drehen` | drehen | `grad` −1080..1080, `w` 0.16..1 (factor of `max_w`, default 0.5), `tol` 1..10° (default 3), `bild` | turn on the spot (+ = left): xgonav `POST /turn` closes the loop on the IMU heading (`behavior.turn_to_kp`, overshoot counted for `turn_settle_s`), MCP waits for `/status` `turn.fertig`; then image |
| `rundblick` | drehen | `bilder` (2..8, default 4), `overlay` | full turn in equal steps, one image per step, ends at the starting heading |
| `hoehe` | drehen | `u8` 61..200 | body height → `/proc/XGORider/height`, waits 4 s |
| `neigen` | drehen | `grad` −20..20 | roll → `/proc/XGORider/roll` |
| `ausrichten` | drehen | | xgonav `/level` |
| `fahren` | fahren | `v`, `w`, `sekunden` 0.2..5, `geprueft` (default true), `bild` | `v`, `w` are factors of `robot.max_v`/`max_w` (since 2026-11-23). Checked: `/drive?sicher=1`, `v` ±0.30, `w` ±0.60; `geprueft: false`: `/drive` like the web D-pad / `rider_nav.py fahr`, `v`, `w` ±1; 10 Hz, then standstill + idle; returns distance, pose before/after, braking messages, image |
| `folge` | fahren | `abschnitte` [[v, w, s], …] (1..10, each 0..5 s, ≤ 30 s total), `geprueft`, `bild` | sections back to back without a stop in between (like `rider_nav.py folge`), limits as `fahren`; then standstill + idle |
| `modus` | fahren | `name`, `ziel`, `bestaetigt` | xgonav `/mode` (all modes; rules as in the web UI) |
| `tempo` | fahren | `anteil` 0.05..1 | xgonav `/tempo` |
| `karte_aendern` | fahren | `aktion` | `leeren`, `start_hier`, `speichern`, `sperren`, `entsperren`, `pose_start`, `pose_hier`, `pose_verwerfen` |
| `route` | fahren | `punkte`, `schleife`, `zonen` | xgonav `/route` |
| `aktion` | fahren | `n` 1..6 | firmware action; 3..5 via xgonav `/action` (arc check) |

Not offered: shutdown, restart, settings, Wi-Fi, calibration, floor model switch, raw recording, ISP contrast/saturation/brightness.

## Movement rules

| Rule | Detail |
|---|---|
| Preconditions | balancing, battery ≥ 20 %, not sleep mode (wake with `ruhemodus`), mode `idle` or `manual` (another mode is never interrupted; only `stopp` ends it) |
| One movement at a time | `flock` on `/tmp/xgonav-mcp.lock`; a second call fails at once |
| End of every movement | `/drive?v=0&w=0`, then `/stop` (mode idle), also on error, timeout (120 s) or client disconnect; xgonav stops by itself 0.5 s after the last setpoint |
| `fahren`/`folge` checked by default | `sicher=1`: the setpoint passes `safety()` (fallen, blind, free space, TTC) and the arc check like the autonomous modes; nominal speed ≈ v · `max_v` · `tempo` · `robot.vx_limit`; refused while blind or with the pipeline stopped. `geprueft: false`: hand control as the web D-pad (no floor boundary, no arc check; fall detection and watchdogs stay), no pipeline needed; only on the owner's request |
| `drehen`, `rundblick` | unchecked turn on the spot (v = 0); `drehen` ≤ `w` (factor, default 0.5), `rundblick` ≤ 0.5 (factor) |
| Limits | `v`, `w`, duration and height are clamped in `mcp.php`; xgonav clamps to `max_v`/`max_w` and the speed slider on top |

## Cost on the Pi

| When | Cost |
|---|---|
| No request | none (php-fpm `ondemand`, no thread, nothing in xgonav) |
| Per tool call | one php-fpm request, 1–3 local HTTP requests to xgonav |
| `bild` | one JPEG in the web thread of xgonav (hardware encoder), not in the image pipeline |
| `karte` | one map render + PNG in the web thread |
| During a movement | `/status` + `/drive` at 10 Hz from one php-fpm worker (pool has 4); turning: one `/turn`, then `/status` at 10 Hz |
| `sicher=1` | one `safety()` + arc check per control step while such a setpoint is active, as in every autonomous mode |

## Clients

```bash
claude mcp add --transport http rider https://<rider>/mcp.php
# with token: --header "Authorization: Bearer <token>"
```

```json
{"mcpServers": {"rider": {"url": "https://<rider>/mcp.php"}}}
```

The client must trust the TLS certificate of the Rider (`/etc/xgonav/tls`).

Check by hand:

```bash
curl -s https://<rider>/mcp.php -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"status","arguments":{}}}'
```
