# Web interface

![Web UI](screenshots/frontend.PNG)

| Item | Value |
|---|---|
| URL | `https://<rider>/xgonav/` (`/` and port 80 redirect here) |
| Stack | nginx (TLS on 443) → php-fpm pool `xgonav` → `web/index.php`, `web/api.php` |
| Backend | xgonav local API `127.0.0.1:8088` (`output.web_host`, `web_port`) |
| Auth | **none** — LAN only |
| Installed to | `/opt/xgonav/web/` |
| Static pages | `https://<rider>/tagebuch/` → `web/tagebuch/index.html` (dev diary, nginx only) |

## Layout (`web/index.php`)

| Area | Content |
|---|---|
| Header (sticky) | host, connection, mode, state (balancing/lying), battery (green ≥ 40 %, yellow ≥ 20 %, red < 20 %), pipeline, recording (only while switched on), WLAN (signal dBm: green ≥ −60, yellow ≥ −70, red below; ↓/↑ kB/s of `wlan0`), language DE/EN, **Einstellungen** (modal, see below), **Herunterfahren** (confirm dialog), **Ruhe**, **STOPP**; below 1000 px the two buttons show only ⚙/⏻ |
| Left | camera image (WebSocket) with camera bar below it: exposure (ms, auto), gain (1–63.9×, auto), actual values (help in the ⓘ tooltip); mode selector (Idle · Hand · Follow · Explore · Approach · Zum Start) + target class dropdown (classes from `?a=detector`, reloaded after an xgonav restart, last choice in `localStorage`; disabled when YOLO is off), pipeline card with start/stop, Aufzeichnen button and status tiles, raw status JSON |
| Lage (open by default, right column top) | 3D model of the Rider rotated by roll, pitch, yaw from `imu_log`: CAD model `web/rider-model.bin` via WebGL, legs follow the `height`/`roll` setpoints through the linkage, smoothed ~0.33 s, at minimum while state = 254 (sliders keep their value), see limitations; wheels turn by the encoder ticks from `wheels_log` (1024/rev, live packet `imu.wheels`, CAD model only) (built by `tools/step_to_model.py` from `docs/Rider-Pi.STEP`, loaded when Verlauf is first opened); without WebGL or the file, simple boxes on canvas 2D; fixed compass ring on the ground (0° = heading at ESP32 start, + = left), blue line = body forward direction; values below. Samples replayed smoothly ~0.4 s behind; rotation about the wheel axle. Drag (mouse/touch) to orbit the view (elevation 5–85°), double-click resets; view kept in `localStorage` `xgonav.lage3d`; live (5 s buffer, memory); open/closed kept in `localStorage` `xgonav.lage`. Controls: vertical *Kamerahöhe* slider beside the model (shows the camera height the floor geometry uses, in mm, from `cam_h_gerade`, including the × 0.921 correction, so about 8 % below a folding-rule reading; writes u8 61–205, u8 in the tooltip; shows `u8 …` until xgonav has answered once), roll −20..20° below |
| Fahren | D-pad, speed 5–100 %, Aufstehen (arm, 255), Neu ausrichten (leveling pulse), Absetzen (disarm, 254); *Extras* sub-section (collapsed, open/closed in `localStorage` `xgonav.extras`): actions 1–6, 4 LEDs + all off + strobe, brightness slider (persistent), display emotions |
| Raumkarte | room map PNG, view 3/6/10/16 m; mouse: drag to pan, wheel to zoom about the pointer (1 m..`grid.size_m`), double-click recentres on the Rider and follows again (touch: page scrolls, no pan). Buttons in three fixed groups: *Karte* (Speichern, sperren/entsperren, Zurücksetzen with confirm), *Start · Pose* (Start hier, Pose verwerfen: only with `reloc`, disabled while the pose is unknown), *Route · Zonen* (Route setzen, Sperrzone, Zonen löschen). Editing bar and pose warning are overlays on the map (top/bottom), groups are disabled while editing; nothing shifts the layout. Status: pose line with distance to start, one reserved line for explore/patrol |
| Horizont (only with `[horizon] enabled`) | stand reference state, rows stand/IMU/image, image − IMU, ego ok share; green line ±1/±5 px, *Für diese Kamerahöhe speichern*, table (shown as camera height, stored per `height_mm`) with delete; checkbox *Bild am IMU-Nick festhalten* (browser shifts the `<img>` by `row_stand − row_imu` via CSS, no server cost; stored in `localStorage`) |
| Verlauf (collapsed) | history charts, see below |

Language: German is in the HTML, English in the dictionaries `EN`/`EN_HTML` of `index.php` (keyed by the German text; long help blocks via `data-i18n-html`). Switched in the browser only, choice in `localStorage` `xgonav.lang`; free text from xgonav/`api.php` (status, errors) stays German. `?i18n=debug` logs missing translations to the console. No cost on the Pi.

Live data: one WebSocket `/xgonav/live` (see below) instead of polling; HTTP `a=status` only at load and after
commands. Map every 1 s (next request after previous finished).

## Einstellungen (settings modal)

| Item | Value |
|---|---|
| Fields | all of `xgonav/config.py` (402), categories Hardware · Wahrnehmung · Karten · Verhalten · Ausgabe → TOML sections; help text = comments from `config.py` |
| Schema | `web/config-schema.json`, generated on the PC by `python tools/config_schema.py`; `test_config_schema_aktuell` fails if stale |
| Editing | line-based: only changed lines rewritten, indentation and comments kept; new keys appended to their section with `# Web <date>`; *Default* removes the line. Multi-line values: read-only (ssh) |
| Check | browser (type), PHP (schema type), `Config.load` (same as service start) |
| Backup | before every write: `/var/lib/xgonav/config-backup/config.toml.<stamp>`, last 50 kept, restore from the modal |
| Takes effect | after restart of xgonav (button *Neu starten* / *Speichern und neu starten*) |
| Rights | `config.toml` group `video`, 0664 (`deploy/install.sh`); sudoers `deploy/sudoers-xgonav-restart` |
| Pi cost | none in the pipeline; per save one Python start (~50 ms) |

## History (Verlauf)

Canvas, no library; crosshair + values on hover/touch (charts). Data comes from `/proc` via PHP and
works without xgonav.

| Chart | Data | Window | Storage |
|---|---|---|---|
| CPU | load per core (%), from `/proc/stat` deltas; temperature + loadavg in heading | 10 min | memory |
| WLAN | ↓/↑ kB/s from `wlan0` byte-counter deltas, signal dBm (−90…−30), quality /70 in heading; from live packet field `sys.wlan` (1 Hz, also in a hidden tab) | 10 min | memory |
| Akku | battery % (from status or `sys`), ≤ 1 point / 30 s | 120 min | `localStorage` `xgonav.akku`; button Leeren |

- Battery is recorded while the page is open, also with the card collapsed. A second open tab merges into the same key.
- Open/closed state is kept in `localStorage` (`xgonav.verlauf`).
- Sign convention as firmware: roll + = right side down, pitch + = nose down, yaw + = left. When seq goes backwards (driver reloaded), the buffer is cleared. Animation runs only while the card is open and the tab is visible.

## Manual driving

- Hold button / pointer / arrow keys / WASD. Switches to mode `manual` automatically.
- Setpoint sent every 200 ms; xgonav stops after 0.5 s without setpoint.
- Limited to `robot.max_v` / `max_w`.
- No floor model, no arc check; fall detection and watchdogs active.
- Works with pipeline stopped (no camera, ~no CPU).
- Window blur releases all keys.

## `api.php`

GET only for `status`, `route`, `horizon`, `detector`; all commands POST. Parameters are validated; invalid → HTTP 400.

| Request | Target | Range |
|---|---|---|
| `GET ?a=status` | xgonav `/status` (once; live via `/xgonav/live`) | |
| `GET ?a=detector` | xgonav `/detector` | JSON `on`, `geladen`, `backend`, `classes`, `modes`, `default` |
| `POST ?a=mode&name=..&target=..` | xgonav `/mode` | name ∈ idle, manual, follow, explore, approach, home, patrol, pet; target `[a-z ]{1,30}` |
| `POST ?a=stop` | xgonav `/stop` | |
| `POST ?a=pipeline&on=0\|1` | xgonav `/pipeline` | |
| `POST ?a=ruhe&on=0\|1` | xgonav `/ruhe` + emotion `sleep` (on only) + `sudo -n /usr/local/sbin/xgo-ruhe on\|off` | always 200; JSON `ok`, `ruhe`, `xgonav`, `system`, `fehler` (both parts are tried even if one fails) |
| `POST ?a=record&on=0\|1` | xgonav `/record` | |
| `POST ?a=ausrichten` | xgonav `/level` | 409 if not balancing |
| `POST ?a=drive&v=..&w=..` | xgonav `/drive` | −1..1; mode `manual` only |
| `POST ?a=exposure&us=..` | xgonav `/exposure` | 0 = auto, 100..33333 (checked by xgonav) |
| `POST ?a=gain&g=..` | xgonav `/gain` | 0 = auto, 1..63.9 |
| `POST ?a=map_reset` | xgonav `/map/reset` | |
| `POST ?a=map_start` | xgonav `/map/start` | |
| `POST ?a=map_save` | xgonav `/map/save` | |
| `POST ?a=map_pose&aktion=start\|hier\|verwerfen` | xgonav `/map/pose` | confirm or discard the pose in the room map (`[reloc] enabled`) |
| `GET ?a=horizon` | xgonav `/horizon` | |
| `POST ?a=horizon_edit&row=..` | xgonav `/horizon/edit` | 0..479.9, empty = off |
| `POST ?a=horizon_cal&row=..` | xgonav `/horizon/cal` | 0..479.9 |
| `POST ?a=horizon_delete&h=..` | xgonav `/horizon/delete` | mm |
| `POST ?a=led&i=0..3&c=%23rrggbb` | `/proc/XGORider/leds/<i>` | |
| `POST ?a=led_brightness&p=0..100` | `/var/lib/xgonav/led_brightness`, then `/proc/XGORider/settings/led_brightness` | percent → n = p·255/100; persistent, set at boot (`rc.local`) and on xgonav start |
| `POST ?a=action&n=..` | `/proc/XGORider/action` | 1..6, 253, 254, 255; 254 first calls xgonav `/stop?nur=pet` (ends mode `pet`, so it cannot stand up by itself) |
| `POST ?a=height&h=..` | `/proc/XGORider/height` | 61..205 |
| `POST ?a=roll&deg=..` | `/proc/XGORider/roll` | −20..20 |
| `POST ?a=calibration&on=0\|1` | `settings/calibration` | API only, no UI button |
| `POST ?a=pitch_calibration` | `settings/pitch_calibration` | API only, no UI button |
| `POST ?a=emotion&name=..` | `tools/emotion.sh play` | name must exist in `/opt/xgonav/emotions` |
| `POST ?a=shutdown` | plays `sleep` emotion, `sudo systemctl poweroff` | |
| `GET ?a=config` | schema `web/config-schema.json` + values parsed from `/opt/xgonav/config.toml` + backups | JSON `schema`, `werte`, `unlesbar`, `hash`, `schreibbar`, `sicherungen` |
| `POST ?a=config_save&hash=..` | rewrites changed lines of `config.toml` | body JSON `{"section.key": value\|null}` (null = remove line → default; top level: `allgemein.control_hz`); type-checked against schema, then `Config.load` via `/opt/xgonav/venv/bin/python`; 409 if file changed since `hash` |
| `POST ?a=config_restore&name=..&hash=..` | copies a backup back | name `config.toml.YYYYMMDD-HHMMSS[-n]` |
| `POST ?a=restart[&trotzdem=1]` | `sudo -n systemctl --no-block restart xgonav` | 409 in Ruhe (always) or while mode ≠ idle / balancing / recording (unless `trotzdem=1`) |

xgonav unreachable → HTTP 502 `xgonav nicht erreichbar`. Exposure/gain changes last until xgonav restarts.

## xgonav local API (`xgonav/web.py`)

| Method | Path | Response / effect |
|---|---|---|
| GET | `/status` | JSON (mode, pipeline, status, cmd, tracks, free, ttc, fps, loop_ms, loop_p95, loop_max, det_ms, grid, exposure, battery, balancing, yaw_drv, height_mm, height_u8, cam_h_mm, cam_h_gerade `[a, b]` (mm = a + b·u8), pitch_src, pitch_deg, ego, odom, …); `?breakdown=1` adds per-section loop timing |
| GET | `/ws` | WebSocket, overlay JPEGs; `?nick=1`: per-frame `row_stand`/`row_imu` in the header |
| GET | `/frame?after=<idx>` | single JPEG newer than idx |
| GET | `/stream` | MJPEG (legacy) |
| GET | `/map.png?px=480&view=6[&cx=&cy=]` | room map; px 120..960, view 1..`grid.size_m`; centre `cx`,`cy` in map metres (both or neither; default Rider pose; clamped to ±`grid.size_m`); header `X-Map-View: cx,cy,view,size`; 404 if no map |
| POST | `/mode?name=..&target=..` | 400 unknown mode, 409 not allowed |
| POST | `/stop` | standstill, mode idle |
| POST | `/pipeline?on=0\|1` | stop/start image processing; 409 in sleep mode |
| POST | `/ruhe?on=0\|1` | sleep mode on/off; `/status` field `ruhe` |
| POST | `/record?on=0\|1` | switch raw recording; `/status` field `aufzeichnung`: `an`, `laeuft`, `ordner`, `bilder`, `fehler` |
| POST | `/level` | leveling pulse (roll 0, 3 s `roll_balance`); 409 if not balancing |
| POST | `/drive?v=..&w=..` | 409 if not mode `manual` |
| POST | `/exposure?us=..`, `/gain?g=..` | 400 if out of range |
| POST | `/map/reset` | clear room map, pose = start = 0 |
| POST | `/map/start` | start point = current pose |
| POST | `/map/save` | save now to `grid.file`; 404 if no map or no file |
| POST | `/map/pose?aktion=start\|hier\|verwerfen` | `start`: Rider stands on the start point, pose jumps there; `hier`: shown pose is right; `verwerfen`: pose unknown (needs `[reloc] enabled`). 409 while lying, during `home`/`patrol`/`explore` |
| GET | `/detector` | selectable targets (detector classes), computed on request only |
| GET | `/horizon` | horizon table + `stand` (same as `/status` `horizon`); 409 if `[horizon]` off |
| POST | `/horizon/edit?row=..` | green line in the overlay (full-frame row), empty = off |
| POST | `/horizon/cal?row=..` | store row for the body height of the stand reference; 409 without valid stand reference |
| POST | `/horizon/delete?h=..` | delete entry at that height (±1 mm); 404 if none |

Mode rules: `explore`/`approach`/`home` need `freespace.enabled` (else 409); `follow`/`approach` need a running detector and a target in its classes (else 409); `home` also needs the room map; all modes except `idle`/`manual` need a running pipeline (else 409).

## Image transport (WebSocket)

| Item | Value |
|---|---|
| Path | `/xgonav/ws` (nginx → xgonav `/ws`, TLS terminates in nginx) |
| Message | binary: 4-byte frame index + JPEG; with `?nick=1` (used by `index.php`): index, 2 × float32 BE `row_stand`, `row_imu` (NaN = no stand reference), JPEG |
| Flow control | client sends `ok` per image; xgonav sends the next (newest) image only after ack |
| Ack timeout | 10 s → connection closed |
| Reconnect | client after 1 s |
| Pipeline state | client connects only while `pipeline` = `läuft` (and not sleep mode), closes otherwise; last image stays dimmed. xgonav sends an unchanged image once, no re-encode |
| Rate | ≤ `output.web_fps` (10) |
| Overlay | no status text lines in web frames (~10 ms saved per image) |

Replaced MJPEG: on weak Wi-Fi, TCP buffers queued up to 20 s of delay.

## Live data (WebSocket)

Daemon `tools/live.php` (PHP CLI, unit `xgonav-live.service`, user `www-data`, `Nice=10`), not under `web/`
(would be callable via php-fpm). Reads `/proc` directly, works without xgonav. One shared read per tick for all tabs.

| Item | Value |
|---|---|
| Path | `/xgonav/live` (nginx → `127.0.0.1:8089`) |
| Idle | no client: blocks in `stream_select`, no `/proc` reads, 0 % CPU |
| `sys` + `status`, 1 s (sleep mode 5 s) | `sys`: `t`, `cpu` `[[total, idle+iowait], …]` per core (jiffies), `temp_c`, `load`, `battery`, `height`, `roll`, `state`, `wlan` `"rx_bytes tx_bytes dBm quality"` (`-` = unknown); `status`: xgonav `/status` verbatim (timeout 0.5 s) or `null` + `status_fehler` |
| `imu`, 0.3 s if `state` = 1 (balancing), else 1 s; none in sleep mode | `seq`, `imu` `[[seq, t_ms, roll, pitch, yaw], …]` (°), `wseq`, `wheels` `[[seq, t_ms, ticks_left, ticks_right], …]`, `state`; only samples newer than the last sent per connection |
| Packing | `sys` and `imu` due together → one text frame `{"sys":…,"status":…,"imu":…}` |
| Client → server | `imu 0\|1 [since wsince]`: IMU on/off for this connection (Lage closed or tab hidden → 0); since/wsince after reconnect avoid duplicates |
| Reconnect | client after 1 s |
| Limit | `imu_log` holds 128 samples: at 1 s while lying, samples are skipped |
| Env (tests) | `XGO_PROC`, `XGO_LIVE_PORT`, `XGO_STATUS_URL` |

## Pipeline stop

`POST /pipeline?on=0`: camera stopped, no image processing, standstill, mode idle. CM4 CPU of the
service ~340 % → ~4 %. Web stays reachable. After start: drives only once a mode is chosen.
With full processing, the charging dock current is insufficient to charge.

## Sleep mode (Ruhe)

Header button **Ruhe** (with confirmation). Only the web UI wakes it (**Wecken**).

| Part | On | Wake |
|---|---|---|
| Rider | set down (action 254) if balancing | stays lying |
| xgonav | pipeline off, mode idle, raw recording off; `imu`, `wheels` threads idle; `robot` thread reads state/battery every `ruhe.tick_s` (1 s), no keepalive; no LCD, no emotions, no WebSocket re-encode; pipeline and modes other than idle → 409 | threads resume; pipeline stays off |
| Driver (`settings/`) | `imu_hz`, `imu_stream_hz`, `wheel_hz` → 0, `sleep_ms_on_loop` → `ruhe.sleep_ms_on_loop` (1000); old values kept in memory | old values written back |
| CPU (`xgo-ruhe`, root) | governor saved to `/run/xgo-ruhe.gov`, all cores `powersave` | saved governor (fallback `ondemand`) |
| Display (`api.php`) | plays `sleep` (~4 s, before the governor change); last frame (closed eyes) stays | unchanged until xgonav or an emotion writes the next frame |
| Page | cover with Wecken + battery; image WebSocket closed, no map polling; live packet every 5 s without IMU | normal rates |

- The display stays on: it cannot be switched off (DISPOFF + SLPIN via SIGUSR1 to `st7789_server` is no longer sent; `xgo-ruhe off` still sends SIGUSR2). Backlight (GPIO0 per `constants.h`) is not switched; untested.
- `POST ?a=ruhe&on=1` returns after the animation (~4 s).
- The ESP32 keeps running (control loop, BLE, amber LEDs).
- If xgonav restarts during sleep mode, the governor stays `powersave` until **Ruhe** + **Wecken**.

## Raw recording (`xgonav/rawrec.py`)

| Item | Value |
|---|---|
| Writes | JPEG (quality 92) + `meta.jsonl` per frame, format for `tools/replay_raw.py` and `tools/floornet/` |
| Target | `output.record_dir/YYYYMMDD-HHMMSS` (default `~/fahrt`, as service user) |
| Condition | switch on, state = balancing, pipeline running; only frames newer than the start |
| Set down | closes the recording; switch stays on, next stand-up starts a new subdirectory, no empty directories |
| Off by itself | service restart; free space < `output.record_min_free_mb` or write error (reason shown next to the button) |
| Cost on the Pi | off: no thread. On, lying: 5 attribute reads/s. Recording: one JPEG per frame in its own thread on `render_cpus`, never in the main loop; frames are skipped if it falls behind (gaps in `idx`) |

## Overlay colours

| Colour | Meaning |
|---|---|
| yellow | floor boundary per column from `method`; absent when FloorNet decides (`primary = "net"`) |
| cyan | horizon (pitch-compensated); with `[horizon]`: horizon from IMU only |
| white / magenta / green | `[horizon]`: stand horizon of this height / IMU + image pitch since last IMU re-anchor (used for distances) / line being set |
| orange | `[horizon]`: raw driver pitch (newest `imu_log` sample, no time alignment to the frame), value in degrees at the left |
| bottom bars | free space per sector |
| red | target track |
| violet | image motion |
| blue | drive command |

## Route, no-go zones, patrol

| Control | Effect |
|---|---|
| Route setzen | click points on the map, optional loop, `POST api.php?a=route&pts=x,y;x,y&loop=0\|1` |
| Sperrzone | click at least 3 corners, added to the existing zones (`zones=x,y;x,y;x,y\|...`) |
| Zonen löschen | `nozones=1` |
| Karte sperren | `POST api.php?a=map_lock&on=0\|1` |
| Streife | mode `patrol` after a confirmation dialog (`bestaetigt=1`) |

Coordinates are metres in the map frame (top = +x, left = +y). `GET api.php?a=route` returns points, loop, zones, `map_id`.
