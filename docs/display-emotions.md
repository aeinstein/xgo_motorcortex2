# Display and emotions

## Display daemon `st7789_server`

| Item | Value |
|---|---|
| Source | `st7789/` (libdriver/st7789), build in `st7789/project/riderpi` |
| Panel | 2", ST7789, 320×240 |
| Input | FIFO `/tmp/lcd0`, raw RGB565, 153 600 bytes per frame |
| Start | `deploy/rc.local` (or `st7789_display.service` via `make install` — not both) |
| Permissions | `/tmp/lcd0` group `video`, g+w |
| Throughput | > 40 fps (measured) |

```
make -C st7789/project/riderpi && sudo cp st7789/project/riderpi/st7789_server /usr/bin/
```

## xgonav overlay on the LCD (`xgonav/lcd.py`)

| Item | Value |
|---|---|
| Enabled | `--robot proc` and `output.lcd_fifo` ≠ `""` (`--no-lcd` disables) |
| Rate | `output.lcd_fps` = 15 (no measurable main-loop cost on CM4) |
| Pauses | while an emotion is playing (`/tmp/lcd0.owner` non-empty, < 10 s old) |

## Emotion player (`tools/emotion.sh`)

```
tools/emotion.sh build SRC DST      # SRC/<Name>/<Name><n>.png -> DST/<Name>/0001.raw ... (numeric order)
tools/emotion.sh play DIR [FPS]     # plays DIR/*.raw once, default 25 fps
```

| Item | Value |
|---|---|
| Source PNGs | `rc_server/emotions/<Name>/` (name `[A-Za-z]+`) |
| Converted | `/opt/xgonav/emotions` by `deploy/install.sh`; RiderPi: 13 emotions, 900 frames, 134 MB, 6 min build |
| Fallback | no `.raw` files → decodes PNGs with ffmpeg |
| CPU (RiderPi) | 55 frames ~0.18 s (old ffmpeg path: 1.04 s for 24 frames) |
| Ownership | newest player writes its PID to `/tmp/lcd0.owner`; older players exit before next frame (works across users) |
| `/tmp/lcd0.owner` | created by `rc.local`: root, group `video`, 0664 (`fs.protected_regular`) |

## Eye animations (`tools/eyes.py`)

Generated alternative to the rc_server PNGs: stylized eyes only, one color on black (antialiased), same names and output format as `emotion.sh build`.

```
python tools/eyes.py OUTDIR [--color R,G,B] [--only NAME]... [--png] [--preview DIR]
python tools/eyes.py --list
```

| Item | Value |
|---|---|
| Output | `OUTDIR/<Name>/0001.raw ...`, 320×240 RGB565 LE, 153 600 bytes (same as `emotion.sh build`) |
| Length | 55–100 frames at 25 fps (2.2–4.0 s), 915 frames / 139 MB in total |
| Loop | first and last frame = neutral eyes (except `sleep`: ends closed) |
| Color | default turquoise `0,225,210` |
| `--png` | also writes `<Name><n>.png` (rc_server source layout) |
| `--preview DIR` | contact sheet `<Name>.png` and `<Name>.mp4` per emotion |
| Runtime | ~7 s on a PC for all 13; needs only numpy + OpenCV |
| Install | e.g. `python3 tools/eyes.py /opt/xgonav/emotions` (replaces the converted rc_server emotions; `rc_server/emotions/<Name>/` must still exist for the web UI list) |

| Emotion | Eyes |
|---|---|
| `Angry` | lids slanted down toward the center, trembling, two pulses |
| `cute` | round eyes with big pupils and highlights, head tilt, short `^ ^` squint |
| `doubt` | one eye raised, other squinted, glance right then left |
| `embarrassed` | darting glances left/right, then trembling `> <` |
| `grievance` | round pleading eyes looking up, sad lids, quivering lower lid, tear |
| `hate` | eye roll upward, then narrowed frowning slits, side glance |
| `laugh` | `^ ^` arcs bouncing |
| `like` | heart eyes beating |
| `sad` | lids drooping outward, eyes sink, slow blink |
| `shy` | smaller half-closed eyes looking down/aside, one glance back |
| `sleep` | lids get heavy, two jerks awake, closed `‿ ‿`, breathing |
| `surprised` | eyes pop to large circles (overshoot), double blink |
| `vertigo` | rotating spirals circling in opposite phase |

## Automatic emotions

| Emotion | Trigger | Source |
|---|---|---|
| `cute` | boot | `rc.local` |
| `sleep` | shutdown from web UI (before `poweroff`, timeout 10 s) | `api.php` |
| `surprised` | person appears after ≥ `gap_s` (5 s) without person | `xgonav/emotion.py` |
| `doubt` | mode `follow`, no person for `lost_s` (2 s); once per loss | `xgonav/emotion.py` |
| `embarrassed` | other modes, no person for `lost_s`; once per loss | `xgonav/emotion.py` |
| `shy` | battery ≤ `battery_low` (20 %), repeated every 300 s | `xgonav/emotion.py` |

- xgonav emotions only with `--robot proc` and `emotion.enabled`.
- Pipeline stopped → person unknown → battery trigger only.
- Emotion already playing → trigger waits (not interrupted).
- Manual: web UI → Extras → Emotionen (`POST api.php?a=emotion&name=..`).
