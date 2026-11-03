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
