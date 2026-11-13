# Audio (speaker and microphones)

Sound card `wm8960soundcard` (WM8960 codec, I²S), ALSA card 0, device `hw:0,0`, playback and capture. Sounds played by xgonav: [display-emotions.md](display-emotions.md#sounds).

## Microphones

Measured 2026-11-13 on the RiderPi, quiet room.

| Item | Value |
|---|---|
| Count | 2 microphones, stereo (left = `LINPUT1`, right = `RINPUT1`) |
| Record | `arecord -D hw:0,0 -f S16_LE -r 48000 -c 2 out.wav` (works while pipewire/wireplumber run as `aeinstein`) |
| Silence | both channels ≈ −27 dBFS noise at full gain, no zero samples; channels uncorrelated (0.09) |
| Noise spectrum | 50–66 % below 200 Hz (hum / structure-borne), ~28 % above 12 kHz, whine at 15.9 kHz in both channels |
| Own speaker (`freude.wav`, 1.0–1.8 kHz) | +43 dB (×145) over silence on both channels, L/R correlation 0.99, same level |
| Placement | close together: equal level and near-identical signal from the own speaker; direction at best coarse |

## Sound direction (`xgonav/ortung.py`)

Time difference left/right from the phase of the cross spectrum (GCC-PHAT, 300–2000 Hz), angle from `asin(c·(τ − tau_offset) / mic_dist_m)`. The microphones are acoustically only ≈ 4 mm apart (τ ≤ ±11 µs, one sample = 21 µs): usable only averaged over ≈ 1 s, i.e. for speech and steady sounds, not for a single clap.

| Item | Value |
|---|---|
| Output | `GET /status` → `ortung.letzter`: `winkel_deg` (0 = centre, > 0 left), `tau_us`, `guete` 0..1, `pegel_db`, `alter_s`; `ortung.verlauf` (last sounds, for the radar card in the web page, [web-interface.md](web-interface.md)); in code `app.ortung.last` (`Schall`) |
| Limits | two microphones: left/right only, front and back are the same (a source at 150° reads as 30°) |
| Active | `[ortung] modes` (`idle`, `pet`), never in sleep mode or during Wi-Fi setup; `arecord` on `hw:0,0` runs only then (device opens once: `arecord` by hand fails meanwhile; mode `manual` frees it) |
| Gate | per 1024-sample block (21 ms) one level value (band ≈ 0.7–3.5 kHz) against a tracked noise floor: opens at `on_db = 9`, stays open down to `off_db = 3`, pauses ≤ `hang_blocks` (0.25 s) do not end a sound. A steady sound is followed for ≈ 15 s, then counts as silence (`noise_up`) |
| Estimate | open gate: one FFT per block (both channels as one complex signal), cross spectrum summed over ≤ 47 blocks (1 s), weighted `|S|^-0.8`, tested against 41 delays within ±17 µs, parabola at the peak; sounds < 0.5 s dropped |
| Own sounds | ignored while `aplay` from xgonav runs and `nachhall_s = 0.3` after; sounds started elsewhere (web, `rc.local`) are not |
| Cost | separate `arecord` process (0.1 % of a core) plus one thread; C++ core `native/ortfast.cpp` (status `kern`). CM4 at 600 MHz (`powersave`, 2026-11-13): 39 µs per quiet block (≈ 0.2 % of a core in silence), 146 µs per FFT block (≈ 0.7 % while a sound lasts), 211 µs per estimate; numpy fallback 317 / 284 / 722 µs. C++ equal to numpy there (max. difference 3e-19 s). Nothing in the frame loop |
| Tools | `python tools/ortung.py live` on the Rider (xgonav in sleep mode or a mode outside `[ortung] modes`), `tools/ortung.py wav file.wav` for a stereo recording; test tone for a phone: band noise 300–2000 Hz, 1.5 s on / 0.5 s off |

### Calibration 2026-11-13 (balancing, test tone 1 m ahead, turned on the spot in 8 steps)

`τ = 11.2 µs · sin(angle) − 2.2 µs`, residual 0.6 µs (300–2000 Hz, 1 s windows) → `mic_dist_m = 0.0038`, `tau_offset_us = -2.2`. Scatter per estimate: 0.13 s ±6 µs, 0.5 s ±3 µs, 1 s ±1–2 µs, 2 s ±0.7–1.3 µs.

| Source at | −1° | −44° | −97° | −150° | +162° | +119° | +71° | +22° |
|---|---|---|---|---|---|---|---|---|
| expected (front/back folded) | −1° | −44° | −83° | −30° | +18° | +61° | +71° | +22° |
| reported, median of 9–10 | +2° | −56° | −90° | −32° | +21° | +57° | +60° | +23° |

Same code on the recordings **set down** with the TV 2–3 m away (smooth, sloping walls): 90° left +15°, 45° left +16°, ahead +14°, 90° right −24°. Lying down only "right" stands out; use the direction while balancing.

Above 2 kHz the channels are not phase-true: at 3.5–4.5 kHz the phase sits at ≈ ±165° whatever the direction (coherence 0.4–0.7). With the first band (500–5000 Hz, 0.13 s windows) this produced apparent delays of −80…−100 µs for every source; those angles meant nothing.

## Mixer state (as found, not changed)

| Control | Value |
|---|---|
| `Capture` | 63/63 (+30 dB), on, both channels |
| `Left/Right Boost Mixer L/RINPUT1` | on |
| `Left Input Mixer Boost` | on |
| `Left Input Boost Mixer LINPUT1` | 3/3 (+29 dB) |
| `ADC PCM` | 195/255 (0 dB) |

Read with `amixer -c wm8960soundcard sget <control>`; note the values before changing any of them.
