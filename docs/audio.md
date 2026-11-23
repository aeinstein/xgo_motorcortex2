# Audio (speaker and microphones)

Sound card `wm8960soundcard` (WM8960 codec, I²S), ALSA card 0, device `hw:0,0`, playback and capture. Sounds played by xgonav: [display-emotions.md](display-emotions.md#sounds).
Intercom with the browser (mode manual, 8 kHz mono both ways, `plughw:0,0`): [web-interface.md](web-interface.md#intercom-websocket).

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

## Voice samples (`xgonav/stimme.py`, mode `pet`)

The pet learns phrases it hears repeatedly and parrots them back, filtered to sound like a budgie. Not yet tried on the Rider (2026-11-19).

| Item | Value |
|---|---|
| Capture | on the `[ortung]` stream (`Ortung.horcher`), only in `pet` (balancing or lying) while a person is in view or gone ≤ `[pet] parrot_listen_s` (3 s); never during own sounds (+ `nachhall_s`) |
| VAD | ortung level vs. noise floor: opens `vad_on_db` 6, ends after `vad_hang_s` 0.6 s below `vad_off_db` 4 (hang-over; 0.3 s split phrases); `vad_pre_s` 0.3 s pre-roll; 0.3–3 s (cut at `max_s`; 0.6 s lost short phrases like "Komm her") |
| Features | 48 kHz stereo → 16 kHz mono (FIR 7 kHz), 13 MFCC per 10 ms, edges more than `trim_db` 12 dB below the loudest frame trimmed (with 20 dB the noise of pre-roll/hang-over stayed in), mean removed; rejected if < `voiced_min` 0.4 of the frames within `laut_db` 10 dB of the loudest are voiced: flatness 300–3000 Hz < `flat_max` 0.3 **and** pitch 70–400 Hz (autocorrelation from the power spectrum, window-corrected) > `per_min` 0.5. Rider 2026-11-19 (kettle running): speech 0.45–1.0, kettle alone 0.12–0.29, running water 0.0–0.26 (with flatness only and 0.3, water noise got learned) |
| Match | DTW (Sakoe-Chiba 25 %, length ratio ≤ 1.6), cost per step ≤ `match_dist` 8. Labelled recording on the Rider (3 phrases × 5, kettle running, `tools/stimme.py strom`): same phrase 5.1–8.2, different phrases ≥ 8.8, mostly 9.5–12. Windows TTS: same phrase other rate 2–4, other phrases ≥ 14 |
| Learning | no match → candidate (≤ `pool_n` 30); a candidate not confirmed within `kandidat_max_min` 1 min is deleted (thread checks every 5 s); match with a candidate → confirmed (louder take kept), filtered, parroted after `parrot_delay_s` 0.2 s (after the 0.6 s hang-over: ≈ 0.9 s after the end of speech); match with a confirmed one → parroted again, every time. At `max_confirmed` 10 a new confirmation replaces the confirmed sample heard least often (tie: longest not heard), never an uploaded one; all 10 uploaded → nothing is confirmed (`lerne` → `voll`); counts in `bestaetigt/zaehler.json` |
| Upload | card **Samples**: any audio file, the browser decodes it, mixes to mono, resamples to 16 kHz and cuts to `max_s` 3 s (`OfflineAudioContext`), sends WAV int16; `Stimmen.hochladen` queues it as confirmed without the voiced check (gate, budgie filter, features like a learned one), not parroted right away. Option *already edited* (`fertig=1`): 48 kHz, ≤ 8 s, stored unchanged as `NN.wav`, `NN_roh.wav` = downsampled to 16 kHz (list, features). Uploaded samples are fixed (`bestaetigt/fest.json`, user 2026-11-23): never replaced, only ✕ deletes them; any confirmed sample can be fixed or released with 📌 (`POST a=stimme_fest&name=NN&an=0|1` → xgonav `POST /stimme/fest`, `Stimmen.festsetzen`); with a free slot it takes that, else it replaces the learned one heard least often, all fixed → 409 |
| Budgie filter | noise gate first (`gate_db` 12 below the loudest frame, `knee_db` 6, silent edges cut; without it 57–86 % of a sample was kettle noise and sounded like gurgling water), `pitch_st` +12 semitones at the same tempo (WSOLA: stretch ×2 with 30 ms pieces matched by cross-correlation ±10 ms, then play ×2 faster; formants move up too), ×`speed` 1.0 (user: not faster), vibrato 25 Hz ±0.3 ms, high-pass 1 kHz + low-pass 5 kHz, tanh drive 2, peak −3 dB, 10 ms fades; once per confirmation |
| Playback | when talking back (`brabbeln`) a learned sample instead with `parrot_p` 0.5, ≥ `parrot_repeat_s` 30 s apart, least played first (sound `stimme:NN`) |
| Files | `[stimme] dir` (`/opt/xgonav/stimmen`): `kandidaten/<time>.wav` (16 kHz raw), `bestaetigt/NN_roh.wav`, `NN.wav` (48 kHz filtered), `fest.json` (uploaded or protected NN); delete the directory to forget everything |
| Web | card **Samples**: `api.php?a=stimmen` lists the files (disk only, works without xgonav), `a=stimme_wav&name=NN\|<time>&roh=0\|1` serves one WAV for the browser; `POST a=stimme_spielen&name=NN` → xgonav `POST /stimme/spielen` (sound `stimme:NN`, sound location ignores it), `POST a=stimme_loeschen&name=…` → xgonav `POST /stimme/loeschen` (`Stimmen.loeschen`: files + memory + counts; 400 bad name, 404 missing), `POST a=stimme_hochladen&fertig=0|1` (body WAV, ≤ 1000 KB) → xgonav `POST /stimme/hochladen`; list field `fest`. Cost on the Pi only on request (upload: gate + filter + features of ≤ 3 s once, C++ core, web thread) |
| Status | `GET /status` → `ortung.stimme` (`aufnahme`, `bestaetigt`, `kandidaten`, `voll`, `fest`, `segmente`, `verworfen`, `wieder`); pet info `nachgeplappert`, `proben` |
| Cost | per 21 ms block only while capturing: a few comparisons and keeping the block; per utterance C++ core `native/stimmfast.cpp` in thread `stimme` (CM4: 3 s features ≈ 21 ms before the pitch check, DTW ≈ 2 ms per sample); filter once per confirmation (≈ 6 ms per s). numpy fallback equal to rounding (1e-13). Nothing in the frame loop |
| Tools | `python tools/stimme.py sittich in.wav out.wav [--set speed=1.8]`, `vergleich a.wav b.wav …` (distance table), `lernen *.wav` (whole flow in a temp dir), `strom long.wav [--aus dir]` (cut a long 48 kHz stereo recording like on the Rider, distance table; record in mode `manual`: `arecord -D hw:0,0 -f S16_LE -r 48000 -c 2 -d 120 x.wav`) |

## Mixer state (as found, not changed)

| Control | Value |
|---|---|
| `Capture` | 63/63 (+30 dB), on, both channels |
| `Left/Right Boost Mixer L/RINPUT1` | on |
| `Left Input Mixer Boost` | on |
| `Left Input Boost Mixer LINPUT1` | 3/3 (+29 dB) |
| `ADC PCM` | 195/255 (0 dB) |

Read with `amixer -c wm8960soundcard sget <control>`; note the values before changing any of them.

Speaker (changed 2026-11-21, user listening test):

| Control | Was | Now | Why |
|---|---|---|---|
| `Speaker AC Volume` / `Speaker DC Volume` (class-D boost, 0–5) | 4 / 4 | 0 / 0 | boost 4 distorted speech badly at any level (also quiet); tones/emotions hid it. Boost 1 better, only boost 0 was really intelligible |
| `Speaker Playback Volume` | 104 at runtime (109 in the state file) | 112 | 108 compensated the −4.4 dB of the lower boost, voice was still too quiet: +4 dB (emotions too) |

Boot: `/usr/bin/wm8960-soundcard` links `/var/lib/alsa/asound.state` → `/etc/wm8960-soundcard/wm8960_asound.state` and runs `alsactl restore`, so the values live in that file (backup `….bak-boost-20261121-2303`). `alsactl store` would replace the link with a plain file, which the next boot deletes. `alsactl restore` also sets `Capture` to the file's 39; at runtime it is 63 (set at boot outside this repo, probably wireplumber) — set it back with `amixer -c 0 cset name='Capture Volume' 63,63`.
