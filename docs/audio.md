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

## Mixer state (as found, not changed)

| Control | Value |
|---|---|
| `Capture` | 63/63 (+30 dB), on, both channels |
| `Left/Right Boost Mixer L/RINPUT1` | on |
| `Left Input Mixer Boost` | on |
| `Left Input Boost Mixer LINPUT1` | 3/3 (+29 dB) |
| `ADC PCM` | 195/255 (0 dB) |

Read with `amixer -c wm8960soundcard sget <control>`; note the values before changing any of them.
