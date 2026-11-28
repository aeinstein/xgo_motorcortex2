# Face recognition (`xgonav/gesicht.py`)

Recognises learned faces in mode `pet` and greets them. Since 2026-11-27; first run on the Rider 2026-11-28 (lying, one person, recognised correctly).

## Pipeline

| Step | Implementation |
|---|---|
| Trigger | detector thread right after YOLO, only in `[gesicht] modes` (`pet`), only for `person` boxes whose track has no name yet, at most `max_hz` = 2 checks/s |
| Crop | head region of the person box (`gesicht.kopf_box`: upper half when standing, more for a squat box), ≤ 320 px to YuNet |
| Find | YuNet `face_detection_yunet_2023mar.onnx` via `cv2.FaceDetectorYN` (OpenCV ≥ 4.8, MIT), 5 landmarks; faces narrower than `min_face_px` = 40 are skipped |
| Align | 5 points → ArcFace template, `cv2.estimateAffinePartial2D` + `warpAffine` 112×112 |
| Features | MobileFaceNet `w600k_mbf` from InsightFace `buffalo_s`, ncnn (`models/mobilefacenet_ncnn`), RGB, (x − 127.5)/127.5, 512 values, L2-normalised. **Weights: non-commercial use only** |
| Match | cosine against every stored photo, maximum per name; ≥ `match_sim` = 0.5 → name |
| Track | `Detection.who` → `Track.who`; the name stays with the track until it is lost, named tracks are not checked again |
| Reaction | `pet`: `Seen.who` → emotion and sound of that person (`gruss.json`, set in the web card), else `greet_emotion` (`like`) / `greet_sound` (`traellern`); how depends on the absence (memory, below): ≥ `greet_kurz_s` 5 min only the sound, ≥ `greet_voll_s` 1 h emotion + sound, ≥ `greet_ausbruch_s` 8 h then `laugh` + `traellern` + wiggle (lying: no wiggle) 2.5 s later, never seen = full; awake and lying (state `liegt`: the greeting replaces the generic `surprised`/`gefunden`) |
| Memory | `xgonav/gedaechtnis.py`, `[pet] gedaechtnis` = `/var/lib/xgonav/gedaechtnis.json` (`""` = RAM only, simulator): per name trust, last and first seen (wall clock), encounters; one object in the app for all modes, written at most once a minute and on exit. Trust per counterpart: name → stored (start `trust_bekannt` 1.0; a startle by that person lowers it for good, calm nearness raises it), stranger → per track from `trust_fremd` 0.1 (RAM only), anyone else → session value from `trust_start` 0.3. The greeting no longer sets trust. Web card shows last seen, encounters, trust |
| Stranger | face seen but below `match_sim`: `Track.fremd_n` counts such checks while the track has no name; ≥ `[pet] fremd_n` 2 → stranger: trust 0.1 (more distance, startles sooner), `doubt` once per track |
| Eye colour | per person (`augen` in `gruss.json`, `tools/eyes.py` `PALETTE`): `PetBrain.augen` set before the greeting, held while recognised and `eye_hold_s` = 20 s after; xgonav sends `farbe NAME` to the eye generator, for living eyes and emotions ([display-emotions.md](display-emotions.md)); no per-frame cost |
| Display | overlay label shows the name instead of the class; `GET /status` → `tracks[].who`, `who_sim`, `gesicht_ms` |

## Cost on the CM4

| Item | Value |
|---|---|
| Frame loop | none (one field per track/`Seen`) |
| MobileFaceNet | 892 MFLOP per face; PC 9.7 ms (2 threads), CM4 estimated 70–100 ms (YOLO 256 px ≈ 1 GFLOP takes 93 ms) — **not measured** |
| YuNet | PC 2 ms per head crop |
| Worst case | 2 Hz × one face on `detector_cpus` (2–3), sequential with YOLO: detector rate drops while an unnamed face ≥ 40 px is visible |
| Learning | once per uploaded photo, in the web thread; learning mode see below |

## Models

`./tools/export_face.sh` on the PC (never on the Rider): downloads YuNet and `buffalo_s`, converts `w600k_mbf.onnx` with `pnnx` to ncnn and compares onnxruntime vs. ncnn on a test image (2026-11-27: cosine 0.999995). `deploy/install.sh` copies `models/*.onnx` and `models/*_ncnn`. Missing models or package `ncnn`: warning, recognition off.

## Learning (web card **Gesichter**)

| Item | Value |
|---|---|
| Upload | name (`[A-Za-z0-9_-]{1,32}`) + photos; browser shrinks to ≤ 1024 px JPEG → `api.php?a=gesicht_hochladen` → `POST /gesicht/hochladen?name=` |
| Stored | `[gesicht] dir` = `/opt/xgonav/gesichter/<name>/NN.npy` (features) + `NN.jpg` (aligned crop), ≤ 50 per name |
| List | `api.php?a=gesichter` reads the directory (only when opened); crops via `api.php?a=gesicht_jpg` |
| Delete | ✕ per photo or name → `POST /gesicht/loeschen?name=&nr=`; the last photo also removes the greeting |
| Greeting | per name: emotion (expressions of the eye generator, `/tmp/lcd0.augen.namen`), eye colour (`PALETTE` names) and sound (`pet_<event>_*.wav` or confirmed voice sample `stimme:NN`) → `api.php?a=gesicht_gruss` → `POST /gesicht/gruss?name=&emotion=&sound=&augen=` → `<name>/gruss.json`; empty = default, `-` = none. A voice sample can be replaced unless uploaded or 📌-protected |
| Advice | 10–20 photos, different light and angles, also from below; delete blurred or skewed crops |
| Rename | ✎ per name → `api.php?a=gesicht_umbenennen` → `POST /gesicht/umbenennen?name=&neu=`; existing name: photos merged (≤ 50), greeting of the target stays (else the moved one); memory entry follows (merged: encounters added, lower trust, latest last seen); tracks carrying the old name get the new one |

## Learning mode (mode `anlernen`, since 2026-11-28, unit tests only)

| Item | Value |
|---|---|
| Start | button **Anlernen** in the card → `api.php?a=gesicht_anlernen` → `POST /gesicht/anlernen` → `set_mode("anlernen")`; needs recognition on (models + detector); also lying (`LIEGEND_OK`) |
| Behaviour | stands still (`behaviors.Anlernen`), detector gets frames (`App.DET_MODES`); after the end `App._lern_fertig` returns to the mode before (follow/approach with its class), if not allowed any more: `idle`; any mode change cancels |
| Collect | detector thread after YOLO, ≤ `max_hz` (2/s): largest face ≥ `min_face_px` over all person boxes (`Gesichter.lern_bild`); dropped: cosine ≥ 0.97 to a candidate (near duplicate), < 0.3 to the candidates' mean (other person) |
| End | always after the full 30 s (`LERN_S`, user 2026-11-28: different poses); `LERN_KAND` = 80 only bounds RAM (over it the most redundant candidate is dropped; at 2 Hz at most 60); cancel: `POST /gesicht/anlernen?stop=1` |
| Choose | user 2026-11-28: at least 20, the most different: greedy farthest point (`gesicht.unaehnlichste`: start at the candidate closest to the mean, then the one with the smallest maximum similarity to those chosen); fewer than 20 → nothing stored |
| Store | `PersonN` (lowest free N) as `00..19.npy` + `.jpg`, written once in the detector thread; result names the most similar known name ≥ `match_sim` (mean over the new photos of the per-name maximum) |
| Status | `GET /status` → `anlernen`: running `{aktiv, n, ziel, max, rest_s, doppelt, fremd}`, else last result `{aktiv: false, ok, name, n, kandidaten, aehnelt, sim}` or `{ok: false, fehler}` |
| Cost on the CM4 | normal operation: one attribute check per detector run; while learning: YuNet + MobileFaceNet ≈ 130–145 ms per candidate at ≤ 2 Hz (≤ 60 × ≈ 8.4 s CPU in 30 s, YOLO rate drops meanwhile); end: ≤ 60 × 60 × 512 products + 20 × (2 KB + ~5 KB) writes once; candidates in RAM ≤ 80 × 39 KB |

## Limits

| Item | Value |
|---|---|
| Range | face ≥ 40 px: at f 310 px a 15 cm face is 46 px at 1 m, 23 px at 2 m → up to ≈ 1.2 m |
| Viewpoint | camera ≈ 13 cm above floor: a standing head is in the image only from ≈ 1.8 m, then too small → works when sitting, kneeling or bending down |
| Threshold | `match_sim` 0.5 (user, 2026-11-28); measured on Rider images lying (~40 s): known 0.45–0.55, unknown 0.19/0.31; photos from a phone differ from the Rider camera (angle, lens) |
| Cost | 129–145 ms per face on the CM4 (detector thread, ≤ `max_hz`, only persons without a name) |
| Privacy | features and crops stay on the Rider |
