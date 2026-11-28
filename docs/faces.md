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
| Eye colour | per person (`augen` in `gruss.json`, `tools/eyes.py` `PALETTE`): `PetBrain.augen` set before the greeting, held while recognised and `eye_hold_s` = 20 s after; emotions from `emotions_<colour>/`, pet eyes from `eyes_gaze_<colour>/` ([display-emotions.md](display-emotions.md)); no per-frame cost |
| Display | overlay label shows the name instead of the class; `GET /status` → `tracks[].who`, `who_sim`, `gesicht_ms` |

## Cost on the CM4

| Item | Value |
|---|---|
| Frame loop | none (one field per track/`Seen`) |
| MobileFaceNet | 892 MFLOP per face; PC 9.7 ms (2 threads), CM4 estimated 70–100 ms (YOLO 256 px ≈ 1 GFLOP takes 93 ms) — **not measured** |
| YuNet | PC 2 ms per head crop |
| Worst case | 2 Hz × one face on `detector_cpus` (2–3), sequential with YOLO: detector rate drops while an unnamed face ≥ 40 px is visible |
| Learning | once per uploaded photo, in the web thread |

## Models

`./tools/export_face.sh` on the PC (never on the Rider): downloads YuNet and `buffalo_s`, converts `w600k_mbf.onnx` with `pnnx` to ncnn and compares onnxruntime vs. ncnn on a test image (2026-11-27: cosine 0.999995). `deploy/install.sh` copies `models/*.onnx` and `models/*_ncnn`. Missing models or package `ncnn`: warning, recognition off.

## Learning (web card **Gesichter**)

| Item | Value |
|---|---|
| Upload | name (`[A-Za-z0-9_-]{1,32}`) + photos; browser shrinks to ≤ 1024 px JPEG → `api.php?a=gesicht_hochladen` → `POST /gesicht/hochladen?name=` |
| Stored | `[gesicht] dir` = `/opt/xgonav/gesichter/<name>/NN.npy` (features) + `NN.jpg` (aligned crop), ≤ 50 per name |
| List | `api.php?a=gesichter` reads the directory (only when opened); crops via `api.php?a=gesicht_jpg` |
| Delete | ✕ per photo or name → `POST /gesicht/loeschen?name=&nr=`; the last photo also removes the greeting |
| Greeting | per name: emotion (`/opt/xgonav/emotions`), eye colour (rendered colours, `eyes_gaze_*`) and sound (`pet_<event>_*.wav` or confirmed voice sample `stimme:NN`) → `api.php?a=gesicht_gruss` → `POST /gesicht/gruss?name=&emotion=&sound=&augen=` → `<name>/gruss.json`; empty = default, `-` = none. A voice sample can be replaced unless uploaded or 📌-protected |
| Advice | 10–20 photos, different light and angles, also from below; delete blurred or skewed crops |

## Limits

| Item | Value |
|---|---|
| Range | face ≥ 40 px: at f 310 px a 15 cm face is 46 px at 1 m, 23 px at 2 m → up to ≈ 1.2 m |
| Viewpoint | camera ≈ 13 cm above floor: a standing head is in the image only from ≈ 1.8 m, then too small → works when sitting, kneeling or bending down |
| Threshold | `match_sim` 0.5 (user, 2026-11-28); measured on Rider images lying (~40 s): known 0.45–0.55, unknown 0.19/0.31; photos from a phone differ from the Rider camera (angle, lens) |
| Cost | 129–145 ms per face on the CM4 (detector thread, ≤ `max_hz`, only persons without a name) |
| Privacy | features and crops stay on the Rider |
