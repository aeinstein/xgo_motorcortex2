# Face recognition (`xgonav/gesicht.py`)

Recognises learned faces in mode `pet` and greets them. Since 2026-11-27, unit tests only, **not run on the Rider**.

## Pipeline

| Step | Implementation |
|---|---|
| Trigger | detector thread right after YOLO, only in `[gesicht] modes` (`pet`), only for `person` boxes whose track has no name yet, at most `max_hz` = 2 checks/s |
| Crop | head region of the person box (`gesicht.kopf_box`: upper half when standing, more for a squat box), ≤ 320 px to YuNet |
| Find | YuNet `face_detection_yunet_2023mar.onnx` via `cv2.FaceDetectorYN` (OpenCV ≥ 4.8, MIT), 5 landmarks; faces narrower than `min_face_px` = 40 are skipped |
| Align | 5 points → ArcFace template, `cv2.estimateAffinePartial2D` + `warpAffine` 112×112 |
| Features | MobileFaceNet `w600k_mbf` from InsightFace `buffalo_s`, ncnn (`models/mobilefacenet_ncnn`), RGB, (x − 127.5)/127.5, 512 values, L2-normalised. **Weights: non-commercial use only** |
| Match | cosine against every stored photo, maximum per name; ≥ `match_sim` = 0.45 (provisional) → name |
| Track | `Detection.who` → `Track.who`; the name stays with the track until it is lost, named tracks are not checked again |
| Reaction | `pet`: `Seen.who` → emotion and sound of that person (`gruss.json`, set in the web card), else `greet_emotion` (`like`) / `greet_sound` (`traellern`); trust = 1.0; again only after `greet_repeat_s` = 600 s without recognising that name; awake and lying (state `liegt`: the greeting replaces the generic `surprised`/`gefunden`) |
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
| Threshold | `match_sim` not tuned on Rider images; photos from a phone differ from the Rider camera (angle, lens) |
| Privacy | features and crops stay on the Rider |
