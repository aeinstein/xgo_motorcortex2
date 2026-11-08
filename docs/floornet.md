# FloorNet

Self-trained network for the floor boundary, one row per image column. **Status: shadow mode only.** It runs next to `kanten`, is drawn in the overlay in magenta and makes no decisions. The preset goal was missed (see below), so `kanten` still decides.

## Network

| Item | Value |
|---|---|
| Input | 640×480 BGR, rows 137–478, 2:1 average → 320×171 |
| Structure | 3×3 stem, 6 depthwise-separable blocks, rows collapsed, column head (1×1, 1×3, 1×1) |
| Output | 80 columns (8 px each), 64 row bins each: 1.5 px at the top, about 10 px at the bottom; best bin refined with a parabola |
| Size | 30 k parameters, 17.8 MMACs (`f2_s` in `bench_kern.cpp`) |
| Time, untrained, CM4 next to YOLO at 12 Hz | 13.6 ms mean, 17.3 ms p95; YOLO 119 → 136 ms |
| Inference | C++ with NEON (`xgonav/native/floornet.cpp`, `floornet_kern.h`), float32, no framework, self-test at load |

Constraints from the kernel: no ReLU and no bias after the depthwise convolution, residual before the ReLU of the 1×1 layer, 1×1 output channels divisible by 8. BatchNorm exists only in training; `net.fold` folds it into the weights.

## Files

| File | Purpose |
|---|---|
| `tools/floornet/label.py` | labels from the room map: `kanten` over a run → occupancy grid → raycast into every frame |
| `tools/floornet/handmap.py` | labels from hand lines: same raycast, map built only from hand lines |
| `tools/floornet/beschriften.html`, `make_testset.py` | click tool for hand lines, runs from the file system, stores in the browser, exports JSON |
| `tools/floornet/schnitt.py` | time window of a recording as its own run: `label.py` uses only the longest stretch without a pose jump (> 12 cm between frames) |
| `tools/floornet/eval_hand.py` | metric against hand lines, in metres per distance band, bootstrap over frames |
| `tools/floornet/comfy_label.py`, `comfy_batch.py`, `comfy_prompt8.txt` | labels from ComfyUI (Qwen-Image-Edit paints the floor green) against hand lines / for many frames, see below |
| `tools/floornet/net.py`, `train.py` | network, folding, training (GPU) |
| `tools/floornet/export.py` | weights and test vector for the C++ core (`models/floornet.bin`, gitignored) |
| `tools/floornet/bench_kern.cpp` | timing of untrained variants on the CM4 |
| `xgonav/floornet.py` | ctypes wrapper; off if the core is missing or the self-test fails |
| `tests/neon_shim/arm_neon.h` | the NEON intrinsics as plain C++, so `tests/test_floornet_native.py` runs without ARM (result only, not speed) |

## Workflow

```
python tools/floornet/label.py DIR... --config rider.toml --out labels/
python tools/floornet/make_testset.py labels/ --out pruefsatz/ --runs explore9:80,streife1:80,unten1:130,unten2:30
python tools/floornet/handmap.py hand.json --labels labels/ --config rider.toml --out labels_hand/
python tools/floornet/train.py --labels labels/ --out run/ --train explore2,explore3 --val "" --dist 0.005 --yaw 0.5 --epochs 100 \
    --hand_labels labels_hand/ --hand_train explore9,unten1 --far_atleast 1.25 --cap_atleast 0.6
python tools/floornet/eval_hand.py hand.json --labels labels/ --lauf run/:streife1,unten2
python tools/floornet/export.py run/floornet.pt --out models/floornet.bin
```

Then `[freespace] shadow_net = "models/floornet.bin"`, needs `threaded = true`. Training takes about a minute on the GPU.

## Labels

| Source | Finding |
|---|---|
| Room map from `kanten` (`label.py`) | Against hand lines under 1 m only 25–30 % of columns fit, 43–55 % are too far: free rays pass through dark furniture `kanten` does not see |
| Hand lines through the map (`handmap.py`) | Leave-one-out on 49 frames, under 1 m: 71 % fit, 20 % too far (room map: 33 %, 56 %) |

An obstacle in the room map needs hits from two standpoints at least 10 cm apart: a lamp reflection burned in during 72 s standing still. Fixed false alarms of `kanten` (shadows, sun patches) stay in those labels. Only hand lines remove them.

A run with hand lines in training must not be used to test that network. Frames used to select a variant do not count for the final test.

## Results

302 hand-labelled frames from four runs that `label.py` labels were never trained on; 287 usable. Share of columns under 1 m more than 15 cm off the hand line: *false free* behind it, *false blocked* in front of it. Ratio = network / `kanten`. Preset goal: false free ≤ 0.5 and false blocked ≤ 1.0.

**Final test, 2026-11-06:** 194 frames not seen before, networks frozen with 93 hand lines, mean of 3 training runs.

| | Network free / blocked | `kanten` free / blocked | Ratio |
|---|---|---|---|
| Other runs, same rooms | 19.2 % / 12.0 % | 30.7 % / 12.4 % | 0.62 / 0.96 |
| Room not in hand lines (trained upstairs, tested downstairs) | 22.9 % / 18.8 % | 42.1 % / 17.6 % | 0.55 / 1.07 |

Goal missed. Fewer false free is significant in all six measurements, false blocked is not distinguishable from `kanten`.

| Further measurements, not a clean test | Other runs | Unknown room |
|---|---|---|
| Cross-validation with all 287 lines (some frames selected the variant) | 0.54 / 0.85 | 0.42 / 1.06 |
| Minimum of network and `kanten`, 93 lines, exploratory | 0.37 / 1.57 | 0.41 / 1.41 |

| Known weakness | Value |
|---|---|
| Dark or glossy cupboards from close up | downstairs 0–0.5 m: 46 % false free (`kanten` 66 %) |
| False blocks from 1 m | 1–1.5 m: 48 % (`kanten` 39 %) |
| Variation between training runs | about ±0.05 to 0.1 in the ratio; never decide on one run |
| No effect | smoothing over 3 columns, brightness normalisation |

The network in `models/floornet.bin` is trained on all hand lines, so it has no clean test yet. That needs new frames.

## Shadow mode on the Rider

Final network since 2026-11-06. CM4 against PyTorch on 30 real frames: at most 0.0003 rows apart; self-test 4.8e-06.

| Measurement | Without network | With network |
|---|---|---|
| Idle, 24 fps: main loop mean / p95 | 8.1 / 14.1 ms | 9.8 / 15.8 ms |
| Idle: YOLO, CPU of xgonav | 120 ms, 154 % | 136 ms, 178 % |
| Idle: network | | 14.9 ms (moving average) |
| `explore` with raw recording, median / p90 | loop 16.6 / 31.8 ms, YOLO 133 ms (`unten1`) | loop 23.1 / 52.6 ms, YOLO 171 ms, network 20.4 / 29.4 ms (`unten3`) |

The drive rows come from different scenes and light; they are not a clean comparison. The plan goal p95 < 15 ms is missed while driving.

## Labels from ComfyUI (2026-11-08)

ComfyUI on kimmy (service `comfyui`, ~29 GB VRAM next to LM Studio) has only image generation and editing models, no segmentation or depth. Qwen-Image-Edit 2511 with the 4-step Lightning LoRA recolours the floor green; the green mask gives the boundary per column (lowest run of floor from the bottom, gaps up to 6 rows bridged). 3.6 s per frame, first load 2.5 min.

```
FN_PROMPT="$(cat comfy_prompt8.txt)" python comfy_label.py --hand teil3.json --labels labels5/ --n 1000 --out lauf8
python comfy_label.py --hand pruefsatz.json --labels labels_u3/ --ids ids.txt --hoehe pruefsatz3/hoehe.json --out p3
python comfy_batch.py --prompt comfy_prompt8.txt --runs explore2,...,unten2 --step 8 --out labels_comfy8
```

| Prompt rule | Finding (287 hand frames of the development set) |
|---|---|
| "recolour the floor, add nothing" | under 1 m false free 10–27 %, `kanten` 40–32 %; reflections and lamp glare become floor |
| naming rugs or "a small robot" | the model paints them into the frame (robot in ~50 % of frames): never name what should not appear |
| floor under furniture only where the gap is higher than 18 cm | under sofas, chair bases, low shelves median 83–123 cm → 4–32 cm |
| baseboards and skirting tiles are wall | tile skirting of the same material is no longer painted |
| rugs are not floor (yellow rug not drivable) | kept in 9 frames, an invented mat at the frame edge in 3 |
| floor ends at the front edge of sofa / chair base | no gain on 60 frames, worse on random ones: dropped |

Prompt 8 (`comfy_prompt8.txt`) against hand lines, false free / false blocked / median:

| Band | Development set, 287 frames | `kanten` | Clean test set 3, 62 frames so far | `kanten` |
|---|---|---|---|---|
| 0–0.5 m | 10.0 / 0.3 % / 1 cm | 40.2 / 1.3 % / 6 cm | 13.3 / 0.0 % / 1 cm | 60.1 / 0.1 % / 35 cm |
| 0.5–1 m | 23.0 / 3.4 % / 4 cm | 31.5 / 14.7 % / 13 cm | 22.7 / 1.3 % / 3 cm | 42.7 / 11.3 % / 17 cm |
| 1–1.5 m | 30.0 / 6.4 % / 9 cm | 34.3 / 35.2 % / 28 cm | 26.6 / 0.7 % / 8 cm | 47.8 / 23.6 % / 27 cm |
| 1.5–2.5 m | 41.0 / 8.8 % / 15 cm | 29.3 / 56.4 % / 47 cm | 29.3 / 7.2 % / 12 cm | 43.0 / 44.3 % / 70 cm |

The development set excludes columns under low furniture (hand lines there had no fixed rule). The prompts were tuned on the development set, so only test set 3 counts. Remaining false free: visible floor under overhanging sofas and chair bases. Hand-line rule since then: line where an 18 cm Rider would touch first; floor under tables counts; sofa, bed, cabinet, chair base: front edge; skirting is wall; rugs are obstacles; reflections, glare and seams are floor.

## Clean test set (2026-11-07)

Two daylight drives downstairs (`unten3`, `unten4`), never used in training. 220 frames for hand lines, ordered by body height:

| Group | Frames | Runs |
|---|---|---|
| Body height 85 mm (camera 128 mm) | 111 | `unten3` before 170 s, `unten4` (+ `unten4b`..`e`) |
| Body height 107 mm (web change during `unten3`, u8 203, firmware drives 200; camera ~150 mm) | 107 | `unten3` from 178 s, `unten3b`, `unten3c` |
| Height unclear (first seconds of `explore`) | 2 | `unten3` |

Runs split with `schnitt.py` at pose jumps (seconds from the first meta line): `unten3b` 396.4–end, `unten3c` 511–end, `unten4b` 117–174, `unten4c` 174–212.7, `unten4d` 212.7–255, `unten4e` 255–end. `eval_hand.py --hoehe pruefsatz3/hoehe.json` reports each group and converts rows to metres with the camera height of that group.
