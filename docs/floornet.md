# FloorNet

Self-trained network for the floor boundary, one row per image column. **Status: decides on the RiderPi since 2026-11-10 (`primary = "net"`, `kanten` no longer computed there), not driven yet; default is shadow mode.** In shadow mode it runs next to `kanten`, is drawn in the overlay in magenta and makes no decisions. The network trained on ComfyUI labels (2026-11-09, on the Rider since then) reaches the preset goal on other runs of the same rooms but misses it in a room it has not seen (see below), so `kanten` still decides.

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
| `tools/floornet/stage2.py`, `inseln.py` | green edits → masks (stage 1 → 2, `floor_mask`, `--gruen` for the green lead); count and show islands and holes in masks, changes nothing |
| `tools/floornet/stage3.py` | stage 2 → 3: per pixel column only what is unbroken from the bottom edge; no islands or holes remain |
| `tools/floornet/stage3_npz.py` | stage 3 → label files for `train.py`: one row per 8 px strip (majority), filters as `comfy_npz.py` |
| sorting out (from stage 2 on) | nothing is deleted. Standing frames: pose in `meta.jsonl` less than 3 cm and 3° from the last kept frame (`stage2.py --steht_m --steht_grad`, 0 = off; 2026-11-11: 9 208 of 21 906 frames stay). Unusable frames: `<stage1>/aus_hand.txt`, one id per line, `-id` restores, written by the viewer. Hand list first, then standing frames among the rest. Rules travel in `info.json` of stage 2 and 3; `index.js` (`BILDER`, `AUS`), `stage3.py` and `stage3_npz.py` apply them |
| `tools/floornet/betrachter.html`, `betrachter_server.py` | original, stage 1–3 side by side, arrow keys step through frames; list from `<stage>/index.js`. `X`/button marks the frame unusable (or restores it), `A` shows sorted-out frames too. Served from `~/floornet` by `betrachter_server.py` (static files + `POST /aus`, port 8765, home network only, no auth) |
| `tools/floornet/ids_fahrt.py` | frame list of raw recordings for `comfy_batch.py --ids`: every n-th frame, standing frames dropped |
| `tools/floornet/comfy_npz.py` | `rows.jsonl` of `comfy_batch.py` → label files for `train.py`; drops frames lying down, tilting (> 10 % of columns above row 205), torn masks (> 6 jumps > 40 px), dark (mean grey < 12) |
| `tools/floornet/hand_npz.py` | hand lines → label files holding only frames with their own line, no map; for `train.py --hand_only 1` |
| `tools/floornet/vergleich_fahrten.py` | network and `kanten` as deciders over recorded runs: jumps, flicker, stops, path blocks, maps; no Rider needed |
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

Then `[freespace] shadow_net = "models/floornet.bin"`, needs `threaded = true`. Training takes about a minute on the GPU. `[freespace] primary = "net"` lets the network decide alone (`method` is then not computed and not drawn, it only takes over if the network fails). RiderPi: set since 2026-11-10, not driven.

Spike removal in the decoder (`[freespace] net_outlier_px`, default 25, 0 = off; `net.decode(logits, outlier_px, outlier_ratio)` in PyTorch, `decode` in `native/floornet.cpp`, both tested against each other): the raw output has 1–3 column wide jumps at vertical high-contrast edges, the output there is bimodal. A column further than `net_outlier_px` rows from the median of the 7 columns around it takes the best bin within ±2 bins of that median, if the probability mass there (5 bins) is at least `net_outlier_ratio` (0.3) of the mass at its own peak. Columns without a second peak keep their value (narrow obstacles). Cost on the Rider: a 7-value median per column, `exp` only in outlier columns.

| explore10–12 (never trained), 4551 frames | < 1 m falsely free / blocked | frames with spike down / up |
|---|---|---|
| plain decode | 5.4 / 6.2 % | 6.1 / 2.1 % |
| spike removal | 5.4 / 5.8 % | 1.0 / 0.6 % |

Spike = column more than 25 rows from the 7-column median; the count includes real narrow obstacles. Measured 2026-11-10 with the deployed net against Stage3 labels; on the Rider not yet run.

With ComfyUI labels (`comfy_batch.py`, see below), replacing `label.py` and `handmap.py`:

```
python tools/floornet/comfy_npz.py labels_comfy8/rows.jsonl --labels labels/ --out labels_npz/
python tools/floornet/hand_npz.py hand.json --labels labels/ --runs unten3,unten3b --out labels_hand_mix/
python tools/floornet/train.py --labels labels_npz/ --out run/ --train explore2,unten1 --val "" --dist 0.005 --yaw 0.5 --epochs 100 \
    --far_w 2 --hand_labels labels_hand_mix/ --hand_train unten3,unten3b --hand_only 1 --hand_repeat 8
python tools/floornet/eval_hand.py hand.json --labels labels/ --lauf run/:unten4 --hoehe pruefsatz3/hoehe.json
```

All `kanten` figures on this page were measured with the `kanten` row shifted up by the former `[grid] row_bias` (1.5 rows = 6 px unless stated, Rider −1 = −4 px). `row_bias` was removed on 2026-11-11; the tools now compare the raw `kanten` row, so new figures are not comparable.

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

The network of 2026-11-06 was trained on all hand lines, so it has no clean test of its own. The same recipe, `lauf_end`, on test set 3 is in the next section: 0.59 / 1.56.

## Results with ComfyUI labels (2026-11-09)

Same network size. Labels: Prompt 8 on every 8th frame of all runs (9763 labelled, 9196 after filters, 0 failures). Selection only on the hand lines of `unten4*`, final test once on `unten3*`, each network trained without the run it is tested on. Selected from `far_w` 1, 2, 4 × hand lines test set only / plus development set, 3 training runs each: `far_w` 2 with both sets (selection 0.25 / 0.35).

**Final test on `unten3*`, 131 frames, under 1 m, range over 3 training runs:**

| | False free | False blocked | Ratio free [95 %] | Ratio blocked [95 %] |
|---|---|---|---|---|
| Network | 21.1–22.0 % | 3.0–3.8 % | | |
| `kanten`, shifted +6 px (former `row_bias` 1.5) | 49.4 % | 7.3 % | 0.43–0.45 [0.28–0.62] | 0.41–0.53 [0.25–0.93] |
| `kanten`, shifted −4 px (former Rider `row_bias` −1) | 47.4 % | 19.4 % | 0.45–0.46 [0.28–0.67] | 0.15–0.20 [0.09–0.32] |
| Older network `lauf_end`, vs. `kanten` shifted +6 px | 29.4 % | 11.3 % | 0.59 [0.44–0.77] | 1.56 [1.08–2.39] |

Goal reached by the point estimate. False blocked is clearly under 1.0; for false free the interval still reaches above 0.5.

| Band | Network free / blocked / median | `kanten` free / blocked / median |
|---|---|---|
| 0–0.5 m | 28.8–30.7 / 0.7–0.9 % / 2 cm | 63.3 / 0.3 % / 67 cm |
| 0.5–1 m | 16.6–17.5 / 4.2–5.7 % / 4 cm | 41.2 / 11.4 % / 16 cm |
| 1–1.5 m | 12.6–14.4 / 8.1–8.5 % / 6–7 cm | 28.4 / 39.7 % / 23 cm |
| 1.5–2.5 m | 16.5–17.6 / 28.7–34.9 % / 13–16 cm | 25.0 / 62.7 % / 45 cm |

**Room not seen (exploratory, after the final test, same variant, 3 training runs):**

| Trained on | Tested on | Network free / blocked | `kanten` free / blocked | Ratio |
|---|---|---|---|---|
| upstairs only, 2 hand-line runs | test set 3, 200 frames | 39.3–42.9 / 4.7–5.3 % | 52.4 / 6.0 % | 0.75–0.82 / 0.77–0.87 |
| same, `kanten` shifted −4 px (former Rider `row_bias` −1), 1 training run | same | 41.4 / 4.8 % | 50.0 / 18.5 % | 0.83 / 0.26 |
| downstairs only | `explore9`, `streife1`, 138 frames, older hand-line rule | 16.5–22.2 / 9.3–14.2 % | 22.3 / 6.0 % | 0.74–1.00 / 1.54–2.37 |

Goal missed in both directions. Both floors are still one home; a foreign home is not measured.

**False free up to 0.5 m, `unten3*`:** 24 of 37 frames have any, 5 frames hold 73 %. Where a column is wrong it is far off: median 114 cm too far, p90 689 cm.

| Rank | Frame | Network wrong (of columns) | Teacher wrong | Cause |
|---|---|---|---|---|
| 1 | `unten3b/010218` | 65 / 70 | 70 | tile-clad step painted as floor by the teacher |
| 2 | `unten3/004838` | 61 / 65 | 54 | same step |
| 3 | `unten3b/009758` | 57 / 64 | 41 | floor under an overhanging sofa |
| 4 | `unten3/005280` | 51 / 58 | 0 | tile-clad step from close by: network jumps to a tile joint |
| 5 | `unten3c/012531` | 25 / 54 | 0 | plain wall from close by, columns jump |
| 6 | `unten3b/009538` | 21 / 32 | 0 | step from close by |
| 7 | `unten3/003751` | 14 / 73 | 0 | plain wall filling the frame, columns jump |

Ranks 1–3 (52 %) are teacher errors the network learned. Ranks 4–9 (37 %) are network errors where the teacher is right; the four viewed (ranks 4–7) show a plain surface from close by, with floor only in part of the frame. So the network is no fall or step protection, and it is weakest directly in front of plain walls. These frames have now been looked at, so `unten3*` is no longer a clean test for a further variant.

Stage 3 network (2026-11-10): prompt 10, input G0h, mask green lead ≥ 30, stage 3; labels `stage3_labels` (13 331 of 13 530 frames, 19 runs), trained on all runs without hand lines (`--far_w 2`, 8 489 frames after thinning), `floornet_s3.bin`, on the Rider as `models/floornet.bin` since 2026-11-10 09:38 (backup of `floornet_d.bin` in `/root/floornet.bin.vor-stage3-*`). C++ against PyTorch: x86, 240 frames, at most 0.0007 rows. Hand lines below 1 m, net vs `kanten` (false free / false blocked): development set 16.9 / 1.8 % vs 34.5 / 11.5 % (ratio 0.49 / 0.15); test set 3 15.0 / 1.7 % vs 42.0 / 9.2 % (0.36 / 0.18). Not clean: all runs were in training (labels from ComfyUI, not from the hand lines). `lauf_end` (2026-11-06, trained on the 287 development lines) on the same sets: 6.6 / 1.2 % and 20.1 / 4.9 %; on the unseen-for-it runs at 85 mm 38.4 / 9.4 % against 11.6 / 1.7 % for the stage 3 network.

Previous final network on all runs and all 488 hand lines: `floornet_d.bin`, 139652 bytes, on the Rider as `models/floornet.bin` 2026-11-09 to 2026-11-10. C++ against PyTorch: x86, 240 frames, at most 0.0005 rows apart; CM4, 30 frames with raw pixels, at most 0.0001 rows; self-test 4.1e-06. Cost on the Pi against the previous network, lying down, 24 fps: `net_ms` 11.1–11.4 before, 10.9–11.3 after, with one reading of 16.3. Not yet measured while driving. Geometry of these labels is the old one (f 328.49), so the metre bands compare methods but are not exact distances.

## Shadow mode on the Rider

The table below is the network of 2026-11-06; the current one has the same structure. CM4 against PyTorch on 30 real frames: at most 0.0003 rows apart; self-test 4.8e-06.

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
python comfy_batch.py --prompt comfy_prompt8.txt --ids ids_alle.txt --edits edits_p8 --nur_edits --out edits_p8   # missing edits only
```

`comfy_batch.py` keeps every edit (`<edits>/<run>/<frame>.png`), so masks can be recomputed without the model. Seed fixed (`SEED` = 1, as for all Prompt 8 masks).

Chosen 2026-11-09 by inspecting edits: Prompt 10 (`comfy_prompt10.txt`; Prompt 9 without its rug sentence, which made the model invent dark mats in dark frames, while the real rugs in 4 test frames stayed unpainted without it) with `--eingang g0h` (green channel 0, larger of R and B set to the grey value; colour kept for rugs, dark green-tinted frames not black), mask with hue and saturation as before and green lead G − max(R, B) ≥ 30 (`stage2.py --gruen 30`), then `stage3.py`. The green channel alone (G > 0) fails: the image decoder leaves small green values on unpainted pixels. Low thresholds count the faint green shimmer the model adds to walls next to the floor (`unten2/002400`: walls up to 48, floor from 170). Every input variant invents something somewhere; no hand-line numbers for this input yet.

Prompt 11 (2026-11-11, `comfy_prompt11.txt`): Prompt 10 painted rugs and things lying on the floor green. Added: "Only the bare floor surface itself is recolored. Everything that lies or stands on it, including rugs, stays unchanged." Chosen by the user on 73 test frames against two variants with the old rug sentence (both invented mats in dark frames and cut out the flat chair mat, which counts as floor). Same prompt for bright (colour) and dark (G0h) frames; run `stage1_p11` over all frames that are not standing frames (9 731).

| Prompt rule | Finding (287 hand frames of the development set) |
|---|---|
| "recolour the floor, add nothing" | under 1 m false free 10–27 %, `kanten` 40–32 %; reflections and lamp glare become floor |
| naming rugs or "a small robot" | the model paints them into the frame (robot in ~50 % of frames): never name what should not appear |
| floor under furniture only where the gap is higher than 18 cm | under sofas, chair bases, low shelves median 83–123 cm → 4–32 cm |
| baseboards and skirting tiles are wall | tile skirting of the same material is no longer painted |
| rugs are not floor (yellow rug not drivable) | kept in 9 frames, an invented mat at the frame edge in 3 |
| floor ends at the front edge of sofa / chair base | no gain on 60 frames, worse on random ones: dropped |

Prompt 8 (`comfy_prompt8.txt`) against hand lines, false free / false blocked / median:

| Band | Development set, 287 frames | `kanten` | Clean test set 3, 200 frames | `kanten` |
|---|---|---|---|---|
| 0–0.5 m | 10.0 / 0.3 % / 1 cm | 40.2 / 1.3 % / 6 cm | 14.6 / 0.0 % / 1 cm | 66.8 / 0.2 % / 46 cm |
| 0.5–1 m | 23.0 / 3.4 % / 4 cm | 31.5 / 14.7 % / 13 cm | 14.9 / 1.5 % / 2 cm | 42.5 / 10.1 % / 16 cm |
| 1–1.5 m | 30.0 / 6.4 % / 9 cm | 34.3 / 35.2 % / 28 cm | 22.5 / 1.0 % / 5 cm | 35.2 / 32.2 % / 23 cm |
| 1.5–2.5 m | 41.0 / 8.8 % / 15 cm | 29.3 / 56.4 % / 47 cm | 28.1 / 5.4 % / 10 cm | 29.4 / 58.5 % / 48 cm |

The development set excludes columns under low furniture (hand lines there had no fixed rule). The prompts were tuned on the development set, so only test set 3 counts. Test set 3: 217 of 220 frames labelled, 16 unusable, 1 without known height.

95 % bootstrap over frames for false free, Prompt 8 / `kanten`: 5.5–25.1 / 57.1–75.3, 9.9–20.2 / 36.4–49.0, 17.6–27.6 / 30.0–40.5, 23.7–32.8 / 25.1–33.9. The bands separate up to 1.5 m, not beyond.

False free means more than 15 cm too far, which is only ~2.5 rows at 1.5 m and ~1.5 rows at 2 m. Counting only columns also more than 4 rows off gives 14.6 / 14.9 / 20.7 / 14.3 % (`kanten` 66.8 / 42.5 / 34.3 / 25.3 %).

Up to 0.5 m the false free columns are concentrated: 16 of 77 frames have any, 5 frames hold 81 %:

| Frame | Share of false free up to 0.5 m | Cause |
|---|---|---|
| `unten4c/004368` | 21 % | grey band in front of a white wall: hand line treats it as obstacle, model and `kanten` as floor; unclear which is right |
| `unten3b/010218` | 19 % | tile-clad step seen from close by, painted as floor |
| `unten3/004838` | 15 % | same step |
| `unten4e/006587` | 14 % | yellow rug in glare painted as floor (hand line at its edge, `kanten` close to it) |
| `unten3b/009758` | 11 % | floor visible under an overhanging sofa from close by |

So the model can paint a step riser as floor; these labels are not a substitute for fall or step protection.

Further on: visible floor under overhanging sofas and chair bases. `floor_mask` requires V ≥ 30 since 2026-11-09. It was V ≥ 90 for all numbers on this page: floor painted dark green in shadow counted as blocked, and `mask_rows` stops at the first gap, so `explore2/000000` lost up to 82 rows. An earlier summed comparison on 136 frames showed no net gain for V ≥ 50 and hid this; below V 30 the hue is noise.

Hand-line rule since then: line where an 18 cm Rider would touch first; floor under tables counts; sofa, bed, cabinet, chair base: front edge; skirting is wall; rugs are obstacles; reflections, glare and seams are floor.

## Clean test set (2026-11-07)

Two daylight drives downstairs (`unten3`, `unten4`), never used in training. 220 frames for hand lines, ordered by body height:

| Group | Frames | Runs |
|---|---|---|
| Body height 85 mm (camera 128 mm) | 111 | `unten3` before 170 s, `unten4` (+ `unten4b`..`e`) |
| Body height 107 mm (web change during `unten3`, u8 203, firmware drives 200; camera ~150 mm) | 107 | `unten3` from 178 s, `unten3b`, `unten3c` |
| Height unclear (first seconds of `explore`) | 2 | `unten3` |

Runs split with `schnitt.py` at pose jumps (seconds from the first meta line): `unten3b` 396.4–end, `unten3c` 511–end, `unten4b` 117–174, `unten4c` 174–212.7, `unten4d` 212.7–255, `unten4e` 255–end. `eval_hand.py --hoehe pruefsatz3/hoehe.json` reports each group and converts rows to metres with the camera height of that group.
