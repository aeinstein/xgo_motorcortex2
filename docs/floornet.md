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
| `tools/floornet/eval_hand.py` | metric against hand lines, in metres per distance band, bootstrap over frames |
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
