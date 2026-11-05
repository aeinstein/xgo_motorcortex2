# Testing

PC only, no Pi needed.

```
python3 -m venv .venv && .venv/bin/pip install "opencv-python-headless>=4.6" "numpy>=1.24" pytest "onnxruntime>=1.16"
```

| Command | What | Time |
|---|---|---|
| `python -m pytest -q tests/` | 45 unit tests (`tests/test_units.py`); `slow` excluded via `pytest.ini` | ~2 s |
| `python -m pytest -q -m slow tests/` | 4 closed-loop simulator tests (`tests/test_closed_loop.py`) | ~150 s |
| `python -m pytest -q tests/test_units.py -k imu_sync` | single test | |
| `python tests/sim_run.py follow 40 [out.mp4]` | scenario, optional overlay video | |
| `python tests/sim_run.py explore 40` | explore scenario, prints collisions and map quality | |
| `python tests/sim_run.py home 60 [--wheels]` | 30 s explore, then `home`; prints true distance to start (`home_end_m`) | |
| `python tests/sim_run.py follow 40 --imu` | simulated IMU, sign −1 and −8 ms offset (deliberately wrong) | |
| `python tests/sim_run.py explore 40 --kanten` | any scenario with `freespace.method = "kanten"` | |
| `python -m xgonav --source video.mp4 --robot sim --mode follow [--fast]` | replay own video | |
| `python tools/replay_raw.py DIR --config rider.toml [--set k=v] [--video out.mp4]` | replay `--record-raw` capture through floor model, local map, arc check; reports false-alarm rate | |
| `driver-test/run.sh kernel_module 6 50 --faults` | driver code vs ESP32 emulator | ~6 s |

`replay_raw.py`: run in the directory containing `shading.json`.

## Simulator (`xgonav/sim.py`)

| Feature | Value |
|---|---|
| Rendering | raycasting, room with boxes, walking person |
| Camera motion | pitch oscillation 2.5 Hz + acceleration tilt, heading jitter, motion blur |
| FOV | 88.5° (default, = real RiderPi camera) |
| Detector | `FakeDetector`: ground-truth boxes, 180 ms latency, 5 Hz |
| Robot | `SimRobot`: integrates heading, logs writes; drive commands move the camera |
| Ground truth | `SimWorld.log`, `collisions` |

## Results (40 s runs)

| Scenario | Result |
|---|---|
| Explore | 7/7 runs without collision after fixes (before: 1 stuck in 11, cause: reverse not arc-checked) |
| Follow, person 0.3 m/s, target 1.2 m | mean 1.8 m, min 1.4 m, no collision |
| Home, 60 s, simulated wheels | end 0.14 m from true start, no collision, plan 2 ms (PC); without wheels camera heading drifts (pose error 0.5–1.8 m) |
| Pitch from image only | 0.56° spread |
| Pitch IMU + image | 0.13° spread |
| Person distance, image only | −16 cm bias, ±23 cm |
| Person distance, IMU + image | −3 cm bias, ±7 cm |
| IMU sync | finds sign −1 and −8 ms offset (correlation 0.99) |
| Ego-motion vs truth | 0.03° pitch/yaw error per frame |
| Driver vs ESP32 emulator | 49 Hz at 50 (block: 97 Hz at 100), pitch error 0.06° median, recovers from drops/noise/bad checksums |

## Tested environments

| Python | OpenCV | numpy |
|---|---|---|
| 3.13 | 5 | |
| 3.11 (Bookworm) | 4.6 | 1.26 |
