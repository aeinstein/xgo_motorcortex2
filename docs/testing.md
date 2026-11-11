# Testing

PC only, no Pi needed.

```
python3 -m venv .venv && .venv/bin/pip install "opencv-python-headless>=4.6" "numpy>=1.24" pytest "onnxruntime>=1.16"
```

| Command | What | Time |
|---|---|---|
| `python -m pytest -q tests/` | 56 unit tests (`tests/test_units.py`; 5 need the C++ core, skipped without it); `slow` excluded via `pytest.ini` | ~2 s |
| `python -m pytest -q -m slow tests/` | 4 closed-loop simulator tests (`tests/test_closed_loop.py`) | ~150 s |
| `python -m pytest -q tests/test_units.py -k imu_sync` | single test | |
| `python -m pytest -q tests/test_live_ws.py` | `tools/live.php`: WebSocket rates (0.3 s / 1 s), IMU filter per connection, `imu 0`; skipped without `php` in PATH | ~10 s |
| `python -m pytest -q tests/test_pet.py` | mode `pet`: synthetic distance tracks, budgets, every stand-up condition | <1 s |
| `python tests/sim_run.py pet 35 --wheels --imu [--animal person\|cat\|dog] [--speed 2.5] [--set pet.faint=True]` | mode `pet`: target walks straight at the Rider | |
| `python tests/sim_run.py follow 40 [out.mp4]` | scenario, optional overlay video | |
| `python tests/sim_run.py explore 40` | explore scenario, prints collisions and map quality | |
| `python tests/sim_run.py home 60 [--wheels]` | 30 s explore, then `home`; prints true distance to start (`home_end_m`) | |
| `python tests/sim_run.py follow 40 --imu` | simulated IMU, sign −1 and −8 ms offset (deliberately wrong) | |
| `python tests/sim_run.py explore 40 --kanten` | any scenario with `freespace.method = "kanten"` | |
| `python tests/sim_run.py home 60 --wheels --dyn` | any scenario with Rider-like inertia (turn lag, dead band, short roll-back on start) | |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_yaw_coupling.cpp $S/control/rider_leg_imu.cpp` | firmware host test, patches 0005/0006 (attitude while turning); `S=firmware/build/xgorider2_rl/firmware/src` after `firmware/build.sh` | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_turn_hold.cpp $S/control/rider_motion.cpp $S/control/pid.cpp` | firmware host test, patch 0008 (position hold while turning) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_yaw_bias.cpp $S/control/rider_leg_imu.cpp` | firmware host test, patches 0009/0011 (gyro-z bias tracking, no yaw dead band) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_roll_trim.cpp $S/control/rider_leg_imu.cpp` | firmware host test, patch 0012 (roll trim kept after leveling) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_diag.cpp $S/control/rider_leg_imu.cpp $S/command_protocol.cpp` | firmware host test, patch 0013 (diagnostic register 0x10) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_stream.cpp $S/command_protocol.cpp` | firmware host test, patch 0014 (IMU/wheel data stream) | <1 s |
| `python -m xgonav --source video.mp4 --robot sim --mode follow [--fast]` | replay own video | |
| `python tools/replay_raw.py DIR --config rider.toml [--set k=v] [--video out.mp4]` | replay `--record-raw` capture through floor model, local map, arc check; reports false-alarm rate | |
| `python tools/map_replay.py DIR --config rider.toml --out OUT [--variante name:key=val,...] [--nur]` | room map of one raw recording, once per variant (pose time offset, turn-rate gate, `grid.*`, roll, IMU heading at frame time); `OUT/DIR_karten.png` side by side, `OUT/DIR.json` with occupied cells and cells between walls ≤ 20 cm apart. `roll=imu` and `kurs=imu` need recordings with IMU samples in `meta.jsonl` (`imu`, `raeder` per frame) | |
| `python tools/map_replay.py DIR --config rider.toml --out OUT --nur --variante abgleich:abgleich=1 [--netz models/floornet.bin]` | same recording without and with running map matching (`xgonav/reloc.py`, `reloc.X=val` tunes it); `--netz`: floor boundary from FloorNet alone, as on the Rider (needs the C++ core) | |
| `python tools/reloc_eval.py suche DIR --config rider.toml [--netz ...] [--fenster 15] [--mit-stueck]` | global search: cut a piece out of a raw recording, build the map from the rest, search the piece in it; counts right / wrong / unsure. `--mit-stueck` keeps the piece in the map (upper bound) | |
| `python tools/reloc_eval.py bench [--karte raumkarte.npz]` | timing of field, local match, global search; run on the Pi | |
| `python tools/map_eval.py reloc B.pkl --karte A.pkl [--schlupf 0.03 --dreh 0.02 --kursdrift 0.1]` | simulator, against ground truth: drive B with drifting odometry matched against the finished map of drive A (same scene), pose error without / with matching; global search of pieces of B in map A. Without `--karte`: matching while the map is being built | |
| `python -m pytest -q tests/test_reloc.py` | map matching on synthetic maps: finds the pose, reports "unsure" for identical corners, corridor, too little wall | <1 s |
| `driver-test/run.sh kernel_module 6 50 --faults` | driver code vs ESP32 emulator | ~6 s |

`replay_raw.py`: run in the directory containing `shading.json`.

FloorNet tests (`tests/test_floornet_*.py`): those that need torch are skipped on the PC; run them on the GPU machine. `test_floornet_native.py` builds the C++ core there with `tests/neon_shim` (result only, not speed).

## Simulator (`xgonav/sim.py`)

| Feature | Value |
|---|---|
| Rendering | raycasting, room with boxes, walking person |
| Camera motion | pitch oscillation 2.5 Hz + acceleration tilt, heading jitter, motion blur |
| FOV | 91.8° (default, = real RiderPi camera) |
| Detector | `FakeDetector`: ground-truth boxes, 180 ms latency, 5 Hz |
| Robot | `SimRobot`: integrates heading, logs writes; drive commands move the camera |
| Ground truth | `SimWorld.log`, `collisions` |

## Results (40 s runs)

| Scenario | Result |
|---|---|
| Explore | frontier mapping, to „Karte fertig“: 6/6 without collision, see `docs/mapping.md` |
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
| 3.11 (Bookworm, RiderPi service) | 4.10 from `~/.local` (system 4.6) | 1.26 from `~/.local` (system 1.24) |
