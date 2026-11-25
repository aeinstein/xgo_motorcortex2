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
| `python -m pytest -q tests/test_xgo_ble.py` | `deploy/xgo-ble/xgo-ble.c` (BLE tunnel on the Pi, `docs/ble.md`) against a fake `xgo-wlan` and a fake xgonav: Wi-Fi requests, percent decoding, `nav` pass-through and limits, disconnect; needs Linux and gcc, skipped on the Windows PC | <1 s |
| `python -m pytest -q tests/test_live_ws.py` | `tools/live.php`: WebSocket rates (0.3 s / 1 s), IMU filter per connection, `imu 0`; skipped without `php` in PATH | ~10 s |
| `python -m pytest -q tests/test_sprech.py` | `tools/sprech.php`: only in mode manual, both directions with stand-ins for `arecord`/`aplay`, one call at a time, end on mode change, `laut 0`; skipped without `php` in PATH | ~5 s |
| `python -m pytest -q tests/test_navfast.py` | C++ core `native/navfast.cpp` against the Python paths, bit-identical: A* + shortening, `plan`, wavefront, frontier groups, `next_goal`, local map (move, enter, confirm, arc check), wall distance, path-follower target, room map (rays, entering, confirmation) (11 tests; skipped without the core). Passed on the RiderPi 2026-11-12 (aarch64, numpy 1.26). The ground geometry uses `atan2`/`tan` from libm; where numpy does not (x86 with AVX-512: own vector functions, last bit differs) the tests detect it (`LIBM`) and compare what depends on it to 1e-9 | ~5 s (CM4 ~26 s) |
| `XGONAV_FASTLIB=navfast.so python -m pytest -q tests/` | all tests through `navfast.cpp`, `gridfast.cpp`, `ortfast.cpp` and `stimmfast.cpp` (add it to either command; `test_native_gleich_numpy` in `tests/test_ortung.py` and `tests/test_stimme.py`, MSVC 2026-11-13/-19: equal to rounding; add the two .cpp files to the build line) on the PC. They need no OpenCV: `g++ -O2 -std=c++17 -fPIC -ffp-contract=off -shared -o navfast.so xgonav/native/navfast.cpp xgonav/native/gridfast.cpp`, or MSVC (x64 tools prompt) `cl /O2 /std:c++17 /EHsc /fp:precise /utf-8 /LD xgonav/native/navfast.cpp xgonav/native/gridfast.cpp /Fe:navfast.dll`. The other function groups then report a missing ABI and run in Python | |
| `python -m pytest -q tests/test_pet.py` | mode `pet`: synthetic distance tracks, budgets, every stand-up condition | <1 s |
| `python tests/sim_run.py pet 35 --wheels --imu [--animal person\|cat\|dog] [--speed 2.5] [--set pet.faint=True]` | mode `pet`: target walks straight at the Rider | |
| `python tests/sim_run.py follow 40 [out.mp4]` | scenario, optional overlay video | |
| `python tests/sim_run.py explore 40` | explore scenario, prints collisions and map quality | |
| `python tests/sim_run.py explore 120 --wheels --dunkel 3.5,3,6,6` | dark zone (x0,y0,x1,y1[,rest light 0.04]): fear of the dark, prints `dunkel` (`angst`, `licht`, `aufgeben`, `zonen`, `ruhe`) | 2026-11-13: 1 fright, light found, 0 collisions; whole room dark: gives up, sleep mode. 2026-11-20, 240 s: without dark zone (`--set dunkel.zone_s=0`) 3 frights, gives up; with it 1 fright, 1 zone, map finished after 179 s, 0 collisions |
| `python tests/sim_run.py home 60 [--wheels]` | 30 s explore, then `home`; prints true distance to start (`home_end_m`) | |
| `python tests/sim_run.py follow 70 --scene zwei --wheels --dyn` | person walks through the door and turns behind the wall: lost, drive to the last position, find again (`suche` = searches, found) | |
| `python tests/sim_run.py explore 300 --scene fuge --wheels --fs-thread --dyn --rider` | room with invisible floor joints (`SimScene.rillen`, axle passes only with command ≥ `rille_v` 0.5): one across the room, one around the right box with 0.12 m left to it (joint in front of a step); checks the fast push | |
| `python tests/sim_run.py follow 40 --imu` | simulated IMU, sign −1 and −8 ms offset (deliberately wrong) | |
| `python tests/sim_run.py home 60 --wheels --dyn` | any scenario with Rider-like inertia (turn lag, dead band, short roll-back on start; speed follows the command with 1.2 s, a 0 command brakes with 0.25 s, fitted to the 2026-11-23 run) | |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_yaw_coupling.cpp $S/control/rider_leg_imu.cpp` | firmware host test, patches 0005/0006 (attitude while turning); `S=firmware/build/xgorider2_rl/firmware/src` after `firmware/build.sh` | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_turn_hold.cpp $S/control/rider_motion.cpp $S/control/pid.cpp` | firmware host test, patch 0008 (position hold while turning) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_yaw_bias.cpp $S/control/rider_leg_imu.cpp` | firmware host test, patches 0009/0011 (gyro-z bias tracking, no yaw dead band) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_roll_trim.cpp $S/control/rider_leg_imu.cpp` | firmware host test, patch 0012 (roll trim kept after leveling) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_diag.cpp $S/control/rider_leg_imu.cpp $S/command_protocol.cpp` | firmware host test, patch 0013 (diagnostic register 0x10) | <1 s |
| `g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_stream.cpp $S/command_protocol.cpp` | firmware host test, patch 0014 (IMU/wheel data stream) | <1 s |
| `python -m xgonav --source video.mp4 --robot sim --mode follow [--fast]` | replay own video | |
| `python tools/replay_raw.py DIR --config rider.toml [--set k=v] [--netz models/floornet.bin] [--video out.mp4]` | replay `--record-raw` capture through FloorNet, local map, arc check; reports false-alarm rate (needs the C++ core) | |
| `python tools/map_replay.py DIR --config rider.toml --out OUT [--variante name:key=val,...] [--nur]` | room map of one raw recording, once per variant (pose time offset, turn-rate gate, `grid.*`, roll, IMU heading at frame time); `OUT/DIR_karten.png` side by side, `OUT/DIR.json` with occupied cells and cells between walls ≤ 20 cm apart. `roll=imu` and `kurs=imu` need recordings with IMU samples in `meta.jsonl` (`imu`, `raeder` per frame) | |
| `python tools/map_replay.py DIR --config rider.toml --out OUT --nur --variante abgleich:abgleich=1 [--netz models/floornet.bin]` | same recording without and with running map matching (`xgonav/reloc.py`, `reloc.X=val` tunes it); `--netz`: floor boundary from FloorNet alone, as on the Rider (needs the C++ core) | |
| `python tools/reloc_eval.py suche DIR --config rider.toml [--netz ...] [--fenster 15] [--mit-stueck]` | global search: cut a piece out of a raw recording, build the map from the rest, search the piece in it; counts right / wrong / unsure. `--mit-stueck` keeps the piece in the map (upper bound) | |
| `python tools/reloc_eval.py bench [--karte raumkarte.npz]` | timing of field, local match, global search; run on the Pi | |
| `python tools/map_eval.py reloc B.pkl --karte A.pkl [--schlupf 0.03 --dreh 0.02 --kursdrift 0.1]` | simulator, against ground truth: drive B with drifting odometry matched against the finished map of drive A (same scene), pose error without / with matching; global search of pieces of B in map A. Without `--karte`: matching while the map is being built | |
| `python -m pytest -q tests/test_reloc.py` | map matching on synthetic maps: finds the pose, reports "unsure" for identical corners, corridor, too little wall | <1 s |
| `driver-test/run.sh kernel_module 6 50 --wheels --combo --faults` | driver code vs ESP32 emulator (without `--wheels --combo` the driver does not poll IMU/wheels) | ~6 s |
| `driver-test/run.sh kernel_module 6 50 --wheels --combo --vel` | drive frame (firmware 0029): `VELPARSE` = text ↔ bytes of `xgo-vel.h` (valid/invalid inputs, state text), `VELTEST` = limits + 3 frames reach the emulator byte for byte with checksum, state read back; needs Linux (kimmy) | ~6 s |

Simulator floor boundary: `sim.SimFloorNet` takes it from the ground truth of each rendered frame (walls, boxes, person); FloorNet only knows real frames. `--phantoms n` adds false alarms. Maps and control are tested against the truth, FloorNet itself only on recordings (`docs/floornet.md`).

FloorNet tests (`tests/test_floornet_*.py`): those that need torch are skipped on the PC; run them on the GPU machine. `test_floornet_native.py` builds the C++ core there with `tests/neon_shim` (result only, not speed).

## Simulator (`xgonav/sim.py`)

| Feature | Value |
|---|---|
| Rendering | raycasting, room with boxes, walking person |
| Camera motion | pitch oscillation from IMU samples of seven RiderPi drives (2026-11-10/11, ~1060 s at 92 Hz): driving 1.0° at 0.8 Hz, 0.6° at 2.5 Hz, 0.2° at 6.1 Hz, 0.1° at 9.3 Hz, 0.05° at 13.1 Hz, standing a quarter of that (`sim.NICK`, `STAND_FACTOR`); acceleration tilt 9°/(m/s²), clipped at 0.5 m/s²; heading jitter 0.8° at 1.7 Hz (standing a quarter); motion blur. Until 2026-11-11: 3° at 2.5 Hz, 1.5° at 6.1 Hz, also standing |
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
| Driver vs ESP32 emulator | polling 0x0d: 49 Hz at 50, pitch error 0.06° median, recovers from drops/noise/bad checksums |

## Tested environments

| Python | OpenCV | numpy |
|---|---|---|
| 3.13 | 5 | |
| 3.11 (Bookworm, RiderPi service) | 4.10 from `~/.local` (system 4.6) | 1.26 from `~/.local` (system 1.24) |
