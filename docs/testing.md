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
| `firmware/tests/run.sh [name]` | firmware host tests (g++, no ESP32), one or more per patch 01-ble … 08-velocity; `name` filters by test or patch name; after `firmware/build.sh` | <1 s each |
| `python -m xgonav --source video.mp4 --robot sim --mode follow [--fast]` | replay own video | |
| `python tools/replay_raw.py DIR --config rider.toml [--set k=v] [--netz models/floornet.bin] [--video out.mp4]` | replay `--record-raw` capture through FloorNet, local map, arc check; reports false-alarm rate (needs the C++ core) | |
| `python tools/map_replay.py DIR --config rider.toml --out OUT [--variante name:key=val,...] [--nur]` | room map of one raw recording, once per variant (pose time offset, turn-rate gate, `grid.*`, roll, IMU heading at frame time); `OUT/DIR_karten.png` side by side, `OUT/DIR.json` with occupied cells and cells between walls ≤ 20 cm apart. `roll=imu` and `kurs=imu` need recordings with IMU samples in `meta.jsonl` (`imu`, `raeder` per frame) | |
| `python tools/map_replay.py DIR --config rider.toml --out OUT --nur --variante abgleich:abgleich=1 [--netz models/floornet.bin]` | same recording without and with running map matching (`xgonav/reloc.py`, `reloc.X=val` tunes it); `--netz`: floor boundary from FloorNet alone, as on the Rider (needs the C++ core) | |
| `python tools/reloc_eval.py suche DIR --config rider.toml [--netz ...] [--fenster 15] [--mit-stueck]` | global search: cut a piece out of a raw recording, build the map from the rest, search the piece in it; counts right / wrong / unsure. `--mit-stueck` keeps the piece in the map (upper bound) | |
| `python tools/reloc_eval.py bench [--karte raumkarte.npz]` | timing of field, local match, global search; run on the Pi | |
| `python tools/map_eval.py reloc B.pkl --karte A.pkl [--schlupf 0.03 --dreh 0.02 --kursdrift 0.1]` | simulator, against ground truth: drive B with drifting odometry matched against the finished map of drive A (same scene), pose error without / with matching; global search of pieces of B in map A. Without `--karte`: matching while the map is being built | |
| `python -m pytest -q tests/test_reloc.py` | map matching on synthetic maps: finds the pose, reports "unsure" for identical corners, corridor, too little wall | <1 s |
| `driver-test/run.sh kernel_module 6 50 --wheels --combo --faults` | driver code vs ESP32 emulator (without `--wheels --combo` the driver does not poll IMU/wheels) | ~6 s |
| `driver-test/run.sh kernel_module 6 50 --wheels --combo --vel` | drive frame (firmware patch 08-velocity): `VELPARSE` = text ↔ bytes of `xgo-vel.h` (valid/invalid inputs, state text), `VELTEST` = limits + 3 frames reach the emulator byte for byte with checksum, state read back; needs Linux (kimmy) | ~6 s |

Simulator floor boundary: `sim.SimFloorNet` takes it from the ground truth of each rendered frame (walls, boxes, person); FloorNet only knows real frames. `--phantoms n` adds false alarms. Maps and control are tested against the truth, FloorNet itself only on recordings (`docs/floornet.md`).

FloorNet tests (`tests/test_floornet_*.py`): those that need torch are skipped on the PC; run them on the GPU machine. `test_floornet_native.py` builds the C++ core there with `tests/neon_shim` (result only, not speed).

## Simulator (`xgonav/sim.py`)

| Feature | Value |
|---|---|
| Rendering | raycasting, room with boxes, walking person |
| Scenes (`--scene`) | `raum` 6 × 6 m, two boxes; `zwei` two rooms, door 0.8 m; `spalt` door 0.25 m; `fuge` invisible floor joints; `unten` ground floor reconstructed from the runs since 2026-11-23 (lamp light), `unten_tag` the same in daylight |
| Extended scenes (`SimScene.teile`, `flaechen`, `boden`) | own path in `SimWorld._ext_bild`: per column the 10 nearest faces drawn back to front, parts with their own height z0..z1, one texture per side, floating parts (table top, seats) leave the floor visible, only `fest` parts collide, slanted faces via `flaechen`; old scenes render bit-identically |
| Scene `unten` (`xgonav/welten/unten.npz`, 2 MB) | 10 × 9 m bounding box: galley kitchen, dining nook (table, four chairs, drivable underneath), main room with wardrobe wall, slanted wall, TV lowboard, rug, sofas, stairs, window platform; 42 parts read off the fused map (±10 cm, not measured), photo textures per face (2 cm per texel) and floor image from the camera frames, tile joints assumed 120 × 60 cm; start pose = charging spot. ~21 ms per frame on the PC |
| Camera motion | pitch oscillation from IMU samples of seven RiderPi drives (2026-11-10/11, ~1060 s at 92 Hz): driving 1.0° at 0.8 Hz, 0.6° at 2.5 Hz, 0.2° at 6.1 Hz, 0.1° at 9.3 Hz, 0.05° at 13.1 Hz, standing a quarter of that (`sim.NICK`, `STAND_FACTOR`); acceleration tilt 9°/(m/s²), clipped at 0.5 m/s²; heading jitter 0.8° at 1.7 Hz (standing a quarter); motion blur. Until 2026-11-11: 3° at 2.5 Hz, 1.5° at 6.1 Hz, also standing |
| FOV | 91.8° (default, = real RiderPi camera) |
| Detector | `FakeDetector`: ground-truth boxes, 180 ms latency, 5 Hz |
| Robot | `SimRobot`: integrates heading, logs writes; drive commands move the camera |
| Ground truth | `SimWorld.log`, `collisions` |

### Scene from recorded runs (`tools/simwelt/`, GPU machine only)

Builds `xgonav/welten/<name>.npz` from raw recordings (`--record-raw`). Nothing of it runs on the Rider.

| Step | Script | What it does |
|---|---|---|
| 1 | `punkte.py CODE DATA OUT run… --jedes 3` | MoGe-2 point cloud per frame in the Rider frame; scale and attitude from the fitted floor plane (`tools/floornet/geo_label.py`), ~22 frames/s |
| 2 | `kompass.py DATA PKT OUT run` | heading per frame from wall directions (modulo 90°, tracked with gain 0.08); removes the yaw drift of the run (4–15° per run seen), path re-integrated |
| 3 | `karte.py DATA PKT OUT run --rmax 1.5 --posendir KOMP` | top-down maps per run, 2 cm cells: floor colour, obstacle colour, height histogram (10 cm bins) |
| 4 | `reg.py KARTEN OUT REF run… --recht` | lays the runs onto a reference run (multiples of 90° ± 3°, shift by FFT correlation); `summe.py` adds the registered maps |
| 5 | `plan.py KARTE.npz OUT --ursprung=-1,-5 [--modell welt.json]` | large plots (colour, walls, heights) in world coordinates, model overlaid; `sicht.py X,Y` shows the frames taken at a place |
| 6 | `modell.py > welt.json` | the model: boxes with height, slanted faces, chairs, rug (edited by hand from the plots) |
| 7 | `textur.py sammeln …`, `textur.py bauen …` | texture per face: per texel the colour from the single best frame (wide, frontal, near), separately for day and evening; floor image from the maps; writes the world file |

| Finding (2026-11-26) | Value |
|---|---|
| Runs used | 20 of 2026-11-23/24 (7 daylight, 13 evening), 17 600 frames |
| Fusing with recorded poses only | walls 20–30 cm wide, kitchen doubled: yaw drift per run |
| Range used for the maps | 1.5 m; beyond that the depth of a single frame smears |
| Not reconstructed | undersides, things above ~1.3 m at close range, tile grid, sofa interior (camera is below the seat) |

### 3D view (`tools/sim3d.py`, `tools/sim3d.html`, PC only)

| Command | Effect |
|---|---|
| `python tests/sim_run.py explore 120 --wheels --dyn --rider --live [--live-port 8765]` | live in the browser at `http://localhost:8765/`, stays open after the run (Ctrl+C) |
| `… --view run.jsonl` | write the run to a file (~0.5 MB per 20 s) |
| `python tools/sim3d.py run.jsonl [--port 8765]` | replay; `?t=12` jumps to 12 s, `?ansicht=2` top view |
| `python tools/sim_rider.py --proc DIR [--port 8793] [--view-port 8792] [--scene unten]` | virtual Rider (default scene `unten`): endless simulator (idle, Rider values, person) with the real xgonav API on 127.0.0.1:8793; `DIR` plays `/proc/XGORider` (`SimRobot(proc_dir=…)`: `height`, `roll`, `action` 253/254, `state`, `battery`; `imu_log`, `wheels_log` at 50 Hz for `tools/live.php`); 3D view of the last 5 min. World runs at `[camera] fps` (24; `--fps`), and keeps moving without images while the image pipeline is stopped (manual drive works as on the Rider) |
| `./tools/sim_kimmy.sh` | deploy the simulator to kimmy: `http://192.168.53.236:8790/` starts runs (mode, seconds, scene, body heights cruise/search/fast = `behavior.height_*_mm` 60–108, flags, `--set`), lists results, shows each run live and as replay (`tools/simserver.py`, user unit `xgo-sim`, runs in `~/xgo_sim/laeufe`, at most 4 at once, nice 10). Virtual Rider: the real web UI (`web/` copied to `/srv/xgo_sim/web`) at `http://192.168.53.236:8791/` (nginx site `tools/sim_rider.nginx.conf`, HTTP only; php-fpm pool `tools/sim_rider.fpm.conf` as `www-data` with `XGO_NAV`/`XGO_PROC`/`XGO_SIM`; user units `xgo-rider`, `xgo-rider-live` = `live.php` on 8794; `/srv/xgo_sim/proc` group `www-data`), 3D at `:8792`, “Neu starten” in the Zentrale. Not in the simulator: intercom, emotions/display, LEDs (file only), Wi-Fi, shutdown/restart, settings editor, voice samples, FloorNet choice; roll is written but the body does not tilt; CPU/WLAN tiles show kimmy |

Shows room, boxes, joints, dark zones, rider (CAD model `web/rider-model.bin` as in the web UI: legs from the height setpoint with the firmware's u8 55–200 clamp and ~0.33 s lag, wheels turn with the true path; boxes if the file is missing; pitch, FOV cone, collision radius), person, true path (green), map pose path (orange), plan (blue), patrol route (pink), occupancy map (1 Hz) and camera frame (4 Hz). Read-only feed thread at 10 Hz (truth from `SimWorld`, estimate from `GridMap`); `loop_ms` unchanged within run-to-run spread. Keys 1–4: follow, top, free, close-up of the rider; space: play/pause. three.js 0.160.0 from jsDelivr (internet needed).

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
