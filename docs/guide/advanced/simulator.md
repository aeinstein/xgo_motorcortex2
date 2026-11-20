# Simulator and tests

Development happens on the PC. The simulator (`xgonav/sim.py`) renders a room by raycasting, with boxes, a walking person, pitch oscillation from real drives and motion blur. It is the only closed-loop validation. Reference: [Testing](../../testing.md).

## Tests

| Command | What | Time |
|---|---|---|
| `python -m pytest -q tests/` | unit tests (C++ ones skipped without the core) | ~2 s |
| `python -m pytest -q -m slow tests/` | closed loop in the simulator | ~10 min |
| `python -m pytest -q tests/test_units.py -k imu_sync` | one test | |

## Simulator runs

```bash
python tests/sim_run.py follow 40 out.mp4                         # follow, overlay video
python tests/sim_run.py explore 300 --wheels --fs-thread --dyn --rider --scene spalt
python tests/sim_run.py home 60 --wheels --dyn                    # 30 s explore, then home
python tests/sim_run.py pet 35 --wheels --imu --animal cat
```

| Flag | Effect |
|---|---|
| `--wheels` | simulated wheel odometry |
| `--dyn` | inertia like the balancing Rider (turn lag, dead band, roll-back) |
| `--rider` | Rider config values |
| `--imu` | simulated IMU (deliberately wrong sign and offset, sync must find them) |
| `--scene zwei\|spalt` | two rooms / 25 cm door |
| `--phantoms n` | false alarms of the floor model |

!!! note "Sim ≠ Rider"
    The simulator floor boundary is ground truth: FloorNet errors are not simulated. What works there can still stick at tight spots on the Rider.

## Replay real recordings

```bash
python -m xgonav --source video.mp4 --robot sim --mode follow --fast
python tools/replay_raw.py <recording> --config rider.toml --video out.mp4
python tools/map_replay.py <recording> --config rider.toml --out out/
```
