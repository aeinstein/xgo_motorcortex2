# Quick start

## On the Rider (web UI)

!!! danger "Before the first drive"
    Clear floor, **no stairs or edges nearby**, someone watching. Body height never below minimum + 10 mm.
    **STOP** (header, top right) stops everything and switches to `idle`.

| # | Do | Where |
|---|---|---|
| 1 | Switch the Rider on, wait for the eyes on the LCD | |
| 2 | Open `https://<rider>/` | browser in the same LAN |
| 3 | Start vision (camera, FloorNet); from sleep mode first **Wake** | *Vision* card → **Start** |
| 4 | Stand up | *Attitude* card → **Stand up** (action 255); wait until the state shows balancing |
| 5 | Drive by hand | hold the D-pad in the image, arrow keys or WASD; mode switches to **Manual** |
| 6 | Try a mode | mode bar: **Follow** (pick a target class), **Explore**, **Home**, **Pet** |
| 7 | Set down | *Attitude* → **Sit down** (action 254) |
| 8 | Save battery | *Vision* → **Stop**, or **Rest** in the header (sleep mode) |

The UI is German by default; **EN** in the header switches the labels to English (used throughout this manual).

## On the Rider (command line)

```bash
cd /opt/xgonav
venv/bin/python -m xgonav --config config.toml --mode idle                      # watch only
venv/bin/python -m xgonav --config config.toml --mode idle --record debug.mp4   # record the overlay
venv/bin/python -m xgonav --config config.toml --mode follow                    # follow a person
python tools/nudge.py turn 20                                                   # turn 20° left (IMU-controlled)
```

Stop the service first (`sudo systemctl stop xgonav`); only one xgonav may own the camera.

## On the PC (no hardware)

```bash
python3 -m venv .venv && .venv/bin/pip install "opencv-python-headless>=4.6" "numpy>=1.24" pytest "onnxruntime>=1.16"
.venv/bin/python -m pytest -q tests/                              # unit tests, ~2 s
.venv/bin/python tests/sim_run.py follow 40 out.mp4               # simulator, overlay video
.venv/bin/python tests/sim_run.py explore 300 --wheels --fs-thread --dyn --rider
```

On Windows use `.venv\Scripts\python`. More: [Simulator and tests](advanced/simulator.md).
