# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Projekt

Das Repo `xgo_motorcortex2` enthält Treiber (`kernel_module/`), Display-Daemon (`st7789/`), Weboberfläche (`web/`, `deploy/`) und xgonav. xgonav ist eine kamerabasierte Navigations-Pipeline (Python 3.11+, OpenCV, numpy, optional onnxruntime) für den XGO Rider, einen selbstbalancierenden Zweiradroboter mit Raspberry Pi CM4 (2 GB RAM). Sie spricht die Hardware ausschließlich über das Kernelmodul `xgo-drv` aus `kernel_module/` (`/proc/XGORider/...`) an. Code, Kommentare, Logs und Statusmeldungen sind durchgehend auf Deutsch; so beibehalten.

Die README.md ist die maßgebliche Doku für Architektur-Begründungen, Installation, Kalibrierung und bekannte Grenzen. Wichtigste Randbedingung: **Fahren ist nicht auf echter Hardware getestet**, Verhaltens- und Regelergebnisse stammen aus dem Simulator. Am RiderPi geprüft sind nur Treiber-Patches 0001/0002, Idle-Läufe (Rechenzeit) und die Kamerageometrie (README, „Grenzen“). Der Simulator rendert noch mit 62,2° Öffnungswinkel, die echte Kamera hat ~91°.

## Befehle

Entwicklung findet am PC statt (kein Pi nötig). Die Umgebung liegt im venv `.venv` (gitignored) mit `opencv-python-headless`, `numpy`, `pytest` und `onnxruntime`. Vor allen Befehlen `source .venv/bin/activate` oder direkt `.venv/bin/python` nutzen. Neu anlegen mit:

```bash
python3 -m venv .venv && .venv/bin/pip install "opencv-python-headless>=4.6" "numpy>=1.24" pytest "onnxruntime>=1.16"
```

```bash
python -m pytest -q tests/                       # Unit-Tests (15 Tests, ~2 s); slow ist per pytest.ini ausgeschlossen
python -m pytest -q -m slow tests/               # Closed-Loop im Simulator (3 Tests, ~90 s gesamt); -m überschreibt addopts
python -m pytest -q tests/test_units.py -k imu_sync   # einzelner Test

python tests/sim_run.py follow 40 out.mp4        # Szenario im Simulator, optional mit Overlay-Video
python tests/sim_run.py explore 40
python tests/sim_run.py follow 40 --imu          # mit simulierter IMU (Vorzeichen -1, Versatz -8 ms absichtlich falsch)

python -m xgonav --source video.mp4 --robot sim --mode follow --fast   # Replay eines eigenen Videos
python -m xgonav --mode idle --record debug.mp4  # auf dem Rider: nur schauen, Overlay aufzeichnen

./tools/export_model.sh [yolo11n] [256]          # ONNX-Export, nur auf PC/GPU-Server, NICHT auf dem Pi
driver-test/run.sh kernel_module 6 50 --faults   # Treiberlogik im Userspace gegen ESP32-Emulator
make -C kernel_module                            # auf dem Rider: Treiber bauen (Kernel-Header nötig)
sudo ./deploy/install.sh                         # auf dem Rider: nach /opt/xgonav + systemd-Unit
sudo ./deploy/install-all.sh                     # auf dem Rider: alles (Treiber, Display, rc.local, xgonav, Web)
python tools/nudge.py turn 20 | move 1.0         # auf dem Rider: balancierenden Rider drehen (IMU-geregelt) / fahren
python tools/calibrate.py shading                # auf dem Rider: Linsenabschattung aus Weißbild -> shading.json
```

Am echten Rider gilt: Bewegungen nur mit Zustimmung und Beobachtung durch den Nutzer, Körperhöhe nie unter Minimum + 10 mm, `rc-control.service` stoppen, solange xgonav fährt.

Es gibt keinen Linter, kein Build-System und kein `pyproject.toml`; das Paket wird direkt aus dem Repo-Root per `python -m xgonav` gestartet. Tests und Tools fügen den Repo-Root selbst zu `sys.path` hinzu.

## Architektur

Datenfluss pro Frame in `App._step()` (`xgonav/app.py`), alles andere hängt daran:

```
camera.FrameSource  →  egomotion.EgoMotion  →  tracker.Tracker  ←  detector.DetectorWorker (eigener Thread)
                              │                      │
                              ├→ imu.ImuSync (Komplementärfilter IMU + Bild → pitch_dev)
                              ├→ freespace.FreeSpaceEstimator → localmap.LocalMap
                              ▼
                    behaviors.World → Behavior.step() → behaviors.safety() → App._arc_check() → robot.Robot.drive()
```

Grundprinzipien, die sich durch alle Module ziehen:

- **Nur der neueste Frame zählt.** `FrameSource.get()` liefert immer den aktuellsten Frame, alte werden verworfen. Keine Queues.
- **Ego-Motion wird überall herausgerechnet.** `EgoMotion` liefert pro Frame eine Homographie `A` (Rotationsmodell, nicht 2D-Shift). Der Tracker wendet sie auf alle Boxen an, verzögerte Detektionen werden per `EgoMotion.transform_between(frame_a, frame_b)` auf den aktuellen Frame geschoben, die Freiraum-Horizontlinie und die Fußpunkt-Entfernung nutzen `pitch_dev`.
- **Zwei Taktraten.** Die Bildschleife läuft mit Kamera-FPS, die Regelung nur mit `control_hz` (20 Hz). Nur im Regelschritt werden `World` gebaut, Verhalten ausgeführt und `robot.drive()` aufgerufen.
- **Sicherheit ist geschichtet und hat das letzte Wort:** `safety()` (gefallen, Freiraum, TTC) → `_arc_check()` (Bahn gegen Merkkarte, auch rückwärts) → `Robot` (Clamp auf `max_v`/`max_w`, Rampe, Userspace-Watchdog `stall_timeout_s`, Keepalive) → Kernel-Watchdog `motion_timeout_ms` (nur mit Patch 0001).
- **Robot-Konvention:** `drive(v, w)` normiert auf [-1, 1], v > 0 vorwärts, w > 0 Linksdrehung. Die Umrechnung auf u8 (128 = Stillstand) liegt in `robot.norm_to_u8`. Sollwerte werden von der Firmware *gehalten*, deshalb müssen Stopps explizit geschrieben werden.
- **Snapshot statt Shared State für die Anzeige.** `App.snapshot` wird pro Frame durch ein neues `Snapshot`-Objekt ersetzt (nie mutiert) und über `snap_cond` signalisiert. Web-Schnittstelle, LCD und Recorder (`--record`, `--record-raw`) lesen nur daraus; `render()` zeichnet das Overlay erst auf Anfrage.

### Konfiguration

`xgonav/config.py` enthält alle Defaults als verschachtelte Dataclasses. `deploy/config.example.toml` überschreibt nur Abweichungen; unbekannte Schlüssel lösen beim Laden einen `KeyError` aus. Neue Parameter gehören also zuerst in die Dataclass. CLI-Flags in `__main__.py` überschreiben wiederum die TOML.

### Austauschbare Backends

Hardware-nahe Teile haben je eine echte und eine Sim-Implementierung mit gleichem Interface; `App` bekommt sie injiziert:

| Rolle | echt | Simulator/Test |
|---|---|---|
| Bildquelle | `camera.PiCameraSource`, `camera.OpenCVSource` | `sim.SimWorld` (ist selbst eine `FrameSource`) |
| Roboter | `robot.ProcRobot` | `robot.SimRobot` (integriert Kurs, protokolliert Writes) |
| Detektor | `detector.DetectorWorker` (YOLO-ONNX) | `sim.FakeDetector` (Ground-Truth-Boxen, 180 ms Latenz, 5 Hz) |
| IMU | `imu.ProcImu` | `sim.SimImu` |

`--robot` wird automatisch gewählt: `proc` bei `source = "picam"`, sonst `sim`. `tests/sim_run.py` zeigt das Zusammenstecken für Closed-Loop-Läufe und ist die Grundlage der slow-Tests.

### Simulator

`xgonav/sim.py` rendert per Raycasting einen Raum mit Kisten und einer laufenden Person, inklusive Nicken (2,5 Hz + Beschleunigungsneigung), Kurszittern und Bewegungsunschärfe. Fahrbefehle des `SimRobot` bewegen die Kamera. Der Simulator ist die einzige Validierung; neue Verhaltens- oder Schätzlogik sollte dort gegen die Ground Truth (`SimWorld.log`, `collisions`) geprüft werden, nicht nur in Unit-Tests.

### Treiber und Display

`kernel_module/` (mit `../constants.h`) ist der Treiber aus `xgo_motorcortex` (github.com/aeinstein/xgo_motorcortex, Stand `c3c888c`) plus die früheren Patches: 0001 Speicherfehler in proc-Handlern + `settings/motion_timeout_ms`; 0002 synchrone Serial-Reads, IMU-Thread mit `/proc/XGORider/imu`, `imu_log`, `serial_stats` (Format `seq t_ns roll pitch yaw`, Milligrad, CLOCK_MONOTONIC); 0003 `force_yaw` hält den Kurs per Impulsen; 0004 `roll` = Roll-Winkel in Grad (0x36), `roll_balance` = 0x61. Im Code wird weiter von „Treiber-Patch 000x“ gesprochen, gemeint ist dieser Stand. `driver-test/run.sh` kompiliert den echten Treibercode gegen `kshim.h` im Userspace und testet ihn mit `esp32_emu.py`. Ohne 0002 ist `imu.py` wirkungslos und `pitch_src` bleibt `"bild"`. `st7789/` ist der Display-Daemon (libdriver, Build in `st7789/project/riderpi`), `deploy/rc.local` startet ihn.

## Stolperfallen

- Der Detektor (`DetectorCfg.model`) wird still deaktiviert, wenn die ONNX-Datei fehlt; `models/*.onnx` ist gitignored.
- Weboberfläche ist PHP (`web/index.php`, `web/api.php`) hinter nginx (`deploy/nginx-xgonav.conf`, eigener php-fpm-Pool `deploy/php-fpm-xgonav.conf`); der Nutzer bevorzugt PHP fürs Web. `web.py` ist nur noch die lokale Schnittstelle (`web_host = 127.0.0.1`), ohne Authentifizierung. Am RiderPi: nginx mit TLS auf 443 → `/xgonav/` (PHP, Bilder per WebSocket `/xgonav/ws` direkt von xgonav, mit Bestätigung je Bild), Port 80 und `/` leiten dorthin um. stunnel und die alte Fernsteuerung rc_server (Node, 1081, `rc-control.service`) sind abgeschaltet; `api.php` liest nur noch die Emotionen aus `rc_server/emotions`.
- `lcd.py` und `picamera2` existieren nur auf dem Pi; beide werden lazy importiert und nur bei `--robot proc` aktiviert.
- Die Annahme „1 Firmware-Einheit = 1 m/s“ (`vx_limit`) fließt nur in die Merkkarte ein und ist unkalibriert.
