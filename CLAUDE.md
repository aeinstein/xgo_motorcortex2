# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Projekt

Das Repo `xgo_motorcortex2` enthält Treiber (`kernel_module/`), Display-Daemon (`st7789/`), ESP32-Firmware-Build (`firmware/`), Weboberfläche (`web/`, `deploy/`) und xgonav. Offene Punkte stehen in `TODO.md`. xgonav ist eine kamerabasierte Navigations-Pipeline (Python 3.11+, OpenCV, numpy, optional ncnn/onnxruntime) für den XGO Rider, einen selbstbalancierenden Zweiradroboter mit Raspberry Pi CM4 (2 GB RAM). Sie spricht die Hardware ausschließlich über das Kernelmodul `xgo-drv` aus `kernel_module/` (`/proc/XGORider/...`) an. Code, Kommentare, Logs und Statusmeldungen sind durchgehend auf Deutsch; so beibehalten.

Maßgebliche Doku: README.md (Überblick) und je Thema eine Datei in `docs/` (Architektur, Installation, Konfiguration, Kalibrierung, Treiber, Weboberfläche, Karten, Display, Tests, Grenzen, LEDs). README und `docs/` sind **englisch** und knapp (Fakten, Tabellen, keine Prosa); Code, Kommentare und Logs bleiben deutsch. Wichtigste Randbedingung: `follow` und Handsteuerung wurden am RiderPi gefahren; `home` kam dort am 2026-11-05 nicht ans Ziel (danach geändert, nicht erneut gefahren), das gezielte `explore` (Frontiers) am 2026-11-06 in drei Testfahrten, noch ohne brauchbare Karte und **ohne Absturzschutz: wäre fast eine Stufe hinuntergefahren**, `approach` gar nicht (`docs/limitations.md`). Der Simulator kennt die Trägheit des balancierenden Riders nur mit `--dyn` und keine Fehlalarme des Bodenmodells; was dort klappt, kann am Rider an engen Stellen hängen. Simulator und Default nutzen 88,5° Öffnungswinkel wie die echte Kamera (Zollstock-Messung).

## Befehle

Entwicklung findet am PC statt (kein Pi nötig). Die Umgebung liegt im venv `.venv` (gitignored) mit `opencv-python-headless`, `numpy`, `pytest` und `onnxruntime`. Vor allen Befehlen `source .venv/bin/activate` oder direkt `.venv/bin/python` nutzen (unter Windows `.venv\Scripts\python`). Neu anlegen mit:

```bash
python3 -m venv .venv && .venv/bin/pip install "opencv-python-headless>=4.6" "numpy>=1.24" pytest "onnxruntime>=1.16"
```

```bash
python -m pytest -q tests/                       # Unit-Tests (88 Tests, ~2 s; 5 ohne C++-Kern übersprungen); slow ist per pytest.ini ausgeschlossen
python -m pytest -q -m slow tests/               # Closed-Loop im Simulator (5 Tests, ~6 min gesamt); -m überschreibt addopts
python -m pytest -q tests/test_units.py -k imu_sync   # einzelner Test

python tests/sim_run.py follow 40 out.mp4        # Szenario im Simulator, optional mit Overlay-Video
python tests/sim_run.py explore 300 --wheels --kanten --fs-thread --dyn --rider [--scene zwei] [--phantoms 8]   # Raum kartieren wie am Rider
python tools/map_eval.py record a.pkl; python tools/map_eval.py eval a.pkl --set grid.max_range_m=1.0,1.5 --rays   # Karte offline messen
python tests/sim_run.py home 60 --wheels         # 30 s erkunden, dann zurück zum Start (--dyn: Trägheit wie am Rider)
python tests/sim_run.py follow 40 --imu          # mit simulierter IMU (Vorzeichen -1, Versatz -8 ms absichtlich falsch)

python -m xgonav --source video.mp4 --robot sim --mode follow --fast   # Replay eines eigenen Videos
python -m xgonav --mode idle --record debug.mp4  # auf dem Rider: nur schauen, Overlay aufzeichnen

./tools/export_model.sh [yolo11n] [256]          # ONNX-Export, nur auf PC/GPU-Server, NICHT auf dem Pi
driver-test/run.sh kernel_module 6 50 --faults   # Treiberlogik im Userspace gegen ESP32-Emulator
make -C kernel_module                            # auf dem Rider: Treiber bauen (Kernel-Header nötig)
make -C xgonav/native                            # auf dem Rider: C++-Kern (Ego-Motion, Bodenmodell, Raumkarte; libopencv-dev); ohne ihn rechnet Python
make                                             # Root-Makefile: Treiber + Display-Daemon (st7789/project/riderpi)
firmware/build.sh [rider2-firmware]              # ESP32-Firmware bauen (PlatformIO, flasht NICHT) -> firmware/out/app0.bin
sudo ./deploy/install.sh                         # auf dem Rider: nach /opt/xgonav + systemd-Unit
sudo ./deploy/install-all.sh                     # auf dem Rider: alles (Treiber, Display, rc.local, xgonav, Web)
python tools/nudge.py turn 20 | move 1.0         # auf dem Rider: balancierenden Rider drehen (IMU-geregelt) / fahren
python tools/calibrate.py shading                # auf dem Rider: Linsenabschattung aus Weißbild -> shading.json
python tools/step_to_model.py --cache rider.npz  # nur PC (pip install cadquery-ocp fast-simplification): docs/Rider-Pi.STEP -> web/rider-model.bin
```

Am echten Rider gilt: Bewegungen nur mit Zustimmung und Beobachtung durch den Nutzer, Körperhöhe nie unter Minimum + 10 mm, `rc-control.service` stoppen, solange xgonav fährt. Firmware nie selbst flashen; nur app0 (`0x10000`), niemals `0x310000–0x32ffff` (Beinnullpunkte, BLE-Name, Pitch-Trim) oder spiffs, vorher Vollbackup (`firmware/README.md`).

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

`firmware/` baut die ESP32-Firmware R-1.1.6 aus der Referenz `LuwuDynamics/xgorider2_rl` plus Patches in `firmware/patches/` (0001 Rad-Register 0x0c/0x0d, 0002 Aktion 254 = Not-Aus, 0003 LED-Helligkeit, 0004 Aktion 253 = Aufstehen aus der Liegelage; 0005 Drehung um die Senkrechte aus Nick- und Rollrate herausrechnen, 0006 Beschleunigungswinkel bei schneller Drehung einfrieren (Fliehkraft), sonst rollt der Rider beim Drehen weg, Host-Test `firmware/tests/test_imu_yaw_coupling.cpp`; 0007 Diagnose-Register 0x0e (Reglergrößen, am Pi über `raw_reg`); 0008 Positionshalten auch beim Drehen auf der Stelle, Aktion 241 an / 240 aus, Host-Test `firmware/tests/test_turn_hold.cpp`; 0009 Gyro-z-Nullpunkt laufend nachführen (Räder/Stillstand), Gier-Totband 2 → 0,3 °/s, Register 0x0f, Host-Test `firmware/tests/test_yaw_bias.cpp`; 0010 Halten beim Drehen per Default an; 0001–0010 seit 2026-11-05 geflasht, siehe `firmware/README.md`). Der Treiber erkennt 0x0d/0x0c automatisch und liefert dann `/proc/XGORider/wheels` (Encoder, 1024 Ticks/Umdrehung, Rad 11 = links); `xgonav/wheels.py` liest das (`sim_run.py --wheels`).

## Stolperfallen

- Detektor: `DetectorCfg.model` ist ein NCNN-Exportordner (`models/yolo11n_256_ncnn`); fehlen Modell oder Paket `ncnn`, greift `fallback_model` (ONNX); fehlen beide, ist die Objekterkennung aus (Warnung im Log). Modelle sind gitignored, Export per `tools/export_model.sh`.
- Weboberfläche ist PHP (`web/index.php`, `web/api.php`) hinter nginx (`deploy/nginx-xgonav.conf`, eigener php-fpm-Pool `deploy/php-fpm-xgonav.conf`); der Nutzer bevorzugt PHP fürs Web. `web.py` ist nur noch die lokale Schnittstelle (`web_host = 127.0.0.1`), ohne Authentifizierung. Am RiderPi: nginx mit TLS auf 443 → `/xgonav/` (PHP, Bilder per WebSocket `/xgonav/ws` direkt von xgonav, mit Bestätigung je Bild), Port 80 und `/` leiten dorthin um. stunnel und die alte Fernsteuerung rc_server (Node, 1081, `rc-control.service`) sind abgeschaltet; `api.php` liest nur noch die Emotionen aus `rc_server/emotions`.
- C++-Kern `xgonav/native/libxgofast.so` (ctypes, geladen in `fastlib.py`): `egofast.cpp` rechnet dasselbe wie `EgoMotion` (`[ego] native`, eine Pyramide je Bild; Grau + 2:1, Schärfe und `goodFeaturesToTrack` bitgleich nachgebaut, nur aktiv nach Selbstprüfung gegen OpenCV beim ersten `ego_new`, siehe `ego_fast_mask`), `floorfast.cpp` dasselbe wie `FreeSpaceEstimator._edge_columns` und das Verkleinern 4:1 (`area_down`, NEON; nur aktiv, wenn es beim Start bitgleich zu `cv2.resize` INTER_AREA ist) (`[freespace] native`), `gridfast.cpp` das Eintragen in `GridMap.add_freespace` (`[grid] native`). Änderungen daran immer in beiden Wegen machen und am Rider vergleichen (`test_egomotion_native_matches_python`, `test_ego_fast_paths_match_opencv`, `test_kanten_native_matches_numpy`, `test_floor_downscale_native_equals_cv2`, `test_gridmap_native_matches_numpy`, am PC übersprungen); bei geänderten Strukturen die ABI-Nummern (`_NATIVE_ABI`/`kAbi`, `_EDGE_ABI`/`kEdgeAbi`, `_GRID_ABI`/`kGridAbi`) erhöhen. Bitgleich nachbauen heißt: OpenCV-Quelle 4.6 lesen (FMA in NEON-Pfaden, Summationsreihenfolge) und an echten Aufnahmen prüfen; synthetische Prüfbilder allein trennen Rundungsvarianten oft nicht.
- `lcd.py` und `picamera2` existieren nur auf dem Pi; beide werden lazy importiert und nur bei `--robot proc` aktiviert.
- Am Rider bleibt der Kernel auf 6.6.74 (Pin `deploy/apt-pin-kernel`): Unter 6.12 baut der Soundtreiber nicht, `xgo-drv` ist dort weder gebaut noch getestet. Der Dienst läuft als `aeinstein` und nimmt OpenCV 4.10 und numpy 1.26 aus `~/.local`, root sieht andere Versionen; Importe nach Upgrades als dieser Nutzer prüfen.
- `robot.vx_limit` (m/s je Firmware-Einheit): am RiderPi 1,8 (gemessen 2026-11-05), Default 1,5; rechnen damit Bahnprüfung und Folgen.
