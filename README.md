# xgo_motorcortex2 – Treiber, Display und Navigation für den XGO RiderPi

Alles, was auf dem RiderPi läuft, in einem Repo:

| Verzeichnis | Inhalt |
|---|---|
| `kernel_module/`, `constants.h` | Kernelmodul `xgo-drv` (`/proc/XGORider`), aus [`xgo_motorcortex`](https://github.com/aeinstein/xgo_motorcortex) `c3c888c` plus den früheren Patches 0001–0004 |
| `st7789/` | Display-Daemon `st7789_server` (FIFO `/tmp/lcd0`, RGB565 320×240), basiert auf [libdriver/st7789](https://github.com/libdriver/st7789) |
| `xgonav/`, `tools/`, `tests/` | xgonav, die kamerabasierte Navigation (Python) |
| `web/` | Weboberfläche (PHP) |
| `deploy/` | Konfigs: xgonav, nginx (mit TLS), php-fpm, sudoers, rc.local |
| `driver-test/` | Treibercode im Userspace gegen einen ESP32-Emulator testen |

Nicht enthalten: die alte Fernsteuerung `rc_server` (Node), Zertifikate und Schlüssel.

## Gesamtinstallation auf dem Rider

In einem Schritt: `sudo ./deploy/install-all.sh [--no-driver] [--no-display] [--no-web]`.
Das Skript führt die Befehle unten aus, sichert überschriebene Dateien in `/etc` als
`*.bak-<Zeitstempel>`, übernimmt den nginx-Benutzer für den php-fpm-Socket und richtet
nginx auf 443 nur ein, wenn Zertifikat und Schlüssel vorhanden sind (ein laufendes
stunnel wird dabei abgeschaltet). Es startet weder xgonav
noch lädt es den Treiber neu; beides passiert erst nach dem Neustart bzw. von Hand.

```
sudo make -C kernel_module && sudo make -C kernel_module install    # Treiber
make -C st7789/project/riderpi && sudo cp st7789/project/riderpi/st7789_server /usr/bin/
sudo cp deploy/rc.local /etc/rc.local && sudo cp deploy/rc-local.service /etc/systemd/system/
sudo ./deploy/install.sh                                            # xgonav nach /opt/xgonav
# Web: nginx (TLS auf 443) + php-fpm
sudo apt-get install -y nginx php8.2-fpm
sudo cp deploy/nginx-xgonav.conf /etc/nginx/sites-available/xgonav && sudo ln -s ../sites-available/xgonav /etc/nginx/sites-enabled/
#   (ssl_certificate/ssl_certificate_key in der Datei ggf. anpassen)
sudo cp deploy/php-fpm-xgonav.conf /etc/php/8.2/fpm/pool.d/xgonav.conf
sudo usermod -aG video www-data
sudo install -m 0440 deploy/sudoers-xgonav-poweroff /etc/sudoers.d/xgonav-poweroff && sudo visudo -c
sudo systemctl disable --now stunnel4 2>/dev/null   # falls noch vorhanden: belegt sonst 443
sudo nginx -t && sudo systemctl restart php8.2-fpm nginx
```

`st7789/project/riderpi`: `make install` richtet zusätzlich `st7789_display.service` ein.
Auf dem RiderPi startet `rc.local` den Daemon; nicht beides verwenden.

---

# xgonav – kamerabasierte Navigation für den XGO Rider

Ersetzt die Hersteller-Software durch eine schlanke Python-Pipeline, die auf dem
CM4 (2 GB) in Echtzeit läuft. Sie setzt auf das Kernelmodul in `kernel_module/`
(`/proc/XGORider`) auf.

**Modi:** `follow` (Person/Objekt folgen), `explore` (Raum erkunden, Hindernissen
ausweichen), `approach` (Objekt suchen, anfahren, davor stoppen), `idle`.

## Architektur

```
Kamera (OV5647, 640x480, kurze Belichtung)
  │  nur der neueste Frame zählt
  ▼
Ego-Motion ── optischer Fluss (LK) + RANSAC → Rotationsmodell (Gier/Nick/Roll + Expansion)
  │           = Homographie K·Rᵀ·K⁻¹ pro Frame, Schärfe-Kennzahl, Time-to-Contact
  │   + IMU ── /proc/XGORider/imu_log (50 Hz): absolute Nicklage, Vorzeichen/Zeitversatz
  │           automatisch per Korrelation mit dem Bild; Komplementärfilter IMU (langsam) + Bild (schnell)
  ├──────────► Tracker ◄── YOLO (ONNX, eigener Thread, nur scharfe Frames)
  │            Ego-Motion auf alle Tracks; verzögerte Detektionen per Transformkette
  │            auf den aktuellen Frame verschoben (Latenzkompensation)
  ├──────────► Freiraum ── Bodenfarbmodell → frei/blockiert je Bildspalte
  │            └► Merkkarte (Bodenkanten im Roboter-KS, 4 s, mit Eigenbewegung mitgeführt)
  ▼
Verhalten (follow/explore/approach) → Sicherheit (gefallen, TTC, Freiraum, Bahnprüfung)
  ▼
Robot: Rampe, Keepalive, Watchdog → /proc/XGORider/speed_x, speed_z, height
```

**Warum so, und nicht anders?**

- **Wackeln:** Der Rider balanciert und nickt deshalb ständig. Statt das Bild zu
  stabilisieren, wird die Kamerabewegung pro Frame gemessen und überall
  herausgerechnet: im Tracker, in der Entfernung, in der Freiraum-Horizontlinie.
  Ein Rotationsmodell (keine 2D-Verschiebung) ist nötig, weil der Fußpunkt am
  unteren Bildrand bei Nicken ~20 % weiter wandert als die Bildmitte.
- **IMU + Bild:** Die Bildbewegung misst schnelle Drehungen sehr genau (Simulator:
  0,03° pro Frame), die langsame Neigung beim Beschleunigen aber gar nicht. Die
  ESP32-IMU liefert die absolute Lage (mit Treiber-Patch 0002), ist aber mit 50 Hz
  zu langsam für Vibrationen. Deshalb kombiniert ein Komplementärfilter beide: den
  langsamen Anteil aus der IMU, den schnellen aus dem Bild. Vorzeichen und
  Zeitversatz der IMU werden nicht angenommen, sondern laufend aus der Korrelation
  der Nickraten bestimmt. Erst bei eindeutigem Ergebnis wird die IMU genutzt.
- **Entfernung:** Die Kamera sitzt nur ~13 cm hoch. Eine Person ist unter ~3,5 m
  oben abgeschnitten, die Box-Höhe taugt also nicht. Genutzt wird der Fußpunkt
  über die Bodenebene, fusioniert mit Breite/Höhe der Box (nur nicht abgeschnittene
  Kanten), gewichtet nach Unsicherheit.
- **Detektor langsam (~5 Hz auf dem CM4):** Er läuft asynchron. Zwischen den
  Detektionen hält der Tracker die Boxen per Ego-Motion und Eigenbewegung.
- **Sicht nur ±31°:** Die Merkkarte erinnert sich an Hindernisse, die seitlich
  aus dem Bild gewandert sind. Jeder Fahrbefehl wird vorher als Bogen gegen die
  Karte geprüft, vorwärts wie rückwärts.

## Installation auf dem Rider

Voraussetzung: das Kernelmodul läuft (rc.local lädt `xgo-drv`).

1. **Treiber bauen und laden.** `kernel_module/` enthält die früheren Patches 0001–0004
   gegenüber `xgo_motorcortex`:
   ```
   make -C kernel_module && sudo make -C kernel_module install && sudo rmmod xgo-drv; sudo modprobe xgo-drv
   cat /proc/XGORider/serial_stats; cat /proc/XGORider/imu   # Kontrolle
   ```
   Die stty-Zeile in `deploy/rc.local` setzt die UART in den Rohmodus
   (`-ixon -icrnl … min 0 time 1`).
   Ist das Modul ohne Neustart neu geladen, die UART-Flags vorher live setzen
   (siehe „Grenzen“: UART im Rohmodus). Die Fernsteuerung (`rc-control.service`)
   stoppen, solange xgonav fährt, sonst schreiben zwei Prozesse Fahrbefehle.
   - **0001** behebt Speicherfehler in den proc-Handlern und ergänzt
     `settings/motion_timeout_ms`. Ist der Wert gesetzt, hält der Kernel den Rider an,
     wenn xgonav abstürzt. Der Default ist 0 (aus), dein RC-Client funktioniert also
     unverändert.
   - **0002** baut den Lesepfad um: Lesezugriffe warten per `completion` auf die
     Antwort statt `msleep(200)`, der Reader liest ohne Pausen. Neu ist ein IMU-Thread
     mit `/proc/XGORider/imu` (neueste Probe), `imu_log` (letzte 128 Proben) und
     `serial_stats`, Format `seq t_ns roll pitch yaw`, Winkel in Milligrad,
     CLOCK_MONOTONIC. Rate über `settings/imu_hz` (Default 50, 0 = aus). Ein
     12-Byte-Blockread wird automatisch erkannt, falls die Firmware ihn kann.
     Außerdem setzt die `stty`-Zeile die UART in den Rohmodus (`-ixon -icrnl …`).
   - **0004** `roll` setzt den Roll-Winkel (Grad −20…20, Register 0x36) statt die
     Roll-Balance (0x61, jetzt `roll_balance`).
   - **0003** repariert `settings/force_yaw`: Soll ist der Kurs beim Aufstehen bzw.
     Einschalten (vorher fest 0°, Überlauf → Dauerdrehen), Korrektur in kurzen
     Impulsen mit Ausschwingen, pausiert bei Drehbefehlen über `speed_z`. Gegen die
     langsame Kursdrift hilft es nicht (siehe „Grenzen“). Gebaut, nicht am Rider
     getestet; xgonav schaltet `force_yaw` ohnehin ab.
   - Test ohne Hardware: `driver-test/run.sh kernel_module 6 50 --faults`
     kompiliert den echten Treibercode im Userspace gegen einen ESP32-Emulator.
2. **Modell exportieren** (auf PC/GPU-Server, nicht auf dem Pi):
   `./tools/export_model.sh` → `models/yolo11n_256.onnx` in `models/` legen.
3. **Installieren:** `sudo ./deploy/install.sh` → `/opt/xgonav`, systemd-Unit `xgonav`.

## Kalibrieren (einmalig, ~10 Minuten)

Alle Befehle im Verzeichnis `/opt/xgonav` mit dem venv-Python. Zum Ausrichten des
balancierenden Riders: `python tools/nudge.py turn <grad>` (IMU-geregelt, + = links)
und `python tools/nudge.py move <sekunden>` (ungeregelt, + = vorwärts).

0. **Öffnungswinkel zuerst.** Der Default `hfov_deg = 62.2` (RPi-Kamera v1) stimmt
   beim RiderPi nicht: gemessen ~91° (f ≈ 314 px bei 640 px, siehe „Grenzen“). Mit
   falschem Öffnungswinkel liefert `solve` Unsinn (54 mm statt gemessener 128 mm).
   Messen z. B. mit einem Brett bekannter Breite in bekanntem Abstand oder einem
   Schachbrett auf einem Bildschirm und `cv2.calibrateCamera`.
1. **Drehrichtung und -maßstab:** `python tools/calibrate.py turn`. Der Rider dreht
   sich 3 s auf der Stelle. Das Werkzeug prüft das Vorzeichen und gibt den Wert
   für `vyaw_limit_dps` aus. Misst über die Ego-Motion, braucht also Schritt 0.
2. **Kamerahöhe und -neigung:** Am zuverlässigsten die Höhe der Objektivmitte über
   dem Boden direkt messen (RiderPi bei Körperhöhe 85 mm: 128 mm) und die Neigung
   aus einer Wandkante bestimmen. Rider frontal vor eine Wand, Abstand Radaufstand →
   Bodenkante **nach** dem Aufstehen messen (er rollt dabei ~20 cm vor):
   `python tools/calibrate.py wall --cols 0.45:0.55 --debug wand.jpg`. Das
   Kontrollbild prüfen: Liegt die Wandbasis im Schatten, misst das Bodenmodell die
   Schattengrenze statt der Kante. Zweite Messung bei ~2 m zur Kontrolle, dann
   `python tools/calibrate.py solve <zeile1>:<m1> <zeile2>:<m2>`.
   `tilt_deg` gilt für die IMU-Rohpitch während der Messung: beide als Paar
   eintragen (`[camera] tilt_deg`, `[imu] pitch_ref_deg`). Für die Messung im
   Stillstand darf die Belichtung lang sein (eigene config.toml mit
   `exposure_us = 33000`), die Geometrie hängt nicht davon ab.
3. **Linsenabschattung** (für das Bodenmodell nötig): Laptop mit weißem Vollbild
   (Browser: `data:text/html,<body style="background:#fff;cursor:none">`, ⌃⌘F),
   Helligkeit maximal, so nah vor die Kamera, dass er das ganze Bild füllt:
   `python tools/calibrate.py shading` → `shading.json`, dann
   `[freespace] shading_file = "shading.json"`. Das Werkzeug lehnt ab, wenn die Fläche
   unter 75 % des Bildes deckt oder übersteuert.
4. **Vorwärtsmaßstab:** Annahme ist, dass 1 Firmware-Einheit = 1 m/s ist. Er geht
   nur in die Merkkarte ein. Am RiderPi bei v = 0,08 gemessen: ~12,5 cm/s, passt
   zur Annahme (12 cm/s).
5. **Rückwärts:** Die Firmware setzt Rückwärtsbefehle deutlich schwächer um. Am
   RiderPi fuhr er 2 s mit u8 147 (v = +0,15) 86 cm vorwärts, mit u8 109 (v = −0,15)
   nur 4 cm rückwärts. Im Stillstand rollt er nicht; beim Rückwärtsbefehl lehnt er
   sich um 2–3° und schwingt beim Anhalten nach vorn über. `[robot] reverse_gain`
   verstärkt negative v beim Senden; mit 2,0 (v = −0,15 → u8 90) kam die Strecke
   der Vorwärtsfahrt am nächsten; im Modus Hand fährt er damit vor- und rückwärts
   gleich schnell (Augenmaß, nicht genau vermessen).

## Benutzung

```
python -m xgonav --mode follow                    # Person folgen
python -m xgonav --mode approach --target chair   # Stuhl suchen und anfahren
python -m xgonav --mode explore
python -m xgonav --mode idle --record debug.mp4   # nur schauen, Overlay aufzeichnen
```

Bedienoberfläche: PHP (`web/`) hinter nginx, `https://riderpi/xgonav/`: Live-Overlay,
Status, Moduswechsel und **STOPP**. xgonav selbst liefert nur eine lokale Schnittstelle
auf `127.0.0.1:8088` (`/status`, `/ws`, `/frame`, `/stream`, `POST /mode|/stop|/pipeline|...`);
`api.php` nimmt nur bekannte Befehle per POST an und prüft die Parameter, das Live-Bild
reicht nginx direkt von xgonav durch. Die Oberfläche bekommt die Bilder über eine
WebSocket-Verbindung (`/xgonav/ws`, TLS endet in nginx) und bestätigt jedes Bild; erst
dann schickt xgonav das nächste, neueste. Der frühere MJPEG-Stream staute sich über
schwaches WLAN in den TCP-Puffern (bis 20 s Verzögerung). Das Overlay enthält dort keine
Statuszeilen (~10 ms je Bild gespart), die Werte stehen in der Statustabelle. Keine Authentifizierung,
also nur im LAN betreiben. Einrichtung am RiderPi (TLS macht nginx auf 443; früher
stunnel, das nach jedem Start und alle 24 h DH-Parameter rechnete, ~15 % CPU):

```
sudo apt-get install php8.2-fpm
sudo cp deploy/php-fpm-xgonav.conf /etc/php/8.2/fpm/pool.d/xgonav.conf   # listen.owner = nginx-Benutzer
sudo cp deploy/nginx-xgonav.conf /etc/nginx/sites-available/xgonav
sudo ln -s /etc/nginx/sites-available/xgonav /etc/nginx/sites-enabled/
sudo rm /etc/nginx/sites-enabled/default                # xgonav ist default_server auf 80 (-> https)
sudo systemctl disable --now rc-control                 # alte Fernsteuerung (rc_server) ist abgelöst
sudo systemctl disable --now stunnel4
sudo nginx -t && sudo systemctl restart php8.2-fpm nginx
```

Die Oberfläche ersetzt die frühere Fernsteuerung (`rc_server`, Node; nicht mehr erreichbar, `/` leitet auf `/xgonav/`):
**Handsteuerung** (Modus `manual`: gedrückt halten per Maus/Touch/Pfeiltasten, alle
0,2 s ein Sollwert, nach 0,5 s ohne Sollwert Stillstand; begrenzt auf `max_v`/`max_w`;
funktioniert auch bei gestoppter Bildverarbeitung, dann ohne Kamera und fast ohne CPU;
ohne Bodenmodell und Bahnprüfung, mit Sturz-Erkennung und Watchdogs), Höhe (u8 43–255,
mit Abstand zum Minimum), Aktionen 1–6/Stopp, LEDs inkl. Strobe, Emotionen aufs
Display, Roll (−20…20°, + = nach rechts, braucht Treiber-Patch 0004), Bein-Kalibrierung, Herunterfahren.
Emotionen spielt `tools/emotion.sh` ab: `deploy/install.sh` konvertiert die PNGs aus
`rc_server/emotions` einmalig nach `/opt/xgonav/emotions` (RGB565-Rohbilder, ~140 MB),
beim Abspielen werden nur noch Bilder ins FIFO kopiert (25 fps) statt sie per ffmpeg
zu dekodieren. Ohne Konvertierung spielt der Player die PNGs wie früher mit ffmpeg ab.
Automatisch: `cute` beim Hochfahren (rc.local), `sleep` vor dem Herunterfahren aus der
Oberfläche (api.php), aus xgonav (`[emotion]`, `xgonav/emotion.py`) `surprised`, wenn eine
Person erscheint (nach `gap_s` ohne Person), nach `lost_s` ohne Person `doubt` im Modus
follow bzw. `embarrassed` in den anderen Modi (einmal je Verlust), und `shy` bei Akku ≤ `battery_low` % (alle 5 min). Der zuletzt gestartete Player gewinnt
(`/tmp/lcd0.owner`, legt rc.local an); das LCD-Overlay pausiert, solange eine Emotion läuft.
LEDs, Aktionen, Höhe, Roll und Kalibrierung schreibt PHP direkt nach `/proc/XGORider`.
Zwei Rechte braucht `www-data` dafür:

```
sudo usermod -aG video www-data            # Emotionen: Schreiben nach /tmp/lcd0
sudo install -m 0440 deploy/sudoers-xgonav-poweroff /etc/sudoers.d/xgonav-poweroff   # nur "systemctl poweroff"
sudo visudo -c && sudo systemctl restart php8.2-fpm
```

Das Overlay erscheint auch auf dem 2"-Display (`/tmp/lcd0`, abschaltbar mit `lcd_fifo = ""`).

**Bildverarbeitung stoppen/starten** (Web-Knöpfe, `POST /pipeline?on=0|1`): hält die
Kamera an und verarbeitet keine Bilder mehr; der Rider bekommt Stillstand, der Modus
geht auf idle. Am CM4 sinkt die CPU-Last des Dienstes von ~340 % auf ~4 %, der
Webserver bleibt erreichbar. Nach dem Starten fährt er erst, wenn ein Modus gewählt
wird. Mit voll laufender Bildverarbeitung reicht der Strom im Ladehäuschen nicht zum
Laden.

Overlay: gelb = Bodengrenze je Spalte, türkis = Horizont (nickkompensiert),
Balken unten = Freiraum je Sektor, rot = Ziel-Track, violett = Bildbewegung,
blau = Fahrbefehl.

## Testen ohne Hardware

Einmalig eine Umgebung anlegen (am PC, kein Pi nötig):

```
python3 -m venv .venv && . .venv/bin/activate
pip install "opencv-python-headless>=4.6" "numpy>=1.24" pytest "onnxruntime>=1.16"
```

```
python -m pytest -q tests/                  # Unit-Tests (2 s); slow-Tests sind per pytest.ini ausgeschlossen
python -m pytest -q -m slow tests/          # Closed-Loop im Simulator (3 Tests, ~90 s)
python tests/sim_run.py follow 40 out.mp4   # Szenario mit Video
python tests/sim_run.py follow 40 --imu     # mit simulierter IMU (Vorzeichen/Versatz absichtlich falsch)
python -m xgonav --source video.mp4 --robot sim --mode follow   # eigenes Video
```

Der Simulator (`xgonav/sim.py`) rendert einen Raum mit Kisten und einer laufenden
Person per Raycasting, inklusive Nicken (2,5 Hz + Beschleunigungsneigung),
Kurszittern und Bewegungsunschärfe. Die Fahrbefehle bewegen die Kamera. Ein
Fake-Detektor liefert Ground-Truth-Boxen mit 180 ms Latenz und 5 Hz.

Ergebnisse (40-s-Läufe):

| Szenario | Ergebnis |
|---|---|
| Erkunden | nach allen Fixes 7/7 Läufe ohne Kollision (davor 1 Klemmer in 11 Läufen, Ursache: Rückwärtsfahrt war nicht bahngeprüft) |
| Folgen, Person 0,3 m/s, Soll 1,2 m | Ø 1,8 m, min 1,4 m, keine Kollision |
| Nicklage nur aus dem Bild | 0,56° Streuung (langsame Neigung unbeobachtbar) |
| Nicklage IMU + Bild (Komplementärfilter) | 0,13° Streuung |
| Entfernung Person (Wahrheitsbox), nur Bild | −16 cm Bias, ±23 cm |
| Entfernung Person (Wahrheitsbox), IMU + Bild | −3 cm Bias, ±7 cm |
| IMU-Synchronisation | findet eingebautes Vorzeichen −1 und −8 ms Versatz (Korrelation 0,99) |
| Ego-Motion vs. Wahrheit | 0,03° Nick-/Gierfehler pro Frame |
| Treiber vs. ESP32-Emulator | 49 Hz bei Soll 50 (Block: 97 Hz bei Soll 100), Pitchfehler 0,06° Median, Parser fängt sich nach Drops/Störbytes/kaputten Prüfsummen |

Getestet mit Python 3.13/OpenCV 5 und mit Python 3.11/OpenCV 4.6/numpy 1.26 (Bookworm-Stand).

## Grenzen und offene Punkte

- **Nur teilweise auf echter Hardware getestet** (RiderPi, CM4, Kernel
  6.6.74+rpt-rpi-v8, ESP32-Firmware R-1.1.3). Geprüft: Patches 0001/0002 gebaut und
  geladen (IMU ~49 Hz, Kernel-Watchdog löst aus), xgonav im Modus idle
  (Rechenzeiten unten), Kamerageometrie (unten). **Nicht** geprüft: Fahren unter
  xgonav (follow/explore/approach), IMU-Sync im Balancieren, Patch 0003.
- **UART im Rohmodus nötig.** Ohne `-ixon -icrnl` in der `stty`-Zeile von
  `rc.local` stoppt ein zufälliges XOFF-Byte (0x13) in einer ESP32-Antwort den
  Sender und der ganze serielle Verkehr hängt (beim schnellen IMU-Polling nach ~1 s).
  Patch 0002 ergänzt die Flags. Live beheben, ohne neu zu laden: in Python
  `termios.tcflow(fd, TCOON)` und Flags per `tcsetattr(..., TCSANOW, ...)`;
  `stty` selbst hängt dann, weil es auf das Leeren der Ausgabe wartet.
- **IMU-Kurs ignoriert langsame Drehungen.** Die Original-Firmware integriert den
  Kurs nur bei |Drehrate| > 2°/s (sichtbar in der Rekonstruktion von R-1.1.6,
  github.com/LuwuDynamics/xgorider2_rl, `firmware/src/control/rider_leg_imu.cpp`).
  Gemessen am RiderPi, 60 s balancierend ohne Befehle: IMU +2,5°, Bild −2,6°.
  Die Kurshaltung der Firmware (Register 0x0A, standardmäßig an) und
  `settings/force_yaw` im Treiber nutzen denselben Kurs und können diese Drift
  nicht sehen. Für Kurstreue zählt in xgonav `vis_yaw` aus der Ego-Motion; die IMU
  taugt für Pitch und schnelle Drehungen.
- **Drehbefehle laufen nach.** Die Firmware filtert `speed_z` und integriert es zu
  einem Kurs-Sollwert; nach dem Stoppbefehl dreht der Rider noch 10–17° weiter.
  Kleine Befehle (bis etwa u8 128 ± 6) ignoriert sie offenbar. `tools/nudge.py` dreht
  deshalb zweistufig (grob bis 10° vor dem Ziel, dann kurze Impulse mit Ausschwingen).
  Beim Balancieren wandert der Rider außerdem langsam in Position und Kurs.
- **Kamerageometrie RiderPi (gemessen, nur in der Rider-config.toml eingetragen):**
  Öffnungswinkel ~91° (f ≈ 314 px; Brett: ~300 px, 16 Schachbrettbilder je nach
  Modell 297–316 px), Verzeichnung gering (≤ 4 px im Bereich x 40–571, Ecken nicht
  vermessen), Kamerahöhe 128 mm bei Körperhöhe 85 mm (`mount_offset_mm = 43`),
  Neigung 1,39° bei IMU-Pitch −2,7° (Brett und zwei Wandmessungen, Streuung
  0,25°). **Default in `config.py` und Simulator sind noch 62,2°**, d. h. der
  Simulator rendert nicht wie die echte Kamera.
- **ESP32-Annahmen:** Antworten tragen die angefragte Adresse, Schreibbefehle
  werden nicht beantwortet (wie in der Hersteller-Lib). Wie oft die Firmware ihre
  Lagewerte intern aktualisiert, ist unbekannt. `serial_stats` zeigt Timeouts und
  „unsolicited“-Frames, falls etwas nicht passt.
- **Rechenzeit auf dem CM4 (gemessen, 640×480 @ 30 fps, Idle):** Hauptschleife
  ~12 ms/Frame ohne Detektor, ~24 ms mit YOLO11n@256 auf 2 Threads (~145 ms
  pro Inferenz, ~7 Hz). Mit 3 Threads oder 320 px verdrängt der Detektor die
  Hauptschleife über das 33-ms-Budget. Der größte Posten ist der optische Fluss
  (`ego.lk_*`). Vorab gebaute LK-Pyramiden lassen sich über die Python-Bindung
  nicht übergeben; jeder Flussaufruf baut sie für beide Bilder neu.
- **Kamera bei wenig Licht:** Die feste Belichtung von 4 ms ist für typische
  Wohnräume am Abend zu kurz: bei 15–50 Lux ist die Verstärkung am Anschlag (8,0)
  und das Bild fast schwarz (mittlere Helligkeit 3/255; automatisch mit 33 ms:
  20–30/255). Mehr Licht oder längere Belichtung (`camera.exposure_us`, mehr
  Bewegungsunschärfe); eine Automatik mit Obergrenze fehlt noch.
- **Ohne Patch 0002 / ohne Sync** bleibt das langsame Nicken unbeobachtbar
  (1° ≈ 40 % Entfernungsfehler bei 2 m). Dann gleicht die Größenfusion aus, wie
  vorher. Im Debug-Web steht unter `pitch_src`, welche Quelle gerade aktiv ist.
- **Folgeabstand** liegt im Simulator bei ~1,5 m ohne und ~1,4 m mit IMU statt 1,2 m,
  wenn die Person geht (vorher ~1,8 m). Das ist im Wesentlichen die Totzone
  (`dist_deadband`) plus etwas Nachlauf. Abstand und Fluchtgeschwindigkeit des Ziels
  schätzt ein Kalman-Filter, der nur neue Detektionen als Messung nimmt; die
  Merkkarte räumt Punkte frei, wo neuer Freiraum gesehen wird.
- **Bodenmodell** braucht einen halbwegs einfarbigen Boden. Sehr gemusterte
  Teppiche oder Fliesen mit harten Fugen erzeugen Schein-Hindernisse. Am RiderPi
  machten Laminatfugen mit dem alten Kantengewicht (0,15) jede zweite Aufnahme zur
  Blockade; mit `edge_weight = 0.4`, `min_obst_rows = 6` keine mehr (40 Rohbilder),
  echte Hindernisse (Person, Schrank) unverändert erkannt.
- **Erkunden im Simulator stößt gelegentlich an:** Bei 40-s-Läufen in 2 von 5 Läufen
  mit den alten Bodenparametern, ähnlich mit den neuen (Zufallsdrehungen). Der
  30-s-Slow-Test besteht. Ursache noch nicht untersucht.
- **Linsenabschattung am RiderPi:** Zum Rand fallen Blau/Grün auf ~20 %, Rot auf 1–9 %
  (ab ~250 px vom Zentrum; gemessen mit Weißbild). Ohne Korrektur hält das Farbmodell
  die grünstichigen Ränder für Hindernisse und die Bahnprüfung blockiert jede Fahrt.
  `xgonav/shading.py` gleicht pro Kanal aus (bis `shading_max_gain`, Default 8); wo
  mehr nötig wäre, ist die Farbe nicht messbar: diese Pixel sind nie Hindernis, und
  die Merkkarte räumt dort nichts frei (`FreeSpace.known_from`). Folge: Der Boden ist
  in den mittleren ~6/8 der Bildbreite ab 18–25 cm messbar, die äußeren Achtel und
  die unteren Ecken sind blind; nahe Hindernisse seitlich vorn sieht das Bodenmodell
  nicht. Ein Kameramodul mit passendem Objektiv wäre die eigentliche Lösung.
  Ohne Freiraum-Erkennung
  (`freespace.enabled = false`) sind nur `idle` und `follow` erlaubt; `explore` und
  `approach` werden abgelehnt (Web: HTTP 409), beim Start fällt xgonav auf `idle` zurück.
- `roll` im Treiber schrieb ursprünglich auf 0x61 (Roll-Balance an/aus) statt auf den
  Roll-Winkel 0x36. Patch 0004 korrigiert das: `roll` nimmt Grad −20…20 (0x36, linear
  0..255), die Roll-Balance liegt jetzt unter `roll_balance` (0/1).
