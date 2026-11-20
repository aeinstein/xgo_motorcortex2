"""Zentrale Konfiguration. Defaults hier, Overrides per TOML (--config)."""
from __future__ import annotations

import dataclasses
import logging
import tomllib
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class RobotCfg:
    proc_dir: str = "/proc/XGORider"
    # Mapping laut XGO-Lib: u8 = 255/(2*L) * (v + L); 128 ~ Stillstand
    vx_limit: float = 1.8          # Einheit der Firmware bei u8=0/255
    vyaw_limit_dps: float = 360.0  # Grad/s bei u8=0/255
    # Sicherheitsgrenzen für den Autonomiebetrieb (Anteil des Vollausschlags)
    # Nutzer 2026-11-13: beide auf 50 % des Firmware-Bereichs (vorher 0,15 / 0,30). 0,5 sind rechnerisch
    # ~0,9 m/s und 180 °/s; über 0,24 (50 cm/s) ist nie gefahren worden.
    max_v: float = 0.5
    max_w: float = 0.5
    # Tempo (Anteil, 0,05..1): jeder Fahr- und Drehbefehl aller Modi wird nach der Begrenzung auf max_v/max_w
    # damit malgenommen (Robot.drive). Startwert; zur Laufzeit über den Temporegler im Web (POST /tempo),
    # gilt bis zum Neustart. Firmware-Aktionen (POST /action) fahren mit eigener Geschwindigkeit.
    tempo: float = 1.0
    # Harte Obergrenze für den gesendeten Fahr- und Drehwert, nach allen Faktoren (reverse_gain,
    # manual_forward_gain) und der Geschwindigkeitsregelung: nie mehr als dieser Anteil des Vollausschlags
    max_sent: float = 0.5
    # Rückwärts war 2026-11-02 viel schwächer (2 s mit u8 109 ~4 cm, mit 147 vorwärts 86 cm), daher
    # werden negative v beim Senden mit reverse_gain multipliziert; max_v begrenzt davor den
    # Sollwert. 1.0 = symmetrisch. Mit den Firmware-Patches vom 2026-11-05 setzt die Firmware beide
    # Richtungen gleich um (Sollwert -> Raddrehzahl 2,67 vorwärts, 2,65 rückwärts).
    reverse_gain: float = 2.0
    # Derselbe Faktor vorwärts, nur in der Handsteuerung. Die autonomen Modi bleiben bei 1: im
    # Simulator (--dyn, 2026-11-05) gab Faktor 2 dort 1 Kollision in 8 Läufen und Zum Start kam nur
    # noch 1 von 4 Mal an (Faktor 1: 0 Kollisionen, 4 von 4); die Freiraum-Schwellen passen nicht dazu.
    manual_forward_gain: float = 2.0
    # Geschwindigkeitsregelung: Die Firmware regelt die Fahrgeschwindigkeit praktisch nur proportional, auf
    # Teppich bleibt der Rider deshalb unter dem Soll. Mit speed_ctrl vergleicht xgonav im Regelschritt Soll
    # (gesendeter Wert x vx_limit) und Ist (Räder), beide über speed_ctrl_window_s gemittelt, und stellt den
    # gesendeten Wert nach: Korrektur += (Soll - Ist) / vx_limit x dt / speed_ctrl_ti_s, höchstens
    # +-speed_ctrl_max (normiert). Ohne frische Raddaten, im Stand, unter dem Totband der Firmware und bei
    # Richtungswechsel ist die Korrektur 0. Steht er an einem Hindernis an, drückt er um speed_ctrl_max stärker.
    # speed_ctrl_lag_s: So träge folgt das Soll dem Fahrwert (Anlaufen des Riders), sonst lädt sich die
    # Korrektur bei jedem Anfahren auf. Abgestimmt am Trägheitsmodell des Simulators (sim._dyn, 2026-11-12):
    # ohne Modell 31-46 % Überschwingen beim Anfahren auf ebenem Boden, mit 0,8 s und ti 2 s 5 %; ein
    # Fehlbetrag von 0,04 ist dann nach ~3 s zu 95 % ausgeglichen. Am Rider nicht gefahren.
    speed_ctrl: bool = False
    speed_ctrl_window_s: float = 0.3
    speed_ctrl_ti_s: float = 2.0
    speed_ctrl_lag_s: float = 0.8
    speed_ctrl_max: float = 0.10
    # Die Beine haben eine Totzone: bei gleichem Befehl steht der Rider je nach Vorgeschichte bei
    # +0,7..+3,4° Roll (2026-11-05). Ein kurzer Impuls der Nivellierung in der Firmware richtet ihn
    # aus, danach bleibt er so stehen (dauerhaft an schwankt er mehr). Auslöser: einmal nach jedem
    # Aufstehen (level_after_standup) und der Knopf im Web (POST /level).
    level_pulse_s: float = 3.0
    level_after_standup: bool = True
    accel_v: float = 0.8           # max. Änderung von v pro Sekunde (normiert)
    accel_w: float = 2.0
    keepalive_s: float = 0.3       # Sollwert periodisch neu schreiben
    stall_timeout_s: float = 0.5   # kein drive() mehr -> Stopp (Userspace-Watchdog)
    kernel_watchdog_ms: int = 1000 # nur wirksam mit Kernel-Patch (settings/motion_timeout_ms)
    # Takt der Firmware-Aktionen 3–5 (jeder 21. Durchlauf ihrer Schleife, 260–333 Hz): lange Grenze. So lange
    # setzt xgonav Keepalive und Kernel-Watchdog aus, sonst hält der Keepalive die Aktion an (robot.FW_ACTIONS).
    action_tick_s: float = 0.08
    height_min_mm: float = 60.0
    height_max_mm: float = 120.0   # min/max legen nur die Skala mm -> u8 fest
    height_limit_u8: int = 205     # höchster Sollwert; bei 235 Stopp 252 (2026-11-06, Ursache offen)
    # LED-Helligkeit (Firmware-Patch 0016, n/256, settings/led_brightness). Das Web speichert sie in led_file,
    # rc.local setzt sie beim Booten. Beim Start blitzt xgonav led_start_s lang mit 255 (Zeichen „läuft“),
    # danach wieder der gespeicherte Wert; 0 = ohne Blitz, nur setzen.
    led_file: str = "/var/lib/xgonav/led_brightness"
    led_default: int = 51          # 20 %, solange nichts gespeichert ist
    led_start_s: float = 0.2


@dataclass
class CameraCfg:
    source: str = "picam"          # "picam", Gerätenummer "0" oder Videodatei
    width: int = 640
    height: int = 480
    fps: int = 24
    # Sensormodus (Rohformat), z. B. "1296x972"; leer = libcamera wählt (beim OV5647 640x480).
    # 640x480 ist 2x2 gebinnt + 2x2 übersprungen, und der Kernel-Treiber (6.6) gibt dafür eine
    # ~6 % zu kleine Pixelrate an: der Sensor läuft dann 26,5 statt 25 fps. 1296x972 ist reines
    # 2x2-Binning (weniger Rauschen), Zeitbasis stimmt; der ISP skaliert auf width x height.
    sensor_mode: str = "1296x972"
    # Sensorausschnitt [x, y, w, h] in Vollbild-Pixeln (ScalerCrop); leer = ganzer Modus.
    # [16, 0, 2560, 1920] = Ausschnitt des 640x480-Modus, damit hfov und tilt gültig bleiben.
    scaler_crop: list = field(default_factory=lambda: [16, 0, 2560, 1920])
    # Bilddauer-Korrektur, nur nötig im 640x480-Modus (dort ~1.061): angefragt = 1/fps * Faktor.
    frame_clock_corr: float = 1.0
    exposure_us: int = 20000        # kurze Belichtung gegen Bewegungsunschärfe; 0 = Auto
    analogue_gain: float = 16.0     # feste Verstärkung 1..63,9 (OV5647); 0 = Auto
    # Kontrast und Sättigung im ISP der Kamera (1,0 = Standard, 0,5..4 über den Regler im Web zur Laufzeit).
    # Idee 2026-11-19: an der Quelle überdrehen, damit FloorNet kräftigere Kanten/Farben bekommt; der Browser zieht
    # es für die Anzeige wieder herunter (CSS-Filter). FloorNet ist auf 1,0 trainiert: überdrehte Bilder sind für
    # das Netz eine andere Eingabe, erst ein Training damit kann davon profitieren. Je Bild in meta.jsonl (cam).
    contrast: float = 1.0
    saturation: float = 1.0
    brightness: float = 0.0         # Helligkeit im ISP (0 = Standard, -0,5..0,5 über den Regler im Web)
    # RiderPi: aus Drehungen gegen den IMU-Kurs gemessen (2026-11-07, f = 310 px @ 640, 304–313 in drei
    # Aufnahmen; Verzeichnung vernachlässigbar). Der Zollstock (88,5°) legt nur f·h fest. RPi Cam v1 hätte 62,2.
    hfov_deg: float = 91.8
    # Linsenverzeichnung: Das Bild wird direkt nach der Kamera entzerrt (camera.lens_maps, eine Tabelle, cv2.remap),
    # alles Weitere rechnet mit dem entzerrten Bild als Lochkamera. Modell: gemessenes Pixel -> ideales Pixel,
    # radial um lens_center: ideal = c + q * (1 + g(r) / r), g(r) in px an den Stützstellen lens_knots_px (linear
    # dazwischen). RiderPi 2026-11-18 aus ~67 000 geraden Kanten von 12 Fahrten (Fugen, Rahmen), geprüft an 4
    # anderen: bis r 340 px höchstens 2 px, in den Ecken bis 18 px (Fugen am unteren Rand: Bogen 6 px -> < 1 px).
    # Leere Listen = aus (Simulator). Nur die echte Kamera entzerrt; Rohaufnahmen sind ab dann entzerrt
    # (meta.jsonl: cam.entzerrt).
    lens_knots_px: list = field(default_factory=lambda: [0, 40, 80, 120, 160, 200, 240, 260, 280, 300, 320, 340, 360, 380, 400, 420, 440, 460])
    lens_g_px: list = field(default_factory=lambda: [0.0, -0.38, -0.53, -0.24, 0.43, 1.5, 2.13, 1.92, 1.43, 0.63, 0.07, 0.46, 3.16, 9.23, 17.95, 26.69, 35.43, 44.17])
    lens_center: list = field(default_factory=lambda: [322.5, 232.5])
    # Einbaulage der Kamera, in derselben Tabelle herausgedreht (um die Bildmitte W/2, H/2; Nick bleibt tilt_deg,
    # sonst fehlten unten Zeilen Nahboden). Danach ist das Bild waagerecht, wenn die IMU Roll 0 meldet, und die
    # Fahrtrichtung liegt in Spalte W/2. Rand ohne Bild wird mit dem Randpixel aufgefüllt (rechts ~13 Spalten).
    # lens_roll_deg: IMU-Rohroll, bei dem das Kamerabild waagerecht ist (RiderPi 1,25: drei Rundumblicke im Flur
    # 2026-11-17, früher [imu] roll_ref_deg).
    lens_roll_deg: float = 1.25
    # Gier des Einbaus: Fahrtrichtung rechts der Bildmitte (Grad), in der Entzerrtabelle herausgedreht. RiderPi
    # atan(13 / 309,6) = 2,40: Fluchtpunkt bei Geradeausfahrt Spalte 333, 8 Fahrten 2026-11-08 (früher cx_offset_px).
    lens_yaw_deg: float = 2.40
    # Entzerren im C++-Kern (native/linsefast.cpp, ein Kern, bitgleich zu cv2.remap; nur aktiv nach Selbstprüfung
    # beim Start). False oder Kern fehlt: cv2.remap (CM4: 6,4 ms je Bild auf vier Kernen).
    lens_native: bool = True
    # Aufnahme direkt über libcamera (native/camfast.cpp, libxgocam.so): das Bild wird aus dem DMA-Puffer heraus
    # entzerrt, die Pixel laufen nicht durch Python (eine Kopie von 0,9 MB je Bild entfällt, kein picamera2-Faden).
    # Nur mit lens_native bzw. ohne Linsenmodell; entzerrt nur nach derselben Selbstprüfung gegen cv2.remap.
    # False, Bibliothek fehlt oder Kamera lässt sich so nicht öffnen: picamera2.
    capture_native: bool = True
    # Geometrie für Entfernung per Bodenebene (KALIBRIEREN, siehe README)
    mount_offset_mm: float = 43.0  # Kamerahöhe über Boden = Körperhöhe + Offset (nur ohne cam_h_u8)
    # Kamerahöhe (Linsenmitte über Boden, mm) = a + b * Höhen-Sollwert u8, [a, b]. RiderPi 2026-11-07 im Stand
    # gemessen: u8 106/150/200 -> 122/125/132,5 mm -> [109.4, 0.1126]. Die Körperhöhe in mm (60..120 über
    # 0..255) steigt doppelt so schnell wie die echte; darum nicht Körperhöhe + Offset. Leer = mount_offset_mm.
    # Gitter am Boden 2026-11-10 bestätigt die Gerade (u8 128: 124-126 mm, u8 205: 131-133 mm); der Faktor
    # 0,921 aus der Klebeband-Probe vom 2026-11-07 ist damit raus.
    cam_h_u8: list = field(default_factory=lambda: [109.4, 0.1126])
    # Linsenmitte vor der Radachse (mm). Ursprung von Merkkarte und Raumkarte ist die Radachse (dort dreht der
    # Rider, dort zählen Räder und Sicherheitsradius); Bodenpunkte werden um diesen Versatz nach vorn eingetragen.
    # RiderPi: CAD 36,9, Bodenprobe 34/41 -> 36. 0 = Kamera auf der Achse (Simulator rendert dann wie bisher).
    cam_fwd_mm: float = 36.0
    tilt_deg: float = 5.18        # >0 = Kamera schaut nach oben
    loop_file: bool = True         # Videodatei endlos abspielen (Replay)
    realtime_file: bool = True     # Videodatei in Echtzeit statt so schnell wie möglich


@dataclass
class ImuCfg:
    enabled: bool = True
    proc_dir: str = "/proc/XGORider"
    imu_hz: int = 50               # wird beim Start nach settings/imu_hz geschrieben
    # IMU/Rad-Strom der Firmware (Patch 0014) mit eigener Messzeit statt Abfrage; > 0 wird beim Start nach
    # settings/imu_stream_hz geschrieben (Treiber mit Strom-Unterstützung). Ohne Strom fragt der Treiber wie bisher ab.
    stream_hz: int = 100
    # Abgesetzt (Rider balanciert nicht) braucht niemand 100 Hz: Strom und Lesen auf diese Rate, beim Aufstehen wieder
    # stream_hz und poll_s. Spart am CM4 ~6 % eines Kerns (2026-11-10). 0 = nie umschalten.
    idle_hz: int = 5
    # Vorzeichen und Zeitversatz fest (RiderPi, gemessen 2026-11-17 mit tools/imu_versatz.py). 0 bzw. None =
    # automatisch aus der Bild-Korrelation (ImuSync, Simulator): rechnet dann einmal je Sekunde, gilt erst nach der
    # ersten Bewegung und fällt bei schwacher Korrelation auf den Bildnick zurück.
    pitch_sign: int = -1           # +1/-1; 0 = automatisch
    time_offset_ms: float | None = 10.5   # IMU-Zeit = Bildzeit + dies; None = automatisch
    # IMU-Rohpitch bei der Kamerakalibrierung (calibrate.py wall): -4.43; seit 2026-11-17 -4.23 (Nick +0,2°): Flur
    # mit 2,04 m befahrbarer Breite (Nutzer) kam in drei Rundumblicken mit Roll-Korrektur auf 1,98 m, mit +0,25° auf 2,03-2,05 m.
    # Seit 2026-11-18 -6.23: Firmware 0021 (Lage aus dem Schwerkraftvektor, Nick-Kreisel kalibriert) liefert den Nick ~2° tiefer
    # (vorher stand er wegen des unkalibrierten Nick-Kreisels ~2,3° zu hoch); Nutzer am Bild im Browser geprüft.
    # Seit 2026-11-19 -5.0: Flurfahrt 20261119-122328 (Flur 2,04 m) kam mit -6.23 auf 1,53 (hin) / 1,67 m (zurück),
    # 2,04 m bei +1,4° / +1,0°.
    # Wirksam ist nur tilt_deg + pitch_ref_deg (geometry: Blickwinkel = tilt_deg + pitch_ref_deg - IMU-Nick).
    pitch_ref_deg: float = -5.0
    # Skalenfehler des Firmware-Kurses (Gier), je Drehrichtung getrennt: Kursänderungen nach links mal
    # yaw_scale_left, nach rechts mal yaw_scale_right. RiderPi 2026-11-18, auf der Stelle an einer Marke, Winkelmesser:
    # 10 × 360° rechts echt 3560°, IMU 3551,3° (1,0025); 10 × links echt 3556°, IMU 3548,4° (1,0021); 4 × links
    # 1431° / 1428,0°. Eine erste Messung (4 × links 1470° / 1453,3°) hat sich nicht wiederholt. Die aufgezeichneten
    # Proben sind schon korrigiert.
    yaw_scale_left: float = 1.0021
    yaw_scale_right: float = 1.0025
    # Schneller dreht der Rider nie (°/s): Ein größerer Kurssprung zwischen zwei Proben ist kein Drehen, sondern ein
    # ESP-Neustart (Kurs beginnt bei 0) oder ein Lesefehler; er wird nicht in den Kurs übernommen (ImuSource.yaw_jumps).
    yaw_jump_dps: float = 500.0
    # Roll: Die Bodengrenze des Netzes wird je Bild um den IMU-Roll gedreht, bevor sie in Karten und Sicherheit
    # geht (freespace.roll_columns); der Einbau-Roll der Kamera ist schon aus dem Bild gedreht ([camera]
    # lens_roll_deg). Ohne das lag dieselbe Flurwand im Rundumblick je nach Bildrand bei 0,7 oder 1,2 m statt 1,0 m (Wand
    # 33-48 cm dick, mit Korrektur 10-14 cm; drei Rundumblicke 2026-11-17, Fahrten 144500, 151254, 153930).
    # use_roll False = aus (Simulator).
    use_roll: bool = True
    sigma_deg: float = 0.3         # Restunsicherheit der Nicklage mit IMU
    min_corr: float = 0.6
    sync_range_ms: float = 100.0
    sync_step_ms: float = 4.0
    max_age_s: float = 0.3
    # imu_log so oft im Faden einlesen (s), solange die Bildschleife es nicht selbst tut. Mit laufender
    # Bildverarbeitung liest die Bildschleife je Bild direkt vor imu_frame (ImuSource.refresh), der Wert zum Bild
    # ist so frisch wie möglich. Vorher 0,01 im Faden: 100 Lesen/s, je Lesen formatiert der Treiber 128 Zeilen,
    # am RiderPi ~5,7 % eines Kerns (2026-11-19). Bei 0,05 ohne refresh lag der neueste Wert bis ~50 ms zurück.
    poll_s: float = 0.05


@dataclass
class EgoCfg:
    proc_width: int = 320          # Auflösung für optischen Fluss
    max_features: int = 150
    min_features: int = 60
    quality: float = 0.01
    min_distance: int = 8
    ransac_px: float = 2.0
    lk_win: int = 11               # Fenster des Lucas-Kanade-Flusses (px, Arbeitsauflösung)
    lk_levels: int = 3             # Pyramidenstufen über der Basis
    lk_iters: int = 10             # max. Iterationen je Stufe
    lk_fb_check: bool = True       # Vorwärts-Rückwärts-Prüfung (zweiter Flussaufruf)
    smooth_alpha: float = 0.15     # Tiefpass für Stabilisierungsanzeige
    # C++-Kern xgonav/native/libxgofast.so nutzen, wenn gebaut (make -C xgonav/native): gleiche
    # Rechnung, aber eine Bildpyramide je Bild statt vier. Fehlt er, rechnet Python.
    native: bool = True


@dataclass
class DetectorCfg:
    enabled: bool = True
    # NCNN-Exportordner (model.ncnn.param/.bin, metadata.yaml) oder .onnx. CM4 2026-11-05:
    # NCNN 93 ms statt 112 ms (onnxruntime), gleiche Boxen. Fehlt das Modell oder das Paket
    # ncnn, wird fallback_model genommen.
    model: str = "models/yolo11n_256_ncnn"
    fallback_model: str = "models/yolo11n_256.onnx"
    input_size: int = 256
    conf: float = 0.35
    iou: float = 0.45
    threads: int = 2               # CM4 gemessen: 3 Threads verdrängen die Hauptschleife (>33 ms)
    sharpness_rel: float = 0.6     # Frame nur nehmen, wenn Schärfe >= rel * Median
    max_wait_s: float = 0.25       # spätestens dann auch unscharfe Frames nehmen
    # Höchstrate der Detektion. 0 = ohne Pause (CM4: ~7 Hz bei 140 ms/Bild, ~2 Kerne).
    # 5 Hz spart rund ein Drittel davon; der Simulator-Detektor liefert ebenfalls 5 Hz.
    max_hz: float = 5.0
    # Wählbare Ziele für follow/approach; nur Erkennung, nie Hindernis. Leer = alle COCO-Klassen.
    classes: list[str] = field(default_factory=lambda: ["person", "chair", "couch", "dining table", "potted plant", "bed", "dog", "cat"])


@dataclass
class TrackerCfg:
    iou_match: float = 0.25
    max_lost_s: float = 1.5
    min_hits: int = 2


@dataclass
class FreespaceCfg:
    # Bodengrenze von FloorNet (xgonav/floornet.py, C++-Kern). Es gibt keinen Ersatzweg: ohne Netz ist der Rider
    # blind und fährt nicht selbst (safety() steht still), Handsteuerung geht weiter.
    enabled: bool = True
    # FloorNet in eigenem Thread (FreeSpaceWorker): nimmt der Hauptschleife jedes 2. Bild die Netzzeit ab, das
    # Ergebnis kommt dafür 1-3 Bilder später an. False = im Takt (Simulator, Tests).
    threaded: bool = True
    width: int = 160               # Spalten der Arbeitsauflösung (Karten, Sektoren)
    sectors: int = 5
    # Nach Start der Bildverarbeitung, Aufstehen und geänderter Belichtung so lange keine Bodengrenze übernehmen
    # (Belichtung und Lage regeln sich ein).
    restart_hold_s: float = 1.0
    # Gewichte von FloorNet. "" = kein Netz, also blind.
    net: str = "models/floornet.bin"
    # Ordner mit wählbaren FloorNet-Gewichten (*.bin). Im Web (Kasten Bildverarbeitung) wählbar: xgonav lädt das
    # Netz neu und zeigt net (dann ein Symlink, deploy/install.sh legt ihn an) auf die Datei. "" = nicht wählbar.
    net_dir: str = "models/floornet"
    # Einzelne Zacken herausnehmen (1-3 Spalten breite Sprünge an senkrechten Kanten): Weicht eine Spalte um
    # mehr als net_outlier_px Vollbildzeilen vom Median der 7 Spalten um sie ab und liegt beim Medianwert mindestens
    # net_outlier_ratio der Masse ihres eigenen Gipfels, gilt das beste Fach beim Medianwert. 0 = aus.
    net_outlier_px: float = 25.0
    net_outlier_ratio: float = 0.3
    # Kante über die Zeit beruhigen (floornet.ZeilenGlaetter): die letzten net_smooth_n Ausgaben werden über Pose
    # und Nicken auf das aktuelle Bild umgerechnet, je Spalte gilt der Median. 3: ein Ausreißer in einem einzelnen
    # Bild verschwindet, ein neues Hindernis erscheint ein Netzbild (83 ms) später. 0 oder 1 = aus. Braucht die
    # Kartenpose; ohne sie gilt die Ausgabe unverändert.
    net_smooth_n: int = 3


@dataclass
class MapCfg:
    max_age_s: float = 4.0         # Kartenpunkte so lange behalten
    # Ein Punkt blockiert die Bahn erst, wenn in seiner Nähe (confirm_radius_m) Punkte aus
    # mindestens confirm Beobachtungen liegen. 1 = jede Einzelbeobachtung zählt (wie bisher).
    confirm: int = 3
    confirm_radius_m: float = 0.08
    # Sicherheitsradius der Bahnprüfung: ein bestätigter Punkt näher als so an der Bahn bremst (Lücken ab ~0,36 m).
    radius_m: float = 0.18
    # Derselbe Radius, solange der Wegfolger eine Engstelle fährt ([home] tight_m). Der Rider ist laut CAD
    # 11,5 cm breit (halbe Breite 5,8 cm); 0,08 lässt ihn mittig durch eine 25 cm breite Lücke (Punkte bei
    # ±12,5 cm) mit 4,5 cm Spiel für Kurs und Rauschen.
    radius_tight_m: float = 0.08
    # Mitdrehen, Eintragen (freiräumen, ausdünnen, vergessen), Bestätigen und Bahnprüfung im C++-Kern
    # (xgonav/native/navfast.cpp), wenn gebaut; gleiches Ergebnis wie numpy
    native: bool = True


@dataclass
class BehaviorCfg:
    mode: str = "idle"             # idle | follow | explore | approach | home
    target_class: str = "person"
    follow_dist_m: float = 0.9
    approach_dist_m: float = 0.5
    kp_turn: float = 2.5           # 1/s: Soll-Drehrate = kp * Winkelfehler
    kd_turn: float = 0.25          # s
    kp_dist: float = 0.35          # normiertes v pro Meter Abstandsfehler
    dist_deadband: float = 0.10    # m
    ff_dist: float = 0.8           # Vorsteuerung mit geschätzter Zielgeschwindigkeit
    cruise_v: float = 0.15
    backup_v: float = 0.10
    search_w: float = 0.07
    turn_w: float = 0.21           # Erkunden: Drehrate beim Wegdrehen (früher fest 1,5 x search_w)
    lost_search_s: float = 1.0
    search_timeout_s: float = 10.0
    blocked_free: float = 0.22     # Freiraum im Zentrum darunter = blockiert
    slow_free: float = 0.55        # darunter proportional abbremsen
    ttc_avoid_s: float = 1.2       # Erkunden: Time-to-contact darunter -> ausweichen
    ttc_stop_s: float = 0.8        # Sicherheitsschicht: darunter keine Vorwärtsfahrt
    # Anschub: Drückt er vorwärts, ohne voranzukommen (PathFollower.PRESS_S), und zeigt das Bodenmodell
    # vorn noch push_free_m frei, fährt er push_s lang mit push_v statt gleich zurückzusetzen. Höchstens
    # push_n Mal, bis er wieder 0,3 m weitergekommen ist. Anlass: RiderPi 2026-11-17, Rille im Teppich
    # vor der Heizung, 15 s mit 0,25 gedrückt, Räder standen. Am Rider nicht gefahren. push_n 0 = aus.
    push_v: float = 0.45
    push_s: float = 0.6            # mit robot.accel_v 0,8/s erreicht er 0,45 erst nach ~0,4 s
    push_n: int = 2
    push_free_m: float = 0.4
    height_cruise_mm: float = 85.0
    height_search_mm: float = 100.0
    height_fast_mm: float = 80.0


@dataclass
class OutputCfg:
    web_host: str = "127.0.0.1"    # Schnittstelle nur lokal; Oberfläche über nginx (web/, deploy/nginx-xgonav.conf)
    web_port: int = 8088           # 0 = aus
    web_fps: int = 10
    # JPEG für die Anzeige im Hardware-Kodierer (native/jpegfast.cpp, bcm2835-codec-encode_image) statt cv2.imencode
    # (RiderPi 640x480: 4,5 ms CPU je gesendetem Bild). Geprüft beim ersten Bild, sonst/bei Fehlern cv2.
    jpeg_native: bool = True
    jpeg_dev: str = "/dev/video31"
    lcd_fifo: str = ""             # Kamera-Overlay aufs Display, "" = aus; WLAN-Einrichtung und pet-Augen immer (emotion.fifo)
    lcd_fps: int = 15             # CM4 gemessen: kostet die Hauptschleife nichts Messbares
    record_dir: str = "~/fahrt"   # Rohaufzeichnung (Knopf in der Oberfläche): je Aufnahme ein Unterverzeichnis
    record_min_free_mb: int = 500 # darunter schaltet sie sich ab
    # JPEG der Rohaufzeichnung: im Hardware-Kodierer (mit jpeg_native) Qualität record_jpeg_q_hw, sonst cv2 mit
    # record_jpeg_q_cv. RiderPi 2026-11-19: cv2 q92 8,6 ms CPU je Bild (bei 24 fps ~20 % eines Kerns), mittl.
    # Abweichung 0,63; Hardware q95 4,6 ms, 0,70, 51 statt 41 KB; über 95 wird er nicht genauer, nur größer.
    # Kodierer je Bild in meta.jsonl ("jpeg": "hw95" / "cv92"). Nutzerentscheidung 2026-11-19: Hardware q95.
    record_jpeg_q_hw: int = 95
    record_jpeg_q_cv: int = 92


@dataclass
class EmotionCfg:
    """Automatische Emotionen (xgonav/emotion.py); nur mit --robot proc."""
    enabled: bool = True
    dir: str = "/opt/xgonav/emotions"   # von tools/emotion.sh build (deploy/install.sh)
    fifo: str = "/tmp/lcd0"         # unabhängig von output.lcd_fifo (Overlay)
    gap_s: float = 5.0              # surprised erst, wenn so lange keine Person zu sehen war
    lost_s: float = 2.0             # so lange keine Person: doubt (follow), sonst embarrassed
    battery_low: int = 20           # % (Treiber schaltet bei 8 % ab); darunter shy
    battery_repeat_s: float = 300.0
    # Geräusche (aplay, Lautsprecher am WM8960): freudig bei Person erkannt, traurig bei Verlust
    # (nach lost_s). WAVs aus tools/sounds.py (deploy/install.sh). "" = Standardgerät von ALSA.
    sound: bool = True
    sound_dir: str = "/opt/xgonav/sounds"
    sound_found: str = "freude.wav"
    sound_lost: str = "trauer.wav"
    sound_device: str = ""
    # „Dum-di-dum-di-dum“ (pet_dudeln_*.wav), solange er kartiert oder sucht: in diesen Modi sowie in pet
    # beim Suchen und Streifen. Erst nach dudeln_start_s, dann etwa alle dudeln_s (±30 %), nie über ein
    # anderes Geräusch.
    dudeln: bool = True
    dudeln_modes: list = field(default_factory=lambda: ["explore", "patrol", "home"])
    dudeln_start_s: float = 1.5
    dudeln_s: float = 25.0


@dataclass
class OdomCfg:
    # Rad-Odometrie aus /proc/XGORider/wheels_log: die Radhälfte des IMU-Stroms (Firmware 0014), ohne Strom aus 0x0d.
    # source: "auto" = Räder, wenn vorhanden und frisch, sonst Fahrbefehl x vx_limit;
    # "command" = immer Fahrbefehl (wie früher).
    enabled: bool = True
    source: str = "auto"
    proc_dir: str = "/proc/XGORider"
    wheel_diameter_m: float = 0.060    # RiderPi, Angabe des Nutzers
    # Spurweite (Abstand der Radaufstandspunkte). Drehmessung 2026-11-18 (10 × 360° je Richtung): Radkurs rechts
    # 0,36 % zu wenig, links 0,0 %; die Streuung (Schlupf) ist größer als eine Korrektur, deshalb unverändert.
    track_m: float = 0.090
    ticks_per_rev: int = 1024          # am Rider gemessen (1 U von Hand: 1029 / 1043)
    # Faktor je Rad auf den Weg (tools/calibrate_odom.py): ungleiche Reifen/Schlupf. Für die
    # Strecke zählt der Mittelwert, für den Kurs der Unterschied (2 % = ~1,3° je 10 cm).
    # 2026-11-19 beide 1,0080 (Mittel der Maßbandfahrt 2026-11-04, 1,0073/1,0087): Räder gegen IMU-Kurs in 66
    # Rohfahrten (tools/odom_replay.py, Vorschlag der Koppelnavigation) ergaben keinen Unterschied der Räder
    # (je Fahrt ±0,2 %), Spur 0,0903.
    scale_left: float = 1.0080
    scale_right: float = 1.0080
    poll_hz: float = 25.0              # wheels_log lesen (Treiber puffert 64 Proben)
    # Abgesetzt (Rider balanciert nicht) werden die Räder nicht gelesen; die Pose bleibt dann stehen, auch wenn er
    # von Hand gerollt wird. Beim Aufstehen beginnt das Lesen neu, ohne den Weg dazwischen. true = immer lesen.
    read_idle: bool = False
    max_age_s: float = 0.3             # älter -> Rückfall auf den Fahrbefehl
    max_jump_ticks: int = 2048         # größerer Sprung zwischen zwei Proben = Neustart/Fehler
    # Koppelnavigation (odomfusion.py): ein Kurs und eine Strecke für Verhalten, Merkkarte und Raumkarte.
    # Kurs aus der IMU (RiderPi 2026-11-06, 32 m: IMU ~6 cm, Räder 75 cm Fehler durch Schlupf beim Drehen), sonst
    # aus den Rädern, sonst aus dem Bild. Strecke aus den Rädern, sonst Fahrbefehl x vx_limit.
    # Widerspruch: Weicht der Radkurs über contra_window_s um mehr als contra_deg + contra_frac x Drehung vom
    # IMU-Kurs ab, ist das Schlupf (Strecke dann aus dem Rad, das zur IMU-Drehung passt und weniger zurücklegt) –
    # außer das Bild gibt klar den Rädern recht: Dann gilt die IMU imu_doubt_s lang als zweifelhaft, der Kurs kommt
    # aus den Rädern und das Fenster wird nachgeholt. Das Bild zählt nur, wenn sein möglicher Fehler cam_err_deg +
    # cam_err_deg_m je gefahrenem Meter (2026-11-04: −8,9° auf 2 m geradeaus) unter einem Viertel des Widerspruchs
    # liegt. 67 Rohfahrten 2026-11-10..19 (tools/odom_replay.py): Räder minus IMU je 1 s p95 ~2°, p99 ~3,5°.
    contra_window_s: float = 1.0
    contra_deg: float = 4.0
    contra_frac: float = 0.10
    cam_err_deg: float = 1.0
    cam_err_deg_m: float = 5.0
    imu_doubt_s: float = 5.0
    # Vorschlag für scale_left/scale_right/track_m aus Rädern gegen IMU-Kurs (nur messen und anzeigen, /status
    # odom.vorschlag): Abschnitte von calib_seg_s ohne Schlupf, die letzten calib_n davon. Die Gesamtskala bleibt
    # beim Mittel der eingestellten Faktoren (sie kennt nur ein Maßband). 0 = aus.
    calib_seg_s: float = 2.0
    calib_n: int = 300


@dataclass
class GridCfg:
    # Belegungskarte (gridmap.py): Welt-KS ab Startpose, nur Anzeige
    enabled: bool = True
    resolution_m: float = 0.05
    # Kantenlänge, Start in der Mitte. 24 seit 2026-11-17: mit 16 endete die Karte 8 m vom Start (Fahrt
    # 20261117-130359 stieß bei y = +7,9 m an den Rand). 48 seit 2026-11-17 nachmittags: Fahrt 20261117-144500
    # (gerader Flur) kam bei 10,7 m an den Rand, fand voraus kein Ziel mehr und kehrte um. Kleinere
    # gespeicherte Karten werden mittig übernommen. 48 m = 960 x 960 Zellen, ~16 MB.
    size_m: float = 48.0
    # Treffer nur bis hierhin: Kamera nur 12,8 cm hoch, eine Bildzeile sind bei 2 m schon ~40 cm.
    # 1,5 seit 2026-11-14 (Nutzer: Hindernisse schon aus 1,5 m eintragen); vorher und im Simulator 1,25
    max_range_m: float = 1.5
    min_range_m: float = 0.15          # davor sieht die Kamera den Boden nicht
    # Treffer erst eintragen, wenn dieselbe Stelle (± confirm_cells Zellen) aus Entfernungen getroffen wurde, die
    # sich um mindestens so viel unterscheiden (m). Spiegelungen im glänzenden Boden und Fugen wandern mit dem
    # Rider und werden so nie belegt (Anzeige: orange). Aus Drehungen auf der Stelle allein entsteht nichts.
    # 0 = aus (jeder Treffer zählt). RiderPi 0.2 ab 2026-11-08 (Fensterunterkante gespiegelt, doppelte Wand),
    # seit 2026-11-14 wieder 0: Mit dem Bodennetz (FloorNet) genügt eine Sichtung (Nutzer); mit 0.2 erschienen
    # Hindernisse erst aus der Nähe und am Bildrand oft gar nicht.
    # Simulator explore 120 s: Wände 82 % (aus) / 57 % (±1 Zelle) / 73 % (±2 Zellen), Präzision 0,90 / 0,95 / 0,93.
    confirm_dr_m: float = 0.0
    confirm_cells: int = 2
    # Punkte der Merkkarte, vor denen die Bahnprüfung bremst, gelten in der Raumkarte als bestätigt und belegt
    # (auch ohne confirm_dr_m): Was ihn bremst, steht in der Karte, und der Planer fährt darum herum.
    mark_blocking: bool = True
    column_step: int = 2              # jede k-te Spalte des Bodenmodells (160 Spalten)
    l_free: float = -0.4               # Log-Odds je Beobachtung
    l_occ: float = 0.5                 # zwei Treffer bis "belegt" (l_show): ein Ausreißer sperrt nichts
    free_margin_m: float = 0.10        # Freistrahl endet so weit vor der Grenze (Messrauschen)
    free_margin_quad: float = 0.05     # dazu so viel x Entfernung² (der Fehler wächst quadratisch)
    # Treffer als Band entlang des Strahls (± so viel x Entfernung², m) statt einer Zelle. Die Entfernung aus dem
    # Bild streut quadratisch: Fahrt 20261118-135118 sah die Wand hinter den Stühlen ab 1–1,5 m mit ±0,3 m, kaum
    # eine Zelle bekam zwei Treffer, Freistrahlen anderer Bilder bauten sie wieder ab; dicht wurde die Wand erst
    # aus der Nähe. 0,05: 1 m ±5 cm, 1,5 m ±11 cm, Wand dort schon aus der Ferne geschlossen; 0,08 ließ Stuhl- und
    # Tischbeine unter dem Tisch zusammenwachsen. 0 = eine Zelle (Simulator). Seit 2026-11-18, nicht gefahren.
    hit_band_quad: float = 0.05
    # Freistrahlen bis hierhin (Treffer nur bis max_range_m); 0 = wie max_range_m. RiderPi 1,1 ab 2026-11-06 (eine
    # Stufe wurde aus > 1 m als Boden gesehen und bei 1,5 m als frei eingetragen), seit 2026-11-14 wieder 1,5
    # (Nutzer: offenen Boden so weit sehen wie Hindernisse; mit dem Bodennetz, nicht gefahren)
    free_range_m: float = 1.5
    driven_free_r_m: float = 0.10      # gefahrener Weg gilt so breit als frei; 0 = aus
    max_turn_dps: float = 40.0         # bei schnellerer Drehung nichts eintragen (Unschärfe, Pose)
    occ_free_factor: float = 0.5       # schon belegte Zellen baut ein Freistrahl nur so stark ab
    l_min: float = -3.0
    l_max: float = 3.5
    l_show: float = 0.7                # ab hier "belegt"/"frei" in Anzeige und Statistik
    view_m: float = 6.0                # Bildausschnitt für die Anzeige
    # Eintragen in eigenem Thread (GridWorker, Kerne wie [cpu] freespace_cpus); False = im Regelschritt
    threaded: bool = True
    # Strahlen im C++-Kern eintragen (xgonav/native/gridfast.cpp), wenn gebaut; gleiches Ergebnis wie numpy
    native: bool = True
    # Speichern/Laden (npz). Leer = aus. Beim Laden gilt Pose = Startpunkt: der Rider muss
    # vor dem Start von xgonav wieder dort stehen (ohne Marker findet er sich nicht wieder).
    file: str = ""
    save_every_s: float = 60.0         # höchstens so oft, und nur bei Änderungen; dazu beim Beenden


@dataclass
class StartbildCfg:
    # Abgleich am Startbild (startbild.py): Bild am Startpunkt (Anker) gegen die Bilder der Heimkehr. home fährt dazu
    # erst einen Vorpunkt vor_m vor dem Start an, dreht auf den Startkurs und fährt gerade auf den Start (die
    # Tiefenbilder); dann misst es die Lage gegen den Anker und korrigiert die Kartenpose. Liegt der Rest über
    # nach_m, fährt es einmal nach. Offline 2026-11-19 (tools/start_abgleich.py): Fahrt 144940 Restfehler von
    # ~12 cm auf ~3 cm, kurze Paare Median 1,1 cm / 0,3°. Seit 2026-11-19, am Rider nicht gefahren.
    enabled: bool = True
    vor_m: float = 0.40                # Vorpunkt so weit vor dem Start (gerade Schlussfahrt); Tiefenbilder bis basen[-1]
    vor_tol_m: float = 0.08            # am Vorpunkt angekommen
    gerade_v: float = 0.08             # normiertes Tempo der Schlussfahrt (~10 cm/s)
    tiefe_schritt_m: float = 0.04      # während der Schlussfahrt je so viel Weg ein Graubild merken
    basen: list = field(default_factory=lambda: [0.1, 0.15, 0.2, 0.25, 0.3])   # Abstände der Tiefenbilder (m)
    features: int = 3000               # SIFT-Merkmale je Bild
    min_inl: int = 30                  # weniger stimmige Treffer: keine Korrektur (Ausreißer offline alle < 27)
    max_m: float = 0.5                 # größere Korrektur der Pose gilt als Fehlmessung
    max_deg: float = 15.0
    max_lage_deg: float = 1.0          # Nick-/Roll-Rest des Ankers (offline +0,3° / −0,2°)
    agree_m: float = 0.05              # Gegenprobe (frei, 6 Freiheitsgrade) muss so nah liegen
    nach_m: float = 0.04               # Rest nach der Korrektur, ab dem nachgefahren wird (einmal)
    # Schlussausrichtung auf den Startkurs nach dem Abgleich: enger als [home] align_tol_deg (10°), die Messung trägt
    # ~1°. Fahrt 2026-11-19 161006: Ende ~1 cm vom Start, aber −9,5° gedreht stehen geblieben. Dreht er über den
    # Startkurs hinaus (Vorzeichenwechsel), ist Schluss statt Pendeln; nach end_s ebenfalls.
    end_deg: float = 2.0
    end_s: float = 8.0
    timeout_s: float = 20.0            # so lange auf die Messung warten, dann ohne Korrektur fertig
    # Anker: Rider steht am Startpunkt (so nah und so wenig gedreht), Merkmale einmal je Startpunkt. Der Anker merkt
    # sich seine tatsächliche Pose, die Grenze sagt nur „am Start“. 1 cm / 1° war zu streng: Nach dem Aufstehen
    # stand der Rider 2026-11-19 bei −0,6 cm / −1,7° und bekam keinen Anker.
    anker_m: float = 0.03
    anker_deg: float = 5.0


@dataclass
class HomeCfg:
    # Modus home: Pfad über die Raumkarte zum Startpunkt (planner.py), Pure Pursuit
    # Belegte Zellen um so viel aufblasen; über dem Radius der Bahnprüfung (localmap, 0,18 m),
    # sonst plant der Planer genau an ihre Bremsgrenze
    inflate_m: float = 0.23
    clear_cost: float = 4.0            # Zusatzkosten nahe an Hindernissen (fällt bis 2*inflate_m ab)
    # Engstellen: Zellen näher als inflate_m sind nicht gesperrt, sondern kosten squeeze_cost mehr; gesperrt ist
    # erst näher als squeeze_m (Abstand der Zellmitten). 0,09 sperrt bei 5-cm-Zellen nur die direkten Nachbarn
    # belegter Zellen: eine 25 cm breite Lücke (4–5 freie Zellen je nach Lage im Raster) bleibt offen. Der Weg
    # geht nur hindurch, wenn es sonst keinen gibt (0,5 m Engstelle kosten wie 10 m Umweg). 0 = aus (wie bis
    # 2026-11-11: Lücken erst ab ~0,45 m). Sperrzonen und das Freiräumen um die Pose rechnen weiter mit inflate_m.
    squeeze_m: float = 0.09
    squeeze_cost: float = 20.0
    unknown_cost: float = 3.0          # unbekannte Zellen kosten so viel mehr als freie
    margin_m: float = 1.5              # Suchgebiet = Pose, Start, gefahrener Weg + Rand
    lookahead_m: float = 0.5
    # Engstelle: belegte Zelle der Raumkarte näher als tight_m an der Pose oder am Weg bis 0,45 m voraus (0 = aus;
    # unter inflate_m, damit normale Wege sie nicht auslösen). Dann kurzer Vorausblick, langsamer, schon bei
    # kleinem Fehler auf der Stelle drehen und Bahnprüfung mit [map] radius_tight_m: Mit 0,5 m Vorausblick
    # schnitt er die Kurve vor einer 25-cm-Lücke und stieß schräg an die Wandecke (Simulator 2026-11-11).
    tight_m: float = 0.21
    tight_lookahead_m: float = 0.2
    tight_v: float = 0.6               # Faktor auf v
    tight_turn_enter_deg: float = 30.0
    tight_turn_exit_deg: float = 10.0
    # In der Engstelle auf die Mitte zwischen den Kanten der Merkkarte links und rechts voraus zielen, wenn sie
    # höchstens so weit von der Wegrichtung abweicht. Die Raumkarte legt eine 25-cm-Lücke um Zentimeter daneben.
    tight_center: bool = True
    tight_center_max_deg: float = 30.0
    arrive_m: float = 0.15
    # Festgefahren, aber so nah am Ziel: gilt als angekommen. Der Sicherheitsabstand der Bahnprüfung (0,18 m) lässt
    # ihn nicht an ein Ziel, neben dem etwas liegt, auch wenn das Ziel selbst frei ist (RiderPi 2026-11-06)
    near_m: float = 0.5
    replan_s: float = 2.0
    keep_factor: float = 1.2           # alter Weg bleibt, solange frei und <= Faktor x neuer + 0,3 m
    use_localmap: bool = True          # Hindernisse der Bahnprüfung (Merkkarte) mit einplanen
    localmap_reach_m: float = 1.0      # nur so nahe: weiter weg misst die Merkkarte zu ungenau
    v: float = 0.15                    # normiert, wie behavior.cruise_v
    # Lenkung: am RiderPi rollt der Rider bei jedem Anfahren erst zurück und eine Drehung läuft
    # ~15° nach; ständiges Wechseln zwischen Drehen und Fahren brachte ihn nicht vom Fleck
    # (2026-11-05). Deshalb in Kurven weiterfahren, auf der Stelle nur bei großem Fehler drehen.
    kp_turn: float = 1.5               # 1/s (Folgen: behavior.kp_turn)
    turn_enter_deg: float = 75.0       # ab diesem Winkelfehler auf der Stelle drehen ...
    turn_exit_deg: float = 25.0        # ... bis er darunter ist
    # Kleine Lenkbefehle ignoriert die Firmware (u8 128 ± 6): darunter sonst keine Korrektur
    # unter ~11° Fehler. Ab min_w_err_deg Fehler deshalb mindestens min_w.
    min_w: float = 0.055
    min_w_err_deg: float = 4.0
    # Drehen auf der Stelle (Wegfolger, Ausrichten am Ziel) höchstens so schnell (normiert). 0,25 seit
    # 2026-11-17: mit robot.max_w 0,5 wendete er mit 0,5, kippte dabei 7° zur Seite und die IMU-Neigung
    # brauchte danach 3-4 s (Fahrt 20261117-144500). 1.0 = nur robot.max_w begrenzt (Simulator).
    turn_w_max: float = 0.25
    align_heading: bool = True         # am Ziel auf den Startkurs drehen
    align_tol_deg: float = 10.0
    max_fail: int = 3                  # so oft kein Weg -> aufgeben
    # Festgefahren aufgeben erst nach so viel Zeit ohne 0,3 m Annäherung: Die Merkkarte vergisst nach 4 s, und
    # am RiderPi wäre die Bahn kurz nach dem Abbruch wieder frei gewesen (2026-11-06)
    give_up_s: float = 20.0
    # Wegsuche (A*, Kürzen), Zielpunkt des Wegfolgers und die Schleifen der Frontier-Suche (explore) im C++-Kern
    # (xgonav/native/navfast.cpp), wenn gebaut; gleiches Ergebnis wie Python, die GIL bleibt dabei frei
    native: bool = True


@dataclass
class ExploreCfg:
    # Modus explore: Frontiers der Raumkarte anfahren (frontier.py), bis nichts Unbekanntes mehr
    # erreichbar ist. Weg und Lenkung wie home ([home]).
    scan_w: float = 0.07               # Rundumblick am Anfang (langsam: unter grid.max_turn_dps)
    # Tempo auf dem Weg zur nächsten Stelle und zurück zum Start (normiert, statt home.v 0,15; Nutzer 2026-11-11:
    # erst 0,30, nach zwei Fahrten am RiderPi 0,20). Gilt über robot.max_v hinaus, nur im Modus explore; in
    # Engstellen weiter mal home.tight_v, das reaktive Befreien bleibt bei behavior.cruise_v.
    v: float = 0.20
    min_cluster_m: float = 0.4         # kleinere Frontier-Gruppen zählen nicht
    # Erst wenn sonst kein Ziel mehr bleibt, zählen auch Gruppen dicht an Hindernissen (näher als home.inflate_m,
    # z. B. die Öffnung einer 25-cm-Lücke: in der Karte nur 3–5 Zellen breit) ab dieser Größe, wenn dahinter
    # mindestens min_unknown_m2 zusammenhängend unbekannt ist (kein einzelnes Loch im bekannten Boden)
    squeeze_cluster_m: float = 0.15
    min_unknown_m2: float = 0.05
    # Zielwahl: die Stelle mit der kleinsten Fahrzeit + Drehzeit zuerst. Viel Unbekanntes am Ziel (Fläche im
    # Umkreis gain_radius_m, voll ab gain_ref_m2) zählt wie höchstens gain_bonus_s weniger Zeit.
    gain_radius_m: float = 1.2
    gain_ref_m2: float = 2.0
    gain_bonus_s: float = 3.0
    travel_mps: float = 0.35           # Fahrzeit-Schätzung (v 0,20 x vx_limit 1,8 = 0,36)
    # Drehzeit-Schätzung: Grad zur Stelle / turn_dps zählt wie Fahrzeit. Kleiner = Geradeaus wird bevorzugt.
    # 15 seit 2026-11-14 (Nutzer: nicht immer drehen, weil etwas anderes näher ist): 90° wiegen wie 2,1 m
    # Fahrt, 180° wie 4,2 m; vorher und im Simulator 40 (90° wie 0,8 m)
    turn_dps: float = 15.0
    # Näher als roll_m am Ziel: alle roll_every_s neu planen statt alle home.replan_s. Die Grenze weicht beim
    # Anfahren zurück, das Ziel wandert mit, und er hält nicht alle 0,7-1 m an (RiderPi 2026-11-11: 26 Halte
    # in 348 s).
    roll_m: float = 1.0
    roll_every_s: float = 0.4
    # Im selben Takt plant er dort abwechselnd schon das übernächste Ziel (ab dem Ziel, ohne dessen Stelle). Liegt
    # die Stelle beim Erreichen im Bild (± pass_view_deg) und ist dieser Plan höchstens pass_age_s alt, fährt er
    # ohne Halt weiter; sonst wie bisher hindrehen, settle_s schauen, neu suchen.
    pass_view_deg: float = 40.0
    pass_age_s: float = 3.0
    # Höchstens so viele Ziele, die nur in Engstellen liegen (Stufe 1), je Erkundung: In einer echten Karte
    # sind das meist Löcher in gezeichneten Wänden (RiderPi 2026-11-11: 17-25 solche Stellen am Ende).
    # 10 seit 2026-11-15 (Nutzer; RiderPi: bei 3 blieben 10 Stellen hinter Engstellen liegen und die Karte galt
    # nach 145 s als fertig); vorher und im Simulator 3
    tight_max: int = 10
    # Ziele nur in Engstellen kosten so viel Zeit (s) mehr als Ziele mit Abstand und konkurrieren sonst mit ihnen.
    # -1 = alte Regel (Engstellen nur, wenn kein Ziel mit Abstand bleibt; Simulator). 3 seit 2026-11-15 (Nutzer):
    # Er kehrte sonst für ein Ziel hinter sich um, obwohl eine Engstelle direkt voraus lag.
    tight_penalty_s: float = 3.0
    standoff_m: float = 0.3            # so weit vor der Frontier anhalten
    start_free_r_m: float = 0.2        # um den Startpunkt gilt der Boden als frei
    reach_m: float = 0.25              # Ziel gilt als erreicht
    settle_s: float = 0.6              # nach Ankunft stehen und schauen (Nicken beruhigen)
    keep_dist_m: float = 0.5           # altes Ziel besteht, solange Frontier-Zellen so nah sind
    goal_timeout_s: float = 60.0       # länger an einem Ziel: gilt als festgefahren
    no_growth_s: float = 120.0         # so lange kaum Zuwachs an bekanntem Boden: Karte gilt als fertig
    min_growth_m2: float = 0.5
    # Umkreis besuchter und gesperrter Stellen. Mit 0,5 m fiel die letzte Stelle eines Gangs weg, die nur
    # 0,49 m weiter lag als die vorige (RiderPi 2026-11-06, „Karte fertig“ nach 39 s)
    avoid_r_m: float = 0.3
    wander_s: float = 15.0             # Befreiung: so lange reaktiv fahren ...
    max_wander: int = 3                # ... höchstens so oft, dann Stopp mit Meldung
    no_goal_limit: int = 3             # so viele Suchen ohne Ziel: Karte gilt als fertig
    max_s: float = 900.0               # Zeitgrenze
    min_battery: int = 25              # %: darunter beenden und zum Start
    return_home: bool = True           # am Ende zum Startpunkt fahren
    # Am Start angekommen noch einmal nach Zielen suchen, bevor die Karte als fertig gilt: Auf dem Heimweg
    # sieht er Neues (RiderPi 2026-11-15: eine offene Stelle neben dem Start blieb so liegen). Nur wenn der
    # Heimweg mindestens recheck_min_m lang war, höchstens recheck_max mal je Erkundung; 0 = aus.
    recheck_max: int = 3
    recheck_min_m: float = 1.0


@dataclass
class PatrolCfg:
    # Streife: Route abfahren (xgonav/route.py, Modus patrol)
    reach_m: float = 0.25              # Wegpunkt erreicht
    pause_s: float = 1.5               # am Wegpunkt stehen und schauen
    point_timeout_s: float = 90.0      # länger: Wegpunkt auslassen
    start_battery: int = 40            # %: darunter nicht losfahren
    return_battery: int = 30           # %: zurück zum Start
    down_battery: int = 15             # %: sofort absetzen


@dataclass
class PetCfg:
    # Modus pet (xgonav/pet.py): reagiert auf Menschen und Tiere, bleibt im Standard auf der Stelle.
    # Abstandszonen der Person (m, Fußpunkt)
    zone_far_m: float = 3.0            # darüber: nur bemerken
    zone_mid_m: float = 1.5            # darüber: neugierig; darunter: Freude
    zone_close_m: float = 0.5          # darunter: schüchtern (wandert mit dem Vertrauen)
    kneel_max_m: float = 1.9           # Box oben frei und näher: hingekniet (stehend wäre der Kopf nicht im Bild)
    zone_hold_s: float = 0.5           # so lange stabil, bevor eine Zone zählt
    zone_repeat_s: float = 15.0        # dieselbe Zone frühestens wieder nach so langer Zeit
    # Schreck: Annäherung aus dem Fußpunkt, Geradenanpassung über startle_window_s
    startle_mps: float = 1.0
    startle_heavy_mps: float = 1.8     # ab hier volle Stärke
    # Eine Bildzeile sind in 2 m schon 10 cm: weiter weg misst der Fußpunkt zu ungenau (Simulator, 3 px Rauschen:
    # Fehlalarme schon bei 0,3 m/s). Darum eng begrenzt und zusätzlich durch das Wachsen der Box bestätigt.
    startle_max_m: float = 1.6         # weiter weg erschrickt er nicht
    startle_window_s: float = 0.8      # bei 4 Hz Detektion ~3 Messungen
    startle_min_samples: int = 3
    # Annäherung = min(Fußpunkt-Gerade, Boxwachstum x Abstand / startle_vs); < 1 traut der Box mehr zu
    startle_vs: float = 1.0
    startle_s: float = 1.4             # Dauer der Reaktion (Abducken, Ruck)
    startle_cooldown_s: float = 3.0
    startle_min: float = 0.12          # schwächer: nur Augen, kein Abducken
    # Körper. gestures = false: nur Augen und Töne
    gestures: bool = True
    height_mm: float = 85.0            # falls die Höhe unbekannt ist
    duck_mm: float = 75.0              # wird auf Minimum + 10 mm angehoben (robot.height_min_mm)
    roll_deg: float = 8.0              # Kopf schief; höchstens 12
    tilt_s: float = 1.2
    wiggle_w: float = 0.10
    wiggle_s: float = 1.2
    turn_kp: float = 1.5               # 1/s
    turn_deadband_deg: float = 8.0
    turn_w: float = 0.40               # Nutzer 2026-11-13: Drehen x2 (vorher 0,20)
    turn_min_w: float = 0.06           # kleinere Lenkbefehle ignoriert die Firmware
    # Budgets, 0 = aus. Rückwärts ist blind und vorwärts sieht das Bodenmodell Stufen zu spät: nur mit
    # Beobachtung und nur, wo keine Treppe ist. Ohne Räder fährt er nie, Obergrenzen 0,30 m / 1,0 m.
    retreat_m: float = 0.0             # je Ereignis (RiderPi-Test: 0.12)
    retreat_v: float = 0.10
    retreat_max_s: float = 1.5
    retreat_cooldown_s: float = 10.0
    # Beide Budgets als Zeitimpuls (Budget / Tempo x pulse_margin), die Räder nur als Obergrenze: Beim Stoppbefehl
    # läuft er noch ~0,9 s x Tempo nach (Simulator --dyn nach Messfahrt 2026-11-05), bei 0,18 m/s mehr als das
    # ganze Budget. Über den ganzen Impuls entspricht der Weg dem Integral des Befehls.
    pulse_margin: float = 0.85
    creep_m: float = 0.0               # zur Katze, je Begegnung (RiderPi-Test: 0.4); braucht Bodenmodell + Merkkarte
    creep_v: float = 0.06
    # Tiere
    animal_conf: float = 0.5
    animal_hits: int = 4
    dog_fear_m: float = 2.0
    dog_close_m: float = 0.5           # näher: Ohnmacht
    cat_stop_m: float = 0.8
    animal_repeat_s: float = 20.0
    # Vertrauen 0..1 und Schreckpegel
    trust_start: float = 0.3
    trust_gain: float = 0.01           # 1/s, während jemand sanft in der Nähe ist
    trust_loss: float = 0.05           # je Schreck
    fright_decay: float = 0.05         # 1/s
    habit_tau_s: float = 60.0          # Gewöhnung klingt so schnell ab
    # Ohnmacht: absetzen, nap_s liegen, von selbst aufstehen. Beim Aufspringen rollt er ~20 cm vor:
    # vorne >= 40 cm frei, keine Stufe. Am RiderPi an seit 2026-11-13 (Nutzer: soll beim Erschrecken umfallen).
    faint: bool = True
    faint_level: float = 0.8           # RiderPi seit 2026-11-13 (vorher 1,0: Schreck 0,90 warf ihn nicht um)
    faint_max: int = 3                 # höchstens so viele in faint_window_s, danach nur Schreck
    faint_window_s: float = 600.0
    nap_s: float = 10.0                # so lange ganz weg, dann gehen langsam die Augen auf
    eyes_open_s: float = 3.0           # Augen öffnen, dabei schaut er sich um
    clear_s: float = 2.5               # so lange niemand im Bild: die Luft ist rein, er springt auf
    nap_max_s: float = 60.0            # bis dahin nicht rein: bleibt liegen
    nap_rest_s: float = 2.0            # so lange nach dem Absetzen, bevor die Liegelage als Referenz gilt
    nap_tilt_deg: float = 5.0          # mehr: hochgehoben, steht nicht von selbst auf
    nap_moved_m: float = 0.01
    standup_warn_s: float = 1.5        # Aufwachton vor dem Aufspringen
    standup_timeout_s: float = 8.0
    settle_s: float = 6.0              # danach keine Gesten (Aufschwung, Nivellier-Impuls, Standreferenz)
    battery_low: int = 20              # %: darunter keine Ohnmacht und kein Aufstehen
    # Sonst
    peek_s: float = 3.0                # Person so kurz weg und wieder da: Guck-guck
    gone_s: float = 3.0                # länger weg: schaut hinterher, dann traurig
    nudge_m: float = 0.05              # so viel Radweg in 0,5 s ohne Fahrbefehl: angestupst
    # Umschauen und Suchen (xgonav/pet.py, Aufgaben). Person weg: zur letzten bekannten Stelle, dann umschauen.
    # Nichts zu tun: umschauen und in Etappen zur freiesten Richtung streifen. Umschauen dreht nur auf der
    # Stelle (wie die Gesten, gestures = false schaltet es ab); gefahren wird nur mit Rädern, Bodenmodell und
    # Merkkarte. Weg und Radius: 0 = aus, -1 = ohne Grenze (RiderPi seit 2026-11-13, Nutzer: keine Treppe im
    # Raum, Bodenmodell zuverlässig). Stufen sieht das Bodenmodell zu spät: nie an einer Treppe.
    look: bool = True                  # nach dem Verschwinden und bei Langeweile umschauen
    # Hören (xgonav/ortung.py, [ortung] modes muss pet enthalten). Die Richtung taugt nur balancierend und nur
    # für Geräusche ab ~1 s (Sprache); vorn und hinten unterscheidet er durch Drehen und erneutes Lauschen.
    hear: bool = True
    hear_min_db: float = 8.0           # leisere Meldungen zählen nicht
    hear_front_deg: float = 20.0       # innerhalb davon gilt die Quelle als vorn (oder genau hinten)
    hear_turns: int = 3                # höchstens so viele Drehungen je Geräusch
    hear_listen_s: float = 3.5         # nach dem Drehen so lange auf das nächste Geräusch warten
    hear_repeat_s: float = 6.0         # nach einem Horchen frühestens wieder
    # Ab diesem Winkel kommt das Geräusch ganz klar von links oder rechts: dreht auch in der Sperrzeit
    # hear_repeat_s hin (Nutzer 2026-11-14). 90 oder mehr = Sperre gilt immer.
    hear_side_deg: float = 45.0
    hear_startle_db: float = 20.0      # nach hear_quiet_s Stille gleich so laut und niemand zu sehen: zuckt zusammen
    hear_quiet_s: float = 8.0
    # Rufen: Stimme hält von vorn an und niemand ist zu sehen -> sucht dort (call_events Meldungen). Fährt
    # höchstens call_m je Ruf und nie mehr als seek_m, mit Bodenmodell und Bahnprüfung wie das Suchen; 0 = nur
    # umschauen, -1 = ohne Grenze (dann je Ruf eine Etappe von 3 m; RiderPi seit 2026-11-13, Nutzer: keine Treppe).
    call_events: int = 3
    # Nach dem Horchen niemand zu sehen: suchen statt aufgeben (Nutzer 2026-11-19: „er muss ja losfahren und
    # suchen“). Richtung bekannt: dorthin wie beim Rufen (dann reicht schon eine Meldung von vorn); unbekannt
    # (beim Streifen gestutzt, danach still): einmal rundum umschauen, je Blick hear_listen_s lauschen; eine
    # Meldung dabei führt dorthin. Sperre wie das Rufen (call_repeat_s).
    hear_seek: bool = True
    # Suchen vorbei, niemand gefunden: zurück zum Start (wo er aufstand), wie explore am Ende (Nutzer 2026-11-19).
    # Der Weg kommt wie alle pet-Fahrten mit Raumkarte vom Planer; findet er vorn keinen Weg zum Rufziel (oder nur
    # einen Umweg über das 1,5-Fache), versucht er die Gegenrichtung (vorn und hinten klingen gleich).
    seek_home: bool = True
    call_m: float = -1.0
    call_repeat_s: float = 45.0        # frühestens dann wieder (ein Fernseher vorn ruft sonst endlos)
    # Komm her: Er sieht eine Person, die weiter als come_stop_m entfernt ist und spricht (come_events Meldungen
    # in Folge, 1 = ein kurzer Zuruf reicht) -> fährt auf sie zu bis come_stop_m, höchstens come_m je Mal (0 = aus, -1 = ohne Grenze; RiderPi
    # seit 2026-11-13). Bedingungen wie jede pet-Fahrt (Räder, Bodenmodell, Merkkarte, Bahnprüfung, Akku).
    come_events: int = 1
    come_m: float = -1.0
    come_stop_m: float = 0.8
    come_repeat_s: float = 8.0
    # Mitreden: nach talk_min_events Meldungen und talk_pause_s Pause antwortet er (Ton brabbeln, Kopf schief)
    talk: bool = True
    talk_min_events: int = 2
    talk_pause_s: float = 1.6
    talk_repeat_s: float = 7.0
    # Nachplappern (xgonav/stimme.py, [stimme]): Proben, die er gelernt hat, spielt er beim Mitreden mit parrot_p statt
    # des Brabbelns, frühestens parrot_repeat_s nach der letzten; eine neu bestätigte oder wiedererkannte gleich nach parrot_delay_s.
    # Mitgeschnitten wird nur, solange eine Person zu sehen ist oder höchstens parrot_listen_s weg ist.
    parrot: bool = True
    parrot_p: float = 0.5
    parrot_repeat_s: float = 30.0
    parrot_delay_s: float = 0.2        # Nutzer 2026-11-19: 0,8 dauerte zu lang (mit Nachlauf 0,6 s ~1,5 s)
    parrot_listen_s: float = 3.0
    seek_m: float = -1.0               # je Verschwinden höchstens so weit zur letzten Stelle
    seek_stop_m: float = 0.4           # so weit vor der letzten Stelle halten
    roam_m: float = -1.0               # je Phase ohne Begegnung höchstens so weit streifen (endet spätestens mit doze_s)
    roam_radius_m: float = -1.0        # beim Streifen nie weiter vom Platz, an dem er in pet aufstand
    roam_leg_m: float = 0.4            # Länge einer Etappe
    roam_after_s: float = 30.0         # so lange niemand: umschauen bzw. streifen
    roam_repeat_s: float = 45.0        # danach wieder
    roam_free_min: float = 0.55        # Freiraum im Zentrum (wie behavior.slow_free), sonst fährt er dort nicht hin
    roam_battery: int = 40             # %: darunter weder suchen noch streifen
    drive_v: float = 0.12              # normiert; gemessen 0,08 -> ~12,5 cm/s, 0,12 -> ~26 cm/s. Nutzer 2026-11-13: schneller (vorher 0,08)
    look_w: float = 0.24               # Drehrate beim Umschauen und Horchen, normiert. Nutzer 2026-11-13: x2 (vorher 0,12)
    look_step_deg: float = 90.0        # in Schritten drehen, dazwischen still stehen: Detektor und Bodenmodell
    look_steps: int = 4                # so viele Blicke (4 x 90° = einmal rundum)
    look_pause_s: float = 1.5          # je Blick stillstehen
    bored_s: float = 60.0
    bored_repeat_s: float = 45.0
    doze_s: float = 180.0              # dösen, Detektor auf doze_hz
    timeout_s: float = 600.0           # so lange niemand: absetzen, Modus idle (Akku)
    active_hz: float = 4.0             # Detektor in pet; spart gegen ~7 Hz fast die Hälfte
    doze_hz: float = 1.0
    gaze_dir: str = "/opt/xgonav/eyes_gaze"   # von tools/eyes.py --gaze (deploy/install.sh)


@dataclass
class RuheCfg:
    # Ruhemodus (Web-Knopf „Ruhe“): abgesetzt, Bildverarbeitung aus, Treiber fragt nichts mehr ab, xgonav
    # wacht nur noch selten auf. Den CPU-Takt schaltet deploy/xgo-ruhe (root), die Schlafaugen spielt api.php.
    tick_s: float = 1.0                # Hauptschleife und robot-Thread: so oft aufwachen
    sleep_ms_on_loop: int = 1000       # Treiber-Hauptschleife (Zustand, Akku); normal 200


@dataclass
class DunkelCfg:
    # Dunkelangst (xgonav/dunkel.py): Bild zu dunkel -> Angst, umdrehen, Licht suchen; sonst absetzen + Ruhemodus.
    # Nur in diesen Modi und nur balancierend. Helligkeit = Mittelwert 0..255 (jedes 8. Pixel), check_hz.
    enabled: bool = True
    modes: list = field(default_factory=lambda: ["explore", "patrol", "home", "follow", "approach", "pet"])
    # Gemessen an Rohfahrten am RiderPi (2026-11-13): Abendlicht im Raum 11..28, unter Möbeln/Licht aus 2..5
    dark_level: float = 7.0            # darunter dunkel
    light_level: float = 11.0          # beim Prüfen darüber hell (Abstand gegen Flattern)
    dark_s: float = 2.0                # so lange dunkel, bevor er Angst bekommt
    check_hz: float = 4.0
    fright_s: float = 1.5              # Angst: so lange stehen (Emotion, Ton)
    settle_s: float = 1.5              # nach dem Drehen warten, bis die Belichtung nachgeregelt hat
    turn_kp: float = 1.5               # 1/s, wie pet
    turn_w: float = 0.20
    turn_min_w: float = 0.06
    tol_deg: float = 8.0
    turn_timeout_s: float = 10.0
    max_events: int = 3                # so oft in events_window_s ins Dunkle: gleich aufgeben
    events_window_s: float = 300.0
    emotion: str = "angst"
    sound: str = "angst"
    sound_relief: str = "kichern"
    # Dunkelzone: Bei jeder Angst wird ein Rechteck voraus gesperrt (Zielsuche, Planer, Bahnprüfung, wie eine
    # Sperrzone), sonst schickt der Planer ihn nach dem Umdrehen zurück ins Dunkle. Nur im Speicher, verfällt
    # nach zone_s; nur in zone_modes angelegt, beachtet in allen Modi. zone_s 0 = keine Zone.
    zone_modes: list = field(default_factory=lambda: ["explore"])
    zone_s: float = 600.0
    zone_w_m: float = 2.0              # Breite quer zur Blickrichtung
    zone_d_m: float = 3.0              # Tiefe voraus
    zone_ahead_m: float = 0.10         # Beginn vor dem Rider: er steht nicht in der Zone und kann wegplanen


@dataclass
class OrtungCfg:
    # Schallrichtung aus den zwei Mikrofonen (xgonav/ortung.py): Laufzeitunterschied aus der Phase des
    # Kreuzspektrums. Läuft nur in `modes` und nie im Ruhemodus; arecord als eigener Prozess, gerechnet wird
    # nur bei Geräuschen über dem Ruhepegel. Vorn/hinten ist mit zwei Mikrofonen nicht zu trennen.
    enabled: bool = True
    modes: list = field(default_factory=lambda: ["idle", "pet"])
    device: str = "hw:0,0"             # WM8960 (docs/audio.md)
    rate: int = 48000
    block: int = 1024                  # Abtastwerte je Block (Zweierpotenz), 21 ms
    # Wirksamer Abstand der Mikrofone quer zur Fahrtrichtung und fester Versatz zwischen den Kanälen. Gemessen
    # 2026-11-13 balancierend, Testton 1 m entfernt, 8 Stellungen im Kreis: tau = 11,2 us x sin(Winkel) - 2,2 us
    # (Restfehler 0,6 us). 11,2 us x 343 m/s = 3,8 mm; ein Abtastwert sind 21 us, deshalb lange mitteln.
    mic_dist_m: float = 0.0038
    tau_offset_us: float = -2.2        # wird von der gemessenen Laufzeit abgezogen
    sign: int = 1                      # -1, wenn links und rechts vertauscht sind
    # Band in Hz: darunter Brummen/Körperschall. Über 2 kHz sind die Kanäle nicht mehr phasentreu (bei 3,5-4,5 kHz
    # steht die Phase fest bei ~170°, unabhängig von der Richtung): das ergab Scheinrichtungen.
    f_lo: float = 300.0
    f_hi: float = 2000.0
    tau_margin: float = 1.5            # gesucht wird bis mic_dist/c x tau_margin
    steer_steps: int = 20              # Laufzeit-Schritte je Seite
    phat_beta: float = 0.8             # 1 = nur Phase (PHAT), 0 = mit Betrag gewichtet
    on_db: float = 9.0                 # Tor öffnet: so weit über dem Ruhepegel
    off_db: float = 3.0                # und bleibt beim Sammeln offen, solange noch so weit darüber
    min_level: float = 0.0             # und mindestens dieser Pegel (Einheit wie Kern.level, 0 = aus)
    # Ruhepegel nachführen je Block: aufwärts langsam, abwärts schnell. 0,001 = 21 s Zeitkonstante: ein
    # gleichbleibendes Geräusch wird etwa 15 s lang verfolgt, dann gilt es als Ruhe (mit 0,01 nur 0,3 s).
    noise_up: float = 0.001
    noise_down: float = 0.2
    warm_blocks: int = 25              # nach dem Start nur Ruhepegel lernen
    # Streuung der Laufzeit je Auswertung (gemessen): 0,13 s +-6 us, 0,5 s +-3 us, 1 s +-1..2 us, 2 s +-0,7..1,3 us
    window_blocks: int = 47            # so viele Blöcke je Auswertung (1 s)
    min_blocks: int = 24               # kürzere Geräusche verwerfen (0,5 s; ein Klatschen reicht nicht)
    hang_blocks: int = 12              # Pausen bis zu dieser Länge unterbrechen das Sammeln nicht (Sprache)
    min_interval_s: float = 0.0        # nach einer Auswertung so lange nichts sammeln (Dauerlärm)
    min_quality: float = 0.4           # Güte 0..1, darunter verwerfen
    nachhall_s: float = 0.3            # nach eigenen Tönen so lange taub
    retry_s: float = 10.0              # arecord beendet: neuer Versuch


@dataclass
class StimmeCfg:
    # Stimmproben im Modus pet (xgonav/stimme.py): Äußerungen aus dem Ortungsstrom (braucht [ortung]), solange eine
    # Person zu sehen ist. Gleicht eine neue einer gespeicherten Probe (MFCC + DTW), gilt die Probe als bestätigt und
    # wird wie ein Wellensittich gefiltert nachgeplappert ([pet] parrot*), danach jedes Mal, wenn er sie wiedererkennt.
    # Ab max_confirmed ersetzt eine neue die am seltensten gehörte (Nutzer 2026-11-19).
    enabled: bool = True
    dir: str = "/opt/xgonav/stimmen"   # kandidaten/ (roh, 16 kHz) und bestaetigt/ (NN_roh.wav, NN.wav gefiltert)
    max_confirmed: int = 10
    pool_n: int = 30                   # so viele unbestätigte Proben, die älteste fliegt raus
    kandidat_max_min: float = 1.0      # unbestätigte so lange behalten (Nutzer 2026-11-19), danach gelöscht
    # Sprecherkennung (VAD) auf dem Pegel der Ortung: öffnet vad_on_db über dem Ruhepegel, endet nach vad_hang_s
    # unter vad_off_db; vad_pre_s Vorlauf. Kürzer als min_s verworfen, bei max_s abgeschnitten.
    # Gemessen an einer beschrifteten Aufnahme am Rider 2026-11-19 (3 Sätze je 5x, Wasserkocher im Hintergrund):
    # mit 9 dB und 0,6 s fehlten kurze und leise Sätze („Komm her“).
    vad_on_db: float = 6.0
    vad_off_db: float = 4.0
    vad_hang_s: float = 0.6            # 0,3 zerschnitt am Rider Sätze an Sprechpausen (2026-11-19)
    vad_pre_s: float = 0.3             # Vorlauf (Nutzer 2026-11-19)
    min_s: float = 0.3
    max_s: float = 3.0
    # Merkmale nur vom Satz: Ränder unter trim_db (unter dem lautesten Rahmen) ab, stimmhaft gezählt nur bis laut_db
    # darunter. Mit 20 dB blieb das Rauschen in den Merkmalen (1,4 s statt 0,6 s), andere Sätze lagen dann nur bei 7-9.
    trim_db: float = 12.0
    laut_db: float = 10.0
    # Anteil stimmhafter Rahmen unter den lauten, sonst kein Sprechen. Stimmhaft: spektrale Flachheit 300-3000 Hz unter
    # flat_max und Grundton 70-400 Hz (Autokorrelation, fensterbereinigt) über per_min. Am Rider 2026-11-19:
    # Sprachausgabe 0,74-0,96, Wasserrauschen 0,0-0,19 (vorher nur Flachheit und 0,3: Rauschen wurde gelernt). Am
    # Rider mit laut_db 10: Sprache 0,50-1,0, Wasserkocher allein 0,12-0,21.
    voiced_min: float = 0.4
    flat_max: float = 0.3
    per_min: float = 0.5
    # Vergleich: DTW über 13 MFCC (Mittelwert abgezogen), Kosten je Schritt; Längen höchstens len_ratio verschieden.
    # match_dist aus Windows-Sprachausgabe mit Raumrauschen (2026-11-19): derselbe Satz, andere Sprechrate 2..4,
    # andere Sätze ab 14. Beschriftete Aufnahme am Rider (tools/stimme.py strom, trim_db 12): derselbe Satz 5,1-8,2,
    # verschiedene ab 8,8, meist 9,5-12. Die geratene 10 hätte Sätze verwechselt.
    match_dist: float = 8.0
    dtw_band: float = 0.25
    len_ratio: float = 1.6
    # Rauschtor vor dem Filter: bis gate_db unter dem lautesten Rahmen offen, auf weiteren knee_db weich zu, stille Ränder
    # weg. Ohne Tor waren 57-86 % der Proben Kocherrauschen (25-31 dB unter der Stimme), die Weichbegrenzung hob es an:
    # klang wie Wassergluckern (Nutzer 2026-11-19).
    gate_db: float = 12.0
    knee_db: float = 6.0
    # Sittich-Filter: speed-mal so schnell (höher und kürzer), Bandpass band_lo..band_hi Hz, Schnarren durch
    # Vibrato (modulierte Verzögerung), Weichbegrenzung drive, Spitze peak_db.
    speed: float = 1.0                 # Nutzer 2026-11-19: nicht schneller abspielen (vorher 1,6)
    pitch_st: float = 12.0             # Halbtöne höher bei gleichem Tempo (WSOLA), Nutzer 2026-11-19: eine Oktave
    band_lo: float = 1000.0
    band_hi: float = 5000.0
    vibrato_hz: float = 25.0
    vibrato_ms: float = 0.3
    drive: float = 2.0
    peak_db: float = -3.0


@dataclass
class WlanCfg:
    # WLAN-Einrichtung über die Tasten (xgonav/wlan.py): Tasten halten -> WLAN-QR vor die Kamera, sonst Hotspot.
    # Kostet ohne Einrichtung nichts (der Tastenwert wird ohnehin gelesen); währenddessen ruht die Bildverarbeitung.
    enabled: bool = True
    buttons: int = 12                  # Bitmaske aus /proc/XGORider/buttons (A=1 B=2 C=4 D=8); A+B fährt herunter
    hold_s: float = 3.0                # so lange halten (im Ruhemodus liest xgonav nur 1x/s)
    cancel_buttons: int = 2            # kurz drücken bricht ab (B)
    qr_hz: float = 5.0                 # QR-Suche höchstens so oft
    qr_model_dir: str = "models/wechat_qr"   # WeChat-QR (opencv-contrib) für gestaltete Codes; "" = einfacher Detektor
    qr_timeout_s: float = 60.0         # ohne Code danach Hotspot (Countdown am Display)
    hotspot_ssid: str = "xgo-rider"
    hotspot_timeout_s: float = 60.0    # Hotspot so lange (Countdown am Display), dann alte Verbindung zurück
    helper: str = "/usr/local/sbin/xgo-wlan"   # root-Teil (deploy/xgo-wlan, sudo-Regel deploy/sudoers-xgonav-wlan)


@dataclass
class CpuCfg:
    # CPU-Pinning (Linux). Leer = nicht pinnen. Am RiderPi: Hauptschleife samt Kamera,
    # OpenCV-Workern und Rest des Systems auf 0-1, YOLO auf 2-3 (Aufrufer + Pool-Thread).
    main_cpus: list = field(default_factory=lambda: [0, 1])
    detector_cpus: list = field(default_factory=lambda: [2, 3])
    # Threads, die Bilder für die Oberfläche zeichnen und kodieren (WebSocket, /frame, /stream).
    # Leer = erben main_cpus. Am RiderPi alle Kerne: neben YOLO ist auf 2-3 noch Luft, und
    # auf 0-1 verdrängten sie sonst die Hauptschleife (p95 39 -> 47 ms gemessen).
    render_cpus: list = field(default_factory=lambda: [0, 1, 2, 3])
    # Bodenmodell-Thread ([freespace] threaded). Leer = erben main_cpus.
    freespace_cpus: list = field(default_factory=lambda: [2, 3])


@dataclass
class RelocCfg:
    # Kartenabgleich (reloc.py): Pose in der Raumkarte aus der Bodengrenze der letzten Sekunden.
    # Rechnet im eigenen Thread, nichts in der Bildschleife. Aus = alles wie bisher.
    enabled: bool = True               # Zustand "Pose unbekannt", Kartensperre, Modus locate
    track: bool = True                 # laufender Abgleich während der Fahrt
    # Puffer der Bodengrenze (Punkte im Roboter-KS): so viele Sekunden und so viel Weg zurück
    window_s: float = 8.0
    window_m: float = 3.0
    hit_res_m: float = 0.05            # Treffer auf dieses Raster ausgedünnt
    free_step_m: float = 0.2           # Freiproben in diesem Abstand auf dem Freistrahl
    free_res_m: float = 0.10           # und auf dieses Raster ausgedünnt
    far_m: float = 0.8                 # Treffergewicht 1 / (1 + (d / far_m)²): der Grenzfehler wächst mit d²
    # laufender Abgleich: höchstens so oft und nur nach so viel Weg oder Drehung
    every_s: float = 2.0
    min_move_m: float = 0.2
    min_turn_deg: float = 20.0
    # Referenzfeld: Breite der Wand (m), Strafe für Treffer auf bekannt freiem Boden, für Freiproben
    # auf Belegtem (free_weight) und auf Unbekanntem (free_weight x unknown_weight)
    sigma_m: float = 0.10
    miss_weight: float = 0.5
    free_weight: float = 0.5
    unknown_weight: float = 0.3
    # lokale Suche um die Odometrie-Pose
    search_m: float = 0.30
    search_deg: float = 8.0
    step_deg: float = 1.0
    # Güte: darunter gilt "unsicher" und es wird nichts korrigiert
    min_points: int = 60               # ausgedünnte Treffer in kartiertem Gebiet (etwa 3 m Wand)
    min_hit_frac: float = 0.55         # Anteil davon, der an einer Wand der Karte liegt
    unique_ratio: float = 0.85         # zweitbeste Lage höchstens so gut wie die beste (nur globale Suche)
    max_slide_m: float = 0.15          # Streuung der fast gleich guten Lagen; darüber gleitet der Abgleich
    # Korrektur je Abgleich: Anteil, Deckel, Totband; über hold_m Rest wird nichts eingetragen
    gain: float = 0.6
    max_step_m: float = 0.08
    max_step_deg: float = 3.0
    deadband_m: float = 0.02
    deadband_deg: float = 0.5
    hold_m: float = 0.10
    hold_deg: float = 3.0
    # Erkunden: Abgleich nur gegen einen Kartenstand, der mindestens so alt ist (sonst bestätigt
    # sich die Karte selbst); neuer Stand alle ref_every_s
    ref_age_s: float = 12.0
    ref_every_s: float = 10.0
    # globale Suche (Lokalisieren): Raster, beste Kandidaten zum Verfeinern
    global_step_deg: float = 3.0
    global_res_m: float = 0.10
    global_top: int = 8
    global_known_frac: float = 0.7     # so viel vom Scan muss in kartiertem Gebiet liegen
    blind_s: float = 2.0               # balancierend so lange ohne frische Raddaten: Pose unbekannt


_log = logging.getLogger(__name__)

# Schlüssel, die es nicht mehr gibt: in einer alten config.toml Warnung statt KeyError (der Dienst startet weiter).
_KANTEN = "klassisches Bodenmodell entfernt 2026-11-18, nur noch FloorNet"
ENTFERNT = {f"freespace.{k}": _KANTEN for k in (
    "primary", "method", "shading_file", "shading_max_gain", "shading_max_gain_bg", "bg_model", "sample_rows",
    "sample_bottom", "sample_cols", "hist_bins", "floor_thresh", "edge_weight", "min_obst_rows", "min_obst_height_m",
    "min_known_frac", "min_learn_v", "horizon_wall", "horizon_margin_rows", "horizon_band_rows", "horizon_floor_frac",
    "wall_edge_thr", "edge_fallback_free", "edge_thin", "edge_k", "edge_chroma_w", "edge_step_thr", "edge_seg_thr",
    "edge_log_s", "edge_glare", "glare_v", "glare_px", "glare_dl", "glare_spread", "native")}
ENTFERNT.update({k: "ein Kurs für alles seit 2026-11-19 ([odom] contra_*, odomfusion.py)"
                 for k in ("odom.yaw_source", "map.yaw_source", "grid.yaw_source")})
UMBENANNT = {"freespace.shadow_net": "freespace.net"}

# Die Defaults oben sind seit 2026-11-12 die Werte des RiderPi (/opt/xgonav/config.toml ohne Testbetrieb und
# ohne Pfade unter /opt). Simulator und Tests rechnen mit dem früheren neutralen Stand: Config.neutral().
NEUTRAL = {"pet.call_m": 0.0, "pet.come_m": 0.0, "pet.drive_v": 0.08, "pet.look_w": 0.12, "pet.turn_w": 0.20, "pet.faint": False, "pet.faint_level": 1.0, "pet.seek_m": 0.0, "pet.roam_m": 0.0, "pet.roam_radius_m": 1.0,
           "robot.vx_limit": 1.5, "robot.max_v": 0.25, "robot.max_w": 0.25, "robot.reverse_gain": 1.0,
           "robot.manual_forward_gain": 1.0, "robot.level_after_standup": False, "camera.sensor_mode": "",
           "camera.scaler_crop": [], "camera.lens_knots_px": [], "camera.lens_g_px": [], "camera.lens_center": [], "camera.lens_roll_deg": 0.0, "camera.lens_yaw_deg": 0.0, "camera.exposure_us": 4000, "camera.analogue_gain": 0.0,
           "camera.mount_offset_mm": 45.0, "camera.cam_h_u8": [], "camera.cam_fwd_mm": 0.0,
           "camera.tilt_deg": 0.0, "imu.stream_hz": 0, "imu.pitch_sign": 0, "imu.yaw_scale_left": 1.0, "imu.yaw_scale_right": 1.0, "imu.time_offset_ms": None, "imu.pitch_ref_deg": 0.0, "imu.use_roll": False,
           "detector.max_hz": 0.0, "detector.classes": [], "freespace.threaded": False,
           "freespace.net": "", "freespace.net_smooth_n": 0, "map.confirm": 1,
           "behavior.follow_dist_m": 1.2, "behavior.dist_deadband": 0.15, "behavior.search_w": 0.12,
           "behavior.turn_w": 0.18, "behavior.search_timeout_s": 20.0, "behavior.height_search_mm": 115.0,
           "behavior.height_fast_mm": 70.0, "output.lcd_fifo": "/tmp/lcd0", "cpu.main_cpus": [],
           "cpu.detector_cpus": [], "cpu.render_cpus": [], "cpu.freespace_cpus": [], "odom.scale_left": 1.0,
           "odom.scale_right": 1.0, "explore.turn_dps": 40.0, "explore.tight_max": 3, "explore.tight_penalty_s": -1.0, "grid.confirm_dr_m": 0.0, "grid.hit_band_quad": 0.0, "grid.max_range_m": 1.25, "grid.free_range_m": 1.5, "grid.size_m": 16.0, "home.turn_w_max": 1.0,
           "reloc.enabled": False, "reloc.track": False, "startbild.enabled": False,
           "control_hz": 20.0}


@dataclass
class Config:
    robot: RobotCfg = field(default_factory=RobotCfg)
    camera: CameraCfg = field(default_factory=CameraCfg)
    ego: EgoCfg = field(default_factory=EgoCfg)
    imu: ImuCfg = field(default_factory=ImuCfg)
    detector: DetectorCfg = field(default_factory=DetectorCfg)
    tracker: TrackerCfg = field(default_factory=TrackerCfg)
    freespace: FreespaceCfg = field(default_factory=FreespaceCfg)
    map: MapCfg = field(default_factory=MapCfg)
    behavior: BehaviorCfg = field(default_factory=BehaviorCfg)
    output: OutputCfg = field(default_factory=OutputCfg)
    emotion: EmotionCfg = field(default_factory=EmotionCfg)
    cpu: CpuCfg = field(default_factory=CpuCfg)
    odom: OdomCfg = field(default_factory=OdomCfg)
    grid: GridCfg = field(default_factory=GridCfg)
    reloc: RelocCfg = field(default_factory=RelocCfg)
    home: HomeCfg = field(default_factory=HomeCfg)
    startbild: StartbildCfg = field(default_factory=StartbildCfg)
    explore: ExploreCfg = field(default_factory=ExploreCfg)
    patrol: PatrolCfg = field(default_factory=PatrolCfg)
    pet: PetCfg = field(default_factory=PetCfg)
    ruhe: RuheCfg = field(default_factory=RuheCfg)
    wlan: WlanCfg = field(default_factory=WlanCfg)
    dunkel: DunkelCfg = field(default_factory=DunkelCfg)
    ortung: OrtungCfg = field(default_factory=OrtungCfg)
    stimme: StimmeCfg = field(default_factory=StimmeCfg)
    # Regelschritte je Sekunde. Mit Bildverarbeitung an dem Bild, das dem Takt am nächsten liegt (App._step), also
    # höchstens die Bildrate: 24 = jedes Bild. Bis 2026-11-17 stand hier 20, geregelt wurde am Rider aber nur jedes
    # 2. Bild (~12 Hz). Ohne Bildverarbeitung (manual) nach der Uhr.
    control_hz: float = 24.0

    @classmethod
    def neutral(cls) -> "Config":
        """Stand vor den RiderPi-Defaults (NEUTRAL): Kamera ohne Neigung und Kalibrierung, kein FloorNet (der
        Simulator gibt sim.SimFloorNet mit), keine CPU-Bindung, Kartenabgleich aus. Für Simulator und Tests; tests/sim_run.py --rider setzt Rider-Werte darauf."""
        cfg = cls()
        for name, wert in NEUTRAL.items():
            ziel, _, key = name.rpartition(".")           # ohne Punkt: Schlüssel auf oberster Ebene
            setattr(getattr(cfg, ziel) if ziel else cfg, key, list(wert) if isinstance(wert, list) else wert)
        return cfg

    @classmethod
    def load(cls, path: str | Path | None) -> "Config":
        cfg = cls()
        if path:
            with open(path, "rb") as f:
                data = tomllib.load(f)
            cfg.apply(data)
        return cfg

    def apply(self, data: dict) -> None:
        for key, val in data.items():
            if not hasattr(self, key):
                raise KeyError(f"Unbekannter Config-Abschnitt: {key}")
            cur = getattr(self, key)
            if dataclasses.is_dataclass(cur) and isinstance(val, dict):
                for k, v in val.items():
                    name = f"{key}.{k}"
                    if name in UMBENANNT:
                        _log.warning("Config: %s heißt jetzt %s", name, UMBENANNT[name])
                        k = UMBENANNT[name].rpartition(".")[2]
                    elif name in ENTFERNT:
                        _log.warning("Config: %s gibt es nicht mehr (%s), wird ignoriert", name, ENTFERNT[name])
                        continue
                    if not hasattr(cur, k):
                        raise KeyError(f"Unbekannter Config-Schlüssel: {key}.{k}")
                    setattr(cur, k, v)
            else:
                setattr(self, key, val)
