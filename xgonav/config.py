"""Zentrale Konfiguration. Defaults hier, Overrides per TOML (--config)."""
from __future__ import annotations

import dataclasses
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
    max_v: float = 0.15            # ~0.37 Firmware-Einheiten
    max_w: float = 0.30            # ~90 °/s
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
    # [16, 0, 2560, 1920] = Ausschnitt des 640x480-Modus, damit hfov/tilt/shading.json gültig bleiben.
    scaler_crop: list = field(default_factory=lambda: [16, 0, 2560, 1920])
    # Bilddauer-Korrektur, nur nötig im 640x480-Modus (dort ~1.061): angefragt = 1/fps * Faktor.
    frame_clock_corr: float = 1.0
    exposure_us: int = 20000        # kurze Belichtung gegen Bewegungsunschärfe; 0 = Auto
    analogue_gain: float = 16.0     # feste Verstärkung 1..63,9 (OV5647); 0 = Auto
    # RiderPi: aus Drehungen gegen den IMU-Kurs gemessen (2026-11-07, f = 310 px @ 640, 304–313 in drei
    # Aufnahmen; Verzeichnung vernachlässigbar). Der Zollstock (88,5°) legt nur f·h fest. RPi Cam v1 hätte 62,2.
    hfov_deg: float = 91.8
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
    # Spalte der Fahrtrichtung minus Bildmitte (px bei width): gedrehter Einbau oder verschobene optische Mitte,
    # für Peilungen gleich. RiderPi: Fluchtpunkt bei Geradeausfahrt (8 Fahrten, 45 m, 2026-11-08) Spalte 333 -> 13.
    cx_offset_px: float = 13.0
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
    pitch_sign: int = 0            # 0 = automatisch aus Bild-Korrelation, sonst +1/-1
    time_offset_ms: float | None = None   # None = automatisch
    pitch_ref_deg: float = -4.43     # IMU-Rohpitch bei der Kamerakalibrierung (calibrate.py wall)
    sigma_deg: float = 0.3         # Restunsicherheit der Nicklage mit IMU
    min_corr: float = 0.6
    sync_range_ms: float = 100.0
    sync_step_ms: float = 4.0
    max_age_s: float = 0.3
    # imu_log so oft einlesen (s). Bei 0,05 lag der neueste Wert beim Bild bis ~50 ms zurück (Nick wird dann
    # gehalten, nicht interpoliert). RiderPi: ein Lesen 185 µs (128 Zeilen), geparst werden nur neue Zeilen.
    poll_s: float = 0.01


@dataclass
class HorizonCfg:
    """Horizont im Stand (xgonav/horizon.py, docs/calibration.md): IMU-Referenz je Aufstehen,
    Horizontzeile je Körperhöhe. Aus: Nick wie bisher aus pitch_ref_deg + tilt_deg."""
    enabled: bool = False
    file: str = "horizont.json"    # Tabelle Körperhöhe -> Horizontzeile (vom Web geschrieben)
    settle_s: float = 3.0          # nach Aufstehen/Höhenänderung warten
    window_s: float = 3.0          # dann über so lange den IMU-Rohnick mitteln
    max_std_deg: float = 0.5       # größere Streuung: Fenster verwerfen, neu messen
    min_samples: int = 50          # IMU-Proben im Fenster (50 Hz -> 150 erwartet)
    # Schnelle Änderungen aus dem Bild: Bildnick ab dem letzten IMU-Wert aufsummiert, spätestens nach
    # bild_max_s wieder auf die IMU gesetzt (aufsummierter Bildnick driftet, 2026-11-07 ~0,13°/min).
    # 0 = nur IMU.
    bild_max_s: float = 0.5


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
    enabled: bool = True
    # Bodenmodell in eigenem Thread (FreeSpaceWorker): nimmt der Hauptschleife jedes 2. Bild
    # ~13 ms ab (CM4); das Ergebnis kommt dafür ein Bild später an. False = wie früher im Takt.
    threaded: bool = True
    width: int = 160
    sample_rows: float = 0.34      # Höhe der Boden-Referenz (Anteil der Bildhöhe)
    sample_bottom: float = 0.0     # Abstand der Referenz vom unteren Bildrand (Anteil der Bildhöhe)
    sample_cols: float = 0.5       # mittlerer Bildanteil
    sectors: int = 5
    hist_bins: int = 16
    floor_thresh: float = 0.08     # Rückprojektions-Schwelle
    edge_weight: float = 0.4       # Faktor an Kanten; 0.15 machte Laminatfugen zu Hindernissen (RiderPi)
    min_obst_rows: int = 6         # Hindernis erst ab so vielen Zeilen Höhe (Arbeitsauflösung, 120 Zeilen)
    # Ein Nicht-Boden-Lauf zählt erst, wenn er so hoch ist wie ein aufrechter Gegenstand dieser
    # Höhe an seinem Fußpunkt (oder bis zum Horizont reicht). Flache Flecken, Fugen und Kabel
    # auf dem Boden sind kürzer. 0 = aus. Braucht die Kamerageometrie.
    min_obst_height_m: float = 0.04
    # Linsenabschattung (tools/calibrate.py shading -> JSON). "" = aus (Simulator).
    # Pixel, die mehr als shading_max_gain Verstärkung bräuchten, gelten als unzuverlässig
    # und nie als Hindernis (RiderPi: untere Ecken, äußere Ränder).
    shading_file: str = "shading.json"
    shading_max_gain: float = 5.0
    # Spalten, in denen zwischen Horizont und unterem Rand weniger als dieser Anteil messbar ist
    # (Linsenabschattung: am RiderPi je ~32 von 160 Spalten am Rand gar nicht), gelten als
    # unbekannt (NaN) statt "frei bis Horizont". Den nicht messbaren Streifen unten in den
    # übrigen Spalten behandelt known_from.
    min_known_frac: float = 0.2
    # Zweites Bodenmodell ohne Rotkanal für den Rand: Wo Rot nicht mehr messbar ist (Abschattung),
    # Blau und Grün aber bis shading_max_gain_bg korrigierbar sind, entscheidet ein Histogramm aus
    # Grünanteil G/(B+G) und Helligkeit (B+G)/2, gelernt aus derselben Probe. Am RiderPi sind mit
    # allen Kanälen nur ~46 % der Pixel messbar, mit Blau+Grün bis Faktor 8 alle.
    bg_model: bool = False         # aus: Fehlalarme in den unteren Ecken bei dunklem Bild (RiderPi 2026-11-04)
    # Probe mit mittlerer Helligkeit (V, nach Abschattungs-Korrektur) darunter wird nicht nachgelernt
    # (z. B. erste Bilder nach dem Kamerastart).
    min_learn_v: float = 12.0
    # Neu lernen nach Start der Bildverarbeitung, Aufstehen und geänderter Belichtung; so lange
    # vorher kein Bodenmodell rechnen (Belichtung/Lage regeln sich ein). Anlass: Ein Modell aus
    # anderer Lage passte nie mehr (Nachführung nur bei passender Probe) -> alles blockiert.
    restart_hold_s: float = 1.0
    shading_max_gain_bg: float = 8.0
    # Horizontregel: Boden liegt nie über dem Horizont. Ist der Streifen knapp darüber
    # (horizon_band_rows Zeilen, horizon_margin_rows Abstand gegen Nickfehler; Arbeitsauflösung)
    # zu horizon_floor_frac "Boden", ist die Spalte eine Wand in Bodenfarbe; Fußpunkt = erste
    # senkrechte Helligkeitsstufe >= wall_edge_thr (Grauwerte je Zeile) unter dem Horizont.
    horizon_wall: bool = False     # aus: im Simulator Fehlalarme durch Nickfehler (Boden knapp über dem Horizont)
    # Verfahren für die Bodengrenze: "farbe" (Histogramm der Bodenfarbe, bisher) oder "kanten"
    # (erste Stufe von unten, über der es anders aussieht; unempfindlich gegen Abschattung und
    # graue Wand über grauem Boden). Das Farbmodell lernt in beiden Fällen weiter (Rand unten).
    # "beide": Kanten, und wo diese in einer Spalte nichts finden (frei bis edge_fallback_free),
    # das Farbmodell (Nutzer: Farbe im Dunkeln besser, Kanten im Hellen).
    method: str = "kanten"
    # FloorNet im Schattenbetrieb (xgonav/floornet.py): Gewichte, z. B. "models/floornet.bin". Rechnet im
    # Bodenmodell-Thread mit (threaded = true) und erscheint im Overlay, entscheidet aber nichts. "" = aus.
    shadow_net: str = "models/floornet.bin"
    # Wer entscheidet: "method" (das Verfahren oben, FloorNet rechnet höchstens mit) oder "net" (FloorNet liefert die
    # Grenze für Karten und Regelung allein; das Verfahren wird dann weder gerechnet noch gezeichnet). "net" braucht
    # shadow_net und threaded; fehlt das Netz oder fällt es aus, rechnet und entscheidet wieder das Verfahren.
    primary: str = "net"
    # FloorNet, einzelne Zacken herausnehmen (1-3 Spalten breite Sprünge an senkrechten Kanten): Weicht eine Spalte um
    # mehr als net_outlier_px Vollbildzeilen vom Median der 7 Spalten um sie ab und liegt beim Medianwert mindestens
    # net_outlier_ratio der Masse ihres eigenen Gipfels, gilt das beste Fach beim Medianwert. 0 = aus.
    net_outlier_px: float = 25.0
    net_outlier_ratio: float = 0.3
    # FloorNet, Kante über die Zeit beruhigen (floornet.ZeilenGlaetter, nur mit primary = "net"): die letzten
    # net_smooth_n Ausgaben werden über Pose und Nicken auf das aktuelle Bild umgerechnet, je Spalte gilt der Median.
    # 3: ein Ausreißer in einem einzelnen Bild verschwindet, ein neues Hindernis erscheint ein Netzbild (83 ms) später.
    # 0 oder 1 = aus. Braucht die Kartenpose; ohne sie gilt die Ausgabe unverändert.
    net_smooth_n: int = 3
    edge_fallback_free: float = 0.98
    edge_thin: bool = True         # dünne Linien vorher entfernen (Fugen – aber auch Kontaktschatten)
    edge_k: int = 3                # Zeilen je Seite für den Stufenvergleich (Arbeitsauflösung)
    edge_chroma_w: float = 2.0     # Gewicht der LAB-Farbkanäle a/b gegenüber der Helligkeit L
    edge_step_thr: float = 35.0    # Mindest-Stufe (Summe |ΔL| + w·(|Δa|+|Δb|), 0..255-Skala)
    edge_seg_thr: float = 28.0     # Mindest-Abweichung des Stücks über der Stufe vom Boden darunter
    # Lichtschein: Spiegelung einer Lampe im glänzenden Boden erzeugt eine Stufe weit vor der Lampe
    # (bei 12,8 cm Kamerahöhe etwa ein Viertel ihrer Entfernung). Erkennung: über der Stufe heller
    # als darunter (>= glare_dl) und zwischen Stufe und Horizont >= glare_px übersteuerte Pixel
    # (Maximum der Kanäle >= glare_v). Solche Spalten gelten als unbekannt (NaN), nicht als frei.
    # Helligkeit für die Stufe logarithmisch (Verhältnis statt Differenz): L' = 255·ln(1+L/s)/ln(1+255/s).
    # Hebt dunkle Szenen an (graue Wand im Dunkeln: L 25 gegen 11), dämpft helle. 0 = linear.
    edge_log_s: float = 16.0
    edge_glare: bool = True
    glare_v: int = 250
    glare_px: int = 2
    glare_dl: float = 8.0
    glare_spread: int = 8          # Schein auf helle Nachbarspalten mit ähnlicher Stufenzeile ausdehnen
    # Kantenverfahren im C++-Kern (xgonav/native, make -C xgonav/native), sonst numpy; gleiches Ergebnis
    native: bool = True
    horizon_margin_rows: int = 2
    horizon_band_rows: int = 6
    horizon_floor_frac: float = 0.6
    wall_edge_thr: float = 8.0


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
    # Drehung der Merkkarte: "odom" = wie [odom] yaw_source (Räder, sonst Kamera); "imu" = Firmware-Kurs aus imu_log
    # (~50 Hz), bei fehlender/alter IMU wie "odom". RiderPi 2026-11-08, volle Drehungen gegen Bilder: IMU 0,997,
    # Räder 0,992 des wahren Winkels; je Drehung weichen beide zufällig ~2° (p90 ~8°) voneinander ab.
    yaw_source: str = "imu"
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
    height_cruise_mm: float = 85.0
    height_search_mm: float = 100.0
    height_fast_mm: float = 80.0


@dataclass
class OutputCfg:
    web_host: str = "127.0.0.1"    # Schnittstelle nur lokal; Oberfläche über nginx (web/, deploy/nginx-xgonav.conf)
    web_port: int = 8088           # 0 = aus
    web_fps: int = 10
    lcd_fifo: str = ""             # "" = aus
    lcd_fps: int = 15             # CM4 gemessen: kostet die Hauptschleife nichts Messbares
    record_dir: str = "~/fahrt"   # Rohaufzeichnung (Knopf in der Oberfläche): je Aufnahme ein Unterverzeichnis
    record_min_free_mb: int = 500 # darunter schaltet sie sich ab


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


@dataclass
class OdomCfg:
    # Rad-Odometrie aus /proc/XGORider/wheels_log (Firmware R-1.1.6+Patch, Treiber mit 0x0d/0x0c).
    # source: "auto" = Räder, wenn vorhanden und frisch, sonst Fahrbefehl x vx_limit;
    # "command" = immer Fahrbefehl (wie früher).
    enabled: bool = True
    source: str = "auto"
    proc_dir: str = "/proc/XGORider"
    wheel_diameter_m: float = 0.060    # RiderPi, Angabe des Nutzers
    track_m: float = 0.090             # Spurweite (Abstand der Radaufstandspunkte)
    ticks_per_rev: int = 1024          # am Rider gemessen (1 U von Hand: 1029 / 1043)
    # Faktor je Rad auf den Weg (tools/calibrate_odom.py): ungleiche Reifen/Schlupf. Für die
    # Strecke zählt der Mittelwert, für den Kurs der Unterschied (2 % = ~1,3° je 10 cm).
    scale_left: float = 1.0073
    scale_right: float = 1.0087
    poll_hz: float = 25.0              # wheels_log lesen (Treiber puffert 64 Proben)
    # Abgesetzt (Rider balanciert nicht) werden die Räder nicht gelesen; die Pose bleibt dann stehen, auch wenn er
    # von Hand gerollt wird. Beim Aufstehen beginnt das Lesen neu, ohne den Weg dazwischen. true = immer lesen.
    read_idle: bool = False
    max_age_s: float = 0.3             # älter -> Rückfall auf den Fahrbefehl
    max_jump_ticks: int = 2048         # größerer Sprung zwischen zwei Proben = Neustart/Fehler
    # Kurs für Verhalten (explore) und Merkkarte: "räder" (wenn frisch, sonst Kamera) oder "kamera".
    # Der Bild-Kurs verfälscht bei Vorwärtsfahrt (2026-11-04: −8,9° auf 2 m geradeaus).
    yaw_source: str = "räder"


@dataclass
class GridCfg:
    # Belegungskarte (gridmap.py): Welt-KS ab Startpose, nur Anzeige
    enabled: bool = True
    resolution_m: float = 0.05
    size_m: float = 16.0               # Kantenlänge, Start in der Mitte
    # Treffer nur bis hierhin: Kamera nur 12,8 cm hoch, eine Bildzeile sind bei 2 m schon ~40 cm
    max_range_m: float = 1.25
    min_range_m: float = 0.15          # davor sieht die Kamera den Boden nicht
    # Treffer erst eintragen, wenn dieselbe Stelle (± confirm_cells Zellen) aus Entfernungen getroffen wurde, die
    # sich um mindestens so viel unterscheiden (m). Spiegelungen im glänzenden Boden und Fugen wandern mit dem
    # Rider und werden so nie belegt (Anzeige: orange). Aus Drehungen auf der Stelle allein entsteht nichts.
    # 0 = aus (jeder Treffer zählt). RiderPi 0.2 (2026-11-08: Fensterunterkante gespiegelt, doppelte Wand).
    # Simulator explore 120 s: Wände 82 % (aus) / 57 % (±1 Zelle) / 73 % (±2 Zellen), Präzision 0,90 / 0,95 / 0,93.
    confirm_dr_m: float = 0.2
    confirm_cells: int = 2
    # Punkte der Merkkarte, vor denen die Bahnprüfung bremst, gelten in der Raumkarte als bestätigt und belegt
    # (auch ohne confirm_dr_m): Was ihn bremst, steht in der Karte, und der Planer fährt darum herum.
    mark_blocking: bool = True
    column_step: int = 2              # jede k-te Spalte des Bodenmodells (160 Spalten)
    l_free: float = -0.4               # Log-Odds je Beobachtung
    l_occ: float = 0.5                 # zwei Treffer bis "belegt" (l_show): ein Ausreißer sperrt nichts
    free_margin_m: float = 0.10        # Freistrahl endet so weit vor der Grenze (Messrauschen)
    free_margin_quad: float = 0.05     # dazu so viel x Entfernung² (der Fehler wächst quadratisch)
    free_range_m: float = 1.1          # Freistrahlen bis hierhin (Treffer nur bis max_range_m); 0 = wie max_range_m
    driven_free_r_m: float = 0.10      # gefahrener Weg gilt so breit als frei; 0 = aus
    max_turn_dps: float = 40.0         # bei schnellerer Drehung nichts eintragen (Unschärfe, Pose)
    occ_free_factor: float = 0.5       # schon belegte Zellen baut ein Freistrahl nur so stark ab
    l_min: float = -3.0
    l_max: float = 3.5
    l_show: float = 0.7                # ab hier "belegt"/"frei" in Anzeige und Statistik
    view_m: float = 6.0                # Bildausschnitt für die Anzeige
    # Drehung der Kartenpose: "räder" (Rad-Odometrie, sonst Kamera), "kamera" (Ego-Motion) oder "imu" (Firmware-Kurs
    # aus imu_log, ist der zu alt die 5-Hz-Treppe aus /proc yaw, sonst wie "räder"). RiderPi 2026-11-06, 32 m: mit Rädern 75 cm Fehler (Schlupf beim Drehen), mit IMU ~6 cm.
    yaw_source: str = "imu"
    # Eintragen in eigenem Thread (GridWorker, Kerne wie [cpu] freespace_cpus); False = im Regelschritt
    threaded: bool = True
    # Strahlen im C++-Kern eintragen (xgonav/native/gridfast.cpp), wenn gebaut; gleiches Ergebnis wie numpy
    native: bool = True
    # Speichern/Laden (npz). Leer = aus. Beim Laden gilt Pose = Startpunkt: der Rider muss
    # vor dem Start von xgonav wieder dort stehen (ohne Marker findet er sich nicht wieder).
    file: str = ""
    save_every_s: float = 60.0         # höchstens so oft, und nur bei Änderungen; dazu beim Beenden


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
    turn_dps: float = 40.0             # Drehzeit-Schätzung
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
    tight_max: int = 3
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
    turn_w: float = 0.20
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
    seek_m: float = -1.0               # je Verschwinden höchstens so weit zur letzten Stelle
    seek_stop_m: float = 0.4           # so weit vor der letzten Stelle halten
    roam_m: float = -1.0               # je Phase ohne Begegnung höchstens so weit streifen (endet spätestens mit doze_s)
    roam_radius_m: float = -1.0        # beim Streifen nie weiter vom Platz, an dem er in pet aufstand
    roam_leg_m: float = 0.4            # Länge einer Etappe
    roam_after_s: float = 30.0         # so lange niemand: umschauen bzw. streifen
    roam_repeat_s: float = 45.0        # danach wieder
    roam_free_min: float = 0.55        # Freiraum im Zentrum (wie behavior.slow_free), sonst fährt er dort nicht hin
    roam_battery: int = 40             # %: darunter weder suchen noch streifen
    drive_v: float = 0.08              # normiert (x vx_limit = m/s)
    look_w: float = 0.12               # Drehrate beim Umschauen, normiert
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
    # Modus locate: Rundumblick 360° + so viel, Abbruch bei so viel Verschiebung, Versuche
    scan_extra_deg: float = 30.0
    max_shift_m: float = 0.15
    tries: int = 2
    blind_s: float = 2.0               # balancierend so lange ohne frische Raddaten: Pose unbekannt


# Die Defaults oben sind seit 2026-11-12 die Werte des RiderPi (/opt/xgonav/config.toml ohne Testbetrieb und
# ohne Pfade unter /opt). Simulator und Tests rechnen mit dem früheren neutralen Stand: Config.neutral().
NEUTRAL = {"pet.faint": False, "pet.faint_level": 1.0, "pet.seek_m": 0.0, "pet.roam_m": 0.0, "pet.roam_radius_m": 1.0,
           "robot.vx_limit": 1.5, "robot.max_v": 0.25, "robot.max_w": 0.25, "robot.reverse_gain": 1.0,
           "robot.manual_forward_gain": 1.0, "robot.level_after_standup": False, "camera.sensor_mode": "",
           "camera.scaler_crop": [], "camera.exposure_us": 4000, "camera.analogue_gain": 0.0,
           "camera.mount_offset_mm": 45.0, "camera.cam_h_u8": [], "camera.cam_fwd_mm": 0.0,
           "camera.cx_offset_px": 0.0, "camera.tilt_deg": 0.0, "imu.stream_hz": 0, "imu.pitch_ref_deg": 0.0,
           "detector.max_hz": 0.0, "detector.classes": [], "freespace.threaded": False,
           "freespace.sample_rows": 0.18, "freespace.sample_cols": 0.3, "freespace.shading_file": "",
           "freespace.shading_max_gain": 8.0, "freespace.method": "farbe", "freespace.shadow_net": "",
           "freespace.primary": "method", "freespace.net_smooth_n": 0, "map.confirm": 1, "map.yaw_source": "odom",
           "behavior.follow_dist_m": 1.2, "behavior.dist_deadband": 0.15, "behavior.search_w": 0.12,
           "behavior.turn_w": 0.18, "behavior.search_timeout_s": 20.0, "behavior.height_search_mm": 115.0,
           "behavior.height_fast_mm": 70.0, "output.lcd_fifo": "/tmp/lcd0", "cpu.main_cpus": [],
           "cpu.detector_cpus": [], "cpu.render_cpus": [], "cpu.freespace_cpus": [], "odom.scale_left": 1.0,
           "odom.scale_right": 1.0, "grid.confirm_dr_m": 0.0, "grid.free_range_m": 1.5, "grid.yaw_source": "räder",
           "reloc.enabled": False, "reloc.track": False}


@dataclass
class Config:
    robot: RobotCfg = field(default_factory=RobotCfg)
    camera: CameraCfg = field(default_factory=CameraCfg)
    ego: EgoCfg = field(default_factory=EgoCfg)
    imu: ImuCfg = field(default_factory=ImuCfg)
    horizon: HorizonCfg = field(default_factory=HorizonCfg)
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
    explore: ExploreCfg = field(default_factory=ExploreCfg)
    patrol: PatrolCfg = field(default_factory=PatrolCfg)
    pet: PetCfg = field(default_factory=PetCfg)
    ruhe: RuheCfg = field(default_factory=RuheCfg)
    control_hz: float = 20.0

    @classmethod
    def neutral(cls) -> "Config":
        """Stand vor den RiderPi-Defaults (NEUTRAL): Kamera ohne Neigung und Kalibrierung, Farbmodell, keine
        CPU-Bindung, Kartenabgleich aus. Für Simulator und Tests; tests/sim_run.py --rider setzt Rider-Werte darauf."""
        cfg = cls()
        for name, wert in NEUTRAL.items():
            abschnitt, key = name.split(".")
            setattr(getattr(cfg, abschnitt), key, list(wert) if isinstance(wert, list) else wert)
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
                    if not hasattr(cur, k):
                        raise KeyError(f"Unbekannter Config-Schlüssel: {key}.{k}")
                    setattr(cur, k, v)
            else:
                setattr(self, key, val)
