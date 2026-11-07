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
    vx_limit: float = 1.5          # Einheit der Firmware bei u8=0/255
    vyaw_limit_dps: float = 360.0  # Grad/s bei u8=0/255
    # Sicherheitsgrenzen für den Autonomiebetrieb (Anteil des Vollausschlags)
    max_v: float = 0.25            # ~0.37 Firmware-Einheiten
    max_w: float = 0.25            # ~90 °/s
    # Rückwärts war 2026-11-02 viel schwächer (2 s mit u8 109 ~4 cm, mit 147 vorwärts 86 cm), daher
    # werden negative v beim Senden mit reverse_gain multipliziert; max_v begrenzt davor den
    # Sollwert. 1.0 = symmetrisch. Mit den Firmware-Patches vom 2026-11-05 setzt die Firmware beide
    # Richtungen gleich um (Sollwert -> Raddrehzahl 2,67 vorwärts, 2,65 rückwärts).
    reverse_gain: float = 1.0
    # Derselbe Faktor vorwärts, nur in der Handsteuerung. Die autonomen Modi bleiben bei 1: im
    # Simulator (--dyn, 2026-11-05) gab Faktor 2 dort 1 Kollision in 8 Läufen und Zum Start kam nur
    # noch 1 von 4 Mal an (Faktor 1: 0 Kollisionen, 4 von 4); die Freiraum-Schwellen passen nicht dazu.
    manual_forward_gain: float = 1.0
    # Die Beine haben eine Totzone: bei gleichem Befehl steht der Rider je nach Vorgeschichte bei
    # +0,7..+3,4° Roll (2026-11-05). Ein kurzer Impuls der Nivellierung in der Firmware richtet ihn
    # aus, danach bleibt er so stehen (dauerhaft an schwankt er mehr). Auslöser: einmal nach jedem
    # Aufstehen (level_after_standup) und der Knopf im Web (POST /level).
    level_pulse_s: float = 3.0
    level_after_standup: bool = False
    accel_v: float = 0.8           # max. Änderung von v pro Sekunde (normiert)
    accel_w: float = 2.0
    keepalive_s: float = 0.3       # Sollwert periodisch neu schreiben
    stall_timeout_s: float = 0.5   # kein drive() mehr -> Stopp (Userspace-Watchdog)
    kernel_watchdog_ms: int = 1000 # nur wirksam mit Kernel-Patch (settings/motion_timeout_ms)
    height_min_mm: float = 60.0
    height_max_mm: float = 120.0   # min/max legen nur die Skala mm -> u8 fest
    height_limit_u8: int = 205     # höchster Sollwert; bei 235 Stopp 252 (2026-11-06, Ursache offen)


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
    sensor_mode: str = ""
    # Sensorausschnitt [x, y, w, h] in Vollbild-Pixeln (ScalerCrop); leer = ganzer Modus.
    # [16, 0, 2560, 1920] = Ausschnitt des 640x480-Modus, damit hfov/tilt/shading.json gültig bleiben.
    scaler_crop: list = field(default_factory=list)
    # Bilddauer-Korrektur, nur nötig im 640x480-Modus (dort ~1.061): angefragt = 1/fps * Faktor.
    frame_clock_corr: float = 1.0
    exposure_us: int = 4000        # kurze Belichtung gegen Bewegungsunschärfe; 0 = Auto
    analogue_gain: float = 0.0     # feste Verstärkung 1..63,9 (OV5647); 0 = Auto
    hfov_deg: float = 88.5         # RiderPi: Zollstock bei 100 mm Kamerahöhe (f = 328 px @ 640); RPi Cam v1 hätte 62,2
    # Geometrie für Entfernung per Bodenebene (KALIBRIEREN, siehe README)
    mount_offset_mm: float = 45.0  # Kamerahöhe über Boden = Körperhöhe + Offset
    tilt_deg: float = 0.0          # >0 = Kamera schaut nach oben
    loop_file: bool = True         # Videodatei endlos abspielen (Replay)
    realtime_file: bool = True     # Videodatei in Echtzeit statt so schnell wie möglich


@dataclass
class ImuCfg:
    enabled: bool = True
    proc_dir: str = "/proc/XGORider"
    imu_hz: int = 50               # wird beim Start nach settings/imu_hz geschrieben
    pitch_sign: int = 0            # 0 = automatisch aus Bild-Korrelation, sonst +1/-1
    time_offset_ms: float | None = None   # None = automatisch
    pitch_ref_deg: float = 0.0     # IMU-Rohpitch bei der Kamerakalibrierung (calibrate.py wall)
    sigma_deg: float = 0.3         # Restunsicherheit der Nicklage mit IMU
    min_corr: float = 0.6
    sync_range_ms: float = 100.0
    sync_step_ms: float = 4.0
    max_age_s: float = 0.3


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
    max_hz: float = 0.0
    classes: list[str] = field(default_factory=list)  # leer = alle COCO-Klassen


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
    threaded: bool = False
    width: int = 160
    sample_rows: float = 0.18      # Höhe der Boden-Referenz (Anteil der Bildhöhe)
    sample_bottom: float = 0.0     # Abstand der Referenz vom unteren Bildrand (Anteil der Bildhöhe)
    sample_cols: float = 0.30      # mittlerer Bildanteil
    sectors: int = 5
    hist_bins: int = 16
    floor_thresh: float = 0.08     # Rückprojektions-Schwelle
    edge_weight: float = 0.4       # Faktor an Kanten; 0.15 machte Laminatfugen zu Hindernissen (RiderPi)
    min_obst_rows: int = 6         # Hindernis erst ab so vielen Zeilen Höhe (Arbeitsauflösung, 120 Zeilen)
    # Ein Nicht-Boden-Lauf zählt erst, wenn er so hoch ist wie ein aufrechter Gegenstand dieser
    # Höhe an seinem Fußpunkt (oder bis zum Horizont reicht). Flache Flecken, Fugen und Kabel
    # auf dem Boden sind kürzer. 0 = aus. Braucht die Kamerageometrie.
    min_obst_height_m: float = 0.04
    # Diese Klassen werden nicht als Hindernis eingetragen: Die Box großer Möbel sagt nichts über die
    # Standfläche, ihre Unterkante ist kein Fußpunkt (RiderPi 2026-11-06: "bed" vom Bett bis zum unteren
    # Bildrand über die volle Breite, alles blockiert). Das Möbel selbst findet das Bodenmodell.
    no_block_cls: list[str] = field(default_factory=lambda: ["bed", "couch", "dining table"])
    # Linsenabschattung (tools/calibrate.py shading -> JSON). "" = aus (Simulator).
    # Pixel, die mehr als shading_max_gain Verstärkung bräuchten, gelten als unzuverlässig
    # und nie als Hindernis (RiderPi: untere Ecken, äußere Ränder).
    shading_file: str = ""
    shading_max_gain: float = 8.0
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
    method: str = "farbe"
    # FloorNet im Schattenbetrieb (xgonav/floornet.py): Gewichte, z. B. "models/floornet.bin". Rechnet im
    # Bodenmodell-Thread mit (threaded = true) und erscheint im Overlay, entscheidet aber nichts. "" = aus.
    shadow_net: str = ""
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
    confirm: int = 1
    confirm_radius_m: float = 0.08


@dataclass
class BehaviorCfg:
    mode: str = "idle"             # idle | follow | explore | approach | home
    target_class: str = "person"
    follow_dist_m: float = 1.2
    approach_dist_m: float = 0.5
    kp_turn: float = 2.5           # 1/s: Soll-Drehrate = kp * Winkelfehler
    kd_turn: float = 0.25          # s
    kp_dist: float = 0.35          # normiertes v pro Meter Abstandsfehler
    dist_deadband: float = 0.15    # m
    ff_dist: float = 0.8           # Vorsteuerung mit geschätzter Zielgeschwindigkeit
    cruise_v: float = 0.15
    backup_v: float = 0.10
    search_w: float = 0.12
    turn_w: float = 0.18           # Erkunden: Drehrate beim Wegdrehen (früher fest 1,5 x search_w)
    lost_search_s: float = 1.0
    search_timeout_s: float = 20.0
    blocked_free: float = 0.22     # Freiraum im Zentrum darunter = blockiert
    slow_free: float = 0.55        # darunter proportional abbremsen
    ttc_avoid_s: float = 1.2       # Erkunden: Time-to-contact darunter -> ausweichen
    ttc_stop_s: float = 0.8        # Sicherheitsschicht: darunter keine Vorwärtsfahrt
    height_cruise_mm: float = 85.0
    height_search_mm: float = 115.0
    height_fast_mm: float = 70.0


@dataclass
class OutputCfg:
    web_host: str = "127.0.0.1"    # Schnittstelle nur lokal; Oberfläche über nginx (web/, deploy/nginx-xgonav.conf)
    web_port: int = 8088           # 0 = aus
    web_fps: int = 10
    lcd_fifo: str = "/tmp/lcd0"    # "" = aus
    lcd_fps: int = 15             # CM4 gemessen: kostet die Hauptschleife nichts Messbares


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
    scale_left: float = 1.0
    scale_right: float = 1.0
    poll_hz: float = 25.0              # wheels_log lesen (Treiber puffert 64 Proben)
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
    # Das Bodenmodell meldet die Grenze so viele Zeilen (seiner Auflösung) unterhalb der wahren
    # (Stufenfilter, Erosion). Nur für die Raumkarte; die Merkkarte behält das als Sicherheitsrand.
    # Simulator-Sweep 2026-11-05: 1,5; am RiderPi noch nicht gegen eine bekannte Entfernung geprüft.
    row_bias: float = 1.5
    column_step: int = 2               # jede k-te Spalte des Bodenmodells (160 Spalten)
    l_free: float = -0.4               # Log-Odds je Beobachtung
    l_occ: float = 0.5                 # zwei Treffer bis "belegt" (l_show): ein Ausreißer sperrt nichts
    free_margin_m: float = 0.10        # Freistrahl endet so weit vor der Grenze (Messrauschen)
    free_margin_quad: float = 0.05     # dazu so viel x Entfernung² (der Fehler wächst quadratisch)
    free_range_m: float = 1.5          # Freistrahlen bis hierhin (Treffer nur bis max_range_m); 0 = wie max_range_m
    driven_free_r_m: float = 0.10      # gefahrener Weg gilt so breit als frei; 0 = aus
    max_turn_dps: float = 40.0         # bei schnellerer Drehung nichts eintragen (Unschärfe, Pose)
    occ_free_factor: float = 0.5       # schon belegte Zellen baut ein Freistrahl nur so stark ab
    l_min: float = -3.0
    l_max: float = 3.5
    l_show: float = 0.7                # ab hier "belegt"/"frei" in Anzeige und Statistik
    view_m: float = 6.0                # Bildausschnitt für die Anzeige
    # Drehung der Kartenpose: "räder" (Rad-Odometrie, sonst Kamera), "kamera" (Ego-Motion) oder "imu" (Firmware,
    # sonst wie "räder"). RiderPi 2026-11-06, 32 m: mit Rädern 75 cm Fehler (Schlupf beim Drehen), mit IMU ~6 cm.
    yaw_source: str = "räder"
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
    unknown_cost: float = 3.0          # unbekannte Zellen kosten so viel mehr als freie
    margin_m: float = 1.5              # Suchgebiet = Pose, Start, gefahrener Weg + Rand
    lookahead_m: float = 0.5
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


@dataclass
class ExploreCfg:
    # Modus explore: Frontiers der Raumkarte anfahren (frontier.py), bis nichts Unbekanntes mehr
    # erreichbar ist. Weg und Lenkung wie home ([home]).
    scan_w: float = 0.07               # Rundumblick am Anfang (langsam: unter grid.max_turn_dps)
    min_cluster_m: float = 0.4         # kleinere Frontier-Gruppen zählen nicht
    gain_radius_m: float = 1.2         # Nutzen: unbekannte Fläche in diesem Umkreis des Ziels
    travel_mps: float = 0.27           # Fahrzeit-Schätzung
    turn_dps: float = 40.0             # Drehzeit-Schätzung
    base_s: float = 3.0                # feste Zeit je Ziel: sonst gewinnt immer das Nächste
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
class CpuCfg:
    # CPU-Pinning (Linux). Leer = nicht pinnen. Am RiderPi: Hauptschleife samt Kamera,
    # OpenCV-Workern und Rest des Systems auf 0-1, YOLO auf 2-3 (Aufrufer + Pool-Thread).
    main_cpus: list = field(default_factory=list)
    detector_cpus: list = field(default_factory=list)
    # Threads, die Bilder für die Oberfläche zeichnen und kodieren (WebSocket, /frame, /stream).
    # Leer = erben main_cpus. Am RiderPi alle Kerne: neben YOLO ist auf 2-3 noch Luft, und
    # auf 0-1 verdrängten sie sonst die Hauptschleife (p95 39 -> 47 ms gemessen).
    render_cpus: list = field(default_factory=list)
    # Bodenmodell-Thread ([freespace] threaded). Leer = erben main_cpus.
    freespace_cpus: list = field(default_factory=list)


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
    home: HomeCfg = field(default_factory=HomeCfg)
    explore: ExploreCfg = field(default_factory=ExploreCfg)
    patrol: PatrolCfg = field(default_factory=PatrolCfg)
    control_hz: float = 20.0

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
