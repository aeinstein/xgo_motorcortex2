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
    # Rückwärts setzt die Firmware schwächer um (am Rider: 2 s mit u8 109 ~4 cm, mit 147
    # vorwärts 86 cm). Negative v werden beim Senden damit multipliziert; max_v begrenzt
    # davor den Sollwert. 1.0 = symmetrisch.
    reverse_gain: float = 1.0
    accel_v: float = 0.8           # max. Änderung von v pro Sekunde (normiert)
    accel_w: float = 2.0
    keepalive_s: float = 0.3       # Sollwert periodisch neu schreiben
    stall_timeout_s: float = 0.5   # kein drive() mehr -> Stopp (Userspace-Watchdog)
    kernel_watchdog_ms: int = 1000 # nur wirksam mit Kernel-Patch (settings/motion_timeout_ms)
    height_min_mm: float = 60.0
    height_max_mm: float = 120.0


@dataclass
class CameraCfg:
    source: str = "picam"          # "picam", Gerätenummer "0" oder Videodatei
    width: int = 640
    height: int = 480
    fps: int = 30
    exposure_us: int = 4000        # kurze Belichtung gegen Bewegungsunschärfe; 0 = Auto
    hfov_deg: float = 62.2         # OV5647 Standardoptik (RPi Cam v1)
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


@dataclass
class DetectorCfg:
    enabled: bool = True
    model: str = "models/yolo11n_256.onnx"
    input_size: int = 256
    conf: float = 0.35
    iou: float = 0.45
    threads: int = 2               # CM4 gemessen: 3 Threads verdrängen die Hauptschleife (>33 ms)
    sharpness_rel: float = 0.6     # Frame nur nehmen, wenn Schärfe >= rel * Median
    max_wait_s: float = 0.25       # spätestens dann auch unscharfe Frames nehmen
    classes: list[str] = field(default_factory=list)  # leer = alle COCO-Klassen


@dataclass
class TrackerCfg:
    iou_match: float = 0.25
    max_lost_s: float = 1.5
    min_hits: int = 2


@dataclass
class FreespaceCfg:
    enabled: bool = True
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
    # Linsenabschattung (tools/calibrate.py shading -> JSON). "" = aus (Simulator).
    # Pixel, die mehr als shading_max_gain Verstärkung bräuchten, gelten als unzuverlässig
    # und nie als Hindernis (RiderPi: untere Ecken, äußere Ränder).
    shading_file: str = ""
    shading_max_gain: float = 8.0


@dataclass
class MapCfg:
    max_age_s: float = 4.0         # Kartenpunkte so lange behalten
    # Ein Punkt blockiert die Bahn erst, wenn in seiner Nähe (confirm_radius_m) Punkte aus
    # mindestens confirm Beobachtungen liegen. 1 = jede Einzelbeobachtung zählt (wie bisher).
    confirm: int = 1
    confirm_radius_m: float = 0.08


@dataclass
class BehaviorCfg:
    mode: str = "idle"             # idle | follow | explore | approach
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
