"""Hauptschleife: Kamera -> Ego-Motion -> Tracker/Detektor -> Freiraum -> Verhalten -> Robot."""
from __future__ import annotations

import collections
import copy
import gc
import logging
import math
import os
import re
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .behaviors import Behavior, Command, World, make_behavior, safety
from .camera import FrameSource, OpenCVSource, make_source
from .config import Config
from .detector import Detection, DetectorWorker
from .egomotion import EgoMotion, EgoResult, warp_box
from .fahrrahmen import KURSABS_VORAUS_DEG
from .floornet import FloorNet
from .gesicht import Gesichter
from .freespace import FreeSpace, FreeSpaceEstimator, FreeSpaceWorker, column_runs
from .gridmap import GridMap, GridWorker
from .mathe import to_world, w_to_yaw_rate, wrap
from .planner import PlannerWorker
from .geometry import CamGeometry
from .wheels import WheelOdom, make_wheels
from .odomfusion import OdomFusion
from . import startbild as sbm
from .imu import ImuFrame, ImuSource, ImuSync, imu_frame, make_imu, max_age as imu_max_age
from .localmap import LocalMap, mask_movable
from .route import arc_hits_zone, check as route_check, load as route_load, save as route_save
from .robot import FW_ACTIONS, Robot, action_cmds, stopp_text
from .tracker import Tracker
from .wlan import Helfer, HoldDetector, PressDetector, WlanSetup
from .dunkel import Dunkelangst, sperr_rechteck
from .innen import Ladewaechter
from .verstaerkung import GainSwitch, bildwerte

log = logging.getLogger(__name__)


@dataclass
class Snapshot:
    """Zustand für Anzeige/Web, wird pro Frame ersetzt (nicht verändert)."""
    frame: np.ndarray | None = None
    idx: int = 0
    tracks: list = field(default_factory=list)
    free: FreeSpace | None = None
    ego: EgoResult | None = None
    cmd: Command = field(default_factory=Command)
    sent: tuple[float, float] = (0.0, 0.0)
    stats: dict = field(default_factory=dict)
    t: float = 0.0                 # Aufnahmezeit des Frames (monoton, s)
    pitch_dev: float = 0.0         # Nickabweichung rad, wie in der Geometrie verwendet
    tracks_all: list = field(default_factory=list)   # alle Tracks inkl. unbestätigter (Diagnose)
    cam_meta: dict | None = None                     # Weißabgleich, Verstärkung
    imu: ImuFrame | None = None                      # IMU zur Bildzeit (eine Wahrheit je Bild); None ohne IMU
    nickrate: tuple[float, float] | None = None      # Nickrate aus Bild / IMU (°/s) zwischen den letzten Bildern
    odo: tuple | None = None                         # Wegsumme der Merkkarte zu diesem Bild (LocalMap.odo)
    geo: object = None                               # CamGeometry (Verweis), für das Umrechnen der Netzlinie


class ModeNotAllowed(ValueError):
    """Modus existiert, ist mit der aktuellen Konfiguration aber nicht sicher."""



_GC_MS = [0.0]          # Summe der GC-Pausen (ms), von gc.callbacks hochgezählt
_GC_T0 = [0.0]


def _gc_cb(phase, info):
    if phase == "start":
        _GC_T0[0] = time.perf_counter()
    else:
        _GC_MS[0] += (time.perf_counter() - _GC_T0[0]) * 1000


gc.callbacks.append(_gc_cb)


class _Marks:
    """Zeitmarken eines Schleifendurchlaufs: Wanduhr und CPU-Zeit des Threads je Abschnitt."""
    __slots__ = ("t", "c", "t0", "c0", "gc0", "seg", "fs_ran")

    def __init__(self):
        self.t0 = self.t = time.perf_counter()
        self.c0 = self.c = time.thread_time()
        self.gc0 = _GC_MS[0]
        self.seg = []
        self.fs_ran = False

    def mark(self, name: str) -> None:
        t, c = time.perf_counter(), time.thread_time()
        self.seg.append((name, (t - self.t) * 1000, (c - self.c) * 1000))
        self.t, self.c = t, c

    def done(self) -> dict:
        return {"seg": self.seg, "wall": (self.t - self.t0) * 1000, "cpu": (self.c - self.c0) * 1000,
                "gc": _GC_MS[0] - self.gc0, "fs_ran": self.fs_ran}

# Bahnprüfung: zusätzlicher Bremsabstand je m/s über ARC_FAST_MPS (siehe App._arc_check)
ARC_FAST_MPS, ARC_FAST_K = 0.3, 0.8
# Bahnprüfung mit Radgeschwindigkeit: Fenster (s) und wie viel schneller (m/s) als erlaubt er sein darf, bevor sie
# voll bremst (Fahrwert 0). Kleinere Fahrwerte setzt der Rider erst nach ~1-2 s um, 0 bremst er in ~0,3 s
# (Fahrt 2026-11-23 vor dem Stuhl: Fahrwert binnen 1 s von 0,33 auf 0,08, die Räder blieben bei 0,5 m/s).
ARC_IST_WINDOW_S, ARC_BRAKE_TOL_MPS = 0.2, 0.05


class App:
    def __init__(self, cfg: Config, robot: Robot, source: FrameSource | None = None,
                 imu: ImuSource | None = None, wheels: WheelOdom | None = None, floor_net=None):
        """floor_net: Bodengrenze statt FloorNet aus [freespace] net (Simulator: sim.SimFloorNet)."""
        self.cfg = cfg
        self.imu = imu if imu is not None else make_imu(cfg.imu)
        self.wheels = wheels if wheels is not None else make_wheels(cfg.odom)
        # Koppelnavigation: ein Kurs und eine Strecke je Regelschritt für Verhalten, Merkkarte und Raumkarte
        self.odomf = OdomFusion(cfg.odom)
        self.odom_src = "befehl"
        self._vis_gap = False              # seit dem letzten Regelschritt ein Bild ohne Ego-Motion
        self.odom_m = 0.0                  # gefahrene Strecke laut Odometrie (Betrag, seit Start)
        # vorzeichenbehaftet (vor +, zurück −): Für Strecken; odom_m zählt das Pendeln beim Balancieren mit (am Rider
        # 2026-11-25 0,99 statt 0,93 m auf 1 m) und taugt nur als Laufleistung
        self.odom_s_m = 0.0
        self._strecke_min = 0.0            # kleinste geschickte Reststrecke der laufenden Handfahrt mit Strecke
        self._v_rad: float | None = None       # Radgeschwindigkeit (m/s) im letzten Regelschritt, für die Bahnprüfung
        self.imu_sync = ImuSync(cfg.imu)
        self._imu_frame: ImuFrame | None = None   # IMU zur Bildzeit, je Bild (imu.imu_frame)
        self._nickrate: tuple[float, float] | None = None
        self.pitch_src = "bild"
        self.last_pitch_dev = 0.0
        self.robot = robot
        if len(cfg.camera.cam_h_u8) == 2:
            robot.cam_h_u8 = tuple(cfg.camera.cam_h_u8)   # Pursuit rechnet seine Mindesthöhe als Kamerahöhe
        self.src = source or make_source(cfg.camera)
        size = (cfg.camera.width, cfg.camera.height)
        self.hfov = math.radians(cfg.camera.hfov_deg)
        self.ego = EgoMotion(cfg.ego, size, cfg.camera.hfov_deg)
        self.tracker = Tracker(cfg.tracker, size)
        self.det = DetectorWorker(cfg.detector, cfg.cpu.detector_cpus)
        self.geo = CamGeometry.from_cfg(cfg)
        self.fse = FreeSpaceEstimator(cfg.freespace, size, self.geo) if cfg.freespace.enabled else None
        self.fs_worker = None
        self.fn_geladen = None             # Datei des geladenen FloorNet (Web-Auswahl), None = keins oder Simulator
        self._fn_lock = threading.Lock()
        if self.fse is not None:
            net = floor_net
            if net is None and cfg.freespace.net:
                net = FloorNet.load(cfg.freespace.net, cfg.freespace.net_outlier_px, cfg.freespace.net_outlier_ratio)
                if net is not None:
                    self.fn_geladen = os.path.basename(os.path.realpath(cfg.freespace.net))
            if net is None:
                log.error("Kein FloorNet ([freespace] net = %r): blind, der Rider fährt nicht selbst", cfg.freespace.net)
            elif cfg.freespace.net_smooth_n > 1:
                log.info("[floornet] Kante über %d Netzbilder beruhigt (net_smooth_n)", cfg.freespace.net_smooth_n)
            self.fs_worker = FreeSpaceWorker(self.fse, net, cfg.cpu.freespace_cpus, cfg.freespace.net_smooth_n)
        m = cfg.map
        self.lmap = LocalMap(self.geo, self.fse.W, self.fse.H, max_age_s=m.max_age_s, confirm=m.confirm,
                             confirm_radius_m=m.confirm_radius_m, radius_m=m.radius_m, native=m.native) \
            if self.fse else None
        if self.lmap is not None and self.lmap.backend == "c++":
            log.info("Merkkarte, Planer, Frontier-Suche: C++-Kern (navfast)")
        self._fs_in_map = None
        # Raumkarte (Welt-KS ab Startpose), nur Anzeige
        self.gridmap = GridMap(cfg.grid, self.geo, self.fse.W, self.fse.H) \
            if self.fse is not None and cfg.grid.enabled else None
        self.pose_grund = ""             # warum die Pose in der Raumkarte unbekannt ist ("" = gültig)
        self._pose_bal: bool | None = None
        self._pose_wheels_t = 0.0
        if self.gridmap is not None and cfg.grid.file and os.path.exists(cfg.grid.file):
            if self.gridmap.load(cfg.grid.file) and cfg.reloc.enabled:
                # Wo der Rider beim Beenden stand, ist die beste Vermutung; ob er noch dort steht, weiß niemand
                if self.gridmap.saved_pose is not None:
                    self.gridmap.set_pose(self.gridmap.saved_pose)
                self._pose_unbekannt("Neustart: Standort nicht bestätigt")
            route_load(cfg.grid.file, self.gridmap)
        self.grid_worker = GridWorker(self.gridmap, cfg.cpu.freespace_cpus, save_path=cfg.grid.file,
                                      save_every_s=cfg.grid.save_every_s) \
            if self.gridmap is not None and cfg.grid.threaded else None
        # Laufender Kartenabgleich ([reloc] track): rechnet dort, wo die Karte beschrieben wird
        self.reloc_track = None
        if self.gridmap is not None and cfg.reloc.track:
            from .reloc import Nachfuehrung
            self.reloc_track = Nachfuehrung(cfg.reloc, cfg.grid.l_show)
            if self.grid_worker is not None:
                self.grid_worker.track = self.reloc_track
            log.info("Kartenabgleich: laufend, alle %.0f s nach %.1f m oder %.0f°", cfg.reloc.every_s,
                     cfg.reloc.min_move_m, cfg.reloc.min_turn_deg)
        # Planer für den Modus home (Thread startet erst beim ersten Auftrag)
        # Abgleich am Startbild: nur mit Raumkarte; rechnet in eigenem Faden (startbild.py)
        self.startbild = sbm.StartAbgleich(cfg.startbild, cfg.cpu.render_cpus) \
            if cfg.startbild.enabled and self.gridmap is not None else None
        self._sb_tiefen: list = []          # Graubilder der geraden Schlussfahrt (home)
        # Startpunkt = Pose beim Aufspringen (sobald er danach still steht) oder beim Absetzen (Nutzer 2026-11-22)
        self._start_bal: bool | None = None
        self._start_t = 0.0
        self._start_due: float | None = None
        self._start_still: tuple | None = None
        self._sb_t_phase = None
        self.planner = PlannerWorker(self.gridmap, cfg.home, cfg.cpu.freespace_cpus) \
            if self.gridmap is not None else None
        self._plan_seq = 0
        self._yaw_at_ctrl = 0.0
        self._kurs_imu_ref: float | None = None    # IMU-Kurs (rad) zur Bildzeit des letzten Regelschritts: Bezug von self.yaw
        mode = cfg.behavior.mode
        try:
            self._check_mode(mode)
        except ModeNotAllowed as e:
            log.error("%s – starte stattdessen im Modus idle", e)
            mode = "idle"
        self.behavior: Behavior = self._make(mode)
        self._mode_lock = threading.Lock()
        self._pending_mode: tuple[str, str | None] | None = None
        self.snapshot = Snapshot()
        self.snap_cond = threading.Condition()
        self.running = False
        self.vis_yaw = 0.0               # Kurs nur aus dem Bild (rad, links +)
        self.yaw = 0.0                   # Kurs der Koppelnavigation (IMU, sonst Räder, sonst Bild)
        self.yaw_src = "kamera"
        self._free: FreeSpace | None = None
        self._fs_hold_until = 0.0
        self._was_balancing: bool | None = None
        self._free_ctrl: FreeSpace | None = None
        self._last_ctrl = 0.0
        self._last_height_t = 0.0
        self._cmd = Command()
        self._det_count = 0
        self._loop_ms = 0.0
        self._loop_hist = collections.deque(maxlen=250)   # letzte Schleifenzeiten (ms): p95/max
        self._seg_hist = collections.deque(maxlen=250)    # je Bild: Abschnittszeiten, siehe loop_breakdown()
        self.emotions = None           # emotion.Emotions, von __main__ gesetzt (nur am Rider)
        self.uhr = None                # () -> Ortszeit in Stunden, von __main__ gesetzt (nur am Rider; Innenleben)
        # Ruhemodus, in den pet wegen des Akkus ging (innen.Ladewaechter): Laden erkennen, geladen aufwachen
        self.lade = Ladewaechter(self.cfg.pet)
        self.rawrec = None             # rawrec.RawRecorder, von __main__ gesetzt
        self._ruhe_saved: dict[str, str] | None = None   # Ruhemodus: alte Treiber-Einstellungen, sonst None
        # WLAN-Einrichtung über die Tasten (wlan.py): läuft in der Hauptschleife statt _step
        self.wlan: WlanSetup | None = None
        self._wlan_req: bool | None = None            # vom Web (POST /wlan), in der Hauptschleife übernommen
        self._wlan_hold = HoldDetector(cfg.wlan.buttons, cfg.wlan.hold_s)
        self._wlan_cancel = PressDetector(cfg.wlan.cancel_buttons)
        self._wlan_pipeline = False                   # lief die Bildverarbeitung vorher?
        # Dunkelangst (dunkel.py): Helligkeit je check_hz, im Regelschritt ausgewertet
        self.dunkel = Dunkelangst(cfg.dunkel, cfg.robot)
        self._luma: float | None = None
        self._dunkel_zonen: list[tuple[float, list]] = []      # (Ablaufzeit, Rechteck): Dunkelzonen der Raumkarte
        self._luma_t = 0.0
        self.gain_switch = GainSwitch(cfg.camera)                # Verstärkung: hell -> Automatik, dunkel -> fest
        self._gain_fest = float(cfg.camera.analogue_gain) or 16.0
        self.ortung = None             # ortung.Ortung (Schallrichtung), von __main__ gesetzt (nur am Rider)
        self.stimmen = None            # stimme.Stimmen (Nachplappern in pet), hängt an der Ortung
        self.gesichter: Gesichter | None = None   # Gesichter wiedererkennen (gesicht.py), in run() geladen
        # Fahr-Aktion der Firmware (robot.FW_ACTIONS): vom Web angefragt, im Regelschritt geprüft und gestartet
        self._action_pending: int | None = None
        self._action_note: tuple[str, float] | None = None   # (Meldung, anzeigen bis) für die Statuszeile

    # ------------------------------------------------------------------
    # Diese Modi fahren auf Freiraum zu und verlassen sich auf Bodenmodell + Merkkarte.
    # Ohne Freiraum-Erkennung hielten sie alles für frei und stoppten nur per TTC.
    NEEDS_FREESPACE = {"explore", "approach", "home", "patrol"}

    # Diese Modi fahren nach der Raumkarte: Mit unbekannter Pose führen sie irgendwohin.
    NEEDS_POSE = {"home", "patrol", "explore"}

    # Nur diese Modi bekommen Bilder für den Detektor (YOLO, ~1 Kern am CM4); sonst schläft sein Thread.
    # Erkennungen dienen nur dem Ziel, nie Freiraum oder Bahnprüfung (Nutzer 2026-11-13).
    DET_MODES = {"follow", "approach", "pet"}
    # Liegt er (abgesetzt, gefallen), enden alle Modi außer diesen sofort (idle). pet legt sich selbst hin und steht
    # wieder auf, Hand kann ihn aufstellen (Nutzer 2026-11-24: explore sperrte liegend Lücken und trug die
    # Merkkarte in die Raumkarte ein)
    LIEGEND_OK = {"idle", "manual", "pet"}
    TARGET_MODES = {"follow", "approach"}

    def _pose_unbekannt(self, grund: str) -> None:
        """Die Pose in der Raumkarte gilt nicht mehr (getragen, gefallen, Neustart). Die Karte wird nicht mehr
        beschrieben, bis der Standort bestätigt ist; Fahrten nach der Karte enden. Eine leere Karte legt ihr
        KS selbst fest, dort gibt es nichts zu verlieren. Nur mit [reloc] enabled."""
        gm = self.gridmap
        if not self.cfg.reloc.enabled or gm is None or not gm.pose_ok or gm.empty():
            return
        gm.set_pose_ok(False)
        self.pose_grund = grund
        if getattr(self, "startbild", None) is not None:     # ohne Pose kein Startpunkt: Startbild weg (auch im Web)
            self.startbild.anker_weg(grund)
        log.warning("Pose in der Raumkarte unbekannt (%s): Karte wird nicht mehr beschrieben", grund)
        b = getattr(self, "behavior", None)
        if b is not None and b.name in self.NEEDS_POSE:
            log.warning("Modus %s endet: Pose unbekannt", b.name)
            with self._mode_lock:
                self._pending_mode = ("idle", None)

    def _pose_watch(self, now: float) -> None:
        """Ereignisse, nach denen die Pose nicht mehr stimmt: abgesetzt oder gefallen (beim Aufstehen rollt er
        vor, liegend zählen die Räder nicht, und er kann getragen worden sein); balancierend länger als
        reloc.blind_s ohne frische Raddaten. Kosten: zwei Vergleiche je Regelschritt."""
        if not self.cfg.reloc.enabled or self.gridmap is None:
            return
        bal = self.robot.state.balancing
        if self._pose_bal and not bal:
            self._pose_unbekannt("abgesetzt oder gefallen")
        if bal and not self._pose_bal:
            self._pose_wheels_t = now
        self._pose_bal = bal
        w = self.wheels
        if bal and w is not None and self.cfg.odom.source != "command":
            if w.fresh(now):
                self._pose_wheels_t = now
            elif now - self._pose_wheels_t > self.cfg.reloc.blind_s:
                self._pose_unbekannt("keine Raddaten")

    def set_pose_state(self, aktion: str) -> None:
        """Web: "start" = der Rider steht am Startpunkt (Pose springt dorthin), "hier" = die angezeigte Pose
        stimmt, "verwerfen" = sie stimmt nicht mehr."""
        gm = self.gridmap
        if gm is None:
            raise ModeNotAllowed("keine Raumkarte")
        if aktion == "verwerfen":
            if not self.cfg.reloc.enabled:
                raise ModeNotAllowed("[reloc] enabled ist aus")
            self._pose_unbekannt("vom Nutzer verworfen")
            return
        if aktion not in ("start", "hier"):
            raise KeyError(aktion)
        if self.behavior.name in self.NEEDS_POSE:
            raise ModeNotAllowed("nicht während der Fahrt nach der Karte")
        if not self.robot.state.balancing:
            raise ModeNotAllowed("erst aufstehen: beim Aufstehen rollt er noch vor")
        if aktion == "start":
            gm.relocate_to_start()
        else:
            gm.set_pose_ok(True)
        self.pose_grund = ""
        log.info("Pose in der Raumkarte bestätigt (%s)", "am Startpunkt" if aktion == "start" else "wie angezeigt")

    def _check_mode(self, name: str, confirmed: bool = False, target: str | None = None) -> None:
        if name in self.NEEDS_FREESPACE and (self.fs_worker is None or self.fs_worker.blind):
            raise ModeNotAllowed(f"Modus {name} braucht die Bodengrenze (FloorNet), die fehlt oder ist ausgefallen")
        if name in self.TARGET_MODES:
            # Ohne Detektor oder mit einer Klasse, die er nicht meldet, suchte follow ewig bzw. erkundete approach ewig
            avail = self.det.classes() if hasattr(self.det, "classes") else []
            cls = target or self.cfg.behavior.target_class
            if not avail:
                raise ModeNotAllowed(f"Modus {name} braucht die Objekterkennung: YOLO ist aus")
            if cls not in avail:
                raise ModeNotAllowed(f"Ziel {cls!r} erkennt YOLO nicht (aktiv: {', '.join(avail)})")
        if name == "home" and getattr(self, "gridmap", None) is None:
            raise ModeNotAllowed("Modus home braucht die Raumkarte ([grid] enabled)")
        gm = getattr(self, "gridmap", None)
        # Streife mit Bestätigung setzt die Pose selbst auf den Startpunkt
        if name in self.NEEDS_POSE and gm is not None and not gm.pose_ok and not (name == "patrol" and confirmed):
            raise ModeNotAllowed("Pose in der Karte unbekannt: erst den Standort bestätigen (Raumkarte)")
        if name == "explore" and gm is not None and gm.locked:
            raise ModeNotAllowed("Karte ist gesperrt: erst entsperren oder neu anfangen")
        if name == "patrol":
            if gm is None:
                raise ModeNotAllowed("Streife braucht die Raumkarte ([grid] enabled)")
            if not gm.route:
                raise ModeNotAllowed("Streife braucht eine Route: Punkte auf der Karte setzen")
            batt = self.robot.state.battery
            if batt and batt < self.cfg.patrol.start_battery:
                raise ModeNotAllowed(f"Akku {batt} %: Streife erst ab {self.cfg.patrol.start_battery} %")
            if not self.robot.state.balancing:
                raise ModeNotAllowed("Streife: Rider muss stehen, er rollt beim Aufstehen noch vor")
        if name not in self.LIEGEND_OK and not self.robot.state.balancing:
            raise ModeNotAllowed(f"Modus {name}: Rider muss stehen (liegt oder gefallen), erst aufstehen")

    def _make(self, name: str, target: str | None = None) -> Behavior:
        b = make_behavior(name, self.cfg.behavior, self.cfg.robot, target, self.cfg.home, self.cfg.explore,
                          has_map=getattr(self, "gridmap", None) is not None, pcfg=self.cfg.patrol,
                          petcfg=self.cfg.pet, sbcfg=self.cfg.startbild if getattr(self, "startbild", None) else None)
        if name == "patrol" and getattr(self, "gridmap", None) is not None:
            b.set_route(self.gridmap.route, self.gridmap.route_loop)
        if name == "pet":                                    # Begrüßung je Person (Web); Gesichter erst in run()
            b.brain.gruss_fuer = lambda n: self.gesichter.gruss(n) if self.gesichter is not None else {}
        return b

    def set_route(self, points, loop: bool, zones=None) -> list[str]:
        """Route und, falls angegeben, Sperrzonen setzen und speichern. Rückgabe: Fehler; leer = übernommen.
        Nicht während der Streife: Der laufende Modus fährt seine Kopie weiter."""
        gm = self.gridmap
        z = gm.zones if zones is None else zones
        errors = route_check(gm, self.cfg.home, points, z)
        if errors:
            return errors
        with gm._lock:
            gm.route, gm.route_loop, gm.zones = list(points), bool(loop), list(z)
        route_save(self.cfg.grid.file, gm)
        if self.cfg.grid.file:
            gm.save(self.cfg.grid.file)               # map_id muss auf der Platte stehen, sonst passt die Route nicht
        return []

    @property
    def pipeline_on(self) -> bool:
        return not self.src.paused

    def set_pipeline(self, on: bool) -> None:
        """Bildverarbeitung starten/stoppen. Beim Stoppen: Stillstand und Modus idle;
        nach dem Starten bleibt der Rider stehen, bis ein Modus gewählt wird."""
        if on:
            if self.ruhe:
                raise ModeNotAllowed("Ruhemodus: erst wecken")
            self.src.resume()
            self.reset_floor(hold_s=self.cfg.freespace.restart_hold_s)
            log.info("Bildverarbeitung gestartet")
        else:
            with self._mode_lock:
                self._pending_mode = ("idle", None)
            self.src.pause()
            self.robot.stop()
            self._action_stop(keep_turn=True)         # ohne Bild keine Bahnprüfung: fahrende Aktion anhalten
            log.info("Bildverarbeitung gestoppt")

    # Treiber-Einstellungen, die der Ruhemodus abschaltet: Wert im Ruhemodus (None = aus RuheCfg)
    RUHE_SETTINGS = {"imu_hz": 0, "imu_stream_hz": 0, "sleep_ms_on_loop": None}

    @property
    def ruhe(self) -> bool:
        return self._ruhe_saved is not None

    def set_ruhe(self, on: bool) -> None:
        """Ruhemodus: absetzen, Bildverarbeitung aus, Treiber fragt nichts mehr ab, alle Threads wachen nur noch
        selten auf. Wecken stellt nur die Abfragen wieder her; Bildverarbeitung bleibt aus, er bleibt liegen.
        Den CPU-Takt schaltet deploy/xgo-ruhe (root), die Schlafaugen am Display spielt api.php."""
        if on == self.ruhe:
            return
        if on:
            self.set_pipeline(False)
            if self.robot.state.balancing:
                self._pose_unbekannt("abgesetzt (Ruhemodus)")
                self.robot.set_down()
            if self.rawrec is not None:
                try:
                    self.rawrec.schalten(False)
                except Exception as e:
                    log.warning("Ruhe: Aufzeichnung nicht gestoppt: %s", e)
            saved = {}
            for name, val in self.RUHE_SETTINGS.items():
                old = self.robot.read_setting(name)
                if old is None or not old.lstrip("-").isdigit():
                    continue                      # Treiber kennt die Einstellung nicht
                saved[name] = old
                self.robot.write_setting(name, self.cfg.ruhe.sleep_ms_on_loop if val is None else val)
            self._ruhe_saved = saved
            for src in (self.imu, self.wheels):
                if src is not None:
                    src.paused = True
            self.robot.ruhe_tick_s = self.cfg.ruhe.tick_s
            log.info("Ruhemodus an (Treiber: %s)", ", ".join(f"{k} {v}->aus" for k, v in saved.items()) or "-")
        else:
            self.lade.vergessen()                 # geweckt (Web, MCP, Ladewächter): nicht von selbst noch einmal
            saved, self._ruhe_saved = self._ruhe_saved, None
            self.robot.ruhe_tick_s = None
            for name, old in saved.items():
                self.robot.write_setting(name, old)
            for src in (self.imu, self.wheels):
                if src is not None:
                    src.paused = False
            self._imu_idle_tick(force=True)       # die zurückgeschriebene Rate passt evtl. nicht mehr zum Zustand
            log.info("Ruhemodus aus, Bildverarbeitung bleibt aus")

    def _imu_idle_tick(self, force: bool = False) -> None:
        """Abgesetzt: IMU-Strom auf [imu] idle_hz, Räder nicht lesen; balancierend volle Rate. Im Ruhemodus ist
        beides ganz aus."""
        if self.ruhe:
            return
        idle = not self.robot.state.balancing
        if self.imu is not None:
            self.imu.set_idle(idle, force)
        if self.wheels is not None:
            self.wheels.set_idle(idle)

    def _ortung_tick(self) -> None:
        """Je Schleifendurchlauf: Schallortung nur in [ortung] modes, nie im Ruhemodus oder bei der
        WLAN-Einrichtung. Ein Vergleich; an- und ausgeschaltet wird nur beim Wechsel."""
        o = self.ortung
        if o is not None:
            an = not self.ruhe and self.wlan is None and self.behavior.name in self.cfg.ortung.modes
            o.set_active(an)
            if self.stimmen is not None:              # Stimmproben nur in pet, solange jemand zu sehen ist
                b = self.behavior
                self.stimmen.aufnahme = (an and b.name == "pet" and self.cfg.pet.parrot
                                         and getattr(getattr(b, "brain", None), "hoert_zu", False))

    def _gain_tick(self, f, hell: float) -> None:
        """Verstärkung umschalten (verstaerkung.py); Fehler nur ins Log, die Bildschleife läuft weiter."""
        try:
            ist = (f.meta or {}).get("gain")
            g = self.gain_switch.update(f.t, self._luma, hell, ist, self.src.gain, self._gain_fest)
            if g is not None:
                log.info("Verstärkung: Bild %s (Mittel %.0f, ausgefressen %.1f %%, ist %s) -> %s",
                         "zu dunkel" if g else "zu hell", self._luma, 100 * hell, ist, g or "Automatik")
                self.src.set_gain(g)
        except Exception as e:
            log.error("Verstärkung umschalten fehlgeschlagen: %s", e)

    # ------------------------------------------------------------------ Dunkelangst (dunkel.py)
    def _dunkel_tick(self, now: float, cmd: Command) -> Command:
        """Im Regelschritt nach safety(): Ist es zu dunkel, übernimmt die Dunkelangst (auf der Stelle drehen)."""
        d = self.dunkel
        if self._dunkel_zonen:
            if self.gridmap is None or not self.gridmap.dark_zones:     # Karte neu oder geladen: Zonen sind weg
                self._dunkel_zonen = []
            elif now >= self._dunkel_zonen[0][0]:                       # älteste zuerst: ein Vergleich je Schritt
                self._dunkel_zonen_setzen([z for z in self._dunkel_zonen if now < z[0]], "abgelaufen")
        if (not self.cfg.dunkel.enabled or self.behavior.name not in self.cfg.dunkel.modes
                or not self.robot.state.balancing):
            d.reset()
            return cmd
        dc = d.update(now, self._luma, self.yaw)
        for art, name in d.drain():
            if art == "ruhe":
                self._dunkel_ruhe()
            elif art == "zone":
                self._dunkel_zone(now)
            elif self.emotions is not None:
                try:
                    self.emotions.express(art, name, urgent=True)
                except Exception as e:                # Anzeige darf die Regelung nie stören
                    log.warning("Dunkelangst: %s %s: %s", art, name, e)
        if dc is None:
            return cmd
        return Command(0.0, dc.w, None, dc.status)

    def _dunkel_zone(self, now: float) -> None:
        """Angst: Rechteck voraus zone_s lang sperren und das Verhalten neu planen lassen, sonst führe der Planer
        nach dem Umdrehen wieder ins Dunkle. Kurs aus der Raumkarte (nicht IMU): Die Zone liegt im Karten-KS."""
        c, gm = self.cfg.dunkel, self.gridmap
        if c.zone_s <= 0 or gm is None or self.behavior.name not in c.zone_modes:
            return
        zone = sperr_rechteck(*gm.pose(), c)
        self.dunkel.n_zonen += 1
        self._dunkel_zonen_setzen(self._dunkel_zonen + [(now + c.zone_s, zone)],
                                  "neu für %.0f s: %s" % (c.zone_s, " ".join("(%.2f, %.2f)" % p for p in zone)))
        neu = getattr(self.behavior, "neu_planen", None)
        if neu is not None:
            neu()

    def _dunkel_zonen_setzen(self, zonen: list, grund: str) -> None:
        """Liste ersetzen (nie ändern): Der Planer-Thread liest gm.dark_zones ohne Sperre."""
        self._dunkel_zonen = zonen
        if self.gridmap is not None:
            self.gridmap.dark_zones = [z for _, z in zonen]
        log.warning("Dunkelzone %s, jetzt %d", grund, len(zonen))

    def _dunkel_ruhe(self) -> None:
        """Kein Licht gefunden: absetzen und Ruhemodus wie der Web-Knopf (Schlafaugen, CPU-Takt per xgo-ruhe)."""
        log.warning("Dunkelangst: setzt sich, Ruhemodus")
        self.robot.stop()
        if self.emotions is not None:
            try:
                self.emotions.play("sleep")
            except Exception as e:
                log.warning("Dunkelangst: Schlafaugen: %s", e)
        self.set_ruhe(True)
        self.dunkel.reset()
        if self._dunkel_zonen:
            self._dunkel_zonen_setzen([], "gelöscht (Ruhemodus)")
        self._ruhe_system(True)

    # ------------------------------------------------------------------ Akku in pet (innen.py)
    def _pet_ruhe(self, grund: str) -> None:
        """pet, Akku erschöpft oder leer: absetzen und Ruhemodus wie die Dunkelangst; der Ladewächter weckt ihn,
        wenn er geladen ist."""
        if self.ruhe:
            return                                    # schon (Dunkelangst im selben Schritt)
        log.warning("pet: %s – setzt sich, Ruhemodus", grund)
        # erst pet verlassen, solange er noch balanciert (Höhe base_h, Roll 0): Im Ruhemodus liest der Roboter den
        # Zustand nur noch selten, _pet_leave schriebe sonst während des Absetzens
        self._pet_leave()
        with self._mode_lock:
            self._pending_mode = None
            self.behavior = self._make("idle")
        self.robot.stop()
        if self.emotions is not None:
            try:
                self.emotions.play("sleep")
            except Exception as e:
                log.warning("pet: Schlafaugen: %s", e)
        self.set_ruhe(True)
        self._ruhe_system(True)
        self.lade.einschlafen()

    def _lade_tick(self, now: float) -> None:
        """Je Schleifendurchlauf (auch im Ruhemodus): Akku glätten; im Ruhemodus von pet Laden erkennen (gurren)
        und geladen aufwachen: Ruhemodus aus, Bildverarbeitung an, pet liegend (steht nie von selbst auf)."""
        for aktion in self.lade.update(now, self.robot.state.battery, self.ruhe):
            if aktion == "gurren":
                log.info("pet: lädt (Akku %.0f %%)", self.lade.akku.wert or 0)
                if self.emotions is not None:
                    self.emotions.express("sound", "gurren", urgent=True)   # nur der Ton, die Schlafaugen bleiben
            elif aktion == "wecken":
                self._pet_wecken()

    def _pet_wecken(self) -> None:
        log.warning("pet: geladen (Akku %.0f %%), wacht auf", self.lade.akku.wert or 0)
        self.set_ruhe(False)
        self._ruhe_system(False)
        if self.emotions is not None:
            try:
                self.emotions.play("aufwachen")
                self.emotions.express("sound", "gaehnen", urgent=True)
            except Exception as e:
                log.warning("pet: Aufwachen: %s", e)
        try:
            self.set_pipeline(True)
            self.set_mode("pet")
        except ValueError as e:                       # ModeNotAllowed
            log.warning("pet: nach dem Laden nicht wieder in pet: %s", e)

    def _ruhe_system(self, on: bool) -> None:
        """CPU-Takt wie api.php (sudo xgo-ruhe on|off), im Hintergrund; fehlt die Regel, nur eine Warnung."""
        if not os.path.exists("/usr/local/sbin/xgo-ruhe"):
            return

        def lauf():
            import subprocess
            try:
                p = subprocess.run(["sudo", "-n", "/usr/local/sbin/xgo-ruhe", "on" if on else "off"],
                                   capture_output=True, text=True, timeout=10)
                if p.returncode != 0:
                    log.warning("xgo-ruhe %s: %s", "on" if on else "off", (p.stdout + p.stderr).strip())
            except (OSError, subprocess.SubprocessError) as e:
                log.warning("xgo-ruhe: %s", e)

        threading.Thread(target=lauf, name="xgo-ruhe", daemon=True).start()

    # ------------------------------------------------------------------ WLAN-Einrichtung (wlan.py)
    def wlan_request(self, on: bool) -> None:
        """Vom Web: WLAN-Einrichtung starten/abbrechen; übernommen in der Hauptschleife."""
        self._wlan_req = on

    def _wlan_buttons(self) -> None:
        """Je Schleifendurchlauf: Tastenwert (liest der robot-Thread ohnehin) vergleichen. Sonst nichts."""
        if not self.cfg.wlan.enabled:
            return
        now = time.monotonic()
        b = self.robot.state.buttons
        req, self._wlan_req = self._wlan_req, None
        if self._wlan_hold.update(b, now):
            req = self.wlan is None              # nochmal halten bricht ab
        if self._wlan_cancel.update(b) and self.wlan is not None:
            req = False
        if req is True and self.wlan is None:
            self._wlan_start(now)
        elif req is False and self.wlan is not None:
            self.wlan.cancel(now)

    def _wlan_start(self, now: float) -> None:
        if self.ruhe:
            self.set_ruhe(False)
            self._ruhe_system(False)             # CPU-Takt wieder hoch (wie Wecken im Web)
        with self._mode_lock:
            self._pending_mode = None
        self._pet_leave()
        self.behavior = self._make("idle")
        self.robot.stop()
        self._cmd = Command(0.0, 0.0, None, "WLAN-Einrichtung")
        self._wlan_pipeline = self.pipeline_on
        if not self._wlan_pipeline:
            self.src.resume()
        self.wlan = WlanSetup(self.cfg.wlan, Helfer(self.cfg.wlan.helper), now)

    def _wlan_tick(self, last: int) -> int:
        """Hauptschleife während der Einrichtung: Bild an wlan.step, Rider steht (Modus idle)."""
        f = self.src.get(last, timeout=0.5) if not self.src.paused else None
        now = time.monotonic()
        self.wlan.step(f.bgr if f is not None else None, now)
        self._cmd = Command(0.0, 0.0, None, "WLAN: " + self.wlan.text)
        if f is not None and isinstance(self.src, OpenCVSource):
            self.src.mark_consumed()
        if f is None:
            time.sleep(0.05)
        if self.wlan.done:
            log.warning("WLAN-Einrichtung beendet: %s %s", self.wlan.text, self.wlan.detail)
            self.wlan = None
            if self._wlan_pipeline:
                self.reset_floor(hold_s=self.cfg.freespace.restart_hold_s)
            else:
                self.src.pause()
        return f.idx if f is not None else last

    def reset_floor(self, hold_s: float = 0.0) -> None:
        """Bodengrenze neu beginnen (Glätten verwerfen); hold_s: so lange vorher keine übernehmen (nach Kamerastart
        sind die ersten Bilder dunkel, nach dem Aufstehen schwankt die Lage). Bis dahin kein Freiraum-Ergebnis."""
        if self.fs_worker is None:
            return
        self._fs_hold_until = time.monotonic() + hold_s
        self._free = None
        self.fs_worker.reset()

    def set_manual(self, v: float, w: float, checked: bool = False, bogen: bool = False,
                   strecke: float | None = None) -> None:
        """Sollwert der Handsteuerung (normiert, v > 0 vor, w > 0 links). Nur im Modus manual;
        geht auch bei gestoppter Bildverarbeitung (braucht keine Kamera).
        checked: dieser Sollwert läuft wie in den anderen Modi durch safety() und die Bahnprüfung
        (/drive?sicher=1, vom MCP-Server genutzt); ohne Bildverarbeitung dreht er dann nur.
        bogen: als Kreisbogen mit κ = w/v (/drive?bogen=1), siehe Manual.set."""
        b = self.behavior
        if b.name != "manual":
            raise ModeNotAllowed("Handsteuerung nur im Modus manual")
        b.set(v, w, time.monotonic(), checked, bogen, strecke)

    TURN_MAX_DEG = 1080.0

    def set_turn(self, deg: float, w: float = 0.5, tol_deg: float = 3.0) -> None:
        """Auf der Stelle um deg drehen (Modus manual, POST /turn): w Anteil von max_w, Zeitgrenze
        (10 s + 1 s je 20°) / tempo. Fortschritt in /status turn."""
        b = self.behavior
        if b.name != "manual":
            raise ModeNotAllowed("Drehen nur im Modus manual")
        if not (abs(deg) <= self.TURN_MAX_DEG and 0.05 <= w <= 1.0 and 1.0 <= tol_deg <= 10.0):
            raise ValueError("Grenzen: deg ±%.0f, w 0.05..1, tol 1..10" % self.TURN_MAX_DEG)
        if not self.robot.state.balancing:
            raise ModeNotAllowed("Drehen nur balancierend")
        timeout = (10.0 + abs(deg) / 20.0) / max(self.robot.tempo, 0.05)
        b.turn(deg, w, tol_deg, time.monotonic(), self.yaw, timeout)

    def set_bahn(self, laenge_m: float = 3.0, winkel_deg: float = 20.0, welle_m: float = 1.5,
                 zurueck: bool = True, v: float | None = None, kurve: bool = True) -> list[tuple[float, float]]:
        """Feste Testbahn (Modus manual, POST /bahn): Schlangenlinie laenge_m geradeaus ab der Pose,
        y = A · sin(2π x / welle_m) mit höchstens winkel_deg gegen die Fahrtrichtung (0 = gerade), Ausschlag über die
        erste und letzte halbe Welle weich ein-/ausgeblendet (gerader Anfang und gerades Ende), Punkte alle 5 cm;
        mit zurueck am Ende 180° wenden und zurück; kurve False: ohne Kurventempo-Grenze (home.kurve_a_mps2, Test der
        Schräglage in der Kurve). Fortschritt in /status bahn. Rückgabe: Punkte (Raumkarte)."""
        b = self.behavior
        if b.name != "manual":
            raise ModeNotAllowed("Bahn nur im Modus manual")
        if not (0.2 <= laenge_m <= 6.0 and 0.0 <= winkel_deg <= 45.0 and 0.3 <= welle_m <= 6.0
                and (v is None or 0.05 <= v <= 1.0)):
            raise ValueError("Grenzen: laenge 0.2..6, winkel 0..45, welle 0.3..6, v 0.05..1")
        if not self.robot.state.balancing:
            raise ModeNotAllowed("Bahn nur balancierend")
        if not self.pipeline_on or self.gridmap is None or not self.gridmap.pose_ok:
            raise ModeNotAllowed("Bahn braucht Bildverarbeitung und Raumkarte mit bekannter Pose")
        x0, y0, th = self.gridmap.pose()
        amp = math.tan(math.radians(winkel_deg)) * welle_m / (2.0 * math.pi)
        n = max(2, int(round(laenge_m / 0.05)))
        c, s = math.cos(th), math.sin(th)
        pts = []
        # Ausschlag über die erste und letzte halbe Welle weich ein-/ausblenden (sin²): die Schlange beginnt und endet
        # gerade, hin wie zurück gleich (vorher begann sie mit voller Steigung, hin schwang er sich erst ein)
        rampe = min(welle_m / 2.0, laenge_m / 2.0)
        for i in range(n + 1):
            u = laenge_m * i / n
            g = min(1.0, u / rampe, (laenge_m - u) / rampe) if rampe > 0 else 1.0
            q = amp * math.sin(0.5 * math.pi * g) ** 2 * math.sin(2.0 * math.pi * u / welle_m)
            pts.append((x0 + u * c - q * s, y0 + u * s + q * c))
        from dataclasses import replace
        b.bahn(pts, zurueck, self.cfg.home if kurve else replace(self.cfg.home, kurve_a_mps2=0.0), v)
        log.info("Bahn: %.1f m, %.0f°, Welle %.1f m (Ausschlag %.2f m), v %s%s", laenge_m, winkel_deg, welle_m, amp,
                 "home.v" if v is None else "%.2f" % v, (", zurück" if zurueck else "") + ("" if kurve else ", ohne Kurventempo"))
        return pts

    # Aktionen der Firmware, die vorwärts/rückwärts fahren (5 dreht nur auf der Stelle)
    FW_ACTION_DRIVES = {3, 4}

    def fw_action(self, n: int) -> None:
        """Fahr-Aktion der Firmware (3, 4, 5) anfragen. Gestartet wird sie im nächsten Regelschritt, wenn
        safety() und die Bahnprüfung ihren ersten Abschnitt ungekürzt durchlassen; solange sie läuft, prüft
        jeder Regelschritt die laufenden und kommenden Abschnitte und bricht sonst ab (_action_tick).
        Der Keepalive ist währenddessen aus, sonst hält er die Aktion an (Robot.action_start)."""
        if n not in FW_ACTIONS:
            raise KeyError(n)
        if self.ruhe:
            raise ModeNotAllowed("Ruhemodus: erst wecken")
        if self.behavior.name != "idle" or self._pending_mode:
            raise ModeNotAllowed(f"Aktion {n} fährt selbst: nur im Modus idle")
        if not self.robot.state.balancing:
            raise ModeNotAllowed(f"Aktion {n}: Rider balanciert nicht")
        if self.robot.action is not None or self._action_pending is not None:
            raise ModeNotAllowed("Es läuft noch eine Aktion")
        if n in self.FW_ACTION_DRIVES:
            if not self.pipeline_on:
                raise ModeNotAllowed(f"Aktion {n} fährt: braucht laufende Bildverarbeitung")
            if self.fs_worker is None or self.fs_worker.blind:
                raise ModeNotAllowed(f"Aktion {n} fährt: braucht die Bodengrenze (FloorNet), die fehlt oder ist ausgefallen")
            if self._free is None:
                raise ModeNotAllowed(f"Aktion {n} fährt: Bodengrenze noch nicht da, kurz warten")
        self._action_pending = n

    def _action_stop(self, keep_turn: bool = False) -> None:
        """Angefragte Aktion verwerfen, laufende abbrechen (keep_turn: die Drehaktion 5 darf weiterlaufen)."""
        self._action_pending = None
        act = self.robot.action
        if act is not None and not (keep_turn and act[0] not in self.FW_ACTION_DRIVES):
            self.robot.action_abort()

    def _action_blocked(self, n: int, elapsed_s: float, w: World | None) -> str:
        """Grund, warum Aktion n jetzt nicht (weiter)fahren darf, sonst "". Prüft die Fahrbefehle ihrer
        laufenden und kommenden Abschnitte wie jeden anderen Fahrbefehl: safety(), dann Bahnprüfung. Kürzen
        lässt sich der Fahrwert der Firmware nicht, also zählt jede Kürzung als gesperrt.
        w None: ohne Bildverarbeitung, nur für die Drehaktion."""
        if not self.robot.state.balancing:
            return "balanciert nicht"
        for v, wz in action_cmds(n, elapsed_s, self.cfg.robot.action_tick_s):
            if abs(v) < 1e-3:
                continue                              # Drehen auf der Stelle: braucht keinen Freiraum
            if w is None:
                return "keine Bildverarbeitung"
            w2 = copy.copy(w)
            w2.cmd_v = v                              # TTC-Prüfung in safety() rechnet mit dem gefahrenen Befehl
            c = self._arc_check(safety(Command(v, wz), w2, self.cfg.behavior))
            if abs(c.v) < abs(v) - 1e-6:
                return c.status.strip(" |") or "gestoppt"
        return ""

    def _action_tick(self, w: World | None, cmd: Command, now: float) -> Command:
        """Im Regelschritt: angefragte Aktion prüfen und starten, laufende weiter prüfen. Ohne Aktion zwei
        Vergleiche; mit Aktion bis zu drei Bahnprüfungen je Regelschritt (C++-Kern), nur für deren Dauer."""
        n, act = self._action_pending, self.robot.action
        if n is not None:
            self._action_pending = None
            grund = "Modus " + self.behavior.name if self.behavior.name != "idle" else self._action_blocked(n, 0.0, w)
            if not grund:
                try:
                    self.robot.action_start(n)
                except OSError as e:
                    grund = str(e)
            if grund:
                log.warning("Aktion %d nicht gestartet: %s", n, grund)
                self._action_note = (f"Aktion {n} nicht gestartet: {grund}", now + 4.0)
            else:
                self._action_note = None
                cmd.status = (cmd.status + " | " if cmd.status else "") + f"Aktion {n}"
        elif act is not None:
            grund = "Modus " + self.behavior.name if self.behavior.name != "idle" \
                else self._action_blocked(act[0], time.monotonic() - act[1], w)
            if grund:
                self.robot.action_abort()
                self._action_note = (f"Aktion {act[0]} abgebrochen: {grund}", now + 4.0)
            else:
                cmd.status = (cmd.status + " | " if cmd.status else "") + f"Aktion {act[0]}"
        note = self._action_note
        if note is not None:
            if now < note[1]:
                cmd.status = (cmd.status + " | " if cmd.status else "") + note[0]
            else:
                self._action_note = None
        return cmd

    # Modi, die ohne Kamerabild laufen (Regelschritt dann in _paused_tick)
    NO_CAMERA = {"idle", "manual"}

    def set_mode(self, name: str, target: str | None = None, confirmed: bool = False) -> None:
        """confirmed: Der Nutzer hat bestätigt, dass der Rider am Startpunkt steht (Streife: ohne Wiederfinden
        gilt beim Laden Pose = Start)."""
        self._make(name, target)  # validiert (KeyError)
        self._check_mode(name, confirmed, target)
        self.lade.vergessen()                         # Modus von Hand (auch der Ladewächter, nach seinem Wecken)
        if name == "patrol" and not confirmed:
            raise ModeNotAllowed("Streife: bitte bestätigen, dass der Rider am Startpunkt steht")
        if name == "patrol":
            self.gridmap.lock()
            self.gridmap.relocate_to_start()
            self.pose_grund = ""
        if self.ruhe and name != "idle":
            raise ModeNotAllowed("Ruhemodus: erst wecken")
        if not self.pipeline_on and name not in self.NO_CAMERA:
            raise ModeNotAllowed(f"Modus {name} braucht laufende Bildverarbeitung")
        self._action_stop()                           # auch POST /stop (setzt idle): Aktion der Firmware anhalten
        with self._mode_lock:
            self._pending_mode = (name, target)

    def merk_punkte(self):
        """Punkte der Merkkarte für das Bild der Raumkarte (alle, bestätigte); None ohne laufende Bildverarbeitung.
        Nur auf Anfrage (Browser 1x je s, Rohaufzeichnung alle record_map_s), nie im Regeltakt; die Felder der
        Merkkarte werden nie verändert, nur ersetzt."""
        lm = getattr(self, "lmap", None)
        if lm is None or not self.pipeline_on:
            return None
        pts = lm.pts
        if not len(pts):
            return None
        try:
            best = lm.confirmed_later(lm.max_range)()
        except Exception:                              # Felder mitten im Wechsel: dieses Bild ohne
            best = None
        return pts, best

    # ------------------------------------------------------------ Bodenmodell wählen (Web)
    FN_NAME =re.compile(r"^[A-Za-z0-9_.+-]{1,80}\.bin$")
    FN_MODES = {"idle", "manual"}          # Wechsel nur, wenn nichts nach dem Bodenmodell fährt

    def floornet_info(self) -> dict:
        """Wählbare Netze in [freespace] net_dir, Ziel des Symlinks net, geladenes Netz. Nur auf Anfrage."""
        fc = self.cfg.freespace
        modelle = []
        if fc.net_dir and os.path.isdir(fc.net_dir):
            for n in sorted(os.listdir(fc.net_dir)):
                p = os.path.join(fc.net_dir, n)
                if self.FN_NAME.match(n) and os.path.isfile(p):
                    st = os.stat(p)
                    modelle.append({"name": n, "kb": round(st.st_size / 1024), "zeit": int(st.st_mtime)})
        ist_link = bool(fc.net) and os.path.islink(fc.net)
        aktiv = None
        if ist_link and fc.net_dir and os.path.dirname(os.path.realpath(fc.net)) == os.path.realpath(fc.net_dir):
            aktiv = os.path.basename(os.path.realpath(fc.net))
        fs = self.fs_worker
        return {"modelle": modelle, "aktiv": aktiv, "geladen": self.fn_geladen, "link": fc.net,
                "ist_link": ist_link, "ordner": fc.net_dir, "blind": fs is None or fs.blind,
                "modes": sorted(self.FN_MODES)}

    def floornet_wechseln(self, name: str) -> str:
        """FloorNet gegen name aus [freespace] net_dir tauschen, ohne Neustart: neues Netz laden und prüfen (läuft im
        Web-Thread, nicht in der Bildschleife), erst dann den Symlink net umstellen und das Netz im FreeSpaceWorker
        tauschen; das alte wird geschlossen. Scheitert das Laden, bleibt alles wie es war. Gibt den Namen zurück."""
        fc = self.cfg.freespace
        if self.fs_worker is None:
            raise ModeNotAllowed("Bodenmodell ist aus ([freespace] enabled)")
        if not fc.net_dir or not fc.net:
            raise ModeNotAllowed("[freespace] net_dir oder net leer: nicht wählbar")
        if not self.FN_NAME.match(name or ""):
            raise ValueError("Modellname ungültig")
        pfad = os.path.join(fc.net_dir, name)
        if not os.path.isfile(pfad):
            raise ValueError(f"{name} fehlt in {fc.net_dir}")
        if os.path.exists(fc.net) and not os.path.islink(fc.net):
            raise ModeNotAllowed(f"{fc.net} ist eine Datei, kein Symlink (deploy/install.sh legt ihn an)")
        if self.mode not in self.FN_MODES:
            raise ModeNotAllowed(f"Bodenmodell nur in {', '.join(sorted(self.FN_MODES))} wechseln, nicht in {self.mode}")
        if not self._fn_lock.acquire(blocking=False):
            raise ModeNotAllowed("Bodenmodell wird gerade gewechselt")
        try:
            t0 = time.monotonic()
            net = FloorNet.load(pfad, fc.net_outlier_px, fc.net_outlier_ratio)
            if net is None:
                raise ModeNotAllowed(f"{name} nicht ladbar oder Selbstprüfung fehlgeschlagen (Log), altes Netz bleibt")
            if self.mode not in self.FN_MODES:          # während des Ladens umgeschaltet
                net.close()
                raise ModeNotAllowed(f"Modus {self.mode} während des Wechsels: altes Netz bleibt")
            neu = fc.net + ".neu"
            if os.path.lexists(neu):
                os.remove(neu)
            os.symlink(os.path.relpath(pfad, os.path.dirname(fc.net) or "."), neu)
            os.replace(neu, fc.net)                      # Symlink atomar umstellen
            alt = self.fs_worker.set_net(net)
            if alt is not None and hasattr(alt, "close"):
                alt.close()
            vorher, self.fn_geladen = self.fn_geladen, name
            self.reset_floor(self.cfg.freespace.restart_hold_s)
            log.info("[floornet] gewechselt: %s -> %s (%.0f ms)", vorher, name, (time.monotonic() - t0) * 1000)
            return name
        finally:
            self._fn_lock.release()

    @property
    def mode(self) -> str:
        return self.behavior.name

    def stop(self) -> None:
        self.running = False

    def run(self, duration: float | None = None) -> None:
        self.running = True
        self.robot.start()
        self.src.start()
        has_det = self.det.start()
        if has_det and self.gesichter is None and self.cfg.gesicht.enabled:
            g = Gesichter(self.cfg.gesicht)
            if g.start():
                self.gesichter = g
        if self.gesichter is not None:
            self.det.gesichter = self.gesichter
        if self.fs_worker is not None and self.cfg.freespace.threaded:
            self.fs_worker.start()
        if self.wheels is not None and not self.wheels.start():
            self.wheels = None
        if self.grid_worker is not None:
            self.grid_worker.start()
        if self.imu is not None and not self.imu.start():
            self.imu = None
        log.info("läuft: Modus=%s, Detektor=%s, IMU=%s, Räder=%s", self.behavior.name,
                 "an" if has_det else "aus", "an" if self.imu else "aus", "an" if self.wheels else "aus")
        t_end = time.monotonic() + duration if duration else None
        last = -1
        try:
            while self.running:
                if t_end and time.monotonic() > t_end:
                    break
                self._wlan_buttons()
                self._ortung_tick()
                self._lade_tick(time.monotonic())     # Akku in pet: auch im Ruhemodus (Laden, Aufwachen)
                if self.wlan is not None:
                    last = self._wlan_tick(last)
                    continue
                if self.src.paused:
                    self._paused_tick()
                    continue
                f = self.src.get(last, timeout=1.0)
                if f is None:
                    if self.src.eof:
                        log.info("Quelle beendet")
                        break
                    log.warning("kein Kamerabild – Stopp")
                    self.robot.stop()
                    continue
                last = f.idx
                t0 = time.perf_counter()
                self._step(f)
                ms = (time.perf_counter() - t0) * 1000
                self._loop_ms = 0.9 * self._loop_ms + 0.1 * ms
                self._loop_hist.append(ms)
                if isinstance(self.src, OpenCVSource):
                    self.src.mark_consumed()
        finally:
            self.running = False
            with self.snap_cond:
                self.snap_cond.notify_all()
            self._pet_leave()
            self.robot.stop()
            if self.imu is not None:
                self.imu.close()
            self.det.close()
            if self.ortung is not None:
                self.ortung.close()
            if self.stimmen is not None:
                self.stimmen.close()
            if self.fs_worker is not None:
                self.fs_worker.close()
            if self.wheels is not None:
                self.wheels.close()
            if self.grid_worker is not None:
                self.grid_worker.close()
            if self.planner is not None:
                self.planner.close()
            if self.gridmap is not None and self.cfg.grid.file:
                self.gridmap.save(self.cfg.grid.file)
            self.src.close()
            self.robot.close()
            log.info("beendet")

    # ------------------------------------------------------------------
    def _liegt_idle(self) -> None:
        """Liegt er (abgesetzt, gefallen), endet jeder Modus außer LIEGEND_OK sofort, noch vor dem Verhalten."""
        b = self.behavior
        if b.name in self.LIEGEND_OK or self.robot.state.balancing:
            return
        with self._mode_lock:
            self._pending_mode = None
            self.behavior = self._make("idle")
            if self.gridmap is not None:
                self.gridmap.plan = []
                self.gridmap.erwartet = None
        log.warning("Modus %s endet: Rider liegt (%s) -> idle", b.name, stopp_text(self.robot.state.mode))

    def _apply_pending_mode(self) -> None:
        with self._mode_lock:
            if self._pending_mode:
                name, target = self._pending_mode
                self._pending_mode = None
                self._pet_leave()
                self.behavior = self._make(name, target)
                if self.gridmap is not None:
                    self.gridmap.plan = []
                    self.gridmap.erwartet = None
                log.info("Modus -> %s %s", name, target or "")

    def _drive(self, cmd: Command) -> None:
        """Fahrbefehl an den Roboter (alle Modi gleich, auch die Handsteuerung: Faktor x tempo x max_v)."""
        bogen_w(cmd, self.robot, self.cfg.robot)
        explore = self.behavior.name == "explore"
        self.robot.a_modus = self.cfg.explore.accel_faktor if explore else 1.0
        kurs = cmd.kurs if abs(cmd.v) < 1e-6 else None
        if explore and abs(cmd.v) < 1e-6:            # Drehen auf der Stelle × explore.dreh_faktor
            f = self.cfg.explore.dreh_faktor
            cmd.w *= f
            if kurs is not None:
                kurs = (kurs[0], kurs[1] * f)
        if kurs is not None and self.robot.kursabs_an:
            kurs = (kurs[0], kurs[1], self._kurs_fw(kurs[0]))
        self.robot.drive(cmd.v, cmd.w, brake=bool(cmd.extra.get("bremsen")), kappa=cmd.kappa, kurs=kurs,
                         strecke=cmd.strecke)

    def _kurs_fw(self, err: float) -> float | None:
        """Absolutes Kursziel (robot.vel_kursabs): Restfehler err (rad, gegen self.yaw) als Kurs der Firmware (°). self.yaw
        gehört zum IMU-Kurs des letzten Regelschritts (_kurs_imu_ref); Korrekturen der Pose ändern nur den Namen des
        Kurses, nicht die Drehung, die noch fehlt. Höchstens KURSABS_VORAUS_DEG voraus (größere Drehungen stückweise,
        das Ziel wandert mit). None ohne IMU-Strom: Robot nimmt dann das Kursziel Δψ."""
        if self.imu is None or self._kurs_imu_ref is None:
            return None
        g = math.radians(KURSABS_VORAUS_DEG)
        return self.imu.fw_kurs_deg(self._kurs_imu_ref + max(-g, min(g, err)))

    def _manual_cmd(self, w: World | None = None) -> Command:
        """Handsteuerung: nur Sturz-Erkennung, kein Bodenmodell/keine Bahnprüfung. Geprüfter Sollwert
        (Manual.checked): safety() und Bahnprüfung wie in den anderen Modi; ohne Weltdaten (Bildverarbeitung
        aus) fährt er dann nicht, Drehen bleibt."""
        cmd = self.behavior.step(w, yaw=self.yaw)
        if not self.robot.state.balancing:
            if getattr(self.behavior, "_turn", None) is not None:
                self.behavior.turn_ende("liegt")
            if getattr(self.behavior, "_bahn", None) is not None:
                self.behavior.bahn_ende("liegt")
            return Command(0.0, 0.0, None, stopp_text(self.robot.state.mode))
        soll = getattr(self.behavior, "strecke_soll", None)
        if soll is not None and cmd.v:
            # Strecke mit Halt (/drive?strecke=S): Reststrecke aus der Odometrie, ab dem ersten Regelschritt gezählt
            if self.behavior.strecke_odo0 is None:
                self.behavior.strecke_odo0 = self.odom_s_m
                self._strecke_min = soll
            gefahren = (self.odom_s_m - self.behavior.strecke_odo0) * (1.0 if cmd.v > 0 else -1.0)
            # nur kleiner werden lassen: ein größerer Rest weckt die Firmware nach dem Halt wieder auf, und das Pendeln
            # der Odometrie beim Balancieren ließ ihn so rückwärts in Stößen 13 cm weiterrollen (Rider 2026-11-25)
            self._strecke_min = min(self._strecke_min, max(0.0, soll - gefahren))
            cmd.strecke = self._strecke_min
            cmd.status += " Strecke %.2fm" % cmd.strecke
        if getattr(self.behavior, "checked", False) and (cmd.v or cmd.w):
            if w is None:
                if cmd.v:
                    cmd = Command(0.0, cmd.w, None, cmd.status + " | geprüft: keine Bildverarbeitung")
            else:
                cmd = self._arc_check(safety(cmd, w, self.cfg.behavior))
        return cmd

    def _paused_tick(self) -> None:
        """Hauptschleife bei gestoppter Bildverarbeitung: Moduswechsel sofort übernehmen;
        im Modus manual mit control_hz regeln (ohne Kamera), sonst Stillstand."""
        self._apply_pending_mode()
        self._liegt_idle()
        self._pose_watch(time.monotonic())
        self._start_tick(time.monotonic())
        if self.ruhe:
            self._cmd = Command(0.0, 0.0, None, "Ruhemodus")
            time.sleep(self.cfg.ruhe.tick_s)
            return
        self._imu_idle_tick()
        self._paused_odom()
        if self.behavior.name not in self.NO_CAMERA:
            self._pet_leave()
            self.behavior = self._make("idle")
        if self.behavior.name != "manual":
            # Drehaktion 5 der Firmware geht auch ohne Bild; fahrende lehnt fw_action hier schon ab
            self._cmd = self._action_tick(None, Command(0.0, 0.0, None, "Bildverarbeitung gestoppt"), time.monotonic())
            self._emotion_tick(None)
            time.sleep(0.1)
            return
        period = 1.0 / self.cfg.control_hz
        now = time.monotonic()
        if now - self._last_ctrl >= period:
            self._last_ctrl = now
            cmd = self._manual_cmd()
            self._drive(cmd)
            self._cmd = Command(cmd.v, cmd.w, None, cmd.status + " (ohne Bildverarbeitung)")
            self._emotion_tick(None)                  # Akku und Sturz
        time.sleep(max(0.0, self._last_ctrl + period - time.monotonic()))

    def _step(self, f) -> None:
        tm = _Marks()
        self._apply_pending_mode()
        self._liegt_idle()
        cc = self.cfg.camera
        if ((self.cfg.dunkel.enabled or cc.gain_switch)
                and f.t - self._luma_t >= 1.0 / max(0.1, self.cfg.dunkel.check_hz)):
            self._luma_t = f.t
            self._luma, hell = bildwerte(f.bgr, cc.gain_sat_level)
            if cc.gain_switch:
                self._gain_tick(f, hell)

        excl = [t.box for t in self.tracker.tracks]
        ego = self.ego.update(f.idx, f.t, f.bgr, excl)
        if ego.ok:
            self.vis_yaw += ego.yaw_rate * ego.dt
        else:
            self._vis_gap = True
        pitch_dev = self._pitch_dev(f, ego)
        self.last_pitch_dev = pitch_dev
        self.tracker.predict(ego.A, ego.dt)
        tm.mark("ego")

        batch = self.det.poll()
        if batch is not None:
            self._det_count += 1
            T = self.ego.transform_between(batch.frame_idx, f.idx)
            if T is not None:
                dets = [Detection(warp_box(T, d.box), d.cls, d.conf, getattr(d, "who", None), getattr(d, "who_sim", None))
                        for d in batch.dets]
                self.tracker.update(dets, f.t, lag=f.t - batch.t)
            else:
                log.debug("Detektion zu alt, verworfen")
        self.tracker.prune(f.t)
        if self.behavior.name in self.DET_MODES:
            g = self.gesichter
            named = None              # Gesichtsprüfung nur in [gesicht] modes; benannte Tracks nicht noch einmal
            if g is not None and g.ok and g.merkmale and self.behavior.name in self.cfg.gesicht.modes:
                named = [t.box for t in self.tracker.tracks if t.who]
            self.det.offer(f, ego.sharpness, self.ego.sharpness_median(), named)
        tm.mark("det")

        bal = self.robot.state.balancing
        self._imu_idle_tick()
        if bal and self._was_balancing is False:
            # aufgestanden: Kamera sieht den Boden jetzt aus anderer Höhe und Lage
            log.info("Rider steht auf: Bodengrenze neu")
            self.reset_floor(hold_s=self.cfg.freespace.restart_hold_s)
        self._was_balancing = bal
        if time.monotonic() < self._fs_hold_until or self.fs_worker is None:
            pass                          # nach Kamerastart/Aufstehen: Belichtung und Lage abwarten
        elif not bal:
            # FloorNet nur balancierend (Nutzer 2026-11-21): liegend/abgesetzt fährt er nicht selbst, und so schräg
            # hat das Netz den Boden nie gesehen. Keine Bodengrenze, spart die Netzzeit.
            self._free = None
        elif f.idx % 2 == 0 or self._free is None:
            # FloorNet jedes 2. Bild: im eigenen Thread (neuestes Ergebnis übernehmen) oder im Takt
            roll = self._imu_frame.roll_deg if self._imu_frame is not None else 0.0
            args = (f.bgr, pitch_dev, self.gridmap.pose() if self.gridmap is not None else None, f.t, roll,
                    self.lmap.odo if self.lmap is not None else None)
            if self.cfg.freespace.threaded:
                self.fs_worker.offer(*args)
            else:
                r = self.fs_worker.compute(self.fs_worker.job(*args))
                if r is not None:
                    self._free = r
                    tm.fs_ran = True
        if self.fs_worker is not None and self.cfg.freespace.threaded and time.monotonic() >= self._fs_hold_until and bal:
            r = self.fs_worker.poll()
            if r is not None:
                self._free = r
        tracks = self.tracker.confirmed(f.t)
        tm.mark("fs")

        now = time.monotonic()
        # Regelschritt an dem Bild, das dem Takt am nächsten liegt (halber Bildabstand Toleranz). Ohne die Toleranz
        # lief er bis 2026-11-17 bei 20 Hz Soll und 41,7 ms Bildabstand nur jedes 2. Bild (~12 Hz); mit
        # control_hz = Bildrate ließe Jitter sonst Schritte aus.
        if now - self._last_ctrl >= 1.0 / self.cfg.control_hz - 0.5 / max(self.cfg.camera.fps, 1):
            dt = now - self._last_ctrl if self._last_ctrl else 0.0
            self._last_ctrl = now
            # Freiraum nur aus dem Bodenmodell: Erkennungen (YOLO) sind kein Hindernis (Nutzer 2026-11-13)
            free = self._free
            # Eigenbewegung seit dem letzten Regelschritt aus der Koppelnavigation (odomfusion.py): IMU-Kurs und
            # Radwege zur Bildzeit, gegeneinander geprüft, das Bild als Zeuge im Stand. Ein Ergebnis für alle.
            dyaw_vis = self.vis_yaw - self._yaw_at_ctrl
            self._yaw_at_ctrl = self.vis_yaw
            fr = self._imu_frame
            st = self._odom_step(fr.t_imu if fr is not None else f.t, fr.yaw if fr is not None else None,
                                 None if self._vis_gap else dyaw_vis, dt)
            self._vis_gap = False
            ds, dyaw = st.ds, st.dyaw
            wh = self.wheels
            self._v_rad = (wh.speed(ARC_IST_WINDOW_S) if wh is not None and self.robot.state.balancing
                           and wh.fresh(now) else None)
            self._pose_watch(now)
            self._start_tick(now)
            w = World(t=f.t, dt=dt, W=self.cfg.camera.width, H=self.cfg.camera.height, hfov=self.hfov,
                      ego=ego, tracks=tracks, free=free, balancing=self.robot.state.balancing,
                      fw_mode=self.robot.state.mode,
                      yaw=self.yaw, cmd_v=self.robot.command[0],
                      geo=self.geo, pitch_dev=pitch_dev, battery=self.robot.state.battery,
                      stunde=self.uhr() if self.uhr is not None else None,
                      blind=self.fs_worker is not None and self.fs_worker.blind,
                      v_rad=self._v_rad, clear_fn=self._clear_ahead if self.lmap is not None else None,
                      v_erw=self._v_erwartet())
            if self.behavior.name == "pet":
                self._pet_world(w, ds, now)
            if self.lmap is not None:
                self.lmap.move(ds, dyaw)
                if self.gridmap is not None:
                    self.gridmap.move(ds, dyaw)
                if free is not None and self._free is not self._fs_in_map:
                    self._fs_in_map = self._free
                    # Nick und Pose der Aufnahme, nicht des Bildes, in dem das Ergebnis ankam
                    pd = free.pitch_dev if free.pitch_dev is not None else pitch_dev
                    self.lmap.add_freespace(free, f.t, pd)      # Bahnprüfung: unmaskiert
                    view = self._map_view(free, tracks)     # Raumkarte: ohne Personen und Tiere
                    # Raumkarte nur auf den Rädern (liegend sieht die Kamera etwas anderes) und
                    # ohne schnelle Drehung
                    rate = abs(math.degrees(dyaw)) / dt if dt > 0 else 0.0
                    if self.robot.state.balancing and rate <= self.cfg.grid.max_turn_dps:
                        if self.grid_worker is not None:
                            self.grid_worker.offer(view, pd)
                        elif self.gridmap is not None:
                            vp = view.pose
                            if self.reloc_track is not None and self.gridmap.pose_ok:
                                vp = vp if vp is not None else self.gridmap.pose()
                                ok, k = self.reloc_track.schritt(self.gridmap, now, vp, view, pd)
                                if k is not None:
                                    self.gridmap.shift_pose(k)
                                    from .reloc import anwenden
                                    vp = anwenden(k, vp)
                                if not ok:
                                    vp = False
                            if vp is not False:
                                self.gridmap.add_freespace(view, pd, vp)
            if self.gridmap is not None:
                if self.grid_worker is None:
                    self.gridmap.maybe_save(self.cfg.grid.file, self.cfg.grid.save_every_s)
                w.pose, w.goal = self.gridmap.pose(), self.gridmap.start
                w.pose_ok = self.gridmap.pose_ok
                if self.startbild is not None:
                    w.pose = self._startbild_tick(f, w, pitch_dev, dyaw, dt)
                # feste Bahn im Modus manual (POST /bahn): derselbe Wegfolger, dieselben Bremsgründe wie mit Planer
                if hasattr(self.behavior, "want_plan") or getattr(self.behavior, "_bahn", None) is not None:
                    if self.lmap is not None and self.cfg.home.free_slow:
                        w.path_free_fn = self._path_free     # Tempo nach freier Strecke (nur beim Fahren gerufen)
                        w.arc_free_fn = self._arc_free
                    if self.cfg.home.bekannt_v != 1.0 or self.cfg.home.unbekannt_v != 1.0:
                        w.known_fn = self.gridmap.known_m   # Tempo nach Kenntnis (nur beim Fahren gerufen)
                    if self.cfg.home.tight_m > 0:           # Engstellen: der Wegfolger fragt nur bei Bedarf
                        w.wall_fn = self.gridmap.wall_dist
                        if self.lmap is not None and self.cfg.home.tight_center:
                            w.gap_fn = self.lmap.gap_center
                    if hasattr(self.behavior, "want_plan"):
                        self._plan_tick(w)
            # approach erkundet mit derselben Logik, bis das Ziel in Sicht ist
            if self.lmap is not None and self.behavior.name in ("explore", "approach"):
                vg = self.cfg.robot.v_of(self.cfg.behavior.cruise_v) * self.cfg.robot.vx_limit
                w.path_blocked = self.lmap.arc_clearance(vg, 0.0, horizon_s=1.2) < 0.6 * vg + 0.08
            elif self.lmap is not None and self.behavior.name == "pet" and self.behavior.drives:
                # pet sucht oder streift: eine Bahnprüfung je Regelschritt, nur solange er fährt
                vg = self.cfg.robot.v_of(self.cfg.pet.drive_v) * self.cfg.robot.vx_limit
                w.path_blocked = self.lmap.arc_clearance(vg, 0.0, horizon_s=1.2) < 0.6 * vg + 0.08
            if getattr(self.behavior, "want_set_down", False):
                self.behavior.want_set_down = False
                log.warning("Modus %s: setze ab", self.behavior.name)
                self.robot.set_down()
            if self.behavior.name == "manual":
                cmd = self._manual_cmd(w)
            else:
                cmd = safety(self.behavior.step(w), w, self.cfg.behavior)
                if getattr(self.behavior, "gap_blocked", False):
                    self.behavior.gap_blocked = False
                    self._luecke_eintragen()
                if getattr(self.behavior, "want_messung", False):
                    self.behavior.want_messung = False
                    self._startbild_messen(f, w, pitch_dev)
                cmd = self._dunkel_tick(now, cmd)
                if self.gridmap is not None and self.gridmap.bremsen_abgelaufen(now):
                    log.info("Bremsstellen abgelaufen, jetzt %d", len(self.gridmap.brems))
                cmd = self._arc_check(cmd)
            if self._action_pending is not None or self.robot._act is not None or self._action_note is not None:
                cmd = self._action_tick(w, cmd, now)
            self._drive(cmd)
            # pet: Abducken beim Schreck darf nicht bis zu 1 s warten; set_height schreibt nur Änderungen
            if cmd.height_mm is not None and (now - self._last_height_t > 1.0 or self.behavior.name == "pet"):
                self.robot.set_height(cmd.height_mm)
                self._last_height_t = now
            if self.robot.height_mm is not None:
                # auch von außen gesetzt (Web) oder nach dem Aufstehen, siehe Robot._poll_height/_height_tick
                self.geo.body_height_mm = self.robot.height_mm
            self._cmd = cmd
            self._free_ctrl = free
            cls = getattr(self.behavior, "cls", None) or self.cfg.behavior.target_class   # gewähltes Ziel
            if self.behavior.name == "pet":
                self._pet_tick()
                self._emotion_tick(None)              # pet entscheidet selbst, bleibt nur der Akku
            else:
                self._emotion_tick(any(t.cls == cls and t.lost_s(f.t) < 0.5 for t in tracks))
        tm.mark("ctrl")

        snap = Snapshot(frame=f.bgr, idx=f.idx, t=f.t, pitch_dev=pitch_dev, tracks=list(tracks),
                        imu=self._imu_frame, nickrate=self._nickrate,
                        odo=self.lmap.odo if self.lmap is not None else None, geo=self.geo,
                        tracks_all=list(self.tracker.tracks), cam_meta=f.meta,
                        free=self._free_ctrl or self._free,
                        ego=ego, cmd=self._cmd, sent=self.robot.command,
                        stats={"fps": round(self.src.fps_measured, 1), "loop_ms": round(self._loop_ms, 1),
                               **self._loop_spread(),
                               "det_ms": round(self.det.infer_ms, 1),
                               "gesicht_ms": None if self.gesichter is None else round(self.gesichter.ms, 1), "det_batches": self._det_count,
                               "plan_ms": round(self.planner.ms, 1) if self.planner is not None else None,
                               "explore": self.behavior.info() if self.behavior.name == "explore" and hasattr(self.behavior, "info") else None,
                               "patrol": self.behavior.info() if self.behavior.name == "patrol" else None,
                               "pet": self.behavior.info() if self.behavior.name == "pet" else None,
                               "net_ms": round(self.fs_worker.net_ms, 1) if self.fs_worker is not None and self.fs_worker.net is not None else None,
                               "fs_src": None if self.fs_worker is None else "blind" if self.fs_worker.blind else
                               self._free.src if self._free is not None else None,
                               "grid": None if self.gridmap is None else {
                                   **self.gridmap.stats(),
                                   "reloc": self.cfg.reloc.enabled, "pose_grund": self.pose_grund,
                                   "ms": round(self.grid_worker.ms, 1) if self.grid_worker else None,
                                   "verworfen": self.grid_worker.dropped if self.grid_worker else 0,
                                   "abgleich": None if self.reloc_track is None else {
                                       **self.reloc_track.info(),
                                       "ms_max": round(self.grid_worker.track_ms, 1) if self.grid_worker else None}},
                               "odom": {"src": self.odom_src, "m": round(self.odom_m, 2),
                                        "hz": round(self.wheels.rate_hz, 1) if self.wheels else None,
                                        **self.odomf.info(),
                                        "imu_spruenge": self.imu.yaw_jumps if self.imu is not None else None},
                               "startbild": None if self.startbild is None else self.startbild.info(),
                               "turn": getattr(self.behavior, "turn_info", None) if self.behavior.name == "manual" else None,
                               "bahn": getattr(self.behavior, "bahn_info", None) if self.behavior.name == "manual" else None,
                               "mode": self.behavior.name, "yaw_vis_deg": round(math.degrees(self.vis_yaw), 1),
                               "yaw_deg": round(math.degrees(self.yaw), 1), "yaw_src": self.yaw_src,
                               "yaw_imu": self.kurs_deg(), "battery": self.robot.state.battery,
                               "balancing": self.robot.state.balancing, "state": self.robot.state.mode,
                               "height_mm": round(self.geo.body_height_mm, 1),
                               "pitch_src": self.pitch_src, "pitch_deg": round(math.degrees(pitch_dev), 2),
                               "imu": None if self.imu is None else {
                                   "hz": round(self.imu.rate_hz, 1), "sync": self.imu_sync.reliable,
                                   "sign": self.imu_sync.sign, "offset_ms": round(self.imu_sync.offset * 1000),
                                   "corr": round(self.imu_sync.corr, 2)}})
        with self.snap_cond:
            self.snapshot = snap
            self.snap_cond.notify_all()
        tm.mark("snap")
        self._seg_hist.append(tm.done())


    def _sb_ansicht(self, f, pose, pitch_dev: float):
        fr = self._imu_frame
        roll = fr.roll_deg if fr is not None and fr.frisch else 0.0
        return sbm.ansicht(f.bgr, pose, self.geo, roll, pitch_dev, f.t)

    def _startbild_tick(self, f, w: World, pitch_dev: float, dyaw: float, dt: float):
        """Je Regelschritt (nur mit Raumkarte): Ergebnis einer Messung übernehmen, Anker setzen, solange der Rider
        am Startpunkt steht, und in der geraden Schlussfahrt von home Graubilder merken. Kosten: ein paar
        Vergleiche; ein Graubild (~1 ms) nur beim Anker und je tiefe_schritt_m Schlussfahrt. Gibt die
        (womöglich korrigierte) Pose zurück."""
        sb, gm, c, b = self.startbild, self.gridmap, self.cfg.startbild, self.behavior
        pose = w.pose
        erg = sb.poll()
        if erg is not None:
            if erg.ok:
                gm.shift_pose(erg.k)
                self.yaw += erg.k[2]
                pose = gm.pose()
                log.info("Startbild: Pose korrigiert um %.0f cm, %+.1f° (%s)", 100 * erg.info.get("schub_m", 0.0),
                         math.degrees(erg.k[2]), erg.grund)
            if hasattr(b, "messung"):
                b.messung = erg
        if sb.anker is not None and sb.anker_start != tuple(gm.start):
            sb.anker_weg("Startpunkt geändert")
        if sb.anker is None and gm.pose_ok and w.balancing:
            sx, sy, sth = gm.start
            rate = abs(dyaw) / dt if dt > 0 else 0.0
            if (math.hypot(pose[0] - sx, pose[1] - sy) < c.anker_m
                    and abs(wrap(pose[2] - sth)) < math.radians(c.anker_deg)
                    and rate < math.radians(2.0) and abs(self.robot.command[0]) < 0.01
                    and getattr(self.robot, "levelled", True)):           # Startbild erst nach der Roll-Kalibrierung
                sb.anker_setzen(self._sb_ansicht(f, pose, pitch_dev), gm.start, f.bgr)
        w.anker = sb.anker is not None and sb.anker_bereit
        # Schlussfahrt: Graubilder je tiefe_schritt_m (neu bei jedem Beginn der Phase); pet fährt heim mit Home
        b = b if b.name == "home" else getattr(b, "home_drive", None)
        if b is not None and getattr(b, "phase", "") == "gerade":
            if self._sb_t_phase != b.t_phase:
                self._sb_t_phase, self._sb_tiefen = b.t_phase, []
            last = self._sb_tiefen[-1].pose if self._sb_tiefen else None
            if last is None or math.hypot(pose[0] - last[0], pose[1] - last[1]) >= c.tiefe_schritt_m:
                self._sb_tiefen.append(self._sb_ansicht(f, pose, pitch_dev))
                del self._sb_tiefen[:-24]
        return pose

    def _startbild_messen(self, f, w: World, pitch_dev: float) -> None:
        """home steht am Start: Endbild nehmen, je Abstand in [startbild] basen das passende Bild der
        Schlussfahrt, Messung im Faden starten. Ohne Bilder: Ergebnis 'keine Tiefenbilder'."""
        ende = self._sb_ansicht(f, w.pose, pitch_dev)
        tiefen, used = [], set()
        for basis in self.cfg.startbild.basen:
            best = None
            for i, a in enumerate(self._sb_tiefen):
                d = math.hypot(a.pose[0] - ende.pose[0], a.pose[1] - ende.pose[1])
                if i not in used and d >= 0.8 * basis and (best is None or abs(d - basis) < best[0]):
                    best = (abs(d - basis), i)
            if best is not None:
                used.add(best[1])
                tiefen.append(self._sb_tiefen[best[1]])
        if not tiefen or not self.startbild.messen(ende, tiefen):
            self.behavior.messung = sbm.Ergebnis(False, "keine Tiefenbilder" if not tiefen else "Messung nicht gestartet")

    def _paused_odom(self) -> None:
        """Pose auch bei gestoppter Bildverarbeitung mitführen (Handsteuerung, Balancier-Drift).
        Sonst stand die Raumkarte still, während der Rider per Hand fuhr, und „Zum Start“
        zielte danach daneben. Koppelnavigation wie im Regelschritt, nur ohne Bild und ohne Fahrbefehl-Strecke."""
        now = time.monotonic()
        y = None
        if self.imu is not None:
            a = self.imu.at(now)
            if a is not None and a[2] is not None and now - a[3] <= imu_max_age(self.cfg.imu, self.imu):
                y = a[2]
        st = self._odom_step(now, y, None, 0.0)
        if st.ds == 0.0 and st.dyaw == 0.0:
            return
        if self.lmap is not None:
            self.lmap.move(st.ds, st.dyaw)
        if self.gridmap is not None:
            self.gridmap.move(st.ds, st.dyaw)

    def _odom_step(self, t: float, yaw_imu: float | None, dyaw_vis: float | None, dt: float):
        """Ein Schritt der Koppelnavigation (odomfusion.py) zur Zeit t (IMU-Zeit des Bildes): Radwege zu t, IMU-Kurs
        zu t, Bild-Kurs seit dem letzten Schritt; Rückfall für die Strecke ist der Fahrbefehl x vx_limit x dt.
        Führt self.yaw, yaw_src, odom_src und odom_m mit."""
        w = self.wheels
        ws = w.at(t) if w is not None and self.cfg.odom.source != "command" else None
        ds_cmd = self.robot.command[0] * self.cfg.robot.vx_limit * dt
        jumps = self.imu.yaw_jumps if self.imu is not None else 0
        st = self.odomf.step(t, yaw_imu, jumps, ws, dyaw_vis, ds_cmd, self.robot.state.balancing)
        self.yaw += st.dyaw
        self._kurs_imu_ref = yaw_imu
        self.yaw_src = st.kurs_src
        self.odom_src = "befehl" if st.ds_src == "befehl" else "räder"
        self.odom_m += abs(st.ds)
        self.odom_s_m += st.ds
        return st

    def kurs_deg(self) -> float | None:
        """Kurs für die Anzeige (Grad, links +): neueste Probe des IMU-Stroms, alle Drehungen aufsummiert, nicht
        auf ±180° gewickelt; dieselbe Quelle wie Karte und Merkkarte. None ohne IMU."""
        y = self.imu.latest_yaw() if self.imu is not None else None
        return None if y is None else round(math.degrees(y), 1)

    def _v_erwartet(self) -> float | None:
        """Tempo (m/s), das der Rider gerade anstrebt: Fahrrahmen = Sollwert des Generators der Firmware (Robot rechnet
        ihn nach); auf dem Byteweg unbekannt (None)."""
        if self.robot.vel:
            return self.robot.command[0] * self.cfg.robot.vx_limit
        return None

    def loop_breakdown(self) -> dict:
        """Wo die Schleifenzeit hingeht (letzte ~250 Bilder): je Abschnitt Mittel und p95 der
        Wanduhrzeit, dazu für die langsamsten 5 % der Bilder der Mittelwert je Abschnitt,
        die CPU-Zeit des Threads (Wand >> CPU = gewartet: GIL oder Kern belegt) und die
        Pausen des Garbage Collectors. Nur auf Anfrage berechnet (/status?breakdown=1)."""
        for _ in range(3):
            try:
                rows = list(self._seg_hist)
                break
            except RuntimeError:            # deque während des Kopierens verändert
                continue
        else:
            return {}
        if len(rows) < 20:
            return {}
        names = [n for n, _, _ in rows[-1]["seg"]]
        tot = sorted(rows, key=lambda r: r["wall"])
        slow = tot[int(0.95 * len(tot)):]

        def stat(rs):
            out = {}
            for i, n in enumerate(names):
                w = sorted(r["seg"][i][1] for r in rs)
                out[n] = {"mean": round(sum(w) / len(w), 1), "p95": round(w[int(0.95 * (len(w) - 1))], 1),
                          "cpu": round(sum(r["seg"][i][2] for r in rs) / len(rs), 1)}
            return out
        return {"n": len(rows), "alle": stat(rows), "langsamste_5pct": stat(slow),
                "langsam_wall": round(sum(r["wall"] for r in slow) / len(slow), 1),
                "langsam_cpu": round(sum(r["cpu"] for r in slow) / len(slow), 1),
                "langsam_gc_ms": round(sum(r["gc"] for r in slow) / len(slow), 1),
                "langsam_fs_anteil": round(sum(1 for r in slow if r["fs_ran"]) / len(slow), 2),
                "fs_anteil": round(sum(1 for r in rows if r["fs_ran"]) / len(rows), 2),
                "gc_ms_gesamt": round(sum(r["gc"] for r in rows), 1)}

    def _loop_spread(self) -> dict:
        """p95 und Maximum der letzten ~10 s Schleifenzeiten (Spitzen sieht das Mittel nicht)."""
        h = self._loop_hist
        if len(h) < 10:
            return {}
        # nur für die Anzeige: alle 12 Bilder (~0,5 s) neu sortieren statt bei jedem Bild
        n = getattr(self, "_spread_n", 0)
        self._spread_n = n + 1
        if n % 12 == 0 or not getattr(self, "_spread", None):
            v = sorted(h)
            self._spread = {"loop_p95": round(v[int(0.95 * (len(v) - 1))], 1), "loop_max": round(v[-1], 1)}
        return self._spread

    START_RUHE_S = 1.0           # nach dem Aufspringen so lange still (< 1 cm, < 1°), dann gilt die Pose als Start
    START_MAX_S = 20.0           # bis dahin nicht still: kein neuer Start

    def _start_tick(self, now: float) -> None:
        """Startpunkt der Raumkarte fest an Aufspringen und Absetzen (Nutzer 2026-11-22), in jedem Modus, nie beim
        Moduswechsel. Aufspringen: Die Pose gilt erst nach der Roll-Kalibrierung (Robot.levelled) und wenn er
        START_RUHE_S still steht (ausgeschwungen; dort nimmt _startbild_tick den Anker). Absetzen: sofort. Nicht bei gesperrter Karte (Start aus explore, Streife) und nur
        mit bestätigter Pose. Lief die Schleife länger nicht (Ruhe, Pause), nur neu einlesen."""
        gm = self.gridmap
        if gm is None:
            return
        bal = self.robot.state.balancing
        gap = now - self._start_t > 1.0
        self._start_t = now
        if self._start_bal is None or gap:
            if not bal and getattr(self, "startbild", None) is not None:
                self.startbild.anker_weg("liegt")           # dazwischen abgesetzt: ebenso kein Startbild
            self._start_bal, self._start_due = bal, None
            return
        if bal != self._start_bal:
            self._start_bal = bal
            self._start_due, self._start_still = (now, None) if bal else (None, None)
            if not bal:
                # Abgesetzt = kann getragen werden (Nutzer 2026-11-22): Startbild weg, ein neues erst nach dem Aufspringen
                if getattr(self, "startbild", None) is not None:
                    self.startbild.anker_weg("abgesetzt oder gefallen")
                self._start_setzen("Absetzen")
        if self._start_due is None:
            return
        if now - self._start_due > self.START_MAX_S:
            log.info("Startpunkt: nach dem Aufspringen nicht still gestanden, bleibt")
            self._start_due = None
            return
        x, y, th = gm.pose()
        cmd = self.robot.command
        ref = self._start_still
        if (abs(cmd[0]) > 0.01 or abs(cmd[1]) > 0.01 or ref is None or math.hypot(x - ref[1], y - ref[2]) > 0.01
                or abs(wrap(th - ref[3])) > math.radians(1.0)):
            self._start_still = (now, x, y, th)
        elif now - ref[0] >= self.START_RUHE_S and getattr(self.robot, "levelled", True):
            self._start_due = None                    # erst nach der Roll-Kalibrierung (Nivellier-Impuls)
            self._start_setzen("Aufspringen")

    def _start_setzen(self, why: str) -> None:
        gm = self.gridmap
        if gm.locked or not gm.pose_ok:
            log.info("Startpunkt beim %s nicht gesetzt (%s)", why, "Karte gesperrt" if gm.locked else "Pose unbekannt")
            return
        gm.set_start()
        log.info("Startpunkt = Pose beim %s", why)

    def _pet_world(self, w: World, ds: float, now: float) -> None:
        """Was nur der Modus pet braucht; die anderen Modi zahlen dafür nichts."""
        w.ds, w.ds_measured = ds, self.odom_src == "räder"
        w.fw_mode = self.robot.state.mode
        w.height_mm = self.robot.height_mm
        w.map_ok = self.lmap is not None
        sch = self.ortung.last if self.ortung is not None else None
        if sch is not None:
            w.sound = (sch, time.monotonic() - sch.t)
        if self.stimmen is not None:
            w.stimmen = (self.stimmen.ids, self.stimmen.drain_neu())
        fr = self._imu_frame                  # Roll/Nick roh zur Bildzeit; ungültig (zu alt) = kein Wert
        if fr is not None and fr.frisch:
            w.tilt = (fr.roll_raw_deg, fr.pitch_raw_deg)

    def _pet_tick(self) -> None:
        """Wünsche des Modus pet ausführen, im Regelschritt und nur bei Änderungen."""
        b = self.behavior
        if b.want_stand_up:
            b.want_stand_up = False
            log.warning("pet: stehe von selbst auf")
            self.robot.stand_up()
        if b.want_set_down:
            # vor dem Moduswechsel: Mit want_idle im selben Schritt tauschte _step das Verhalten, bevor es das
            # Absetzen sah (timeout_s: „setzt ab“ ging verloren)
            b.want_set_down = False
            log.warning("pet: setze ab")
            self.robot.set_down()
        if b.want_idle:
            b.want_idle = False
            self.set_mode("idle")
        # Fahrbefehl (nicht Drehen): schräg gestellt (Kopfneigen) -> Roll auf 0 und einmal neu ausrichten, sonst
        # bleiben in der Totzone der Beine bis 2,6° stehen (Nutzer 2026-11-22). Kosten: ein Vergleich je Regelschritt.
        if abs(self._cmd.v) > 0.01 and self.robot.state.balancing and b.brain.roll_zurueck():
            log.info("pet: fährt, Roll zurück auf 0, neu ausrichten")
            self._pet_roll = 0.0
            try:
                self.robot.set_roll(0.0)
                self.robot.request_level()
            except OSError as e:
                log.error("pet: Roll zurückstellen fehlgeschlagen: %s", e)
        roll = b.brain.roll_deg
        if roll != getattr(self, "_pet_roll", 0.0):
            self._pet_roll = roll
            self.robot.set_roll(roll)
        det = getattr(self.det, "cfg", None)          # FakeDetector im Simulator hat keine
        if det is not None:
            if getattr(self, "_pet_det_hz", None) is None:
                self._pet_det_hz = det.max_hz
            hz = b.brain.det_hz if b.brain.det_hz is not None else self._pet_det_hz   # None: wie im Modus follow
            if det.max_hz != hz:
                det.max_hz = hz
        if self.emotions is not None:
            self.emotions.farbe = b.brain.augen       # Augenfarbe der erkannten Person (vor ihrer Begrüßung)
        for kind, name, urgent in b.brain.drain():
            if self.emotions is not None:
                try:
                    self.emotions.express(kind, name, urgent)
                except Exception as e:                # Anzeige darf die Regelung nie stören
                    log.warning("pet: %s %s: %s", kind, name, e)
        if b.want_ruhe:                               # zuletzt: danach schreibt in diesem Schritt nichts mehr
            grund, b.want_ruhe = b.want_ruhe, None
            self._pet_ruhe(grund)

    def _pet_leave(self) -> None:
        """Beim Verlassen von pet: Höhe, Roll und Detektorrate zurück."""
        b = self.behavior
        if b.name != "pet":
            return
        if self.emotions is not None:
            self.emotions.farbe = None
        det = getattr(self.det, "cfg", None)
        if det is not None and getattr(self, "_pet_det_hz", None) is not None:
            det.max_hz, self._pet_det_hz = self._pet_det_hz, None
        self._pet_roll = 0.0
        if not self.robot.state.balancing:
            return
        try:
            if b.brain.base_h is not None:
                self.robot.set_height(b.brain.base_h)
            if b.brain.rolled:
                self.robot.set_roll(0.0)
                self.robot.request_level()            # Totzone der Beine, sonst bleiben bis 2,6° stehen
        except OSError as e:
            log.error("pet: Zurückstellen fehlgeschlagen: %s", e)

    def _emotion_tick(self, person: bool | None) -> None:
        if self.emotions is not None and not self.ruhe:
            try:
                brain = getattr(self.behavior, "brain", None)          # pet: sucht oder streift
                # pet: den Akku zeigt pet selbst (hungrig, erschöpft), kein shy; dudeln nur mit genug Energie
                pet = self.behavior.name == "pet"
                self.emotions.update(time.monotonic(), self.behavior.name, person,
                                     None if pet else self.robot.state.battery, self.robot.state.mode,
                                     sucht=(getattr(brain, "task", None) is not None
                                            and getattr(brain, "dudeln_ok", True)))
            except Exception as e:                  # Anzeige darf die Regelung nie stören
                log.warning("Emotion: %s", e)

    def _pitch_dev(self, f, ego) -> float:
        """Nickabweichung der Kamera: der reine IMU-Nick zur
        Bildzeit (imu_frame, Versatz [imu] time_offset_ms); ohne gültige IMU nur der schnelle Anteil aus dem Bild
        (langsame Neigung dann unbekannt, pitch_src "bild"). Bis 2026-11-17 lief hier ein Komplementärfilter
        (IMU-Tiefpass + Bildanteil); der Bildanteil ist jetzt nur noch Vergleich (Snapshot.nickrate)."""
        prev = self._imu_frame
        self._imu_frame = None
        if self.imu is not None:
            self.imu.refresh()                        # neueste Proben je Bild selbst lesen (statt alle 10 ms im Faden)
            if not self.imu_sync.fixed:               # fest eingetragen (RiderPi): nichts zu schätzen
                if ego.ok and ego.dt > 0:
                    self.imu_sync.add(f.t - ego.dt, f.t, ego.pitch)
                self.imu_sync.update(self.imu, time.monotonic())
            self._imu_frame = imu_frame(self.imu_sync, self.imu, f.t)
            fr = self._imu_frame
            if (ego.ok and ego.dt > 0 and fr.pitch is not None and prev is not None and prev.pitch is not None
                    and 0 < fr.t_imu - prev.t_imu < 0.5):
                d = fr.t_imu - prev.t_imu
                self._nickrate = (math.degrees(ego.pitch / ego.dt), math.degrees((fr.pitch - prev.pitch) / d))
            else:
                self._nickrate = None
        if self._imu_frame is not None and self._imu_frame.pitch is not None:
            self.pitch_src = "imu"
            self.geo.pitch_sigma_deg = self.cfg.imu.sigma_deg
            return self._imu_frame.pitch
        if self.pitch_src != "bild" and self.imu is not None:
            log.warning("IMU-Nick fehlt (Probe zu alt oder Sync unsicher): Nick nur noch aus dem Bild")
        self.pitch_src = "bild"
        self.geo.pitch_sigma_deg = 1.5
        return ego.pitch_dev

    def _plan_tick(self, w: World) -> None:
        """Verhalten mit Planung: fertiges Ergebnis übergeben, bei Bedarf neu planen (eigener Thread)."""
        b = self.behavior
        r = self.planner.poll()
        if r is not None and r[0] == self._plan_seq and b.pending:
            b.set_path(r[1])
        if b.want_plan(w.t):
            self._plan_seq = b.request_plan(self.planner, self.gridmap, w, self._lmap_world(w.pose))
        self.gridmap.plan = list(b.path) if b.path else []
        self.gridmap.erwartet = getattr(b, "ziel_anzeige", None)
        if b.name == "explore" and getattr(b, "done", False) and not self.gridmap.locked:
            self.gridmap.lock()                       # fertig kartiert: Fahrten ändern die Karte nicht mehr
            b.save_map = True
        if getattr(b, "save_map", False):
            b.save_map = False
            if self.cfg.grid.file:
                threading.Thread(target=self.gridmap.save, args=(self.cfg.grid.file,), name="karte-speichern", daemon=True).start()

    def _luecke_eintragen(self) -> None:
        """Erkunden hat ein Ziel an einer Lücke gesperrt (explore.gap_block_s): bestätigte Punkte der Merkkarte bis
        explore.gap_mark_r_m um den Rider gelten in der Raumkarte als belegt (GridMap.mark_blocking). Die Raumkarte
        kannte Stuhlbeine oft nur als einzelne Flecken, die Merkkarte aus der Nähe alle (RiderPi 2026-11-23); so
        plant er nicht wieder durch dieselbe Lücke. Eigener Faden: confirmed_points kann am Pi ~100 ms dauern."""
        r = self.cfg.explore.gap_mark_r_m
        gm = self.gridmap
        if r <= 0 or gm is None or self.lmap is None:
            return
        pose = gm.pose()
        points = self.lmap.confirmed_later(r)

        def run() -> None:
            p = points()
            if len(p):
                n = gm.mark_blocking(np.column_stack(to_world(pose, p[:, 0], p[:, 1])))
                log.info("Erkunden: Lücke gesperrt, %d Zellen aus der Merkkarte in die Raumkarte", n)
        threading.Thread(target=run, name="luecke-eintragen", daemon=True).start()

    def _lmap_world(self, pose):
        """Funktion für den Planer-Thread: bestätigte Punkte der Merkkarte (dieselben, die die Bahnprüfung
        bremsen) in Welt-m, oder None. Bei vielen Punkten dauert das am RiderPi um 100 ms, zu lang für die
        Hauptschleife."""
        if self.lmap is None or not self.cfg.home.use_localmap:
            return None
        points = self.lmap.confirmed_later(self.cfg.home.localmap_reach_m)
        def world() -> np.ndarray | None:
            p = points()
            if not len(p):
                return None
            return np.column_stack(to_world(pose, p[:, 0], p[:, 1]))
        return world

    def _map_view(self, free: FreeSpace, tracks) -> FreeSpace:
        return mask_movable(free, [(t.cls, t.box) for t in tracks], self.cfg.camera.width, self.cfg.camera.height)

    def _path_free(self, pts, pose, tight: bool) -> float:
        """Freie Strecke (m) entlang des Wegs pts (Welt-m, ab der Pose) laut Merkkarte, inf frei; tight: Radius
        der Engstelle wie in der Bahnprüfung."""
        p = np.asarray(pts, np.float64).reshape(-1, 2)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = p[:, 0] - pose[0], p[:, 1] - pose[1]
        r = np.column_stack([dx * c + dy * s, -dx * s + dy * c])
        return self.lmap.path_clearance(r, radius=self.cfg.map.radius_tight_m if tight else None)

    def _arc_free(self, v: float, w: float, tight: bool) -> float:
        """Freie Strecke (m) auf dem Bogen des Fahrbefehls (normiert) bis zur Reichweite der Merkkarte, gerechnet wie in
        _arc_check (schnelleres von Befehl und Rädern, Drehrate mitskaliert, gleicher Radius)."""
        vm = v * self.cfg.robot.vx_limit
        if abs(vm) < 1e-3:
            return math.inf
        wr = w_to_yaw_rate(w, self.cfg.robot)
        ist, lauf = self._v_rad, abs(vm)
        if ist is not None and ist * vm > 0 and abs(ist) > lauf:
            wr *= abs(ist) / lauf
            lauf = abs(ist)
        reach = self.lmap.max_range
        # immer mit dem Radius der Engstelle: Mit map.radius_m bremste er schon weit vor der 25-cm-Tür auf
        # Schritttempo (streift die Rahmen), sah sie lange aus der Nähe, und die Raumkarte schloss sie
        # (Simulator spalt 2026-11-24, 0 von 3 Läufen). Seitliches Streifen bleibt Sache der Bahnprüfung.
        return self.lmap.arc_clearance(math.copysign(lauf, vm), wr, horizon_s=reach / lauf,
                                       steps=max(10, int(math.ceil(reach / 0.05))),
                                       radius=self.cfg.map.radius_tight_m)

    def _clear_ahead(self) -> float:
        """Freie Strecke geradeaus laut Merkkarte (m, bis 0,3 m geprüft, sonst inf); nur beim Auslösen eines Anschubs."""
        return self.lmap.arc_clearance(0.3, 0.0, horizon_s=1.0)

    def _arc_check(self, cmd: Command) -> Command:
        """Geplanten Bogen gegen die Hinderniskarte prüfen (auch außerhalb des Sichtfelds)."""
        bogen_w(cmd, self.robot, self.cfg.robot)
        if self.lmap is None or abs(cmd.v) < 1e-3 or self.cfg.behavior.bremsen_aus:
            return cmd
        v = cmd.v * self.cfg.robot.vx_limit
        wr = w_to_yaw_rate(cmd.w, self.cfg.robot)
        gm = self.gridmap
        zonen = gm.all_zones() if gm is not None else None
        if zonen and arc_hits_zone(gm.pose(), v, wr, zonen, self.lmap.radius):
            cmd.v = 0.0
            cmd.status += " | Sperrzone"
            return cmd
        if cmd.extra.get("anschub"):
            return cmd              # Strecke begrenzt der Wegfolger selbst (freie Bahn der Merkkarte, Bodenmodell)
        # Gerechnet wird mit dem, was schneller ist: Befehl oder Räder. Der Rider folgt einem kleineren Fahrwert
        # erst nach ~1-2 s und fährt so lange schneller als befohlen; Bahn und Bremsabstand gelten dann für das
        # echte Tempo, auf demselben Bogen (Drehrate mitskaliert).
        ist = self._v_rad
        lauf = abs(v)
        if ist is not None and ist * v > 0 and abs(ist) > lauf:
            wr *= abs(ist) / lauf
            lauf = abs(ist)
        vl = math.copysign(lauf, v)
        # Über 0,3 m/s (nur explore mit explore.v) mehr Abstand: Latenz und Nachlaufen kosten dann doppelt so
        # viel Weg; im Simulator berührte er sonst 1-2 Mal je 300 s ein Hindernis mit Resttempo (2026-11-11)
        need = 0.6 * lauf + 0.08 + ARC_FAST_K * max(0.0, lauf - ARC_FAST_MPS)
        # auch rückwärts: die Karte kennt Hindernisse hinter dem Rider (vorher gesehen)
        # Engstelle (Wegfolger): kleinerer Radius, sonst bremst die Bahnprüfung vor jeder Lücke unter ~0,36 m
        # Vorschau mindestens 0,15 m über den Bremsabstand hinaus, sonst meldet sie ein Hindernis erst, wenn er
        # schon bremsen müsste (bis 2026-11-23 1,2 s x Tempo, bei 0,6 m/s kaum mehr als need)
        hits: list = []
        clear = self.lmap.arc_clearance(vl, wr, horizon_s=max(1.2, (need + 0.15) / lauf),
                                        radius=self.cfg.map.radius_tight_m if cmd.extra.get("tight") else None,
                                        hit_out=hits)
        if clear < need:
            k = max(0.0, (clear - 0.05) / need)
            # erlaubt aus dem schnelleren von Befehl und Rädern: Senkt der Wegfolger den Befehl schon früh
            # (home.free_slow), während die Räder noch nachlaufen, wäre es sonst eine Vollbremsung, im
            # Engstellenmodus samt Bremsstelle (Simulator spalt 2026-11-24: Tür in 0 von 8 Läufen)
            erlaubt = lauf * k
            if ist is not None and abs(ist) > erlaubt + ARC_BRAKE_TOL_MPS:
                cmd.extra["bremsen"] = True
                cmd.status += " | Bahn %.2fm, bremse" % clear
                if self.robot.vel:
                    # Fahrrahmen: die Firmware verzögert mit vel_brake auf das erlaubte Tempo, er rollt ohne Halt bis
                    # an die Grenze (mit 0 hielt er schlagartig und fuhr dann langsam wieder an, Wand 2026-11-25)
                    cmd.v = math.copysign(min(abs(cmd.v), erlaubt / self.cfg.robot.vx_limit), cmd.v)
                else:
                    cmd.v = 0.0             # Byteweg: nur 0 setzt der Rider schnell um
            else:
                cmd.v *= k
                cmd.status += " | Bahn %.2fm" % clear
            if gm is not None and hits:
                p = hits[0]
                pw = np.column_stack(to_world(gm.pose(), p[:, 0], p[:, 1]))
                if self.cfg.grid.mark_blocking:
                    # Wofür er bremst, kommt in die Raumkarte (wenige Punkte, einmal je Regelschritt beim Bremsen)
                    gm.mark_blocking(pw)
                if (cmd.extra.get("bremsen") and cmd.extra.get("tight") and self.cfg.grid.brake_block_s > 0
                        and (ist is None or abs(ist) <= self.cfg.grid.brake_block_max_mps)):
                    # Vollbremsung schon mit dem Radius der Engstelle: passt nicht, auf Zeit sperren
                    # (grid.brake_block_s); Freistrahlen öffnen sie nicht wieder
                    neu = gm.bremse_sperren(pw, time.monotonic() + self.cfg.grid.brake_block_s)
                    if neu:
                        log.info("Bremsstelle in Engstelle gesperrt: %d Zellen um (%.2f, %.2f) für %.0f s, jetzt %d",
                                 neu, float(pw[:, 0].mean()), float(pw[:, 1].mean()),
                                 self.cfg.grid.brake_block_s, len(gm.brems))
                        neu_planen = getattr(self.behavior, "neu_planen", None)
                        if neu_planen is not None:
                            neu_planen()
        return cmd



def bogen_w(cmd: Command, robot, rcfg) -> None:
    """Krümmungsbetrieb (robot.bogen): w aus κ und dem jetzigen v, damit jede Schicht, die v senkt (safety(),
    Bahnprüfung), den Bogen behält statt ihn enger zu machen. Ohne Krümmungsbetrieb bleibt alles wie bisher."""
    if cmd.kappa is not None and getattr(robot, "bogen", False):
        cmd.w = cmd.kappa * cmd.v * rcfg.vx_limit / math.radians(rcfg.vyaw_limit_dps)

# ---------------------------------------------------------------------------
_ASCII = str.maketrans({"–": "-", "—": "-", "°": "deg", "µ": "u", "ä": "ae", "ö": "oe", "ü": "ue",
                        "Ä": "Ae", "Ö": "Oe", "Ü": "Ue", "ß": "ss", "≈": "~", "→": "->"})


def _ascii(s: str) -> str:
    """cv2.putText kann nur ASCII (Hershey-Fonts), alles andere wird zu '?'."""
    return s.translate(_ASCII).encode("ascii", "replace").decode()


def net_rows_jetzt(snap: Snapshot) -> np.ndarray:
    """FloorNet-Grenzzeilen auf das Bild des Snapshots umgerechnet: Das Netz rechnet im eigenen Thread, sein
    Ergebnis gehört zu einem 1-3 Bilder älteren Bild. Über die Bodenebene mit Nick und Wegsumme der Aufnahme
    (ZeilenGlaetter.umrechnen). Nur zum Zeichnen (auf Anfrage), nichts entscheidet damit. NaN: keine Aussage."""
    fs = snap.free
    rows = np.asarray(fs.net_rows, np.float32)
    if snap.geo is None or fs.pitch_dev is None:
        return rows
    from .floornet import ZeilenGlaetter
    p0 = fs.odo if fs.odo is not None and snap.odo is not None else (0.0, 0.0, 0.0)
    p1 = snap.odo if fs.odo is not None and snap.odo is not None else (0.0, 0.0, 0.0)
    if p0 == p1 and fs.pitch_dev == snap.pitch_dev:
        return rows
    try:
        neu = ZeilenGlaetter.umrechnen(rows, p0, fs.pitch_dev, p1, snap.pitch_dev, snap.geo)
    except Exception:
        return rows
    # am Horizont oder aus dem Bild gedreht (keine Bodenaussage): mit dem Horizont mitschieben statt Lücke
    dh = snap.geo.horizon_row(snap.pitch_dev) - snap.geo.horizon_row(fs.pitch_dev)
    return np.where(np.isfinite(neu), neu, rows + dh).astype(np.float32)


def render(snap: Snapshot, text: bool = True, horizont: tuple[float, float, float] | None = None) -> np.ndarray | None:
    """Debug-Overlay zeichnen (nur auf Anfrage, kostet Zeit). text=False lässt die
    Statuszeilen weg (~10 ms je Bild am CM4); die Weboberfläche zeigt sie als HTML.
    horizont: (Zeile in der Bildmitte cx, Roll in Grad, cx) für die cyan Horizontlinie, Vollbild-Pixel
    (horizont_linie); ohne sie die Horizontzeile des Bodenmodells, waagerecht und auf 20..80 % geklemmt."""
    if snap.frame is None:
        return None
    img = snap.frame.copy()
    H, W = img.shape[:2]
    if horizont is not None:
        row, roll, cx = horizont
        if math.isfinite(row) and abs(roll) < 80:
            k = math.tan(math.radians(roll))      # Roll + = rechts tief: Horizont rechts höher im Bild
            y0, y1 = row + cx * k, row - (W - cx) * k
            if max(y0, y1) > -H and min(y0, y1) < 2 * H:     # sonst weit außerhalb (cv2 rechnet in int)
                cv2.line(img, (0, int(round(y0))), (W, int(round(y1))), (255, 255, 0), 1, cv2.LINE_AA)
    fs = snap.free
    if fs is not None and fs.columns is not None:
        if horizont is None:
            hy = int(fs.horizon_row * W / len(fs.columns))      # Arbeitsauflösung -> Bild (gleiches Seitenverhältnis)
            cv2.line(img, (0, hy), (W, hy), (255, 255, 0), 1)
        n = len(fs.sectors)
        for i, s in enumerate(fs.sectors):
            x0, x1 = int(i * W / n), int((i + 1) * W / n)
            col = (0, int(255 * s), int(255 * (1 - s)))
            cv2.rectangle(img, (x0 + 2, H - 8), (x1 - 2, H - 2), col, -1)
    if fs is not None and fs.net_rows is not None:       # FloorNet
        rows = net_rows_jetzt(snap)
        n = len(rows)
        for a, b in column_runs(rows):
            pts = [(int((i + 0.5) * W / n), int(round(rows[i]))) for i in range(a, b)]
            cv2.polylines(img, [np.array(pts, np.int32)], False, (255, 0, 255), 2)
    for t in snap.tracks:
        x1, y1, x2, y2 = map(int, t.box)
        tgt = t.id == snap.cmd.target_id
        col = (0, 0, 255) if tgt else (0, 200, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 3 if tgt else 1)
        cv2.putText(img, f"#{t.id} {t.who or t.cls} {t.conf:.2f}", (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    e = snap.ego
    if e is not None:
        c = (W // 2, H // 2)
        cv2.arrowedLine(img, c, (int(c[0] + e.dx * 3), int(c[1] + e.dy * 3)), (255, 0, 255), 2)
    v, w = snap.sent
    cv2.rectangle(img, (W - 20, H // 2), (W - 10, int(H // 2 - v * H)), (255, 128, 0), -1)
    cv2.rectangle(img, (W // 2, 4), (int(W // 2 - w * W), 12), (255, 128, 0), -1)
    if not text:
        return img
    lines = [snap.cmd.status,
             "fps %s loop %sms det %sms | nick %s %+.1f°" % (
                 snap.stats.get("fps"), snap.stats.get("loop_ms"), snap.stats.get("det_ms"),
                 snap.stats.get("pitch_src", "?"), snap.stats.get("pitch_deg", 0.0))]
    if e is not None:
        lines.append("yaw %+.0f°/s pitch %+.0f°/s ttc %s sharp %.0f" % (
            math.degrees(e.yaw_rate), math.degrees(e.pitch_rate),
            "%.1fs" % e.ttc if e.ttc < 99 else "-", e.sharpness))
    for i, s in enumerate(lines):
        s = _ascii(s)
        cv2.putText(img, s, (6, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, s, (6, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img
