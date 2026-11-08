"""Hauptschleife: Kamera -> Ego-Motion -> Tracker/Detektor -> Freiraum -> Verhalten -> Robot."""
from __future__ import annotations

import collections
import dataclasses
import gc
import logging
import math
import os
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
from .floornet import FloorNet
from .freespace import FreeSpace, FreeSpaceEstimator, FreeSpaceWorker, column_runs
from .gridmap import GridMap, GridWorker
from .planner import PlannerWorker
from .geometry import CamGeometry
from .horizon import HorizonTable, StandRef, row_to_tilt_deg
from .wheels import WheelOdom, make_wheels
from .imu import ImuSource, ImuSync, make_imu
from .localmap import LocalMap, blocking_boxes, mask_movable
from .route import arc_hits_zone, check as route_check, load as route_load, save as route_save
from .robot import Robot
from .tracker import Tracker

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
    fs_diag: dict = field(default_factory=dict)      # Diagnose des Bodenmodells
    cam_meta: dict | None = None                     # Weißabgleich, Verstärkung
    horizon: dict | None = None                      # Horizont im Stand ([horizon] enabled)


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

class App:
    def __init__(self, cfg: Config, robot: Robot, source: FrameSource | None = None,
                 imu: ImuSource | None = None, wheels: WheelOdom | None = None):
        self.cfg = cfg
        self.imu = imu if imu is not None else make_imu(cfg.imu)
        self.wheels = wheels if wheels is not None else make_wheels(cfg.odom)
        self._wheel_at_ctrl = None         # Radprobe beim letzten Regelschritt
        self.odom_src = "befehl"
        self._wheel_dyaw: float | None = None   # Drehung laut Rädern im letzten Regelschritt
        self._grid_yaw: tuple[str, float] | None = None   # (Quelle, letzter IMU-Kurs rad) für die Raumkarte
        self.grid_yaw_src = "odom"
        self._lmap_yaw: float | None = None     # letzter Kurs aus imu_log (rad) für die Merkkarte
        self.lmap_yaw_src = "odom"
        self.odom_m = 0.0                  # gefahrene Strecke laut Odometrie (Betrag, seit Start)
        self.imu_sync = ImuSync(cfg.imu)
        self._imu_lp: float | None = None
        self.pitch_src = "bild"
        self.last_pitch_dev = 0.0
        self.robot = robot
        self.src = source or make_source(cfg.camera)
        size = (cfg.camera.width, cfg.camera.height)
        self.hfov = math.radians(cfg.camera.hfov_deg)
        self.ego = EgoMotion(cfg.ego, size, cfg.camera.hfov_deg)
        self.tracker = Tracker(cfg.tracker, size)
        self.det = DetectorWorker(cfg.detector, cfg.cpu.detector_cpus)
        self.geo = CamGeometry.from_cfg(cfg)
        # Horizont im Stand: IMU-Referenz je Aufstehen, Horizontzeile je Körperhöhe (horizon.py)
        self.stand = StandRef(cfg.horizon) if cfg.horizon.enabled else None
        self.htable = HorizonTable(cfg.horizon.file) if cfg.horizon.enabled else None
        self.hz_edit_row: float | None = None    # Linie, die der Nutzer im Web gerade einstellt
        self._hz_bild: float | None = None       # IMU-Wert beim letzten Abgleich + Bildnick seitdem (rad)
        self._hz_bild_t = 0.0                    # Bildzeit des letzten Abgleichs mit der IMU
        self._hz_ok = collections.deque(maxlen=100)   # Ego-Motion ok je Bild
        self._hz_info: dict | None = None
        self.fse = FreeSpaceEstimator(cfg.freespace, size, self.geo) if cfg.freespace.enabled else None
        shadow = None
        if self.fse is not None and cfg.freespace.shadow_net:
            if cfg.freespace.threaded:
                shadow = FloorNet.load(cfg.freespace.shadow_net)
            else:
                log.warning("[floornet] Schattenbetrieb braucht freespace.threaded: aus")
        self.fs_worker = FreeSpaceWorker(self.fse, cfg.cpu.freespace_cpus, shadow) \
            if self.fse is not None and cfg.freespace.threaded else None
        m = cfg.map
        self.lmap = LocalMap(self.geo, self.fse.W, self.fse.H, max_age_s=m.max_age_s, confirm=m.confirm,
                             confirm_radius_m=m.confirm_radius_m) if self.fse else None
        self._fs_in_map = None
        # Raumkarte (Welt-KS ab Startpose), nur Anzeige
        self.gridmap = GridMap(cfg.grid, self.geo, self.fse.W, self.fse.H) \
            if self.fse is not None and cfg.grid.enabled else None
        if self.gridmap is not None and cfg.grid.file and os.path.exists(cfg.grid.file):
            self.gridmap.load(cfg.grid.file)
            route_load(cfg.grid.file, self.gridmap)
        self.grid_worker = GridWorker(self.gridmap, cfg.cpu.freespace_cpus, save_path=cfg.grid.file,
                                      save_every_s=cfg.grid.save_every_s) \
            if self.gridmap is not None and cfg.grid.threaded else None
        # Planer für den Modus home (Thread startet erst beim ersten Auftrag)
        self.planner = PlannerWorker(self.gridmap, cfg.home, cfg.cpu.freespace_cpus) \
            if self.gridmap is not None else None
        self._plan_seq = 0
        self._yaw_at_ctrl = 0.0
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
        self.yaw = 0.0                   # Kurs für Verhalten und Merkkarte (Räder, sonst Bild)
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

    # ------------------------------------------------------------------
    # Diese Modi fahren auf Freiraum zu und verlassen sich auf Bodenmodell + Merkkarte.
    # Ohne Freiraum-Erkennung hielten sie alles für frei und stoppten nur per TTC.
    NEEDS_FREESPACE = {"explore", "approach", "home", "patrol"}

    def _check_mode(self, name: str) -> None:
        if name in self.NEEDS_FREESPACE and self.fse is None:
            raise ModeNotAllowed(f"Modus {name} braucht die Freiraum-Erkennung (freespace.enabled)")
        if name == "home" and getattr(self, "gridmap", None) is None:
            raise ModeNotAllowed("Modus home braucht die Raumkarte ([grid] enabled)")
        gm = getattr(self, "gridmap", None)
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

    def _make(self, name: str, target: str | None = None) -> Behavior:
        b = make_behavior(name, self.cfg.behavior, self.cfg.robot, target, self.cfg.home, self.cfg.explore,
                          has_map=getattr(self, "gridmap", None) is not None, pcfg=self.cfg.patrol)
        if name == "patrol" and getattr(self, "gridmap", None) is not None:
            b.set_route(self.gridmap.route, self.gridmap.route_loop)
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
            self.src.resume()
            self.reset_floor(hold_s=self.cfg.freespace.restart_hold_s)
            log.info("Bildverarbeitung gestartet")
        else:
            with self._mode_lock:
                self._pending_mode = ("idle", None)
            self.src.pause()
            self.robot.stop()
            log.info("Bildverarbeitung gestoppt")

    def reset_floor(self, hold_s: float = 0.0) -> None:
        """Bodenmodell neu lernen lassen; hold_s: so lange vorher kein Bodenmodell rechnen
        (nach Kamerastart sind die ersten Bilder dunkel). Bis dahin kein Freiraum-Ergebnis."""
        if self.fse is None:
            return
        self._fs_hold_until = time.monotonic() + hold_s
        self._free = None
        self.fse.reset()

    def set_manual(self, v: float, w: float) -> None:
        """Sollwert der Handsteuerung (normiert, v > 0 vor, w > 0 links). Nur im Modus manual;
        geht auch bei gestoppter Bildverarbeitung (braucht keine Kamera)."""
        b = self.behavior
        if b.name != "manual":
            raise ModeNotAllowed("Handsteuerung nur im Modus manual")
        b.set(v, w, time.monotonic())

    # Modi, die ohne Kamerabild laufen (Regelschritt dann in _paused_tick)
    NO_CAMERA = {"idle", "manual"}

    def set_mode(self, name: str, target: str | None = None, confirmed: bool = False) -> None:
        """confirmed: Der Nutzer hat bestätigt, dass der Rider am Startpunkt steht (Streife: ohne Wiederfinden
        gilt beim Laden Pose = Start)."""
        self._make(name, target)  # validiert (KeyError)
        self._check_mode(name)
        if name == "patrol" and not confirmed:
            raise ModeNotAllowed("Streife: bitte bestätigen, dass der Rider am Startpunkt steht")
        if name == "patrol":
            self.gridmap.lock()
            self.gridmap.relocate_to_start()
        if not self.pipeline_on and name not in self.NO_CAMERA:
            raise ModeNotAllowed(f"Modus {name} braucht laufende Bildverarbeitung")
        with self._mode_lock:
            self._pending_mode = (name, target)

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
        if self.fs_worker is not None:
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
            self.robot.stop()
            if self.imu is not None:
                self.imu.close()
            self.det.close()
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
    def _apply_pending_mode(self) -> None:
        with self._mode_lock:
            if self._pending_mode:
                name, target = self._pending_mode
                self._pending_mode = None
                self.behavior = self._make(name, target)
                if self.gridmap is not None:
                    self.gridmap.plan = []
                log.info("Modus -> %s %s", name, target or "")

    def _drive(self, cmd: Command) -> None:
        """Fahrbefehl an den Roboter; der Vorwärtsfaktor gilt nur in der Handsteuerung."""
        self.robot.forward_gain = self.cfg.robot.manual_forward_gain if self.behavior.name == "manual" else 1.0
        self.robot.drive(cmd.v, cmd.w)

    def _manual_cmd(self) -> Command:
        """Handsteuerung: nur Sturz-Erkennung, kein Bodenmodell/keine Bahnprüfung."""
        cmd = self.behavior.step(None)
        if not self.robot.state.balancing:
            cmd = Command(0.0, 0.0, None, "GEFALLEN – Stopp")
        return cmd

    def _paused_tick(self) -> None:
        """Hauptschleife bei gestoppter Bildverarbeitung: Moduswechsel sofort übernehmen;
        im Modus manual mit control_hz regeln (ohne Kamera), sonst Stillstand."""
        self._apply_pending_mode()
        self._paused_odom()
        if self.behavior.name not in self.NO_CAMERA:
            self.behavior = self._make("idle")
        if self.behavior.name != "manual":
            self._cmd = Command(0.0, 0.0, None, "Bildverarbeitung gestoppt")
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
        time.sleep(max(0.0, self._last_ctrl + period - time.monotonic()))

    def _step(self, f) -> None:
        tm = _Marks()
        self._apply_pending_mode()

        excl = [t.box for t in self.tracker.tracks]
        ego = self.ego.update(f.idx, f.t, f.bgr, excl)
        if ego.ok:
            self.vis_yaw += ego.yaw_rate * ego.dt
        pitch_dev = self._pitch_dev(f, ego)
        self.last_pitch_dev = pitch_dev
        self.tracker.predict(ego.A, ego.dt)
        tm.mark("ego")

        batch = self.det.poll()
        if batch is not None:
            self._det_count += 1
            T = self.ego.transform_between(batch.frame_idx, f.idx)
            if T is not None:
                dets = [Detection(warp_box(T, d.box), d.cls, d.conf) for d in batch.dets]
                self.tracker.update(dets, f.t, lag=f.t - batch.t)
            else:
                log.debug("Detektion zu alt, verworfen")
        self.tracker.prune(f.t)
        self.det.offer(f, ego.sharpness, self.ego.sharpness_median())
        tm.mark("det")

        bal = self.robot.state.balancing
        if bal and self._was_balancing is False:
            # aufgestanden: Kamera sieht den Boden jetzt aus anderer Höhe/Lage. Ein im Liegen
            # gelerntes Modell passte am RiderPi danach nie mehr (Nachführung nur bei passender
            # Probe) und meldete überall "blockiert" (2026-11-04).
            log.info("Rider steht auf: Bodenmodell lernt neu")
            self.reset_floor(hold_s=self.cfg.freespace.restart_hold_s)
        self._was_balancing = bal
        if time.monotonic() < self._fs_hold_until:
            pass                          # nach Kamerastart: Belichtung abwarten, nichts lernen
        elif self.fs_worker is not None:
            # eigener Thread: neuestes Ergebnis übernehmen, jedes 2. Bild einen Auftrag geben
            r = self.fs_worker.poll()
            if r is not None:
                self._free = r
            if f.idx % 2 == 0 or self._free is None:
                self.fs_worker.offer(f.bgr, pitch_dev, [t.box for t in self.tracker.tracks],
                                     self.gridmap.pose() if self.gridmap is not None else None)
        elif self.fse is not None and (f.idx % 2 == 0 or self._free is None):
            self._free = self.fse.update(f.bgr, pitch_dev, [t.box for t in self.tracker.tracks])
            self._free.pose = self.gridmap.pose() if self.gridmap is not None else None
            tm.fs_ran = True
        tracks = self.tracker.confirmed(f.t)
        tm.mark("fs")

        now = time.monotonic()
        if now - self._last_ctrl >= 1.0 / self.cfg.control_hz:
            dt = now - self._last_ctrl if self._last_ctrl else 0.0
            self._last_ctrl = now
            free = self._free
            if free is not None and self.fse is not None:
                # Objekte als Hindernisse, außer dem aktuellen Ziel
                tgt = self._cmd.target_id
                free = dataclasses.replace(free, sectors=list(free.sectors), columns=free.columns.copy())
                self.fse.block_boxes(free, blocking_boxes(tracks, tgt, f.t, self.cfg.freespace.no_block_cls))
            # Eigenbewegung seit dem letzten Regelschritt. Kurs aus den Rädern, wenn vorhanden:
            # Der Bild-Kurs verfälscht bei Vorwärtsfahrt (Tiefenunterschiede landen im
            # Drehmodell, gemessen 2026-11-04: −8,9° bzw. +3° auf 2 m geradeaus).
            ds = self._odom_ds(now, dt)
            dyaw_vis = self.vis_yaw - self._yaw_at_ctrl
            self._yaw_at_ctrl = self.vis_yaw
            use_wheels = self.cfg.odom.yaw_source == "räder" and self._wheel_dyaw is not None
            dyaw = self._wheel_dyaw if use_wheels else dyaw_vis
            self.yaw += dyaw
            self.yaw_src = "räder" if use_wheels else "kamera"
            w = World(t=f.t, dt=dt, W=self.cfg.camera.width, H=self.cfg.camera.height, hfov=self.hfov,
                      ego=ego, tracks=tracks, free=free, balancing=self.robot.state.balancing,
                      yaw=self.yaw, cmd_v=self.robot.command[0],
                      geo=self.geo, pitch_dev=pitch_dev, battery=self.robot.state.battery)
            if self.lmap is not None:
                self.lmap.move(ds, self._lmap_dyaw(dyaw, now))
                if self.gridmap is not None:
                    # Kartenkurs: Räder driften bei Drehungen weniger als die Ego-Motion
                    gyaw = self._wheel_dyaw if (self.cfg.grid.yaw_source != "kamera"
                                                and self._wheel_dyaw is not None) else dyaw_vis
                    self.gridmap.move(ds, self._grid_dyaw(gyaw, now))
                if free is not None and self._free is not self._fs_in_map:
                    self._fs_in_map = self._free
                    view = self._map_view(free, tracks, f.t)
                    # Nick und Pose der Aufnahme, nicht des Bildes, in dem das Ergebnis ankam
                    pd = view.pitch_dev if view.pitch_dev is not None else pitch_dev
                    self.lmap.add_freespace(view, f.t, pd)
                    # Raumkarte nur auf den Rädern (liegend sieht die Kamera etwas anderes) und
                    # ohne schnelle Drehung
                    rate = abs(math.degrees(dyaw)) / dt if dt > 0 else 0.0
                    if self.robot.state.balancing and rate <= self.cfg.grid.max_turn_dps:
                        if self.grid_worker is not None:
                            self.grid_worker.offer(view, pd)
                        elif self.gridmap is not None:
                            self.gridmap.add_freespace(view, pd, view.pose)
            if self.gridmap is not None:
                if self.grid_worker is None:
                    self.gridmap.maybe_save(self.cfg.grid.file, self.cfg.grid.save_every_s)
                w.pose, w.goal = self.gridmap.pose(), self.gridmap.start
                if hasattr(self.behavior, "want_plan"):
                    self._plan_tick(w)
            # approach erkundet mit derselben Logik, bis das Ziel in Sicht ist
            if self.lmap is not None and self.behavior.name in ("explore", "approach"):
                vg = self.cfg.behavior.cruise_v * self.cfg.robot.vx_limit
                w.path_blocked = self.lmap.arc_clearance(vg, 0.0, horizon_s=1.2) < 0.6 * vg + 0.08
            if getattr(self.behavior, "want_set_down", False):
                self.behavior.want_set_down = False
                log.warning("Akku leer: setze ab")
                self.robot.set_down()
            if self.behavior.name == "manual":
                cmd = self._manual_cmd()
            else:
                cmd = safety(self.behavior.step(w), w, self.cfg.behavior)
                cmd = self._arc_check(cmd)
            self._drive(cmd)
            if cmd.height_mm is not None and now - self._last_height_t > 1.0:
                self.robot.set_height(cmd.height_mm)
                self._last_height_t = now
            if self.robot.height_mm is not None:
                # auch von außen gesetzt (Web) oder nach dem Aufstehen, siehe Robot._poll_height/_height_tick
                self.geo.body_height_mm = self.robot.height_mm
            self._cmd = cmd
            self._free_ctrl = free
            cls = self.cfg.behavior.target_class
            self._emotion_tick(any(t.cls == cls and t.lost_s(f.t) < 0.5 for t in tracks))
        tm.mark("ctrl")

        snap = Snapshot(frame=f.bgr, idx=f.idx, t=f.t, pitch_dev=pitch_dev, tracks=list(tracks), horizon=self._hz_info,
                        tracks_all=list(self.tracker.tracks), cam_meta=f.meta,
                        fs_diag=dict(self.fse.diag) if self.fse is not None else {},
                        free=self._free_ctrl or self._free,
                        ego=ego, cmd=self._cmd, sent=self.robot.command,
                        stats={"fps": round(self.src.fps_measured, 1), "loop_ms": round(self._loop_ms, 1),
                               **self._loop_spread(),
                               "det_ms": round(self.det.infer_ms, 1), "det_batches": self._det_count,
                               "plan_ms": round(self.planner.ms, 1) if self.planner is not None else None,
                               "explore": self.behavior.info() if self.behavior.name == "explore" and hasattr(self.behavior, "info") else None,
                               "patrol": self.behavior.info() if self.behavior.name == "patrol" else None,
                               "fs_ms": round(self.fs_worker.ms, 1) if self.fs_worker is not None else None,
                               "net_ms": round(self.fs_worker.net_ms, 1) if self.fs_worker is not None and self.fs_worker.net is not None else None,
                               "grid": None if self.gridmap is None else {
                                   **self.gridmap.stats(),
                                   "ms": round(self.grid_worker.ms, 1) if self.grid_worker else None,
                                   "verworfen": self.grid_worker.dropped if self.grid_worker else 0},
                               "odom": {"src": self.odom_src, "m": round(self.odom_m, 2),
                                        "hz": round(self.wheels.rate_hz, 1) if self.wheels else None},
                               "mode": self.behavior.name, "yaw_vis_deg": round(math.degrees(self.vis_yaw), 1),
                               "yaw_deg": round(math.degrees(self.yaw), 1), "yaw_src": self.yaw_src, "lmap_yaw_src": self.lmap_yaw_src,
                               "grid_yaw_src": self.grid_yaw_src,
                               "yaw_drv": self.robot.state.yaw_deg, "battery": self.robot.state.battery,
                               "balancing": self.robot.state.balancing,
                               "height_mm": round(self.geo.body_height_mm, 1),
                               "pitch_src": self.pitch_src, "pitch_deg": round(math.degrees(pitch_dev), 2),
                               "horizon": self._hz_info,
                               "imu": None if self.imu is None else {
                                   "hz": round(self.imu.rate_hz, 1), "sync": self.imu_sync.reliable,
                                   "sign": self.imu_sync.sign, "offset_ms": round(self.imu_sync.offset * 1000),
                                   "corr": round(self.imu_sync.corr, 2)}})
        with self.snap_cond:
            self.snapshot = snap
            self.snap_cond.notify_all()
        tm.mark("snap")
        self._seg_hist.append(tm.done())


    def _paused_odom(self) -> None:
        """Pose auch bei gestoppter Bildverarbeitung mitführen (Handsteuerung, Balancier-Drift).
        Sonst stand die Raumkarte still, während der Rider per Hand fuhr, und „Zum Start“
        zielte danach daneben. Nur mit frischen Raddaten: ohne Kamera gibt es sonst keinen Kurs."""
        now = time.monotonic()
        w = self.wheels
        if w is None or self.cfg.odom.source == "command" or not w.fresh(now):
            self._wheel_at_ctrl = None
            return
        ds = self._odom_ds(now, 0.0)
        dyaw = self._wheel_dyaw or 0.0
        self.yaw += dyaw
        if self.lmap is not None:
            self.lmap.move(ds, self._lmap_dyaw(dyaw, now))
        if self.gridmap is not None:
            self.gridmap.move(ds, self._grid_dyaw(dyaw, now))

    def _lmap_dyaw(self, fallback: float, now: float) -> float:
        """Drehung der Merkkarte seit dem letzten Aufruf (rad, links +): Kurs aus imu_log bei [map] yaw_source
        "imu", sonst (oder bei fehlender/alter IMU) fallback = Räder bzw. Kamera."""
        p = self.imu.latest_yaw() if self.cfg.map.yaw_source == "imu" and self.imu is not None else None
        if p is None or now - p[0] > self.cfg.imu.max_age_s:
            self._lmap_yaw, self.lmap_yaw_src = None, "odom"
            return fallback
        last, self._lmap_yaw = self._lmap_yaw, p[1]
        self.lmap_yaw_src = "imu"
        return fallback if last is None else p[1] - last

    def _grid_dyaw(self, fallback: float, now: float) -> float:
        """Drehung der Kartenpose seit dem letzten Aufruf (rad, links +) bei [grid] yaw_source "imu": Kurs aus
        imu_log (Strom 100 Hz), ist der zu alt oder fehlt, die 5-Hz-Treppe aus /proc yaw; sonst und beim ersten
        Wert bzw. Quellenwechsel fallback (Räder bzw. Kamera)."""
        if self.cfg.grid.yaw_source != "imu":
            self._grid_yaw, self.grid_yaw_src = None, "odom"
            return fallback
        p = self.imu.latest_yaw() if self.imu is not None else None
        if p is not None and now - p[0] <= self.cfg.imu.max_age_s:
            cur = ("imu", p[1])
        elif self.robot.state.yaw_deg is not None:
            cur = ("status", math.radians(self.robot.state.yaw_deg))
        else:
            self._grid_yaw, self.grid_yaw_src = None, "odom"
            return fallback
        last, self._grid_yaw = self._grid_yaw, cur
        self.grid_yaw_src = cur[0]
        if last is None or last[0] != cur[0]:
            return fallback
        return (cur[1] - last[1] + math.pi) % (2 * math.pi) - math.pi

    def _odom_ds(self, now: float, dt: float) -> float:
        """Strecke seit dem letzten Regelschritt (m): aus den Rad-Encodern, wenn frisch;
        sonst wie früher Fahrbefehl x vx_limit (unkalibriert). Setzt nebenbei _wheel_dyaw."""
        w = self.wheels
        if w is not None and self.cfg.odom.source != "command" and w.fresh(now):
            cur = w.latest()
            prev, self._wheel_at_ctrl = self._wheel_at_ctrl, cur
            self.odom_src = "räder"
            ds, self._wheel_dyaw = w.motion(prev, cur) if prev is not None else (0.0, 0.0)
        else:
            self._wheel_at_ctrl = None
            self._wheel_dyaw = None
            self.odom_src = "befehl"
            ds = self.robot.command[0] * self.cfg.robot.vx_limit * dt
        self.odom_m += abs(ds)
        return ds

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

    def _emotion_tick(self, person: bool | None) -> None:
        if self.emotions is not None:
            try:
                self.emotions.update(time.monotonic(), self.behavior.name, person, self.robot.state.battery)
            except Exception as e:                  # Anzeige darf die Regelung nie stören
                log.warning("Emotion: %s", e)

    def _pitch_dev(self, f, ego) -> float:
        """Nickabweichung der Kamera: mit [horizon] aus IMU minus Standreferenz; sonst absolut aus
        der IMU, wenn synchronisiert, sonst nur der schnelle Anteil aus dem Bild (langsame Neigung
        dann unbekannt)."""
        if self.imu is not None:
            if ego.ok and ego.dt > 0:
                self.imu_sync.add(f.t - ego.dt, f.t, ego.pitch)
            self.imu_sync.update(self.imu, time.monotonic())
        if self.stand is not None:
            pd = self._pitch_dev_stand(f, ego)
            if pd is not None:
                return pd
            self.geo.tilt_deg = self.cfg.camera.tilt_deg
        if self.imu is not None:
            pd = self.imu_sync.pitch_dev(self.imu, f.t)
            if pd is not None:
                # Komplementärfilter: langsamer Anteil aus der IMU (absolut), schneller aus dem
                # Bild. Gleicher Tiefpass wie in der Ego-Motion (ego.pitch_dev = Bild - Tiefpass),
                # dadurch ergänzen sich beide exakt; IMU-Abtastung (50 Hz) limitiert nicht.
                a = self.cfg.ego.smooth_alpha
                self._imu_lp = pd if self._imu_lp is None else (1 - a) * self._imu_lp + a * pd
                self.pitch_src = "imu"
                self.geo.pitch_sigma_deg = self.cfg.imu.sigma_deg
                return self._imu_lp + ego.pitch_dev
        self._imu_lp = None
        self.pitch_src = "bild"
        self.geo.pitch_sigma_deg = 1.5
        return ego.pitch_dev

    def _pitch_dev_stand(self, f, ego) -> float | None:
        """Nick gegen die Standreferenz: IMU minus Mittel im Stand (kein Tiefpass). Schnelle Änderungen aus
        dem Bild: ab dem IMU-Wert wird der Bildnick aufsummiert und spätestens nach [horizon] bild_max_s wieder
        auf die IMU gesetzt; diese Linie (magenta) geht in die Geometrie, die reine IMU (cyan) ist Vergleich.
        None: noch keine Referenz (vor dem Aufstehen, beim Einschwingen) oder IMU fehlt/veraltet."""
        now = time.monotonic()
        cmd = self.robot.command
        still = abs(cmd[0]) < 1e-3 and abs(cmd[1]) < 1e-3
        h = self.robot.height_mm if self.robot.height_mm is not None else self.geo.body_height_mm
        new = self.stand.update(now, self.robot.state.balancing, h, still, self.imu)
        self._hz_ok.append(bool(ego.ok))
        st = self.stand
        row_cal = self.htable.row_at(h)
        info = {"zustand": st.state, "hoehe_mm": round(h, 1),
                "ref_deg": None if st.ref is None else round(math.degrees(st.ref), 2),
                "std_deg": None if st.std is None else round(math.degrees(st.std), 2),
                "verworfen": st.verworfen, "row_cal": None if row_cal is None else round(row_cal, 1),
                "tabelle_deckt": self.htable.covers(h), "edit_row": self.hz_edit_row,
                "ego_ok": round(sum(self._hz_ok) / len(self._hz_ok), 2),
                "row_stand": None, "row_imu": None, "row_bild": None, "bild_minus_imu_deg": None,
                "imu_alter_ms": None, "row_imu_roh": None, "imu_roh_deg": None}
        self._hz_info = info
        pd = None
        if st.ref is not None and self.imu is not None and self.imu_sync.reliable:
            lt = self.imu.latest_t()
            t_imu = f.t + self.imu_sync.offset
            if lt is not None:
                info["imu_alter_ms"] = round((t_imu - lt) * 1000)
            p = self.imu.pitch_at(t_imu) if lt is not None and t_imu - lt <= self.cfg.imu.max_age_s else None
            if p is not None:
                pd = self.imu_sync.sign * (float(p) - st.ref)
        if pd is None:
            self._hz_bild = None
            return None
        self.geo.tilt_deg = row_to_tilt_deg(row_cal, self.geo.f, self.geo.H) if row_cal is not None \
            else self.cfg.camera.tilt_deg
        max_s = self.cfg.horizon.bild_max_s
        if new or self._hz_bild is None or max_s <= 0 or f.t - self._hz_bild_t >= max_s:
            self._hz_bild, self._hz_bild_t = pd, f.t        # Abgleich: Bildlinie auf die IMU
        elif ego.ok:
            self._hz_bild += ego.pitch                      # schnelle Änderung aus dem Bild
        info["row_stand"] = round(self.geo.horizon_row(0.0), 1)
        info["row_imu"] = round(self.geo.horizon_row(pd), 1)
        info["row_bild"] = round(self.geo.horizon_row(self._hz_bild), 1)
        info["bild_minus_imu_deg"] = round(math.degrees(self._hz_bild - pd), 2)
        # roher Nick des Treibers: neueste Probe aus imu_log, ohne Zeitabgleich zum Bild (zeigt die Reaktion)
        _, ps = self.imu.arrays()
        if len(ps):
            info["imu_roh_deg"] = round(math.degrees(float(ps[-1])), 2)
            info["row_imu_roh"] = round(self.geo.horizon_row(self.imu_sync.sign * (float(ps[-1]) - st.ref)), 1)
        self.pitch_src = "stand+bild" if max_s > 0 else "stand"
        self.geo.pitch_sigma_deg = self.cfg.imu.sigma_deg
        return self._hz_bild

    def horizon_table(self) -> dict:
        if self.htable is None:
            raise ModeNotAllowed("Horizont aus ([horizon] enabled = false)")
        # kamera_mm nur für die Anzeige; Schlüssel der Tabelle bleibt height_mm
        return {"datei": self.htable.path,
                "punkte": [{**p, "kamera_mm": round(self.geo.cam_h_at(p["height_mm"]), 1)} for p in self.htable.points]}

    def horizon_calibrate(self, row: float) -> dict:
        """Horizontzeile (Vollbild) für die Körperhöhe der Standreferenz speichern. Nur im Stand mit gültiger
        Referenz: Die Zeile gilt für die mittlere Lage, nicht für einen Augenblick beim Pendeln."""
        if self.stand is None:
            raise ModeNotAllowed("Horizont aus ([horizon] enabled = false)")
        if self.stand.state != "gültig" or self.stand.ref is None:
            raise ModeNotAllowed("erst aufstehen und still stehen lassen (Standreferenz: %s)" % self.stand.state)
        if not 0 <= row < self.cfg.camera.height:
            raise ValueError("Zeile außerhalb des Bildes")
        h = self.stand.height_mm if self.stand.height_mm is not None else self.geo.body_height_mm
        e = self.htable.set(h, row, math.degrees(self.stand.ref))
        log.info("Horizont eingestellt: %.1f mm -> Zeile %.1f (IMU im Stand %+.2f°)", h, row, e["ref_deg"])
        return e

    def horizon_delete(self, height_mm: float) -> bool:
        if self.htable is None:
            raise ModeNotAllowed("Horizont aus ([horizon] enabled = false)")
        return self.htable.delete(height_mm)

    def _plan_tick(self, w: World) -> None:
        """Verhalten mit Planung: fertiges Ergebnis übergeben, bei Bedarf neu planen (eigener Thread)."""
        b = self.behavior
        r = self.planner.poll()
        if r is not None and r[0] == self._plan_seq and b.pending:
            b.set_path(r[1])
        if b.want_plan(w.t):
            self._plan_seq = b.request_plan(self.planner, self.gridmap, w, self._lmap_world(w.pose))
        self.gridmap.plan = list(b.path) if b.path else []
        if b.name == "explore" and getattr(b, "done", False) and not self.gridmap.locked:
            self.gridmap.lock()                       # fertig kartiert: Fahrten ändern die Karte nicht mehr
            b.save_map = True
        if getattr(b, "save_map", False):
            b.save_map = False
            if self.cfg.grid.file:
                threading.Thread(target=self.gridmap.save, args=(self.cfg.grid.file,), name="karte-speichern", daemon=True).start()

    def _lmap_world(self, pose):
        """Funktion für den Planer-Thread: bestätigte Punkte der Merkkarte (dieselben, die die Bahnprüfung
        bremsen) in Welt-m, oder None. Bei vielen Punkten dauert das am RiderPi um 100 ms, zu lang für die
        Hauptschleife."""
        if self.lmap is None or not self.cfg.home.use_localmap:
            return None
        points = self.lmap.confirmed_later(self.cfg.home.localmap_reach_m)
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)

        def world() -> np.ndarray | None:
            p = points()
            if not len(p):
                return None
            return np.column_stack([x + p[:, 0] * c - p[:, 1] * s, y + p[:, 0] * s + p[:, 1] * c])
        return world

    def _map_view(self, free: FreeSpace, tracks, now: float) -> FreeSpace:
        return mask_movable(free, [(t.cls, t.box) for t in tracks], self.cfg.camera.width, self.cfg.camera.height)

    def _arc_check(self, cmd: Command) -> Command:
        """Geplanten Bogen gegen die Hinderniskarte prüfen (auch außerhalb des Sichtfelds)."""
        if self.lmap is None or abs(cmd.v) < 1e-3:
            return cmd
        v = cmd.v * self.cfg.robot.vx_limit
        wr = math.radians(cmd.w * self.cfg.robot.vyaw_limit_dps)
        gm = self.gridmap
        if gm is not None and gm.zones and arc_hits_zone(gm.pose(), v, wr, gm.zones, self.lmap.radius):
            cmd.v = 0.0
            cmd.status += " | Sperrzone"
            return cmd
        # auch rückwärts: die Karte kennt Hindernisse hinter dem Rider (vorher gesehen)
        clear = self.lmap.arc_clearance(v, wr, horizon_s=1.2)
        need = 0.6 * abs(v) + 0.08
        if clear < need:
            k = max(0.0, (clear - 0.05) / need)
            cmd.v *= k
            cmd.status += " | Bahn %.2fm" % clear
        return cmd


# ---------------------------------------------------------------------------
_ASCII = str.maketrans({"–": "-", "—": "-", "°": "deg", "µ": "u", "ä": "ae", "ö": "oe", "ü": "ue",
                        "Ä": "Ae", "Ö": "Oe", "Ü": "Ue", "ß": "ss", "≈": "~", "→": "->"})


def _ascii(s: str) -> str:
    """cv2.putText kann nur ASCII (Hershey-Fonts), alles andere wird zu '?'."""
    return s.translate(_ASCII).encode("ascii", "replace").decode()


def render(snap: Snapshot, text: bool = True) -> np.ndarray | None:
    """Debug-Overlay zeichnen (nur auf Anfrage, kostet Zeit). text=False lässt die
    Statuszeilen weg (~10 ms je Bild am CM4); die Weboberfläche zeigt sie als HTML."""
    if snap.frame is None:
        return None
    img = snap.frame.copy()
    H, W = img.shape[:2]
    fs = snap.free
    if fs is not None and fs.columns is not None:
        sc = W / len(fs.columns)
        span = (fs.mask.shape[0] - fs.horizon_row)
        fh = fs.mask.shape[0]
        for a, b in column_runs(fs.columns):         # unbekannte Spalten (NaN): Lücke
            pts = [(int(i * sc), int((fh - fs.columns[i] * span) * H / fh)) for i in range(a, b)]
            cv2.polylines(img, [np.array(pts, np.int32)], False, (0, 255, 255), 2)
        hy = int(fs.horizon_row * H / fh)
        cv2.line(img, (0, hy), (W, hy), (255, 255, 0), 1)
        n = len(fs.sectors)
        for i, s in enumerate(fs.sectors):
            x0, x1 = int(i * W / n), int((i + 1) * W / n)
            col = (0, int(255 * s), int(255 * (1 - s)))
            cv2.rectangle(img, (x0 + 2, H - 8), (x1 - 2, H - 2), col, -1)
    if fs is not None and fs.net_rows is not None:       # FloorNet im Schattenbetrieb
        n = len(fs.net_rows)
        pts = [(int((i + 0.5) * W / n), int(r)) for i, r in enumerate(fs.net_rows)]
        cv2.polylines(img, [np.array(pts, np.int32)], False, (255, 0, 255), 2)
    for t in snap.tracks:
        x1, y1, x2, y2 = map(int, t.box)
        tgt = t.id == snap.cmd.target_id
        col = (0, 0, 255) if tgt else (0, 200, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 3 if tgt else 1)
        cv2.putText(img, f"#{t.id} {t.cls} {t.conf:.2f}", (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    hz = snap.horizon
    if hz is not None:
        # weiß: Standhorizont dieser Höhe, cyan: IMU, magenta: Bild, orange: roher IMU-Nick, grün: Einstellen
        roh = hz.get("imu_roh_deg")
        for key, col, th, lab in (("row_stand", (255, 255, 255), 1, "Stand"), ("row_imu", (255, 255, 0), 2, "IMU"),
                                  ("row_bild", (255, 0, 255), 1, "Bild"),
                                  ("row_imu_roh", (0, 140, 255), 1, "roh %+.1f" % roh if roh is not None else "roh"),
                                  ("edit_row", (0, 255, 0), 1, "Einst.")):
            r = hz.get(key)
            if r is None:
                continue
            y = int(round(r))
            cv2.line(img, (0, y), (W, y), col, th)
            cv2.putText(img, _ascii(lab), (W - 70 if key != "row_imu_roh" else 6, y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
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
