"""Hauptschleife: Kamera -> Ego-Motion -> Tracker/Detektor -> Freiraum -> Verhalten -> Robot."""
from __future__ import annotations

import collections
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
from .freespace import FreeSpace, FreeSpaceEstimator, FreeSpaceWorker, column_runs
from .gridmap import GridMap, GridWorker
from .planner import PlannerWorker
from .geometry import CamGeometry
from .wheels import WheelOdom, make_wheels
from .imu import ImuSource, ImuSync, make_imu
from .localmap import LocalMap, mask_movable
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
        self.geo = CamGeometry(size[0], size[1], cfg.camera.hfov_deg, cfg.camera.tilt_deg,
                               cfg.camera.mount_offset_mm, cfg.behavior.height_cruise_mm)
        self.fse = FreeSpaceEstimator(cfg.freespace, size, self.geo) if cfg.freespace.enabled else None
        self.fs_worker = FreeSpaceWorker(self.fse, cfg.cpu.freespace_cpus) \
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
        self.vis_yaw = 0.0
        self._free: FreeSpace | None = None
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
    NEEDS_FREESPACE = {"explore", "approach", "home"}

    def _check_mode(self, name: str) -> None:
        if name in self.NEEDS_FREESPACE and self.fse is None:
            raise ModeNotAllowed(f"Modus {name} braucht die Freiraum-Erkennung (freespace.enabled)")
        if name == "home" and getattr(self, "gridmap", None) is None:
            raise ModeNotAllowed("Modus home braucht die Raumkarte ([grid] enabled)")

    def _make(self, name: str, target: str | None = None) -> Behavior:
        return make_behavior(name, self.cfg.behavior, self.cfg.robot, target, self.cfg.home)

    @property
    def pipeline_on(self) -> bool:
        return not self.src.paused

    def set_pipeline(self, on: bool) -> None:
        """Bildverarbeitung starten/stoppen. Beim Stoppen: Stillstand und Modus idle;
        nach dem Starten bleibt der Rider stehen, bis ein Modus gewählt wird."""
        if on:
            self.src.resume()
            log.info("Bildverarbeitung gestartet")
        else:
            with self._mode_lock:
                self._pending_mode = ("idle", None)
            self.src.pause()
            self.robot.stop()
            log.info("Bildverarbeitung gestoppt")

    def set_manual(self, v: float, w: float) -> None:
        """Sollwert der Handsteuerung (normiert, v > 0 vor, w > 0 links). Nur im Modus manual;
        geht auch bei gestoppter Bildverarbeitung (braucht keine Kamera)."""
        b = self.behavior
        if b.name != "manual":
            raise ModeNotAllowed("Handsteuerung nur im Modus manual")
        b.set(v, w, time.monotonic())

    # Modi, die ohne Kamerabild laufen (Regelschritt dann in _paused_tick)
    NO_CAMERA = {"idle", "manual"}

    def set_mode(self, name: str, target: str | None = None) -> None:
        self._make(name, target)  # validiert (KeyError)
        self._check_mode(name)
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
            self.robot.drive(cmd.v, cmd.w)
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

        if self.fs_worker is not None:
            # eigener Thread: neuestes Ergebnis übernehmen, jedes 2. Bild einen Auftrag geben
            r = self.fs_worker.poll()
            if r is not None:
                self._free = r
            if f.idx % 2 == 0 or self._free is None:
                self.fs_worker.offer(f.bgr, pitch_dev, [t.box for t in self.tracker.tracks])
        elif self.fse is not None and (f.idx % 2 == 0 or self._free is None):
            self._free = self.fse.update(f.bgr, pitch_dev, [t.box for t in self.tracker.tracks])
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
                free = FreeSpace(list(free.sectors), free.columns.copy(), free.horizon_row, free.mask, free.valid,
                                 free.known_from)
                self.fse.block_boxes(free, [t.box for t in tracks if t.id != tgt and t.lost_s(f.t) < 0.5])
            w = World(t=f.t, dt=dt, W=self.cfg.camera.width, H=self.cfg.camera.height, hfov=self.hfov,
                      ego=ego, tracks=tracks, free=free, balancing=self.robot.state.balancing,
                      vis_yaw=self.vis_yaw, cmd_v=self.robot.command[0],
                      geo=self.geo, pitch_dev=pitch_dev)
            if self.lmap is not None:
                # Eigenbewegung seit letztem Regelschritt (Strecke: Fahrbefehl, Annahme 1 Einheit = 1 m/s)
                ds = self._odom_ds(now, dt)
                dyaw = self.vis_yaw - self._yaw_at_ctrl
                self.lmap.move(ds, dyaw)
                if self.gridmap is not None:
                    # Kartenkurs: Räder driften bei Drehungen weniger als die Ego-Motion
                    gyaw = self._wheel_dyaw if (self.cfg.grid.yaw_source == "räder"
                                                and self._wheel_dyaw is not None) else dyaw
                    self.gridmap.move(ds, gyaw)
                self._yaw_at_ctrl = self.vis_yaw
                if free is not None and self._free is not self._fs_in_map:
                    self._fs_in_map = self._free
                    view = self._map_view(free, tracks, f.t)
                    self.lmap.add_freespace(view, f.t, pitch_dev)
                    if self.grid_worker is not None:
                        self.grid_worker.offer(view, pitch_dev)
                    elif self.gridmap is not None:
                        self.gridmap.add_freespace(view, pitch_dev)
            if self.gridmap is not None:
                if self.grid_worker is None:
                    self.gridmap.maybe_save(self.cfg.grid.file, self.cfg.grid.save_every_s)
                w.pose, w.goal = self.gridmap.pose(), self.gridmap.start
                if self.behavior.name == "home":
                    self._home_plan(w)
            if self.behavior.name == "manual":
                cmd = self._manual_cmd()
            else:
                cmd = safety(self.behavior.step(w), w, self.cfg.behavior)
                cmd = self._arc_check(cmd)
            self.robot.drive(cmd.v, cmd.w)
            if cmd.height_mm is not None and now - self._last_height_t > 1.0:
                self.robot.set_height(cmd.height_mm)
                self.geo.body_height_mm = self.robot.height_mm or self.geo.body_height_mm
                self._last_height_t = now
            self._cmd = cmd
            self._free_ctrl = free
            cls = self.cfg.behavior.target_class
            self._emotion_tick(any(t.cls == cls and t.lost_s(f.t) < 0.5 for t in tracks))
        tm.mark("ctrl")

        snap = Snapshot(frame=f.bgr, idx=f.idx, t=f.t, pitch_dev=pitch_dev, tracks=list(tracks),
                        tracks_all=list(self.tracker.tracks), cam_meta=f.meta,
                        fs_diag=dict(self.fse.diag) if self.fse is not None else {},
                        free=self._free_ctrl or self._free,
                        ego=ego, cmd=self._cmd, sent=self.robot.command,
                        stats={"fps": round(self.src.fps_measured, 1), "loop_ms": round(self._loop_ms, 1),
                               **self._loop_spread(),
                               "det_ms": round(self.det.infer_ms, 1), "det_batches": self._det_count,
                               "plan_ms": round(self.planner.ms, 1) if self.planner is not None else None,
                               "fs_ms": round(self.fs_worker.ms, 1) if self.fs_worker is not None else None,
                               "grid": None if self.gridmap is None else {
                                   **self.gridmap.stats(),
                                   "ms": round(self.grid_worker.ms, 1) if self.grid_worker else None,
                                   "verworfen": self.grid_worker.dropped if self.grid_worker else 0},
                               "odom": {"src": self.odom_src, "m": round(self.odom_m, 2),
                                        "hz": round(self.wheels.rate_hz, 1) if self.wheels else None},
                               "mode": self.behavior.name, "yaw_vis_deg": round(math.degrees(self.vis_yaw), 1),
                               "yaw_drv": self.robot.state.yaw_deg, "battery": self.robot.state.battery,
                               "balancing": self.robot.state.balancing,
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


    def _odom_ds(self, now: float, dt: float) -> float:
        """Strecke seit dem letzten Regelschritt (m): aus den Rad-Encodern, wenn frisch;
        sonst wie früher Fahrbefehl x vx_limit (unkalibriert). Den Kurs liefert weiter vis_yaw."""
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
        v = sorted(h)
        return {"loop_p95": round(v[int(0.95 * (len(v) - 1))], 1), "loop_max": round(v[-1], 1)}

    def _emotion_tick(self, person: bool | None) -> None:
        if self.emotions is not None:
            try:
                self.emotions.update(time.monotonic(), self.behavior.name, person, self.robot.state.battery)
            except Exception as e:                  # Anzeige darf die Regelung nie stören
                log.warning("Emotion: %s", e)

    def _pitch_dev(self, f, ego) -> float:
        """Nickabweichung der Kamera: absolut aus der IMU, wenn synchronisiert, sonst
        nur der schnelle Anteil aus dem Bild (langsame Neigung dann unbekannt)."""
        if self.imu is not None:
            if ego.ok and ego.dt > 0:
                self.imu_sync.add(f.t - ego.dt, f.t, ego.pitch)
            self.imu_sync.update(self.imu, time.monotonic())
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

    def _home_plan(self, w: World) -> None:
        """Modus home: fertigen Weg übernehmen, bei Bedarf neu planen lassen (eigener Thread)."""
        b = self.behavior
        r = self.planner.poll()
        if r is not None and r[0] == self._plan_seq and b.pending:
            b.set_path(r[1])
            if r[1] is None:
                log.info("kein Weg zum Start gefunden (%d.)", b.fails)
        if b.want_plan(w.t):
            self._plan_seq = self.planner.request(w.pose[:2], w.goal[:2])
        self.gridmap.plan = list(b.path) if b.path else []

    def _map_view(self, free: FreeSpace, tracks, now: float) -> FreeSpace:
        return mask_movable(free, [(t.cls, t.box) for t in tracks], self.cfg.camera.width, self.cfg.camera.height)

    def _arc_check(self, cmd: Command) -> Command:
        """Geplanten Bogen gegen die Hinderniskarte prüfen (auch außerhalb des Sichtfelds)."""
        if self.lmap is None or abs(cmd.v) < 1e-3:
            return cmd
        v = cmd.v * self.cfg.robot.vx_limit
        wr = math.radians(cmd.w * self.cfg.robot.vyaw_limit_dps)
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
    for t in snap.tracks:
        x1, y1, x2, y2 = map(int, t.box)
        tgt = t.id == snap.cmd.target_id
        col = (0, 0, 255) if tgt else (0, 200, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 3 if tgt else 1)
        cv2.putText(img, f"#{t.id} {t.cls} {t.conf:.2f}", (x1, max(12, y1 - 4)),
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
