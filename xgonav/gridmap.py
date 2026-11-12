"""Belegungskarte des Raums (Occupancy Grid) aus Bodenmodell und Odometrie.

Anders als die Kurzzeit-Merkkarte (localmap.py, Roboter-KS, vergisst nach Sekunden) ist das
eine Karte im festen Welt-KS: Ursprung = Startpose (Radachse), x vorn, y links (beim Start), Meter.
Die eigene Pose wird mitgeführt: Strecke aus der Rad-Odometrie (bzw. Fahrbefehl), Drehung aus
der Ego-Motion – dieselben Größen wie für die Merkkarte. Ohne Korrektur (Marker, Stufe 3)
driftet sie mit der Zeit.

Jede Bildspalte des Bodenmodells wird ein Strahl: frei bis zur Bodengrenze, belegt an der
Grenze (Log-Odds, begrenzt). Spalten "frei bis Horizont" setzen nur frei bis max_range,
nicht messbare Bereiche (Linsenabschattung) bleiben unbekannt. Für Fahrentscheidungen wird
die Karte (noch) nicht benutzt, nur angezeigt.
"""
from __future__ import annotations

import ctypes
import logging
import math
import os
import secrets
import threading
import time

import cv2
import numpy as np

from . import fastlib, navfast
from .config import GridCfg
from .freespace import FreeSpace
from .geometry import CamGeometry

log = logging.getLogger("xgonav.gridmap")

# ------------------------------------------------------------------ C++-Kern (optional)
_GRID_ABI = 2


class _GridParams(ctypes.Structure):         # = GridParams in native/gridfast.cpp
    _fields_ = [(n, ctypes.c_int32) for n in ("n", "nrays", "step", "confirm_cells")] + \
               [(n, ctypes.c_double) for n in ("res", "half", "px", "py", "cp", "sp", "l_show", "l_free_occ", "l_free",
                                               "l_min", "l_occ", "l_max", "min_range", "max_range", "free_r",
                                               "margin0", "margin_quad", "confirm_dr")] + \
               [("geo", navfast.GeoRow)]


def _grid_lib():
    """C++-Kern mit passender Raumkarten-ABI, sonst None (siehe fastlib.py)."""
    lib = fastlib.load()
    if not fastlib.check_abi(lib, "grid_abi", _GRID_ABI):
        return None
    p = ctypes.c_void_p
    lib.grid_rays.restype = None
    lib.grid_rays.argtypes = [ctypes.POINTER(_GridParams)] + [p] * 8
    lib.grid_observe.restype = ctypes.c_int64
    lib.grid_observe.argtypes = [ctypes.POINTER(_GridParams), p, p, ctypes.c_uint32] + [p] * 8
    return lib


class GridMap:
    def __init__(self, cfg: GridCfg, geo: CamGeometry, fs_width: int, fs_height: int):
        self.cfg = cfg
        self.geo = geo
        self.fw, self.fh = fs_width, fs_height
        self.res = cfg.resolution_m
        self.n = int(round(cfg.size_m / self.res))
        self._lock = threading.Lock()
        self._peil: tuple | None = None           # Peilung je Strahl samt 1/cos, cos, sin (konstant je Geometrie)
        self._lib = _grid_lib() if getattr(cfg, "native", False) else None
        if self._lib is not None:
            self._stamp = np.zeros(self.n * self.n, np.uint32)   # Markierung "Zelle schon eingetragen"
            self._gen = 0
            c = cfg
            self._gp = _GridParams(n=self.n, res=self.res, half=self.res / 2, l_show=c.l_show,
                                   l_free_occ=c.l_free * c.occ_free_factor, l_free=c.l_free, l_min=c.l_min,
                                   l_occ=c.l_occ, l_max=c.l_max, step=max(1, c.column_step),
                                   confirm_cells=max(0, int(c.confirm_cells)), min_range=c.min_range_m,
                                   max_range=c.max_range_m, free_r=c.free_range_m if c.free_range_m > 0 else c.max_range_m,
                                   margin0=max(self.res, c.free_margin_m), margin_quad=c.free_margin_quad,
                                   confirm_dr=c.confirm_dr_m)
            log.info("Raumkarte: C++-Kern %s", fastlib.PATH)
        self.backend = "c++" if self._lib is not None else "python"
        self._nav = navfast.lib() if getattr(cfg, "native", False) else None   # wall_dist (native/navfast.cpp)
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self.lo = np.zeros((self.n, self.n), np.float32)      # Log-Odds, 0 = unbekannt
            self._reset_confirm()
            self.x = self.y = self.th = 0.0                       # Pose im Karten-KS (m, rad)
            self.path: list[tuple[float, float]] = [(0.0, 0.0)]
            self.updates = 0
            self._stats, self._stats_t = None, 0.0
            self.start = (0.0, 0.0, 0.0)                        # Startpunkt (x, y, th) im Karten-KS
            self.plan: list[tuple[float, float]] = []             # geplanter Weg (nur Anzeige, von App)
            self.map_id = secrets.token_hex(4)                    # Route und Zonen gehören zu genau dieser Karte
            self.locked = False                                   # gesperrt: Beobachtungen ändern die Karte nicht
            # False: Die Pose in der Karte ist unbekannt (getragen, gefallen, Neustart). Dann ändert nichts die
            # Karte und der Weg wird nicht aufgezeichnet; die Pose läuft weiter mit (Anzeige). Setzt nur App.
            self.pose_ok = True
            self.saved_pose: tuple | None = None                  # Pose beim letzten Speichern (aus der Datei)
            self.route: list[tuple[float, float]] = []            # Wegpunkte der Streife (xgonav/route.py)
            self.route_loop = False
            self.zones: list[list[tuple[float, float]]] = []      # Sperrzonen: Eckpunkte, gelten für alle Modi
            self._saved_key = None                                # (updates, start) beim letzten Speichern
            self._saved_t = time.monotonic()

    def _reset_confirm(self) -> None:
        """Bestätigung aus zwei Abständen (confirm_dr_m): je Zelle kleinste/größte Entfernung, aus der sie
        getroffen wurde, und ob sie bestätigt ist. Aufrufer hält das Lock (oder ist __init__)."""
        if self.cfg.confirm_dr_m > 0:
            self.rmin = np.full((self.n, self.n), np.inf, np.float32)
            self.rmax = np.full((self.n, self.n), -np.inf, np.float32)
            self.conf = np.zeros((self.n, self.n), bool)
        else:
            self.rmin = self.rmax = self.conf = None

    def lock(self, on: bool = True) -> None:
        with self._lock:
            self.locked = on
        log.info("Raumkarte %s", "gesperrt" if on else "entsperrt")

    def zones_mask(self, zones, r0: int, c0: int, shape: tuple, margin_m: float) -> np.ndarray:
        """Zellen in den Zonen samt Rand (bool, Ausschnitt ab r0, c0)."""
        m = np.zeros(shape, np.uint8)
        rad = int(math.ceil(margin_m / self.res))
        for z in zones:
            p = (np.asarray(z, np.float64) / self.res + self.n / 2 - [c0, r0])
            px = np.round(p).astype(np.int32).reshape(-1, 1, 2)
            cv2.fillPoly(m, [px], 1)
            if rad > 0:
                cv2.polylines(m, [px], True, 1, thickness=2 * rad + 1)
        return m.astype(bool)

    def set_pose_ok(self, ok: bool) -> None:
        """Pose gültig oder unbekannt. Beim Gültigwerden beginnt der Weg neu: Was dazwischen lag, ist unbekannt."""
        with self._lock:
            if ok and not self.pose_ok:
                self.path = [(self.x, self.y)]
            self.pose_ok = ok

    def set_pose(self, pose: tuple) -> None:
        with self._lock:
            self.x, self.y, self.th = (float(v) for v in pose)
            self.path = [(self.x, self.y)]

    def empty(self) -> bool:
        """Noch nichts eingetragen: Dann legt die Karte ihr KS selbst fest, die Pose kann nicht falsch sein."""
        with self._lock:
            return not bool(np.any(self.lo))

    def relocate_to_start(self) -> None:
        """Der Nutzer bestätigt: Der Rider steht am Startpunkt. Die Pose springt dorthin, die Drift ist weg."""
        with self._lock:
            d = math.hypot(self.x - self.start[0], self.y - self.start[1])
            self.x, self.y, self.th = self.start
            self.path = [(self.x, self.y)]
            self.pose_ok = True
        log.info("Pose auf den Startpunkt gesetzt (%.2f m Korrektur)", d)

    def set_start(self) -> None:
        """Startpunkt = aktuelle Pose (Knopf „Start hier“)."""
        with self._lock:
            self.start = (self.x, self.y, self.th)
        log.info("Startpunkt gesetzt: x %.2f m, y %.2f m, %.0f°", self.x, self.y, math.degrees(self.th))

    # ---- Speichern / Laden ------------------------------------------
    def save(self, path: str) -> bool:
        """Karte, Startpunkt und Weg atomar speichern (erst .tmp, dann umbenennen)."""
        with self._lock:
            lo = self.lo.copy()
            start = np.array(self.start, np.float64)
            pose = np.array((self.x, self.y, self.th), np.float64)
            path_pts = np.array(self.path, np.float64).reshape(-1, 2)
            key = (self.updates, self.start)
            map_id, locked = self.map_id, self.locked
            extra = {} if self.conf is None else {"rmin": self.rmin.astype(np.float16),
                                                  "rmax": self.rmax.astype(np.float16), "conf": self.conf.copy()}
        tmp = path + ".tmp"
        try:
            with open(tmp, "wb") as f:
                np.savez_compressed(f, lo=lo.astype(np.float16), res=self.res, n=self.n,
                                    start=start, path=path_pts, map_id=map_id, locked=locked, pose=pose, **extra)
            os.replace(tmp, path)
        except OSError as e:
            log.warning("Raumkarte nicht gespeichert (%s): %s", path, e)
            return False
        self._saved_key, self._saved_t = key, time.monotonic()
        log.debug("Raumkarte gespeichert: %s", path)
        return True

    def maybe_save(self, path: str, every_s: float) -> None:
        """Speichern, wenn sich seit dem letzten Mal etwas geändert hat und every_s vorbei ist."""
        if not path or time.monotonic() - self._saved_t < every_s:
            return
        with self._lock:
            key = (self.updates, self.start)
        if key != self._saved_key:
            self.save(path)
        else:
            self._saved_t = time.monotonic()

    def load(self, path: str) -> bool:
        """Gespeicherte Karte laden. Ohne Marker kann sich der Rider nicht wiederfinden:
        die Pose wird auf den Startpunkt gesetzt, er muss also dort stehen."""
        try:
            with np.load(path) as d:
                res, n = float(d["res"]), int(d["n"])
                if abs(res - self.res) > 1e-9 or n != self.n:
                    log.warning("Raumkarte %s passt nicht (%.3f m / %d Zellen statt %.3f m / %d) – ignoriert",
                                path, res, n, self.res, self.n)
                    return False
                lo = d["lo"].astype(np.float32)
                start = tuple(float(v) for v in d["start"])
                map_id = str(d["map_id"]) if "map_id" in d.files else secrets.token_hex(4)
                locked = bool(d["locked"]) if "locked" in d.files else False
                saved_pose = tuple(float(v) for v in d["pose"]) if "pose" in d.files else None
                cand = tuple(d[k] for k in ("rmin", "rmax", "conf")) if "conf" in d.files else None
        except (OSError, KeyError, ValueError) as e:
            log.warning("Raumkarte %s nicht geladen: %s", path, e)
            return False
        with self._lock:
            self.lo = lo
            self._reset_confirm()
            if self.conf is not None and cand is not None:
                # ältere Karten ohne diese Felder: belegte Zellen gelten als bestätigt (siehe unten)
                self.rmin, self.rmax = cand[0].astype(np.float32), cand[1].astype(np.float32)
                self.conf = cand[2].astype(bool)
            elif self.conf is not None:
                self.conf = lo > self.cfg.l_show
            self.start = start
            self.x, self.y, self.th = start
            self.path = [(start[0], start[1])]
            self.plan = []
            self.map_id, self.locked = map_id, locked
            self.saved_pose = saved_pose
            self.route, self.route_loop, self.zones = [], False, []
            self._stats = None
            self._saved_key = (self.updates, self.start)
        log.info("Raumkarte geladen (%s) – Rider muss am Startpunkt stehen", path)
        return True

    # ---- Pose ---------------------------------------------------------
    def move(self, ds: float, dyaw: float) -> None:
        """Roboter fuhr ds Meter vor und drehte um dyaw (rad, links +); Mittelpunktsregel."""
        with self._lock:
            mid = self.th + dyaw / 2
            self.x += ds * math.cos(mid)
            self.y += ds * math.sin(mid)
            self.th += dyaw
            px, py = self.path[-1]
            if self.pose_ok and math.hypot(self.x - px, self.y - py) > 0.05:
                self.path.append((self.x, self.y))
                if len(self.path) > 5000:
                    del self.path[:1000]
                self._clear_disk(self.x, self.y, self.cfg.driven_free_r_m)

    def clear_here(self, r: float) -> None:
        """Um die aktuelle Pose ist Boden (m)."""
        with self._lock:
            self._clear_disk(self.x, self.y, r)

    def _clear_disk(self, x: float, y: float, r: float) -> None:
        """Wo der Rider stand oder fuhr, ist Boden. Aufrufer hält das Lock."""
        if r <= 0 or self.locked or not self.pose_ok:
            return
        k = int(math.ceil(r / self.res))
        rc, cc = (int(v) for v in self.cell(x, y))
        r0, r1, c0, c1 = max(0, rc - k), min(self.n, rc + k + 1), max(0, cc - k), min(self.n, cc + k + 1)
        if r0 >= r1 or c0 >= c1:
            return
        rr, cc2 = np.ogrid[r0 - rc:r1 - rc, c0 - cc:c1 - cc]
        disk = rr * rr + cc2 * cc2 <= k * k
        view = self.lo[r0:r1, c0:c1]
        view[disk] = np.minimum(view[disk], self.cfg.l_min)
        if self.conf is not None:
            self.rmin[r0:r1, c0:c1][disk] = np.inf
            self.rmax[r0:r1, c0:c1][disk] = -np.inf
            self.conf[r0:r1, c0:c1][disk] = False

    def mark_blocking(self, pts: np.ndarray) -> int:
        """Hindernispunkte (N x 2, Welt-m), vor denen die Bahnprüfung gerade bremst: gelten als bestätigt und
        belegt. Sonst bremste er vor Hindernissen, die die Karte nie zeigte, weil sie nur aus einem Abstand
        getroffen wurden (confirm_dr_m; RiderPi 2026-11-11: 1382 unbestätigte Zellen bei 1192 belegten).
        Sieht er die Stelle später frei, bauen die Freistrahlen sie wieder ab. Rückgabe: Zellen."""
        if self.locked or not self.pose_ok or pts is None or not len(pts):
            return 0
        r, c = self.cell(pts[:, 0], pts[:, 1])
        m = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        lin = np.unique(r[m] * self.n + c[m])
        with self._lock:
            if self.conf is not None:
                self.conf.flat[lin] = True
            self.lo.flat[lin] = np.maximum(self.lo.flat[lin], self.cfg.l_show + self.cfg.l_occ)
        return int(lin.size)

    def cell(self, x, y):
        """Welt-Meter -> Zellindizes (Zeile, Spalte); Kartenmitte = Ursprung."""
        c = np.floor(np.asarray(x) / self.res + self.n / 2).astype(np.int64)
        r = np.floor(np.asarray(y) / self.res + self.n / 2).astype(np.int64)
        return r, c

    def wall_dist(self, x: float, y: float, reach_m: float) -> float:
        """Abstand (m) von (x, y) zur Mitte der nächsten belegten Zelle, inf wenn keine näher als reach_m.
        Nur ein Fenster um den Punkt (bei 0,3 m 13 x 13 Zellen)."""
        r = int(math.floor(y / self.res + self.n / 2))             # wie cell(), ohne numpy
        c = int(math.floor(x / self.res + self.n / 2))
        k = int(math.ceil(reach_m / self.res))
        L = self._nav
        if L is not None:
            with self._lock:
                lo = self.lo
                if lo.dtype == np.float32 and lo.flags.c_contiguous:
                    return navfast.wall_dist(L, lo, self.res, self.cfg.l_show, x, y, r, c, k, reach_m)
        r0, c0 = max(0, r - k), max(0, c - k)
        with self._lock:
            rr, cc = np.nonzero(self.lo[r0:max(r0, r + k + 1), c0:max(c0, c + k + 1)] > self.cfg.l_show)
        if not len(rr):
            return math.inf
        d = np.hypot((cc + c0 + 0.5 - self.n / 2) * self.res - x, (rr + r0 + 0.5 - self.n / 2) * self.res - y).min()
        return float(d) if d < reach_m else math.inf

    # ---- Eintragen ----------------------------------------------------
    def pose(self) -> tuple[float, float, float]:
        with self._lock:
            return self.x, self.y, self.th

    def _rays(self, fs: FreeSpace, pitch_dev: float):
        """Strahlen einer Beobachtung im Kamera-KS: je genutzter Spalte Peilung (links +), Länge bis zur
        Bodengrenze, Ende und Anfang des Freistrahls (alle entlang der Peilung, m), dazu ok (Freistrahl
        vorhanden) und hit (Treffer bis max_range_m). Nur Geometrie, ohne Lock; auch der Kartenabgleich
        (reloc.py) rechnet damit, damit beide dieselben Punkte sehen."""
        cfg = self.cfg
        sc = self.geo.W / self.fw
        span = self.fh - fs.horizon_row
        cols = np.asarray(fs.columns, float)
        idx, bear, rad, _, _ = self._peilung(len(cols), sc)        # NaN-Spalten = unbekannt
        if self._lib is not None:
            # im C++-Kern (gleiches Ergebnis wie unten)
            g, cols, kf = self._params(fs, cols, len(idx), span, sc, pitch_dev)
            dist, free_len, start = (np.empty(len(idx)) for _ in range(3))
            ok, hit = np.empty(len(idx), bool), np.empty(len(idx), bool)
            self._lib.grid_rays(ctypes.byref(g), cols.ctypes.data, None if kf is None else kf.ctypes.data,
                                rad.ctypes.data, dist.ctypes.data, free_len.ctypes.data, start.ctypes.data,
                                ok.ctypes.data, hit.ctypes.data)
            return idx, bear, dist, free_len, start, ok, hit
        v = cols[idx]
        dist = self.geo.ground_dist_np((self.fh - v * span) * sc, pitch_dev)
        start = np.full(len(idx), cfg.min_range_m)
        if fs.known_from is not None:
            kf = np.asarray(fs.known_from, float)[idx]
            kd = self.geo.ground_dist_np((self.fh - kf * span) * sc, pitch_dev)
            start = np.where(kf >= 0.98, np.inf, np.maximum(start, np.nan_to_num(kd, nan=np.inf)))
        to_horizon = v >= 0.98
        with np.errstate(invalid="ignore"):
            hit = ~to_horizon & np.isfinite(dist) & (dist <= cfg.max_range_m)
            # Freistrahl endet vor der Grenze: die gemessene Grenze schwankt mit der Entfernung,
            # sonst löscht jede etwas zu weite Messung die zuvor belegte Zelle
            free_r = cfg.free_range_m if cfg.free_range_m > 0 else cfg.max_range_m
            margin = max(self.res, cfg.free_margin_m) + cfg.free_margin_quad * dist * dist
            free_len = np.where(to_horizon, free_r, np.where(np.isfinite(dist), np.minimum(dist - margin, free_r), np.nan))
        # ground_dist_np liefert den Abstand nach vorn (wie LocalMap: x = d, y = d*tan),
        # die Strahlen brauchen die Länge entlang der Peilung
        dist, free_len, start = dist * rad, free_len * rad, start * rad
        ok = np.isfinite(free_len) & (free_len > start)
        return idx, bear, dist, free_len, start, ok, hit

    def _peilung(self, ncols: int, sc: float) -> tuple:
        """Je Strahl (jede column_step-te Spalte): Spaltenindex, Peilung (links +), 1 / cos, cos und sin der
        Peilung. Hängt nur an der Geometrie, deshalb gemerkt."""
        key = (ncols, sc, self.geo.W, self.geo.f, self.geo.cx, self.cfg.column_step)
        p = self._peil
        if p is None or p[0] != key:
            idx = np.arange(0, ncols, max(1, self.cfg.column_step))
            bear = np.ascontiguousarray(self.geo.bearing((idx + 0.5) * sc))
            p = self._peil = (key, idx, bear, 1.0 / np.cos(bear), np.cos(bear), np.sin(bear))
        return p[1:]

    def _params(self, fs: FreeSpace, cols: np.ndarray, nrays: int, span: float, sc: float, pitch_dev: float) -> tuple:
        """Für den C++-Kern: eigene Kopie der Parameter mit der Geometrie dieser Beobachtung, Bodengrenze und
        messbarer Bereich (oder None) als zusammenhängende float64-Felder."""
        g = _GridParams.from_buffer_copy(self._gp)
        g.nrays = nrays
        g.geo = navfast.geo_row(self.geo, self.fh, span, sc, pitch_dev)
        kf = None if fs.known_from is None else np.ascontiguousarray(fs.known_from, np.float64)
        return g, np.ascontiguousarray(cols, np.float64), kf

    def _add_native(self, fs: FreeSpace, pitch_dev: float, pose) -> bool:
        """Beobachtung in einem Aufruf im C++-Kern eintragen (Strahlen, Freistrahlen, Treffer samt
        Bestätigung); False, wenn die Felder dafür nicht passen."""
        sc = self.geo.W / self.fw
        cols = np.asarray(fs.columns, float)
        idx, _, rad, cb, sb = self._peilung(len(cols), sc)
        g, cols, kf = self._params(fs, cols, len(idx), self.fh - fs.horizon_row, sc, pitch_dev)
        with self._lock:
            lo, conf = self.lo, self.conf
            if not (lo.dtype == np.float32 and lo.flags.c_contiguous):
                return False
            if conf is not None and not (conf.dtype == np.bool_ and conf.flags.c_contiguous
                                         and self.rmin.dtype == np.float32 and self.rmin.flags.c_contiguous
                                         and self.rmax.dtype == np.float32 and self.rmax.flags.c_contiguous):
                return False
            px, py, pth = pose if pose is not None else (self.x, self.y, self.th)
            g.cp, g.sp = math.cos(pth), math.sin(pth)
            # Pose = Radachse; die Strahlen beginnen am Kamerafußpunkt cam_fwd davor
            g.px, g.py = px + self.geo.cam_fwd * g.cp, py + self.geo.cam_fwd * g.sp
            self._gen += 2
            if self._gen >= 0xFFFFFFF0:
                self._stamp[:] = 0
                self._gen = 2
            self._lib.grid_observe(ctypes.byref(g), lo.ctypes.data, self._stamp.ctypes.data, self._gen,
                                   cols.ctypes.data, None if kf is None else kf.ctypes.data, rad.ctypes.data,
                                   cb.ctypes.data, sb.ctypes.data,
                                   None if conf is None else self.rmin.ctypes.data,
                                   None if conf is None else self.rmax.ctypes.data,
                                   None if conf is None else conf.ctypes.data)
            self.updates += 1
        return True

    def add_freespace(self, fs: FreeSpace, pitch_dev: float,
                      pose: tuple[float, float, float] | None = None) -> None:
        """Beobachtung eintragen. pose = (x, y, th) zum Zeitpunkt der Aufnahme (GridWorker
        rechnet später, der Rider ist dann schon weiter); None = aktuelle Pose."""
        if fs is None or fs.columns is None or self.locked or not self.pose_ok:
            return
        if self._lib is not None and self._add_native(fs, pitch_dev, pose):
            return
        cfg = self.cfg
        idx, bear, dist, free_len, start, ok, hit = self._rays(fs, pitch_dev)
        _, _, _, cb, sb = self._peilung(len(fs.columns), self.geo.W / self.fw)
        with self._lock:
            px, py, pth = pose if pose is not None else (self.x, self.y, self.th)
            # Pose = Radachse; die Strahlen beginnen am Kamerafußpunkt cam_fwd davor
            off = self.geo.cam_fwd
            cp, sp = math.cos(pth), math.sin(pth)
            px, py = px + off * cp, py + off * sp
            # Weltrichtung je Strahl aus cos/sin von Kurs und Peilung (Additionssatz): nur Produkte, so rundet
            # numpy überall wie der C++-Kern (np.cos/np.sin runden je nach numpy-Version anders)
            ca, sa = cb * cp - sb * sp, cb * sp + sb * cp
            # freie Strecken: Stützstellen alle res/2 von start bis free_len
            if ok.any():
                steps = np.ceil((free_len[ok] - start[ok]) / (self.res / 2)).astype(np.int64) + 1
                ray = np.repeat(np.arange(ok.sum()), steps)
                k = np.arange(steps.sum()) - np.repeat(np.cumsum(steps) - steps, steps)
                s = start[ok][ray] + k * (self.res / 2)
                fx = px + s * ca[ok][ray]
                fy = py + s * sa[ok][ray]
                r, c = self.cell(fx, fy)
                m = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
                lin = np.unique(r[m] * self.n + c[m])                     # je Zelle einmal pro Bild
                cur = self.lo.flat[lin]
                # Hysterese: schon belegte Zellen nur abgeschwächt abbauen
                step = np.where(cur > cfg.l_show, cfg.l_free * cfg.occ_free_factor, cfg.l_free)
                self.lo.flat[lin] = np.maximum(cur + step, cfg.l_min)
            self._add_hits(px, py, dist, ca, sa, hit)
            # nur berührte Zellen begrenzt (ganzes Raster: 100k Werte je Bild, ~1 ms am CM4)
            self.updates += 1

    def _add_hits(self, px: float, py: float, dist, ca, sa, hit) -> None:
        """Treffer eintragen (Aufrufer hält das Lock). Mit confirm_dr_m > 0 zählt eine Zelle erst, wenn sie
        (oder eine Nachbarzelle bis confirm_cells) aus Entfernungen getroffen wurde, die sich um mindestens
        confirm_dr_m unterscheiden. Ein echtes Hindernis bleibt an seiner Stelle, wenn der Rider näher
        kommt; Spiegelungen im glänzenden Boden (RiderPi 2026-11-08: Fensterunterkante, d = D·h/(h_K + h))
        und Fugen, die je nach Abstand eine andere sind, wandern mit und werden nie bestätigt. Aus einer
        Drehung auf der Stelle allein wird daher nichts belegt."""
        if not hit.any():
            return
        cfg = self.cfg
        ox = px + dist[hit] * ca[hit]
        oy = py + dist[hit] * sa[hit]
        r, c = self.cell(ox, oy)
        m = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        lin, first = np.unique(r[m] * self.n + c[m], return_index=True)
        if self.conf is not None and lin.size:
            d = dist[hit][m][first].astype(np.float32)
            # Nachbarschaft in einem Zugriff (je Treffer (2k+1)² Zellen, am Rand abgeschnitten)
            k = max(0, int(cfg.confirm_cells))
            o = np.arange(-k, k + 1)
            r2 = np.clip((lin // self.n)[:, None] + np.repeat(o, len(o))[None, :], 0, self.n - 1)
            c2 = np.clip((lin % self.n)[:, None] + np.tile(o, len(o))[None, :], 0, self.n - 1)
            nb = r2 * self.n + c2
            lo_r = np.minimum(self.rmin.flat[nb].min(axis=1), d)
            hi_r = np.maximum(self.rmax.flat[nb].max(axis=1), d)
            ok = self.conf.flat[lin] | (hi_r - lo_r >= np.float32(cfg.confirm_dr_m))
            self.rmin.flat[lin] = np.minimum(self.rmin.flat[lin], d)
            self.rmax.flat[lin] = np.maximum(self.rmax.flat[lin], d)
            self.conf.flat[lin] = ok
            lin = lin[ok]
        self.lo.flat[lin] = np.minimum(self.lo.flat[lin] + cfg.l_occ, cfg.l_max)

    # ---- Abfragen / Anzeige ------------------------------------------
    def occupied_cells(self, thresh: float | None = None) -> np.ndarray:
        """Welt-Koordinaten (m) der belegten Zellen, Spalten x, y."""
        t = self.cfg.l_show if thresh is None else thresh
        with self._lock:
            r, c = np.nonzero(self.lo > t)
        return np.column_stack([(c + 0.5 - self.n / 2) * self.res, (r + 0.5 - self.n / 2) * self.res])

    def stats(self, max_age_s: float = 1.0) -> dict:
        """Pose (immer aktuell) und Flächen (höchstens einmal pro max_age_s neu gezählt:
        der Snapshot fragt jedes Bild, das Zählen über das ganze Raster kostet ~2 ms)."""
        now = time.monotonic()
        with self._lock:
            pose = [round(self.x, 3), round(self.y, 3), round(math.degrees(self.th), 1)]
            sx, sy, sth = self.start
            home = math.hypot(self.x - sx, self.y - sy)
            pose_ok = self.pose_ok
        if self._stats is None or now - self._stats_t > max_age_s:
            self._stats, self._stats_t = self._count(), now
        return {"pose": pose, "start": [round(sx, 3), round(sy, 3), round(math.degrees(sth), 1)],
                "zum_start_m": round(home, 2), "pose_ok": pose_ok, **self._stats}

    def _count(self) -> dict:
        with self._lock:
            lo = self.lo
            occ = int((lo > self.cfg.l_show).sum())
            free = int((lo < -self.cfg.l_show).sum())
            return {"belegt_m2": round(occ * self.res ** 2, 2), "frei_m2": round(free * self.res ** 2, 2),
                    "updates": self.updates, "gesperrt": self.locked, "map_id": self.map_id,
                    "route": len(self.route), "zonen": len(self.zones)}

    def image(self, px: int = 480, view_m: float | None = None) -> np.ndarray:
        return self.render(px, view_m)[0]

    def render(self, px: int = 480, view_m: float | None = None, center: tuple | None = None):
        """Karte als BGR-Bild, Ausschnitt view_m x view_m um center (Karten-Koordinaten in m),
        ohne center um die aktuelle Pose. Oben = Karten-x (vorn beim Start), links = Karten-y
        (links beim Start). Grau unbekannt, hell frei, dunkel belegt; Weg grün, Plan cyan, Start
        blau, Pose als rotes Dreieck, Maßstab 1 m. Rückgabe: Bild, Mitte x, Mitte y, Kante in m
        (auf ganze Zellen gerundet)."""
        view_m = view_m or self.cfg.view_m
        with self._lock:
            lo = self.lo.copy()
            cand_map = None if self.conf is None else (np.isfinite(self.rmin) & ~self.conf)
            x, y, th = self.x, self.y, self.th
            path = list(self.path)
            plan = list(self.plan)
            sx, sy, sth = self.start
            route, loop, zones = list(self.route), self.route_loop, list(self.zones)
            pose_ok = self.pose_ok
        n_px = max(1, int(round(view_m / self.res)))                   # Zellen je Bildkante
        k = px / n_px
        mx, my = center if center is not None else (x, y)
        v, u = np.mgrid[0:n_px, 0:n_px]
        wx = mx + (n_px / 2 - v - 0.5) * self.res                       # oben = +x
        wy = my + (n_px / 2 - u - 0.5) * self.res                       # links = +y
        r, c = self.cell(wx, wy)
        inside = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        val = np.zeros((n_px, n_px), np.float32)
        val[inside] = lo[r[inside], c[inside]]
        g = (255 / (1.0 + np.exp(val))).astype(np.uint8)                 # 1 - P(belegt)
        g[np.abs(val) < 0.05] = 128
        rgb = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if cand_map is not None:
            # unbestätigte Treffer (nur aus einem Abstand gesehen, z. B. Spiegelungen): orange, nicht belegt
            cm = np.zeros((n_px, n_px), bool)
            cm[inside] = cand_map[r[inside], c[inside]]
            rgb[cm & (val <= self.cfg.l_show)] = (0, 150, 255)
        img = cv2.resize(rgb, (px, px), interpolation=cv2.INTER_NEAREST)

        def to_px(ax, ay):
            return (int(round((n_px / 2 - (ay - my) / self.res) * k)),
                    int(round((n_px / 2 - (ax - mx) / self.res) * k)))

        if len(path) > 1:
            cv2.polylines(img, [np.array([to_px(a, b) for a, b in path], np.int32)], False, (60, 170, 60), 2)
        if len(plan) > 1:
            cv2.polylines(img, [np.array([to_px(a, b) for a, b in plan], np.int32)], False, (230, 210, 0), 2)
        for z in zones:                                                  # Sperrzonen rot, Route magenta
            cv2.polylines(img, [np.array([to_px(a, b) for a, b in z], np.int32)], True, (40, 40, 220), 2)
        if route:
            cv2.polylines(img, [np.array([to_px(a, b) for a, b in route], np.int32)], loop, (200, 60, 200), 1)
            for i, (a, b) in enumerate(route, 1):
                q = to_px(a, b)
                cv2.circle(img, q, 5, (200, 60, 200), -1)
                cv2.putText(img, str(i), (q[0] + 6, q[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 60, 200), 1, cv2.LINE_AA)
        sp = to_px(sx, sy)
        cv2.circle(img, sp, max(4, int(round(0.08 / self.res * k))), (230, 110, 30), 2)
        cv2.line(img, sp, to_px(sx + 0.16 * math.cos(sth), sy + 0.16 * math.sin(sth)), (230, 110, 30), 2)
        tip = to_px(x + 0.18 * math.cos(th), y + 0.18 * math.sin(th))
        lft = to_px(x + 0.08 * math.cos(th + 2.4), y + 0.08 * math.sin(th + 2.4))
        rgt = to_px(x + 0.08 * math.cos(th - 2.4), y + 0.08 * math.sin(th - 2.4))
        if pose_ok:
            cv2.fillPoly(img, [np.array([tip, lft, rgt], np.int32)], (40, 40, 220))
        else:                                                            # Pose unbekannt: nur der Umriss
            cv2.polylines(img, [np.array([tip, lft, rgt], np.int32)], True, (40, 40, 220), 2)
        cv2.line(img, (10, px - 12), (10 + int(round(1.0 / self.res * k)), px - 12), (220, 120, 0), 3)
        cv2.putText(img, "1 m", (12, px - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 120, 0), 1, cv2.LINE_AA)
        return img, mx, my, n_px * self.res


class GridWorker:
    """Trägt Beobachtungen in eigenem Thread in die GridMap ein (am CM4 2,7 ms Median,
    7,7 ms p95 je Bild – in der Hauptschleife zu teuer). Jede Beobachtung trägt die Pose
    zum Zeitpunkt der Aufnahme; die Pose selbst führt weiter die Hauptschleife (move())."""

    def __init__(self, grid: GridMap, cpus=None, max_pending: int = 8, save_path: str = "",
                 save_every_s: float = 60.0):
        from collections import deque
        self.grid = grid
        self.cpus = list(cpus or [])
        self._q = deque(maxlen=max_pending)      # voll: älteste Beobachtung fällt weg
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.ms = 0.0
        self.dropped = 0
        self.save_path, self.save_every_s = save_path, save_every_s

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gridmap", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread:
            self._thread.join(timeout=2)

    def offer(self, fs: FreeSpace, pitch_dev: float) -> None:
        job = (fs, pitch_dev, fs.pose if fs.pose is not None else self.grid.pose())
        with self._cond:
            if len(self._q) == self._q.maxlen:
                self.dropped += 1
            self._q.append(job)
            self._cond.notify()

    def idle(self) -> bool:
        with self._cond:
            return not self._q

    def _run(self) -> None:
        if self.cpus:
            from .affinity import pin_thread
            pin_thread(self.cpus, "Raumkarte")
        while not self._stop.is_set():
            with self._cond:
                if not self._q and not self._stop.is_set():
                    self._cond.wait(0.2)
                if self._stop.is_set():
                    return
                job = self._q.popleft() if self._q else None
            # Speichern hier statt in der Hauptschleife (komprimieren kostet einige 10 ms)
            self.grid.maybe_save(self.save_path, self.save_every_s)
            if job is None:
                continue
            fs, pd, pose = job
            t0 = time.perf_counter()
            self.grid.add_freespace(fs, pd, pose)
            ms = (time.perf_counter() - t0) * 1000
            self.ms = 0.8 * self.ms + 0.2 * ms if self.ms else ms
