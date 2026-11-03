"""Belegungskarte des Raums (Occupancy Grid) aus Bodenmodell und Odometrie.

Anders als die Kurzzeit-Merkkarte (localmap.py, Roboter-KS, vergisst nach Sekunden) ist das
eine Karte im festen Welt-KS: Ursprung = Startpose, x vorn, y links (beim Start), Meter.
Die eigene Pose wird mitgeführt: Strecke aus der Rad-Odometrie (bzw. Fahrbefehl), Drehung aus
der Ego-Motion – dieselben Größen wie für die Merkkarte. Ohne Korrektur (Marker, Stufe 3)
driftet sie mit der Zeit.

Jede Bildspalte des Bodenmodells wird ein Strahl: frei bis zur Bodengrenze, belegt an der
Grenze (Log-Odds, begrenzt). Spalten "frei bis Horizont" setzen nur frei bis max_range,
nicht messbare Bereiche (Linsenabschattung) bleiben unbekannt. Für Fahrentscheidungen wird
die Karte (noch) nicht benutzt, nur angezeigt.
"""
from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np

from .config import GridCfg
from .freespace import FreeSpace
from .geometry import CamGeometry


class GridMap:
    def __init__(self, cfg: GridCfg, geo: CamGeometry, fs_width: int, fs_height: int):
        self.cfg = cfg
        self.geo = geo
        self.fw, self.fh = fs_width, fs_height
        self.res = cfg.resolution_m
        self.n = int(round(cfg.size_m / self.res))
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self.lo = np.zeros((self.n, self.n), np.float32)      # Log-Odds, 0 = unbekannt
            self.x = self.y = self.th = 0.0                       # Pose im Karten-KS (m, rad)
            self.path: list[tuple[float, float]] = [(0.0, 0.0)]
            self.updates = 0
            self._stats, self._stats_t = None, 0.0

    # ---- Pose ---------------------------------------------------------
    def move(self, ds: float, dyaw: float) -> None:
        """Roboter fuhr ds Meter vor und drehte um dyaw (rad, links +); Mittelpunktsregel."""
        with self._lock:
            mid = self.th + dyaw / 2
            self.x += ds * math.cos(mid)
            self.y += ds * math.sin(mid)
            self.th += dyaw
            px, py = self.path[-1]
            if math.hypot(self.x - px, self.y - py) > 0.05:
                self.path.append((self.x, self.y))
                if len(self.path) > 5000:
                    del self.path[:1000]

    def cell(self, x, y):
        """Welt-Meter -> Zellindizes (Zeile, Spalte); Kartenmitte = Ursprung."""
        c = np.floor(np.asarray(x) / self.res + self.n / 2).astype(np.int64)
        r = np.floor(np.asarray(y) / self.res + self.n / 2).astype(np.int64)
        return r, c

    # ---- Eintragen ----------------------------------------------------
    def pose(self) -> tuple[float, float, float]:
        with self._lock:
            return self.x, self.y, self.th

    def add_freespace(self, fs: FreeSpace, pitch_dev: float,
                      pose: tuple[float, float, float] | None = None) -> None:
        """Beobachtung eintragen. pose = (x, y, th) zum Zeitpunkt der Aufnahme (GridWorker
        rechnet später, der Rider ist dann schon weiter); None = aktuelle Pose."""
        if fs is None or fs.columns is None:
            return
        cfg = self.cfg
        sc = self.geo.W / self.fw
        span = self.fh - fs.horizon_row
        cols = np.asarray(fs.columns, float)
        idx = np.arange(0, len(cols), max(1, cfg.column_step))
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
            free_len = np.where(to_horizon, cfg.max_range_m, np.where(np.isfinite(dist), np.minimum(dist, cfg.max_range_m), np.nan))
            # Freistrahl endet free_margin vor der Grenze: die gemessene Grenze schwankt um
            # einige cm, sonst löscht jede etwas zu weite Messung die zuvor belegte Zelle
            free_len = np.where(hit, dist - max(self.res, cfg.free_margin_m), free_len)
        x_full = (idx + 0.5) * sc
        bear = np.arctan2(self.geo.W / 2 - x_full, self.geo.f)          # Peilung, links +
        # ground_dist_np liefert den Abstand nach vorn (wie LocalMap: x = d, y = d*tan),
        # die Strahlen brauchen die Länge entlang der Peilung
        rad = 1.0 / np.cos(bear)
        dist, free_len, start = dist * rad, free_len * rad, start * rad
        ok = np.isfinite(free_len) & (free_len > start)
        with self._lock:
            px, py, pth = pose if pose is not None else (self.x, self.y, self.th)
            ang = pth + bear
            ca, sa = np.cos(ang), np.sin(ang)
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
            if hit.any():
                ox = px + dist[hit] * ca[hit]
                oy = py + dist[hit] * sa[hit]
                r, c = self.cell(ox, oy)
                m = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
                lin = np.unique(r[m] * self.n + c[m])
                self.lo.flat[lin] = np.minimum(self.lo.flat[lin] + cfg.l_occ, cfg.l_max)
            # nur berührte Zellen begrenzt (ganzes Raster: 100k Werte je Bild, ~1 ms am CM4)
            self.updates += 1

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
        if self._stats is None or now - self._stats_t > max_age_s:
            self._stats, self._stats_t = self._count(), now
        return {"pose": pose, **self._stats}

    def _count(self) -> dict:
        with self._lock:
            lo = self.lo
            occ = int((lo > self.cfg.l_show).sum())
            free = int((lo < -self.cfg.l_show).sum())
            return {"belegt_m2": round(occ * self.res ** 2, 2), "frei_m2": round(free * self.res ** 2, 2),
                    "updates": self.updates}

    def image(self, px: int = 480, view_m: float | None = None) -> np.ndarray:
        """Karte als BGR-Bild, Ausschnitt view_m x view_m um die aktuelle Pose. Oben = Karten-x
        (vorn beim Start), links = Karten-y (links beim Start). Grau unbekannt, hell frei,
        dunkel belegt; Weg grün, Pose als rotes Dreieck, Maßstab 1 m."""
        view_m = view_m or self.cfg.view_m
        with self._lock:
            lo = self.lo.copy()
            x, y, th = self.x, self.y, self.th
            path = list(self.path)
        n_px = int(round(view_m / self.res))                           # Zellen je Bildkante
        k = px / n_px
        v, u = np.mgrid[0:n_px, 0:n_px]
        wx = x + (n_px / 2 - v - 0.5) * self.res                        # oben = +x
        wy = y + (n_px / 2 - u - 0.5) * self.res                        # links = +y
        r, c = self.cell(wx, wy)
        inside = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        val = np.zeros((n_px, n_px), np.float32)
        val[inside] = lo[r[inside], c[inside]]
        g = (255 / (1.0 + np.exp(val))).astype(np.uint8)                 # 1 - P(belegt)
        g[np.abs(val) < 0.05] = 128
        img = cv2.resize(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR), (px, px), interpolation=cv2.INTER_NEAREST)

        def to_px(ax, ay):
            return (int(round((n_px / 2 - (ay - y) / self.res) * k)),
                    int(round((n_px / 2 - (ax - x) / self.res) * k)))

        if len(path) > 1:
            cv2.polylines(img, [np.array([to_px(a, b) for a, b in path], np.int32)], False, (60, 170, 60), 2)
        tip = to_px(x + 0.18 * math.cos(th), y + 0.18 * math.sin(th))
        lft = to_px(x + 0.08 * math.cos(th + 2.4), y + 0.08 * math.sin(th + 2.4))
        rgt = to_px(x + 0.08 * math.cos(th - 2.4), y + 0.08 * math.sin(th - 2.4))
        cv2.fillPoly(img, [np.array([tip, lft, rgt], np.int32)], (40, 40, 220))
        cv2.line(img, (10, px - 12), (10 + int(round(1.0 / self.res * k)), px - 12), (220, 120, 0), 3)
        cv2.putText(img, "1 m", (12, px - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 120, 0), 1, cv2.LINE_AA)
        return img


class GridWorker:
    """Trägt Beobachtungen in eigenem Thread in die GridMap ein (am CM4 2,7 ms Median,
    7,7 ms p95 je Bild – in der Hauptschleife zu teuer). Jede Beobachtung trägt die Pose
    zum Zeitpunkt der Aufnahme; die Pose selbst führt weiter die Hauptschleife (move())."""

    def __init__(self, grid: GridMap, cpus=None, max_pending: int = 8):
        from collections import deque
        self.grid = grid
        self.cpus = list(cpus or [])
        self._q = deque(maxlen=max_pending)      # voll: älteste Beobachtung fällt weg
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.ms = 0.0
        self.dropped = 0

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
        job = (fs, pitch_dev, self.grid.pose())
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
                while not self._q and not self._stop.is_set():
                    self._cond.wait(0.2)
                if self._stop.is_set():
                    return
                fs, pd, pose = self._q.popleft()
            t0 = time.perf_counter()
            self.grid.add_freespace(fs, pd, pose)
            ms = (time.perf_counter() - t0) * 1000
            self.ms = 0.8 * self.ms + 0.2 * ms if self.ms else ms
