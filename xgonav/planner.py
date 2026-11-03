"""Pfadplanung auf der Raumkarte (gridmap.py) für den Modus home.

A* auf den Zellen der Belegungskarte: belegte Zellen werden um den Rider-Radius
aufgeblasen und sind gesperrt, Zellen nahe an Hindernissen und unbekannte Zellen
kosten mehr (lieber durch bekannten freien Raum). Gesucht wird nur in einem Ausschnitt
um Pose, Ziel und gefahrenen Weg, sonst wäre A* in Python am CM4 zu langsam. Der
Planer läuft in eigenem Thread (PlannerWorker); die Hauptschleife fragt nur ab.
"""
from __future__ import annotations

import heapq
import logging
import math
import threading
import time

import cv2
import numpy as np

from .config import HomeCfg
from .gridmap import GridMap

log = logging.getLogger("xgonav.planner")

_NB = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def cost_map(lo: np.ndarray, res: float, l_show: float, cfg: HomeCfg) -> tuple[np.ndarray, np.ndarray]:
    """(gesperrt, Kosten je Zelle) aus Log-Odds. Kosten >= 1."""
    occ = lo > l_show
    # Abstand jeder Zelle (m) zur nächsten belegten Zelle
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res if occ.any() \
        else np.full(lo.shape, np.inf, np.float32)
    blocked = dist < cfg.inflate_m
    near = np.clip(1.0 - (dist - cfg.inflate_m) / max(cfg.inflate_m, 1e-3), 0.0, 1.0)
    unknown = (lo >= -l_show) & ~occ
    cost = 1.0 + cfg.clear_cost * near + cfg.unknown_cost * unknown
    return blocked, cost.astype(np.float32)


def astar(blocked: np.ndarray, cost: np.ndarray, s: tuple[int, int], g: tuple[int, int]) -> list[tuple[int, int]] | None:
    """A* (8er-Nachbarschaft) von Zelle s nach g (Zeile, Spalte). None = kein Weg."""
    h, w = blocked.shape
    if blocked[s] or blocked[g]:
        return None
    gr, gc = g
    best = np.full((h, w), np.inf, np.float64)
    prev = np.full((h, w), -1, np.int64)
    best[s] = 0.0
    heap = [(math.hypot(s[0] - gr, s[1] - gc), 0.0, s[0], s[1])]
    while heap:
        _, d, r, c = heapq.heappop(heap)
        if (r, c) == g:
            out = [(r, c)]
            i = prev[r, c]
            while i >= 0:
                r, c = divmod(int(i), w)
                out.append((r, c))
                i = prev[r, c]
            return out[::-1]
        if d > best[r, c]:
            continue
        for dr, dc, step in _NB:
            rr, cc = r + dr, c + dc
            if rr < 0 or rr >= h or cc < 0 or cc >= w or blocked[rr, cc]:
                continue
            nd = d + step * 0.5 * (cost[r, c] + cost[rr, cc])
            if nd < best[rr, cc]:
                best[rr, cc] = nd
                prev[rr, cc] = r * w + c
                heapq.heappush(heap, (nd + math.hypot(rr - gr, cc - gc), nd, rr, cc))
    return None


def _line_free(blocked: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    rr = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    cc = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return not blocked[rr, cc].any()


def shortcut(cells: list[tuple[int, int]], blocked: np.ndarray, cost: np.ndarray) -> list[tuple[int, int]]:
    """Zwischenpunkte weglassen, solange die gerade Linie frei bleibt und nicht teurer
    wird (sonst schnitte die Kürzung durch unbekanntes Gebiet oder an Hindernissen entlang)."""
    if len(cells) < 3:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1:
            a, b = cells[i], cells[j]
            if _line_free(blocked, a, b):
                seg = np.array(cells[i:j + 1])
                n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
                rr = np.rint(np.linspace(a[0], b[0], n)).astype(int)
                cc = np.rint(np.linspace(a[1], b[1], n)).astype(int)
                if cost[rr, cc].max() <= cost[seg[:, 0], seg[:, 1]].max() + 1e-3:
                    break
            j -= 1
        out.append(cells[j])
        i = j
    return out


def plan(grid: GridMap, start_xy: tuple[float, float], goal_xy: tuple[float, float], cfg: HomeCfg) -> list[tuple[float, float]] | None:
    """Weg (Weltpunkte, m) von start_xy nach goal_xy über die Raumkarte; None = kein Weg."""
    res, n = grid.res, grid.n
    with grid._lock:
        pts = np.array(grid.path + [tuple(start_xy), tuple(goal_xy)], np.float64)
    lo_xy = pts.min(axis=0) - cfg.margin_m
    hi_xy = pts.max(axis=0) + cfg.margin_m
    r0, c0 = (int(v) for v in grid.cell(lo_xy[0], lo_xy[1]))
    r1, c1 = (int(v) for v in grid.cell(hi_xy[0], hi_xy[1]))
    r0, c0 = max(0, r0), max(0, c0)
    r1, c1 = min(n - 1, r1), min(n - 1, c1)
    with grid._lock:
        lo = grid.lo[r0:r1 + 1, c0:c1 + 1].copy()
    blocked, cost = cost_map(lo, res, grid.cfg.l_show, cfg)

    def local(xy):
        r, c = grid.cell(xy[0], xy[1])
        return int(np.clip(r - r0, 0, lo.shape[0] - 1)), int(np.clip(c - c0, 0, lo.shape[1] - 1))

    s, g = local(start_xy), local(goal_xy)
    # Um Pose und Ziel freigeben: die Pose driftet, sonst sperrt sie eine Wand neben dem Rider ein
    rad = int(math.ceil(cfg.inflate_m / res))
    for (r, c) in (s, g):
        disk = np.zeros_like(blocked, np.uint8)
        cv2.circle(disk, (c, r), rad, 1, -1)
        blocked[disk.astype(bool)] = False
    cells = astar(blocked, cost, s, g)
    if cells is None:
        return None
    cells = shortcut(cells, blocked, cost)
    out = [((c + c0 + 0.5 - n / 2) * res, (r + r0 + 0.5 - n / 2) * res) for r, c in cells]
    out[0], out[-1] = tuple(start_xy), tuple(goal_xy)
    return out


class PlannerWorker:
    """Plant in eigenem Thread. request() ersetzt einen noch nicht begonnenen Auftrag;
    poll() liefert (Auftragsnummer, Weg oder None) genau einmal je fertigem Auftrag."""

    def __init__(self, grid: GridMap, cfg: HomeCfg, cpus=None):
        self.grid, self.cfg = grid, cfg
        self.cpus = list(cpus or [])
        self._cond = threading.Condition()
        self._job = None
        self._result = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.ms = 0.0

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="planer", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread:
            self._thread.join(timeout=2)

    def request(self, start_xy, goal_xy) -> int:
        self.start()
        with self._cond:
            self._seq += 1
            self._job = (self._seq, tuple(start_xy), tuple(goal_xy))
            self._cond.notify()
            return self._seq

    def poll(self):
        with self._cond:
            r, self._result = self._result, None
            return r

    def _run(self) -> None:
        if self.cpus:
            from .affinity import pin_thread
            pin_thread(self.cpus, "Planer")
        while not self._stop.is_set():
            with self._cond:
                while self._job is None and not self._stop.is_set():
                    self._cond.wait(0.5)
                if self._stop.is_set():
                    return
                seq, s, g = self._job
                self._job = None
            t0 = time.perf_counter()
            try:
                path = plan(self.grid, s, g, self.cfg)
            except Exception:                       # Planer darf die Fahrt nicht mitreißen
                log.exception("Planung fehlgeschlagen")
                path = None
            self.ms = (time.perf_counter() - t0) * 1000
            log.debug("Plan #%d: %s in %.0f ms", seq, "%d Punkte" % len(path) if path else "kein Weg", self.ms)
            with self._cond:
                self._result = (seq, path)
