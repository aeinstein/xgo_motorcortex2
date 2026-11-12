"""Pfadplanung auf der Raumkarte (gridmap.py) für den Modus home.

A* auf den Zellen der Belegungskarte: belegte Zellen werden um den Rider-Radius
aufgeblasen und sind gesperrt, Zellen nahe an Hindernissen und unbekannte Zellen
kosten mehr (lieber durch bekannten freien Raum). Gesucht wird nur in einem Ausschnitt
um Pose, Ziel und gefahrenen Weg, sonst wäre A* in Python am CM4 zu langsam. Der
Planer läuft in eigenem Thread (PlannerWorker); die Hauptschleife fragt nur ab.

Wegsuche und Kürzen (astar, shortcut) rechnet der C++-Kern (native/navfast.cpp), wenn gebaut; die
Funktionen hier sind der gleichwertige Python-Weg und die Vorlage dafür. Änderungen in beiden machen.
"""
from __future__ import annotations

import heapq
import logging
import math
import threading
import time

import cv2
import numpy as np

from . import navfast
from .config import HomeCfg
from .gridmap import GridMap

log = logging.getLogger("xgonav.planner")

_NB = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def driven_mask(grid: GridMap, r0: int, c0: int, shape: tuple) -> np.ndarray | None:
    """Zellen um die gefahrene Bahn (Ausschnitt ab r0, c0). Dort ist der Rider durchgekommen, auch wenn
    belegte Flecken samt Sicherheitsabstand die Karte sonst durchschneiden (RiderPi 2026-11-06)."""
    with grid._lock:
        pts = list(grid.path)
    if len(pts) < 2:
        return None
    p = np.asarray(pts, np.float64) / grid.res + grid.n / 2
    px = np.round(p - [c0, r0]).astype(np.int32)
    m = np.zeros(shape, np.uint8)
    rad = max(1, int(round(grid.cfg.driven_free_r_m / grid.res)))
    cv2.polylines(m, [px], False, 1, thickness=2 * rad + 1)
    return m.astype(bool)


def cost_map(lo: np.ndarray, res: float, l_show: float, cfg: HomeCfg,
             extra_occ: np.ndarray | None = None, driven: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(gesperrt, Kosten je Zelle) aus Log-Odds. Kosten >= 1. extra_occ: zusätzlich belegte
    Zellen (bool, gleiche Form), z. B. Hindernisse der Merkkarte. driven: passierbar trotz Raumkarte
    (driven_mask); was extra_occ gerade sieht, sperrt auch dort."""
    occ = lo > l_show
    if extra_occ is not None:
        occ |= extra_occ
    # Abstand jeder Zelle (m) zur nächsten belegten Zelle
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res if occ.any() \
        else np.full(lo.shape, np.inf, np.float32)
    hard = cfg.squeeze_m if 0 < cfg.squeeze_m < cfg.inflate_m else cfg.inflate_m
    blocked = dist < hard
    if driven is not None:
        opened = driven
        if extra_occ is not None and extra_occ.any():
            opened = driven & (cv2.distanceTransform((~extra_occ).astype(np.uint8), cv2.DIST_L2, 5) * res >= hard)
        blocked[opened] = False
    near = np.clip(1.0 - (dist - cfg.inflate_m) / max(cfg.inflate_m, 1e-3), 0.0, 1.0)
    unknown = (lo >= -l_show) & ~occ
    cost = 1.0 + cfg.clear_cost * near + cfg.unknown_cost * unknown
    if hard < cfg.inflate_m:                                # Engstelle: befahrbar, aber teuer
        tight = dist < cfg.inflate_m
        if driven is not None:
            # Wo er schon durchgefahren ist, kostet die Engstelle nichts extra: Sonst nahm er ~10 m Umweg über
            # den Startpunkt statt ~6 m zurück durch den Durchgang von eben (RiderPi 2026-11-11)
            tight &= ~opened
        cost = cost + cfg.squeeze_cost * tight
    return blocked, cost.astype(np.float32)


def squeeze_zone(cost: np.ndarray, cfg: HomeCfg) -> np.ndarray:
    """Zellen der Kostenkarte, die nur als Engstelle befahrbar sind (näher als inflate_m an Belegtem)."""
    if not 0 < cfg.squeeze_m < cfg.inflate_m:
        return np.zeros(cost.shape, bool)
    return cost >= 1.0 + cfg.clear_cost + cfg.squeeze_cost - 1e-3


def astar(blocked: np.ndarray, cost: np.ndarray, s: tuple[int, int], g: tuple[int, int]) -> list[tuple[int, int]] | None:
    """A* (8er-Nachbarschaft) von Zelle s nach g (Zeile, Spalte). None = kein Weg. Auf Listen statt
    numpy-Skalaren, und gibt die GIL regelmäßig ab: Reines Python hält sie sonst bis zu 5 ms am Stück,
    und die Hauptschleife, die sie nach jedem OpenCV-Aufruf neu braucht, bleibt jedes Mal hängen."""
    h, w = blocked.shape
    if blocked[s] or blocked[g]:
        return None
    gr, gc = g
    blk = blocked.ravel().tolist()
    cst = cost.ravel().tolist()
    best = [math.inf] * (h * w)
    prev = [-1] * (h * w)
    si = s[0] * w + s[1]
    gi = gr * w + gc
    best[si] = 0.0
    # Luftlinie als sqrt der exakten Summe (= math.hypot ganzer Zahlen): so rechnet auch der C++-Kern
    heap = [(math.sqrt((s[0] - gr) ** 2 + (s[1] - gc) ** 2), 0.0, si)]
    pops = 0
    while heap:
        _, d, i = heapq.heappop(heap)
        if i == gi:
            out = []
            while i >= 0:
                out.append(divmod(i, w))
                i = prev[i]
            return out[::-1]
        if d > best[i]:
            continue
        pops += 1
        if pops % 128 == 0:
            time.sleep(0)
        r, c = divmod(i, w)
        ci = cst[i]
        for dr, dc, step in _NB:
            rr, cc = r + dr, c + dc
            if rr < 0 or rr >= h or cc < 0 or cc >= w:
                continue
            j = rr * w + cc
            if blk[j]:
                continue
            nd = d + step * 0.5 * (ci + cst[j])
            if nd < best[j]:
                best[j] = nd
                prev[j] = i
                heapq.heappush(heap, (nd + math.sqrt((rr - gr) ** 2 + (cc - gc) ** 2), nd, j))
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
                # Auch in der Summe nicht teurer: Durch eine enge Lücke (dort gelten die höchsten Kosten) erlaubte
                # das Maximum allein jede Abkürzung schräg an der Wand entlang auf die Lücke zu.
                d = np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1]))
                cs = cost[seg[:, 0], seg[:, 1]]
                weg = float((d * (cs[:-1] + cs[1:]) / 2).sum())
                gerade = float(cost[rr, cc].mean()) * math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
                # Vergleich in float64 (wie numpy 1.26 am Rider; numpy 2 rechnete float32 + 1e-3 in float32)
                if float(cost[rr, cc].max()) <= float(cs.max()) + 1e-3 and gerade <= 1.02 * weg + 1e-3:
                    break
            j -= 1
        out.append(cells[j])
        i = j
    return out


def path_length(p: list[tuple[float, float]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))


def remaining(path: list[tuple[float, float]], xy: tuple[float, float]) -> list[tuple[float, float]]:
    """Rest des Wegs ab der Pose: xy, dann alle Punkte nach dem nächstgelegenen Abschnitt."""
    if len(path) < 2:
        return [tuple(xy)] + list(path[-1:])
    best, bi = math.inf, 0
    for i, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = min(1.0, max(0.0, ((xy[0] - a[0]) * dx + (xy[1] - a[1]) * dy) / L2)) if L2 > 1e-12 else 0.0
        d = math.hypot(a[0] + t * dx - xy[0], a[1] + t * dy - xy[1])
        if d < best:
            best, bi = d, i
    return [tuple(xy)] + [tuple(p) for p in path[bi + 1:]]


def known_area(grid: GridMap, pts, margin_m: float) -> tuple[int, int, int, int]:
    """Suchgebiet (r0, c0, r1, c1): alle bekannten Zellen und die Punkte pts, mit Rand. Nur aus dem
    gefahrenen Weg reichte nicht: nach dem Laden einer Karte ist der Weg leer."""
    n = grid.n
    with grid._lock:
        lo = grid.lo
        rows = np.flatnonzero((np.abs(lo) > grid.cfg.l_show).any(axis=1))
        cols = np.flatnonzero((np.abs(lo) > grid.cfg.l_show).any(axis=0))
    p = np.asarray(pts, np.float64).reshape(-1, 2)
    rr, cc = grid.cell(p[:, 0], p[:, 1])
    m = int(math.ceil(margin_m / grid.res))
    r0 = min(int(rr.min()), int(rows[0]) if len(rows) else n) - m
    r1 = max(int(rr.max()), int(rows[-1]) if len(rows) else -1) + m
    c0 = min(int(cc.min()), int(cols[0]) if len(cols) else n) - m
    c1 = max(int(cc.max()), int(cols[-1]) if len(cols) else -1) + m
    return max(0, r0), max(0, c0), min(n - 1, r1), min(n - 1, c1)


def kept_path(keep, start_xy, blocked, local) -> list | None:
    """Rest des bisherigen Wegs ab der Pose, wenn er noch frei ist; sonst None."""
    if not keep or len(keep) < 2:
        return None
    rest = remaining(keep, start_xy)
    cells = [local(p) for p in rest]
    return rest if all(_line_free(blocked, a, b) for a, b in zip(cells, cells[1:])) else None


def route(blocked, cost, s, g, to_world, start_xy, goal_xy, native: bool = True) -> list | None:
    """Weg von Zelle s nach g auf fertigen Kostenkarten, in Weltpunkten; None = kein Weg.
    native: A* und Kürzen im C++-Kern, wenn gebaut (gleiches Ergebnis, GIL frei)."""
    L = navfast.lib() if native else None
    if L is not None:
        cells = navfast.route_cells(L, blocked, cost, s, g)
        if cells is None:
            return None
        cells = cells.tolist()
    else:
        cells = astar(blocked, cost, s, g)
        if cells is None:
            return None
        cells = shortcut(cells, blocked, cost)
    out = [to_world(r, c) for r, c in cells]
    out[0], out[-1] = tuple(start_xy), tuple(goal_xy)
    return out


def plan(grid: GridMap, start_xy: tuple[float, float], goal_xy: tuple[float, float], cfg: HomeCfg,
         keep: list[tuple[float, float]] | None = None,
         obstacles: np.ndarray | None = None, clear_goal: bool = True) -> list[tuple[float, float]] | None:
    """Weg (Weltpunkte, m) von start_xy nach goal_xy über die Raumkarte; None = kein Weg.

    keep: bisheriger Weg. Er bleibt (ab der Pose), solange er frei ist und höchstens
    keep_factor x Länge + 0,3 m des neuen kostet: ein bei jedem Neuplanen anders gewählter Weg
    ließ am RiderPi die Zielrichtung um bis zu 50° springen (2026-11-05).
    obstacles: zusätzliche Hindernispunkte (N x 2, Welt-m), z. B. aus der Merkkarte der
    Bahnprüfung. Sonst plante der Planer durch eine Lücke, die die Bahnprüfung für zu eng
    hält, und der Rider drehte davor auf der Stelle.
    clear_goal: Umgebung des Ziels freiräumen (der Startpunkt driftet). False für frei gewählte
    Ziele wie Wegpunkte: ein Punkt in der Wand soll auffallen, nicht angefahren werden."""
    res, n = grid.res, grid.n
    keep = [tuple(p) for p in keep] if keep else None
    if callable(obstacles):                             # erst hier rechnen, im Planer-Thread
        obstacles = obstacles()
    r0, c0, r1, c1 = known_area(grid, [tuple(start_xy), tuple(goal_xy)] + (keep or []), cfg.margin_m)
    with grid._lock:
        lo = grid.lo[r0:r1 + 1, c0:c1 + 1].copy()
    extra = None
    if obstacles is not None and len(obstacles):
        rr, cc = grid.cell(obstacles[:, 0], obstacles[:, 1])
        rr, cc = rr - r0, cc - c0
        ok = (rr >= 0) & (rr < lo.shape[0]) & (cc >= 0) & (cc < lo.shape[1])
        extra = np.zeros(lo.shape, bool)
        extra[rr[ok], cc[ok]] = True
    driven = driven_mask(grid, r0, c0, lo.shape)
    blocked, cost = cost_map(lo, res, grid.cfg.l_show, cfg, extra, driven)

    def local(xy):
        r, c = grid.cell(xy[0], xy[1])
        return int(np.clip(r - r0, 0, lo.shape[0] - 1)), int(np.clip(c - c0, 0, lo.shape[1] - 1))

    s, g = local(start_xy), local(goal_xy)
    # Um die Pose freigeben: sie driftet, sonst sperrt eine Wand neben dem Rider ihn ein
    rad = int(math.ceil(cfg.inflate_m / res))
    forbid = grid.zones_mask(grid.zones, r0, c0, lo.shape, cfg.inflate_m) if grid.zones else None
    core = grid.zones_mask(grid.zones, r0, c0, lo.shape, 0.0) if grid.zones else None

    def clear_ends(b: np.ndarray) -> None:
        for (r, c) in ((s, g) if clear_goal else (s,)):
            disk = np.zeros_like(b, np.uint8)
            cv2.circle(disk, (c, r), rad, 1, -1)
            b[disk.astype(bool)] = False
        if forbid is not None:                          # Sperrzone bleibt zu; nur am Rand wegfahren darf er
            b[forbid] = True
            near = np.zeros_like(b, np.uint8)
            cv2.circle(near, (s[1], s[0]), rad, 1, -1)
            b[near.astype(bool) & ~core] = False
            b[core] = True
    clear_ends(blocked)

    def to_world(r, c):
        return (c + c0 + 0.5 - n / 2) * res, (r + r0 + 0.5 - n / 2) * res

    kept = None
    if keep and math.hypot(keep[-1][0] - goal_xy[0], keep[-1][1] - goal_xy[1]) < 0.05:
        kept = kept_path(keep, start_xy, blocked, local)
    out = route(blocked, cost, s, g, to_world, start_xy, goal_xy, cfg.native)
    if out is None and kept is None and extra is not None:
        # Die Merkkarte schloss eine Engstelle, durch die er gerade fuhr, und die Fahrt endete mit „kein Weg“
        # (RiderPi 2026-11-06). Zur Not ohne sie, wie die Zielsuche: Die Bahnprüfung bremst ohnehin.
        blocked, cost = cost_map(lo, res, grid.cfg.l_show, cfg, None, driven)
        clear_ends(blocked)
        out = route(blocked, cost, s, g, to_world, start_xy, goal_xy, cfg.native)
    if out is None:
        return kept
    if kept is not None and path_length(kept) <= cfg.keep_factor * path_length(out) + 0.3:
        return kept
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

    def request(self, start_xy, goal_xy, keep=None, obstacles=None, **kw) -> int:
        """keep: bisheriger Weg, obstacles: zusätzliche Hindernispunkte (siehe plan())."""
        return self.request_fn(plan, self.grid, tuple(start_xy), tuple(goal_xy), self.cfg, keep, obstacles, **kw)

    def request_fn(self, fn, *args, **kw) -> int:
        """Beliebiger Auftrag im Planer-Thread; poll() liefert (Nummer, Rückgabewert oder None)."""
        self.start()
        with self._cond:
            self._seq += 1
            self._job = (self._seq, fn, args, kw)
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
                seq, fn, args, kw = self._job
                self._job = None
            t0 = time.perf_counter()
            try:
                path = fn(*args, **kw)
            except Exception:                       # Planer darf die Fahrt nicht mitreißen
                log.exception("Planung fehlgeschlagen")
                path = None
            self.ms = (time.perf_counter() - t0) * 1000
            log.debug("Plan #%d: %s in %.0f ms", seq, "Ergebnis" if path else "kein Weg", self.ms)
            with self._cond:
                self._result = (seq, path)
