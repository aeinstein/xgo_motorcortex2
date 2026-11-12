"""Gezieltes Erkunden: Grenzen zwischen bekanntem freiem Boden und Unbekanntem finden (Frontiers)
und die lohnendste anfahren. Reine Funktionen auf der Raumkarte, laufen im Planer-Thread
(planner.PlannerWorker), nie in der Hauptschleife.

Ablauf von next_goal():
  1. Erreichbar ist, was vom Rider aus über freie, nicht aufgeblasene Zellen zusammenhängt.
  2. Frontier: erreichbare Zelle neben unbekannt, zusammenhängende Gruppen ab min_cluster_m. Gruppen dicht an
     Hindernissen (Engstellen, home.squeeze_m) erst, wenn sonst kein Ziel bleibt.
  3. Die nächste zuerst: kleinste Fahrzeit + Drehzeit; viel Unbekanntes am Ziel zählt wie höchstens
     gain_bonus_s weniger. So bleibt nichts liegen, zu dem er später quer durch die Karte zurück müsste.
  4. Das bisherige Ziel bleibt, solange seine Gruppe besteht, und wandert mit ihr mit; gewechselt wird nicht.

Die Schleifen (Fahrweg-Schätzung, Werten der Gruppen, Wegsuche) rechnet der C++-Kern (native/navfast.cpp),
wenn gebaut und home.native gesetzt; _groups_py und die Schleife in wavefront sind der gleichwertige
Python-Weg. Änderungen in beiden machen.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from . import navfast
from .config import ExploreCfg, HomeCfg
from .gridmap import GridMap
from .planner import cost_map, driven_mask, kept_path, known_area, path_length, route, squeeze_zone


@dataclass
class ExplorePlan:
    path: list | None               # Weg (Weltpunkte, m), None = kein erreichbares Ziel
    goal: tuple | None = None       # Ende des Wegs (Abstand standoff_m vor der Frontier)
    frontier: tuple | None = None   # Frontier-Punkt (Welt-m), Kennung des Ziels für keep/avoid
    clusters: int = 0               # erreichbare Frontier-Gruppen (ohne gesperrte)
    avoided: int = 0                # davon wegen avoid übergangen
    gain_m2: float = 0.0
    known_m2: float = 0.0           # bekannter freier Boden (für das Ende bei ausbleibendem Zuwachs)
    tight: bool = False             # Ziel liegt nur in einer Engstelle (Stufe 1)


_K3 = np.ones((3, 3), np.uint8)
_CROSS = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))


def wavefront(passable: np.ndarray, start: tuple[int, int], step: int = 2, native: bool = True) -> tuple[np.ndarray, int]:
    """Weglänge (in Schritten des groben Rasters) von start über passierbare Zellen, auf einem
    step-mal gröberen Raster, -1 = nicht erreichbar. Grob genug für die Fahrzeit-Schätzung; das
    Ausbreiten läuft im C++-Kern (native, wenn gebaut), sonst als Dilatieren in OpenCV. Rückgabe: grobe
    Karte und step."""
    h, w = passable.shape
    # Fläche mitteln auf float: jede passierbare Zelle zählt, schmale Durchgänge bleiben erhalten
    small = cv2.resize(passable.astype(np.float32), (max(1, w // step), max(1, h // step)),
                       interpolation=cv2.INTER_AREA) > 0
    sr, sc = min(start[0] // step, small.shape[0] - 1), min(start[1] // step, small.shape[1] - 1)
    L = navfast.lib() if native else None
    if L is not None:
        return navfast.wave(L, small, sr, sc), step
    dist = np.full(small.shape, -1, np.int32)
    cur = np.zeros(small.shape, np.uint8)
    cur[sr, sc] = 1
    dist[sr, sc] = 0
    small8 = small.astype(np.uint8)
    d = 0
    while True:
        d += 1
        nxt = cv2.dilate(cur, _K3) & small8
        new = (nxt > 0) & (dist < 0)
        if not new.any():
            break
        dist[new] = d
        cur = nxt
    return dist, step


def next_goal(grid: GridMap, pose: tuple, hcfg: HomeCfg, ecfg: ExploreCfg, obstacles: np.ndarray | None = None,
              avoid: list | tuple = (), keep_frontier: tuple | None = None, keep_path: list | None = None,
              allow_tight: bool = True) -> ExplorePlan:
    """pose: (x, y, th). avoid: gesperrte Stellen (x, y, r). keep_frontier/keep_path: bisheriges Ziel.
    allow_tight: False = Stellen, die nur in Engstellen liegen, zählen nicht mehr (explore.tight_max erreicht)."""
    res = grid.res
    x, y, th = pose
    if callable(obstacles):                             # erst hier rechnen, im Planer-Thread
        obstacles = obstacles()
    r0, c0, r1, c1 = known_area(grid, [(x, y)], hcfg.margin_m)
    with grid._lock:
        lo = grid.lo[r0:r1 + 1, c0:c1 + 1].copy()
    l_show = grid.cfg.l_show
    extra = None
    if obstacles is not None and len(obstacles):
        rr, cc = grid.cell(obstacles[:, 0], obstacles[:, 1])
        rr, cc = rr - r0, cc - c0
        ok = (rr >= 0) & (rr < lo.shape[0]) & (cc >= 0) & (cc < lo.shape[1])
        extra = np.zeros(lo.shape, bool)
        extra[rr[ok], cc[ok]] = True
    # Erreichbarkeit nur aus der Raumkarte. Die Merkkarte ändert sich mit jeder Blickrichtung: Mit ihr
    # fielen Ziele hinter einer Engstelle immer wieder weg, und der Rider pendelte (Simulator 2026-11-05).
    # Für den Weg zählt sie: erst drumherum planen, zur Not ohne sie (die Bahnprüfung bremst ohnehin).
    driven = driven_mask(grid, r0, c0, lo.shape)
    blocked, cost = cost_map(lo, res, l_show, hcfg, None, driven)
    blocked_x = cost_x = None
    if extra is not None:
        blocked_x, cost_x = cost_map(lo, res, l_show, hcfg, extra, driven)
    occ = lo > l_show
    free = lo < -l_show
    unknown = ~free & ~occ
    n = grid.n

    def to_world(r: int, c: int) -> tuple[float, float]:
        return (c + c0 + 0.5 - n / 2) * res, (r + r0 + 0.5 - n / 2) * res

    def to_local(px: float, py: float) -> tuple[int, int]:
        rr_, cc_ = grid.cell(px, py)
        return (int(np.clip(int(rr_) - r0, 0, lo.shape[0] - 1)), int(np.clip(int(cc_) - c0, 0, lo.shape[1] - 1)))

    s = to_local(x, y)
    rad = int(math.ceil(hcfg.inflate_m / res))
    disk = np.zeros(lo.shape, np.uint8)
    cv2.circle(disk, (s[1], s[0]), rad, 1, -1)
    blocked[disk.astype(bool)] = False                 # die Pose driftet: Wand neben dem Rider sperrt ihn nicht ein
    if blocked_x is not None:
        blocked_x[disk.astype(bool)] = False
    forbid = None
    if grid.zones:                                     # Sperrzonen bleiben zu, auch als Ziel
        forbid = grid.zones_mask(grid.zones, r0, c0, lo.shape, hcfg.inflate_m)
        core = grid.zones_mask(grid.zones, r0, c0, lo.shape, 0.0)
        for b_ in (blocked, blocked_x):
            if b_ is not None:
                b_[forbid & ~(disk.astype(bool) & ~core)] = True
                b_[core] = True
    known_m2 = round(float(free.sum()) * res * res, 2)
    passable = (free | (disk.astype(bool) if forbid is None else disk.astype(bool) & ~forbid)
                | (driven if driven is not None else False)) & ~blocked
    _, labels = cv2.connectedComponents(passable.astype(np.uint8), connectivity=8)
    reach = labels == labels[s]
    fr = reach & (cv2.dilate(unknown.astype(np.uint8), _CROSS) > 0)
    if not fr.any():
        return ExplorePlan(None, known_m2=known_m2)
    zone = squeeze_zone(cost, hcfg)                    # dicht an Hindernissen: nur als Engstelle befahrbar

    # Fahrweg-Schätzung und unbekannte Fläche um jeden Punkt
    dist, step = wavefront(passable, s, native=hcfg.native)
    side = max(3, int(round(2 * ecfg.gain_radius_m / res)) | 1)
    gain_cells = cv2.boxFilter(unknown.astype(np.float32), -1, (side, side), normalize=False,
                               borderType=cv2.BORDER_CONSTANT)
    # Größe (Zellen) des zusammenhängenden unbekannten Gebiets neben jeder Zelle
    _, ulab, ustats, _ = cv2.connectedComponentsWithStats(unknown.astype(np.uint8), connectivity=4)
    uarea = ustats[:, cv2.CC_STAT_AREA].astype(np.float32)
    uarea[0] = 0.0
    behind = cv2.dilate(uarea[ulab], _CROSS)
    keep_rc = to_local(*keep_frontier) if keep_frontier is not None else None
    # Zwei Stufen: Gruppen mit normalem Abstand zu Hindernissen (code 1) und Gruppen, die nur in Engstellen
    # liegen (code 2, z. B. die Öffnung einer schmalen Lücke). Stufe 2 zählt erst, wenn Stufe 1 kein Ziel hat.
    nlab, lab = cv2.connectedComponents(fr.astype(np.uint8), connectivity=8)
    L = navfast.lib() if hcfg.native else None
    args = (lab, nlab, zone, behind, dist, step, gain_cells, avoid, pose, r0, c0, n, res, keep_rc, ecfg)
    code, score, gain, cr, cc, kd2, kcr, kcc = navfast.frontier(L, *args) if L is not None else _groups_py(*args)
    weit, eng = np.flatnonzero(code == 1), np.flatnonzero(code == 2)
    cand = weit if len(weit) or not allow_tight else eng
    avoided = int((code == 3).sum()) + (0 if len(weit) or not allow_tight else int((code == 4).sum()))
    if not len(cand):
        return ExplorePlan(None, clusters=avoided, avoided=avoided, known_m2=known_m2)
    cand = cand[np.argsort(-score[cand], kind="stable")]      # die Beste zuerst, bei Gleichstand die frühere Gruppe

    # Bisheriges Ziel behalten, solange seine Gruppe besteht. Umschwenken nach Nutzen ließ den
    # Rider im Simulator alle 2 s umdrehen: Unterwegs wird immer eine andere Stelle kurz besser.
    order = list(zip(cand[:4].tolist(), cr[cand[:4]].tolist(), cc[cand[:4]].tolist()))   # (Gruppe, Zeile, Spalte)
    keeping = False
    if keep_rc is not None:
        kr, kc = keep_rc
        k = int(cand[np.argmin(kd2[cand])])                # nächste Gruppe, bei Gleichstand die bessere
        if int(kd2[k]) * res * res <= ecfg.keep_dist_m ** 2:
            # Das Ziel wandert mit der Gruppe: ihre Mitte, wenn sie nah am alten Punkt liegt (die Grenze weicht
            # beim Anfahren zurück, er fährt dann ohne Halt weiter), sonst die Zelle am nächsten zum alten Punkt
            r, c = int(cr[k]), int(cc[k])
            if ((r - kr) ** 2 + (c - kc) ** 2) * res * res > ecfg.keep_dist_m ** 2:
                r, c = int(kcr[k]), int(kcc[k])
            order = [(k, r, c)] + order[:3]
            keeping = True

    for i, (k, r, c) in enumerate(order):               # der Beste und zur Not die nächsten
        g = (r, c)
        path = None
        if blocked_x is not None and not blocked_x[g]:
            path = route(blocked_x, cost_x, s, g, to_world, (x, y), to_world(*g), hcfg.native)
        if path is None:
            path = route(blocked, cost, s, g, to_world, (x, y), to_world(*g), hcfg.native)
        if path is None:
            continue
        path = shorten(path, ecfg.standoff_m)
        if keeping and i == 0:                          # Weg nur bei gleichem Ziel behalten
            kept = kept_path(keep_path, (x, y), blocked_x if blocked_x is not None else blocked, lambda p: to_local(*p))
            if kept is not None and path_length(kept) <= hcfg.keep_factor * path_length(path) + 0.3:
                path = kept
        return ExplorePlan(path, path[-1], to_world(r, c), len(cand) + avoided, avoided, round(float(gain[k]), 2),
                           known_m2, tight=bool(code[k] == 2))
    return ExplorePlan(None, clusters=len(cand) + avoided, avoided=avoided, known_m2=known_m2)


def _hyp(a: float, b: float) -> float:
    """sqrt(a² + b²) ausgeschrieben statt math.hypot: rundet überall gleich, auch im C++-Kern."""
    return math.sqrt(a * a + b * b)


def _groups_py(lab: np.ndarray, nlab: int, zone: np.ndarray, behind: np.ndarray, dist: np.ndarray, step: int,
               gain_cells: np.ndarray, avoid, pose: tuple, r0: int, c0: int, n: int, res: float, keep_rc,
               ecfg: ExploreCfg) -> tuple:
    """Frontier-Gruppen werten (Python-Weg von navfast.frontier). lab: Gruppen aus cv2.connectedComponents.
    Je Gruppe k: code (0 zählt nicht, 1 Ziel mit Abstand, 2 Ziel nur in einer Engstelle, 3/4 wie 1/2, aber
    durch avoid gesperrt), score, gain (m²), Zelle (cr, cc) am nächsten zur Mitte, kleinster Abstand² (Zellen)
    zum bisherigen Ziel keep_rc (kd2) und dessen Zelle (kcr, kcc)."""
    x, y, th = pose
    code = np.zeros(nlab, np.int8)
    score, gain = np.zeros(nlab), np.zeros(nlab)
    cr, cc_, kcr, kcc = (np.zeros(nlab, np.int32) for _ in range(4))
    kd2 = np.full(nlab, np.iinfo(np.int64).max, np.int64)
    # alle Frontier-Zellen einmal nach Gruppe sortieren (stabil: je Gruppe die Reihenfolge von np.nonzero)
    # statt je Gruppe die ganze Karte zu durchsuchen
    rr_all, cc_all = np.nonzero(lab)
    labs = lab[rr_all, cc_all]
    o = np.argsort(labs, kind="stable")
    rr_all, cc_all = rr_all[o], cc_all[o]
    ends = np.cumsum(np.bincount(labs, minlength=nlab)).tolist()
    zone_any = bool(zone.any())
    for k in range(1, nlab):
        rr, cc = rr_all[ends[k - 1]:ends[k]], cc_all[ends[k - 1]:ends[k]]
        weit = ~zone[rr, cc]
        stufe = 0 if weit.any() else 1
        if stufe == 0:
            ra, ca = rr[weit], cc[weit]
            ok = _hyp(int(np.ptp(ca)) + 1, int(np.ptp(ra)) + 1) * res >= ecfg.min_cluster_m
            if not ok:                                  # der Teil mit Abstand ist zu klein: als Engstelle werten
                stufe = 1
        if stufe == 1:
            # Zu klein, oder das Unbekannte dahinter ist nur ein Loch im bekannten Boden
            ra, ca = rr, cc
            ok = (_hyp(int(np.ptp(cc)) + 1, int(np.ptp(rr)) + 1) * res >= ecfg.squeeze_cluster_m
                  and float(behind[rr, cc].max()) * res * res >= ecfg.min_unknown_m2
                  and zone_any)
        if keep_rc is not None:
            d2 = (rr - keep_rc[0]) ** 2 + (cc - keep_rc[1]) ** 2
            i = int(np.argmin(d2))
            kd2[k], kcr[k], kcc[k] = d2[i], rr[i], cc[i]
        if not ok:
            # Zu klein, außer es ist das aktuelle Ziel: Die Größe schwankt um die Schwelle
            if keep_rc is None or int(kd2[k]) * res * res > ecfg.keep_dist_m ** 2:
                continue
        i = int(np.argmin((ra - ra.mean()) ** 2 + (ca - ca.mean()) ** 2))  # Zelle am nächsten zur Mitte
        r, c = int(ra[i]), int(ca[i])
        cr[k], cc_[k] = r, c
        fx, fy = (c + c0 + 0.5 - n / 2) * res, (r + r0 + 0.5 - n / 2) * res
        if any(_hyp(fx - ax, fy - ay) < ar for ax, ay, ar in avoid):
            code[k] = 3 + stufe
            continue
        d = int(dist[min(r // step, dist.shape[0] - 1), min(c // step, dist.shape[1] - 1)])
        dm = d * step * res if d >= 0 else 1.5 * _hyp(fx - x, fy - y)   # grobes Raster verfehlt die Zelle
        g = float(gain_cells[r, c]) * res * res
        drive_s = dm / ecfg.travel_mps
        dth = abs((math.atan2(fy - y, fx - x) - th + math.pi) % (2 * math.pi) - math.pi)
        # Die nächste zuerst (Zeit bis dorthin); viel Unbekanntes wiegt höchstens gain_bonus_s auf. Mit Nutzen
        # je Zeit fuhr er am RiderPi viermal 3,5-9,6 m quer durch Bekanntes zu einer fernen großen Stelle und
        # später zu den liegengebliebenen zurück (2026-11-11)
        score[k] = (ecfg.gain_bonus_s * min(1.0, g / ecfg.gain_ref_m2)
                    - drive_s - math.degrees(dth) / ecfg.turn_dps)
        gain[k] = g
        code[k] = 1 + stufe
    return code, score, gain, cr, cc_, kd2, kcr, kcc


def shorten(path: list, by_m: float) -> list:
    """Weg um by_m vor dem Ende kürzen: die Kamera sieht die Frontier schon von weitem, und
    direkt davor (unter ~0,2 m) sähe sie den Boden gar nicht mehr. Mindestens der Anfang bleibt."""
    if by_m <= 0 or len(path) < 2:
        return path
    rest = by_m
    out = list(path)
    while len(out) >= 2:
        ax, ay = out[-2]
        bx, by = out[-1]
        seg = math.hypot(bx - ax, by - ay)
        if seg > rest:
            t = (seg - rest) / seg
            out[-1] = (ax + t * (bx - ax), ay + t * (by - ay))
            return out
        rest -= seg
        out.pop()
    return out if len(out) >= 2 else [path[0], path[0]]
