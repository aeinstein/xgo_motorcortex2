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
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

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


def wavefront(passable: np.ndarray, start: tuple[int, int], step: int = 2) -> tuple[np.ndarray, int]:
    """Weglänge (in Schritten des groben Rasters) von start über passierbare Zellen, auf einem
    step-mal gröberen Raster, -1 = nicht erreichbar. Grob genug für die Fahrzeit-Schätzung; das
    Dilatieren läuft in OpenCV ohne GIL. Rückgabe: grobe Karte und step."""
    h, w = passable.shape
    # Fläche mitteln auf float: jede passierbare Zelle zählt, schmale Durchgänge bleiben erhalten
    small = cv2.resize(passable.astype(np.float32), (max(1, w // step), max(1, h // step)),
                       interpolation=cv2.INTER_AREA) > 0
    dist = np.full(small.shape, -1, np.int32)
    cur = np.zeros(small.shape, np.uint8)
    sr, sc = min(start[0] // step, small.shape[0] - 1), min(start[1] // step, small.shape[1] - 1)
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
    dist, step = wavefront(passable, s)
    side = max(3, int(round(2 * ecfg.gain_radius_m / res)) | 1)
    gain_cells = cv2.boxFilter(unknown.astype(np.float32), -1, (side, side), normalize=False,
                               borderType=cv2.BORDER_CONSTANT)
    # Größe (Zellen) des zusammenhängenden unbekannten Gebiets neben jeder Zelle
    _, ulab, ustats, _ = cv2.connectedComponentsWithStats(unknown.astype(np.uint8), connectivity=4)
    uarea = ustats[:, cv2.CC_STAT_AREA].astype(np.float32)
    uarea[0] = 0.0
    behind = cv2.dilate(uarea[ulab], _CROSS)
    keep_rc = to_local(*keep_frontier) if keep_frontier is not None else None
    # Zwei Stufen: [0] Gruppen mit normalem Abstand zu Hindernissen, [1] Gruppen, die nur in Engstellen liegen
    # (z. B. die Öffnung einer schmalen Lücke). Stufe 1 zählt erst, wenn Stufe 0 kein Ziel hat.
    stufen, gemieden = ([], []), [0, 0]
    nlab, lab = cv2.connectedComponents(fr.astype(np.uint8), connectivity=8)
    for k in range(1, nlab):
        rr, cc = np.nonzero(lab == k)
        weit = ~zone[rr, cc]
        stufe = 0 if weit.any() else 1
        if stufe == 0:
            ra, ca = rr[weit], cc[weit]
            ok = math.hypot(np.ptp(ca) + 1, np.ptp(ra) + 1) * res >= ecfg.min_cluster_m
            if not ok:                                  # der Teil mit Abstand ist zu klein: als Engstelle werten
                stufe = 1
        if stufe == 1:
            # Zu klein, oder das Unbekannte dahinter ist nur ein Loch im bekannten Boden
            ra, ca = rr, cc
            ok = (math.hypot(np.ptp(cc) + 1, np.ptp(rr) + 1) * res >= ecfg.squeeze_cluster_m
                  and float(behind[rr, cc].max()) * res * res >= ecfg.min_unknown_m2
                  and bool(zone.any()))
        if not ok:
            # Zu klein, außer es ist das aktuelle Ziel: Die Größe schwankt um die Schwelle
            if keep_rc is None or ((rr - keep_rc[0]) ** 2 + (cc - keep_rc[1]) ** 2).min() * res * res > ecfg.keep_dist_m ** 2:
                continue
        i = int(np.argmin((ra - ra.mean()) ** 2 + (ca - ca.mean()) ** 2))  # Zelle am nächsten zur Mitte
        r, c = int(ra[i]), int(ca[i])
        fx, fy = to_world(r, c)
        if any(math.hypot(fx - ax, fy - ay) < ar for ax, ay, ar in avoid):
            gemieden[stufe] += 1
            continue
        d = dist[min(r // step, dist.shape[0] - 1), min(c // step, dist.shape[1] - 1)]
        dm = d * step * res if d >= 0 else 1.5 * math.hypot(fx - x, fy - y)   # grobes Raster verfehlt die Zelle
        gain = float(gain_cells[r, c]) * res * res
        drive_s = dm / ecfg.travel_mps
        dth = abs((math.atan2(fy - y, fx - x) - th + math.pi) % (2 * math.pi) - math.pi)
        # Die nächste zuerst (Zeit bis dorthin); viel Unbekanntes wiegt höchstens gain_bonus_s auf. Mit Nutzen
        # je Zeit fuhr er am RiderPi viermal 3,5-9,6 m quer durch Bekanntes zu einer fernen großen Stelle und
        # später zu den liegengebliebenen zurück (2026-11-11)
        score = (ecfg.gain_bonus_s * min(1.0, gain / ecfg.gain_ref_m2)
                 - drive_s - math.degrees(dth) / ecfg.turn_dps)
        stufen[stufe].append((score, gain, r, c, fx, fy, rr, cc, stufe))
    cands = stufen[0] or (stufen[1] if allow_tight else [])
    avoided = gemieden[0] if stufen[0] or not allow_tight else sum(gemieden)
    if not cands:
        return ExplorePlan(None, clusters=avoided, avoided=avoided, known_m2=known_m2)
    cands.sort(key=lambda t: -t[0])

    # Bisheriges Ziel behalten, solange seine Gruppe besteht. Umschwenken nach Nutzen ließ den
    # Rider im Simulator alle 2 s umdrehen: Unterwegs wird immer eine andere Stelle kurz besser.
    order = cands
    keeping = False
    if keep_rc is not None:
        kr, kc = keep_rc
        near = [(int(((t[6] - kr) ** 2 + (t[7] - kc) ** 2).min()), j) for j, t in enumerate(cands)]
        d2, j = min(near)
        if d2 * res * res <= ecfg.keep_dist_m ** 2:
            t = cands[j]
            # Das Ziel wandert mit der Gruppe: ihre Mitte, wenn sie nah am alten Punkt liegt (die Grenze weicht
            # beim Anfahren zurück, er fährt dann ohne Halt weiter), sonst die Zelle am nächsten zum alten Punkt
            r, c = t[2], t[3]
            if ((r - kr) ** 2 + (c - kc) ** 2) * res * res > ecfg.keep_dist_m ** 2:
                i = int(np.argmin((t[6] - kr) ** 2 + (t[7] - kc) ** 2))
                r, c = int(t[6][i]), int(t[7][i])
            order = [(t[0], t[1], r, c, *to_world(r, c), t[6], t[7], t[8])] + cands
            keeping = True

    for k, t in enumerate(order[:4]):                   # der Beste und zur Not die nächsten
        g = (t[2], t[3])
        path = None
        if blocked_x is not None and not blocked_x[g]:
            path = route(blocked_x, cost_x, s, g, to_world, (x, y), to_world(*g))
        if path is None:
            path = route(blocked, cost, s, g, to_world, (x, y), to_world(*g))
        if path is None:
            continue
        path = shorten(path, ecfg.standoff_m)
        if keeping and k == 0:                          # Weg nur bei gleichem Ziel behalten
            kept = kept_path(keep_path, (x, y), blocked_x if blocked_x is not None else blocked, lambda p: to_local(*p))
            if kept is not None and path_length(kept) <= hcfg.keep_factor * path_length(path) + 0.3:
                path = kept
        return ExplorePlan(path, path[-1], (t[4], t[5]), len(cands) + avoided, avoided, round(t[1], 2), known_m2,
                           tight=t[8] == 1)
    return ExplorePlan(None, clusters=len(cands) + avoided, avoided=avoided, known_m2=known_m2)


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
