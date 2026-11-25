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
from .mathe import wrap
from .planner import cost_map, driven_mask, glaetten, kept_path, known_area, merk_maske, path_length, route, squeeze_zone


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
              allow_tight: bool = True, prefer: tuple | None = None, min_m: float = 0.0) -> ExplorePlan:
    """pose: (x, y, th). avoid: gesperrte Stellen (x, y, r). keep_frontier/keep_path: bisheriges Ziel.
    allow_tight: False = Stellen, die nur in Engstellen liegen, zählen nicht mehr (explore.tight_max erreicht).
    prefer: (Richtung rad im Karten-KS, s je Grad): Stellen abseits dieser Richtung (von der Pose aus) kosten so
    viel mehr (pet: Rufsuche nach klarer Peilung). min_m: Stellen, deren Weg kürzer wäre, zählen nicht (Frontier
    näher als standoff_m + min_m). Beides nur in Python auf den Kandidaten, nicht im C++-Kern."""
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
        blocked_x, cost_x = cost_map(lo, res, l_show, hcfg, extra, driven,
                                     merk_maske(grid, obstacles, r0, c0, lo.shape, hcfg.merk_r_m))
    occ = lo > l_show
    free = lo < -l_show
    unknown = ~free & ~occ
    # Merkkarte, aufgezogen um merk_abstand_m (explore.ziel_sicht): Wo sie eine bestätigte Wand sieht, ist nichts
    # unbekannt. Sonst wurde ein Loch in einer gezeichneten Wand zur Stelle, und er fuhr in die Ecke davor (RiderPi
    # 2026-11-24 20:17, Nische am Heizkörper). Dient auch der Prüfung des Rückfallwegs.
    merk_weg = None
    if ecfg.ziel_sicht and extra is not None:
        merk_weg = merk_maske(grid, obstacles, r0, c0, lo.shape, ecfg.merk_abstand_m)
        merk_weg = extra | merk_weg if merk_weg is not None else extra
        unknown &= ~merk_weg
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
        # Die Punkte der Merkkarte selbst bleiben auch um die Pose zu, nur die Zelle der Pose nicht: Sonst plante er
        # durch ein Tischbein 10 cm vor sich (RiderPi 2026-11-24 20:43, „frei 0,00 m“, 4 s gedrückt)
        blocked_x[extra & disk.astype(bool)] = True
        blocked_x[s] = False
    # Bremsstellen (grid.brake_block_s): gesperrt auch für die Erreichbarkeit, sonst bliebe eine Stelle hinter
    # einer Lücke, die nicht passt, das Ziel; nur um die Pose nicht
    brems = grid.brems_maske(r0, c0, lo.shape)
    if brems is not None:
        brems &= ~disk.astype(bool)
        blocked |= brems
        if blocked_x is not None:
            blocked_x |= brems
    forbid = None
    zones = grid.all_zones()                           # Sperr- und Dunkelzonen
    if zones:                                          # Sperrzonen bleiben zu, auch als Ziel
        forbid = grid.zones_mask(zones, r0, c0, lo.shape, hcfg.inflate_m)
        core = grid.zones_mask(zones, r0, c0, lo.shape, 0.0)
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
    if ecfg.weg_bekannt:
        # Wege zu Stellen nur über bekannten Boden: Eine Stelle liegt am Rand des Bekannten, der Weg durchs
        # Unbekannte führte durch eine andere Öffnung von hinten an sie heran (RiderPi 2026-11-24, Mülleimer)
        fremd = unknown & ~disk.astype(bool)
        if driven is not None:
            fremd &= ~driven
        blocked = blocked | fremd
        if blocked_x is not None:
            blocked_x = blocked_x | fremd

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
    if ecfg.tight_penalty_s < 0:
        # alte Regel: Engstellen nur, wenn es kein Ziel mit Abstand gibt
        cand = weit if len(weit) or not allow_tight else eng
        avoided = int((code == 3).sum()) + (0 if len(weit) or not allow_tight else int((code == 4).sum()))
    else:
        # Engstellen zählen mit, kosten aber tight_penalty_s mehr: Sonst kehrte er für ein fernes Ziel mit Abstand
        # hinter sich um, obwohl eine Engstelle direkt voraus lag (RiderPi 2026-11-15)
        cand = np.concatenate([weit, eng]) if allow_tight else weit
        avoided = int((code == 3).sum()) + (int((code == 4).sum()) if allow_tight else 0)
        score = score.copy()
        score[eng] -= ecfg.tight_penalty_s
    if len(cand) and (prefer is not None or min_m > 0):
        fx = (cc[cand] + c0 + 0.5 - n / 2) * res
        fy = (cr[cand] + r0 + 0.5 - n / 2) * res
        if min_m > 0:
            near = np.hypot(fx - x, fy - y) < ecfg.standoff_m + min_m
            cand, fx, fy = cand[~near], fx[~near], fy[~near]
        if prefer is not None and len(cand):
            dev = np.abs(wrap(np.arctan2(fy - y, fx - x) - prefer[0]))
            score = score.copy()
            score[cand] -= np.degrees(dev) * prefer[1]
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

    # Erst die Stellen, die auch mit der Merkkarte erreichbar sind (der Beste und zur Not die nächsten), dann
    # die übrigen ohne sie. Ein Weg, den nur die Raumkarte erlaubt, führte am RiderPi durch Stuhlbeine, die
    # nur die Merkkarte kannte (2026-11-23); er gilt deshalb als Engstelle (explore.tight_max).
    gaenge = [True, False] if blocked_x is not None else [False]
    # Sichtprüfung (explore.ziel_sicht): Das Ziel muss vom Haltepunkt aus zu sehen sein, gemessen an Raumkarte und
    # Merkkarte. Sonst lag die Stelle hinter einem Hindernis, das nur die Merkkarte kannte, und er fuhr darauf zu und
    # dahinter (RiderPi 2026-11-24, Mülleimer). Nur auf den Kandidaten, im Planer-Thread.
    sicht = ecfg.ziel_sicht
    sperre = (occ | extra) if (sicht and extra is not None) else occ
    verworfen = set()
    for mit_merk in gaenge:
        for i, (k, r, c) in enumerate(order):
            g = (r, c)
            if k in verworfen:
                continue
            if blocked_x is not None and (mit_merk or sicht) and blocked_x[g]:
                continue
            if mit_merk:
                path = route(blocked_x, cost_x, s, g, to_world, (x, y), to_world(*g), hcfg.native)
            else:
                path = route(blocked, cost, s, g, to_world, (x, y), to_world(*g), hcfg.native)
                if path is not None and merk_weg is not None and (_weg_trifft(merk_weg, path, to_local, s, rad)
                                                                  or _weg_trifft(extra, path, to_local, s, 1)):
                    continue                            # Rückfall nur, wenn der Weg die Merkkarte nicht schneidet
            if path is None:
                continue
            path = shorten(path, ecfg.standoff_m)
            if sicht and not _sicht_frei(sperre, to_local(*path[-1]), g):
                verworfen.add(k)
                continue
            kept = None
            if keeping and i == 0:                      # Weg nur bei gleichem Ziel behalten
                kept = kept_path(keep_path, (x, y), blocked_x if blocked_x is not None else blocked, lambda p: to_local(*p))
                if kept is not None and path_length(kept) <= hcfg.keep_factor * path_length(path) + 0.3:
                    path = kept
                else:
                    kept = None
            if kept is None:                            # nur der gewählte neue Weg: Ecken zu Bögen (home.glatt_r_m)
                blk = blocked_x if mit_merk else blocked
                path = glaetten(path, hcfg.glatt_r_m, hcfg.glatt_step_m, lambda px, py: not blk[to_local(px, py)])
            nur_raum = blocked_x is not None and not mit_merk
            return ExplorePlan(path, path[-1], to_world(r, c), len(cand) + avoided, avoided, round(float(gain[k]), 2),
                               known_m2, tight=bool(code[k] == 2) or nur_raum)
    return ExplorePlan(None, clusters=len(cand) + avoided, avoided=avoided + len(verworfen), known_m2=known_m2)


def _sicht_frei(sperre: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    """Gerade von Zelle a nach Zelle b (Zeile, Spalte) ohne gesperrte Zelle dazwischen (Endpunkte zählen nicht)."""
    m = np.zeros(sperre.shape, np.uint8)
    cv2.line(m, (a[1], a[0]), (b[1], b[0]), 1, 1)
    m[a] = 0
    m[b] = 0
    return not bool((sperre & (m > 0)).any())


def _weg_trifft(maske: np.ndarray, path: list, to_local, s: tuple[int, int], frei_r: int = 0) -> bool:
    """Schneidet der Weg (Weltpunkte) eine Zelle der Maske? Zellen bis frei_r um die Pose zählen nicht (steht er
    schon dicht an etwas, sperrte das sonst jeden Weg)."""
    m = np.zeros(maske.shape, np.uint8)
    pts = np.array([to_local(*p)[::-1] for p in path], np.int32)
    cv2.polylines(m, [pts], False, 1, 1)
    cv2.circle(m, (s[1], s[0]), frei_r, 0, -1)
    return bool((maske & (m > 0)).any())


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
        dth = abs(wrap(math.atan2(fy - y, fx - x) - th))
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
