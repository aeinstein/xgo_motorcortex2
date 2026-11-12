"""C++-Kern für Planer, Frontier-Suche, Merkkarte, Wandabstand und Wegfolger (native/navfast.cpp, Teil von
libxgofast.so).

Die Aufrufer (planner.py, frontier.py, localmap.py, gridmap.py, behaviors.py) fragen lib(); ohne Bibliothek oder bei falscher
ABI-Nummer rechnen sie in Python (gleiches Ergebnis). ctypes gibt die GIL während jedes Aufrufs frei.
"""
from __future__ import annotations

import ctypes
import math

import numpy as np

from . import fastlib

_NAV_ABI = 2
_cache: list = []


class _FrontierParams(ctypes.Structure):     # = FrontierParams in native/navfast.cpp
    _fields_ = [(n, ctypes.c_int32) for n in ("h", "w", "nlab", "hs", "ws", "step", "r0", "c0", "n", "keep", "kr",
                                              "kc", "navoid", "zone_any", "pad0", "pad1")] + \
               [(n, ctypes.c_double) for n in ("res", "x", "y", "th", "min_cluster_m", "squeeze_cluster_m",
                                               "min_unknown_m2", "keep_dist2", "travel_mps", "gain_bonus_s",
                                               "gain_ref_m2", "turn_dps")]


class _LmapAdd(ctypes.Structure):            # = LmapAdd in native/navfast.cpp
    _fields_ = [(n, ctypes.c_int64) for n in ("n", "ncols", "obs_n", "cap")] +                [(n, ctypes.c_double) for n in ("sc", "cx", "f", "cam_fwd", "max_range", "far_range", "max_age",
                                               "near", "now", "lim")]


def lib():
    """Bibliothek mit passender ABI, sonst None (einmal je Prozess geprüft)."""
    if not _cache:
        L = fastlib.load()
        if not fastlib.check_abi(L, "nav_abi", _NAV_ABI):
            L = None
        else:
            p, i, d = ctypes.c_void_p, ctypes.c_int, ctypes.c_double
            L.nav_route.restype = i
            L.nav_route.argtypes = [p, p, i, i, i, i, i, i, i, p, i]
            L.nav_wave.restype = None
            L.nav_wave.argtypes = [p, i, i, i, i, p]
            L.nav_frontier.restype = None
            L.nav_frontier.argtypes = [ctypes.POINTER(_FrontierParams)] + [p] * 14
            L.lmap_thin.restype = ctypes.c_int64
            L.lmap_thin.argtypes = [p, ctypes.c_int64, d, p]
            L.lmap_confirm.restype = None
            L.lmap_confirm.argtypes = [p, p, ctypes.c_int64, p, p, d, d, i, p]
            L.lmap_arc.restype = d
            L.lmap_arc.argtypes = [p, ctypes.c_int64, d, d, d, i, d, d, p]
            L.lmap_move.restype = None
            L.lmap_move.argtypes = [p, ctypes.c_int64, d, d, d, p]
            L.lmap_add.restype = ctypes.c_int64
            L.lmap_add.argtypes = [ctypes.POINTER(_LmapAdd)] + [p] * 9
            L.nav_wall_dist.restype = d
            L.nav_wall_dist.argtypes = [p, i, d, ctypes.c_float, d, d, i, i, i, d]
            L.nav_path_target.restype = None
            L.nav_path_target.argtypes = [p, ctypes.c_int64, d, d, d, p]
        _cache.append(L)
    return _cache[0]


def _u8(a: np.ndarray) -> np.ndarray:
    """bool/uint8-Feld zusammenhängend als uint8 (ohne Kopie, wenn es schon so liegt)."""
    a = np.ascontiguousarray(a)
    return a.view(np.uint8) if a.dtype == np.bool_ else np.ascontiguousarray(a, np.uint8)


def route_cells(L, blocked: np.ndarray, cost: np.ndarray, s: tuple, g: tuple, shorten: bool = True) -> np.ndarray | None:
    """planner.astar (und mit shorten planner.shortcut): Zellen (N x 2: Zeile, Spalte) oder None = kein Weg."""
    h, w = blocked.shape
    b, c = _u8(blocked), np.ascontiguousarray(cost, np.float32)
    out = np.empty((h * w, 2), np.int32)                 # länger als alle Zellen kann kein Weg sein
    n = L.nav_route(b.ctypes.data, c.ctypes.data, h, w, int(s[0]), int(s[1]), int(g[0]), int(g[1]),
                    1 if shorten else 0, out.ctypes.data, h * w)
    return out[:n] if n >= 0 else None


def wave(L, small: np.ndarray, sr: int, sc: int) -> np.ndarray:
    """Schleife von frontier.wavefront: Schritte ab (sr, sc) über die Zellen von small, -1 = nicht erreichbar."""
    s8 = _u8(small)
    dist = np.empty(s8.shape, np.int32)
    L.nav_wave(s8.ctypes.data, s8.shape[0], s8.shape[1], int(sr), int(sc), dist.ctypes.data)
    return dist


def frontier(L, lab: np.ndarray, nlab: int, zone: np.ndarray, behind: np.ndarray, dist: np.ndarray, step: int,
             gain_cells: np.ndarray, avoid, pose: tuple, r0: int, c0: int, n: int, res: float, keep_rc, ecfg) -> tuple:
    """Gruppen-Schleife von frontier.next_goal; Rückgabe wie frontier._groups_py."""
    h, w = lab.shape
    lab = np.ascontiguousarray(lab, np.int32)
    z8 = _u8(zone)
    behind = np.ascontiguousarray(behind, np.float32)
    dist = np.ascontiguousarray(dist, np.int32)
    gain_cells = np.ascontiguousarray(gain_cells, np.float32)
    av = np.ascontiguousarray(np.asarray(avoid, np.float64).reshape(-1, 3))
    p = _FrontierParams(h=h, w=w, nlab=nlab, hs=dist.shape[0], ws=dist.shape[1], step=step, r0=r0, c0=c0, n=n,
                        keep=keep_rc is not None, kr=keep_rc[0] if keep_rc is not None else 0,
                        kc=keep_rc[1] if keep_rc is not None else 0, navoid=len(av), zone_any=bool(z8.any()),
                        res=res, x=pose[0], y=pose[1], th=pose[2], min_cluster_m=ecfg.min_cluster_m,
                        squeeze_cluster_m=ecfg.squeeze_cluster_m, min_unknown_m2=ecfg.min_unknown_m2,
                        keep_dist2=ecfg.keep_dist_m ** 2, travel_mps=ecfg.travel_mps,
                        gain_bonus_s=ecfg.gain_bonus_s, gain_ref_m2=ecfg.gain_ref_m2, turn_dps=ecfg.turn_dps)
    code = np.empty(nlab, np.int8)
    score, gain = np.empty(nlab), np.empty(nlab)
    cr, cc, kr, kc = (np.empty(nlab, np.int32) for _ in range(4))
    kd2 = np.empty(nlab, np.int64)
    L.nav_frontier(ctypes.byref(p), lab.ctypes.data, z8.ctypes.data, behind.ctypes.data, dist.ctypes.data,
                   gain_cells.ctypes.data, av.ctypes.data, code.ctypes.data, score.ctypes.data, gain.ctypes.data,
                   cr.ctypes.data, cc.ctypes.data, kd2.ctypes.data, kr.ctypes.data, kc.ctypes.data)
    return code, score, gain, cr, cc, kd2, kr, kc


def thin(L, pts: np.ndarray, lim: float) -> np.ndarray:
    """LocalMap._thin: Indizes der behaltenen Punkte."""
    p = np.ascontiguousarray(pts, np.float64)
    keep = np.empty(len(p), np.int64)
    m = L.lmap_thin(p.ctypes.data, len(p), lim, keep.ctypes.data)
    return keep[:m]


def confirm(L, pts: np.ndarray, obs: np.ndarray, need: np.ndarray, nb: np.ndarray, rad: float, count: int,
            conf: np.ndarray) -> None:
    """LocalMap.confirmed_points: conf (int8) für die Punkte mit need setzen (1 bestätigt, 0 nicht)."""
    p = np.ascontiguousarray(pts, np.float64)
    o = np.ascontiguousarray(obs, np.int64)
    n8, b8 = _u8(need), _u8(nb)                          # bis nach dem Aufruf halten
    # Suchzellen etwas größer als der Radius: Paare näher als rad liegen dann sicher in Nachbarzellen
    L.lmap_confirm(p.ctypes.data, o.ctypes.data, len(p), n8.ctypes.data, b8.ctypes.data, rad ** 2,
                   rad * 1.001, int(count), conf.ctypes.data)


def arc(L, pts: np.ndarray, v: float, w: float, horizon_s: float, steps: int, radius: float, tol: float) -> tuple:
    """Schleife von LocalMap.arc_clearance: (freie Strecke oder inf, Maske der sperrenden Punkte)."""
    p = np.ascontiguousarray(pts, np.float64)
    hit = np.zeros(len(p), np.uint8)
    s = L.lmap_arc(p.ctypes.data, len(p), v, w, horizon_s, int(steps), radius, tol, hit.ctypes.data)
    return (s, hit.view(bool)) if s < math.inf else (math.inf, None)


def move(L, pts: np.ndarray, ds: float, c: float, s: float) -> np.ndarray:
    """LocalMap.move: neue Punkte, um ds zurück und mit (cos c, sin s) gedreht."""
    p = np.ascontiguousarray(pts, np.float64)
    out = np.empty_like(p)
    L.lmap_move(p.ctypes.data, len(p), ds, c, s, out.ctypes.data)
    return out


def add(L, pts: np.ndarray, t: np.ndarray, obs: np.ndarray, cols: np.ndarray, dist: np.ndarray, known, par: dict) -> tuple:
    """LocalMap.add_freespace ab den Entfernungen je Spalte: (pts, t, obs) neu, die alten bleiben unverändert.
    par: Felder von _LmapAdd außer n und ncols."""
    p = np.ascontiguousarray(pts, np.float64)
    tt = np.ascontiguousarray(t, np.float64)
    o = np.ascontiguousarray(obs, np.int64)
    c = np.ascontiguousarray(cols, np.float64)
    d = np.ascontiguousarray(dist, np.float64)
    k = None if known is None else np.ascontiguousarray(known, np.float64)
    room = len(p) + len(c) // 2 + 1
    po, to, oo = np.empty((room, 2)), np.empty(room), np.empty(room, np.int64)
    a = _LmapAdd(n=len(p), ncols=len(c), **par)
    m = L.lmap_add(ctypes.byref(a), p.ctypes.data, tt.ctypes.data, o.ctypes.data, c.ctypes.data, d.ctypes.data,
                   None if k is None else k.ctypes.data, po.ctypes.data, to.ctypes.data, oo.ctypes.data)
    return po[:m], to[:m], oo[:m]


def wall_dist(L, lo: np.ndarray, res: float, l_show: float, x: float, y: float, r: int, c: int, k: int,
              reach: float) -> float:
    """GridMap.wall_dist; lo: n x n float32, zusammenhängend (Aufrufer hält das Lock)."""
    return L.nav_wall_dist(lo.ctypes.data, lo.shape[0], res, l_show, x, y, r, c, k, reach)


def path_target(L, path: np.ndarray, x: float, y: float, rest: float) -> tuple:
    """PathFollower.target; path: N x 2 float64, zusammenhängend."""
    out = (ctypes.c_double * 2)()
    L.nav_path_target(path.ctypes.data, len(path), x, y, rest, out)
    return out[0], out[1]
