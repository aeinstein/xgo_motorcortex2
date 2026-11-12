"""C++-Kern für Planer, Frontier-Suche und Merkkarte (xgonav/native/navfast.cpp) gegen die Python-Wege.

Alle Vergleiche sind bitgleich. Ohne gebauten Kern übersprungen (am Rider: make -C xgonav/native; am PC
lässt sich navfast.cpp allein bauen und mit XGONAV_FASTLIB=<Datei> laden, siehe docs/testing.md)."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav import navfast  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402

L = navfast.lib()
pytestmark = pytest.mark.skipif(L is None, reason="C++-Kern nicht gebaut (make -C xgonav/native)")

# Die Bodengeometrie (arctan2, tan) rechnet der Kern mit der libm. numpy tut das auch (RiderPi, PC ohne AVX-512);
# mit AVX-512 nimmt numpy eigene Vektorfunktionen, die im letzten Bit abweichen. Dann gilt für alles, was von
# der Geometrie abhängt, statt Bitgleichheit eine Toleranz von 1e-9.
_x = np.linspace(-3.0, 3.0, 4001)
LIBM = (np.array_equal(np.tan(_x), [math.tan(v) for v in _x])
        and np.array_equal(np.arctan2(_x, 310.0), [math.atan2(v, 310.0) for v in _x]))


def _gleich(a, b) -> bool:
    """Felder gleich: bitgleich, wo numpy wie der Kern die libm nutzt, sonst bis auf 1e-9."""
    a, b = np.asarray(a), np.asarray(b)
    if LIBM:
        return a.dtype == b.dtype and np.array_equal(a, b, equal_nan=a.dtype.kind == "f")
    return a.shape == b.shape and np.allclose(a, b, rtol=1e-9, atol=1e-12, equal_nan=True)


def _karte(rng, n=120, waende=14, loecher=0.0):
    """Zufällige Raumkarte (Log-Odds): bekannter Boden, Wandstücke, außen und in Flecken unbekannt."""
    cfg = Config()
    lo = np.full((n, n), cfg.grid.l_min, np.float32)
    lo[:6], lo[-6:], lo[:, :6], lo[:, -6:] = 0.0, 0.0, 0.0, 0.0
    for _ in range(waende):
        r, c = rng.integers(8, n - 8, 2)
        if rng.random() < 0.5:
            lo[r, c:c + rng.integers(4, n // 2)] = cfg.grid.l_max
        else:
            lo[r:r + rng.integers(4, n // 2), c] = cfg.grid.l_max
    if loecher:
        lo[cv2.blur((rng.random((n, n)) < loecher).astype(np.float32), (5, 5)) > 0.2] = 0.0
    return cfg, lo


def test_route_matches_python():
    """A* und Kürzen: dieselben Zellen wie planner.astar / planner.shortcut, an Engstellen, ohne Weg und auf
    freiem Feld mit wechselnden Kosten (lange Geraden: paarweise Summen über mehr als 128 Werte)."""
    from xgonav.planner import astar, cost_map, shortcut
    rng = np.random.default_rng(5)

    def gleich(blocked, cost, s, g):
        ref = astar(blocked, cost, s, g)
        got = navfast.route_cells(L, blocked, cost, s, g, shorten=False)
        kurz = navfast.route_cells(L, blocked, cost, s, g)
        if ref is None:
            assert got is None and kurz is None
            return 0
        assert [tuple(p) for p in got.tolist()] == ref
        assert [tuple(p) for p in kurz.tolist()] == shortcut(ref, blocked, cost)
        return len(ref)

    wege = ohne = 0
    for k in range(6):
        cfg, lo = _karte(rng, waende=(14, 24)[k % 2])
        blocked, cost = cost_map(lo, cfg.grid.resolution_m, cfg.grid.l_show, cfg.home)
        frei = np.argwhere(~blocked)
        for _ in range(8):
            s, g = (tuple(int(v) for v in frei[i]) for i in rng.integers(0, len(frei), 2))
            n = gleich(blocked, cost, s, g)
            wege += n > 0
            ohne += n == 0
    assert wege > 25 and ohne > 0
    # gesperrter Start oder gesperrtes Ziel: kein Weg
    b = tuple(int(v) for v in np.argwhere(blocked)[0])
    assert navfast.route_cells(L, blocked, cost, b, s) is None
    assert navfast.route_cells(L, blocked, cost, s, b) is None
    # freies Feld: die Gerade ist immer frei, gekürzt wird nach den Kostensummen
    frei = np.zeros((150, 150), bool)
    lang = 0
    for k in range(4):
        cost = (1.0 + cv2.blur(rng.random((150, 150)).astype(np.float32), (9, 9)) * (0.05, 3.0)[k % 2]).astype(np.float32)
        s, g = (int(rng.integers(0, 8)), int(rng.integers(0, 150))), (int(rng.integers(140, 150)), int(rng.integers(0, 150)))
        lang += gleich(frei, cost, s, g) > 130
    assert lang == 4


def test_plan_matches_python():
    """planner.plan über den Kern liefert denselben Weg in Weltpunkten."""
    from xgonav.gridmap import GridMap
    from xgonav.planner import plan
    cfg = Config()
    m = GridMap(replace(cfg.grid, native=False, file=""), CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)
    rng = np.random.default_rng(2)
    n0 = m.n // 2 - 60
    _, lo = _karte(rng)
    m.lo[n0:n0 + 120, n0:n0 + 120] = lo
    frei = np.argwhere(m.lo < -cfg.grid.l_show)
    gleich = 0
    for _ in range(25):
        (r0, c0), (r1, c1) = frei[rng.integers(0, len(frei), 2)]
        a = ((c0 + 0.5 - m.n / 2) * m.res, (r0 + 0.5 - m.n / 2) * m.res)
        b = ((c1 + 0.5 - m.n / 2) * m.res, (r1 + 0.5 - m.n / 2) * m.res)
        pn = plan(m, a, b, cfg.home)
        pp = plan(m, a, b, replace(cfg.home, native=False))
        assert pn == pp
        gleich += pn is not None
    assert gleich > 10


def test_wavefront_matches_dilate():
    from xgonav.frontier import wavefront
    rng = np.random.default_rng(3)
    for k in range(20):
        h, w = rng.integers(20, 90, 2)
        p = cv2.blur((rng.random((h, w)) < 0.45).astype(np.float32), (3, 3)) > 0.4
        s = (int(rng.integers(0, h)), int(rng.integers(0, w)))
        step = (2, 3)[k % 2]
        dn, _ = wavefront(p, s, step)
        dp, _ = wavefront(p, s, step, native=False)
        assert dn.dtype == dp.dtype and np.array_equal(dn, dp)
    assert (dp > 3).any() and (dp < 0).any()


def test_frontier_groups_match_python():
    """Werten der Frontier-Gruppen: alle Felder gleich, mit und ohne bisheriges Ziel, gesperrten Stellen,
    Engstellen-Zone."""
    from xgonav.frontier import _groups_py
    rng = np.random.default_rng(9)
    cfg = Config()
    codes = set()
    for k in range(40):
        h, w = (int(v) for v in rng.integers(40, 110, 2))
        fr = cv2.blur((rng.random((h, w)) < 0.08).astype(np.float32), (3, 3)) > (0.15, 0.3)[k % 2]
        nlab, lab = cv2.connectedComponents(fr.astype(np.uint8), connectivity=8)
        zone = cv2.blur((rng.random((h, w)) < 0.05).astype(np.float32), (7, 7)) > 0.05 if k % 4 else np.zeros((h, w), bool)
        behind = (rng.random((h, w)) * (4000, 40)[k % 2]).astype(np.float32)
        step = (2, 3)[k % 2]
        dist = rng.integers(-1, 60, (max(1, h // step), max(1, w // step))).astype(np.int32)
        gain_cells = (rng.random((h, w)) * 300).astype(np.float32)
        pose = (float(rng.normal(0, 1)), float(rng.normal(0, 1)), float(rng.uniform(-7, 7)))
        avoid = [(float(rng.normal(0, 2)), float(rng.normal(0, 2)), 1.0) for _ in range(k % 3)]
        keep = None if k % 3 == 0 else (int(rng.integers(0, h)), int(rng.integers(0, w)))
        ecfg = replace(cfg.explore, min_cluster_m=(0.4, 0.15)[k % 2], squeeze_cluster_m=(0.2, 0.1)[k % 2])
        args = (lab, nlab, zone, behind, dist, step, gain_cells, avoid, pose, 100 + k, 90 - k, 320, 0.05, keep, ecfg)
        a, b = navfast.frontier(L, *args), _groups_py(*args)
        for name, x, y in zip(("code", "score", "gain", "cr", "cc", "kd2", "kcr", "kcc"), a, b):
            assert x.dtype == y.dtype and np.array_equal(x, y), (k, name)
        codes |= set(a[0].tolist())
    assert codes == {0, 1, 2, 3, 4}


def test_next_goal_matches_python():
    """frontier.next_goal über den Kern: dasselbe Ziel, derselbe Weg."""
    from xgonav.frontier import next_goal
    from xgonav.gridmap import GridMap
    cfg = Config()
    rng = np.random.default_rng(4)
    ziele = 0
    for k in range(12):
        m = GridMap(replace(cfg.grid, native=False, file=""), CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)
        _, lo = _karte(rng, loecher=0.01)
        n0 = m.n // 2 - 60
        m.lo[n0:n0 + 120, n0:n0 + 120] = lo
        frei = np.argwhere(m.lo < -cfg.grid.l_show)
        r, c = frei[rng.integers(0, len(frei))]
        pose = ((c + 0.5 - m.n / 2) * m.res, (r + 0.5 - m.n / 2) * m.res, float(rng.uniform(-3, 3)))
        kw = dict(avoid=[(1.0, 1.0, 0.5)] if k % 2 else (), keep_frontier=(0.5, -0.5) if k % 3 else None)
        a = next_goal(m, pose, cfg.home, cfg.explore, **kw)
        b = next_goal(m, pose, replace(cfg.home, native=False), cfg.explore, **kw)
        assert a == b
        ziele += a.path is not None
    assert ziele > 6


def _lmap_pair(confirm=3):
    from xgonav.localmap import LocalMap
    geo = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    a = LocalMap(geo, 160, 120, max_age_s=1.5, confirm=confirm, confirm_radius_m=0.06)
    b = LocalMap(geo, 160, 120, max_age_s=1.5, confirm=confirm, confirm_radius_m=0.06, native=False)
    assert (a.backend, b.backend) == ("c++", "python")
    return geo, a, b


def test_localmap_matches_numpy():
    """Merkkarte: dieselben Punkte nach dem Ausdünnen, dieselben bestätigten Punkte (mit und ohne
    Zwischenspeicher) und dieselbe freie Strecke samt sperrenden Punkten."""
    from xgonav.freespace import FreeSpace
    rng = np.random.default_rng(7)
    for confirm in (3, 1):
        geo, a, b = _lmap_pair(confirm)
        hz = geo.horizon_row(0) / 4
        gesperrt = frei = 0
        base = np.clip(0.55 + 0.2 * np.sin(np.linspace(0, 9, 160)), 0, 1)
        for k in range(60):
            cols = np.clip(base + rng.normal(0, 0.02, 160), 0.0, 1.0)
            cols[rng.random(160) < 0.05] = np.nan
            cols[rng.random(160) < 0.1] = rng.uniform(0.2, 1.0)        # flackernde Fehlalarme
            fs = FreeSpace([0.5] * 5, cols, hz, np.zeros((120, 160), np.uint8), True, None)
            pd = float(rng.normal(0, 0.004))
            ds, dyaw = float(rng.normal(0.01, 0.01)), float(rng.normal(0, 0.03))
            for m in (a, b):
                m.move(ds, dyaw)
                m.add_freespace(fs, 0.1 * k, pd)
            assert _gleich(a.pts, b.pts) and np.array_equal(a.obs, b.obs)
            for reach in (0.4, 1.0, math.inf):
                assert _gleich(a.confirmed_points(reach), b.confirmed_points(reach))
            assert _gleich(a.confirmed_later(1.0)(), b.confirmed_later(1.0)())
            for _ in range(3):
                v, w = float(rng.uniform(-0.4, 0.6)), float(rng.uniform(-1.5, 1.5))
                rad = (None, 0.08)[k % 2]
                ha, hb = [], []
                sa = a.arc_clearance(v, w, horizon_s=1.2, radius=rad, hit_out=ha)
                sb = b.arc_clearance(v, w, horizon_s=1.2, radius=rad, hit_out=hb)
                assert sa == sb and len(ha) == len(hb)
                if ha:
                    assert _gleich(ha[0], hb[0])
                gesperrt += sa < math.inf
                frei += sa == math.inf
            ga, gb = a.gap_center(), b.gap_center()
            assert (ga is None) == (gb is None) and (ga is None or _gleich(ga, gb))
        assert gesperrt > 10 and frei > 10 and len(a.pts) > 200


def test_localmap_confirm_far_points_and_large_extent():
    """Bestätigen: Punkte weit auseinander (Suchraster zu groß -> alle gegen alle) und Punkte außerhalb der
    Nachbarmenge ergeben dasselbe wie numpy."""
    from xgonav.localmap import LocalMap
    rng = np.random.default_rng(1)
    for skala in (1.0, 1e5):
        _, a, b = _lmap_pair(3)
        pts = np.vstack([rng.normal(0, 0.3, (400, 2)), rng.normal(0, 0.3, (50, 2)) * skala])
        obs = rng.integers(0, 12, len(pts)).astype(np.int64)
        for m in (a, b):
            m.pts, m.obs, m.t = pts.copy(), obs.copy(), np.zeros(len(pts))
        for reach in (0.2, 0.5, math.inf):
            assert np.array_equal(a.confirmed_points(reach), b.confirmed_points(reach))
            assert np.array_equal(a.confirmed_later(reach)(), b.confirmed_later(reach)())
        assert 0 < len(a.confirmed_points(math.inf)) < len(pts)
    assert isinstance(a, LocalMap)


def test_localmap_add_edge_cases_match_numpy():
    """Eintragen: bekannter Bereich je Spalte (known_from), frei bis zum Horizont, alles verdeckt, leere Karte,
    mehr als MAX_POINTS Punkte und altes Vergessen ergeben dieselben Felder wie numpy."""
    from xgonav.freespace import FreeSpace
    rng = np.random.default_rng(12)
    geo, a, b = _lmap_pair(2)
    a.MAX_POINTS = b.MAX_POINTS = 400
    hz = geo.horizon_row(0) / 4
    geloescht = 0
    for k in range(120):
        cols = rng.uniform(0.0, 1.0, 160) if k % 4 else np.clip(0.5 + rng.normal(0, 0.01, 160), 0, 1)
        cols[rng.random(160) < 0.1] = np.nan
        cols[rng.random(160) < 0.15] = 1.0
        if k % 17 == 0:
            cols[:] = np.nan
        kf = None if k % 3 == 0 else np.clip(rng.uniform(-0.1, 0.4, 160), 0, 1)
        if kf is not None and k % 5 == 0:
            kf[rng.random(160) < 0.2] = 1.0
        fs = FreeSpace([0.5] * 5, cols, hz, np.zeros((120, 160), np.uint8), True, kf)
        pd, t = float(rng.normal(0, 0.01)), 0.1 * k + (5.0 if k == 60 else 0.0)
        ds, dyaw = float(rng.normal(0.02, 0.02)), float(rng.normal(0, 0.1))
        for m in (a, b):
            n0 = len(m.pts)
            m.move(ds, dyaw)
            m.add_freespace(fs, t, pd)
        geloescht += len(a.pts) < n0
        assert a.pts.dtype == b.pts.dtype and a.t.dtype == b.t.dtype and a.obs.dtype == b.obs.dtype
        assert _gleich(a.pts, b.pts) and np.array_equal(a.t, b.t) and np.array_equal(a.obs, b.obs), k
        assert a._obs_n == b._obs_n and len(a.pts) <= 400
    assert geloescht > 10 and len(a.pts) > 50


def test_wall_dist_matches_numpy():
    """Wandabstand der Raumkarte: Fenster am Rand, außerhalb der Karte, mit und ohne Wand in Reichweite."""
    from xgonav.gridmap import GridMap
    cfg = Config()
    geo = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    mn = GridMap(replace(cfg.grid, file=""), geo, 160, 120)
    mp = GridMap(replace(cfg.grid, native=False, file=""), geo, 160, 120)
    assert mn._nav is not None and mp._nav is None
    rng = np.random.default_rng(6)
    mn.lo[:] = np.where(rng.random(mn.lo.shape) < 0.01, cfg.grid.l_max, cfg.grid.l_min)
    mp.lo[:] = mn.lo
    nah = 0
    for _ in range(600):
        x, y = (float(v) for v in rng.uniform(-8.6, 8.6, 2))
        reach = float(rng.choice([0.05, 0.21, 0.3, 0.7]))
        dn, dp = mn.wall_dist(x, y, reach), mp.wall_dist(x, y, reach)
        assert dn == dp
        nah += dn < math.inf
    assert 50 < nah < 550


def test_path_target_matches_python():
    """Zielpunkt des Wegfolgers: Projektion auf den Weg, Vorausblick über Ecken und über das Ende hinaus,
    Abschnitte der Länge null."""
    from xgonav.behaviors import PathFollower
    cfg = Config()
    fn = PathFollower(cfg.behavior, cfg.robot, cfg.home)
    fp = PathFollower(cfg.behavior, cfg.robot, replace(cfg.home, native=False))
    rng = np.random.default_rng(8)
    for k in range(60):
        n = int(rng.integers(1, 9))
        path = [(float(a), float(b)) for a, b in rng.normal(0, 1.5, (n, 2))]
        if n > 3 and k % 4 == 0:
            path[2] = path[1]                            # doppelter Punkt
        fn.set_path(path)
        fp.set_path(path)
        for _ in range(12):
            x, y = (float(v) for v in rng.normal(0, 2, 2))
            la = None if k % 3 == 0 else float(rng.choice([0.15, 0.3, 0.45, 5.0]))
            assert tuple(fn.target(x, y, la)) == tuple(fp.target(x, y, la))


def test_gridmap_observe_matches_numpy():
    """Raumkarte: Strahlen (_rays) und Eintragen einer Beobachtung in einem Aufruf (Freistrahlen, Treffer,
    Bestätigung aus zwei Abständen) ergeben dieselben Felder wie numpy; mit messbarem Bereich je Spalte,
    Strahlen bis zum Horizont, verdeckten Spalten, wechselnder Körperhöhe und am Kartenrand."""
    from xgonav.freespace import FreeSpace
    from xgonav.gridmap import GridMap, _grid_lib
    if _grid_lib() is None:
        pytest.skip("Raumkarten-Kern nicht gebaut (make -C xgonav/native)")
    cfg = Config()
    for dr, schritt in ((0.2, 2), (0.0, 1)):
        geo = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0, cam_fwd_mm=30.0)
        gc = replace(cfg.grid, confirm_dr_m=dr, column_step=schritt, file="")
        mn, mp = GridMap(gc, geo, 160, 120), GridMap(replace(gc, native=False), geo, 160, 120)
        assert (mn.backend, mp.backend) == ("c++", "python")
        rng = np.random.default_rng(21)
        hz = geo.horizon_row(0) / 4
        pose, belegt = np.zeros(3), 0
        base = np.clip(0.5 + 0.25 * np.sin(np.linspace(0, 7, 160)), 0, 1)
        for k in range(250):
            cols = (rng.uniform(0.0, 1.0, 160) if k % 5 == 0 else np.clip(base + rng.normal(0, 0.03, 160), 0, 1))
            cols[rng.random(160) < 0.1] = np.nan
            cols[rng.random(160) < 0.15] = 1.0                       # frei bis Horizont
            cols = cols.astype(np.float32) if k % 2 else cols
            kf = None if k % 3 == 0 else np.clip(rng.uniform(-0.1, 0.3, 160), 0, 1)
            if kf is not None and k % 4 == 0:
                kf[rng.random(160) < 0.2] = 1.0
            fs = FreeSpace([0.5] * 5, cols, hz, np.zeros((120, 160), np.uint8), True, kf)
            pose += (rng.normal(0.02, 0.03), rng.normal(0, 0.03), rng.normal(0, 0.15))
            if k == 150:
                pose[:] = (7.9, -7.95, 1.0)                          # Kartenrand: Strahlen laufen hinaus
            if k == 100:
                geo.body_height_mm = 100.0                           # andere Kamerahöhe ab hier
            pd = float(rng.normal(0, 0.01))
            for a, b in zip(mn._rays(fs, pd), mp._rays(fs, pd)):
                assert a.dtype == b.dtype and _gleich(a, b), k
            mn.add_freespace(fs, pd, tuple(pose))
            mp.add_freespace(fs, pd, tuple(pose))
            assert np.array_equal(mn.lo, mp.lo), k
            belegt = max(belegt, int((mn.lo > 0).sum()))
        assert np.abs(mn.lo).max() > 1.0 and mn.updates == mp.updates == 250
        assert belegt > 5                                     # Treffer wurden eingetragen (naher Bogen: wenige Zellen)
        if dr > 0:
            assert _gleich(mn.rmin, mp.rmin) and _gleich(mn.rmax, mp.rmax)
            assert np.array_equal(mn.conf, mp.conf) and mn.conf.any() and (np.isfinite(mn.rmin) & ~mn.conf).any()
