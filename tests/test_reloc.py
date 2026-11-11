"""Kartenabgleich (xgonav/reloc.py) auf künstlichen Karten: findet er die Lage, und meldet er
"unsicher", wo sie nicht bestimmbar ist (gleiche Ecken, Flur, zu wenig Wand)?"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav import reloc  # noqa: E402
from xgonav.config import Config  # noqa: E402

N, RES, L_SHOW = 320, 0.05, 0.7


def _px(pts):
    return np.round(np.asarray(pts, float) / RES + N / 2 - 0.5).astype(np.int32).reshape(-1, 1, 2)


def karte(umriss, kisten=()):
    """Raum als Log-Odds: innen frei, Wände und Kisten belegt, außen unbekannt (Punkte in m, x = Spalte)."""
    lo = np.zeros((N, N), np.float32)
    cv2.fillPoly(lo, [_px(umriss)], -2.0)
    cv2.polylines(lo, [_px(umriss)], True, 2.0, 1)
    for k in kisten:
        cv2.fillPoly(lo, [_px(k)], 2.0)
    return lo


def rundumblick(lo, pose, rmax=1.25, rausch=0.03, ausreisser=0.1, seed=1, winkel=None):
    """Was der Rider an pose sähe: je Richtung der erste belegte Punkt bis rmax (verrauscht, ein Teil
    der Strahlen endet zu früh) und Freiproben davor. Punkte im Roboter-KS."""
    rng = np.random.default_rng(seed)
    occ = lo > L_SHOW
    s = np.arange(0.05, rmax, 0.01)
    hits, free = [], []
    for a in (np.radians(np.arange(0, 360, 1.5)) if winkel is None else winkel):
        x = pose[0] + s * math.cos(pose[2] + a)
        y = pose[1] + s * math.sin(pose[2] + a)
        r = np.floor(y / RES + N / 2).astype(int)
        c = np.floor(x / RES + N / 2).astype(int)
        i = np.flatnonzero(occ[r, c])
        d = s[i[0]] + rng.normal(0, rausch) if len(i) else None
        if d is not None and rng.random() < ausreisser:
            d *= rng.uniform(0.5, 0.9)
        if d is not None:
            hits.append((d * math.cos(a), d * math.sin(a)))
        for f in np.arange(0.15, (d - 0.15) if d is not None else rmax, 0.2):
            free.append((f * math.cos(a), f * math.sin(a)))
    hits = np.array(hits).reshape(-1, 2)
    w = 1.0 / (1.0 + (np.hypot(hits[:, 0], hits[:, 1]) / 0.8) ** 2)
    cfg = Config().reloc
    hits, w = reloc._duenn(hits, cfg.hit_res_m, w)
    free, _ = reloc._duenn(np.array(free).reshape(-1, 2), cfg.free_res_m)
    return reloc.Scan(hits, w, free)


L_RAUM = [(-2, -1.5), (2, -1.5), (2, 0.5), (0.5, 0.5), (0.5, 2), (-2, 2)]
KISTE = [(-1.0, -1.5), (-0.7, -1.5), (-0.7, -1.1), (-1.0, -1.1)]
RECHTECK = [(-1.5, -1.0), (1.5, -1.0), (1.5, 1.0), (-1.5, 1.0)]
FLUR = [(-4, -0.6), (4, -0.6), (4, 0.6), (-4, 0.6)]


def _feld(lo):
    return reloc.Feld(lo, RES, L_SHOW, Config().reloc)


def _fehler(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), math.degrees(reloc._winkel_diff(a[2], b[2]))


def test_reloc_global_findet_lage():
    """Ecke mit Kiste davor: die Lage ist eindeutig und wird auf 5 cm / 2° gefunden."""
    cfg = Config().reloc
    lo = karte(L_RAUM, [KISTE])
    wahr = (-1.35, -0.85, math.radians(30))
    e = reloc.suche_global(_feld(lo), rundumblick(lo, wahr), cfg)
    assert e.ok, (e.grund, e.kandidaten)
    dm, dg = _fehler(e.pose, wahr)
    assert dm < 0.05 and dg < 2.0, (dm, dg)
    assert e.anteil > 0.7 and e.zweit < cfg.unique_ratio


def test_reloc_global_gleiche_ecken_mehrdeutig():
    """Leeres Rechteck: alle vier Ecken sehen gleich aus. Er darf keine annehmen."""
    lo = karte(RECHTECK)
    e = reloc.suche_global(_feld(lo), rundumblick(lo, (-0.9, -0.4, 0.3)), Config().reloc)
    assert not e.ok and e.grund == "mehrdeutig", (e.grund, e.kandidaten)


def test_reloc_flur_gleitet():
    """Flur: längs ist die Lage unbestimmt. Global unsicher; lokal wird nur quer und der Kurs
    korrigiert, längs bleibt die Odometrie."""
    cfg = Config().reloc
    lo = karte(FLUR)
    wahr = (0.3, 0.1, math.radians(5))
    scan = rundumblick(lo, wahr)
    feld = _feld(lo)
    e = reloc.suche_global(feld, scan, cfg)
    assert not e.ok and e.grund in ("gleitet", "mehrdeutig"), e.grund
    geschaetzt = (wahr[0] + 0.15, wahr[1] - 0.10, wahr[2] + math.radians(3))
    e = reloc.abgleich_lokal(feld, scan, geschaetzt, cfg)
    assert e.ok and e.teil, e.grund
    assert abs(e.pose[1] - wahr[1]) < 0.03                    # quer gefunden
    assert abs(e.pose[0] - geschaetzt[0]) < 0.02              # längs nicht angefasst
    assert abs(math.degrees(e.pose[2] - wahr[2])) < 2.5       # 2 m Wand auf 5-cm-Zellen: mehr geht nicht


def test_reloc_lokal_zieht_drift_nach():
    cfg = Config().reloc
    lo = karte(L_RAUM, [KISTE])
    wahr = (-1.35, -0.85, math.radians(30))
    scan = rundumblick(lo, wahr, seed=3)
    e = reloc.abgleich_lokal(_feld(lo), scan, (wahr[0] + 0.12, wahr[1] - 0.17, wahr[2] + math.radians(-5)), cfg)
    assert e.ok and not e.teil, e.grund
    dm, dg = _fehler(e.pose, wahr)
    assert dm < 0.04 and dg < 1.5, (dm, dg)
    # außerhalb des Fensters: kein Ergebnis statt eines falschen
    e = reloc.abgleich_lokal(_feld(lo), scan, (wahr[0] + 0.6, wahr[1], wahr[2]), cfg)
    assert not e.ok


def test_reloc_zu_wenig_wand():
    cfg = Config().reloc
    lo = karte(L_RAUM, [KISTE])
    feld = _feld(lo)
    mitte = rundumblick(lo, (-0.6, 0.0, 0.0))                   # Raummitte: fast keine Wand in Reichweite
    assert mitte.n < cfg.min_points
    for e in (reloc.suche_global(feld, mitte, cfg), reloc.abgleich_lokal(feld, mitte, (-0.6, 0.0, 0.0), cfg)):
        assert not e.ok and e.grund == "zu wenig Wand"
    leer = reloc.Feld(np.zeros((N, N), np.float32), RES, L_SHOW, cfg)
    assert reloc.suche_global(leer, rundumblick(lo, (-1.35, -0.85, 0.5)), cfg).grund == "Karte leer"


def test_reloc_freistrahl_trennt_seiten():
    """Eine Wand von vorn oder von hinten gesehen ergibt dieselben Treffer. Die Freiproben liegen
    nur von der richtigen Seite auf freiem Boden."""
    cfg = Config().reloc
    lo = np.zeros((N, N), np.float32)
    cv2.fillPoly(lo, [_px([(-1.5, -1.0), (1.5, -1.0), (1.5, 1.0), (-1.5, 1.0)])], -2.0)
    cv2.line(lo, tuple(_px([(-1.5, 1.0)])[0, 0]), tuple(_px([(1.5, 1.0)])[0, 0]), 2.0, 1)
    wahr = (0.0, 0.4, math.radians(90))
    scan = rundumblick(lo, wahr, rausch=0.0, ausreisser=0.0)
    feld = _feld(lo)

    def wert(pose, lam):
        r, c = feld.cell(pose[0], pose[1])
        return float(reloc._wertung(feld.treffer, feld.kosten, cfg.unknown_weight, RES, scan, pose[2],
                                    (0.0, 0.0), (r, r, c, c), lam)[0, 0])
    hinten = (0.0, 1.6, math.radians(-90))
    assert abs(wert(wahr, 0.0) - wert(hinten, 0.0)) < 0.1      # nur Treffer: kaum zu unterscheiden
    assert wert(wahr, cfg.free_weight) > wert(hinten, cfg.free_weight) + 0.1


def test_reloc_puffer_und_schritt():
    cfg = Config().reloc
    p = reloc.ScanPuffer(cfg, window_s=5.0, window_m=1.0)
    pkt = np.array([[1.0, 0.0]])
    p.add(0.0, (0.0, 0.0, 0.0), pkt, np.ones(1), np.zeros((0, 2)))
    p.add(1.0, (0.5, 0.0, math.pi / 2), pkt, np.ones(1), np.zeros((0, 2)))
    s = p.scan((0.5, 0.0, math.pi / 2))                        # im KS der zweiten Pose
    got = {tuple(np.round(h, 3)) for h in s.hits}
    assert got == {(1.0, 0.0), (0.0, -0.5)}                    # erster Punkt liegt jetzt rechts
    p.add(2.0, (1.2, 0.0, 0.0), pkt, np.ones(1), np.zeros((0, 2)))
    assert len(p) == 2                                         # älteste Beobachtung liegt mehr als 1 m zurück
    p.add(9.0, (1.2, 0.0, 0.0), pkt, np.ones(1), np.zeros((0, 2)))
    assert len(p) == 1                                         # und nach 5 s ohnehin
    # Korrektur als starre Transformation: k ∘ von = nach
    von, nach = (1.0, 2.0, 0.3), (1.1, 1.9, 0.35)
    k = reloc.korrektur(von, nach)
    assert np.allclose(reloc.anwenden(k, von), nach)
    assert reloc.schritt(von, (1.01, 2.0, 0.3), cfg) is None    # Totband
    neu = reloc.schritt(von, (1.5, 2.0, 0.3 + math.radians(20)), cfg)
    assert abs(neu[0] - von[0] - cfg.max_step_m) < 1e-9 and abs(neu[2] - von[2] - math.radians(cfg.max_step_deg)) < 1e-9


def test_reloc_scan_punkte_wie_karte():
    """Die Punkte des Abgleichs liegen dort, wo GridMap.add_freespace dieselbe Beobachtung einträgt."""
    from xgonav.freespace import FreeSpace
    from xgonav.geometry import CamGeometry
    from xgonav.gridmap import GridMap
    cfg = Config()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    m = GridMap(cfg.grid, g, 160, 120)
    hz = g.horizon_row(0) / 4
    ys = np.arange(int(hz * 4) + 1, 480)
    y1 = ys[np.nanargmin(np.abs(g.ground_dist_np(ys) - 1.0))]
    col = (120 - y1 / 4) / (120 - hz)
    fs = FreeSpace([col] * 5, np.full(160, col), hz, np.zeros((120, 160), np.uint8), True, None)
    for _ in range(3):
        m.add_freespace(fs, 0.0)
    hits, w, free = reloc.scan_punkte(m, fs, 0.0, cfg.reloc)
    assert len(hits) == 80 and np.all(np.abs(hits[:, 0] - 1.0) < 0.03)       # gerade Wand 1 m vor der Radachse
    assert w.min() > 0.2 and w.max() < 0.45
    assert len(free) > 100 and free[:, 0].max() < 0.9 and free[:, 0].min() > 0.1
    feld = reloc.Feld(m.lo, m.res, cfg.grid.l_show, cfg.reloc)
    scan = reloc.Scan(hits, w, free)
    assert reloc._anteil(feld, scan, (0.0, 0.0, 0.0))[0] > 0.95


# ---------------------------------------------------------------- Pose-Zustand und Kartensperre (Stufe 1)
def _wand_fs(g):
    from xgonav.freespace import FreeSpace
    hz = g.horizon_row(0) / 4
    ys = np.arange(int(hz * 4) + 1, 480)
    y1 = ys[np.nanargmin(np.abs(g.ground_dist_np(ys) - 1.0))]
    col = (120 - y1 / 4) / (120 - hz)
    return FreeSpace([col] * 5, np.full(160, col), hz, np.zeros((120, 160), np.uint8), True, None)


def test_gridmap_pose_unbekannt_sperrt_karte(tmp_path):
    """Pose unbekannt: Beobachtungen, gefahrener Weg und Freiräumen ändern die Karte nicht; die Pose läuft
    weiter mit. Die Pose wird mitgespeichert, nach dem Laden gilt trotzdem der Startpunkt."""
    from xgonav.geometry import CamGeometry
    from xgonav.gridmap import GridMap
    cfg = Config()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    m = GridMap(cfg.grid, g, 160, 120)
    fs = _wand_fs(g)
    for _ in range(3):
        m.add_freespace(fs, 0.0)
    assert m.empty() is False and m.stats()["pose_ok"] is True
    vorher, n = m.lo.copy(), m.updates
    m.set_pose_ok(False)
    m.add_freespace(fs, 0.0)
    m.move(0.9, 0.0)                                            # bis an die Wand: räumte sonst die Wand frei
    m.clear_here(0.3)
    assert m.updates == n and np.array_equal(m.lo, vorher)
    assert len(m.path) == 1 and abs(m.pose()[0] - 0.9) < 1e-9   # Pose läuft mit, Weg nicht
    assert m.stats()["pose_ok"] is False
    assert m.render(100)[0].shape == (100, 100, 3)
    f = str(tmp_path / "karte.npz")
    assert m.save(f)
    m2 = GridMap(cfg.grid, g, 160, 120)
    assert m2.load(f)
    assert np.allclose(m2.saved_pose, (0.9, 0.0, 0.0)) and m2.pose() == (0.0, 0.0, 0.0)
    m.set_pose_ok(True)                                          # bestätigt: der Weg beginnt hier neu
    assert m.path == [(0.9, 0.0)]
    m.move(0.06, 0.0)
    assert len(m.path) == 2
    m.set_pose_ok(False)
    m.relocate_to_start()                                        # "steht am Startpunkt" macht die Pose gültig
    assert m.pose_ok and m.pose() == (0.0, 0.0, 0.0)


def _app_reloc(enabled=True, file=""):
    from xgonav.app import App
    from xgonav.robot import SimRobot
    from xgonav.sim import SimWorld
    cfg = Config()
    cfg.output.web_port = 0
    cfg.reloc.enabled = enabled
    cfg.grid.file = file
    robot = SimRobot(cfg.robot)
    return App(cfg, robot, SimWorld(cfg.camera, robot, None), imu=None)


def test_app_pose_unbekannt_sperrt_fahrten():
    import time
    from xgonav.app import ModeNotAllowed
    app = _app_reloc()
    app._pose_unbekannt("Test")
    assert app.gridmap.pose_ok                                   # leere Karte: nichts zu verlieren
    app.gridmap.lo[100, 100] = 2.0
    app._pose_bal = True
    app.robot.state.balancing = False                            # abgesetzt
    app._pose_watch(time.monotonic())
    assert not app.gridmap.pose_ok and "abgesetzt" in app.pose_grund
    for name in ("home", "explore"):
        try:
            app.set_mode(name)
            assert False
        except ModeNotAllowed as e:
            assert "Pose" in str(e)
    try:
        app.set_pose_state("hier")                               # liegend: beim Aufstehen rollt er noch vor
        assert False
    except ModeNotAllowed:
        pass
    app.robot.state.balancing = True
    app.set_pose_state("hier")
    assert app.gridmap.pose_ok and app.pose_grund == ""
    app.set_pose_state("verwerfen")
    assert not app.gridmap.pose_ok
    app.set_pose_state("start")
    assert app.gridmap.pose_ok
    # während einer Fahrt nach der Karte: Modus endet
    app.behavior = app._make("home")
    app._pose_unbekannt("Test")
    assert app._pending_mode == ("idle", None)
    # ohne [reloc] enabled bleibt alles wie bisher
    aus = _app_reloc(enabled=False)
    aus.gridmap.lo[100, 100] = 2.0
    aus._pose_unbekannt("Test")
    aus._pose_bal = True
    aus.robot.state.balancing = False
    aus._pose_watch(time.monotonic())
    assert aus.gridmap.pose_ok


def test_app_neustart_mit_karte_pose_unbekannt(tmp_path):
    """Dienststart mit gespeicherter Karte: Pose unbekannt, angezeigt wird die Pose vom Beenden."""
    f = str(tmp_path / "karte.npz")
    a = _app_reloc(file="")
    a.gridmap.lo[100, 100] = 2.0
    a.gridmap.move(0.5, 0.3)
    assert a.gridmap.save(f)
    b = _app_reloc(file=f)
    assert not b.gridmap.pose_ok and "Neustart" in b.pose_grund
    assert np.allclose(b.gridmap.pose(), a.gridmap.pose())
    c = _app_reloc(enabled=False, file=f)                        # aus: wie bisher Pose = Startpunkt
    assert c.gridmap.pose_ok and c.gridmap.pose() == (0.0, 0.0, 0.0)
