"""tools/map_replay.py: Raumkarte aus einer Aufnahme je Variante, ohne Bilder (Bodengrenze künstlich)."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import map_replay as M  # noqa: E402
from label import ground_row, make_geo  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.freespace import FreeSpace  # noqa: E402

FW, FH = 160, 120


def _cfg():
    cfg = Config.neutral()
    cfg.grid.native = False
    return cfg


def _wand_view(geo, th: float, wand_x: float = 1.0) -> FreeSpace:
    """Bodengrenze einer Wand x = wand_x, gesehen vom Ursprung mit Kurs th."""
    sc = geo.W / FW
    bear = geo.bearing((np.arange(FW) + 0.5) * sc)
    ang = th + bear
    fuss = geo.cam_fwd * math.cos(th)
    with np.errstate(divide="ignore"):
        d = np.where(np.cos(ang) > 0.05, (wand_x - fuss) / np.cos(ang) * np.cos(bear), np.inf)
    hz = geo.horizon_row(0.0) / sc
    cols = np.clip((FH - ground_row(geo, d) / sc) / (FH - hz), 0.0, 1.0)
    return FreeSpace(sectors=[1.0] * 5, columns=cols, horizon_row=hz, valid=True)


def _schwenk(n: int = 150, fps: float = 25.0, dps: float = 60.0, weite: float = 25.0):
    """Drehen auf der Stelle, hin und her zwischen ±weite Grad."""
    t = np.arange(n) / fps
    per = 4 * weite / dps
    ph = (t / per) % 1.0
    deg = np.where(ph < 0.5, -weite + 4 * weite * ph, 3 * weite - 4 * weite * ph)
    return t, np.radians(deg)


def _karten(specs, **kw):
    cfg = _cfg()
    geo = make_geo(cfg)
    t, th = _schwenk(**kw)
    bahn = M.Bahn(t, np.zeros_like(t), np.zeros_like(t), th)
    views = {i: _wand_view(geo, th[i]) for i in range(0, len(t), 2)}
    return M.rechne(t, bahn, views, np.zeros(len(t)), cfg, geo, FW, FH, [M.parse_variante(s) for s in specs])


def _belegt(gm):
    return gm.occupied_cells()


def test_parse_variante():
    assert M.parse_variante("basis:") == ("basis", {})
    assert M.parse_variante("a:versatz=-40, grid.max_range_m=1.5") == ("a", {"versatz": "-40", "grid.max_range_m": "1.5"})


def test_wand_liegt_richtig_mit_pose_des_bildes():
    (_, gm, z), = _karten(["genau:versatz=0,sperre=1000"])
    occ = _belegt(gm)
    assert len(occ) > 10 and z["eingetragen"] == 75
    assert np.abs(occ[:, 0] - 1.0).max() <= 0.08          # eine Zelle + Rundung


def test_alte_pose_beim_drehen_verschmiert_die_wand():
    genau, alt = _karten(["genau:versatz=0,sperre=1000", "alt:versatz=-100,sperre=1000"])
    a, b = _belegt(genau[1]), _belegt(alt[1])
    # 60 °/s × 100 ms = 6°: die Wand steht einmal links-, einmal rechtsherum verdreht in der Karte
    assert np.abs(b[:, 0] - 1.0).max() > np.abs(a[:, 0] - 1.0).max() + 0.03
    assert alt[2]["Zwischenzellen"] > 3 * genau[2]["Zwischenzellen"] + 10


def test_sperre_wie_app_und_zur_bildzeit():
    live, bild = _karten(["live:sperre=40", "bild:sperre=40,quelle=pose"])
    # Dauernd 60 °/s: beide Quellen sperren fast alles (an den Umkehrpunkten ist die Rate kleiner)
    assert live[2]["gesperrt"] > 60 and bild[2]["gesperrt"] > 60
    frei, = _karten(["langsam:sperre=40"], dps=20.0)
    assert frei[2]["gesperrt"] == 0


def test_grid_parameter():
    a, b = _karten(["null:versatz=0,sperre=1000", "kurz:versatz=0,sperre=1000,grid.max_range_m=0.5"])
    # Wand bei 1 m: mit Treffern nur bis 0,5 m wird nichts belegt
    assert len(_belegt(a[1])) > 0 and len(_belegt(b[1])) == 0
    assert a[1].cfg.max_range_m == Config.neutral().grid.max_range_m and b[1].cfg.max_range_m == 0.5


def test_stuetzstellen_treppe_und_stillstand():
    t = np.arange(10) * 0.04
    x = np.array([0, 0, 1, 1, 2, 2, 2, 2, 2, 2], float)       # Stufen je 2 Bilder, dann 0,2 s Stand
    k = M.stuetzstellen(t, x, np.zeros(10), np.zeros(10))
    assert k.tolist() == [0, 2, 4, 9]
    b = M.Bahn(t, x, np.zeros(10), np.zeros(10))
    assert abs(float(b.at(0.04)[0]) - 0.5) < 1e-9             # zwischen den Stufen interpoliert


def test_roll_dreht_die_grenze_um_die_bildmitte():
    cfg = _cfg()
    geo = make_geo(cfg)
    v = FreeSpace(columns=np.full(FW, 0.5), horizon_row=67.0, valid=True)
    r = M.roll_spalten(v, geo, FW, FH, 2.0)
    d = r.columns - 0.5
    assert d[0] > 0 > d[-1] and abs(d[0] + d[-1]) < 0.01 + abs(geo.cx - geo.W / 2) / 100
    assert M.roll_spalten(v, geo, FW, FH, 0.0) is v
    frei = FreeSpace(columns=np.array([1.0, np.nan, 0.99]), horizon_row=67.0, valid=True)
    assert np.array_equal(M.roll_spalten(frei, geo, 3, FH, 3.0).columns, frei.columns, equal_nan=True)


def test_doppelt_zaehlt_zellen_zwischen_nahen_waenden():
    cfg = _cfg()
    geo = make_geo(cfg)
    from xgonav.gridmap import GridMap
    gm = GridMap(cfg.grid, geo, FW, FH)
    gm.lo[100, 100:140] = 3.0                                  # eine Wand: nichts dazwischen
    assert M.doppelt(gm) == (40, 0)
    gm.lo[103, 100:140] = 3.0                                  # zweite Wand 15 cm daneben
    assert M.doppelt(gm) == (80, 80)
    gm.lo[103, 100:140] = 0.0
    gm.lo[110, 100:140] = 3.0                                  # 50 cm daneben: zwei Wände, keine Doppelwand
    assert M.doppelt(gm) == (80, 0)


def test_quer_misst_die_flurbreite_der_karte():
    cfg = _cfg()
    geo = make_geo(cfg)
    from xgonav.gridmap import GridMap
    gm = GridMap(cfg.grid, geo, FW, FH)
    n = gm.n // 2
    gm.lo[n + 12, :] = 3.0                                     # Wand bei y = +0,6 m (Zelle 0,60..0,65)
    gm.lo[n - 10, :] = 3.0                                     # Wand bei y = −0,5 m (Zelle −0,50..−0,45)
    t = np.arange(50) * 0.04
    b = M.Bahn(t, t * 0.5, np.zeros(50), np.zeros(50))        # fährt in x mitten durch den Flur
    breite, stellen = M.quer(gm, b)
    assert abs(breite - 1.05) <= 0.05 and stellen == 10
    gm.lo[n - 10, :] = 0.0                                     # nur eine Wand: keine Breite
    assert M.quer(gm, b) == (0.0, 0)


def test_imu_proben():
    assert M.imu_proben([{"idx": 1}, {"imu": []}]) is None
    ms = [{"imu": [[0.0, 1.0, 0.0, 350.0], [0.01, 1.2, 0.0, 359.0]]}, {"imu": [[0.02, 1.4, 0.0, 8.0]]}]
    ti, ro, ya = M.imu_proben(ms)
    assert ti.tolist() == [0.0, 0.01, 0.02] and ro[-1] == 1.4 and ya.tolist() == [0.0, 9.0, 18.0]
