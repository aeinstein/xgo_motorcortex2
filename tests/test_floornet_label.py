"""Geometrie und Bahn der FloorNet-Labels (tools/floornet/label.py) gegen bekannte Werte."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "floornet"))

import label as L  # noqa: E402
from xgonav.config import Config, GridCfg  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402


def _geo():
    return CamGeometry(640, 480, 88.5, 5.18, 43.0, 85.0)


@pytest.mark.parametrize("d", [0.3, 0.5, 1.0, 1.5, 2.2])
@pytest.mark.parametrize("pitch", [-0.06, 0.0, 0.05])
def test_ground_row_ist_umkehrung_von_ground_dist(d, pitch):
    g = _geo()
    assert g.ground_dist(float(L.ground_row(g, d, pitch)), pitch) == pytest.approx(d, abs=1e-6)


def _grid(g, cells):
    """Karte: überall frei bis 3 m vor dem Start, dazu belegte Zellen."""
    cfg = Config.neutral().grid
    cfg.native = False
    gm = GridMap(cfg, g, 160, 120)
    r, c = np.mgrid[0:gm.n, 0:gm.n]
    wx, wy = (c + 0.5 - gm.n / 2) * gm.res, (r + 0.5 - gm.n / 2) * gm.res
    gm.lo[(wx > -0.3) & (wx < 3.0) & (np.abs(wy) < 3.0)] = -3.0
    for ax, ay in cells:
        rr, cc = gm.cell(ax, ay)
        gm.lo[rr, cc] = 3.0
    return gm


def test_wand_voraus_trifft_die_zeile():
    g = _geo()
    gm = _grid(g, [(1.025, y) for y in np.arange(-2.5, 2.5, 0.025)])
    rows, kinds = L.cast_labels(gm, g, np.array([0.0]), np.array([0.0]), np.array([0.0]), np.array([0.02]))
    assert (kinds == L.EXACT).all()
    # Wand bei x = 1,0 (Zellgrenze): jede Spalte sieht dieselbe Vorwärtsentfernung, also dieselbe Zeile
    want = float(L.ground_row(g, 1.0, 0.02))
    assert np.abs(rows - want).max() < 1.5        # 1 px im Vollbild sind bei 1 m ~2,4 cm


def test_gedrehter_rider_sieht_die_wand_nicht_mehr_voraus():
    g = _geo()
    gm = _grid(g, [(1.025, y) for y in np.arange(-0.2, 0.2, 0.025)])       # nur ein kurzes Stück voraus
    a = np.array([0.0])
    _, k0 = L.cast_labels(gm, g, a, a, np.array([0.0]), a)
    _, k1 = L.cast_labels(gm, g, a, a, np.array([math.pi / 2]), a)
    assert k0[0, 40] == L.EXACT
    assert k1[0, 40] != L.EXACT
    # Linksdrehung: die Stelle wandert im Bild nach rechts, also in höhere Spalten
    _, k2 = L.cast_labels(gm, g, a, a, np.array([math.radians(30)]), a)
    hit = np.nonzero(k2[0] == L.EXACT)[0]
    assert hit.size and hit.mean() > 40


def test_unbekanntes_gilt_als_mindestens_frei_oder_nichts():
    g = _geo()
    cfg = Config.neutral().grid
    cfg.native = False
    gm = GridMap(cfg, g, 160, 120)
    r, c = np.mgrid[0:gm.n, 0:gm.n]
    wx = (c + 0.5 - gm.n / 2) * gm.res
    gm.lo[(wx > -0.3) & (wx < 0.8)] = -3.0                                   # frei bis 0,8 m, dahinter unbekannt
    a = np.array([0.0])
    rows, kinds = L.cast_labels(gm, g, a, a, a, a)
    assert kinds[0, 40] == L.ATLEAST
    assert g.ground_dist(float(rows[0, 40])) == pytest.approx(0.8, abs=0.06)
    gm.lo[:] = 0.0                                                           # gar nichts bekannt
    _, kinds = L.cast_labels(gm, g, a, a, a, a)
    assert (kinds == L.IGNORE).all()


def test_row_dist_ist_umkehrung_von_ground_row():
    g = _geo()
    assert float(L.row_dist(g, L.ground_row(g, 0.9, 0.03), 0.03)) == pytest.approx(0.9, abs=1e-6)
    assert np.isnan(L.row_dist(g, 10.0, 0.0))                                # über dem Horizont


def _meta(t, x, y, th_deg, yaw, **kw):
    m = {"t": t, "pose": [x, y, th_deg], "yaw_drv": yaw, "balancing": True, "pitch_src": "imu", "mode": "explore"}
    m.update(kw)
    return m


def test_bahn_nimmt_den_imu_kurs_statt_des_radkurses():
    # 1 m geradeaus laut Pose, die aber (Radschlupf) 90° zu wenig Kurs kennt: Die IMU sagt, der Rider fuhr nach links
    ms = [_meta(i * 0.05, 0.01 * i, 0.0, 0.0, 90.0) for i in range(101)]
    x, y, th, _ = L.trajectory(ms)
    assert x[-1] == pytest.approx(1.0, abs=1e-6) and y[-1] == pytest.approx(0.0, abs=1e-6)
    ms = [_meta(0.0, 0.0, 0.0, 0.0, 0.0), _meta(0.05, 0.0, 0.0, 0.0, 90.0)]    # Drehen auf der Stelle
    ms += [_meta(0.05 + i * 0.05, 0.01 * i, 0.0, 0.0, 90.0) for i in range(1, 101)]
    x, y, th, _ = L.trajectory(ms)
    assert th[-1] == pytest.approx(math.pi / 2)
    assert x[-1] == pytest.approx(0.0, abs=1e-3) and y[-1] == pytest.approx(1.0, abs=1e-3)


def test_kurs_treppe_wird_interpoliert_und_ueberlauf_stoert_nicht():
    # 5-Hz-Treppe: 180°/s Drehung, Werte nur alle 0,2 s, mit Überlauf bei 360°
    ms = [_meta(i * 0.04, 0.0, 0.0, 0.0, (350.0 + 36.0 * (i // 5)) % 360.0) for i in range(51)]
    _, _, th, rate = L.trajectory(ms)
    assert np.all(np.diff(th) >= -1e-9)                                     # keine Rücksprünge
    assert np.diff(th)[10:40].std() < 0.02                                  # gleichmäßig statt Treppe
    assert 120 < np.median(rate) < 240


def test_select_segment_ueberspringt_aufstehen_und_posesprung():
    ms = [_meta(i * 0.04, 0.0, 0.0, 0.0, 0.0, mode="idle", balancing=False) for i in range(10)]
    ms += [_meta(0.4 + i * 0.04, 0.01 * i, 0.0, 0.0, 0.0) for i in range(30)]
    ms += [_meta(2.0 + i * 0.04, 5.0 + 0.01 * i, 0.0, 0.0, 0.0) for i in range(5)]     # kurzes Stück nach dem Sprung
    seg = L.select_segment(ms)
    assert len(seg) == 30
    assert seg[0]["t"] == pytest.approx(0.4)


def test_load_run_nimmt_den_laengsten_lauf_und_nur_seine_bilder(tmp_path):
    lines = [{"idx": i, "t": float(i), "file": "%06d.jpg" % i} for i in range(5)]       # Lauf 0
    lines += [{"idx": i, "t": 100.0 + i, "file": "%06d.jpg" % i} for i in range(3)]     # Lauf 1 überschreibt 0-2
    (tmp_path / "meta.jsonl").write_text("\n".join(json.dumps(m) for m in lines) + '\n{"idx": 9, "t"', encoding="utf-8")
    for i in range(5):
        (tmp_path / ("%06d.jpg" % i)).write_bytes(b"x")
    ms = L.load_run(str(tmp_path))
    assert [m["file"] for m in ms] == ["000003.jpg", "000004.jpg"]           # 0-2 gehören nicht mehr zu Lauf 0
    assert ms[0]["t"] == 3.0
