"""FloorNet: Rechnungen von tools/floornet/vergleich_fahrten.py, ohne torch."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "floornet"))

import vergleich_fahrten as V  # noqa: E402


def test_spruenge_zaehlt_nur_grosse():
    r = np.full(80, 300.0)
    assert V.spruenge(r) == 0
    r[10:15] = 400.0            # Spitze: zwei Sprünge
    r[40] = 330.0               # klein
    assert V.spruenge(r) == 2
    r[60] = np.nan              # unbekannt zählt nicht
    assert V.spruenge(r) == 2


def test_regelung_wie_safety():
    assert V.regelung(0.10, 0.22, 0.55) == "halt"
    assert V.regelung(0.14, 0.22, 0.55) == "bremst"
    assert V.regelung(0.60, 0.22, 0.55) == "frei"


def test_gefahren_nur_geradeaus_und_vorwaerts():
    t = np.arange(40) * 0.1
    x, y, th = t * 0.2, np.zeros(40), np.zeros(40)
    assert V.gefahren(x, y, th, t, 0, sek=3.0) == 0.2 * t[t <= 3.0][-1]
    assert V.gefahren(-x, y, th, t, 0) == 0.0                 # rückwärts
    th2 = th.copy()
    th2[11:] = math.radians(45.0)                             # ab 1,1 s abgebogen
    assert abs(V.gefahren(x, y, th2, t, 0) - 0.2) < 1e-9
    # Kurs 90°: Fahrt in y-Richtung zählt
    assert abs(V.gefahren(np.zeros(40), t * 0.2, np.full(40, math.pi / 2), t, 0, sek=1.0) - 0.2) < 1e-9


def test_zellen_auf_bahn():
    occ = np.array([[0.0, 0.05], [0.0, 0.5], [2.0, 0.0]])
    assert V.zellen_auf_bahn(occ, np.array([0.0, 0.1]), np.array([0.0, 0.0])) == 1
    assert V.zellen_auf_bahn(np.zeros((0, 2)), np.array([0.0]), np.array([0.0])) == 0


def test_hand_center_ignoriert_unsichere_spalten():
    rows = np.full(80, 470.0, np.float32)                     # fast direkt vor uns
    rows[:32] = 260.0                                         # links frei
    ok = np.ones(80, bool)
    nah = V.hand_center(rows, ok, 60.0, 120, 160, 4.0, 5)
    assert nah < 0.1
    # Zentrum unsicher: Es bleibt nichts, das gilt wie überall nicht als frei
    ok[32:48] = False
    assert V.hand_center(rows, ok, 60.0, 120, 160, 4.0, 5) == 0.0


def test_punkte_auf_spur_rechnet_im_welt_ks():
    # Rider bei (1, 0) mit Kurs 90°: Ein Punkt 0,5 m voraus liegt in der Welt bei (1, 0,5)
    pts = np.array([[0.5, 0.0], [0.5, 0.3]])
    wx, wy = np.full(10, 1.0), np.linspace(0.0, 0.9, 10)
    m = V.punkte_auf_spur(pts, (1.0, 0.0, math.pi / 2), wx, wy)
    assert m.tolist() == [True, False]
    assert V.punkte_auf_spur(np.zeros((0, 2)), (0.0, 0.0, 0.0), wx, wy).shape == (0,)
