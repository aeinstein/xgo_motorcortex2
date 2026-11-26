"""Szene "unten" (Erdgeschoss aus den Fahrten ab 2026-11-23, Weltdatei xgonav/welten/unten.npz): Kollision mit Wänden
und Möbeln, unterfahrbarer Tisch, schräge Wand, Bild mit Fototexturen und Bodengrenze, beide Lichter."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xgonav.config import Config  # noqa: E402
from xgonav.robot import SimRobot  # noqa: E402
from xgonav.sim import SCENES, SimWorld, scene_floor  # noqa: E402


def welt(x, y, th, name="unten"):
    cfg = Config.neutral()
    sc = SCENES[name]()
    w = SimWorld(cfg.camera, SimRobot(cfg.robot), None, start=(x, y, th), scene=sc,
                 mount_offset_mm=cfg.camera.mount_offset_mm)
    w.cam_h, w.pitch, w._pitch_prev, w.yaw_jit, w._t0, w._idx = 0.128, 0.0, 0.0, 0.0, 0, 0
    return w


def test_kollision_und_unterfahren():
    sc = SCENES["unten"]()
    assert sc.erweitert and sc.size == 10.0 and sc.sy == 9.0
    assert not sc.collides(*sc.start[:2])
    for x, y in ((7.0, 2.5), (0.3, 5.0), (2.5, 8.2), (2.5, 6.7), (6.5, 5.6), (4.4, 1.5)):
        assert sc.collides(x, y), (x, y)               # Sofa, Wand, Küchenzeilen, Schrankwand, TV-Schrank
    for x, y in ((2.5, 7.45), (4.1, 6.5), (1.0, 4.8), (5.5, 4.5), (4.9, 1.5), (8.4, 3.6)):
        assert not sc.collides(x, y), (x, y)           # Küchengang, Durchgang, Essnische, Hauptraum, vor dem TV, Fensterecke
    assert sc.collides(1.58, 5.2, 0.05)                # Tischbein
    assert not sc.collides(2.3, 5.5)                   # unter dem Tisch zwischen den Beinen (Platte schwebt)
    assert sc.collides(4.1, 3.72) and not sc.collides(4.4, 4.0)       # schräge Wand, davor frei
    assert not scene_floor(sc, np.array([8.0]), np.array([0.8]))[0]   # Treppe
    assert scene_floor(sc, np.array([5.6]), np.array([1.6]))[0]       # Teppich ist Boden


def test_bild_und_bodengrenze():
    w = welt(7.3, 4.3, math.pi)
    img, box, rows = w._render(0.0, 0.05)
    assert img.shape == (480, 640, 3) and box is None
    assert np.isfinite(rows).all() and (rows > 240).all() and (rows <= 480).all()
    assert img[:230].std() > 15                        # Fototexturen: kein einfarbiges Bild über dem Boden
    unten = img[400:].astype(int).sum(-1)
    assert (unten < 0.8 * np.median(unten)).mean() > 0.002           # dunkle Fugen im Fliesenboden


def test_wand_direkt_vor_dem_rider():
    w = welt(2.5, 7.45, math.pi / 2)                   # Küchengang, Blick nach Norden: Zeile 0,45 m voraus
    _, _, rows = w._render(0.0, 0.05)
    mitte = rows[len(rows) // 2 - 3: len(rows) // 2 + 3]
    assert mitte.max() > 300                            # Bodengrenze nah (tief im Bild)


def test_schraege_wand_und_licht():
    sc = SCENES["unten"]()
    assert len(sc.flaechen) == 1 and any(not t.sichtbar for t in sc.teile)
    w = welt(4.6, 4.2, math.radians(230))              # Blick auf die schräge Wand, 0,5 m voraus
    _, _, rows = w._render(0.0, 0.05)
    assert rows[len(rows) // 2] > 290
    a = welt(7.3, 4.3, math.pi)._render(0.0, 0.05)[0]
    b = welt(7.3, 4.3, math.pi, "unten_tag")._render(0.0, 0.05)[0]
    assert b.mean() > a.mean() + 10                     # Tageslicht ist heller als Lampenlicht


def test_alte_szenen_unveraendert():
    for name in ("raum", "zwei", "spalt", "fuge"):
        sc = SCENES[name]()
        assert not sc.erweitert and sc.sy == sc.size and sc.segments().shape[1] == 5
