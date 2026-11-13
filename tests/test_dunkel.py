"""Dunkelangst (xgonav/dunkel.py): Angst, umdrehen, Licht suchen, aufgeben."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from xgonav.config import Config
from xgonav.dunkel import Dunkelangst, helligkeit


def _lauf(d, luma_fn, t_end, dt=0.05, yaw=0.0):
    """Regelschritte mit idealer Drehung (w x vyaw); luma_fn(yaw) -> Helligkeit. Gibt (t, yaw, cmds) zurück."""
    t, cmds = 0.0, []
    while t < t_end and d.state != "aufgeben":
        c = d.update(t, luma_fn(yaw), yaw)
        cmds.append(c)
        if c is not None:
            yaw = (yaw + math.radians(c.w * d.vyaw) * dt + math.pi) % (2 * math.pi) - math.pi
        t += dt
    return t, yaw, cmds


def test_hell_nichts_tun():
    d = Dunkelangst(Config.neutral().dunkel, 360.0)
    _, _, cmds = _lauf(d, lambda y: 120.0, 3.0)
    assert all(c is None for c in cmds) and d.n_angst == 0


def test_kurz_dunkel_keine_angst():
    cfg = Config.neutral().dunkel
    d = Dunkelangst(cfg, 360.0)
    for i in range(int(cfg.dark_s / 0.05) - 2):          # knapp unter dark_s
        assert d.update(i * 0.05, 3.0, 0.0) is None
    assert d.update(5.0, 120.0, 0.0) is None and d.n_angst == 0


def test_angst_umdrehen_licht_hinten():
    """Vorn dunkel, hinten hell: Angst, 180° drehen, Licht -> weiter (None)."""
    cfg = Config.neutral().dunkel
    d = Dunkelangst(cfg, 360.0)
    licht = lambda yaw: 3.0 if abs(yaw) < math.radians(90) else 120.0
    t, yaw, cmds = _lauf(d, licht, 3.0)
    assert d.state == "angst" and d.ausgaben == [("emotion", "angst"), ("sound", "angst")]
    d.drain()
    t2, yaw, cmds = _lauf(d, licht, 20.0, yaw=yaw)
    assert d.n_licht == 1 and d.state == "ok"
    assert abs(abs(yaw) - math.pi) < math.radians(cfg.tol_deg + 2)
    assert any(c is not None and abs(c.w) > 0 for c in cmds)
    assert all(c is None or c.w == 0 or abs(c.w) >= cfg.turn_min_w for c in cmds)


def test_kein_licht_drei_richtungen_dann_aufgeben():
    cfg = Config.neutral().dunkel
    d = Dunkelangst(cfg, 360.0)
    gesehen = []

    def dunkel(yaw):
        gesehen.append(round(math.degrees(yaw)) % 360)
        return 3.0
    _lauf(d, dunkel, 60.0)
    assert d.state == "aufgeben" and d.n_aufgeben == 1 and ("ruhe", "") in d.ausgaben
    # nach hinten, rechts und links geschaut (je Richtung ±tol)
    for ziel in (180, 270, 90):
        assert any(min(abs(g - ziel), 360 - abs(g - ziel)) <= cfg.tol_deg + 2 for g in gesehen), ziel


def test_wiederholt_ins_dunkle_gibt_auf():
    cfg = Config.neutral().dunkel
    d = Dunkelangst(cfg, 360.0)
    t = 0.0
    for n in range(cfg.max_events):
        while d.state == "ok":                            # fährt (Modus) wieder ins Dunkle
            d.update(t, 3.0, 0.0)
            t += 0.05
        if n < cfg.max_events - 1:
            d.reset()                                     # wie nach Licht gefunden
    while d.state == "angst":
        d.update(t, 3.0, 0.0)
        t += 0.05
    assert d.state == "aufgeben" and d.n_angst == cfg.max_events


def test_helligkeit():
    assert helligkeit(np.full((480, 640, 3), 77, np.uint8)) == 77.0
