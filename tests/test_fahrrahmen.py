"""Fahrrahmen (Firmware 0029–0032, xgonav/fahrrahmen.py, Robot): Text für /proc, Generator wie die Firmware,
Robot schreibt Rahmen statt Bytes, Grenzen beim Start und periodisch, Einschalten nur mit Treiber und Firmware."""
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from xgonav.config import Config  # noqa: E402
from xgonav.fahrrahmen import Achse, grenzen_text, rahmen_text, zustand  # noqa: E402
from xgonav.robot import ProcRobot, SimRobot  # noqa: E402


def test_rahmen_text_einheiten():
    # 0,36 m/s bei 8,4 mm/s je Einheit = 42,9 -> 43; a 0,5 m/s² -> 60; Ruck 2,5 -> 298
    assert rahmen_text(0.36, 0.5, 2.5, 0.0, 360, 1800, 8.4) == "43 60 298 0.0 360 1800"
    assert rahmen_text(-0.1, -1.0, 2.5, -12.34, 120, 600, 8.4) == "-12 119 298 -12.3 120 600"   # a/Ruck als Betrag
    assert rahmen_text(0.0, 0.5, 2.5, -0.05, 0, 0, 8.4).split()[3] == "-0.1"                    # Rundung auf 0,1 °/s
    assert rahmen_text(99.0, 999.0, 0, 9999.0, 0, 0, 8.4) == "11786 32767 0 3276.7 0 0"           # Grenzen des Rahmens
    assert grenzen_text(0.9, 180.0, 500, 8.4) == "107 180.0 500"
    assert grenzen_text(0.5 * 1.8, 0.5 * 360, 0, 1.0) == "900 180.0 0"
    assert grenzen_text(0.9, 180.0, 500, 3.9, 0.55) == "231 180.0 500 3900 550"   # Firmware 0035
    assert grenzen_text(0.9, 180.0, 500, 3.9, 0.01).endswith(" 3900 100")             # Vorfaktor ≥ 0,1


def test_zustand_lesen():
    z = zustand("1 0 0 1 250 -12.3 500 90.0 300")
    assert z == {"aktiv": True, "beschl": False, "waechter": False, "grenzen": True, "v": 250, "w": -12.3,
                 "v_max": 500, "w_max": 90.0, "timeout_ms": 300}
    assert zustand("Fehler -110") is None and zustand("") is None and zustand("1 2 3") is None


def _lauf(ax, ziel, a, j, s, dt=0.004):
    t_end, peak, xmax = None, 0.0, -1e9
    for i in range(int(s / dt)):
        ax.schritt(ziel, a, j, dt)
        peak, xmax = max(peak, abs(ax.a)), max(xmax, ax.x)
        if t_end is None and ax.x == ziel and ax.a == 0:
            t_end = (i + 1) * dt
    return t_end, peak, xmax


def test_generator_wie_firmware():
    """Dieselben Eckwerte wie test_velocity_cmd.cpp: v/a + a/j, kein Überschwingen, Grenzen."""
    t, peak, xmax = _lauf(Achse(), 0.4, 0.8, 4.0, 2.0)
    assert 0.66 < t < 0.76 and peak <= 0.8 + 1e-9 and xmax <= 0.4
    t, peak, _ = _lauf(Achse(), 0.02, 0.8, 4.0, 1.0)                      # Dreieck: 2·sqrt(dv/j) = 0,14 s
    assert 0.13 < t < 0.17 and peak < 0.3
    ax = Achse(x=0.3)
    t, _, _ = _lauf(ax, -0.3, 0.6, 0.0, 2.0)                               # ohne Ruckgrenze: Rampe 1 s
    assert 0.99 < t < 1.02
    ax = Achse()
    ax.schritt(0.5, 0.0, 0.0, 0.0)
    assert ax.x == 0.5                                                     # a = 0: Sprung


def _cfg(**kw):
    c = Config.neutral().robot
    assert c.vel_frame                                                     # Default: wie am Rider mit 0029
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def _writes(r, name, t0=0.0):
    return [v for t, n, v in r.writes if n == name and t >= t0]


def test_fahrrahmen_robot_schreibt_rahmen():
    cfg = _cfg()
    r = SimRobot(cfg)
    r.start()
    try:
        assert _writes(r, "vel_limits") == ["231 180.0 500 3900 550"]
        for _ in range(60):                                                # 1,2 s fahren (0,2 x 1,8 = 0,36 m/s)
            r.drive(0.2, 0.0)
            time.sleep(0.02)
        assert _writes(r, "speed_x") == [128] and _writes(r, "speed_z") == [128] or not _writes(r, "speed_x")
        assert "92 128 641 0.0 360 1800" in _writes(r, "vel")
        assert r.command[0] == pytest.approx(0.2, abs=1e-9)               # Generator nach 0,92 s am Ziel
        t0 = time.monotonic()
        time.sleep(cfg.stall_timeout_s + 0.25)                             # Userspace-Watchdog -> Ziel 0
        assert "0 256 641 0.0 360 1800" in _writes(r, "vel", t0)          # langsamer: vel_decel 1,0 m/s²
        assert 0.0 <= r.command[0] < 0.2                                   # bremst über die Rampe
    finally:
        r.close()
    assert r.writes[-1][2] == 128                                          # close: Bytes 128 beenden den Rahmenweg


def test_fahrrahmen_rampe_am_pi_folgt_generator():
    r = SimRobot(_cfg())
    r.start()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.3:
            r.drive(0.5, 0.0)
            time.sleep(0.02)
        v = r.command[0]
        # 0,3 s mit Ruck 2,5: a steigt auf 0,5 in 0,2 s, v ~0,075 m/s = 0,04 normiert (Pi-Rampe gäbe 0,24)
        assert 0.01 < v < 0.08
    finally:
        r.close()


def test_fahrrahmen_ohne_untergrenze_und_bremse():
    cfg = _cfg()
    r = SimRobot(cfg)
    r.start()
    try:
        r.drive(0.0, 0.01)
        assert r._target == pytest.approx((0.0, 0.01))                     # kein turn_min_w ohne Totband
        r.tempo = 0.5
        r.drive(0.0, 0.08)
        assert r._target == pytest.approx((0.0, 0.04))
        r.tempo = 1.0
        for _ in range(25):
            r.drive(0.2, 0.0)
            time.sleep(0.02)
        t0 = time.monotonic()
        r.drive(0.0, 0.0, brake=True)
        time.sleep(0.06)
        assert "0 385 2564 0.0 360 1800" in _writes(r, "vel", t0)         # vel_brake 1,5 m/s², Ruck 10
    finally:
        r.close()


def test_fahrrahmen_grenzen_periodisch_und_aktion():
    cfg = _cfg(vel_limits_s=0.3, action_tick_s=0.01)
    r = SimRobot(cfg)
    r.start()
    try:
        time.sleep(1.4)
        assert len(_writes(r, "vel_limits")) >= 3
        r.action_start(3)                                                  # 0,2 s + 0,3 s Reserve
        t0 = time.monotonic()
        time.sleep(0.35)
        assert _writes(r, "vel", t0) == []                                 # Aktion läuft: kein Keepalive
        r.action_abort()
        t1 = time.monotonic()
        time.sleep(0.1)
        assert _writes(r, "speed_x", t1) and all(v == 128 for v in _writes(r, "speed_x", t1))   # Abbruch über Bytes
    finally:
        r.close()


def test_fahrrahmen_einschalten_nur_mit_treiber_und_firmware(tmp_path):
    cfg = _cfg(proc_dir=str(tmp_path))
    r = ProcRobot(cfg)
    r._vel_setup()
    assert not r.vel                                                       # Treiber ohne vel
    (tmp_path / "vel").write_text("Fehler -110\n")
    r._vel_setup()
    assert not r.vel                                                       # Firmware vor 0029
    (tmp_path / "vel").write_text("0 0 0 0 0 0.0 0 0.0 0\n")
    (tmp_path / "vel_limits").write_text("")
    r._vel_setup()
    assert r.vel and (tmp_path / "vel_limits").read_text() == "231 180.0 500 3900 550"
    cfg.vel_frame = False
    r._vel_setup()
    assert not r.vel
