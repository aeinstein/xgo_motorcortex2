"""Fahrrahmen (Firmware-Patch 08-velocity, xgonav/fahrrahmen.py, Robot): Text für /proc, Generator wie die Firmware,
Robot schreibt Rahmen statt Bytes, Grenzen beim Start und periodisch, Einschalten nur mit Treiber und Firmware."""
import math
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from xgonav.config import Config  # noqa: E402
from xgonav.fahrrahmen import CAP_KAPPA, Achse, Bogen, bogen_schritt, bogen_text, grenzen_text, rahmen_text, zustand  # noqa: E402
from xgonav.robot import ProcRobot, SimRobot  # noqa: E402


def test_rahmen_text_einheiten():
    # 0,36 m/s bei 8,4 mm/s je Einheit = 42,9 -> 43; a 0,5 m/s² -> 60; Ruck 2,5 -> 298
    assert rahmen_text(0.36, 0.5, 2.5, 0.0, 360, 1800, 8.4) == "43 60 298 0.0 360 1800"
    assert rahmen_text(-0.1, -1.0, 2.5, -12.34, 120, 600, 8.4) == "-12 119 298 -12.3 120 600"   # a/Ruck als Betrag
    assert rahmen_text(0.0, 0.5, 2.5, -0.05, 0, 0, 8.4).split()[3] == "-0.1"                    # Rundung auf 0,1 °/s
    assert rahmen_text(99.0, 999.0, 0, 9999.0, 0, 0, 8.4) == "11786 32767 0 3276.7 0 0"           # Grenzen des Rahmens
    assert grenzen_text(0.9, 180.0, 500, 3.9, 0.03, 6.0, 0.025) == "231 180.0 500 3900 300 600 250"
    assert grenzen_text(0.5 * 1.8, 0.5 * 360, 0, 1.0, 0.0, 0.0, 0.0) == "900 180.0 0 1000 1 1 0"   # ki/i_max ≥ 1


def test_zustand_lesen():
    z = zustand("1 0 0 1 250 -12.3 500 90.0 300")
    assert z == {"aktiv": True, "beschl": False, "waechter": False, "grenzen": True, "v": 250, "w": -12.3,
                 "v_max": 500, "w_max": 90.0, "timeout_ms": 300, "kappa": False, "caps": 0}   # alter Treiber
    z = zustand("1 0 0 1 46 4.6 600 300.0 300 1 1")
    assert z["kappa"] and z["caps"] == CAP_KAPPA and z["w"] == 4.6
    assert zustand("Fehler -110") is None and zustand("") is None and zustand("1 2 3") is None
    assert zustand("1 0 0 1 250 -12.3 500 90.0 300 1") is None


def test_bogen_text():
    # κ in 0,001 1/m, Tempo wie rahmen_text (0,18 m/s / 3,9 = 46)
    assert bogen_text(0.18, 0.5, 2.5, 2.0, 360, 1800, 3.9) == "kap 46 128 641 2000 360 1800"
    assert bogen_text(-0.18, 0.0, 0.0, -0.5004, 0, 0, 3.9) == "kap -46 0 0 -500 0 0"
    assert bogen_text(0.1, 0, 0, 99.0, 0, 0, 3.9).split()[4] == "32767"


VECTORS = os.path.join(os.path.dirname(__file__), "..", "firmware", "tests", "vectors")


def _i16(b, i):
    return int.from_bytes(b[i:i + 2], "little", signed=True)


def _u16(b, i):
    return int.from_bytes(b[i:i + 2], "little")


def _firmware_nachbau(rows):
    """velocity_cmd.h receive + tick (Drehraten- und Krümmungsweg) mit Achse und bogen_schritt nachgerechnet; je Takt
    (v, w, a, kappa, timed_out)."""
    fwd, turn, bog = Achse(), Achse(), Bogen()
    aktiv = ticked = timed_out = beschl = False
    last_rx = last_tick = 0
    f = None
    out = []
    for r in rows:
        t = int(r["t_us"])
        v_max, w_max, timeout, m = float(r["v_max"]), float(r["w_max"]), int(r["timeout_ms"]), float(r["unit_mm"]) * 1e-3
        if r["rx"] == "1":
            d = bytes.fromhex(r["frame_hex"])
            kap = bool(d[0] & 2)
            f = {"beschl": bool(d[0] & 1), "kap": kap, "v": _i16(d, 1), "a": _i16(d, 3), "j": _u16(d, 5),
                 "w": 0.0 if kap else _i16(d, 7) * 0.1, "k": _i16(d, 7) * 0.001 if kap else 0.0,
                 "al": _u16(d, 9), "jw": _u16(d, 11)}
            if not aktiv:
                ticked = False
            aktiv, timed_out, last_rx, beschl = True, False, t, f["beschl"]
        dt = 0.0
        if not ticked:
            fwd, turn = Achse(), Achse()
        else:
            dt = min(0.05, (t - last_tick) * 1e-6)
        last_tick, ticked = t, True
        wd = timeout and t - last_rx > timeout * 1000
        if wd and not timed_out:
            timed_out, beschl = True, False
        if f["kap"]:
            if wd:
                a = abs(f["a"]) if f["a"] else 500.0
                turn.x = bogen_schritt(fwd, bog, turn.x, 0.0, f["k"], a, f["j"], f["al"], f["jw"], v_max, w_max, dt, m)
            else:
                turn.x = bogen_schritt(fwd, bog, turn.x, f["v"], f["k"], f["a"], f["j"], f["al"], f["jw"], v_max,
                                       w_max, dt, m, beschl=beschl)
        else:
            bog.an = False
            if wd:
                fwd.schritt(0.0, abs(f["a"]) if f["a"] else 500.0, f["j"], dt)
                turn.schritt(0.0, f["al"] if f["al"] else 180.0, f["jw"], dt)
            else:
                if beschl:
                    fwd.beschleunigen(f["a"], f["j"], v_max, dt)
                else:
                    fwd.schritt(max(-v_max, min(v_max, f["v"])), abs(f["a"]), f["j"], dt)
                turn.schritt(max(-w_max, min(w_max, f["w"])), f["al"], f["jw"], dt)
            fwd.x, turn.x = max(-v_max, min(v_max, fwd.x)), max(-w_max, min(w_max, turn.x))
        out.append((fwd.x, turn.x, fwd.a, bog.kappa, timed_out))
    return out


@pytest.mark.parametrize("name", sorted(n[:-4] for n in os.listdir(VECTORS) if n.endswith(".csv"))
                         if os.path.isdir(VECTORS) else [])
def test_firmware_vektoren(name):
    """Prüfvektoren der Firmware-Host-Tests (XGO_VECTORS=1, ESP32 float): Nachbau am Pi gleich bis 1e-4 (relativ ab 1)."""
    import csv
    with open(os.path.join(VECTORS, name + ".csv"), newline="") as fh:
        rows = list(csv.DictReader(fh))
    for i, (r, (v, w, a, k, to)) in enumerate(zip(rows, _firmware_nachbau(rows))):
        for key, x in (("v", v), ("w", w), ("a", a), ("kappa", k)):
            soll = float(r[key])
            assert abs(x - soll) <= 1e-4 * max(1.0, abs(soll)), "%s Zeile %d t %s: %s %.7g statt %.7g" % (
                name, i + 2, r["t_us"], key, x, soll)
        assert to == (r["timed_out"] == "1"), "%s Zeile %d: timed_out" % (name, i + 2)


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
    assert c.vel_frame                                                     # Default: wie am Rider mit 08-velocity
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
        assert _writes(r, "vel_limits") == ["231 180.0 500 3900 300 600 250"]
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
        assert r._target == pytest.approx((0.0, 0.01))                     # keine Untergrenze
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
        for _ in range(40):                                                # wieder 0,36 m/s
            r.drive(0.2, 0.0)
            time.sleep(0.02)
        t1 = time.monotonic()
        r.drive(0.05, 0.0, brake=True)                                     # Bahnprüfung: auf ein kleineres Tempo
        time.sleep(0.06)
        assert "23 385 2564 0.0 360 1800" in _writes(r, "vel", t1)        # 0,09 m/s mit vel_brake
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
    assert not r.vel                                                       # Firmware ohne 08-velocity
    (tmp_path / "vel").write_text("0 0 0 0 0 0.0 0 0.0 0\n")
    (tmp_path / "vel_limits").write_text("")
    r._vel_setup()
    assert r.vel and (tmp_path / "vel_limits").read_text() == "231 180.0 500 3900 300 600 250"
    cfg.vel_frame = False
    r._vel_setup()
    assert not r.vel
    cfg.vel_frame = True
    r._vel_setup()
    assert r.vel and r.caps == 0 and not r.bogen                           # alter Treiber: kein Fähigkeitsbyte
    cfg.vel_kappa = True
    r._vel_setup()
    assert not r.bogen                                                     # Firmware ohne Krümmungsbetrieb
    (tmp_path / "vel").write_text("0 0 0 1 0 0.0 0 0.0 0 0 1\n")
    r._vel_setup()
    assert r.bogen and r.caps == CAP_KAPPA
    cfg.vel_kappa = False
    r._vel_setup()
    assert not r.bogen                                                     # vel_kappa aus: Drehratenweg


def test_bogen_robot_schreibt_kruemmung_und_haelt_den_bogen():
    """Krümmungsbetrieb: Rahmen "kap …", w = κ · v_soll schon beim Anfahren (Drehratenweg: w früher am Ziel)."""
    cfg = _cfg(vel_kappa=True)
    r = SimRobot(cfg)
    r.start()
    try:
        assert r.bogen
        k = 2.0                                                            # 1/m, 0,5 m Radius links
        vk = []
        for _ in range(40):
            r.drive(0.2, 0.0, kappa=k)                                     # w egal, κ zählt
            time.sleep(0.02)
            v, w = r.command
            if v > 1e-3:
                vk.append(math.radians(w * cfg.vyaw_limit_dps) / (v * cfg.vx_limit))
        assert "kap 92 128 641 2000 360 1800" in _writes(r, "vel")
        assert vk and all(abs(x - k) < 1e-6 for x in vk)                   # Krümmung in jedem Takt
        r.drive(0.2, 0.3)                                                  # ohne κ: wieder Drehratenweg
        time.sleep(0.1)
        assert not _writes(r, "vel")[-1].startswith("kap ")
    finally:
        r.close()


def test_bogen_ohne_vel_kappa_wie_bisher():
    cfg = _cfg()
    assert not cfg.vel_kappa
    r = SimRobot(cfg)
    r.start()
    try:
        assert not r.bogen
        for _ in range(10):
            r.drive(0.2, 0.1, kappa=2.0)
            time.sleep(0.02)
        assert not any(t.startswith("kap ") for t in _writes(r, "vel"))
    finally:
        r.close()
