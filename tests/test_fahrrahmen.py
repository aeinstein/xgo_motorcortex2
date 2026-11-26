"""Fahrrahmen (Firmware-Patch 08-velocity, xgonav/fahrrahmen.py, Robot): Text für /proc, Generator wie die Firmware,
Robot schreibt Rahmen statt Bytes, Grenzen beim Start und periodisch, Einschalten nur mit Treiber und Firmware."""
import math
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from xgonav.config import Config  # noqa: E402
from xgonav.fahrrahmen import (CAP_KAPPA, CAP_KURS, CAP_STRECKE, Achse, grenzen_text, kursabs_schritt, kursabs_text, Bogen, Kurs, Strecke, bogen_schritt, bogen_text,  # noqa: E402
                                grenzen_text, kurs_empfang, kurs_schritt, kurs_text, rahmen_text, strecke_empfang,
                                strecke_messen, strecke_schritt, strecke_text, zustand)
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
                 "v_max": 500, "w_max": 90.0, "timeout_ms": 300, "kappa": False, "caps": 0,
                 "kurs": False, "erreicht": False, "strecke": False, "abs_grenzen": False}   # alter Treiber
    z = zustand("1 0 0 1 46 4.6 600 300.0 300 1 1")
    assert z["kappa"] and z["caps"] == CAP_KAPPA and z["w"] == 4.6
    z = zustand("1 0 0 1 46 4.6 600 300.0 300 1 3")
    assert z["caps"] == CAP_KAPPA                    # Treiber mit 11 Feldern kennt keinen Kursrahmen: Bit nicht nutzen
    z = zustand("1 0 0 1 0 -12.5 600 100.0 300 0 3 1 1")
    assert z["caps"] == CAP_KAPPA | CAP_KURS and z["kurs"] and z["erreicht"] and not z["kappa"]
    assert zustand("Fehler -110") is None and zustand("") is None and zustand("1 2 3") is None
    assert zustand("1 0 0 1 250 -12.3 500 90.0 300 1") is None
    z = zustand("1 0 0 1 0 0.0 600 100.0 300 0 23 0 0 0")
    assert z["caps"] == CAP_KAPPA | CAP_KURS | CAP_STRECKE     # Treiber mit 14 Feldern kennt kein absolutes Kursziel
    z = zustand("1 0 0 1 0 0.0 600 100.0 300 0 23 1 0 0 1")
    assert z["caps"] == 23 and z["kurs"] and z["abs_grenzen"]


def test_kursabs_text_und_grenzen():
    assert kursabs_text(90.0, 45.0) == "kursabs 9000 45.0"
    assert kursabs_text(-190.004, 0.0) == "kursabs 17000 0.0"     # auf ±180 gewickelt
    assert kursabs_text(180.0, 100.04) == "kursabs -18000 100.0"
    assert kursabs_text(-12.345, 30.0) == "kursabs -1235 30.0"
    assert grenzen_text(0.9, 100.0, 300, 3.97, 0.06, 6.0, 0.025) == "227 100.0 300 3970 600 600 250"
    assert grenzen_text(0.9, 100.0, 300, 3.97, 0.06, 6.0, 0.025, 360, 1800) == "227 100.0 300 3970 600 600 250 360 1800"


def test_kurs_text():
    assert kurs_text(True, 0.0, 0.5, 2.5, 90.0, 360, 1800, 3.9) == "kurs 1 0 128 641 90.0 360 1800"
    assert kurs_text(False, 0.18, 0.5, 2.5, -12.34, 360, 1800, 3.9) == "kurs 0 46 128 641 -12.3 360 1800"


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
    fwd, turn, bog, kz, st = Achse(), Achse(), Bogen(), Kurs(), Strecke()
    aktiv = ticked = timed_out = beschl = kabs = False
    last_rx = last_tick = 0
    f = None
    out = []
    for r in rows:
        t = int(r["t_us"])
        v_max, w_max, timeout, m = float(r["v_max"]), float(r["w_max"]), int(r["timeout_ms"]), float(r["unit_mm"]) * 1e-3
        if r["rx"] == "1" and len(bytes.fromhex(r["frame_hex"])) == 5:
            # absolutes Kursziel: nur Ziel und w_max; Fahrachse bremst mit a/Ruck des letzten Rahmens mit Fahrachse
            d = bytes.fromhex(r["frame_hex"])
            alt = f if aktiv else None
            f = {"beschl": False, "kap": False, "kurs": False, "abs": True, "v": 0, "w": 0.0, "k": 0.0,
                 "a": abs(alt["a"]) if alt else 0, "j": alt["j"] if alt else 0, "al": 0, "jw": 0,
                 "psi": _i16(d, 1) * 0.01, "w_abs": _u16(d, 3) * 0.1}
            if not aktiv:
                ticked = False
            aktiv, timed_out, last_rx, beschl = True, False, t, False
            if not (kz.an and kabs):
                kz = Kurs(an=True)
            kabs = True
            strecke_empfang(st, None)
        elif r["rx"] == "1":
            d = bytes.fromhex(r["frame_hex"])
            kap, kur = bool(d[0] & 2), bool(d[0] & 4)
            f = {"beschl": bool(d[0] & 1), "kap": kap, "kurs": kur, "v": _i16(d, 1), "a": _i16(d, 3), "j": _u16(d, 5),
                 "w": 0.0 if kap or kur else _i16(d, 7) * 0.1, "k": _i16(d, 7) * 0.001 if kap else 0.0,
                 "al": _u16(d, 9), "jw": _u16(d, 11), "abs": False}
            if not aktiv:
                ticked = False
            aktiv, timed_out, last_rx, beschl = True, False, t, f["beschl"]
            if kabs and kur:
                kz.an = False                               # relativ nach absolut: immer neues Ziel
            kabs = False
            kurs_empfang(kz, _i16(d, 7) * 0.1 if kur else None, bool(d[0] & 16))
            strecke_empfang(st, float(_u16(d, 13)) if d[0] & 8 else None)
        dt = 0.0
        if not ticked:
            fwd, turn = Achse(), Achse()
        else:
            dt = min(0.05, (t - last_tick) * 1e-6)
        last_tick, ticked = t, True
        if st.an and dt > 0:
            strecke_messen(st, float(r.get("v_meas") or 0.0), 1.0 if f["v"] >= 0 else -1.0, float(r["unit_mm"]), dt)
        wd = timeout and t - last_rx > timeout * 1000
        if f["abs"]:
            ref = float(r["psi_ref"])
            fwd.schritt(0.0, abs(f["a"]) if f["a"] else 500.0, f["j"], dt)
            kursabs_schritt(turn, kz, f["psi"], None if math.isnan(ref) else ref, float(r["abs_alpha"]),
                            float(r["abs_jerk"]), f["w_abs"], w_max, dt, waechter=bool(wd), timed_out=timed_out)
            if wd and not timed_out:
                timed_out = True
            fwd.x = max(-v_max, min(v_max, fwd.x))
            out.append((fwd.x, turn.x, fwd.a, 0.0, timed_out, kz.rest, kz.done, st.rest, st.done, st.lag))
            continue
        if wd and not timed_out:
            timed_out, beschl = True, False
        if f["kap"]:
            if wd:
                a = abs(f["a"]) if f["a"] else 500.0
                turn.x = bogen_schritt(fwd, bog, turn.x, 0.0, f["k"], a, f["j"], f["al"], f["jw"], v_max, w_max, dt, m)
            else:
                turn.x = bogen_schritt(fwd, bog, turn.x, f["v"], f["k"], f["a"], f["j"], f["al"], f["jw"], v_max,
                                       w_max, dt, m, beschl=beschl, st=st)
        elif f["kurs"]:
            bog.an = False
            if wd:
                fwd.schritt(0.0, abs(f["a"]) if f["a"] else 500.0, f["j"], dt)
            elif beschl:
                fwd.beschleunigen(f["a"], f["j"], v_max, dt)
            else:
                fwd.schritt(max(-v_max, min(v_max, f["v"])), abs(f["a"]), f["j"], dt)
            fwd.x = max(-v_max, min(v_max, fwd.x))
            kurs_schritt(turn, kz, f["al"], f["jw"], w_max, dt, waechter=bool(wd))
        else:
            bog.an = False
            if wd:
                fwd.schritt(0.0, abs(f["a"]) if f["a"] else 500.0, f["j"], dt)
                turn.schritt(0.0, f["al"] if f["al"] else 180.0, f["jw"], dt)
            else:
                if beschl:
                    fwd.beschleunigen(f["a"], f["j"], v_max, dt)
                elif st.an:
                    strecke_schritt(fwd, st, max(-v_max, min(v_max, f["v"])), abs(f["a"]), f["j"], dt,
                                    float(r["unit_mm"]))
                else:
                    fwd.schritt(max(-v_max, min(v_max, f["v"])), abs(f["a"]), f["j"], dt)
                turn.schritt(max(-w_max, min(w_max, f["w"])), f["al"], f["jw"], dt)
            fwd.x, turn.x = max(-v_max, min(v_max, fwd.x)), max(-w_max, min(w_max, turn.x))
        out.append((fwd.x, turn.x, fwd.a, bog.kappa, timed_out, kz.rest, kz.done, st.rest, st.done, st.lag))
    return out


@pytest.mark.parametrize("name", sorted(n[:-4] for n in os.listdir(VECTORS) if n.endswith(".csv"))
                         if os.path.isdir(VECTORS) else [])
def test_firmware_vektoren(name):
    """Prüfvektoren der Firmware-Host-Tests (XGO_VECTORS=1, ESP32 float): Nachbau am Pi gleich bis 1e-4 (relativ ab 1)."""
    import csv
    with open(os.path.join(VECTORS, name + ".csv"), newline="") as fh:
        rows = list(csv.DictReader(fh))
    flags = {bytes.fromhex(r["frame_hex"])[0] for r in rows}
    if any(f & ~(1 | 2 | 4 | 8 | 16 | 32) for f in flags):
        pytest.skip("Betrieb mit Flag %s noch nicht nachgebaut" % sorted(flags))
    # Kursziel: Die Vorausschau entscheidet je Takt „weiter oder bremsen“ über ein Vorzeichen; float (ESP32) und double
    # kippen an der Grenze einen Takt verschieden. Gemessen 2026-11-25: höchstens 0,61 °/s, Rest 0,05°, „angekommen“
    # ±2 Takte, Endrest gleich auf 0,001°. Deshalb dort diese Grenzen statt 1e-4.
    kurs = any(f & (4 | 32) for f in flags)
    tol = {"w": 1.0, "rest": 0.1} if kurs else {}
    tol.update({"dist_rest": 0.01, "dist_lag": 0.01})   # mm: Summen über bis 1600 Takte in float (ESP32) gegen double
    falsch_done = 0
    out = _firmware_nachbau(rows)
    for i, (r, (v, w, a, k, to, rest, done, d_rest, d_done, d_lag)) in enumerate(zip(rows, out)):
        werte = [("v", v), ("w", w), ("a", a)] + ([("kappa", k)] if "kappa" in r else []) + (
            [("rest", rest)] if "rest" in r else [])
        if "dist_rest" in r:
            werte += [("dist_rest", d_rest), ("dist_lag", d_lag)]
            assert d_done == (r["dist_done"] == "1"), "%s Zeile %d t %s: dist_done" % (name, i + 2, r["t_us"])
        for key, x in werte:
            soll = float(r[key])
            grenze = tol.get(key, 1e-4 * max(1.0, abs(soll)))
            assert abs(x - soll) <= grenze, "%s Zeile %d t %s: %s %.7g statt %.7g" % (
                name, i + 2, r["t_us"], key, x, soll)
        assert to == (r["timed_out"] == "1"), "%s Zeile %d: timed_out" % (name, i + 2)
        if "done" in r:
            falsch_done += done != (r["done"] == "1")
    # absolutes Kursziel: am Ende bleibt in float ein Rest-w von ~4e-5 °/s einige Takte stehen (Rundung, Firmware-Sitzung
    # 2026-11-26), der Nachbau in double ist dann schon angekommen; stueckweise: 6 Takte
    grenze_done = (8 if any(f & 32 for f in flags) else 3) if kurs else 0
    assert falsch_done <= grenze_done, "%s: angekommen in %d Takten anders" % (name, falsch_done)
    if kurs:
        assert abs(out[-1][5] - float(rows[-1]["rest"])) < 0.01 and out[-1][6] == (rows[-1]["done"] == "1")


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
        assert _writes(r, "vel_limits") == ["227 180.0 500 3970 300 600 250"]
        for _ in range(60):                                                # 1,2 s fahren (0,2 x 1,8 = 0,36 m/s)
            r.drive(0.2, 0.0)
            time.sleep(0.02)
        assert _writes(r, "speed_x") == [128] and _writes(r, "speed_z") == [128] or not _writes(r, "speed_x")
        assert "91 126 630 0.0 360 1800" in _writes(r, "vel")
        assert r.command[0] == pytest.approx(0.2, abs=1e-9)               # Generator nach 0,92 s am Ziel
        t0 = time.monotonic()
        time.sleep(cfg.stall_timeout_s + 0.25)                             # Userspace-Watchdog -> Ziel 0
        assert "0 252 630 0.0 360 1800" in _writes(r, "vel", t0)          # langsamer: vel_decel 1,0 m/s²
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
        assert "0 378 2519 0.0 360 1800" in _writes(r, "vel", t0)         # vel_brake 1,5 m/s², Ruck 10
        for _ in range(40):                                                # wieder 0,36 m/s
            r.drive(0.2, 0.0)
            time.sleep(0.02)
        t1 = time.monotonic()
        r.drive(0.05, 0.0, brake=True)                                     # Bahnprüfung: auf ein kleineres Tempo
        time.sleep(0.06)
        assert "23 378 2519 0.0 360 1800" in _writes(r, "vel", t1)        # 0,09 m/s mit vel_brake
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
    assert r.vel and (tmp_path / "vel_limits").read_text() == "227 180.0 500 3970 300 600 250"
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
        assert "kap 91 126 630 2000 360 1800" in _writes(r, "vel")
        assert vk and all(abs(x - k) < 1e-6 for x in vk)                   # Krümmung in jedem Takt
        r.drive(0.2, 0.3)                                                  # ohne κ: wieder Drehratenweg
        time.sleep(0.1)
        assert not _writes(r, "vel")[-1].startswith("kap ")
    finally:
        r.close()


def test_bogen_ohne_vel_kappa_wie_bisher():
    cfg = _cfg(vel_kappa=False)
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


def test_kurs_robot_dreht_auf_ziel_ein_rahmen_je_ziel():
    """Kursziel: ein Ziel Δψ mit Wechselbit, gleiche Rahmen danach, w_max des Modus über vel_limits, angekommen ohne
    Überschwingen; ein verschobenes Ziel schaltet das Wechselbit um, danach w_max wieder voll."""
    cfg = _cfg(vel_kurs=True, vel_kappa=False)
    r = SimRobot(cfg)
    r.start()
    try:
        assert r.kurs_an and not r.bogen
        t0 = time.monotonic()
        ziel = 90.0
        for _ in range(150):                                               # 3 s
            rest = math.radians(ziel - r._gen_winkel)                     # Kurs = Generatorwinkel (ohne Nachlauf)
            r.drive(0.0, 0.5, kurs=(rest, cfg.w_of(0.25)))                 # Obergrenze 0,25 x max_w (0,5) = 45 °/s
            time.sleep(0.02)
        assert abs(r._gen_winkel - ziel) < 0.6 and r._kurs.done
        vel = [t for t in _writes(r, "vel", t0) if t.startswith("kurs ")]
        assert vel and len(set(vel)) == 1 and vel[0] == "kurs 1 0 252 630 90.0 360 1800"
        assert "227 45.0 500 3970 300 600 250" in _writes(r, "vel_limits", t0)
        assert max(abs(x) for x in [r.command[1]]) <= 45.0 / cfg.vyaw_limit_dps + 1e-9
        t1 = time.monotonic()
        for _ in range(10):
            r.drive(0.0, 0.5, kurs=(math.radians(ziel + 30.0 - r._gen_winkel), cfg.w_of(0.25)))   # neues Ziel
            time.sleep(0.02)
        assert any(t.startswith("kurs 0 ") for t in _writes(r, "vel", t1))
        t2 = time.monotonic()
        for _ in range(10):
            r.drive(0.0, 0.0)                                              # ohne Kursziel: Drehrate, w_max zurück
            time.sleep(0.02)
        assert "227 180.0 500 3970 300 600 250" in _writes(r, "vel_limits", t2)
        assert not _writes(r, "vel")[-1].startswith("kurs ")
    finally:
        r.close()


def test_kurs_senkt_w_max_nicht_waehrend_des_kursziels():
    """w_max über vel_limits: beim Start des Kursziels nicht unter die laufende Drehrate, währenddessen nie kleiner
    (die Firmware klemmt w sofort, ein Sprung ohne alpha); auch nicht, wenn der Temporegler mitten drin sinkt."""
    cfg = _cfg(vel_kurs=True)
    r = SimRobot(cfg)
    r.start()
    try:
        for _ in range(40):                                                # 0,8 s mit 0,5 x 360 = 180 °/s drehen
            r.drive(0.0, 0.5)
            time.sleep(0.02)
        w0 = abs(r._gen_w.x)
        assert w0 > 100
        t0 = time.monotonic()
        for i in range(60):
            if i == 20:
                r.tempo = 0.5
            r.drive(0.0, 0.5, kurs=(math.radians(720.0), cfg.w_of(0.25)))
            time.sleep(0.02)
        grenzen = [float(t.split()[1]) for t in _writes(r, "vel_limits", t0)]
        assert grenzen and grenzen[0] >= w0 - 1e-6
        assert all(b >= a for a, b in zip(grenzen, grenzen[1:]))          # nur anheben
    finally:
        r.close()


def test_kurs_nachlaufender_imu_kurs_schiebt_ziel_nicht():
    """Der Restfehler kommt vom IMU-Kurs, der dem Generator nachläuft (Rider 2026-11-25: bei 90 °/s mehrere Grad). Das
    darf kein neues Ziel auslösen: ein Rahmen, Generator dreht genau 90° (vorher 98° bzw. 105° durch nachgeschobene
    Ziele; 0,3 s Nachlauf = 27° bei 90 °/s wie am Rider)."""
    import collections
    cfg = _cfg(vel_kurs=True)
    r = SimRobot(cfg)
    r.start()
    try:
        t0 = time.monotonic()
        verlauf = collections.deque()                                      # (t, Generatorwinkel) für 0,3 s Nachlauf
        for _ in range(150):
            jetzt = time.monotonic()
            verlauf.append((jetzt, r._gen_winkel))
            while len(verlauf) > 1 and verlauf[1][0] <= jetzt - 0.3:
                verlauf.popleft()
            r.drive(0.0, 0.5, kurs=(math.radians(90.0 - verlauf[0][1]), cfg.w_of(0.5)))   # bis 90 °/s
            time.sleep(0.02)
        kurse = {t for t in _writes(r, "vel", t0) if t.startswith("kurs ")}
        assert len(kurse) == 1, kurse
        assert abs(r._gen_winkel - 90.0) < 0.6
    finally:
        r.close()


def test_kursabs_robot_schickt_nur_ziel():
    """Absolutes Kursziel: vel_limits mit alpha/Drehruck (18 Byte), je Durchlauf nur Ziel + w_max (kein Wechselbit, kein
    w_max über vel_limits), nachlaufender IMU-Kurs verschiebt das Ziel nicht, 300° stückweise (≤ 170° voraus), ohne
    Kursziel wieder Drehrate."""
    import collections
    from xgonav.fahrrahmen import wrap180

    def fw(ref, err):                       # wie App._kurs_fw: Bezug = Kurs zur selben Zeit wie der Restfehler
        return wrap180(ref + max(-170.0, min(170.0, err)))

    cfg = _cfg(vel_kursabs=True)
    r = SimRobot(cfg)
    r.start()
    try:
        assert r.kursabs_an
        assert _writes(r, "vel_limits")[-1] == "227 180.0 500 3970 300 600 250 360 1800"
        t0 = time.monotonic()
        verlauf = collections.deque()
        for _ in range(150):                                               # 3 s, Kurs läuft 0,3 s nach
            jetzt = time.monotonic()
            verlauf.append((jetzt, r._gen_winkel))
            while len(verlauf) > 1 and verlauf[1][0] <= jetzt - 0.3:
                verlauf.popleft()
            err = 90.0 - verlauf[0][1]
            r.drive(0.0, 0.5, kurs=(math.radians(err), cfg.w_of(0.25), fw(verlauf[0][1], err)))
            time.sleep(0.02)
        assert abs(r._gen_winkel - 90.0) < 0.6 and r._kurs.done
        vel = _writes(r, "vel", t0)
        assert vel and all(t.startswith("kursabs ") for t in vel)
        assert {t.split()[2] for t in vel} == {"45.0"}                     # w_max 0,25 x max_w im Rahmen
        assert not _writes(r, "vel_limits", t0)[1:] or all(t.split()[1] == "180.0" for t in _writes(r, "vel_limits", t0))
        t1 = time.monotonic()
        start = r._gen_winkel
        for _ in range(300):                                               # 300° links: Ziel wandert mit
            err = start + 300.0 - r._gen_winkel
            r.drive(0.0, 0.5, kurs=(math.radians(err), cfg.w_of(0.5), fw(r._gen_winkel, err)))
            time.sleep(0.02)
            if r._kurs.done and abs(r._gen_winkel - start - 300.0) < 1.0:
                break
        assert abs(r._gen_winkel - start - 300.0) < 1.0
        w = [float(t) for t in [r.command[1]]]
        assert w[0] == pytest.approx(0.0, abs=1e-6)
        assert min(int(t.split()[1]) for t in _writes(r, "vel", t1)) > -18001
        t2 = time.monotonic()
        for _ in range(10):
            r.drive(0.0, 0.0)
            time.sleep(0.02)
        assert not _writes(r, "vel", t2)[-1].startswith("kursabs ")
    finally:
        r.close()


def test_kursabs_aus_oder_ohne_faehigkeit():
    """vel_kursabs aus: Kursziel Δψ wie bisher, vel_limits mit 7 Feldern; ohne Fähigkeitsbit (Treiber mit 14 Feldern)
    kein absolutes Kursziel und keine 18-Byte-Grenzen (ältere Firmware verwürfe sie ganz)."""
    cfg = _cfg()
    r = SimRobot(cfg)
    r.start()
    try:
        assert not r.kursabs_an and r.kurs_an
        assert len(_writes(r, "vel_limits")[-1].split()) == 7
        for _ in range(5):
            r.drive(0.0, 0.5, kurs=(math.radians(30.0), cfg.w_of(0.25)))
            time.sleep(0.02)
        assert _writes(r, "vel")[-1].startswith("kurs ")
    finally:
        r.close()
    r = SimRobot(_cfg(vel_kursabs=True))
    r._caps_setup(zustand("1 0 0 1 0 0.0 600 100.0 300 0 23 0 0 0")["caps"])
    assert not r.kursabs_an and r.kurs_an


def test_kursabs_imu_bezug():
    """Kurs der Firmware aus dem IMU-Kurs: Abstand zur neuesten Probe durch den Kursfaktor der Richtung, auf ±180."""
    from xgonav.imu import ImuSource
    c = Config.neutral().imu
    c.yaw_scale_left, c.yaw_scale_right = 1.01, 1.02
    imu = ImuSource(c)
    assert imu.fw_kurs_deg(0.0) is None
    imu.add(1, 1.0, 0.0, 0.0, math.radians(170.0))
    imu.add(2, 1.01, 0.0, 0.0, math.radians(175.0))                        # Historie: 170 + 5 · 1,01
    y = imu.latest_yaw()
    assert math.degrees(y) == pytest.approx(175.05)
    assert imu.fw_kurs_deg(y + math.radians(20.2)) == pytest.approx(-165.0)   # 175 + 20,2/1,01 = 195 -> -165
    assert imu.fw_kurs_deg(y - math.radians(10.2)) == pytest.approx(165.0)    # 175 − 10,2/1,02


def test_handsteuerung_im_bogen_setzt_kruemmung():
    """/drive?bogen=1: Manual trägt κ = Drehrate/Tempo; ohne bogen und im Stand keins."""
    from xgonav.behaviors import Manual
    from xgonav.mathe import w_to_yaw_rate
    cfg = Config.neutral()
    m = Manual(cfg.behavior, cfg.robot)
    t = time.monotonic()
    m.set(0.2, 0.3, t, bogen=True)
    c = m.step(None, now=t)
    v_mps = cfg.robot.v_of(0.2) * cfg.robot.vx_limit
    assert c.kappa == pytest.approx(w_to_yaw_rate(cfg.robot.w_of(0.3), cfg.robot) / v_mps) and "Bogen" in c.status
    m.set(0.2, 0.3, t)
    assert m.step(None, now=t).kappa is None
    m.set(0.0, 0.3, t, bogen=True)
    assert m.step(None, now=t).kappa is None                              # auf der Stelle: kein Bogen


def test_handdrehung_schickt_kursziel_im_nachlauf():
    """POST /turn: Nach der Ankunft schickt die Handsteuerung turn_settle_s lang weiter das Kursziel (Firmware 3c9f6fc0
    hält den Kurs nur, solange Kursziel-Rahmen kommen), danach Stillstand ohne Kursziel."""
    from xgonav.behaviors import Manual
    cfg = Config.neutral()
    m = Manual(cfg.behavior, cfg.robot)
    t = 100.0
    m.turn(90.0, 0.25, 3.0, t, 0.0, 10.0)
    c = m.step(None, yaw=math.radians(88.5), now=t + 1.0)                 # innerhalb tol: angekommen
    assert c.w == 0.0 and c.kurs is not None and c.kurs[0] == pytest.approx(math.radians(1.5))
    c = m.step(None, yaw=math.radians(89.5), now=t + 1.5)                 # Nachlauf: weiter Kursziel
    assert c.kurs is not None
    c = m.step(None, yaw=math.radians(90.0), now=t + 1.0 + cfg.behavior.turn_settle_s + 0.01)
    assert c.kurs is None and m.turn_info["fertig"]


def test_halt_nach_bogen_bremst_auf_dem_bogen():
    """Halt (drive(0, 0)) nach einer Bogenfahrt: weiter Krümmungsrahmen mit v 0, w = κ · v_soll bis zum Stand (Rider
    2026-11-25: w fiel sofort auf 0, er bremste geradeaus); danach Drehratenweg. Drehen auf der Stelle schaltet sofort."""
    cfg = _cfg(vel_kappa=True)
    r = SimRobot(cfg)
    r.start()
    try:
        k = 2.0
        for _ in range(40):
            r.drive(0.2, 0.0, kappa=k)
            time.sleep(0.02)
        t0 = time.monotonic()
        vk, v_alt = [], r.command[0]
        for _ in range(60):
            r.drive(0.0, 0.0)
            time.sleep(0.02)
            v, w = r.command
            if v > 1e-3:
                vk.append(math.radians(w * cfg.vyaw_limit_dps) / (v * cfg.vx_limit))
        assert v_alt > 0.1 and vk and all(abs(x - k) < 1e-6 for x in vk)
        vel = _writes(r, "vel", t0)
        assert any(t.startswith("kap 0 ") for t in vel)                    # Bogenrahmen mit Ziel v 0
        assert r.command == (0.0, 0.0) and not vel[-1].startswith("kap ")  # steht: wieder Drehratenweg
        for _ in range(20):
            r.drive(0.2, 0.0, kappa=k)
            time.sleep(0.02)
        t1 = time.monotonic()
        r.drive(0.0, 0.3)                                                  # Drehen auf der Stelle: sofort Drehrate
        time.sleep(0.1)
        assert not any(t.startswith("kap ") for t in _writes(r, "vel", t1))
    finally:
        r.close()


def test_fahrrahmen_wird_nachtraeglich_erkannt(tmp_path):
    """Antwortet die Firmware beim Start nicht (Flashen), schaltet ProcRobot später selbst auf den Fahrrahmen um."""
    cfg = _cfg(proc_dir=str(tmp_path))
    r = ProcRobot(cfg)
    (tmp_path / "vel").write_text("Fehler -110\n")
    (tmp_path / "vel_limits").write_text("")
    r._vel_setup()
    assert not r.vel
    r._vel_nachpruefen(r._vel_neu_t + 1.0)
    assert not r.vel                                                       # zu früh: keine Abfrage
    r._vel_nachpruefen(r._vel_neu_t + r.VEL_NEU_S)
    assert not r.vel                                                       # Firmware antwortet noch nicht
    (tmp_path / "vel").write_text("0 0 0 0 0 0.0 0 0.0 0 0 3 0 0\n")
    r._vel_nachpruefen(r._vel_neu_t + r.VEL_NEU_S)
    assert r.vel and r.bogen and r.kurs_an
    assert (tmp_path / "vel_limits").read_text().startswith("227 180.0")


def test_strecke_text_und_robot_haelt_am_ende():
    """Stufe 3: Rahmen "dist …"/"kapdist …" mit Reststrecke in mm; Robot (Nachbau) steht nach der verlangten Strecke."""
    assert strecke_text(0.18, 0.5, 2.5, 0.0, None, 360, 1800, 3.9, 1.0) == "dist 46 128 641 0.0 360 1800 1000"
    assert strecke_text(0.18, 0.5, 2.5, None, 2.0, 360, 1800, 3.9, 0.785) == "kapdist 46 128 641 2000 360 1800 785"
    cfg = _cfg(vel_strecke=True)
    r = SimRobot(cfg)
    r._setup()                                                             # ohne Thread: fester Takt 20 ms
    assert r.strecke_an
    ziel, weg, dt = 0.5, 0.0, 0.02
    for _ in range(250):                                                   # bis 5 s
        r._ramp_vel((r.cfg.v_of(0.2), 0.0), dt, strecke=max(0.0, ziel - weg))
        weg += r.command[0] * cfg.vx_limit * dt                            # Odometrie = Sollwert
        if weg > 0.1 and r.command[0] == 0.0:
            break
    assert r.command[0] == 0.0 and abs(weg - ziel) < 0.01, weg          # steht auf 1 cm genau
    assert r._frame.startswith("dist ")


def test_handsteuerung_mit_strecke():
    from xgonav.behaviors import Manual
    cfg = Config.neutral()
    m = Manual(cfg.behavior, cfg.robot)
    m.set(0.2, 0.0, 10.0, strecke=1.0)
    assert m.strecke_soll == 1.0 and m.strecke_odo0 is None
    m.strecke_odo0 = 3.0
    m.set(0.2, 0.0, 10.1, strecke=1.0)                                     # Wiederholung mit 10 Hz: Bezug bleibt
    assert m.strecke_odo0 == 3.0
    m.set(0.2, 0.0, 10.2, strecke=0.5)                                     # neue Strecke: neuer Bezug
    assert m.strecke_odo0 is None
    m.set(0.2, 0.0, 10.3)
    assert m.strecke_soll is None
