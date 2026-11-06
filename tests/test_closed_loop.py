"""Closed-Loop im Simulator (dauert je ~30 s). pytest -m slow"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_run import run  # noqa: E402


@pytest.mark.slow
def test_follow_person_with_shake():
    r = run("follow", 30.0)
    assert r["collisions"] == 0
    assert r["dist_min"] > 0.8          # nicht auffahren
    assert 1.1 < r["dist_mean"] < 2.4   # dranbleiben (Soll 1.2 m + Totzone, Person läuft)
    assert r["loop_ms"] < 30


@pytest.mark.slow
def test_explore_no_collisions():
    r = run("explore", 30.0)
    assert r["collisions"] == 0
    assert r["dist_m"] > 2.0            # bewegt sich tatsächlich


@pytest.mark.slow
def test_follow_with_imu_autosync():
    r = run("follow", 30.0, imu=True)
    ok, sign, off_ms, corr = r["imu_sync"]
    assert ok and sign == -1 and abs(off_ms + 8) <= 4   # Simulator: Vorzeichen -1, Versatz -8 ms
    assert r["collisions"] == 0


@pytest.mark.slow
def test_home_returns_to_start():
    # 50 s erkunden (davon ~14 s Rundumblick), dann zum Start; mit Rad-Odometrie (sonst driftet der Kamerakurs)
    r = run("home", 100.0, wheels=True)
    assert r["collisions"] == 0
    assert r["home_weg_m"] > 1.0        # war tatsächlich weg
    assert r["home_end_m"] < 0.35       # echte Endpose nahe der echten Startpose


@pytest.mark.slow
def test_patrol_route_after_mapping():
    # Kartieren, drei Punkte wählen, abfahren, zum Start; wie am Rider, ohne echten IMU-Kurs
    from sim_run import RIDER, RIDER_VX
    r = run("patrol", 420.0, wheels=True, method="kanten", fs_thread=True, dyn=True, vx_true=RIDER_VX, overrides=RIDER)
    assert r["collisions"] == 0 and r["route_fehler"] == [] and r["streife_s"] is not None
    p = r["explore"]
    assert p["zustand"] == "fertig" and p["erreicht"] == 3 and p["ausgelassen"] == []
    assert "am Start" in p["ende"] and r["karte_gesperrt"]


@pytest.mark.slow
def test_explore_maps_room():
    # Bis „Karte fertig“ (2-4 min): mit Rider-Einstellungen, kanten, Rädern und Trägheit
    from sim_run import RIDER, RIDER_VX
    r = run("explore", 300.0, wheels=True, method="kanten", fs_thread=True, dyn=True, vx_true=RIDER_VX, overrides=RIDER)
    g = r["grid"]
    assert r["collisions"] == 0
    assert r["status"].startswith("Karte fertig") and r["fertig_s"] is not None
    assert g["boden_bekannt"] >= 0.8 and g["waende_10cm"] >= 0.7
    assert g["praezision_15cm"] >= 0.85 and g["frei_aussen_m2"] < 1.0
