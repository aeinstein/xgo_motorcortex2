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
def test_pet_ruck_im_budget_und_keine_fehlalarme():
    r = run("pet", 8.0, imu=True, wheels=True, dyn=True, animal="person", speed=2.5,
            overrides={"robot.vx_limit": 1.8, "pet.retreat_m": 0.12})
    assert r["pet"]["schrecks"] >= 1 and r["pet"]["ruck"] == 1
    assert r["collisions"] == 0 and r["weg_von_start_m"] <= 0.12 and r["dist_m"] > 0.03
    r = run("pet", 14.0, imu=True, wheels=True, dyn=True, animal="person", speed=0.3)
    assert r["pet"]["schrecks"] == 0 and r["dist_m"] == 0.0


@pytest.mark.slow
def test_pet_hund_ohnmacht_steht_wieder_auf():
    r = run("pet", 30.0, imu=True, wheels=True, dyn=True, animal="dog", speed=0.5,
            overrides={"robot.vx_limit": 1.8, "pet.faint": True})
    assert r["aktionen"] == [254, 253] and r["pet"]["zustand"] == "wach"


@pytest.mark.slow
def test_explore_no_collisions():
    r = run("explore", 30.0)
    assert r["collisions"] == 0
    assert r["dist_m"] > 2.0            # bewegt sich tatsächlich


@pytest.mark.slow
def test_follow_with_imu_autosync():
    r = run("follow", 30.0, imu=True)
    ok, sign, off_ms, corr = r["imu_sync"]
    # Simulator: Vorzeichen -1, Versatz -8 ms. Mit dem am Rider gemessenen Nicken (sim.NICK, vor allem 0,8 Hz)
    # trifft die Suche den Versatz nur noch auf 8-16 ms (vier Läufe 2026-11-11: 0, +4, +4, +8 ms); mit dem früheren
    # 3°-Nicken bei 2,5 Hz waren es 4 ms.
    assert ok and sign == -1 and abs(off_ms + 8) <= 20
    assert r["collisions"] == 0


@pytest.mark.slow
def test_home_returns_to_start():
    # 50 s erkunden (davon ~14 s Rundumblick), dann zum Start; mit Rad-Odometrie (sonst driftet der Kamerakurs)
    r = run("home", 100.0, wheels=True)
    assert r["collisions"] == 0
    assert r["home_weg_m"] > 1.0        # war tatsächlich weg
    assert r["home_end_m"] < 0.35       # echte Endpose nahe der echten Startpose


# Navigation im Simulator ruht seit 2026-11-11 (Nutzer: deaktiviert lassen, bis es an der Navigationslogik weitergeht).
# Mit dem am Rider gemessenen Nicken (sim.NICK) zeichnet die Raumkarte Wände zu nah; test_explore_maps_room war
# schon vorher rot (Bodenabdeckung 0,79 statt 0,8), Streife stieß unter Last zweimal an.
NAV_RUHT = pytest.mark.skip(reason="Navigationslogik ruht seit 2026-11-11, siehe Kommentar")


@NAV_RUHT
@pytest.mark.slow
def test_patrol_route_after_mapping():
    # Kartieren, drei Punkte wählen, abfahren, zum Start; wie am Rider (IMU für das Nicken, Kurs aus den Rädern)
    from sim_run import RIDER, RIDER_VX
    r = run("patrol", 600.0, wheels=True, imu=True, method="kanten", fs_thread=True, dyn=True, vx_true=RIDER_VX, overrides=RIDER)
    assert r["collisions"] == 0 and r["route_fehler"] == [] and r["streife_s"] is not None
    p = r["explore"]
    assert p["zustand"] == "fertig" and p["erreicht"] == 3 and p["ausgelassen"] == []
    assert "am Start" in p["ende"] and r["karte_gesperrt"]


@NAV_RUHT
@pytest.mark.slow
def test_explore_maps_room():
    # Bis „Karte fertig“ (3-5 min, zuletzt die Engstellen): mit Rider-Einstellungen, kanten, Rädern und Trägheit
    from sim_run import RIDER, RIDER_VX
    r = run("explore", 450.0, wheels=True, imu=True, method="kanten", fs_thread=True, dyn=True, vx_true=RIDER_VX, overrides=RIDER)
    g = r["grid"]
    assert r["collisions"] == 0
    assert r["status"].startswith("Karte fertig") and r["fertig_s"] is not None
    assert g["boden_bekannt"] >= 0.8 and g["waende_10cm"] >= 0.7
    assert g["praezision_15cm"] >= 0.85 and g["frei_aussen_m2"] < 1.0
