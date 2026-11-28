"""Innenleben des Modus pet (xgonav/innen.py): Akku glätten und in Stufen, Laden erkennen, Triebe und ihre Faktoren,
Ladewächter; dazu App: Ruhemodus aus pet und Aufwachen nach dem Laden (Simulator)."""
import dataclasses
import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xgonav.config import Config, PetCfg  # noqa: E402
from xgonav.innen import Akku, Innenleben, Ladewaechter, nachts  # noqa: E402


def speise(akku, t0, t1, wert, dt=0.5, faehrt=False):
    t = t0
    while t < t1:
        akku.update(t, wert(t) if callable(wert) else wert, faehrt)
        t += dt
    return t


def test_akku_median_und_stufen_mit_hysterese():
    a = Akku(PetCfg())
    t = speise(a, 0.0, 70.0, 100)                        # Fenster (60 s) voll
    assert a.wert == 100 and a.stufe == "munter"
    t = speise(a, t, t + 25.0, 39)                       # weniger als die halbe Minute: Median noch 100
    assert a.stufe == "munter"
    t = speise(a, t, t + 40.0, 39)
    assert a.wert == 39 and a.stufe == "hungrig"
    t = speise(a, t, t + 70.0, 41)                       # über 40, aber unter 40 + 3: bleibt hungrig
    assert a.stufe == "hungrig"
    speise(a, t, t + 70.0, 43)
    assert a.stufe == "gesetzt"


def test_akku_springt_mehrere_stufen_und_kennt_unbekannt():
    a = Akku(PetCfg())
    speise(a, 0.0, 5.0, 20)
    assert a.stufe == "erschoepft"                       # erster Wert: gleich die passende Stufe
    a.update(10.0, 0)
    a.update(11.0, None)                                 # 0 und None: unbekannt, nichts ändert sich
    assert a.wert == 20 and a.stufe == "erschoepft"
    b = Akku(PetCfg())
    speise(b, 0.0, 70.0, 100)
    speise(b, 70.0, 140.0, 8)
    assert b.stufe == "leer"


def test_akku_laden_nur_in_ruhe():
    a = Akku(PetCfg())
    t = speise(a, 0.0, 400.0, 20)                        # ruhig, Wert steht
    assert not a.laedt
    t = speise(a, t, t + 300.0, lambda t: 20 + (t - 400.0) / 60.0)   # steigt um 1 Punkt je Minute
    assert a.laedt
    speise(a, t, t + 1.0, 25, faehrt=True)               # Fahrt bzw. Lastwechsel: nichts erkannt
    assert not a.laedt


def test_laden_nicht_gleich_nach_dem_absetzen():
    """Last fällt weg, die Spannung erholt sich: in den ersten Minuten kein Laden."""
    a = Akku(PetCfg())
    speise(a, 0.0, 30.0, 20, faehrt=True)
    speise(a, 30.0, 200.0, lambda t: 20 + min(3, (t - 30.0) / 20.0))
    assert not a.laedt


def test_faktoren_genau_eins_bei_voller_energie():
    i = Innenleben(PetCfg())
    i.update(0.05, "munter", 0.0, 0.0, False, False, 0.0, 0.3, None)
    assert i.energie == 1.0
    assert i.f_zeit() == 1.0 and i.f_streifen() == 1.0 and i.f_tempo() == 1.0
    assert i.lid_muede() == 0.0 and i.hoehe_minus() == 0.0 and i.rate() == 15.0 and i.dudeln_ok


def test_energie_akku_muede_nacht():
    i = Innenleben(PetCfg())
    i.update(0.05, "hungrig", 0.0, 0.0, False, False, 0.0, 0.3, 12.0)
    assert i.energie == pytest.approx(0.6)
    assert i.f_zeit() == pytest.approx(0.72) and i.f_tempo() == pytest.approx(0.84)
    assert i.lid_muede() == pytest.approx(0.1) and i.rate() == pytest.approx(15 * 0.84)
    i.update(0.05, "munter", 0.0, 0.0, False, False, 0.0, 0.3, 23.5)
    assert i.energie == pytest.approx(0.7)
    for _ in range(600):                                 # 60 m gefahren
        i.update(0.1, "munter", 0.1, 0.0, False, False, 0.0, 0.3, None)
    assert i.muede == pytest.approx(0.4, abs=0.01) and i.energie == pytest.approx(0.6, abs=0.01)
    assert nachts(23.0, 22, 7) and nachts(6.5, 22, 7) and not nachts(12.0, 22, 7) and nachts(14.0, 13, 15)


def test_geselligkeit_steigt_allein_faellt_bei_gesellschaft():
    i = Innenleben(PetCfg())
    for _ in range(3000):                                # 300 s allein
        i.update(0.1, "munter", 0.0, 0.0, False, False, 0.0, 0.3, None)
    assert i.gesellig == pytest.approx(0.8, abs=0.01)
    for _ in range(100):                                 # 10 s jemand weiter weg: bleibt
        i.update(0.1, "munter", 0.0, 0.0, True, False, 0.0, 0.3, None)
    assert i.gesellig == pytest.approx(0.8, abs=0.01)
    for _ in range(200):                                 # 20 s jemand nah: weg
        i.update(0.1, "munter", 0.0, 0.0, True, True, 0.0, 0.3, None)
    assert i.gesellig == 0.0
    i.wiedersehen()
    assert i.gesellig == 0.3 and i.freude == 1.0


def test_dudeln_mit_hysterese():
    i = Innenleben(PetCfg())
    i.energie = 0.58
    assert i.dudeln_ok                                   # noch über 0,55
    i.energie = 0.5
    assert not i.dudeln_ok
    i.energie = 0.58
    assert not i.dudeln_ok                               # erst ab 0,6 wieder
    i.energie = 0.6
    assert i.dudeln_ok


def test_ladewaechter():
    w = Ladewaechter(PetCfg())
    t, akt = 0.0, []
    while t < 400.0:
        akt += w.update(t, 20, ruhe=True)
        t += 1.0
    assert akt == []                                     # schläft nicht (nicht von pet eingeschläfert)
    w.einschlafen()
    while t < 4000.0:
        akt += w.update(t, int(20 + (t - 400.0) / 50.0), ruhe=True)
        t += 1.0
        if "wecken" in akt:
            break
    assert akt == ["gurren", "wecken"] and not w.schlaf
    w.einschlafen()
    w.vergessen()                                        # Nutzer hat geweckt
    assert w.update(t + 1.0, 95, ruhe=True) == []


def test_app_pet_ruhe_und_aufwachen():
    """Simulator: pet bei leerem Akku -> absetzen, Ruhemodus; geladen -> wach, Bildverarbeitung an, pet liegend."""
    from xgonav.app import App
    from xgonav.robot import SimRobot
    from xgonav.sim import SimFloorNet, SimWorld
    cfg = Config.neutral()
    cfg.output.web_port = 0
    cfg.detector.enabled = False
    cfg.ruhe.tick_s = 0.2
    cfg.pet = dataclasses.replace(cfg.pet, akku_median_s=1.0)
    robot = SimRobot(cfg.robot)
    akku = {"v": 8}
    robot.akku_fn = lambda: akku["v"]
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
    app.imu = None
    th = threading.Thread(target=app.run, kwargs={"duration": 12.0}, daemon=True)
    th.start()
    time.sleep(1.0)
    app.set_mode("pet")
    for _ in range(40):
        time.sleep(0.1)
        if app.ruhe:
            break
    assert app.ruhe and app.lade.schlaf and not app.pipeline_on and app.mode == "idle"
    akku["v"] = 95                                       # geladen
    for _ in range(40):
        time.sleep(0.1)
        if not app.ruhe and app.mode == "pet":
            break
    assert not app.ruhe and app.pipeline_on and app.mode == "pet" and not app.lade.schlaf
    app.stop()
    th.join(timeout=5)
