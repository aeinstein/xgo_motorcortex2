"""Gedächtnis des Modus pet (xgonav/gedaechtnis.py): Datei im Kreis, kaputte Datei, Schreibtakt, Uhr zurück; App
hält ein Objekt für alle Modi und schreibt es beim Beenden (Simulator)."""
import dataclasses
import json
import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xgonav.config import Config, PetCfg  # noqa: E402
from xgonav.gedaechtnis import SPEICHER_S, Gedaechtnis  # noqa: E402


def test_datei_im_kreis(tmp_path):
    pfad = str(tmp_path / "g.json")
    g = Gedaechtnis(pfad, PetCfg())
    assert g.personen == {} and not g.dirty
    g.gesehen("anna", 100.0)
    g.begegnung("anna")
    g.setze_vertrauen("anna", 0.8)
    assert g.speichern() and not g.dirty
    d = json.load(open(pfad, encoding="utf-8"))
    assert d["version"] == 1 and d["personen"]["anna"] == {"vertrauen": 0.8, "zuletzt": 100.0, "erstmals": 100.0,
                                                           "begegnungen": 1}
    h = Gedaechtnis(pfad, PetCfg())
    assert h.vertrauen("anna") == 0.8 and h.zuletzt("anna") == 100.0 and h.abwesend("anna", 160.0) == 60.0
    assert h.vertrauen("bob") == 1.0 and h.abwesend("bob", 160.0) is None   # unbekannt: trust_bekannt, nie gesehen
    assert h.info(160.0) == {"anna": {"vertrauen": 0.8, "begegnungen": 1, "zuletzt_vor_s": 60}}


def test_kaputte_datei_startet_leer(tmp_path, caplog):
    pfad = tmp_path / "g.json"
    pfad.write_text("{kein json", encoding="utf-8")
    with caplog.at_level(logging.WARNING):
        g = Gedaechtnis(str(pfad), PetCfg())
    assert g.personen == {} and "nicht lesbar" in caplog.text


def test_schreibt_hoechstens_je_minute(tmp_path):
    g = Gedaechtnis(str(tmp_path / "g.json"), PetCfg())
    assert not g.speichern(0.0)                                  # nichts neu
    g.gesehen("anna", 1.0)
    assert g.speichern(0.0)
    g.gesehen("anna", 2.0)
    assert not g.speichern(SPEICHER_S - 1.0) and g.dirty
    assert g.speichern(SPEICHER_S + 0.1) and not g.dirty
    g.gesehen("anna", 2.0)                                       # gleiche Zeit: nichts neu
    assert not g.dirty


def test_uhr_zurueck_und_ohne_datei(tmp_path):
    g = Gedaechtnis("", PetCfg())
    g.gesehen("anna", 1000.0)
    assert g.abwesend("anna", 900.0) == 0.0                      # Uhr zurück: nicht negativ
    assert not g.speichern()                                     # ohne Pfad: nur im Speicher
    h = Gedaechtnis(str(tmp_path / "fehlt" / "g.json"), PetCfg())   # Ordner fehlt: Warnung, weiter im Speicher
    h.gesehen("anna", 1.0)
    assert not h.speichern() and h.dirty


def test_app_ein_gedaechtnis_fuer_alle_modi(tmp_path):
    from xgonav.app import App
    from xgonav.robot import SimRobot
    from xgonav.sim import SimFloorNet, SimWorld
    pfad = str(tmp_path / "g.json")
    cfg = Config.neutral()
    assert cfg.pet.gedaechtnis == ""                             # Simulator und Tests schreiben nichts
    cfg.output.web_port = 0
    cfg.detector.enabled = False
    cfg.pet = dataclasses.replace(cfg.pet, gedaechtnis=pfad)
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
    app.imu = None
    a = app._make("pet")
    app._make("idle")
    b = app._make("pet")
    assert a.brain.gedaechtnis is app.gedaechtnis and b.brain.gedaechtnis is app.gedaechtnis
    app.gedaechtnis.gesehen("anna", 1.0)
    app.gedaechtnis._t_save = 1e18                               # Takt nie erreicht: erst das Beenden schreibt
    app.run(duration=1.0)
    assert json.load(open(pfad, encoding="utf-8"))["personen"]["anna"]["zuletzt"] == 1.0
