"""Rohaufzeichnung: schreibt nur mit Schalter an, balancierend und laufender Bildverarbeitung."""
import json
import os
import sys
import threading
import time
from collections import namedtuple
from types import SimpleNamespace
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xgonav.app import Snapshot
from xgonav.rawrec import RawRecorder


class FakeApp:
    def __init__(self):
        self.running = True
        self.pipeline_on = True
        self.robot = SimpleNamespace(state=SimpleNamespace(balancing=True))
        self.snapshot = Snapshot()
        self.snap_cond = threading.Condition()
        self.idx = 0

    def bild(self):
        self.idx += 1
        snap = Snapshot(frame=np.zeros((8, 8, 3), np.uint8), idx=self.idx,
                        stats={"balancing": self.robot.state.balancing})
        with self.snap_cond:
            self.snapshot = snap
            self.snap_cond.notify_all()


def warte(bedingung, s=3.0):
    t_end = time.monotonic() + s
    while time.monotonic() < t_end:
        if bedingung():
            return True
        time.sleep(0.005)
    return False


def bilder_bis(app, rec, n):
    """Bilder nachlegen, bis n weitere geschrieben sind (der Aufzeichner darf Bilder auslassen)."""
    ziel = rec.bilder + n if rec.status()["laeuft"] else n
    return warte(lambda: (app.bild(), time.sleep(0.01), rec.bilder >= ziel and rec.status()["laeuft"])[2])


def laeufe(base):
    return sorted(os.listdir(base)) if os.path.isdir(base) else []


def test_aus_startet_keinen_thread_und_schreibt_nichts(tmp_path):
    app, base = FakeApp(), str(tmp_path / "fahrt")
    rec = RawRecorder(app, base)
    app.bild()
    assert rec._th is None
    assert not rec.status()["an"] and not os.path.exists(base)
    rec.beenden()


def test_nur_waehrend_des_balancierens_und_neue_aufnahme_je_aufstehen(tmp_path):
    app, base = FakeApp(), str(tmp_path / "fahrt")
    app.robot.state.balancing = False
    app.bild()
    rec = RawRecorder(app, base)
    rec.schalten(True)
    for _ in range(5):
        app.bild()
        time.sleep(0.01)
    assert rec.status()["an"] and not rec.status()["laeuft"]
    assert laeufe(base) == []                       # liegt: nicht einmal ein leeres Verzeichnis

    app.robot.state.balancing = True
    with mock.patch("time.strftime", return_value="20261109-120000"):
        assert bilder_bis(app, rec, 3)

        app.robot.state.balancing = False           # absetzen: Aufnahme zu, Schalter bleibt an
        assert warte(lambda: not rec.status()["laeuft"])
        assert rec.status()["an"]
        erste = os.path.join(base, "20261109-120000")
        n = len(open(os.path.join(erste, "meta.jsonl")).readlines())
        assert n >= 3
        for _ in range(5):
            app.bild()
            time.sleep(0.01)
        assert len(open(os.path.join(erste, "meta.jsonl")).readlines()) == n

        app.robot.state.balancing = True            # wieder aufstehen: neues Unterverzeichnis
        assert bilder_bis(app, rec, 2)
    assert laeufe(base) == ["20261109-120000", "20261109-120000-2"]
    rec.beenden()
    zeilen = [json.loads(z) for z in open(os.path.join(erste, "meta.jsonl"))]
    assert all(z["balancing"] is True for z in zeilen)
    assert all(os.path.exists(os.path.join(erste, z["file"])) for z in zeilen)


def test_standbild_von_vorher_wird_nicht_aufgenommen(tmp_path):
    app, base = FakeApp(), str(tmp_path / "fahrt")
    app.bild()
    rec = RawRecorder(app, base)
    rec.schalten(True)
    time.sleep(0.1)
    assert not rec.status()["laeuft"] and laeufe(base) == []
    assert bilder_bis(app, rec, 1)
    rec.beenden()


def test_bildverarbeitung_aus_beendet_die_aufnahme(tmp_path):
    app, base = FakeApp(), str(tmp_path / "fahrt")
    rec = RawRecorder(app, base)
    rec.schalten(True)
    assert bilder_bis(app, rec, 1)
    app.pipeline_on = False
    assert warte(lambda: not rec.status()["laeuft"])
    assert rec.status()["an"]
    rec.beenden()


def test_abschalten_und_kein_platz(tmp_path):
    app, base = FakeApp(), str(tmp_path / "fahrt")
    rec = RawRecorder(app, base)
    rec.schalten(True)
    assert bilder_bis(app, rec, 1)
    rec.schalten(False)
    assert warte(lambda: not rec.status()["laeuft"])

    Du = namedtuple("Du", "total used free")
    with mock.patch("shutil.disk_usage", return_value=Du(1, 1, 10 * 1024 * 1024)):
        rec.schalten(True)
        assert warte(lambda: (app.bild(), time.sleep(0.01), not rec.status()["an"])[2])
    st = rec.status()
    assert "10 MB" in st["fehler"] and not st["laeuft"]
    assert len(laeufe(base)) == 1                   # kein leeres zweites Verzeichnis
    rec.beenden()


def test_proben_seit_dem_letzten_bild_stehen_genau_einmal_in_der_meta(tmp_path):
    import math

    from xgonav.config import ImuCfg, OdomCfg
    from xgonav.imu import ImuSource
    from xgonav.wheels import WheelOdom

    app, base = FakeApp(), str(tmp_path / "fahrt")
    app.imu, app.wheels = ImuSource(ImuCfg()), WheelOdom(OdomCfg())
    seq = [0]

    def bild():                                     # je Bild zwei IMU-Proben und eine Radprobe, Zeit wie das Bild
        app.idx += 1
        t = 100.0 + app.idx * 0.04
        for dt in (-0.02, -0.01):
            seq[0] += 1
            app.imu.add(seq[0], t + dt, math.radians(1.5), math.radians(-2.0), math.radians(seq[0] * 0.5))
        app.wheels.add(app.idx, t - 0.01, app.idx * 100, app.idx * 100)
        snap = Snapshot(frame=np.zeros((8, 8, 3), np.uint8), idx=app.idx, t=t, stats={"balancing": True})
        with app.snap_cond:
            app.snapshot = snap
            app.snap_cond.notify_all()

    app.bild = bild
    rec = RawRecorder(app, base)
    rec.schalten(True)
    assert bilder_bis(app, rec, 5)
    rec.beenden()
    zeilen = [json.loads(z) for z in open(os.path.join(base, laeufe(base)[0], "meta.jsonl"))]
    imu = [p for z in zeilen for p in z["imu"]]
    rad = [p for z in zeilen for p in z["raeder"]]
    ti = [p[0] for p in imu]
    assert ti == sorted(set(ti)) and len(ti) >= 2 * len(zeilen) - 1      # keine doppelt, keine Lücke
    assert all(b - a < 0.035 for a, b in zip(ti, ti[1:]))
    assert imu[0][1:3] == [1.5, -2.0] and abs(imu[1][3] - imu[0][3] - 0.5) < 1e-6   # Grad
    assert ti[0] > zeilen[0]["t"] - 0.2                                  # nichts von lange vor der Aufnahme
    tr = [p[0] for p in rad]
    assert tr == sorted(set(tr)) and rad[-1][1] > rad[0][1] >= 0.0       # Weg in m, fortlaufend


def test_ohne_imu_und_raeder_bleiben_die_felder_leer(tmp_path):
    app, base = FakeApp(), str(tmp_path / "fahrt")
    rec = RawRecorder(app, base)
    rec.schalten(True)
    assert bilder_bis(app, rec, 1)
    rec.beenden()
    z = json.loads(open(os.path.join(base, laeufe(base)[0], "meta.jsonl")).readline())
    assert z["imu"] is None and z["raeder"] is None
