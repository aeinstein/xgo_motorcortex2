"""Horizont im Stand: Standreferenz, Tabelle je Körperhöhe, Einbindung in App (xgonav/horizon.py)."""
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.config import Config, HorizonCfg  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.horizon import HorizonTable, StandRef, row_to_tilt_deg  # noqa: E402


class FakeImu:
    def __init__(self):
        self.t, self.p = [], []

    def feed(self, t0, t1, deg, noise_deg=0.0, hz=50.0, seed=1):
        rng = np.random.default_rng(seed)
        for t in np.arange(t0, t1, 1.0 / hz):
            self.t.append(float(t))
            self.p.append(math.radians(deg + noise_deg * rng.standard_normal()))

    def arrays(self):
        return np.array(self.t), np.array(self.p)


def _cfg(**kw):
    c = HorizonCfg(settle_s=3.0, window_s=3.0, max_std_deg=0.5, min_samples=50)
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def test_standref_misst_erst_nach_aufstehen_und_einschwingen():
    imu = FakeImu()
    imu.feed(0.0, 20.0, -1.2, noise_deg=0.1)
    s = StandRef(_cfg())
    assert not s.update(0.0, False, 90.0, True, imu) and s.state == "liegt" and s.ref is None
    assert not s.update(1.0, True, 90.0, True, imu) and s.state == "wartet"       # aufgestanden
    assert not s.update(3.5, True, 90.0, True, imu) and s.state == "wartet"
    assert not s.update(5.0, True, 90.0, True, imu) and s.state == "misst"
    assert s.update(7.1, True, 90.0, True, imu)
    assert s.state == "gültig" and math.degrees(s.ref) == pytest.approx(-1.2, abs=0.03)
    assert s.height_mm == 90.0 and s.n >= 140
    assert not s.update(8.0, True, 90.0, False, imu) and s.state == "gültig"      # Fahren ändert ref nicht
    # absetzen: Referenz weg, nach erneutem Aufstehen wieder warten
    s.update(9.0, False, 90.0, True, imu)
    assert s.ref is None and s.state == "liegt"
    s.update(10.0, True, 90.0, True, imu)
    assert s.state == "wartet" and s.ref is None


def test_standref_neu_bei_hoehe_bewegung_und_streuung():
    imu = FakeImu()
    imu.feed(0.0, 6.0, -1.0, noise_deg=0.1)
    imu.feed(6.0, 40.0, 2.0, noise_deg=0.1)
    s = StandRef(_cfg())
    s.update(0.0, True, 90.0, True, imu)
    assert s.update(6.1, True, 90.0, True, imu) and math.degrees(s.ref) == pytest.approx(-1.0, abs=0.03)
    # Höhe geändert: neu messen, ab dem Ereignis
    assert not s.update(6.5, True, 100.0, True, imu) and s.ref is None and s.state == "wartet"
    assert not s.update(8.0, True, 100.0, False, imu)       # fährt beim Einschwingen: Neustart ab 8,0
    assert not s.update(13.0, True, 100.0, True, imu) and s.state == "misst"
    assert s.update(14.1, True, 100.0, True, imu)
    assert math.degrees(s.ref) == pytest.approx(2.0, abs=0.03) and s.height_mm == 100.0
    # Streuung zu groß: Fenster verworfen, kein ref
    imu2 = FakeImu()
    imu2.feed(0.0, 20.0, 0.0, noise_deg=2.0)
    s2 = StandRef(_cfg())
    s2.update(0.0, True, 90.0, True, imu2)
    assert not s2.update(6.1, True, 90.0, True, imu2)
    assert s2.ref is None and s2.verworfen == 1 and s2.state == "misst"


def test_horizontabelle_interpoliert_ersetzt_und_speichert(tmp_path):
    p = str(tmp_path / "horizont.json")
    t = HorizonTable(p)
    assert t.row_at(90.0) is None and not t.covers(90.0)
    t.set(90.1, 262.0, -1.1)
    t.set(110.0, 250.0, -0.4)
    assert t.row_at(90.1) == pytest.approx(262.0)
    assert t.row_at(100.05) == pytest.approx(256.0, abs=0.01)
    assert t.row_at(80.0) == pytest.approx(262.0) and not t.covers(80.0)     # außerhalb: nächster Wert
    assert t.covers(100.0)
    t.set(110.5, 248.0, -0.5)                                              # gleiche Höhe (±1 mm): ersetzt
    assert len(t.points) == 2 and t.row_at(110.5) == pytest.approx(248.0)
    t2 = HorizonTable(p)                                                   # von der Platte
    assert [q["row"] for q in t2.points] == [262.0, 248.0]
    assert t2.delete(90.0) and not t2.delete(50.0)
    assert len(HorizonTable(p).points) == 1


def test_zeile_neigung_hin_und_zurueck():
    g = CamGeometry(640, 480, 88.5, 0.0, 43.0, 85.0)
    for row in (200.0, 240.0, 271.3):
        g.tilt_deg = row_to_tilt_deg(row, g.f, g.H)
        assert g.horizon_row(0.0) == pytest.approx(row, abs=1e-6)


def test_app_horizont_folgt_imu_ohne_verzoegerung(tmp_path):
    """Im Simulator: Standreferenz nach dem Start, Nick = IMU minus Referenz folgt dem wahren Nicken
    (ohne Tiefpass), eingestellte Zeile wird zur Neigung dieser Höhe."""
    from xgonav.app import App
    from xgonav.robot import SimRobot
    from xgonav.sim import SimImu, SimWorld
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    cfg.freespace.enabled = False
    cfg.horizon.enabled = True
    cfg.horizon.file = str(tmp_path / "horizont.json")
    cfg.horizon.settle_s, cfg.horizon.window_s, cfg.horizon.max_std_deg = 0.3, 2.0, 5.0
    cfg.imu.pitch_sign, cfg.imu.time_offset_ms = -1, -8.0      # wie SimImu: Vorzeichen -1, Stempel 8 ms früh
    robot = SimRobot(cfg.robot)
    # shake 4: im Stand nickt der Simulator nur ein Viertel so stark wie in Fahrt (sim.STAND_FACTOR); der Test
    # braucht einen Hub deutlich über dem Rauschen der Sim-IMU (0,05°)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0), shake=4.0)
    app = App(cfg, robot, world, imu=SimImu(world, cfg.imu))
    th = threading.Thread(target=app.run, kwargs={"duration": 8.0}, daemon=True)
    th.start()
    try:
        t_end = time.monotonic() + 5.0
        while time.monotonic() < t_end and app.pitch_src != "stand+bild":
            time.sleep(0.05)
        assert app.pitch_src == "stand+bild", app.snapshot.horizon
        with pytest.raises(ValueError):
            app.horizon_calibrate(600.0)
        e = app.horizon_calibrate(255.0)
        assert e["row"] == 255.0 and HorizonTable(cfg.horizon.file).points
        pairs = []
        seen = set()
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            s = app.snapshot
            tr = world.truth_by_t.get(s.t)
            if tr is not None and s.idx not in seen and s.horizon and s.horizon["row_imu"] is not None:
                seen.add(s.idx)
                pairs.append((s.pitch_dev, tr["pitch"], s.horizon["row_stand"]))
            time.sleep(0.01)
        assert len(pairs) > 20
        pd, truth, row_stand = map(np.array, zip(*pairs))
        # Rest: konstanter Versatz (Mittel im Fenster), Rauschen und lineare Interpolation der IMU (unter
        # Windows tickt die Sim-IMU nur alle ~31 ms, dann fehlen die 9- und 13-Hz-Anteile teilweise).
        # Mit dem alten Tiefpass (α 0,15 je Bild, ~0,5 s) läge der Rest beim vollen Hub.
        d = pd - truth
        assert np.std(d) < 0.35 * np.std(truth), (math.degrees(np.std(d)), math.degrees(np.std(truth)))
        assert np.corrcoef(pd, truth)[0, 1] > 0.93
        assert np.allclose(row_stand[2:], 255.0, atol=0.1)      # erstes Bild kann noch vor dem Speichern liegen
        assert app.geo.tilt_deg == pytest.approx(row_to_tilt_deg(255.0, app.geo.f, app.geo.H))
        st = app.snapshot.stats["horizon"]
        assert st["zustand"] == "gültig" and st["row_cal"] == 255.0
        assert st["imu_roh_deg"] is not None and abs(st["row_imu_roh"] - st["row_imu"]) < 30
        # Bildlinie läuft nie weg: spätestens nach bild_max_s wieder auf der IMU
        app._hz_bild += math.radians(5.0)               # künstliche Drift
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end and app.snapshot.horizon["bild_minus_imu_deg"] > 2.0:
            time.sleep(0.02)
        assert abs(app.snapshot.horizon["bild_minus_imu_deg"]) < 2.0, app.snapshot.horizon
    finally:
        app.stop(); th.join(timeout=5)


def test_ws_kopf_horizontzeilen():
    """/ws?nick=1: row_stand/row_imu als float32 im Kopf, fehlend NaN (Browser verschiebt dann nicht)."""
    import struct
    from xgonav.web import _hz_rows
    st, imu = struct.unpack("!ff", struct.pack("!ff", *_hz_rows({"row_stand": 240.0, "row_imu": 251.5})))
    assert (st, imu) == (240.0, 251.5)
    assert all(math.isnan(v) for v in _hz_rows(None))
    assert math.isnan(_hz_rows({"row_stand": 240.0, "row_imu": None})[1])
