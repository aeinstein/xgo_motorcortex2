import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.config import Config  # noqa: E402
from xgonav.detector import YoloOnnx, DetectorCfg  # noqa: E402
from xgonav.egomotion import EgoMotion, warp_box  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.robot import SimRobot, norm_to_u8, u8_to_norm  # noqa: E402
from xgonav.sim import SimWorld  # noqa: E402
from xgonav.tracker import Tracker  # noqa: E402
from xgonav.detector import Detection  # noqa: E402


# ---------------------------------------------------------------- Robot
def test_u8_mapping():
    assert norm_to_u8(0) == 128
    assert norm_to_u8(1) == 255 and norm_to_u8(-1) == 1
    assert norm_to_u8(5) == 255
    assert abs(u8_to_norm(norm_to_u8(0.3)) - 0.3) < 0.01


def test_robot_ramp_clamp_and_watchdog():
    cfg = Config().robot
    r = SimRobot(cfg)
    r.start()
    try:
        r.drive(1.0, -1.0)                       # wird auf max_v/max_w begrenzt
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.25:
            r.drive(1.0, -1.0)
            time.sleep(0.02)
        v, w = r.command
        assert 0 < v <= cfg.max_v + 1e-9          # Rampe: noch nicht am Ziel, aber > 0
        assert v < cfg.max_v                       # accel_v 0.8/s -> nach 0.25 s ~0.2
        assert -cfg.max_w - 1e-9 <= w < 0
        time.sleep(cfg.stall_timeout_s + 0.6)      # kein drive() mehr -> Watchdog
        assert r.command == pytest.approx((0.0, 0.0), abs=1e-6)
        names = {n for _, n, _ in r.writes}
        assert {"speed_x", "speed_z"} <= names
    finally:
        r.close()
    assert r.writes[-1][2] == 128


def test_robot_height_mapping():
    r = SimRobot(Config().robot)
    r.set_height(60)
    r.set_height(120)
    r.set_height(500)   # begrenzt, identisch -> kein neuer Write
    vals = [v for _, n, v in r.writes if n == "height"]
    assert vals == [0, 255]


# ---------------------------------------------------------------- Ego-Motion
def test_egomotion_rotation_vs_sim():
    cfg = Config()
    r = SimRobot(cfg.robot)
    r._cur = (0.1, 0.05)
    r._height_mm = 85
    w = SimWorld(cfg.camera, r, None, start=(1.0, 1.5, 0.1), shake=1.0, vx_mps=1.5)
    em = EgoMotion(cfg.ego, (640, 480), cfg.camera.hfov_deg)
    pp = py = None
    ep, ey = [], []
    for k in range(60):
        t = k / 20
        w._integrate(t, 0.05)
        img, _ = w._render(t, 0.05)
        res = em.update(k, t, img)
        if pp is not None and res.ok:
            ep.append(res.pitch - (w.pitch - pp))
            ey.append(res.yaw - ((w.th + w.yaw_jit) - py))
        pp, py = w.pitch, w.th + w.yaw_jit
    assert len(ep) > 50
    assert math.degrees(np.std(ep)) < 0.15
    assert math.degrees(np.std(ey)) < 0.15
    assert abs(math.degrees(np.mean(ey))) < 0.05


def test_warp_box_identity_and_shift():
    A = np.eye(3)
    assert warp_box(A, (10, 20, 30, 40)) == pytest.approx((10, 20, 30, 40))
    A[1, 2] = 5
    assert warp_box(A, (10, 20, 30, 40)) == pytest.approx((10, 25, 30, 45))


# ---------------------------------------------------------------- Geometrie
def test_ground_distance_roundtrip():
    g = CamGeometry(640, 480, 62.2, 0.0, 45.0, 85.0)
    for d in (0.5, 1.0, 2.0, 3.0):
        y = 240 + g.f * g.cam_h / d
        assert g.ground_dist(y) == pytest.approx(d, rel=1e-6)
    # Nicken nach oben um 1° -> Fußpunkt wandert nach unten; mit pitch_dev korrigiert
    y = 240 + g.f * math.tan(math.atan(g.cam_h / 1.5) + math.radians(1))
    assert g.ground_dist(y, math.radians(1)) == pytest.approx(1.5, rel=1e-3)
    assert g.ground_dist(200) is None


def test_object_dist_ignores_clipped_edges():
    g = CamGeometry(640, 480, 62.2, 0.0, 45.0, 85.0)
    d = 1.5
    y2 = 240 + g.f * g.cam_h / d
    w = g.f * 0.45 / d
    box = (320 - w / 2, 0, 320 + w / 2, y2)           # Kopf oben abgeschnitten
    est, how = g.object_dist(box, "person", 0.0, (False, True, False, False))
    assert "hoehe" not in how
    assert est == pytest.approx(d, rel=0.05)


# ---------------------------------------------------------------- Tracker
def test_tracker_follows_shift_and_keeps_id():
    tr = Tracker(Config().tracker, (640, 480))
    tr.update([Detection((100, 100, 200, 300), "person", 0.9)], now=0.0)
    tid = tr.tracks[0].id
    A = np.eye(3)
    A[0, 2] = 20  # Kamera dreht -> Bild verschiebt sich 20 px nach rechts
    for k in range(5):
        tr.predict(A, 0.05)
    b = tr.tracks[0].box
    assert b[0] == pytest.approx(200, abs=2)
    tr.update([Detection((201, 100, 301, 300), "person", 0.9)], now=0.25)
    assert len(tr.tracks) == 1 and tr.tracks[0].id == tid and tr.tracks[0].hits == 2


def test_tracker_prunes_lost():
    cfg = Config().tracker
    tr = Tracker(cfg, (640, 480))
    tr.update([Detection((100, 100, 200, 300), "person", 0.9)], now=0.0)
    tr.prune(cfg.max_lost_s + 0.1)
    assert tr.tracks == []


# ---------------------------------------------------------------- Detektor-Postprocessing
def _fake_yolo(conf=0.35):
    y = YoloOnnx.__new__(YoloOnnx)
    from xgonav.detector import COCO
    y.cfg = DetectorCfg(conf=conf)
    y.names = COCO
    y.allowed = None
    return y


def test_yolo_v8_postprocess_letterbox_and_nms():
    y = _fake_yolo()
    nc = 80
    out = np.zeros((1, 4 + nc, 3), np.float32)
    # zwei überlappende Personen-Boxen + ein Stuhl, in 320er-Letterbox-Koordinaten
    out[0, :4, 0] = [160, 160, 50, 100]; out[0, 4 + 0, 0] = 0.9
    out[0, :4, 1] = [162, 161, 50, 100]; out[0, 4 + 0, 1] = 0.6
    out[0, :4, 2] = [60, 200, 40, 40];   out[0, 4 + 56, 2] = 0.8
    # 640x480 -> r = 0.5, pad y = 40
    dets = y._post(out, 0.5, 0, 40, 640, 480)
    cls = sorted(d.cls for d in dets)
    assert cls == ["chair", "person"]
    p = next(d for d in dets if d.cls == "person")
    assert p.box == pytest.approx((270, 140, 370, 340), abs=1)


# ---------------------------------------------------------------- IMU-Synchronisation
def _sync_scenario(sign, off_ms, noise_deg=0.05, still=False):
    from xgonav.config import ImuCfg
    from xgonav.imu import ImuSource, ImuSync
    cfg = ImuCfg()
    imu = ImuSource(cfg)
    sync = ImuSync(cfg)
    rng = np.random.default_rng(0)
    pitch = (lambda t: 0.0) if still else (
        lambda t: math.radians(3 * math.sin(2 * math.pi * 2.5 * t) + 1.5 * math.sin(2 * math.pi * 6.1 * t + 1)))
    ref = math.radians(2.0)
    for k in range(1, 1000):                   # IMU 100 Hz, Stempel um off_ms verschoben
        t = k * 0.01
        imu.add(k, t + off_ms / 1000, 0.0, sign * pitch(t) + ref + math.radians(noise_deg) * rng.standard_normal())
    for k in range(20, 190):                   # Kamera 20 Hz
        t0, t1 = (k - 1) * 0.05, k * 0.05
        sync.add(t0, t1, pitch(t1) - pitch(t0))
    sync.update(imu, now=100.0)
    return sync, imu, pitch


def test_imu_sync_finds_sign_and_offset():
    for sign, off in ((1, 0), (-1, 12), (1, -36)):
        s, imu, pitch = _sync_scenario(sign, off)
        assert s.reliable, (sign, off, s.corr)
        assert s.sign == sign
        assert abs(s.offset * 1000 - off) <= 4
        s.cfg.pitch_ref_deg = 2.0
        s.cfg.max_age_s = 1e9                  # synthetische Zeitbasis
        for t in (3.0, 5.0, 7.2):
            assert math.degrees(abs(s.pitch_dev(imu, t) - pitch(t))) < 0.3


def test_imu_sync_needs_motion():
    s, _, _ = _sync_scenario(1, 0, still=True)
    assert not s.reliable


# ---------------------------------------------------------------- App
def _app(freespace: bool, mode: str = "idle"):
    from xgonav.app import App
    cfg = Config()
    cfg.freespace.enabled = freespace
    cfg.behavior.mode = mode
    cfg.output.web_port = 0
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None)
    return App(cfg, robot, world, imu=None)


def test_modes_need_freespace():
    """Ohne Bodenmodell hielten Erkunden/Anfahren alles für frei -> gesperrt."""
    from xgonav.app import ModeNotAllowed
    app = _app(freespace=False)
    for name in ("explore", "approach"):
        with pytest.raises(ModeNotAllowed):
            app.set_mode(name)
    app.set_mode("follow")      # Folgen und Idle bleiben erlaubt
    app.set_mode("idle")
    with pytest.raises(KeyError):
        app.set_mode("gibtsnicht")
    # Start mit gesperrtem Modus -> bleibt im Idle statt blind zu fahren
    assert _app(freespace=False, mode="explore").mode == "idle"
    assert _app(freespace=True, mode="explore").mode == "explore"


# ---------------------------------------------------------------- Linsenabschattung
def test_shading_fit_recovers_radial_falloff():
    from xgonav.shading import ShadingModel
    H, W = 240, 320
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    s = ((xx - W / 2) ** 2 + (yy - H / 2) ** 2) / (W / 2) ** 2 / 1.5625   # (r / halbe Diagonale)^2
    true = np.stack([1 - 0.8 * s, 1 - 0.75 * s, 1 - 2.5 * s + 1.6 * s * s], -1).clip(0.01, 1)
    flat = (200 * true).astype(np.float32)
    m = ShadingModel.fit(flat, np.ones((H, W), bool))
    F = m.falloff(W, H)
    assert np.abs(F - true)[true > 0.05].max() < 0.03
    gain, rel = m.gain_and_reliable(W, H, 8.0)
    assert rel[H // 2, W // 2] and not rel[0, 0]          # Mitte messbar, Ecke nicht
    assert gain.max() <= 8.0 + 1e-6


def test_localmap_unknown_columns_do_not_erase():
    """Spalten ohne messbare Farbe (known_from) dürfen bekannte Hindernisse nicht löschen."""
    from xgonav.freespace import FreeSpace
    from xgonav.localmap import LocalMap
    geo = CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0)
    fw, fh = 160, 120
    horizon = fh / 2
    free_cols = np.ones(fw)                               # alles "frei bis zum Horizont"

    def fresh():
        lm = LocalMap(geo, fw, fh)
        lm.pts = np.array([[0.30, 0.0], [0.80, 0.0], [0.40, 0.30]])   # vorn nah, vorn fern, links
        lm.t = np.zeros(3)
        lm.obs = np.arange(3)
        return lm

    # ohne known_from: alles im Sichtfeld wird freigeräumt
    lm = fresh(); lm.add_freespace(FreeSpace([1.0] * 5, free_cols, horizon, None, True), 0.0, 0.0)
    assert len(lm.pts) == 0
    # Mitte erst ab 0,5 m messbar, linke Hälfte gar nicht: nur der ferne Punkt vorn verschwindet
    span = fh - horizon
    row_05 = fh - next(r for r in range(fh, 0, -1) if (geo.ground_dist(r * 4) or 9) >= 0.5) / 1
    kf = np.full(fw, row_05 / span)
    kf[: fw // 2 - 10] = 1.0
    lm = fresh(); lm.add_freespace(FreeSpace([1.0] * 5, free_cols, horizon, None, True, kf), 0.0, 0.0)
    left = {tuple(p) for p in lm.pts.round(2)}
    assert (0.30, 0.0) in left and (0.40, 0.30) in left and (0.80, 0.0) not in left


def test_localmap_confirm_ignores_single_observations():
    """Mit confirm=3 blockiert ein einmal gesehener Punkt die Bahn nicht, ein mehrfach gesehener schon."""
    from xgonav.localmap import LocalMap
    geo = CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0)
    lm = LocalMap(geo, 160, 120, confirm=3, confirm_radius_m=0.08)
    lm.pts = np.array([[0.25, 0.0]]); lm.t = np.zeros(1); lm.obs = np.array([0])
    assert lm.arc_clearance(0.15, 0.0, 1.2) == math.inf            # nur einmal gesehen
    lm.pts = np.array([[0.25, 0.0], [0.27, 0.01], [0.24, -0.02]]); lm.t = np.zeros(3); lm.obs = np.array([0, 1, 2])
    assert lm.arc_clearance(0.15, 0.0, 1.2) < 0.2                   # in drei Bildern gesehen
    lm = LocalMap(geo, 160, 120, confirm=3, confirm_radius_m=0.08)   # drei Punkte, aber ein Bild
    lm.pts = np.array([[0.25, 0.0], [0.27, 0.01], [0.24, -0.02]]); lm.t = np.zeros(3); lm.obs = np.array([5, 5, 5])
    assert lm.arc_clearance(0.15, 0.0, 1.2) == math.inf


def test_freespace_flat_spot_vs_upright_object():
    """Flacher Fleck nah vorn (Fuge, Astloch) ist kein Hindernis, aufrechter Gegenstand schon."""
    from xgonav.freespace import FreeSpaceEstimator
    geo = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    est = FreeSpaceEstimator(Config().freespace, (640, 480), geo)
    floor = np.full((120, 160), 255, np.uint8)
    floor[106:114, 70:90] = 0          # 8 Zeilen, darüber wieder Boden
    floor[60:100, 100:110] = 0         # steht auf, reicht über den Horizont
    c = est._column_free(floor, 62.5)
    assert c[75:85].min() == 1.0
    assert c[102:108].max() < 0.5


def test_localmap_confirm_counts_on_insert():
    """Bestätigung über echte Beobachtungen: derselbe Fleck dreimal gesehen -> blockiert."""
    from xgonav.freespace import FreeSpace
    from xgonav.localmap import LocalMap
    geo = CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0)
    lm = LocalMap(geo, 160, 120, confirm=3)
    cols = np.ones(160); cols[76:84] = 0.15           # Grenze ~0,3 m geradeaus
    fs = FreeSpace([1.0] * 5, cols, 60.0, None, True)
    for i in range(2):
        lm.add_freespace(fs, i * 0.1, 0.0)
        assert lm.arc_clearance(0.15, 0.0, 1.2) == math.inf, "nach %d Beobachtungen schon bestätigt" % (i + 1)
    lm.move(0.0, 0.05)                                 # starre Bewegung ändert nichts an der Zählung
    lm.add_freespace(fs, 0.3, 0.0)
    assert lm.arc_clearance(0.15, 0.0, 1.2) < math.inf


def test_freespace_missing_shading_file_is_tolerated(tmp_path):
    """Frische Installation ohne shading.json: Warnung statt Absturz, ohne Korrektur."""
    from xgonav.freespace import FreeSpaceEstimator
    cfg = Config()
    cfg.freespace.shading_file = str(tmp_path / "gibtsnicht.json")
    est = FreeSpaceEstimator(cfg.freespace, (640, 480), CamGeometry(640, 480, 91.0))
    assert est._sgain is None and est._srel is None


def test_pipeline_pause_and_resume():
    """Bildverarbeitung stoppen: keine Frames mehr, Stillstand, Modus idle; starten: läuft weiter."""
    import threading
    from xgonav.app import App
    from xgonav.sim import Person
    cfg = Config(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, Person([(3.0, 1.5), (3.5, 3.0)], speed=0.3), start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None)
    app.imu = None
    app.set_mode("follow")
    th = threading.Thread(target=app.run, kwargs={"duration": 6.0}, daemon=True)
    th.start()
    time.sleep(1.5)
    assert app.mode == "follow" and app.pipeline_on
    app.set_pipeline(False)
    time.sleep(0.5)
    idx = app.snapshot.idx
    time.sleep(1.0)
    assert app.snapshot.idx == idx                      # keine neuen Bilder verarbeitet
    assert app.mode == "idle" and robot.command == (0.0, 0.0)
    app.set_pipeline(True)
    time.sleep(1.0)
    assert app.snapshot.idx > idx and app.mode == "idle"   # läuft wieder, fährt aber nicht los
    app.stop(); th.join(timeout=5)


def test_manual_drive_and_deadman():
    """Handsteuerung: nur im Modus manual; ohne neue Sollwerte nach 0,5 s Stillstand."""
    import threading
    from xgonav.app import App, ModeNotAllowed
    cfg = Config(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None)
    app.imu = None
    th = threading.Thread(target=app.run, kwargs={"duration": 5.0}, daemon=True)
    th.start()
    time.sleep(1.0)
    with pytest.raises(ModeNotAllowed):
        app.set_manual(0.1, 0.0)                         # noch Modus idle
    app.set_mode("manual")
    time.sleep(0.3)
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:                      # Taste gedrückt halten
        app.set_manual(1.0, 0.0)
        time.sleep(0.1)
    assert robot.command[0] == pytest.approx(cfg.robot.max_v, abs=1e-6)   # auf max_v begrenzt
    time.sleep(1.0)                                      # Taste los, keine Sollwerte mehr
    assert robot.command == (0.0, 0.0)
    app.stop(); th.join(timeout=5)


def test_manual_without_pipeline():
    """Handsteuerung bei gestoppter Bildverarbeitung: regelt ohne Kamera, Totmann greift,
    kamerabasierte Modi bleiben gesperrt."""
    import threading
    from xgonav.app import App, ModeNotAllowed
    cfg = Config(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None)
    app.imu = None
    th = threading.Thread(target=app.run, kwargs={"duration": 6.0}, daemon=True)
    th.start()
    time.sleep(1.0)
    app.set_pipeline(False)
    time.sleep(0.3)
    with pytest.raises(ModeNotAllowed):
        app.set_mode("follow")                           # braucht Kamera
    app.set_mode("manual")
    time.sleep(0.3)
    assert app.mode == "manual"
    idx = app.snapshot.idx
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        app.set_manual(0.0, -1.0)                        # Rechtsdrehung
        time.sleep(0.1)
    assert robot.command[1] == pytest.approx(-cfg.robot.max_w, abs=1e-6)
    assert "ohne Bildverarbeitung" in app._cmd.status
    assert app.snapshot.idx == idx                       # Kamera bleibt aus
    time.sleep(1.0)
    assert robot.command == (0.0, 0.0)                   # Totmann
    app.set_pipeline(True)
    time.sleep(0.5)
    assert app.mode == "manual" and app.snapshot.idx > idx
    app.stop(); th.join(timeout=5)


def test_reverse_gain():
    """Rückwärts wird beim Senden mit reverse_gain verstärkt, vorwärts nicht."""
    cfg = Config(); cfg.robot.reverse_gain = 2.0; cfg.robot.max_v = 0.15
    r = SimRobot(cfg.robot)
    r._cur = (-0.15, 0.0); r._send(time.monotonic() + 10)
    assert r._sent[0] == 128 - round(0.30 * 127)          # 90
    r._cur = (0.15, 0.0); r._send(time.monotonic() + 20)
    assert r._sent[0] == 128 + round(0.15 * 127)          # 147


def test_emotion_trigger():
    from xgonav.config import EmotionCfg
    from xgonav.emotion import EmotionTrigger
    played, busy = [], [False]
    tr = EmotionTrigger(EmotionCfg(gap_s=5, lost_s=2, battery_low=20, battery_repeat_s=300),
                        played.append, lambda: busy[0])
    # Person erscheint -> surprised, bleibt sie, nichts mehr
    assert tr.update(0.0, "idle", True, 80) == "surprised"
    assert tr.update(1.0, "idle", True, 80) is None
    # kurz weg (< gap_s) und wieder da: kein neues surprised
    tr.update(2.0, "idle", False, 80)
    assert tr.update(3.0, "idle", True, 80) is None
    # follow ohne Person: doubt erst lost_s nach dem Moduswechsel, nur einmal
    assert tr.update(4.0, "follow", False, 80) is None
    assert tr.update(5.5, "follow", False, 80) is None
    assert tr.update(6.1, "follow", False, 80) == "doubt"
    assert tr.update(7.0, "follow", False, 80) is None
    # nach > gap_s ohne Person: surprised; läuft gerade etwas, wird gewartet
    busy[0] = True
    assert tr.update(9.0, "follow", True, 80) is None
    busy[0] = False
    assert tr.update(9.1, "follow", True, 80) == "surprised"
    # Akku: shy, dann erst nach battery_repeat_s wieder; 0/None = unbekannt
    assert tr.update(10.0, "idle", None, 19) == "shy"
    assert tr.update(11.0, "idle", None, 19) is None
    assert tr.update(311.0, "idle", None, 19) == "shy"
    assert tr.update(700.0, "idle", None, 0) is None
    # anderer Modus ohne Person: embarrassed nach lost_s, einmal je Verlust; Person
    # zurück (< gap_s, kein surprised) macht es wieder scharf
    assert tr.update(800.0, "idle", True, 80) == "surprised"
    assert tr.update(801.0, "idle", False, 80) is None
    assert tr.update(802.1, "idle", False, 80) == "embarrassed"
    assert tr.update(803.0, "idle", False, 80) is None
    assert tr.update(804.0, "idle", True, 80) is None
    assert tr.update(806.1, "idle", False, 80) == "embarrassed"
    # Bildverarbeitung gestoppt (person=None): keine Personen-Emotionen
    assert tr.update(900.0, "idle", None, 80) is None
    assert played == ["surprised", "doubt", "surprised", "shy", "shy", "surprised", "embarrassed", "embarrassed"]


def test_exposure_runtime_limits():
    from xgonav.camera import FrameSource
    from xgonav.config import CameraCfg
    src = FrameSource(CameraCfg(fps=30, exposure_us=15000))
    assert src.exposure_max_us == 33333
    assert src.set_exposure(8000) == 8000 and src.exposure_us == 8000
    assert src.set_exposure(0) == 0                     # Automatik
    for bad in (50, 40000, -1):
        with pytest.raises(ValueError):
            src.set_exposure(bad)
    assert src.exposure_us == 0                         # ungültige Werte ändern nichts


def test_gain_runtime_limits():
    from xgonav.camera import FrameSource
    from xgonav.config import CameraCfg
    src = FrameSource(CameraCfg())
    assert src.gain == 0.0                              # Default: Automatik
    assert src.set_gain(4) == 4.0 and src.gain == 4.0
    assert src.set_gain(16) == 16.0
    for bad in (0.5, 17, -1):
        with pytest.raises(ValueError):
            src.set_gain(bad)
    assert src.set_gain(0) == 0.0


def test_websocket_handshake_and_frames():
    import io
    import struct
    from xgonav.web import ws_accept, ws_frame, ws_read
    assert ws_accept("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="   # RFC 6455, 1.3
    for n in (5, 300, 70000):                                  # 7-, 16- und 64-Bit-Länge
        f = ws_frame(0x2, b"x" * n)
        assert f[0] == 0x82 and f.endswith(b"x" * n)
    # maskierte Client-Nachricht "ok"
    mask = b"\x01\x02\x03\x04"
    data = bytes(c ^ mask[i % 4] for i, c in enumerate(b"ok"))
    assert ws_read(io.BytesIO(struct.pack("!BB", 0x81, 0x80 | 2) + mask + data)) == (0x1, b"ok")
    assert ws_read(io.BytesIO(b"")) is None

