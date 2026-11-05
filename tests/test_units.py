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


def _kanten_est(**kw):
    from xgonav.freespace import FreeSpaceEstimator
    cfg = Config()
    cfg.freespace.method = "kanten"
    cfg.freespace.shading_file = ""
    for k, v in kw.items():
        setattr(cfg.freespace, k, v)
    geo = CamGeometry(640, 480, 88.5, 5.18, 43.0, 85.0)      # Horizont bei Zeile ~270
    return FreeSpaceEstimator(cfg.freespace, (640, 480), geo), geo


def test_kanten_glare_is_unknown_but_lit_wall_at_border_stays():
    """Lampenschein auf glänzendem Boden: unbekannt (NaN) statt Hindernis weit vor der Lampe.
    Gleich aussehender heller Streifen am Bildrand (angestrahlte Wand): bleibt Hindernis."""
    img = np.full((480, 640, 3), 90, np.uint8)
    img[:270] = 40                                            # dunkle Wand über dem Horizont
    img[200:268, 300:340] = 255                               # Lampe (übersteuert)
    rows = np.arange(270, 330)
    img[270:330, 296:344] = np.linspace(250, 170, len(rows)).astype(np.uint8)[:, None, None]
    img[200:268, 0:24] = 255                                  # dasselbe am linken Rand
    img[270:330, 0:28] = np.linspace(250, 170, len(rows)).astype(np.uint8)[:, None, None]
    est, _ = _kanten_est()
    fs = est.update(img, 0.0)
    c = fs.columns
    assert np.isnan(c[78:83]).all()                           # Mitte des Scheins
    assert np.isfinite(c[0:4]).all() and c[0:4].max() < 0.9  # Rand: Hindernis
    assert np.isfinite(c[20:60]).all() and np.isfinite(c[100:160]).all()
    est2, _ = _kanten_est(edge_glare=False)
    c2 = est2.update(img, 0.0).columns
    assert np.isfinite(c2).all() and c2[78:83].max() < 0.9   # ohne Regel: Hindernis


def test_kanten_log_finds_dark_wall_in_dark_scene():
    """Dunkle Szene: Boden 25, Wand 11 (Grauwerte). Linear ist die Stufe zu klein, als
    Helligkeitsverhältnis (edge_log_s) wird die Wand erkannt."""
    img = np.full((480, 640, 3), 25, np.uint8)
    img[:330] = 11                                            # Wand, Fuß in Zeile 330
    img[:270] = 9
    lin, _ = _kanten_est(edge_log_s=0.0)
    log, _ = _kanten_est(edge_log_s=16.0)
    assert lin.update(img, 0.0).columns[20:140].min() >= 0.98
    c = log.update(img, 0.0).columns
    assert c[20:140].max() < 0.9


def test_arc_clearance_escape_from_point_inside_radius():
    """Punkt schon im Sicherheitsradius (neben dem Rider): Wegfahren geht, Hinfahren nicht."""
    from xgonav.localmap import LocalMap
    lm = LocalMap(CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0), 160, 120, confirm=1)
    lm.pts = np.array([[0.0, 0.15], [-0.05, 0.15]]); lm.t = np.zeros(2); lm.obs = np.array([1, 2])
    assert lm.arc_clearance(0.15, -0.3, 1.2) == math.inf          # nach rechts weg
    assert lm.arc_clearance(0.15, 0.0, 1.2) == math.inf           # geradeaus: Abstand wächst
    assert lm.arc_clearance(0.15, 1.5, 1.2) < 0.1                 # scharf nach links: hin


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


def test_emotion_sounds():
    """Geräusche: freudig beim Auftauchen, traurig nach lost_s; unabhängig vom Display."""
    from xgonav.config import EmotionCfg
    from xgonav.emotion import EmotionTrigger
    snd, busy = [], [True]                      # Display belegt: Geräusche kommen trotzdem
    tr = EmotionTrigger(EmotionCfg(gap_s=5, lost_s=2), lambda n: None, lambda: busy[0], snd.append)
    tr.update(0.0, "follow", True, 80)          # Person da -> gefunden
    tr.update(0.5, "follow", True, 80)
    tr.update(1.0, "follow", False, 80)         # kurz weg (< lost_s): nichts
    tr.update(1.5, "follow", True, 80)
    tr.update(2.0, "follow", False, 80)
    tr.update(3.6, "follow", False, 80)         # 2,1 s weg -> verloren
    tr.update(5.0, "follow", False, 80)         # nur einmal
    tr.update(6.0, "follow", None, 80)          # Bildverarbeitung aus: nichts
    tr.update(7.0, "idle", True, 80)            # wieder da -> gefunden
    assert snd == ["gefunden", "verloren", "gefunden"]


def test_sounds_tool(tmp_path):
    import importlib.util, wave
    spec = importlib.util.spec_from_file_location("sounds", Path(__file__).resolve().parents[1] / "tools" / "sounds.py")
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    for name, fn in (("freude.wav", m.freude), ("trauer.wav", m.trauer)):
        p = str(tmp_path / name)
        m.write(p, fn())
        with wave.open(p) as w:
            assert w.getframerate() == m.RATE and w.getnchannels() == 1
            assert 0.3 < w.getnframes() / m.RATE < 1.5


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


def test_detector_max_hz_limits_rate():
    from xgonav.camera import Frame
    from xgonav.detector import DetectorWorker
    from xgonav.config import DetectorCfg
    w = DetectorWorker(DetectorCfg(max_hz=5.0, max_wait_s=0.0))
    calls = []

    class M:
        def __call__(self, bgr):
            calls.append(time.monotonic())
            return []
    w.model = M()
    import threading
    w._thread = threading.Thread(target=w._run, daemon=True)
    w._thread.start()
    f = Frame(0, 0.0, np.zeros((4, 4, 3), np.uint8))
    t_end = time.monotonic() + 1.05
    while time.monotonic() < t_end:            # Kamera bietet laufend Bilder an
        w.offer(f, 1.0, 1.0)
        time.sleep(0.01)
    w.close()
    assert 4 <= len(calls) <= 6                # ~5 Hz statt ~100 Hz
    assert min(b - a for a, b in zip(calls, calls[1:])) >= 0.19


def test_cpu_pinning_helpers(tmp_path):
    import os
    from xgonav.affinity import ort_pool_affinities, pin_thread
    assert pin_thread([], "x") == set()                         # leer = nichts tun
    assert ort_pool_affinities([2, 3], 2) == "4"                # Aufrufer 2, Pool-Thread Kern 3 (1-basiert)
    assert ort_pool_affinities([2, 3], 3) == "4;4"
    assert ort_pool_affinities([1, 2, 3], 3) == "3;4"
    assert ort_pool_affinities([2], 2) == "3"                   # nur ein Kern: Pool teilt ihn
    assert ort_pool_affinities([], 2) == "" and ort_pool_affinities([2, 3], 1) == ""
    if hasattr(os, "sched_setaffinity"):
        import threading
        res = {}
        before = os.sched_getaffinity(0)

        def worker():                                           # nur dieser Thread wird gepinnt
            res["set"] = pin_thread([0, 9999], "Test")          # 9999 gibt es nicht: verworfen
            res["now"] = os.sched_getaffinity(0)
        t = threading.Thread(target=worker)
        t.start(); t.join()
        assert res["set"] == {0} and res["now"] == {0}
        assert os.sched_getaffinity(0) == before                 # aufrufender Thread unverändert
    p = tmp_path / "c.toml"
    p.write_text("[cpu]\nmain_cpus = [0, 1]\ndetector_cpus = [2, 3]\nrender_cpus = [0, 1, 2, 3]\n")
    c = Config.load(p)
    assert c.cpu.main_cpus == [0, 1] and c.cpu.detector_cpus == [2, 3] and c.cpu.render_cpus == [0, 1, 2, 3]


def test_freespace_worker_matches_direct_update():
    from xgonav.config import FreespaceCfg
    from xgonav.freespace import FreeSpaceEstimator, FreeSpaceWorker
    rng = np.random.default_rng(3)
    imgs = []
    for _ in range(4):                                      # Boden unten grau, oben "Wand"
        im = np.full((480, 640, 3), 120, np.uint8)
        im[:200] = (30, 90, 160)
        imgs.append(np.clip(im + rng.integers(-6, 7, im.shape), 0, 255).astype(np.uint8))
    direct = FreeSpaceEstimator(FreespaceCfg(), (640, 480))
    w = FreeSpaceWorker(FreeSpaceEstimator(FreespaceCfg(), (640, 480)))
    w.start()
    try:
        assert w.poll() is None
        for im in imgs:
            ref = direct.update(im, 0.0, [])
            w.offer(im, 0.0, [])
            t_end = time.monotonic() + 5
            got = None
            while got is None and time.monotonic() < t_end:
                got = w.poll()
                time.sleep(0.005)
            assert got is not None
            assert got.sectors == ref.sectors and np.array_equal(got.columns, ref.columns)
            assert w.poll() is None                         # Ergebnis nur einmal
    finally:
        w.close()
    assert not w._thread.is_alive()


def test_wheel_odometry_wrap_reset_and_signs():
    import math
    from xgonav.config import OdomCfg
    from xgonav.wheels import WheelOdom
    cfg = OdomCfg()
    w = WheelOdom(cfg)
    mpt = math.pi * 0.060 / 1024
    w.add(1, 0.00, 2**31 - 100, -50, 1000)              # kurz vor dem int32-Überlauf
    a = w.latest()
    w.add(2, 0.02, -(2**31) + 100, 150, 1020)           # +200 Ticks über den Überlauf, rechts +200
    b = w.latest()
    ds, dth = w.motion(a, b)
    assert abs(ds - 200 * mpt) < 1e-9 and abs(dth) < 1e-9    # geradeaus vorwärts
    w.add(3, 0.04, -(2**31) + 100, 250, 1040)           # nur rechts +100 -> Linksdrehung
    ds, dth = w.motion(b, w.latest())
    assert abs(ds - 50 * mpt) < 1e-9 and dth > 0 and abs(dth - 100 * mpt / 0.090) < 1e-9
    c = w.latest()
    w.add(4, 0.06, 5, 7, 30)                             # ESP32-Neustart: fw_ms springt zurück
    assert w.resets == 1 and w.motion(c, w.latest()) == (0.0, 0.0)
    w.add(5, 0.08, 5 + 3000, 7, 50)                      # unplausibler Sprung > max_jump_ticks
    assert w.resets == 2
    w.add(5, 0.10, 9999, 9999, 70)                       # gleiche seq: ignoriert
    assert w.latest()[0] == 0.08
    assert w.fresh(0.3) and not w.fresh(0.5)


def test_wheel_scale_calibration():
    import math
    from xgonav.config import OdomCfg
    from xgonav.wheels import WheelOdom, calibrate_scales
    # Fahrt wie am Rider: links 30,3 cm, rechts 28,7 cm gemessen, wahr 29,5 cm und -5,3°
    sl, sr, th, track = 0.303, 0.287, math.radians(-5.3), 0.090
    kl, kr = calibrate_scales(sl, sr, th, track, 0.295)
    assert abs((kl * sl + kr * sr) / 2 - 0.295) < 1e-12
    assert abs((kr * sr - kl * sl) / track - th) < 1e-12
    assert 0.97 < kl < 1.0 < kr < 1.03                      # links misst zu viel
    kl2, kr2 = calibrate_scales(sl, sr, th, track)           # ohne Maßband: Mittel bleibt
    assert abs((kl2 * sl + kr2 * sr) / 2 - (sl + sr) / 2) < 1e-12
    # Faktoren wirken in WheelOdom
    w = WheelOdom(OdomCfg(scale_left=kl, scale_right=kr))
    mpt = w.m_per_tick
    w.add(1, 0.0, 0, 0); a = w.latest()
    w.add(2, 0.1, round(sl / mpt), round(sr / mpt)); b = w.latest()
    ds, dth = w.motion(a, b)
    assert abs(ds - 0.295) < 0.001 and abs(dth - th) < math.radians(0.3)


def test_odom_calibration_segments():
    import math
    from xgonav.wheels import calibrate_odom
    kl, kr, track = 0.99, 1.01, 0.160                       # Wahrheit
    def seg(dl, dr):                                         # wahre Radwege -> (gemessen, Drehung)
        return (dl / kl, dr / kr, (dr - dl) / track)
    segs = [seg(0.40, 0.41), seg(-0.38, -0.40),              # hin und zurück, leicht krumm
            seg(-0.5, 0.5), seg(0.49, -0.5)]                  # links und rechts auf der Stelle
    a, b, t = calibrate_odom(segs, 0.090, mean_scale=1.0)
    assert abs(a - kl) < 1e-6 and abs(b - kr) < 1e-6 and abs(t - track) < 1e-6
    # Maßband statt Mittelwert
    a, b, t = calibrate_odom(segs, 0.090, mean_scale=0.5, tape=(0, 0.405))
    assert abs(a - kl) < 1e-6 and abs(b - kr) < 1e-6 and abs(t - track) < 1e-6
    # nur Fahrten: Spurweite fest, Unterschied der Räder trotzdem bestimmbar
    a, b, t = calibrate_odom(segs[:2], track, mean_scale=1.0, fit_track=False)
    assert abs(a - kl) < 1e-6 and abs(b - kr) < 1e-6 and t == track
    try:                                                     # nur Fahrten, Spurweite frei: unterbestimmt
        calibrate_odom([seg(0.4, 0.4), seg(-0.4, -0.4)], 0.09)
        assert False
    except ValueError:
        pass
    assert math.isfinite(t)


def test_gridmap_wall_and_turn():
    import math
    from xgonav.config import Config
    from xgonav.freespace import FreeSpace
    from xgonav.geometry import CamGeometry
    from xgonav.gridmap import GridMap
    cfg = Config()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    m = GridMap(cfg.grid, g, 160, 120)
    hz = g.horizon_row(0) / 4
    ys = np.arange(int(hz * 4) + 1, 480)
    y1 = ys[np.nanargmin(np.abs(g.ground_dist_np(ys) - 1.0))]
    col = (120 - y1 / 4) / (120 - hz)
    fs = FreeSpace([col] * 5, np.full(160, col), hz, np.zeros((120, 160), np.uint8), True, None)
    for _ in range(3):
        m.add_freespace(fs, 0.0)
    occ = m.occupied_cells()
    assert np.all(np.abs(occ[:, 0] - 1.0) < 0.06)              # gerade Wand 1 m vor dem Start
    assert occ[:, 1].max() > 0.9 and occ[:, 1].min() < -0.9     # über die Bildbreite (91°)
    m.move(0.0, math.radians(90))                               # links drehen: Wand bei y = +1
    for _ in range(3):
        m.add_freespace(fs, 0.0)
    new = m.occupied_cells()
    assert (np.abs(new[:, 1] - 1.0) < 0.06).sum() > 30
    assert m.stats()["frei_m2"] > 1.0
    img = m.image(200)
    assert img.shape == (200, 200, 3)
    m.reset()
    assert len(m.occupied_cells()) == 0 and m.stats()["pose"] == [0.0, 0.0, 0.0]


def test_gridworker_uses_pose_of_observation():
    import math
    from xgonav.config import Config
    from xgonav.freespace import FreeSpace
    from xgonav.geometry import CamGeometry
    from xgonav.gridmap import GridMap, GridWorker
    cfg = Config()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    hz = g.horizon_row(0) / 4
    ys = np.arange(int(hz * 4) + 1, 480)
    y1 = ys[np.nanargmin(np.abs(g.ground_dist_np(ys) - 1.0))]
    col = (120 - y1 / 4) / (120 - hz)
    fs = FreeSpace([col] * 5, np.full(160, col), hz, np.zeros((120, 160), np.uint8), True, None)
    direct = GridMap(cfg.grid, g, 160, 120)
    m = GridMap(cfg.grid, g, 160, 120)
    w = GridWorker(m)
    for _ in range(3):
        direct.add_freespace(fs, 0.0)
        w.offer(fs, 0.0)
    m.move(2.0, math.radians(90))          # Pose ändert sich, bevor der Worker rechnet
    w.start()
    t_end = time.monotonic() + 5
    while not w.idle() and time.monotonic() < t_end:
        time.sleep(0.01)
    time.sleep(0.05)
    w.close()
    # Wand dort, wo sie aus der Startpose gesehen wurde – nicht verschoben zur neuen Pose
    assert np.array_equal(np.sort(m.occupied_cells(), axis=0), np.sort(direct.occupied_cells(), axis=0))


# ---------------------------------------------------------------- Startpunkt, Speichern, Heimfahrt
def _grid():
    from xgonav.gridmap import GridMap
    cfg = Config()
    return cfg, GridMap(cfg.grid, CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)


def test_gridmap_start_save_load(tmp_path):
    cfg, m = _grid()
    m.lo[100:110, 50:60] = 2.5
    m.move(1.0, 0.0)
    m.move(0.0, math.radians(30))
    m.set_start()
    m.move(0.5, math.radians(45))
    st = m.stats()
    assert st["start"][:2] == pytest.approx([1.0, 0.0], abs=1e-6) and st["zum_start_m"] > 0.3
    f = str(tmp_path / "karte.npz")
    assert m.save(f)
    _, m2 = _grid()
    assert m2.load(f)
    assert np.allclose(m2.lo, m.lo, atol=0.01)
    assert m2.pose() == pytest.approx(m.start)                 # Pose = Startpunkt
    assert m2.stats()["zum_start_m"] == 0.0
    cfg.grid.resolution_m = 0.1                                # andere Auflösung -> ignoriert
    from xgonav.gridmap import GridMap
    m3 = GridMap(cfg.grid, m.geo, 160, 120)
    assert not m3.load(f) and not m3.lo.any()
    m2.maybe_save(f, 0.0)                                      # unverändert -> kein Schreiben nötig
    assert not (tmp_path / "karte.npz.tmp").exists()


def test_planner_avoids_wall_and_reports_no_path():
    from xgonav.planner import cost_map, plan
    cfg, m = _grid()
    hc = cfg.home
    # Wand bei x = 1 m von y = -1,5 .. +0,5 m; Ziel dahinter
    r0, c = m.cell(1.0, -1.5)                                  # Zeile = y, Spalte = x
    r1, _ = m.cell(1.0, 0.5)
    m.lo[r0:r1 + 1, c] = 3.0
    p = plan(m, (0.0, 0.0), (2.0, 0.0), hc)
    assert p is not None and p[0] == (0.0, 0.0) and p[-1] == (2.0, 0.0)
    assert max(y for _, y in p) > 0.5                          # oben um das Wandende herum
    blocked, _ = cost_map(m.lo, m.res, cfg.grid.l_show, hc)
    for (ax, ay), (bx, by) in zip(p, p[1:]):                   # kein Abschnitt durch gesperrte Zellen
        for t in np.linspace(0, 1, 50):
            r, cc = m.cell(ax + t * (bx - ax), ay + t * (by - ay))
            near_end = min(math.hypot(ax + t * (bx - ax) - q[0], ay + t * (by - ay) - q[1])
                           for q in ((0.0, 0.0), (2.0, 0.0))) < hc.inflate_m + m.res
            assert near_end or not blocked[r, cc]
    # Ziel ringsum eingemauert -> kein Weg
    gr, gc = m.cell(2.0, 0.0)
    k = 12
    m.lo[gr - k, gc - k:gc + k + 1] = m.lo[gr + k, gc - k:gc + k + 1] = 3.0
    m.lo[gr - k:gr + k + 1, gc - k] = m.lo[gr - k:gr + k + 1, gc + k] = 3.0
    assert plan(m, (0.0, 0.0), (2.0, 0.0), hc) is None


def test_home_behavior_steers_and_arrives():
    from xgonav.behaviors import Home, World
    from xgonav.egomotion import EgoResult
    cfg = Config()
    b = Home(cfg.behavior, cfg.robot, cfg.home)

    def world(pose, t=0.0):
        return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[], free=None,
                     balancing=True, yaw=0.0, cmd_v=0.0, pose=pose, goal=(0.0, 0.0, 0.0))
    assert b.want_plan(0.0) and not b.want_plan(0.1)           # ein Auftrag zur Zeit
    c = b.step(world((1.0, -1.0, math.pi / 2)))
    assert c.v == 0 and "plane" in c.status
    b.set_path([(1.0, -1.0), (0.0, 0.0)])
    c = b.step(world((1.0, -1.0, math.radians(135))))         # Ziel genau voraus -> fahren
    assert c.v > 0 and abs(c.w) < 0.05
    c = b.step(world((1.0, -1.0, math.radians(90))))          # Ziel 45° links
    assert c.w > 0
    c = b.step(world((0.05, 0.0, math.radians(40))))          # angekommen, aber verdreht
    assert c.v == 0 and c.w < 0
    c = b.step(world((0.05, 0.0, 0.0)))
    assert c.v == 0 and c.w == 0 and b.done
    b2 = Home(cfg.behavior, cfg.robot, cfg.home)
    for _ in range(cfg.home.max_fail):
        b2.want_plan(0.0)
        b2.set_path(None)
    assert b2.gave_up and b2.step(world((1.0, 1.0, 0.0))).v == 0


def test_home_needs_gridmap():
    from xgonav.app import ModeNotAllowed
    app = _app(freespace=True)
    app.set_mode("home")
    app.gridmap = None
    with pytest.raises(ModeNotAllowed):
        app.set_mode("home")


def test_freespace_reset_and_dark_sample():
    """Dunkle Probe (Kamerastart) darf das Modell nicht verderben; reset() lernt neu."""
    import numpy as np
    from xgonav.config import FreespaceCfg
    from xgonav.freespace import FreeSpaceEstimator
    cfg = FreespaceCfg()
    est = FreeSpaceEstimator(cfg, (640, 480))
    floor = np.full((480, 640, 3), (90, 110, 130), np.uint8)       # gleichmäßiger Boden
    floor[:200] = (200, 60, 40)                                     # Wand oben, andere Farbe
    for _ in range(3):
        fs = est.update(floor)
    good = list(fs.sectors)
    assert min(good) > 0.5
    dark = np.full_like(floor, 3)
    for _ in range(20):                                             # schwarze Bilder: nicht lernen
        est.update(dark)
    assert min(est.update(floor).sectors) > 0.5
    # verdorbenes Modell (z. B. von früher): reset() lernt aus dem nächsten Bild neu
    est.model = est.model * 0
    est.model[0, 0] = 255
    assert max(est.update(floor).sectors) < 0.5                      # alles "blockiert"
    est.reset()
    assert min(est.update(floor).sectors) > 0.5
