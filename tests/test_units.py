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
    r.set_height(90)
    r.set_height(120)   # begrenzt auf height_limit_u8
    r.set_height(500)   # begrenzt, identisch -> kein neuer Write
    vals = [v for _, n, v in r.writes if n == "height"]
    assert vals == [0, 128, 205]
    assert r.height_mm == pytest.approx(60 + 60 * 205 / 255)


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


def test_egomotion_native_matches_python():
    """C++-Kern (xgonav/native, nur wenn gebaut) rechnet wie Python, bis auf Rundung."""
    from dataclasses import replace
    from xgonav.egomotion import _native_lib
    if _native_lib() is None:
        pytest.skip("C++-Kern nicht gebaut (make -C xgonav/native)")
    cfg = Config()
    r = SimRobot(cfg.robot)
    r._cur = (0.1, 0.05)
    r._height_mm = 85
    w = SimWorld(cfg.camera, r, None, start=(1.0, 1.5, 0.1), shake=1.0, vx_mps=1.5)
    ep = EgoMotion(replace(cfg.ego, native=False), (640, 480), cfg.camera.hfov_deg)
    en = EgoMotion(cfg.ego, (640, 480), cfg.camera.hfov_deg)
    assert (ep.backend, en.backend) == ("python", "c++")
    for k in range(60):
        t = k / 20
        w._integrate(t, 0.05)
        img, _ = w._render(t, 0.05)
        box = [(200.0, 100.0, 300.0, 400.0)]
        a, b = ep.update(k, t, img, box), en.update(k, t, img, box)
        assert (a.ok, a.inliers) == (b.ok, b.inliers)
        assert b.yaw == pytest.approx(a.yaw, abs=1e-12) and b.pitch == pytest.approx(a.pitch, abs=1e-12)
        assert b.sharpness == pytest.approx(a.sharpness, rel=1e-12)
        assert b.expansion == pytest.approx(a.expansion, abs=1e-12)
        assert np.allclose(a.A, b.A, atol=1e-9)


def test_ego_fast_paths_match_opencv():
    """Abkürzungen im C++-Kern (Grau + 2:1, Schärfe, goodFeaturesToTrack) bitgleich zu den
    OpenCV-Aufrufen in egomotion.py; Pfade, die die Selbstprüfung abgeschaltet hat (-1), entfallen."""
    import ctypes
    import cv2
    from xgonav.egomotion import _native_lib
    lib = _native_lib()
    if lib is None:
        pytest.skip("C++-Kern nicht gebaut (make -C xgonav/native)")
    rng = np.random.default_rng(5)
    cfg = Config()
    r = SimRobot(cfg.robot)
    r._cur = (0.1, 0.05)
    r._height_mm = 85
    w = SimWorld(cfg.camera, r, None, start=(1.0, 1.5, 0.1), shake=1.0, vx_mps=1.5)
    imgs = []
    for k in range(8):
        w._integrate(k / 10, 0.1)
        imgs.append(w._render(k / 10, 0.1)[0])
    n = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
    imgs += [n, cv2.GaussianBlur(n, (0, 0), 3)]
    for img in imgs:
        g = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (320, 240), interpolation=cv2.INTER_AREA)
        out = np.empty((240, 320), np.uint8)
        if lib.xg_gray_half(img.ctypes.data, 640, 480, img.strides[0], out.ctypes.data) == 0:
            assert np.array_equal(out, g)
        s = ctypes.c_double()
        if lib.xg_sharpness(g.ctypes.data, 320, 240, 320, ctypes.byref(s)) == 0:
            assert s.value == float(cv2.meanStdDev(cv2.Laplacian(g, cv2.CV_32F))[1][0, 0] ** 2)
        mask = np.full(g.shape, 255, np.uint8)
        for x, y in rng.integers(0, 240, (40, 2)):
            cv2.circle(mask, (int(x), int(y)), 8, 0, -1)
        cv2.rectangle(mask, (100, 50), (180, 200), 0, -1)
        for mk, maxc in ((mask, 90), (None, 150), (mask, 7)):
            ref = cv2.goodFeaturesToTrack(g, maxCorners=maxc, qualityLevel=0.01, minDistance=8, mask=mk, blockSize=5)
            ref = np.zeros((0, 2), np.float32) if ref is None else ref.reshape(-1, 2)
            pts = np.empty((maxc, 2), np.float32)
            k = lib.xg_good_features(g.ctypes.data, 320, 240, 320, None if mk is None else mk.ctypes.data, maxc,
                                     0.01, 8.0, pts.ctypes.data, maxc)
            if k >= 0:
                assert np.array_equal(pts[:k], ref)


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


def _post_reference(y, out, r, px, py, w0, h0):
    """Nachbearbeitung wie bis 2026-11-05 (argmax über alle Klassen, dann Filter)."""
    import cv2
    o = np.squeeze(out, 0).T
    boxes, scores = o[:, :4], o[:, 4:]
    cls = scores.argmax(1)
    conf = scores[np.arange(len(cls)), cls]
    m = conf >= y.cfg.conf
    if y.allowed is not None:
        m &= np.isin(cls, list(y.allowed))
    boxes, cls, conf = boxes[m], cls[m], conf[m]
    if not len(conf):
        return []
    xy = np.empty_like(boxes)
    xy[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2 - px) / r
    xy[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2 - py) / r
    xy[:, 2] = boxes[:, 2] / r
    xy[:, 3] = boxes[:, 3] / r
    off = xy.copy()
    off[:, :2] += cls[:, None] * 4096.0
    keep = cv2.dnn.NMSBoxes(off.tolist(), conf.tolist(), y.cfg.conf, y.cfg.iou)
    return [(y.names[int(cls[i])], float(conf[i])) + tuple(float(v) for v in xy[i])
            for i in np.array(keep).reshape(-1)]


@pytest.mark.parametrize("classes", [[], ["person", "chair", "couch", "dining table", "potted plant"]])
def test_yolo_post_fast_path_equals_reference(classes):
    from xgonav.detector import COCO
    rng = np.random.default_rng(1)
    y = _fake_yolo()
    y.cfg = DetectorCfg(conf=0.35, classes=classes)
    y._set_names(COCO)
    for _ in range(30):
        out = np.zeros((1, 84, 1344), np.float32)
        out[0, :2] = rng.uniform(20, 236, (2, 1344))
        out[0, 2:4] = rng.uniform(5, 80, (2, 1344))
        out[0, 4:] = rng.uniform(0, 0.3, (80, 1344))
        hot = rng.choice(1344, 40, replace=False)          # Treffer, teils mit stärkerer fremder Klasse
        out[0, 4 + rng.integers(0, 80, 40), hot] = rng.uniform(0.3, 0.95, 40)
        out[0, 4 + rng.integers(0, 80, 40), hot] = rng.uniform(0.3, 0.95, 40)
        got = [(d.cls, d.conf, d.box) for d in y._post(out, 0.4, 0, 32, 640, 480)]
        ref = _post_reference(y, out, 0.4, 0, 32, 640, 480)
        assert len(got) == len(ref)
        for (c, cf, b), (rc, rcf, x, yy, w, h) in zip(got, ref):
            assert c == rc and cf == pytest.approx(rcf)
            assert b == pytest.approx((max(0.0, x), max(0.0, yy), min(640, x + w), min(480, yy + h)))


def test_ncnn_metadata_and_model_fallback(tmp_path):
    from xgonav.detector import DetectorWorker, _read_ultralytics_meta, is_ncnn_model, model_exists
    d = tmp_path / "yolo11n_256_ncnn"
    d.mkdir()
    (d / "metadata.yaml").write_text("task: detect\nbatch: 1\nimgsz:\n- 256\n- 256\nnames:\n  0: person\n"
                                     "  1: bicycle\n  2: 'dining table'\nargs:\n  half: false\n")
    names, size = _read_ultralytics_meta(str(d / "metadata.yaml"))
    assert names == ["person", "bicycle", "dining table"] and size == 256
    (d / "model.ncnn.param").write_text("kaputt")
    assert is_ncnn_model(str(d)) and not model_exists(str(d))   # .bin fehlt
    # weder Haupt- noch Rückfallmodell vorhanden -> Detektor aus, kein Absturz
    w = DetectorWorker(DetectorCfg(model=str(d), fallback_model=str(tmp_path / "fehlt.onnx")))
    assert w.start() is False and w.model is None


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


def test_kanten_native_matches_numpy():
    """C++-Kantenverfahren (nur wenn gebaut) gleich der numpy-Fassung: Simulator-Bilder,
    Lampenschein (auch am Rand), dunkle Szene, Rauschen; mit und ohne Log/Schein-Regel."""
    import cv2
    from xgonav.freespace import _edge_lib
    if _edge_lib() is None:
        pytest.skip("C++-Kern nicht gebaut (make -C xgonav/native)")
    rng = np.random.default_rng(3)
    imgs = []
    g = np.full((480, 640, 3), 90, np.uint8)
    g[:270] = 40
    g[200:268, 300:340] = 255
    g[270:330, 296:344] = np.linspace(250, 170, 60).astype(np.uint8)[:, None, None]
    g[200:268, 0:24] = 255
    g[270:330, 0:28] = np.linspace(250, 170, 60).astype(np.uint8)[:, None, None]
    imgs.append(g)
    d = np.full((480, 640, 3), 25, np.uint8)
    d[:330] = 11
    d[:270] = 9
    imgs.append(d)
    for _ in range(4):
        n = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
        imgs.append(cv2.GaussianBlur(n, (0, 0), 6) if _ % 2 else n)
    cfg = Config()
    r = SimRobot(cfg.robot)
    r._cur = (0.1, 0.05)
    r._height_mm = 85
    w = SimWorld(cfg.camera, r, None, start=(1.0, 1.5, 0.1), shake=1.0, vx_mps=1.5)
    for k in range(10):
        w._integrate(k / 10, 0.1)
        imgs.append(w._render(k / 10, 0.1)[0])
    for kw in ({}, {"edge_log_s": 0.0}, {"edge_glare": False}, {"edge_thin": False}, {"glare_spread": 0}):
        en, _ = _kanten_est(**kw)
        ep, _ = _kanten_est(native=False, **kw)
        assert (en.edge_backend, ep.edge_backend) == ("c++", "python")
        for i, img in enumerate(imgs):
            for pd in (0.0, 0.02):
                a, b = ep.update(img, pd), en.update(img, pd)
                assert np.array_equal(a.columns, b.columns, equal_nan=True), (kw, i, pd)
                assert ep.diag["glare_cols"] == en.diag["glare_cols"]


def test_floor_downscale_native_equals_cv2():
    """Verkleinern im C++-Kern (nur wenn gebaut und beim Start bitgleich) = cv2.resize(INTER_AREA)."""
    import cv2
    from xgonav.freespace import _edge_lib
    if _edge_lib() is None:
        pytest.skip("C++-Kern nicht gebaut (make -C xgonav/native)")
    est, _ = _kanten_est()
    assert est._area is not None, "area_down nicht bitgleich zu cv2.resize"
    rng = np.random.default_rng(11)
    img = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
    big = np.zeros((480, 700, 3), np.uint8)          # Zeilenabstand größer als die Breite
    big[:, :640] = img
    for x in (img, big[:, :640], cv2.GaussianBlur(img, (0, 0), 2)):
        assert np.array_equal(est._downscale(x), cv2.resize(x, (160, 120), interpolation=cv2.INTER_AREA))


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


def test_paused_pipeline_keeps_pose_from_wheels():
    """Handfahrt bei gestoppter Bildverarbeitung: Raumkarte und Kurs folgen den Rädern
    (sonst stand die Pose still und „Zum Start“ zielte danach daneben)."""
    import math
    from xgonav.app import App
    from xgonav.wheels import WheelOdom
    cfg = Config(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    wh = WheelOdom(cfg.odom)
    app = App(cfg, robot, SimWorld(cfg.camera, robot, None), imu=None, wheels=wh)
    k = 1.0 / wh.m_per_tick
    seq = [1]

    def drive(l0, r0, l1, r1, n=50):                     # kleine Schritte (große gelten als Sprung)
        for i in range(1, n + 1):
            seq[0] += 1
            wh.add(seq[0], time.monotonic(), round((l0 + (l1 - l0) * i / n) * k), round((r0 + (r1 - r0) * i / n) * k))
            app._paused_odom()

    wh.add(1, time.monotonic(), 0, 0)
    app._paused_odom()                                   # erste Probe: Bezug
    drive(0, 0, 0.5, 0.5)                                # 0,5 m geradeaus
    x, y, th = app.gridmap.pose()
    assert x == pytest.approx(0.5, abs=0.005) and abs(y) < 1e-6 and abs(th) < 1e-6
    d = math.pi / 2 * cfg.odom.track_m / 2                # 90° links auf der Stelle
    drive(0.5, 0.5, 0.5 - d, 0.5 + d)
    assert app.gridmap.pose()[2] == pytest.approx(math.pi / 2, abs=0.01)
    assert app.yaw == pytest.approx(math.pi / 2, abs=0.01)
    assert app.gridmap.pose()[0] == pytest.approx(0.5, abs=0.005)


def test_drive_gains():
    """reverse_gain gilt immer, der Vorwärtsfaktor nur in der Handsteuerung."""
    from types import SimpleNamespace as NS
    from xgonav.app import App
    from xgonav.behaviors import Command
    cfg = Config(); cfg.robot.reverse_gain = 2.0; cfg.robot.manual_forward_gain = 2.0; cfg.robot.max_v = 0.15
    r = SimRobot(cfg.robot)
    r._cur = (-0.15, 0.0); r._send(time.monotonic() + 10)
    assert r._sent[0] == 128 - round(0.30 * 127)          # 90
    app = NS(cfg=cfg, robot=r, behavior=NS(name="explore"))
    App._drive(app, Command(0.15, 0.0))
    r._cur = (0.15, 0.0); r._send(time.monotonic() + 20)
    assert r._sent[0] == 128 + round(0.15 * 127)          # 147: autonom ohne Faktor
    app.behavior.name = "manual"
    App._drive(app, Command(0.15, 0.0))
    r._cur = (0.15, 0.0); r._send(time.monotonic() + 30)
    assert r._sent[0] == 128 + round(0.30 * 127)          # 166: Handsteuerung wie rückwärts
    assert r._target[0] == pytest.approx(0.15)            # max_v begrenzt den Sollwert davor


def test_level_pulse():
    """Impuls einmal je Aufstehen, nach dem Aufschwung und auf Knopfdruck; liegend nicht."""
    cfg = Config(); cfg.robot.level_after_standup = True; cfg.robot.level_pulse_s = 3.0
    r = SimRobot(cfg.robot)
    sent = lambda: [(w[1], w[2]) for w in r.writes if w[1] in ("roll", "roll_balance")]
    puls = [("roll", 0), ("roll_balance", 1), ("roll_balance", 0)]
    r.state.balancing = True
    r._level_tick(0.0)
    r._level_tick(10.0)
    assert sent() == []                                   # xgonav startet, er steht schon: nichts
    r.state.balancing = False
    r._level_tick(11.0)
    assert not r.request_level()                          # liegt: Knopf tut nichts
    r.state.balancing = True
    r._level_tick(12.0)
    r._level_tick(14.0)
    assert sent() == []                                   # noch im Aufschwung
    r._level_tick(14.6)
    assert sent() == puls[:2]
    r._level_tick(17.0)
    assert sent() == puls[:2]                             # 3 s noch nicht um
    r._level_tick(17.7)
    r._level_tick(60.0)
    assert sent() == puls                                 # aus, und danach nicht wieder
    assert r.request_level()                              # Knopf: gleicher Impuls noch einmal
    r._level_tick(61.0)
    assert r.request_level()                              # zweiter Druck während des Impulses zählt nicht
    r._level_tick(64.1)
    r._level_tick(70.0)
    assert sent() == puls * 2
    r.state.balancing = False
    r._level_tick(71.0)
    r.state.balancing = True
    r._level_tick(72.0)
    r._level_tick(74.6)
    r._level_tick(77.7)
    assert sent() == puls * 3                             # nächstes Aufstehen wieder einmal
    r0 = SimRobot(Config().robot)                         # Default: automatisch nie, Knopf geht
    r0.state.balancing = False
    r0._level_tick(0.0)
    r0.state.balancing = True
    r0._level_tick(1.0)
    r0._level_tick(9.0)
    assert not [w for w in r0.writes if w[1] == "roll_balance"]
    assert r0.request_level()
    r0._level_tick(10.0)
    assert [w[2] for w in r0.writes if w[1] == "roll_balance"] == [1]


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


def test_raw_run_dir_keeps_previous_recording(tmp_path):
    """Jeder Start bekommt ein eigenes Verzeichnis, auch bei gleicher Uhrzeit."""
    import os
    from unittest import mock
    from xgonav.__main__ import _raw_run_dir
    base = tmp_path / "neu" / "fahrt"
    with mock.patch("time.strftime", return_value="20261106-150000"):
        a = _raw_run_dir(str(base))
        with open(os.path.join(a, "000000.jpg"), "w") as f:
            f.write("alt")
        b = _raw_run_dir(str(base))
        c = _raw_run_dir(str(base))
    assert os.path.basename(a) == "20261106-150000"
    assert os.path.basename(b) == "20261106-150000-2" and os.path.basename(c) == "20261106-150000-3"
    with open(os.path.join(a, "000000.jpg")) as f:
        assert f.read() == "alt"
    assert not os.listdir(b)


def test_gain_runtime_limits():
    from xgonav.camera import FrameSource
    from xgonav.config import CameraCfg
    src = FrameSource(CameraCfg())
    assert src.gain == 0.0                              # Default: Automatik
    assert src.set_gain(4) == 4.0 and src.gain == 4.0
    assert src.set_gain(16) == 16.0 and src.set_gain(63.9) == 63.9
    for bad in (0.5, 64, -1):
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
    # 0,2 s Soll; Windows-Timer (~15,6 ms Auflösung) lieferte 0,188 s -> Spielraum
    assert min(b - a for a, b in zip(calls, calls[1:])) >= 0.18


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
    cfg.grid.row_bias = 0.0                                     # hier zählt die reine Geometrie
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


def _wall_fs(g, dist):
    """Bodenmodell-Ergebnis mit gerader Wand in dist (m) bei Nick 0."""
    from xgonav.freespace import FreeSpace
    hz = g.horizon_row(0) / 4
    ys = np.arange(int(hz * 4) + 1, 480)
    y1 = ys[np.nanargmin(np.abs(g.ground_dist_np(ys) - dist))]
    col = (120 - y1 / 4) / (120 - hz)
    return FreeSpace([col] * 5, np.full(160, col), hz, np.zeros((120, 160), np.uint8), True, None)


def test_gridmap_range_hits_and_margin():
    """Ein Treffer reicht nicht für "belegt", Treffer nur bis max_range_m, Freistrahl weiter und mit Rand."""
    from xgonav.gridmap import GridMap
    cfg = Config()
    cfg.grid.row_bias = 0.0
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    m = GridMap(cfg.grid, g, 160, 120)
    near = _wall_fs(g, 1.0)
    m.add_freespace(near, 0.0)
    assert len(m.occupied_cells()) == 0                         # Ausreißer sperrt nichts
    m.add_freespace(near, 0.0)
    assert len(m.occupied_cells()) > 0
    far = GridMap(cfg.grid, g, 160, 120)
    wall = _wall_fs(g, 2.2)
    for _ in range(5):
        far.add_freespace(wall, 0.0)
    assert len(far.occupied_cells()) == 0                       # zu weit für einen Treffer
    r, c = np.nonzero(far.lo < -cfg.grid.l_show)
    xs = (c + 0.5 - far.n / 2) * far.res
    assert 1.3 < xs.max() < cfg.grid.free_range_m + 0.1         # frei nur bis free_range_m
    # row_bias schiebt die Grenze nach hinten
    biased = Config()
    b = GridMap(biased.grid, g, 160, 120)
    for _ in range(2):
        b.add_freespace(near, 0.0)
    plain = m.occupied_cells()[:, 0].mean()
    assert b.occupied_cells()[:, 0].mean() > plain + 0.05


def test_gridmap_driven_path_is_free():
    from xgonav.gridmap import GridMap
    cfg = Config()
    m = GridMap(cfg.grid, CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)
    r, c = m.cell(0.3, 0.0)
    m.lo[int(r), int(c)] = cfg.grid.l_max                       # angeblich belegt, liegt auf dem Weg
    for _ in range(10):
        m.move(0.05, 0.0)
    assert m.lo[int(r), int(c)] < -cfg.grid.l_show
    r2, c2 = m.cell(0.3, cfg.grid.driven_free_r_m + 0.10)
    assert m.lo[int(r2), int(c2)] == 0.0                        # daneben bleibt unbekannt


def test_freespace_keeps_pitch_and_pose_of_capture():
    """Das Ergebnis des Bodenmodell-Threads kommt später an: Karten rechnen mit Nick und Pose
    der Aufnahme, sonst streut die Entfernung um Dezimeter (2026-11-05)."""
    from xgonav.freespace import FreeSpaceEstimator, FreeSpaceWorker
    from xgonav.localmap import mask_movable
    cfg = Config()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    est = FreeSpaceEstimator(cfg.freespace, (640, 480), g)
    w = FreeSpaceWorker(est)
    w.start()
    w.offer(np.full((480, 640, 3), 90, np.uint8), 0.03, [], (1.0, 2.0, 0.5))
    t_end = time.monotonic() + 5
    fs = None
    while fs is None and time.monotonic() < t_end:
        fs = w.poll()
        time.sleep(0.01)
    w.close()
    assert fs.pitch_dev == 0.03 and fs.pose == (1.0, 2.0, 0.5)
    view = mask_movable(fs, [("person", (0, 0, 100, 100))], 640, 480)
    assert view is not fs and view.pitch_dev == 0.03 and view.pose == (1.0, 2.0, 0.5)


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


def test_gridmap_native_matches_numpy():
    """Eintragen im C++-Kern (nur wenn gebaut) bitgleich zu numpy: zufällige Bodengrenzen mit
    NaN-Spalten, bekanntem Bereich, Strahlen bis Horizont, wechselnde Posen und Nicken."""
    from dataclasses import replace
    from xgonav.freespace import FreeSpace
    from xgonav.gridmap import GridMap, _grid_lib
    if _grid_lib() is None:
        pytest.skip("C++-Kern nicht gebaut (make -C xgonav/native)")
    cfg = Config()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    mn = GridMap(cfg.grid, g, 160, 120)
    mp = GridMap(replace(cfg.grid, native=False), g, 160, 120)
    assert (mn.backend, mp.backend) == ("c++", "python")
    rng = np.random.default_rng(11)
    hz = g.horizon_row(0) / 4
    pose = np.zeros(3)
    for k in range(300):
        cols = rng.uniform(0.0, 1.0, 160)
        cols[rng.random(160) < 0.1] = np.nan
        cols[rng.random(160) < 0.15] = 1.0                       # frei bis Horizont
        kf = None if k % 3 == 0 else np.clip(rng.uniform(-0.1, 0.3, 160), 0, 1)
        fs = FreeSpace([0.5] * 5, cols, hz, np.zeros((120, 160), np.uint8), True, kf)
        pose += (rng.normal(0, 0.05), rng.normal(0, 0.05), rng.normal(0, 0.2))
        if k == 150:
            pose[:] = (7.9, -7.95, 1.0)                          # Kartenrand: Strahlen laufen hinaus
        pd = float(rng.normal(0, 0.01))
        mn.add_freespace(fs, pd, tuple(pose))
        mp.add_freespace(fs, pd, tuple(pose))
    assert np.array_equal(mn.lo, mp.lo)
    assert np.abs(mn.lo).max() > 1.0 and mn.updates == mp.updates == 300


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


def test_planner_keeps_path_and_plans_around_obstacles():
    """Bisheriger Weg bleibt, solange frei und nicht viel länger (sonst sprang am RiderPi die
    Zielrichtung); Merkkarten-Punkte sperren wie belegte Zellen."""
    import numpy as np
    from xgonav.planner import path_length, plan
    cfg, m = _grid()
    hc = cfg.home
    s, g = (0.0, 0.0), (2.0, 0.0)
    straight = plan(m, s, g, hc)
    assert straight is not None and max(abs(y) for _, y in straight) < 0.06
    small = [(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)]               # 2,24 m <= 1,2 x 2 + 0,3
    assert plan(m, (0.1, 0.05), g, hc, keep=small) == [(0.1, 0.05), (1.0, 0.5), (2.0, 0.0)]
    big = [(0.0, 0.0), (1.0, 1.5), (2.0, 0.0)]                 # 3,6 m: neuer Weg ist viel kürzer
    assert path_length(plan(m, s, g, hc, keep=big)) < 2.1
    other_goal = [(0.0, 0.0), (1.0, 0.5), (2.0, 0.5)]          # anderes Ziel -> nicht behalten
    assert plan(m, s, g, hc, keep=other_goal)[-1] == g
    # Hindernispunkte quer über die Gerade: Weg geht außen herum, gerader Weg wird nicht behalten
    obst = np.array([(1.0, y) for y in np.arange(-0.4, 0.41, 0.03)])
    p = plan(m, s, g, hc, keep=straight, obstacles=obst)
    assert p is not None and max(abs(y) for _, y in p) > 0.5


def test_home_turns_in_place_only_for_large_errors():
    """Lenkung mit Hysterese: bei mittlerem Fehler in der Kurve weiterfahren, auf der Stelle
    drehen erst ab turn_enter_deg und dann bis unter turn_exit_deg."""
    from xgonav.behaviors import Home, World
    from xgonav.egomotion import EgoResult
    cfg = Config()
    b = Home(cfg.behavior, cfg.robot, cfg.home)
    b.set_path([(0.0, 0.0), (3.0, 0.0)])

    def step(err_deg):
        w = World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, 0.0, 0.05), tracks=[], free=None,
                  balancing=True, yaw=0.0, cmd_v=0.0, pose=(0.0, 0.0, -math.radians(err_deg)), goal=(3.0, 0.0, 0.0))
        return b.step(w)
    c = step(50)
    assert c.v > 0 and c.w > 0                                  # Kurve, nicht anhalten
    assert step(100).v == 0                                     # großer Fehler: auf der Stelle
    c = step(40)
    assert c.v == 0 and c.w > 0                                 # dreht weiter bis unter 25°
    assert step(20).v > 0


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


# ---------------------------------------------------------------- Erkunden (frontier.py, MapExplore)
def _room_map(door=False):
    """Bekannter Raum 2,4 x 2,4 m um den Ursprung, Wände belegt; door: 0,8 m Lücke in der +x-Wand
    (dahinter unbekannt)."""
    from xgonav.gridmap import GridMap
    cfg = Config()
    m = GridMap(cfg.grid, CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)
    r0, c0 = (int(v) for v in m.cell(-1.2, -1.2))
    r1, c1 = (int(v) for v in m.cell(1.2, 1.2))
    m.lo[r0:r1 + 1, c0:c1 + 1] = cfg.grid.l_max
    m.lo[r0 + 1:r1, c0 + 1:c1] = cfg.grid.l_min
    if door:
        dr0, _ = (int(v) for v in m.cell(0.0, -0.4))
        dr1, _ = (int(v) for v in m.cell(0.0, 0.4))
        m.lo[dr0:dr1 + 1, c1] = cfg.grid.l_min
    return cfg, m


def test_frontier_finds_door_and_nothing_in_closed_room():
    from xgonav.frontier import next_goal
    cfg, m = _room_map()
    p = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert p.path is None and p.clusters == 0                  # geschlossen: nichts zu erkunden
    assert p.known_m2 > 4.0
    cfg, m = _room_map(door=True)
    p = next_goal(m, (-0.5, 0.0, 0.0), cfg.home, cfg.explore)
    assert p.path is not None and p.clusters == 1
    assert abs(p.frontier[0] - 1.2) < 0.15 and abs(p.frontier[1]) < 0.3
    assert p.goal[0] < p.frontier[0] - 0.1                     # hält davor an
    # gesperrt: kein Ziel mehr, wird aber gezählt
    p = next_goal(m, (-0.5, 0.0, 0.0), cfg.home, cfg.explore, avoid=[(p.frontier[0], p.frontier[1], 0.5)])
    assert p.path is None and p.avoided == 1


def test_frontier_ignores_unreachable_and_small():
    from xgonav.frontier import next_goal
    cfg, m = _room_map()
    # Freie Insel hinter der Wand: hat Grenze zu unbekannt, ist aber nicht erreichbar
    r, c = (int(v) for v in m.cell(2.5, 0.0))
    m.lo[r - 8:r + 8, c - 8:c + 8] = cfg.grid.l_min
    assert next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore).path is None
    # Winziges Loch in der Mitte des Raums: zu klein für ein Ziel
    r, c = (int(v) for v in m.cell(0.3, 0.3))
    m.lo[r, c] = 0.0
    assert next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore).path is None


def test_frontier_keeps_goal_instead_of_switching():
    """Ein bestehendes Ziel bleibt, auch wenn ein anderes gerade besser ist (sonst pendelt er)."""
    from xgonav.frontier import next_goal
    cfg, m = _room_map(door=True)
    r0, _ = (int(v) for v in m.cell(0.0, -0.4))
    r1, _ = (int(v) for v in m.cell(0.0, 0.4))
    _, c0 = (int(v) for v in m.cell(-1.2, 0.0))
    m.lo[r0:r1 + 1, c0] = cfg.grid.l_min                       # zweite Tür in der -x-Wand
    pose = (-0.8, 0.0, math.pi)
    best = next_goal(m, pose, cfg.home, cfg.explore)
    assert best.clusters == 2 and best.frontier[0] < 0         # nah und voraus
    far = (1.2, 0.0)
    kept = next_goal(m, pose, cfg.home, cfg.explore, keep_frontier=far)
    assert kept.frontier[0] > 0.9


def test_frontier_ignores_localmap_for_reachability():
    """Merkkarte versperrt den Weg: Das Ziel bleibt erhalten und wird notfalls ohne sie angefahren."""
    from xgonav.frontier import next_goal
    cfg, m = _room_map(door=True)
    ys = np.linspace(-1.2, 1.2, 60)
    wall = np.column_stack([np.full(60, 0.3), ys])             # angebliche Wand quer durch den Raum
    p = next_goal(m, (-0.5, 0.0, 0.0), cfg.home, cfg.explore, obstacles=wall)
    assert p.path is not None and p.clusters == 1


def test_shorten_path():
    from xgonav.frontier import shorten
    assert shorten([(0.0, 0.0), (1.0, 0.0)], 0.3)[-1] == pytest.approx((0.7, 0.0))
    assert shorten([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 1.2)[-1] == pytest.approx((0.8, 0.0))
    assert shorten([(0.0, 0.0), (0.1, 0.0)], 0.3) == [(0.0, 0.0), (0.0, 0.0)]


def test_wavefront_keeps_narrow_gap():
    from xgonav.frontier import wavefront
    p = np.zeros((40, 40), bool)
    p[:, :18] = p[:, 22:] = True                               # zwei Hälften
    p[20, 18:22] = True                                        # einzellig breite Verbindung
    d, step = wavefront(p, (5, 5))
    assert d[5 // step, 35 // step] > 0
    q = p.copy()
    q[20, 18:22] = False
    d, step = wavefront(q, (5, 5))
    assert d[5 // step, 35 // step] < 0


def test_pathfollower_detour_is_not_stuck():
    """Umweg: Luftlinie zum Ziel wächst, der Rider kommt aber voran. Stehen: gilt als fest."""
    from xgonav.behaviors import PathFollower, World
    from xgonav.egomotion import EgoResult
    cfg = Config()
    f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
    f.set_path([(0.0, 0.0), (-3.0, 0.0), (-3.0, 3.0), (3.0, 3.0)])

    def w(t, x, y, th=math.pi):
        return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[], free=None,
                     balancing=True, yaw=0.0, cmd_v=0.0, pose=(x, y, th))
    for i in range(100):
        f.drive(w(i * 0.1, -i * 0.03, 0.0), "Test")
    assert not f.stuck
    for i in range(100, 145):
        f.drive(w(i * 0.1, -3.0, 0.0), "Test")
    assert f.stuck


def test_pathfollower_small_error_still_steers():
    """Unter ~11° Fehler läge der Lenkbefehl unter der Totzone der Firmware (u8 128 ± 6)."""
    from xgonav.behaviors import PathFollower, World
    from xgonav.egomotion import EgoResult
    cfg = Config()
    f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
    f.set_path([(0.0, 0.0), (3.0, 0.0)])

    def w(deg):
        return World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, 0.0, 0.05), tracks=[], free=None,
                     balancing=True, yaw=0.0, cmd_v=0.0, pose=(0.0, 0.0, -math.radians(deg)))
    assert f.drive(w(8), "T").w >= cfg.home.min_w
    assert f.drive(w(-8), "T").w <= -cfg.home.min_w
    assert f.drive(w(0.5), "T").w == pytest.approx(0.0, abs=0.01)


def _ex_world(t, pose, goal=(0.0, 0.0, 0.0), battery=None):
    from xgonav.behaviors import World
    from xgonav.egomotion import EgoResult
    return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[], free=None,
                 balancing=True, yaw=pose[2], cmd_v=0.0, pose=pose, goal=goal, battery=battery)


def _plan(goal=(1.0, 0.0), frontier=(1.3, 0.0), clusters=1, avoided=0, known=5.0):
    from xgonav.frontier import ExplorePlan
    return ExplorePlan([(0.0, 0.0), goal], goal, frontier, clusters, avoided, 2.0, known)


def test_map_explore_scan_then_goal_then_face():
    from xgonav.behaviors import MapExplore
    cfg = Config()
    b = MapExplore(cfg.behavior, cfg.robot, cfg.home, cfg.explore)
    th, t = 0.0, 0.0
    c = b.step(_ex_world(t, (0.0, 0.0, th)))
    assert c.v == 0 and c.w > 0 and b.state == "scan" and not b.want_plan(t)   # erst Rundumblick
    while b.state == "scan":
        th += 0.1
        t += 0.1
        c = b.step(_ex_world(t, (0.0, 0.0, th)))
    assert b.want_plan(t) and not b.want_plan(t)               # ein Auftrag zur Zeit
    b.set_path(_plan())
    c = b.step(_ex_world(t, (0.0, 0.0, 0.0)))
    assert c.v > 0 and b.frontier == (1.3, 0.0)
    c = b.step(_ex_world(t + 1, (0.9, 0.0, math.pi)))          # angekommen, schaut aber zurück
    assert b.state == "face" and b.visited == [(1.3, 0.0)]
    c = b.step(_ex_world(t + 1.1, (0.9, 0.0, math.pi)))
    assert c.v == 0 and abs(c.w) > 0
    b.step(_ex_world(t + 1.2, (0.9, 0.0, 0.0)))
    assert b.state == "settle"
    b.step(_ex_world(t + 5, (0.9, 0.0, 0.0)))
    assert b.state == "ziel"


def _explore_ready():
    from xgonav.behaviors import MapExplore
    cfg = Config()
    b = MapExplore(cfg.behavior, cfg.robot, cfg.home, cfg.explore)
    b.state = "ziel"
    return cfg, b


def test_map_explore_stuck_replans_then_blocks_goal():
    cfg, b = _explore_ready()
    b.want_plan(0.0)
    b.set_path(_plan())
    b.f.stuck = True
    c = b.step(_ex_world(1.0, (0.0, 0.0, 0.0)))
    assert c.v == 0 and b.f.path is None and b.stucks == 1 and not b.blocked_goals
    b.t_plan = -1e9
    assert b.want_plan(5.0) and not b.keep_ok
    b.set_path(_plan())
    b.f.stuck = True
    b.step(_ex_world(6.0, (0.0, 0.0, 0.0)))
    assert b.blocked_goals == [(1.3, 0.0)] and b.state == "backup"
    c = b.step(_ex_world(6.1, (0.0, 0.0, 0.0)))
    assert c.v < 0                                             # setzt zurück (Bahnprüfung gilt)
    b.step(_ex_world(7.0, (0.0, 0.0, 0.0)))
    assert b.state == "ziel"


def test_map_explore_goal_timeout_blocks_goal():
    cfg, b = _explore_ready()
    b.t_plan = 0.0
    b.set_path(_plan())
    b.step(_ex_world(1.0, (0.0, 0.0, 0.0)))
    assert not b.blocked_goals
    b.step(_ex_world(cfg.explore.goal_timeout_s + 1, (0.0, 0.0, 0.0)))
    assert b.blocked_goals == [(1.3, 0.0)]


def test_map_explore_finishes_saves_and_returns_home():
    from xgonav.frontier import ExplorePlan
    cfg, b = _explore_ready()
    b.visited.append((5.0, 5.0))
    for _ in range(cfg.explore.no_goal_limit):
        b.set_path(ExplorePlan(None, known_m2=20.0))
    b.pending = True                                           # eine Zielsuche läuft noch
    b.step(_ex_world(10.0, (1.0, 1.0, 0.0)))
    assert b.state == "heim" and b.reason == "Karte fertig" and b.save_map
    b.set_path(ExplorePlan([(1.0, 1.0), (2.0, 2.0)], (2.0, 2.0), (2.0, 2.0)))   # veraltet: wird verworfen
    assert b.f.path is None
    b.set_path([(1.0, 1.0), (0.0, 0.0)])
    assert b.step(_ex_world(11.0, (1.0, 1.0, math.radians(-135)))).v > 0
    c = b.step(_ex_world(30.0, (0.05, 0.0, 0.0)))
    assert b.done and c.v == 0 and c.status == "Karte fertig"


def test_map_explore_wanders_before_giving_up():
    """Noch kein Ziel gefunden: erst woanders hinfahren, nach max_wander aufhören."""
    from xgonav.frontier import ExplorePlan
    cfg, b = _explore_ready()
    t = 0.0
    for k in range(cfg.explore.max_wander):
        for _ in range(cfg.explore.no_goal_limit):
            b.set_path(ExplorePlan(None))
        b.step(_ex_world(t, (0.0, 0.0, 0.0)))
        assert b.state == "wander" and b.wanders == k + 1
        t += cfg.explore.wander_s + 1
        b.step(_ex_world(t, (0.0, 0.0, 0.0)))
        assert b.state == "ziel"
    for _ in range(cfg.explore.no_goal_limit):
        b.set_path(ExplorePlan(None))
    b.step(_ex_world(t, (0.0, 0.0, 0.0)))
    assert b.state == "heim" and b.reason == "festgefahren"


def test_map_explore_ends_on_battery_and_no_growth():
    cfg, b = _explore_ready()
    b.want_plan(0.0)
    b.set_path(_plan())
    b.step(_ex_world(1.0, (1.0, 1.0, 0.0), battery=cfg.explore.min_battery))
    assert b.state == "heim" and b.reason == "Akku"
    cfg, b = _explore_ready()
    b.t_plan = 0.0
    b.set_path(_plan(known=5.0))
    b.step(_ex_world(1.0, (0.0, 0.0, 0.0)))
    assert b.state == "ziel"
    b.step(_ex_world(cfg.explore.no_growth_s + 1, (0.0, 0.0, 0.0)))
    assert b.state == "heim" and b.reason == "Karte fertig"


def test_plan_goal_in_wall_is_not_cleared_on_request():
    from xgonav.planner import plan
    cfg, m = _room_map()
    assert plan(m, (0.0, 0.0), (1.2, 0.0), cfg.home, clear_goal=False) is None
    assert plan(m, (0.0, 0.0), (0.5, 0.5), cfg.home, clear_goal=False) is not None


def test_plan_after_load_searches_known_area():
    """Nach dem Laden ist der gefahrene Weg leer: Umwege müssen trotzdem im Suchgebiet liegen."""
    from xgonav.planner import known_area
    cfg, m = _room_map()
    r0, c0, r1, c1 = known_area(m, [(0.0, 0.0)], 0.5)
    rw, cw = (int(v) for v in m.cell(-1.2, -1.2))
    assert r0 <= rw - 9 and c0 <= cw - 9


def test_phantoms_shorten_columns():
    """Der Simulator speist Fehlalarme wirklich ein."""
    from xgonav.freespace import FreeSpace
    from xgonav.sim import Phantoms, SimScene

    class Est:
        H, W = 120, 160
        geo = CamGeometry(640, 480, 88.5, 1.39, 43.0, 85.0)

        def update(self, bgr, pitch_dev=0.0, boxes=()):
            hz = self.geo.horizon_row(0) / 4
            return FreeSpace([1.0] * 5, np.full(160, 1.0), hz, None, True)

    class W:
        scene = SimScene()
        x, y, th = 1.0, 1.0, 0.0
    est = Est()
    p = Phantoms(W, est, n=1, on_prob=1.0)
    p.pts = [(2.0, 1.0)]
    p.dirs = [math.pi]                                         # sichtbar aus Richtung des Riders
    fs = est.update(None)
    assert p.hits == 1 and np.nanmin(fs.columns) < 0.9
    assert (fs.columns < 0.9).sum() < 40                       # nur ein Fleck, nicht das ganze Bild


def test_blocking_boxes_skip_furniture_target_and_stale():
    from xgonav.config import FreespaceCfg
    from xgonav.localmap import blocking_boxes
    from xgonav.tracker import Track
    bed = Track("bed", 320, 370, 640, 200, 0.6, 10.0)
    person = Track("person", 100, 300, 50, 200, 0.8, 10.0)
    target = Track("person", 400, 300, 50, 200, 0.8, 10.0)
    alt = Track("chair", 200, 300, 50, 80, 0.8, 9.0)
    boxes = blocking_boxes([bed, person, target, alt], target.id, 10.1, FreespaceCfg().no_block_cls)
    assert boxes == [person.box]


def _world_at(t, pose):
    from xgonav.behaviors import World
    from xgonav.egomotion import EgoResult
    return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[], free=None,
                 balancing=True, yaw=0.0, cmd_v=0.0, pose=pose, goal=(0.0, 0.0, 0.0))


def test_home_gives_up_when_stuck():
    """Hindernis im Weg: Neuplanen hilft nicht, Fahrt endet mit Meldung statt endlos zu warten."""
    from xgonav.behaviors import Home
    cfg = Config()
    b = Home(cfg.behavior, cfg.robot, cfg.home)
    pose = (1.0, 0.0, math.pi)
    assert b.want_plan(0.0)
    b.set_path([pose[:2], (0.0, 0.0)])
    b.step(_world_at(0.0, pose))
    for k in range(cfg.home.max_fail):
        b.f.stuck = True
        assert b.want_plan(float(k + 1))
        b.set_path([pose[:2], (0.0, 0.0)])
        c = b.step(_world_at(float(k + 1), pose))
    assert not b.gave_up                                         # noch nicht lange genug
    c = b.step(_world_at(cfg.home.give_up_s + 1.0, pose))
    assert b.gave_up and c.v == 0 and "kein Weg" in c.status


def test_home_stuck_counter_resets_with_progress():
    from xgonav.behaviors import Home
    cfg = Config()
    b = Home(cfg.behavior, cfg.robot, cfg.home)
    far, near = (3.0, 0.0, math.pi), (2.0, 0.0, math.pi)
    assert b.want_plan(0.0)
    b.set_path([far[:2], (0.0, 0.0)])
    b.step(_world_at(0.0, far))
    for k in range(cfg.home.max_fail - 1):
        b.f.stuck = True
        b.pending = False
        assert b.want_plan(float(k + 1))
    b.pending = False
    b.step(_world_at(5.0, near))                                 # 1 m näher: Zähler von vorn
    assert b.stuck_n == 0 and not b.gave_up


def test_map_explore_home_stuck_finishes_with_message():
    from xgonav.behaviors import MapExplore
    cfg = Config()
    b = MapExplore(cfg.behavior, cfg.robot, cfg.home, cfg.explore)
    pose = (1.0, 0.0, math.pi)
    b.state, b.reason = "heim", "Karte fertig"
    b.f.set_path([pose[:2], (0.0, 0.0)])
    b.step(_world_at(0.0, pose))
    class P:
        def request(self, *a, **k):
            return 1
    for k in range(cfg.home.max_fail):
        b.f.stuck = True
        b.request_plan(P(), None, _world_at(float(k + 1), pose), None)
    b.step(_world_at(9.0, pose))
    assert not b.done
    c = b.step(_world_at(cfg.home.give_up_s + 1.0, pose))
    assert b.done and b.state == "fertig" and c.v == 0 and "nicht erreicht" in c.status


def test_localmap_confirm_cache_matches_exact_and_later():
    """Zwischenspeicher: gleiches Ergebnis wie frisch gerechnet, auch nach move() und mit wachsender Reichweite."""
    from xgonav.localmap import LocalMap
    geo = CamGeometry(640, 480, 88.5, 5.18, 43.0, 85.0)
    rng = np.random.default_rng(3)
    lm = LocalMap(geo, 160, 120, max_age_s=99.0, confirm=3, confirm_radius_m=0.06)
    lm.pts = rng.uniform(-1.2, 1.2, (400, 2))
    lm.t = np.zeros(400)
    lm.obs = rng.integers(0, 6, 400)

    def fresh(reach):
        f = LocalMap(geo, 160, 120, max_age_s=99.0, confirm=3, confirm_radius_m=0.06)
        f.pts, f.t, f.obs = lm.pts.copy(), lm.t.copy(), lm.obs.copy()
        return f.confirmed_points(reach)
    for reach, ds, dy in ((0.4, 0.0, 0.0), (0.4, 0.05, 0.1), (1.0, 0.1, -0.2), (0.6, 0.2, 0.3), (1.5, 0.0, 0.0)):
        lm.move(ds, dy)
        a, b = lm.confirmed_points(reach), fresh(reach)
        assert a.shape == b.shape and np.allclose(a, b)
        assert np.allclose(lm.confirmed_later(reach)(), b)
    lm.obs = lm.obs.copy()
    lm.pts, lm.t, lm.obs = lm.pts[:200], lm.t[:200], lm.obs[:200]        # neue Punktmenge: alter Stand gilt nicht
    a, b = lm.confirmed_points(1.0), fresh(1.0)
    assert a.shape == b.shape and np.allclose(a, b)


def _driven_room():
    """Raum mit Fleckenreihe quer: Mit Abstand kommt nur durch, wer weiß, dass er dort gefahren ist."""
    from xgonav.gridmap import GridMap
    from dataclasses import replace
    cfg = Config()
    geo = CamGeometry(640, 480, 88.5, 5.18, 43.0, 85.0)
    m = GridMap(replace(cfg.grid, native=False, file=""), geo, 160, 120)
    r0, c0 = (int(k) for k in m.cell(-2.0, -1.0))
    r1, c1 = (int(k) for k in m.cell(2.0, 1.0))
    m.lo[r0 - 1:r1 + 1, c0 - 1:c1 + 1] = cfg.grid.l_max                 # geschlossene Wände
    m.lo[r0:r1, c0:c1] = cfg.grid.l_min
    for y in np.arange(-1.0, 1.0, 0.3):                              # Flecken bei x = 0 alle 30 cm
        r, c = m.cell(0.0, float(y))
        m.lo[int(r), int(c)] = cfg.grid.l_max
    return cfg, m


def test_planner_passes_driven_path_through_specks():
    from xgonav.planner import plan
    cfg, m = _driven_room()
    assert plan(m, (-1.0, 0.0), (1.0, 0.0), cfg.home) is None
    m.path = [(-1.0, 0.15), (0.0, 0.15), (1.0, 0.15)]
    assert plan(m, (-1.0, 0.15), (1.0, 0.15), cfg.home) is not None


def test_driven_path_stays_closed_for_fresh_obstacles():
    from xgonav.planner import cost_map, driven_mask
    cfg, m = _driven_room()
    m.path = [(-1.0, 0.15), (0.0, 0.15), (1.0, 0.15)]
    lo = m.lo
    driven = driven_mask(m, 0, 0, lo.shape)
    r, c = (int(k) for k in m.cell(0.0, 0.15))
    assert driven[r, c]
    blocked, _ = cost_map(lo, m.res, m.cfg.l_show, cfg.home, None, driven)
    assert not blocked[r, c]
    extra = np.zeros(lo.shape, bool)
    extra[r, c] = True                                               # Merkkarte sieht dort jetzt etwas
    blocked, _ = cost_map(lo, m.res, m.cfg.l_show, cfg.home, extra, driven)
    assert blocked[r, c]


def test_plan_falls_back_without_localmap():
    """Schließt nur die Merkkarte den Weg, wird trotzdem geplant (die Bahnprüfung bremst); die Karte allein sperrt."""
    from xgonav.planner import plan
    cfg, m = _driven_room()
    m.path = [(-1.0, 0.15), (0.0, 0.15), (1.0, 0.15)]
    wall = np.column_stack([np.zeros(41), np.linspace(-1.0, 1.0, 41)])
    assert plan(m, (-1.0, 0.15), (1.0, 0.15), cfg.home, obstacles=wall) is not None
    assert plan(m, (-1.0, 0.15), (1.0, 0.15), cfg.home, obstacles=lambda: wall) is not None   # auch als Funktion
    m.path = [(0.0, 0.0)]
    assert plan(m, (-1.0, 0.15), (1.0, 0.15), cfg.home, obstacles=wall) is None


def test_pathfollower_detects_pressing_quickly():
    from xgonav.behaviors import PathFollower, World
    from xgonav.egomotion import EgoResult
    cfg = Config()
    f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
    f.set_path([(0.0, 0.0), (3.0, 0.0)])

    def step(t, x, v):
        return f.drive(World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[], free=None, balancing=True,
                             yaw=0.0, cmd_v=v, pose=(x, 0.0, 0.0), goal=(3.0, 0.0, 0.0)), "x")
    for k in range(21):                                              # fährt: nichts
        step(k * 0.1, k * 0.015, 0.15)
    assert not f.pressing
    for k in range(21, 36):                                          # gebremst, Befehl klein: kein Kontakt
        step(k * 0.1, 0.3, 0.02)
    assert not f.pressing
    for k in range(36, 70):                                          # Befehl groß, Räder stehen
        step(k * 0.1, 0.3, 0.15)
    assert f.pressing and f.stuck


def test_map_explore_contact_blocks_goal_at_once():
    from xgonav.behaviors import MapExplore
    cfg = Config()
    b = MapExplore(cfg.behavior, cfg.robot, cfg.home, cfg.explore)
    pose = (1.0, 0.0, 0.0)
    b.state, b.frontier, b.goal, b.t_goal = "ziel", (3.0, 0.0), (2.7, 0.0), 0.0
    b.f.set_path([pose[:2], (2.7, 0.0)])
    b.f.stuck = b.f.pressing = True
    c = b.step(_world_at(1.0, pose))
    assert b.state == "backup" and b.blocked_goals == [(3.0, 0.0)] and c.v == 0 and not b.f.pressing


def test_grid_heading_from_imu():
    app = _app(freespace=True)
    assert app._grid_dyaw(0.5) == 0.5                                  # Default: Räder
    app.cfg.grid.yaw_source = "imu"
    app.robot.state.yaw_deg = 179.0
    assert app._grid_dyaw(0.5) == 0.0                                  # erster Wert: nur merken
    app.robot.state.yaw_deg = -179.0                                   # über die Naht: +2°
    assert abs(app._grid_dyaw(0.5) - math.radians(2.0)) < 1e-9
    app.robot.state.yaw_deg = None                                     # keine IMU: Rückfall
    assert app._grid_dyaw(0.5) == 0.5


def test_home_stuck_near_goal_counts_as_arrived():
    from xgonav.behaviors import Home, MapExplore
    cfg = Config()
    pose = (0.28, 0.0, math.pi)
    h = Home(cfg.behavior, cfg.robot, cfg.home)
    assert h.want_plan(0.0)
    h.set_path([pose[:2], (0.0, 0.0)])
    h.f.stuck = True
    h.pending = False
    assert h.want_plan(1.0)
    c = h.step(_world_at(1.0, pose))
    assert h.done and c.v == 0 and "angekommen" in c.status
    m = MapExplore(cfg.behavior, cfg.robot, cfg.home, cfg.explore)
    m.state, m.reason, m.home_stuck = "heim", "Karte fertig", 1
    c = m.step(_world_at(1.0, pose))
    assert m.done and c.v == 0 and "daneben" in m.reason


def _patrol(points, loop=False):
    from xgonav.behaviors import Patrol
    cfg = Config()
    b = Patrol(cfg.behavior, cfg.robot, cfg.home, cfg.patrol)
    b.set_route(points, loop)
    return cfg, b


def test_patrol_visits_points_and_returns():
    cfg, b = _patrol([(1.0, 0.0), (1.0, 1.0)])
    assert b.want_plan(0.0)
    b.set_path([(0.0, 0.0), (1.0, 0.0)])
    assert b.step(_world_at(0.0, (0.0, 0.0, 0.0))).v > 0
    c = b.step(_world_at(5.0, (0.95, 0.0, 0.0)))
    assert c.v == 0 and b.state == "schau" and b.reached == 1
    b.step(_world_at(5.0 + cfg.patrol.pause_s + 0.1, (0.95, 0.0, 0.0)))
    assert b.state == "fahrt" and b.idx == 1
    b.set_path([(0.95, 0.0), (1.0, 1.0)])
    b.step(_world_at(8.0, (1.0, 0.95, 0.0)))
    b.step(_world_at(8.0 + cfg.patrol.pause_s + 0.1, (1.0, 0.95, 0.0)))
    assert b.state == "heim" and b.reason == "Runde fertig"
    b.set_path([(1.0, 0.95), (0.0, 0.0)])
    b.step(_world_at(12.0, (0.05, 0.0, 0.0)))
    assert b.done and "am Start" in b.reason


def test_patrol_skips_stuck_point_and_loops():
    cfg, b = _patrol([(2.0, 0.0), (0.0, 2.0)], loop=True)
    pose = (0.0, 0.0, 0.0)
    b.set_path([pose[:2], (2.0, 0.0)])
    b.f.stuck = True
    c = b.step(_world_at(1.0, pose))
    assert c.v == 0 and b.f.path is None and b.state == "fahrt"            # erst neu planen
    b.set_path([pose[:2], (2.0, 0.0)])
    b.f.stuck = True
    b.step(_world_at(2.0, pose))
    assert b.state == "backup" and b.skipped == [1]
    assert b.step(_world_at(2.1, pose)).v < 0                               # zurücksetzen
    b.step(_world_at(3.0, pose))
    assert b.state == "fahrt" and b.idx == 1
    b.set_path([pose[:2], (0.0, 2.0)])
    b.step(_world_at(4.0, (0.0, 1.9, 0.0)))
    assert b.state == "schau"
    b.step(_world_at(4.0 + cfg.patrol.pause_s + 0.1, (0.0, 1.9, 0.0)))
    assert b.state == "fahrt" and b.idx == 0 and b.lap == 2 and b.skipped == []


def test_patrol_all_unreachable_goes_home():
    cfg, b = _patrol([(2.0, 0.0)])
    pose = (1.0, 0.0, 0.0)
    for _ in range(cfg.home.max_fail):
        b.pending = True
        b.set_path(None)
    b.step(_world_at(1.0, pose))
    assert b.state == "backup"
    b.step(_world_at(2.0, pose))
    assert b.state == "heim" and b.reason == "kein Punkt erreichbar"


def test_patrol_battery_home_and_set_down():
    cfg, b = _patrol([(2.0, 0.0)])
    w = _world_at(1.0, (0.0, 0.5, 0.0))
    w.battery = cfg.patrol.return_battery
    b.step(w)
    assert b.state == "heim" and b.reason == "Akku"
    w = _world_at(2.0, (0.0, 0.5, 0.0))
    w.battery = cfg.patrol.down_battery
    c = b.step(w)
    assert b.done and b.want_set_down and c.v == 0


def _route_map():
    from xgonav.gridmap import GridMap
    from dataclasses import replace
    cfg = Config()
    geo = CamGeometry(640, 480, 88.5, 5.18, 43.0, 85.0)
    m = GridMap(replace(cfg.grid, native=False, file=""), geo, 160, 120)
    r0, c0 = (int(k) for k in m.cell(-3.0, -3.0))
    r1, c1 = (int(k) for k in m.cell(3.0, 3.0))
    m.lo[r0 - 1:r1 + 1, c0 - 1:c1 + 1] = cfg.grid.l_max
    m.lo[r0:r1, c0:c1] = cfg.grid.l_min
    return cfg, m


def test_route_check_messages():
    from xgonav.route import check
    cfg, m = _route_map()
    assert check(m, cfg.home, [(1.0, 1.0), (-1.0, 2.0)], []) == []
    assert "nicht auf bekanntem Boden" in check(m, cfg.home, [(5.0, 5.0)], [])[0]
    assert "zu nah" in check(m, cfg.home, [(2.95, 0.0)], [])[0]
    assert "Sperrzone" in check(m, cfg.home, [(1.0, 1.0)], [[(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)]])[0]
    wall = int(m.cell(1.5, 0.0)[1])
    m.lo[:, wall - 1:wall + 2] = cfg.grid.l_max
    assert "nicht erreichbar" in check(m, cfg.home, [(2.5, 0.0)], [])[0]


def test_route_file_belongs_to_map(tmp_path):
    from xgonav.route import load, save
    cfg, m = _route_map()
    f = str(tmp_path / "karte.npz")
    m.route, m.route_loop, m.zones = [(1.0, 1.0), (2.0, 0.5)], True, [[(0.0, 2.0), (1.0, 2.0), (1.0, 2.5)]]
    assert m.save(f) and save(f, m)
    n = _route_map()[1]
    assert n.load(f) and load(f, n)
    assert n.map_id == m.map_id and n.route == m.route and n.route_loop and n.zones == m.zones
    o = _route_map()[1]                                                 # andere Karte
    assert not load(f, o) and o.route == []


def test_parse_route_text():
    from xgonav.route import parse_points, parse_zones
    assert parse_points("1,2;-0.5,3") == [(1.0, 2.0), (-0.5, 3.0)] and parse_points("") == []
    assert len(parse_zones("0,0;1,0;1,1|2,2;3,2;3,3")) == 2
    for bad in ("1", "a,b", "1,2,3", "nan,1"):
        with pytest.raises(ValueError):
            parse_points(bad)
    with pytest.raises(ValueError):
        parse_zones("0,0;1,0")


def test_locked_map_does_not_change():
    from xgonav.freespace import FreeSpace
    cfg, m = _route_map()
    geo = m.geo
    hz = geo.horizon_row(0) / 4
    fs = FreeSpace([0.5] * 5, np.full(160, 0.4, np.float32), hz, np.zeros((120, 160), np.uint8), True, None)
    m.lock()
    before = m.lo.copy()
    m.add_freespace(fs, 0.0, (0.0, 0.0, 0.0))
    m.move(0.3, 0.0)
    assert np.array_equal(m.lo, before) and m.x > 0.2                   # Pose läuft weiter


def test_zone_blocks_planner_frontier_and_arc():
    from xgonav.frontier import next_goal
    from xgonav.planner import plan
    from xgonav.route import arc_hits_zone
    cfg, m = _route_map()
    zone = [(1.0, -4.0), (1.5, -4.0), (1.5, 4.0), (1.0, 4.0)]            # Band quer durch den Raum
    assert plan(m, (0.0, 0.0), (2.5, 0.0), cfg.home) is not None
    m.zones = [zone]
    assert plan(m, (0.0, 0.0), (2.5, 0.0), cfg.home) is None
    assert plan(m, (0.0, 0.0), (0.5, 1.0), cfg.home) is not None
    r0, c0 = (int(k) for k in m.cell(2.0, -3.0))
    m.lo[r0:, :] = 0.0                                                  # dahinter unbekannt
    r = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert r.path is None or all(p[0] < 1.0 for p in r.path)
    assert arc_hits_zone((0.5, 0.0, 0.0), 0.27, 0.0, [zone], 0.18)      # fährt darauf zu
    assert not arc_hits_zone((0.5, 0.0, 0.0), -0.27, 0.0, [zone], 0.18)  # fährt weg
    assert not arc_hits_zone((0.9, 0.0, 0.0), -0.1, 0.0, [zone], 0.18)  # nah dran, entfernt sich
    assert arc_hits_zone((0.9, 0.0, 0.0), 0.1, 0.0, [zone], 0.18)
    assert not arc_hits_zone((-2.0, 0.0, 0.0), 0.27, 0.0, [zone], 0.18)


def test_app_checks_patrol_start_and_locks_explore():
    from xgonav.app import ModeNotAllowed
    app = _app(freespace=True)
    gm = app.gridmap
    with pytest.raises(ModeNotAllowed, match="Route"):
        app.set_mode("patrol", confirmed=True)
    gm.route = [(1.0, 0.0)]
    app.robot.state.battery = 100
    with pytest.raises(ModeNotAllowed, match="bestätigen"):
        app.set_mode("patrol")
    app.robot.state.battery = app.cfg.patrol.start_battery - 1
    with pytest.raises(ModeNotAllowed, match="Akku"):
        app.set_mode("patrol", confirmed=True)
    app.robot.state.battery = 100
    app.robot.state.balancing = False
    with pytest.raises(ModeNotAllowed, match="stehen"):
        app.set_mode("patrol", confirmed=True)
    app.robot.state.balancing = True
    gm.move(2.0, 0.3)
    app.set_mode("patrol", confirmed=True)
    assert gm.locked and gm.pose() == gm.start and gm.path == [gm.start[:2]]   # Pose springt zum Start
    with pytest.raises(ModeNotAllowed, match="gesperrt"):
        app.set_mode("explore")


def test_arc_check_stops_at_zone():
    from xgonav.behaviors import Command
    app = _app(freespace=True)
    app.gridmap.zones = [[(0.2, -1.0), (0.6, -1.0), (0.6, 1.0), (0.2, 1.0)]]
    c = app._arc_check(Command(0.15, 0.0, 100.0, "x"))
    assert c.v == 0 and "Sperrzone" in c.status
    c = app._arc_check(Command(-0.1, 0.0, 100.0, "x"))
    assert "Sperrzone" not in c.status
