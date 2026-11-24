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
from xgonav.sim import SimFloorNet, SimWorld  # noqa: E402
from xgonav.tracker import Tracker  # noqa: E402
from xgonav.detector import Detection  # noqa: E402


# ---------------------------------------------------------------- Robot
def test_u8_mapping():
    assert norm_to_u8(0) == 128
    assert norm_to_u8(1) == 255 and norm_to_u8(-1) == 1
    assert norm_to_u8(5) == 255
    assert abs(u8_to_norm(norm_to_u8(0.3)) - 0.3) < 0.01


def test_robot_ramp_clamp_and_watchdog():
    cfg = Config.neutral().robot
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


def test_action_cmds_abschnitte():
    """Fahrbefehle der Firmware-Aktionen: laufender Abschnitt plus die, die bald beginnen können."""
    from xgonav.robot import action_cmds
    vor, rueck = u8_to_norm(192), u8_to_norm(64)
    assert action_cmds(3, 0.0, 0.08) == [(vor, 0.0)]
    assert action_cmds(3, 0.5, 0.08) == [(vor, 0.0), (rueck, 0.0)]      # Wechsel bei Takt 8: 0,50–0,64 s
    assert action_cmds(3, 1.0, 0.08) == [(rueck, 0.0)]
    assert all(v == 0.0 and w != 0.0 for v, w in action_cmds(5, 2.0, 0.08))   # 5 dreht nur
    assert len(action_cmds(4, 0.0, 0.08)) == 1 and len(action_cmds(4, 1.0, 0.08)) == 2


def _speed_writes(r, t0):
    return [(t, n, v) for t, n, v in r.writes if t > t0 and n in ("speed_x", "speed_z")]


def test_robot_action_setzt_keepalive_aus():
    """Während einer Fahr-Aktion der Firmware schreibt Robot keine Sollwerte (sonst hält der Keepalive sie
    an) und der Kernel-Watchdog ist aus; danach läuft beides wieder."""
    cfg = Config.neutral().robot
    cfg.action_tick_s = 0.01                     # Aktion 3: 20 Takte = 0,2 s + 0,3 s Reserve
    r = SimRobot(cfg)
    r.settings["motion_timeout_ms"] = "1000"
    r.start()
    try:
        time.sleep(0.1)
        assert _speed_writes(r, 0.0)
        t_end = r.action_start(3)
        t0 = time.monotonic()
        assert r.action is not None and r.action[0] == 3
        time.sleep(0.35)                         # länger als keepalive_s
        assert _speed_writes(r, t0) == []
        assert r.settings["motion_timeout_ms"] == "0"
        assert ("action", 3) in [(n, v) for _, n, v in r.writes]
        time.sleep(max(0.0, t_end - time.monotonic()) + 0.15)
        assert r.action is None
        assert _speed_writes(r, t_end - 1e-3)    # Windows-Uhr rastet auf 15,6 ms: Zeitstempel kann gleich t_end sein
        assert r.settings["motion_timeout_ms"] == str(cfg.kernel_watchdog_ms)
    finally:
        r.close()


def test_robot_action_abbruch_schreibt_stillstand():
    """Abbruch: bis zum Ende der Aktion in jedem Durchlauf 128/128, damit ihr Fahrwert nicht gilt."""
    cfg = Config.neutral().robot
    cfg.action_tick_s = 0.01                     # Aktion 4: 0,6 s + Reserve
    r = SimRobot(cfg)
    r.start()
    try:
        r.action_start(4)
        time.sleep(0.1)
        r.action_abort()
        assert r.action is None
        t0 = time.monotonic()
        time.sleep(0.25)
        w = _speed_writes(r, t0)
        assert len([1 for _, n, _ in w if n == "speed_x"]) >= 5       # 50 Hz statt alle 0,3 s
        assert all(v == 128 for _, _, v in w)
    finally:
        r.close()


def test_robot_height_mapping():
    r = SimRobot(Config.neutral().robot)
    r.set_height(60)
    r.set_height(90)
    r.set_height(120)   # begrenzt auf height_limit_u8
    r.set_height(500)   # begrenzt, identisch -> kein neuer Write
    vals = [v for _, n, v in r.writes if n == "height"]
    assert vals == [0, 128, 205]
    assert r.height_mm == pytest.approx(60 + 60 * 200 / 255)   # die Firmware fährt höchstens 200


def test_robot_led_start_flash(tmp_path):
    """Start: kurz 255, danach die gespeicherte Helligkeit; ohne Datei led_default (20 %)."""
    import dataclasses
    import time
    from xgonav.robot import ProcRobot

    (tmp_path / "settings").mkdir()
    (tmp_path / "settings" / "led_brightness").write_text("-1")
    gespeichert = tmp_path / "led_gespeichert"
    d = str(Config.neutral().robot.led_default)
    for datei, erwartet in ((None, d), ("77\n", "77"), ("kaputt", d)):
        if datei is not None:
            gespeichert.write_text(datei)
        r = ProcRobot(dataclasses.replace(Config.neutral().robot, proc_dir=str(tmp_path), led_file=str(gespeichert),
                                          led_start_s=0.05))
        writes = []
        r._write_raw = lambda name, text: writes.append((name, text))   # /proc kürzt, eine Datei nicht
        r._led_start()
        assert writes == [("settings/led_brightness", "255")]
        time.sleep(0.3)
        assert writes == [("settings/led_brightness", "255"), ("settings/led_brightness", erwartet)]


def test_robot_height_readback(tmp_path):
    """Höhe aus /proc: Web-Änderungen übernehmen, nach dem Aufstehen gilt die Mittelstellung der Firmware,
    der alte Inhalt von /proc nicht mehr; danach schreibt das Verhalten seine Höhe neu."""
    import dataclasses
    from xgonav.robot import ProcRobot

    def setze(name, text):                               # wie ein Schreiber von außen (Web, Treiber)
        (tmp_path / name).write_text(text)

    for name, text in (("height", ""), ("state", "1"), ("yaw", "0"), ("battery", "90"), ("buttons", "0")):
        setze(name, text)
    r = ProcRobot(dataclasses.replace(Config.neutral().robot, proc_dir=str(tmp_path)))
    mm = r.height_u8_to_mm

    def takt():
        r._poll_state()
        r._height_tick()

    takt()
    assert r.height_mm is None                           # leer: noch nie geschrieben
    r.set_height(85)
    assert (tmp_path / "height").read_text() == "106" and r.height_mm == pytest.approx(mm(106))
    takt()
    assert r.height_mm == pytest.approx(mm(106))
    setze("height", "203")                               # Web während der Fahrt
    takt()
    assert r.height_mm == pytest.approx(mm(200))         # Firmware begrenzt auf 200
    r.set_height(85)                                     # Verhalten will weiter 85: kein neuer Write
    assert (tmp_path / "height").read_text() == "203" and r.height_mm == pytest.approx(mm(200))
    setze("state", "254")                                # absetzen ...
    takt()
    setze("state", "1")                                  # ... und aufstehen
    takt()
    assert r.height_mm == pytest.approx(mm(128))
    takt()
    assert r.height_mm == pytest.approx(mm(128))         # "203" in /proc ist veraltet
    r.set_height(85)
    assert (tmp_path / "height").read_text() == "106" and r.height_mm == pytest.approx(mm(106))


def test_app_geometry_follows_robot_height():
    """Die Bodengeometrie rechnet mit dem gültigen Höhen-Sollwert, auch wenn ihn nicht xgonav setzte."""
    import threading
    from xgonav.app import App
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
    app.imu = None
    th = threading.Thread(target=app.run, kwargs={"duration": 4.0}, daemon=True)
    th.start()
    try:
        time.sleep(1.0)
        assert app.geo.body_height_mm == pytest.approx(cfg.behavior.height_cruise_mm)
        robot._height_mm = 107.0                         # wie von Robot._poll_height übernommen
        time.sleep(0.6)
        assert app.geo.body_height_mm == pytest.approx(107.0)
        assert app.geo.cam_h == pytest.approx((107.0 + cfg.camera.mount_offset_mm) / 1000)
        assert app.snapshot.stats["height_mm"] == pytest.approx(107.0)
    finally:
        app.stop(); th.join(timeout=5)


# ---------------------------------------------------------------- Ego-Motion
def test_egomotion_rotation_vs_sim():
    cfg = Config.neutral()
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
        img, _, _ = w._render(t, 0.05)
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
    cfg = Config.neutral()
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
        img, _, _ = w._render(t, 0.05)
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
    cfg = Config.neutral()
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


def test_cam_h_from_height_u8():
    """[camera] cam_h_u8: Kamerahöhe als Gerade über den Höhen-Sollwert (Messung 2026-11-07), leer = alt."""
    from xgonav.robot import FW_HEIGHT_STANDUP_U8
    cfg = Config.neutral()
    cfg.camera.cam_h_u8 = [109.4, 0.1126]
    g = CamGeometry.from_cfg(cfg)
    for u, soll in ((106, 121.34), (150, 126.29), (200, 131.92), (FW_HEIGHT_STANDUP_U8, 123.81)):
        g.body_height_mm = cfg.robot.height_min_mm + (cfg.robot.height_max_mm - cfg.robot.height_min_mm) * u / 255
        assert g.cam_h * 1000 == pytest.approx(soll, abs=0.01)
    cfg.camera.cam_h_u8 = []
    g = CamGeometry.from_cfg(cfg, 85.0)
    assert g.cam_h * 1000 == pytest.approx(85.0 + cfg.camera.mount_offset_mm)


@pytest.mark.parametrize("gerade", [[100.8, 0.1037], []])
def test_cam_h_display_matches_geometry(gerade):
    """Anzeige im Web: Gerade und cam_h_at müssen dasselbe ergeben wie die Bodengeometrie (cam_h)."""
    from xgonav.robot import SimRobot
    cfg = Config.neutral()
    cfg.camera.cam_h_u8 = gerade
    g = CamGeometry.from_cfg(cfg)
    r = SimRobot(cfg.robot)
    a, b = g.cam_h_line()
    for u in (55, 106, 128, 150, 200):
        g.body_height_mm = r.height_u8_to_mm(u)
        assert g.body_to_u8(g.body_height_mm) == u
        assert g.cam_h_at(g.body_height_mm) == pytest.approx(g.cam_h * 1000)
        assert a + b * u == pytest.approx(g.cam_h * 1000)
    if gerade:
        g.body_height_mm = r.height_u8_to_mm(128)
        assert g.cam_h * 1000 == pytest.approx(114.07, abs=0.01)


def test_status_camera_height_fields():
    """/status: Kamerahöhe aus dem gültigen Sollwert, null solange keiner bekannt ist."""
    import json
    import urllib.request
    from xgonav.app import App
    from xgonav.web import serve
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    cfg.camera.cam_h_u8 = [100.8, 0.1037]
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
    srv = serve(app, "127.0.0.1", 0, 5)
    try:
        def status():
            with urllib.request.urlopen("http://127.0.0.1:%d/status" % srv.server_address[1], timeout=3) as r:
                return json.load(r)
        robot._height_mm = None
        s = status()
        assert s["cam_h_gerade"] == [100.8, 0.1037]
        assert s["height_u8"] is None and s["cam_h_mm"] is None
        robot._height_mm = robot.height_u8_to_mm(128)
        s = status()
        assert s["height_u8"] == 128 and s["cam_h_mm"] == pytest.approx(114.1)
    finally:
        srv.shutdown(); srv.server_close(); app.stop()


def test_startbild_endpunkt():
    """/startbild.jpg: 404 ohne Anker, sonst JPEG; /status startbild live (auch ohne neues Bild)."""
    import json
    import urllib.error
    import urllib.request
    import numpy as np
    from xgonav import startbild as sbm
    from xgonav.app import App
    from xgonav.web import serve
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
    app.startbild = sb = sbm.StartAbgleich(cfg.startbild)
    srv = serve(app, "127.0.0.1", 0, 5)
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    try:
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + "/startbild.jpg", timeout=3)
        assert e.value.code == 404
        a = sbm.Ansicht(np.zeros((480, 640), np.uint8), (0.0, 0.0, 0.0), 0.0, 0.0, 310.0, 320.0, 240.0, 5.0, 0.12, 0.036)
        sb.anker_setzen(a, (0.0, 0.0, 0.0), np.full((480, 640, 3), 90, np.uint8))
        sb.anker_bereit = True
        with urllib.request.urlopen(base + "/startbild.jpg", timeout=3) as r:
            assert r.headers["Content-Type"] == "image/jpeg" and r.read()[:2] == bytes((0xFF, 0xD8))
        with urllib.request.urlopen(base + "/status", timeout=3) as r:
            assert json.load(r)["startbild"]["bild"] == 1
    finally:
        srv.shutdown(); srv.server_close(); app.stop()


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
    tr = Tracker(Config.neutral().tracker, (640, 480))
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
    cfg = Config.neutral().tracker
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
    cfg = Config.neutral().imu
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
    from xgonav.imu import imu_frame
    for sign, off in ((1, 0), (-1, 12), (1, -36)):
        s, imu, pitch = _sync_scenario(sign, off)
        assert s.reliable, (sign, off, s.corr)
        assert s.sign == sign
        assert abs(s.offset * 1000 - off) <= 4
        s.cfg.pitch_ref_deg = 2.0
        s.cfg.max_age_s = 1e9                  # synthetische Zeitbasis
        for t in (3.0, 5.0, 7.2):
            assert math.degrees(abs(imu_frame(s, imu, t).pitch - pitch(t))) < 0.3


def test_imu_alter_im_liegen():
    """Abgesetzt (idle, 5 Hz) ist die neueste Probe bis ~0,4 s alt: Nick und Roll gelten dann bis 2,5 Abstände
    (0,5 s) statt max_age_s (0,3 s); sonst fiel jedes vierte Bild auf Nick 0 (Horizontlinie sprang, 2026-11-17)."""
    from xgonav.imu import ImuSource, ImuSync, imu_frame, max_age
    cfg = Config.neutral().imu
    cfg.pitch_sign, cfg.time_offset_ms, cfg.idle_hz, cfg.max_age_s = -1, 0.0, 5, 0.3
    cfg.use_roll = True
    imu = ImuSource(cfg)
    for k in range(10):                        # 5 Hz bis t = 1,8 s
        imu.add(k, k * 0.2, math.radians(2.0), math.radians(-20.0))
    sync = ImuSync(cfg)
    fr = imu_frame(sync, imu, 2.2)
    assert fr.pitch is None and fr.roll_deg == 0.0                                       # balancierend: 0,4 s zu alt
    imu.idle = True
    assert max_age(cfg, imu) == 0.5
    fr = imu_frame(sync, imu, 2.2)
    assert fr.pitch is not None and abs(fr.roll_deg - 2.0) < 1e-6
    assert imu_frame(sync, imu, 2.4).pitch is None                                           # 0,6 s: zu alt


def test_imu_sync_needs_motion():
    s, _, _ = _sync_scenario(1, 0, still=True)
    assert not s.reliable


# ---------------------------------------------------------------- App
def _app(freespace: bool, mode: str = "idle"):
    from xgonav.app import App
    cfg = Config.neutral()
    cfg.freespace.enabled = freespace
    cfg.behavior.mode = mode
    cfg.output.web_port = 0
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None)
    return App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))


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


def test_fw_action_nur_wenn_erlaubt():
    """Fahr-Aktionen der Firmware: nur idle, balancierend, 3/4 nur mit Bodengrenze; 5 dreht nur."""
    from xgonav.app import ModeNotAllowed
    from xgonav.behaviors import Command
    app = _app(freespace=True)
    with pytest.raises(KeyError):
        app.fw_action(1)
    app._free = None
    with pytest.raises(ModeNotAllowed):
        app.fw_action(3)                          # Bodengrenze noch nicht da
    app.robot.state.balancing = False
    with pytest.raises(ModeNotAllowed):
        app.fw_action(5)
    app.robot.state.balancing = True
    app.behavior = app._make("manual")
    with pytest.raises(ModeNotAllowed):
        app.fw_action(5)
    app.behavior = app._make("idle")
    app.fw_action(5)
    with pytest.raises(ModeNotAllowed):
        app.fw_action(5)                          # schon angefragt
    app._action_tick(None, Command(), time.monotonic())      # ohne Bildverarbeitung: 5 startet trotzdem
    assert app.robot.action is not None and app.robot.action[0] == 5
    with pytest.raises(ModeNotAllowed):
        app.fw_action(5)                          # läuft noch
    app.set_mode("idle")                          # wie POST /stop
    assert app.robot.action is None


def test_fw_action_bahnpruefung():
    """Die Bahnprüfung gilt für den Fahrbefehl der Aktion: gesperrt startet sie nicht, später bricht sie ab."""
    from types import SimpleNamespace
    from xgonav.behaviors import Command
    app = _app(freespace=True)
    app.gridmap = None

    class Karte:
        radius = 0.15
        frei = 0.05

        def arc_clearance(self, *a, **k):
            return self.frei
    app.lmap = Karte()
    app._free = object()
    w = SimpleNamespace(balancing=True, blind=False, free=None, ego=SimpleNamespace(ttc=99.0), cmd_v=0.0)
    now = time.monotonic()

    app.fw_action(3)
    cmd = app._action_tick(w, Command(), now)
    assert "nicht gestartet" in cmd.status and "Bahn" in cmd.status
    assert app.robot.action is None and not any(n == "action" for _, n, _ in app.robot.writes)

    app.lmap.frei = 9.0
    app.fw_action(3)
    cmd = app._action_tick(w, Command(), now)
    assert app.robot.action is not None and "Aktion 3" in cmd.status
    cmd = app._action_tick(w, Command(), now)
    assert app.robot.action is not None           # frei: läuft weiter

    app.lmap.frei = 0.05
    cmd = app._action_tick(w, Command(), now)
    assert app.robot.action is None and "abgebrochen" in cmd.status

    # blind (FloorNet ausgefallen) hält auch an
    app.robot._action_end()
    app.lmap.frei = 9.0
    app.fw_action(3)
    app._action_tick(w, Command(), now)
    assert app.robot.action is not None
    w.blind = True
    cmd = app._action_tick(w, Command(), now)
    assert app.robot.action is None and "BLIND" in cmd.status


# ---------------------------------------------------------------- Linsenabschattung
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
    lm = fresh(); lm.add_freespace(FreeSpace([1.0] * 5, free_cols, horizon, True), 0.0, 0.0)
    assert len(lm.pts) == 0
    # Mitte erst ab 0,5 m messbar, linke Hälfte gar nicht: nur der ferne Punkt vorn verschwindet
    span = fh - horizon
    row_05 = fh - next(r for r in range(fh, 0, -1) if (geo.ground_dist(r * 4) or 9) >= 0.5) / 1
    kf = np.full(fw, row_05 / span)
    kf[: fw // 2 - 10] = 1.0
    lm = fresh(); lm.add_freespace(FreeSpace([1.0] * 5, free_cols, horizon, True, kf), 0.0, 0.0)
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
    fs = FreeSpace([1.0] * 5, cols, 60.0, True)
    for i in range(2):
        lm.add_freespace(fs, i * 0.1, 0.0)
        assert lm.arc_clearance(0.15, 0.0, 1.2) == math.inf, "nach %d Beobachtungen schon bestätigt" % (i + 1)
    lm.move(0.0, 0.05)                                 # starre Bewegung ändert nichts an der Zählung
    lm.add_freespace(fs, 0.3, 0.0)
    assert lm.arc_clearance(0.15, 0.0, 1.2) < math.inf


def test_pipeline_pause_and_resume():
    """Bildverarbeitung stoppen: keine Frames mehr, Stillstand, Modus idle; starten: läuft weiter."""
    import threading
    from xgonav.app import App
    from xgonav.sim import Person
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, Person([(3.0, 1.5), (3.5, 3.0)], speed=0.3), start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
    app.imu = None
    from xgonav.sim import FakeDetector
    app.det = FakeDetector(world)                       # follow braucht einen Detektor
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
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
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


def test_manual_checked_uses_safety():
    """Geprüfte Handsteuerung (/drive?sicher=1, MCP): safety() gilt wie in den anderen Modi. Ohne Bodengrenze
    (blind) steht er; derselbe Sollwert ungeprüft fährt. Ohne Bildverarbeitung dreht geprüft nur."""
    import threading
    from xgonav.app import App
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=None)        # kein Netz: blind
    app.imu = None
    th = threading.Thread(target=app.run, kwargs={"duration": 8.0}, daemon=True)
    th.start()
    time.sleep(1.0)
    app.set_mode("manual")
    time.sleep(0.3)

    def halten(v, w, checked, sek=0.8):
        t_end = time.monotonic() + sek
        while time.monotonic() < t_end:
            app.set_manual(v, w, checked=checked)
            time.sleep(0.1)

    halten(1.0, 0.5, True)
    assert robot.command == (0.0, 0.0) and "BLIND" in app._cmd.status
    halten(1.0, 0.0, False)
    assert robot.command[0] == pytest.approx(cfg.robot.max_v, abs=1e-6)
    app.set_pipeline(False)                                       # schaltet auf idle
    time.sleep(0.3)
    app.set_mode("manual")
    time.sleep(0.3)
    halten(1.0, 1.0, True)
    assert robot.command[0] == 0.0 and robot.command[1] == pytest.approx(cfg.robot.max_w, abs=1e-6)
    assert "keine Bildverarbeitung" in app._cmd.status
    app.stop(); th.join(timeout=5)


def test_manual_without_pipeline():
    """Handsteuerung bei gestoppter Bildverarbeitung: regelt ohne Kamera, Totmann greift,
    kamerabasierte Modi bleiben gesperrt."""
    import threading
    from xgonav.app import App, ModeNotAllowed
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
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
    import threading
    from xgonav.wheels import WheelOdom
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    wh = WheelOdom(cfg.odom)
    world = SimWorld(cfg.camera, robot, None)
    app = App(cfg, robot, world, imu=None, wheels=wh, floor_net=SimFloorNet(world))
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


def test_drive_faktor_mal_tempo_mal_max_v():
    """Seit 2026-11-23: Fahrwerte der Modi und die Handsteuerung sind Anteile von max_v/max_w, gesendet wird
    Faktor x tempo x max_v, rückwärts wie vorwärts, explore ohne Ausnahme; keine Gerätefaktoren mehr."""
    from types import SimpleNamespace as NS
    from xgonav.app import App
    from xgonav.behaviors import Command, Manual
    cfg = Config.neutral()
    rc = cfg.robot
    assert rc.v_of(0.5) == pytest.approx(0.5 * rc.max_v) and rc.w_of(1.0) == pytest.approx(rc.max_w)
    r = SimRobot(rc)
    m = Manual(cfg.behavior, rc)
    m.set(1.0, -1.0, time.monotonic())
    c = m.step(None)
    assert c.v == pytest.approx(rc.max_v) and c.w == pytest.approx(-rc.max_w)      # Vollausschlag = max_v
    for name in ("manual", "explore", "follow"):
        app = NS(cfg=cfg, robot=r, behavior=NS(name=name))
        App._drive(app, Command(1.0, 0.0))
        assert r._target[0] == pytest.approx(rc.max_v)                             # max_v ist das Maximum
        App._drive(app, Command(-1.0, 0.0))
        assert r._target[0] == pytest.approx(-rc.max_v)
    r.tempo = 0.5
    App._drive(app, Command(rc.v_of(0.4), rc.w_of(0.2)))
    assert r._target == pytest.approx((0.4 * 0.5 * rc.max_v, 0.2 * 0.5 * rc.max_w))
    r._cur = (-0.15, 0.0); r._send(time.monotonic() + 10)
    assert r._sent[0] == norm_to_u8(-0.15)                                         # rückwärts ohne Faktor

def test_level_while_driving():
    """Nivellieren beim Fahren: an nach 1 s geradeaus, aus beim Drehen, Anhalten und Rollbefehl."""
    cfg = Config.neutral(); cfg.robot.level_while_driving = True
    r = SimRobot(cfg.robot)
    lv = lambda: [w[2] for w in r.writes if w[1] == "roll_balance"]
    r.state.balancing = True
    r._level_tick(0.0)
    r._cur = (0.2, 0.0); r._level_tick(1.0); r._level_tick(1.8)
    assert lv() == []                                     # noch keine Sekunde geradeaus
    r._level_tick(2.0)
    assert lv() == [1]
    r._level_tick(3.0)
    assert lv() == [1]                                    # nur beim Wechsel schreiben
    r._cur = (0.2, 0.2); r._level_tick(3.2)
    assert lv() == [1, 0]                                 # Kurve/Drehen: aus
    r._cur = (0.2, 0.03); r._level_tick(3.4); r._level_tick(4.2)
    assert lv() == [1, 0]                                 # nach dem Drehen erst wieder 1 s abwarten
    r._level_tick(4.4)
    assert lv() == [1, 0, 1]
    r._cur = (0.0, 0.0); r._level_tick(5.0)
    assert lv() == [1, 0, 1, 0]                           # Stillstand: aus, Trimm bleibt in der Firmware
    r.set_roll(8.0); r._cur = (0.2, 0.0); r._level_tick(6.0); r._level_tick(8.0)
    assert lv() == [1, 0, 1, 0]                           # Kopf schief (pet): nie
    r.state.balancing = False; r._level_tick(9.0)
    assert r._roll_cmd == 0 and not r._drive_level_on


def test_level_always():
    """Test-Schalter: Nivellierung bleibt an, auch im Stand und beim Drehen (nach jedem Drehbefehl neu)."""
    cfg = Config.neutral(); cfg.robot.level_always = True
    r = SimRobot(cfg.robot)
    lv = lambda: [w[2] for w in r.writes if w[1] == "roll_balance"]
    r.state.balancing = True
    r._level_tick(0.0)
    assert lv() == [1]
    r._cur = (0.0, 0.5); r._send(1.0)
    assert lv() == [1, 1]
    r._cur = (0.0, 0.0); r._level_tick(2.0); r._send(2.0)
    assert lv() == [1, 1]


def test_level_pulse():
    """Impuls einmal je Aufstehen, nach dem Aufschwung und auf Knopfdruck; liegend nicht."""
    cfg = Config.neutral(); cfg.robot.level_after_standup = True; cfg.robot.level_pulse_s = 3.0
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
    r0 = SimRobot(Config.neutral().robot)                         # Default: automatisch nie, Knopf geht
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


def test_emotion_sturz_schrei_und_aufrappeln():
    """Sturz = Firmware 1 -> 0, in allen Modi auch ohne Bildverarbeitung; Absetzen (254) schreit nicht."""
    from xgonav.config import EmotionCfg
    from xgonav.emotion import EmotionTrigger
    snd = []
    tr = EmotionTrigger(EmotionCfg(), lambda n: None, lambda: False, snd.append)
    for t, fw in ((0, 1), (1, 254), (2, 253), (3, 1), (4, 0), (5, 0), (6, 254), (7, 253), (8, 1), (9, 1),
                  (10, None), (11, 0), (12, 1)):
        tr.update(float(t), "idle", None, 80, fw)
    assert snd == ["schrei", "aufrappeln", "schrei", "aufrappeln"]


def test_sounds_tool(tmp_path):
    import importlib.util, wave
    spec = importlib.util.spec_from_file_location("sounds", Path(__file__).resolve().parents[1] / "tools" / "sounds.py")
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    for name, fn in (("freude.wav", m.freude), ("trauer.wav", m.trauer), ("schrei.wav", lambda: m.schrei(1.0)),
                     ("aufrappeln.wav", lambda: m.aufrappeln(1.0))) + tuple(
                        (f"dudeln{k}.wav", lambda k=k: m.dudeln(k)) for k in m.VARIANTS):
        p = str(tmp_path / name)
        m.write(p, fn())
        with wave.open(p) as w:
            assert w.getframerate() == m.RATE and w.getnchannels() == 1
            assert 0.3 < w.getnframes() / m.RATE < 1.5


def test_dudler():
    """„Dum-di-dum“: erst nach start_s, dann etwa alle every_s; Ende der Suche setzt zurück."""
    import random
    from xgonav.emotion import Dudler
    d = Dudler(1.5, 25.0, random.Random(1))
    assert not d.update(0.0, True)
    assert not d.update(1.4, True)
    assert d.update(1.5, True)
    t = [x / 10 for x in range(16, 600)]
    hits = [x for x in t if d.update(x, True)]
    assert all(17.5 - 0.1 <= b - a <= 32.5 + 0.1 for a, b in zip([1.5] + hits, hits)) and len(hits) >= 1
    assert not d.update(61.0, False)
    assert not d.update(61.1, True) and not d.update(62.0, True) and d.update(62.6, True)


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
    from xgonav.rawrec import raw_run_dir
    base = tmp_path / "neu" / "fahrt"
    with mock.patch("time.strftime", return_value="20261106-150000"):
        a = raw_run_dir(str(base))
        with open(os.path.join(a, "000000.jpg"), "w") as f:
            f.write("alt")
        b = raw_run_dir(str(base))
        c = raw_run_dir(str(base))
    assert os.path.basename(a) == "20261106-150000"
    assert os.path.basename(b) == "20261106-150000-2" and os.path.basename(c) == "20261106-150000-3"
    with open(os.path.join(a, "000000.jpg")) as f:
        assert f.read() == "alt"
    assert not os.listdir(b)


def test_gain_runtime_limits():
    from xgonav.camera import FrameSource
    from xgonav.config import CameraCfg
    src = FrameSource(Config.neutral().camera)
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


class _NetzZeilen:
    """Ersatz für FloorNet: feste Grenzzeile je Netzspalte, zählt die Aufrufe."""

    def __init__(self, row=300.0, fehler=False):
        self.row, self.fehler, self.n = row, fehler, 0

    def run(self, bgr):
        self.n += 1
        if self.fehler:
            raise RuntimeError("kaputt")
        return np.full(80, self.row, np.float32)


def test_freespace_worker_matches_direct_update():
    """Thread und Aufruf im Takt (compute) liefern dasselbe wie from_net, jedes Ergebnis nur einmal."""
    from xgonav.freespace import FreeSpaceEstimator, FreeSpaceWorker
    est = FreeSpaceEstimator(Config.neutral().freespace, (640, 480))
    w = FreeSpaceWorker(est, _NetzZeilen())
    w.start()
    try:
        assert w.poll() is None
        for row in (300.0, 360.0, 420.0):
            w.net.row = row
            ref = est.from_net(np.full(80, row, np.float32), 0.0)
            w.offer(np.zeros((480, 640, 3), np.uint8), 0.0)
            t_end = time.monotonic() + 5
            got = None
            while got is None and time.monotonic() < t_end:
                got = w.poll()
                time.sleep(0.005)
            assert got is not None and got.src == "net"
            assert got.sectors == ref.sectors and np.array_equal(got.columns, ref.columns)
            assert w.poll() is None                         # Ergebnis nur einmal
    finally:
        w.close()
    assert not w._thread.is_alive()
    inline = FreeSpaceWorker(est, _NetzZeilen(420.0)).compute(FreeSpaceWorker.job(None, 0.0))
    assert np.array_equal(inline.columns, ref.columns)


def test_freespace_blind_ohne_oder_nach_ausfall_des_netzes():
    """Kein Ersatzweg: ohne Netz oder nach einem Fehler im Netz ist der Worker blind und liefert nichts mehr;
    safety() lässt dann nichts fahren (Nutzer 2026-11-18)."""
    from xgonav.behaviors import Command, World, safety
    from xgonav.egomotion import EgoResult
    from xgonav.freespace import FreeSpaceEstimator, FreeSpaceWorker
    est = FreeSpaceEstimator(Config.neutral().freespace, (640, 480))
    assert FreeSpaceWorker(est, None).blind
    net = _NetzZeilen(fehler=True)
    w = FreeSpaceWorker(est, net)
    assert not w.blind
    assert w.compute(w.job(np.zeros((480, 640, 3), np.uint8), 0.0)) is None
    assert w.blind and w.net is None
    assert w.compute(w.job(np.zeros((480, 640, 3), np.uint8), 0.0)) is None and net.n == 1
    welt = dict(t=1.0, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, 10.0, 0.05), tracks=[], free=None,
                balancing=True, yaw=0.0, cmd_v=0.0)
    c = safety(Command(0.1, 0.2, None, "x"), World(**welt, blind=True), Config.neutral().behavior)
    assert (c.v, c.w) == (0.0, 0.0) and "BLIND" in c.status
    c = safety(Command(0.1, 0.2, None, "x"), World(**welt), Config.neutral().behavior)
    assert (c.v, c.w) == (0.1, 0.2)


def test_app_ohne_floornet_ist_blind(caplog):
    """Fehlt das Netz, startet xgonav blind: Modi mit Freiraum sind gesperrt, der Status sagt "blind"."""
    from xgonav.app import App, ModeNotAllowed
    from xgonav.robot import SimRobot
    from xgonav.sim import SimWorld
    cfg = Config.neutral()
    cfg.output.web_port = 0
    cfg.odom.source = "command"
    robot = SimRobot(cfg.robot)
    app = App(cfg, robot, SimWorld(cfg.camera, robot))
    assert app.fs_worker is not None and app.fs_worker.blind
    with pytest.raises(ModeNotAllowed):
        app._check_mode("explore")
    assert any("blind" in r.getMessage() for r in caplog.records)


def test_wheel_odometry_wrap_reset_and_signs():
    import math
    from xgonav.config import OdomCfg
    import threading
    from xgonav.wheels import WheelOdom
    cfg = Config.neutral().odom
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


def test_wheel_speed_window():
    from xgonav.config import OdomCfg
    from xgonav.wheels import WheelOdom
    w = WheelOdom(Config.neutral().odom)
    k = 1.0 / w.m_per_tick
    assert w.speed(0.3) is None                          # keine Proben
    for i in range(10):                                  # 0,18 s mit 0,25 m/s
        w.add(i + 1, i * 0.02, int(0.25 * i * 0.02 * k), int(0.25 * i * 0.02 * k), 1000 + i * 20)
    assert w.speed(0.3) is None                          # Historie deckt das Fenster noch nicht ab
    for i in range(10, 40):
        w.add(i + 1, i * 0.02, int(0.25 * i * 0.02 * k), int(0.25 * i * 0.02 * k), 1000 + i * 20)
    assert w.speed(0.3) == pytest.approx(0.25, abs=0.003)
    # links schneller als rechts: Radmitte
    s0 = w.latest()
    for i in range(40, 60):
        d = (i - 39) * 0.02
        w.add(i + 1, i * 0.02, int((s0[1] + 0.30 * d) * k), int((s0[2] + 0.10 * d) * k), 1000 + i * 20)
    assert w.speed(0.3) == pytest.approx(0.20, abs=0.003)
    # Lücke (abgesetzt): kein Mittel über die Pause hinweg
    w.add(100, 5.0, int(w.latest()[1] * k), int(w.latest()[2] * k), 6000)
    assert w.speed(0.3) is None


def _speed_ctrl_fahrt(abzug=0.0, v_cmd=0.2, on=True, secs=9.0):
    """Regelkreis mit dem Trägheitsmodell des Simulators (sim._dyn): Rampe 50 Hz, Räder 50 Hz, Regelschritt
    ~20 Hz. abzug: Fehlbetrag am Fahrwert (Teppich). Liefert (Soll m/s, Spitze, Mittel der letzten 2 s,
    Spanne der letzten 2 s, Korrektur am Ende)."""
    import math
    from xgonav.config import OdomCfg
    from xgonav.robot import SpeedCtrl
    from xgonav.wheels import WheelOdom
    cfg = Config.neutral().robot
    cfg.vx_limit, cfg.speed_ctrl = 1.8, on
    r = SimRobot(cfg)
    sc, w = SpeedCtrl(cfg), WheelOdom(Config.neutral().odom)
    tau, nmp, h = SimWorld.DYN_TAU_V, SimWorld.DYN_NMP_S, 0.004
    cur = vf = s = 0.0
    vs = []
    for k in range(int(secs / h)):
        t = k * h
        if k % 5 == 0:
            cur += max(-cfg.accel_v * 0.02, min(cfg.accel_v * 0.02, v_cmd - cur))
            r._cur = (cur, 0.0)
            w.add(k // 5 + 1, t, int(s / w.m_per_tick), int(s / w.m_per_tick), int(t * 1000) + 1)
        eff = max(0.0, r.with_trim(r.v_nominal) - abzug) * cfg.vx_limit
        prev = vf
        vf += (eff - vf) * (1 - math.exp(-h / tau))
        v = vf - nmp * (vf - prev) / h
        s += v * h
        if k % 12 == 0 and on:
            r.v_trim = sc.step(t, r.v_nominal, w.speed(cfg.speed_ctrl_window_s))
        vs.append(v)
    late = vs[-int(2.0 / h):]
    return v_cmd * cfg.vx_limit, max(vs), sum(late) / len(late), max(late) - min(late), r.v_trim


def test_speed_ctrl_reaches_setpoint():
    """Teppich (Fehlbetrag am Fahrwert): ohne Regelung bleibt er unter dem Soll, mit Regelung erreicht er es,
    ohne zu pendeln. Auf ebenem Boden ändert die Regelung fast nichts."""
    soll, _, ende, _, _ = _speed_ctrl_fahrt(abzug=0.04, on=False)
    assert ende < 0.82 * soll
    soll, peak, ende, spanne, trim = _speed_ctrl_fahrt(abzug=0.04)
    assert abs(ende - soll) < 0.05 * soll and peak < 1.10 * soll and spanne < 0.05 * soll
    assert trim == pytest.approx(0.04, abs=0.008)
    soll, peak, ende, spanne, trim = _speed_ctrl_fahrt(abzug=0.04, v_cmd=0.10)   # langsam, wie an Engstellen
    assert abs(ende - soll) < 0.06 * soll and peak < 1.10 * soll
    soll, peak, ende, spanne, trim = _speed_ctrl_fahrt()                          # eben
    assert abs(ende - soll) < 0.03 * soll and peak < 1.10 * soll and abs(trim) < 0.008


def test_speed_ctrl_rules():
    """Korrektur 0 ohne Raddaten, unter dem Totband und bei Richtungswechsel; begrenzt; Stopp bleibt 128."""
    from xgonav.robot import FW_DEADBAND, SpeedCtrl
    cfg = Config.neutral().robot
    cfg.vx_limit, cfg.speed_ctrl, cfg.speed_ctrl_lag_s = 1.8, True, 0.0
    sc = SpeedCtrl(cfg)
    t = 0.0
    for _ in range(400):                                 # steht an: Ist 0 bei Soll 0,36 m/s
        t += 0.05
        sc.step(t, 0.2, 0.0)
    assert sc.trim == pytest.approx(cfg.speed_ctrl_max)  # begrenzt
    assert sc.v_soll == pytest.approx(0.36)
    assert sc.step(t + 0.05, 0.2, None) == 0.0           # Raddaten weg
    for _ in range(40):
        t += 0.05
        sc.step(t, 0.2, 0.1)
    assert sc.trim > 0.02
    assert sc.step(t + 0.05, -0.2, -0.1) == 0.0          # Richtungswechsel
    for _ in range(40):
        t += 0.05
        sc.step(t, -0.2, -0.1)
    assert sc.trim < -0.02                               # rückwärts zu langsam: mehr rückwärts
    assert sc.step(t + 0.05, -0.5 * FW_DEADBAND, -0.1) == 0.0   # unter dem Totband der Firmware
    for _ in range(400):                                 # schneller als verlangt: nimmt zurück, begrenzt
        t += 0.05
        sc.step(t, 0.2, 0.6)
    assert sc.trim == pytest.approx(-cfg.speed_ctrl_max)
    # Senden: Korrektur nach den Faktoren, nie über den Stillstand hinweg, Stopp exakt 128
    r = SimRobot(cfg)
    r.v_trim = 0.05
    r._cur = (0.15, 0.0); r._send(100.0)
    assert r._sent[0] == norm_to_u8(0.20)
    r._cur = (-0.10, 0.0); r._send(200.0)
    assert r._sent[0] == norm_to_u8(-0.05)               # -0,10 + 0,05 (ohne reverse_gain seit 2026-11-23)
    r._cur = (0.0, 0.0); r._send(300.0)
    assert r._sent[0] == 128
    r.v_trim = -0.10
    r._cur = (0.06, 0.0); r._send(400.0)
    assert r._sent[0] == 128                             # kehrt die Richtung nicht um
    r.v_trim = 0.10
    r._cur = (0.5 * FW_DEADBAND, 0.0); r._send(500.0)
    assert r._sent[0] == norm_to_u8(0.5 * FW_DEADBAND)   # unter dem Totband bleibt der Wert


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
    cfg = Config.neutral()
    cfg.grid.max_range_m = 1.5                                  # entlang des Strahls: Wand 1 m am Rand ~1,44 m
    cfg.grid.hit_band_quad = 0.0                                # ohne Trefferband: Wand genau eine Zelle
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    m = GridMap(cfg.grid, g, 160, 120)
    hz = g.horizon_row(0) / 4
    ys = np.arange(int(hz * 4) + 1, 480)
    y1 = ys[np.nanargmin(np.abs(g.ground_dist_np(ys) - 1.0))]
    col = (120 - y1 / 4) / (120 - hz)
    fs = FreeSpace([col] * 5, np.full(160, col), hz, True, None)
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
    return FreeSpace([col] * 5, np.full(160, col), hz, True, None)


def test_gridmap_range_hits_and_margin():
    """Ein Treffer reicht nicht für "belegt", Treffer nur bis max_range_m, Freistrahl weiter und mit Rand."""
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
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


def test_gridmap_range_along_ray():
    """max_range_m und free_range_m gelten entlang des Strahls: eine Bildzeile ist am Rand weiter weg als in der
    Mitte (Peilung ~45°, Faktor ~1,4)."""
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    m = GridMap(cfg.grid, g, 160, 120)
    x = 0.9 * cfg.grid.max_range_m                              # Wand quer, 0,9 x Reichweite voraus
    _, bear, dist, _, _, _, hit = m._rays(_wall_fs(g, x), 0.0)
    mitte, rand = np.argmin(np.abs(bear)), np.argmax(np.abs(bear))
    assert hit[mitte] and dist[mitte] <= cfg.grid.max_range_m
    assert dist[rand] > cfg.grid.max_range_m and not hit[rand]   # vorher (Abstand nach vorn) ein Treffer
    assert np.all(dist[hit] <= cfg.grid.max_range_m)
    _, bear, _, free_len, _, ok, _ = m._rays(_wall_fs(g, 2.2), 0.0)
    assert np.all(free_len[ok] <= cfg.grid.free_range_m + 1e-9)
    assert free_len[np.argmin(np.abs(bear))] > cfg.grid.free_range_m - 0.05


def test_localmap_origin_is_wheel_axle():
    """[camera] cam_fwd_mm: Kamera sitzt vor der Radachse; Punkte liegen um den Versatz weiter vorn,
    Freiräumen trifft dieselben Punkte wie ohne Versatz."""
    from xgonav.freespace import FreeSpace
    from xgonav.localmap import LocalMap
    g0 = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    g36 = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0, cam_fwd_mm=36.0)
    wall = _wall_fs(g0, 0.5)
    maps = []
    for g in (g0, g36):
        lm = LocalMap(g, 160, 120, confirm=1)
        lm.add_freespace(wall, 0.0, 0.0)
        maps.append(lm)
    a, b = maps
    assert len(a.pts) == len(b.pts) > 10
    assert np.allclose(b.pts[:, 0] - a.pts[:, 0], 0.036) and np.allclose(b.pts[:, 1], a.pts[:, 1])
    # Drehung auf der Stelle um 90° links: Punkt geradeaus wandert nach rechts, Abstand zur Achse bleibt
    i = int(np.argmin(np.abs(b.pts[:, 1])))
    x0, y0 = b.pts[i]
    b.move(0.0, math.radians(90))
    assert b.pts[i] == pytest.approx([y0, -x0], abs=1e-9)          # um die Achse, Abstand ab Achse bleibt
    b.move(0.0, math.radians(-90))
    # Freiräumen: ferne Wand löscht die Punkte bei 0,5 m, Wand bei 0,6 m nicht (Toleranz) – mit und ohne Versatz
    for far, keep in ((1.2, False), (0.6, True)):
        n = []
        for lm in maps:
            c = LocalMap(lm.geo, 160, 120, confirm=1)
            c.pts, c.t, c.obs = lm.pts.copy(), lm.t.copy(), lm.obs.copy()
            c.add_freespace(_wall_fs(g0, far), 0.1, 0.0)
            n.append(int((c.t == 0.0).sum()))                # alte Punkte (neue haben t 0,1)
        assert n[0] == n[1]
        assert n[0] > len(a.pts) // 2 if keep else n[0] <= 2      # Randspalten werden nie geräumt


def test_gridmap_rays_start_ahead_of_axle():
    """Raumkarte: Pose = Radachse, Strahlen beginnen cam_fwd davor (gleich wie verschobene Pose ohne Versatz)."""
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
    g0 = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    g36 = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0, cam_fwd_mm=36.0)
    fs = _wall_fs(g0, 1.0)
    th = math.radians(30)
    a, b = GridMap(cfg.grid, g36, 160, 120), GridMap(cfg.grid, g0, 160, 120)
    for _ in range(3):
        a.add_freespace(fs, 0.0, (0.2, -0.1, th))
        b.add_freespace(fs, 0.0, (0.2 + 0.036 * math.cos(th), -0.1 + 0.036 * math.sin(th), th))
    assert np.array_equal(a.lo, b.lo) and len(a.occupied_cells()) > 0


def test_sim_camera_ahead_of_axle():
    """Simulator: mit cam_fwd_mm rendert die Kamera vor der Radachse (Achse = Pose, Kollision, Räder)."""
    import dataclasses
    cfg = Config.neutral()
    imgs = []
    for off, x in ((36.0, 1.0), (0.0, 1.036)):
        cam = dataclasses.replace(cfg.camera, cam_fwd_mm=off)
        r = SimRobot(cfg.robot)
        r._height_mm = 85
        w = SimWorld(cam, r, None, start=(x, 1.5, 0.0), shake=0.0)
        w._integrate(0.0, 0.05)
        imgs.append(w._render(0.0, 0.05)[0])
    assert np.array_equal(imgs[0], imgs[1])


def test_imu_yaw_unwrap_and_heading_into_maps():
    """imu_log-Kurs: über ±180° stetig; die Koppelnavigation dreht Merkkarte, Raumkarte und Verhalten mit dem
    IMU-Kurs zur Bildzeit, ist der zu alt, mit den Rädern bzw. dem Bild."""
    from xgonav.app import App
    from xgonav.imu import ImuSource
    cfg = Config.neutral()
    imu = ImuSource(cfg.imu)
    assert imu.at(0.0) is None
    t0 = time.monotonic()
    for k, deg in enumerate((170.0, 179.0, -178.0, -170.0)):              # Sprung über ±180°
        imu.add(k + 1, t0 + 0.02 * k, 0.0, 0.0, math.radians(deg))
    assert math.degrees(imu.at(t0 + 0.06)[2]) == pytest.approx(190.0)
    old = ImuSource(cfg.imu)
    old.add(1, t0, 0.0, 0.0)                                               # ohne Kurs (alte Aufrufer)
    old.add(2, t0 + 0.02, 0.0, 0.0)
    assert old.at(t0)[2] is None
    cfg.output.web_port = 0
    cfg.detector.enabled = False
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None)
    app = App(cfg, robot, world, imu=imu, floor_net=SimFloorNet(world))
    st = app._odom_step(t0 + 0.06, imu.at(t0 + 0.06)[2], 0.5, 0.05)
    assert st.dyaw == 0.5 and st.kurs_src == "kamera"                      # erster Wert: noch kein Bezug
    imu.add(10, t0 + 0.08, 0.0, 0.0, math.radians(-165.0))
    st = app._odom_step(t0 + 0.08, imu.at(t0 + 0.08)[2], 0.5, 0.05)
    assert math.degrees(st.dyaw) == pytest.approx(5.0) and app.yaw_src == "imu"
    st = app._odom_step(t0 + 5.0, None, 0.25, 0.05)                         # IMU zu alt: Bild
    assert st.dyaw == 0.25 and app.yaw_src == "kamera"
    assert app.yaw == pytest.approx(0.75 + math.radians(5.0))


def test_imu_yaw_jump_discarded():
    """ESP-Neustart: Kurs beginnt bei 0, dazu ~1 s Lücke; zu schnelle Sprünge und Änderungen über Lücken
    gelangen nicht in den Kurs, der läuft stetig weiter."""
    from xgonav.imu import ImuSource
    imu = ImuSource(Config.neutral().imu)
    for k in range(5):
        imu.add(k + 1, 10.0 + 0.01 * k, 0.0, 0.0, math.radians(90.0 + k))    # 100 °/s: echt
    assert imu.yaw_jumps == 0 and math.degrees(imu.latest_yaw()) == pytest.approx(94.0)
    imu.add(6, 10.05, 0.0, 0.0, math.radians(40.0))                       # 54° in 10 ms
    assert imu.yaw_jumps == 1 and math.degrees(imu.latest_yaw()) == pytest.approx(94.0)
    imu.add(7, 11.2, 0.0, 0.0, math.radians(0.0))                         # nach 1,15 s Lücke ab 0
    assert imu.yaw_jumps == 2 and math.degrees(imu.latest_yaw()) == pytest.approx(94.0)
    imu.add(8, 11.21, 0.0, 0.0, math.radians(1.0))
    assert math.degrees(imu.latest_yaw()) == pytest.approx(95.0)


def test_proc_imu_reads_only_new_lines(tmp_path):
    """imu_log ist ein Ringpuffer: je Lesen nur die neuen Zeilen übernehmen, in seq-Reihenfolge."""
    from xgonav.imu import ProcImu
    cfg = Config.neutral()
    cfg.imu.proc_dir = str(tmp_path)
    imu = ProcImu(cfg.imu)
    imu.path = str(tmp_path / "imu_log")

    def write(rng):
        (tmp_path / "imu_log").write_text("".join("%d %d 0 %d %d\n" % (s, s * 10_000_000, s * 100, s * 1000) for s in rng))
    write(range(1, 6))
    assert imu._poll() == 5
    write(range(3, 9))                                   # Puffer rückt weiter, 6..8 sind neu
    assert imu._poll() == 3 and imu._poll() == 0
    ts, ps = imu.arrays()
    assert list(np.round(ts, 3)) == [0.01 * s for s in range(1, 9)]
    assert math.degrees(imu.at(0.08)[2]) == pytest.approx(8.0)


def test_proc_imu_requests_stream(tmp_path):
    """[imu] stream_hz wird beim Öffnen nach settings/imu_stream_hz geschrieben, wenn der Treiber das kennt."""
    from xgonav.imu import ProcImu
    (tmp_path / "settings").mkdir()
    (tmp_path / "imu_log").write_text("")
    (tmp_path / "settings" / "imu_hz").write_text("50\n")
    cfg = Config.neutral()
    cfg.imu.proc_dir = str(tmp_path)
    cfg.imu.stream_hz = 100
    assert ProcImu(cfg.imu)._open()                      # alter Treiber: Datei fehlt, nur Warnung
    assert not (tmp_path / "settings" / "imu_stream_hz").exists()
    (tmp_path / "settings" / "imu_stream_hz").write_text("0\n")
    assert ProcImu(cfg.imu)._open()
    assert (tmp_path / "settings" / "imu_stream_hz").read_text() == "100"


def test_proc_imu_idle_rate(tmp_path):
    """Abgesetzt: Strom und Lesen mit [imu] idle_hz, beim Aufstehen wieder stream_hz und poll_s."""
    from xgonav.imu import ProcImu
    (tmp_path / "settings").mkdir()
    (tmp_path / "imu_log").write_text("")
    (tmp_path / "settings" / "imu_hz").write_text("50\n")
    sp = tmp_path / "settings" / "imu_stream_hz"
    sp.write_text("0\n")
    cfg = Config.neutral()
    cfg.imu.proc_dir = str(tmp_path)
    cfg.imu.stream_hz, cfg.imu.idle_hz = 100, 5          # am Rider idle_hz 0
    imu = ProcImu(cfg.imu)
    assert imu._open() and sp.read_text() == "100" and imu._wait_s() == cfg.imu.poll_s
    imu.set_idle(True)
    assert sp.read_text() == "5" and imu._wait_s() == pytest.approx(0.2)
    sp.write_text("77")
    imu.set_idle(True)                                   # kein Wechsel: nichts geschrieben
    assert sp.read_text() == "77"
    imu.set_idle(True, force=True)                       # nach dem Ruhemodus
    assert sp.read_text() == "5"
    imu.set_idle(False)
    assert sp.read_text() == "100" and imu._wait_s() == cfg.imu.poll_s
    imu.paused = True
    assert imu._wait_s() == 1.0
    cfg.imu.idle_hz = 0                                  # abgeschaltet: bleibt bei voller Rate
    imu.paused = False
    imu.set_idle(True)
    assert sp.read_text() == "100" and not imu.idle
    cfg.imu.idle_hz, cfg.imu.stream_hz = 5, 0            # ohne Strom: nur seltener lesen, Treiber unberührt
    sp.write_text("0")
    imu.set_idle(True)
    assert sp.read_text() == "0" and imu._wait_s() == pytest.approx(0.2)


def test_proc_imu_refresh(tmp_path):
    """Bildschleife liest selbst (refresh): neue Proben werden übernommen, schon gelesene nicht doppelt; liest
    gerade der Faden (Sperre) oder ist Ruhe, passiert nichts."""
    from xgonav.imu import ProcImu
    (tmp_path / "settings").mkdir()
    log_ = tmp_path / "imu_log"
    log_.write_text("1 1000000000 100 -2000 0\n2 1010000000 110 -2100 500\n")
    cfg = Config.neutral()
    cfg.imu.proc_dir = str(tmp_path)
    imu = ProcImu(cfg.imu)
    assert imu._open()
    assert imu.refresh() == 2 and imu.latest_t() == pytest.approx(1.01)
    log_.write_text("1 1000000000 100 -2000 0\n2 1010000000 110 -2100 500\n3 1020000000 120 -2200 900\n")
    with imu._poll_lock:
        assert imu.refresh() == 0                        # Faden liest gerade
    imu.paused = True
    assert imu.refresh() == 0
    imu.paused = False
    assert imu.refresh() == 1 and imu.latest_t() == pytest.approx(1.02)
    assert imu.refresh() == 0                            # nichts Neues


def test_wheels_idle_reads_nothing_and_restarts_without_gap():
    """Abgesetzt werden die Räder nicht gelesen; was sie dazwischen drehten, zählt danach nicht als Weg."""
    import threading
    from xgonav.wheels import WheelOdom

    class _W(WheelOdom):
        polls = 0
        def _poll(self):
            self.polls += 1
            return 0

    cfg = Config.neutral()
    w = _W(cfg.odom)
    w.add(1, 1.0, 0, 0)
    w.add(2, 1.1, 1024, 1024)
    s0 = w.latest()[1]
    assert s0 > 0.1
    w.set_idle(True)
    w._thread = threading.Thread(target=w._run, daemon=True)
    w._thread.start()
    time.sleep(0.5)
    assert w.polls == 0 and w.rate_hz == 0.0
    w.set_idle(False)
    time.sleep(0.3)
    assert w.polls > 0
    w.close()
    w.add(3, 9.0, 5000, 5000)                            # im Sitzen geschoben: kein Weg
    assert w.latest()[1] == pytest.approx(s0)
    w.add(4, 9.1, 6024, 6024)                            # danach zählt es wieder
    assert w.latest()[1] == pytest.approx(2 * s0)
    cfg.odom.read_idle = True
    w.set_idle(True)
    assert not w.idle


def test_gridmap_driven_path_is_free():
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
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
    cfg = Config.neutral()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    est = FreeSpaceEstimator(cfg.freespace, (640, 480), g)
    w = FreeSpaceWorker(est, _NetzZeilen())
    w.start()
    w.offer(np.full((480, 640, 3), 90, np.uint8), 0.03, (1.0, 2.0, 0.5))
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
    cfg = Config.neutral()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    hz = g.horizon_row(0) / 4
    ys = np.arange(int(hz * 4) + 1, 480)
    y1 = ys[np.nanargmin(np.abs(g.ground_dist_np(ys) - 1.0))]
    col = (120 - y1 / 4) / (120 - hz)
    fs = FreeSpace([col] * 5, np.full(160, col), hz, True, None)
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
    cfg = Config.neutral()
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
        fs = FreeSpace([0.5] * 5, cols, hz, True, kf)
        pose += (rng.normal(0, 0.05), rng.normal(0, 0.05), rng.normal(0, 0.2))
        if k == 150:
            pose[:] = (7.9, -7.95, 1.0)                          # Kartenrand: Strahlen laufen hinaus
        pd = float(rng.normal(0, 0.01))
        mn.add_freespace(fs, pd, tuple(pose))
        mp.add_freespace(fs, pd, tuple(pose))
    assert np.array_equal(mn.lo, mp.lo)
    assert np.abs(mn.lo).max() > 1.0 and mn.updates == mp.updates == 300


def test_gridmap_confirm_from_two_distances(tmp_path):
    """confirm_dr_m: Treffer zählen erst, wenn dieselbe Stelle aus zwei verschiedenen Entfernungen getroffen
    wurde. Echtes Hindernis: bleibt stehen, wird belegt. Spiegelung (wandert mit dem Rider) und Drehung auf der
    Stelle (immer gleiche Entfernung): nie belegt, nur Kandidat. Gefahrener Weg löscht Kandidaten."""
    from dataclasses import replace
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
    g = CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0)
    m = GridMap(replace(cfg.grid, confirm_dr_m=0.2, native=False), g, 160, 120)
    ang = np.radians(np.linspace(-3, 3, 7))
    ca, sa, hit = np.cos(ang), np.sin(ang), np.ones(7, bool)

    def shot(px, d, n=3):
        with m._lock:
            for _ in range(n):
                m._add_hits(px, 0.0, np.full(7, d), ca, sa, hit)

    def occ(x):
        r, c = m.cell(x, 0.0)
        return m.lo[r, c] > cfg.grid.l_show

    shot(0.0, 1.0)                                    # echtes Hindernis bei x = 1,0 aus 1,0 m
    assert not occ(1.0) and np.isfinite(m.rmin[m.cell(1.0, 0.0)])
    shot(0.3, 0.7)                                    # näher dran, gleiche Stelle: bestätigt
    assert occ(1.0)
    shot(0.0, 0.6)                                    # Spiegelung: vom Start bei 0,6 m ...
    shot(0.72, 0.82)                                  # ... 0,72 m weiter vorn bei 1,54 m (wandert mit)
    assert not occ(0.6) and not occ(1.54)
    shot(5.0, 0.8, n=10)                              # Drehung auf der Stelle: immer 0,8 m
    assert not occ(5.8)
    img = m.image(480, 4.0)                           # Kandidaten orange in der Anzeige
    assert img.shape == (480, 480, 3)
    m._clear_disk(0.6, 0.0, 0.1)                      # darüber gefahren: Kandidat weg
    assert not np.isfinite(m.rmin[m.cell(0.6, 0.0)])
    f = str(tmp_path / "k.npz")                       # Kandidaten und Bestätigung werden gespeichert
    assert m.save(f)
    m2 = GridMap(replace(cfg.grid, confirm_dr_m=0.2, native=False), g, 160, 120)
    assert m2.load(f) and m2.conf[m.cell(1.0, 0.0)] and np.isfinite(m2.rmin[m.cell(1.54, 0.0)])
    # Kern (falls gebaut) und numpy gleich, auch mit Bestätigung
    from xgonav.freespace import FreeSpace
    from xgonav.gridmap import _grid_lib
    if _grid_lib() is None:
        return
    mn = GridMap(replace(cfg.grid, confirm_dr_m=0.2), g, 160, 120)
    mp = GridMap(replace(cfg.grid, confirm_dr_m=0.2, native=False), g, 160, 120)
    rng = np.random.default_rng(5)
    hz = g.horizon_row(0) / 4
    for k in range(200):
        cols = rng.uniform(0.0, 1.0, 160)
        fs = FreeSpace([0.5] * 5, cols, hz, True, None)
        pose = (0.01 * k, 0.0, 0.0)
        mn.add_freespace(fs, 0.0, pose)
        mp.add_freespace(fs, 0.0, pose)
    assert np.array_equal(mn.lo, mp.lo) and np.array_equal(mn.conf, mp.conf)


# ---------------------------------------------------------------- Startpunkt, Speichern, Heimfahrt
def _grid():
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
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


def test_gridmap_kleinere_karte_mittig_geladen(tmp_path):
    """size_m vergrößert: gespeicherte kleinere Karte landet mittig, Weltkoordinaten bleiben; größere wird ignoriert."""
    from xgonav.gridmap import GridMap
    cfg, m = _grid()
    x, y = 1.2, -0.7
    r, c = m.cell(x, y)
    m.lo[r, c] = 2.5
    f = str(tmp_path / "karte.npz")
    assert m.save(f)
    cfg.grid.size_m = m.cfg.size_m + 8
    big = GridMap(cfg.grid, m.geo, 160, 120)
    assert big.n == m.n + 160 and big.load(f)
    rb, cb = big.cell(x, y)
    assert big.lo[rb, cb] == pytest.approx(2.5, abs=0.01) and np.count_nonzero(big.lo) == 1
    if big.conf is not None:
        assert big.conf.shape == big.lo.shape and big.rmin.shape == big.lo.shape
    assert big.save(f)
    assert not m.load(f)                                       # größere Karte passt nicht in die kleine


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


def test_planner_takes_25cm_gap_not_15cm():
    """Wand quer mit Lücke: 25 cm lichte Weite ist in jeder Lage zum 5-cm-Raster offen, 10 cm nie."""
    from xgonav.planner import plan
    from xgonav.localmap import LocalMap

    def weg(breite: float, versatz: float):
        cfg, m = _grid()
        r0, c0 = (int(k) for k in m.cell(-1.0, -1.0))
        r1, c1 = (int(k) for k in m.cell(1.0, 1.0))
        m.lo[r0 - 1:r1 + 2, c0 - 1:c1 + 2] = 3.0                # geschlossener Raum 2 x 2 m
        m.lo[r0:r1 + 1, c0:c1 + 1] = -3.0
        _, c = m.cell(0.0, 0.0)
        for y in np.arange(-1.0, 1.0, 0.01):                    # Wand bei x = 0, Lücke um y = versatz
            if abs(y - versatz) >= breite / 2:
                m.lo[int(m.cell(0.0, float(y))[0]), int(c)] = 3.0
        return plan(m, (-0.6, 0.0), (0.6, 0.0), cfg.home)

    for versatz in (0.0, 0.01, 0.02, 0.03, 0.04):
        assert weg(0.25, versatz) is not None, versatz
        assert weg(0.10, versatz) is None, versatz
    # Gibt es daneben eine breite Öffnung, nimmt der Weg die, auch wenn er länger ist
    cfg, m = _grid()
    r0, c0 = (int(k) for k in m.cell(-1.0, -1.0))
    r1, c1 = (int(k) for k in m.cell(1.0, 1.0))
    m.lo[r0 - 1:r1 + 2, c0 - 1:c1 + 2] = 3.0
    m.lo[r0:r1 + 1, c0:c1 + 1] = -3.0
    for y in np.arange(-1.0, 1.0, 0.01):
        if abs(y) >= 0.125 and not 0.3 <= y <= 0.9:              # Lücke 25 cm bei y = 0, 60 cm bei y = 0,6
            m.lo[int(m.cell(0.0, float(y))[0]), int(m.cell(0.0, 0.0)[1])] = 3.0
    p = plan(m, (-0.6, 0.0), (0.6, 0.0), cfg.home)
    assert p is not None and max(y for _, y in p) > 0.45
    # Bahnprüfung: mittig durch 25 cm nur mit dem Radius für Engstellen frei, 15 cm bremst auch dann
    geo = CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0)
    for breite, frei in ((0.25, True), (0.15, False)):
        lm = LocalMap(geo, 160, 120, confirm=1, radius_m=cfg.map.radius_m)
        lm.pts = np.array([[0.3, breite / 2], [0.3, -breite / 2]])
        lm.t = np.zeros(2); lm.obs = np.array([1, 2])
        assert lm.arc_clearance(0.25, 0.0, 1.2) < math.inf
        assert (lm.arc_clearance(0.25, 0.0, 1.2, radius=cfg.map.radius_tight_m) == math.inf) == frei


def test_tight_spot_steers_to_gap_center():
    """Engstelle: Wandabstand aus der Raumkarte, Lückenmitte aus der Merkkarte, Wegfolger zielt darauf."""
    from xgonav.behaviors import PathFollower, World
    from xgonav.localmap import LocalMap
    cfg, m = _grid()
    assert m.wall_dist(0.0, 0.0, 0.25) == math.inf
    r, c = (int(k) for k in m.cell(0.1, 0.0))
    m.lo[r, c] = 3.0
    assert abs(m.wall_dist(0.0, 0.0, 0.25) - 0.125) < 0.04
    assert m.wall_dist(-0.3, 0.0, 0.25) == math.inf
    lm = LocalMap(CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0), 160, 120, confirm=1)
    lm.pts = np.array([[0.3, 0.16], [0.3, 0.30], [0.3, -0.09], [0.3, -0.25]])
    lm.t = np.zeros(4); lm.obs = np.arange(4)
    gx, gy = lm.gap_center()
    assert abs(gx - 0.3) < 1e-9 and abs(gy - 0.035) < 1e-9          # zwischen den innersten Punkten
    lm.pts = lm.pts[:2]; lm.t = np.zeros(2); lm.obs = np.arange(2)
    assert lm.gap_center() is None                                   # nur eine Seite
    f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
    f.set_path([(0.0, 0.0), (2.0, 0.0)])
    welt = lambda **k: World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[], free=None, balancing=True,
                             yaw=0.0, cmd_v=0.0, pose=(0.0, 0.0, 0.0), **k)
    nah = lambda px, py, reach: 0.15 if px > 0.4 else math.inf       # Wand erst am Weg voraus
    frei = f.drive(welt(wall_fn=lambda *a: math.inf, gap_fn=lambda: (0.3, 0.035)), "x")
    eng = f.drive(welt(wall_fn=nah, gap_fn=lambda: (0.3, 0.035)), "x")
    assert frei.w == 0.0 and not frei.extra["tight"]
    assert eng.w > 0.0 and "Lücke" in eng.status and eng.extra["tight"]   # Weg geradeaus, Lücke 3,5 cm links
    assert 0.0 < eng.v < frei.v
    quer = f.drive(welt(wall_fn=nah, gap_fn=lambda: (0.1, 0.25)), "x")   # Lücke weit neben dem Weg: Weg gilt
    assert "Lücke" not in quer.status and quer.w == 0.0


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
    cfg = Config.neutral()
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
    cfg = Config.neutral()
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


def _room_map(door=False):
    """Bekannter Raum 2,4 x 2,4 m um den Ursprung, Wände belegt; door: 0,8 m Lücke in der +x-Wand
    (dahinter unbekannt)."""
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
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


def test_frontier_merkkarte_weg_zuerst():
    """Eine Stelle, die nur ohne die Merkkarte erreichbar ist (Stuhlbeine nur dort), kommt nach einer, die auch
    mit ihr erreichbar ist, und gilt als Engstelle; bleibt sie die einzige, wird sie trotzdem angefahren."""
    from xgonav.frontier import next_goal
    cfg, m = _room_map(door=True)                              # Tür in +x (geradeaus, sonst die beste)
    r1, _ = (int(v) for v in m.cell(1.2, 1.2))
    _, dc0 = (int(v) for v in m.cell(-0.4, 0.0))
    _, dc1 = (int(v) for v in m.cell(0.4, 0.0))
    m.lo[r1, dc0:dc1 + 1] = cfg.grid.l_min                     # zweite Tür in +y
    beine = np.array([(1.1, y) for y in np.arange(-0.4, 0.41, 0.1)])    # Merkkarte: Beine in der Tür +x
    p = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert p.frontier[0] > 1.0 and not p.tight                 # ohne Merkkarte: geradeaus
    p = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore, obstacles=beine)
    assert not p.tight                                         # mit ihr: ein Weg, der die Beine meidet
    pfad = np.array(p.path)
    assert np.min(np.hypot(pfad[:, None, 0] - beine[None, :, 0], pfad[:, None, 1] - beine[None, :, 1])) > 0.2
    cfg, m = _room_map(door=True)
    p = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore, obstacles=beine)
    assert p.frontier[0] > 1.0 and p.tight                     # einzige Stelle: trotzdem, als Engstelle


def test_frontier_prefer_richtung_und_min_m():
    """Rufsuche (pet, Nutzer 2026-11-22): prefer zieht die Öffnung in der gehörten Richtung vor, min_m lässt
    Öffnungen direkt am Rider weg."""
    from xgonav.frontier import next_goal
    cfg, m = _room_map(door=True)                              # Tür in +x
    r1, _ = (int(v) for v in m.cell(1.2, 1.2))
    _, dc0 = (int(v) for v in m.cell(-0.4, 0.0))
    _, dc1 = (int(v) for v in m.cell(0.4, 0.0))
    m.lo[r1, dc0:dc1 + 1] = cfg.grid.l_min                     # zweite Tür in +y
    p = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert p.frontier[0] > 1.0                                 # ohne Vorgabe: geradeaus (+x)
    p = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore, prefer=(math.pi / 2, 0.1))
    assert p.frontier[1] > 1.0                                 # gehört von links: Tür in +y
    p = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore, min_m=5.0)
    assert p.path is None                                      # alle zu nah


def test_pet_rufsuche_richtung_nur_nach_drehen():
    """Peilung von der Seite (hingedreht): Richtung gilt für die Rufsuche; nur von vorn gehört: keine."""
    from xgonav.pet import PetBrain
    cfg = Config.neutral()
    cfg.pet.call_m = -1.0                                      # neutral 0: dann gar keine Suche
    b = PetBrain(cfg.pet, cfg.robot, 1)
    b._ear_turns = 1
    seen = {}
    b._ruf_start = lambda s, budget, seite=False: seen.update(seite=seite)
    b.seek_m = 5.0

    class S:
        t, karte = 0.0, True
    b._hear_call(S())
    assert seen["seite"] is True
    b._ear_turns = 0
    b._hear_call(S())
    assert seen["seite"] is False


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


def test_roll_columns_straightens_boundary():
    """Roll der Kamera: Die Grenze wird je Spalte um (Spalte - Mitte) * tan(Roll) Zeilen gedreht; freie Spalten bleiben."""
    from xgonav.freespace import roll_columns
    H, W, hz, cx = 120, 160, 60.0, 80.0
    cols = np.full(W, 0.5, np.float32)
    cols[:10] = 1.0                                             # frei bis zum Horizont
    out = roll_columns(cols, hz, H, W, cx, 2.0)
    assert out[:10] == pytest.approx(1.0)
    u = (np.arange(W) + 0.5) - cx
    soll = 0.5 - u * math.tan(math.radians(2.0)) / (H - hz)
    assert out[10:] == pytest.approx(soll[10:], abs=1e-6)
    assert roll_columns(cols, hz, H, W, cx, 0.0) == pytest.approx(cols)
    # Gegenprobe: eine um 2° gerollte gerade Grenze wird wieder gerade
    schief = np.clip(0.5 + u * math.tan(math.radians(2.0)) / (H - hz), 0, 0.979).astype(np.float32)
    assert np.ptp(roll_columns(schief, hz, H, W, cx, 2.0)) < 1e-5


def test_pathfollower_align_turn_limit():
    """Drehen auf der Stelle höchstens mit home.turn_w_max (Fahrt 20261117-144500: Wende mit 0,5 störte die IMU)."""
    from dataclasses import replace
    from xgonav.behaviors import PathFollower
    cfg = Config.neutral()
    f = PathFollower(cfg.behavior, cfg.robot, cfg.home)             # Faktor 0,5 von max_w 0,5: absolut 0,25
    assert f.align(math.radians(170), "x").w == pytest.approx(cfg.robot.w_of(cfg.home.turn_w_max)) == pytest.approx(0.25)
    assert f.align(math.radians(-170), "x").w == pytest.approx(-0.25)
    f = PathFollower(cfg.behavior, cfg.robot, replace(cfg.home, turn_w_max=1.0))
    assert abs(f.align(math.radians(170), "x").w) > 0.25


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
    cfg = Config.neutral()
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
    cfg = Config.neutral()
    f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
    f.set_path([(0.0, 0.0), (3.0, 0.0)])

    def w(deg):
        return World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, 0.0, 0.05), tracks=[], free=None,
                     balancing=True, yaw=0.0, cmd_v=0.0, pose=(0.0, 0.0, -math.radians(deg)))
    assert f.drive(w(8), "T").w >= cfg.robot.w_of(cfg.home.min_w)
    assert f.drive(w(-8), "T").w <= -cfg.robot.w_of(cfg.home.min_w)
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
    cfg = Config.neutral()
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


def test_map_explore_passes_goal_without_stop_when_next_is_planned():
    """Kurz vor dem Ziel plant er abwechselnd das übernächste; liegt die Stelle im Bild, fährt er ohne Halt weiter."""
    cfg, b = _explore_ready()
    assert b.want_plan(0.0)
    b.set_path(_plan())                                        # Ziel (1.0, 0), Stelle (1.3, 0)
    assert b.step(_ex_world(0.1, (0.3, 0.0, 0.0))).v > 0       # 0,7 m vor dem Ziel: nah
    assert b.want_plan(0.1 + cfg.explore.roll_every_s + 0.01) and b._req == "next"
    nxt = _plan(goal=(2.0, 0.3), frontier=(2.3, 0.3))
    nxt.path = [(1.0, 0.0), (2.0, 0.3)]

    class P:                                                   # Planer: merkt den Auftrag
        def request_fn(self, fn, *a):
            self.a = a
            return 1
    p = P()
    b._cleared = True
    b.request_plan(p, None, _ex_world(0.6, (0.3, 0.0, 0.0)), None)
    assert p.a[1] == (1.0, 0.0, 0.0) and (1.3, 0.0, cfg.explore.avoid_r_m) in p.a[5] and p.a[6] is None   # ab dem Ziel, ohne dessen Stelle
    b.set_path(nxt)
    assert b.frontier == (1.3, 0.0) and b._next is nxt         # nur gemerkt
    c = b.step(_ex_world(1.0, (0.8, 0.0, 0.0)))                # Ziel erreicht, Stelle voraus
    assert c.v > 0 and b.state == "ziel" and b.visited == [(1.3, 0.0)]
    assert b.frontier == (2.3, 0.3) and b.f.path == nxt.path and b._next is None
    # Ohne Vorausplan oder mit der Stelle im Rücken: hindrehen und schauen wie bisher
    cfg, b = _explore_ready()
    b.want_plan(0.0)
    b.set_path(_plan())
    c = b.step(_ex_world(1.0, (0.8, 0.0, 0.0)))
    assert c.v == 0 and b.state == "face"


def test_cost_map_no_squeeze_cost_on_driven_path():
    """Wo er schon durchgefahren ist, kostet die Engstelle nichts extra (sonst Umweg statt Rückweg)."""
    from xgonav.planner import cost_map
    cfg = Config.neutral()
    lo = np.full((40, 40), cfg.grid.l_min, np.float32)
    lo[:, 17] = lo[:, 23] = cfg.grid.l_max                    # Gang, 25 cm frei zwischen den Wänden
    driven = np.zeros(lo.shape, bool)
    driven[:20, 19:22] = True
    _, cost = cost_map(lo.copy(), 0.05, cfg.grid.l_show, cfg.home, None, driven)
    assert cost[30, 20] >= cfg.home.squeeze_cost               # nicht gefahren: teuer
    assert cost[10, 20] < 2.0 + cfg.home.clear_cost            # gefahren: nur der Abstandszuschlag


def test_gridmap_mark_blocking_confirms_and_occupies():
    """Punkte, vor denen die Bahnprüfung bremst, sind in der Raumkarte bestätigt und belegt."""
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
    cfg.grid.confirm_dr_m = 0.2
    m = GridMap(cfg.grid, CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)
    pts = np.array([[0.62, 0.11], [0.63, 0.12], [0.80, -0.30]])
    assert m.mark_blocking(pts) == 2
    r, c = m.cell(pts[:, 0], pts[:, 1])
    assert (m.lo[r, c] > cfg.grid.l_show).all() and m.conf[r, c].all()
    assert len(m.occupied_cells()) == 2
    m.locked = True
    assert m.mark_blocking(np.array([[1.5, 0.0]])) == 0        # gesperrte Karte bleibt unverändert


def test_localmap_arc_clearance_reports_blocking_points():
    from xgonav.localmap import LocalMap
    lm = LocalMap(CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0), 160, 120, confirm=1)
    lm.pts = np.array([[0.5, 0.0], [0.5, 0.6]])
    lm.t = np.zeros(2)
    lm.obs = np.zeros(2, np.int64)
    hits: list = []
    assert lm.arc_clearance(0.5, 0.0, horizon_s=1.2, hit_out=hits) < 0.5
    assert len(hits) == 1 and hits[0].tolist() == [[0.5, 0.0]]


def _explore_ready():
    from xgonav.behaviors import MapExplore
    cfg = Config.neutral()
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


def test_map_explore_stuck_in_gap_blocks_goal_at_once():
    """explore.gap_block_s: Festgefahren direkt nach einer Engstelle/Lücke sperrt das Ziel sofort (RiderPi
    2026-11-23: Neu planen führte wieder durch die Lücke zwischen Stuhlbeinen); nach gap_block_s wie bisher."""
    cfg, b = _explore_ready()
    b.want_plan(0.0)
    b.set_path(_plan())
    b.f.tight_t = 0.5
    b.f.stuck = True
    b.step(_ex_world(2.0, (0.0, 0.0, 0.0)))
    assert b.blocked_goals == [(1.3, 0.0)] and b.state == "backup"
    assert b.gap_blocked                                       # App trägt die Merkkarte ringsum ein
    cfg, b = _explore_ready()
    b.want_plan(0.0)
    b.set_path(_plan())
    b.f.tight_t = 0.5
    b.f.stuck = True
    b.step(_ex_world(3.0, (0.0, 0.0, 0.0)))                    # 2,5 s nach der Engstelle: erst neu planen
    assert b.stucks == 1 and not b.blocked_goals


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
    # Am Start: Der Heimweg war 1,4 m lang, also noch einmal suchen (auf dem Weg kam Neues in die Karte)
    c = b.step(_ex_world(30.0, (0.05, 0.0, 0.0)))
    assert not b.done and b.state == "ziel" and b.rechecks == 1 and c.v == 0 and "noch einmal" in c.status
    assert b.want_plan(30.1)
    for _ in range(cfg.explore.no_goal_limit):                 # wieder kein Ziel: jetzt ist die Karte fertig
        b.set_path(ExplorePlan(None, known_m2=20.0))
    b.step(_ex_world(31.0, (0.05, 0.0, 0.0)))
    assert b.state == "heim" and b.reason == "Karte fertig"
    c = b.step(_ex_world(31.1, (0.05, 0.0, 0.0)))              # steht schon am Start: kein weiteres Suchen
    assert b.done and c.v == 0 and c.status == "Karte fertig" and b.rechecks == 1


def test_map_explore_recheck_finds_goal_and_off():
    """Findet die Suche am Start ein Ziel, erkundet er weiter; recheck_max = 0 schaltet das ab."""
    from xgonav.frontier import ExplorePlan
    cfg, b = _explore_ready()
    b.visited.append((5.0, 5.0))
    for _ in range(cfg.explore.no_goal_limit):
        b.set_path(ExplorePlan(None, known_m2=20.0))
    b.step(_ex_world(10.0, (2.0, 0.0, 0.0)))
    b.step(_ex_world(10.1, (2.0, 0.0, 0.0)))                   # Heimweg beginnt 2 m vom Start
    b.step(_ex_world(30.0, (0.05, 0.0, 0.0)))
    assert b.state == "ziel"
    b.set_path(ExplorePlan([(0.05, 0.0), (0.0, 1.5)], (0.0, 1.5), (0.0, 1.8), 1, 0, 1.0, 21.0))
    assert b.frontier == (0.0, 1.8) and b.f.path is not None and not b.done
    cfg.explore.recheck_max = 0
    cfg, b = _explore_ready()
    b.e.recheck_max = 0
    b.visited.append((5.0, 5.0))
    for _ in range(cfg.explore.no_goal_limit):
        b.set_path(ExplorePlan(None, known_m2=20.0))
    b.step(_ex_world(10.0, (2.0, 0.0, 0.0)))
    b.step(_ex_world(10.1, (2.0, 0.0, 0.0)))
    b.step(_ex_world(30.0, (0.05, 0.0, 0.0)))
    assert b.done


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
    """Der Simulator speist Fehlalarme wirklich ein (Grenzzeilen im Vollbild weiter unten = näher)."""
    from xgonav.sim import Phantoms, SimScene

    class Netz:
        def run(self, bgr):
            return np.full(80, 200.0, np.float32)          # alles frei bis knapp unter den Horizont

    class W:
        scene = SimScene()
        x, y, th, pitch = 1.0, 1.0, 0.0, 0.0
    net = Netz()
    p = Phantoms(W, net, CamGeometry(640, 480, 88.5, 1.39, 43.0, 85.0), n=1, on_prob=1.0)
    p.pts = [(2.0, 1.0)]
    p.dirs = [math.pi]                                         # sichtbar aus Richtung des Riders
    rows = net.run(None)
    assert p.hits == 1 and rows.max() > 250
    assert (rows > 250).sum() < 20                             # nur ein Fleck, nicht das ganze Bild


def test_sim_floornet_grenze_aus_ground_truth():
    """SimFloorNet: Wand voraus -> Grenze auf Höhe ihres Fußpunkts; das Bild muss eines der letzten sein."""
    from xgonav.robot import SimRobot
    from xgonav.sim import SimFloorNet, SimWorld
    cfg = Config.neutral()
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, start=(1.0, 1.2, 0.0), shake=0.0)
    world._integrate(0.0, 0.05)
    img, _, rows = world._render(0.0, 0.05)
    world.floor_rows.append((img, rows))
    net = SimFloorNet(world)
    got = net.run(img)
    assert got.shape == (80,) and np.all((got > 240) & (got <= 480))
    with pytest.raises(ValueError):
        net.run(img.copy())


def test_detections_never_block_path():
    """YOLO dient nur der Erkennung: Eine Box vor dem Rider ändert weder safety() noch die Merkkarte; die
    Raumkarte blendet bewegliche Objekte aus, die Merkkarte (Bahnprüfung) nicht (Nutzer 2026-11-13)."""
    from xgonav.behaviors import Command, World, safety
    from xgonav.egomotion import EgoResult
    from xgonav.freespace import FreeSpace
    from xgonav.localmap import mask_movable
    from xgonav.tracker import Track
    import xgonav.app as appmod
    assert not hasattr(appmod, "blocking_boxes")
    cols = np.full(160, 0.9)
    fs = FreeSpace(sectors=[0.9] * 7, horizon_row=40.0, columns=cols)
    chair = Track("chair", 320, 400, 120, 160, 0.9, 10.0)
    w = World(t=10.0, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, 10.0, 0.05), tracks=[chair], free=fs,
              balancing=True, yaw=0.0, cmd_v=0.1)
    c = safety(Command(0.1, 0.0, None, "x"), w, Config.neutral().behavior)
    assert c.v == 0.1 and "Freiraum" not in c.status
    view = mask_movable(fs, [("person", (300, 0, 340, 480))], 640, 480)
    assert np.isnan(view.columns).any() and not np.isnan(fs.columns).any()   # nur die Kopie für die Raumkarte


def test_detector_classes():
    """Wählbare Ziele: Klassen des Modells (gefiltert), vor dem Start die konfigurierten, ohne Detektor keine."""
    from xgonav.detector import COCO, DetectorWorker
    cfg = Config.neutral().detector
    cfg.classes = ["person", "chair"]
    d = DetectorWorker(cfg)
    assert d.classes() == ["person", "chair"]
    class M:
        names = ["person", "bicycle", "chair"]
        allowed = {0, 2}
    d.model = M()
    assert d.classes() == ["person", "chair"]
    M.allowed = None
    assert d.classes() == ["person", "bicycle", "chair"]
    cfg.classes = []
    assert DetectorWorker(cfg).classes() == COCO
    d2 = DetectorWorker(cfg)
    d2.failed = True
    assert d2.classes() == []
    cfg.enabled = False
    assert DetectorWorker(cfg).classes() == []


def test_follow_approach_need_known_target():
    """Ziel muss eine aktive Detektorklasse sein, sonst sucht follow bzw. erkundet approach ewig."""
    from xgonav.app import ModeNotAllowed
    app = _app(freespace=True)
    app.det.cfg.classes = ["person", "chair"]
    app.set_mode("approach", "chair")
    app.set_mode("follow", None)                         # Default target_class person
    with pytest.raises(ModeNotAllowed):
        app.set_mode("follow", "bottle")
    app.det.cfg.enabled = False
    for name in ("follow", "approach"):
        with pytest.raises(ModeNotAllowed):
            app.set_mode(name, "person")
    app.set_mode("explore")                              # andere Modi brauchen keinen Detektor


def test_detector_only_in_det_modes():
    """Nur follow/approach/pet geben dem Detektor Bilder (~1 Kern am CM4), sonst ruht er."""
    from xgonav.app import App
    from xgonav.sim import FakeDetector
    n = {}
    for mode in ("idle", "explore", "follow"):
        cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
        cfg.behavior.mode = "idle" if mode == "follow" else mode
        robot = SimRobot(cfg.robot)
        world = SimWorld(cfg.camera, robot, None)
        app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
        app.imu = None
        det = FakeDetector(world)
        cnt = [0]
        orig = det.offer
        def offer(*a, _o=orig, _c=cnt):
            _c[0] += 1
            _o(*a)
        det.offer = offer
        app.det = det
        if mode == "follow":
            app.set_mode("follow")                       # erst mit Detektor erlaubt
        app.run(duration=1.0)
        n[mode] = cnt[0]
    assert n["idle"] == 0 and n["explore"] == 0 and n["follow"] > 0


class _GeoStub:
    cx = 320.0
    def __init__(self):
        self.d = 2.0
    def object_dist(self, box, cls, pitch_dev=0.0, clip=None):
        return self.d, "stub"


def _approach_world(t, tr, cmd_v, geo):
    from xgonav.behaviors import World
    from xgonav.egomotion import EgoResult
    return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[tr], free=None,
                 balancing=True, yaw=0.0, cmd_v=cmd_v, geo=geo)


def test_approach_arrives_at_distance():
    from xgonav.behaviors import Approach
    from xgonav.tracker import Track
    cfg = Config.neutral()
    a = Approach(cfg.behavior, cfg.robot, "chair")
    geo = _GeoStub()
    tr = Track("chair", 320, 300, 100, 120, 0.9, 0.0)
    t = 0.0
    geo.d = cfg.behavior.approach_dist_m + 0.05
    while t < 3.0 and not a.done:
        tr.t_seen = t                                      # jede Runde eine neue Detektion
        c = a.step(_approach_world(t, tr, 0.0, geo))
        t += 0.05
    assert a.done and c.v == 0 and "angekommen bei chair" in a.result
    assert t <= 1.0 + 3 * 0.05 + 1e-6


def test_approach_arrives_when_blocked():
    """Bremst die Sicherheitsschicht vor dem Ziel, will das Verhalten weiter vor: nach BLOCK_S fertig."""
    from xgonav.behaviors import Approach
    from xgonav.tracker import Track
    cfg = Config.neutral()
    a = Approach(cfg.behavior, cfg.robot, "chair")
    geo = _GeoStub()
    geo.d = cfg.behavior.approach_dist_m + 0.25
    tr = Track("chair", 320, 300, 100, 120, 0.9, 0.0)
    t = 0.0
    while t < 6.0 and not a.done:
        tr.t_seen = t
        c = a.step(_approach_world(t, tr, 0.0, geo))       # gefahrener Befehl bleibt 0 (gebremst)
        t += 0.05
    assert a.done and "blockiert" in a.result and 3.0 <= t <= 3.5
    # weit weg und gebremst: nicht fertig
    b = Approach(cfg.behavior, cfg.robot, "chair")
    geo.d = 2.0
    t = 0.0
    while t < 6.0:
        tr.t_seen = t
        b.step(_approach_world(t, tr, 0.0, geo))
        t += 0.05
    assert not b.done


def _world_at(t, pose):
    from xgonav.behaviors import World
    from xgonav.egomotion import EgoResult
    return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[], free=None,
                 balancing=True, yaw=0.0, cmd_v=0.0, pose=pose, goal=(0.0, 0.0, 0.0))


def test_home_startbild_phases():
    """Mit Startbild: Planziel ist der Vorpunkt; dort ausrichten, gerade auf den Start, stehen, nach SB_RUHE_S
    messen lassen; nach einer Korrektur mit Rest > nach_m einmal nachfahren, dann auf den Startkurs drehen."""
    from xgonav.behaviors import Home
    from xgonav.startbild import Ergebnis
    cfg = Config.neutral()
    cfg.startbild.enabled = True
    b = Home(cfg.behavior, cfg.robot, cfg.home, cfg.startbild)

    def w(t, pose, anker=True):
        x = _world_at(t, pose)
        x.anker = anker
        return x

    class Plan:
        def request(self, start, ziel, keep=None, obstacles=None):
            self.ziel = ziel
            return 1
    pl = Plan()
    assert b.want_plan(0.0)
    b.request_plan(pl, None, w(0.0, (2.0, 1.0, 0.0)), None)
    assert pl.ziel == pytest.approx((-cfg.startbild.vor_m, 0.0))          # Vorpunkt vor dem Start
    b2 = Home(cfg.behavior, cfg.robot, cfg.home, cfg.startbild)           # ohne Anker: wie bisher zum Start
    b2.want_plan(0.0)
    b2.request_plan(pl, None, w(0.0, (2.0, 1.0, 0.0), anker=False), None)
    assert pl.ziel == pytest.approx((0.0, 0.0))
    # am Vorpunkt, quer stehend: ausrichten
    c = b.step(w(1.0, (-0.38, 0.0, math.radians(60))))
    assert b.phase == "dreh" and c.v == 0 and c.w < 0
    assert not b.want_plan(1.1)                                            # keine Planung mehr
    c = b.step(w(1.2, (-0.38, 0.0, math.radians(1))))
    assert b.phase == "gerade" and c.v == pytest.approx(cfg.robot.v_of(cfg.startbild.gerade_v))
    c = b.step(w(3.0, (-0.2, 0.01, 0.0)))
    assert c.v > 0 and c.w < 0                                             # zielt auf den Start (rechts)
    c = b.step(w(5.0, (-0.005, 0.0, 0.0)))
    assert b.phase == "messen" and c.v == 0 and not b.want_messung
    b.step(w(5.2, (-0.005, 0.0, 0.0)))
    assert not b.want_messung                                              # erst stehen
    b.step(w(5.6, (-0.005, 0.0, 0.0)))
    assert b.want_messung
    b.want_messung = False
    b.step(w(5.7, (-0.005, 0.0, 0.0)))
    assert not b.want_messung                                              # nur einmal anfordern
    # Korrektur: Pose liegt nun 8 cm rechts vom Start -> nachfahren
    b.messung = Ergebnis(True, "8 cm")
    c = b.step(w(6.0, (0.0, -0.08, 0.0)))
    assert b.phase == "fein" and c.v == 0 and c.w > 0                      # erst zum Start drehen
    c = b.step(w(7.0, (0.0, -0.08, math.radians(90))))
    assert c.v > 0
    c = b.step(w(8.0, (0.0, -0.005, math.radians(90))))
    assert b.phase == "ende" and c.v == 0 and c.w < 0                      # zurück auf den Startkurs
    c = b.step(w(9.0, (0.0, -0.005, math.radians(1.5))))
    assert b.done and "8 cm" in c.status


def test_home_startbild_endkurs_eng():
    """Schlussausrichtung nach dem Abgleich: enger als align_tol_deg (Fahrt 161006 blieb bei −9,5° stehen), über den
    Startkurs hinaus gedreht oder nach end_s: fertig statt pendeln."""
    from xgonav.behaviors import Home
    from xgonav.startbild import Ergebnis
    cfg = Config.neutral()
    cfg.startbild.enabled = True
    assert cfg.startbild.end_deg < cfg.home.align_tol_deg

    def neu():
        b = Home(cfg.behavior, cfg.robot, cfg.home, cfg.startbild)
        b.phase, b.t_phase = "messen", 0.0
        b.messung = Ergebnis(True, "ok")
        return b

    def w(t, deg):
        x = _world_at(t, (0.0, 0.0, math.radians(deg)))
        x.anker = True
        return x
    b = neu()
    c = b.step(w(1.0, -9.5))
    assert b.phase == "ende" and not b.done and c.w > 0                    # früher: 9,5° < 10° -> fertig
    c = b.step(w(1.5, -4.0))
    assert not b.done and c.w > 0
    b.step(w(2.0, -1.0))
    assert b.done
    b = neu()
    b.step(w(1.0, -9.5))
    b.step(w(1.5, 3.0))                                                   # übers Ziel hinaus: nicht zurückdrehen
    assert b.done
    b = neu()
    b.step(w(1.0, -9.5))
    b.step(w(1.0 + cfg.startbild.end_s + 0.1, -6.0))                      # dreht nicht: nach end_s fertig
    assert b.done


def test_home_startbild_rejected_and_stuck():
    """Messung verworfen: kein Nachfahren, fertig mit Grund. Schlussfahrt kommt nicht voran: weiter ohne Startbild."""
    from xgonav.behaviors import Home
    from xgonav.startbild import Ergebnis
    cfg = Config.neutral()
    cfg.startbild.enabled = True
    b = Home(cfg.behavior, cfg.robot, cfg.home, cfg.startbild)
    b.phase, b.t_phase = "messen", 0.0
    x = _world_at(1.0, (0.0, -0.1, 0.0))
    x.anker = True
    b.messung = Ergebnis(False, "zu wenig Treffer")
    c = b.step(x)
    assert b.done and "zu wenig Treffer" in c.status
    b = Home(cfg.behavior, cfg.robot, cfg.home, cfg.startbild)
    b.phase, b.t_phase = "gerade", 0.0
    x = _world_at(100.0, (-0.3, 0.0, 0.0))
    x.anker = True
    b.step(x)
    assert b.ohne_sb and b.phase == "weg" and b.want_plan(100.1)


def test_startbild_messung_synthetisch():
    """Rechnung von startbild.messen an einer synthetischen Szene: zufällige Texturpunkte an Wänden, Kamera wie am
    Rider; das Ende liegt 6 cm vor / 9 cm links / −4° vom Anker, die Odometrie glaubt aber (0, 0, 0). Gemessen
    wird die wahre Lage, die Korrektur führt das Ende dorthin (Rest 2026-11-19: 0,4 / 1,3 cm, 0,2°)."""
    import cv2
    from xgonav import startbild as sbm
    from xgonav.geometry import CamGeometry
    cfg = Config()
    geo = CamGeometry.from_cfg(cfg, 90.0)
    rng = np.random.default_rng(3)
    # Wände 2,5–4 m vor und seitlich, Flecken als Merkmale
    P = np.concatenate([np.c_[rng.uniform(2.5, 4, 600), rng.uniform(-3, 3, 600), rng.uniform(0.0, 1.2, 600)],
                        np.c_[rng.uniform(0.5, 3, 300), np.full(300, 1.5), rng.uniform(0.0, 1.0, 300)],
                        np.c_[rng.uniform(0.5, 3, 300), np.full(300, -1.5), rng.uniform(0.0, 1.0, 300)]])
    farben = rng.integers(0, 255, len(P))

    def bild(pose):
        a = sbm.Ansicht(np.zeros((480, 640), np.uint8), pose, 0.0, 0.0, geo.f, geo.cx, 240.0, geo.tilt_deg,
                        geo.cam_h, geo.cam_fwd)
        K, R, C = sbm.kamera(a)
        xc = (R @ (P - C).T)
        uv = ((K @ xc)[:2] / xc[2]).T
        img = np.full((480, 640), 128, np.uint8)
        for i in np.argsort(-xc[2]):
            if xc[2][i] > 0.2:
                r = max(2, int(60 / xc[2][i] / 4))
                cv2.circle(img, (int(uv[i, 0]), int(uv[i, 1])), r, int(farben[i]), -1)
        a.gray = cv2.equalizeHist(cv2.GaussianBlur(img, (3, 3), 0))
        return a
    wahr_ende = (0.06, 0.09, math.radians(-4.0))
    anker = bild((0.0, 0.0, 0.0))
    # alle Bilder des Endes: wahre Lage in der Szene, Pose laut Odometrie um wahr_ende verschoben (Drift)
    def mit_drift(pose_wahr):
        a = bild(pose_wahr)
        k = sbm.korrektur(wahr_ende, (0.0, 0.0, 0.0))
        a.pose = sbm.anwenden(k, pose_wahr)
        return a
    ende = mit_drift(wahr_ende)
    th = wahr_ende[2]
    tiefen = [mit_drift((wahr_ende[0] - d * math.cos(th), wahr_ende[1] - d * math.sin(th), th))
              for d in cfg.startbild.basen]
    erg = sbm.messen(anker, ende, tiefen, cfg.startbild)
    assert erg.ok, erg.grund
    neu = sbm.anwenden(erg.k, ende.pose)
    # ~1–2 cm Rest: seitlicher Versatz und Kurs sind bei fernen Punkten fast gleichwertig (0,2° = 1,4 cm in 4 m)
    assert neu[0] == pytest.approx(wahr_ende[0], abs=0.02) and neu[1] == pytest.approx(wahr_ende[1], abs=0.02)
    assert math.degrees(neu[2]) == pytest.approx(-4.0, abs=0.5)


def test_home_gives_up_when_stuck():
    """Hindernis im Weg: Neuplanen hilft nicht, Fahrt endet mit Meldung statt endlos zu warten."""
    from xgonav.behaviors import Home
    cfg = Config.neutral()
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
    cfg = Config.neutral()
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
    assert b.an.n == 0 and not b.gave_up


def test_map_explore_home_stuck_finishes_with_message():
    from xgonav.behaviors import MapExplore
    cfg = Config.neutral()
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
    cfg = Config.neutral()
    geo = CamGeometry(640, 480, 88.5, 5.18, 43.0, 85.0)
    m = GridMap(replace(cfg.grid, native=False, file=""), geo, 160, 120)
    r0, c0 = (int(k) for k in m.cell(-2.0, -1.0))
    r1, c1 = (int(k) for k in m.cell(2.0, 1.0))
    m.lo[r0 - 1:r1 + 1, c0 - 1:c1 + 1] = cfg.grid.l_max                 # geschlossene Wände
    m.lo[r0:r1, c0:c1] = cfg.grid.l_min
    for y in np.arange(-1.0, 1.0, 0.15):                             # Flecken bei x = 0 alle 15 cm (Lücken 10 cm)
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
    cfg = Config.neutral()
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


def _press_world(t, x, v, free_c):
    """Welt für die Drückerkennung: Pose (x, 0), Befehl v, Bodenmodell frei bis free_c (Anteil bis Horizont)."""
    from xgonav.behaviors import World
    from xgonav.egomotion import EgoResult
    from xgonav.freespace import FreeSpace
    from xgonav.geometry import CamGeometry
    cfg = Config.neutral()
    fs = FreeSpace(sectors=[free_c] * 5, valid=True) if free_c is not None else None
    return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=EgoResult(0, t, 0.05), tracks=[], free=fs, balancing=True,
                 yaw=0.0, cmd_v=v, geo=CamGeometry.from_cfg(cfg, 85.0), pose=(x, 0.0, 0.0), goal=(3.0, 0.0, 0.0))


def test_pathfollower_anschub_bei_rille():
    """Räder stehen, vorn frei: erst zweimal kurz anschieben (push_v), dann wie bisher festgefahren."""
    from xgonav.behaviors import PathFollower
    cfg = Config.neutral()
    f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
    f.set_path([(0.0, 0.0), (3.0, 0.0)])
    assert f.free_ahead_m(_press_world(0, 0.3, 0.15, 0.71)) > cfg.behavior.push_free_m
    pushes, t = [], 0.0
    while t < 12 and not f.pressing:
        c = f.drive(_press_world(t, 0.3, 0.15, 0.71), "x")
        if "Anschub" in c.status:
            assert c.v == cfg.robot.v_of(cfg.behavior.push_v) and c.w == 0
            pushes.append(round(t, 1))
        t += 0.05
    assert f.pressing and f.stuck
    starts = [p for i, p in enumerate(pushes) if i == 0 or p - pushes[i - 1] > 0.2]
    assert len(starts) == cfg.behavior.push_n and 2.5 <= starts[0] <= 2.7
    # Nach 0,3 m echtem Fortschritt sind wieder Anschübe erlaubt
    f.clear()
    f.set_path([(0.0, 0.0), (3.0, 0.0)])
    f.drive(_press_world(20.0, 0.7, 0.15, 0.71), "x")
    assert f._push_n == 0


def test_pathfollower_kein_anschub_ohne_freiraum():
    """Vorn kaum frei (Wand) oder Bodenmodell unbekannt: kein Anschub, gleich festgefahren."""
    from xgonav.behaviors import PathFollower
    cfg = Config.neutral()
    for free_c in (0.05, None):
        f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
        f.set_path([(0.0, 0.0), (3.0, 0.0)])
        t = 0.0
        while t < 4:
            c = f.drive(_press_world(t, 0.3, 0.15, free_c), "x")
            assert "Anschub" not in c.status
            t += 0.05
        assert f.pressing


def test_pathfollower_schneller_anschub_an_fuge():
    """Räder stehen schlagartig (Fuge vor einer Stufe): nach kante_s Anschub, auch wenn safety/Bahnprüfung den
    Fahrwert schon gesenkt haben; Strecke auf freien Boden minus push_rest_m begrenzt, dann Vollbremsung."""
    import dataclasses
    from xgonav.behaviors import Command, PathFollower, safety
    cfg = Config.neutral()
    b = cfg.behavior
    f = PathFollower(b, cfg.robot, cfg.home)
    f.set_path([(0.0, 0.0), (3.0, 0.0)])

    def welt(t, x, v, vr, clear=0.12):
        w = _press_world(t, x, v, 0.71)
        return dataclasses.replace(w, v_rad=vr, clear_fn=lambda: clear)
    t, x = 0.0, 0.0
    while t < 1.0:                                       # fährt mit 0,25 m/s
        assert "Anschub" not in f.drive(welt(t, x, 0.23, 0.25), "x").status
        x += 0.25 * 0.05
        t += 0.05
    st = []
    while t < 2.0:                                       # Fuge: Räder stehen, Fahrwert sinkt (Freiraum!/Bahn)
        c = f.drive(welt(t, x, 0.23 if t < 1.3 else 0.036, 0.0), "x")
        st.append((round(t, 2), c))
        t += 0.05
    push = [(tt, c) for tt, c in st if "Anschub" in c.status]
    assert push and 1.45 <= push[0][0] <= 1.6, push[:1]
    c = push[0][1]
    assert c.v == cfg.robot.v_of(b.push_v) and c.extra.get("anschub")
    assert abs(f._push_budget - (0.12 - b.push_rest_m)) < 1e-9
    # safety() lässt den Anschub trotz „Freiraum!“ durch (Bodenmodell sieht die Stufe)
    w = dataclasses.replace(_press_world(t, x, 0.2, 0.05), v_rad=0.0)
    vp = cfg.robot.v_of(b.push_v)
    assert safety(Command(vp, 0.0, extra={"anschub": True}), w, b).v == vp
    assert safety(Command(b.push_v, 0.0), w, b).v == 0.0
    # Er kommt frei und rollt: Bremsweg eingerechnet, Ende mit Vollbremsung vor der Stufe
    x0, last = x, None
    for _ in range(20):
        last = f.drive(welt(t, x, b.push_v, 0.2), "x")
        if "Anschub" in last.status and last.v == 0:
            break
        x += 0.01
        t += 0.05
    assert last.v == 0 and last.extra.get("bremsen")
    assert x - x0 + 0.2 * b.push_brake_s >= f._push_budget and x - x0 < f._push_budget


def test_pathfollower_kein_schneller_anschub_ohne_platz_oder_beim_bremsen():
    """Kein Platz vor der Kante (Merkkarte) oder Fahrwert war schon 0 (Bremsen): kein schneller Anschub."""
    import dataclasses
    from xgonav.behaviors import PathFollower
    cfg = Config.neutral()
    for clear, v_stop in ((0.06, 0.23), (1.0, 0.0)):
        f = PathFollower(cfg.behavior, cfg.robot, cfg.home)
        f.set_path([(0.0, 0.0), (3.0, 0.0)])
        t = 0.0
        while t < 2.0:
            moving = t < 1.0
            w = dataclasses.replace(_press_world(t, min(t, 1.0) * 0.25, 0.23 if moving else v_stop, 0.71),
                                    v_rad=0.25 if moving else 0.0, clear_fn=lambda c=clear: c)
            assert "Anschub" not in f.drive(w, "x").status
            t += 0.05


def test_map_explore_befreien_erkennt_druecken():
    """„befreie mich“ (reaktiv) drückt nicht mehr wander_s lang: Anschub, dann zurücksetzen."""
    from xgonav.behaviors import MapExplore
    cfg = Config.neutral()
    b = MapExplore(cfg.behavior, cfg.robot, cfg.home, cfg.explore)
    b.state, b.until = "wander", 1e9
    st, t = [], 0.0
    while t < 10 and b.state == "wander":
        st.append(b.step(_press_world(t, 0.3, 0.15, 0.71)).status)
        t += 0.05
    assert b.state == "backup" and t < 8                 # 2,5 s + 2 Anschübe à push_s 1,2 s + je 1 s danach
    assert any("befreie mich – Anschub" in s for s in st) and st[-1] == "befreie mich: hänge fest"


def test_map_explore_contact_blocks_goal_at_once():
    from xgonav.behaviors import MapExplore
    cfg = Config.neutral()
    b = MapExplore(cfg.behavior, cfg.robot, cfg.home, cfg.explore)
    pose = (1.0, 0.0, 0.0)
    b.state, b.frontier, b.goal, b.t_goal = "ziel", (3.0, 0.0), (2.7, 0.0), 0.0
    b.f.set_path([pose[:2], (2.7, 0.0)])
    b.f.stuck = b.f.pressing = True
    c = b.step(_world_at(1.0, pose))
    assert b.state == "backup" and b.blocked_goals == [(3.0, 0.0)] and c.v == 0 and not b.f.pressing


def test_grid_heading_from_imu():
    """Raumkarte und Verhalten drehen mit dem Kurs der Koppelnavigation: IMU zur Bildzeit, fehlt die, Räder bzw.
    Bild. Die Anzeige (kurs_deg) summiert über ±180° weiter."""
    from xgonav.imu import ImuSource
    app = _app(freespace=True)
    now = time.monotonic()
    app.imu = ImuSource(app.cfg.imu)
    assert app.kurs_deg() is None
    app.imu.add(1, now - 0.01, 0.0, 0.0, math.radians(179.0))
    app.imu.add(2, now, 0.0, 0.0, math.radians(179.0))
    assert app._odom_step(now, app.imu.at(now)[2], 0.5, 0.05).kurs_src == "kamera"   # erster Wert: Bezug
    app.imu.add(3, now + 0.01, 0.0, 0.0, math.radians(-179.0))         # über die Naht: +2° (mal yaw_scale_left)
    d = 2.0 * app.cfg.imu.yaw_scale_left
    st = app._odom_step(now + 0.01, app.imu.at(now + 0.01)[2], 0.5, 0.05)
    assert abs(st.dyaw - math.radians(d)) < 1e-9 and st.kurs_src == "imu"
    assert app.kurs_deg() == round(179.0 + d, 1)                       # nicht gewickelt
    app.imu = None                                                     # keine IMU: Rückfall
    assert app._odom_step(now + 0.05, None, 0.5, 0.05).kurs_src == "kamera" and app.kurs_deg() is None


def _fusion(**kw):
    from xgonav.odomfusion import OdomFusion
    cfg = Config.neutral().odom
    for k, v in kw.items():
        setattr(cfg, k, v)
    return OdomFusion(cfg)


def _fahre(f, t, yaw, sl, sr, n, dyaw_imu, dl, dr, dt=0.04, vis=None, jumps=0):
    """n Schritte: IMU dreht dyaw_imu je Schritt, Räder dl/dr je Schritt; vis = Bild-Drehung je Schritt."""
    out = []
    for _ in range(n):
        t += dt; yaw += dyaw_imu; sl += dl; sr += dr
        out.append(f.step(t, yaw, jumps, (sl, sr), vis, 0.0))
    return t, yaw, sl, sr, out


def test_odomfusion_imu_heading_wheel_distance():
    """Ohne Widerspruch: Kurs aus der IMU, Strecke als Mittel beider Räder."""
    f = _fusion()
    f.step(0.0, 0.0, 0, (0.0, 0.0), None, 0.0)
    th = math.radians(0.5)
    tr = f.cfg.track_m
    _, _, _, _, out = _fahre(f, 0.0, 0.0, 0.0, 0.0, 25, th, 0.004 - tr / 2 * th, 0.004 + tr / 2 * th)
    assert all(s.kurs_src == "imu" and s.ds_src == "räder" for s in out)
    assert sum(s.ds for s in out) == pytest.approx(0.1) and sum(s.dyaw for s in out) == pytest.approx(25 * th)
    assert f.widersprueche == 0 and not f.schlupf


def test_odomfusion_slip_keeps_imu_and_uses_matching_wheel():
    """Drehen auf glattem Boden: das rechte Rad dreht durch (Räder sagen 3 x so viel Drehung wie die IMU).
    Kurs bleibt IMU, Strecke kommt aus dem Rad, das zur IMU-Drehung passt (links)."""
    f = _fusion()
    f.step(0.0, 0.0, 0, (0.0, 0.0), None, 0.0)
    tr = f.cfg.track_m
    th = math.radians(2.0)                                             # IMU: 2° je Schritt links, Mitte steht
    dl = -tr / 2 * th                                                  # links passt dazu
    dr = 5 * tr / 2 * th                                               # rechts dreht durch
    _, _, _, _, out = _fahre(f, 0.0, 0.0, 0.0, 0.0, 25, th, dl, dr)
    assert f.schlupf and f.widersprueche == 1 and f.imu_zweifel == 0
    assert out[-1].kurs_src == "imu" and out[-1].ds_src == "schlupf"
    assert abs(out[-1].ds) < 1e-9                                     # Mitte steht: links + Spur/2 x Drehung
    assert sum(s.dyaw for s in out) == pytest.approx(25 * th)


def test_odomfusion_camera_overrules_imu_when_turning_in_place():
    """Drehen im Stand, IMU sagt nichts (Kurs hängt), Räder und Bild sagen 2° je Schritt: Die IMU ist
    zweifelhaft, der Kurs kommt imu_doubt_s lang aus den Rädern."""
    f = _fusion()
    f.step(0.0, 0.0, 0, (0.0, 0.0), 0.0, 0.0)
    tr = f.cfg.track_m
    th = math.radians(2.0)
    t, yaw, sl, sr, out = _fahre(f, 0.0, 0.0, 0.0, 0.0, 25, 0.0, -tr / 2 * th, tr / 2 * th, vis=th)
    assert f.imu_zweifel == 1 and out[-1].kurs_src == "räder"
    assert sum(s.dyaw for s in out) == pytest.approx(25 * th)         # was die IMU im Fenster verpasste, nachgeholt
    t, *_ , out = _fahre(f, t + f.cfg.imu_doubt_s, yaw, sl, sr, 2, 0.0, 0.0, 0.0)
    assert out[-1].kurs_src == "imu"                                   # Zweifel abgelaufen


def test_odomfusion_camera_witness_depends_on_its_error():
    """Bei Fahrt ist der Bild-Kurs verfälscht (~5°/m): Ein kleiner Widerspruch (10° bei 1 m/s) bleibt Schlupf, ein
    grober (IMU schlägt beim Kippen aus, Rohfahrt 20261117-110738) wird vom Bild entschieden, das Fenster nachgeholt."""
    tr = Config.neutral().odom.track_m
    f = _fusion()
    f.step(0.0, 0.0, 0, (0.0, 0.0), 0.0, 0.0)
    th = math.radians(0.4)
    _, _, _, _, out = _fahre(f, 0.0, 0.0, 0.0, 0.0, 25, 0.0, 0.04 - tr / 2 * th, 0.04 + tr / 2 * th, vis=th)
    assert f.imu_zweifel == 0 and f.schlupf and out[-1].kurs_src == "imu"
    f = _fusion()
    f.step(0.0, 0.0, 0, (0.0, 0.0), 0.0, 0.0)
    th = math.radians(0.4)
    _, _, _, _, out = _fahre(f, 0.0, 0.0, 0.0, 0.0, 25, math.radians(-6.0), 0.016 - tr / 2 * th,
                             0.016 + tr / 2 * th, vis=th)
    assert f.imu_zweifel == 1 and out[-1].kurs_src == "räder"
    assert sum(s.dyaw for s in out) == pytest.approx(25 * th, abs=math.radians(1.0))


def test_odomfusion_fallbacks():
    """IMU fehlt: Kurs Räder; Räder fehlen: Strecke aus dem Fahrbefehl; beides fehlt: Bild; IMU-Sprung: in
    dem Schritt Räder."""
    f = _fusion()
    assert f.step(0.0, 0.0, 0, (0.0, 0.0), None, 0.0).ds_src == "räder"
    s = f.step(0.04, None, 0, (0.0, 0.01), 0.3, 0.0)
    assert s.kurs_src == "räder" and s.dyaw == pytest.approx(0.01 / f.cfg.track_m)
    s = f.step(0.08, 0.1, 0, None, 0.3, 0.02)
    assert s.kurs_src == "kamera" and s.ds == 0.02 and s.ds_src == "befehl"
    f.step(0.12, 0.2, 0, (0.0, 0.0), None, 0.0)
    s = f.step(0.16, 0.2, 1, (0.0, 0.0), None, 0.0)                    # Zähler der IMU-Sprünge ändert sich
    assert s.kurs_src == "räder" and s.dyaw == 0.0
    assert f.step(0.20, 0.25, 1, (0.0, 0.0), None, 0.0).kurs_src == "imu"


def test_odomfusion_calibration_suggestion():
    """Vorschlag für Radfaktoren und Spur aus Abschnitten gegen den IMU-Kurs: falsche Spur (9,5 statt 9 cm)
    und rechts 2 % zu kleines Rad werden wiedergefunden."""
    f = _fusion(scale_left=1.0, scale_right=1.0, calib_seg_s=1.0)
    tr_echt, kr_echt = 0.095, 1.02
    t, yaw, sl, sr = 0.0, 0.0, 0.0, 0.0
    f.step(t, yaw, 0, (sl, sr), None, 0.0)
    for k in range(80):                                                # abwechselnd fahren und drehen, 2 s je Stück
        v, w = (0.1, 0.0) if k % 2 == 0 else (0.0, (1.0 if k % 4 == 1 else -1.0))
        for _ in range(50):
            t += 0.04
            yaw += w * 0.04
            l_echt, r_echt = (v - w * tr_echt / 2) * 0.04, (v + w * tr_echt / 2) * 0.04
            sl += l_echt; sr += r_echt / kr_echt                       # Rad rechts zählt 2 % zu wenig
            f.step(t, yaw, 0, (sl, sr), None, 0.0)
    v = f.vorschlag
    assert v is not None and v["n"] > 50
    assert v["track_m"] == pytest.approx(tr_echt * 2 / (1 + kr_echt), rel=0.01)
    assert v["scale_right"] / v["scale_left"] == pytest.approx(kr_echt, rel=0.005)


def test_home_stuck_near_goal_counts_as_arrived():
    from xgonav.behaviors import Home, MapExplore
    cfg = Config.neutral()
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
    m.state, m.reason, m.home_an.n = "heim", "Karte fertig", 1
    c = m.step(_world_at(1.0, pose))
    assert m.done and c.v == 0 and "daneben" in m.reason


def _patrol(points, loop=False):
    from xgonav.behaviors import Patrol
    cfg = Config.neutral()
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
    cfg = Config.neutral()
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
    fs = FreeSpace([0.5] * 5, np.full(160, 0.4, np.float32), hz, True, None)
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


def test_arc_check_bremst_voll_wenn_raeder_zu_schnell():
    """Fahrt 2026-11-23 vor dem Stuhl: Der Rider setzt kleinere Fahrwerte erst nach ~1-2 s um. Die Bahnprüfung
    rechnet deshalb mit der Radgeschwindigkeit, wenn sie höher ist (Vorschau weiter als der Bremsabstand, Bogen
    gleich), und bremst voll (Fahrwert 0, ohne Rampe), wenn er schneller ist, als die Bahn erlaubt."""
    from xgonav.behaviors import Command
    app = _app(freespace=True)
    app.cfg.robot.vx_limit = 1.8
    rufe = []

    class Merk:
        radius = 0.2
        frei = 0.5

        def arc_clearance(self, v, w, horizon_s=1.0, radius=None, hit_out=None):
            rufe.append((v, w, horizon_s))
            return self.frei
    app.lmap = Merk()
    app._v_rad = None                                                   # ohne Räder: wie bisher anteilig
    c = app._arc_check(Command(0.33, 0.0, None, "x"))
    assert 0 < c.v < 0.33 and "bremsen" not in c.extra and "bremse" not in c.status
    app._v_rad = 0.6                                                    # Räder 0,6 m/s, Befehl nur 0,18 m/s
    c = app._arc_check(Command(0.1, 0.1, None, "x"))
    v, w, h = rufe[-1]
    assert v == pytest.approx(0.6) and w == pytest.approx(math.radians(0.1 * app.cfg.robot.vyaw_limit_dps) * 0.6 / 0.18)
    need = 0.6 * 0.6 + 0.08 + 0.8 * 0.3
    assert h == pytest.approx((need + 0.15) / 0.6) and h > 1.2
    assert c.v == 0.0 and c.extra.get("bremsen") and "bremse" in c.status
    app._v_rad = 0.1                                                    # langsam genug: anteilig wie bisher
    Merk.frei = 0.15
    c = app._arc_check(Command(0.1, 0.0, None, "x"))
    assert 0 < c.v < 0.1 and "bremsen" not in c.extra
    app._v_rad = -0.6                                                   # Räder rückwärts zählen vorwärts nicht
    Merk.frei = 0.5
    c = app._arc_check(Command(0.1, 0.0, None, "x"))
    assert c.v == 0.1 and rufe[-1][0] == pytest.approx(0.18)


def test_robot_vollbremsung_ohne_rampe():
    """drive(brake=True) mit v = 0 setzt den Fahrwert im nächsten Rampenschritt auf 0; ohne brake über die Rampe."""
    r = SimRobot(Config.neutral().robot)
    r._cur, r._last_ramp_t = (0.3, 0.1), 0.0
    r.drive(0.0, 0.1)
    r._ramp(r._target, 0.02)
    assert 0.2 < r._cur[0] < 0.3
    r.drive(0.0, 0.1, brake=True)
    r._ramp(r._target, 0.04)
    assert r._cur[0] == 0.0 and r._cur[1] > 0
    r.drive(0.2, 0.0, brake=True)                                       # brake gilt nur für v = 0
    r._ramp(r._target, 0.06)
    assert 0 < r._cur[0] < 0.2


def test_dunkelzone_sperrt_wie_sperrzone_ohne_speichern(tmp_path):
    """Dunkelzonen (gridmap.dark_zones) wirken auf Planer und Zielsuche wie Sperrzonen, stehen aber nicht in
    der Route-Datei und sind nach dem Leeren wieder weg."""
    import json
    from xgonav.frontier import next_goal
    from xgonav.planner import plan
    from xgonav.route import save
    cfg, m = _route_map()
    zone = [(1.0, -4.0), (1.5, -4.0), (1.5, 4.0), (1.0, 4.0)]
    _, c0 = (int(k) for k in m.cell(2.0, -3.0))
    m.lo[:, c0:] = 0.0                                                  # ab x = 2 m unbekannt
    frei = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert frei.path is not None and frei.path[-1][0] > 1.0
    m.dark_zones = [zone]
    assert m.all_zones() == [zone] and m.zones == []
    assert plan(m, (0.0, 0.0), (2.5, 0.0), cfg.home) is None
    r = next_goal(m, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert r.path is None or all(p[0] < 1.0 for p in r.path)
    datei = str(tmp_path / "karte.npz")
    assert save(datei, m)
    gespeichert = [json.loads(p.read_text()) for p in tmp_path.iterdir()]
    assert len(gespeichert) == 1 and gespeichert[0]["zones"] == []
    m.dark_zones = []
    assert plan(m, (0.0, 0.0), (2.5, 0.0), cfg.home) is not None


def test_app_dunkelzone_nur_explore_und_auf_zeit():
    """Angst im Erkunden: Rechteck voraus gesperrt, Verhalten plant neu, Bahnprüfung hält; nach zone_s weg.
    In anderen Modi und mit zone_s = 0 entsteht keine Zone."""
    from types import SimpleNamespace
    from xgonav.behaviors import Command
    app = _app(freespace=True)
    gm, c = app.gridmap, app.cfg.dunkel
    neu = []
    app.behavior = SimpleNamespace(name="follow", neu_planen=lambda: neu.append(1))
    app._dunkel_zone(10.0)
    assert gm.dark_zones == [] and not neu                              # nur in zone_modes
    app.behavior.name = "explore"
    c.zone_s = 0.0
    app._dunkel_zone(10.0)
    assert gm.dark_zones == []
    c.zone_s = 600.0
    app._dunkel_zone(10.0)
    assert len(gm.dark_zones) == 1 and neu == [1] and gm.zones == []
    assert gm.stats(0.0)["dunkelzonen"] == 1
    k = app._arc_check(Command(0.15, 0.0, 100.0, "x"))
    assert k.v == 0 and "Sperrzone" in k.status                         # voraus zu
    assert "Sperrzone" not in app._arc_check(Command(-0.1, 0.0, 100.0, "x")).status
    app._dunkel_tick(10.0 + c.zone_s - 1.0, Command(0.0, 0.0, 100.0, "x"))
    assert len(gm.dark_zones) == 1
    app._dunkel_tick(10.0 + c.zone_s + 1.0, Command(0.0, 0.0, 100.0, "x"))
    assert gm.dark_zones == [] and app._dunkel_zonen == []
    app._dunkel_zone(700.0)
    gm.reset()                                                          # neue Karte: Zonen weg, kommen nicht wieder
    app._dunkel_tick(701.0, Command(0.0, 0.0, 100.0, "x"))
    assert gm.dark_zones == [] and app._dunkel_zonen == []


def test_ruhe_mode():
    """Ruhemodus: absetzen, Bildverarbeitung aus, Treiber-Abfragen aus, keine Fahrbefehle; Wecken stellt nur
    die Abfragen wieder her (bleibt liegen, Bildverarbeitung aus)."""
    import threading
    from xgonav.app import App, ModeNotAllowed
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    cfg.ruhe.tick_s = 0.2
    robot = SimRobot(cfg.robot)
    robot.settings.update({"imu_hz": "50", "imu_stream_hz": "100", "sleep_ms_on_loop": "200"})
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    app = App(cfg, robot, world, imu=None, floor_net=SimFloorNet(world))
    app.imu = None
    th = threading.Thread(target=app.run, kwargs={"duration": 6.0}, daemon=True)
    th.start()
    time.sleep(1.0)
    assert app.pipeline_on and robot.state.balancing
    app.set_ruhe(True)
    assert app.ruhe and not app.pipeline_on and not robot.state.balancing
    assert robot.settings == {"imu_hz": "0", "imu_stream_hz": "0", "sleep_ms_on_loop": "1000"}
    with pytest.raises(ModeNotAllowed):
        app.set_pipeline(True)
    with pytest.raises(ModeNotAllowed):
        app.set_mode("manual")
    time.sleep(0.5)
    n = len(robot.writes)
    idx = app.snapshot.idx
    time.sleep(1.0)
    assert len(robot.writes) == n                        # kein Keepalive, keine Fahrbefehle
    assert app.snapshot.idx == idx and app.mode == "idle"
    app.set_ruhe(False)
    assert not app.ruhe and robot.ruhe_tick_s is None
    assert robot.settings == {"imu_hz": "50", "imu_stream_hz": "100", "sleep_ms_on_loop": "200"}
    time.sleep(0.5)
    assert not app.pipeline_on and not robot.state.balancing   # Wecken startet nichts
    app.stop(); th.join(timeout=5)


# ---------------------------------------------------------------- Weboberfläche: Einstellungen
def test_config_schema_aktuell():
    """web/config-schema.json passt zu xgonav/config.py (sonst: python tools/config_schema.py)."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import config_schema
    assert (Path(config_schema.ZIEL).read_text(encoding="utf-8") == config_schema.text()), \
        "web/config-schema.json veraltet: python tools/config_schema.py"


def test_gesendet_hoechstens_vollausschlag():
    """robot.max_sent ist weg (2026-11-23): Gesendet wird nur auf ±1 begrenzt, auch mit Korrektur der
    Geschwindigkeitsregelung; der alte Schlüssel in einer config.toml ergibt nur eine Warnung."""
    from xgonav.config import ENTFERNT
    from xgonav.robot import SimRobot
    assert {"robot.max_sent", "robot.manual_forward_gain", "robot.reverse_gain"} <= set(ENTFERNT)
    cfg = Config.neutral()
    r = SimRobot(cfg.robot)
    r.v_trim = 0.1
    for cur, soll in (((0.5, 0.0), norm_to_u8(0.6)), ((0.95, 0.0), norm_to_u8(1.0)), ((-0.5, 0.0), norm_to_u8(-0.4))):
        r._cur = cur
        r._sent = None
        r._send(time.monotonic() + 10)
        assert abs(r._sent[0] - soll) <= 1, (cur, r._sent)

def test_frontier_engstelle_voraus_mit_zuschlag(monkeypatch):
    """Engstellen-Ziele konkurrieren mit Zuschlag tight_penalty_s; -1 = alte Regel (nur, wenn kein Ziel mit Abstand)."""
    from xgonav import frontier
    from xgonav.geometry import CamGeometry
    from xgonav.gridmap import GridMap
    cfg = Config.neutral()
    cfg.home.native = False
    gm = GridMap(cfg.grid, CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)
    r, c = gm.cell(0.0, 0.0)
    gm.lo[int(r) - 20:int(r) + 21, int(c) - 20:int(c) + 21] = -2.0          # 2 m x 2 m frei um den Rider
    ziele = {}

    def gruppen(lab, nlab, zone, behind, dist, step, gain_cells, avoid, pose, r0, c0, n, res, keep_rc, ecfg):
        # Gruppe 1 mit Abstand 0,8 m hinter ihm (Wert -6), Gruppe 2 Engstelle 0,8 m voraus (Wert -1)
        code = np.array([0, 1, 2], np.int8)
        score = np.array([0.0, -6.0, -1.0])
        cr, cc = np.zeros(3, np.int32), np.zeros(3, np.int32)
        for k, (x, y) in ((1, (-0.8, 0.0)), (2, (0.8, 0.0))):
            rr, cc_ = gm.cell(x, y)
            cr[k], cc[k] = int(rr) - r0, int(cc_) - c0
            ziele[k] = (x, y)
        z = np.zeros(3, np.int32)
        return code, score, np.zeros(3), cr, cc, np.full(3, 10 ** 9, np.int64), z, z

    monkeypatch.setattr(frontier, "_groups_py", gruppen)
    monkeypatch.setattr(frontier, "route", lambda *a: [a[5], a[6]])
    # Frontier-Zellen (Rand des freien Quadrats) braucht next_goal, damit es überhaupt Gruppen wertet
    cfg.explore.tight_penalty_s = 3.0
    p = frontier.next_goal(gm, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert p.tight and p.frontier[0] > 0.5                                   # -1 - 3 = -4 schlägt -6: geradeaus
    cfg.explore.tight_penalty_s = 6.0
    p = frontier.next_goal(gm, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert not p.tight and p.frontier[0] < -0.5                              # -7 gegen -6: das Ziel hinter ihm
    cfg.explore.tight_penalty_s = -1.0
    p = frontier.next_goal(gm, (0.0, 0.0, 0.0), cfg.home, cfg.explore)
    assert not p.tight and p.frontier[0] < -0.5                              # alte Regel: Abstand geht vor
    p = frontier.next_goal(gm, (0.0, 0.0, 0.0), cfg.home, cfg.explore, allow_tight=False)
    assert not p.tight


def test_localmap_traegt_an_der_aufnahmepose_ein():
    """Ergebnis aus dem Bodenmodell-Thread (FreeSpace.odo): Eintragen an der Aufnahmepose und danach mitfahren
    ergibt dasselbe wie sofort eintragen und dann fahren (beide Wege: C++ und numpy)."""
    from xgonav.freespace import FreeSpace
    from xgonav.localmap import LocalMap
    geo = CamGeometry(640, 480, 91.0, 0.0, 43.0, 85.0)
    fw, fh = 160, 120
    cols = np.full(fw, 0.5)
    cols[:40] = np.nan
    for native in (True, False):
        a = LocalMap(geo, fw, fh, native=native)
        b = LocalMap(geo, fw, fh, native=native)
        for lm in (a, b):
            lm.move(0.3, 0.2)                              # irgendeine Vorgeschichte
            lm.add_freespace(FreeSpace([1.0] * 5, np.full(fw, 0.7), fh / 2, True), 0.0, 0.0)
        odo = b.odo
        a.add_freespace(FreeSpace([1.0] * 5, cols, fh / 2, True), 0.1, 0.01)
        for lm in (a, b):
            lm.move(0.05, 0.15)
            lm.move(0.04, -0.05)
        fs = FreeSpace([1.0] * 5, cols, fh / 2, True)
        fs.odo = odo
        b.add_freespace(fs, 0.1, 0.01)
        assert len(a.pts) == len(b.pts) > 0
        assert np.allclose(np.sort(a.pts, axis=0), np.sort(b.pts, axis=0), atol=1e-9)
        assert a.odo == b.odo


def test_render_netzlinie_auf_aktuelles_bild():
    """Die FloorNet-Linie gehört zu einem älteren Bild: beim Zeichnen mit Nick und Wegsumme umrechnen."""
    from xgonav.app import Snapshot, net_rows_jetzt
    from xgonav.freespace import FreeSpace
    geo = CamGeometry(640, 480, 91.8, 0.0, 109.0, 85.0)
    rows = np.full(80, 330.0, np.float32)
    fs = FreeSpace([1.0] * 5, None, 60.0, True, None, 0.0)
    fs.net_rows, fs.odo = rows, (0.0, 0.0, 0.0)
    gleich = net_rows_jetzt(Snapshot(free=fs, pitch_dev=0.0, odo=(0.0, 0.0, 0.0), geo=geo))
    assert np.allclose(gleich, rows)
    # Kamera nickt 2° hoch: alles im Bild wandert nach unten (wie horizon_row)
    hoch = net_rows_jetzt(Snapshot(free=fs, pitch_dev=math.radians(2), odo=(0.0, 0.0, 0.0), geo=geo))
    assert np.all(hoch > rows + 5)
    # vorgefahren: dieselbe Bodenstelle ist näher, also tiefer im Bild
    vor = net_rows_jetzt(Snapshot(free=fs, pitch_dev=0.0, odo=(0.05, 0.0, 0.0), geo=geo))
    assert np.nanmedian(vor) > 335
    # ohne Geometrie (alte Aufrufer): unverändert
    assert np.allclose(net_rows_jetzt(Snapshot(free=fs, pitch_dev=0.1)), rows)


def test_imu_kurs_skalenfaktor():
    """[imu] yaw_scale_left/right strecken Kursänderungen je Drehrichtung (Skalenfehler des Gyros), auch über den
    ±180°-Sprung; der erste Wert bleibt, wie er ist."""
    from xgonav.imu import ImuSource
    cfg = Config.neutral()
    assert cfg.imu.yaw_scale_left == 1.0 and cfg.imu.yaw_scale_right == 1.0
    assert Config().imu.yaw_scale_left == pytest.approx(1.0028) and Config().imu.yaw_scale_right == pytest.approx(1.0043)
    cfg.imu.yaw_scale_left, cfg.imu.yaw_scale_right = 1.01, 1.02
    imu = ImuSource(cfg.imu)
    t0 = time.monotonic()
    grad = [30.0 + 10.0 * k for k in range(73)] + [750.0 - 10.0 * k for k in range(1, 73)]   # 720° links, 720° zurück
    for k, deg in enumerate(grad):
        imu.add(k + 1, t0 + 0.05 * k, 0.0, 0.0, math.radians((deg + 180.0) % 360.0 - 180.0))   # 200 °/s
    assert math.degrees(imu.at(t0)[2]) == pytest.approx(30.0)
    assert math.degrees(imu.at(t0 + 3.6)[2]) == pytest.approx(30.0 + 720.0 * 1.01)
    assert math.degrees(imu.at(t0 + 7.2)[2]) == pytest.approx(30.0 + 720.0 * 1.01 - 720.0 * 1.02)


def test_linse_entzerrt_gerade():
    """camera.lens_maps: Ein Bild, das mit dem Linsenmodell verzeichnet wurde, kommt entzerrt wieder gerade heraus;
    ohne Modell (Config.neutral) gibt es keine Tabellen."""
    import dataclasses
    import cv2
    from xgonav.camera import lens_maps
    assert lens_maps(Config.neutral().camera) is None
    cam = dataclasses.replace(Config().camera, lens_roll_deg=0.0, lens_yaw_deg=0.0)     # nur die Radialverzeichnung
    maps = lens_maps(cam)
    assert maps is not None
    kn, g, c = np.array(cam.lens_knots_px, float), np.array(cam.lens_g_px, float), np.array(cam.lens_center, float)
    # ideales Bild: Geraden; verzeichnetes Bild = ideales Bild an den Stellen vor(p) abgetastet
    ideal = np.zeros((cam.height, cam.width), np.uint8)
    for y in (420, 450, 470):
        cv2.line(ideal, (0, y), (cam.width - 1, y - 60), 255, 7)
    ideal = cv2.GaussianBlur(ideal, (9, 9), 2.0)          # weiche Kanten: der Vergleich misst die Lage, nicht die Interpolation
    gx, gy = np.meshgrid(np.arange(cam.width, dtype=np.float64), np.arange(cam.height, dtype=np.float64))
    q = np.stack([gx, gy], -1) - c
    r = np.hypot(q[..., 0], q[..., 1])
    v = c + q * (1.0 + np.interp(r, kn, g) / np.maximum(r, 1e-6))[..., None]
    roh = cv2.remap(ideal, v[..., 0].astype(np.float32), v[..., 1].astype(np.float32), cv2.INTER_LINEAR)
    ent = cv2.remap(roh, maps[0], maps[1], cv2.INTER_LINEAR)
    assert np.abs(roh.astype(int) - ideal).mean() > 3 * np.abs(ent.astype(int) - ideal).mean()
    innen = (slice(20, cam.height - 20), slice(20, cam.width - 20))
    assert np.abs(ent[innen].astype(int) - ideal[innen]).mean() < 1.0


def test_linse_roll_gier():
    """Einbau-Roll und -Gier in der Entzerrtabelle: Der Roll wirkt genau wie früher [imu] roll_ref_deg in
    freespace.roll_columns, die Fahrtrichtung (früher cx_offset_px 13, Spalte 333) landet bei W/2; keine Quelle
    außerhalb des Bilds (Rand aufgefüllt); vorwärts und rückwärts passen zusammen; die Kennung hängt am Block."""
    import dataclasses
    import cv2
    from xgonav import camera as C
    from xgonav.freespace import roll_columns
    cam = Config().camera
    assert C.lens_id(Config.neutral().camera) is None
    # Roll: Grenzzeilen im Bild ohne Drehung bei IMU-Roll 0, die roll_columns (roll_deg = 0 - roll_ref) waagerecht macht
    nur_roll = dataclasses.replace(cam, lens_yaw_deg=0.0)
    H, W, hor = cam.height, cam.width, 200.0
    u = (np.arange(80) + 0.5) * W / 80
    rows = 300.0 + (u - W / 2) * math.tan(math.radians(cam.lens_roll_deg))
    alt = H - roll_columns((H - rows) / (H - hor), hor, H, W, W / 2, -cam.lens_roll_deg) * (H - hor)
    assert np.ptp(alt) < 0.01
    neu = C._homog(C.lens_rot(nur_roll), np.stack([u, rows], 1))[:, 1]
    assert np.ptp(neu) < 0.05
    # Gier: Fluchtpunkt der Geradeausfahrt (Rohspalte 333, Zeile 240) -> Bildmitte
    x, y = C.lens_forward(cam, np.array([333.0, 240.0]))
    assert abs(x - W / 2) < 0.3 and abs(y - H / 2) < 0.3
    # Tabelle: alle Quellen im Bild, und sie stimmt mit lens_forward überein
    m1, m2 = C.lens_maps(cam)
    fx, fy = cv2.convertMaps(m1, m2, cv2.CV_32FC1)
    assert fx.min() >= 0 and fx.max() <= W - 1 and fy.min() >= 0 and fy.max() <= H - 1
    gx, gy = np.meshgrid(np.arange(40.0, W - 60, 37), np.arange(40.0, H - 40, 31))
    v = np.stack([gx, gy], -1)
    zurueck = C.lens_forward(cam, np.stack([fx[gy.astype(int), gx.astype(int)], fy[gy.astype(int), gx.astype(int)]], -1))
    assert np.abs(zurueck - v).max() < 0.05
    assert C.lens_id(cam) != C.lens_id(nur_roll) != C.lens_id(dataclasses.replace(cam, lens_roll_deg=0.0))


def test_linse_kern_gleich_cv2():
    """native/linsefast.cpp rechnet bitgleich zu cv2.remap, auch mit Quellpixeln außerhalb des Bilds
    (ohne Kern übersprungen). camera.lens_remap wählt den Kern nur nach derselben Prüfung."""
    import ctypes
    import cv2
    from xgonav import fastlib
    from xgonav.camera import _LENS_ABI, lens_maps, lens_remap
    lib = fastlib.load()
    if lib is None or not hasattr(lib, "lens_abi"):
        pytest.skip("C++-Kern ohne linsefast")
    assert fastlib.check_abi(lib, "lens_abi", _LENS_ABI)
    lib.lens_remap.restype = ctypes.c_int
    lib.lens_remap.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                               ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    cam = Config().camera
    rng = np.random.default_rng(5)
    img = rng.integers(0, 256, (cam.height, cam.width, 3), dtype=np.uint8)
    gx, gy = np.meshgrid(np.arange(cam.width, dtype=np.float32), np.arange(cam.height, dtype=np.float32))
    ueber = cv2.convertMaps(gx * 1.02 - 5.3, gy * 1.02 - 4.7, cv2.CV_16SC2)        # läuft über alle vier Ränder
    for m1, m2 in (lens_maps(cam), ueber):
        m1, m2 = np.ascontiguousarray(m1), np.ascontiguousarray(m2)
        out = np.empty_like(img)
        assert lib.lens_remap(img.ctypes.data, cam.width, cam.height, img.strides[0], 3, m1.ctypes.data, m2.ctypes.data,
                              out.ctypes.data, cam.width, cam.height, out.strides[0]) == 0
        assert np.array_equal(out, cv2.remap(img, m1, m2, cv2.INTER_LINEAR))
    assert np.array_equal(lens_remap(cam)(img), cv2.remap(img, *lens_maps(cam), cv2.INTER_LINEAR))


def test_kamera_direkt_schnittstelle(monkeypatch):
    """libcamera direkt (native/camfast.cpp): CamMeta-Layout wie in C (64 Bytes, dropped bei 56); die
    Selbstprüfung des Entzerrers lehnt einen abweichenden Kern ab; ohne libxgocam.so kein Handle (-> picamera2)."""
    import ctypes
    import cv2
    from xgonav import camera, fastlib
    assert ctypes.sizeof(camera._CamMeta) == 64 and camera._CamMeta.dropped.offset == 56
    assert camera._CamMeta.crop.offset == 36
    cam = Config().camera
    m1, m2 = (np.ascontiguousarray(m) for m in camera.lens_maps(cam))

    def mit_cv(img):
        return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
    assert camera._lens_kern_ok(cam, mit_cv, mit_cv)
    assert not camera._lens_kern_ok(cam, lambda img: mit_cv(img) ^ 1, mit_cv)
    monkeypatch.setattr(fastlib, "CAM_PATH", "/gibt/es/nicht/libxgocam.so")
    assert camera._cam_lib() is None


def test_floornet_wechseln(tmp_path, monkeypatch):
    """Bodenmodell aus dem Web wechseln: erst laden und prüfen, dann Symlink umstellen und Netz im Worker tauschen;
    scheitert das Laden, bleibt alles; nur in idle/manual; Namen nur aus dem Ordner."""
    import os
    import xgonav.app as appmod
    from xgonav.app import ModeNotAllowed
    d = tmp_path / "floornet"
    d.mkdir()
    (d / "a.bin").write_bytes(b"a")
    (d / "b.bin").write_bytes(b"bb")
    link = tmp_path / "floornet.bin"
    try:
        os.symlink("floornet/a.bin", link)
    except (OSError, NotImplementedError):
        pytest.skip("Symlinks hier nicht erlaubt (Windows ohne Entwicklermodus)")
    app = _app(freespace=True)
    app.cfg.freespace.net, app.cfg.freespace.net_dir = str(link), str(d)
    info = app.floornet_info()
    assert [m["name"] for m in info["modelle"]] == ["a.bin", "b.bin"] and info["aktiv"] == "a.bin" and info["ist_link"]

    class Netz:
        zu = False
        def run(self, bgr): return np.zeros(80, np.float32)
        def close(self): self.zu = True
    alt = app.fs_worker.net
    monkeypatch.setattr(appmod.FloorNet, "load", staticmethod(lambda p, *a: None))
    with pytest.raises(ModeNotAllowed):
        app.floornet_wechseln("b.bin")                     # nicht ladbar: alles bleibt
    assert os.readlink(link) == "floornet/a.bin" and app.fs_worker.net is alt
    neu = Netz()
    monkeypatch.setattr(appmod.FloorNet, "load", staticmethod(lambda p, *a: neu))
    for falsch in ("../x.bin", "c.bin", "b.txt"):
        with pytest.raises(ValueError):
            app.floornet_wechseln(falsch)
    assert app.floornet_wechseln("b.bin") == "b.bin"
    assert os.readlink(link) == os.path.join("floornet", "b.bin") and app.fs_worker.net is neu
    assert not app.fs_worker.blind and app.floornet_info()["aktiv"] == "b.bin" and app.fn_geladen == "b.bin"
    app.behavior.name = "explore"
    with pytest.raises(ModeNotAllowed):
        app.floornet_wechseln("a.bin")                     # fährt nach dem Bodenmodell: kein Wechsel


def test_jpeg_anzeige_rueckfall(monkeypatch):
    """Anzeige-JPEG (jpeg.py): ohne Hardware-Kodierer kodiert cv2; das Ergebnis ist ein lesbares JPEG in voller
    Größe. Mit native=False wird die Bibliothek gar nicht erst geladen."""
    import cv2
    from xgonav import fastlib
    from xgonav.jpeg import Jpeg
    monkeypatch.setattr(fastlib, "PATH", "/gibt/es/nicht/libxgofast.so")
    fastlib._cache.clear()
    try:
        img = np.random.default_rng(2).integers(0, 256, (480, 640, 3), dtype=np.uint8)
        for j in (Jpeg(True), Jpeg(False)):
            b = j.encode(img, 70)
            assert b is not None and not j.native
            assert cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR).shape == img.shape
    finally:
        fastlib._cache.clear()


def test_robot_tempo_alle_befehle():
    """Temporegler: jeder Fahrbefehl wird nach der Begrenzung auf max_v/max_w mit tempo malgenommen (alle Modi,
    auch Drehen); außerhalb 0,05..1 abgelehnt, Wert bleibt."""
    cfg = Config.neutral()
    r = SimRobot(cfg.robot)
    assert r.tempo == 1.0
    r.drive(1.0, -1.0)
    assert r._target == (cfg.robot.max_v, -cfg.robot.max_w)
    r.tempo = 0.4
    r.drive(1.0, -1.0)
    assert r._target == pytest.approx((cfg.robot.max_v * 0.4, -cfg.robot.max_w * 0.4))
    r.drive(0.1, 0.05)
    assert r._target == pytest.approx((0.04, 0.02))
    for bad in (0.0, 1.5, -0.2):
        with pytest.raises(ValueError):
            r.tempo = bad
    assert r.tempo == 0.4


def test_kamera_kontrast_saettigung():
    """Kontrast/Sättigung/Helligkeit an der Quelle (ISP): Bereichsprüfung, Werte in cfg (Status/Meta lesen sie daraus)
    und an das Gerät weitergereicht; ungültig = nichts geändert, Helligkeit None = unverändert."""
    from xgonav.camera import FrameSource
    gesetzt = []

    class Quelle(FrameSource):
        def _apply_bild(self, k, s, b):
            gesetzt.append((k, s, b))

    q = Quelle(Config.neutral().camera)
    assert q.bild == (1.0, 1.0, 0.0)
    assert q.set_bild(2.0, 1.5, 0.2) == (2.0, 1.5, 0.2) and q.bild == (2.0, 1.5, 0.2) and gesetzt == [(2.0, 1.5, 0.2)]
    assert q.set_bild(2.0, 1.5) == (2.0, 1.5, 0.2)
    for k, s, b in ((0.2, 1.0, 0.0), (1.0, 5.0, 0.0), (1.0, 1.0, 0.8)):
        with pytest.raises(ValueError):
            q.set_bild(k, s, b)
    assert q.bild == (2.0, 1.5, 0.2) and len(gesetzt) == 2


def test_floornet_nur_balancierend():
    """FloorNet läuft nur balancierend: liegend keine Bodengrenze und kein Netzaufruf, nach dem Aufstehen wieder."""
    import threading
    from xgonav.app import App
    cfg = Config.neutral(); cfg.output.web_port = 0; cfg.detector.enabled = False
    cfg.freespace.restart_hold_s = 0.2
    robot = SimRobot(cfg.robot)
    world = SimWorld(cfg.camera, robot, None, start=(1.0, 1.5, 0.0))
    net = SimFloorNet(world)
    aufrufe = []
    lauf = net.run
    net.run = lambda bgr: (aufrufe.append(1), lauf(bgr))[1]
    app = App(cfg, robot, world, imu=None, floor_net=net)
    app.imu = None
    th = threading.Thread(target=app.run, kwargs={"duration": 6.0}, daemon=True)
    th.start()
    try:
        time.sleep(1.0)
        assert app._free is not None and aufrufe
        robot.state.balancing = False                   # abgesetzt
        time.sleep(0.5)
        n = len(aufrufe)
        time.sleep(1.0)
        assert app._free is None and len(aufrufe) <= n + 1      # höchstens ein Auftrag war schon unterwegs
        robot.state.balancing = True                    # aufgestanden
        time.sleep(1.2)
        assert app._free is not None and len(aufrufe) > n + 1
    finally:
        app.stop(); th.join(timeout=5)


def test_sichtfeld_lage_korrektur_bringt_horizont_auf_nick_null():
    """tools/floornet/sichtfeld.py: Nick und Roll eines Bildes herausrechnen; der Horizont landet bei Nick 0."""
    import math
    import cv2
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "floornet"))
    import sichtfeld as SF
    from xgonav.geometry import CamGeometry
    geo = CamGeometry.from_cfg(Config())
    nick, roll = math.radians(-3.0), 2.0
    img = np.zeros((480, 640), np.uint8)
    hz = SF.horizont_zeile(geo, nick)
    # Horizont bei Roll + (rechts tief) erscheint rechts höher
    for x in range(640):
        y = hz - (x - 320) * math.tan(math.radians(roll))
        img[int(round(y)):, x] = 255
    out = SF.lage_korrigieren(img, geo, nick, roll)
    soll = SF.horizont_zeile(geo, 0.0)
    for x in (100, 320, 540):
        zeile = int(np.argmax(out[:, x] > 127))
        assert abs(zeile - soll) <= 1.5, (x, zeile, soll)


def test_fw_zustand_liegt_ist_kein_sturz():
    """Absetzen (254, LEDs orange) und Kippen (0, LEDs rot) sind getrennte Zustände (robot.FW_ZUSTAND)."""
    from xgonav.behaviors import Command, World, safety
    from xgonav.robot import lage, stopp_text
    assert lage(1) == "balanciert" and lage(254) == "liegt" and lage(0) == "gefallen"
    assert lage(None) is None and lage(77) == "zustand 77"
    assert stopp_text(254) == "LIEGT – Stopp" and stopp_text(0) == "GEFALLEN – Stopp"
    assert stopp_text(None) == "GEFALLEN – Stopp"
    cfg = Config.neutral()

    def welt(mode):
        return World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[], free=None, balancing=False,
                     yaw=0.0, cmd_v=0.0, fw_mode=mode)
    for mode, text in ((254, "LIEGT – Stopp"), (0, "GEFALLEN – Stopp")):
        c = safety(Command(0.3, 0.2, None, "fährt"), welt(mode), cfg.behavior)
        assert (c.v, c.w, c.status) == (0, 0, text)
    r = SimRobot(cfg.robot)
    r._write("action", 254)
    assert r.state.lage == "liegt" and not r.state.balancing


def test_manual_turn_dreht_um_winkel_mit_nachlauf():
    """POST /turn (Manual.turn): dreht mit dreh_auf_kurs bis tol, zählt turn_settle_s Nachlauf mit; ein Sollwert bricht ab."""
    from xgonav.behaviors import Manual
    cfg = Config.neutral()
    m = Manual(cfg.behavior, cfg.robot)
    yaw, t, dt = 3.0, 0.0, 0.05                                     # über ±pi hinweg
    m.turn(90.0, 0.5, 3.0, t, yaw, 20.0)
    ws = []
    while not m.turn_info["fertig"] and t < 30.0:
        c = m.step(None, yaw=yaw, now=t)
        ws.append(c.w)
        yaw = (yaw + math.radians(c.w * cfg.robot.vyaw_limit_dps) * dt + math.pi) % (2 * math.pi) - math.pi
        t += dt
    i = m.turn_info
    assert i["grund"] == "angekommen" and abs(i["gedreht_deg"] - 90.0) <= 3.0 and m._turn is None
    assert max(ws) <= cfg.robot.w_of(0.5) + 1e-9 and min(w for w in ws if w) >= cfg.robot.turn_min_w
    m.turn(-45.0, 0.5, 3.0, t, yaw, 20.0)
    m.step(None, yaw=yaw, now=t)
    m.set(0.0, 0.0, t)
    assert m.turn_info["fertig"] and m.turn_info["grund"] == "abgebrochen"


def test_mathe_bausteine_wie_frueher():
    """xgonav/mathe.py rechnet bitgleich wie die früheren Stellen (Wrap, Posen, Bogen)."""
    import numpy as np
    from xgonav import mathe
    for a in (-7.0, -math.pi, 0.0, 3.2, 12.5):
        assert mathe.wrap(a) == (a + math.pi) % (2 * math.pi) - math.pi
    arr = np.linspace(-10, 10, 7)
    assert np.array_equal(mathe.wrap(arr), (arr + np.pi) % (2 * np.pi) - np.pi)
    k, p = (0.3, -0.2, 0.4), (1.1, 0.7, -2.0)
    c, s = math.cos(k[2]), math.sin(k[2])
    assert mathe.pose_compose(k, p) == (k[0] + c * p[0] - s * p[1], k[1] + s * p[0] + c * p[1], p[2] + k[2])
    x, y, th = 0.0, 0.0, 0.0
    xa, ya, tha = 0.0, 0.0, 0.0
    for _ in range(10):
        th += 0.7 * 0.12
        x += 0.3 * math.cos(th) * 0.12
        y += 0.3 * math.sin(th) * 0.12
        xa, ya, tha = mathe.arc_step(xa, ya, tha, 0.3, 0.7, 0.12)
    assert (xa, ya, tha) == (x, y, th)
    assert mathe.wrap32(5 - (2 ** 32 - 3)) == 8


def test_pathfollower_bremst_frueh_vor_dem_wegende():
    """Seit 2026-11-23: Bremsweg-Profil auf den Restweg entlang des Wegs, um die Trägheit vorausgerechnet; schnell
    beginnt er früher, weit weg bremst er nicht, kurz vor dem Ende bleibt end_min_v."""
    from types import SimpleNamespace as NS
    from xgonav.behaviors import PathFollower
    cfg = Config.neutral()
    rc, h = cfg.robot, cfg.home
    f = PathFollower(cfg.behavior, rc, h, v=cfg.explore.v)
    f.set_path([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])       # 3 m Weg, Luftlinie zum Ende 1 m
    assert f.rest_m(0.0, 0.0) == pytest.approx(3.0) and f.rest_m(1.0, 0.5) == pytest.approx(1.5)
    w = lambda v_rad: NS(v_rad=v_rad, cmd_v=0.0)
    langsam, schnell = f.end_v(w(0.1), 0.5, 1.0), f.end_v(w(0.7), 0.5, 1.0)  # 0,5 m vor dem Ende
    assert schnell < langsam                                             # schneller: früher bremsen
    assert f.end_v(w(0.0), 0.0, 0.0) > rc.v_of(cfg.explore.v)            # 3 m vorher und steht: keine Grenze
    assert f.end_v(w(0.7), 0.05, 1.0) == pytest.approx(rc.v_of(h.end_min_v))
    # bei voller explore-Fahrt (0,4 absolut = 0,72 m/s) beginnt das Bremsen ~1,8 m vor dem Ende (vorher 0,9 m)
    v_mps = rc.v_of(cfg.explore.v) * rc.vx_limit
    assert f.end_v(w(v_mps), 1.0, 0.4) * rc.vx_limit < v_mps <= f.end_v(w(v_mps), 1.0, 0.15) * rc.vx_limit
