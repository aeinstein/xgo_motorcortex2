"""follow: Ziel verloren -> zur letzten bekannten Stelle fahren (Planer), dort umschauen, dann warten."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xgonav.behaviors import Follow, World, make_behavior  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.tracker import Track  # noqa: E402


class FakePlanner:
    def __init__(self):
        self.req = []

    def request(self, start, goal, keep=None, obstacles=None, **kw):
        self.req.append((tuple(start), tuple(goal), kw))
        return len(self.req)


def _cfg():
    return Config.neutral()


def _welt(t, pose, tracks=(), pose_ok=True, geo=None):
    return World(t=t, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=list(tracks), free=None, balancing=True,
                 yaw=pose[2] if pose else 0.0, cmd_v=0.0, geo=geo, pose=pose, pose_ok=pose_ok)


def _follow(seek=True):
    cfg = _cfg()
    return Follow(cfg.behavior, cfg.robot, "person", hcfg=cfg.home, seek=seek), cfg


def _sehen(f, t, pose, dist=2.0, cx=320.0):
    """Ziel in Bildmitte bei bekanntem Abstand: Servo-Abstand direkt setzen (ohne Kamerageometrie)."""
    tr = Track("person", cx, 240, 80, 200, 0.9, t)
    f.target = tr.id                       # sonst setzt pick() den Servo zurück (neues Ziel)
    f.servo._dist = dist
    f.servo._P = [[0.01, 0], [0, 0.01]]
    f.servo._t_meas = t
    return f.step(_welt(t, pose, [tr])), tr


def test_letzte_stelle_aus_pose_abstand_peilung():
    f, _ = _follow()
    _sehen(f, 0.0, (1.0, 0.5, math.pi / 2), dist=2.0)
    assert f.last_pos is not None
    assert math.isclose(f.last_pos[0], 1.0, abs_tol=0.05) and math.isclose(f.last_pos[1], 2.5, abs_tol=0.05)


def test_verloren_plant_zur_letzten_stelle_und_schaut_um():
    f, cfg = _follow()
    pose = (0.0, 0.0, 0.0)
    _sehen(f, 0.0, pose, dist=2.0)
    t = cfg.behavior.lost_search_s + 0.5
    c = f.step(_welt(t, pose))
    assert f.phase == "fahren" and "plane Weg" in c.status
    assert f.want_plan(t)
    pl = FakePlanner()
    f.request_plan(pl, None, _welt(t, pose), None)
    start, goal, kw = pl.req[0]
    assert math.isclose(goal[0], 2.0, abs_tol=0.05) and abs(goal[1]) < 0.05 and kw.get("clear_goal")
    f.set_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    c = f.step(_welt(t + 0.05, pose))
    assert c.v > 0 and "letzten Stelle" in c.status
    # am Ziel: umschauen, zur Seite der letzten Sichtung
    th, tt = 0.0, t + 5.0
    c = f.step(_welt(tt, (1.9, 0.0, th)))
    assert f.phase == "umschauen" and c.v == 0 and c.w != 0
    for _ in range(200):
        tt += 0.1
        th += math.radians(10)
        c = f.step(_welt(tt, (1.9, 0.0, th)))
        if f.phase == "warten":
            break
    assert f.phase == "warten" and c.v == 0 and c.w == 0 and "warte" in c.status


def test_wiedergefunden_beendet_suche():
    f, cfg = _follow()
    pose = (0.0, 0.0, 0.0)
    _sehen(f, 0.0, pose)
    t = cfg.behavior.lost_search_s + 0.5
    f.step(_welt(t, pose))
    assert f.phase == "fahren"
    f.set_path([(0.0, 0.0), (2.0, 0.0)])
    c, _ = _sehen(f, t + 1.0, pose)
    assert f.phase == "" and f.path is None and c.target_id is not None


def test_kein_weg_schaut_vor_ort_um():
    f, cfg = _follow()
    pose = (0.0, 0.0, 0.0)
    _sehen(f, 0.0, pose)
    t = cfg.behavior.lost_search_s + 0.5
    f.step(_welt(t, pose))
    for k in range(Follow.PLAN_FAILS):
        f.pending, f._req = True, f._goal
        f.set_path(None)
    c = f.step(_welt(t + 0.1, pose))
    assert f.phase == "umschauen" and c.w != 0


def test_ohne_pose_oder_seek_alter_suchweg():
    for seek, pose_ok in ((False, True), (True, False)):
        f, cfg = _follow(seek)
        pose = (0.0, 0.0, 0.0)
        tr = Track("person", 600, 240, 80, 200, 0.9, 0.0)          # rechts im Bild
        f.target = tr.id
        f.servo._dist, f.servo._P = 2.0, [[0.01, 0], [0, 0.01]]
        f.step(_welt(0.0, pose, [tr], pose_ok=pose_ok))
        c = f.step(_welt(cfg.behavior.lost_search_s + 0.5, pose, pose_ok=pose_ok))
        assert f.phase == "" and not f.want_plan(5.0)
        assert c.v == 0 and c.w < 0 and "rechts" in c.status


def test_make_behavior_seek_nur_mit_karte():
    cfg = _cfg()
    assert make_behavior("follow", cfg.behavior, cfg.robot, "person", cfg.home, has_map=True).seek
    assert not make_behavior("follow", cfg.behavior, cfg.robot, "person", cfg.home, has_map=False).seek
    assert not make_behavior("approach", cfg.behavior, cfg.robot, "chair", cfg.home).follow.seek


def _app():
    from xgonav.app import App
    from xgonav.robot import SimRobot
    from xgonav.sim import SimWorld
    cfg = Config.neutral()
    cfg.output.web_port = 0
    cfg.odom.source = "command"
    robot = SimRobot(cfg.robot)
    return App(cfg, robot, SimWorld(cfg.camera, robot)), robot


def test_startpunkt_nur_beim_aufspringen_und_absetzen():
    """Nutzer 2026-11-22: Start = Pose beim Aufspringen (sobald still) oder Absetzen, nie beim Moduswechsel."""
    app, robot = _app()
    gm = app.gridmap
    robot.state.balancing = True
    app._start_tick(0.0)                                       # einlesen
    gm.set_pose((1.0, 2.0, 0.3))
    app.set_mode("pet")                                        # Moduswechsel: kein neuer Start
    app._start_tick(0.1)
    assert gm.start == (0.0, 0.0, 0.0)
    robot.state.balancing = False                              # absetzen: sofort
    app._start_tick(0.2)
    assert gm.start == (1.0, 2.0, 0.3)
    robot.state.balancing = True                               # aufspringen: erst wenn er still steht, zählt die Pose
    app._start_tick(0.3)
    for i, x in enumerate((1.05, 1.10, 1.15, 1.20)):
        gm.set_pose((x, 2.0, 0.3))
        app._start_tick(0.4 + 0.1 * i)
    assert gm.start == (1.0, 2.0, 0.3)
    t = 0.8
    while t < 2.2:
        app._start_tick(t)
        t += 0.1
    assert gm.start == (1.20, 2.0, 0.3)
    gm.lock()                                                  # gesperrte Karte: Start bleibt
    robot.state.balancing = False
    gm.set_pose((3.0, 3.0, 0.0))
    app._start_tick(t)
    assert gm.start == (1.20, 2.0, 0.3)


def test_startpunkt_nicht_nach_pause():
    app, robot = _app()
    gm = app.gridmap
    robot.state.balancing = False
    app._start_tick(0.0)
    robot.state.balancing = True                               # während einer Pause aufgestanden
    gm.set_pose((1.0, 1.0, 0.0))
    app._start_tick(5.0)
    for k in range(30):
        app._start_tick(5.1 + 0.1 * k)
    assert gm.start == (0.0, 0.0, 0.0)


def test_startpunkt_erst_nach_roll_kalibrierung():
    app, robot = _app()
    gm = app.gridmap
    robot.state.balancing = False
    app._start_tick(0.0)
    robot.state.balancing = True
    robot.cfg.level_after_standup = True                       # Simulator: sonst aus
    robot._seen_down, robot._standup_levelled = True, False    # Nivellier-Impuls steht noch an
    gm.set_pose((0.5, 0.0, 0.0))
    t = 0.1
    while t < 5.0:
        app._start_tick(t)
        t += 0.1
    assert gm.start == (0.0, 0.0, 0.0) and not robot.levelled
    robot._standup_levelled = True                              # Impuls vorbei
    while t < 7.0:
        app._start_tick(t)
        t += 0.1
    assert gm.start == (0.5, 0.0, 0.0)
