"""Closed-Loop-Szenarien im Simulator. python3 tests/sim_run.py follow|explore [sekunden] [video.mp4]"""
import logging, math, sys, threading, time
sys.path.insert(0, ".")
import numpy as np
from xgonav.app import App, render
from xgonav.config import Config
from xgonav.robot import SimRobot
from xgonav.sim import FakeDetector, Person, SimImu, SimWorld


def run(mode="follow", secs=25.0, video=None, shake=1.0, quiet=True, imu=False):
    logging.basicConfig(level=logging.WARNING if quiet else logging.INFO)
    cfg = Config()
    cfg.behavior.mode = mode
    cfg.output.web_port = 0
    robot = SimRobot(cfg.robot)
    person = Person([(3.2, 1.3), (4.8, 4.6), (1.4, 5.0), (1.2, 3.0)], speed=0.3) if mode != "explore" else None
    world = SimWorld(cfg.camera, robot, person, start=(1.0, 1.2, 0.2), shake=shake,
                     mount_offset_mm=cfg.camera.mount_offset_mm, vx_mps=cfg.robot.vx_limit)
    if imu:
        cfg.imu.pitch_ref_deg = 2.5   # wie nach calibrate.py wall
    app = App(cfg, robot, world, imu=SimImu(world, cfg.imu) if imu else None)
    if not imu:
        app.imu = None
    app.det = FakeDetector(world)
    if video:
        import cv2
        def rec():
            vw, last = None, -1
            while not app.running: time.sleep(0.02)
            while app.running:
                with app.snap_cond:
                    app.snap_cond.wait_for(lambda: app.snapshot.idx != last, timeout=1)
                    s = app.snapshot
                if s.idx == last: continue
                last = s.idx
                img = render(s)
                if img is None: continue
                if vw is None: vw = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 20, (img.shape[1], img.shape[0]))
                vw.write(img)
            vw and vw.release()
        th = threading.Thread(target=rec, daemon=True); th.start()
    app.run(duration=secs)
    if video:
        with app.snap_cond: app.snap_cond.notify_all()
        th.join(timeout=3)
    L = world.log
    out = {"imu_sync": (app.imu_sync.reliable, app.imu_sync.sign, round(app.imu_sync.offset * 1000), round(app.imu_sync.corr, 2)) if imu else None,
           "collisions": world.collisions, "coll_frames": world.coll_frames, "dist_m": round(world.dist_travelled, 2), "frames": len(L),
           "loop_ms": round(app._loop_ms, 1)}
    if person:
        d = np.array([math.hypot(r["px"] - r["x"], r["py"] - r["y"]) for r in L])
        b = np.array([math.degrees((math.atan2(r["py"] - r["y"], r["px"] - r["x"]) - r["th"] + math.pi) % (2 * math.pi) - math.pi) for r in L])
        half = len(L) // 3
        out.update(dist_mean=round(float(d[half:].mean()), 2), dist_min=round(float(d.min()), 2),
                   dist_max_late=round(float(d[half:].max()), 2),
                   bearing_abs_mean=round(float(np.abs(b[half:]).mean()), 1),
                   bearing_abs_p95=round(float(np.percentile(np.abs(b[half:]), 95)), 1))
    return out


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "follow"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else 25
    args = [a for a in sys.argv[3:] if not a.startswith("--")]
    print(run(mode, secs, args[0] if args else None, imu="--imu" in sys.argv))
