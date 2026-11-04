"""Closed-Loop-Szenarien im Simulator. python3 tests/sim_run.py follow|explore|home [sekunden] [video.mp4] [--imu] [--fs-thread] [--wheels] [--kanten]

home: erste Hälfte explore, dann Modus home (zurück zum Startpunkt der Raumkarte)."""
import logging, math, sys, threading, time
sys.path.insert(0, ".")
import numpy as np
from xgonav.app import App, render
from xgonav.config import Config
from xgonav.robot import SimRobot
from xgonav.sim import FakeDetector, Person, SimImu, SimWheels, SimWorld


def run(mode="follow", secs=25.0, video=None, shake=1.0, quiet=True, imu=False, fs_thread=False, wheels=False, vx_true=None, method=None):
    logging.basicConfig(level=logging.WARNING if quiet else logging.INFO)
    cfg = Config()
    home = mode == "home"
    cfg.behavior.mode = "explore" if home else mode
    cfg.output.web_port = 0
    cfg.freespace.threaded = fs_thread
    if method:
        cfg.freespace.method = method
    robot = SimRobot(cfg.robot)
    person = Person([(3.2, 1.3), (4.8, 4.6), (1.4, 5.0), (1.2, 3.0)], speed=0.3) if mode not in ("explore", "home") else None
    world = SimWorld(cfg.camera, robot, person, start=(1.0, 1.2, 0.2), shake=shake,
                     mount_offset_mm=cfg.camera.mount_offset_mm,
                     vx_mps=vx_true if vx_true is not None else cfg.robot.vx_limit)  # vx_true: echte m/s je Einheit
    if imu:
        cfg.imu.pitch_ref_deg = 2.5   # wie nach calibrate.py wall
    if not wheels:
        cfg.odom.source = "command"          # sonst sucht App die Raddaten in /proc
    app = App(cfg, robot, world, imu=SimImu(world, cfg.imu) if imu else None,
              wheels=SimWheels(world, cfg.odom) if wheels else None)
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
    switch = {}
    if home:
        def to_home():
            t_sw = time.monotonic() + secs / 2
            while not app.running: time.sleep(0.02)
            while app.running and time.monotonic() < t_sw: time.sleep(0.05)
            switch["frame"] = len(world.log)
            app.set_mode("home")
        threading.Thread(target=to_home, daemon=True).start()
    app.run(duration=secs)
    if video:
        with app.snap_cond: app.snap_cond.notify_all()
        th.join(timeout=3)
    L = world.log
    gm = getattr(app, "gridmap", None)
    grid = None
    if gm is not None:
        # Karten-KS = Startpose -> Welt; Abstand jeder belegten Zelle zur nächsten Wand/Kiste
        x0, y0, th0 = L[0]["x"], L[0]["y"], L[0]["th"]
        c0, s0 = math.cos(th0), math.sin(th0)
        occ = gm.occupied_cells()
        if len(occ):
            wx = x0 + occ[:, 0] * c0 - occ[:, 1] * s0
            wy = y0 + occ[:, 0] * s0 + occ[:, 1] * c0
            seg = world.segs
            ax, ay, bx, by = seg[:, 0], seg[:, 1], seg[:, 2], seg[:, 3]
            dx, dy = bx - ax, by - ay
            t = np.clip(((wx[:, None] - ax) * dx + (wy[:, None] - ay) * dy) / (dx * dx + dy * dy), 0, 1)
            d = np.hypot(ax + t * dx - wx[:, None], ay + t * dy - wy[:, None]).min(axis=1)
            prec = float((d < 0.15).mean())
        else:
            prec = float("nan")
        ex = x0 + gm.x * c0 - gm.y * s0
        ey = y0 + gm.x * s0 + gm.y * c0
        # Abdeckung: Punkte auf Wänden/Kisten (alle 5 cm) in <= 2 m vom gefahrenen Weg,
        # davon Anteil mit belegter Zelle in <= 10 cm
        pts = []
        for ax_, ay_, bx_, by_, _ in world.segs:
            n_ = max(2, int(math.hypot(bx_ - ax_, by_ - ay_) / 0.05))
            pts += [(ax_ + (bx_ - ax_) * k / n_, ay_ + (by_ - ay_) * k / n_) for k in range(n_ + 1)]
        pts = np.array(pts)
        path = np.array([(r["x"], r["y"]) for r in L[::5]])
        near = np.hypot(pts[:, None, 0] - path[None, :, 0], pts[:, None, 1] - path[None, :, 1]).min(axis=1) <= 2.0
        pts = pts[near]
        if len(pts) and len(occ):
            dd = np.hypot(pts[:, None, 0] - wx[None, :], pts[:, None, 1] - wy[None, :]).min(axis=1)
            recall = float((dd <= 0.10).mean())
        else:
            recall = float("nan")
        grid = {"belegt_zellen": int(len(occ)), "praezision_15cm": round(prec, 3), "abdeckung_10cm": round(recall, 3),
                "pose_fehler_m": round(math.hypot(ex - world.x, ey - world.y), 3),
                "kurs_fehler_deg": round(math.degrees((th0 + gm.th - world.th + math.pi) % (2 * math.pi) - math.pi), 1)}
    out = {"imu_sync": (app.imu_sync.reliable, app.imu_sync.sign, round(app.imu_sync.offset * 1000), round(app.imu_sync.corr, 2)) if imu else None,
           "collisions": world.collisions, "coll_frames": world.coll_frames, "dist_m": round(world.dist_travelled, 2), "frames": len(L),
           "loop_ms": round(app._loop_ms, 1), "odom": app.odom_src, "odom_m": round(app.odom_m, 2),
           "grid": grid}
    if home:
        x0, y0 = L[0]["x"], L[0]["y"]
        k = min(switch.get("frame", len(L)) , len(L)) - 1
        out.update(home_weg_m=round(math.hypot(L[k]["x"] - x0, L[k]["y"] - y0), 2),
                   home_end_m=round(math.hypot(world.x - x0, world.y - y0), 2),
                   home_status=app._cmd.status, plan_ms=round(app.planner.ms, 1))
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
    print(run(mode, secs, args[0] if args else None, imu="--imu" in sys.argv, fs_thread="--fs-thread" in sys.argv,
              wheels="--wheels" in sys.argv, method="kanten" if "--kanten" in sys.argv else None))
