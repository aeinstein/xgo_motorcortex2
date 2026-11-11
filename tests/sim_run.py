"""Closed-Loop-Szenarien im Simulator. python3 tests/sim_run.py follow|explore|home [sekunden] [video.mp4] [--imu] [--fs-thread] [--wheels] [--kanten] [--dyn] [--rider] [--scene raum|zwei] [--set a.b=wert]

home: erste Hälfte explore, dann Modus home (zurück zum Startpunkt der Raumkarte).
--dyn: Trägheit wie am RiderPi (Totzone, Nachlaufen der Drehung, Rückrollen beim Anfahren).
--rider: Einstellungen des RiderPi statt der Defaults (RIDER), Tempo wie gemessen."""
import logging, math, sys, threading, time
sys.path.insert(0, ".")
import numpy as np
from xgonav.app import App, render
from xgonav.config import Config
from xgonav.robot import SimRobot
from xgonav.sim import FakeDetector, Person, Phantoms, SCENES, SimImu, SimWheels, SimWorld, map_quality

# Fahrrelevante Abweichungen in deploy/config.example.toml; echtes Tempo 1,8 m/s je Firmware-Einheit
# (gemessen 2026-11-05: ~27 cm/s bei 0,15)
RIDER = {"robot.vx_limit": 1.8, "robot.max_v": 0.15, "robot.max_w": 0.30, "robot.reverse_gain": 2.0, "behavior.turn_w": 0.21,
         "behavior.search_w": 0.07, "behavior.search_timeout_s": 10.0, "behavior.height_fast_mm": 80.0,
         "behavior.height_search_mm": 100.0, "map.confirm": 3}
RIDER_VX = 1.8


def apply_overrides(cfg, overrides) -> None:
    """{"grid.l_occ": 0.6, ...} in die verschachtelte Konfiguration; Tippfehler fallen auf."""
    for key, val in (overrides or {}).items():
        sec, name = key.split(".")
        part = getattr(cfg, sec)
        if not hasattr(part, name):
            raise KeyError(key)
        setattr(part, name, val)


def pick_route(gm, cfg, n=3):
    """n weit auseinanderliegende, erreichbare Punkte auf der Karte (Ersatz für die Klicks im Web)."""
    from xgonav.route import check
    rr, cc = np.nonzero(gm.lo < -gm.cfg.l_show)
    cand = [((c + 0.5 - gm.n / 2) * gm.res, (r + 0.5 - gm.n / 2) * gm.res) for r, c in zip(rr[::7], cc[::7])]
    cand = [p for p in cand if not check(gm, cfg.home, [p], [])]
    pts = []
    ref = [(gm.start[0], gm.start[1])]
    for _ in range(n):
        p = max(cand, key=lambda q: min(math.hypot(q[0] - a, q[1] - b) for a, b in ref + pts))
        pts.append(p)
    return pts


def run(mode="follow", secs=25.0, video=None, shake=1.0, quiet=True, imu=False, fs_thread=False, wheels=False, vx_true=None, method=None, dyn=False,
        overrides=None, scene="raum", record=None, phantoms=0, animal="person", speed=None):
    """overrides: siehe apply_overrides. record: Liste, bekommt je Eintrag in die Raumkarte die Beobachtung
    (tools/map_eval.py spielt sie offline ab); dafür trägt die Karte im Regelschritt ein.
    animal, speed (m/s): nur Modus pet, wer auf den Rider zukommt (person, cat, dog)."""
    logging.basicConfig(level=logging.WARNING if quiet else logging.INFO)
    cfg = Config()
    home = mode == "home"
    patrol = mode == "patrol"                # erst kartieren, dann Streife
    cfg.behavior.mode = "explore" if home or patrol else mode
    cfg.output.web_port = 0
    cfg.freespace.threaded = fs_thread
    if method:
        cfg.freespace.method = method
    apply_overrides(cfg, overrides)
    if record is not None:
        cfg.grid.threaded = False
    robot = SimRobot(cfg.robot)
    person = Person([(3.2, 1.3), (4.8, 4.6), (1.4, 5.0), (1.2, 3.0)], speed=0.3) if mode not in ("explore", "home", "patrol") else None
    if mode == "pet":
        # kommt geradewegs auf den Rider zu (Start bei x=1, y=1,2, Blick 0,2 rad), wartet 0,3 m davor kurz und
        # geht seitlich aus dem Bild: Danach ist die Luft rein
        h, b = {"person": (1.72, 0.50), "dog": (0.5, 0.5), "cat": (0.3, 0.35)}[animal]
        person = Person([(4.2, 1.85), (1.3, 1.26), (1.4, 3.6)], speed=speed or 0.3, cls=animal, height_m=h, width_m=b, loop=False)
    world = SimWorld(cfg.camera, robot, person, start=(1.0, 1.2, 0.2), shake=shake,
                     mount_offset_mm=cfg.camera.mount_offset_mm,
                     vx_mps=vx_true if vx_true is not None else cfg.robot.vx_limit,  # vx_true: echte m/s je Einheit
                     dynamics=dyn, scene=SCENES[scene]())
    if imu:
        cfg.imu.pitch_ref_deg = 2.5   # wie nach calibrate.py wall
    if not wheels:
        cfg.odom.source = "command"          # sonst sucht App die Raddaten in /proc
    app = App(cfg, robot, world, imu=SimImu(world, cfg.imu) if imu else None,
              wheels=SimWheels(world, cfg.odom) if wheels else None)
    if not imu:
        app.imu = None
    app.det = FakeDetector(world)
    ph = Phantoms(world, app.fse, phantoms) if phantoms else None
    if record is not None and app.gridmap is not None:
        add = app.gridmap.add_freespace
        def add_rec(fs, pd, pose=None):
            record.append({"columns": np.array(fs.columns, np.float32), "horizon_row": fs.horizon_row,
                           "known_from": None if fs.known_from is None else np.array(fs.known_from, np.float32),
                           "pitch_cap": pd, "pitch_ins": app.last_pitch_dev,
                           "pose_cap": pose, "pose_ins": app.gridmap.pose(),
                           "balancing": robot.state.balancing, "v": robot.command[0], "w": robot.command[1],
                           "body_h": app.geo.body_height_mm, "truth": (world.x, world.y, world.th),
                           "t": time.monotonic()})
            add(fs, pd, pose)
        app.gridmap.add_freespace = add_rec
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
    done_t = {}
    if patrol:
        def to_patrol():
            while not app.running: time.sleep(0.02)
            t0 = time.monotonic()
            while app.running and not getattr(app.behavior, "done", False): time.sleep(0.2)
            done_t["s"] = round(time.monotonic() - t0, 1)
            time.sleep(1.5)
            pts = pick_route(app.gridmap, cfg)
            done_t["route"] = app.set_route(pts, False, [])
            done_t["pts"] = [(round(a, 2), round(b, 2)) for a, b in pts]
            app.set_mode("patrol", confirmed=True)
            while app.running and app.behavior.name != "patrol": time.sleep(0.05)
            t1 = time.monotonic()
            while app.running and not getattr(app.behavior, "done", False): time.sleep(0.2)
            done_t["streife_s"] = round(time.monotonic() - t1, 1)
            time.sleep(1.0)
            app.stop()
        threading.Thread(target=to_patrol, daemon=True).start()
    if mode == "explore":
        def watch():
            while not app.running: time.sleep(0.02)
            t0 = time.monotonic()
            while app.running:
                if getattr(app.behavior, "done", False):
                    done_t["s"] = round(time.monotonic() - t0, 1)
                    time.sleep(1.0)                  # Speichern und letzten Status abwarten
                    app.stop()
                time.sleep(0.2)
        threading.Thread(target=watch, daemon=True).start()
    app.run(duration=secs)
    if video:
        with app.snap_cond: app.snap_cond.notify_all()
        th.join(timeout=3)
    L = world.log
    gm = getattr(app, "gridmap", None)
    grid = None
    if gm is not None:
        # Karten-KS = Startpose -> Welt
        x0, y0, th0 = L[0]["x"], L[0]["y"], L[0]["th"]
        c0, s0 = math.cos(th0), math.sin(th0)
        ex = x0 + gm.x * c0 - gm.y * s0
        ey = y0 + gm.x * s0 + gm.y * c0
        path = [(r["x"], r["y"]) for r in L]
        grid = {"belegt_zellen": len(gm.occupied_cells()),
                **map_quality(gm.lo, gm.res, gm.cfg.l_show, world.scene, (x0, y0, th0), path),
                "pose_fehler_m": round(math.hypot(ex - world.x, ey - world.y), 3),
                "kurs_fehler_deg": round(math.degrees((th0 + gm.th - world.th + math.pi) % (2 * math.pi) - math.pi), 1)}
    out = {"imu_sync": (app.imu_sync.reliable, app.imu_sync.sign, round(app.imu_sync.offset * 1000), round(app.imu_sync.corr, 2)) if imu else None,
           "collisions": world.collisions, "coll_frames": world.coll_frames, "dist_m": round(world.dist_travelled, 2), "frames": len(L),
           "loop_ms": round(app._loop_ms, 1), "odom": app.odom_src, "odom_m": round(app.odom_m, 2),
           "grid": grid, "status": app._cmd.status, "fertig_s": done_t.get("s"),
           "explore": app.behavior.info() if hasattr(app.behavior, "info") else None,
           "plan_ms": round(app.planner.ms, 1) if app.planner is not None else None}
    if mode == "pet":
        out.update(pet=app.behavior.info() if app.behavior.name == "pet" else None,
                   aktionen=[v for _, n, v in robot.writes if n == "action"],
                   weg_von_start_m=round(math.hypot(world.x - L[0]["x"], world.y - L[0]["y"]), 3))
    if patrol:
        out.update(route=done_t.get("pts"), route_fehler=done_t.get("route"), streife_s=done_t.get("streife_s"),
                   karte_gesperrt=gm.locked, belegt_ende=len(gm.occupied_cells()))
    if record is not None:
        out.update(start_pose=(L[0]["x"], L[0]["y"], L[0]["th"]), path=[(r["x"], r["y"]) for r in L])
    if home:
        x0, y0 = L[0]["x"], L[0]["y"]
        k = min(switch.get("frame", len(L)) , len(L)) - 1
        out.update(home_weg_m=round(math.hypot(L[k]["x"] - x0, L[k]["y"] - y0), 2),
                   home_end_m=round(math.hypot(world.x - x0, world.y - y0), 2),
                   home_status=app._cmd.status)
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
    args = [a for i, a in enumerate(sys.argv) if i >= 3 and not a.startswith("--") and sys.argv[i - 1] not in ("--set", "--scene", "--phantoms", "--animal", "--speed")]
    rider = "--rider" in sys.argv
    ov = dict(RIDER) if rider else {}
    for i, a in enumerate(sys.argv):
        if a == "--set":
            k, v = sys.argv[i + 1].split("=")
            ov[k] = eval(v, {})
    print(run(mode, secs, args[0] if args else None, imu="--imu" in sys.argv, fs_thread="--fs-thread" in sys.argv,
              wheels="--wheels" in sys.argv, method="kanten" if "--kanten" in sys.argv else None,
              dyn="--dyn" in sys.argv, overrides=ov, vx_true=RIDER_VX if rider else None,
              scene=sys.argv[sys.argv.index("--scene") + 1] if "--scene" in sys.argv else "raum",
              phantoms=int(sys.argv[sys.argv.index("--phantoms") + 1]) if "--phantoms" in sys.argv else 0,
              animal=sys.argv[sys.argv.index("--animal") + 1] if "--animal" in sys.argv else "person",
              speed=float(sys.argv[sys.argv.index("--speed") + 1]) if "--speed" in sys.argv else None))
