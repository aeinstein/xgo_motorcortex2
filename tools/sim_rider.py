"""Virtueller Rider: Simulator mit der echten xgonav-Schnittstelle, gesteuert über die Rider-Weboberfläche (kimmy).

    python tools/sim_rider.py --proc /srv/xgo_sim/proc [--port 8793] [--view-port 8792] [--scene raum]
                              [--ohne-person] [--phantoms n] [--set abschnitt.name=wert ...]

Läuft unbegrenzt im Modus idle, Rider-Werte wie tests/sim_run.py --wheels --dyn --rider --fs-thread.
- xgonav.web auf 127.0.0.1:<port>: dieselbe Schnittstelle wie am Rider (web/api.php, /ws, /map.png, live.php /status).
- --proc: Verzeichnis statt /proc/XGORider. SimRobot spiegelt height/roll/action/state/battery (robot.py), dieser
  Faden schreibt imu_log und wheels_log im Treiberformat für tools/live.php (Lage-Modell, Räder).
- 3D-Ansicht (tools/sim3d.html) auf --view-port, die letzten 5 Minuten.
Einspielen: tools/sim_kimmy.sh (nginx :8791 -> web/, php-fpm-Pool mit XGO_NAV/XGO_PROC/XGO_SIM).
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from xgonav.app import App  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.robot import SimRobot  # noqa: E402
from xgonav.sim import FakeDetector, Person, Phantoms, SCENES, SimFloorNet, SimImu, SimWheels, SimWorld  # noqa: E402
from tests.sim_run import RIDER, RIDER_VX, apply_overrides  # noqa: E402

RING = 300            # Zeilen je Log (6 s bei 50 Hz; live.php holt alle 0,3 s)


def imu_zeile(seq: int, t_ns: int, pitch_sim: float, yaw_deg: float) -> str:
    """Treiberformat /proc/XGORider/imu_log: seq t_ns roll pitch yaw in Milligrad. Firmware: Nick + = vorn tief,
    der Simulator zählt Nick + = Nase hoch (wie SimImu mit Vorzeichen -1); Gier fortlaufend, + = links."""
    return "%d %d %d %d %d" % (seq, t_ns, 0, round(-math.degrees(pitch_sim) * 1000), round(yaw_deg * 1000))


def rad_zeile(seq: int, t_ns: int, ticks_l: int, ticks_r: int, fw_ms: int) -> str:
    """Treiberformat /proc/XGORider/wheels_log: seq t_ns Ticks links rechts (1024/U, vorwärts +) fw_ms."""
    return "%d %d %d %d %d" % (seq, t_ns, ticks_l, ticks_r, fw_ms)


def ersetzen(pfad: str, text: str) -> None:
    tmp = "%s.%d.tmp" % (pfad, os.getpid())
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, pfad)


class Spiegel:
    """imu_log und wheels_log mit 50 Hz aus der Wahrheit des Simulators (nur Anzeige im Web)."""

    def __init__(self, world, robot, proc: str, m_per_tick: float, hz: float = 50.0):
        self.world, self.robot, self.proc, self.m_per_tick, self.hz = world, robot, proc, m_per_tick, hz
        self.imu: list[str] = []
        self.rad: list[str] = []
        self.seq = 0
        self._th_alt = None
        self._gier = 0.0
        self._stop = threading.Event()

    def schritt(self) -> None:
        w = self.world
        if w._t0 is None:
            return
        now = time.monotonic()
        t_ns = time.monotonic_ns()
        # liegend ruht die Lage (wie SimImu)
        p = w.pitch_truth(now - w._t0) if self.robot.state.balancing else math.radians(60.0)
        if self._th_alt is not None:
            self._gier += math.degrees((w.th - self._th_alt + math.pi) % (2 * math.pi) - math.pi)
        else:
            self._gier = math.degrees(w.th)
        self._th_alt = w.th
        self.seq += 1
        self.imu.append(imu_zeile(self.seq, t_ns, p, self._gier))
        self.rad.append(rad_zeile(self.seq, t_ns, int(w.wheel_l / self.m_per_tick), int(w.wheel_r / self.m_per_tick),
                                  int((now - w._t0) * 1000)))
        del self.imu[:-RING], self.rad[:-RING]
        ersetzen(os.path.join(self.proc, "imu_log"), "\n".join(self.imu) + "\n")
        ersetzen(os.path.join(self.proc, "wheels_log"), "\n".join(self.rad) + "\n")

    def _run(self) -> None:
        while not self._stop.wait(1.0 / self.hz):
            try:
                self.schritt()
            except OSError as e:
                logging.getLogger("sim_rider").warning("Spiegel: %s", e)
                self._stop.wait(1.0)

    def start(self) -> "Spiegel":
        threading.Thread(target=self._run, name="spiegel", daemon=True).start()
        return self

    def stop(self) -> None:
        self._stop.set()


def baue(proc: str, port: int, scene: str = "raum", person: bool = True, phantoms: int = 0, overrides=None,
         fps: int | None = None):
    """Welt, SimRobot mit /proc-Spiegel und App wie tests/sim_run.py (--wheels --dyn --rider --fs-thread).
    fps: Bildrate der Welt; Standard wie die Kamera des Riders ([camera] fps, 24), damit auch der Regelschritt im
    selben Takt läuft (tests/sim_run.py rechnet mit 20)."""
    cfg = Config.neutral()
    cfg.behavior.mode = "idle"
    cfg.output.web_host, cfg.output.web_port = "127.0.0.1", port
    cfg.freespace.threaded = True
    cfg.robot.proc_dir = proc
    apply_overrides(cfg, {**RIDER, **(overrides or {})})
    robot = SimRobot(cfg.robot, proc_dir=proc)
    sc = SCENES[scene]()
    pers = Person(sc.person_weg or [(3.2, 1.3), (4.8, 4.6), (1.4, 5.0), (1.2, 3.0)], speed=0.3) if person else None
    world = SimWorld(cfg.camera, robot, pers, start=sc.start or (1.0, 1.2, 0.2),
                     mount_offset_mm=cfg.camera.mount_offset_mm, vx_mps=RIDER_VX, dynamics=True, scene=sc,
                     fps=fps or cfg.camera.fps)
    net = SimFloorNet(world)
    app = App(cfg, robot, world, imu=SimImu(world, cfg.imu), wheels=SimWheels(world, cfg.odom), floor_net=net)
    app.det = FakeDetector(world)
    if phantoms:
        app.sim_phantoms = Phantoms(world, net, app.geo, phantoms)     # Bezug halten
    return cfg, robot, world, app


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--proc", required=True, help="Verzeichnis statt /proc/XGORider (muss dem Web schreibbar sein)")
    ap.add_argument("--port", type=int, default=8793, help="xgonav-Schnittstelle (127.0.0.1)")
    ap.add_argument("--view-port", type=int, default=8792, help="3D-Ansicht (0 = aus)")
    ap.add_argument("--view-host", default="0.0.0.0")
    ap.add_argument("--scene", default="unten", choices=sorted(SCENES))
    ap.add_argument("--ohne-person", action="store_true")
    ap.add_argument("--phantoms", type=int, default=0)
    ap.add_argument("--fps", type=int, default=0, help="Bildrate der Welt (0 = wie [camera] fps)")
    ap.add_argument("--set", action="append", default=[], help="abschnitt.name=wert (Zahl, True/False, Liste)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    os.umask(0o002)                                   # Dateien für die Gruppe des Webs schreibbar
    os.makedirs(a.proc, exist_ok=True)
    import ast
    ov = {}
    for s in a.set:
        k, v = s.split("=", 1)
        ov[k] = ast.literal_eval(v)
    cfg, robot, world, app = baue(a.proc, a.port, a.scene, not a.ohne_person, a.phantoms, ov, a.fps or None)

    from xgonav.web import serve
    serve(app, cfg.output.web_host, cfg.output.web_port, cfg.output.web_fps, None)
    m_per_tick = math.pi * cfg.odom.wheel_diameter_m / cfg.odom.ticks_per_rev
    sp = Spiegel(world, robot, a.proc, m_per_tick).start()
    feed = None
    if a.view_port:
        from tools.sim3d import Feed, Hub, serve as serve3d
        hub = Hub(max_lines=3000)                     # 5 min bei 10 Hz
        serve3d(hub, a.view_port, a.view_host)
        feed = Feed(world, app, "virtueller Rider", None, hub).start()

    def ende(*_):
        app.stop()
    signal.signal(signal.SIGTERM, ende)
    signal.signal(signal.SIGINT, ende)
    print(f"Virtueller Rider: Schnittstelle http://127.0.0.1:{a.port}/, /proc in {a.proc}"
          + (f", 3D http://{a.view_host}:{a.view_port}/" if a.view_port else ""), flush=True)
    app.run(duration=None)
    sp.stop()
    if feed:
        feed.close()


if __name__ == "__main__":
    main()
