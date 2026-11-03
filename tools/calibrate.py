"""Kalibrierung am echten Rider.

  python3 tools/calibrate.py turn            # Drehrichtung + °/s pro Einheit (Rider dreht 3 s!)
  python3 tools/calibrate.py wall            # Bodenkante einer Wand messen (Rider steht)
  python3 tools/calibrate.py wall --cols 0.52:0.70 --debug wand.jpg   # Spaltenbereich wählen, Kontrollbild
  python3 tools/calibrate.py solve 300:1.0 262:2.0   # aus zwei Wandmessungen Höhe+Neigung
  python3 tools/calibrate.py shading [shading.json]  # Linsenabschattung aus einem Weißbild

shading: eine gleichmäßig weiße, selbstleuchtende Fläche (z. B. Laptop mit weißem
Vollbild, maximale Helligkeit) so nah vor die Kamera halten, dass sie das ganze Bild
füllt; unscharf ist egal. Danach [freespace] shading_file = "shading.json" eintragen.

wall: Rider gerade vor eine Wand stellen (Abstand Rad-Aufstandspunkt -> Wand,
z. B. 1.0 m und 2.0 m), jeweils 'wall' laufen lassen und die Zeile notieren.
Mit Treiber-Patch 0002 wird dabei auch die IMU-Rohpitch gemessen -> [imu] pitch_ref_deg.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.camera import make_source  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.egomotion import EgoMotion  # noqa: E402
from xgonav.freespace import FreeSpaceEstimator, column_runs  # noqa: E402
from xgonav.imu import ProcImu  # noqa: E402
from xgonav.robot import ProcRobot, norm_to_u8, u8_to_norm  # noqa: E402


def turn(cfg: Config, w: float = 0.1, secs: float = 3.0) -> None:
    src = make_source(cfg.camera)
    robot = ProcRobot(cfg.robot)
    em = EgoMotion(cfg.ego, (cfg.camera.width, cfg.camera.height), cfg.camera.hfov_deg)
    src.start()
    robot.start()
    yaw = 0.0
    last = -1
    t_start = None
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs + 1.0:
            robot.drive(0.0, w)
            f = src.get(last, 1.0)
            if f is None:
                continue
            last = f.idx
            r = em.update(f.idx, f.t, f.bgr)
            # erst messen, wenn die Rampe durch ist
            if abs(robot.command[1] - w) < 1e-3:
                t_start = t_start or f.t
                if r.ok:
                    yaw += r.yaw
            t_end = f.t
    finally:
        robot.close()
        src.close()
    dur = t_end - (t_start or t_end)
    if dur <= 0.5:
        print("zu wenig Messzeit")
        return
    rate = math.degrees(yaw) / dur
    sent = u8_to_norm(norm_to_u8(w))
    expect = sent * cfg.robot.vyaw_limit_dps
    print(f"Befehl w={sent:+.3f} (u8 {norm_to_u8(w)}) -> gemessen {rate:+.1f} °/s, erwartet {expect:+.1f} °/s")
    if rate * expect < 0:
        print("!! Vorzeichen falsch: w>0 dreht nach RECHTS. In robot.py speed_z invertieren.")
    else:
        print(f"Vorzeichen ok. Für exakte Regelung: [robot] vyaw_limit_dps = {abs(rate / sent):.0f}")


def wall(cfg: Config, frames: int = 40, cols: tuple[float, float] = (0.4, 0.6), debug: str | None = None) -> None:
    # definierte Körperhöhe für die Messung (wie im Fahrbetrieb)
    hmm = cfg.behavior.height_cruise_mm
    u = int(round(255 * (hmm - cfg.robot.height_min_mm) / (cfg.robot.height_max_mm - cfg.robot.height_min_mm)))
    try:
        Path(cfg.robot.proc_dir, "height").write_text(str(u))
        time.sleep(1.5)
    except OSError as e:
        print(f"Höhe nicht gesetzt ({e}) – Messung gilt für die aktuelle Höhe")
    src = make_source(cfg.camera)
    fse = FreeSpaceEstimator(cfg.freespace, (cfg.camera.width, cfg.camera.height))
    imu = ProcImu(cfg.imu)
    has_imu = imu.start()
    src.start()
    rows = []
    last = -1
    fs = f = None
    try:
        while len(rows) < frames:
            f = src.get(last, 1.0)
            if f is None:
                continue
            last = f.idx
            fs = fse.update(f.bgr)
            n = len(fs.columns)
            c = fs.columns[int(n * cols[0]): int(n * cols[1])]
            span = fs.mask.shape[0] - fs.horizon_row
            y = (fs.mask.shape[0] - float(np.nanmedian(c)) * span) * fse.scale
            rows.append(y)
    finally:
        src.close()
    print(f"Bodenkante Spalten {cols[0]:.0%}-{cols[1]:.0%}: Zeile {np.median(rows):.1f} (Streuung {np.std(rows):.1f} px)")
    if debug and f is not None:
        import cv2
        img = f.bgr.copy()
        H, W = img.shape[:2]
        span = fs.mask.shape[0] - fs.horizon_row
        sc = W / len(fs.columns)
        for a, b in column_runs(fs.columns):
            pts = [(int((i + 0.5) * sc), int((fs.mask.shape[0] - fs.columns[i] * span) * fse.scale)) for i in range(a, b)]
            cv2.polylines(img, [np.array(pts, np.int32)], False, (0, 255, 255), 1)
        y = int(round(np.median(rows)))
        cv2.line(img, (int(W * cols[0]), y), (int(W * cols[1]), y), (0, 0, 255), 2)
        cv2.rectangle(img, (int(W * cols[0]), 0), (int(W * cols[1]), H - 1), (0, 0, 255), 1)
        cv2.imwrite(debug, img)
        print(f"Kontrollbild: {debug} (gelb: erkannte Bodengrenze, rot: Messbereich und Zeile)")
    if has_imu:
        _, ps = imu.arrays()
        imu.close()
        if len(ps):
            print(f"IMU-Rohpitch während der Messung: {math.degrees(float(np.median(ps))):+.2f}° "
                  f"(Streuung {math.degrees(float(np.std(ps))):.2f}°)")
            print("-> config.toml [imu] pitch_ref_deg = <Mittel beider Messungen>")


def solve(cfg: Config, meas: list[tuple[float, float]]) -> None:
    W, H = cfg.camera.width, cfg.camera.height
    f = (W / 2) / math.tan(math.radians(cfg.camera.hfov_deg) / 2)
    best = None
    for tilt in np.linspace(-15, 15, 3001):
        hs = [d * math.tan(math.atan2(y - H / 2, f) - math.radians(tilt)) for y, d in meas]
        if min(hs) <= 0:
            continue
        err = float(np.std(hs))
        if best is None or err < best[0]:
            best = (err, tilt, float(np.mean(hs)))
    if best is None:
        print("keine Lösung – Messungen prüfen")
        return
    _, tilt, h = best
    off = h * 1000 - cfg.behavior.height_cruise_mm
    print(f"Kamerahöhe {h * 1000:.0f} mm, Neigung {tilt:+.2f}°")
    print(f"-> config.toml [camera] tilt_deg = {tilt:.2f}, mount_offset_mm = {off:.0f}"
          f"  (bei Körperhöhe {cfg.behavior.height_cruise_mm:.0f} mm während der Messung)")


def shading(out: str = "shading.json", frames: int = 15) -> None:
    import cv2
    from picamera2 import Picamera2
    from xgonav.shading import ShadingModel
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"},
                                                 controls={"FrameRate": 30}))
    cam.start()                       # automatische Belichtung: die Fläche leuchtet selbst
    time.sleep(3)
    f = np.median(np.stack([cam.capture_array() for _ in range(frames)]), 0).astype(np.float32)
    cam.stop()
    sat = float((f.max(-1) >= 250).mean())
    valid = cv2.erode((f.max(-1) > 20).astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    print(f"Fläche deckt {100 * valid.mean():.0f} % des Bildes, übersteuert {100 * sat:.1f} %")
    if valid.mean() < 0.75:
        print("!! Fläche füllt das Bild nicht: näher an die Kamera und erneut versuchen")
        return
    if sat > 0.01:
        print("!! übersteuert: Bildschirm dunkler stellen und erneut versuchen")
        return
    m = ShadingModel.fit(f, valid)
    F = m.falloff(640, 480)
    yy, xx = np.mgrid[0:480, 0:640]
    r = np.hypot(xx - m.cx * 640, yy - m.cy * 480)
    meas = f / np.median(f[(r < 20) & valid], axis=0)
    err = np.abs(F - meas)[valid]
    print(f"Zentrum {m.cx * 640:.0f}/{m.cy * 480:.0f} px, Fit-Fehler im Mittel {err.mean():.3f}, 95 % {np.percentile(err, 95):.3f}")
    print(" Radius  | Blau Grün  Rot  (relativ zur Mitte)")
    for a in range(0, 400, 80):
        sel = valid & (r >= a) & (r < a + 80)
        if sel.sum() > 50:
            b, g, rr = F[sel].mean(0)
            print(f"  {a:3d}-{a + 80:3d} | {b:.2f} {g:.2f} {rr:.3f}")
    _, rel = m.gain_and_reliable(640, 480, Config().freespace.shading_max_gain)
    print(f"bei max. Verstärkung {Config().freespace.shading_max_gain:g}: {100 * rel.mean():.0f} % des Bildes messbar")
    m.save(out)
    print(f"-> {out}; in config.toml: [freespace] shading_file = \"{out}\"")


if __name__ == "__main__":
    cfg = Config.load(Path("config.toml") if Path("config.toml").exists() else None)
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "turn":
        turn(cfg)
    elif cmd == "wall":
        a = sys.argv
        cols = tuple(map(float, a[a.index("--cols") + 1].split(":"))) if "--cols" in a else (0.4, 0.6)
        wall(cfg, cols=cols, debug=a[a.index("--debug") + 1] if "--debug" in a else None)
    elif cmd == "solve":
        solve(cfg, [tuple(map(float, a.split(":"))) for a in sys.argv[2:]])
    elif cmd == "shading":
        shading(sys.argv[2] if len(sys.argv) > 2 else "shading.json")
    else:
        print(__doc__)
