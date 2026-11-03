"""Rad-Odometrie kalibrieren: Faktor je Rad ([odom] scale_left / scale_right).

Auf dem Rider, xgonav muss laufen (Bildverarbeitung an, Modus idle), der Rider balanciert.
Fährt über den Handmodus von xgonav geradeaus und vergleicht den Kurs aus den Rädern mit
Kamera (Ego-Motion) und IMU; optional die Strecke mit dem Maßband.

  python3 tools/calibrate_odom.py fahrt [--v 0.12] [--secs 8] [--out /tmp/odomcal.json]
      -> fährt (ca. 1 m bei v=0.12), speichert Rohwerte, zeigt vorläufige Faktoren
  python3 tools/calibrate_odom.py rechne --dist-cm 97.5 [--ref kamera|imu|mittel] [--in /tmp/odomcal.json]
      -> Faktoren mit gemessener Strecke (Radachse am Maßband, Start bis Ende)

Vorher Platz in Kamerarichtung lassen (Strecke + 30 cm). Abbruch mit Strg+C: Stillstand und
Modus idle werden immer geschrieben. Ergebnis von Hand in config.toml eintragen ([odom]).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.config import Config  # noqa: E402
from xgonav.wheels import calibrate_scales  # noqa: E402

API = "http://127.0.0.1:8088"
PROC = "/proc/XGORider"


def post(path: str) -> dict:
    return json.load(urllib.request.urlopen(urllib.request.Request(API + path, method="POST"), timeout=2))


def status() -> dict:
    return json.load(urllib.request.urlopen(API + "/status", timeout=2))


def read_log(name: str) -> list[list[int]]:
    with open(f"{PROC}/{name}") as f:
        return [[int(x) for x in ln.split()] for ln in f if ln.strip()]


class Recorder(threading.Thread):
    """Räder (wheels_log), IMU (imu_log) und Kamera-Kurs (status.yaw_vis_deg) mitschneiden."""

    def __init__(self):
        super().__init__(daemon=True)
        self.wheels, self.imu, self.cam = {}, {}, []
        self.stop_evt = threading.Event()

    def run(self):
        while not self.stop_evt.is_set():
            for r in read_log("wheels_log"):
                self.wheels[r[0]] = r              # seq t_ns ticks11 ticks21 [fw_ms]
            for r in read_log("imu_log"):
                self.imu[r[0]] = r                 # seq t_ns roll pitch yaw (Milligrad)
            try:
                s = status()
                if s.get("yaw_vis_deg") is not None:
                    self.cam.append((time.monotonic(), s["yaw_vis_deg"]))
            except OSError:
                pass
            time.sleep(0.1)


def fahrt(a, cfg) -> dict:
    s = status()
    if s["pipeline"] != "läuft":
        sys.exit("Bildverarbeitung läuft nicht (Kamera-Kurs fehlt) – in der Oberfläche starten")
    if not s.get("balancing"):
        sys.exit("Rider balanciert nicht – abgebrochen")
    if s["mode"] != "idle":
        sys.exit(f"xgonav ist im Modus {s['mode']}, erwartet idle – abgebrochen")
    if not Path(f"{PROC}/wheels_log").exists():
        sys.exit("keine Raddaten (/proc/XGORider/wheels_log) – Treiber/Firmware ohne Rad-Encoder")
    rec = Recorder()
    rec.start()
    time.sleep(2.0)                                # Ruhe vorher
    t_start = time.monotonic()
    print(f"fahre {a.secs:.1f} s geradeaus mit v={a.v:+.2f} …")
    try:
        post("/mode?name=manual")
        time.sleep(0.3)
        t_end = time.monotonic() + a.secs
        while time.monotonic() < t_end:
            if not status().get("balancing"):
                print("ABBRUCH: Rider balanciert nicht mehr")
                break
            post(f"/drive?v={a.v}&w=0")
            time.sleep(0.1)
    finally:
        try:
            post("/drive?v=0&w=0")
        finally:
            time.sleep(0.3)
            post("/mode?name=idle")
    print("Stillstand, Modus idle – Ausschwingen 4 s")
    time.sleep(4.0)
    rec.stop_evt.set()
    rec.join()

    W = [rec.wheels[k] for k in sorted(rec.wheels)]
    I = [rec.imu[k] for k in sorted(rec.imu)]
    if len(W) < 20 or len(I) < 20 or len(rec.cam) < 10:
        sys.exit(f"zu wenige Proben (Räder {len(W)}, IMU {len(I)}, Kamera {len(rec.cam)})")
    mpt = math.pi * cfg.odom.wheel_diameter_m / cfg.odom.ticks_per_rev
    wrap = lambda d: (d + 2 ** 31) % 2 ** 32 - 2 ** 31
    sl = sum(wrap(b[2] - a_[2]) for a_, b in zip(W, W[1:])) * mpt      # roh, ohne Faktor
    sr = sum(wrap(b[3] - a_[3]) for a_, b in zip(W, W[1:])) * mpt
    yaw = [math.radians(r[4] / 1000) for r in I]
    d_imu = 0.0
    for y0, y1 in zip(yaw, yaw[1:]):
        d_imu += (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
    d_cam = math.radians(rec.cam[-1][1] - rec.cam[0][1])
    res = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "v": a.v, "secs": a.secs, "sl_m": sl, "sr_m": sr,
           "dtheta_imu_rad": d_imu, "dtheta_cam_rad": d_cam, "track_m": cfg.odom.track_m,
           "wheel_diameter_m": cfg.odom.wheel_diameter_m, "n_wheels": len(W), "n_imu": len(I),
           "n_cam": len(rec.cam), "dauer_s": round(time.monotonic() - t_start, 1)}
    Path(a.out).write_text(json.dumps(res, indent=1))
    print(f"gespeichert: {a.out}")
    return res


def rechne(res: dict, dist_cm: float | None, ref: str) -> None:
    sl, sr, track = res["sl_m"], res["sr_m"], res["track_m"]
    th = {"kamera": res["dtheta_cam_rad"], "imu": res["dtheta_imu_rad"],
          "mittel": (res["dtheta_cam_rad"] + res["dtheta_imu_rad"]) / 2}[ref]
    print(f"Radwege roh: links {sl*100:.1f} cm, rechts {sr*100:.1f} cm, Mitte {(sl+sr)/2*100:.1f} cm")
    print(f"Kurs: Räder (ohne Faktor) {math.degrees((sr - sl) / track):+.1f}°, "
          f"Kamera {math.degrees(res['dtheta_cam_rad']):+.1f}°, IMU {math.degrees(res['dtheta_imu_rad']):+.1f}°")
    dist = dist_cm / 100 if dist_cm is not None else None
    kl, kr = calibrate_scales(sl, sr, th, track, dist)
    note = f"Maßband {dist_cm:.1f} cm" if dist is not None else "ohne Maßband (Mittel der Räder gilt als richtig)"
    print(f"Referenz für den Kurs: {ref}; Strecke: {note}")
    print("\nIn config.toml eintragen:\n  [odom]\n  scale_left = %.4f\n  scale_right = %.4f" % (kl, kr))
    if abs(kl - 1) > 0.08 or abs(kr - 1) > 0.08:
        print("WARNUNG: Faktor weicht mehr als 8 % ab – Messung prüfen (Schlupf, falsche Strecke?)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("was", choices=["fahrt", "rechne"])
    ap.add_argument("--v", type=float, default=0.12, help="Fahrbefehl (normiert), max_v begrenzt")
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--out", default="/tmp/odomcal.json")
    ap.add_argument("--in", dest="inp", default="/tmp/odomcal.json")
    ap.add_argument("--dist-cm", type=float, default=None, help="gemessene Strecke der Radachse")
    ap.add_argument("--ref", choices=["kamera", "imu", "mittel"], default="mittel")
    ap.add_argument("--config", default="/opt/xgonav/config.toml")
    a = ap.parse_args()
    cfg = Config.load(a.config if Path(a.config).exists() else None)
    if not 0 < a.v <= 0.15 and a.was == "fahrt":
        sys.exit("--v bitte zwischen 0 und 0.15 (vorwärts, getestete Geschwindigkeiten)")
    res = fahrt(a, cfg) if a.was == "fahrt" else json.loads(Path(a.inp).read_text())
    rechne(res, a.dist_cm, a.ref)


if __name__ == "__main__":
    main()
