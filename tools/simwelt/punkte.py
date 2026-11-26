#!/usr/bin/env python3
"""Stufe 1: je Fahrbild eine Punktwolke im Rider-Koordinatensystem (x vorn, y links, z oben, Ursprung am Boden unter
der Radachse). MoGe-2 schätzt die Punkte, die Bodenebene legt Maßstab und Lage fest (geo_label.ebene).

  python punkte.py CODE DATA OUT unten1 unten3 --jedes 3
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("code")
ap.add_argument("data")
ap.add_argument("out")
ap.add_argument("runs", nargs="+")
ap.add_argument("--jedes", type=int, default=3)
ap.add_argument("--batch", type=int, default=8)
ap.add_argument("--max_dps", type=float, default=50.0)
a = ap.parse_args()

sys.path.insert(0, a.code)
sys.path.insert(0, os.path.join(a.code, "tools", "floornet"))
import geo_label as G  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402

cfg = Config()
geo = CamGeometry.from_cfg(cfg, None)
R = G.Regeln()
print("Kamera: f %.1f cam_h %.4f tilt %.2f fwd %.3f hfov %.1f" % (geo.f, geo.cam_h, geo.tilt_deg, geo.cam_fwd, geo.hfov_deg), flush=True)
model = G.Moge()

for run in a.runs:
    src = os.path.join(a.data, run)
    ms = [json.loads(z) for z in open(os.path.join(src, "meta.jsonl"), encoding="utf-8")]
    t = np.array([m["t"] for m in ms])
    yaw = np.array([m["yaw_deg"] for m in ms])
    ypose = np.array([m["pose"][2] for m in ms])
    bal = np.array([bool(m.get("balancing")) and m["mode"] != "idle" for m in ms])
    off = float(np.median((yaw - ypose)[bal])) if bal.any() else 0.0
    print(run, "Kursversatz yaw_deg - pose: median %.2f, Streuung %.2f" % (off, float(np.std((yaw - ypose)[bal]))), flush=True)
    todo = []
    for i, m in enumerate(ms):
        if not bal[i] or m["idx"] % a.jedes or not os.path.exists(os.path.join(src, m["file"])):
            continue
        j0, j1 = max(0, i - 3), min(len(ms) - 1, i + 3)
        dps = abs(yaw[j1] - yaw[j0]) / max(1e-3, t[j1] - t[j0])
        if dps > a.max_dps:
            continue
        todo.append(i)
    od = os.path.join(a.out, run)
    os.makedirs(od, exist_ok=True)
    t0 = time.time()
    liste = []
    for b in range(0, len(todo), a.batch):
        teil = todo[b:b + a.batch]
        imgs = [cv2.imread(os.path.join(src, ms[i]["file"])) for i in teil]
        P = model.punkte(imgs, geo.hfov_deg)
        for j, i in enumerate(teil):
            m = ms[i]
            hb = m.get("height_mm")
            geo = CamGeometry.from_cfg(cfg, hb if isinstance(hb, (int, float)) else None)
            aw = math.radians(geo.tilt_deg) + float(m.get("pitch_dev") or 0.0)
            p = P[j]
            ok = np.isfinite(p).all(-1) & (p[..., 2] > 0)
            H, W = ok.shape
            eb = G.ebene(p, ok, G.unten(aw), geo.cam_h, H / 2 + geo.f * math.tan(aw) + 15, R, None)
            if eb["art"] != "ebene":
                continue
            n, d, s = eb["n"], eb["d"], eb["s"]
            u = -n
            f = np.array([0.0, 0.0, 1.0]) - n[2] * n
            f /= np.linalg.norm(f)
            li = np.cross(u, f)
            q = p[::2, ::2]
            q = np.where(ok[::2, ::2, None], q, np.nan)
            X = (q @ f) * s + geo.cam_fwd
            Y = (q @ li) * s
            Z = (d - q @ n) * s
            np.savez(os.path.join(od, "%06d.npz" % m["idx"]),
                     xyz=np.stack([X, Y, Z], -1).astype(np.float16), bgr=imgs[j][::2, ::2])
            liste.append({"idx": m["idx"], "file": m["file"], "t": m["t"], "pose": [m["pose"][0], m["pose"][1], m["yaw_deg"] - off],
                          "s": round(float(s), 4), "winkel": eb["winkel"], "inl": eb["inl"], "pitch_dev": m.get("pitch_dev"),
                          "nick_ebene": round(math.degrees(math.asin(-float(n[2]))), 3), "roll_ebene": round(math.degrees(math.asin(float(n[0]))), 3),
                          "mode": m["mode"], "height_mm": m.get("height_mm"), "cam_h": round(geo.cam_h, 4), "status": m.get("status"), "gain": (m.get("cam") or {}).get("gain")})
        if (b // a.batch) % 20 == 0:
            n_ = b + len(teil)
            print("%-8s %5d/%d  %.1f Bilder/s  behalten %d" % (run, n_, len(todo), n_ / (time.time() - t0), len(liste)), flush=True)
    json.dump(liste, open(os.path.join(od, "liste.json"), "w"))
    print(run, "fertig:", len(liste), "von", len(todo), flush=True)
