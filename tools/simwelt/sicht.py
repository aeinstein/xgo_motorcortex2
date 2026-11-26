#!/usr/bin/env python3
"""Bilder zu einem Ort im gemeinsamen Bezug: python sicht.py index|X,Y[,R] ...  -> bogen/sicht_X_Y.jpg (12 Blickrichtungen)."""
import json, math, os, sys
import cv2
import numpy as np

DATA = os.path.expanduser("~/floornet/data/rider/fahrt")
TRAFO, KOMP, IDX = "regK/trafo.json", "komp", "index.json"


def index():
    T = json.load(open(TRAFO))
    out = []
    for run, t in T.items():
        if not (t["anteil"] > 0.4 and t["zweit"] < 0.9 * max(t["score"], 1e-9)):
            continue
        P = json.load(open(os.path.join(KOMP, run + ".json")))
        c, s = math.cos(t["th"]), math.sin(t["th"])
        for k, (x, y, th) in P.items():
            out.append([run, int(k), round(c * x - s * y + t["dx"], 3), round(s * x + c * y + t["dy"], 3), round(th + t["th"], 4)])
    json.dump(out, open(IDX, "w"))
    print(len(out), "Bilder im Index")


def bogen(x, y, r, name):
    I = json.load(open(IDX))
    tiles = []
    for b in range(12):
        ziel = math.radians(b * 30)
        best = None
        for run, k, px, py, th in I:
            d = math.hypot(px - x, py - y)
            if d > r:
                continue
            dw = abs((th - ziel + math.pi) % (2 * math.pi) - math.pi)
            if dw > math.radians(12):
                continue
            tag = 0.0 if int(run[9:11]) < 17 else 0.6          # Tageslicht bevorzugt
            kost = d + dw + tag
            if best is None or kost < best[0]:
                best = (kost, run, k, px, py, th)
        if best is None:
            im = np.zeros((300, 400, 3), np.uint8)
        else:
            _, run, k, px, py, th = best
            im = cv2.resize(cv2.imread(os.path.join(DATA, run, "%06d.jpg" % k)), (400, 300))
            txt = "%d deg (%.1f, %.1f) %s" % (b * 30, px, py, run[4:])
            cv2.putText(im, txt, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(im, txt, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        tiles.append(im)
    cv2.imwrite(name, np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, 12, 4)]), [cv2.IMWRITE_JPEG_QUALITY, 78])


if sys.argv[1] == "index":
    index()
else:
    for arg in sys.argv[1:]:
        v = [float(t) for t in arg.split(",")]
        bogen(v[0], v[1], v[2] if len(v) > 2 else 0.5, "bogen/sicht_%s.jpg" % arg.replace(",", "_"))
