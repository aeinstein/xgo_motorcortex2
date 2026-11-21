#!/usr/bin/env python3
"""Tiefenkarten mit Depth Anything 3 (DA3) für geo_label.py (--tiefe). Läuft in einem eigenen venv auf kimmy
(~/floornet/da3/.venv: DA3 verlangt numpy < 2, das FloorNet-venv bleibt unangetastet).

  # jedes Bild einzeln (DA3METRIC-LARGE, Apache 2.0):
  python da3_tiefe.py --ids ids_pruef.txt --modell depth-anything/DA3METRIC-LARGE --fenster 1 --out tiefe_metric
  # Bildfolgen: je Fahrt aufeinanderfolgende IDs in Fenstern zu 8, gemeinsam geschätzt (Any-View, CC BY-NC 4.0):
  python da3_tiefe.py --ids ids_pruef.txt --modell depth-anything/DA3-LARGE-1.1 --fenster 8 --out tiefe_l11_f8

Je Bild <out>/<fahrt>/<datei>.npz mit tiefe (H, W, float16, Abstand entlang der Blickachse, Modellmaß) und conf,
auf Bildgröße hochgerechnet. Der Maßstab ist egal: geo_label.py passt die Bodenebene im Modellmaß ein und skaliert auf
die Kamerahöhe. Die Kamerageometrie (f, Bildmitte) kommt dort aus der Rider-Konfiguration, nicht aus DA3.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", required=True, help="Datei mit fahrt/datei.jpg je Zeile")
    ap.add_argument("--data", default=os.path.expanduser("~/floornet/data/rider/fahrt"))
    ap.add_argument("--modell", default="depth-anything/DA3METRIC-LARGE")
    ap.add_argument("--fenster", type=int, default=1, help="Bilder je Aufruf (1 = einzeln, >1 = gemeinsam)")
    ap.add_argument("--res", type=int, default=504, help="Rechenauflösung (lange Seite, Vielfaches von 14)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    import torch
    from depth_anything_3.api import DepthAnything3

    ids = [z.strip() for z in open(a.ids, encoding="utf-8") if z.strip()]
    je_fahrt: dict = {}
    for k in ids:
        je_fahrt.setdefault(k.split("/")[0], []).append(k)
    gruppen = []
    for ks in je_fahrt.values():
        ks = sorted(ks)
        gruppen += [ks[i:i + a.fenster] for i in range(0, len(ks), a.fenster)]
    ziel = lambda k: os.path.join(a.out, k.rsplit(".", 1)[0] + ".npz")
    gruppen = [g for g in gruppen if not all(os.path.exists(ziel(k)) for k in g)]
    print("%d Bilder, %d Gruppen offen, Modell %s" % (len(ids), len(gruppen), a.modell), flush=True)
    if not gruppen:
        return 0
    m = DepthAnything3.from_pretrained(a.modell).to("cuda").eval()
    t0, n = time.time(), 0
    for g in gruppen:
        imgs = [cv2.imread(os.path.join(a.data, k))[..., ::-1].copy() for k in g]
        H, W = imgs[0].shape[:2]
        with torch.inference_mode():
            p = m.inference(imgs, process_res=a.res)
        for j, k in enumerate(g):
            t = cv2.resize(p.depth[j].astype(np.float32), (W, H), interpolation=cv2.INTER_LINEAR)
            c = cv2.resize(p.conf[j].astype(np.float32), (W, H), interpolation=cv2.INTER_LINEAR) \
                if p.conf is not None else np.ones((H, W), np.float32)
            os.makedirs(os.path.dirname(ziel(k)), exist_ok=True)
            np.savez_compressed(ziel(k), tiefe=t.astype(np.float16), conf=c.astype(np.float16))
        n += len(g)
        if n % 64 < len(g):
            print("%5d Bilder  %.0f ms/Bild" % (n, 1000 * (time.time() - t0) / n), flush=True)
    print("fertig: %d Bilder, %.0f ms/Bild, VRAM max %.1f GB" % (
        n, 1000 * (time.time() - t0) / n, torch.cuda.max_memory_allocated() / 1e9), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
