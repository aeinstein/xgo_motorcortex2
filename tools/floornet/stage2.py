#!/usr/bin/env python3
"""FloorNet-Daten, Stage1 -> Stage2: grüne Bearbeitungen (<stage1>/<fahrt>/<bild>.png) -> Masken (0/255, gleicher Pfad
unter <stage2>). Rechnung unverändert comfy_label.floor_mask. Vorhandene Masken bleiben, also jederzeit wiederholbar.

  python stage2.py ~/floornet/stage1 ~/floornet/stage2 [--jobs 4]
"""
from __future__ import annotations

import argparse
import glob
import json
import os
from functools import partial
from multiprocessing import Pool

import cv2
import numpy as np

import comfy_label as C  # gleiches Verzeichnis

DATA = os.path.expanduser("~/floornet/data/rider/fahrt")
INFO = {"rechnung": "comfy_label.floor_mask", "groesse": [C.W, C.H], "hsv": "H 60 +- 12, S >= 120; V >= 30 nur ohne min_gruen", "oeffnen": "3x3"}


def make_mask(job: tuple[str, str], min_gruen: int | None = None) -> str:
    """Eine Maske schreiben, atomar. Rückgabe: "neu" oder "fehler"."""
    src, dst = job
    edit = cv2.imread(src, cv2.IMREAD_COLOR)
    if edit is None:
        return "fehler"
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    tmp = dst + ".tmp.png"
    cv2.imwrite(tmp, C.floor_mask(edit, min_gruen).astype(np.uint8) * 255)
    os.replace(tmp, dst)
    return "neu"


def jobs_for(stage1: str, stage2: str) -> tuple[list[tuple[str, str]], int]:
    """Noch fehlende Masken als (Quelle, Ziel); dazu die Zahl der Bilder in Stage1."""
    files = sorted(glob.glob(os.path.join(stage1, "*", "*.png")))
    jobs = []
    for f in files:
        dst = os.path.join(stage2, *f.replace("\\", "/").split("/")[-2:])
        if not os.path.exists(dst):
            jobs.append((f, dst))
    return jobs, len(files)


def write_index(stage1: str, stage2: str, data: str = DATA) -> int:
    """stage2/index.js für betrachter.html: IDs (fahrt/bild, zeitlich sortiert), für die alle drei Bilder vorliegen."""
    ids = []
    for f in sorted(glob.glob(os.path.join(stage2, "*", "*.png"))):
        run, name = f.replace("\\", "/").split("/")[-2:]
        stem = name[:-4]
        if os.path.exists(os.path.join(stage1, run, name)) and os.path.exists(os.path.join(data, run, stem + ".jpg")):
            ids.append(run + "/" + stem)
    tmp = os.path.join(stage2, "index.js.tmp")
    with open(tmp, "w", encoding="utf-8") as fo:
        fo.write("window.BILDER = " + json.dumps(ids) + ";\n")
    os.replace(tmp, os.path.join(stage2, "index.js"))
    return len(ids)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage1")
    ap.add_argument("stage2")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--gruen", type=int, default=None, help="statt V >= 30: G - max(R, B) mindestens so groß")
    a = ap.parse_args()
    cv2.setNumThreads(1)
    jobs, n = jobs_for(a.stage1, a.stage2)
    os.makedirs(a.stage2, exist_ok=True)
    with open(os.path.join(a.stage2, "info.json"), "w", encoding="utf-8") as fo:
        json.dump({**INFO, "min_gruen": a.gruen}, fo, ensure_ascii=False)
    with Pool(a.jobs) as pool:
        res = pool.map(partial(make_mask, min_gruen=a.gruen), jobs, chunksize=32)
    print("Stage1 %d Bilder, neu %d, Fehler %d, im Betrachter %d" % (n, res.count("neu"), res.count("fehler"), write_index(a.stage1, a.stage2)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
