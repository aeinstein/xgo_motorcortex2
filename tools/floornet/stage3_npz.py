#!/usr/bin/env python3
"""FloorNet-Daten, Stage3 -> Label-Dateien für train.py, je Fahrt eine. Filter und Aufbau wie comfy_npz.py.

  python tools/floornet/stage3_npz.py ~/floornet/stage3_p9g --labels ~/floornet/stage0/kanten --out ~/floornet/stage3_labels

Je 8 px breitem Streifen gilt die Zeile, bis zu der mindestens die Hälfte der Pixelspalten Boden hat (comfy_label.mask_rows).
Ganz dünne senkrechte Dinge fallen dabei weg, ein Schuh bleibt. Stage3 ist je Pixelspalte lückenlos, die Lückenregel von
mask_rows greift hier also nie. Die Dateien von --labels liefern nur Geometrie, Quelle und Wohnung je Fahrt.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import comfy_label as C  # noqa: E402
import comfy_npz as Z  # noqa: E402


def rows_of(path: str) -> np.ndarray | None:
    """Maske aus Stage3 -> Zeile der Grenze je Streifen. None, wenn sie nicht lesbar ist."""
    m = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if m is None or m.shape != (C.H, C.W):
        return None
    return C.mask_rows(m > 0)


def collect(stage3: str, jobs: int) -> tuple[dict, int]:
    """Fahrt -> {Datei.jpg: Zeilen}; dazu die Zahl nicht lesbarer Masken."""
    paths = sorted(glob.glob(os.path.join(stage3, "*", "*.png")))
    with Pool(jobs) as pool:
        res = pool.map(rows_of, paths, chunksize=32)
    out: dict = {}
    bad = 0
    for p, r in zip(paths, res):
        if r is None:
            bad += 1
            continue
        run, name = p.replace("\\", "/").split("/")[-2:]
        out.setdefault(run, {})[name[:-4] + ".jpg"] = r
    return out, bad


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage3")
    ap.add_argument("--labels", required=True, help="stage0/kanten: Dateien von label.py")
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--min_run", type=int, default=1, help="Fahrten mit weniger Bildern auslassen")
    a = ap.parse_args()
    cv2.setNumThreads(1)
    os.makedirs(a.out, exist_ok=True)
    runs, bad = collect(a.stage3, a.jobs)
    total = 0
    for run, frames in sorted(runs.items()):
        z = np.load(os.path.join(a.labels, run + ".npz"), allow_pickle=False)
        d, count = Z.build_run(run, frames, str(z["geom"]), str(z["source_dir"]), str(z["wohnung"]))
        print("%-10s %5d von %5d Bildern, verworfen: %s" % (run, len(d["files"]), len(frames), count or "-"), flush=True)
        if len(d["files"]) >= a.min_run:
            tmp = os.path.join(a.out, run + ".tmp.npz")
            np.savez_compressed(tmp, **d)
            os.replace(tmp, os.path.join(a.out, run + ".npz"))
            total += len(d["files"])
    print("gesamt %d, nicht lesbar %d" % (total, bad))
    return 0


if __name__ == "__main__":
    sys.exit(main())
