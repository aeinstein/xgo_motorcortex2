#!/usr/bin/env python3
"""FloorNet-Daten, Stage2 -> Stage3: Je Pixelspalte zählt nur der Boden, der von unten durchgehend weiß ist. Ab dem
ersten schwarzen Pixel ist alles darüber schwarz; Inseln und Löcher gibt es danach nicht mehr. Vorhandene Masken bleiben.

  python stage3.py ~/floornet/stage2_p9g ~/floornet/stage3_p9g [--jobs 2]
"""
from __future__ import annotations

import argparse
import glob
import json
import os
from multiprocessing import Pool

import cv2
import numpy as np

import stage2 as S2  # gleiches Verzeichnis


def from_bottom(mask: np.ndarray) -> np.ndarray:
    """Bool-Maske -> nur der je Spalte vom unteren Rand aus ununterbrochene Teil."""
    return np.logical_and.accumulate(mask[::-1], axis=0)[::-1]


def make_mask(job: tuple[str, str]) -> str:
    """Eine Maske schreiben, atomar. Rückgabe: "neu" oder "fehler"."""
    src, dst = job
    m = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
    if m is None:
        return "fehler"
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    tmp = dst + ".tmp.png"
    cv2.imwrite(tmp, from_bottom(m > 0).astype(np.uint8) * 255)
    os.replace(tmp, dst)
    return "neu"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage2")
    ap.add_argument("stage3")
    ap.add_argument("--jobs", type=int, default=2)
    a = ap.parse_args()
    cv2.setNumThreads(1)
    jobs, n = S2.jobs_for(a.stage2, a.stage3)
    os.makedirs(a.stage3, exist_ok=True)
    with open(os.path.join(a.stage3, "info.json"), "w", encoding="utf-8") as fo:
        json.dump({"rechnung": "je Pixelspalte von unten bis zum ersten schwarzen Pixel", "quelle": a.stage2}, fo, ensure_ascii=False)
    with Pool(a.jobs) as pool:
        res = pool.map(make_mask, jobs, chunksize=32)
    print("Stage2 %d Masken, neu %d, Fehler %d, im Betrachter %d" % (n, res.count("neu"), res.count("fehler"), S2.write_index(a.stage2, a.stage3)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
