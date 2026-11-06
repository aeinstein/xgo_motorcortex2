#!/usr/bin/env python3
"""FloorNet: aus den Labels einer Fahrt die Bilder wählen, die fürs Training taugen.

Nachbarbilder zeigen fast dasselbe, und die Funkstrecke zum Rider schafft nur etwa 100 kB/s. Genommen
wird ein Bild, wenn der Rider seit dem letzten gewählten mindestens --dist Meter oder --yaw Grad
weiter ist, nicht schnell dreht und genug Spalten ein Label haben.

  python tools/floornet/pick.py labels/*.npz --out liste.txt [--dist 0.03] [--yaw 4] [--skip explore1]

Die Liste enthält Pfade relativ zum gemeinsamen Elternverzeichnis der Aufnahmen (für tar -T).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

MAX_TURN_DPS = 30.0
MIN_LABELED = 0.6
IGNORE = 0


def pick(x, y, th, rate, kind, dist: float = 0.03, yaw_deg: float = 4.0, min_labeled: float = MIN_LABELED) -> np.ndarray:
    """Indizes der gewählten Bilder."""
    labeled = (kind != IGNORE).mean(axis=1)
    out, last = [], None
    yaw = math.radians(yaw_deg)
    for i in range(len(x)):
        if rate[i] > MAX_TURN_DPS or labeled[i] < min_labeled:
            continue
        if last is None or math.hypot(x[i] - x[last], y[i] - y[last]) >= dist or abs(th[i] - th[last]) >= yaw:
            out.append(i)
            last = i
    return np.array(out, int)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("labels", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--dist", type=float, default=0.03)
    ap.add_argument("--yaw", type=float, default=4.0)
    ap.add_argument("--skip", action="append", default=[], help="diese Fahrt nicht nehmen")
    a = ap.parse_args()
    total = 0
    with open(a.out, "w", encoding="utf-8") as f:
        for p in a.labels:
            run = Path(p).stem
            if run in a.skip:
                continue
            d = np.load(p, allow_pickle=False)
            idx = pick(d["x"], d["y"], d["th"], d["rate"], d["kind"], a.dist, a.yaw)
            src = Path(str(d["source_dir"]))
            for i in idx:
                f.write("%s/%s/%s\n" % (src.parent.name, src.name, str(d["files"][i])))
            print("%-10s %5d von %5d Bildern" % (run, len(idx), len(d["files"])))
            total += len(idx)
    print("gesamt %d" % total)
    return 0


if __name__ == "__main__":
    sys.exit(main())
