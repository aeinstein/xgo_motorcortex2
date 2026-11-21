#!/usr/bin/env python3
"""FloorNet: Netz gegen die Stage3-Labels von Prüffahrten. Seit 2026-11-10 der Maßstab: Die Labels aus ComfyUI
gelten als Wahrheit.

  python tools/floornet/eval_labels.py lauf/ --labels stage3_pruef/ \\
      --fahrten explore10,explore11,explore12 [--bilder aus/]

Prüffahrten dürfen nie im Training des gemessenen Netzes gewesen sein. Bänder und Schwelle wie train.py:
fälschlich frei = Vorhersage mehr als 15 cm hinter dem Label, fälschlich blockiert = mehr als 15 cm davor.
--bilder schreibt je Fahrt eine Übersicht (train.tiles: grün Label, magenta Netz).
"""
from __future__ import annotations

import argparse
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

NEAR_M = 1.0


def near_counts(pred_row, label_row, kind, pitch, geo, exact: int, far: float, near: float = NEAR_M) -> np.ndarray:
    """Unter near m: Zahl der Spalten, fälschlich frei, fälschlich blockiert."""
    import train as T
    with np.errstate(invalid="ignore"):
        dp, dl = T.row_dist(geo, pred_row, pitch), T.row_dist(geo, label_row, pitch)
        m = (kind == exact) & (dl < near) & np.isfinite(pred_row)
        e = dp - dl
        return np.array([m.sum(), (m & (e > far)).sum(), (m & (e < -far)).sum()], float)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lauf", help="Verzeichnis mit floornet.pt")
    ap.add_argument("--labels", required=True, help="Stage3-Label-Dateien der Prüffahrten (stage3_npz.py)")
    ap.add_argument("--fahrten", required=True, help="mit Komma getrennt")
    ap.add_argument("--cache", default=os.path.expanduser("~/floornet/cache"))
    ap.add_argument("--dist", type=float, default=0.005)
    ap.add_argument("--yaw", type=float, default=0.5)
    ap.add_argument("--norm", type=int, default=0)
    ap.add_argument("--bilder", default="", help="Verzeichnis für Übersichtsbilder")
    a = ap.parse_args()

    import torch

    import train as T
    from net import load_net

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    net = load_net(a.lauf, dev)
    with Pool(12) as pool:
        runs = [T.load_run(os.path.join(a.labels, r + ".npz"), a.cache, a.dist, a.yaw, pool) for r in a.fahrten.split(",") if r]
    res, total = {}, np.zeros(3)
    for r in runs:
        pr = T.predict(net, r, dev, bool(a.norm))
        res[r["run"]] = {"netz": T.band_report(pr, r["row"], r["kind"], r["pitch"], r["geom"])}
        total += near_counts(pr, r["row"], r["kind"], r["pitch"], r["geom"], T.EXACT, T.FAR_M)
        print("%-10s %5d Bilder" % (r["run"], len(r["imgs"])), flush=True)
        if a.bilder:
            os.makedirs(a.bilder, exist_ok=True)
            T.tiles(net, r, os.path.join(a.bilder, r["run"] + "_netz.jpg"), dev, bool(a.norm))
    print(T.show(res))
    n, fr, bl = total
    if n:
        print("unter %.1f m, alle Fahrten: Spalten %d, fälschlich frei %.1f %%, fälschlich blockiert %.1f %%" % (
            NEAR_M, n, 100 * fr / n, 100 * bl / n))
    return 0


if __name__ == "__main__":
    sys.exit(main())
