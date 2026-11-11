#!/usr/bin/env python3
"""FloorNet: Netz und Kantenverfahren gegen die Stage3-Labels von Prüffahrten. Seit 2026-11-10 der Maßstab:
Die Labels aus ComfyUI gelten als Wahrheit, die Handlinien (eval_hand.py) sind gröber.

  python tools/floornet/eval_labels.py lauf/ --labels stage3_pruef/ --kanten labels_neu/ \\
      --fahrten explore10,explore11,explore12 [--bilder aus/]

Prüffahrten dürfen nie im Training des gemessenen Netzes gewesen sein. Bänder und Schwelle wie train.py:
fälschlich frei = Vorhersage mehr als 15 cm hinter dem Label, fälschlich blockiert = mehr als 15 cm davor.
--kanten: Dateien von label.py derselben Fahrten, liefern die Kantenzeile zum Vergleich. --bilder schreibt je Fahrt
eine Übersicht (train.tiles: grün Label, gelb Kanten, magenta Netz). Summenzeile netz*: das Netz nur in den Spalten,
in denen auch Kanten eine Grenze nennt (Kanten zählt nur dort).
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


def kanten_rows(path: str, files: np.ndarray, ncols: int) -> np.ndarray:
    """Kantenzeile (roh, ohne Versatz) je Bild aus einer Datei von label.py; fehlende Bilder NaN."""
    d = np.load(path, allow_pickle=False)
    kr = d["kanten_row"]
    idx = np.linspace(0, kr.shape[1] - 1, ncols).round().astype(int)
    pos = {str(f): i for i, f in enumerate(d["files"])}
    out = np.full((len(files), ncols), np.nan, np.float32)
    for j, f in enumerate(files):
        i = pos.get(str(f))
        if i is not None:
            out[j] = kr[i, idx]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lauf", help="Verzeichnis mit floornet.pt")
    ap.add_argument("--labels", required=True, help="Stage3-Label-Dateien der Prüffahrten (stage3_npz.py)")
    ap.add_argument("--fahrten", required=True, help="mit Komma getrennt")
    ap.add_argument("--kanten", default="", help="Dateien von label.py derselben Fahrten")
    ap.add_argument("--cache", default=os.path.expanduser("~/floornet/cache"))
    ap.add_argument("--dist", type=float, default=0.005)
    ap.add_argument("--yaw", type=float, default=0.5)
    ap.add_argument("--norm", type=int, default=0)
    ap.add_argument("--bilder", default="", help="Verzeichnis für Übersichtsbilder")
    a = ap.parse_args()

    import torch

    import train as T
    from net import NCOLS, load_net

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    net = load_net(a.lauf, dev)
    with Pool(12) as pool:
        runs = [T.load_run(os.path.join(a.labels, r + ".npz"), a.cache, a.dist, a.yaw, pool) for r in a.fahrten.split(",") if r]
    res, total = {}, {"netz": np.zeros(3), "kanten": np.zeros(3), "netz*": np.zeros(3)}
    for r in runs:
        pr = T.predict(net, r, dev, bool(a.norm))
        kr = np.full_like(pr, np.nan)
        if a.kanten:
            kr = kanten_rows(os.path.join(a.kanten, r["run"] + ".npz"), r["files"], NCOLS)
            r["kanten"] = kr                # für tiles: hier schon auf NCOLS Spalten
        res[r["run"]] = {"netz": T.band_report(pr, r["row"], r["kind"], r["pitch"], r["geom"]),
                         "kanten": T.band_report(kr, r["row"], r["kind"], r["pitch"], r["geom"])}
        # netz*: nur Spalten, in denen auch Kanten eine Grenze nennt, sonst zählen beide über verschiedene Spalten
        for who, rows in (("netz", pr), ("kanten", kr), ("netz*", np.where(np.isfinite(kr), pr, np.nan))):
            total[who] += near_counts(rows, r["row"], r["kind"], r["pitch"], r["geom"], T.EXACT, T.FAR_M)
        print("%-10s %5d Bilder" % (r["run"], len(r["imgs"])), flush=True)
        if a.bilder:
            os.makedirs(a.bilder, exist_ok=True)
            T.tiles(net, r, os.path.join(a.bilder, r["run"] + "_netz.jpg"), dev, bool(a.norm))
    print(T.show(res))
    for who in ("netz", "netz*", "kanten"):
        n, fr, bl = total[who]
        if n:
            print("unter %.1f m, alle Fahrten, %-6s: Spalten %d, fälschlich frei %.1f %%, fälschlich blockiert %.1f %%" % (
                NEAR_M, who, n, 100 * fr / n, 100 * bl / n))
    return 0


if __name__ == "__main__":
    sys.exit(main())
