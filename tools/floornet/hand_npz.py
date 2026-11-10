#!/usr/bin/env python3
"""FloorNet: Handlinien als Label-Datei nur mit den Bildern, die eine eigene Linie haben, ohne Karte.

  python tools/floornet/hand_npz.py hand.json --labels labels_u3/ --runs unten3,unten4 --out labels_hand_mix/

Für train.py --hand_labels labels_hand_mix/ --hand_only 1. handmap.py überträgt die Linien zusätzlich über die Pose
auf die Nachbarbilder; das braucht es hier nicht, wenn die übrigen Bilder ihre Labels aus ComfyUI bekommen.
Eine Fahrt mit Handlinien im Training darf dasselbe Netz nicht prüfen.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import eval_hand as E  # noqa: E402

EXACT, IGNORE = 1, 0


def build_run(z, items: dict) -> dict:
    """z: Felder der Datei von label.py als dict. items: Datei -> (Zeilen, gültig). Rückgabe: nur diese Bilder, hand = True."""
    pos = {str(f): i for i, f in enumerate(z["files"])}
    names = sorted(f for f in items if f in pos)
    idx = np.array([pos[f] for f in names], int)
    d = {k: v for k, v in z.items() if k not in ("lo", "hand")}
    for k in ("files", "t", "x", "y", "th", "rate", "pitch", "kanten_row"):
        d[k] = d[k][idx]
    d["row"] = np.stack([items[f][0] for f in names]).astype(np.float32)
    d["kind"] = np.stack([np.where(items[f][1], EXACT, IGNORE) for f in names]).astype(np.uint8)
    d["hand"] = np.ones(len(names), bool)
    return d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hand")
    ap.add_argument("--labels", required=True)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    hand = E.load_hand(a.hand)
    os.makedirs(a.out, exist_ok=True)
    for run in a.runs.split(","):
        items = {k.split("/", 1)[1]: v for k, v in hand.items() if k.split("/", 1)[0] == run}
        z = np.load(os.path.join(a.labels, run + ".npz"), allow_pickle=False)
        d = build_run({k: z[k] for k in z.files}, items)
        np.savez_compressed(os.path.join(a.out, run + ".npz"), **d)
        print("%-10s %3d Linien" % (run, len(d["files"])), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
