#!/usr/bin/env python3
"""FloorNet: rows.jsonl aus comfy_batch.py -> Label-Dateien für train.py (derselbe Aufbau wie label.py, je Fahrt eine).

  python tools/floornet/comfy_npz.py labels_comfy8/rows.jsonl --labels labels5/ --out labels_comfy8_npz/

Nicht jedes Bild taugt (Stichprobe 2026-11-08). Es fliegt raus:
- liegender Rider (meta "balancing" false): Kamera schaut schräg an die Decke, kommt beim Fahren nicht vor
- Grenze weit über dem Bereich des Netzes in mehr als 10 % der Spalten: Rider kippt, Horizont ganz oben
- mehr als 6 Sprünge über 40 Zeilen zwischen Nachbarspalten: Maske zerrissen
- Bild fast schwarz: Dort rät das Modell, die Linie ist gezackt

Pose und Nick stammen aus meta.jsonl, nicht aus labels5: Dort fehlen Bilder (nur längstes Stück ohne Posesprung,
nur jedes zweite). Kantenzeilen gibt es hier nicht, also taugen diese Dateien nur fürs Training, nicht als --val.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

EXACT, IGNORE = 1, 0
NCOLS = 80
KANTEN_COLS = 160           # Breite von kanten_row in label.py
ROW_MIN = 205.0             # net.ROW_TOP: Zeilen darüber kann das Netz nicht ausgeben
MAX_ABOVE = 0.10
JUMP_PX, MAX_JUMPS = 40.0, 6
MIN_GRAY = 12.0             # mittlere Helligkeit 0..255, darunter nur Rauschen


def load_rows(path: str) -> dict:
    """rows.jsonl: Fahrt -> {Datei: Zeilen}. Eine abgeschnittene letzte Zeile (Lauf noch aktiv) wird übersprungen."""
    out: dict = {}
    with open(path, encoding="utf-8") as f:
        for ln in f:
            try:
                e = json.loads(ln)
            except ValueError:
                continue
            run, name = e["id"].split("/", 1)
            out.setdefault(run, {})[name] = np.array(e["rows"], np.float32)
    return out


def frame_reason(rows: np.ndarray, balancing: bool, gray: float) -> str:
    """Grund, warum dieses Bild nicht ins Training soll; leer, wenn es taugt."""
    if not balancing:
        return "liegt"
    if (rows < ROW_MIN).mean() > MAX_ABOVE:
        return "kippt"
    if (np.abs(np.diff(rows)) > JUMP_PX).sum() > MAX_JUMPS:
        return "zerrissen"
    if gray < MIN_GRAY:
        return "dunkel"
    return ""


def load_meta(src: str) -> dict:
    out = {}
    with open(os.path.join(src, "meta.jsonl"), encoding="utf-8") as f:
        for ln in f:
            try:
                m = json.loads(ln)
            except ValueError:
                continue
            out[m["file"]] = m
    return out


def build_run(run: str, frames: dict, geom: str, src: str, wohnung: str) -> tuple[dict, dict]:
    """Rückgabe: Felder der Label-Datei und Zählung der verworfenen Bilder je Grund."""
    meta = load_meta(src)
    keep, count = [], {}
    for name in sorted(frames):
        m = meta.get(name, {})
        img = cv2.imread(os.path.join(src, name), cv2.IMREAD_GRAYSCALE)
        gray = float(img.mean()) if img is not None else 0.0
        why = frame_reason(frames[name], bool(m.get("balancing")), gray)
        if why:
            count[why] = count.get(why, 0) + 1
        else:
            keep.append(name)
    n = len(keep)
    pose = np.array([meta[f].get("pose") or [0.0, 0.0, 0.0] for f in keep], float).reshape(n, 3)
    row = np.stack([frames[f] for f in keep]) if n else np.zeros((0, NCOLS), np.float32)
    kind = np.where(row < ROW_MIN, IGNORE, EXACT).astype(np.uint8)
    d = {"files": np.array(keep, "U10"), "t": np.array([meta[f].get("t", 0.0) for f in keep], float),
         "x": pose[:, 0], "y": pose[:, 1], "th": np.radians(pose[:, 2]),
         "rate": np.zeros(n),                # Drehen stört hier nicht: Die Linie gehört zu genau diesem Bild
         "pitch": np.array([meta[f].get("pitch_dev", 0.0) for f in keep], np.float32),   # Radiant, wie label.py
         "row": row.astype(np.float32), "kind": kind,
         "kanten_row": np.full((n, KANTEN_COLS), np.nan, np.float32),
         "run": np.array(run), "wohnung": np.array(wohnung), "geom": np.array(geom), "source_dir": np.array(src)}
    return d, count


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rows")
    ap.add_argument("--labels", required=True, help="Dateien von label.py, liefern Geometrie, Quelle und Wohnung je Fahrt")
    ap.add_argument("--out", required=True)
    ap.add_argument("--min_run", type=int, default=1, help="Fahrten mit weniger Bildern auslassen")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    total = 0
    for run, frames in sorted(load_rows(a.rows).items()):
        z = np.load(os.path.join(a.labels, run + ".npz"), allow_pickle=False)
        d, count = build_run(run, frames, str(z["geom"]), str(z["source_dir"]), str(z["wohnung"]))
        print("%-10s %5d von %5d Bildern, verworfen: %s" % (run, len(d["files"]), len(frames), count or "-"), flush=True)
        if len(d["files"]) >= a.min_run:
            np.savez_compressed(os.path.join(a.out, run + ".npz"), **d)
            total += len(d["files"])
    print("gesamt", total)
    return 0


if __name__ == "__main__":
    sys.exit(main())
