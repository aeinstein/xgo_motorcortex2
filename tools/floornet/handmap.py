#!/usr/bin/env python3
"""FloorNet: Handlinien über die Pose auf alle Bilder einer Fahrt übertragen.

  python tools/floornet/handmap.py hand.json --labels labels/ --config rider.toml --out labels_hand/

Die Karte aus dem Kantenverfahren trägt nur, was dieses Verfahren sieht: Unter 1 m waren so die Hälfte der
Labels zu weit, weil Freistrahlen durch dunkle Schränke liefen. Hier stammt die Karte allein aus den Handlinien:
Unter der Linie ist Boden, auf der Linie steht etwas. Das Ergebnis hat denselben Aufbau wie die Dateien von
label.py, dazu "hand": Bilder mit eigener Linie (dort gilt die Linie selbst).

Eine so beschriftete Fahrt darf nicht mehr zum Prüfen desselben Netzes dienen: Prüfen über Fahrten getrennt.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import eval_hand as E  # noqa: E402
import label as L  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402

R_OCC = 1.6         # weiter weg ist eine Handlinie in Metern ungenau (1 Zeile > 8 cm): nur noch Freistrahl
MARGIN = 0.10       # Freistrahl endet so weit vor der Linie, wie im Kartenbau von gridmap
R_MIN = 0.15
L_HIT, L_FREE, L_MAX = 2.0, 1.0, 3.0    # ein Treffer wiegt zwei Freistrahlen auf: Linien sind verlässlich


def draw_line(gm: GridMap, hit: np.ndarray, free: np.ndarray, pose, pitch: float, rows, ok, geo) -> None:
    """Eine Handlinie in die Zählfelder eintragen. rows: Zeile je Spalte, ok: Spalte zählt."""
    x, y, th = pose
    n = len(rows)
    bear = geo.bearing((np.arange(n) + 0.5) * geo.W / n)
    rad = 1.0 / np.cos(bear)
    d = L.row_dist(geo, rows, pitch)
    for c in np.nonzero(ok)[0]:
        beyond = not np.isfinite(d[c]) or d[c] > R_OCC
        length = (R_OCC if beyond else d[c]) * rad[c]
        a = th + bear[c]
        s = np.arange(R_MIN, max(length - MARGIN, R_MIN), gm.res / 2)
        if len(s):
            r, k = gm.cell(x + s * math.cos(a), y + s * math.sin(a))
            m = (r >= 0) & (r < gm.n) & (k >= 0) & (k < gm.n)
            np.add.at(free, (r[m], k[m]), 1)
        if not beyond:
            r, k = gm.cell(x + length * math.cos(a), y + length * math.sin(a))
            if 0 <= r < gm.n and 0 <= k < gm.n:
                hit[r, k] += 1


def label_run(npz: str, items: list, unusable: set, cfg) -> dict:
    """items: (Datei, Zeilen, gültig) der Handbilder dieser Fahrt. Rückgabe: Felder für die neue Label-Datei."""
    z = np.load(npz, allow_pickle=False)
    d = {k: z[k] for k in z.files}
    geo = L.make_geo(cfg)
    pos = {str(f): i for i, f in enumerate(d["files"])}
    src = str(d["source_dir"])
    metas = {m["file"]: m for m in L.load_run(src)}
    mv = np.stack([L.movable_columns(metas.get(str(f), {}), geo.W, L.NCOLS) for f in d["files"]])
    gm = GridMap(cfg.grid, geo, 160, 120)
    hit = np.zeros(gm.lo.shape, np.int32)
    free = np.zeros(gm.lo.shape, np.int32)
    for f, rows, ok in items:
        j = pos[f]
        draw_line(gm, hit, free, (float(d["x"][j]), float(d["y"][j]), float(d["th"][j])), float(d["pitch"][j]), rows, ok & ~mv[j], geo)
    gm.lo[:] = np.clip(L_HIT * hit - L_FREE * free, -L_MAX, L_MAX)
    row, kind = L.cast_labels(gm, geo, d["x"], d["y"], d["th"], d["pitch"])
    kind[mv] = L.IGNORE                 # was sich bewegt, stimmt nicht mit der Karte überein
    hand = np.zeros(len(row), bool)
    for f, rows, ok in items:           # im eigenen Bild gilt die Linie selbst, auch in der Ferne und auf Personen
        j = pos[f]
        hand[j] = True
        row[j] = rows
        kind[j] = np.where(ok, L.EXACT, L.IGNORE)
    for f in unusable:
        if f in pos:
            kind[pos[f]] = L.IGNORE
    d.update(row=row, kind=kind, hand=hand, lo=gm.lo.astype(np.float16))
    return d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hand")
    ap.add_argument("--labels", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    cfg = L.load_cfg(a.config, [])
    hand = E.load_hand(a.hand)
    raw = json.load(open(a.hand, encoding="utf-8"))["bilder"]
    by_run: dict = {}
    for k, (rows, ok) in hand.items():
        run, f = k.split("/", 1)
        by_run.setdefault(run, []).append((f, rows, ok))
    os.makedirs(a.out, exist_ok=True)
    for run, items in sorted(by_run.items()):
        unusable = {k.split("/", 1)[1] for k, e in raw.items() if k.startswith(run + "/") and e.get("unbrauchbar")}
        d = label_run(os.path.join(a.labels, run + ".npz"), items, unusable, cfg)
        np.savez_compressed(os.path.join(a.out, run + ".npz"), **d)
        k = d["kind"]
        print("%-10s %3d Linien: %5d Bilder, exakt %4.1f %%, mindestens %4.1f %%, nichts %4.1f %%" % (
            run, len(items), len(k), 100 * (k == L.EXACT).mean(), 100 * (k == L.ATLEAST).mean(), 100 * (k == L.IGNORE).mean()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
