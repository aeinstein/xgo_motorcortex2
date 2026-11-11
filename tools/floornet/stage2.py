#!/usr/bin/env python3
"""FloorNet-Daten, Stage1 -> Stage2: grüne Bearbeitungen (<stage1>/<fahrt>/<bild>.png) -> Masken (0/255, gleicher Pfad
unter <stage2>). Rechnung unverändert comfy_label.floor_mask. Vorhandene Masken bleiben, also jederzeit wiederholbar.

  python stage2.py ~/floornet/stage1 ~/floornet/stage2 [--jobs 4] [--steht_m 0.03 --steht_grad 3]

Aussortiert wird ab hier, ohne eine Datei zu löschen: Standbilder (Pose aus meta.jsonl kaum anders als beim letzten
behaltenen Bild) und was im Betrachter von Hand als untauglich markiert wurde (<stage1>/aus_hand.txt). Beides steht im
Index für den Betrachter und gilt für Stage3 und die Label-Dateien (stage3_npz.py).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
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


HAND = "aus_hand.txt"                   # Handliste im Stage1-Verzeichnis, vom Betrachter-Server geschrieben
STEHT_M, STEHT_GRAD = 0.03, 3.0         # Standbild: weniger als so weit vom letzten behaltenen Bild entfernt und gedreht


def hand_liste(path: str) -> set[str]:
    """IDs (fahrt/bild) der Handliste. Je Zeile eine ID, "-" davor holt sie zurück; die letzte Zeile je ID gilt."""
    aus: set[str] = set()
    if path and os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            for z in f:
                z = z.strip()
                if z.startswith("-"):
                    aus.discard(z[1:])
                elif z:
                    aus.add(z)
    return aus


def posen(data: str, run: str) -> dict[str, list]:
    """Bild (ohne Endung) -> Pose [x, y, Grad] aus meta.jsonl der Fahrt; leer, wenn es keine gibt."""
    out = {}
    p = os.path.join(data, run, "meta.jsonl")
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            for z in f:
                if z.strip():
                    m = json.loads(z)
                    if m.get("pose"):
                        out[m["file"].rsplit(".", 1)[0]] = m["pose"]
    return out


def stehend(pose: dict[str, list], stems: list[str], d_m: float = STEHT_M, w_grad: float = STEHT_GRAD) -> set[str]:
    """Standbilder unter stems (in Reihenfolge): Ein Bild bleibt, wenn es mindestens d_m oder w_grad vom letzten
    behaltenen abweicht. Bilder ohne Pose bleiben immer. d_m und w_grad 0 schaltet ab."""
    aus: set[str] = set()
    if d_m <= 0 and w_grad <= 0:
        return aus
    last = None
    for s in stems:
        p = pose.get(s)
        if p is None:
            last = None
        elif last is None or math.hypot(p[0] - last[0], p[1] - last[1]) >= d_m \
                or abs((p[2] - last[2] + 180.0) % 360.0 - 180.0) >= w_grad:
            last = p
        else:
            aus.add(s)
    return aus


def aussortiert(ids: list[str], data: str = DATA, hand: str = "", d_m: float = STEHT_M, w_grad: float = STEHT_GRAD) -> dict[str, str]:
    """ID -> Grund ("hand" oder "steht") für sortierte IDs (fahrt/bild). Erst die Handliste, dann Standbilder unter
    dem Rest: Fällt das behaltene Bild einer Standfolge von Hand weg, rückt das nächste nach."""
    von_hand = hand_liste(hand)
    aus = {i: "hand" for i in ids if i in von_hand}
    je_fahrt: dict[str, list[str]] = {}
    for i in ids:
        if i not in aus:
            run, stem = i.split("/")
            je_fahrt.setdefault(run, []).append(stem)
    for run, stems in je_fahrt.items():
        for s in stehend(posen(data, run), stems, d_m, w_grad):
            aus[run + "/" + s] = "steht"
    return aus


def write_index(stage1: str, stage2: str, data: str = DATA, hand: str = "", d_m: float = 0.0, w_grad: float = 0.0) -> int:
    """stage2/index.js für betrachter.html: BILDER = IDs (fahrt/bild, zeitlich sortiert), für die alle drei Bilder
    vorliegen; AUS = davon aussortiert (ID -> Grund). Rückgabe: Zahl der nicht aussortierten."""
    ids = []
    for f in sorted(glob.glob(os.path.join(stage2, "*", "*.png"))):
        run, name = f.replace("\\", "/").split("/")[-2:]
        stem = name[:-4]
        if os.path.exists(os.path.join(stage1, run, name)) and os.path.exists(os.path.join(data, run, stem + ".jpg")):
            ids.append(run + "/" + stem)
    aus = aussortiert(ids, data, hand, d_m, w_grad)
    tmp = os.path.join(stage2, "index.js.tmp")
    with open(tmp, "w", encoding="utf-8") as fo:
        fo.write("window.BILDER = " + json.dumps(ids) + ";\nwindow.AUS = " + json.dumps(aus) + ";\n")
    os.replace(tmp, os.path.join(stage2, "index.js"))
    return len(ids) - len(aus)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage1")
    ap.add_argument("stage2")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--gruen", type=int, default=None, help="statt V >= 30: G - max(R, B) mindestens so groß")
    ap.add_argument("--steht_m", type=float, default=STEHT_M, help="Standbild: näher als so am letzten behaltenen Bild (m)")
    ap.add_argument("--steht_grad", type=float, default=STEHT_GRAD, help="... und weniger als so gedreht (Grad); beide 0 = aus")
    a = ap.parse_args()
    cv2.setNumThreads(1)
    jobs, n = jobs_for(a.stage1, a.stage2)
    os.makedirs(a.stage2, exist_ok=True)
    hand = os.path.join(os.path.abspath(a.stage1), HAND)
    with open(os.path.join(a.stage2, "info.json"), "w", encoding="utf-8") as fo:
        json.dump({**INFO, "min_gruen": a.gruen, "aus_hand": hand, "steht_m": a.steht_m, "steht_grad": a.steht_grad}, fo, ensure_ascii=False)
    with Pool(a.jobs) as pool:
        res = pool.map(partial(make_mask, min_gruen=a.gruen), jobs, chunksize=32)
    print("Stage1 %d Bilder, neu %d, Fehler %d, im Betrachter %d" % (
        n, res.count("neu"), res.count("fehler"), write_index(a.stage1, a.stage2, DATA, hand, a.steht_m, a.steht_grad)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
