#!/usr/bin/env python3
"""Inseln und Löcher in den Bodenmasken zählen und zeigen. Ändert keine Maske.

Grüne Insel: grüne Fläche (8er-Nachbarschaft) ohne Verbindung zur Hauptfläche. Hauptfläche: die größte, die den unteren
Bildrand berührt. Berührt keine ihn, gilt das Bild als „ohne Hauptfläche“ und alle grünen Flächen zählen als Inseln.
Loch: nicht-grüne Fläche (4er-Nachbarschaft), die keinen Bildrand berührt, also ganz von Grün umschlossen ist.
Lage einer Insel: über oder unter der Grenze aus comfy_label.mask_rows, gemessen in der Spalte ihres Schwerpunkts.

  python inseln.py ~/floornet/comfy/labels_comfy8 ~/floornet/comfy/labels_comfy8_u --out inseln/

Ausgabe in --out: bilder.jsonl (je Bild), bericht.txt, tafel_zufall.jpg, tafel_inseln.jpg, tafel_loecher.jpg.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import random
from multiprocessing import Pool

import cv2
import numpy as np

import comfy_label as C  # gleiches Verzeichnis

DATA = os.path.expanduser("~/floornet/data/rider/fahrt")
AREA_EDGES = (50, 200, 1000)
AREA_NAMES = ("< 50", "< 200", "< 1000", ">= 1000")
TILE_W, TILE_H = 320, 240


def area_class(area: int) -> int:
    """Flächenklasse in px: Index in AREA_NAMES."""
    return int(np.searchsorted(AREA_EDGES, area, side="right"))


def components(mask: np.ndarray) -> tuple[np.ndarray, int | None, list[int], np.ndarray, list[int]]:
    """Bool-Maske -> (Etiketten Grün, Hauptfläche oder None, Inseln, Etiketten Nicht-Grün, Löcher)."""
    h, w = mask.shape
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    ids = range(1, n)
    bottom = [i for i in ids if st[i, cv2.CC_STAT_TOP] + st[i, cv2.CC_STAT_HEIGHT] == h]
    main = max(bottom, key=lambda i: st[i, cv2.CC_STAT_AREA]) if bottom else None
    islands = [i for i in ids if i != main]
    nh, lab_h, sh, _ = cv2.connectedComponentsWithStats((~mask).astype(np.uint8), connectivity=4)
    holes = [i for i in range(1, nh)
             if sh[i, cv2.CC_STAT_LEFT] > 0 and sh[i, cv2.CC_STAT_TOP] > 0
             and sh[i, cv2.CC_STAT_LEFT] + sh[i, cv2.CC_STAT_WIDTH] < w and sh[i, cv2.CC_STAT_TOP] + sh[i, cv2.CC_STAT_HEIGHT] < h]
    return lab, main, islands, lab_h, holes


def analyse(mask: np.ndarray) -> dict:
    """Bool-Maske (480 x 640) -> Kennzahlen eines Bildes."""
    lab, main, islands, lab_h, holes = components(mask)
    rows = C.mask_rows(mask)
    is_area, is_pos = [], []
    for i in islands:
        ys, xs = np.nonzero(lab == i)
        is_area.append(int(len(ys)))
        col = min(int(xs.mean()) * C.NCOLS // C.W, C.NCOLS - 1)
        is_pos.append("ueber" if ys.mean() < rows[col] else "unter")
    return {"haupt": main is not None, "boden": int(mask.sum()), "inseln": is_area, "lage": is_pos,
            "loecher": [int((lab_h == i).sum()) for i in holes]}


def one(job: tuple[str, str]) -> dict | None:
    key, path = job
    m = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if m is None or m.shape != (C.H, C.W):      # z. B. gerade im Schreiben
        return None
    return {"id": key, **analyse(m > 0)}


def tile(rec: dict, path: str) -> np.ndarray:
    """Originalbild mit Maske: Hauptfläche grün, Inseln rot, Löcher blau."""
    run, f = rec["id"].split("/", 1)
    img = cv2.imread(os.path.join(DATA, run, f))
    img = np.zeros((C.H, C.W, 3), np.uint8) if img is None else img
    mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE) > 0
    lab, main, islands, lab_h, holes = components(mask)
    col = np.zeros_like(img)
    col[lab == main if main is not None else np.zeros_like(mask)] = (0, 200, 0)
    if islands:
        col[np.isin(lab, islands)] = (0, 0, 255)
    if holes:
        col[np.isin(lab_h, holes)] = (255, 80, 0)
    show = np.where(col.any(axis=2, keepdims=True), (img * 0.45 + col * 0.55).astype(np.uint8), img)
    show = cv2.resize(show, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA)
    a, b = sum(rec["inseln"]), sum(rec["loecher"])
    text = "%s I%d/%dpx L%d/%dpx%s" % (rec["id"][:-4], len(rec["inseln"]), a, len(rec["loecher"]), b, "" if rec["haupt"] else " ohne")
    cv2.putText(show, text, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(show, text, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    return show


def sheet(rows: list[list[dict]], paths: dict[str, str], out: str) -> None:
    grid = []
    for r in rows:
        tiles = [tile(x, paths[x["id"]]) for x in r]
        tiles += [np.zeros((TILE_H, TILE_W, 3), np.uint8)] * (len(rows[0]) - len(tiles))
        grid.append(np.hstack(tiles))
    cv2.imwrite(out, np.vstack(grid), [cv2.IMWRITE_JPEG_QUALITY, 88])


def report(recs: list[dict]) -> str:
    """Bericht als Text: gesamt und je Fahrt."""
    def block(name: str, rs: list[dict]) -> list[str]:
        n = max(len(rs), 1)
        cnt = np.array([len(r["inseln"]) for r in rs] or [0])
        hol = np.array([len(r["loecher"]) for r in rs] or [0])
        return ["%-10s %6d Bilder | mit Insel %5.1f %% | Inseln je Bild Median %d p90 %d max %d | mit Loch %5.1f %% | Löcher je Bild Median %d p90 %d max %d | ohne Hauptfläche %d" % (
            name, len(rs), 100 * (cnt > 0).sum() / n, np.median(cnt), np.percentile(cnt, 90), cnt.max(),
            100 * (hol > 0).sum() / n, np.median(hol), np.percentile(hol, 90), hol.max(), sum(not r["haupt"] for r in rs))]

    out = block("gesamt", recs)
    for what, key in (("Inseln", "inseln"), ("Löcher", "loecher")):
        c = np.zeros(4, int)
        px = np.zeros(4, np.int64)
        for r in recs:
            for a in r[key]:
                c[area_class(a)] += 1
                px[area_class(a)] += a
        out.append("%s nach Fläche: " % what + ", ".join("%s px: %d (%d px gesamt)" % (AREA_NAMES[i], c[i], px[i]) for i in range(4)))
        biggest = np.array([max(r[key], default=0) for r in recs] or [0])
        out.append("  Bilder nach größter Fläche: " + ", ".join("%s px: %d" % (AREA_NAMES[i], sum(area_class(b) == i and b > 0 for b in biggest)) for i in range(4)))
    ue = sum(p == "ueber" for r in recs for p in r["lage"])
    un = sum(p == "unter" for r in recs for p in r["lage"])
    out.append("Lage der Inseln: über der Grenze %d, unter der Grenze %d" % (ue, un))
    out.append("Je Fahrt:")
    for run in sorted({r["id"].split("/")[0] for r in recs}):
        out += block(run, [r for r in recs if r["id"].startswith(run + "/")])
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", help="Verzeichnisse mit <fahrt>/<bild>.png")
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=4)
    a = ap.parse_args()
    cv2.setNumThreads(1)
    paths = {}
    for d in a.dirs:
        for f in glob.glob(os.path.join(d, "*", "*.png")):
            run, name = f.replace("\\", "/").split("/")[-2:]
            paths["%s/%s.jpg" % (run, name[:-4])] = f
    with Pool(a.jobs) as pool:
        res = pool.map(one, sorted(paths.items()), chunksize=32)
    recs = [r for r in res if r is not None]
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "bilder.jsonl"), "w", encoding="utf-8") as fo:
        for r in recs:
            fo.write(json.dumps(r) + "\n")
    text = "Masken %d, nicht lesbar %d\n" % (len(recs), len(res) - len(recs)) + report(recs)
    with open(os.path.join(a.out, "bericht.txt"), "w", encoding="utf-8") as fo:
        fo.write(text + "\n")
    print(text)
    rng = random.Random(1)
    # Zufällig je Klasse (größte Insel): zeigt den Normalfall, nicht nur die Extreme
    klassen = [[r for r in recs if not r["inseln"]]] + [[r for r in recs if r["inseln"] and area_class(max(r["inseln"])) == i] for i in range(4)]
    sheet([rng.sample(k, min(4, len(k))) for k in klassen], paths, os.path.join(a.out, "tafel_zufall.jpg"))
    for key, name in (("inseln", "tafel_inseln.jpg"), ("loecher", "tafel_loecher.jpg")):
        top = sorted(recs, key=lambda r: -sum(r[key]))[:20]
        sheet([top[i:i + 4] for i in range(0, len(top), 4)], paths, os.path.join(a.out, name))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
