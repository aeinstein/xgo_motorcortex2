#!/usr/bin/env python3
"""FloorNet: Zeitfenster einer Aufnahme als eigenen Lauf anlegen.

  python tools/floornet/schnitt.py data/fahrt/unten4 data/fahrt/unten4b 117.0 174.0

label.py nimmt je Lauf nur das längste Stück ohne Posesprung (> JUMP_M zwischen zwei Bildern). Hat eine
Fahrt mehrere Sprünge, legt dieses Werkzeug jedes Stück als eigenen Lauf an: meta.jsonl mit den Zeilen im
Fenster, die Bilder als Verweis (Symlink, sonst harter Link). Zeiten in Sekunden ab der ersten Metazeile.
"""
from __future__ import annotations

import argparse
import json
import os
import sys


def schneide(src: str, dst: str, t_von: float, t_bis: float) -> int:
    """Legt dst an; Rückgabe: Anzahl der übernommenen Metazeilen."""
    ms = []
    for line in open(os.path.join(src, "meta.jsonl"), encoding="utf-8"):
        try:
            ms.append((json.loads(line), line))
        except ValueError:
            continue                    # letzte Zeile beim Abschalten abgeschnitten
    if not ms:
        raise SystemExit("keine Metazeilen in " + src)
    t0 = ms[0][0]["t"]
    keep = [(m, line) for m, line in ms if t_von <= m["t"] - t0 < t_bis]
    if not keep:
        raise SystemExit("keine Zeilen im Fenster %.1f-%.1f s" % (t_von, t_bis))
    os.makedirs(dst, exist_ok=False)
    with open(os.path.join(dst, "meta.jsonl"), "w", encoding="utf-8") as f:
        for _, line in keep:
            f.write(line if line.endswith("\n") else line + "\n")
    for m, _ in keep:
        q = os.path.abspath(os.path.join(src, m["file"]))
        if not os.path.exists(q):
            continue
        z = os.path.join(dst, m["file"])
        try:
            os.symlink(q, z)
        except OSError:
            os.link(q, z)               # Windows ohne Symlink-Recht
    return len(keep)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("quelle")
    ap.add_argument("ziel")
    ap.add_argument("von_s", type=float)
    ap.add_argument("bis_s", type=float)
    a = ap.parse_args()
    n = schneide(a.quelle, a.ziel, a.von_s, a.bis_s)
    print("%s: %d Zeilen" % (a.ziel, n))
    return 0


if __name__ == "__main__":
    sys.exit(main())
