#!/usr/bin/env python3
"""FloorNet: Bilder für den Prüfsatz von Hand auswählen und mit dem Werkzeug zusammen in einen Ordner legen.

  python tools/floornet/make_testset.py labels/ --out pruefsatz/ --runs explore9:80,streife1:80,unten1:130,unten2:30

Danach beschriften.html im Ordner im Browser öffnen. Die Bilder liegen gemischt: Wer früher aufhört, hat
trotzdem eine ausgewogene Auswahl. Diese Fahrten dürfen nie ins Training, sonst misst der Prüfsatz nur
Auswendiglernen. Das Ergebnis wertet eval_hand.py aus.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pick import pick  # noqa: E402

VIEW_M, VIEW_DEG = 0.10, 15.0    # zwei Bilder desselben Blicks bringen dem Prüfsatz nichts
SEED = 20261106


def choose(x, y, th, rate, n: int, rng) -> np.ndarray:
    """n Bildindizes mit deutlich verschiedenem Blick, aus ruhiger Fahrt. Bewusst unabhängig von den Labels:
    Bilder, in denen sie versagen, gehören genauso in den Prüfsatz."""
    alle = np.ones((len(x), 1), np.uint8)
    cand = pick(x, y, th, rate, alle, VIEW_M, VIEW_DEG)
    if len(cand) <= n:
        return cand
    return np.sort(rng.choice(cand, n, replace=False))


def parse_runs(text: str) -> list[tuple[str, int]]:
    out = []
    for part in text.split(","):
        name, _, n = part.partition(":")
        out.append((name, int(n)))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("labels")
    ap.add_argument("--out", required=True)
    ap.add_argument("--runs", required=True, help="Fahrt:Anzahl, mit Komma getrennt")
    a = ap.parse_args()
    rng = np.random.default_rng(SEED)
    wahl = []
    for run, n in parse_runs(a.runs):
        d = np.load(os.path.join(a.labels, run + ".npz"), allow_pickle=False)
        idx = choose(d["x"], d["y"], d["th"], d["rate"], n, rng)
        src = str(d["source_dir"])
        wahl += [{"id": "%s/%s" % (run, str(d["files"][i])), "quelle": os.path.join(src, str(d["files"][i]))} for i in idx]
        print("%-10s %4d Bilder" % (run, len(idx)))
    rng.shuffle(wahl)
    os.makedirs(os.path.join(a.out, "bilder"), exist_ok=True)
    index = []
    for k, w in enumerate(wahl):
        name = "bilder/%04d.jpg" % k
        shutil.copyfile(w["quelle"], os.path.join(a.out, name))
        index.append({"id": w["id"], "src": name})
    with open(os.path.join(a.out, "index.js"), "w", encoding="utf-8") as f:
        f.write("const BILDER = " + json.dumps(index) + ";\n")
    shutil.copyfile(Path(__file__).with_name("beschriften.html"), os.path.join(a.out, "beschriften.html"))
    print("gesamt %d, Ordner %s" % (len(index), a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
