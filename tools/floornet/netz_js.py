#!/usr/bin/env python3
"""FloorNet: Netzergebnis je Bild als Daten für den Betrachter im Browser (netz_betrachter.html).

  python tools/floornet/netz_js.py lauf/ --labels stage3_pruef/ --fahrten explore10,explore11,explore12 \\
      --out ~/floornet/netz [--name lauf_stage3]

Schreibt out/<name>/<fahrt>.js (Bildnamen und Grenzzeile je Spalte, Zeilen im 640x480-Bild) und out/index.js (alle
Läufe und Fahrten, die in out liegen). Gezeichnet wird im Browser. Die Label-Dateien liefern nur die Bildliste und
die Nickwinkel; alle Bilder der Fahrt, nichts ausgedünnt.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def index_schreiben(out: str) -> None:
    laeufe = {}
    for name in sorted(os.listdir(out)):
        d = os.path.join(out, name)
        if os.path.isdir(d):
            fahrten = sorted(f[:-3] for f in os.listdir(d) if f.endswith(".js"))
            if fahrten:
                laeufe[name] = fahrten
    with open(os.path.join(out, "index.js"), "w", encoding="utf-8") as f:
        f.write("window.NETZE = " + json.dumps(laeufe) + ";\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lauf", help="Verzeichnis mit floornet.pt")
    ap.add_argument("--labels", required=True, help="Label-Dateien der Fahrten (stage3_npz.py)")
    ap.add_argument("--fahrten", required=True, help="mit Komma getrennt")
    ap.add_argument("--out", required=True, help="Datenverzeichnis des Betrachters")
    ap.add_argument("--name", default="", help="Name des Laufs im Betrachter, Default Verzeichnisname")
    ap.add_argument("--cache", default=os.path.expanduser("~/floornet/cache"))
    ap.add_argument("--norm", type=int, default=0)
    a = ap.parse_args()

    import torch

    import train as T
    from net import load_net

    name = a.name or os.path.basename(os.path.normpath(a.lauf))
    ziel = os.path.join(a.out, name)
    os.makedirs(ziel, exist_ok=True)
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    net = load_net(a.lauf, dev)
    with Pool(12) as pool:
        runs = [T.load_run(os.path.join(a.labels, r + ".npz"), a.cache, 0.0, 0.0, pool) for r in a.fahrten.split(",") if r]
    for r in runs:
        pr = T.predict(net, r, dev, bool(a.norm))
        rows = [[None if not np.isfinite(v) else round(float(v), 1) for v in zeile] for zeile in pr]
        daten = {"lauf": name, "fahrt": r["run"], "files": [str(f) for f in r["files"]], "rows": rows}
        with open(os.path.join(ziel, r["run"] + ".js"), "w", encoding="utf-8") as f:
            f.write("window.NETZ = " + json.dumps(daten, separators=(",", ":")) + ";\n")
        print("%-10s %5d Bilder" % (r["run"], len(rows)), flush=True)
    index_schreiben(a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
