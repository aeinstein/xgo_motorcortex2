#!/usr/bin/env python3
"""Bildliste einer oder mehrerer Rohaufnahmen für comfy_batch.py --ids (fahrt/datei.jpg je Zeile), ohne Standbilder.

  python ids_fahrt.py 20261110-201720 [weitere ...] [--jedes 2] [--steht_m 0.03 --steht_grad 3] >> comfy/ids.txt

Erst jedes n-te Bild (Bildnummer durch n teilbar), dann fallen Standbilder weg wie in stage2.py. So kostet ein Bild,
das später ohnehin aussortiert würde, keine Rechenzeit im ComfyUI.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def main() -> int:
    import stage2 as S2

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fahrten", nargs="+")
    ap.add_argument("--data", default=S2.DATA)
    ap.add_argument("--jedes", type=int, default=2)
    ap.add_argument("--steht_m", type=float, default=S2.STEHT_M)
    ap.add_argument("--steht_grad", type=float, default=S2.STEHT_GRAD)
    a = ap.parse_args()
    for run in a.fahrten:
        stems = sorted(f[:-4] for f in os.listdir(os.path.join(a.data, run)) if f.endswith(".jpg"))
        teil = [s for s in stems if not s.isdigit() or int(s) % a.jedes == 0]
        aus = S2.stehend(S2.posen(a.data, run), teil, a.steht_m, a.steht_grad)
        for s in teil:
            if s not in aus:
                print("%s/%s.jpg" % (run, s))
        print("%s: %d Bilder, jedes %d.: %d, ohne Standbilder: %d" % (run, len(stems), a.jedes, len(teil), len(teil) - len(aus)), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
