#!/usr/bin/env python3
"""Ein Bild von ComfyUI mit neuem Seed neu malen lassen (Taste N im Netz-Betrachter, POST /neu des Betrachter-Servers).

  python tools/floornet/neu_malen.py [--dir ~/floornet] [--seed 7] [--prompt t2] p11 explore10/005842

Die alte Bearbeitung in Stage1 wird ÜBERSCHRIEBEN (Nutzer 2026-11-12: so lange neu würfeln, bis kein Gegenstand mehr
übermalt ist), ebenso die Masken in Stage2 und Stage3. Eingangskette wie comfy_batch.py --eingang auto. Jeder Lauf
steht in <stage1>/neu.jsonl ({"id", "seed", "prompt", "eingang", "zeit"}); Seed (1) und Prompt des Stapellaufs gelten
für diese Bilder nicht mehr. --prompt wählt eine Variante für die Situation im Bild: Name x -> comfy_promptx.txt oder
comfy_prompt_x.txt neben diesem Skript (11 = Standard der Ansicht, t1 = Teppichsatz, t2 = Teppich und Dinge einzeln
genannt); eine neue Variante ist eine neue Datei dort plus ein Eintrag in der Auswahl von netz_betrachter.html. Letzte Zeile der Ausgabe: JSON {"id", "seed", "rows"} (Labelzeile je 8-px-Spalte wie stage3_npz.py).
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# Ansicht -> Verzeichnisse unter --dir, Prompt-Datei neben diesem Skript, Grünschwelle von stage2.py --gruen
ANSICHT = {"p11": {"stage1": "stage1_p11", "stage2": "stage2_p11", "stage3": "stage3_p11",
                   "prompt": "comfy_prompt11.txt", "gruen": 30}}
LOG = "neu.jsonl"


def prompt_datei(name: str) -> Path:
    """Variante -> Prompt-Datei neben diesem Skript. KeyError, wenn es sie nicht gibt oder der Name nicht taugt."""
    if name.isascii() and name.isalnum() and len(name) <= 12:
        for f in ("comfy_prompt%s.txt" % name, "comfy_prompt_%s.txt" % name):
            p = Path(__file__).resolve().parent / f
            if p.exists():
                return p
    raise KeyError("unbekannte Prompt-Variante: %r" % name)


def neu(root: str, ansicht: str, bild: str, seed: int, data: str = "", variante: str = "") -> dict:
    """Bild neu malen und alle drei Stufen ersetzen. bild: fahrt/nummer ohne Endung; variante "" = Prompt der Ansicht."""
    import comfy_batch as B
    import comfy_label as C
    import cv2
    import stage2 as S2
    import stage3 as S3

    a = ANSICHT[ansicht]
    quelle = os.path.join(data or os.path.join(root, "data", "rider", "fahrt"), bild + ".jpg")
    if not os.path.exists(quelle):
        raise FileNotFoundError("Rohbild fehlt: " + quelle)
    with open(prompt_datei(variante) if variante else Path(__file__).resolve().parent / a["prompt"], encoding="utf-8") as f:
        prompt = f.read().strip()
    s1, s2, s3 = (os.path.join(root, a[k], bild + ".png") for k in ("stage1", "stage2", "stage3"))
    eingang = B.kette(quelle)[0]
    B.edit_and_keep(quelle, prompt, s1, eingang, seed)
    if S2.make_mask((s1, s2), a["gruen"]) != "neu" or S3.make_mask((s2, s3)) != "neu":
        raise RuntimeError("Maske ließ sich nicht schreiben")
    rows = [round(float(v), 1) for v in C.mask_rows(cv2.imread(s3, cv2.IMREAD_GRAYSCALE) > 0)]
    # rows steht mit im Protokoll: Der Betrachter holt daraus nach dem Neuladen die neue Labellinie und umgeht mit dem
    # Seed den Zwischenspeicher des Browsers (Stage1-Bilder gelten dort einen Tag lang als unveränderlich).
    with open(os.path.join(root, a["stage1"], LOG), "a", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps({"id": bild, "seed": seed, "prompt": variante or a["prompt"], "eingang": eingang,
                            "zeit": time.strftime("%Y-%m-%d %H:%M:%S"), "rows": rows}) + "\n")
    return {"id": bild, "seed": seed, "rows": rows}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ansicht", choices=sorted(ANSICHT))
    ap.add_argument("bild", help="fahrt/nummer, z. B. explore10/005842")
    ap.add_argument("--dir", default=os.path.expanduser("~/floornet"))
    ap.add_argument("--seed", type=int, default=0, help="0 = zufällig")
    ap.add_argument("--prompt", default="", help="Prompt-Variante (11, t1, t2, ...); ohne: Prompt der Ansicht")
    a = ap.parse_args()
    seed = a.seed or random.randrange(2, 2 ** 31)
    print(json.dumps(neu(os.path.abspath(a.dir), a.ansicht, a.bild, seed, variante=a.prompt)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
