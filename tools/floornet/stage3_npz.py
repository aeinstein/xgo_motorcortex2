#!/usr/bin/env python3
"""FloorNet-Daten, Stage3 -> Label-Dateien für train.py, je Fahrt eine. Filter und Aufbau wie comfy_npz.py.

  python tools/floornet/stage3_npz.py ~/floornet/stage3_p11 --config rider.toml --out ~/floornet/stage3_p11_labels

Je 8 px breitem Streifen gilt die Zeile, bis zu der mindestens die Hälfte der Pixelspalten Boden hat (comfy_label.mask_rows).
Ganz dünne senkrechte Dinge fallen dabei weg, ein Schuh bleibt. Stage3 ist je Pixelspalte lückenlos, die Lückenregel von
mask_rows greift hier also nie.

Braucht keine Dateien von label.py mehr: Die Bilder liegen unter --data/<Fahrt>, die Kamerageometrie (nur für Angaben
in Metern beim Messen und für Gewichte nach Entfernung) kommt aus --config, der Konfig des Riders; die Kamerahöhe aus
der Körperhöhe in meta.jsonl (height_mm, Median der Fahrt), fehlt sie dort, aus behavior.height_cruise_mm.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import comfy_label as C  # noqa: E402
import comfy_npz as Z  # noqa: E402
import stage2 as S2  # noqa: E402
import stage3 as S3  # noqa: E402


def rows_of(path: str) -> np.ndarray | None:
    """Maske aus Stage3 -> Zeile der Grenze je Streifen. None, wenn sie nicht lesbar ist."""
    m = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if m is None or m.shape != (C.H, C.W):
        return None
    return C.mask_rows(m > 0)


def _id(path: str) -> str:
    return "/".join(path.replace("\\", "/").split("/")[-2:])[:-4]


def collect(stage3: str, jobs: int, data: str = S2.DATA) -> tuple[dict, int, dict]:
    """Fahrt -> {Datei.jpg: Zeilen}; dazu die Zahl nicht lesbarer Masken und je Fahrt die Zahl der aussortierten
    (Standbilder, Handliste des Betrachters; Regeln aus <stage3>/info.json, siehe stage2.py)."""
    paths = sorted(glob.glob(os.path.join(stage3, "*", "*.png")))
    aus = S2.aussortiert([_id(p) for p in paths], data, **S3.aus_regeln(stage3))
    weg: dict = {}
    for i in aus:
        weg[i.split("/")[0]] = weg.get(i.split("/")[0], 0) + 1
    paths = [p for p in paths if _id(p) not in aus]
    with Pool(jobs) as pool:
        res = pool.map(rows_of, paths, chunksize=32)
    out: dict = {}
    bad = 0
    for p, r in zip(paths, res):
        if r is None:
            bad += 1
            continue
        run, name = p.replace("\\", "/").split("/")[-2:]
        out.setdefault(run, {})[name[:-4] + ".jpg"] = r
    return out, bad, weg


def koerperhoehe(src: str) -> float | None:
    """Körperhöhe der Fahrt (mm): Median von height_mm in meta.jsonl. None, wenn die Aufnahme sie nicht kennt."""
    h = [m["height_mm"] for m in Z.load_meta(src).values() if isinstance(m.get("height_mm"), (int, float))]
    return float(np.median(h)) if h else None


def geometrie(cfg, body_mm: float | None) -> str:
    """Kamerageometrie als JSON wie in den Dateien von label.py (W, H, f, cam_h, tilt_deg, hfov_deg)."""
    from xgonav.geometry import CamGeometry
    geo = CamGeometry.from_cfg(cfg, body_mm)
    return json.dumps({"W": geo.W, "H": geo.H, "f": geo.f, "cam_h": geo.cam_h, "tilt_deg": geo.tilt_deg, "hfov_deg": geo.hfov_deg})


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage3")
    ap.add_argument("--config", default=None, help="TOML des Riders für die Kamerageometrie; ohne: Standardwerte von xgonav")
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--min_run", type=int, default=1, help="Fahrten mit weniger Bildern auslassen")
    ap.add_argument("--wohnung", default="a", help="Kennung der Wohnung in den Label-Dateien (das Training nutzt sie nicht)")
    ap.add_argument("--data", default=S2.DATA, help="Rohaufnahmen, je Fahrt ein Verzeichnis")
    ap.add_argument("--fahrten", default="", help="nur diese Fahrten, mit Komma getrennt (z. B. Prüffahrten in ein eigenes Verzeichnis)")
    a = ap.parse_args()
    from xgonav.config import Config
    cfg = Config.load(a.config)
    cv2.setNumThreads(1)
    os.makedirs(a.out, exist_ok=True)
    runs, bad, weg = collect(a.stage3, a.jobs, a.data)
    only = {r for r in a.fahrten.split(",") if r}
    total = 0
    for run, frames in sorted(runs.items()):
        if only and run not in only:
            continue
        src = os.path.abspath(os.path.join(a.data, run))
        hoehe = koerperhoehe(src)
        geom = geometrie(cfg, hoehe)
        d, count = Z.build_run(run, frames, geom, src, a.wohnung)
        print("%-10s %5d von %5d Bildern, verworfen: %s, vorher aussortiert %d, Kamerahöhe %.0f mm%s" % (
            run, len(d["files"]), len(frames), count or "-", weg.get(run, 0), json.loads(geom)["cam_h"] * 1000,
            "" if hoehe is not None else " (Körperhöhe aus der Konfig)"), flush=True)
        if len(d["files"]) >= a.min_run:
            tmp = os.path.join(a.out, run + ".tmp.npz")
            np.savez_compressed(tmp, **d)
            os.replace(tmp, os.path.join(a.out, run + ".npz"))
            total += len(d["files"])
    print("gesamt %d, nicht lesbar %d" % (total, bad))
    return 0


if __name__ == "__main__":
    sys.exit(main())
