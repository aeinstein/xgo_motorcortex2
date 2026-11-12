#!/usr/bin/env python3
"""FloorNet: Kante über die Zeit beruhigen (xgonav.floornet.ZeilenGlaetter), gemessen auf Prüffahrten.

  python tools/floornet/glaetten_eval.py lauf/ --labels stage3_pruef/ --fahrten explore10,explore11,explore12 \\
      [--n 3,5] [--out ~/floornet/netz --name lauf_stage3]

Das Netz rechnet über alle Bilder der Label-Dateien in ihrer Reihenfolge (jedes zweite Kamerabild, wie am Rider),
einmal roh und je --n einmal geglättet. Pose, Nicken und Zeit je Bild stehen in den Label-Dateien. Ausgabe:
  - Abweichung von den Labels wie eval_labels.py (train.band_report, unter 1 m fälschlich frei/blockiert): das
    Glätten darf die Kante nicht schlechter machen;
  - Sprünge: Abstand der Kante zum vorigen Bild je Spalte, nachdem das vorige auf das aktuelle umgerechnet wurde
    (Median, 90-%-Wert, Anteil über --sprung Zeilen). Das soll kleiner werden;
  - mit --out je Fassung ein Lauf für netz_betrachter.html (<name>_roh, <name>_glatt3, ...).
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
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

MAX_DT = 0.5


def geo_aus(geom: dict, cam_fwd_mm: float):
    """CamGeometry mit den Werten der Label-Datei (Brennweite über den Öffnungswinkel, Kamerahöhe fest)."""
    from xgonav.geometry import CamGeometry
    hfov = float(np.degrees(2 * np.arctan((geom["W"] / 2) / geom["f"])))
    return CamGeometry(geom["W"], geom["H"], hfov, tilt_deg=geom["tilt_deg"], mount_offset_mm=0.0,
                       body_height_mm=geom["cam_h"] * 1000.0, cam_fwd_mm=cam_fwd_mm)


def glatt(rows: np.ndarray, posen: np.ndarray, pitch: np.ndarray, t: np.ndarray, geo, n: int) -> np.ndarray:
    """Alle Bilder einer Fahrt der Reihe nach durch den Glätter."""
    from xgonav.floornet import ZeilenGlaetter
    g = ZeilenGlaetter(n)
    return np.stack([g.glaetten(rows[i], tuple(posen[i]), float(pitch[i]), float(t[i]), geo) for i in range(len(rows))])


def spruenge(rows: np.ndarray, posen: np.ndarray, pitch: np.ndarray, t: np.ndarray, geo) -> np.ndarray:
    """|Zeile - umgerechnete Zeile des vorigen Bildes| je Spalte, alle Bildpaare mit höchstens MAX_DT Abstand."""
    from xgonav.floornet import ZeilenGlaetter
    out = []
    for i in range(1, len(rows)):
        if 0 <= t[i] - t[i - 1] <= MAX_DT:
            um = ZeilenGlaetter.umrechnen(rows[i - 1], posen[i - 1], pitch[i - 1], posen[i], pitch[i], geo)
            d = np.abs(rows[i] - um)
            out.append(d[np.isfinite(d)])
    return np.concatenate(out) if out else np.zeros(0, np.float32)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lauf", help="Verzeichnis mit floornet.pt")
    ap.add_argument("--labels", required=True, help="Stage3-Label-Dateien der Prüffahrten (stage3_npz.py)")
    ap.add_argument("--fahrten", required=True, help="mit Komma getrennt")
    ap.add_argument("--n", default="3,5", help="Fensterlängen des Glätters, mit Komma getrennt")
    ap.add_argument("--zacken", type=float, default=25.0, help="net_outlier_px wie am Rider, 0 = aus")
    ap.add_argument("--sprung", type=float, default=20.0, help="ab so vielen Zeilen zählt ein Sprung")
    ap.add_argument("--cam_fwd_mm", type=float, default=None, help="Default: camera.cam_fwd_mm aus xgonav/config.py")
    ap.add_argument("--cache", default=os.path.expanduser("~/floornet/cache"))
    ap.add_argument("--norm", type=int, default=0)
    ap.add_argument("--out", default="", help="Datenverzeichnis von netz_betrachter.html")
    ap.add_argument("--name", default="", help="Name im Betrachter, Default Verzeichnisname des Laufs")
    ap.add_argument("--ansicht", default="", help="Ansicht des Betrachters (z. B. p11): zeigt daneben das ComfyUI-Grünbild")
    a = ap.parse_args()

    import torch

    import eval_labels as E
    import netz_js
    import train as T
    from net import decode, load_net, prep
    from xgonav.config import Config

    fwd = Config().camera.cam_fwd_mm if a.cam_fwd_mm is None else a.cam_fwd_mm
    ns = [int(v) for v in a.n.split(",") if v]
    name = a.name or os.path.basename(os.path.normpath(a.lauf))
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    net = load_net(a.lauf, dev).eval()
    fassungen = ["roh"] + ["glatt%d" % n for n in ns]
    res = {f: {} for f in fassungen}
    nah = {f: np.zeros(3) for f in fassungen}
    spr = {f: [] for f in fassungen}
    with Pool(12) as pool:
        runs = [T.load_run(os.path.join(a.labels, r + ".npz"), a.cache, 0.0, 0.0, pool) for r in a.fahrten.split(",") if r]
    for r in runs:
        d = np.load(os.path.join(a.labels, r["run"] + ".npz"), allow_pickle=False)
        wo = {str(f): i for i, f in enumerate(d["files"])}
        idx = np.array([wo[str(f)] for f in r["files"]])
        posen = np.stack([d["x"][idx], d["y"][idx], d["th"][idx]], axis=1).astype(float)
        t = d["t"][idx].astype(float)
        geo = geo_aus(r["geom"], fwd)
        with torch.no_grad():
            teile = []
            for i in range(0, len(r["imgs"]), 256):
                x = torch.as_tensor(r["imgs"][i:i + 256][:, T.MARGIN:], device=dev).permute(0, 3, 1, 2).float() / 255.0
                teile.append(decode(net(prep(x, bool(a.norm))), a.zacken).cpu().numpy())
        zeilen = {"roh": np.concatenate(teile)}
        for n in ns:
            zeilen["glatt%d" % n] = glatt(zeilen["roh"], posen, r["pitch"], t, geo, n)
        for f in fassungen:
            res[f][r["run"]] = {"netz": T.band_report(zeilen[f], r["row"], r["kind"], r["pitch"], r["geom"]), "kanten": {}}
            nah[f] += E.near_counts(zeilen[f], r["row"], r["kind"], r["pitch"], r["geom"], T.EXACT, T.FAR_M)
            spr[f].append(spruenge(zeilen[f], posen, r["pitch"], t, geo))
            if a.out:
                ziel = os.path.join(a.out, name + "_" + f)
                os.makedirs(ziel, exist_ok=True)
                daten = {"lauf": name + "_" + f, "fahrt": r["run"], "files": [str(v) for v in r["files"]],
                         "rows": [[round(float(v), 1) for v in z] for z in zeilen[f]]}
                if a.ansicht:
                    daten["ansicht"] = a.ansicht
                with open(os.path.join(ziel, r["run"] + ".js"), "w", encoding="utf-8") as fh:
                    fh.write("window.NETZ = " + json.dumps(daten, separators=(",", ":")) + ";\n")
        print("%-10s %5d Bilder" % (r["run"], len(r["imgs"])), flush=True)
    for f in fassungen:
        print("\n== %s ==" % f)
        print("\n".join(z for z in T.show(res[f]).split("\n") if " kanten " not in z))
        n, fr, bl = nah[f]
        s = np.concatenate(spr[f])
        print("unter %.1f m: Spalten %d, fälschlich frei %.2f %%, fälschlich blockiert %.2f %%" % (E.NEAR_M, n, 100 * fr / n, 100 * bl / n))
        print("Sprünge zum vorigen Bild: Median %.1f, 90 %% %.1f, 99 %% %.1f Zeilen, über %.0f Zeilen %.2f %% (%d Spaltenpaare)" % (
            np.median(s), np.percentile(s, 90), np.percentile(s, 99), a.sprung, 100 * (s > a.sprung).mean(), len(s)))
    if a.out:
        netz_js.index_schreiben(a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
