#!/usr/bin/env python3
"""FloorNet: Netz und Kantenverfahren gegen den Prüfsatz von Hand (beschriften.html). Das ist die Entscheidungsgrundlage.

  python tools/floornet/eval_hand.py pruefsatz_beschriftung.json --labels labels/ --lauf lauf2/ [--norm 1]
  python tools/floornet/eval_hand.py hand.json --labels labels/ --lauf a/:streife1,unten2 b/:explore9,unten1 --skip erste.json
  python tools/floornet/eval_hand.py hand.json --labels labels_u3/ --lauf lauf_end/ --hoehe pruefsatz3/hoehe.json

Mehrere Läufe: Jede Fahrt wird nur mit dem Netz gemessen, das sie nicht im Training hatte. --skip lässt Bilder aus,
die schon zur Fehlersuche dienten. --median 3 glättet die Vorhersage über drei Spalten. --minimum 1 wertet statt
des Netzes je Spalte die nähere Grenze von Netz und Kanten aus (Plan, Schritt 7). --hoehe wertet je Gruppe aus
(Gruppenname = Körperhöhe in mm, andere Gruppen fallen weg) und rechnet mit der Kamerahöhe dieser Körperhöhe; die
Labels nehmen BASE_BODY_MM an (2026-11-07: Höhe während der Fahrt im Web verstellt).

Je Spalte der Fehler in Metern, Bänder nach der Entfernung der Handlinie. Fälschlich frei: Vorhersage mehr als
15 cm hinter der Linie. Fälschlich blockiert: mehr als 15 cm davor. Der Unterschied wird je Bild ausgezählt und
per Bootstrap über Bilder abgesichert: Spalten eines Bildes hängen zusammen und zählen nicht einzeln.

Ziel, vorab festgelegt (Plan, Schritt 3): fälschlich frei unter 1 m höchstens halb so oft wie Kanten,
fälschlich blockiert nicht schlechter. Sonst bleibt das Kantenverfahren.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

NCOLS = 80
W, H = 640, 480
NEAR_M = 1.0                # Band für die Entscheidung
GROUPS = {"oben": ("explore9", "streife1"), "unten": ("unten1", "unten2")}
BASE_BODY_MM = 85.0         # Körperhöhe, mit der label.py die Geometrie baut (behavior.height_cruise_mm)


def body_heights(groups: dict) -> tuple[dict, set]:
    """hoehe.json: Gruppe -> Bild-IDs. Rückgabe: ID -> Körperhöhe (mm) für Gruppen mit Zahl als Namen,
    dazu die IDs aller übrigen Gruppen (Höhe unklar, zählen nicht)."""
    body, drop = {}, set()
    for g, ids in groups.items():
        try:
            mm = float(g)
        except ValueError:
            drop.update(ids)
            continue
        body.update({i: mm for i in ids})
    return body, drop


def cam_heights(cam_h: float, ids: list, body: dict) -> np.ndarray:
    """Kamerahöhe je Bild (m): die der Labels, verschoben um die Abweichung der Körperhöhe."""
    return np.array([cam_h + (body[i] - BASE_BODY_MM) / 1000.0 if i in body else cam_h for i in ids])


def polyline_rows(pts: list, ncols: int = NCOLS, width: int = W) -> tuple[np.ndarray, np.ndarray]:
    """Linie des Werkzeugs -> Zeile je Spalte und ob die Spalte zählt (nicht unsicher). Punkte [x, y] oder
    [x, y, 1]; die 1 markiert den Abschnitt rechts vom Punkt als unsicher."""
    xs = (np.arange(ncols) + 0.5) * width / ncols
    if len(pts) < 2:
        return np.full(ncols, np.nan, np.float32), np.zeros(ncols, bool)
    px = np.array([p[0] for p in pts], float)
    py = np.array([p[1] for p in pts], float)
    rows = np.interp(xs, px, py).astype(np.float32)
    ok = np.ones(ncols, bool)
    for i in range(len(pts) - 1):
        if len(pts[i]) > 2 and pts[i][2]:
            ok[(xs >= px[i]) & (xs <= px[i + 1])] = False
    return rows, ok


def load_hand(path: str) -> dict:
    """Fertige, brauchbare Einträge: id -> (Zeilen, gültig)."""
    d = json.load(open(path, encoding="utf-8"))
    out = {}
    for k, e in d["bilder"].items():
        if e.get("fertig") and not e.get("unbrauchbar"):
            out[k] = polyline_rows(e["pts"])
    return out


def rates(err: np.ndarray, m: np.ndarray, far: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Je Bild: Anzahl gültiger Spalten, fälschlich frei, fälschlich blockiert. err und m: N x Spalten."""
    with np.errstate(invalid="ignore"):
        return m.sum(1).astype(float), ((err > far) & m).sum(1).astype(float), ((err < -far) & m).sum(1).astype(float)


def ratio_ci(a_num, b_num, den_a, den_b, seed: int = 1, n: int = 2000) -> tuple[float, float, float]:
    """Verhältnis der Fehlerquoten (a / b) mit 95-%-Intervall, Bootstrap über Bilder."""
    rng = np.random.default_rng(seed)
    N = len(a_num)

    def one(i):
        qa, qb = a_num[i].sum() / max(den_a[i].sum(), 1.0), b_num[i].sum() / max(den_b[i].sum(), 1.0)
        return qa / max(qb, 1e-6)

    boot = [one(rng.integers(0, N, N)) for _ in range(n)]
    return float(one(np.arange(N))), float(np.percentile(boot, 2.5)), float(np.percentile(boot, 97.5))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hand")
    ap.add_argument("--labels", required=True)
    ap.add_argument("--lauf", required=True, nargs="+", help="Verzeichnis oder Verzeichnis:Fahrt,Fahrt")
    ap.add_argument("--skip", default="", help="Beschriftung, deren Bilder hier nicht zählen")
    ap.add_argument("--median", type=int, default=0, help="3: Vorhersage über drei Spalten glätten")
    ap.add_argument("--minimum", type=int, default=0, help="1: je Spalte die nähere Grenze von Netz und Kanten")
    ap.add_argument("--norm", type=int, default=0)
    ap.add_argument("--far", type=float, default=0.15)
    ap.add_argument("--hoehe", default="", help="JSON Gruppe -> Bild-IDs (hoehe.json des Prüfsatzes)")
    ap.add_argument("--pred_shift", type=float, default=0.0, help="Vorhersage des Netzes so viele px näher (nach unten) legen")
    a = ap.parse_args()

    import cv2
    import torch

    import train as T
    from net import CROP_H, CROP_Y, IN_H, IN_W, FloorNet, decode, prep

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    nets = {}
    for spec in a.lauf:
        path, _, runs = spec.partition(":")
        net = FloorNet().to(dev)
        net.load_state_dict(torch.load(os.path.join(path, "floornet.pt"), map_location=dev))
        net.eval()
        for r in runs.split(",") if runs else ("*",):
            nets[r] = net

    hand = load_hand(a.hand)
    if a.skip:
        for k in load_hand(a.skip):
            hand.pop(k, None)
    hand = {k: v for k, v in hand.items() if k.split("/", 1)[0] in nets or "*" in nets}
    groups, body = {}, {}
    if a.hoehe:
        groups = json.load(open(a.hoehe, encoding="utf-8"))
        body, drop = body_heights(groups)
        n0 = len(hand)
        hand = {k: v for k, v in hand.items() if k not in drop}
        print("Höhe unklar, ausgelassen:", n0 - len(hand))
    print("Bilder von Hand:", len(hand))
    res, ids_of = {}, {}
    by_run = {}
    for k in hand:
        run, f = k.split("/", 1)
        by_run.setdefault(run, []).append(f)
    for run, files in by_run.items():
        d = np.load(os.path.join(a.labels, run + ".npz"), allow_pickle=False)
        pos = {str(f): i for i, f in enumerate(d["files"])}
        idx = np.array([pos[f] for f in files])
        src, geo = str(d["source_dir"]), json.loads(str(d["geom"]))
        x = np.stack([cv2.resize(cv2.imread(os.path.join(src, f))[CROP_Y:CROP_Y + CROP_H], (IN_W, IN_H), interpolation=cv2.INTER_AREA)
                      for f in files])
        with torch.no_grad():
            xt = torch.as_tensor(x, device=dev).permute(0, 3, 1, 2).float() / 255.0
            pr = decode(nets.get(run, nets.get("*"))(prep(xt, bool(a.norm)))).cpu().numpy() + a.pred_shift
        if a.median == 3:
            pr[:, 1:-1] = np.median(np.stack([pr[:, :-2], pr[:, 1:-1], pr[:, 2:]]), axis=0)
        kidx = np.linspace(0, d["kanten_row"].shape[1] - 1, NCOLS).round().astype(int)
        kr = d["kanten_row"][idx][:, kidx]
        if a.minimum:
            pr = np.fmax(pr, kr)    # größere Zeile = näher; unbekannte Kanten (NaN) zählen nicht
        rows = np.stack([hand[run + "/" + f][0] for f in files])
        ok = np.stack([hand[run + "/" + f][1] for f in files])
        pitch = d["pitch"][idx]
        ids = [run + "/" + f for f in files]
        geo = dict(geo, cam_h=cam_heights(geo["cam_h"], ids, body)[:, None])
        res[run] = {"netz": pr, "kanten": kr, "hand": rows, "ok": ok, "dist": T.row_dist(geo, rows, pitch),
                    "dnetz": T.row_dist(geo, pr, pitch), "dkanten": T.row_dist(geo, kr, pitch)}
        ids_of[run] = ids

    def table(sel, title):
        """sel: Fahrt -> Indizes der Bilder, die zählen."""
        sel = {r: i for r, i in sel.items() if r in res and len(i)}
        if not sel:
            return
        sub = {r: {k: v[i] for k, v in res[r].items()} for r, i in sel.items()}
        runs = list(sel)
        print("\n== %s (%s), %d Bilder" % (title, ",".join(runs), sum(len(i) for i in sel.values())))
        for lo, hi in ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5)):
            line = "%3.1f-%3.1f m " % (lo, hi)
            for who, dk in (("netz", "dnetz"), ("kanten", "dkanten")):
                n = fr = bl = 0.0
                meds = []
                for r in runs:
                    q = sub[r]
                    m = q["ok"] & (q["dist"] >= lo) & (q["dist"] < hi) & np.isfinite(q[who])
                    e = q[dk] - q["dist"]
                    c, f, b = rates(e, m, a.far)
                    n, fr, bl = n + c.sum(), fr + f.sum(), bl + b.sum()
                    meds.append(np.abs(np.nan_to_num(e[m], posinf=9.0)))
                med = float(np.median(np.concatenate(meds)) * 100) if n else float("nan")
                line += "| %-6s n%6d frei %5.1f%% block %5.1f%% med %4.0f cm " % (who, n, 100 * fr / max(n, 1), 100 * bl / max(n, 1), med)
            print(line)
        # Entscheidung: Spalten mit Handlinie unter NEAR_M
        parts = {"netz": [], "kanten": []}
        for r in runs:
            q = sub[r]
            for who, dk in (("netz", "dnetz"), ("kanten", "dkanten")):
                m = q["ok"] & (q["dist"] < NEAR_M) & np.isfinite(q[who])
                parts[who].append(rates(q[dk] - q["dist"], m, a.far))
        cn, fn, bn = (np.concatenate([p[j] for p in parts["netz"]]) for j in range(3))
        ck, fk, bk = (np.concatenate([p[j] for p in parts["kanten"]]) for j in range(3))
        ff = ratio_ci(fn, fk, cn, ck)
        fb = ratio_ci(bn, bk, cn, ck)
        print("unter %.1f m: Netz frei %.1f%% block %.1f%% | Kanten frei %.1f%% block %.1f%%" % (
            NEAR_M, 100 * fn.sum() / max(cn.sum(), 1), 100 * bn.sum() / max(cn.sum(), 1), 100 * fk.sum() / max(ck.sum(), 1), 100 * bk.sum() / max(ck.sum(), 1)))
        print("unter %.1f m, Netz / Kanten: fälschlich frei %.2f [%.2f..%.2f], fälschlich blockiert %.2f [%.2f..%.2f]" % (NEAR_M, *ff, *fb))
        print("Ziel fälschlich frei <= 0,5 und fälschlich blockiert <= 1,0: %s" % ("ERREICHT" if ff[0] <= 0.5 and fb[0] <= 1.0 else "nicht erreicht"))

    def alle(runs):
        return {r: np.arange(len(res[r]["hand"])) for r in runs if r in res}

    if groups:
        for g, members in groups.items():
            members = set(members)
            table({r: np.array([j for j, k in enumerate(ids_of[r]) if k in members], int) for r in res},
                  "Körperhöhe %s mm" % g)
    else:
        for title, group in GROUPS.items():
            table(alle(group), title)
    table(alle(list(res)), "alle")
    return 0


if __name__ == "__main__":
    sys.exit(main())
