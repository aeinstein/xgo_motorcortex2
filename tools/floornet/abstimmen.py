#!/usr/bin/env python3
"""Teppich an/aus stabil machen: Qwen entscheidet je Bild neu, ob Teppich oder Matte Boden sind. Hier entscheidet je
Stelle des Bodens die Mehrheit der Bilder einer Fahrt (Nutzer 2026-11-19: probieren).

Je Bild:
  1. MoGe-2-Punktwolke, Bodenebene wie geo_label.py; Pixel mit |h| <= --band_m liegen in der Ebene (Boden, Teppich,
     Matte). Möbel, Wände, Kanten fallen heraus und stimmen nie ab.
  2. Diese Pixel bis --weit_m mit Pose (meta.jsonl, x/y m, Kurs Grad) in eine Bodenkarte der Fahrt (--zelle_m).
     Je Zelle eine Stimme des Bildes: grün oder nicht (Mehrheit seiner Pixel in der Zelle, Maske aus Stage2).
  3. Abstimmung je Zelle über alle Bilder im Zeitfenster ±--fenster_s (Drift der Pose bleibt klein).
  4. Neue Maske: in der Ebene gilt die Mehrheit, wenn mindestens --min_bilder Bilder die Zelle gesehen haben und die
     Mehrheit deutlich ist (Anteil <= 1 − --klar oder >= --klar); sonst und außerhalb der Ebene bleibt Qwen.
Ausgabe (nur neue Verzeichnisse): <out>/<fahrt>/<bild>.png (Maske 0/255 wie Stage2), <out>/rows.jsonl je Bild
{"id", "rows" (Stage3-Regel), "rows_qwen", "gekippt" (Anteil geänderter Ebenen-Pixel), "art"}, <out>/bilder/ Ansicht.

  python abstimmen.py --runs 20261112-124502 --stage2 ~/floornet/stage2_p11 --out ~/floornet/abstimm
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import comfy_label as C  # noqa: E402
import geo_label as G  # noqa: E402
import nick_bezug as NB  # noqa: E402
import stage3 as S3  # noqa: E402


def boden_koord(P: np.ndarray, a: float, cam_h: float, f: float, R: G.Regeln, band_m: float, weit_m: float):
    """Punktwolke -> (Maske in der Ebene, vorwärts D, links L in m) oder None ohne eigene Ebene."""
    H, W = P.shape[:2]
    ok = np.isfinite(P).all(-1) & (P[..., 2] > 0)
    eb = G.ebene(P, ok, G.unten(a), cam_h, H / 2 + f * math.tan(a) + 15, R, None)
    if eb["art"] != "ebene":
        return None
    n, d, s = eb["n"], eb["d"], eb["s"]
    Pz = np.where(ok[..., None], P, 0.0)
    h = s * (d - Pz @ n)
    fw = np.array([0.0, 0.0, 1.0]) - n[2] * n
    fw /= np.linalg.norm(fw)
    li = np.cross(fw, n)                       # Kamera x rechts, y unten, z vorn: fw × unten = links
    D = s * (Pz @ fw)
    L = s * (Pz @ li)
    eben = ok & (np.abs(h) <= band_m) & (D > 0.05) & (D <= weit_m)
    return eben, D, L


def welt(D, L, pose, vz=1):
    x, y, gier = pose[0], pose[1], math.radians(vz * pose[2])
    c, s = math.cos(gier), math.sin(gier)
    return x + D * c - L * s, y + D * s + L * c


class Karte:
    """Stimmen je Zelle als dünn besetzte Zähler (Schlüssel = Zellnummer)."""

    def __init__(self):
        self.ja: dict = {}
        self.alle: dict = {}

    def add(self, zellen, gruen, vz):
        for z, g in zip(zellen.tolist(), gruen.tolist()):
            self.alle[z] = self.alle.get(z, 0) + vz
            if g:
                self.ja[z] = self.ja.get(z, 0) + vz

    def anteil(self, zellen):
        a = np.array([self.alle.get(z, 0) for z in zellen.tolist()], np.float32)
        j = np.array([self.ja.get(z, 0) for z in zellen.tolist()], np.float32)
        return np.divide(j, a, out=np.full_like(a, 0.5), where=a > 0), a


def zellnummer(X, Y, zelle_m):
    i = np.floor(X / zelle_m).astype(np.int64) + (1 << 20)
    j = np.floor(Y / zelle_m).astype(np.int64) + (1 << 20)
    return i * (1 << 21) + j


def stimmen(zellen_px: np.ndarray, gruen_px: np.ndarray):
    """Pixel -> je Zelle eine Stimme (Mehrheit der Pixel)."""
    u, inv = np.unique(zellen_px, return_inverse=True)
    g = np.bincount(inv, weights=gruen_px.astype(np.float64), minlength=len(u))
    n = np.bincount(inv, minlength=len(u))
    return u, (g / n) >= 0.5, inv


def ansicht(img, maske_q, maske_n, eben, rq, rn):
    """Links Bild mit Linien (grün Qwen, magenta abgestimmt), rechts Änderungen (rot: Boden weg, blau: Boden dazu)."""
    H, W = img.shape[:2]
    xs = ((np.arange(80) + 0.5) * W / 80).astype(int)
    li = img.copy()
    for r, col in ((rq, (0, 220, 0)), (rn, (255, 0, 255))):
        cv2.polylines(li, [np.array([(int(x), int(min(y, H - 1))) for x, y in zip(xs, r)], np.int32)], False, col, 2)
    re = (img * 0.4).astype(np.uint8)
    re[maske_n & ~maske_q] = (230, 120, 30)
    re[maske_q & ~maske_n] = (40, 40, 230)
    re[eben & (maske_q == maske_n)] = (re[eben & (maske_q == maske_n)] * 0.5 + np.array([0, 90, 0])).astype(np.uint8)
    return np.hstack([li, re])


def main() -> int:
    from xgonav.config import Config
    from xgonav.geometry import CamGeometry
    import comfy_npz as Z
    import stage3_npz as SN
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--stage2", default=os.path.expanduser("~/floornet/stage2_p11"))
    ap.add_argument("--data", default=os.path.expanduser("~/floornet/data/rider/fahrt"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", default=None)
    ap.add_argument("--band_m", type=float, default=0.025)
    ap.add_argument("--weit_m", type=float, default=2.0)
    ap.add_argument("--zelle_m", type=float, default=0.03)
    ap.add_argument("--fenster_s", type=float, default=30.0)
    ap.add_argument("--min_bilder", type=int, default=4)
    ap.add_argument("--klar", type=float, default=0.65)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--gier_vz", type=int, default=1, help="Vorzeichen des Kurses aus meta.jsonl (Prüfung: Schärfe)")
    ap.add_argument("--links", type=int, default=1, help="Vorzeichen der Seitenrichtung (Prüfung: Schärfe)")
    ap.add_argument("--bilder", type=int, default=400, help="so viele Ansichten mit der größten Änderung schreiben")
    a = ap.parse_args()
    cfg = Config.load(a.config)
    R = G.Regeln()
    model = G.Moge()
    os.makedirs(a.out, exist_ok=True)
    fo = open(os.path.join(a.out, "rows.jsonl"), "a")
    for run in a.runs.split(","):
        src = os.path.join(a.data, run)
        meta = Z.load_meta(src)
        hoehe = SN.koerperhoehe(src)
        stems = sorted(f[:-4] for f in os.listdir(os.path.join(a.stage2, run)) if f.endswith(".png"))
        stems = [s for s in stems if s + ".jpg" in meta and "pose" in meta[s + ".jpg"]]
        t0 = time.time()
        bilder = []          # je Bild: t, Zellen der Ebenenpixel (je Pixel), Maske Qwen, eben
        for i in range(0, len(stems), a.batch):
            teil = stems[i:i + a.batch]
            imgs = G.lade_bilder([os.path.join(src, s + ".jpg") for s in teil])
            P = model.punkte(imgs, CamGeometry.from_cfg(cfg, hoehe).hfov_deg)
            for j, s in enumerate(teil):
                m = meta[s + ".jpg"]
                hb = m.get("height_mm") if isinstance(m.get("height_mm"), (int, float)) else hoehe
                geo = CamGeometry.from_cfg(cfg, hb)
                q = cv2.imread(os.path.join(a.stage2, run, s + ".png"), cv2.IMREAD_GRAYSCALE)
                if q is None:
                    continue
                q = cv2.resize(q, (P.shape[2], P.shape[1]), interpolation=cv2.INTER_NEAREST) > 127
                r = boden_koord(P[j], math.radians(geo.tilt_deg) + NB.pitch_dev(run, m), geo.cam_h, geo.f,
                                R, a.band_m, a.weit_m)
                if r is None:
                    bilder.append({"s": s, "t": float(m["t"]), "q": q, "eben": None})
                    continue
                eben, D, L = r
                X, Y = welt(D[eben], a.links * L[eben], m["pose"], a.gier_vz)
                u, v, inv = stimmen(zellnummer(X, Y, a.zelle_m), q[eben])
                bilder.append({"s": s, "t": float(m["t"]), "q": q, "eben": eben, "inv": inv, "u": u, "v": v})
            print("%s MoGe %d/%d  %.1f Bilder/s" % (run, min(i + a.batch, len(stems)), len(stems),
                                                   min(i + a.batch, len(stems)) / (time.time() - t0)), flush=True)
        # Abstimmung im gleitenden Zeitfenster
        karte, lo, hi = Karte(), 0, 0
        n = len(bilder)
        ergebnisse = []
        for k, b in enumerate(bilder):
            while hi < n and bilder[hi]["t"] <= b["t"] + a.fenster_s:
                if bilder[hi]["eben"] is not None:
                    karte.add(bilder[hi]["u"], bilder[hi]["v"], 1)
                hi += 1
            while bilder[lo]["t"] < b["t"] - a.fenster_s:
                if bilder[lo]["eben"] is not None:
                    karte.add(bilder[lo]["u"], bilder[lo]["v"], -1)
                lo += 1
            q = b["q"]
            neu = q.copy()
            gekippt = 0.0
            if b["eben"] is not None:
                ant, anz = karte.anteil(b["u"])
                ant, anz = ant[b["inv"]], anz[b["inv"]]
                klar = anz >= a.min_bilder
                ja = klar & (ant >= a.klar)
                nein = klar & (ant <= 1 - a.klar)
                e = neu[b["eben"]]
                e = np.where(ja, True, np.where(nein, False, e))
                neu[b["eben"]] = e
                gekippt = float((neu != q).sum()) / max(int(b["eben"].sum()), 1)
            rq = C.mask_rows(S3.from_bottom(q))
            rn = C.mask_rows(S3.from_bottom(neu))
            os.makedirs(os.path.join(a.out, run), exist_ok=True)
            cv2.imwrite(os.path.join(a.out, run, b["s"] + ".png"), neu.astype(np.uint8) * 255)
            e = {"id": "%s/%s.jpg" % (run, b["s"]), "rows": [round(float(x), 1) for x in rn],
                 "rows_qwen": [round(float(x), 1) for x in rq], "gekippt": round(gekippt, 4),
                 "art": "ebene" if b["eben"] is not None else "ersatz"}
            fo.write(json.dumps(e) + "\n")
            ergebnisse.append((gekippt, b, neu, rq, rn))
        fo.flush()
        bdir = os.path.join(a.out, "bilder")
        os.makedirs(bdir, exist_ok=True)
        for gekippt, b, neu, rq, rn in sorted(ergebnisse, key=lambda x: -x[0])[:a.bilder]:
            img = cv2.imread(os.path.join(src, b["s"] + ".jpg"))
            eben = b["eben"] if b["eben"] is not None else np.zeros_like(b["q"])
            cv2.imwrite(os.path.join(bdir, "%s__%s.jpg" % (run, b["s"])), ansicht(img, b["q"], neu, eben, rq, rn))
        # Schärfe: Anteil der Bildstimmen, die mit der Mehrheit der ganzen Fahrt übereinstimmen (Zellen ab min_bilder);
        # falsche Posen oder Vorzeichen verschmieren die Karte und senken sie
        ganz = Karte()
        for b in bilder:
            if b["eben"] is not None:
                ganz.add(b["u"], b["v"], 1)
        gut = tot = 0
        for b in bilder:
            if b["eben"] is not None:
                ant, anz = ganz.anteil(b["u"])
                k_ = anz >= a.min_bilder
                gut += int(((ant >= 0.5) == b["v"])[k_].sum())
                tot += int(k_.sum())
        print("%s Schärfe %.3f (%d Stimmen)" % (run, gut / max(tot, 1), tot), flush=True)
        g = np.array([x[0] for x in ergebnisse])
        print("%s fertig: %d Bilder, ohne Ebene %d, gekippt Median %.3f, >5 %% bei %d Bildern, %.0f s" % (
            run, n, sum(b["eben"] is None for b in bilder), float(np.median(g)) if len(g) else 0,
            int((g > 0.05).sum()), time.time() - t0), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
