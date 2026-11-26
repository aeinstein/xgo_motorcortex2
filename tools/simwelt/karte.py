#!/usr/bin/env python3
"""Stufe 2: Punktwolken einer Fahrt über die Fahrposen zu Draufsichten zusammensetzen.

  python karte.py DATA PKT OUT unten3 [--posen posen.json] [--rmax 3.5]
Ausgabe OUT/<run>_{boden,hoehe,baender}.png und OUT/<run>.npz (Raster 2 cm, 24 m x 24 m, Ursprung Mitte).
"""
import argparse
import json
import math
import os

import cv2
import numpy as np

RES = 0.02
N = 1200
ZB = 24          # Höhenfächer je 10 cm bis 2,4 m
BAENDER = [(0.04, 0.20), (0.20, 0.60), (0.60, 1.20), (1.20, 2.40)]


def posen_aus_meta(src):
    """Je Bild-idx (x, y, Kurs rad): Lage linear zwischen den Stufen der aufgezeichneten Pose, Kurs aus yaw_deg mit dem
    Versatz des Abschnitts (die Pose der Karte hinkt als Treppe hinterher, yaw_deg kommt je Bild)."""
    ms = [json.loads(z) for z in open(os.path.join(src, "meta.jsonl"), encoding="utf-8")]
    t = np.array([m["t"] for m in ms])
    yaw = np.array([m["yaw_deg"] for m in ms], float)
    px = np.array([m["pose"][0] for m in ms], float)
    py = np.array([m["pose"][1] for m in ms], float)
    pth = np.array([m["pose"][2] for m in ms], float)
    diff = yaw - pth
    # Abschnitte: Kartenpose neu gesetzt (Sprung im Versatz oder in der Lage)
    neu = np.zeros(len(ms), bool)
    neu[0] = True
    glatt = np.convolve(diff, np.ones(9) / 9, mode="same")
    for i in range(1, len(ms)):
        if abs(px[i] - px[i - 1]) + abs(py[i] - py[i - 1]) > 0.4 or abs(glatt[min(len(ms) - 1, i + 6)] - glatt[max(0, i - 6)]) > 25 and abs(diff[i] - diff[i - 1]) > 15:
            neu[i] = True
    seg = np.cumsum(neu) - 1
    rate = np.abs(np.gradient(yaw, t))
    out = {}
    for s in range(seg.max() + 1):
        k = np.where(seg == s)[0]
        ruhig = k[rate[k] < 8.0]
        off = float(np.median(diff[ruhig if len(ruhig) > 5 else k]))
        # Stufen der Lage: nur Zeitpunkte, an denen sie sich ändert
        ch = [k[0]] + [i for i in k[1:] if px[i] != px[i - 1] or py[i] != py[i - 1]] + [k[-1]]
        ch = np.unique(ch)
        xi = np.interp(t[k], t[ch], px[ch])
        yi = np.interp(t[k], t[ch], py[ch])
        for j, i in enumerate(k):
            out[ms[i]["idx"]] = (float(xi[j]), float(yi[j]), math.radians(yaw[i] - off), s)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("pkt")
    ap.add_argument("out")
    ap.add_argument("run", help="Fahrt oder mehrere mit Komma")
    ap.add_argument("--trafo", default=None, help="trafo.json aus reg.py: Fahrten in den gemeinsamen Bezug legen")
    ap.add_argument("--name", default=None)
    ap.add_argument("--posendir", default=None, help="verfeinerte Posen aus fein.py (schon im gemeinsamen Bezug)")
    ap.add_argument("--zmin", type=float, default=0.04)
    ap.add_argument("--mit_trafo", action="store_true", help="Posen aus --posendir sind im Bezug der Fahrt: Trafo darauf anwenden")
    ap.add_argument("--posen", default=None)
    ap.add_argument("--rmax", type=float, default=3.5)
    ap.add_argument("--skala", type=float, default=1.0)
    a = ap.parse_args()
    runs = a.run.split(",")
    name = a.name or runs[0]
    trafo = json.load(open(a.trafo)) if a.trafo else {}
    os.makedirs(a.out, exist_ok=True)
    bod = np.zeros((N * N, 3), np.float64)
    bn = np.zeros(N * N, np.float64)
    band = np.zeros((len(BAENDER), N * N), np.float32)
    top = np.zeros((N * N, 3), np.float64)
    tn = np.zeros(N * N, np.float64)
    hist = np.zeros(N * N * ZB, np.float32)
    stapel = []
    bahn = []
    for run in runs:
        od = os.path.join(a.pkt, run)
        liste = json.load(open(os.path.join(od, "liste.json")))
        posen = posen_aus_meta(os.path.join(a.data, run))
        T = trafo.get(run, {"th": 0.0, "s": 1.0, "dx": 0.0, "dy": 0.0})
        ts, tc, tsn = T["s"], math.cos(T["th"]), math.sin(T["th"])
        pf = os.path.join(a.posendir, run + ".json") if a.posendir else None
        if pf and os.path.exists(pf):
            posen = {int(k): v for k, v in json.load(open(pf)).items()}
            if not a.mit_trafo:
                ts, tc, tsn, T = 1.0, 1.0, 0.0, {"dx": 0.0, "dy": 0.0, "th": 0.0}
        for e in liste:
            if e["idx"] not in posen:
                continue
            x0, y0, th = posen[e["idx"]][:3]
            x0, y0, th = ts * (tc * x0 - tsn * y0) + T["dx"], ts * (tsn * x0 + tc * y0) + T["dy"], th + T["th"]
            bahn.append((x0, y0))
            z = np.load(os.path.join(od, "%06d.npz" % e["idx"]))
            P3 = z["xyz"].astype(np.float32) * a.skala * ts
            # Mischpunkte an Tiefensprüngen weg: Entfernung weicht von einem Nachbarn um mehr als 4 % ab
            rr = np.linalg.norm(P3 - np.array([0.0, 0.0, 0.12], np.float32), axis=-1)
            kante = np.zeros(rr.shape, bool)
            with np.errstate(invalid="ignore"):
                dx = np.abs(rr[:, 1:] - rr[:, :-1]) > 0.04 * rr[:, 1:]
                dy = np.abs(rr[1:] - rr[:-1]) > 0.04 * rr[1:]
            kante[:, 1:] |= dx
            kante[:, :-1] |= dx
            kante[1:] |= dy
            kante[:-1] |= dy
            P = P3.reshape(-1, 3)
            C = z["bgr"].reshape(-1, 3)
            r = np.hypot(P[:, 0], P[:, 1])
            ok = np.isfinite(P).all(1) & (r < a.rmax * ts) & (P[:, 0] > 0.15) & ~kante.reshape(-1)
            P, C = P[ok], C[ok]
            c, s = math.cos(th), math.sin(th)
            wx = x0 + c * P[:, 0] - s * P[:, 1]
            wy = y0 + s * P[:, 0] + c * P[:, 1]
            ix = np.floor(wx / RES).astype(np.int64) + N // 2
            iy = np.floor(wy / RES).astype(np.int64) + N // 2
            g = (ix >= 0) & (ix < N) & (iy >= 0) & (iy < N)
            ix, iy, P, C = ix[g], iy[g], P[g], C[g]
            cell = iy * N + ix
            zz = P[:, 2]
            fl = np.abs(zz) < 0.03
            bn += np.bincount(cell[fl], minlength=N * N)
            for k in range(3):
                bod[:, k] += np.bincount(cell[fl], weights=C[fl, k], minlength=N * N)
            for b, (lo, hi) in enumerate(BAENDER):
                m = (zz >= lo) & (zz < hi)
                band[b] += np.bincount(cell[m], minlength=N * N)
            m = (zz >= 0.04) & (zz < 2.4)
            tn += np.bincount(cell[m], minlength=N * N)
            for k in range(3):
                top[:, k] += np.bincount(cell[m], weights=C[m, k], minlength=N * N)
            stapel.append(cell[m] * ZB + np.minimum((zz[m] / 0.1).astype(np.int64), ZB - 1))
            if len(stapel) >= 40:
                hist += np.bincount(np.concatenate(stapel), minlength=N * N * ZB)
                stapel = []
    if stapel:
        hist += np.bincount(np.concatenate(stapel), minlength=N * N * ZB)
    sh = (N, N)
    boden = (bod / np.maximum(bn, 1)[:, None]).reshape(N, N, 3).astype(np.uint8)
    boden[bn.reshape(sh) < 3] = 0
    np.savez_compressed(os.path.join(a.out, name + ".npz"), boden=boden, bn=bn.reshape(sh).astype(np.float32),
                        band=band.reshape(-1, N, N), top=(top / np.maximum(tn, 1)[:, None]).reshape(N, N, 3).astype(np.uint8),
                        tn=tn.reshape(sh).astype(np.float32), hist=hist.reshape(N, N, ZB).astype(np.uint16), bahn=np.array(bahn, np.float32))
    bild(a.out, name)


def ausschnitt(maske, rand=20):
    ys, xs = np.where(maske)
    if len(xs) == 0:
        return 0, N, 0, N
    return max(0, ys.min() - rand), min(N, ys.max() + rand), max(0, xs.min() - rand), min(N, xs.max() + rand)


def bild(out, run, gross=2):
    z = np.load(os.path.join(out, run + ".npz"))
    band, bn, tn = z["band"], z["bn"], z["tn"]
    alle = bn + tn
    y0, y1, x0, x1 = ausschnitt(alle > 20)
    anteil = tn / np.maximum(alle, 1)
    hind = (anteil > 0.35) & (tn > 15)
    img = z["boden"].copy()
    img[alle < 10] = 0
    img[hind] = z["top"][hind]
    # Bänder, je Zelle als Anteil an allen Treffern: Blau 4–20 cm, Grün 20–60 cm, Rot 60–120 cm
    bimg = np.stack([np.clip(band[k] / np.maximum(alle, 1) * 3, 0, 1) * 255 for k in range(3)], -1)
    bimg[alle < 15] = 0
    bimg = bimg.astype(np.uint8)
    w = band[2] + band[3]
    wimg = np.clip(w / max(1.0, float(np.percentile(w[w > 0], 97))) if (w > 0).any() else w, 0, 1) ** 0.5 * 255
    wimg = cv2.cvtColor(wimg.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for name, im in (("drauf", img), ("baender", bimg), ("wand", wimg)):
        im = im[y0:y1, x0:x1].copy()
        for p in z["bahn"][::3]:
            cx, cy = int(p[0] / RES) + N // 2 - x0, int(p[1] / RES) + N // 2 - y0
            if 0 <= cx < im.shape[1] and 0 <= cy < im.shape[0]:
                im[cy, cx] = (0, 0, 255)
        im = cv2.resize(im[::-1], None, fx=gross, fy=gross, interpolation=cv2.INTER_NEAREST)   # y nach oben
        for gx in range(int(math.floor((x0 - N // 2) * RES)), int((x1 - N // 2) * RES) + 1):
            c = int(((gx / RES) + N // 2 - x0) * gross)
            cv2.line(im, (c, 0), (c, im.shape[0]), (90, 90, 90), 1)
            cv2.putText(im, str(gx), (c + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        for gy in range(int(math.floor((y0 - N // 2) * RES)), int((y1 - N // 2) * RES) + 1):
            r = im.shape[0] - int(((gy / RES) + N // 2 - y0) * gross)
            cv2.line(im, (0, r), (im.shape[1], r), (90, 90, 90), 1)
            cv2.putText(im, str(gy), (2, r - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.imwrite(os.path.join(out, "%s_%s.png" % (run, name)), im)


if __name__ == "__main__":
    main()
