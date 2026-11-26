#!/usr/bin/env python3
"""Stufe 3: Karten mehrerer Fahrten aufeinander legen (Drehung, Verschiebung, bei alten Fahrten Maßstab).

  python reg.py KARTEN OUT REF run1 run2 ... [--skala unten1,unten2,...]
Ergebnis OUT/trafo.json: je Fahrt {th (rad), dx, dy (m), s, score}; Punkt der Fahrt p -> s * R(th) p + d im Bezug.
OUT/konsens.npz: Summe der gelegten Karten; OUT/<run>_ueber.png: Bezug rot, Fahrt grün.
"""
import argparse
import json
import math
import os

import cv2
import numpy as np

RES = 0.02
N = 1200


def merkmale(z, f=2):
    band, bn, tn = z["band"], z["bn"], z["tn"]
    wand = band[1] + band[2] + band[3]
    alle = bn + tn
    W = np.where((wand > 6) & (wand > 0.3 * alle), 1.0, 0.0).astype(np.float32)
    F = np.where((bn > 8) & (tn < 0.15 * alle), 1.0, 0.0).astype(np.float32)
    if f > 1:
        W = cv2.resize(W, None, fx=1 / f, fy=1 / f, interpolation=cv2.INTER_AREA)
        F = cv2.resize(F, None, fx=1 / f, fy=1 / f, interpolation=cv2.INTER_AREA)
    W = cv2.GaussianBlur(W, (0, 0), 1.5)
    return W, F


def warp(M, th, s=1.0, dx=0.0, dy=0.0, res=RES):
    """Karte (Ursprung Bildmitte, y = Zeile) mit p' = s R(th) p + d abbilden."""
    n = M.shape[0]
    c, si = math.cos(th) * s, math.sin(th) * s
    cx = n / 2
    A = np.array([[c, -si, cx - c * cx + si * cx + dx / res], [si, c, cx - si * cx - c * cx + dy / res]], np.float32)
    return cv2.warpAffine(M, A, (n, n), flags=cv2.INTER_LINEAR)


def korr(FA, B):
    """C[dy, dx] = Summe A(p + d) B(p): B um d verschoben gegen A."""
    return np.fft.irfft2(FA * np.conj(np.fft.rfft2(B)), s=B.shape)


def suche(Wa, Fa, Wb, Fb, winkel, skalen, res):
    FA1 = np.fft.rfft2(Wa - 0.5 * Fa)
    FA2 = np.fft.rfft2(Wa)
    n = Wa.shape[0]
    best = []
    for s in skalen:
        for th in winkel:
            wb, fb = warp(Wb, th, s, res=res), warp(Fb, th, s, res=res)
            C = korr(FA1, wb) - 0.5 * korr(FA2, fb)
            k = int(np.argmax(C))
            dy, dx = divmod(k, n)
            dy = dy - n if dy > n // 2 else dy
            dx = dx - n if dx > n // 2 else dx
            best.append((float(C.flat[k]), th, s, dx * res, dy * res))
    best.sort(reverse=True)
    return best


def lege(ref, z, mit_skala, recht=False):
    Wa, Fa = ref
    Wb, Fb = merkmale(z, 2)
    wk = np.radians([k * 90 + d for k in range(4) for d in (-3, -1.5, 0, 1.5, 3)]) if recht else np.radians(np.arange(0, 360, 1.5))
    grob = suche(Wa, Fa, Wb, Fb, wk, np.arange(0.85, 1.151, 0.03) if mit_skala else [1.0], RES * 2)
    sc, th, s, dx, dy = grob[0]
    zweit = next((g for g in grob if abs((g[1] - th + math.pi) % (2 * math.pi) - math.pi) > math.radians(8) or abs(g[2] - s) > 0.05), grob[-1])
    fein = suche(Wa, Fa, Wb, Fb, th + np.radians(np.arange(-1.5, 1.51, 0.25)),
                 s + np.arange(-0.02, 0.021, 0.01) if mit_skala else [1.0], RES * 2)
    sc, th, s, dx, dy = fein[0]
    eigen = float((Wb * Wb).sum())
    return {"th": float(th), "s": float(s), "dx": float(dx), "dy": float(dy), "score": sc, "zweit": zweit[0], "anteil": sc / max(eigen, 1e-6)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("karten")
    ap.add_argument("out")
    ap.add_argument("ref")
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--skala", default="")
    ap.add_argument("--runden", type=int, default=2)
    ap.add_argument("--recht", action="store_true", help="Karten sind schon auf die Hausachsen gedreht: nur Vielfache von 90° (±3°)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    mit_s = set(a.skala.split(",")) if a.skala else set()
    Z = {r: np.load(os.path.join(a.karten, r + ".npz")) for r in [a.ref] + a.runs}
    trafo = {a.ref: {"th": 0.0, "s": 1.0, "dx": 0.0, "dy": 0.0, "score": 0.0, "zweit": 0.0, "anteil": 1.0}}
    ref = merkmale(Z[a.ref], 2)
    for runde in range(a.runden):
        for r in a.runs:
            t = lege(ref, Z[r], r in mit_s, a.recht)
            trafo[r] = t
            print("Runde %d %-18s th %7.2f°  s %.3f  d (%.2f, %.2f)  Deckung %.2f  zweitbeste %.2f" % (
                runde, r, math.degrees(t["th"]) % 360, t["s"], t["dx"], t["dy"], t["anteil"], t["zweit"] / max(t["score"], 1e-6)), flush=True)
        # Konsens aus allen gut gelegten Fahrten als neuer Bezug
        gut = [r for r in trafo if trafo[r]["anteil"] > 0.25]
        Ws = [warp(merkmale(Z[r], 2)[0], trafo[r]["th"], trafo[r]["s"], trafo[r]["dx"], trafo[r]["dy"], RES * 2) for r in gut]
        Fs = [warp(merkmale(Z[r], 2)[1], trafo[r]["th"], trafo[r]["s"], trafo[r]["dx"], trafo[r]["dy"], RES * 2) for r in gut]
        Wk = np.clip(np.sum(Ws, 0) / max(2.0, len(gut) * 0.3), 0, 1).astype(np.float32)
        Fk = np.clip(np.sum(Fs, 0) / max(2.0, len(gut) * 0.3), 0, 1).astype(np.float32)
        ref = (Wk, Fk)
    json.dump(trafo, open(os.path.join(a.out, "trafo.json"), "w"), indent=1)
    np.savez_compressed(os.path.join(a.out, "konsens.npz"), W=ref[0], F=ref[1])
    Wr = merkmale(Z[a.ref], 2)[0]
    for r in a.runs:
        t = trafo[r]
        wb = warp(merkmale(Z[r], 2)[0], t["th"], t["s"], t["dx"], t["dy"], RES * 2)
        im = np.zeros(wb.shape + (3,), np.uint8)
        im[..., 2] = np.clip(ref[0] * 255, 0, 255)
        im[..., 1] = np.clip(wb * 400, 0, 255)
        cv2.imwrite(os.path.join(a.out, r + "_ueber.png"), im[100:500, 100:500][::-1])
    k = np.zeros(ref[0].shape + (3,), np.uint8)
    k[..., 1] = np.clip(ref[1] * 120, 0, 255)
    k[..., 2] = np.clip(ref[0] * 255, 0, 255)
    k[..., 0] = k[..., 2]
    cv2.imwrite(os.path.join(a.out, "konsens.png"), cv2.resize(k[100:500, 100:500][::-1], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))


if __name__ == "__main__":
    main()
