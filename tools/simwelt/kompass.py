#!/usr/bin/env python3
"""Kurs je Bild aus den Wandrichtungen (rechtwinkliges Haus): nimmt die Kursdrift der Fahrt heraus.

  python kompass.py DATA PKT AUS run [--bild]
Je Bild: Richtungen kurzer Stücke zwischen Nachbarpunkten (15 cm bis 1,2 m hoch) im Rider-Bezug, modulo 90°. Ein
Maßstabsfehler der Tiefenschätzung dreht nichts, die Richtung einer Wand bleibt. Zusammen mit dem Odometriekurs ergibt
das den Kursfehler gegen die Hausachsen; er wird als langsam wandernde Größe verfolgt (Ausreißer wie schräg stehende
Möbel fallen durch die Schranke). Danach wird die Bahn mit dem berichtigten Kurs neu aufgerechnet.
Ausgabe AUS/<run>.json: idx -> [x, y, Kurs] (Hausachsen = Koordinatenachsen, Ursprung = Start der Fahrt).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from karte import posen_aus_meta  # noqa: E402


def wrap90(a):
    return (a + 45.0) % 90.0 - 45.0


def richtung(P):
    """Hauptrichtung (Grad, modulo 90, im Rider-Bezug) und Güte aus einer Punktwolke (H, W, 3)."""
    z = P[..., 2]
    ok = np.isfinite(P).all(-1) & (z > 0.15) & (z < 1.2) & (np.hypot(P[..., 0], P[..., 1]) < 3.0)
    k = 3
    a, b = P[:, :-k, :2], P[:, k:, :2]
    d = b - a
    ln = np.hypot(d[..., 0], d[..., 1])
    m = ok[:, :-k] & ok[:, k:] & (ln > 0.008) & (ln < 0.12)
    if m.sum() < 1500:
        return None, 0.0, int(m.sum())
    ang = np.degrees(np.arctan2(d[..., 1][m], d[..., 0][m])) % 90.0
    h, _ = np.histogram(ang, bins=90, range=(0, 90))
    hs = np.convolve(np.concatenate([h[-3:], h, h[:3]]), np.ones(5) / 5, mode="same")[3:-3]
    p = int(np.argmax(hs))
    nah = np.abs(wrap90(ang - (p + 0.5))) < 5.0
    mitte = (p + 0.5) + float(np.mean(wrap90(ang[nah] - (p + 0.5))))
    return mitte % 90.0, float(nah.mean()), int(m.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("pkt")
    ap.add_argument("aus")
    ap.add_argument("run")
    ap.add_argument("--gain", type=float, default=0.25)
    ap.add_argument("--tor", type=float, default=10.0)
    ap.add_argument("--guete", type=float, default=0.45)
    ap.add_argument("--bild", action="store_true")
    a = ap.parse_args()
    od = os.path.join(a.pkt, a.run)
    liste = json.load(open(os.path.join(od, "liste.json")))
    odo = posen_aus_meta(os.path.join(a.data, a.run))
    mess = []
    for e in liste:
        if e["idx"] not in odo:
            continue
        P = np.load(os.path.join(od, "%06d.npz" % e["idx"]))["xyz"].astype(np.float32)
        r, g, n = richtung(P)
        x, y, th, seg = odo[e["idx"]]
        mess.append((e["idx"], x, y, math.degrees(th), r, g, seg))
    # Kursfehler d(t): (Kurs + d + Richtung) mod 90 = 0
    gut = [m for m in mess if m[4] is not None and m[5] >= a.guete]
    if not gut:
        print(a.run, "keine brauchbaren Richtungen")
        return
    erste = np.array([wrap90(-(m[3] + m[4])) for m in gut[:40]])
    # Startwert: häufigster Wert der ersten Bilder (kreisförmig modulo 90)
    w = np.exp(1j * np.radians(erste * 4))
    d = float(np.degrees(np.angle(w.sum())) / 4)
    out, spur, n_ok = {}, [], 0
    px = py = None
    lx = ly = None
    for idx, x, y, psi, r, g, seg in mess:
        roh = None
        if r is not None and g >= a.guete:
            roh = wrap90(-(psi + r) - d)
            if abs(roh) < a.tor:
                d += a.gain * roh
                n_ok += 1
        if px is None:
            px, py = 0.0, 0.0
        else:
            # Weg seit dem letzten Bild mit dem berichtigten Kurs neu aufrechnen (Kursfehler dreht den Weg mit)
            dx, dy = x - lx, y - ly
            c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
            px += c * dx - s * dy
            py += s * dx + c * dy
        lx, ly = x, y
        out[idx] = [px, py, math.radians(psi + d)]
        spur.append((idx, psi, d, roh if roh is not None else float("nan"), g))
    os.makedirs(a.aus, exist_ok=True)
    json.dump(out, open(os.path.join(a.aus, a.run + ".json"), "w"))
    sp = np.array(spur)
    dreh = sp[-1, 1] - sp[0, 1]
    print("%-18s Bilder %5d, Richtung brauchbar %5d, übernommen %5d | Kursfehler Anfang %6.1f° Ende %6.1f° (Spanne %.1f°), Nettodrehung %6.0f°" % (
        a.run, len(mess), len(gut), n_ok, sp[0, 2], sp[-1, 2], sp[:, 2].max() - sp[:, 2].min(), dreh), flush=True)
    if a.bild:
        import cv2
        Wd, Hd = 1400, 420
        im = np.full((Hd, Wd, 3), 255, np.uint8)
        n = len(sp)
        psi = sp[:, 1]
        lo, hi = psi.min(), psi.max()
        for k in range(-4, 5):
            yy = int(Hd / 2 - k * 10 * 4)
            cv2.line(im, (0, yy), (Wd, yy), (220, 220, 220), 1)
            cv2.putText(im, "%d" % (k * 10), (2, yy - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
        d0 = sp[0, 2]
        for i in range(n):
            xx = int(i / n * Wd)
            if not np.isnan(sp[i, 3]):
                yy = int(Hd / 2 - (sp[i, 2] + sp[i, 3] - d0) * 4)       # Rohmessung
                if 0 <= yy < Hd:
                    im[yy, xx] = (160, 160, 255)
            yy = int(Hd / 2 - (sp[i, 2] - d0) * 4)
            if 0 <= yy < Hd:
                cv2.circle(im, (xx, yy), 1, (0, 0, 200), -1)
            yy = int(Hd - 10 - (psi[i] - lo) / max(1.0, hi - lo) * (Hd - 20))    # Odometriekurs (grün, eigene Skala)
            im[yy, xx] = (0, 150, 0)
        cv2.putText(im, "%s: Kursfehler (rot, Grad, 0 = Anfang), Rohmessung (hellrot), Odometriekurs %.0f..%.0f (gruen)" % (a.run, lo, hi),
                    (40, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.imwrite(os.path.join(a.aus, a.run + "_kurs.png"), im)


if __name__ == "__main__":
    main()
