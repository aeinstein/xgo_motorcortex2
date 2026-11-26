#!/usr/bin/env python3
"""Karte einer Fahrt auf die Hausachsen drehen und groß zeichnen (Draufsicht, Wände, Höhen), optional mit Modell.

  python plan.py KARTE.npz AUS_PREFIX [--winkel GRAD] [--modell welt.json] [--zoom 4]
Ohne --winkel: Hauptrichtung der Wände aus den Kanten der Wandkarte (modulo 90°).
Ausgabe: Koordinaten im gedrehten Bezug (Meter), Raster 0,5 m.
"""
import argparse
import json
import math

import cv2
import numpy as np

RES = 0.02


def hauptrichtung(W):
    gx = cv2.Sobel(W, cv2.CV_32F, 1, 0, ksize=5)
    gy = cv2.Sobel(W, cv2.CV_32F, 0, 1, ksize=5)
    mag = np.hypot(gx, gy)
    ang = np.degrees(np.arctan2(gy, gx)) % 90.0
    h, _ = np.histogram(ang, bins=360, range=(0, 90), weights=mag)
    h = np.convolve(np.concatenate([h[-8:], h, h[:8]]), np.ones(9) / 9, mode="same")[8:-8]
    return float((np.argmax(h) + 0.5) * 0.25)


def dreh(M, grad, interp=cv2.INTER_LINEAR):
    n = M.shape[0]
    A = cv2.getRotationMatrix2D((n / 2, n / 2), grad, 1.0)      # Zeile = y nach unten im Feld: Drehsinn siehe main
    return cv2.warpAffine(M, A, (n, n), flags=interp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("karte")
    ap.add_argument("aus")
    ap.add_argument("--winkel", type=float, default=None)
    ap.add_argument("--modell", default=None)
    ap.add_argument("--zoom", type=int, default=4)
    ap.add_argument("--ursprung", default="0,0", help="Verschiebung (m) nach dem Drehen: dieser Punkt wird (0, 0)")
    a = ap.parse_args()
    z = np.load(a.karte)
    hist = z["hist"].astype(np.float32)
    bn, tn = z["bn"], z["tn"]
    alle = bn + tn
    wand = hist[..., 2:].sum(-1)
    W = np.where((alle > 12), wand / np.maximum(alle, 1), 0).astype(np.float32)
    al = hauptrichtung(cv2.GaussianBlur(W, (0, 0), 1.5)) if a.winkel is None else a.winkel
    if al > 45:
        al -= 90
    print("Hauptrichtung der Wände: %.2f° gegen die x-Achse der Fahrt" % al)
    # Höhe der Oberkante je Zelle: höchstes Fach mit mindestens 15 % des stärksten Fachs
    mx = hist.max(-1, keepdims=True)
    gut = hist >= np.maximum(0.15 * mx, 4)
    top = np.where(gut.any(-1), (hist.shape[-1] - np.argmax(gut[..., ::-1], -1)) * 0.1, 0.0).astype(np.float32)
    unter = np.where(gut.any(-1), np.argmax(gut, -1) * 0.1, 0.0).astype(np.float32)   # Unterkante (schwebende Teile)
    hind = (tn > 0.35 * alle) & (tn > 15)
    n = W.shape[0]
    # Feld: Zeile = y, Spalte = x, beide aufsteigend. Punkt p' = R(-al) p  ->  im Feld Drehung um +al (y nach unten gezählt
    # wäre es umgekehrt; hier wächst y mit der Zeile, also mathematisch positive Drehung = cv2 mit negativem Winkel)
    A = cv2.getRotationMatrix2D((n / 2, n / 2), al, 1.0)
    ox, oy = [float(v) for v in a.ursprung.split(",")]
    A[0, 2] -= ox / RES
    A[1, 2] -= oy / RES

    def d(M, interp=cv2.INTER_LINEAR):
        return cv2.warpAffine(M, A, (n, n), flags=interp)

    boden, topc = d(z["boden"]), d(z["top"])
    bn_r, alle_r, hind_r = d(bn), d(alle), d(hind.astype(np.float32)) > 0.5
    W_r, top_r, unter_r = d(W), d(top, cv2.INTER_NEAREST), d(unter, cv2.INTER_NEAREST)
    ys, xs = np.where(alle_r > 20)
    y0, y1, x0, x1 = max(0, ys.min() - 15), min(n, ys.max() + 15), max(0, xs.min() - 15), min(n, xs.max() + 15)
    drauf = boden.copy()
    drauf[alle_r < 10] = 0
    drauf[hind_r] = topc[hind_r]
    wimg = cv2.cvtColor((np.clip((W_r - 0.25) / 0.5, 0, 1) * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    stufen = [(0.25, (255, 120, 0)), (0.55, (220, 220, 0)), (0.95, (0, 200, 0)), (1.35, (0, 220, 255)), (9.9, (0, 0, 255))]
    himg = (boden * 0.45).astype(np.uint8)
    himg[alle_r < 10] = 0
    for i, (grenze, farbe) in enumerate(stufen):
        lo = 0.0 if i == 0 else stufen[i - 1][0]
        m = hind_r & (top_r > lo) & (top_r <= grenze)
        himg[m] = farbe
    schweb = hind_r & (unter_r >= 0.3)
    himg[schweb] = (himg[schweb] * 0.5 + np.array([255, 0, 255]) * 0.5).astype(np.uint8)
    modell = json.load(open(a.modell)) if a.modell else None
    zo = a.zoom
    for name, im in (("drauf", drauf), ("wand", wimg), ("hoehe", himg)):
        im = cv2.resize(im[y0:y1, x0:x1][::-1], None, fx=zo, fy=zo, interpolation=cv2.INTER_NEAREST)
        Hh = im.shape[0]

        def px(x, y):
            return int(round((x / RES + n / 2 - x0) * zo)), int(round(Hh - (y / RES + n / 2 - y0) * zo))

        xa, xb = (x0 - n / 2) * RES, (x1 - n / 2) * RES
        ya, yb = (y0 - n / 2) * RES, (y1 - n / 2) * RES
        for k in range(int(math.floor(xa * 2)), int(xb * 2) + 1):
            c = px(k / 2, 0)[0]
            cv2.line(im, (c, 0), (c, Hh), (110, 110, 110) if k % 2 == 0 else (60, 60, 60), 1)
            if k % 2 == 0:
                cv2.putText(im, str(k // 2), (c + 3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        for k in range(int(math.floor(ya * 2)), int(yb * 2) + 1):
            r = px(0, k / 2)[1]
            cv2.line(im, (0, r), (im.shape[1], r), (110, 110, 110) if k % 2 == 0 else (60, 60, 60), 1)
            if k % 2 == 0:
                cv2.putText(im, str(k // 2), (3, r - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if modell:
            for t in modell.get("teile", []):
                p, q = px(t["x1"], t["y1"]), px(t["x2"], t["y2"])
                farbe = (0, 0, 255) if t.get("fest", True) else (255, 0, 255)
                cv2.rectangle(im, p, q, farbe, 1)
                if t.get("name") and abs(p[0] - q[0]) > 60 and not t["name"].startswith("Wand"):
                    cv2.putText(im, t["name"], (min(p[0], q[0]) + 2, min(p[1], q[1]) + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)
            for t in modell.get("schraeg", []):
                cv2.line(im, px(t["x1"], t["y1"]), px(t["x2"], t["y2"]), (0, 0, 255), 2)
            for t in modell.get("teppiche", []):
                cv2.rectangle(im, px(t["x1"], t["y1"]), px(t["x2"], t["y2"]), (0, 220, 255), 1)
            for x, y, _ in modell.get("stuehle", []):
                cv2.circle(im, px(x, y), 22, (0, 200, 255), 1)
            if modell.get("start"):
                sx, sy, sth = modell["start"]
                cv2.arrowedLine(im, px(sx, sy), px(sx + 0.4 * math.cos(sth), sy + 0.4 * math.sin(sth)), (0, 255, 0), 2)
        cv2.imwrite("%s_%s.png" % (a.aus, name), im)
    print("Ausschnitt x %.2f..%.2f  y %.2f..%.2f" % (xa, xb, ya, yb))


if __name__ == "__main__":
    main()
