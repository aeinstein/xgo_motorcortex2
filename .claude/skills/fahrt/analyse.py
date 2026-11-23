"""Auswertung einer Rohaufnahme (fahrt/<datum-zeit>/meta.jsonl + Bilder), am PC:
  .venv/Scripts/python .claude/skills/fahrt/analyse.py <fahrtverzeichnis> [--von s --bis s] [--spur]

Gibt aus: Dauer, Modi, Statuswechsel (Zahlen zusammengefasst), Ereignisse (Bremsen der Bahnprüfung, Freiraum!,
Sperrzone, komme nicht weiter, Ziel gesperrt), Steckenbleiben (Fahrwert > 0,08, Räder < 0,05 m/s länger als STECKT_S),
Nick-Spitzen (|Nick| > 9°), höchstes Radtempo. Schreibt <fahrt>_ereignisse.jpg: Bilder zu Beginn jedes Steckens und
jedes Bremsens (höchstens 12). --spur: je Bild Zeit, Fahrwert, Räder, Pose, Nick, Status im Bereich --von/--bis."""
from __future__ import annotations

import argparse
import json
import os
import re
from collections import Counter

import cv2
import numpy as np

STECKT_S = 1.2      # der Rider rollt erst ~1 s nach dem Befehl an; kürzer zählt nicht als Stecken
# Reihenfolge im Ereignisbild: Wichtiges zuerst (höchstens 12 Bilder), angezeigt nach Zeit
WICHTIG = {"steckt": 0, "Freiraum!": 0, "gefallen": 0, "komme nicht weiter": 1, "Ziel gesperrt": 1, "Sperrzone": 1,
           "LIEGT": 2, "bremse": 3}


def lade(d):
    with open(os.path.join(d, "meta.jsonl"), encoding="utf-8") as f:
        return [json.loads(z) for z in f if z.strip()]


def rad(e):
    sc = e.get("speed_ctrl") or {}
    if sc.get("ist") is not None:
        return sc["ist"]
    r = e.get("raeder") or []
    if len(r) > 1 and r[-1][0] > r[0][0]:
        return (r[-1][1] - r[0][1] + r[-1][2] - r[0][2]) / 2 / (r[-1][0] - r[0][0])
    return None


def nick(e):
    im = e.get("imu") or []
    return im[-1][2] if im else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fahrt")
    ap.add_argument("--von", type=float, default=0.0)
    ap.add_argument("--bis", type=float, default=1e9)
    ap.add_argument("--spur", action="store_true")
    a = ap.parse_args()
    L = lade(a.fahrt)
    t0 = L[0]["t"]
    T = lambda e: e["t"] - t0
    print("Fahrt %s: %d Bilder, %.1f s, Modi %s" % (os.path.basename(a.fahrt.rstrip("/\\")), len(L), T(L[-1]),
                                                   dict(Counter(e["mode"] for e in L))))
    vs = [v for v in (rad(e) for e in L) if v is not None]
    if vs:
        print("Räder: höchstens %.2f m/s, rückwärts bis %.2f m/s" % (max(vs), min(vs)))

    if a.spur:
        for e in L:
            if a.von <= T(e) <= a.bis:
                v = rad(e)
                print("%7.2f %s v=%+.3f w=%+.3f rad=%s pose=%s nick=%s %s" % (
                    T(e), e["file"][:6], e["sent"][0], e["sent"][1], "-" if v is None else "%+.2f" % v,
                    e.get("pose"), "-" if nick(e) is None else "%.1f" % nick(e), e["status"][:100]))
        return

    print("\nStatuswechsel:")
    prev = None
    for e in L:
        s = re.sub(r"[-+]?\d+(\.\d+)?", "#", e["status"])
        if s != prev:
            print("  %7.2f %s %-8s %s" % (T(e), e["file"][:6], e["mode"], e["status"][:110]))
            prev = s

    ereig = []
    print("\nEreignisse:")
    muster = ("bremse", "Freiraum!", "Sperrzone", "komme nicht weiter", "Ziel gesperrt", "gefallen", "LIEGT")
    last = {}
    for e in L:
        for m in muster:
            if m in e["status"] and T(e) - last.get(m, -9) > 1.0:
                v = rad(e)
                print("  %7.2f %s %-18s Räder %s  %s" % (T(e), e["file"][:6], m, "-" if v is None else "%.2f" % v,
                                                        e["status"][:90]))
                ereig.append((T(e), e["file"], m))
            if m in e["status"]:
                last[m] = T(e)

    print("\nSteckt (Fahrwert > 0,08, Räder < 0,05 m/s, > %.1f s; kürzer ist nur das Anfahren):" % STECKT_S)
    start = None
    for e in L + [None]:
        st = e is not None and e["sent"][0] > 0.08 and rad(e) is not None and rad(e) < 0.05
        if st and start is None:
            start = e
        elif not st and start is not None:
            ende = T(e) if e is not None else T(L[-1])
            if ende - T(start) > STECKT_S:
                print("  %7.2f-%7.2f %s  %s" % (T(start), ende, start["file"][:6], start["status"][:80]))
                ereig.append((T(start), start["file"], "steckt"))
            start = None

    print("\nNick-Spitzen (|Nick| > 9°):")
    last_t = -9
    for e in L:
        n = nick(e)
        if n is not None and abs(n) > 9 and T(e) - last_t > 1.0:
            print("  %7.2f %s Nick %.1f°  %s" % (T(e), e["file"][:6], n, e["status"][:80]))
            last_t = T(e)

    ereig = sorted(sorted(ereig, key=lambda x: (WICHTIG.get(x[2], 2), x[0]))[:12])
    if ereig:
        tiles = []
        for t, f, m in ereig:
            img = cv2.imread(os.path.join(a.fahrt, f))
            if img is None:
                continue
            img = cv2.resize(img, (320, 240))
            cv2.putText(img, "%.1fs %s %s" % (t, f[:6], m), (5, 20), 0, 0.5, (0, 255, 255), 2)
            tiles.append(img)
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        out = a.fahrt.rstrip("/\\") + "_ereignisse.jpg"
        cv2.imwrite(out, np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))
        print("\nBilder:", out)


if __name__ == "__main__":
    main()
