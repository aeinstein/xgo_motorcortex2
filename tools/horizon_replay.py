#!/usr/bin/env python3
"""Nickwinkel offline prüfen: Aufnahme von tools/record_status.py gegen ein farbiges Band in Kamerahöhe
(Wand, Rider davor) vergleichen. Das Band zeigt die echte Kameraneigung: Liegt es auf Kamerahöhe, hängt
seine Bildzeile nicht von der Entfernung ab, nur von der Neigung.

  python tools/horizon_replay.py DIR
  python tools/horizon_replay.py DIR --basis 0:2.5 --hsv-lo 15,120,100 --hsv-hi 35,255,255

Verglichen werden die aufgezeichneten Linien aus /status ([horizon] enabled): row_imu (cyan, IMU minus
Standreferenz) und row_bild (magenta, das, was xgonav für die Entfernungen nimmt). Ausgabe je Sekunde:
Band, IMU, Bildlinie in px relativ zum Basisfenster (Rider steht), Fehler = Abstand zum Band; dazu eine
Zusammenfassung nach Abschnitten (vor der ersten Drehung, 0–5 s nach einer Drehung, später). Drehung =
Bild-Gierrate über --dreh-dps.
"""
from __future__ import annotations

import argparse
import json
import os

import cv2
import numpy as np


def band_row(path: str, lo: tuple, hi: tuple) -> float | None:
    """Mitte des größten Farbflecks (Zeile, Vollbild). Overlay-Linien (1–2 px quer) werden überbrückt."""
    im = cv2.imread(path)
    if im is None:
        return None
    m = cv2.inRange(cv2.cvtColor(im, cv2.COLOR_BGR2HSV), lo, hi)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < 400:
        return None
    return st[k, cv2.CC_STAT_TOP] + st[k, cv2.CC_STAT_HEIGHT] / 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir")
    ap.add_argument("--basis", default="0:2.5", help="Basisfenster in s (Rider steht, Band im Bild)")
    ap.add_argument("--hsv-lo", default="15,120,100")
    ap.add_argument("--hsv-hi", default="35,255,255")
    ap.add_argument("--dreh-dps", type=float, default=15.0, help="Bild-Gierrate, ab der eine Drehung zählt")
    a = ap.parse_args()
    d = json.load(open(os.path.join(a.dir, "daten.json")))
    lo, hi = tuple(map(int, a.hsv_lo.split(","))), tuple(map(int, a.hsv_hi.split(",")))
    frames = sorted(d["frames"], key=lambda f: f["idx"])
    st = {}
    for s in d["status"]:
        if s.get("idx") is not None:
            st.setdefault(s["idx"], s)
    t0 = frames[0]["t"]
    rows, t_turn_end, turning_seen = [], None, False
    for f in frames:
        s = st.get(f["idx"]) or {}
        h, e = s.get("h") or {}, s.get("ego") or {}
        ts = f["t"] - t0
        turning = abs(e.get("yaw_dps", 0.0)) > a.dreh_dps
        if turning:
            t_turn_end, turning_seen = ts, True
        if h.get("row_imu") is None or h.get("row_bild") is None:
            continue
        rows.append({"ts": ts, "imu": h["row_imu"], "bild": h["row_bild"], "turning": turning,
                     "seit": None if t_turn_end is None or turning else ts - t_turn_end, "vor": not turning_seen,
                     "band": band_row(os.path.join(a.dir, "%06d.jpg" % f["idx"]), lo, hi)})
    b0, b1 = map(float, a.basis.split(":"))
    base = [r for r in rows if b0 <= r["ts"] < b1 and r["band"] is not None]
    if not base:
        raise SystemExit("kein Band im Basisfenster")
    bb = float(np.median([r["band"] for r in base]))
    bi = float(np.median([r["imu"] for r in base]))
    bm = float(np.median([r["bild"] for r in base]))
    print("Basis %s s: Band %.1f, IMU %.1f, Bildlinie %.1f (Zeilen)" % (a.basis, bb, bi, bm))
    print("   s         Band    IMU   Bildl. | Fehler IMU  Bildl.")

    def gut(sel):
        return [r for r in sel if r["band"] is not None and abs(r["band"] - bb) < 40]   # Band weg: Fleck woanders

    for s0 in np.arange(0, max(r["ts"] for r in rows) + 1, 1.0):
        w = [r for r in rows if s0 <= r["ts"] < s0 + 1]
        if not w:
            continue
        tag = "dreht" if sum(r["turning"] for r in w) > len(w) / 2 else ""
        wb = gut(w)
        if len(wb) < 3:
            print("%4.0f  %-6s  (kein Band)" % (s0, tag))
            continue
        b = np.median([r["band"] for r in wb]) - bb
        i = np.median([r["imu"] for r in wb]) - bi
        m = np.median([r["bild"] for r in wb]) - bm
        print("%4.0f  %-6s %+6.1f %+6.1f %+7.1f | %+8.1f %+7.1f" % (s0, tag, b, i, m, i - b, m - b))

    groups = {"vor der ersten Drehung": [r for r in rows if r["vor"]],
              "0-5 s nach Drehung": [r for r in rows if r["seit"] is not None and r["seit"] < 5],
              "später (> 5 s)": [r for r in rows if r["seit"] is not None and r["seit"] >= 5]}
    print("\nAbschnitt                 Bilder | IMU Median/p95 px | Bildlinie Median/p95 px")
    for name, sel in groups.items():
        sel = gut(sel)
        if len(sel) < 5:
            continue
        fi = np.abs([(r["imu"] - bi) - (r["band"] - bb) for r in sel])
        fm = np.abs([(r["bild"] - bm) - (r["band"] - bb) for r in sel])
        print("%-24s %6d | %6.1f / %5.1f     | %6.1f / %5.1f" % (name, len(sel), np.median(fi), np.percentile(fi, 95),
                                                                np.median(fm), np.percentile(fm, 95)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
