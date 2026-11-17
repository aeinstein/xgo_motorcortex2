#!/usr/bin/env python3
"""Zeitversatz Bild -> IMU aus einer Rohaufnahme messen ([imu] time_offset_ms). Läuft am PC, nicht am Rider.

Die Rohaufnahme (Web-Knopf „Aufzeichnen“, xgonav/rawrec.py) enthält je Bild die Bildzeit t (SensorTimestamp,
CLOCK_MONOTONIC) und die IMU-Proben (t, Roll, Nick, Kurs) aus imu_log. Das Werkzeug rechnet den Bildnick je
Bildpaar mit EgoMotion (wie xgonav) und vergleicht ihn mit der IMU-Nickänderung im selben Zeitraum, verschoben
um einen Versatz o: IMU-Zeit = Bildzeit + o. Bestes o = höchste Korrelation (Raster 0,5 ms, Parabel durch das
Maximum). Gemessen wird nur in Abschnitten mit Nickbewegung; je Abschnitt eine eigene Schätzung (Streuung).

  python tools/imu_versatz.py ~/fahrt/20261117-161237                  # alle Bilder
  python tools/imu_versatz.py AUFNAHME --fenster 5784,6144,6777 --halb 48   # nur um diese Bilder (±48 Bilder)

Ausgabe: Versatz in ms (Eintrag time_offset_ms), Vorzeichen (pitch_sign), Korrelation und Steigung (≈ 1).
Bewegung: kräftiges Nicken (Höhe ändern, kurz anfahren/bremsen); im ruhigen Stand gibt es nichts zu messen.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from xgonav.config import Config  # noqa: E402
from xgonav.egomotion import EgoMotion  # noqa: E402


def lade(ordner: Path):
    meta = [json.loads(z) for z in open(ordner / "meta.jsonl", encoding="utf-8")]
    meta.sort(key=lambda m: m["idx"])
    proben = np.array([p for m in meta for p in (m.get("imu") or [])], float)
    proben = proben[np.argsort(proben[:, 0])]
    _, eind = np.unique(proben[:, 0], return_index=True)
    proben = proben[eind]
    proben[:, 1:] = np.radians(proben[:, 1:])      # rawrec speichert Roll, Nick, Kurs in Grad
    return meta, proben


def bildnick(ordner: Path, meta: list[dict], cfg: Config) -> list[tuple[float, float, float]]:
    """(t0, t1, Bildnick rad + = hoch) je aufeinanderfolgendem Bildpaar mit gültiger Ego-Motion."""
    ego = EgoMotion(cfg.ego, (cfg.camera.width, cfg.camera.height), cfg.camera.hfov_deg)
    out, prev = [], None
    for m in meta:
        if prev is not None and m["idx"] != prev["idx"] + 1:     # Lücke: neu anfangen
            ego = EgoMotion(cfg.ego, (cfg.camera.width, cfg.camera.height), cfg.camera.hfov_deg)
        bgr = cv2.imread(str(ordner / m["file"]))
        if bgr is None:
            prev = None
            continue
        r = ego.update(m["idx"], m["t"], bgr)
        if r.ok and r.dt > 0 and prev is not None and m["idx"] == prev["idx"] + 1:
            out.append((m["t"] - r.dt, m["t"], r.pitch))
        prev = m
    return out


def korrelation(paare, proben, versaetze):
    t0, t1, v = (np.array(x) for x in zip(*paare))
    ts, ps = proben[:, 0], proben[:, 2]
    ok = (t0 + versaetze.min() >= ts[0]) & (t1 + versaetze.max() <= ts[-1])
    t0, t1, v = t0[ok], t1[ok], v[ok]
    res = []
    for o in versaetze:
        d = np.interp(t1 + o, ts, ps) - np.interp(t0 + o, ts, ps)
        c = float(np.corrcoef(v, d)[0, 1]) if np.std(d) > 1e-9 and np.std(v) > 1e-9 else 0.0
        k = float(np.dot(v, d) / np.dot(d, d)) if np.dot(d, d) > 0 else 0.0
        res.append((c, k))
    return np.array(res), len(v)


def bestes(versaetze, res):
    c = np.abs(res[:, 0])
    i = int(np.argmax(c))
    o = versaetze[i]
    if 0 < i < len(c) - 1:                          # Parabel durch das Maximum
        a, b, cc = c[i - 1], c[i], c[i + 1]
        den = a - 2 * b + cc
        if den < 0:
            o += 0.5 * (a - cc) / den * (versaetze[1] - versaetze[0])
    return o, res[i, 0], res[i, 1]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ordner", type=Path)
    ap.add_argument("--fenster", default="", help="Bildnummern, um die gemessen wird (Komma); leer = alle Bilder")
    ap.add_argument("--halb", type=int, default=48, help="Bilder je Seite um jede Fenstermitte")
    ap.add_argument("--bereich-ms", type=float, default=80.0)
    ap.add_argument("--min-rate", type=float, default=1.0, help="Abschnitt nur bei mittlerer IMU-Nickrate über (°/s)")
    a = ap.parse_args()

    cfg = Config.neutral()
    meta, proben = lade(a.ordner)
    if len(proben) < 20:
        print("keine IMU-Proben in meta.jsonl")
        return 1
    versaetze = np.arange(-a.bereich_ms, a.bereich_ms + 1e-9, 0.5) / 1000.0
    if a.fenster:
        mitten = [int(x) for x in a.fenster.split(",")]
        abschnitte = [[m for m in meta if abs(m["idx"] - c) <= a.halb] for c in mitten]
    else:
        abschnitte, akt = [], []
        for m in meta:                                # zusammenhängende Läufe von 2*halb Bildern
            if akt and m["idx"] != akt[-1]["idx"] + 1 or len(akt) >= 2 * a.halb:
                abschnitte.append(akt)
                akt = []
            akt.append(m)
        if akt:
            abschnitte.append(akt)
    alle = []
    print("%-14s %6s %8s %7s %6s %9s" % ("Abschnitt", "Paare", "Nick°/s", "ms", "Korr", "Steigung"))
    for ab in abschnitte:
        if len(ab) < 10:
            continue
        t = np.array([m["t"] for m in ab])
        rate = np.degrees(np.abs(np.gradient(np.interp(t, proben[:, 0], proben[:, 2]), t))).mean()
        if rate < a.min_rate:
            continue
        paare = bildnick(a.ordner, ab, cfg)
        if len(paare) < 20:
            continue
        res, n = korrelation(paare, proben, versaetze)
        o, c, k = bestes(versaetze, res)
        print("%-14s %6d %8.1f %+7.1f %6.2f %9.2f" % ("%d-%d" % (ab[0]["idx"], ab[-1]["idx"]), n, rate, o * 1000, c, k))
        alle += paare
    if not alle:
        print("zu wenig Nickbewegung")
        return 1
    res, n = korrelation(alle, proben, versaetze)
    o, c, k = bestes(versaetze, res)
    print("\nalle zusammen: %d Bildpaare, Versatz %+.1f ms (IMU-Zeit = Bildzeit %+.1f ms), Korrelation %.2f, "
          "Steigung %.2f" % (n, o * 1000, o * 1000, c, k))
    print("Eintrag: [imu] time_offset_ms = %.0f, pitch_sign = %d" % (o * 1000, 1 if c > 0 else -1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
