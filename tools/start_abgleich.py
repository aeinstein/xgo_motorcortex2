"""Abgleich am Startbild offline an Rohfahrten (rawrec: Bilder + meta.jsonl): dieselbe Rechnung wie live
(xgonav/startbild.py messen), nur die Bilder kommen aus der Aufnahme. Läuft am PC oder auf kimmy, nicht am Rider.

  python tools/start_abgleich.py FAHRT [--anker IDX] [--ende IDX] [--basen 0.1,0.15,0.2,0.25,0.3] [--paare]

Anker = Bild IDX (Default das erste), Ende = Bild IDX (Default das letzte), Tiefenbilder = frühere Bilder mit den
Abständen --basen laut Odometrie und höchstens 45° anderem Kurs (live: die gerade Schlussfahrt von home).
Ausgabe: Ende im KS des Ankers, gemessen und laut Odometrie, dazu Güte und Korrektur.
--paare: Gegenprobe an kurzen Paaren derselben Fahrt (0,3–1,2 m, < 25° Kursänderung), wo die Odometrie fast
exakt ist: Messung minus Odometrie, Median und p90.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from xgonav.config import Config  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.startbild import Ansicht, messen  # noqa: E402


def lade(d: str) -> list[dict]:
    ms = []
    with open(os.path.join(d, "meta.jsonl"), encoding="utf-8") as f:
        for ln in f:
            try:
                ms.append(json.loads(ln))
            except json.JSONDecodeError:
                pass
    return sorted(ms, key=lambda m: m["t"])


def ansicht(d: str, m: dict, cfg) -> Ansicht:
    """Wie App._sb_ansicht, aus einer Zeile der Aufnahme: Pose (Grad -> rad), IMU-Roll (Mittel der Proben des
    Bildes), pitch_dev, Kamerahöhe aus dem Höhen-Sollwert."""
    geo = CamGeometry.from_cfg(cfg, m.get("height_mm") or 90.0)
    g = cv2.equalizeHist(cv2.imread(os.path.join(d, m["file"]), 0))
    r = [q[1] for q in (m.get("imu") or [])]
    return Ansicht(g, (m["pose"][0], m["pose"][1], math.radians(m["pose"][2])), float(np.mean(r)) if r else 0.0,
                   float(m.get("pitch_dev") or 0.0), geo.f, geo.cx, geo.H / 2, geo.tilt_deg, geo.cam_h, geo.cam_fwd,
                   m["t"])


def relativ(a: tuple, b: tuple) -> tuple[float, float, float]:
    """Lage von b im KS von a (vor, links, Kurs Grad); Posen (x, y, rad)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, math.degrees((b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi)


def tiefenbilder(ms: list[dict], i0: int, basen) -> list[int]:
    """Je Abstand das erste frühere Bild mit mindestens diesem Abstand und höchstens 45° anderem Kurs."""
    q, out = ms[i0]["pose"], []
    for basis in basen:
        for j in range(i0 - 1, max(i0 - 3000, 0), -1):
            p = ms[j]["pose"]
            if math.hypot(p[0] - q[0], p[1] - q[1]) >= basis:
                if abs((p[2] - q[2] + 180) % 360 - 180) <= 45 and j not in out:
                    out.append(j)
                break
    return out


def abgleich(d: str, ia: int = 0, i0: int = -1, basen=(0.1, 0.15, 0.2, 0.25, 0.3), cfg=None, ms=None) -> dict:
    cfg = cfg or Config()
    ms = ms or lade(d)
    ia, i0 = ia % len(ms), i0 % len(ms)
    tj = tiefenbilder(ms, i0, basen)
    A, E = ansicht(d, ms[ia], cfg), ansicht(d, ms[i0], cfg)
    erg = messen(A, E, [ansicht(d, ms[j], cfg) for j in tj], cfg.startbild)
    out = {"fahrt": os.path.basename(d.rstrip("/")), "anker": ms[ia]["file"], "ende": ms[i0]["file"],
           "tiefen": [ms[j]["file"] for j in tj], "ok": erg.ok, "grund": erg.grund, "inl": erg.inl, "n3d": erg.n3d,
           "rep_px": erg.rep_px, "dnick_deg": round(erg.dnick_deg, 2), "droll_deg": round(erg.droll_deg, 2),
           "ms": round(erg.ms), "ende_odometrie": [round(v, 3) for v in relativ(A.pose, E.pose)]}
    if erg.anker_wahr is not None:
        out["ende_gemessen"] = [round(v, 3) for v in relativ(erg.anker_wahr, E.pose)]
        out["korrektur"] = erg.info
    if erg.frei6 is not None:
        out["ende_frei6"] = [round(v, 3) for v in relativ(erg.frei6, E.pose)]
    return out


def paare(d: str, cfg) -> None:
    ms = lade(d)
    fehler = []
    for ia in range(0, len(ms) - 300, 97):
        a = ms[ia]
        for ib in range(ia + 60, min(ia + 300, len(ms))):
            b = ms[ib]
            dd = math.hypot(b["pose"][0] - a["pose"][0], b["pose"][1] - a["pose"][1])
            dth = abs((b["pose"][2] - a["pose"][2] + 180) % 360 - 180)
            if 0.3 <= dd <= 1.2 and dth < 25 and a.get("balancing") and b.get("balancing"):
                r = abgleich(d, ia, ib, cfg.startbild.basen, cfg=cfg, ms=ms)
                if "ende_gemessen" in r:
                    o, g = r["ende_odometrie"], r["ende_gemessen"]
                    e = (g[0] - o[0], g[1] - o[1], g[2] - o[2])
                    if r["ok"]:
                        fehler.append(e)
                    print("%5d->%5d %.2f m inl %3d %-4s Fehler %+.3f %+.3f %+.2f°  %s" % (
                        ia, ib, dd, r["inl"], "ok" if r["ok"] else "nein", *e, r["grund"]), flush=True)
                break
    if fehler:
        f = np.array(fehler)
        print("angenommen n=%d: |Fehler| Median vor %.3f, links %.3f m, Kurs %.2f°; p90 Abstand %.3f m, max %.3f m" % (
            len(f), np.median(abs(f[:, 0])), np.median(abs(f[:, 1])), np.median(abs(f[:, 2])),
            np.percentile(np.hypot(f[:, 0], f[:, 1]), 90), np.hypot(f[:, 0], f[:, 1]).max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fahrt")
    ap.add_argument("--anker", type=int, default=0)
    ap.add_argument("--ende", type=int, default=-1)
    ap.add_argument("--basen", default=None, help="Abstände der Tiefenbilder vor dem Ende (m), Default [startbild] basen")
    ap.add_argument("--paare", action="store_true")
    a = ap.parse_args()
    cfg = Config()
    if a.basen:
        cfg.startbild.basen = [float(v) for v in a.basen.split(",")]
    if a.paare:
        paare(a.fahrt, cfg)
    else:
        print(json.dumps(abgleich(a.fahrt, a.anker, a.ende, cfg.startbild.basen, cfg), ensure_ascii=False))


if __name__ == "__main__":
    main()
