"""Ego-Motion einer Rohaufnahme nachrechnen und den optischen Fluss als Linien einzeichnen.

Zum Untersuchen, warum der Kamera-Kurs (vis_yaw) bei Geradeausfahrt wegläuft.

  python tools/ego_debug.py DIR --config rider.toml [--video out.mp4] [--odom odomcal.json]
                            [--von T] [--bis T] [--set ego.proc_width=320]

DIR: Rohaufnahme (xgonav --record-raw). --odom: Messdatei von tools/calibrate_odom.py
(gleiche Uhr, CLOCK_MONOTONIC) -> Kurs aus den Rädern zum Vergleich.

Im Video: grün = Inlier (Fluss x4), rot = Außenseiter, blau = Fluss, den das Modell
(nur Drehung + Expansion um die Bildmitte) für den Punkt vorhersagt; gelb = Horizont-
linie. Text: Kurs Kamera (nachgerechnet) / Kamera (aufgezeichnet) / Räder, und je Frame
der Gier-Anteil aus Teilmengen der Punkte (links/rechts, oben/unten), um zu sehen,
woher der Fehler kommt.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.config import Config  # noqa: E402
from xgonav.egomotion import EgoMotion  # noqa: E402


def set_param(cfg: Config, expr: str) -> None:
    key, val = expr.split("=", 1)
    obj = cfg
    parts = key.split(".")
    for p in parts[:-1]:
        obj = getattr(obj, p)
    cur = getattr(obj, parts[-1])
    setattr(obj, parts[-1], type(cur)(val) if not isinstance(cur, bool) else val.lower() in ("1", "true", "ja"))


class Spy(EgoMotion):
    """EgoMotion, die die Punktpaare und Inlier des letzten Schritts merkt."""
    last_ab = None

    def _fit_rotation(self, a, b, inl):
        q, m = super()._fit_rotation(a, b, inl)
        self.last_ab = (a.copy(), b.copy(), m.copy(), q.copy())
        return q, m

    def fit_subset(self, a, b, sel):
        """Gier (rad) nur aus einer Teilmenge der Punkte."""
        if sel.sum() < 8:
            return None
        q, _ = EgoMotion._fit_rotation(self, a[sel], b[sel], np.ones(int(sel.sum()), bool))
        return float(q[0])


def wheel_yaw(odom: dict, kl: float, kr: float, track: float):
    """Funktion t (s) -> Kurs aus den Rädern (rad, links +), relativ zum ersten Wert."""
    mpt = math.pi * odom["wheel_diameter_m"] / odom["ticks_per_rev"]
    W = odom["wheels"]
    ts = np.array([w[1] / 1e9 for w in W])
    yaw = [0.0]
    for p, n in zip(W, W[1:]):
        dl = ((n[2] - p[2] + 2 ** 31) % 2 ** 32 - 2 ** 31) * mpt * kl
        dr = ((n[3] - p[3] + 2 ** 31) % 2 ** 32 - 2 ** 31) * mpt * kr
        yaw.append(yaw[-1] + (dr - dl) / track)
    yaw = np.array(yaw)
    return lambda t: float(np.interp(t, ts, yaw))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir")
    ap.add_argument("--config")
    ap.add_argument("--video")
    ap.add_argument("--odom")
    ap.add_argument("--von", type=float, default=None, help="Startzeit (s, Uhr der Aufnahme)")
    ap.add_argument("--bis", type=float, default=None)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--csv", help="je Frame t, Kurse, Teilmengen-Gier")
    a = ap.parse_args()
    cfg = Config.load(a.config)
    for s in a.set:
        set_param(cfg, s)
    meta = [json.loads(ln) for ln in open(os.path.join(a.dir, "meta.jsonl"))]
    meta = [m for m in meta if (a.von is None or m["t"] >= a.von) and (a.bis is None or m["t"] <= a.bis)
            and os.path.exists(os.path.join(a.dir, m["file"]))]
    if not meta:
        sys.exit("keine Frames")
    W, H = cfg.camera.width, cfg.camera.height
    em = Spy(cfg.ego, (W, H), cfg.camera.hfov_deg)
    wy = wheel_yaw(json.load(open(a.odom)), cfg.odom.scale_left, cfg.odom.scale_right,
                   cfg.odom.track_m) if a.odom else None
    vw = None
    if a.video:
        vw = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), cfg.camera.fps, (W, H))
    csv = open(a.csv, "w") if a.csv else None
    if csv:
        csv.write("t;kamera_neu;kamera_alt;raeder;links;rechts;oben;unten;inlier;n\n")
    yaw_cam = 0.0
    sub_cum = {"links": 0.0, "rechts": 0.0, "oben": 0.0, "unten": 0.0}
    y0_rec = meta[0]["yaw_vis_deg"]
    t0 = meta[0]["t"]
    w0 = wy(t0) if wy else None
    s = em.s
    for m in meta:
        img = cv2.imread(os.path.join(a.dir, m["file"]))
        em.last_ab = None
        res = em.update(m["idx"], m["t"], img)
        yaw_cam += res.yaw
        sub = {}
        if em.last_ab is not None:
            pa, pb, inl, q = em.last_ab
            cx, cy = em.pw / 2, em.ph / 2
            sel = {"links": pa[:, 0] < cx, "rechts": pa[:, 0] >= cx,
                   "oben": pa[:, 1] < cy, "unten": pa[:, 1] >= cy}
            for k, v in sel.items():
                y = em.fit_subset(pa, pb, v & inl)
                sub[k] = y
                if y is not None:
                    sub_cum[k] += y
        if vw is not None:
            out = img.copy()
            hz = em.ph / 2 * s
            cv2.line(out, (0, int(hz)), (W, int(hz)), (0, 220, 255), 1)
            if em.last_ab is not None:
                pa, pb, inl, q = em.last_ab
                Hm = em._model_h(*q)
                P = np.hstack([pa, np.ones((len(pa), 1))]).T
                Q = Hm @ P
                pred = (Q[:2] / Q[2]).T
                for (x0, y0), (x1, y1), (xp, yp), ok in zip(pa, pb, pred, inl):
                    p0 = (int(x0 * s), int(y0 * s))
                    p1 = (int((x0 + 4 * (x1 - x0)) * s), int((y0 + 4 * (y1 - y0)) * s))
                    pp = (int((x0 + 4 * (xp - x0)) * s), int((y0 + 4 * (yp - y0)) * s))
                    cv2.line(out, p0, pp, (255, 120, 0), 1)
                    cv2.line(out, p0, p1, (0, 220, 0) if ok else (0, 0, 255), 2)
                    cv2.circle(out, p0, 2, (255, 255, 255), -1)
            lines = [
                "t=%.2f s  v=%+.2f  Inlier %d" % (m["t"] - t0, m["sent"][0], res.inliers),
                "Kurs Kamera neu %+6.1f  aufgez. %+6.1f  Raeder %s" % (
                    math.degrees(yaw_cam), (m["yaw_vis_deg"] or 0) - (y0_rec or 0),
                    "%+6.1f" % math.degrees(wy(m["t"]) - w0) if wy else "-"),
                "Teilmengen kumuliert: L %+5.1f R %+5.1f oben %+5.1f unten %+5.1f" % tuple(
                    math.degrees(sub_cum[k]) for k in ("links", "rechts", "oben", "unten")),
                "Frame: Gier %+.2f  Nick %+.2f  Roll %+.2f  Exp %+.4f" % (
                    math.degrees(res.yaw), math.degrees(res.pitch), math.degrees(res.roll), res.scale - 1),
            ]
            for i, txt in enumerate(lines):
                cv2.putText(out, txt, (6, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3)
                cv2.putText(out, txt, (6, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            vw.write(out)
        if csv:
            f = lambda v: "" if v is None else "%.4f" % math.degrees(v)
            csv.write("%.3f;%.3f;%s;%s;%s;%s;%s;%s;%d;%d\n" % (
                m["t"], math.degrees(yaw_cam), m["yaw_vis_deg"],
                f(wy(m["t"]) - w0) if wy else "", f(sub.get("links")), f(sub.get("rechts")),
                f(sub.get("oben")), f(sub.get("unten")), res.inliers,
                0 if em.last_ab is None else len(em.last_ab[0])))
    if vw is not None:
        vw.release()
    print("Frames %d, Dauer %.1f s" % (len(meta), meta[-1]["t"] - t0))
    print("Kurs Kamera nachgerechnet %+.1f°, aufgezeichnet %+.1f°, Räder %s" % (
        math.degrees(yaw_cam), (meta[-1]["yaw_vis_deg"] or 0) - (y0_rec or 0),
        "%+.1f°" % math.degrees(wy(meta[-1]["t"]) - w0) if wy else "-"))
    print("Teilmengen kumuliert: " + ", ".join("%s %+.1f°" % (k, math.degrees(v)) for k, v in sub_cum.items()))


if __name__ == "__main__":
    main()
