"""Koppelnavigation (xgonav/odomfusion.py) offline an Rohfahrten prüfen: IMU-Kurs, Radwege und Bild-Kurs aus
meta.jsonl (rawrec) je Bild durch OdomFusion, wie im Regelschritt am Rider (jedes Bild, IMU-Zeit = Bildzeit +
[imu] time_offset_ms). Läuft am PC oder auf kimmy, nicht am Rider.

  python tools/odom_replay.py FAHRT/meta.jsonl [...] [--set odom.contra_deg=6] [--ereignisse]

Je Fahrt: Drehung laut IMU und Rädern, Abweichung Räder minus IMU im Fenster (Verteilung), Widersprüche, Anteil
Schlupf, Urteile der Kamera, Kalibriervorschlag. Achtung: Die Bild-Drehung ist in meta.jsonl auf 0,1° gerundet und
zeigt Bilder ohne Ego-Motion nicht an (dort 0 statt None).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from xgonav.config import Config  # noqa: E402
from xgonav.odomfusion import OdomFusion  # noqa: E402


def lade(path: str):
    ms = []
    with open(path, encoding="utf-8") as f:
        for ln in f:
            try:
                ms.append(json.loads(ln))
            except json.JSONDecodeError:
                pass
    ms.sort(key=lambda m: m["t"])
    imu = sorted({tuple(s) for m in ms for s in (m.get("imu") or [])})
    rad = sorted({tuple(s) for m in ms for s in (m.get("raeder") or [])})
    return ms, (np.array(imu) if imu else None), (np.array(rad) if rad else None)


def bei(arr, t, max_age):
    """Werte (Spalten 1..) zu t, linear; nach der neuesten Probe gehalten, solange <= max_age alt; sonst None."""
    if arr is None or len(arr) < 2 or t < arr[0, 0]:
        return None
    k = int(np.searchsorted(arr[:, 0], t))
    if k >= len(arr):
        return arr[-1, 1:] if t - arr[-1, 0] <= max_age else None
    if t - arr[k - 1, 0] > max_age and arr[k, 0] - t > max_age:
        return None                                          # Lücke (liegend, Aufnahme unterbrochen)
    a, b = arr[k - 1], arr[k]
    f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 1.0
    return a[1:] + f * (b[1:] - a[1:])


def fahrt(path: str, cfg, ereignisse: bool = False) -> dict:
    ms, imu, rad = lade(path)
    f = OdomFusion(cfg.odom)
    off = (cfg.imu.time_offset_ms or 0.0) / 1000.0
    abw, n_schlupf, n, quellen = [], 0, 0, {}
    sum_i = sum_w = sum_ds = 0.0
    w_alt = z_alt = 0
    vis_alt = None
    t_alt = None
    episoden, ep = [], None              # Schlupf-Abschnitte: Beginn, größte Abweichung, Status, IMU, Räder
    for m in ms:
        t = m["t"] + off
        bal = bool(m.get("balancing"))
        y = bei(imu, t, cfg.imu.max_age_s) if imu is not None else None
        yaw = math.radians(float(y[2])) if y is not None else None
        ws = bei(rad, t, cfg.odom.max_age_s) if (rad is not None and bal) else None
        vis = m.get("yaw_vis_deg")
        dv = math.radians(vis - vis_alt) if vis is not None and vis_alt is not None else None
        vis_alt = vis
        dt = t - t_alt if t_alt is not None else 0.0
        t_alt = t
        st = f.step(t, yaw, 0, (float(ws[0]), float(ws[1])) if ws is not None else None, dv, 0.0, bal)
        n += 1
        quellen[st.kurs_src + "/" + st.ds_src] = quellen.get(st.kurs_src + "/" + st.ds_src, 0) + 1
        if f._win:                                        # nur Schritte mit beiden Kursen
            abw.append(math.degrees(f.abw))
            ei = sum(x[1] for x in f._win)
            sum_ds += abs(st.ds)
        if f.schlupf:
            n_schlupf += 1
            if ep is None:
                ep = [m["t"] - ms[0]["t"], 0.0, "", 0.0, 0.0]
            if abs(f.abw) > abs(ep[1]):
                ep[1:] = [f.abw, m.get("status", "")[:50], sum(x[1] for x in f._win), sum(x[2] for x in f._win)]
        elif ep is not None:
            episoden.append(ep)
            ep = None
        if ereignisse and (f.widersprueche != w_alt or f.imu_zweifel != z_alt):
            w = f._win
            ei = math.degrees(sum(x[1] for x in w)) if w else float("nan")
            ew = math.degrees(sum(x[2] for x in w)) if w else float("nan")
            print("  t %8.1f s  %-9s IMU %6.1f°  Räder %6.1f°  Abw %5.1f°  ds %.3f m  %s" % (
                m["t"] - ms[0]["t"], "ZWEIFEL" if f.imu_zweifel != z_alt else "Schlupf", ei, ew,
                math.degrees(f.abw), st.ds, m.get("status", "")[:40]))
        w_alt, z_alt = f.widersprueche, f.imu_zweifel
    if ep is not None:
        episoden.append(ep)
    if ereignisse:
        for e in sorted(episoden, key=lambda e: -abs(e[1]))[:5]:
            print("  größte: t %7.1f s  Abw %6.1f°  (IMU %6.1f°, Räder %6.1f° im Fenster)  %s" % (
                e[0], math.degrees(e[1]), math.degrees(e[3]), math.degrees(e[4]), e[2]))
    a = np.abs(np.array(abw)) if abw else np.zeros(1)
    dauer = ms[-1]["t"] - ms[0]["t"] if ms else 0.0
    f._rechne_vorschlag()
    return {"fahrt": os.path.basename(os.path.dirname(path)), "bilder": n, "dauer_s": round(dauer),
            "imu": imu is not None, "raeder": rad is not None,
            "abw_p50": round(float(np.percentile(a, 50)), 2), "abw_p95": round(float(np.percentile(a, 95)), 2),
            "abw_p99": round(float(np.percentile(a, 99)), 2), "abw_max": round(float(a.max()), 1),
            "widersprueche": f.widersprueche, "schlupf_anteil": round(n_schlupf / max(n, 1), 3),
            "imu_zweifel": f.imu_zweifel, "quellen": quellen, "vorschlag": f.vorschlag}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("meta", nargs="+")
    ap.add_argument("--set", action="append", default=[], help="z. B. odom.contra_deg=6")
    ap.add_argument("--ereignisse", action="store_true")
    a = ap.parse_args()
    cfg = Config()
    for s in a.set:
        k, v = s.split("=", 1)
        sec, key = k.split(".")
        setattr(getattr(cfg, sec), key, type(getattr(getattr(cfg, sec), key))(v))
    for p in a.meta:
        print(json.dumps(fahrt(p, cfg, a.ereignisse), ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
