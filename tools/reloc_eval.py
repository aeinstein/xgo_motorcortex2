#!/usr/bin/env python3
"""Kartenabgleich (xgonav/reloc.py) messen. Liest nur.

  python tools/reloc_eval.py suche FAHRT [...] --config rider/config.toml [--fenster 15] [--abstand 10]
  python tools/reloc_eval.py bench [--karte raumkarte.npz] [--n 5]

suche: Findet sich der Rider in seiner eigenen Karte wieder? Aus einer Rohaufnahme wird je ein Stück
(--fenster Sekunden) herausgenommen, die Karte aus dem Rest gebaut und das Stück darin gesucht, als wäre
er dort abgesetzt worden. Soll ist die aufgezeichnete Pose am Ende des Stücks; die driftet selbst, deshalb
gilt bis 0,3 m und 10° als richtig. Das erste Stück ist der Rundumblick am Anfang von explore.
Wichtig ist die Zeile "falsch": angenommen, aber daneben. "unsicher" ist erlaubt.

bench: Rechenzeit von Feld, lokalem Abgleich und globaler Suche, am Pi auszuführen. Ohne --karte ein
künstlicher Raum, mit --karte die gespeicherte Raumkarte (grid.file).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "floornet"))

from xgonav import reloc  # noqa: E402
from xgonav.config import Config  # noqa: E402


def suche_fahrt(d: str, cfg, fenster: float, abstand: float, sperre_ab: float | None = None, netz: str = "",
                mit_stueck: bool = False) -> dict:
    from map_replay import _fahre, lade
    from xgonav.gridmap import GridMap
    f = lade(d, cfg, netz=netz)
    t, bahn, views, pitch = f["t"], f["bahn"], f["views"], f["pitch"]
    rc = cfg.reloc
    dt = float(np.median(np.diff(t)))
    rate = f["rate_rad"] if f["rate_rad"] is not None else bahn.rate_dps(t)
    sperre = cfg.grid.max_turn_dps if sperre_ab is None else sperre_ab
    idx = [i for i in sorted(views) if rate[min(i + 2, len(t) - 1)] <= sperre]
    gm0 = GridMap(cfg.grid, f["geo"], f["fw"], f["fh"])
    pose = {i: tuple(float(v) for v in bahn.at(t[i] - dt)) for i in idx}
    pkt = {i: reloc.scan_punkte(gm0, views[i], float(pitch[i]), rc) for i in idx}
    zahl = {"richtig": 0, "falsch": 0, "unsicher": 0}
    gruende: dict = {}
    zeilen = []
    for ende in np.arange(t[0] + fenster, t[-1], abstand):
        gm = GridMap(cfg.grid, f["geo"], f["fw"], f["fh"])
        puffer = reloc.ScanPuffer(rc, window_s=1e9, window_m=1e9)
        letzte = None
        for i in idx:
            if ende - fenster < t[i] <= ende:
                puffer.add(float(t[i]), pose[i], *pkt[i])
                letzte = i
            if mit_stueck or not (ende - fenster - 2.0 < t[i] <= ende + 2.0):       # Nachbarn sehen dasselbe
                _fahre(gm, *pose[i])
                gm.add_freespace(views[i], float(pitch[i]), pose[i])
        if letzte is None or len(puffer) < 20:
            continue
        soll = pose[letzte]
        t0 = time.perf_counter()
        e = reloc.suche_global(reloc.Feld(gm.lo, gm.res, cfg.grid.l_show, rc), puffer.scan(soll), rc)
        ms = (time.perf_counter() - t0) * 1000
        dm = math.hypot(e.pose[0] - soll[0], e.pose[1] - soll[1])
        dg = math.degrees(reloc._winkel_diff(e.pose[2], soll[2]))
        art = "unsicher" if not e.ok else ("richtig" if dm < 0.3 and dg < 10 else "falsch")
        zahl[art] += 1
        if not e.ok:
            gruende[e.grund] = gruende.get(e.grund, 0) + 1
        zeilen.append({"Ende s": round(float(ende - t[0]), 0), "Drehung Grad": round(math.degrees(puffer.spanne()[1])),
                       "Punkte": e.n, "Ergebnis": art, "Grund": e.grund, "Güte": round(e.guete, 2),
                       "Anteil": round(e.anteil, 2), "zweit": round(e.zweit, 2), "daneben m": round(dm, 2),
                       "daneben Grad": round(dg), "ms": round(ms)})
    return {"Fahrt": Path(d).name, "Zahl": zahl, "unsicher wegen": gruende, "Stücke": zeilen}


def _kunstraum(n: int = 320, res: float = 0.05) -> np.ndarray:
    """Raum mit Nische und Kiste als Log-Odds (wie tests/test_reloc.py)."""
    def px(pts):
        return np.round(np.asarray(pts, float) / res + n / 2 - 0.5).astype(np.int32).reshape(-1, 1, 2)
    lo = np.zeros((n, n), np.float32)
    raum = [(-2, -1.5), (2, -1.5), (2, 0.5), (0.5, 0.5), (0.5, 2), (-2, 2)]
    cv2.fillPoly(lo, [px(raum)], -2.0)
    cv2.polylines(lo, [px(raum)], True, 2.0, 1)
    cv2.fillPoly(lo, [px([(-1.0, -1.5), (-0.7, -1.5), (-0.7, -1.1), (-1.0, -1.1)])], 2.0)
    return lo


def _blick(lo: np.ndarray, res: float, l_show: float, pose: tuple, rc, rmax: float = 1.25) -> reloc.Scan:
    """Rundumblick an pose durch Strahlwurf in der Karte (ohne Rauschen)."""
    n = lo.shape[0]
    occ = lo > l_show
    s = np.arange(0.05, rmax, 0.01)
    hits, free = [], []
    for a in np.radians(np.arange(0, 360, 1.5)):
        r = np.clip(np.floor((pose[1] + s * math.sin(pose[2] + a)) / res + n / 2).astype(int), 0, n - 1)
        c = np.clip(np.floor((pose[0] + s * math.cos(pose[2] + a)) / res + n / 2).astype(int), 0, n - 1)
        i = np.flatnonzero(occ[r, c])
        d = s[i[0]] if len(i) else None
        if d is not None:
            hits.append((d * math.cos(a), d * math.sin(a)))
        free += [(q * math.cos(a), q * math.sin(a)) for q in np.arange(0.15, (d - 0.15) if d is not None else rmax, rc.free_step_m)]
    hits = np.array(hits).reshape(-1, 2)
    w = 1.0 / (1.0 + (np.hypot(hits[:, 0], hits[:, 1]) / rc.far_m) ** 2) if len(hits) else np.zeros(0)
    hits, w = reloc._duenn(hits, rc.hit_res_m, w)
    free, _ = reloc._duenn(np.array(free).reshape(-1, 2), rc.free_res_m)
    return reloc.Scan(hits, w, free)


def bench(a) -> None:
    cfg = Config.load(a.config) if a.config else Config()
    rc, res, l_show = cfg.reloc, cfg.grid.resolution_m, cfg.grid.l_show
    if a.karte:
        with np.load(a.karte) as d:
            lo, res = d["lo"].astype(np.float32), float(d["res"])
    else:
        lo = _kunstraum(int(round(cfg.grid.size_m / res)), res)
    # Standort: freie Zelle mit der meisten Wand in 0,6 bis 1,0 m (dort ist die Suche am teuersten)
    occ = (lo > l_show).astype(np.float32)
    k = int(round(1.0 / res))
    nah = cv2.boxFilter(occ, -1, (2 * k + 1, 2 * k + 1), normalize=False)
    weit = cv2.distanceTransform((occ < 0.5).astype(np.uint8), cv2.DIST_L2, 3) * res
    nah[(lo > -l_show) | (weit < 0.5)] = -1
    r, c = np.unravel_index(int(np.argmax(nah)), nah.shape)
    n = lo.shape[0]
    pose = ((c + 0.5 - n / 2) * res, (r + 0.5 - n / 2) * res, 0.5)

    def zeit(fn):
        ms = []
        for _ in range(a.n):
            t0 = time.perf_counter()
            out = fn()
            ms.append((time.perf_counter() - t0) * 1000)
        return out, float(np.median(ms)), float(max(ms))
    feld, m, mx = zeit(lambda: reloc.Feld(lo, res, l_show, rc))
    print("Karte %d x %d Zellen, %d belegt; Standort %.2f, %.2f" % (n, n, feld.n_occ, pose[0], pose[1]))
    print("Feld                 Median %7.1f ms   max %7.1f ms" % (m, mx))
    scan = _blick(lo, res, l_show, pose, rc)
    print("Scan: %d Treffer, %d Freiproben" % (scan.n, len(scan.free)))
    e, m, mx = zeit(lambda: reloc.abgleich_lokal(feld, scan, (pose[0] + 0.1, pose[1] - 0.1, pose[2] + 0.05), rc))
    print("lokaler Abgleich     Median %7.1f ms   max %7.1f ms   %s %s" % (m, mx, "ok" if e.ok else "unsicher:", e.grund))
    feld._grob.clear()
    e, m, mx = zeit(lambda: reloc.suche_global(feld, scan, rc))
    print("globale Suche        Median %7.1f ms   max %7.1f ms   %s %s" % (m, mx, "ok" if e.ok else "unsicher:", e.grund))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("suche")
    s.add_argument("dirs", nargs="+")
    s.add_argument("--config", required=True)
    s.add_argument("--fenster", type=float, default=15.0)
    s.add_argument("--abstand", type=float, default=10.0)
    s.add_argument("--out", default="", help="Ergebnis je Fahrt als JSON in diesen Ordner")
    s.add_argument("--set", action="append", default=[], metavar="ABSCHNITT.SCHLÜSSEL=WERT")
    s.add_argument("--netz", default="", help="Bodengrenze allein aus FloorNet mit diesen Gewichten (wie am Rider)")
    s.add_argument("--mit-stueck", action="store_true",
                   help="das Stück bleibt in der Karte (Obergrenze: so gut ginge es, wenn die Karte die Stelle kennt)")
    b = sub.add_parser("bench")
    b.add_argument("--karte", default="")
    b.add_argument("--config", default="")
    b.add_argument("--n", type=int, default=5)
    a = ap.parse_args()
    if a.cmd == "bench":
        bench(a)
        return 0
    from label import load_cfg
    cfg = load_cfg(a.config, a.set)
    for d in a.dirs:
        res = suche_fahrt(d, cfg, a.fenster, a.abstand, netz=a.netz, mit_stueck=a.mit_stueck)
        print("%s: %s  unsicher wegen %s" % (res["Fahrt"], res["Zahl"], res["unsicher wegen"]))
        for z in res["Stücke"]:
            print("  %4.0f s  Drehung %4d°  Punkte %3d  %-9s %-24s Güte %.2f  Anteil %.2f  zweit %.2f  daneben %.2f m %3d°  %d ms" % (
                z["Ende s"], z["Drehung Grad"], z["Punkte"], z["Ergebnis"], z["Grund"], z["Güte"], z["Anteil"], z["zweit"],
                z["daneben m"], z["daneben Grad"], z["ms"]), flush=True)
        if a.out:
            Path(a.out).mkdir(parents=True, exist_ok=True)
            with open(Path(a.out) / (res["Fahrt"] + "_suche.json"), "w", encoding="utf-8") as fh:
                json.dump(res, fh, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
