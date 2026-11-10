#!/usr/bin/env python3
"""Raumkarte aus einer Rohaufnahme nachrechnen, je Variante eine Karte. Nur am PC, liest nur.

  python tools/map_replay.py FAHRT --config rider/config.toml --out vgl/
  python tools/map_replay.py FAHRT --config rider/config.toml --out vgl/ --variante spaet2:versatz=80

Wozu: Am Rider entstehen Doppelwände. Dieselbe Fahrt wird mit einzeln umgeschalteten Ursachen neu
eingetragen; die Karten stehen nebeneinander in FAHRT_karten.png, Kennzahlen in FAHRT.json. Was die
Doppelwand beseitigt, zeigt das Bild; die Kennzahlen sind Beiwerk.

Die Bodengrenze wird je Bild einmal gerechnet (jedes zweite Bild, wie live) und für alle Varianten
verwendet. Eine Variante ist "name:schlüssel=wert,..." und ändert die Basis:
  versatz=MS       Pose zur Zeit Bildzeit + MS aus der aufgezeichneten Posenfolge (Basis: ein Bild früher,
                   so trägt die App ein: Pose des letzten Regelschritts vor dem Bild; 0 = Pose des Bildes)
  sperre=DPS       Drehratensperre (Basis: grid.max_turn_dps)
  quelle=rad|pose  Drehrate aus dem Radkurs beim Eintreffen des Ergebnisses (Basis, wie App) oder aus dem
                   Kurs der Pose zur Bildzeit
  ankunft=N        so viele Bilder später trifft das Ergebnis ein (Basis 2)
  roll=GRAD|imu    Grenzzeile je Spalte um diesen Roll gedreht; imu = Roll aus den Proben der Aufnahme
                   (nur Aufnahmen mit IMU-Proben), abzüglich --roll-ref
  kurs=imu         Kurs zur Bildzeit aus den IMU-Proben der Aufnahme statt aus der Posenfolge
  grid.X=WERT      Parameter aus [grid], z. B. grid.row_bias=1.5

Grenzen: Die aufgezeichnete Pose ist die der Raumkarte nach dem Regelschritt des Bildes, also selbst
eine Treppe mit etwa 12 Stufen je Sekunde. Wie alt sie gegenüber dem Bild ist, steht nicht in der
Aufnahme; versatz verschiebt nur relativ dazu. Mit --bahn drv wird die Bahn wie in label.py aus dem
Firmware-Kurs neu gerechnet (alte Aufnahmen mit Radkurs).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent / "floornet"))

from label import load_cfg, load_run, make_geo, select_segment, trajectory  # noqa: E402
from xgonav.freespace import FreeSpaceEstimator  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402
from xgonav.localmap import mask_movable  # noqa: E402

PRESETS = ["basis:", "pose_bild:versatz=0", "frueh:versatz=-85", "spaet:versatz=40",
           "sperre_aufnahme:quelle=pose", "sperre20:quelle=pose,sperre=20",
           "bias+1.5:grid.row_bias=1.5", "bias0:grid.row_bias=0", "ohne_bestaetigung:grid.confirm_dr_m=0",
           "roll+1:roll=1", "roll-1:roll=-1"]
STILL_S = 0.2          # Posenfolge: so lange unverändert gilt als Stillstand, nicht als Stufe der Treppe


def parse_variante(s: str) -> tuple[str, dict]:
    name, _, rest = s.partition(":")
    p = {}
    for kv in filter(None, rest.split(",")):
        k, _, v = kv.partition("=")
        p[k.strip()] = v.strip()
    return name.strip(), p


def stuetzstellen(t: np.ndarray, x: np.ndarray, y: np.ndarray, th: np.ndarray) -> np.ndarray:
    """Indizes, an denen die Posenfolge gilt: das erste Bild jeder Stufe (dort wurde sie fortgeschrieben)
    und das letzte, wenn sie länger als STILL_S stand. Dazwischen wird linear interpoliert."""
    neu = np.concatenate([[True], (np.diff(x) != 0) | (np.diff(y) != 0) | (np.diff(th) != 0)])
    keep = neu.copy()
    first = np.flatnonzero(neu)
    last = np.concatenate([first[1:] - 1, [len(t) - 1]])
    lang = t[last] - t[first] > STILL_S
    keep[last[lang]] = True
    keep[-1] = True
    return np.flatnonzero(keep)


class Bahn:
    """Pose zu beliebiger Zeit aus der aufgezeichneten Folge (th in rad, fortlaufend)."""

    def __init__(self, t, x, y, th):
        self.t, self.x, self.y, self.th = (np.asarray(a, float) for a in (t, x, y, th))
        self._k = stuetzstellen(self.t, self.x, self.y, self.th)

    def at(self, tq):
        k = self._k
        return (np.interp(tq, self.t[k], self.x[k]), np.interp(tq, self.t[k], self.y[k]),
                np.interp(tq, self.t[k], self.th[k]))

    def rate_dps(self, tq, win: float = 0.1):
        a, b = self.at(np.asarray(tq) - win)[2], self.at(np.asarray(tq) + win)[2]
        return np.abs(np.degrees(b - a)) / (2 * win)


def fortlaufend_deg(a) -> np.ndarray:
    a = np.asarray(a, float)
    d = (np.diff(a) + 180.0) % 360.0 - 180.0
    return np.concatenate([[0.0], np.cumsum(d)])


def rate_fenster(t: np.ndarray, deg: np.ndarray, win: float = 0.05) -> np.ndarray:
    """Drehrate (°/s) aus einer Kursfolge über ±win um jedes Bild."""
    a, b = np.interp(t - win, t, deg), np.interp(t + win, t, deg)
    return np.abs(b - a) / (2 * win)


def roll_spalten(view, geo, fw: int, fh: int, roll_deg: float):
    """Bodengrenze um den Roll gedreht: Bei Roll liegt die Grenze je Spalte um (Spalte − Mitte)·tan(Roll)
    Zeilen falsch. Spalten ohne Treffer (frei bis Horizont) und unbekannte bleiben."""
    if not roll_deg:
        return view
    cols = np.asarray(view.columns, float).copy()
    sc = geo.W / fw
    span = max(1.0, fh - view.horizon_row)
    u = (np.arange(len(cols)) + 0.5) * (fw / len(cols)) - geo.cx / sc
    with np.errstate(invalid="ignore"):
        hit = cols < 0.98
    cols[hit] = np.clip(cols[hit] - u[hit] * math.tan(math.radians(roll_deg)) / span, 0.0, 0.979)
    return dataclasses.replace(view, columns=cols)


def doppelt(gm: GridMap, k: int = 5) -> tuple[int, int]:
    """Belegte Zellen und Zwischenzellen: nicht belegte Zellen, die zwischen belegten liegen, die
    höchstens k − 1 Zellen (20 cm) auseinander sind (Schließen k×k minus belegt). Zwei Wände dicht
    nebeneinander oder eine aufgefächerte Wand ergeben viele, eine saubere Wand fast keine.
    Die Dicke (Fläche / Umfang) taugt dafür nicht: Eine ausgefranste Doppelwand hat viel Umfang."""
    occ = (gm.lo > gm.cfg.l_show).astype(np.uint8)
    zu = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    return int(occ.sum()), int(zu.sum() - occ.sum())


def quer(gm: GridMap, bahn: "Bahn", r_max: float = 2.5, schritt: int = 5) -> tuple[float, int]:
    """Lichte Breite quer zur Bahn: von Bahnpunkten nach links und rechts bis zur ersten belegten Zelle,
    Median der Summe dort, wo beide Seiten eine Wand haben (m), und Zahl dieser Stellen. In einem Flur
    ist das die Flurbreite der Karte; gegen das Maßband zeigt sie, ob die Wände zu nah oder zu weit liegen."""
    occ = gm.lo > gm.cfg.l_show
    s = np.arange(gm.res / 2, r_max, gm.res / 2)
    breiten = []
    for x, y, th in zip(bahn.x[::schritt], bahn.y[::schritt], bahn.th[::schritt]):
        d = []
        for seite in (1.0, -1.0):
            a = th + seite * math.pi / 2
            r, c = gm.cell(x + s * math.cos(a), y + s * math.sin(a))
            ok = (r >= 0) & (r < gm.n) & (c >= 0) & (c < gm.n)
            hit = np.flatnonzero(ok & occ[np.clip(r, 0, gm.n - 1), np.clip(c, 0, gm.n - 1)])
            d.append(s[hit[0]] if len(hit) else None)
        if d[0] is not None and d[1] is not None:
            breiten.append(d[0] + d[1])
    return (round(float(np.median(breiten)), 2), len(breiten)) if breiten else (0.0, 0)


def _fahre(gm: GridMap, x: float, y: float, th: float) -> None:
    """Pose setzen wie GridMap.move: Weg merken, gefahrenen Weg freiräumen."""
    gm.x, gm.y, gm.th = x, y, th
    px, py = gm.path[-1]
    if math.hypot(x - px, y - py) > 0.05:
        gm.path.append((x, y))
        gm._clear_disk(x, y, gm.cfg.driven_free_r_m)


def rechne(t, bahn: Bahn, views: dict, pitch, cfg, geo, fw: int, fh: int, varianten, rate_rad=None,
           roll_imu=None, kurs_imu=None) -> list[tuple[str, GridMap, dict]]:
    """Je Variante eine Karte. views: Bildindex -> FreeSpace (schon ohne bewegliche Objekte),
    pitch je Bild (rad). rate_rad: Drehrate des Radkurses je Bild (°/s), None = aus der Pose.
    roll_imu je Bild (Grad) und kurs_imu (Funktion Zeit -> rad) nur bei Aufnahmen mit IMU-Proben."""
    t = np.asarray(t, float)
    dt_bild = float(np.median(np.diff(t))) if len(t) > 1 else 0.04
    rate_pose = bahn.rate_dps(t)
    if rate_rad is None:
        rate_rad = rate_pose
    out = []
    for name, p in varianten:
        gcfg = dataclasses.replace(cfg.grid)
        for k, v in p.items():
            if k.startswith("grid."):
                cur = getattr(gcfg, k[5:])
                setattr(gcfg, k[5:], v.lower() in ("1", "true", "ja") if isinstance(cur, bool) else type(cur)(v))
        versatz = float(p["versatz"]) / 1000.0 if "versatz" in p else -dt_bild
        sperre = float(p.get("sperre", gcfg.max_turn_dps))
        quelle = p.get("quelle", "rad")
        ankunft = int(p.get("ankunft", 2))
        roll = p.get("roll", "")
        if (roll == "imu" and roll_imu is None) or (p.get("kurs") == "imu" and kurs_imu is None):
            continue                        # Aufnahme ohne IMU-Proben
        gm = GridMap(gcfg, geo, fw, fh)
        x0, y0, th0 = bahn.at(t[0])
        gm.x, gm.y, gm.th = float(x0), float(y0), float(th0)
        gm.path = [(gm.x, gm.y)]
        n_in = n_sperre = 0
        for i in sorted(views):
            j = min(i + ankunft, len(t) - 1)
            _fahre(gm, float(bahn.x[j]), float(bahn.y[j]), float(bahn.th[j]))
            rate = rate_rad[j] if quelle == "rad" else rate_pose[i]
            if rate > sperre:
                n_sperre += 1
                continue
            px, py, pth = (float(v) for v in bahn.at(t[i] + versatz))
            if p.get("kurs") == "imu":
                pth = float(kurs_imu(t[i]))
            view = views[i]
            if roll:
                view = roll_spalten(view, geo, fw, fh, float(roll_imu[i]) if roll == "imu" else float(roll))
            gm.add_freespace(view, float(pitch[i]), (px, py, pth))
            n_in += 1
        _fahre(gm, float(bahn.x[-1]), float(bahn.y[-1]), float(bahn.th[-1]))
        zellen, zw = doppelt(gm)
        kand = 0 if gm.conf is None else int((np.isfinite(gm.rmin) & ~gm.conf & (gm.lo <= gcfg.l_show)).sum())
        breite, stellen = quer(gm, bahn)
        out.append((name, gm, {"eingetragen": n_in, "gesperrt": n_sperre, "belegte Zellen": zellen,
                               "Zwischenzellen": zw, "Kandidaten": kand, "Breite quer m": breite,
                               "Stellen quer": stellen, "Parameter": p}))
    return out


def bild(ergebnis, bahn: Bahn, px: int = 480, je_zeile: int = 4) -> np.ndarray:
    span = max(float(np.ptp(bahn.x)), float(np.ptp(bahn.y))) + 3.0
    mitte = (float((bahn.x.max() + bahn.x.min()) / 2), float((bahn.y.max() + bahn.y.min()) / 2))
    kacheln = []
    for name, gm, z in ergebnis:
        img = gm.render(px, span, mitte)[0].copy()
        cv2.putText(img, name, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(img, "%d belegt, %d dazwischen, quer %.2f m" % (z["belegte Zellen"], z["Zwischenzellen"],
                                                                     z["Breite quer m"]), (8, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        kacheln.append(img)
    while len(kacheln) > je_zeile and len(kacheln) % je_zeile:
        kacheln.append(np.full_like(kacheln[0], 128))
    return np.vstack([np.hstack(kacheln[i:i + je_zeile]) for i in range(0, len(kacheln), je_zeile)])


def imu_proben(ms: list[dict]):
    """IMU-Proben der Aufnahme (rawrec, Feld "imu": [[t, roll, pitch, yaw] in Grad]) als Felder
    (Zeit, Roll, Kurs fortlaufend), None bei Aufnahmen ohne sie."""
    rows = [s for m in ms for s in (m.get("imu") or [])]
    if len(rows) < 2:
        return None
    a = np.array(rows, float)
    a = a[np.argsort(a[:, 0])]
    return a[:, 0], a[:, 1], fortlaufend_deg(a[:, 3])


def fahrt(d: str, cfg, varianten, out: str, bahn_quelle: str = "pose", roll_ref: float = 0.0,
          imu_versatz_ms: float = 0.0, px: int = 480) -> dict:
    run = Path(d).name
    geo = make_geo(cfg)
    ms = select_segment(load_run(d))
    if len(ms) < 10:
        raise SystemExit("%s: kein zusammenhängendes Stück mit Pose, balancierend und IMU-Nick" % d)
    t = np.array([m["t"] for m in ms], float)
    if bahn_quelle == "drv":
        x, y, th, _ = trajectory(ms)
    else:
        pose = np.array([m["pose"] for m in ms], float)
        x, y = pose[:, 0], pose[:, 1]
        th = math.radians(pose[0, 2]) + np.radians(fortlaufend_deg(pose[:, 2]))
    bahn = Bahn(t, x, y, th)
    rad = [m.get("yaw_deg") for m in ms]
    rate_rad = rate_fenster(t, fortlaufend_deg(rad)) if all(v is not None for v in rad) else None
    roll_imu = kurs_imu = None
    imu = imu_proben(ms)
    if imu is not None:
        ti, ro, ya = imu
        dv = imu_versatz_ms / 1000.0
        roll_imu = np.interp(t + dv, ti, ro) - roll_ref
        # Kurs der IMU an die Pose anhängen: gleicher Kurs im ersten Bild
        off = th[0] - math.radians(float(np.interp(t[0] + dv, ti, ya)))

        def kurs_imu(q):
            return off + math.radians(float(np.interp(q + dv, ti, ya)))
    fse = FreeSpaceEstimator(cfg.freespace, (geo.W, geo.H), geo)
    views = {}
    for i, m in enumerate(ms):
        if i % 2:
            continue
        img = cv2.imread(os.path.join(d, m["file"]))
        if img is None:
            continue
        fs = fse.update(img, m["pitch_dev"], [tr["box"] for tr in m.get("tracks_all", m.get("tracks", []))])
        tracks = m.get("tracks", [])
        free = dataclasses.replace(fs, sectors=list(fs.sectors), columns=fs.columns.copy())
        fse.block_boxes(free, [tr["box"] for tr in tracks if tr.get("id") != m.get("target")])
        views[i] = mask_movable(free, [(tr.get("cls"), tr["box"]) for tr in tracks], geo.W, geo.H)
    pitch = np.array([m["pitch_dev"] for m in ms], float)
    erg = rechne(t, bahn, views, pitch, cfg, geo, fse.W, fse.H, varianten, rate_rad, roll_imu, kurs_imu)
    os.makedirs(out, exist_ok=True)
    cv2.imwrite(os.path.join(out, run + "_karten.png"), bild(erg, bahn, px, 4 if px <= 480 else 2))
    res = {"Fahrt": run, "Bilder": len(ms), "Sekunden": round(float(t[-1] - t[0]), 1),
           "Strecke m": round(float(np.hypot(np.diff(x), np.diff(y)).sum()), 2),
           "Drehung Grad": round(float(np.abs(np.diff(np.degrees(th))).sum()), 0),
           "Bahn": bahn_quelle, "IMU-Proben": imu is not None,
           "Varianten": {name: z for name, _, z in erg}}
    with open(os.path.join(out, run + ".json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--variante", action="append", default=[], metavar="NAME:SCHLÜSSEL=WERT,...",
                    help="zusätzlich zu den Standardvarianten (--nur: statt ihrer)")
    ap.add_argument("--nur", action="store_true", help="nur basis und die mit --variante genannten")
    ap.add_argument("--bahn", choices=("pose", "drv"), default="pose")
    ap.add_argument("--px", type=int, default=480, help="Kantenlänge einer Karte im Bild")
    ap.add_argument("--roll-ref", type=float, default=0.0, help="IMU-Roll bei waagerechtem Bild (Grad)")
    ap.add_argument("--imu-versatz-ms", type=float, default=0.0, help="IMU-Zeit = Bildzeit + so viel")
    ap.add_argument("--set", action="append", default=[], metavar="ABSCHNITT.SCHLÜSSEL=WERT")
    a = ap.parse_args()
    cfg = load_cfg(a.config, a.set)
    specs = (["basis:"] if a.nur else PRESETS + ["roll_imu:roll=imu", "kurs_imu:kurs=imu"]) + a.variante
    varianten = [parse_variante(s) for s in specs]
    for d in a.dirs:
        res = fahrt(d, cfg, varianten, a.out, a.bahn, a.roll_ref, a.imu_versatz_ms, a.px)
        print(json.dumps(res, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
