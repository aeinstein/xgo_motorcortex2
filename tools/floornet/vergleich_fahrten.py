#!/usr/bin/env python3
"""FloorNet: Netz und Verfahren (kanten) als Entscheider über aufgezeichnete Fahrten vergleichen, ohne Fahrt am Rider.

  python tools/floornet/vergleich_fahrten.py DATEN/unten4 --config rider/config.toml --net lauf/ --out vgl/ \\
      --hand hand.json --set grid.max_range_m=1.0

Beide Spuren bekommen dieselben Bilder, denselben Nick und dieselbe Bahn und durchlaufen je eine Merkkarte und eine
Raumkarte wie in App._step. Das Netz muss die Fahrt nicht im Training gehabt haben (--net wie bei eval_hand.py).

Bericht je Spur:
- Sprünge: Nachbarspalten mehr als 40 px auseinander, je Bild. Flackern: Spalten, die sich im Stillstand von einem Bild
  zum nächsten um mehr als 40 px ändern.
- Regelung wie safety(): Anteil Halt, Bremsen, Wechsel von Halt je Minute.
- Bahn geradeaus wie _arc_check(): Anteil blockiert, Wechsel je Minute; „verschwunden“: Blockade weg, ohne dass der
  Rider sich bewegt hat. Das ist der Fall, in dem eine springende Spalte gemerkte Hindernisse löscht.
- Falsch blockiert auf der gefahrenen Bahn: Die Merkkarte sperrt im Korridor von ±radius (18 cm) die Strecke, die der
  Rider danach geradeaus gefahren ist. Die Fahrten lenkte kanten; das begünstigt kanten. Ein echtes Hindernis neben
  der Bahn zählt hier mit, und das Netz sieht mehr davon.
- Durchfahren: Ein bestätigter Kartenpunkt liegt höchstens 6 cm neben der Spur, die der Rider in den nächsten 3 s
  gefahren ist. Dort war sicher kein Hindernis (außer flachen Dingen). Die strengste Zahl gegen Fehlalarme.
- Raumkarte: belegte Zellen, davon auf der gefahrenen Bahn (Rand 0,1 m). PNG mit beiden Karten nebeneinander.
- --hand: Regelung gegen die Handlinie im selben Bild (übersehen: Linie sagt blockiert, Spur nicht; Fehlalarm: umgekehrt).

Ob etwas wirklich frei war, zeigen die Fahrten nicht: Wo das Netz zu weit sieht, ist nur an den Handlinien messbar.
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

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from label import MAX_TURN_DPS, load_cfg, load_run, make_geo, select_segment, trajectory  # noqa: E402
from xgonav.freespace import FreeSpace, FreeSpaceEstimator, net_to_columns, _p20  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402
from xgonav.localmap import LocalMap  # noqa: E402

JUMP_PX = 40.0
NCOLS = 80
STILL_M, STILL_RAD = 0.03, math.radians(3.0)    # So wenig Bewegung zwischen zwei Schritten gilt als Stillstand
DRIVEN_S = 3.0                                  # So weit voraus zählt die gefahrene Bahn
DRIVEN_MIN_M, DRIVEN_TOL_M = 0.2, 0.1
PATH_MARGIN_M = 0.1
THROUGH_M = 0.06                                # So nah an der Spur, dass der Rider durch den Punkt gefahren ist


def voll_zeilen(fs: FreeSpace, H: int, scale: float, n: int = NCOLS, cols: np.ndarray | None = None) -> np.ndarray:
    """Grenzzeile im Vollbild, n Spalten. Die Spalten des Verfahrens werden gleichmäßig ausgedünnt."""
    c = fs.columns if cols is None else cols
    idx = ((np.arange(n) + 0.5) * len(c) / n).astype(int)
    return (H - c[idx] * (H - fs.horizon_row)) * scale


def spruenge(rows: np.ndarray, px: float = JUMP_PX) -> int:
    with np.errstate(invalid="ignore"):
        return int((np.abs(np.diff(rows)) > px).sum())


def regelung(center: float, blocked_free: float, slow_free: float) -> str:
    """Was safety() aus dem Freiraum im Zentrum macht."""
    if center < blocked_free * 0.6:
        return "halt"
    return "bremst" if center < slow_free else "frei"


def gefahren(x, y, th, t, i: int, sek: float = DRIVEN_S, max_kurs: float = math.radians(10.0)) -> float:
    """Strecke, die der Rider ab Bild i geradeaus vorwärts gefahren ist (m), bis er mehr als max_kurs vom Kurs abweicht."""
    c, s = math.cos(th[i]), math.sin(th[i])
    best = 0.0
    j = i + 1
    while j < len(t) and t[j] - t[i] <= sek:
        if abs((th[j] - th[i] + math.pi) % (2 * math.pi) - math.pi) > max_kurs:
            break
        best = max(best, (x[j] - x[i]) * c + (y[j] - y[i]) * s)
        j += 1
    return best


def punkte_auf_spur(pts, pose, wx, wy, r: float = THROUGH_M) -> np.ndarray:
    """Punkte der Merkkarte (Rider-KS, Radachse) näher als r an der Spur (Welt). Rückgabe: Maske."""
    if not len(pts) or not len(wx):
        return np.zeros(len(pts), bool)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    a = pose[0] + c * pts[:, 0] - s * pts[:, 1]
    b = pose[1] + s * pts[:, 0] + c * pts[:, 1]
    d2 = (a[:, None] - np.asarray(wx)[None, :]) ** 2 + (b[:, None] - np.asarray(wy)[None, :]) ** 2
    return (d2 < r * r).any(axis=1)


def zellen_auf_bahn(occ: np.ndarray, x, y, rand: float = PATH_MARGIN_M) -> int:
    """Belegte Zellen (Welt-m, Spalten x, y) näher als rand an der Bahn."""
    if not len(occ):
        return 0
    n = 0
    for a, b in occ:
        n += bool(((x - a) ** 2 + (y - b) ** 2 < rand * rand).any())
    return n


def hand_center(rows, ok, horizon: float, H: int, W: int, scale: float, n_sectors: int) -> float:
    """Freiraum im Zentrum, wenn die Handlinie die Grenze wäre. Unsichere Spalten zählen nicht."""
    cols = net_to_columns(rows, horizon, H, W, scale)
    mask = np.asarray(ok)[((np.arange(W) + 0.5) * len(ok) / W).astype(int)]
    cols = np.where(mask, cols, np.nan)
    ex = np.linspace(0, W, n_sectors + 1).astype(int)
    return FreeSpace(sectors=[_p20(cols[ex[i]:ex[i + 1]]) for i in range(n_sectors)]).center


class Spur:
    """Eine Entscheidungsquelle mit eigenen Karten und Zählern."""

    def __init__(self, name, cfg, geo, fse):
        self.name, self.cfg, self.fse = name, cfg, fse
        m = cfg.map
        self.lmap = LocalMap(geo, fse.W, fse.H, max_age_s=m.max_age_s, confirm=m.confirm, confirm_radius_m=m.confirm_radius_m)
        self.gm = GridMap(cfg.grid, geo, fse.W, fse.H)
        self.z = dict(schritte=0, sprung=0, still_spalten=0, still_flacker=0, halt=0, bremst=0, halt_wechsel=0, bahn_block=0, bahn_wechsel=0,
                      verschwunden=0, gef_n=0, gef_falsch=0, spur_n=0, spur_treffer=0, hand_n=0, hand_block=0, uebersehen=0, fehlalarm=0, halt_hand=0)
        self.prev_rows = self.prev_halt = self.prev_blk = None
        self.ereignisse = []
        self.durch = None                  # im letzten Schritt durchfahrener Punkt (x, y im Rider-KS)
        self.durch_x = []

    def schritt(self, fs, m, t, ds, dyaw, pose, still, rate, L, v_chk, need, W, H, wx=(), wy=()):
        z = self.z
        z["schritte"] += 1
        rows = voll_zeilen(fs, self.fse.H, self.fse.scale)
        z["sprung"] += spruenge(rows)
        if self.prev_rows is not None and still:
            z["still_spalten"] += NCOLS
            with np.errstate(invalid="ignore"):
                z["still_flacker"] += int((np.abs(rows - self.prev_rows) > JUMP_PX).sum())
        self.prev_rows = rows
        # Erkennungen sind kein Hindernis, die Merkkarte bekommt das Bodenmodell unmaskiert (wie App._step)
        b = self.cfg.behavior
        r = regelung(fs.center, b.blocked_free, b.slow_free)
        z["halt"] += r == "halt"
        z["bremst"] += r == "bremst"
        if self.prev_halt is not None and (r == "halt") != self.prev_halt:
            z["halt_wechsel"] += 1
        self.prev_halt = r == "halt"
        self.lmap.move(ds, dyaw)
        self.lmap.add_freespace(fs, t, m["pitch_dev"])
        blk = self.lmap.arc_clearance(v_chk, 0.0, horizon_s=1.2) < need
        z["bahn_block"] += blk
        if self.prev_blk is not None and blk != self.prev_blk:
            z["bahn_wechsel"] += 1
            if self.prev_blk and still:
                z["verschwunden"] += 1
                self.ereignisse.append(m["file"])
        self.prev_blk = blk
        if L >= DRIVEN_MIN_M:
            z["gef_n"] += 1
            z["gef_falsch"] += self.lmap.arc_clearance(1.0, 0.0, horizon_s=1.5, steps=15) < L - DRIVEN_TOL_M
        self.durch = None
        if len(wx) > 1 and float(np.hypot(np.diff(wx), np.diff(wy)).sum()) >= DRIVEN_MIN_M:
            z["spur_n"] += 1
            pts = self.lmap.confirmed_points(2.5)
            hit = punkte_auf_spur(pts, pose, wx, wy)
            if hit.any():
                z["spur_treffer"] += 1
                q = pts[hit]
                self.durch = q[np.argmin(np.hypot(q[:, 0], q[:, 1]))]
                self.durch_x.append(float(np.hypot(*self.durch)))
        if rate <= MAX_TURN_DPS:
            self.gm.x, self.gm.y, self.gm.th = pose
            self.gm.add_freespace(view, m["pitch_dev"], pose)


def bericht(z: dict, minuten: float) -> dict:
    n = max(z["schritte"], 1)
    return {"Sprünge je Bild": round(z["sprung"] / n, 2),
            "Flackern im Stand %": round(100 * z["still_flacker"] / max(z["still_spalten"], 1), 2),
            "Halt %": round(100 * z["halt"] / n, 1), "Bremst %": round(100 * z["bremst"] / n, 1),
            "Halt-Wechsel je min": round(z["halt_wechsel"] / minuten, 1),
            "Bahn blockiert %": round(100 * z["bahn_block"] / n, 1), "Bahn-Wechsel je min": round(z["bahn_wechsel"] / minuten, 1),
            "verschwunden je min": round(z["verschwunden"] / minuten, 2),
            "falsch blockiert auf Bahn %": round(100 * z["gef_falsch"] / max(z["gef_n"], 1), 1), "Stellen Bahn": z["gef_n"],
            "durchfahren %": round(100 * z["spur_treffer"] / max(z["spur_n"], 1), 1), "Stellen Spur": z["spur_n"],
            "Hand-Bilder": z["hand_n"], "Hand blockiert": z["hand_block"], "übersehen": z["uebersehen"], "Fehlalarm": z["fehlalarm"],
            "belegte Zellen": z.get("belegt", 0), "belegt auf Bahn": z.get("auf_bahn", 0)}


def karte(sp: Spur, x, y, th) -> np.ndarray:
    gm = sp.gm
    for a, b in zip(x[::5], y[::5]):
        gm._clear_disk(float(a), float(b), gm.cfg.driven_free_r_m)
    gm.path = list(zip(x[::5].tolist(), y[::5].tolist()))
    gm.x, gm.y, gm.th = float(x[-1]), float(y[-1]), float(th[-1])
    span = max(float(np.ptp(x)), float(np.ptp(y))) + 3.0
    img = gm.render(720, span, (float(x.mean()), float(y.mean())))[0]
    img = img.copy()
    cv2.putText(img, sp.name, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return img


def durch_bild(img, kr, nr, W, sp, m, run, out) -> None:
    """Bild zum durchfahrenen Punkt: gelb kanten, magenta Netz. Bleibt auf dem Rechner, auf dem es entsteht."""
    vis = img.copy()
    xs = (np.arange(NCOLS) + 0.5) * W / NCOLS
    for r, c in ((kr, (0, 255, 255)), (nr, (255, 0, 255))):
        ok = np.isfinite(r)
        if ok.sum() > 1:
            cv2.polylines(vis, [np.stack([xs[ok], r[ok]], 1).astype(np.int32)], False, c, 2, cv2.LINE_AA)
    cv2.putText(vis, "%s: Punkt x %.2f y %.2f" % (sp.name, sp.durch[0], sp.durch[1]), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    os.makedirs(os.path.join(out, "durch"), exist_ok=True)
    cv2.imwrite(os.path.join(out, "durch", "%s_%s_%s" % (run, sp.name, m["file"])), vis)


def fahrt(d: str, cfg, net, dev, hand: dict, out: str) -> dict:
    import torch

    from net import CROP_H, CROP_Y, IN_H, IN_W, decode, prep

    run = Path(d).name
    geo = make_geo(cfg)
    ms = select_segment(load_run(d))
    x, y, th, rate = trajectory(ms)
    t = np.array([m["t"] for m in ms], float)
    fse = FreeSpaceEstimator(cfg.freespace, (geo.W, geo.H), geo)
    W, H = geo.W, geo.H
    spuren = [Spur("kanten", cfg, geo, fse), Spur("netz", cfg, geo, fse)]
    v_chk = cfg.robot.max_v * cfg.robot.vx_limit
    need = 0.6 * v_chk + 0.08
    b = cfg.behavior
    last = None
    letzte_zeilen = {}
    for i, m in enumerate(ms):
        key = run + "/" + m["file"]
        if i % 2 and key not in hand:
            continue
        img = cv2.imread(os.path.join(d, m["file"]))
        if img is None:
            continue
        fs = fse.update(img, m["pitch_dev"], [tr["box"] for tr in m.get("tracks_all", m.get("tracks", []))])
        small = cv2.resize(img[CROP_Y:CROP_Y + CROP_H], (IN_W, IN_H), interpolation=cv2.INTER_AREA)
        with torch.no_grad():
            xt = torch.as_tensor(small[None], device=dev).permute(0, 3, 1, 2).float() / 255.0
            nr = decode(net(prep(xt, False))).cpu().numpy()[0]
        fn = dataclasses.replace(fs, net_rows=nr)
        fse.adopt_net(fn, nr)
        ds = dyaw = 0.0
        if last is not None:
            dyaw = (th[i] - th[last] + math.pi) % (2 * math.pi) - math.pi
            mid = (th[i] + th[last]) / 2
            ds = (x[i] - x[last]) * math.cos(mid) + (y[i] - y[last]) * math.sin(mid)
        still = last is not None and abs(ds) < STILL_M and abs(dyaw) < STILL_RAD
        last = i
        pose = (float(x[i]), float(y[i]), float(th[i]))
        L = gefahren(x, y, th, t, i)
        j = int(np.searchsorted(t, t[i] + DRIVEN_S))
        for sp, f in zip(spuren, (fs, fn)):
            sp.schritt(f, m, m["t"], ds, dyaw, pose, still, rate[i], L, v_chk, need, W, H, x[i:j], y[i:j])
            if sp.ereignisse and sp.ereignisse[-1] == m["file"]:
                letzte_zeilen[(sp.name, m["file"])] = (img.copy(), voll_zeilen(fs, fse.H, fse.scale), nr)
            if sp.durch is not None and len(sp.durch_x) % 25 == 1:
                durch_bild(img, voll_zeilen(fs, fse.H, fse.scale), nr, W, sp, m, run, out)
        if key in hand:
            hc = hand_center(hand[key][0], hand[key][1], fs.horizon_row, fse.H, fse.W, fse.scale, len(fs.sectors))
            for sp, f in zip(spuren, (fs, fn)):
                z = sp.z
                z["hand_n"] += 1
                hb = hc < b.blocked_free
                z["hand_block"] += hb
                sb = f.center < b.blocked_free
                z["uebersehen"] += hb and not sb
                z["fehlalarm"] += sb and not hb
    minuten = max((t[-1] - t[0]) / 60.0, 1e-3)
    os.makedirs(out, exist_ok=True)
    res = {"Fahrt": run, "Minuten": round(minuten, 1), "Bilder": len(ms)}
    bilder = []
    for sp in spuren:
        occ = sp.gm.occupied_cells()
        sp.z["belegt"], sp.z["auf_bahn"] = len(occ), zellen_auf_bahn(occ, x[::5], y[::5])
        res[sp.name] = bericht(sp.z, minuten)
        if sp.durch_x:
            res[sp.name]["durchfahren: Punkt entfernt m, Median/p90"] = [round(float(np.median(sp.durch_x)), 2), round(float(np.percentile(sp.durch_x, 90)), 2)]
        bilder.append(karte(sp, x, y, th))
        for f in sp.ereignisse[:6]:
            q = letzte_zeilen.get((sp.name, f))
            if q is None:
                continue
            vis, kr, nr = q
            xs = (np.arange(NCOLS) + 0.5) * W / NCOLS
            for r, c in ((kr, (0, 255, 255)), (nr, (255, 0, 255))):
                ok = np.isfinite(r)
                if ok.sum() > 1:
                    cv2.polylines(vis, [np.stack([xs[ok], r[ok]], 1).astype(np.int32)], False, c, 1, cv2.LINE_AA)
            os.makedirs(os.path.join(out, "ereignisse"), exist_ok=True)
            cv2.imwrite(os.path.join(out, "ereignisse", "%s_%s_%s" % (run, sp.name, f)), vis)
    cv2.imwrite(os.path.join(out, run + "_karten.png"), np.hstack(bilder))
    with open(os.path.join(out, run + ".json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--config", required=True)
    ap.add_argument("--net", required=True, help="Verzeichnis mit floornet.pt")
    ap.add_argument("--out", required=True)
    ap.add_argument("--hand", default="")
    ap.add_argument("--set", action="append", default=[], metavar="ABSCHNITT.SCHLÜSSEL=WERT")
    a = ap.parse_args()

    import torch

    import eval_hand as E
    from net import FloorNet

    cfg = load_cfg(a.config, a.set)
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    net = FloorNet().to(dev)
    net.load_state_dict(torch.load(os.path.join(a.net, "floornet.pt"), map_location=dev))
    net.eval()
    hand = E.load_hand(a.hand) if a.hand else {}
    for d in a.dirs:
        print(json.dumps(fahrt(d, cfg, net, dev, hand, a.out), ensure_ascii=False, indent=1), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
