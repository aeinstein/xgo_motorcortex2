#!/usr/bin/env python3
"""FloorNet-Labels aus Geometrie statt aus einem Bildbearbeitungsmodell: MoGe-2 (Ruicheng/moge-2-vitl-normal) schätzt
je Pixel einen 3D-Punkt, die Bodenebene wird mit der bekannten Kamerahöhe eingepasst, und je 8-px-Streifen gilt die
Regel als Rechnung: Grenze = erster Punkt, an dem ein 18 cm hoher Rider anstößt. Probe, nicht fürs Training
(Nutzer 2026-11-20: die Stage3-Labels sind besser).

  # Viele Fahrbilder -> rows.jsonl und Label-Dateien für train.py (Filter wie comfy_npz.py):
  python geo_label.py fahrten --runs explore10,explore11 --jedes 2 --config rider.toml --out geo_lab

Je Pixel, Höhe h über der eingepassten Ebene:
  Boden       h <= boden_m                  (Fugen, flache Teppiche, auch alles unter der Ebene: Spiegelungen,
                                            gekrümmter Nahbereich; kein Absturz, Nutzer 2026-11-20)
  Hindernis   boden_m < h < dach_m          (Sockelleiste, Stufe nach oben, Sofakante unter 18 cm)
  egal        h >= dach_m                   (Tischplatte, Sofa mit genug Luft darunter)
Je Streifen: freier Boden von unten (Lücken bis --luecke Zeilen überbrückt, wie comfy_label.mask_rows), dazu das
nächste Hindernis nach vorn (das --min_px-nächste Pixel des Streifens). Liegt das Hindernis näher als das Ende des
freien Bodens (Boden sichtbar unter einer niedrigen Sofakante), zählt sein Fußpunkt: die Zeile, in der der Boden
genau diese Entfernung hat.

Maßstab: MoGe liefert Meter, bei 13-15 cm Kamerahöhe aber nicht verlässlich. Deshalb wird die Ebene im Modellmaß
eingepasst und alles so skaliert, dass sie genau cam_h unter der Kamera liegt. Ohne eigene Ebene (Wand direkt davor)
rechnet das Bild mit IMU-Nick und dem Median der Skala der übrigen Bilder dieser Fahrt (art "ersatz"), nur für die
Anzeige: Solche Bilder kommen nicht in die Label-Datei (explore10-12 gegen p11 am
2026-11-18: der Ersatzweg legte die Linie ganz nach unten).

Nicht abgedeckt: Teppiche sind für den Rider Hindernis, geometrisch aber Boden. Stufen nach unten sind hier
Boden (kein Absturz, Nutzer 2026-11-20).

Braucht auf kimmy: pip install git+https://github.com/microsoft/MoGe.git (torch mit CUDA ist schon da).
Statt MoGe: --tiefe DIR liest vorab gerechnete Tiefenkarten (da3_tiefe.py, Depth Anything 3, eigenes venv) und
macht daraus mit f und Bildmitte der Rider-Kamera die Punktwolke.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

NCOLS = 80
MODELL = "Ruicheng/moge-2-vitl-normal"


@dataclass
class Regeln:
    boden_m: float = 0.025      # |h| bis hier: Boden
    dach_m: float = 0.18        # ab hier passt der Rider darunter
    luecke: int = 6             # Zeilen ohne Boden, die der freie Lauf überbrückt
    min_px: int = 16            # Pixel, die ein Hindernis im Streifen haben muss (2 Zeilen à 8 px)
    rand: int = 1               # Hindernis-Rampe am unteren Bildrand mit Boden darüber zählt als Boden (rand_rampe)
    min_inl: int = 2000         # Bodenpunkte für eine eigene Ebene, sonst Ersatzskala der Fahrt und IMU-Nick
    max_winkel: float = 8.0     # Ebenen weiter weg vom IMU-Nick gelten nicht als Boden
    toleranz: float = 0.10      # Bodenpunkt: Abstand zur Ebene unter 10 % der Kamerahöhe (1,3 cm)
    ransac: int = 256           # Ebenen aus je drei Punkten
    skala_lo: float = 0.6       # Fenster für den Modellmaßstab: Ebene zwischen 0,6 und 3,0 cam_h im Modellmaß
    skala_hi: float = 3.0       # MoGe setzt die Kamera am Rider 1,2-1,7x zu hoch (explore10-12), 1,6 verwarf 22 %


def unten(a: float) -> np.ndarray:
    """Richtung „nach unten" in Kamerakoordinaten (x rechts, y unten, z vorn) bei Blickwinkel a (rad, + = nach oben),
    passend zu train.row_dist."""
    return np.array([0.0, math.cos(a), -math.sin(a)])


def ebene(P: np.ndarray, ok: np.ndarray, n0: np.ndarray, cam_h: float, rmin: float, R: Regeln,
          s_ersatz: float | None) -> dict:
    """Bodenebene n·p = d im Modellmaß. Kandidaten: gültige Punkte unter Zeile rmin. Startwert d: häufigster Abstand
    entlang n0 im Fenster [skala_lo, skala_hi]·cam_h (Boden ist eine dichte Fläche, Wände streuen), dann dreimal
    Ebene durch die nahen Punkte (kleinster Eigenvektor). s = cam_h / d skaliert auf Meter."""
    sel = ok.copy()
    sel[:max(0, int(math.ceil(rmin))), :] = False
    Q = P[sel]
    if len(Q) > 20000:
        Q = Q[::len(Q) // 20000 + 1]
    lo, hi = R.skala_lo * cam_h, R.skala_hi * cam_h
    if len(Q) >= R.min_inl // 4:
        # RANSAC: Ebenen aus je drei Punkten, nur nahe am IMU-Nick und im Maßstabsfenster; Wände, Tischunterseiten
        # und der Boden hinter einer Stufe fallen so heraus. Toleranz relativ zu d, weil das Modellmaß noch frei ist.
        rng = np.random.default_rng(len(Q))
        tri = Q[rng.integers(0, len(Q), (R.ransac, 3))]
        nn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nn /= np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-12)
        nn *= np.sign(nn @ n0)[:, None]
        dd = np.einsum("ij,ij->i", nn, tri[:, 0])
        gut = (nn @ n0 > math.cos(math.radians(R.max_winkel))) & (dd > lo) & (dd < hi)
        if gut.any():
            nn, dd = nn[gut], dd[gut]
            S = Q[::max(1, len(Q) // 4000)]                 # Zählen auf 4000 Punkten reicht für die Wahl
            zahl = (np.abs(S @ nn.T - dd) < R.toleranz * dd).sum(0)
            b = int(np.argmax(zahl))
            n, d = nn[b], float(dd[b])
            inl = 0
            for _ in range(3):                          # nachschärfen: kleinster Eigenvektor der Inlier
                X = Q[np.abs(Q @ n - d) < R.toleranz * d]
                if len(X) < 3:
                    break
                c = X.mean(0)
                _, V = np.linalg.eigh((X - c).T @ (X - c))
                n = V[:, 0] if V[:, 0] @ n0 > 0 else -V[:, 0]
                d, inl = float(c @ n), len(X)
            winkel = math.degrees(math.acos(min(1.0, float(n @ n0))))
            if inl >= R.min_inl and winkel <= R.max_winkel and lo < d < hi:
                return {"n": n, "d": d, "s": cam_h / d, "inl": int(inl), "winkel": round(winkel, 2), "art": "ebene"}
    s = s_ersatz or 1.0
    return {"n": n0, "d": cam_h / s, "s": s, "inl": 0, "winkel": 0.0, "art": "ersatz"}


def frei_oben(frei: np.ndarray, luecke: int) -> np.ndarray:
    """Je Streifen (Pixel-Mehrheit) die oberste Zeile des freien Bodens von unten, Lücken bis luecke Zeilen
    überbrückt; H = kein Boden ganz unten. Gleiche Regel wie comfy_label.mask_rows, ohne Schleife über Zeilen."""
    H, W = frei.shape
    fb = (frei.reshape(H, NCOLS, W // NCOLS).mean(2) >= 0.5)[::-1]       # Index 0 = unterste Zeile
    k = luecke + 1
    cs = np.vstack([np.zeros((1, NCOLS), int), np.cumsum(~fb, 0)])
    lauf = (cs[k:] - cs[:-k]) == k                                         # ab Index i k Zeilen ohne Boden
    stop = np.where(lauf.any(0), lauf.argmax(0), H)
    idx = np.arange(H)[:, None]
    j = np.where(fb & (idx < stop), idx, -1).max(0)
    return np.where(j >= 0, H - 1 - j, H)


def rand_rampe(h: np.ndarray, frei: np.ndarray, hind: np.ndarray, min_zeilen: int) -> np.ndarray:
    """Hindernis-Band am unteren Bildrand, das als Boden zählt: je Streifen (Pixel-Mehrheit) die Zeilen von unten,
    solange Hindernis, wenn direkt darüber mindestens min_zeilen Zeilen Boden folgen und das Band nach unten höher wird
    (Mittel des untersten Viertels über dem obersten). So biegt DA3 den nahen Boden hoch (explore11: bis 9 cm am Rand,
    darüber 100 Zeilen Boden); eine echte Kante am Bildrand ist oben höher oder hat keinen Boden direkt darüber."""
    H, W = h.shape
    sw = W // NCOLS
    rot = (hind.reshape(H, NCOLS, sw).mean(2) >= 0.5)[::-1]                  # Index 0 = unterste Zeile
    gr = (frei.reshape(H, NCOLS, sw).mean(2) >= 0.5)[::-1]
    n = np.where((~rot).any(0), (~rot).argmax(0), H)                         # Länge des Bandes von unten
    hs = np.where(hind, h, np.nan).reshape(H, NCOLS, sw)[::-1]
    out = np.zeros((H, W), bool)
    for c in np.flatnonzero((n > 0) & (n < H)):
        k = int(n[c])
        g = gr[k:k + min_zeilen, c]
        if len(g) < min_zeilen or not g.all():
            continue
        q = max(1, k // 4)
        with np.errstate(all="ignore"):
            if not np.nanmean(hs[:q, c]) > np.nanmean(hs[k - q:k, c]) + 0.001:    # 1 mm: flache Kistendecke bleibt
                continue
        out[H - k:, c * sw:(c + 1) * sw] = hind[H - k:, c * sw:(c + 1) * sw]
    return out


def grenze(P: np.ndarray, ok: np.ndarray, eb: dict, f: float, cx: float, cy: float, R: Regeln) -> tuple[np.ndarray, dict]:
    """Zeile der Bodengrenze je Streifen (NCOLS) und die Karten h (Höhe, m) und Klasse für die Anzeige."""
    H, W = ok.shape
    sw = W // NCOLS
    n, d, s = eb["n"], eb["d"], eb["s"]
    Pz = np.where(ok[..., None], P, 0.0)
    h = s * (d - Pz @ n)
    fw = np.array([0.0, 0.0, 1.0]) - n[2] * n
    fw /= np.linalg.norm(fw)
    D = s * (Pz @ fw)                                                      # Abstand nach vorn (m)
    frei = ok & (h <= R.boden_m)
    hind = ok & (h > R.boden_m) & (h < R.dach_m)
    if R.rand:
        rampe = rand_rampe(h, frei, hind, R.min_px)
        frei |= rampe
        hind &= ~rampe
    top = frei_oben(frei, R.luecke)
    Dh = np.where(hind, D, np.inf).reshape(H, NCOLS, sw).transpose(1, 0, 2).reshape(NCOLS, -1)
    dob = np.partition(Dh, R.min_px - 1, axis=1)[:, R.min_px - 1]
    # Boden je Zeile in der Streifenmitte: Strahl schneidet die Ebene, Abstand nach vorn
    v = np.arange(H, dtype=float)
    u = (np.arange(NCOLS) + 0.5) * sw
    ray = np.stack(np.broadcast_arrays(((u - cx) / f)[None, :], ((v - cy) / f)[:, None], np.ones((1, 1))), -1)
    nr = ray @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        df = np.where(nr > 1e-6, s * (d / nr) * (ray @ fw), np.inf)        # (H, NCOLS)
    rows = np.empty(NCOLS, np.float32)
    fuss = np.zeros(NCOLS, bool)
    for c in range(NCOLS):
        tr = int(top[c])
        if tr >= H:
            rows[c] = H
            continue
        if not dob[c] < df[tr, c]:
            rows[c] = tr
            continue
        g = (v >= tr) & np.isfinite(df[:, c])
        if not g.any() or dob[c] <= df[H - 1, c]:
            rows[c] = H
        else:
            rows[c] = np.interp(dob[c], df[g, c][::-1], v[g][::-1])
        fuss[c] = True
    klasse = np.zeros((H, W), np.uint8)                                    # 0 ungültig, 1 Boden, 2 Hindernis, 4 egal
    klasse[ok] = 4
    klasse[frei] = 1
    klasse[hind] = 2
    return rows, {"h": h, "klasse": klasse, "fuss": fuss}


def ergaenzt_maske(cfg) -> np.ndarray | None:
    """Pixel, die die Entzerrung nur aufgefüllt hat (Quelle außerhalb des Kamerabilds, camera.lens_maps wiederholt dort
    den Rand), oder None ohne Linsenmodell. Am RiderPi rechts: unten 13 px, auf Höhe der Bodenlinie 24-26 px."""
    from xgonav.camera import has_lens, lens_source
    c = cfg.camera
    if not has_lens(c):
        return None
    gx, gy = np.meshgrid(np.arange(c.width, dtype=np.float64), np.arange(c.height, dtype=np.float64))
    p = lens_source(c, np.stack([gx, gy], -1))
    return (p[..., 0] < 0) | (p[..., 0] > c.width - 1) | (p[..., 1] < 0) | (p[..., 1] > c.height - 1)


def waagrecht(rows: np.ndarray, erg: np.ndarray, horizont: float) -> np.ndarray:
    """Streifen, deren Mitte unterhalb des Horizonts überwiegend aus aufgefüllten Pixeln besteht, haben kein eigenes
    Bild: Dort läuft die Linie waagrecht weiter, mit der Zeile des nächsten echten Streifens (rechts von links her,
    links von rechts), dazu je ein Streifen nach innen (explore10/008606: MoGe knickt bei x≈607). Überwiegend, weil der Einbau-Roll auch die untersten ~10 Zeilen der rechten Hälfte auffüllt."""
    H, W = erg.shape
    u = ((np.arange(NCOLS) + 0.5) * W / NCOLS).astype(int)
    leer = erg[min(H - 1, max(0, int(horizont))):, u].mean(0) > 0.5
    if leer.any():                                  # einen Streifen breiter: MoGe knickt schon ~8 px vor dem Rand
        leer = leer | np.r_[leer[1:], False] | np.r_[False, leer[:-1]]
    if leer.all() or not leer.any():
        return rows
    echt = np.flatnonzero(~leer)
    rows = rows.copy()
    for c in np.flatnonzero(leer):
        rows[c] = rows[echt[np.argmin(np.abs(echt - c))]]
    return rows


def label_bild(P: np.ndarray, a: float, cam_h: float, f: float, R: Regeln, s_ersatz: float | None,
               erg: np.ndarray | None = None) -> tuple[np.ndarray, dict, dict]:
    """Ein Bild: Punktwolke (H, W, 3, ungültig = inf) -> Zeilen, Ebene, Karten. erg: aufgefüllte Pixel der
    Entzerrung (ergaenzt_maske); sie zählen nicht für die Ebene, und am Seitenrand läuft die Linie waagrecht weiter.
    In der Grenze bleiben sie: Die aufgefüllte Unterkante (~10 Zeilen) würde sonst den freien Boden von unten
    unterbrechen."""
    H, W = P.shape[:2]
    ok = np.isfinite(P).all(-1) & (P[..., 2] > 0)
    cx, cy = W / 2, H / 2
    horizont = cy + f * math.tan(a)
    eb = ebene(P, ok if erg is None else ok & ~erg, unten(a), cam_h, horizont + 15, R, s_ersatz)
    rows, karten = grenze(P, ok, eb, f, cx, cy, R)
    if erg is not None:
        rows = waagrecht(rows, erg, horizont)
    return rows, eb, karten


class Moge:
    """MoGe-2 auf der GPU. punkte(): BGR-Bilder gleicher Größe -> Punktwolken (B, H, W, 3), ungültig = inf."""

    def __init__(self, name: str = MODELL, tokens: int | None = None):
        import torch
        from moge.model.v2 import MoGeModel
        self.torch = torch
        self.dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.m = MoGeModel.from_pretrained(name).to(self.dev).eval()
        self.tokens = tokens

    def punkte(self, bilder: list, fov_deg: float, ids: list | None = None) -> np.ndarray:
        t = self.torch
        x = t.from_numpy(np.stack([b[..., ::-1] for b in bilder])).to(self.dev).permute(0, 3, 1, 2).float() / 255.0
        with t.inference_mode():
            out = self.m.infer(x, fov_x=float(fov_deg), num_tokens=self.tokens, use_fp16=True)
        return out["points"].float().cpu().numpy()


class Tiefe:
    """Punktwolken aus Tiefenkarten von da3_tiefe.py (<dir>/<fahrt>/<datei>.npz, Tiefe entlang der Blickachse im
    Modellmaß). Strahlen mit f aus dem Öffnungswinkel und Bildmitte W/2, H/2 wie grenze(); Pixel mit conf unter
    conf_min oder ohne Tiefe gelten als ungültig (inf)."""

    def __init__(self, ordner: str, conf_min: float = 0.0):
        self.ordner, self.conf_min = ordner, conf_min

    def punkte(self, bilder: list, fov_deg: float, ids: list) -> np.ndarray:
        H, W = bilder[0].shape[:2]
        f = W / 2 / math.tan(math.radians(fov_deg) / 2)
        u, v = np.meshgrid((np.arange(W) - W / 2) / f, (np.arange(H) - H / 2) / f)
        ray = np.stack([u, v, np.ones_like(u)], -1)
        out = np.empty((len(ids), H, W, 3), np.float32)
        for j, k in enumerate(ids):
            z = np.load(os.path.join(self.ordner, k.rsplit(".", 1)[0] + ".npz"))
            t = z["tiefe"].astype(np.float32)
            schlecht = ~(np.isfinite(t) & (t > 0)) | (z["conf"].astype(np.float32) < self.conf_min)
            out[j] = ray * t[..., None]
            out[j][schlecht] = np.inf
        return out


def quelle(a):
    return Tiefe(a.tiefe, a.conf_min) if a.tiefe else Moge(a.modell, a.tokens)


FARBE = {0: (80, 80, 80), 1: (60, 200, 60), 2: (40, 40, 230), 4: (200, 200, 200)}


def bild(img: np.ndarray, rows: np.ndarray, karten: dict, linien: tuple = ()) -> np.ndarray:
    """Links das Bild mit Linien (magenta Geometrie, dazu linien = ((Zeilen, BGR), ...)), rechts die Klassen
    (grün Boden, rot Hindernis, hellgrau egal, dunkelgrau ungültig)."""
    H, W = img.shape[:2]
    xs = ((np.arange(NCOLS) + 0.5) * W / NCOLS).astype(int)
    links = img.copy()
    for r_, col in tuple(linien) + ((rows, (255, 0, 255)),):
        pts = [(int(x), int(min(y, H - 1))) for x, y in zip(xs, r_) if np.isfinite(y)]
        if len(pts) > 1:
            cv2.polylines(links, [np.array(pts, np.int32)], False, col, 2)
    k = np.zeros_like(img)
    for c, bgr in FARBE.items():
        k[karten["klasse"] == c] = bgr
    rechts = cv2.addWeighted(img, 0.35, k, 0.65, 0)
    pts = [(int(x), int(min(y, H - 1))) for x, y in zip(xs, rows)]
    cv2.polylines(rechts, [np.array(pts, np.int32)], False, (255, 0, 255), 2)
    return np.hstack([links, rechts])


def lade_bilder(pfade: list) -> list:
    with ThreadPoolExecutor(8) as ex:
        return list(ex.map(cv2.imread, pfade))


def regeln_args(ap: argparse.ArgumentParser) -> None:
    for k, v in Regeln().__dict__.items():
        ap.add_argument("--" + k, type=type(v), default=v)
    ap.add_argument("--modell", default=MODELL)
    ap.add_argument("--tokens", type=int, default=None, help="ViT-Tokens (MoGe: 1200-2500); ohne: Höchstwert")
    ap.add_argument("--tiefe", default="", help="Tiefenkarten von da3_tiefe.py statt MoGe")
    ap.add_argument("--conf_min", type=float, default=0.0, help="mit --tiefe: Pixel mit kleinerer conf ungültig")
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--jobs", type=int, default=8, help="Threads für die Rechnung nach dem Modell")


def regeln_aus(a) -> Regeln:
    return Regeln(**{k: getattr(a, k) for k in Regeln().__dict__})


def fahrten(a) -> int:
    """Fahrbilder labeln: <out>/rows.jsonl (setzt fort), danach je Fahrt <out>/npz/<fahrt>.npz für train.py."""
    import comfy_npz as Z
    import nick_bezug as NB
    import stage2 as S2
    import stage3_npz as SN
    from xgonav.config import Config
    from xgonav.geometry import CamGeometry
    R = regeln_aus(a)
    cfg = Config.load(a.config)
    os.makedirs(a.out, exist_ok=True)
    todo: list = []
    if a.ids:
        todo = open(a.ids, encoding="utf-8").read().split()
    for run in a.runs.split(",") if a.runs else ():
        stems = sorted(f[:-4] for f in os.listdir(os.path.join(a.data, run)) if f.endswith(".jpg"))
        teil = [s for s in stems if not s.isdigit() or int(s) % a.jedes == 0]
        aus = S2.stehend(S2.posen(a.data, run), teil, a.steht_m, a.steht_grad) if a.steht_m > 0 else set()
        todo += ["%s/%s.jpg" % (run, s) for s in teil if s not in aus]
    rows_path = os.path.join(a.out, "rows.jsonl")
    fertig: dict = {}
    if os.path.exists(rows_path):
        for z in open(rows_path, encoding="utf-8"):
            try:
                e = json.loads(z)
                fertig[e["id"]] = e
            except ValueError:
                pass
    je_fahrt: dict = {}
    for k in todo:
        je_fahrt.setdefault(k.split("/")[0], []).append(k)
    model = None
    erg = ergaenzt_maske(cfg) if a.entzerrt else None
    if erg is not None:
        print("aufgefüllte Pixel der Entzerrung: %d, Linie dort waagrecht" % int(erg.sum()))
    pool = ThreadPoolExecutor(a.jobs)
    bdir = os.path.join(a.out, "bilder")
    gezeigt = 0
    with open(rows_path, "a", encoding="utf-8") as fo:
        for run, ks in je_fahrt.items():
            src = os.path.join(a.data, run)
            meta = Z.load_meta(src)
            hoehe = SN.koerperhoehe(src)
            skala = [fertig[k]["s"] for k in ks if k in fertig and fertig[k]["art"] == "ebene"]
            offen = [k for k in ks if k not in fertig]
            t0 = time.time()
            for i in range(0, len(offen), a.batch):
                teil = offen[i:i + a.batch]
                if model is None:
                    model = quelle(a)
                imgs = lade_bilder([os.path.join(a.data, k) for k in teil])
                geo0 = CamGeometry.from_cfg(cfg, hoehe)
                P = model.punkte(imgs, geo0.hfov_deg, teil)
                s_e = float(np.median(skala)) if skala else None

                def eins(j):
                    m = meta.get(teil[j].split("/", 1)[1], {})
                    hb = m.get("height_mm") if isinstance(m.get("height_mm"), (int, float)) else hoehe
                    geo = CamGeometry.from_cfg(cfg, hb)
                    a_ = math.radians(geo.tilt_deg) + NB.pitch_dev(run, m)
                    return label_bild(P[j], a_, geo.cam_h, geo.f, R, s_e, erg)

                for k, img, (rows, eb, karten) in zip(teil, imgs, pool.map(eins, range(len(teil)))):
                    if eb["art"] == "ebene":
                        skala.append(eb["s"])
                    e = {"id": k, "rows": [round(float(x), 2) for x in rows], "s": round(eb["s"], 4),
                         "inl": eb["inl"], "winkel": eb["winkel"], "art": eb["art"]}
                    fo.write(json.dumps(e) + "\n")
                    fertig[k] = e
                    if gezeigt < a.bilder:
                        os.makedirs(bdir, exist_ok=True)
                        cv2.imwrite(os.path.join(bdir, k.replace("/", "__")), bild(img, rows, karten))
                        gezeigt += 1
                fo.flush()
                n = min(i + a.batch, len(offen))
                print("%-10s %5d/%d  %.1f Bilder/s" % (run, n, len(offen), n / (time.time() - t0)), flush=True)
            frames = {k.split("/", 1)[1]: np.array(fertig[k]["rows"], np.float32) for k in ks
                      if k in fertig and fertig[k]["art"] == "ebene"}          # Ersatzbilder verworfen
            d, count = Z.build_run(run, frames, SN.geometrie(cfg, hoehe), os.path.abspath(src), a.wohnung)
            os.makedirs(os.path.join(a.out, "npz"), exist_ok=True)
            tmp = os.path.join(a.out, "npz", run + ".tmp.npz")
            np.savez_compressed(tmp, **d)
            os.replace(tmp, os.path.join(a.out, "npz", run + ".npz"))
            arten: dict = {}
            for k in ks:
                if k in fertig:
                    arten[fertig[k]["art"]] = arten.get(fertig[k]["art"], 0) + 1
            print("%-10s %5d Labels, verworfen: %s, Ebene: %s (ersatz nicht in der Label-Datei)" % (
                run, len(d["files"]), count or "-", arten), flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    q = sub.add_parser("fahrten", help="Fahrbilder labeln, Label-Dateien für train.py")
    q.add_argument("--runs", default="")
    q.add_argument("--ids", default="", help="Datei mit fahrt/datei.jpg je Zeile (z. B. aus ids_fahrt.py)")
    q.add_argument("--data", default=os.path.expanduser("~/floornet/data/rider/fahrt"))
    q.add_argument("--jedes", type=int, default=2)
    q.add_argument("--steht_m", type=float, default=0.03, help="Standbilder auslassen wie stage2.py (0 = alle)")
    q.add_argument("--steht_grad", type=float, default=3.0)
    q.add_argument("--config", default=None, help="TOML des Riders für die Kamerageometrie")
    q.add_argument("--wohnung", default="a")
    q.add_argument("--entzerrt", type=int, default=1, help="Bilder sind entzerrt: aufgefüllten Rand der Linsentabelle "
                                                           "auslassen, Linie dort waagrecht (0 = aus)")
    q.add_argument("--bilder", type=int, default=40)
    q.add_argument("--out", required=True)
    regeln_args(q)
    a = ap.parse_args()
    return fahrten(a)


if __name__ == "__main__":
    sys.exit(main())
