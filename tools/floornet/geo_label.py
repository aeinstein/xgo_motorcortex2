#!/usr/bin/env python3
"""FloorNet-Labels aus Geometrie statt aus einem Bildbearbeitungsmodell: MoGe-2 (Ruicheng/moge-2-vitl-normal) schätzt
je Pixel einen 3D-Punkt, die Bodenebene wird mit der bekannten Kamerahöhe eingepasst, und je 8-px-Streifen gilt die
Handlinien-Regel als Rechnung: Grenze = erster Punkt, an dem ein 18 cm hoher Rider anstößt.

  # Prüfen gegen die Handlinien (Prüfsatz 3), daneben dieselbe Tabelle für vorhandene Labels (z. B. Prompt 11):
  python geo_label.py pruef --hand pruefsatz.json --labels labels_u3/ --hoehe pruefsatz3/hoehe.json \\
      --vergleich stage3_p11_labels/ --out geo_p3

  # Viele Fahrbilder -> rows.jsonl und Label-Dateien für train.py (Filter wie comfy_npz.py):
  python geo_label.py fahrten --runs explore10,explore11 --jedes 2 --config rider.toml --out geo_lab

Je Pixel, Höhe h über der eingepassten Ebene:
  Boden       -stufe_m <= h <= boden_m      (Fugen, Spiegelungen ohne Tiefe, flache Teppiche)
  Hindernis   boden_m < h < dach_m          (Sockelleiste, Stufe nach oben, Sofakante unter 18 cm)
  Absturz     h < -stufe_m                  (Stufe nach unten; --stufe_m 0 schaltet das ab)
  egal        h >= dach_m                   (Tischplatte, Sofa mit genug Luft darunter)
Je Streifen: freier Boden von unten (Lücken bis --luecke Zeilen überbrückt, wie comfy_label.mask_rows), dazu das
nächste Hindernis nach vorn (das --min_px-nächste Pixel des Streifens). Liegt das Hindernis näher als das Ende des
freien Bodens (Boden sichtbar unter einer niedrigen Sofakante), zählt sein Fußpunkt: die Zeile, in der der Boden
genau diese Entfernung hat.

Maßstab: MoGe liefert Meter, bei 13-15 cm Kamerahöhe aber nicht verlässlich. Deshalb wird die Ebene im Modellmaß
eingepasst und alles so skaliert, dass sie genau cam_h unter der Kamera liegt. Ohne eigene Ebene (Wand direkt davor)
rechnet das Bild mit IMU-Nick und dem Median der Skala der übrigen Bilder dieser Fahrt (art "ersatz"), nur für die
Anzeige: Solche Bilder kommen nicht in die Label-Datei und nicht in die Prüftabelle (explore10-12 gegen p11 am
2026-11-18: der Ersatzweg hält sichtbaren Boden für Absturz, Linie ganz unten).

Nicht abgedeckt: Teppiche sind nach der Handregel Hindernis, geometrisch aber Boden. Spiegelnder Boden kann
Spiegelbilder unter die Ebene legen (dann Absturz = fälschlich blockiert).

Braucht auf kimmy: pip install git+https://github.com/microsoft/MoGe.git (torch mit CUDA ist schon da).
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
    stufe_m: float = 0.03       # tiefer als die Ebene: Absturz (0 = aus)
    luecke: int = 6             # Zeilen ohne Boden, die der freie Lauf überbrückt
    min_px: int = 16            # Pixel, die ein Hindernis im Streifen haben muss (2 Zeilen à 8 px)
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
    stufe = R.stufe_m if R.stufe_m > 0 else np.inf
    frei = ok & (h >= -stufe) & (h <= R.boden_m)
    hind = ok & (((h > R.boden_m) & (h < R.dach_m)) | (h < -stufe))
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
    klasse = np.zeros((H, W), np.uint8)                                    # 0 ungültig, 1 Boden, 2 Hindernis, 3 Absturz, 4 egal
    klasse[ok] = 4
    klasse[frei] = 1
    klasse[hind & (h > 0)] = 2
    klasse[hind & (h < 0)] = 3
    return rows, {"h": h, "klasse": klasse, "fuss": fuss}


def label_bild(P: np.ndarray, a: float, cam_h: float, f: float, R: Regeln, s_ersatz: float | None) -> tuple[np.ndarray, dict, dict]:
    """Ein Bild: Punktwolke (H, W, 3, ungültig = inf) -> Zeilen, Ebene, Karten."""
    H, W = P.shape[:2]
    ok = np.isfinite(P).all(-1) & (P[..., 2] > 0)
    cx, cy = W / 2, H / 2
    horizont = cy + f * math.tan(a)
    eb = ebene(P, ok, unten(a), cam_h, horizont + 15, R, s_ersatz)
    rows, karten = grenze(P, ok, eb, f, cx, cy, R)
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

    def punkte(self, bilder: list, fov_deg: float) -> np.ndarray:
        t = self.torch
        x = t.from_numpy(np.stack([b[..., ::-1] for b in bilder])).to(self.dev).permute(0, 3, 1, 2).float() / 255.0
        with t.inference_mode():
            out = self.m.infer(x, fov_x=float(fov_deg), num_tokens=self.tokens, use_fp16=True)
        return out["points"].float().cpu().numpy()


FARBE = {0: (80, 80, 80), 1: (60, 200, 60), 2: (40, 40, 230), 3: (230, 120, 30), 4: (200, 200, 200)}


def bild(img: np.ndarray, rows: np.ndarray, karten: dict, linien: tuple = ()) -> np.ndarray:
    """Links das Bild mit Linien (magenta Geometrie, dazu linien = ((Zeilen, BGR), ...)), rechts die Klassen
    (grün Boden, rot Hindernis, blau Absturz, hellgrau egal, dunkelgrau ungültig)."""
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


def tabelle(eintraege: list, titel: str, far: float = 0.15) -> None:
    """Wie comfy_label.py / eval_hand.py: Fehler in Metern je Spalte, Bänder nach der Entfernung der Handlinie.
    eintraege: (Zeilen, Handzeilen, gültig, Nick (1,), geom)."""
    import comfy_label as C
    print("\n== %s, %d Bilder" % (titel, len(eintraege)))
    tot = np.zeros(3)
    for lo, hi in ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5)):
        n = fr = bl = 0
        meds = []
        for pr, hr, ok, pitch, geo in eintraege:
            dh = C.row_dist(geo, hr[None], pitch)[0]
            dp = C.row_dist(geo, pr[None], pitch)[0]
            m = ok & (dh >= lo) & (dh < hi) & np.isfinite(pr)
            e = dp - dh
            n += int(m.sum())
            fr += int(((e > far) & m).sum())
            bl += int(((e < -far) & m).sum())
            meds.append(np.abs(np.nan_to_num(e[m], posinf=9.0)))
        if hi <= 1.0:
            tot += (n, fr, bl)
        med = float(np.median(np.concatenate(meds)) * 100) if n else float("nan")
        print("%3.1f-%3.1f m | n%5d frei %5.1f%% block %5.1f%% med %4.0f cm" % (
            lo, hi, n, 100 * fr / max(n, 1), 100 * bl / max(n, 1), med))
    print("unter 1.0 m: fälschlich frei %.1f%%, fälschlich blockiert %.1f%%" % (
        100 * tot[1] / max(tot[0], 1), 100 * tot[2] / max(tot[0], 1)))


def regeln_args(ap: argparse.ArgumentParser) -> None:
    for k, v in Regeln().__dict__.items():
        ap.add_argument("--" + k, type=type(v), default=v)
    ap.add_argument("--modell", default=MODELL)
    ap.add_argument("--tokens", type=int, default=None, help="ViT-Tokens (MoGe: 1200-2500); ohne: Höchstwert")
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--jobs", type=int, default=8, help="Threads für die Rechnung nach dem Modell")


def regeln_aus(a) -> Regeln:
    return Regeln(**{k: getattr(a, k) for k in Regeln().__dict__})


def pruef(a) -> int:
    """Geometrie-Labels für die Handbilder rechnen (ergebnis.json, setzt fort) und gegen die Handlinien messen."""
    import comfy_label as C
    R = regeln_aus(a)
    os.makedirs(a.out, exist_ok=True)
    hd = json.load(open(a.hand, encoding="utf-8"))["bilder"]
    ids = sorted(k for k, e in hd.items() if e.get("fertig") and not e.get("unbrauchbar"))
    if a.ids:
        ids = [x.strip() for x in open(a.ids, encoding="utf-8") if x.strip()]
    elif a.n:
        random.Random(a.seed).shuffle(ids)
        ids = sorted(ids[:a.n])
    body = {}
    if a.hoehe:
        for g, lst in json.load(open(a.hoehe, encoding="utf-8")).items():
            try:
                body.update({i: float(g) for i in lst})
            except ValueError:
                pass
        n0 = len(ids)
        ids = [k for k in ids if k in body]
        print("Höhe unklar, ausgelassen:", n0 - len(ids))
    lab: dict = {}

    def npz(run):
        if run not in lab:
            d = np.load(os.path.join(a.labels, run + ".npz"), allow_pickle=False)
            lab[run] = (d, {str(x): i for i, x in enumerate(d["files"])})
        return lab[run]

    def info(k):
        run, f = k.split("/", 1)
        d, pos = npz(run)
        geo = json.loads(str(d["geom"]))
        if k in body:                                   # Labels rechnen mit 85 mm Körperhöhe (eval_hand.py)
            geo["cam_h"] = geo["cam_h"] + (body[k] - 85.0) / 1000.0
        return os.path.join(str(d["source_dir"]), f), geo, np.array([d["pitch"][pos[f]]], np.float64)

    res_path = os.path.join(a.out, "ergebnis.json")
    res = json.load(open(res_path, encoding="utf-8")) if os.path.exists(res_path) else {}
    todo = [k for k in ids if k not in res]
    if todo:
        model = Moge(a.modell, a.tokens)
        skala: dict = {}
        bdir = os.path.join(a.out, "bilder")
        os.makedirs(bdir, exist_ok=True)
        t0, fertig = time.time(), 0
        pool = ThreadPoolExecutor(a.jobs)                  # numpy gibt die GIL in den großen Rechnungen frei
        for i in range(0, len(todo), a.batch):
            teil = todo[i:i + a.batch]
            inf = [info(k) for k in teil]
            imgs = lade_bilder([p for p, _, _ in inf])
            ts = time.time()
            P = model.punkte(imgs, inf[0][1]["hfov_deg"])       # alle Fahrten einer Wohnung: gleiche Kamera
            tm = (time.time() - ts) / len(teil)
            def eins(j):
                run, (_, geo, pitch) = teil[j].split("/")[0], inf[j]
                s_e = float(np.median(skala[run])) if skala.get(run) else None
                return label_bild(P[j], math.radians(geo["tilt_deg"]) + float(pitch[0]), geo["cam_h"], geo["f"], R, s_e)

            for k, img, (rows, eb, karten) in zip(teil, imgs, pool.map(eins, range(len(teil)))):
                run = k.split("/")[0]
                if eb["art"] == "ebene":
                    skala.setdefault(run, []).append(eb["s"])
                res[k] = {"rows": [round(float(x), 2) for x in rows], "s": round(eb["s"], 4), "inl": eb["inl"],
                          "winkel": eb["winkel"], "art": eb["art"], "ms": round(1000 * tm, 1)}
                if fertig < a.bilder:
                    hr, _ = C.polyline_rows(hd[k]["pts"])
                    cv2.imwrite(os.path.join(bdir, k.replace("/", "__").rsplit(".", 1)[0] + ".jpg"),
                                bild(img, rows, karten, ((hr, (0, 220, 0)),)))
                fertig += 1
            json.dump(res, open(res_path, "w", encoding="utf-8"))
            print("%5d/%d  %.0f ms/Bild Modell, gesamt %.1f Bilder/s" % (
                fertig, len(todo), 1000 * tm, fertig / (time.time() - t0)), flush=True)
    arten: dict = {}
    for k in ids:
        arten[res[k]["art"]] = arten.get(res[k]["art"], 0) + 1
    print("\nEbene je Bild:", arten, " Skala Median %.3f" % float(np.median([res[k]["s"] for k in ids])))

    vgl: dict = {}
    if a.vergleich:
        for k in ids:
            run, f = k.split("/", 1)
            p = os.path.join(a.vergleich, run + ".npz")
            if run not in vgl:
                vgl[run] = None
                if os.path.exists(p):
                    z = np.load(p, allow_pickle=False)
                    vgl[run] = {str(x): r for x, r in zip(z["files"], z["row"])}
    gemeinsam = [k for k in ids if res[k]["art"] == "ebene" and
                 (not vgl or (vgl.get(k.split("/")[0]) or {}).get(k.split("/", 1)[1]) is not None)]
    print("ohne eigene Ebene, verworfen: %d" % sum(res[k]["art"] != "ebene" for k in ids))
    if vgl:
        print("Bilder mit beiden Labels: %d von %d" % (len(gemeinsam), len(ids)))

    def eintraege(quelle):
        out = []
        for k in gemeinsam:
            _, geo, pitch = info(k)
            hr, ok = C.polyline_rows(hd[k]["pts"])
            out.append((quelle(k), hr, ok, pitch, geo))
        return out

    tabelle(eintraege(lambda k: np.array(res[k]["rows"], np.float32)), "Geometrie (%s)" % a.modell)
    if vgl:
        tabelle(eintraege(lambda k: np.asarray(vgl[k.split("/")[0]][k.split("/", 1)[1]], np.float32)),
                "Vergleich %s" % a.vergleich)
    return 0


def fahrten(a) -> int:
    """Fahrbilder labeln: <out>/rows.jsonl (setzt fort), danach je Fahrt <out>/npz/<fahrt>.npz für train.py."""
    import comfy_npz as Z
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
                    model = Moge(a.modell, a.tokens)
                imgs = lade_bilder([os.path.join(a.data, k) for k in teil])
                geo0 = CamGeometry.from_cfg(cfg, hoehe)
                P = model.punkte(imgs, geo0.hfov_deg)
                s_e = float(np.median(skala)) if skala else None

                def eins(j):
                    m = meta.get(teil[j].split("/", 1)[1], {})
                    hb = m.get("height_mm") if isinstance(m.get("height_mm"), (int, float)) else hoehe
                    geo = CamGeometry.from_cfg(cfg, hb)
                    a_ = math.radians(geo.tilt_deg) + float(m.get("pitch_dev", 0.0))
                    return label_bild(P[j], a_, geo.cam_h, geo.f, R, s_e)

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
    p = sub.add_parser("pruef", help="gegen Handlinien messen")
    p.add_argument("--hand", required=True)
    p.add_argument("--labels", required=True, help="Label-Dateien der Fahrten (Quelle, Geometrie, Nick je Bild)")
    p.add_argument("--hoehe", default="", help="hoehe.json (Gruppe = Körperhöhe mm -> IDs), wie comfy_label.py")
    p.add_argument("--vergleich", default="", help="Label-Dateien eines anderen Lehrers (z. B. Stage3 Prompt 11), "
                                                    "gleiche Tabelle auf denselben Bildern")
    p.add_argument("--ids", default="")
    p.add_argument("--n", type=int, default=0, help="nur so viele zufällige Handbilder (0 = alle)")
    p.add_argument("--seed", type=int, default=3)
    p.add_argument("--bilder", type=int, default=60, help="so viele Vergleichsbilder schreiben")
    p.add_argument("--out", required=True)
    regeln_args(p)
    q = sub.add_parser("fahrten", help="Fahrbilder labeln, Label-Dateien für train.py")
    q.add_argument("--runs", default="")
    q.add_argument("--ids", default="", help="Datei mit fahrt/datei.jpg je Zeile (z. B. aus ids_fahrt.py)")
    q.add_argument("--data", default=os.path.expanduser("~/floornet/data/rider/fahrt"))
    q.add_argument("--jedes", type=int, default=2)
    q.add_argument("--steht_m", type=float, default=0.03, help="Standbilder auslassen wie stage2.py (0 = alle)")
    q.add_argument("--steht_grad", type=float, default=3.0)
    q.add_argument("--config", default=None, help="TOML des Riders für die Kamerageometrie")
    q.add_argument("--wohnung", default="a")
    q.add_argument("--bilder", type=int, default=40)
    q.add_argument("--out", required=True)
    regeln_args(q)
    a = ap.parse_args()
    return pruef(a) if a.cmd == "pruef" else fahrten(a)


if __name__ == "__main__":
    sys.exit(main())
