#!/usr/bin/env python3
"""FloorNet-Bestand auf kimmy an Ort und Stelle von Fassung 1 auf den aktuellen Linsenblock nachziehen.

  python tools/floornet/entzerren_bestand.py ordner bild   WURZEL [...]   # Kamerabilder 640x480, bilinear
  python tools/floornet/entzerren_bestand.py ordner maske  WURZEL [...]   # Masken 640x480, nächster Nachbar
  python tools/floornet/entzerren_bestand.py ordner stage3 WURZEL [...]   # Stage3 neu aus der fertigen Stage2 (info.json)
  python tools/floornet/entzerren_bestand.py ordner gruen  WURZEL [...]   # ComfyUI-Grünbilder (4:3, jede Größe)
  python tools/floornet/entzerren_bestand.py npz NPZ_ORDNER [...] --zuordnung npz_zuordnung.json

Fassung 1 (2026-11-18 vormittags): nur radial entzerrt; Ordner mit .entzerrt ohne "lens_id", meta.jsonl
"entzerrt": true, npz ohne lens_id. Ziel: camera.lens_id(Config().camera) mit Einbau-Roll und -Gier; Ordner, Bilder
(meta), npz und Grünbilder (PNG-Text) tragen danach diese Kennung. Rohbilder gibt es im Bestand nicht mehr; neue
Aufnahmen entzerrt der Rider selbst.

Abbruchsicher, nie doppelt: neue Dateien erst in <dir>/.entz_neu, dann .entz_bereit, dann über die alten geschoben,
zuletzt .entzerrt mit lens_id. Links auf Bilder werden nie bearbeitet, nur ihr Ziel. Zeilen-Labels in npz werden aus
der fertigen Stage3-Maske neu gebildet (Zuordnung aus dem ersten Lauf), fehlt sie, als Kurve umgerechnet.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xgonav import camera as K  # noqa: E402
from xgonav.config import Config  # noqa: E402

CAM = Config().camera
W, H = CAM.width, CAM.height
ZIEL = K.lens_id(CAM)
BILD_ENDUNG = (".jpg", ".jpeg", ".png")
JPEG_Q = 95
NEU, BEREIT, FERTIG = ".entz_neu", ".entz_bereit", ".entzerrt"
_TAB: dict = {}


def _quelle_v1() -> np.ndarray:
    """Je Zielpixel die Lesestelle im Bild der Fassung 1: Zielpixel -> Rohpixel -> Radialverzeichnung vorwärts."""
    gx, gy = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    q = K.lens_radial(CAM, K.lens_source(CAM, np.stack([gx, gy], -1)))
    return np.stack([np.clip(q[..., 0], 0, W - 1), np.clip(q[..., 1], 0, H - 1)], -1).astype(np.float32)


def _tab(w: int = W, h: int = H):
    """(Festkomma-Tabellen bilinear, Tabellen nächster Nachbar) für Bilder w x h, die das Kamerabild gestreckt zeigen."""
    if (w, h) not in _TAB:
        q = _quelle_v1()
        if (w, h) != (W, H):                    # Grünbilder 1184x880: Tabelle strecken, Koordinaten umrechnen
            sx, sy = W / w, H / h
            u = ((np.arange(w, dtype=np.float32) + 0.5) * sx - 0.5)[None, :].repeat(h, 0)
            v = ((np.arange(h, dtype=np.float32) + 0.5) * sy - 0.5)[:, None].repeat(w, 1)
            q = np.stack([(cv2.remap(q[..., k], u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE) + 0.5) / s - 0.5
                          for k, s in ((0, sx), (1, sy))], -1)
        lin = cv2.convertMaps(q[..., 0], q[..., 1], cv2.CV_16SC2)
        _TAB[(w, h)] = (lin, (np.rint(q[..., 0]), np.rint(q[..., 1])))
    return _TAB[(w, h)]


def _eine(job) -> str:
    """(art, quelle, ziel, stage2) -> "" oder Grund fürs Überspringen."""
    art, src, dst, s2 = job
    try:
        if art == "gruen":
            from PIL import Image, PngImagePlugin
            im = Image.open(src)
            if abs(im.width / im.height - W / H) > 0.02 or im.width < W:
                return "kein Kamerabild %dx%d" % (im.width, im.height)      # Collagen, Vergleichstafeln
            a = np.asarray(im.convert("RGB"))
            (m1, m2), _ = _tab(im.width, im.height)
            meta = PngImagePlugin.PngInfo()
            for k, v in im.info.items():
                if isinstance(v, str) and k != "entzerrt":
                    meta.add_text(k, v)
            meta.add_text("entzerrt", ZIEL)
            Image.fromarray(cv2.remap(a, m1, m2, cv2.INTER_LINEAR)).save(dst, pnginfo=meta)
            return ""
        if art == "stage3" and s2 and os.path.exists(s2):
            import stage3 as S3
            out = S3.from_bottom(cv2.imread(s2, cv2.IMREAD_GRAYSCALE) > 0).astype(np.uint8) * 255
        else:                                   # bild, maske, stage3 ohne Stage2
            img = cv2.imread(src, cv2.IMREAD_UNCHANGED)
            if img is None or img.shape[:2] != (H, W):
                return "nicht lesbar" if img is None else "Größe %dx%d" % (img.shape[1], img.shape[0])
            (m1, m2), (nx, ny) = _tab()
            out = cv2.remap(img, m1, m2, cv2.INTER_LINEAR) if art == "bild" else cv2.remap(img, nx, ny, cv2.INTER_NEAREST)
        par = [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q] if dst.lower().endswith((".jpg", ".jpeg")) else []
        return "" if cv2.imwrite(dst, out, par) and os.path.getsize(dst) > 0 else "Schreiben fehlgeschlagen"
    except Exception as e:  # noqa: BLE001
        return "Fehler %s" % e


def stand(d: str) -> str:
    """ "roh" (keine Markierung), "v1" (ohne lens_id), "fertig" (lens_id == ZIEL), sonst "anders"."""
    try:
        with open(os.path.join(d, FERTIG), encoding="utf-8") as fh:
            lid = json.load(fh).get("lens_id")
    except FileNotFoundError:
        return "roh"
    except ValueError:
        lid = None
    return "v1" if not lid else "fertig" if lid == ZIEL else "anders"


def _meta(d: str, neu: set) -> int:
    """meta.jsonl: entzerrt = ZIEL für die neu entzerrten Bilder und für Links in fertige Verzeichnisse."""
    p = os.path.join(d, "meta.jsonl")
    if not os.path.isfile(p):
        return 0
    with open(p, encoding="utf-8") as f:
        zeilen = f.read().splitlines()
    n = 0
    for i, ln in enumerate(zeilen):
        try:
            m = json.loads(ln)
        except ValueError:
            continue
        rp = os.path.realpath(os.path.join(d, m.get("file", "")))
        if m.get("file") in neu or (os.path.exists(rp) and stand(os.path.dirname(rp)) == "fertig"):
            cam = m.get("cam") if isinstance(m.get("cam"), dict) else {}
            if cam.get("entzerrt") != ZIEL:
                cam["entzerrt"], m["cam"] = ZIEL, cam
                zeilen[i] = json.dumps(m, ensure_ascii=False)
                n += 1
    with open(p + ".tmp", "w", encoding="utf-8") as f:
        f.write("\n".join(zeilen) + "\n")
    os.replace(p + ".tmp", p)
    return n


def _stage2_von(d: str) -> str | None:
    """Stage3-Verzeichnis <wurzel>/<fahrt> -> Stage2-Verzeichnis derselben Fahrt (aus <wurzel>/info.json "quelle")."""
    try:
        with open(os.path.join(os.path.dirname(d), "info.json"), encoding="utf-8") as fh:
            q = os.path.join(os.path.expanduser(json.load(fh)["quelle"]), os.path.basename(d))
    except (OSError, ValueError, KeyError):
        return None
    return q if os.path.isdir(q) else None


def einheit(d: str, art: str, pool) -> dict:
    """Ein Verzeichnis (nicht rekursiv) von Fassung 1 auf ZIEL."""
    rep = {"neu": 0, "links": 0, "uebersprungen": {}, "status": ""}
    st = stand(d) if os.path.isdir(d) else "gelöscht"
    if st != "v1" and not os.path.exists(os.path.join(d, BEREIT)):
        rep["status"] = "Stand " + st
        return rep
    s2 = _stage2_von(d) if art == "stage3" else None
    if s2 and stand(s2) != "fertig":
        rep["status"] = "warte auf Stage2 " + s2
        return rep
    neu = os.path.join(d, NEU)
    namen = sorted(f for f in os.listdir(d) if f.lower().endswith(BILD_ENDUNG) and not f.startswith("."))
    echt = [f for f in namen if not os.path.islink(os.path.join(d, f))]
    rep["links"] = len(namen) - len(echt)
    if not os.path.exists(os.path.join(d, BEREIT)):
        shutil.rmtree(neu, ignore_errors=True)
        os.makedirs(neu)
        res = pool.map(_eine, [(art, os.path.join(d, f), os.path.join(neu, f), s2 and os.path.join(s2, f)) for f in echt],
                       chunksize=32)
        for f, why in zip(echt, res):
            if why:
                rep["uebersprungen"][f] = why
                if os.path.exists(os.path.join(neu, f)):
                    os.remove(os.path.join(neu, f))
        with open(os.path.join(d, BEREIT), "w") as fh:
            json.dump(rep["uebersprungen"], fh, ensure_ascii=False)
    else:
        with open(os.path.join(d, BEREIT)) as fh:
            rep["uebersprungen"] = json.load(fh)
    for f in os.listdir(neu):
        os.replace(os.path.join(neu, f), os.path.join(d, f))
    os.rmdir(neu)
    fertig = {f for f in echt if f not in rep["uebersprungen"]}
    rep["neu"] = len(fertig)
    with open(os.path.join(d, FERTIG), "w") as fh:
        json.dump({"art": art, "lens_id": ZIEL, "von": "v1", "zeit": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "neu": rep["neu"], "uebersprungen": rep["uebersprungen"], "jpeg_q": JPEG_Q}, fh, ensure_ascii=False)
    if art == "bild":
        rep["meta"] = _meta(d, fertig)
    os.remove(os.path.join(d, BEREIT))
    rep["status"] = "entzerrt"
    return rep


def verzeichnisse(wurzeln: list[str]) -> list[str]:
    """Echte Verzeichnisse mit Bildern unter den Wurzeln (Verzeichnis-Links nicht verfolgt); solche nur mit Links
    zuletzt, damit ihre Ziele schon fertig sind."""
    echt, nur_links, seen = [], [], set()
    for w in wurzeln:
        for wurzel, dirs, files in os.walk(w):
            dirs[:] = sorted(x for x in dirs if not x.startswith(".") and not os.path.islink(os.path.join(wurzel, x)))
            b = [f for f in files if f.lower().endswith(BILD_ENDUNG) and not f.startswith(".")]
            if not b or os.path.realpath(wurzel) in seen:
                continue
            seen.add(os.path.realpath(wurzel))
            (echt if any(not os.path.islink(os.path.join(wurzel, f)) for f in b) else nur_links).append(wurzel)
    return echt + nur_links


def cmd_ordner(a) -> int:
    dirs = verzeichnisse(a.wurzeln)
    print("%d Verzeichnisse, Art %s, Fassung 1 -> %s" % (len(dirs), a.art, ZIEL), flush=True)
    t0, summe = time.time(), 0
    with Pool(a.jobs, initializer=cv2.setNumThreads, initargs=(1,)) as pool:
        for i, d in enumerate(dirs):
            rep = einheit(d, a.art, pool)
            summe += rep["neu"]
            gr: dict = {}
            for why in rep["uebersprungen"].values():
                gr[why] = gr.get(why, 0) + 1
            print("[%d/%d] %-16s %6d neu, %d Links, meta %s, übersprungen %s  %s" % (
                i + 1, len(dirs), rep["status"], rep["neu"], rep["links"], rep.get("meta", "-"), gr or "-", d), flush=True)
    print("FERTIG %d Dateien, %.0f s" % (summe, time.time() - t0), flush=True)
    return 0


def kurve(row: np.ndarray, kind: np.ndarray | None = None):
    """Zeilen je Spalte (N x n, Fassung 1) als Grenzkurve auf ZIEL drehen und an den Spaltenmitten neu abtasten.
    NaN bleibt NaN; kind nimmt die Art der nächsten umgerechneten Spalte."""
    n = row.shape[1]
    xc = (np.arange(n) + 0.5) * W / n
    out = np.full(row.shape, np.nan, np.float32)
    kout = None if kind is None else kind.copy()
    Hr = K.lens_rot(CAM)
    for k in range(row.shape[0]):
        ok = np.isfinite(row[k])
        if not ok.any():
            continue
        p = K._homog(Hr, np.stack([xc[ok], row[k][ok].astype(float)], 1))
        o = np.argsort(p[:, 0], kind="stable")
        alle_x = xc.copy()
        alle_x[ok] = p[:, 0]
        naechste = np.abs(alle_x[None, :] - xc[:, None]).argmin(1)
        neu = np.interp(xc, p[o, 0], p[o, 1])
        neu[~ok[naechste]] = np.nan
        out[k] = np.clip(neu, 0, H)
        if kout is not None:
            kout[k] = kind[k][naechste]
    return out, kout


def _rows_of(path):
    import comfy_label as C
    m = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    return None if m is None or m.shape != (C.H, C.W) else C.mask_rows(m > 0)


def cmd_npz(a) -> int:
    import glob
    from comfy_npz import EXACT, IGNORE, ROW_MIN
    with open(a.zuordnung) as fh:
        zu = json.load(fh)
    seen = set()
    with Pool(a.jobs) as pool:
        for f in sorted(p for nd in a.npz for p in glob.glob(os.path.join(nd, "*.npz"))):
            rp = os.path.realpath(f)
            if rp in seen:
                continue
            seen.add(rp)
            d = dict(np.load(rp, allow_pickle=False))
            if "row" not in d or str(d.get("lens_id", "")) == ZIEL or "entzerrt" not in d:
                print("%-50s ausgelassen (%s)" % (f, "keine Zeilen" if "row" not in d else "nicht Fassung 1"), flush=True)
                continue
            m, run = (zu.get(rp) or {}).get("masken"), str(d["run"])
            if m and not os.path.isdir(os.path.join(m, run)):
                m = None                                        # Masken gelöscht: Kurve
            if m and stand(os.path.join(m, run)) != "fertig":
                print("%-50s Masken %s/%s noch nicht fertig: ausgelassen" % (f, m, run), flush=True)
                continue
            if m:
                rs = pool.map(_rows_of, [os.path.join(m, run, str(x)[:-4] + ".png") for x in d["files"]], chunksize=32)
                fehlt = [i for i, r in enumerate(rs) if r is None]
                ersatz = kurve(d["row"][fehlt])[0] if fehlt else None
                row = np.stack([r if r is not None else ersatz[fehlt.index(i)] for i, r in enumerate(rs)]) if rs else d["row"]
                d["row"], d["kind"] = row.astype(np.float32), np.where(row < ROW_MIN, IGNORE, EXACT).astype(np.uint8)
                how = "Maske %s (%d als Kurve)" % (m, len(fehlt))
            else:
                d["row"], d["kind"] = kurve(d["row"], d["kind"])
                how = "Kurve"
            if "kanten_row" in d and np.isfinite(d["kanten_row"]).any():
                d["kanten_row"] = kurve(d["kanten_row"])[0]
            d["entzerrt"], d["lens_id"] = np.array(how), np.array(ZIEL)
            np.savez_compressed(rp[:-4] + ".tmp.npz", **d)
            os.replace(rp[:-4] + ".tmp.npz", rp)
            print("%-50s %s" % (f, how), flush=True)
    print("FERTIG")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    o = sp.add_parser("ordner")
    o.add_argument("art", choices=("bild", "maske", "stage3", "gruen"))
    o.add_argument("wurzeln", nargs="+")
    o.add_argument("--jobs", type=int, default=24)
    n = sp.add_parser("npz")
    n.add_argument("npz", nargs="+")
    n.add_argument("--zuordnung", required=True)
    n.add_argument("--jobs", type=int, default=24)
    a = ap.parse_args()
    return {"ordner": cmd_ordner, "npz": cmd_npz}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
