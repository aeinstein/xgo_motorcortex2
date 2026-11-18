#!/usr/bin/env python3
"""Den ganzen FloorNet-Bestand auf kimmy an Ort und Stelle entzerren (Linsenmodell wie am Rider seit 2026-11-18).

  python tools/floornet/entzerren_bestand.py ordner bild  WURZEL [WURZEL ...]   # Kamerabilder (jpg/png 640x480), bilinear
  python tools/floornet/entzerren_bestand.py ordner maske WURZEL [...]          # Masken 640x480, nächster Nachbar
  python tools/floornet/entzerren_bestand.py ordner stage3 WURZEL [...]         # Stage3 neu aus der entzerrten Stage2
                                                                                #   (info.json "quelle"), sonst Maske entzerrt
  python tools/floornet/entzerren_bestand.py ordner gruen WURZEL [...]          # ComfyUI-Grünbilder (jede Größe), bilinear,
                                                                                #   PNG-Metadaten bleiben
  python tools/floornet/entzerren_bestand.py zuordnen NPZ_ORDNER ... --masken STAGE3 [STAGE3 ...] --out zuordnung.json
                                                       # VOR dem Entzerren der Masken: aus welchem Stage3-Ordner stammen die Zeilen?
  python tools/floornet/entzerren_bestand.py npz NPZ_ORDNER [...] [--zuordnung zuordnung.json]
                                                       # Zeilen-Labels: aus entzerrter Maske neu (falls zugeordnet), sonst Kurve umrechnen
  python tools/floornet/entzerren_bestand.py hand JSON [...]                    # Handlinien (Punkte) umrechnen

Sicher gegen Abbruch und doppeltes Entzerren: Je Verzeichnis entstehen die neuen Dateien erst in <dir>/.entz_neu, dann
markiert .entz_bereit, dass alle fertig sind, dann werden sie über die alten geschoben (os.replace), zuletzt steht
<dir>/.entzerrt. Ein erneuter Aufruf überspringt fertige Verzeichnisse und setzt halb getauschte fort. Bilder, die
meta.jsonl schon als entzerrt führt ("cam": {"entzerrt": true}), werden nicht angefasst; nach dem Tausch trägt meta.jsonl
das Feld für jedes entzerrte Bild (auch für Links in andere, schon entzerrte Verzeichnisse). npz und JSON tragen
"entzerrt" selbst. Links auf Bilder werden nie bearbeitet, nur ihr Ziel.

Zeilen-Labels (je Spalte eine Bildzeile, row N x 80, kanten_row N x 160): Stammen sie aus Stage3-Masken (zuordnen
findet den Ordner, dessen Masken die Zeilen ergeben (höchstens 1 Zeile Abweichung in 80 % der Proben)), werden sie aus der entzerrten Maske neu gebildet
(comfy_label.mask_rows), kind wie in comfy_npz (row < ROW_MIN -> ignorieren). Sonst wird die Grenzlinie als Kurve
umgerechnet: Spaltenmitten (x, row) -> vorwaerts() -> an den Spaltenmitten neu abgetastet; kind nimmt die Art der
nächsten umgerechneten Spalte.
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

import entzerren as E  # noqa: E402

W, H = 640, 480
BILD_ENDUNG = (".jpg", ".jpeg", ".png")
JPEG_Q = 95
NEU, BEREIT, FERTIG = ".entz_neu", ".entz_bereit", ".entzerrt"

_TAB = None
_GROSS: dict = {}


def _tab():
    global _TAB
    if _TAB is None:
        _TAB = E.tabellen()
    return _TAB


def _gross_tab(w: int, h: int):
    """Float-Tabellen für ein Bild w x h, das das Kamerabild 640x480 gestreckt zeigt (ComfyUI 1184x880)."""
    if (w, h) not in _GROSS:
        (m1, m2), _ = _tab()
        fx, fy = cv2.convertMaps(m1, m2, cv2.CV_32FC1)
        sx, sy = W / w, H / h
        u = ((np.arange(w, dtype=np.float32) + 0.5) * sx - 0.5)[None, :].repeat(h, 0)
        v = ((np.arange(h, dtype=np.float32) + 0.5) * sy - 0.5)[:, None].repeat(w, 1)
        qx = cv2.remap(fx, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        qy = cv2.remap(fy, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        _GROSS[(w, h)] = ((qx + 0.5) / sx - 0.5, (qy + 0.5) / sy - 0.5)
    return _GROSS[(w, h)]


# ---------------------------------------------------------------- Bilder je Datei (im Worker)

def _eine(job) -> str:
    """(art, quelle, ziel) -> "" oder Grund fürs Überspringen."""
    art, src, dst, s2 = job
    try:
        if art == "stage3":
            import stage3 as S3
            m = cv2.imread(s2, cv2.IMREAD_GRAYSCALE) if s2 and os.path.exists(s2) else None
            if m is None:
                m = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
                if m is None or m.shape != (H, W):
                    return "nicht lesbar"
                m = E.maske(m, _tab())
            if m.shape != (H, W):
                return "Größe %dx%d" % (m.shape[1], m.shape[0])
            if not cv2.imwrite(dst, S3.from_bottom(m > 0).astype(np.uint8) * 255):
                return "Schreiben fehlgeschlagen"
            return ""
        if art == "gruen":
            from PIL import Image, PngImagePlugin
            im = Image.open(src)
            info = dict(im.info)
            mode = im.mode
            if abs(im.width / im.height - W / H) > 0.02 or im.width < W:
                return "kein Kamerabild %dx%d" % (im.width, im.height)      # Collagen, Vergleichstafeln
            a = np.asarray(im.convert("RGB") if mode not in ("RGB", "L") else im)
            mx, my = _gross_tab(a.shape[1], a.shape[0])
            out = cv2.remap(a, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            meta = PngImagePlugin.PngInfo()
            for k, v in info.items():
                if isinstance(v, str):
                    meta.add_text(k, v)
            meta.add_text("entzerrt", "1")
            Image.fromarray(out).save(dst, pnginfo=meta)
        else:
            img = cv2.imread(src, cv2.IMREAD_UNCHANGED)
            if img is None:
                return "nicht lesbar"
            if img.shape[:2] != (H, W):
                return "Größe %dx%d" % (img.shape[1], img.shape[0])
            out = E.bild(img, _tab()) if art == "bild" else E.maske(img, _tab())
            par = [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q] if dst.lower().endswith((".jpg", ".jpeg")) else []
            if not cv2.imwrite(dst, out, par):
                return "Schreiben fehlgeschlagen"
        if os.path.getsize(dst) == 0:
            return "leer geschrieben"
        return ""
    except Exception as e:  # noqa: BLE001
        return "Fehler %s" % e


# ---------------------------------------------------------------- meta.jsonl

def _meta_laden(d: str) -> list | None:
    p = os.path.join(d, "meta.jsonl")
    if not os.path.isfile(p):
        return None
    out = []
    with open(p, encoding="utf-8") as f:
        for ln in f:
            try:
                out.append(json.loads(ln))
            except ValueError:
                out.append(ln.rstrip("\n"))      # unlesbare Zeile unverändert lassen
    return out


def _schon_entzerrt(meta: list | None) -> set:
    if not meta:
        return set()
    return {m["file"] for m in meta if isinstance(m, dict) and isinstance(m.get("cam"), dict) and m["cam"].get("entzerrt")}


def _ziel_fertig(d: str, name: str) -> bool:
    """Liegt das Bild (oder das Ziel des Links) in einem schon entzerrten Verzeichnis?"""
    p = os.path.realpath(os.path.join(d, name))
    return os.path.exists(p) and os.path.exists(os.path.join(os.path.dirname(p), FERTIG))


def _meta_markieren(d: str, entz: set) -> int:
    meta = _meta_laden(d)
    if meta is None:
        return 0
    n = 0
    for m in meta:
        if not isinstance(m, dict) or "file" not in m:
            continue
        if m["file"] in entz or _ziel_fertig(d, m["file"]):
            cam = m.get("cam") if isinstance(m.get("cam"), dict) else {}
            if not cam.get("entzerrt"):
                cam["entzerrt"] = True
                m["cam"] = cam
                n += 1
    p = os.path.join(d, "meta.jsonl")
    tmp = p + ".entz.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        for m in meta:
            f.write((m if isinstance(m, str) else json.dumps(m, ensure_ascii=False)) + "\n")
    os.replace(tmp, p)
    return n


# ---------------------------------------------------------------- Verzeichnis als Einheit

def _stage2_von(d: str) -> str | None:
    """Stage3-Verzeichnis <wurzel>/<fahrt> -> entzerrtes Stage2-Verzeichnis derselben Fahrt (aus <wurzel>/info.json)."""
    p = os.path.join(os.path.dirname(d), "info.json")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as fh:
        q = json.load(fh).get("quelle")
    q = q and os.path.join(os.path.expanduser(q), os.path.basename(d))
    return q if q and os.path.isdir(q) else None


def einheit(d: str, art: str, pool) -> dict:
    """Ein Verzeichnis (nicht rekursiv). Rückgabe: Bericht."""
    rep = {"dir": d, "art": art, "neu": 0, "uebersprungen": {}, "links": 0, "status": ""}
    if not os.path.isdir(d):
        rep["status"] = "gelöscht"
        return rep
    if os.path.exists(os.path.join(d, FERTIG)):
        rep["status"] = "schon fertig"
        return rep
    s2 = _stage2_von(d) if art == "stage3" else None
    if art == "stage3" and s2 and not os.path.exists(os.path.join(s2, FERTIG)):
        rep["status"] = "warte auf Stage2 " + s2
        return rep
    rep["stage2"] = s2
    neu = os.path.join(d, NEU)
    meta = _meta_laden(d) if art == "bild" else None
    vorher = _schon_entzerrt(meta)
    namen = sorted(f for f in os.listdir(d) if f.lower().endswith(BILD_ENDUNG) and not f.startswith("."))
    echt = [f for f in namen if not os.path.islink(os.path.join(d, f))]
    rep["links"] = len(namen) - len(echt)
    if not os.path.exists(os.path.join(d, BEREIT)):
        shutil.rmtree(neu, ignore_errors=True)
        os.makedirs(neu)
        todo = [f for f in echt if f not in vorher]
        rep["uebersprungen"].update({f: "laut meta schon entzerrt" for f in echt if f in vorher})
        res = pool.map(_eine, [(art, os.path.join(d, f), os.path.join(neu, f), s2 and os.path.join(s2, f)) for f in todo],
                       chunksize=32)
        for f, why in zip(todo, res):
            if why:
                rep["uebersprungen"][f] = why
                try:
                    os.remove(os.path.join(neu, f))
                except OSError:
                    pass
        with open(os.path.join(d, BEREIT), "w") as fh:
            json.dump({"uebersprungen": rep["uebersprungen"]}, fh, ensure_ascii=False)
    else:
        with open(os.path.join(d, BEREIT)) as fh:
            rep["uebersprungen"] = json.load(fh).get("uebersprungen", {})
    getauscht = set()
    for f in sorted(os.listdir(neu)):
        os.replace(os.path.join(neu, f), os.path.join(d, f))
        getauscht.add(f)
    os.rmdir(neu)
    entz = {f for f in echt if f not in rep["uebersprungen"]}
    rep["neu"] = len(entz)
    with open(os.path.join(d, FERTIG), "w") as fh:
        json.dump({"art": art, "zeit": time.strftime("%Y-%m-%d %H:%M:%S"), "neu": rep["neu"],
                   "uebersprungen": rep["uebersprungen"], "stage2": rep.get("stage2"), "linsenmodell": "xgonav/config.py [camera] lens_*",
                   "jpeg_q": JPEG_Q}, fh, ensure_ascii=False, indent=1)
    if art == "bild":
        rep["meta"] = _meta_markieren(d, entz)
    os.remove(os.path.join(d, BEREIT))
    rep["status"] = "entzerrt"
    return rep


def verzeichnisse(wurzeln: list[str]) -> list[str]:
    """Alle echten Verzeichnisse unter den Wurzeln, die Bilder enthalten; Links auf Verzeichnisse werden nicht verfolgt.
    Verzeichnisse nur mit Links kommen zuletzt (ihre Ziele müssen erst fertig sein)."""
    echt, nur_links = [], []
    seen = set()
    for w in wurzeln:
        for wurzel, dirs, files in os.walk(w):
            dirs[:] = sorted(x for x in dirs if not x.startswith(".") and not os.path.islink(os.path.join(wurzel, x)))
            rp = os.path.realpath(wurzel)
            if rp in seen:
                continue
            seen.add(rp)
            b = [f for f in files if f.lower().endswith(BILD_ENDUNG) and not f.startswith(".")]
            if not b:
                continue
            if any(not os.path.islink(os.path.join(wurzel, f)) for f in b):
                echt.append(wurzel)
            else:
                nur_links.append(wurzel)
    return echt + nur_links


def cmd_ordner(a) -> int:
    dirs = verzeichnisse(a.wurzeln)
    print("%d Verzeichnisse, Art %s" % (len(dirs), a.art), flush=True)
    t0 = time.time()
    summe, ueb = 0, 0
    with Pool(a.jobs, initializer=cv2.setNumThreads, initargs=(1,)) as pool:
        for i, d in enumerate(dirs):
            rep = einheit(d, a.art, pool)
            summe += rep["neu"] if rep["status"] == "entzerrt" else 0
            ueb += len(rep["uebersprungen"])
            gr = {}
            for why in rep["uebersprungen"].values():
                gr[why] = gr.get(why, 0) + 1
            print("[%d/%d] %-14s %6d neu, %d Links, meta %s, übersprungen %s  %s  (%.0f s)" % (
                i + 1, len(dirs), rep["status"], rep["neu"], rep["links"], rep.get("meta", "-"), gr or "-", d,
                time.time() - t0), flush=True)
    print("FERTIG %d Dateien entzerrt, %d übersprungen, %.0f s" % (summe, ueb, time.time() - t0), flush=True)
    return 0


# ---------------------------------------------------------------- Zeilen-Labels

def _spalten(n: int) -> np.ndarray:
    return (np.arange(n) + 0.5) * W / n


def kurve(row: np.ndarray, kind: np.ndarray | None = None):
    """Zeilen je Spalte (N x n) als Grenzkurve umrechnen. NaN bleibt NaN (auch dort, wo die Nachbarn fehlen)."""
    n = row.shape[1]
    xc = _spalten(n)
    out = np.full(row.shape, np.nan, np.float32)
    kout = None if kind is None else kind.copy()
    for k in range(row.shape[0]):
        r = row[k].astype(float)
        ok = np.isfinite(r)
        if not ok.any():
            continue
        p = E.vorwaerts(np.stack([xc[ok], r[ok]], 1))
        o = np.argsort(p[:, 0], kind="stable")
        neu = np.interp(xc, p[o, 0], p[o, 1])
        # Lücken bleiben Lücken: Zielspalte nimmt die nächste umgerechnete Spalte; war deren Quelle NaN -> NaN
        alle_x = xc.copy()                        # NaN-Spalten bleiben an ihrer alten Stelle
        alle_x[ok] = p[:, 0]
        naechste = np.abs(alle_x[None, :] - xc[:, None]).argmin(1)
        neu[~ok[naechste]] = np.nan
        out[k] = np.clip(neu, 0, H)
        if kout is not None:
            kout[k] = kind[k][naechste]
    return out, kout


def _rows_of(path):
    import comfy_label as C
    m = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if m is None or m.shape != (C.H, C.W):
        return None
    return C.mask_rows(m > 0)


def cmd_zuordnen(a) -> int:
    import glob
    out = {}
    with Pool(a.jobs) as pool:
        for nd in a.npz:
            for f in sorted(glob.glob(os.path.join(nd, "*.npz"))):
                d = np.load(f, allow_pickle=False)
                if "row" not in d.files or "entzerrt" in d.files:
                    continue
                run, files, row = str(d["run"]), d["files"], d["row"]
                idx = np.linspace(0, len(files) - 1, min(len(files), a.probe)).round().astype(int) if len(files) else []
                best = (0.0, None)
                for m in a.masken:
                    paths = [os.path.join(m, run, str(files[i])[:-4] + ".png") for i in idx]
                    if not all(os.path.exists(p) for p in paths):
                        print("   %s: Masken fehlen (%d von %d)" % (m, sum(not os.path.exists(p) for p in paths), len(paths)))
                        continue
                    rs = pool.map(_rows_of, paths)
                    # gleich: höchstens 1 Zeile Abweichung je Spalte (einzelne Masken wurden nach dem Bau der Labels neu gemalt)
                    gleich = np.mean([r is not None and np.abs(r - row[i]).max() <= 1 for r, i in zip(rs, idx)])
                    if gleich > best[0]:
                        best = (float(gleich), m)
                out[os.path.realpath(f)] = {"masken": best[1] if best[0] >= 0.8 else None, "gleich": best[0], "n": len(idx)}
                print("%-50s %-30s %.2f" % (f, best[1], best[0]), flush=True)
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=1)
    print("FERTIG %d Dateien, zugeordnet %d" % (len(out), sum(v["masken"] is not None for v in out.values())))
    return 0


def cmd_npz(a) -> int:
    import glob
    from comfy_npz import EXACT, IGNORE, ROW_MIN
    zu = {}
    if a.zuordnung:
        with open(a.zuordnung) as fh:
            zu = json.load(fh)
    seen = set()
    with Pool(a.jobs) as pool:
        for nd in a.npz:
            for f in sorted(glob.glob(os.path.join(nd, "*.npz"))):
                rp = os.path.realpath(f)
                if rp in seen:
                    continue
                seen.add(rp)
                d = dict(np.load(rp, allow_pickle=False))
                if "row" not in d or "entzerrt" in d:
                    print("%-50s %s" % (f, "schon entzerrt" if "entzerrt" in d else "keine Zeilen"), flush=True)
                    continue
                m = (zu.get(rp) or {}).get("masken")
                how = "Kurve"
                if m:
                    run = str(d["run"])
                    if not os.path.isdir(os.path.join(m, run)):
                        m = None                            # Masken gelöscht: Zeilen als Kurve umrechnen
                    elif not os.path.exists(os.path.join(m, run, FERTIG)):
                        print("%-50s Masken %s/%s noch nicht entzerrt: ausgelassen" % (f, m, run), flush=True)
                        continue
                if m:
                    rs = pool.map(_rows_of, [os.path.join(m, run, str(x)[:-4] + ".png") for x in d["files"]], chunksize=32)
                    fehlt = [i for i, r in enumerate(rs) if r is None]
                    neu, _ = kurve(d["row"][fehlt]) if fehlt else (None, None)
                    row = np.stack([r if r is not None else neu[fehlt.index(i)] for i, r in enumerate(rs)]).astype(np.float32) \
                        if len(rs) else d["row"]
                    d["kind"] = np.where(row < ROW_MIN, IGNORE, EXACT).astype(np.uint8)
                    d["row"] = row
                    how = "Maske %s (%d ohne Maske als Kurve)" % (m, len(fehlt))
                else:
                    d["row"], d["kind"] = kurve(d["row"], d["kind"])
                if "kanten_row" in d and np.isfinite(d["kanten_row"]).any():
                    d["kanten_row"], _ = kurve(d["kanten_row"])
                d["entzerrt"] = np.array(how)
                tmp = rp[:-4] + ".entz.tmp.npz"
                np.savez_compressed(tmp, **d)
                os.replace(tmp, rp)
                print("%-50s %s" % (f, how), flush=True)
    print("FERTIG")
    return 0


def cmd_hand(a) -> int:
    for p in a.json:
        with open(p, encoding="utf-8") as fh:
            j = json.load(fh)
        if j.get("entzerrt"):
            print("%s schon entzerrt" % p)
            continue
        if not isinstance(j.get("bilder"), dict):
            print("%s: keine Handlinien (kein Feld bilder), unverändert" % p)
            continue
        n = 0
        for v in j["bilder"].values():
            if v.get("pts"):
                # Punkt [x, y] oder [x, y, Kennung]: nur x, y umrechnen, den Rest behalten
                q = E.vorwaerts(np.asarray([pt[:2] for pt in v["pts"]], float))
                q[:, 0] = np.clip(q[:, 0], 0, W - 1)
                q[:, 1] = np.clip(q[:, 1], 0, H)
                v["pts"] = [[round(float(x), 1), round(float(y), 1)] + list(pt[2:]) for (x, y), pt in zip(q, v["pts"])]
                n += 1
        j["entzerrt"] = True
        tmp = p + ".entz.tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(j, fh, ensure_ascii=False)
        os.replace(tmp, p)
        print("%s: %d Linien umgerechnet" % (p, n))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    o = sp.add_parser("ordner")
    o.add_argument("art", choices=("bild", "maske", "stage3", "gruen"))
    o.add_argument("wurzeln", nargs="+")
    o.add_argument("--jobs", type=int, default=24)
    z = sp.add_parser("zuordnen")
    z.add_argument("npz", nargs="+")
    z.add_argument("--masken", nargs="+", required=True)
    z.add_argument("--out", required=True)
    z.add_argument("--probe", type=int, default=30)
    z.add_argument("--jobs", type=int, default=24)
    n = sp.add_parser("npz")
    n.add_argument("npz", nargs="+")
    n.add_argument("--zuordnung", default="")
    n.add_argument("--jobs", type=int, default=24)
    h = sp.add_parser("hand")
    h.add_argument("json", nargs="+")
    a = ap.parse_args()
    return {"ordner": cmd_ordner, "zuordnen": cmd_zuordnen, "npz": cmd_npz, "hand": cmd_hand}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
