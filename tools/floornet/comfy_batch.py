#!/usr/bin/env python3
"""Bodenlabels für viele Fahrbilder mit ComfyUI (Prompt aus Datei, z. B. prompt8.txt). Je Bild: Bearbeitung des Modells
(<edits>/<fahrt>/<datei>.png, Default <edits> = <out>/edits), Grünmaske als PNG (<out>/<fahrt>/<datei>.png, 0/255) und
eine Zeile in <out>/rows.jsonl {"id", "rows" (80 Spalten), "s"}. Setzt nach Abbruch fort (bereits in rows.jsonl).
Die Ausgabe von ComfyUI wird ins Archiv verschoben, Bearbeitungen werden nie gelöscht: Die Maske lässt sich daraus
ohne Modell neu rechnen. Der Seed ist fest (SEED), damit dasselbe Bild dieselbe Bearbeitung ergibt.

  python comfy_batch.py --prompt prompt8.txt --runs explore2,...,unten2 --step 8 --out labels_comfy8
  python comfy_batch.py --prompt prompt8.txt --ids ids_nah.txt --out labels_comfy8_nah
  python comfy_batch.py --prompt prompt9.txt --ids ids_g0h.txt --edits stage1_g0h --nur_edits --eingang g0h --out stage1_g0h   # nur fehlende
  python comfy_batch.py --prompt prompt10.txt --ids ids.txt --edits stage1_hd --nur_edits --eingang auto --dunkel_aus stage1_g0h --out stage1_hd

--eingang auto wählt die Kette je Bild (Nutzer 2026-11-10): Helligkeit ab --hell_ab (60) = helle Szene, das Bild geht
unverändert ins Modell; darunter G0h. Für dunkle Bilder wird eine vorhandene G0h-Bearbeitung aus --dunkel_aus verlinkt
statt neu gerechnet (gleicher Prompt und Seed ergeben dasselbe Bild). Welche Kette ein Bild bekam, steht in
<out>/eingang.jsonl.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np

import comfy_label as C  # gleiches Verzeichnis

DATA = os.path.expanduser("~/floornet/data/rider/fahrt")
COMFY = os.path.expanduser("~/comfyui")


SEED = 1            # fest: Damit sind auch die Masken von Prompt 8 entstanden. Nicht ändern, sonst passt das Archiv nicht mehr.


def edit_path(edits: str, key: str) -> str:
    """Bild-ID (fahrt/datei.jpg) -> Datei im Archiv."""
    run, f = key.split("/", 1)
    return os.path.join(edits, run, f.rsplit(".", 1)[0] + ".png")


def missing_edits(keys: list[str], edits: str) -> list[str]:
    """IDs ohne lesbare Bearbeitung im Archiv, Reihenfolge bleibt."""
    return [k for k in keys if not os.path.exists(edit_path(edits, k)) or cv2.imread(edit_path(edits, k), cv2.IMREAD_COLOR) is None]


def keep_edit(buf: bytes, comfy_file: str, target: str) -> None:
    """Bearbeitung unverändert ins Archiv: ComfyUIs Datei verschieben, sonst die abgeholten Bytes schreiben. Atomar,
    ein Abbruch hinterlässt keine halbe PNG unter dem Zielnamen."""
    os.makedirs(os.path.dirname(target), exist_ok=True)
    try:
        os.replace(comfy_file, target)
        return
    except OSError:
        pass
    tmp = target + ".tmp"
    with open(tmp, "wb") as fo:
        fo.write(buf)
    os.replace(tmp, target)


def gray_png(path: str) -> bytes:
    """Fahrbild als Grau-PNG im Speicher: Ohne Farbe im Eingang stammt alles Grüne im Ergebnis vom Modell, nicht vom Farbstich der Kamera."""
    return cv2.imencode(".png", cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY))[1].tobytes()


def g0h_image(o: np.ndarray) -> np.ndarray:
    """Grünkanal 0 bei Helligkeit des Graubildes: R und B behalten ihr Verhältnis, der größere von beiden bekommt den Grauwert.
    Rein grüne Pixel: R = B = Grau. So bleibt Farbe für den Teppich, ohne dass dunkle, grünstichige Bilder schwarz werden."""
    gray = cv2.cvtColor(o, cv2.COLOR_BGR2GRAY).astype(np.float32)
    b, r = o[..., 0].astype(np.float32), o[..., 2].astype(np.float32)
    top = np.maximum(b, r)
    s = np.divide(gray, top, out=np.zeros_like(gray), where=top > 0)
    aus = np.zeros_like(o)
    aus[..., 0] = np.clip(np.where(top > 0, b * s, gray) + 0.5, 0, 255).astype(np.uint8)
    aus[..., 2] = np.clip(np.where(top > 0, r * s, gray) + 0.5, 0, 255).astype(np.uint8)
    return aus


def g0h_png(path: str) -> bytes:
    """Fahrbild als G0h-PNG im Speicher."""
    return cv2.imencode(".png", g0h_image(cv2.imread(path)))[1].tobytes()


EINGANG = {"farbe": None, "grau": gray_png, "g0h": g0h_png}
HELL_AB = 60.0      # Nutzer 2026-11-10
HELL_ZEILE = 137    # ab hier sieht das FloorNet das Bild (net.py), darüber liegen Fenster und Lampen


def helligkeit(o: np.ndarray) -> float:
    """Mittlerer Grauwert (0..255) des unteren Bildteils, Maß für die Trennung helle/dunkle Szene."""
    return float(cv2.cvtColor(o, cv2.COLOR_BGR2GRAY)[HELL_ZEILE * o.shape[0] // 480:].mean())


def kette(path: str, hell_ab: float = HELL_AB) -> tuple[str, float]:
    """Eingang für --eingang auto: (farbe oder g0h, Helligkeit)."""
    h = helligkeit(cv2.imread(path))
    return ("farbe" if h >= hell_ab else "g0h"), h


def link_edit(quelle: str, target: str) -> bool:
    """Vorhandene Bearbeitung ins Archiv übernehmen (harter Link, sonst Kopie). False, wenn sie fehlt oder unlesbar ist."""
    if not os.path.exists(quelle) or cv2.imread(quelle, cv2.IMREAD_COLOR) is None:
        return False
    os.makedirs(os.path.dirname(target), exist_ok=True)
    tmp = target + ".tmp"
    if os.path.exists(tmp):
        os.remove(tmp)
    try:
        os.link(quelle, tmp)
    except OSError:
        with open(quelle, "rb") as fi, open(tmp, "wb") as fo:
            fo.write(fi.read())
    os.replace(tmp, target)
    return True


def edit_and_keep(path: str, prompt: str, target: str, eingang: str = "farbe", seed: int = SEED) -> np.ndarray:
    """Wie C.run_edit, mit festem Seed (anderer Seed nur für neu_malen.py); die Bearbeitung landet in target. Upload-Name je Prozess, parallele Läufe stören sich nicht."""
    fn = EINGANG[eingang]
    ref = C.upload(fn(path) if fn else path, "batch_%d.png" % os.getpid())
    req = urllib.request.Request(C.URL + "/prompt", json.dumps({"prompt": C.workflow(ref, prompt, seed)}).encode(),
                                 {"Content-Type": "application/json"})
    pid = json.loads(urllib.request.urlopen(req, timeout=60).read())["prompt_id"]
    t0 = time.time()
    while True:
        h = json.loads(urllib.request.urlopen(C.URL + "/history/" + pid, timeout=60).read())
        if pid in h:
            st = h[pid].get("status", {})
            if st.get("status_str") == "error":
                raise RuntimeError("ComfyUI-Fehler: %s" % json.dumps(st)[:300])
            outs = [im for o in h[pid]["outputs"].values() for im in o.get("images", [])]
            if outs:
                im = outs[0]
                q = urllib.parse.urlencode({"filename": im["filename"], "subfolder": im["subfolder"], "type": im["type"]})
                buf = urllib.request.urlopen(C.URL + "/view?" + q, timeout=60).read()
                keep_edit(buf, os.path.join(COMFY, im["type"], im["subfolder"], im["filename"]), target)
                return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
        if time.time() - t0 > 600:
            raise TimeoutError("ComfyUI antwortet nicht")
        time.sleep(0.3)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prompt", required=True)
    ap.add_argument("--runs", default="")
    ap.add_argument("--ids", default="", help="Datei mit Bild-IDs (fahrt/datei.jpg), statt --runs und --step")
    ap.add_argument("--step", type=int, default=8)
    ap.add_argument("--out", required=True)
    ap.add_argument("--edits", default="", help="Archiv der Bearbeitungen, Default <out>/edits")
    ap.add_argument("--nur_edits", action="store_true", help="nur fehlende Bearbeitungen erzeugen, keine Masken, kein rows.jsonl")
    ap.add_argument("--eingang", choices=sorted(EINGANG) + ["auto"], default="farbe", help="Bild vor dem Upload wandeln (wird nicht gespeichert); g0h: Grünkanal 0 mit Helligkeit des Graubildes; "
                                                                                    "auto: je Bild farbe (hell) oder g0h (dunkel)")
    ap.add_argument("--hell_ab", type=float, default=HELL_AB, help="--eingang auto: ab dieser Helligkeit (0..255, unterer Bildteil) farbe")
    ap.add_argument("--dunkel_aus", default="", help="--eingang auto: Archiv mit G0h-Bearbeitungen, die für dunkle Bilder verlinkt statt neu gerechnet werden")
    a = ap.parse_args()
    prompt = open(a.prompt, encoding="utf-8").read().strip()
    edits = a.edits or os.path.join(a.out, "edits")
    os.makedirs(a.out, exist_ok=True)
    rows_path = os.devnull if a.nur_edits else os.path.join(a.out, "rows.jsonl")
    done = set()
    if os.path.exists(rows_path):
        for ln in open(rows_path):
            try:
                done.add(json.loads(ln)["id"])
            except (ValueError, KeyError):
                pass
    todo = []
    if a.ids:
        todo = open(a.ids, encoding="utf-8").read().split()
    for run in a.runs.split(",") if a.runs else ():
        files = sorted(os.path.basename(f) for f in glob.glob(os.path.join(DATA, run, "*.jpg")))
        todo += [run + "/" + f for f in files[::a.step]]
    n_all = len(todo)
    todo = missing_edits(todo, edits) if a.nur_edits else [k for k in todo if k not in done]
    json.dump({"prompt": prompt, "runs": a.runs, "step": a.step, "seed": SEED, "eingang": a.eingang,
               **({"hell_ab": a.hell_ab, "dunkel_aus": a.dunkel_aus} if a.eingang == "auto" else {})}, open(os.path.join(a.out, "info.json"), "w"))
    print("zu rechnen: %d (fertig: %d)" % (len(todo), n_all - len(todo)), flush=True)
    t_start, fehler, verlinkt = time.time(), 0, 0
    with open(rows_path, "a") as fo, open(os.path.join(a.out, "eingang.jsonl") if a.eingang == "auto" else os.devnull, "a") as fe:
        for j, k in enumerate(todo):
            run, f = k.split("/", 1)
            t0 = time.time()
            try:
                eingang, edit = a.eingang, None
                if eingang == "auto":
                    eingang, hell = kette(os.path.join(DATA, run, f), a.hell_ab)
                    if eingang == "g0h" and a.dunkel_aus and link_edit(edit_path(a.dunkel_aus, k), edit_path(edits, k)):
                        edit = cv2.imread(edit_path(edits, k), cv2.IMREAD_COLOR)
                        verlinkt += 1
                    fe.write(json.dumps({"id": k, "eingang": eingang, "hell": round(hell, 1), "verlinkt": edit is not None}) + "\n")
                    fe.flush()
                if edit is None:
                    edit = edit_and_keep(os.path.join(DATA, run, f), prompt, edit_path(edits, k), eingang)
            except Exception as e:                       # einzelnes Bild: weiter, am Ende zählen
                fehler += 1
                print("FEHLER %s: %s" % (k, e), flush=True)
                if fehler > 50:
                    raise
                time.sleep(5)
                continue
            if not a.nur_edits:
                m = C.floor_mask(edit)
                os.makedirs(os.path.join(a.out, run), exist_ok=True)
                cv2.imwrite(os.path.join(a.out, run, f.rsplit(".", 1)[0] + ".png"), m.astype(np.uint8) * 255)
                fo.write(json.dumps({"id": k, "rows": C.mask_rows(m).tolist(), "s": round(time.time() - t0, 2)}) + "\n")
                fo.flush()
            if (j + 1) % 100 == 0:
                el = time.time() - t_start
                print("%d/%d  %.2f s je Bild  Rest %.1f h  Fehler %d" % (j + 1, len(todo), el / (j + 1),
                                                                      el / (j + 1) * (len(todo) - j - 1) / 3600, fehler), flush=True)
    print("FERTIG %d Bilder, Fehler %d, %.1f h%s" % (len(todo), fehler, (time.time() - t_start) / 3600,
                                                     ", davon %d verlinkt" % verlinkt if a.eingang == "auto" else ""), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
