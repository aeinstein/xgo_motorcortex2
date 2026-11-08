#!/usr/bin/env python3
"""Bodenlabels für viele Fahrbilder mit ComfyUI (Prompt aus Datei, z. B. prompt8.txt). Je Bild: Grünmaske als PNG
(<out>/<fahrt>/<datei>.png, 0/255) und eine Zeile in <out>/rows.jsonl {"id", "rows" (80 Spalten), "s"}.
Setzt nach Abbruch fort (bereits in rows.jsonl). Hochgeladenes Bild und Ausgabe von ComfyUI werden nach dem Abholen
gelöscht, damit sich dort nichts ansammelt.

  python comfy_batch.py --prompt prompt8.txt --runs explore2,...,unten2 --step 8 --out labels_comfy8
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


def edit_and_clean(path: str, prompt: str) -> np.ndarray:
    """Wie C.run_edit, aber immer derselbe Upload-Name und die Ausgabedatei danach löschen."""
    ref = C.upload(path, "batch_aktuell.png")
    req = urllib.request.Request(C.URL + "/prompt", json.dumps({"prompt": C.workflow(ref, prompt, 1)}).encode(),
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
                buf = np.frombuffer(urllib.request.urlopen(C.URL + "/view?" + q, timeout=60).read(), np.uint8)
                f = os.path.join(COMFY, im["type"], im["subfolder"], im["filename"])
                if im["type"] == "output" and os.path.basename(f).startswith("floornet_seg") and os.path.exists(f):
                    os.remove(f)
                return cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if time.time() - t0 > 600:
            raise TimeoutError("ComfyUI antwortet nicht")
        time.sleep(0.3)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prompt", required=True)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--step", type=int, default=8)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    prompt = open(a.prompt, encoding="utf-8").read().strip()
    os.makedirs(a.out, exist_ok=True)
    rows_path = os.path.join(a.out, "rows.jsonl")
    done = set()
    if os.path.exists(rows_path):
        for ln in open(rows_path):
            try:
                done.add(json.loads(ln)["id"])
            except (ValueError, KeyError):
                pass
    todo = []
    for run in a.runs.split(","):
        files = sorted(os.path.basename(f) for f in glob.glob(os.path.join(DATA, run, "*.jpg")))
        todo += [run + "/" + f for f in files[::a.step]]
    todo = [k for k in todo if k not in done]
    json.dump({"prompt": prompt, "runs": a.runs, "step": a.step}, open(os.path.join(a.out, "info.json"), "w"))
    print("zu rechnen: %d (fertig: %d)" % (len(todo), len(done)), flush=True)
    t_start, fehler = time.time(), 0
    with open(rows_path, "a") as fo:
        for j, k in enumerate(todo):
            run, f = k.split("/", 1)
            t0 = time.time()
            try:
                edit = edit_and_clean(os.path.join(DATA, run, f), prompt)
            except Exception as e:                       # einzelnes Bild: weiter, am Ende zählen
                fehler += 1
                print("FEHLER %s: %s" % (k, e), flush=True)
                if fehler > 50:
                    raise
                time.sleep(5)
                continue
            m = C.floor_mask(edit)
            os.makedirs(os.path.join(a.out, run), exist_ok=True)
            cv2.imwrite(os.path.join(a.out, run, f.rsplit(".", 1)[0] + ".png"), m.astype(np.uint8) * 255)
            fo.write(json.dumps({"id": k, "rows": C.mask_rows(m).tolist(), "s": round(time.time() - t0, 2)}) + "\n")
            fo.flush()
            if (j + 1) % 100 == 0:
                el = time.time() - t_start
                print("%d/%d  %.2f s je Bild  Rest %.1f h  Fehler %d" % (j + 1, len(todo), el / (j + 1),
                                                                      el / (j + 1) * (len(todo) - j - 1) / 3600, fehler), flush=True)
    print("FERTIG %d Bilder, Fehler %d, %.1f h" % (len(todo), fehler, (time.time() - t_start) / 3600), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
