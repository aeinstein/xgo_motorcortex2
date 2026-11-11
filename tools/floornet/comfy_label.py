#!/usr/bin/env python3
"""Bodenlabels aus ComfyUI (Qwen-Image-Edit 2511 + Lightning 4 Schritte): Das Modell färbt den Boden grün, aus dem
Ergebnis wird je Spalte die Bodengrenze (unterste Zeile, über der kein Boden mehr kommt). Prüfung gegen die
Handlinien (teil3.json) mit derselben Rechnung wie tools/floornet/eval_hand.py, daneben das Kantenverfahren.

  python comfy_label.py --hand ~/floornet/hand/teil3.json --labels ~/floornet/labels5 --n 8 --out lauf1
  python comfy_label.py ... --n 287 --out lauf2          # alle Handbilder

Ausgabe in --out: <id>_edit.png (Modell), <id>_vgl.jpg (Linien: grün Hand, magenta Modell, gelb Kanten),
ergebnis.json (Zeilen je Bild) und die Tabelle auf stdout. Bereits gerechnete Bilder werden übersprungen.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import time
import urllib.parse
import urllib.request
import uuid

import cv2
import numpy as np

URL = "http://127.0.0.1:8188"
NCOLS, W, H = 80, 640, 480
FAR = 0.15             # wie eval_hand.py
PROMPT = os.environ.get("FN_PROMPT") or (
    "Recolor the floor to solid pure green (#00FF00). The floor is every part of the ground the camera sees, up to "
    "where it meets a wall, furniture or an object; shadows, seams and reflections on the floor belong to the floor. "
    "Do not add, remove or change anything else: no new objects, no mats, no patterns. Walls, furniture and all other "
    "things stay exactly as they are.")


def workflow(img_name: str, prompt: str, seed: int) -> dict:
    return {
        "1": {"class_type": "UNETLoader", "inputs": {"unet_name": "qwen_image_edit_2511_fp8mixed.safetensors",
                                                     "weight_dtype": "default"}},
        "2": {"class_type": "LoraLoaderModelOnly", "inputs": {
            "model": ["1", 0], "lora_name": "Qwen-Image-Edit-2511-Lightning-4steps-V1.0-bf16.safetensors",
            "strength_model": 1.0}},
        "3": {"class_type": "ModelSamplingAuraFlow", "inputs": {"model": ["2", 0], "shift": 3.1}},
        "4": {"class_type": "CLIPLoader", "inputs": {"clip_name": "qwen_2.5_vl_7b_fp8_scaled.safetensors",
                                                     "type": "qwen_image"}},
        "5": {"class_type": "VAELoader", "inputs": {"vae_name": "qwen_image_vae.safetensors"}},
        "6": {"class_type": "LoadImage", "inputs": {"image": img_name}},
        "7": {"class_type": "FluxKontextImageScale", "inputs": {"image": ["6", 0]}},
        "8": {"class_type": "TextEncodeQwenImageEditPlus", "inputs": {"clip": ["4", 0], "prompt": prompt,
                                                                      "vae": ["5", 0], "image1": ["7", 0]}},
        "9": {"class_type": "TextEncodeQwenImageEditPlus", "inputs": {"clip": ["4", 0], "prompt": "",
                                                                      "vae": ["5", 0], "image1": ["7", 0]}},
        "10": {"class_type": "VAEEncode", "inputs": {"pixels": ["7", 0], "vae": ["5", 0]}},
        "11": {"class_type": "KSampler", "inputs": {
            "model": ["3", 0], "seed": seed, "steps": 4, "cfg": 1.0, "sampler_name": "euler", "scheduler": "simple",
            "positive": ["8", 0], "negative": ["9", 0], "latent_image": ["10", 0], "denoise": 1.0}},
        "12": {"class_type": "VAEDecode", "inputs": {"samples": ["11", 0], "vae": ["5", 0]}},
        "13": {"class_type": "SaveImage", "inputs": {"images": ["12", 0], "filename_prefix": "floornet_seg"}},
    }


def upload(path: str | bytes, name: str) -> str:
    """Bild (Pfad oder fertig kodierte Bytes) per /upload/image an ComfyUI (multipart, ohne Zusatzpakete)."""
    data = path if isinstance(path, bytes) else open(path, "rb").read()
    b = uuid.uuid4().hex
    body = (f"--{b}\r\nContent-Disposition: form-data; name=\"overwrite\"\r\n\r\ntrue\r\n"
            f"--{b}\r\nContent-Disposition: form-data; name=\"subfolder\"\r\n\r\nfloornet\r\n"
            f"--{b}\r\nContent-Disposition: form-data; name=\"image\"; filename=\"{name}\"\r\n"
            f"Content-Type: image/png\r\n\r\n").encode() + data + f"\r\n--{b}--\r\n".encode()
    req = urllib.request.Request(URL + "/upload/image", body, {"Content-Type": f"multipart/form-data; boundary={b}"})
    r = json.loads(urllib.request.urlopen(req, timeout=60).read())
    return (r.get("subfolder") + "/" if r.get("subfolder") else "") + r["name"]


def run_edit(path: str, name: str, seed: int = 1) -> np.ndarray:
    ref = upload(path, name)
    cid = uuid.uuid4().hex
    req = urllib.request.Request(URL + "/prompt", json.dumps({"prompt": workflow(ref, PROMPT, seed),
                                                              "client_id": cid}).encode(),
                                 {"Content-Type": "application/json"})
    pid = json.loads(urllib.request.urlopen(req, timeout=60).read())["prompt_id"]
    t0 = time.time()
    while True:
        h = json.loads(urllib.request.urlopen(URL + "/history/" + pid, timeout=60).read())
        if pid in h:
            st = h[pid].get("status", {})
            if st.get("status_str") == "error":
                raise RuntimeError("ComfyUI-Fehler: %s" % json.dumps(st)[:500])
            outs = [im for o in h[pid]["outputs"].values() for im in o.get("images", [])]
            if outs:
                im = outs[0]
                q = urllib.parse.urlencode({"filename": im["filename"], "subfolder": im["subfolder"],
                                            "type": im["type"]})
                buf = np.frombuffer(urllib.request.urlopen(URL + "/view?" + q, timeout=60).read(), np.uint8)
                return cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if time.time() - t0 > 600:
            raise TimeoutError("ComfyUI antwortet nicht")
        time.sleep(0.5)


def floor_mask(edit: np.ndarray, min_gruen: int | None = None) -> np.ndarray:
    """Grüne Pixel (Bodenfarbe des Modells) in 640x480. min_gruen: statt der Helligkeit G - max(R, B) prüfen."""
    e = cv2.resize(edit, (W, H), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(e, cv2.COLOR_BGR2HSV)
    # V ab 30: Schatten auf dem Boden malt das Modell dunkelgrün, darunter ist der Farbton nur noch Rauschen
    hell = hsv[..., 2] >= 30
    if min_gruen is not None:
        ei = e.astype(int)
        hell = ei[..., 1] - np.maximum(ei[..., 0], ei[..., 2]) >= min_gruen
    m = (np.abs(hsv[..., 0].astype(int) - 60) <= 12) & (hsv[..., 1] >= 120) & hell
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m.astype(bool)


def mask_rows(m: np.ndarray, gap: int = 6) -> np.ndarray:
    """Je Spalte (NCOLS Streifen): Zeile der Grenze = Oberkante des zusammenhängenden Bodens von unten
    (Lücken bis gap Zeilen werden überbrückt). Kein Boden unten: Zeile H (Hindernis direkt davor)."""
    rows = np.full(NCOLS, float(H), np.float32)
    sw = W // NCOLS
    frac = m.reshape(H, NCOLS, sw).mean(axis=2) >= 0.5
    for c in range(NCOLS):
        col = frac[:, c]
        y, miss = H - 1, 0
        top = H
        while y >= 0:
            if col[y]:
                top, miss = y, 0
            else:
                miss += 1
                if miss > gap:
                    break
            y -= 1
        rows[c] = top
    return rows


def row_dist(geo: dict, row, pitch):
    beta = np.arctan2(row - geo["H"] / 2, geo["f"]) - math.radians(geo["tilt_deg"]) - pitch[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(beta > math.radians(0.5), geo["cam_h"] / np.tan(beta), np.inf)


def polyline_rows(pts):
    xs = (np.arange(NCOLS) + 0.5) * W / NCOLS
    if len(pts) < 2:
        return np.full(NCOLS, np.nan, np.float32), np.zeros(NCOLS, bool)
    px = np.array([p[0] for p in pts], float)
    py = np.array([p[1] for p in pts], float)
    rows = np.interp(xs, px, py).astype(np.float32)
    ok = np.ones(NCOLS, bool)
    for i in range(len(pts) - 1):
        if len(pts[i]) > 2 and pts[i][2]:
            ok[(xs >= px[i]) & (xs <= px[i + 1])] = False
    return rows, ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand", required=True)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--ids", default="", help="Datei mit Bild-IDs (Fahrt/Datei je Zeile) statt --n")
    ap.add_argument("--hoehe", default="", help="hoehe.json (Gruppe = Körperhöhe mm -> IDs); Kamerahöhe je Bild, "
                                                "andere Gruppen fallen weg (wie eval_hand.py)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    hd = json.load(open(a.hand, encoding="utf-8"))["bilder"]
    ids = sorted(k for k, e in hd.items() if e.get("fertig") and not e.get("unbrauchbar"))
    random.Random(a.seed).shuffle(ids)
    ids = ids[:a.n]
    if a.ids:
        ids = [x.strip() for x in open(a.ids) if x.strip()]
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
    res_path = os.path.join(a.out, "ergebnis.json")
    res = json.load(open(res_path)) if os.path.exists(res_path) else {}
    lab = {}
    per_run = {}
    for k in ids:
        run, f = k.split("/", 1)
        if run not in lab:
            d = np.load(os.path.join(a.labels, run + ".npz"), allow_pickle=False)
            lab[run] = (d, {str(x): i for i, x in enumerate(d["files"])})
        d, pos = lab[run]
        i = pos[f]
        src = os.path.join(str(d["source_dir"]), f)
        tag = k.replace("/", "__").rsplit(".", 1)[0]
        if k not in res:
            t0 = time.time()
            edit = run_edit(src, tag + ".png")
            cv2.imwrite(os.path.join(a.out, tag + "_edit.png"), edit)
            rows = mask_rows(floor_mask(edit))
            res[k] = {"rows": rows.tolist(), "s": round(time.time() - t0, 1)}
            json.dump(res, open(res_path, "w"))
            print("%-40s %.1f s" % (k, res[k]["s"]), flush=True)
        rows = np.array(res[k]["rows"], np.float32)
        hr, ok = polyline_rows(hd[k]["pts"])
        kidx = np.linspace(0, d["kanten_row"].shape[1] - 1, NCOLS).round().astype(int)
        kr = d["kanten_row"][i][kidx]
        geo = json.loads(str(d["geom"]))
        if k in body:                                   # Labels rechnen mit 85 mm Körperhöhe (eval_hand.py)
            geo["cam_h"] = geo["cam_h"] + (body[k] - 85.0) / 1000.0
        pitch = np.array([d["pitch"][i]])
        per_run.setdefault(run, []).append((rows, kr, hr, ok, pitch, geo))
        # Vergleichsbild
        img = cv2.imread(src)
        xs = ((np.arange(NCOLS) + 0.5) * W / NCOLS).astype(int)
        for r_, col in ((hr, (0, 220, 0)), (kr, (0, 230, 230)), (rows, (255, 0, 255))):
            pts = [(int(x), int(y)) for x, y in zip(xs, r_) if np.isfinite(y)]
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts, np.int32)], False, col, 2)
        e = cv2.resize(cv2.imread(os.path.join(a.out, tag + "_edit.png")), (W, H))
        cv2.imwrite(os.path.join(a.out, tag + "_vgl.jpg"), np.hstack([img, e]))

    # Auswertung wie eval_hand.py: Fehler in Metern je Spalte, Bänder nach Entfernung der Handlinie
    print("\n%d Bilder, Modell %.1f s je Bild (Median)" % (len(ids), float(np.median([res[k]["s"] for k in ids]))))
    for lo, hi in ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5)):
        line = "%3.1f-%3.1f m " % (lo, hi)
        for who in (0, 1):
            n = fr = bl = 0
            meds = []
            for run, lst in per_run.items():
                for rows, kr, hr, ok, pitch, geo in lst:
                    pr = rows if who == 0 else kr
                    dh = row_dist(geo, hr[None], pitch)[0]
                    dp = row_dist(geo, pr[None], pitch)[0]
                    m = ok & (dh >= lo) & (dh < hi) & np.isfinite(pr)
                    e = dp - dh
                    n += int(m.sum())
                    fr += int(((e > FAR) & m).sum())
                    bl += int(((e < -FAR) & m).sum())
                    meds.append(np.abs(np.nan_to_num(e[m], posinf=9.0)))
            med = float(np.median(np.concatenate(meds)) * 100) if n else float("nan")
            line += "| %-6s n%5d frei %5.1f%% block %5.1f%% med %4.0f cm " % (
                ("modell", "kanten")[who], n, 100 * fr / max(n, 1), 100 * bl / max(n, 1), med)
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
