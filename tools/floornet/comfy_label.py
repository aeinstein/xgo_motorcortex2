#!/usr/bin/env python3
"""Bodenlabels aus ComfyUI (Qwen-Image-Edit 2511 + Lightning 4 Schritte), Bibliothek: Das Modell färbt den Boden grün
(workflow, run_edit), floor_mask macht daraus die Bodenmaske, mask_rows je Spalte die Bodengrenze (unterste Zeile,
über der kein Boden mehr kommt). Genutzt von comfy_batch.py, stage2.py, stage3_npz.py, neu_malen.py und anderen."""
from __future__ import annotations

import json
import math
import os
import time
import urllib.parse
import urllib.request
import uuid

import cv2
import numpy as np

URL = os.environ.get("COMFY_URL", "http://127.0.0.1:8190")   # 8188 belegt der virtuelle Rider (tools/sim_rider.py)
NCOLS, W, H = 80, 640, 480
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

