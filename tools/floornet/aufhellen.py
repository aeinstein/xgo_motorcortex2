#!/usr/bin/env python3
"""Zero-DCE-Aufhellung dunkler Fahrbilder vor ComfyUI (nur kimmy, nie am Rider).

DCE-Net aus keras-io/low-light-image-enhancement (7 Faltungen, 32 Kanäle) liefert je Pixel 8 Kurven r_i; die Aufhellung
ist LE(x) = x + s·r_i·(x² − x), achtmal hintereinander. s = Stärke: 1 wie im Keras-Beispiel (Probelauf 2026-11-18,
Nutzer: zu stark), kleiner = dezenter, 0 = Bild unverändert.

Gewichte: einmal aus dem Keras-Modell exportieren (Venv mit tf_keras, z. B. ~/zerodce):
  python aufhellen.py export ~/zerodce/modell ~/floornet/comfy/zerodce.npz
Probe: python aufhellen.py bilder IDS DATA OUT --staerke 0.25,0.5
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

GEWICHTE = os.path.expanduser("~/floornet/comfy/zerodce.npz")
NAMEN = ("c1", "c2", "c3", "c4", "c5", "c6", "c7")


def kurven(x: np.ndarray, r: np.ndarray, staerke: float) -> np.ndarray:
    """x (H, W, 3) in 0..1, r (H, W, 24) Kurven des Netzes -> aufgehelltes Bild (H, W, 3) in 0..1."""
    for i in range(8):
        x = x + staerke * r[..., 3 * i:3 * i + 3] * (x * x - x)
    return np.clip(x, 0.0, 1.0)


class ZeroDCE:
    """DCE-Net in torch (GPU, sonst CPU). aufhellen(): BGR uint8 -> BGR uint8."""

    def __init__(self, pfad: str = GEWICHTE):
        import torch
        self.torch = torch
        self.dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        g = np.load(pfad)
        # Keras-Kern (kh, kw, ein, aus) -> torch (aus, ein, kh, kw)
        self.w = [(torch.from_numpy(g[n + "_w"].transpose(3, 2, 0, 1).copy()).to(self.dev),
                   torch.from_numpy(g[n + "_b"]).to(self.dev)) for n in NAMEN]

    def netz(self, rgb: np.ndarray) -> np.ndarray:
        t = self.torch
        F = t.nn.functional
        c = lambda x, i: F.conv2d(x, self.w[i][0], self.w[i][1], padding=1)   # noqa: E731
        with t.inference_mode():
            x = t.from_numpy(rgb).to(self.dev).permute(2, 0, 1)[None]
            x1 = F.relu(c(x, 0))
            x2 = F.relu(c(x1, 1))
            x3 = F.relu(c(x2, 2))
            x4 = F.relu(c(x3, 3))
            x5 = F.relu(c(t.cat([x4, x3], 1), 4))
            x6 = F.relu(c(t.cat([x5, x2], 1), 5))
            r = t.tanh(c(t.cat([x6, x1], 1), 6))
        return r[0].permute(1, 2, 0).cpu().numpy()

    def aufhellen(self, bgr: np.ndarray, staerke: float) -> np.ndarray:
        if staerke <= 0:
            return bgr
        x = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        y = kurven(x, self.netz(x), staerke)
        return cv2.cvtColor((y * 255 + 0.5).astype(np.uint8), cv2.COLOR_RGB2BGR)


def export(modell: str, ziel: str) -> None:
    import tf_keras
    m = tf_keras.models.load_model(modell, compile=False)
    conv = [l for l in m.layers if type(l).__name__ == "Conv2D"]
    assert len(conv) == 7, "DCE-Net hat 7 Faltungen"
    out = {}
    for n, l in zip(NAMEN, conv):
        w, b = l.get_weights()
        out[n + "_w"], out[n + "_b"] = w.astype(np.float32), b.astype(np.float32)
    np.savez(ziel, **out)
    print("geschrieben:", ziel)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export")
    e.add_argument("modell")
    e.add_argument("ziel")
    b = sub.add_parser("bilder", help="Liste aufhellen, je Stärke <out>/<stärke>/<fahrt>/<datei>.png")
    b.add_argument("ids")
    b.add_argument("data")
    b.add_argument("out")
    b.add_argument("--staerke", default="0.25,0.5")
    b.add_argument("--gewichte", default=GEWICHTE)
    a = ap.parse_args()
    if a.cmd == "export":
        export(a.modell, a.ziel)
        return 0
    z = ZeroDCE(a.gewichte)
    for k in open(a.ids, encoding="utf-8").read().split():
        bgr = cv2.imread(os.path.join(a.data, k))
        for s in a.staerke.split(","):
            y = z.aufhellen(bgr, float(s))
            p = os.path.join(a.out, s, k.rsplit(".", 1)[0] + ".png")
            os.makedirs(os.path.dirname(p), exist_ok=True)
            cv2.imwrite(p, y)
        print(k, round(float(bgr.mean()), 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
