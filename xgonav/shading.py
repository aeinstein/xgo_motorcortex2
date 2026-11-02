"""Linsenabschattung (Lens Shading) ausgleichen.

Beim RiderPi sitzt ein Weitwinkelobjektiv (~91°) vor der OV5647. Zum Rand hin fällt
die Helligkeit stark ab (Blau/Grün auf ~20 %), und Rot bricht fast ganz weg (1-9 %
ab ~250 px vom Zentrum): schräg einfallendes rotes Licht geht am IR-Sperrfilter bzw.
an den Mikrolinsen verloren. Die libcamera-Tuningdatei korrigiert das nicht.

Modell: pro Farbkanal eine radiale Abschwächung F_c(r) = Helligkeit / Helligkeit im
Zentrum, als Polynom in s = (r / r0)^2 aus einem Weißbild gefittet (tools/calibrate.py
shading). Korrektur = 1 / F_c, begrenzt auf max_gain. Wo ein Kanal mehr als max_gain
bräuchte, ist die Farbe nicht mehr messbar: diese Pixel gelten als unzuverlässig.
"""
from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np

DEGREE = 4


@dataclass
class ShadingModel:
    cx: float              # Zentrum, normiert auf Bildbreite/-höhe (0..1)
    cy: float
    r0_rel: float          # Bezugsradius relativ zur halben Bilddiagonale
    coef: list[list[float]]  # je Kanal (B, G, R): a1..aN in F = 1 + a1 s + a2 s^2 + ...

    # ------------------------------------------------------------------
    @staticmethod
    def fit(flat_bgr: np.ndarray, valid: np.ndarray) -> "ShadingModel":
        """Aus einem Weißbild (BGR, float) und einer Maske gültiger Pixel fitten."""
        H, W = flat_bgr.shape[:2]
        yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
        g = flat_bgr.mean(-1)
        thr = np.percentile(g[valid], 95)
        m = valid & (g >= thr)
        cx, cy = float(xx[m].mean()), float(yy[m].mean())
        r0 = float(np.hypot(W / 2, H / 2))
        s = ((xx - cx) ** 2 + (yy - cy) ** 2) / (r0 * r0)
        idx = np.flatnonzero(valid.ravel())[::7]          # Unterabtastung reicht
        S = s.ravel()[idx]
        # konstanten Term mitfitten und danach normieren (statt auf einen Kern-Mittelwert,
        # der unter dem wahren Zentrumswert liegt und die Kurve verzerren würde)
        A = np.stack([S ** k for k in range(0, DEGREE + 1)], axis=1)
        coef = []
        for c in range(3):
            a, *_ = np.linalg.lstsq(A, flat_bgr[..., c].ravel()[idx].astype(np.float64), rcond=None)
            coef.append([float(v / a[0]) for v in a[1:]])
        return ShadingModel(cx / W, cy / H, 1.0, coef)

    def falloff(self, W: int, H: int) -> np.ndarray:
        """F_c(x, y) für ein Bild der Größe W x H, Form (H, W, 3), geclippt auf >= 0,002."""
        yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
        r0 = np.hypot(W / 2, H / 2) * self.r0_rel
        s = ((xx + 0.5 - self.cx * W) ** 2 + (yy + 0.5 - self.cy * H) ** 2) / (r0 * r0)
        out = np.empty((H, W, 3), np.float32)
        for c in range(3):
            f = np.ones_like(s)
            for k, a in enumerate(self.coef[c], start=1):
                f += a * s ** k
            out[..., c] = f
        return np.clip(out, 0.002, 1.5)

    def gain_and_reliable(self, W: int, H: int, max_gain: float) -> tuple[np.ndarray, np.ndarray]:
        """Korrekturfaktoren (H, W, 3), begrenzt, und Maske der zuverlässigen Pixel."""
        f = self.falloff(W, H)
        need = 1.0 / f
        reliable = (need <= max_gain).all(axis=-1)
        return np.minimum(need, max_gain).astype(np.float32), reliable

    # ------------------------------------------------------------------
    def save(self, path: str) -> None:
        with open(path, "w") as fh:
            json.dump({"cx": self.cx, "cy": self.cy, "r0_rel": self.r0_rel, "coef_bgr": self.coef}, fh, indent=1)

    @staticmethod
    def load(path: str) -> "ShadingModel":
        with open(path) as fh:
            d = json.load(fh)
        return ShadingModel(d["cx"], d["cy"], d["r0_rel"], d["coef_bgr"])
