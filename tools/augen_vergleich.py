"""Kontaktabzug Augengenerator (C, Display-Daemon) neben tools/eyes.py (Python): je Ausdruck drei Zeilen
(Python, C, Unterschied: rot = Python mehr, grün = C mehr, ×3 verstärkt) zu 8 Zeitpunkten.

  python tools/augen_vergleich.py DIR [--only NAME]...     # DIR/<Name>.png und DIR/alle.png

Braucht libxgoaugen.so (XGONAV_AUGENLIB oder gcc, siehe tools/augenlib.py); nur am PC/kimmy, nie am Rider nötig.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import augenlib  # noqa: E402
import eyes  # noqa: E402


def blatt(lib, name: str, n: int = 8, farbe=eyes.FARBE) -> np.ndarray:
    fn, T = eyes.EMOTIONS[name]
    zeilen = [[], [], []]
    for i in range(n):
        t = T * i / (n - 1)
        frame = fn(t, T)
        py = eyes.render(frame)
        c = augenlib.raster(lib, augenlib.aus_python(frame))
        d = py - c
        diff = np.zeros((eyes.H, eyes.W, 3), np.uint8)
        diff[..., 2] = np.clip(d * 765, 0, 255)
        diff[..., 1] = np.clip(-d * 765, 0, 255)
        diff[..., 0] = (np.minimum(py, c) * 70).astype(np.uint8)
        for k, bild in enumerate((eyes.to_bgr(py, farbe), eyes.to_bgr(c, farbe), diff)):
            kachel = cv2.copyMakeBorder(bild, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=(70, 70, 70))
            if k == 0:
                cv2.putText(kachel, f"{t:.2f} s", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1,
                            cv2.LINE_AA)
            zeilen[k].append(kachel)
    kopf = np.full((28, (eyes.W + 4) * n, 3), 30, np.uint8)
    cv2.putText(kopf, f"{name}: Python / C / Unterschied", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220),
                1, cv2.LINE_AA)
    return np.vstack([kopf] + [np.hstack(z) for z in zeilen])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dir", type=Path)
    ap.add_argument("--only", action="append", help="nur diesen Ausdruck (mehrfach möglich)")
    a = ap.parse_args()
    lib = augenlib.laden()
    if lib is None:
        print("libxgoaugen.so fehlt und kein gcc (make -C st7789/project/riderpi augen-lib)", file=sys.stderr)
        return 1
    a.dir.mkdir(parents=True, exist_ok=True)
    blaetter = []
    for name in a.only or list(eyes.EMOTIONS):
        b = blatt(lib, name)
        cv2.imwrite(str(a.dir / f"{name}.png"), b)
        blaetter.append(cv2.resize(b, (b.shape[1] // 2, b.shape[0] // 2), interpolation=cv2.INTER_AREA))
        print(name)
    cv2.imwrite(str(a.dir / "alle.png"), np.vstack(blaetter))
    return 0


if __name__ == "__main__":
    sys.exit(main())
