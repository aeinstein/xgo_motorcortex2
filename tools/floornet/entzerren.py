#!/usr/bin/env python3
"""Bilder, Masken und Punkte mit dem Linsenmodell des Riders entzerren (wie xgonav/camera.py seit 2026-11-18).

  python tools/floornet/entzerren.py bild  EIN_ORDNER AUS_ORDNER            # Kamerabilder (*.jpg, *.png), bilinear
  python tools/floornet/entzerren.py maske EIN_ORDNER AUS_ORDNER            # Masken/Labelbilder, nächster Nachbar
  python tools/floornet/entzerren.py punkt x y [x y ...]                    # gemessene Pixel -> entzerrte Pixel

Seit 2026-11-18 entzerrt der Rider jedes Bild direkt nach der Kamera; FloorNet bekommt dort nur noch entzerrte
Bilder. Trainingsbilder aus älteren Aufnahmen und alles, was daraus abgeleitet ist (Grünbilder, Masken, Labels),
müssen deshalb genauso entzerrt werden. Aufnahmen, in deren meta.jsonl "cam": {"entzerrt": true} steht, sind es
schon: nicht doppelt entzerren.

Bilder: dieselbe Tabelle und Rechnung wie am Rider (camera.lens_maps, cv2.remap INTER_LINEAR). Masken: dieselbe
Tabelle mit nächstem Nachbarn, damit keine Zwischenwerte entstehen. Zeilen-Labels je Spalte (Stage3) danach aus
der entzerrten Maske neu bilden, nicht umrechnen. Punkte (Handlinien): vorwaerts(), gemessen -> ideal.
Das Modell steht in xgonav/config.py ([camera] lens_knots_px, lens_g_px, lens_center); Bildgröße 640x480.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xgonav.camera import lens_maps  # noqa: E402
from xgonav.config import Config  # noqa: E402


def vorwaerts(pts, cam=None) -> np.ndarray:
    """Gemessene Pixel (…, 2) -> entzerrte Pixel: ideal = c + q * (1 + g(r) / r)."""
    cam = cam or Config().camera
    kn, g = np.asarray(cam.lens_knots_px, float), np.asarray(cam.lens_g_px, float)
    c = np.asarray(cam.lens_center, float)
    q = np.asarray(pts, float) - c
    r = np.hypot(q[..., 0], q[..., 1])
    return c + q * (1.0 + np.interp(r, kn, g) / np.maximum(r, 1e-6))[..., None]


def tabellen(cam=None):
    """(Festkomma-Tabellen für Bilder, ganzzahlige Quellpixel für Masken)."""
    cam = cam or Config().camera
    m1, m2 = lens_maps(cam)
    # nächster Nachbar: Quellpixel = gerundete Quellkoordinate (m1 ganzzahlig + Bruchteil in 1/32 aus m2)
    fx, fy = (m2 & 31).astype(np.float32) / 32.0, (m2 >> 5).astype(np.float32) / 32.0
    nx = np.clip(np.rint(m1[..., 0] + fx), 0, cam.width - 1).astype(np.float32)
    ny = np.clip(np.rint(m1[..., 1] + fy), 0, cam.height - 1).astype(np.float32)
    return (m1, m2), (nx, ny)


def bild(img: np.ndarray, tab=None) -> np.ndarray:
    tab = tab or tabellen()
    return cv2.remap(img, tab[0][0], tab[0][1], cv2.INTER_LINEAR)


def maske(img: np.ndarray, tab=None) -> np.ndarray:
    tab = tab or tabellen()
    return cv2.remap(img, tab[1][0], tab[1][1], cv2.INTER_NEAREST)


def main() -> int:
    if len(sys.argv) < 4 or sys.argv[1] not in ("bild", "maske", "punkt"):
        print(__doc__)
        return 2
    if sys.argv[1] == "punkt":
        p = np.array([float(v) for v in sys.argv[2:]]).reshape(-1, 2)
        for a, b in zip(p, vorwaerts(p)):
            print("%.2f %.2f -> %.2f %.2f" % (a[0], a[1], b[0], b[1]))
        return 0
    ein, aus = sys.argv[2], sys.argv[3]
    if os.path.abspath(ein) == os.path.abspath(aus):
        print("Ein- und Ausgabeordner sind gleich: nichts überschrieben.", file=sys.stderr)
        return 2
    tab = tabellen()
    cam = Config().camera
    fn = bild if sys.argv[1] == "bild" else maske
    n = 0
    for wurzel, _, dateien in os.walk(ein):
        for f in sorted(dateien):
            if not f.lower().endswith((".jpg", ".jpeg", ".png")):
                continue
            img = cv2.imread(os.path.join(wurzel, f), cv2.IMREAD_UNCHANGED)
            if img is None or img.shape[:2] != (cam.height, cam.width):
                print("übersprungen (nicht %dx%d): %s" % (cam.width, cam.height, os.path.join(wurzel, f)), file=sys.stderr)
                continue
            ziel = os.path.join(aus, os.path.relpath(wurzel, ein))
            os.makedirs(ziel, exist_ok=True)
            cv2.imwrite(os.path.join(ziel, f), fn(img, tab))
            n += 1
    print("%d Dateien entzerrt -> %s" % (n, aus))
    return 0


if __name__ == "__main__":
    sys.exit(main())
