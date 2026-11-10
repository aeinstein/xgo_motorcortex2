#!/usr/bin/env python3
"""FloorNet und Raumkarte einer Rohaufnahme als Daten für den Betrachter im Browser (netz_betrachter.html).

  python tools/floornet/fahrt_js.py lauf/ --fahrt data/rider/fahrt/20261110-191711 --karte raumkarte.npz \\
      --config rider/config.toml --out ~/floornet/netz [--name fahrt]

Je Bild: Grenzzeile des Netzes (Auswertung wie am Rider, mit Zacken herausnehmen), die Raumkarte von der
aufgezeichneten Pose aus ins Bild geworfen (je Spalte die erste belegte Zelle, wie label.cast_labels), Statuszeile,
Fahrbefehl und Pose. Dazu die Karte als Bild von oben. Nur PC/GPU-Rechner, am Rider kostet das nichts.

Grenzen: Die Karte ist der gespeicherte Endstand, nicht der Stand zur Zeit des Bildes. Was die Bahnprüfung anhält
("Bahn x m"), ist die Merkkarte um den Rider (localmap); die steht nicht in der Aufnahme.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

R_MAX = 4.0             # so weit wird die Karte ins Bild geworfen (m)
RAND = 12               # Zellen Rand um den bekannten Teil der Karte im Bild von oben
ZOOM = 6                # Bildpunkte je Zelle


def karte_laden(path: str, cfg) -> SimpleNamespace:
    """Gespeicherte Raumkarte, so weit label.cast_labels sie braucht."""
    with np.load(path) as d:
        lo, res, n = d["lo"].astype(np.float32), float(d["res"]), int(d["n"])

    def cell(x, y):
        return (np.floor(np.asarray(y) / res + n / 2).astype(np.int64),
                np.floor(np.asarray(x) / res + n / 2).astype(np.int64))

    return SimpleNamespace(lo=lo, res=res, n=n, cfg=cfg.grid, cell=cell)


def karte_bild(gm: SimpleNamespace, extra_xy: np.ndarray) -> tuple[np.ndarray, dict]:
    """Karte von oben (y nach oben): belegt hell, frei mittel, unbekannt dunkel. Dazu die Lage im Weltmaß."""
    thr = gm.cfg.l_show
    bekannt = np.argwhere(np.abs(gm.lo) > thr)
    er, ec = gm.cell(extra_xy[:, 0], extra_xy[:, 1])
    rr = np.concatenate([bekannt[:, 0], er]) if len(bekannt) else er
    cc = np.concatenate([bekannt[:, 1], ec]) if len(bekannt) else ec
    r0, r1 = max(0, int(rr.min()) - RAND), min(gm.n, int(rr.max()) + RAND + 1)
    c0, c1 = max(0, int(cc.min()) - RAND), min(gm.n, int(cc.max()) + RAND + 1)
    lo = gm.lo[r0:r1, c0:c1]
    img = np.full(lo.shape + (3,), 28, np.uint8)
    img[lo < -thr] = (95, 95, 95)
    img[lo > thr] = (235, 235, 235)
    img = cv2.resize(img[::-1], None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_NEAREST)
    # Welt -> Bild: px = (x - x0) / m_px, py = (y1 - y) / m_px
    lage = {"x0": (c0 - gm.n / 2) * gm.res, "y1": (r1 - gm.n / 2) * gm.res, "m_px": gm.res / ZOOM}
    return img, lage


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lauf", help="Verzeichnis mit floornet.pt")
    ap.add_argument("--fahrt", required=True, help="Rohaufnahme (meta.jsonl + Bilder)")
    ap.add_argument("--karte", required=True, help="gespeicherte Raumkarte (raumkarte.npz)")
    ap.add_argument("--config", required=True, help="config.toml des Riders")
    ap.add_argument("--out", required=True, help="Datenverzeichnis des Betrachters")
    ap.add_argument("--name", default="fahrt", help="Name des Laufs im Betrachter")
    a = ap.parse_args()

    import torch

    import export as X
    import net as N
    from label import EXACT, cast_labels
    from netz_js import index_schreiben
    from xgonav.config import Config
    from xgonav.geometry import CamGeometry

    fahrt = os.path.basename(os.path.normpath(a.fahrt))
    with open(os.path.join(a.fahrt, "meta.jsonl"), encoding="utf-8") as f:
        ms = [json.loads(z) for z in f if z.strip()]
    cfg = Config.load(a.config)
    cfg_px = cfg.freespace.net_outlier_px, cfg.freespace.net_outlier_ratio
    hoehe = float(np.median([m.get("height_mm", cfg.behavior.height_cruise_mm) for m in ms]))
    geo = CamGeometry.from_cfg(cfg, hoehe)
    gm = karte_laden(a.karte, cfg)

    pose = np.array([m["pose"] for m in ms], float)
    x, y, th = pose[:, 0], pose[:, 1], np.radians(pose[:, 2])
    pitch = np.array([m.get("pitch_dev", 0.0) for m in ms], float)
    krow, kart = cast_labels(gm, geo, x, y, th, pitch, ncols=N.NCOLS, r_max=R_MAX)

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    net = N.load_net(a.lauf, dev).eval()
    rows = []
    with torch.no_grad():
        for i in range(0, len(ms), 64):
            xs = torch.cat([X.to_input(cv2.imread(os.path.join(a.fahrt, m["file"]))) for m in ms[i:i + 64]]).to(dev)
            rows.append(N.decode(net(xs), *cfg_px).cpu().numpy())
    rows = np.concatenate(rows)

    ziel = os.path.join(a.out, a.name)
    os.makedirs(ziel, exist_ok=True)
    img, lage = karte_bild(gm, pose[:, :2])
    cv2.imwrite(os.path.join(ziel, fahrt + "_karte.png"), img)
    t0 = ms[0]["t"]
    daten = {
        "lauf": a.name, "fahrt": fahrt, "files": [m["file"] for m in ms],
        "rows": [[round(float(v), 1) for v in z] for z in rows],
        "karte": [[round(float(v), 1) if k == EXACT and np.isfinite(v) and v < geo.H else None for v, k in zip(z, ka)]
                  for z, ka in zip(krow, kart)],
        "text": ["%.1f s | v %+.2f w %+.2f | %s" % (m["t"] - t0, m["sent"][0], m["sent"][1], m.get("status", "")) for m in ms],
        "pose": [[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 1)] for p in pose],
        "plan": dict(lage, bild=fahrt + "_karte.png", hfov=geo.hfov_deg, vor=geo.cam_fwd),
    }
    with open(os.path.join(ziel, fahrt + ".js"), "w", encoding="utf-8") as f:
        f.write("window.NETZ = " + json.dumps(daten, separators=(",", ":"), ensure_ascii=False) + ";\n")
    index_schreiben(a.out)
    print("%s: %d Bilder, Körperhöhe %.1f mm, Karte %dx%d px, Spalten mit Kartentreffer %.0f %%" % (
        fahrt, len(ms), hoehe, img.shape[1], img.shape[0], 100 * float((kart == EXACT).mean())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
