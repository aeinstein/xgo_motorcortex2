"""FloorNet: Hilfsfunktionen für Label-Dateien und Rohaufnahmen (xgonav --record-raw).

Früher entstanden hier Labels aus einer Raumkarte des klassischen Bodenmodells; das ist mit diesem entfernt.
Labels kommen heute aus ComfyUI (stage3_npz.py, comfy_npz.py) oder aus Handlinien (handmap.py, hand_npz.py).
Geblieben sind, was andere Werkzeuge nutzen:
  ground_row, row_dist         Bodengeometrie Zeile <-> Abstand
  load_run, select_segment     Metazeilen einer Aufnahme, brauchbares Stück daraus
  trajectory                   Bahn mit dem Firmware-Kurs neu gerechnet
  movable_columns              Spalten mit beweglichen Objekten (Detektor)
  cast_labels                  Raumkarte in die Bilder projizieren (handmap.py, fahrt_js.py)
  load_cfg, make_geo           Konfiguration und Kamerageometrie

Eine Spalte ist eines von:
  EXACT    Hindernis, Bodengrenze in Zeile row (Vollbild-Pixel)
  ATLEAST  bis Zeile row sicher frei, was dahinter kommt, ist unbekannt
  IGNORE   nichts bekannt oder eine Person in der Spalte (zählt nicht im Training)
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replay_raw import set_param  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402

IGNORE, EXACT, ATLEAST = 0, 1, 2
MOVABLE = {"person", "dog", "cat"}
ACTIVE_MODES = {"explore", "patrol", "manual", "home", "follow", "approach"}
# Nick aus der IMU: Komplementärfilter ("imu") oder seit dem Horizont-Modul (2026-11-07) IMU gegen die
# Standreferenz je Aufstehen, mit Bildanteil ("stand+bild") oder ohne ("stand"). Nicht: "bild" allein.
IMU_PITCH = {"imu", "stand", "stand+bild"}
NCOLS = 80                 # Spalten je Bild (8 Vollbild-Pixel), wie die Netzausgabe
JUMP_M = 0.12              # größerer Posesprung zwischen zwei Bildern: nicht mehr dieselbe Bahn


def ground_row(geo: CamGeometry, d_fwd, pitch_dev: float = 0.0):
    """Bildzeile (Vollbild) eines Bodenpunkts d_fwd Meter voraus; Umkehrung von geo.ground_dist."""
    beta = np.arctan2(geo.cam_h, np.asarray(d_fwd, float))
    return geo.H / 2 + geo.f * np.tan(beta + math.radians(geo.tilt_deg) + pitch_dev)


def row_dist(geo: CamGeometry, row, pitch_dev):
    """Bildzeile -> Abstand nach vorn, Umkehrung von ground_row; NaN am oder über dem Horizont."""
    beta = np.arctan2(np.asarray(row, float) - geo.H / 2, geo.f) - math.radians(geo.tilt_deg) - pitch_dev
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(beta > math.radians(0.5), geo.cam_h / np.tan(beta), np.nan)


def load_run(d: str) -> list[dict]:
    """Metazeilen des längsten Laufs im Verzeichnis, deren Bild von diesem Lauf stammt.
    (Vor 2026-11-06 überschrieb ein Neustart die Bilder im selben Verzeichnis; explore1.)"""
    runs, cur = [], []
    for line in open(os.path.join(d, "meta.jsonl"), encoding="utf-8"):
        try:
            m = json.loads(line)
        except ValueError:
            continue                    # letzte Zeile beim Abschalten abgeschnitten
        if cur and m["idx"] <= cur[-1]["idx"]:
            runs.append(cur)
            cur = []
        cur.append(m)
    if cur:
        runs.append(cur)
    if not runs:
        return []
    best = max(range(len(runs)), key=lambda i: len(runs[i]))
    later = {m["file"] for r in runs[best + 1:] for m in r}
    return [m for m in runs[best] if m["file"] not in later and os.path.exists(os.path.join(d, m["file"]))]


def select_segment(ms: list[dict]) -> list[dict]:
    """Längstes zusammenhängendes Stück ab dem ersten fahrenden Modus: balancierend, IMU-Nick,
    Pose bekannt, ohne Posesprung. Davor liegen Aufstehen und das Rücksetzen der Pose beim Start."""
    first = next((i for i, m in enumerate(ms) if m.get("mode") in ACTIVE_MODES), None)
    if first is None:
        return []
    segs, cur, last = [], [], None
    for m in ms[first:]:
        ok = m.get("pose") and m.get("yaw_drv") is not None and m.get("balancing") and m.get("pitch_src") in IMU_PITCH
        if ok and last is not None and math.hypot(m["pose"][0] - last[0], m["pose"][1] - last[1]) > JUMP_M:
            ok = False
        if ok:
            cur.append(m)
            last = m["pose"]
        else:
            if cur:
                segs.append(cur)
            cur, last = [], None
    if cur:
        segs.append(cur)
    return max(segs, key=len) if segs else []


def trajectory(ms: list[dict]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Bahn neu rechnen: Strecke aus der aufgezeichneten Pose, Kurs aus der Firmware-IMU.
    Die IMU ist nur als Treppe bekannt (~5 Hz, ganze Grad), darum zwischen ihren Änderungen linear
    interpoliert. Räder rutschen beim Drehen (Fahrt 6: 75 cm nach 32 m), die IMU kaum.
    Rückgabe x, y, th (rad), Drehrate (°/s)."""
    t = np.array([m["t"] for m in ms], float)
    pose = np.array([m["pose"] for m in ms], float)
    drv = np.array([m["yaw_drv"] for m in ms], float)
    delta = (np.diff(drv) + 180.0) % 360.0 - 180.0
    unwrapped = np.concatenate([[0.0], np.cumsum(delta)])
    keep = np.concatenate([[True], delta != 0.0])
    keep[-1] = True
    yaw = np.interp(t, t[keep], unwrapped[keep])        # Grad seit dem ersten Bild
    th = math.radians(pose[0, 2]) + np.radians(yaw)
    # Strecke mit dem Kurs der Pose selbst zurückrechnen (so wurde sie integriert), vorzeichenrichtig
    pth = np.radians(pose[:, 2])
    mid = (pth[1:] + pth[:-1]) / 2
    ds = np.diff(pose[:, 0]) * np.cos(mid) + np.diff(pose[:, 1]) * np.sin(mid)
    tm = (th[1:] + th[:-1]) / 2
    x = pose[0, 0] + np.concatenate([[0.0], np.cumsum(ds * np.cos(tm))])
    y = pose[0, 1] + np.concatenate([[0.0], np.cumsum(ds * np.sin(tm))])
    # Drehrate über etwa eine IMU-Stufe (0,2 s), einzelne Bilder sind zu kurz
    rate = np.zeros(len(t))
    j = np.searchsorted(t, t - 0.1)
    k = np.minimum(np.searchsorted(t, t + 0.1), len(t) - 1)
    span = np.maximum(t[k] - t[j], 1e-3)
    rate[:] = np.abs(yaw[k] - yaw[j]) / span
    return x, y, th, rate


def movable_columns(m: dict, W: int, ncols: int) -> np.ndarray:
    """Spalten (bool, ncols), in denen laut Detektor gerade etwas Bewegliches steht."""
    out = np.zeros(ncols, bool)
    for tr in m.get("tracks_all", m.get("tracks", [])):
        if tr.get("cls") in MOVABLE:
            x1, _, x2, _ = tr["box"]
            out[max(0, int(x1 / W * ncols) - 1):min(ncols, int(x2 / W * ncols) + 2)] = True
    return out


def cast_labels(gm: GridMap, geo: CamGeometry, x, y, th, pitch, ncols: int = NCOLS, r_max: float = 2.5,
                r_min: float = 0.15, min_free: float = 0.5, tol_cells: float = 2.0,
                l_thr: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Karte in die Bilder projizieren. x, y, th, pitch je Bild. Rückgabe: Zeile, Art (je N x ncols).
    Je Spalte ein Strahl: Trifft er eine belegte Zelle, bevor er länger durch Unbekanntes lief, ist
    das die Grenze (EXACT). Sonst ist er bis zum Unbekannten frei (ATLEAST), wenn das mindestens
    min_free weit ist; davor ist nichts bekannt (IGNORE)."""
    thr = gm.cfg.l_show if l_thr is None else l_thr
    res = gm.res
    s = np.arange(r_min, r_max, res / 2)
    x_c = (np.arange(ncols) + 0.5) * geo.W / ncols
    bear = geo.bearing(x_c)
    cosb = np.cos(bear)
    tol = int(round(tol_cells * 2))
    N = len(x)
    rows = np.zeros((N, ncols), np.float32)
    kinds = np.zeros((N, ncols), np.uint8)
    lo = gm.lo
    off = geo.cam_fwd                       # Pose = Radachse, Strahlen ab Kamerafußpunkt (wie GridMap)
    for i in range(N):
        ang = th[i] + bear
        wx = x[i] + off * math.cos(th[i]) + np.outer(np.cos(ang), s)
        wy = y[i] + off * math.sin(th[i]) + np.outer(np.sin(ang), s)
        r, c = gm.cell(wx, wy)
        inside = (r >= 0) & (r < gm.n) & (c >= 0) & (c < gm.n)
        val = np.zeros(r.shape, np.float32)
        val[inside] = lo[r[inside], c[inside]]
        occ = val > thr
        free = val < -thr
        unk = ~occ & ~free
        n = len(s)
        hit = np.where(occ.any(axis=1), occ.argmax(axis=1), n)
        unk_i = np.where(unk.any(axis=1), unk.argmax(axis=1), n)
        is_hit = (hit < n) & (hit <= unk_i + tol)
        dist = np.where(is_hit, s[np.minimum(hit, n - 1)], s[np.minimum(unk_i, n - 1)])
        dist = np.where(~is_hit & (unk_i >= n), r_max, dist)
        kind = np.where(is_hit, EXACT, np.where(dist >= min_free, ATLEAST, IGNORE))
        rows[i] = ground_row(geo, dist * cosb, pitch[i])
        kinds[i] = kind
    return rows, kinds


def load_cfg(path: str | None, sets: list[str]) -> Config:
    """Konfiguration für die Werkzeuge: Raumkarte in Python (am PC fehlt der C++-Kern meist), dazu --set a.b=wert."""
    cfg = Config.load(path)
    cfg.grid.native = False
    for s in sets:
        set_param(cfg, s)
    return cfg


def make_geo(cfg: Config) -> CamGeometry:
    return CamGeometry.from_cfg(cfg)

