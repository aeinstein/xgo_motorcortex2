#!/usr/bin/env python3
"""FloorNet: Labels je Bild und Spalte aus Rohaufnahmen (xgonav --record-raw).

Je Fahrt entsteht aus allen Bildern eine Karte (Kantenverfahren, Pose neu mit IMU-Kurs gerechnet,
Personen ausgenommen, gefahrener Weg gilt als frei). Diese Karte wird dann in jedes Bild
zurückprojiziert: So lehrt das, was aus der Nähe sicher erkannt wurde, das Bild von weiter weg, und
ein Fehlalarm, den nur wenige Blickwinkel sehen, fällt heraus.

  python tools/floornet/label.py DIR... --config rider.toml --out labels/ [--jobs 8] [--preview 12]
                                 [--wohnung a] [--set grid.max_range_m=1.0]

Eine Spalte ist danach eines von:
  EXACT    Hindernis, Bodengrenze in Zeile row (Vollbild-Pixel)
  ATLEAST  bis Zeile row sicher frei, was dahinter kommt, ist unbekannt
  IGNORE   nichts bekannt oder eine Person in der Spalte (zählt nicht im Training)

Grenzen: Die Karte stammt selbst aus dem Kantenverfahren. Was es aus jedem Blickwinkel falsch
sieht (dünne Stuhlbeine), steht auch hier falsch. Ein Vergleich gegen `kanten` braucht darum von
Hand beschriftete Bilder, nicht diese Labels.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replay_raw import set_param  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.freespace import FreeSpaceEstimator  # noqa: E402
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
MAX_TURN_DPS = 30.0        # Kurs ist nur mit ~5 Hz bekannt: bei schneller Drehung passt die Pose nicht zum Bild
NEW_VIEW_M = 0.10          # Karte: neuer Blick erst nach so viel Fahrt; Drehen auf der Stelle zeigt nichts Neues
MIN_VIEWS = 2              # belegte Zelle braucht Treffer aus so vielen Blicken
SNAP_D_M = 0.7             # bis hierhin ist die Kantenzeile feiner als das Raster (5 cm = 3,3 Zeilen bei 0,8 m)
SNAP_TOL_M = 0.10          # Kantenzeile gilt nur, wenn sie höchstens so weit vom Karten-Label entfernt liegt


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


def moved_enough(a, b, dist: float = NEW_VIEW_M) -> bool:
    """Reicht die Fahrt von Pose a zu Pose b (x, y, ...) für einen neuen Blick?"""
    return math.hypot(b[0] - a[0], b[1] - a[1]) >= dist


def build_map(d: str, ms: list[dict], traj, cfg: Config, geo: CamGeometry):
    """Kantenverfahren über die Bilder (jedes zweite, wie live), Karte aufbauen, gefahrenen Weg
    freiräumen. Rückgabe: Karte, Liste der gerechneten Bildindizes, Grenzzeilen je Bild (Vollbild)."""
    x, y, th, rate = traj
    fse = FreeSpaceEstimator(cfg.freespace, (geo.W, geo.H), geo)
    gm = GridMap(cfg.grid, geo, fse.W, fse.H)
    used, rows = [], []
    sc = geo.W / fse.W
    # Ein Hindernis muss aus mehreren Standorten an derselben Stelle liegen. Stand der Rider 72 s still
    # (explore3), brannte sich sonst die Spiegelung einer Lampe ein; von anderswo wandert sie. Eine Drehung
    # auf der Stelle zählt nicht: Sie bestätigte Fehlalarme im Dunkeln (unten1).
    views = np.zeros(gm.lo.shape, np.uint16)    # Anzahl Blicke mit Treffer je Zelle
    seen = np.zeros(gm.lo.shape, bool)          # Treffer im laufenden Blick
    anchor = None
    cap = cfg.grid.l_max - cfg.grid.l_occ
    for i, m in enumerate(ms):
        if i % 2:
            continue
        img = cv2.imread(os.path.join(d, m["file"]))
        if img is None:
            continue
        boxes = [tr["box"] for tr in m.get("tracks_all", m.get("tracks", []))]
        fs = fse.update(img, m["pitch_dev"], boxes)
        span = fse.H - fs.horizon_row
        rows.append(((fse.H - fs.columns * span) * sc).astype(np.float32))
        used.append(i)
        if rate[i] > MAX_TURN_DPS:
            continue
        pose = (float(x[i]), float(y[i]), float(th[i]))
        if anchor is None or moved_enough(anchor, pose):
            views += seen
            seen[:] = False
            anchor = pose
        mv = movable_columns(m, geo.W, fse.W)
        if mv.any():
            fs.columns = fs.columns.copy()
            fs.columns[mv] = np.nan
        np.minimum(gm.lo, cap, out=gm.lo)       # sonst zeigt eine volle Zelle keinen weiteren Treffer mehr
        before = gm.lo.copy()
        gm.add_freespace(fs, m["pitch_dev"], pose)
        seen |= gm.lo > before
    views += seen
    gm.lo[(gm.lo > 0) & (views < MIN_VIEWS)] = 0.0   # nur aus einem Blick gesehen: gilt als unbekannt
    for a, b in zip(x[::5], y[::5]):
        gm._clear_disk(float(a), float(b), cfg.grid.driven_free_r_m)   # wo er fuhr, ist Boden
    gm.path = list(zip(x[::5].tolist(), y[::5].tolist()))
    gm.x, gm.y, gm.th = float(x[-1]), float(y[-1]), float(th[-1])
    return gm, used, np.array(rows, np.float32)


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


def snap_near(rows, kinds, kanten_rows, geo: CamGeometry, pitch, bias_px: float, d_max: float = SNAP_D_M,
              tol_m: float = SNAP_TOL_M) -> np.ndarray:
    """Nahbereich genauer: Das 5-cm-Raster ist dort gröber als das Bild (bei 0,3 m rund 23 Zeilen). Die Karte
    sagt, dass dort etwas steht; liegt die Kantenzeile dieses Bildes in der Nähe, gilt sie. bias_px ist der
    feste Versatz der Kanten (grid.row_bias), den auch die Karte abzieht. Rückgabe: neue Zeilen."""
    idx = np.linspace(0, kanten_rows.shape[1] - 1, rows.shape[1]).round().astype(int)
    kr = kanten_rows[:, idx] - bias_px
    pitch = np.asarray(pitch)[:, None]
    near = row_dist(geo, rows, pitch)
    with np.errstate(invalid="ignore"):
        use = (kinds == EXACT) & (near < d_max) & (np.abs(near - row_dist(geo, kr, pitch)) <= tol_m)
    return np.where(use, kr, rows)


def label_stats(rows, kinds, kanten_rows, geo: CamGeometry, pitch) -> dict:
    """Anteile je Art und Abstand der Labels zum Kantenverfahren je Entfernungsband (m). Das ist
    keine Qualitätsmessung (die Labels stammen daraus), zeigt aber grobe Fehler und Versatz."""
    idx = np.linspace(0, kanten_rows.shape[1] - 1, rows.shape[1]).round().astype(int)
    kr = kanten_rows[:, idx]
    out = {"ignore": float((kinds == IGNORE).mean()), "exact": float((kinds == EXACT).mean()),
           "atleast": float((kinds == ATLEAST).mean())}
    hz = geo.H / 2 + geo.f * np.tan(math.radians(geo.tilt_deg) + np.asarray(pitch))[:, None]
    d = geo.cam_h * geo.f / np.maximum(rows - hz, 1e-3)    # Näherung für die Bandeinteilung
    for lo_, hi_ in ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5)):
        m = (kinds == EXACT) & (d >= lo_) & (d < hi_) & np.isfinite(kr)
        if m.sum() > 50:
            e = kr[m] - rows[m]                            # Pixel im Vollbild, > 0: Kanten sehen sie näher
            out["%.1f-%.1f" % (lo_, hi_)] = {"n": int(m.sum()), "versatz_px": round(float(np.median(e)), 1),
                                             "streu_px": round(float(np.median(np.abs(e - np.median(e)))), 1)}
    return out


def preview(d: str, ms, used, rows, kinds, kanten_rows, out_png: str, n: int = 12, picks=None) -> None:
    """Übersicht zum Anschauen: gelb Kantenverfahren, grün Label EXACT, blau ATLEAST, rot IGNORE."""
    picks = picks if picks is not None else np.linspace(0, len(used) - 1, n + 2)[1:-1].round().astype(int)
    tiles = []
    for k in picks:
        img = cv2.imread(os.path.join(d, ms[used[k]]["file"]))
        H, W = img.shape[:2]
        xs = ((np.arange(rows.shape[1]) + 0.5) * W / rows.shape[1]).astype(int)
        col = ((np.arange(kanten_rows.shape[1]) + 0.5) * W / kanten_rows.shape[1]).astype(int)
        for a in range(len(col) - 1):
            if np.isfinite(kanten_rows[k, a]) and np.isfinite(kanten_rows[k, a + 1]):
                cv2.line(img, (col[a], int(kanten_rows[k, a])), (col[a + 1], int(kanten_rows[k, a + 1])), (0, 220, 255), 1)
        for c, x_ in enumerate(xs):
            r = int(np.clip(rows[k, c], 0, H - 1))
            if kinds[k, c] == EXACT:
                cv2.circle(img, (x_, r), 3, (60, 220, 60), -1)
            elif kinds[k, c] == ATLEAST:
                cv2.line(img, (x_ - 3, r), (x_ + 3, r), (255, 120, 40), 2)
            else:
                cv2.line(img, (x_, H - 8), (x_, H - 1), (40, 40, 230), 2)
        cv2.putText(img, "%s %s" % (Path(d).name, ms[used[k]]["file"][:6]), (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(cv2.resize(img, (480, 360), interpolation=cv2.INTER_AREA))
    cols = 4
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
    cv2.imwrite(out_png, grid, [cv2.IMWRITE_JPEG_QUALITY, 85] if out_png.endswith(".jpg") else [])


def load_cfg(path: str | None, sets: list[str]) -> Config:
    cfg = Config.load(path)
    if path and cfg.freespace.shading_file and not os.path.isabs(cfg.freespace.shading_file):
        cfg.freespace.shading_file = str(Path(path).resolve().parent / cfg.freespace.shading_file)
    cfg.grid.native = False
    cfg.freespace.native = False
    for s in sets:
        set_param(cfg, s)
    return cfg


def make_geo(cfg: Config) -> CamGeometry:
    return CamGeometry.from_cfg(cfg)


def process(job: tuple) -> dict:
    d, cfg_path, sets, out_dir, wohnung, n_preview = job
    cv2.setNumThreads(1)
    run = Path(d).name
    cfg = load_cfg(cfg_path, sets)
    geo = make_geo(cfg)
    ms = select_segment(load_run(d))
    if len(ms) < 100:
        return {"run": run, "fehler": "keine brauchbare Bahn (%d Bilder)" % len(ms)}
    traj = trajectory(ms)
    gm, used, kanten_rows = build_map(d, ms, traj, cfg, geo)
    x, y, th, rate = traj
    idx = np.array(used)
    pitch = np.array([ms[i]["pitch_dev"] for i in used], np.float32)
    rows, kinds = cast_labels(gm, geo, x[idx], y[idx], th[idx], pitch)
    rows = snap_near(rows, kinds, kanten_rows, geo, pitch, cfg.grid.row_bias * geo.W / kanten_rows.shape[1])
    for k, i in enumerate(used):
        kinds[k][movable_columns(ms[i], geo.W, NCOLS)] = IGNORE
    os.makedirs(out_dir, exist_ok=True)
    geom = {"W": geo.W, "H": geo.H, "f": geo.f, "cam_h": geo.cam_h, "tilt_deg": geo.tilt_deg, "hfov_deg": geo.hfov_deg}
    np.savez_compressed(os.path.join(out_dir, run + ".npz"), files=np.array([ms[i]["file"] for i in used]),
                        t=np.array([ms[i]["t"] for i in used]), x=x[idx], y=y[idx], th=th[idx], rate=rate[idx],
                        pitch=pitch, row=rows, kind=kinds, kanten_row=kanten_rows, lo=gm.lo.astype(np.float16),
                        run=run, wohnung=wohnung, geom=json.dumps(geom), source_dir=str(Path(d).resolve()))
    stats = {"run": run, "bilder": len(used), "ruhig": float((rate[idx] <= MAX_TURN_DPS).mean()),
             "weg_m": float(np.hypot(np.diff(x), np.diff(y)).sum()),
             "belegt_m2": float((gm.lo > gm.cfg.l_show).sum()) * gm.res ** 2,
             "frei_m2": float((gm.lo < -gm.cfg.l_show).sum()) * gm.res ** 2,
             **label_stats(rows, kinds, kanten_rows, geo, pitch)}
    cv2.imwrite(os.path.join(out_dir, run + "_karte.png"), gm.render(640, 10.0, (float(x.mean()), float(y.mean())))[0])
    if n_preview:
        preview(d, ms, used, rows, kinds, kanten_rows, os.path.join(out_dir, run + "_vorschau.jpg"), n_preview)
    return stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--config", help="TOML des Riders (mit shading_file daneben)")
    ap.add_argument("--out", default="labels")
    ap.add_argument("--wohnung", default="a", help="Kennung der Wohnung, damit später fremde dazukommen können")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--preview", type=int, default=0)
    ap.add_argument("--set", action="append", default=[], metavar="a.b=wert")
    a = ap.parse_args()
    jobs = [(d, a.config, a.set, a.out, a.wohnung, a.preview) for d in a.dirs]
    with Pool(min(a.jobs, len(jobs))) as pool:
        for r in pool.imap_unordered(process, jobs):
            print(json.dumps(r, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
