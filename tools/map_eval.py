#!/usr/bin/env python3
"""Raumkarte messen: Beobachtungen im Simulator aufzeichnen und offline durch GridMap spielen.

  python tools/map_eval.py record AUFNAHME.pkl --secs 90 [--scene zwei] [--set grid.l_occ=0.6]
  python tools/map_eval.py eval AUFNAHME.pkl [...] [--set grid.max_range_m=1.0,1.5,2.5] [--set ...]
      [--pitch cap|ins] [--pose cap|ins] [--rays]

record fährt einen Simulator-Lauf (Echtzeit, Rider-Einstellungen). eval dauert je Aufnahme nur
Sekunden: für jede Kombination der --set-Werte eine Zeile mit den Kennzahlen aus sim.map_quality
gegen die Wahrheit, gemittelt über die Aufnahmen. --pitch/--pose: Nick und Pose der Aufnahme (cap,
wie live) oder des späteren Bildes, in dem das Ergebnis ankam (ins, Fehler bis 2026-11-05).
--rays: Fehler der gemessenen Bodengrenze je Entfernung gegen die Wahrheit."""
from __future__ import annotations

import argparse
import itertools
import math
import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests"))
from sim_run import RIDER, RIDER_VX, apply_overrides, run  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.freespace import FreeSpace  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402
from xgonav.sim import SCENES, map_quality  # noqa: E402


def parse_sets(items) -> dict:
    """["a.b=1,2", ...] -> {"a.b": [1, 2]}"""
    out = {}
    for it in items or []:
        k, v = it.split("=")
        out[k] = [eval(x, {}) for x in v.split(",")]
    return out


def record(a) -> None:
    rec: list = []
    ov = dict(RIDER)
    ov.update({k: v[0] for k, v in parse_sets(a.set).items()})
    res = run(a.mode, a.secs, wheels=True, method="kanten", fs_thread=True, dyn=True, vx_true=RIDER_VX,
              overrides=ov, scene=a.scene, record=rec)
    start = res.pop("start_pose")
    data = {"obs": rec, "scene": a.scene, "start": start, "path": res.pop("path"), "res": res, "overrides": ov}
    with open(a.file, "wb") as f:
        pickle.dump(data, f)
    print(a.file, {k: res.get(k) for k in ("collisions", "dist_m", "frames")}, res.get("grid"), "Beobachtungen", len(rec))


def make_grid(cfg: Config, body_h: float) -> GridMap:
    c = cfg.camera
    geo = CamGeometry.from_cfg(cfg, body_h)
    fw = cfg.freespace.width
    return GridMap(cfg.grid, geo, fw, int(round(c.height * fw / c.width)))


def replay(data: dict, sets: dict, pitch: str, pose: str, max_pitch: float, max_turn: float):
    cfg = Config()
    apply_overrides(cfg, data["overrides"])
    apply_overrides(cfg, sets)
    cfg.grid.native = False
    scene = SCENES[data["scene"]]()
    g = make_grid(cfg, data["obs"][0]["body_h"])
    used = 0
    for o in data["obs"]:
        pd = o["pitch_" + pitch]
        if abs(pd) > max_pitch or abs(o["w"]) * cfg.robot.vyaw_limit_dps > max_turn or not o["balancing"]:
            continue
        fs = FreeSpace([], o["columns"], o["horizon_row"], None, True, o["known_from"])
        g.add_freespace(fs, pd, o["pose_" + pose])
        used += 1
    q = map_quality(g.lo, g.res, g.cfg.l_show, scene, data["start"], data["path"])
    q["genutzt"] = round(used / max(1, len(data["obs"])), 2)
    return q


def rays(data: dict, pitch: str) -> None:
    """Gemessene Bodengrenze gegen die wahre Entfernung entlang desselben Strahls."""
    cfg = Config()
    apply_overrides(cfg, data["overrides"])
    g = make_grid(cfg, data["obs"][0]["body_h"])
    scene = SCENES[data["scene"]]()
    seg = scene.segments()
    ax, ay, ex, ey = seg[:, 0], seg[:, 1], seg[:, 2] - seg[:, 0], seg[:, 3] - seg[:, 1]
    x0, y0, th0 = data["start"]
    c0, s0 = math.cos(th0), math.sin(th0)
    out = []
    sc = g.geo.W / g.fw
    for o in data["obs"]:
        cols = np.asarray(o["columns"], float)
        idx = np.arange(0, len(cols), 2)
        v = cols[idx]
        span = g.fh - o["horizon_row"]
        d = g.geo.ground_dist_np((g.fh - v * span) * sc, o["pitch_" + pitch])
        bear = np.arctan2(g.geo.W / 2 - (idx + 0.5) * sc, g.geo.f)
        px, py, pth = o["pose_cap"]
        wx, wy = x0 + px * c0 - py * s0, y0 + px * s0 + py * c0
        ang = pth + th0 + bear
        ca, sa = np.cos(ang)[:, None], np.sin(ang)[:, None]
        den = ca * ey - sa * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((ax - wx) * ey - (ay - wy) * ex) / den
            u = ((ax - wx) * sa - (ay - wy) * ca) / den
        t = np.where((t > 0) & (u >= 0) & (u <= 1) & np.isfinite(t), t, np.inf).min(axis=1)
        ok = np.isfinite(d) & np.isfinite(t) & (v < 0.98)
        out.append(np.column_stack([t[ok], d[ok] / np.cos(bear[ok])]))
    a = np.vstack(out)
    for lo, hi in ((0.15, 0.5), (0.5, 0.75), (0.75, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 9.0)):
        m = (a[:, 0] >= lo) & (a[:, 0] < hi)
        if not m.any():
            continue
        e = a[m, 1] - a[m, 0]
        print("  wahr %.2f-%.2f m: n=%6d  Median %+.2f  p10 %+.2f  p90 %+.2f  >15 cm zu weit %3.0f %%  zu nah %3.0f %%" % (
            lo, hi, m.sum(), np.median(e), np.percentile(e, 10), np.percentile(e, 90),
            100 * (e > 0.15).mean(), 100 * (e < -0.15).mean()))


def evaluate(a) -> None:
    datas = []
    for f in a.files:
        with open(f, "rb") as fh:
            datas.append(pickle.load(fh))
    sets = parse_sets(a.set)
    keys = list(sets)
    show = ("praezision_15cm", "waende_10cm", "abdeckung_10cm", "frei_aussen_m2", "frei_innen_m2", "boden_bekannt", "belegt_m2", "genutzt")
    print(" | ".join(keys + list(show)))
    for combo in itertools.product(*[sets[k] for k in keys]):
        s = dict(zip(keys, combo))
        qs = [replay(d, s, a.pitch, a.pose, math.radians(a.max_pitch), a.max_turn) for d in datas]
        vals = [sum(q[k] for q in qs) / len(qs) for k in show]
        print(" | ".join([str(c) for c in combo] + ["%.2f" % v for v in vals]), flush=True)
    if a.rays:
        for d in datas[:1]:
            print("Strahlen (Nick %s):" % a.pitch)
            rays(d, a.pitch)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("file")
    r.add_argument("--secs", type=float, default=90.0)
    r.add_argument("--mode", default="explore")
    r.add_argument("--scene", default="raum")
    r.add_argument("--set", action="append")
    e = sub.add_parser("eval")
    e.add_argument("files", nargs="+")
    e.add_argument("--set", action="append")
    e.add_argument("--pitch", default="cap", choices=("cap", "ins"))
    e.add_argument("--pose", default="cap", choices=("cap", "ins"))
    e.add_argument("--max-pitch", type=float, default=1e3, help="Beobachtung verwerfen ab |Nick| in Grad")
    e.add_argument("--max-turn", type=float, default=1e6, help="Beobachtung verwerfen ab Drehrate in °/s")
    e.add_argument("--rays", action="store_true")
    a = ap.parse_args()
    (record if a.cmd == "record" else evaluate)(a)


if __name__ == "__main__":
    main()
