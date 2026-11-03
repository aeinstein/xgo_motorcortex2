"""Rohaufnahme (xgonav --record-raw DIR) am PC durch Bodenmodell, Merkkarte und
Bahnprüfung spielen – mit den aufgezeichneten Nickwinkeln, Fahrbefehlen, Kursen und
Objekten, wie in App._step. Zum Vergleichen von Parametern ohne Fahrt am Rider.

  python tools/replay_raw.py DIR --config rider.toml
  python tools/replay_raw.py DIR --config rider.toml --set freespace.edge_weight=0.4 --set freespace.min_obst_rows=6
  python tools/replay_raw.py DIR --config rider.toml --no-objects     # erkannte Objekte nicht in die Karte
  python tools/replay_raw.py DIR --config rider.toml --video out.mp4  # Bodengrenze + Karte einzeichnen

Bewertet wird je Regelschritt (control_hz), ob die Bahn geradeaus mit max_v blockiert
wäre. Ist die Person (aus dem Status) weiter als --person-min entfernt, gilt eine
Blockade als Fehlalarm, sofern sonst nichts im Weg steht (Szene vorher prüfen).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.config import Config  # noqa: E402
from xgonav.freespace import FreeSpace, FreeSpaceEstimator, column_runs  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.localmap import LocalMap, mask_movable  # noqa: E402

DIST_RE = re.compile(r"verfolge #\d+ \S+ \([+-]?\d+°, ([\d.]+) m\)")


def set_param(cfg: Config, expr: str) -> None:
    key, val = expr.split("=", 1)
    obj = cfg
    parts = key.split(".")
    for p in parts[:-1]:
        obj = getattr(obj, p)
    cur = getattr(obj, parts[-1])
    setattr(obj, parts[-1], type(cur)(val) if not isinstance(cur, bool) else val.lower() in ("1", "true", "ja"))


def replay(d: str, cfg: Config, objects: bool = True, person_min: float = 1.2, video: str | None = None,
           quiet: bool = False, exclude: bool = True) -> dict:
    meta = [json.loads(line) for line in open(os.path.join(d, "meta.jsonl"))]
    W, H = cfg.camera.width, cfg.camera.height
    geo = CamGeometry(W, H, cfg.camera.hfov_deg, cfg.camera.tilt_deg, cfg.camera.mount_offset_mm,
                      cfg.behavior.height_cruise_mm)
    fse = FreeSpaceEstimator(cfg.freespace, (W, H), geo)
    lmap = LocalMap(geo, fse.W, fse.H, max_age_s=cfg.map.max_age_s, confirm=cfg.map.confirm,
                    confirm_radius_m=cfg.map.confirm_radius_m)
    blockers = []
    v_chk = cfg.robot.max_v * cfg.robot.vx_limit
    need = 0.6 * v_chk + 0.08
    free_raw = fs_in_map = None
    last_ctrl = last_yaw = None
    steps, blocked, false_blk, timeline = 0, 0, 0, []
    vw = None
    for k, m in enumerate(meta):
        img = cv2.imread(os.path.join(d, m["file"]))
        if img is None:
            continue
        if m["idx"] % 2 == 0 or free_raw is None:
            # live schließt alle Tracks aus (auch unbestätigte); ältere Aufnahmen kennen nur bestätigte
            excl = [tr["box"] for tr in m.get("tracks_all", m["tracks"])] if exclude else ()
            free_raw = fse.update(img, m["pitch_dev"], excl)
        t = m["t"]
        if last_ctrl is not None and t - last_ctrl < 1.0 / cfg.control_hz:
            continue
        dt = t - last_ctrl if last_ctrl is not None else 0.0
        last_ctrl = t
        tracks = m["tracks"]
        free = FreeSpace(list(free_raw.sectors), free_raw.columns.copy(), free_raw.horizon_row, free_raw.mask,
                         free_raw.valid, free_raw.known_from)
        if objects:
            fse.block_boxes(free, [tr["box"] for tr in tracks if tr["id"] != m["target"]])
        yaw = math.radians(m["yaw_vis_deg"] or 0.0)
        lmap.move(m["sent"][0] * cfg.robot.vx_limit * dt, (yaw - last_yaw) if last_yaw is not None else 0.0)
        last_yaw = yaw
        if free_raw is not fs_in_map:
            fs_in_map = free_raw
            lmap.add_freespace(mask_movable(free, [(tr["cls"], tr["box"]) for tr in tracks], W, H), t,
                               m["pitch_dev"])
        wr = math.radians(m["sent"][1] * cfg.robot.vyaw_limit_dps)
        clr = lmap.arc_clearance(v_chk, wr, horizon_s=1.2)
        blk = clr < need
        if blk:                                      # nächster bestätigter Punkt im Nahbereich
            cp = lmap.confirmed_points(v_chk * 1.2 + lmap.radius)
            if len(cp):
                blockers.append(cp[np.argmin(np.hypot(cp[:, 0], cp[:, 1]))])
        mm = DIST_RE.search(m["status"] or "")
        pdist = float(mm.group(1)) if mm else None
        steps += 1
        blocked += blk
        false_blk += blk and pdist is not None and pdist > person_min
        timeline.append((t, blk, pdist))
        if video:
            vis = img.copy()
            sc = W / len(free.columns)
            span = fse.H - free.horizon_row
            for a, b in column_runs(free.columns):
                pts = [(int((i + .5) * sc), int((fse.H - free.columns[i] * span) * fse.scale)) for i in range(a, b)]
                cv2.polylines(vis, [np.array(pts, np.int32)], False, (0, 255, 255), 2)
            for x, y in lmap.pts:                      # Kartenpunkte von oben (unten rechts)
                px, py = int(W - 90 - y * 120), int(H - 10 - x * 120)
                if 0 <= px < W and 0 <= py < H:
                    cv2.circle(vis, (px, py), 1, (0, 0, 255), -1)
            cv2.rectangle(vis, (W - 90 - 22, H - 10 - 22), (W - 90 + 22, H - 10), (0, 255, 0), 1)
            cv2.putText(vis, "%s  Bahn %s  Person %s" % ("BLOCK" if blk else "frei", "inf" if clr == math.inf else
                        "%.2f" % clr, "-" if pdist is None else "%.2f m" % pdist), (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if blk else (0, 255, 0), 2)
            if vw is None:
                vw = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), cfg.control_hz, (W, H))
            vw.write(vis)
    if vw is not None:
        vw.release()
    with_person = sum(1 for _, _, p in timeline if p is not None and p > person_min)
    res = {"Regelschritte": steps, "blockiert": blocked,
           "Person > %.1f m" % person_min: with_person,
           "davon blockiert (Fehlalarm)": false_blk,
           "Fehlalarm-Quote": round(false_blk / with_person, 3) if with_person else None}
    if blockers:
        b = np.array(blockers)
        res["Blockierer x [m] Median/5-95%"] = [round(float(v), 2) for v in (np.median(b[:, 0]), *np.percentile(b[:, 0], [5, 95]))]
        res["Blockierer y [m] Median/5-95%"] = [round(float(v), 2) for v in (np.median(b[:, 1]), *np.percentile(b[:, 1], [5, 95]))]
    if not quiet:
        print(json.dumps(res, ensure_ascii=False))
    return res


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir")
    ap.add_argument("--config")
    ap.add_argument("--set", action="append", default=[], metavar="ABSCHNITT.SCHLÜSSEL=WERT")
    ap.add_argument("--no-objects", action="store_true", help="erkannte Objekte nicht als Hindernis eintragen")
    ap.add_argument("--no-exclude", action="store_true", help="Bodenmodell auch aus Objektboxen lernen (altes Verhalten)")
    ap.add_argument("--person-min", type=float, default=1.2)
    ap.add_argument("--video")
    a = ap.parse_args()
    cfg = Config.load(a.config)
    for e in a.set:
        set_param(cfg, e)
    replay(a.dir, cfg, objects=not a.no_objects, person_min=a.person_min, video=a.video, exclude=not a.no_exclude)


if __name__ == "__main__":
    main()
