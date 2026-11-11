#!/usr/bin/env python3
"""Raumkarte messen: Beobachtungen im Simulator aufzeichnen und offline durch GridMap spielen.

  python tools/map_eval.py record AUFNAHME.pkl --secs 90 [--scene zwei] [--set grid.l_occ=0.6]
  python tools/map_eval.py eval AUFNAHME.pkl [...] [--set grid.max_range_m=1.0,1.5,2.5] [--set ...]
      [--pitch cap|ins] [--pose cap|ins] [--rays]
  python tools/map_eval.py reloc AUFNAHME.pkl [...] [--schlupf 0.03] [--kursdrift 0.3] [--set reloc.gain=1.0]

record fährt einen Simulator-Lauf (Echtzeit, Rider-Einstellungen). eval dauert je Aufnahme nur
Sekunden: für jede Kombination der --set-Werte eine Zeile mit den Kennzahlen aus sim.map_quality
gegen die Wahrheit, gemittelt über die Aufnahmen. --pitch/--pose: Nick und Pose der Aufnahme (cap,
wie live) oder des späteren Bildes, in dem das Ergebnis ankam (ins, Fehler bis 2026-11-05).
--rays: Fehler der gemessenen Bodengrenze je Entfernung gegen die Wahrheit.
reloc: Kartenabgleich (xgonav/reloc.py) gegen die Wahrheit. Laufend: dieselbe Fahrt mit verdrifteter
Odometrie ohne und mit Abgleich. Global: je ein Stück herausnehmen und in der Karte des Rests suchen."""
from __future__ import annotations

import argparse
import dataclasses
import itertools
import math
import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests"))
from sim_run import RIDER, RIDER_VX, apply_overrides, run  # noqa: E402
from xgonav import reloc  # noqa: E402
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
        bear = g.geo.bearing((idx + 0.5) * sc)
        px, py, pth = o["pose_cap"]
        wx, wy = x0 + px * c0 - py * s0, y0 + px * s0 + py * c0
        off = g.geo.cam_fwd                       # Pose = Radachse, gemessen wird ab Kamera
        wx, wy = wx + off * math.cos(pth + th0), wy + off * math.sin(pth + th0)
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


# ------------------------------------------------------------------ Kartenabgleich (xgonav/reloc.py)
def _wahr_karte(data: dict) -> list[tuple]:
    """Wahre Pose je Beobachtung zur Zeit der Aufnahme, im Karten-KS. Aufgezeichnet ist die Wahrheit
    beim Eintragen (1 bis 3 Bilder später); der Weg dazwischen stammt aus der Kartenpose."""
    x0, y0, th0 = data["start"]
    c0, s0 = math.cos(th0), math.sin(th0)
    out = []
    for o in data["obs"]:
        dx, dy = o["truth"][0] - x0, o["truth"][1] - y0
        ins = (c0 * dx + s0 * dy, -s0 * dx + c0 * dy, o["truth"][2] - th0)
        pi, pc = o["pose_ins"], o["pose_cap"]
        c, s = math.cos(pi[2]), math.sin(pi[2])
        rel = (c * (pc[0] - pi[0]) + s * (pc[1] - pi[1]), -s * (pc[0] - pi[0]) + c * (pc[1] - pi[1]), pc[2] - pi[2])
        out.append(reloc.anwenden(ins, rel))
    return out


def _zeiten(data: dict) -> np.ndarray:
    obs = data["obs"]
    return np.array([o["t"] for o in obs]) - obs[0]["t"] if "t" in obs[0] else np.arange(len(obs)) / 12.0


def drift_bahn(wahr: list, t: np.ndarray, schlupf: float, kursdrift_dps: float, dreh: float) -> list[tuple]:
    """Odometrie mit Fehlern aus der wahren Bahn: Strecke x (1 + schlupf), Drehung x (1 + dreh),
    dazu stetige Kursdrift. Beginnt an der wahren Pose."""
    out = [wahr[0]]
    for a, b, dt in zip(wahr, wahr[1:], np.diff(t)):
        dth = (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi
        mid = a[2] + dth / 2
        ds = (b[0] - a[0]) * math.cos(mid) + (b[1] - a[1]) * math.sin(mid)
        ds, dth = ds * (1 + schlupf), dth * (1 + dreh) + math.radians(kursdrift_dps) * dt
        x, y, th = out[-1]
        out.append((x + ds * math.cos(th + dth / 2), y + ds * math.sin(th + dth / 2), th + dth))
    return out


def _fehler(a: tuple, b: tuple) -> tuple[float, float]:
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(math.degrees((a[2] - b[2] + math.pi) % (2 * math.pi) - math.pi))


def karte_aus(data: dict, cfg: Config) -> GridMap:
    """Fertige Karte einer Aufnahme (Posen der Aufnahme), wie nach explore gespeichert."""
    g = make_grid(cfg, data["obs"][0]["body_h"])
    for o in data["obs"]:
        if o["balancing"]:
            g._clear_disk(o["pose_cap"][0], o["pose_cap"][1], g.cfg.driven_free_r_m)
            g.add_freespace(FreeSpace([], o["columns"], o["horizon_row"], None, True, o["known_from"]),
                            o["pitch_cap"], o["pose_cap"])
    return g


def reloc_lokal(data: dict, cfg: Config, a, karte: GridMap | None = None) -> dict:
    """Dieselben Beobachtungen mit verdrifteter Odometrie eintragen, einmal ohne und einmal mit
    laufendem Abgleich. Posenfehler gegen die Wahrheit und Kartenqualität. Mit karte wird nichts
    eingetragen: Abgleich gegen die fertige Karte einer anderen Fahrt (wie bei patrol und home)."""
    from map_replay import Abgleich
    obs, t, wahr = data["obs"], _zeiten(data), _wahr_karte(data)
    odo = drift_bahn(wahr, t, a.schlupf, a.kursdrift, a.dreh)
    scene = SCENES[data["scene"]]()
    out = {}
    for name in ("ohne", "mit"):
        g = make_grid(cfg, obs[0]["body_h"])
        feld = reloc.Feld(karte.lo, karte.res, cfg.grid.l_show, cfg.reloc) if karte is not None else None
        ab = Abgleich(dataclasses.replace(cfg.reloc), {}, cfg.grid, feld) if name == "mit" else None
        err = []
        for o, ti, p in zip(obs, t, odo):
            if not o["balancing"]:
                continue
            fs = FreeSpace([], o["columns"], o["horizon_row"], None, True, o["known_from"])
            ein = ab.schritt(g, float(ti), p, fs, o["pitch_cap"]) if ab else True
            pose = reloc.anwenden(ab.k, p) if ab else p
            if karte is None:
                g._clear_disk(pose[0], pose[1], g.cfg.driven_free_r_m)
            if ein and karte is None:
                g.add_freespace(fs, o["pitch_cap"], pose)
            err.append(_fehler(pose, wahr[len(err)]))
        e = np.array(err)
        out[name] = {"Fehler Mittel m": round(float(e[:, 0].mean()), 3), "Fehler Ende m": round(float(e[-1, 0]), 3),
                     "Fehler max m": round(float(e[:, 0].max()), 3), "Kurs Ende Grad": round(float(e[-1, 1]), 1)}
        if karte is None:
            q = map_quality(g.lo, g.res, g.cfg.l_show, scene, data["start"], data["path"])
            out[name].update({"praezision_15cm": round(q["praezision_15cm"], 2), "waende_10cm": round(q["waende_10cm"], 2)})
        if ab:
            out[name]["Abgleich"] = ab.bericht(odo[-1])
    return out


def reloc_global(data: dict, cfg: Config, a, karte: GridMap | None = None) -> dict:
    """Ein Stück der Fahrt herausnehmen, die Karte aus dem Rest bauen und das Stück darin suchen
    (als wäre der Rider dort abgesetzt worden). Soll: die Pose am Ende des Stücks. Mit karte wird
    in der fertigen Karte einer anderen Fahrt gesucht, Soll ist dann die wahre Pose."""
    obs, t = data["obs"], _zeiten(data)
    wahr = _wahr_karte(data)
    rc = cfg.reloc
    zahl = {"richtig": 0, "falsch": 0, "unsicher": 0}
    gruende: dict = {}
    zeilen = []
    for ende in np.arange(a.fenster, float(t[-1]), a.abstand):
        drin = (t > ende - a.fenster) & (t <= ende)
        nah = (t > ende - a.fenster - 2.0) & (t <= ende + 2.0)       # Nachbarn sehen dasselbe: auch nicht in die Karte
        if drin.sum() < 20:
            continue
        g = karte if karte is not None else make_grid(cfg, obs[0]["body_h"])
        puffer = reloc.ScanPuffer(rc, window_s=1e9, window_m=1e9)
        for o, ti, d, n in zip(obs, t, drin, nah):
            if not o["balancing"]:
                continue
            fs = FreeSpace([], o["columns"], o["horizon_row"], None, True, o["known_from"])
            if d:
                puffer.add(float(ti), o["pose_cap"], *reloc.scan_punkte(g, fs, o["pitch_cap"], rc))
            elif not n and karte is None:
                g._clear_disk(o["pose_cap"][0], o["pose_cap"][1], g.cfg.driven_free_r_m)
                g.add_freespace(fs, o["pitch_cap"], o["pose_cap"])
        letzte = int(np.flatnonzero(drin)[-1])
        soll = wahr[letzte] if karte is not None else obs[letzte]["pose_cap"]
        e = reloc.suche_global(reloc.Feld(g.lo, g.res, g.cfg.l_show, rc), puffer.scan(obs[letzte]["pose_cap"]), rc)
        dm, dg = _fehler(e.pose, soll)
        art = "unsicher" if not e.ok else ("richtig" if dm < 0.3 and dg < 10 else "falsch")
        zahl[art] += 1
        if not e.ok:
            gruende[e.grund] = gruende.get(e.grund, 0) + 1
        zeilen.append("  %5.0f s  Drehung %4.0f°  Punkte %3d  %-9s %-14s Güte %.2f  zweit %.2f  daneben %.2f m %3.0f°" % (
            ende, math.degrees(puffer.spanne()[1]), e.n, art, e.grund, e.guete, e.zweit, dm, dg))
    return {"zahl": zahl, "gruende": gruende, "zeilen": zeilen}


def reloc_cmd(a) -> None:
    sets = {k: v[0] for k, v in parse_sets(a.set).items()}
    for f in a.files:
        with open(f, "rb") as fh:
            data = pickle.load(fh)
        cfg = Config()
        apply_overrides(cfg, data["overrides"])
        apply_overrides(cfg, sets)
        cfg.grid.native = False
        print(f)
        print(" laufender Abgleich, Odometrie mit Schlupf %.0f %%, Drehung %+.0f %%, Kursdrift %.2f °/s:" % (
            100 * a.schlupf, 100 * a.dreh, a.kursdrift))
        karte = None
        if a.karte:
            with open(a.karte, "rb") as fh:
                kd = pickle.load(fh)
            assert kd["scene"] == data["scene"] and np.allclose(kd["start"], data["start"]), "andere Szene oder anderer Start"
            karte = karte_aus(kd, cfg)
            print(" gegen die fertige Karte aus %s" % a.karte)
        for name, z in reloc_lokal(data, cfg, a, karte).items():
            print("  %-4s %s" % (name, z))
        g = reloc_global(data, cfg, a, karte)
        print(" globale Suche (Stück von %.0f s herausgenommen): %s  unsicher wegen %s" % (a.fenster, g["zahl"], g["gruende"]))
        print("\n".join(g["zeilen"]), flush=True)


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
    q = sub.add_parser("reloc")
    q.add_argument("files", nargs="+")
    q.add_argument("--set", action="append")
    q.add_argument("--schlupf", type=float, default=0.03, help="Odometrie misst die Strecke um diesen Anteil zu lang")
    q.add_argument("--kursdrift", type=float, default=0.3, help="Kursdrift der Odometrie in °/s")
    q.add_argument("--dreh", type=float, default=0.02, help="Odometrie misst Drehungen um diesen Anteil zu groß")
    q.add_argument("--karte", default="", help="fertige Karte aus dieser Aufnahme (gleiche Szene, gleicher Start) statt der eigenen")
    q.add_argument("--fenster", type=float, default=15.0, help="globale Suche: Länge des herausgenommenen Stücks in s")
    q.add_argument("--abstand", type=float, default=10.0, help="globale Suche: alle so viele Sekunden ein Stück")
    a = ap.parse_args()
    {"record": record, "eval": evaluate, "reloc": reloc_cmd}[a.cmd](a)


if __name__ == "__main__":
    main()
