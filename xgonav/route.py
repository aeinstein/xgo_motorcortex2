"""Route und Sperrzonen auf der Raumkarte: prüfen, speichern, Bahn gegen Sperrzonen.

Die Datei <grid.file>.route.json gehört zu genau einer Karte (map_id). Eine neu kartierte Karte hat
ein anderes Koordinatensystem, Route und Zonen wären dort falsch: Sie werden dann nicht geladen.
"""
from __future__ import annotations

import json
import logging
import math
import os

import cv2
import numpy as np

from .config import HomeCfg
from .gridmap import GridMap
from .planner import cost_map, driven_mask

log = logging.getLogger(__name__)


def route_path(map_file: str) -> str:
    return map_file + ".route.json"


def parse_points(text: str) -> list[tuple[float, float]]:
    """"x,y;x,y" in Punkte (Meter im Karten-KS). ValueError bei ungültigen Werten."""
    pts = []
    for part in text.split(";"):
        if not part.strip():
            continue
        a, b = part.split(",")
        x, y = float(a), float(b)
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("Punkt nicht endlich")
        pts.append((x, y))
    return pts


def parse_zones(text: str) -> list[list[tuple[float, float]]]:
    """Zonen getrennt durch "|", je Zone mindestens drei Eckpunkte."""
    zones = [parse_points(z) for z in text.split("|") if z.strip()]
    if any(len(z) < 3 for z in zones):
        raise ValueError("Sperrzone braucht mindestens 3 Ecken")
    return zones


def zone_dist(zone, x: float, y: float) -> float:
    """Abstand zum Rand der Zone (m); negativ = drinnen."""
    poly = np.array(zone, np.float32).reshape(-1, 1, 2)
    return -float(cv2.pointPolygonTest(poly, (float(x), float(y)), True))


def arc_hits_zone(pose: tuple, v_mps: float, w_rads: float, zones, margin_m: float,
                  horizon_s: float = 1.2, steps: int = 10) -> bool:
    """Führt der Bogen näher als margin_m an eine Sperrzone heran? Wer schon nah dran ist, darf sich entfernen."""
    if not zones or abs(v_mps) < 1e-3:
        return False
    x, y, th = pose
    now = [zone_dist(z, x, y) for z in zones]
    dt = horizon_s / steps
    for _ in range(steps):
        th += w_rads * dt
        x += v_mps * math.cos(th) * dt
        y += v_mps * math.sin(th) * dt
        for z, d0 in zip(zones, now):
            d = zone_dist(z, x, y)
            if d < margin_m and d < d0 - 1e-3:
                return True
    return False


def check(grid: GridMap, cfg: HomeCfg, pts, zones) -> list[str]:
    """Fehlermeldungen für eine Route; leer = in Ordnung. Jeder Punkt muss auf bekanntem Boden liegen und vom
    Startpunkt aus erreichbar sein, mit Sicherheitsabstand und außerhalb der Sperrzonen."""
    errors: list[str] = []
    if not pts:
        return errors
    n = grid.n
    with grid._lock:
        lo = grid.lo.copy()
        sx, sy, _ = grid.start
    blocked, _cost = cost_map(lo, grid.res, grid.cfg.l_show, cfg, None, driven_mask(grid, 0, 0, lo.shape))
    forbid = grid.zones_mask(zones, 0, 0, lo.shape, cfg.inflate_m)
    free = lo < -grid.cfg.l_show
    sr, sc = (int(k) for k in grid.cell(sx, sy))
    disk = np.zeros(lo.shape, np.uint8)
    cv2.circle(disk, (sc, sr), int(math.ceil(cfg.inflate_m / grid.res)), 1, -1)
    start_area = disk.astype(bool) & ~forbid
    passable = ((free & ~blocked) | start_area) & ~forbid
    _, labels = cv2.connectedComponents(passable.astype(np.uint8), connectivity=8)
    reach = labels == labels[sr, sc] if passable[sr, sc] else np.zeros(lo.shape, bool)
    for i, (x, y) in enumerate(pts, 1):
        r, c = (int(k) for k in grid.cell(x, y))
        if not (0 <= r < n and 0 <= c < n):
            errors.append(f"Punkt {i} liegt außerhalb der Karte")
        elif forbid[r, c]:
            errors.append(f"Punkt {i} liegt in oder zu nah an einer Sperrzone")
        elif not free[r, c]:
            errors.append(f"Punkt {i} liegt nicht auf bekanntem Boden")
        elif blocked[r, c]:
            errors.append(f"Punkt {i} liegt zu nah an einem Hindernis")
        elif not reach[r, c]:
            errors.append(f"Punkt {i} ist vom Start aus nicht erreichbar")
    return errors


def save(map_file: str, grid: GridMap) -> bool:
    """Route und Zonen atomar speichern, mit der Kennung der Karte."""
    if not map_file:
        return False
    with grid._lock:
        data = {"map_id": grid.map_id, "points": [list(p) for p in grid.route], "loop": grid.route_loop,
                "zones": [[list(p) for p in z] for z in grid.zones]}
    path = route_path(map_file)
    tmp = path + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
        os.replace(tmp, path)
    except OSError as e:
        log.warning("Route nicht gespeichert (%s): %s", path, e)
        return False
    return True


def load(map_file: str, grid: GridMap) -> bool:
    """Route und Zonen laden, wenn sie zu dieser Karte gehören."""
    path = route_path(map_file) if map_file else ""
    if not path or not os.path.exists(path):
        return False
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        if d.get("map_id") != grid.map_id:
            log.warning("Route %s gehört zu einer anderen Karte – ignoriert", path)
            return False
        pts = [(float(x), float(y)) for x, y in d["points"]]
        zones = [[(float(x), float(y)) for x, y in z] for z in d["zones"]]
        loop = bool(d["loop"])
    except (OSError, ValueError, KeyError, TypeError) as e:
        log.warning("Route %s nicht geladen: %s", path, e)
        return False
    with grid._lock:
        grid.route, grid.route_loop, grid.zones = pts, loop, zones
    log.info("Route geladen: %d Punkte, %d Sperrzonen", len(pts), len(zones))
    return True
