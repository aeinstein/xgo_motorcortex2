"""Kleine Rechenbausteine ohne schwere Importe, für alle Module und tools/ (seit 2026-11-23 an einer Stelle statt
mehrfach nachgebaut): Winkel, Drehraten, starre Posen, Bogenschritte. Gleiche Rechenreihenfolge wie die früheren
Stellen, daher bitgleich (navfast-Vergleiche bleiben gültig). Alles geht mit float und mit numpy-Feldern."""
from __future__ import annotations

import math


def wrap(a):
    """Winkel (rad) nach [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap32(d: int) -> int:
    """Differenz zweier 32-Bit-Zähler (Radencoder) mit Überlauf."""
    return (d + 2 ** 31) % 2 ** 32 - 2 ** 31


def yaw_rate_to_w(rate_rad_s: float, rcfg) -> float:
    """Drehrate (rad/s) -> normierter Drehwert der Firmware."""
    return math.degrees(rate_rad_s) / rcfg.vyaw_limit_dps


def w_to_yaw_rate(w: float, rcfg) -> float:
    """Normierter Drehwert -> Drehrate (rad/s); Kehrwert von yaw_rate_to_w."""
    return math.radians(w * rcfg.vyaw_limit_dps)


def pose_compose(k, pose) -> tuple:
    """k ∘ pose: Pose (x, y, th) nach der starren Korrektur k = (kx, ky, kth)."""
    c, s = math.cos(k[2]), math.sin(k[2])
    return (k[0] + c * pose[0] - s * pose[1], k[1] + s * pose[0] + c * pose[1], pose[2] + k[2])


def to_world(pose, px, py) -> tuple:
    """Punkt(e) im Roboter-KS der Pose (x, y, th) -> Welt; px, py float oder Felder."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * px - s * py, pose[1] + s * px + c * py


def polar(x: float, y: float, d: float, a: float) -> tuple[float, float]:
    """Punkt im Abstand d in Richtung a (rad, Welt) von (x, y)."""
    return x + d * math.cos(a), y + d * math.sin(a)


def arc_step(x: float, y: float, th: float, v: float, w: float, dt: float) -> tuple[float, float, float]:
    """Ein Schritt entlang eines Bogens (v m/s, w rad/s): erst drehen, dann fahren (Bahnprüfung, Sperrzonen)."""
    th += w * dt
    return x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th
