"""Monokulare Entfernung über die Bodenebene (Fußpunkt) bzw. Objektgröße."""
from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass

# typische Objektmaße (Höhe, Breite in m) und relative Unsicherheit der Größenschätzung
NOMINAL = {"person": (1.7, 0.45, 0.20), "chair": (0.9, 0.45, 0.35), "dog": (0.5, 0.5, 0.45),
           "cat": (0.3, 0.35, 0.45), "bottle": (0.25, 0.08, 0.30), "cup": (0.1, 0.08, 0.30),
           "potted plant": (0.5, 0.35, 0.40), "couch": (0.85, 1.8, 0.45), "dining table": (0.75, 1.2, 0.45),
           "tv": (0.5, 0.9, 0.30), "backpack": (0.45, 0.3, 0.35), "sports ball": (0.22, 0.22, 0.15),
           "teddy bear": (0.3, 0.25, 0.35), "suitcase": (0.6, 0.4, 0.35), "bed": (0.6, 1.5, 0.5)}
DEFAULT_NOMINAL = (0.5, 0.4, 0.5)


@dataclass
class CamGeometry:
    W: int
    H: int
    hfov_deg: float
    tilt_deg: float = 0.0
    mount_offset_mm: float = 45.0
    body_height_mm: float = 85.0
    pitch_sigma_deg: float = 1.5   # unbeobachtbares langsames Nicken (Neigung beim Beschleunigen)
    # Kamerahöhe = a + b * u8 (mm) über den Höhen-Sollwert der Firmware; leer = Körperhöhe + mount_offset_mm.
    # body_min/max_mm: Skala Körperhöhe (mm) <-> u8 wie robot.height_u8_to_mm.
    cam_h_u8: tuple = ()
    body_min_mm: float = 60.0
    body_max_mm: float = 120.0
    # Linse vor der Radachse (mm): Karten haben ihren Ursprung an der Achse, ground_dist misst ab Kamerafußpunkt
    cam_fwd_mm: float = 0.0

    @classmethod
    def from_cfg(cls, cfg, body_height_mm: float | None = None) -> "CamGeometry":
        c = cfg.camera
        return cls(c.width, c.height, c.hfov_deg, c.tilt_deg, c.mount_offset_mm,
                   cfg.behavior.height_cruise_mm if body_height_mm is None else body_height_mm,
                   cam_h_u8=tuple(c.cam_h_u8), body_min_mm=cfg.robot.height_min_mm,
                   body_max_mm=cfg.robot.height_max_mm, cam_fwd_mm=c.cam_fwd_mm)

    @property
    def cam_fwd(self) -> float:
        """Kamerafußpunkt vor der Radachse (m)."""
        return self.cam_fwd_mm / 1000.0

    @property
    def f(self) -> float:
        return (self.W / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    @property
    def cam_h(self) -> float:
        if self.cam_h_u8:
            u = 255.0 * (self.body_height_mm - self.body_min_mm) / (self.body_max_mm - self.body_min_mm)
            return (self.cam_h_u8[0] + self.cam_h_u8[1] * u) / 1000.0
        return (self.body_height_mm + self.mount_offset_mm) / 1000.0

    def horizon_row(self, pitch_dev: float = 0.0) -> float:
        """pitch_dev: aktuelles Nicken (rad, + = schaut höher) ggü. Normallage."""
        return self.H / 2 + self.f * math.tan(math.radians(self.tilt_deg) + pitch_dev)

    def ground_dist(self, y: float, pitch_dev: float = 0.0) -> float | None:
        """Bildzeile eines Bodenpunkts -> horizontale Entfernung (m)."""
        beta = math.atan2(y - self.H / 2, self.f) - math.radians(self.tilt_deg) - pitch_dev
        if beta <= math.radians(0.5):
            return None  # am/über dem Horizont
        return self.cam_h / math.tan(beta)

    def ground_dist_np(self, y: np.ndarray, pitch_dev: float = 0.0) -> np.ndarray:
        """Vektorisiert wie ground_dist; NaN am/über dem Horizont."""
        beta = np.arctan2(np.asarray(y, float) - self.H / 2, self.f) - math.radians(self.tilt_deg) - pitch_dev
        with np.errstate(divide="ignore", invalid="ignore"):
            d = self.cam_h / np.tan(beta)
        d[beta <= math.radians(0.5)] = np.nan
        return d

    def object_dist(self, box, cls: str, pitch_dev: float = 0.0, clip=None) -> tuple[float | None, str]:
        """Entfernung fusioniert aus Fußpunkt (nickempfindlich) und Objektgröße
        (nickunabhängig, aber Größenannahme). Gewichtung per inverser Varianz."""
        x1, y1, x2, y2 = box
        m = 3
        if clip is None:
            mx, my = 0.03 * self.W, 0.03 * self.H
            clip = (x1 <= mx, y1 <= my, x2 >= self.W - mx, y2 >= self.H - my)
        cl, ct, cr, cb = clip
        nh, nw, rel = NOMINAL.get(cls, DEFAULT_NOMINAL)
        est: list[tuple[float, float, str]] = []
        if not cb and y2 < self.H - m:
            d = self.ground_dist(y2, pitch_dev)
            if d is not None:
                sb = math.radians(self.pitch_sigma_deg)
                sd = d * d * sb / self.cam_h + 0.03
                est.append((d, sd * sd, "boden"))
        if not (cl or cr) and x1 > m and x2 < self.W - m and x2 - x1 > 4:
            d = self.f * nw / (x2 - x1)
            est.append((d, (rel * d) ** 2, "breite"))
        if not (ct or cb) and y1 > m and y2 < self.H - m and y2 - y1 > 4:
            d = self.f * nh / (y2 - y1)
            est.append((d, (0.75 * rel * d) ** 2, "hoehe"))
        if not est:
            if cb or y2 >= self.H - m:
                return 0.0, "nah"  # Fußpunkt unten abgeschnitten -> sehr nah
            return None, "?"
        wsum = sum(1 / v for _, v, _ in est)
        d = sum(dd / v for dd, v, _ in est) / wsum
        return d, "+".join(n for _, _, n in est)
