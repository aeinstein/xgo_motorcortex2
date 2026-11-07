"""Horizont im Stand: Standreferenz der IMU nach jedem Aufstehen und Horizontzeile je Körperhöhe.

Warum: Der Rider balanciert. Im Stand pendelt er um eine Lage, die von Körperhöhe und
Schwerpunkt abhängt; das Mittel des IMU-Nickwinkels über einige Sekunden ist diese Lage.
Bisher stammten tilt_deg und pitch_ref_deg aus einer Messung im angelehnten Liegen
(2026-11-04), im Stand und bei anderer Höhe lag der Horizont daneben.

Ablauf ([horizon] enabled):
  1. StandRef: nach dem Aufstehen (und nach jeder Höhenänderung) settle_s warten, dann den
     IMU-Rohnick über window_s mitteln. Gültig nur bei Stillstand und kleiner Streuung.
  2. HorizonTable: je Körperhöhe die Bildzeile des Horizonts im Stand (von Hand im Webbild
     eingestellt, siehe docs/calibration.md). Dazwischen linear interpoliert.
  3. App: pitch_dev = Vorzeichen * (IMU - Standreferenz), Neigung aus der Tabelle. Kein Tiefpass,
     keine Mischung mit dem Bild; der Bildnick läuft getrennt als Vergleich mit.
"""
from __future__ import annotations

import json
import logging
import math
import os
import time

import numpy as np

from .config import HorizonCfg

log = logging.getLogger(__name__)


class StandRef:
    """Mittlerer IMU-Rohnick (rad) im Stand, je Aufstehen und Körperhöhe neu gemessen.

    Zustände: "liegt" (balanciert nicht), "wartet" (Einschwingen), "misst", "gültig".
    """

    def __init__(self, cfg: HorizonCfg):
        self.cfg = cfg
        self.state = "liegt"
        self.ref: float | None = None
        self.std: float | None = None          # Streuung der letzten Messung (rad)
        self.n = 0                             # IMU-Proben der letzten Messung
        self.height_mm: float | None = None    # Körperhöhe, für die ref gilt
        self.t_valid: float | None = None
        self.verworfen = 0                     # Messfenster mit zu großer Streuung oder zu wenig Proben
        self._t_event: float | None = None     # Aufstehen, Höhenänderung oder Bewegung
        self._h_event: float | None = None

    def _restart(self, now: float, height_mm: float | None) -> None:
        self._t_event = now
        self._h_event = height_mm
        self.state = "wartet"

    def update(self, now: float, balancing: bool, height_mm: float | None, still: bool, imu) -> bool:
        """Je Bild aufrufen. now: CLOCK_MONOTONIC (wie die IMU-Zeitstempel).
        Rückgabe True, wenn in diesem Aufruf eine neue Referenz festgelegt wurde."""
        if not balancing:
            if self.state != "liegt":
                log.info("Standreferenz: liegt, Referenz verworfen")
            self.state, self.ref, self._t_event = "liegt", None, None
            return False
        if self._t_event is None:
            log.info("Standreferenz: steht, warte %.1f s", self.cfg.settle_s)
            self.ref = None
            self._restart(now, height_mm)
            return False
        if height_mm is not None and self._h_event is not None and abs(height_mm - self._h_event) > 0.5:
            log.info("Standreferenz: Körperhöhe %.1f -> %.1f mm, messe neu", self._h_event, height_mm)
            self.ref = None
            self._restart(now, height_mm)
            return False
        if self.ref is not None:
            return False                         # gilt bis zum Absetzen oder zur nächsten Höhenänderung
        if not still:
            self._restart(now, height_mm)        # fährt: Mittel wäre die Fahrlage, nicht der Stand
            return False
        t_a = self._t_event + self.cfg.settle_s
        t_b = t_a + self.cfg.window_s
        if now < t_a:
            self.state = "wartet"
            return False
        if now < t_b:
            self.state = "misst"
            return False
        if imu is None:
            return False
        ts, ps = imu.arrays()
        m = (ts >= t_a) & (ts <= t_b)
        n = int(m.sum())
        std = float(np.std(ps[m])) if n else float("inf")
        if n < self.cfg.min_samples or std > math.radians(self.cfg.max_std_deg):
            self.verworfen += 1
            log.info("Standreferenz verworfen: %d Proben, Streuung %.2f°", n, math.degrees(std) if n else -1)
            self._t_event = now - self.cfg.settle_s      # nächstes Fenster sofort
            self.state = "misst"
            return False
        self.ref, self.std, self.n = float(np.mean(ps[m])), std, n
        self.height_mm, self.t_valid, self.state = height_mm, now, "gültig"
        log.info("Standreferenz: IMU %+.2f° (Streuung %.2f°, %d Proben) bei %s mm", math.degrees(self.ref),
                 math.degrees(std), n, "?" if height_mm is None else "%.1f" % height_mm)
        return True


class HorizonTable:
    """Bildzeile des Horizonts im Stand je Körperhöhe (Vollbild-Pixel), als JSON gespeichert.

    Eintrag: {"height_mm", "row", "ref_deg" (Standreferenz beim Einstellen, nur Diagnose), "zeit"}.
    """

    TOL_MM = 1.0                                 # gleiche Höhe: Eintrag ersetzen

    def __init__(self, path: str = ""):
        self.path = path
        self.points: list[dict] = []
        if path and os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as f:
                    self.points = sorted(json.load(f).get("punkte", []), key=lambda p: p["height_mm"])
                log.info("Horizonttabelle %s: %d Höhen", path, len(self.points))
            except (OSError, ValueError, KeyError, TypeError) as e:
                log.warning("Horizonttabelle %s unlesbar (%s): leer", path, e)
                self.points = []

    def row_at(self, height_mm: float | None) -> float | None:
        """Interpolierte Zeile; außerhalb der Tabelle die nächste Höhe, leer oder ohne Höhe None."""
        if not self.points or height_mm is None:
            return None
        hs = [p["height_mm"] for p in self.points]
        rs = [p["row"] for p in self.points]
        return float(np.interp(height_mm, hs, rs))

    def covers(self, height_mm: float | None) -> bool:
        if not self.points or height_mm is None:
            return False
        return self.points[0]["height_mm"] - self.TOL_MM <= height_mm <= self.points[-1]["height_mm"] + self.TOL_MM

    def set(self, height_mm: float, row: float, ref_deg: float | None) -> dict:
        e = {"height_mm": round(float(height_mm), 1), "row": round(float(row), 1),
             "ref_deg": None if ref_deg is None else round(float(ref_deg), 3),
             "zeit": time.strftime("%Y-%m-%d %H:%M:%S")}
        self.points = [p for p in self.points if abs(p["height_mm"] - height_mm) > self.TOL_MM] + [e]
        self.points.sort(key=lambda p: p["height_mm"])
        self.save()
        return e

    def delete(self, height_mm: float) -> bool:
        n = len(self.points)
        self.points = [p for p in self.points if abs(p["height_mm"] - height_mm) > self.TOL_MM]
        if len(self.points) != n:
            self.save()
            return True
        return False

    def save(self) -> None:
        if not self.path:
            return
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"punkte": self.points}, f, indent=1, ensure_ascii=False)
        os.replace(tmp, self.path)


def row_to_tilt_deg(row: float, f_px: float, H: int) -> float:
    """Horizontzeile (bei pitch_dev = 0) -> Neigung wie CamGeometry.tilt_deg (Umkehrung von horizon_row)."""
    return math.degrees(math.atan((row - H / 2) / f_px))
