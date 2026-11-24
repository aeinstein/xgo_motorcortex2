"""Verstärkung der Kamera umschalten (Nutzer 2026-11-24): Bild zu hell -> Automatik, zu dunkel -> fest (16).

Die feste Verstärkung 16 bei 20 ms Belichtung brennt bei hellem Licht aus; die Automatik von libcamera regelt nur
bis 8 und ist im Dunkeln zu dunkel. Gemessen wird im 4-Hz-Block von App._step an jedem 8. Pixel.

  fest --(Anteil ausgefressen > gain_bright_frac ODER Mittel > gain_bright_mean, gain_dwell_s lang)--> Automatik
  Automatik --(gemeldete Verstärkung am Anschlag UND Mittel < gain_dark_mean, gain_dwell_s lang)--> fest

Nach jedem Wechsel gain_hold_s keine weitere. Steht im Web ein Handwert (weder 0 noch der feste Wert), ruht das
Umschalten. Gegen Flattern: Mittel 120 bei 16 ergibt mit 8 etwa 85 (Gamma), das liegt über gain_dark_mean 70.
"""
from __future__ import annotations

import numpy as np


def bildwerte(bgr, sat: int = 250) -> tuple[float, float]:
    """(mittlere Helligkeit 0..255, Anteil ausgefressener Pixel) aus jedem 8. Pixel (am CM4 < 0,1 ms)."""
    s = bgr[::8, ::8]
    return float(s.mean()), float((s.max(axis=2) >= sat).mean())


class GainSwitch:
    def __init__(self, cfg):
        self.cfg = cfg
        self._seit: float | None = None      # Bedingung für einen Wechsel steht seit
        self._gesperrt_bis = -1e9

    def update(self, now: float, mittel: float, anteil: float, gain_ist: float | None,
               gain_soll: float, gain_fest: float) -> float | None:
        """Neuer Sollwert (0 = Automatik, sonst gain_fest) oder None (bleibt)."""
        c = self.cfg
        if gain_soll == 0:
            wechsel = (gain_ist is not None and gain_ist >= c.gain_auto_max - 0.5
                       and mittel < c.gain_dark_mean)
            ziel = gain_fest
        elif abs(gain_soll - gain_fest) < 1e-6:
            wechsel = anteil > c.gain_bright_frac or mittel > c.gain_bright_mean
            ziel = 0.0
        else:                                # Handwert aus dem Web: ruhen
            self._seit = None
            return None
        if not wechsel or now < self._gesperrt_bis:
            self._seit = None
            return None
        if self._seit is None:
            self._seit = now
        if now - self._seit < c.gain_dwell_s:
            return None
        self._seit = None
        self._gesperrt_bis = now + c.gain_hold_s
        return ziel
