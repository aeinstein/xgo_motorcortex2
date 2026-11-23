"""Dunkelangst: Wird das Bild zu dunkel, bekommt der Rider Angst, dreht um und sucht Licht. Findet er keins,
setzt er sich und geht in den Ruhemodus.

Ablauf (nur in [dunkel] modes, nur balancierend):
  ok --(Helligkeit < dark_level für dark_s)--> angst (stehen, Emotion + Ton)
     --> drehen 180° --> prüfen (settle_s warten, Belichtung regelt nach)
         hell (> light_level): ok, der Modus fährt weiter
         dunkel: nächste Richtung (+90°, dann +180°: hinten, rechts, links), nach allen -> aufgeben
  aufgeben: App setzt ab und schaltet den Ruhemodus ein.
Bei jeder Angst meldet er der App eine Dunkelzone ("zone"): Rechteck voraus (sperr_rechteck), das Zielsuche,
Planer und Bahnprüfung zone_s lang meiden; sonst schickte der Planer ihn nach dem Umdrehen zurück ins Dunkle.
Wiederholt sich die Angst max_events-mal in events_window_s (er fährt immer wieder ins Dunkle), gibt er sofort auf.

Kosten am Pi: Helligkeit = Mittelwert jedes 8. Pixels (80x60), höchstens check_hz je Sekunde; sonst nichts.
Gedreht wird auf der Stelle; Sicherheitsschichten (safety, Bahnprüfung, Robot) bleiben wie in jedem Modus.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from .behaviors import dreh_w

log = logging.getLogger(__name__)

SCHRITTE_DEG = (180.0, 90.0, 180.0)      # relative Drehungen: nach hinten, dann rechts/links davon


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def sperr_rechteck(x: float, y: float, th: float, cfg) -> list[tuple[float, float]]:
    """Dunkelzone: Rechteck voraus (Karten-KS, m), zone_w_m breit, zone_d_m tief, ab zone_ahead_m vor (x, y)."""
    c, s = math.cos(th), math.sin(th)
    a, b, h = cfg.zone_ahead_m, cfg.zone_ahead_m + cfg.zone_d_m, cfg.zone_w_m / 2
    return [(x + c * d - s * q, y + s * d + c * q) for d, q in ((a, -h), (b, -h), (b, h), (a, h))]


def helligkeit(bgr) -> float:
    """Mittlere Helligkeit 0..255 aus jedem 8. Pixel (am CM4 vernachlässigbar)."""
    s = bgr[::8, ::8]
    return float(s.mean())


@dataclass
class DunkelCmd:
    w: float                 # normierte Drehrate (v ist immer 0)
    status: str


class Dunkelangst:
    def __init__(self, cfg, rcfg):
        self.cfg, self.rcfg = cfg, rcfg
        self.state = "ok"
        self.t_state = 0.0
        self._dark_since: float | None = None
        self._yaw_goal: float | None = None
        self._schritt = 0
        self._events: list[float] = []
        self.ausgaben: list[tuple[str, str]] = []   # (emotion|sound|ruhe, Name), App holt sie ab
        self.luma: float | None = None
        self.n_angst = self.n_licht = self.n_aufgeben = self.n_zonen = 0      # Zähler (Simulator, Status)

    @property
    def aktiv(self) -> bool:
        return self.state not in ("ok", "aufgeben")

    def reset(self) -> None:
        if self.state != "ok":
            log.info("Dunkelangst: zurückgesetzt (%s)", self.state)
        self.state, self._dark_since, self._yaw_goal, self._schritt = "ok", None, None, 0

    def drain(self) -> list[tuple[str, str]]:
        out, self.ausgaben = self.ausgaben, []
        return out

    def _set(self, state: str, now: float) -> None:
        self.state, self.t_state = state, now

    def update(self, now: float, luma: float | None, yaw: float) -> DunkelCmd | None:
        """Je Regelschritt. None = nichts zu tun (Modus regelt). Sonst Drehrate statt des Modus."""
        c = self.cfg
        if luma is not None:
            self.luma = luma
        luma = self.luma
        st = self.state
        if st == "ok":
            if luma is None or luma >= c.dark_level:
                self._dark_since = None
                return None
            if self._dark_since is None:
                self._dark_since = now
            if now - self._dark_since < c.dark_s:
                return None
            self._events = [t for t in self._events if now - t < c.events_window_s] + [now]
            log.warning("Dunkelangst: Bild zu dunkel (%.0f < %.0f), Angst #%d", luma, c.dark_level, len(self._events))
            self.ausgaben += [("emotion", c.emotion), ("sound", c.sound), ("zone", "")]
            self.n_angst += 1
            self._set("angst", now)
            self._schritt = 0
            self._yaw0 = yaw
            return DunkelCmd(0.0, "Dunkel – Angst")
        if st == "angst":
            if now - self.t_state < c.fright_s:
                return DunkelCmd(0.0, "Dunkel – Angst")
            if len(self._events) >= c.max_events:
                log.warning("Dunkelangst: %d-mal in %.0f s ins Dunkle, gibt auf", len(self._events), c.events_window_s)
                return self._aufgeben(now)
            return self._naechste_richtung(now, yaw)
        if st == "drehen":
            err = _wrap(self._yaw_goal - yaw)
            if abs(err) > math.radians(c.tol_deg) and now - self.t_state < c.turn_timeout_s:
                return DunkelCmd(dreh_w(err, c.turn_kp, c.turn_w, self.rcfg), "Dunkel – dreht um")
            self._set("pruefen", now)
            return DunkelCmd(0.0, "Dunkel – schaut")
        if st == "pruefen":
            if now - self.t_state < c.settle_s:
                return DunkelCmd(0.0, "Dunkel – schaut")
            if luma is not None and luma >= c.light_level:
                log.warning("Dunkelangst: Licht gefunden (%.0f), weiter", luma)
                self.ausgaben.append(("sound", c.sound_relief))
                self.n_licht += 1
                self._set("ok", now)
                self._dark_since = None
                return None
            if self._schritt >= len(SCHRITTE_DEG):
                log.warning("Dunkelangst: kein Licht gefunden (%.0f), setzt sich", luma if luma is not None else -1)
                return self._aufgeben(now)
            return self._naechste_richtung(now, yaw)
        return None                                  # aufgeben: App übernimmt

    def _naechste_richtung(self, now: float, yaw: float) -> DunkelCmd:
        self._yaw_goal = _wrap(yaw + math.radians(SCHRITTE_DEG[self._schritt]))
        self._schritt += 1
        self._set("drehen", now)
        return DunkelCmd(0.0, "Dunkel – dreht um")

    def _aufgeben(self, now: float) -> DunkelCmd:
        self._set("aufgeben", now)
        self.n_aufgeben += 1
        self.ausgaben.append(("ruhe", ""))
        return DunkelCmd(0.0, "Dunkel – setzt sich")
