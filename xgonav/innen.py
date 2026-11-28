"""Innenleben des Modus pet (Nutzer 2026-11-28, PET-IDEEN.md Blöcke C und H).

- Akku: der ganzzahlige Prozentwert, geglättet (Median), in Stufen mit Hysterese; Laden erkennen (Wert steigt in
  Ruhe). Beim Fahren sackt der Wert ab, ungeglättet würde die Stimmung flackern.
- Innenleben: Triebe Energie (Akku × Müdigkeit aus Aktivität × Tageszeit), Geselligkeit (Sehnsucht), Erregung
  (Schreck + Freude) und Stimmung. Sie verschieben nur Zeiten, Tempo, Gesten, Augen und Höhe von pet; bei vollem
  Akku, am Tag und ausgeruht ist jeder Faktor genau 1 (Verhalten wie ohne Innenleben).
- Ladewaechter: Ruhemodus, in den pet wegen des Akkus ging: Laden erkannt -> gurren, geladen -> wecken (App).

Reine Logik ohne E/A. Kosten am Pi: ein paar Floats je Regelschritt, ein Median über ≤ 60 Werte je Sekunde.
"""
from __future__ import annotations

import collections
import math
import statistics

from .config import PetCfg

STUFEN = ("munter", "gesetzt", "hungrig", "erschoepft", "leer")
AKKU_FAKTOR = {"munter": 1.0, "gesetzt": 0.85, "hungrig": 0.6, "erschoepft": 0.35, "leer": 0.15}
VERLAUF_S = 10.0                     # Laden: so oft den geglätteten Wert merken
LADEN_RUHIG_S = 300.0                # Laden: nach Fahrt oder Lastwechsel erst so lange warten (Spannung erholt sich)
AUGEN_RATE = 15.0                    # Lidschläge je Minute wach (pet.AUGEN_RATE)


def _clamp(x: float, a: float, b: float) -> float:
    return a if x < a else b if x > b else x


class Akku:
    def __init__(self, cfg: PetCfg):
        self.cfg = cfg
        self._proben: collections.deque = collections.deque()     # (t, %) eine je Sekunde
        self._t_probe = -1e18
        self._verlauf: collections.deque = collections.deque()    # (t, geglättet) für das Laden
        self._t_verlauf = -1e18
        self._ruhig_seit = None
        self.wert: float | None = None       # geglättet, None = unbekannt
        self.stufe = "munter"
        self.laedt = False

    def grenzen(self) -> tuple:
        c = self.cfg
        return (c.akku_gesetzt, c.akku_hungrig, c.akku_erschoepft, c.akku_leer)

    def _stufe(self, v: float) -> str:
        g = self.grenzen()
        ziel = sum(1 for x in g if v < x)
        i = STUFEN.index(self.stufe)
        if ziel > i:
            i = ziel                                         # hinunter sofort
        while i > ziel and v >= g[i - 1] + self.cfg.akku_hyst:
            i -= 1                                           # hinauf erst mit Abstand zur Grenze
        return STUFEN[i]

    def update(self, t: float, prozent: int | None, faehrt: bool = False) -> None:
        """faehrt: Fahrt oder Lastwechsel (z. B. außerhalb des Ruhemodus); dann erkennt er kein Laden."""
        if faehrt or self._ruhig_seit is None:
            self._ruhig_seit = t
            self._verlauf.clear()
            if faehrt:
                self.laedt = False
        if prozent is None or prozent <= 0:
            return                                           # 0 = unbekannt (leere /proc-Datei)
        if t - self._t_probe >= 1.0:                          # eine Probe je Sekunde; nur dann neu rechnen
            self._t_probe = t
            self._proben.append((t, float(prozent)))
            while self._proben and self._proben[0][0] < t - self.cfg.akku_median_s:
                self._proben.popleft()
            neu = self.wert is None
            self.wert = statistics.median(v for _, v in self._proben)
            self.stufe = STUFEN[sum(1 for x in self.grenzen() if self.wert < x)] if neu else self._stufe(self.wert)
        if t - self._ruhig_seit >= LADEN_RUHIG_S and t - self._t_verlauf >= VERLAUF_S:
            self._t_verlauf = t
            self._verlauf.append((t, self.wert))
            while self._verlauf and self._verlauf[0][0] < t - self.cfg.laden_fenster_s:
                self._verlauf.popleft()
        if self._verlauf:
            self.laedt = self.wert - min(v for _, v in self._verlauf) >= self.cfg.laden_punkte

    def info(self) -> dict:
        return {"wert": None if self.wert is None else round(self.wert, 1), "stufe": self.stufe, "laedt": self.laedt}


def nachts(stunde: float, von: float, bis: float) -> bool:
    return stunde >= von or stunde < bis if von > bis else von <= stunde < bis


class Innenleben:
    """Triebe und Stimmung; die Faktoren sind bei Energie 1 und Standard-Persönlichkeit genau 1."""

    def __init__(self, cfg: PetCfg):
        self.cfg = cfg
        self.energie = 1.0
        self.muede = 0.0
        self._aktiv_m = 0.0                  # gefahrene Meter + 0,1 m/rad Drehung, Zerfall 30 min
        self.gesellig = 0.3                  # Sehnsucht nach Gesellschaft 0..1
        self.freude = 0.0
        self.erregung = 0.0
        self.stimmung = 0.7
        self._dudeln = True

    def update(self, dt: float, stufe: str, ds: float, dyaw: float, person_da: bool, person_nah: bool,
               fright: float, trust: float, stunde: float | None) -> None:
        c = self.cfg
        self._aktiv_m = self._aktiv_m * math.exp(-dt / 1800.0) + abs(ds) + 0.1 * abs(dyaw)
        spanne = max(c.muede_m - c.muede_ab_m, 1e-3)
        self.muede = 0.4 * _clamp((self._aktiv_m - c.muede_ab_m) / spanne, 0.0, 1.0)
        nacht = c.nacht_energie if stunde is not None and nachts(stunde, c.nacht_von, c.nacht_bis) else 1.0
        self.energie = AKKU_FAKTOR.get(stufe, 1.0) * (1.0 - self.muede) * nacht
        if person_nah:
            self.gesellig -= dt / 20.0
        elif not person_da:
            self.gesellig += dt / max(c.gesellig_s, 1e-3) * (0.5 + c.anhaenglich)
        self.gesellig = _clamp(self.gesellig, 0.0, 1.0)
        self.freude *= math.exp(-dt / 10.0)
        self.erregung = _clamp(fright + self.freude, 0.0, 1.0)
        valenz = _clamp(0.45 * self.energie + 0.35 * (1.0 - self.gesellig) + 0.2 * trust - 0.5 * fright, 0.0, 1.0)
        self.stimmung += (valenz - self.stimmung) * (1.0 - math.exp(-dt / 60.0))

    def wiedersehen(self) -> None:
        self.gesellig = 0.3
        self.freude = 1.0

    # ---- Faktoren (1 - k * (1 - E): bei E = 1 exakt 1) -------------------------------------------------------
    def f_zeit(self) -> float:
        """dösen, absetzen: müde früher"""
        return 1.0 - 0.7 * (1.0 - self.energie)

    def f_streifen(self) -> float:
        """streifen: müde später, verspielt früher"""
        return 1.0 / max(0.1, self.energie * (0.5 + self.cfg.verspielt))

    def f_tempo(self) -> float:
        """Fahrtempo und Gesten"""
        return 1.0 - 0.4 * (1.0 - self.energie)

    def lid_muede(self) -> float:
        return min(0.35, max(0.0, 0.8 - self.energie) * 0.5)

    def rate(self) -> float:
        return AUGEN_RATE * self.f_tempo() + 8.0 * self.erregung

    def hoehe_minus(self) -> float:
        return self.cfg.hoehe_muede_mm * (1.0 - self.energie)

    @property
    def dudeln_ok(self) -> bool:
        """Dudeln nur mit genug Energie; Hysterese 0,55/0,6 (ein Wechsel setzt den Dudler zurück)"""
        if self.energie >= 0.6:
            self._dudeln = True
        elif self.energie < 0.55:
            self._dudeln = False
        return self._dudeln

    def info(self) -> dict:
        return {"energie": round(self.energie, 2), "muede": round(self.muede, 2), "gesellig": round(self.gesellig, 2),
                "erregung": round(self.erregung, 2), "stimmung": round(self.stimmung, 2)}


class Ladewaechter:
    """Ruhemodus, in den pet wegen des Akkus ging (App._pet_ruhe): Laden erkannt -> "gurren" (einmal), geladen
    (≥ laden_wach) -> "wecken". Ein Ruhemodus, den der Nutzer einschaltet, ist nicht gemeint (vergessen())."""

    def __init__(self, cfg: PetCfg):
        self.cfg = cfg
        self.akku = Akku(cfg)
        self.schlaf = False
        self._gemeldet = False

    def einschlafen(self) -> None:
        self.schlaf, self._gemeldet = True, False

    def vergessen(self) -> None:
        self.schlaf = False

    def update(self, t: float, prozent: int | None, ruhe: bool) -> list[str]:
        self.akku.update(t, prozent, faehrt=not ruhe)
        if not (self.schlaf and ruhe):
            return []
        out = []
        if self.akku.laedt and not self._gemeldet:
            self._gemeldet = True
            out.append("gurren")
        if self.cfg.laden_wach > 0 and self.akku.wert is not None and self.akku.wert >= self.cfg.laden_wach:
            self.schlaf = False
            out.append("wecken")
        return out
