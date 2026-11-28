"""Gedächtnis des Modus pet (Nutzer 2026-11-28, PET-IDEEN.md Block D, Teile 1 und 4).

Je angelernter Person (Name aus [gesicht]): Vertrauen, letzte und erste Begegnung, Zahl der Begegnungen. Die App
hält ein Objekt für alle Modi und gibt es jedem neuen Pet mit; so übersteht es Moduswechsel. Die Datei
([pet] gedaechtnis) übersteht den Neustart. Zeiten in der Uhr, die das Gehirn mitgibt: am Rider Epoch-Sekunden
(time.time), ohne Datei (Simulator, Tests) auch die Simulationszeit.

Kosten am Pi: ein dict-Zugriff je Person mit Namen und Regelschritt; höchstens einmal je SPEICHER_S ein JSON von
wenigen hundert Byte (ohne fsync).
"""
from __future__ import annotations

import json
import logging
import os

from .config import PetCfg

log = logging.getLogger(__name__)

VERSION = 1
SPEICHER_S = 60.0                    # höchstens so oft schreiben


class Gedaechtnis:
    def __init__(self, pfad: str, cfg: PetCfg):
        self.pfad = pfad or ""
        self.cfg = cfg
        self.personen: dict[str, dict] = {}
        self.dirty = False
        self._t_save: float | None = None      # monotone Zeit des letzten Schreibens (App)
        self._warnung = False
        if self.pfad:
            self.laden()

    # ------------------------------------------------------------------ Datei
    def laden(self) -> None:
        try:
            with open(self.pfad, encoding="utf-8") as fh:
                d = json.load(fh)
            p = d.get("personen") if isinstance(d, dict) else None
            if not isinstance(p, dict):
                raise ValueError("kein Feld personen")
            self.personen = {}
            for name, e in p.items():
                if isinstance(name, str) and isinstance(e, dict):
                    self.personen[name] = {
                        "vertrauen": min(1.0, max(0.0, float(e.get("vertrauen", self.cfg.trust_bekannt)))),
                        "zuletzt": float(e["zuletzt"]) if e.get("zuletzt") is not None else None,
                        "erstmals": float(e["erstmals"]) if e.get("erstmals") is not None else None,
                        "begegnungen": int(e.get("begegnungen", 0))}
            log.info("Gedächtnis: %d Personen aus %s", len(self.personen), self.pfad)
        except FileNotFoundError:
            self.personen = {}
        except Exception as e:
            log.warning("Gedächtnis %s nicht lesbar (%s): starte leer", self.pfad, e)
            self.personen = {}

    def speichern(self, now: float | None = None) -> bool:
        """Schreiben (tmp + replace), wenn etwas neu ist. now: monotone Zeit für das Intervall (None = sofort)."""
        if not self.pfad or not self.dirty:
            return False
        if now is not None and self._t_save is not None and now - self._t_save < SPEICHER_S:
            return False
        self._t_save = now
        tmp = self.pfad + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump({"version": VERSION, "personen": self.personen}, fh, ensure_ascii=False, indent=1)
            os.replace(tmp, self.pfad)
        except OSError as e:
            if not self._warnung:
                log.warning("Gedächtnis %s nicht schreibbar (%s): nur im Speicher", self.pfad, e)
                self._warnung = True
            return False
        self.dirty = False
        return True

    # ------------------------------------------------------------------ Personen
    def _p(self, name: str) -> dict:
        e = self.personen.get(name)
        if e is None:
            e = self.personen[name] = {"vertrauen": min(1.0, max(0.0, self.cfg.trust_bekannt)), "zuletzt": None,
                                       "erstmals": None, "begegnungen": 0}
            self.dirty = True
        return e

    def zuletzt(self, name: str) -> float | None:
        e = self.personen.get(name)
        return None if e is None else e["zuletzt"]

    def abwesend(self, name: str, jetzt: float) -> float | None:
        """Sekunden seit der letzten Begegnung; None = noch nie gesehen; Uhr zurückgegangen: 0."""
        z = self.zuletzt(name)
        return None if z is None else max(0.0, jetzt - z)

    def gesehen(self, name: str, jetzt: float) -> None:
        e = self._p(name)
        if e["erstmals"] is None:
            e["erstmals"] = jetzt
        if e["zuletzt"] != jetzt:
            e["zuletzt"] = jetzt
            self.dirty = True

    def begegnung(self, name: str) -> int:
        e = self._p(name)
        e["begegnungen"] += 1
        self.dirty = True
        return e["begegnungen"]

    def vertrauen(self, name: str) -> float:
        e = self.personen.get(name)
        return min(1.0, max(0.0, self.cfg.trust_bekannt)) if e is None else e["vertrauen"]

    def setze_vertrauen(self, name: str, v: float) -> None:
        e = self._p(name)
        v = round(min(1.0, max(0.0, v)), 4)
        if v != e["vertrauen"]:
            e["vertrauen"] = v
            self.dirty = True

    def info(self, jetzt: float) -> dict:
        return {n: {"vertrauen": round(e["vertrauen"], 2), "begegnungen": e["begegnungen"],
                    "zuletzt_vor_s": None if e["zuletzt"] is None else round(max(0.0, jetzt - e["zuletzt"]))}
                for n, e in self.personen.items()}
