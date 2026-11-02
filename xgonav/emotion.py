"""Emotionen automatisch aufs Display: Person erkannt, keine Person, Akku niedrig.

Abgespielt wird mit tools/emotion.sh (vorkonvertierte RGB565-Bilder aus
EmotionCfg.dir). Der Player läuft als eigener Prozess; ein neuerer Player
(auch aus der Weboberfläche) beendet einen älteren über $FIFO.owner.
Hochfahren (cute) spielt rc.local, Herunterfahren (sleep) web/api.php.
"""
from __future__ import annotations

import logging
import os
import subprocess
import time
from pathlib import Path
from typing import Callable

from .config import EmotionCfg

log = logging.getLogger(__name__)

PLAYER = Path(__file__).resolve().parent.parent / "tools" / "emotion.sh"


def active(fifo: str, max_age_s: float = 10.0) -> bool:
    """Spielt gerade irgendein Player (auch aus dem Web)? Eine verwaiste Besitzer-Datei
    (Player per SIGKILL beendet) zählt nach max_age_s nicht mehr."""
    try:
        st = os.stat(fifo + ".owner")
    except OSError:
        return False
    return st.st_size > 0 and time.time() - st.st_mtime < max_age_s


class EmotionTrigger:
    """Entscheidet, wann welche Emotion kommt (ohne Prozesse, testbar).

    - surprised: Person erscheint, nachdem gap_s lang keine zu sehen war
    - doubt:     Modus follow und seit lost_s keine Person (einmal je Verlust)
    - embarrassed: anderer Modus und seit lost_s keine Person (einmal je Verlust)
    - shy:       Akku <= battery_low %, danach alle battery_repeat_s erneut
    Läuft schon eine Emotion, wird gewartet (Bedingung bleibt gemerkt) statt unterbrochen.
    """

    def __init__(self, cfg: EmotionCfg, play: Callable[[str], None], busy: Callable[[], bool]):
        self.cfg, self._play, self._busy = cfg, play, busy
        self._last_person: float | None = None
        self._surprise_armed = True
        self._doubt_armed = True
        self._next_shy = 0.0
        self._mode = None

    def update(self, now: float, mode: str, person: bool | None, battery: int | None) -> str | None:
        """person=None: unbekannt (Bildverarbeitung gestoppt) – dann nur Akku."""
        c = self.cfg
        if mode != self._mode:                       # neuer Modus: Verlust neu bewerten
            self._mode = mode
            self._doubt_armed = True
            if not person:
                self._last_person = now              # lost_s ab Moduswechsel zählen
        want = None
        if person is not None:
            if person:
                if self._last_person is None or now - self._last_person >= c.gap_s:
                    self._surprise_armed = True
                self._last_person = now
                self._doubt_armed = True
                if self._surprise_armed:
                    want = "surprised"
            else:
                gone = now - self._last_person if self._last_person is not None else float("inf")
                if gone >= c.gap_s:
                    self._surprise_armed = True
                if self._doubt_armed and gone >= c.lost_s:
                    want = "doubt" if mode == "follow" else "embarrassed"
        if battery is not None and 0 < battery <= c.battery_low and now >= self._next_shy:
            want = "shy"                             # Akku hat Vorrang
        if want is None or self._busy():
            return None
        if want == "surprised":
            self._surprise_armed = False
        elif want in ("doubt", "embarrassed"):
            self._doubt_armed = False
        else:
            self._next_shy = now + c.battery_repeat_s
        self._play(want)
        return want


class Emotions:
    """Startet den Player; aus, wenn EmotionCfg.dir fehlt (z. B. am PC)."""

    def __init__(self, cfg: EmotionCfg, fifo: str):
        self.cfg, self.fifo = cfg, fifo
        self.enabled = bool(cfg.enabled and fifo and os.path.isdir(cfg.dir) and PLAYER.exists())
        self._proc: subprocess.Popen | None = None
        self.trigger = EmotionTrigger(cfg, self.play, lambda: active(self.fifo))
        if cfg.enabled and not self.enabled:
            log.info("Emotionen aus (%s oder %s fehlt)", cfg.dir, PLAYER)

    def play(self, name: str) -> None:
        d = os.path.join(self.cfg.dir, name)
        if not os.path.isdir(d):
            log.warning("Emotion %s fehlt in %s", name, self.cfg.dir)
            return
        if self._proc is not None:
            self._proc.poll()                        # Zombie des Vorgängers abräumen
        log.info("Emotion %s", name)
        env = dict(os.environ, LCD_FIFO=self.fifo)
        self._proc = subprocess.Popen([str(PLAYER), "play", d], env=env, stdin=subprocess.DEVNULL,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def update(self, now: float, mode: str, person: bool | None, battery: int | None) -> None:
        if self.enabled:
            self.trigger.update(now, mode, person, battery)
