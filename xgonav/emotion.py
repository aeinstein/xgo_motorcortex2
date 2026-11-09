"""Emotionen automatisch aufs Display: Person erkannt, keine Person, Akku niedrig.
Dazu Geräusche über den Lautsprecher (aplay): freudig, wenn eine Person auftaucht,
traurig, wenn sie lost_s lang verschwunden ist (unabhängig vom Display).

Abgespielt wird mit tools/emotion.sh (vorkonvertierte RGB565-Bilder aus
EmotionCfg.dir). Der Player läuft als eigener Prozess; ein neuerer Player
(auch aus der Weboberfläche) beendet einen älteren über $FIFO.owner.
Hochfahren (cute) spielt rc.local, Herunterfahren (sleep) web/api.php.
"""
from __future__ import annotations

import glob
import logging
import os
import random
import shutil
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


LCD_FRAME_BYTES = 320 * 240 * 2


def load_gaze(path: str, bins: int = 7, lids: int = 5) -> dict[tuple[int, int], bytes] | None:
    """Augen-Atlas aus tools/eyes.py --gaze: {(Blick, Lid): RGB565}. None, wenn etwas fehlt."""
    atlas = {}
    for g in range(bins):
        for l in range(lids):
            try:
                with open(os.path.join(path, f"g{g}_l{l}.raw"), "rb") as f:
                    data = f.read()
            except OSError:
                return None
            if len(data) != LCD_FRAME_BYTES:
                return None
            atlas[(g, l)] = data
    return atlas


class EmotionTrigger:
    """Entscheidet, wann welche Emotion kommt (ohne Prozesse, testbar).

    - surprised: Person erscheint, nachdem gap_s lang keine zu sehen war
    - doubt:     Modus follow und seit lost_s keine Person (einmal je Verlust)
    - embarrassed: anderer Modus und seit lost_s keine Person (einmal je Verlust)
    - shy:       Akku <= battery_low %, danach alle battery_repeat_s erneut
    Läuft schon eine Emotion, wird gewartet (Bedingung bleibt gemerkt) statt unterbrochen.
    """

    def __init__(self, cfg: EmotionCfg, play: Callable[[str], None], busy: Callable[[], bool],
                 sound: Callable[[str], None] | None = None):
        self.cfg, self._play, self._busy, self._sound = cfg, play, busy, sound
        self._present = False                        # für die Geräusche: Person gilt als da
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
        if person is not None and self._sound is not None:
            # Geräusche: "gefunden" beim Auftauchen, "verloren" nach lost_s ohne Person.
            # Unabhängig vom Display (dort kann gerade etwas laufen) und vom Modus.
            if person and not self._present:
                self._present = True
                self._sound("gefunden")
            elif not person and self._present and self._last_person is not None                     and now - self._last_person >= c.lost_s:
                self._present = False
                self._sound("verloren")
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
        self._snd_proc: subprocess.Popen | None = None
        self._aplay = shutil.which("aplay")
        self.sounds = {"gefunden": os.path.join(cfg.sound_dir, cfg.sound_found),
                       "verloren": os.path.join(cfg.sound_dir, cfg.sound_lost)}
        self.sound_enabled = bool(cfg.sound and self._aplay
                                  and all(os.path.isfile(f) for f in self.sounds.values()))
        # Töne des Modus pet: pet_<Ereignis>_<n>.wav (tools/sounds.py), einmal beim Start gesucht
        self.pet_sounds: dict[str, list[str]] = {}
        for f in sorted(glob.glob(os.path.join(cfg.sound_dir, "pet_*.wav"))):
            self.pet_sounds.setdefault(os.path.basename(f).split("_")[1], []).append(f)
        self.trigger = EmotionTrigger(cfg, self.play, lambda: active(self.fifo),
                                      self.sound if self.sound_enabled else None)
        if cfg.enabled and not self.enabled:
            log.info("Emotionen aus (%s oder %s fehlt)", cfg.dir, PLAYER)
        if cfg.sound and not self.sound_enabled:
            log.info("Geräusche aus (aplay oder %s fehlt)", " / ".join(self.sounds.values()))

    def sound(self, event: str) -> None:
        """Geräusch im eigenen Prozess; ein noch laufendes wird abgebrochen."""
        variants = self.pet_sounds.get(event)
        f = random.choice(variants) if variants else self.sounds.get(event)
        if not f:
            return
        if self._snd_proc is not None and self._snd_proc.poll() is None:
            self._snd_proc.terminate()
        log.info("Geräusch %s", event)
        cmd = [self._aplay, "-q"] + (["-D", self.cfg.sound_device] if self.cfg.sound_device else []) + [f]
        self._snd_proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL)

    def _play_display(self, name: str) -> None:
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

    def play(self, name: str) -> None:
        if self.enabled:
            self._play_display(name)

    def express(self, kind: str, name: str, urgent: bool = False) -> None:
        """Ereignis aus dem Modus pet. Dringendes unterbricht (der neueste Player gewinnt), alles andere
        entfällt, solange noch etwas läuft."""
        if kind == "emotion":
            if urgent or not active(self.fifo):
                self.play(name)
        elif self.sound_enabled and (urgent or self._snd_proc is None or self._snd_proc.poll() is not None):
            self.sound(name)

    def update(self, now: float, mode: str, person: bool | None, battery: int | None) -> None:
        if self.enabled or self.sound_enabled:
            self.trigger.update(now, mode, person, battery)
