"""Ausgabe aufs 2"-Display über den st7789-Daemon (FIFO /tmp/lcd0, RGB565 320x240)."""
from __future__ import annotations

import errno
import fcntl
import logging
import os
import threading
import time

import cv2
import numpy as np

from .app import App, render
from .emotion import active as emotion_active, load_gaze

log = logging.getLogger(__name__)


def to_rgb565(bgr: np.ndarray) -> bytes:
    """Wie ffmpeg -pix_fmt rgb565 (LE). OpenCV packt BGR565 genauso (R oben, B unten),
    ist aber ~3x schneller als die numpy-Bitschieberei (CM4: 0,9 statt 2,7 ms)."""
    img = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2BGR565).tobytes()


class LcdOut:
    def __init__(self, app: App, fifo: str, fps: int):
        self.app, self.fifo, self.period = app, fifo, 1.0 / max(1, fps)
        self._fd: int | None = None
        self._atlas = None              # Augen des Modus pet, beim ersten Bedarf geladen
        self._atlas_tried = False
        self._gaze_sent = None
        threading.Thread(target=self._run, name="lcd", daemon=True).start()

    def _open(self) -> bool:
        try:
            fd = os.open(self.fifo, os.O_WRONLY | os.O_NONBLOCK)
        except OSError as e:
            if e.errno not in (errno.ENXIO, errno.ENOENT):
                log.warning("LCD: %s", e)
            return False
        # blockierend schreiben, sonst zerreißen Teil-Writes den Rohdatenstrom
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl & ~os.O_NONBLOCK)
        self._fd = fd
        return True

    def _gaze_frame(self, gaze) -> bool:
        """Modus pet: Augenbild statt Overlay; schreibt nur, wenn es sich ändert (spart render + Verkleinern).
        False: kein Atlas, das Overlay läuft weiter."""
        if not self._atlas_tried:
            self._atlas_tried = True
            path = self.app.cfg.pet.gaze_dir
            self._atlas = load_gaze(path)
            if self._atlas is None:
                log.warning("LCD: kein Augen-Atlas in %s (tools/eyes.py --gaze)", path)
        if self._atlas is None:
            return False
        if gaze != self._gaze_sent:
            try:
                os.write(self._fd, self._atlas[gaze])
                self._gaze_sent = gaze
            except OSError:
                os.close(self._fd)
                self._fd = None
        time.sleep(self.period)
        return True

    def _run(self) -> None:
        while not self.app.running:
            time.sleep(0.1)
        while self.app.running:
            t0 = time.monotonic()
            if emotion_active(self.fifo):
                # Emotion läuft: FIFO freigeben. Erst wenn alle Schreiber zu sind, setzt
                # der Daemon seinen Bildanfang zurück (falls ein Player mitten im Bild endete).
                if self._fd is not None:
                    os.close(self._fd)
                    self._fd = None
                self._gaze_sent = None          # danach das Augenbild neu schreiben
                time.sleep(0.2)
                continue
            if self._fd is None and not self._open():
                time.sleep(2.0)
                continue
            gaze = getattr(self.app.behavior, "gaze", None)
            if gaze is not None and self._gaze_frame(gaze):
                continue
            self._gaze_sent = None
            snap = self.app.snapshot
            if snap.idx == getattr(self, "_last_idx", None):   # kein neues Bild (z. B. pausiert)
                time.sleep(self.period)
                continue
            self._last_idx = snap.idx
            img = render(snap)
            if img is not None:
                try:
                    os.write(self._fd, to_rgb565(img))
                except OSError:
                    os.close(self._fd)
                    self._fd = None
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))
