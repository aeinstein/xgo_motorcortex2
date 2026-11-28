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
    def __init__(self, app: App, fifo: str, fps: int, overlay: bool = True):
        self.app, self.fifo, self.period = app, fifo, 1.0 / max(1, fps)
        self.overlay = overlay          # False: nur WLAN-Einrichtung und pet-Augen, kein Kamerabild
        self._fd: int | None = None
        self._atlas = None              # Augen des Modus pet, beim ersten Bedarf geladen
        self._atlas_tried = False
        self._gaze_sent = None
        self._farbe = None              # Augenfarbe des gezeigten Atlas (None = Standard)
        self._farb_atlas: dict = {}     # Farbe -> Atlas (oder None, wenn er fehlt); höchstens einer
        self._wlan_sent = None
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
        atlas = self._atlas
        farbe = getattr(self.app.behavior, "augen", None)      # erkannte Person mit eigener Augenfarbe (pet)
        if farbe:
            if farbe not in self._farb_atlas:
                path = f"{self.app.cfg.pet.gaze_dir}_{farbe}"
                self._farb_atlas = {farbe: load_gaze(path)}     # nur einen halten (je 8 MB)
                if self._farb_atlas[farbe] is None:
                    log.warning("LCD: kein Augen-Atlas in %s (deploy/install.sh)", path)
            atlas = self._farb_atlas[farbe] or self._atlas
        if farbe != self._farbe:
            self._farbe, self._gaze_sent = farbe, None
        if gaze != self._gaze_sent:
            try:
                os.write(self._fd, atlas[gaze])
                self._gaze_sent = gaze
            except OSError:
                os.close(self._fd)
                self._fd = None
        time.sleep(min(self.period, 0.03))   # schneller als der Regelschritt, sonst fehlen Stufen des Lidschlags; schreibt nur bei Änderung
        return True

    def _run(self) -> None:
        while not self.app.running:
            time.sleep(0.1)
        while self.app.running:
            t0 = time.monotonic()
            if self.app.ruhe:
                # Ruhemodus: am Display stehen die Schlafaugen (api.php), nichts schreiben. Das FIFO bleibt
                # offen, dann wartet der Daemon im read() statt nachzusehen.
                self._gaze_sent = None
                time.sleep(1.0)
                continue
            if emotion_active(self.fifo):
                # Emotion läuft: FIFO freigeben. Erst wenn alle Schreiber zu sind, setzt
                # der Daemon seinen Bildanfang zurück (falls ein Player mitten im Bild endete).
                if self._fd is not None:
                    os.close(self._fd)
                    self._fd = None
                self._gaze_sent = None          # danach das Augenbild neu schreiben
                time.sleep(0.2)
                continue
            wlan = self.app.wlan
            gaze = getattr(self.app.behavior, "gaze", None)
            if not self.overlay and wlan is None and (gaze is None or self._atlas_tried and self._atlas is None):
                # nichts zu zeigen: FIFO frei lassen (Emotionen, Schlafaugen bleiben stehen)
                if self._fd is not None:
                    os.close(self._fd)
                    self._fd = None
                if self._wlan_sent is not None:
                    # Einrichtung vorbei: sonst bliebe das letzte WLAN-Bild stehen
                    self._wlan_sent = None
                    emo = getattr(self.app, "emotions", None)
                    if emo is not None:
                        emo.play("cute")
                self._gaze_sent = None
                time.sleep(0.2)
                continue
            if self._fd is None and not self._open():
                time.sleep(2.0)
                continue
            if wlan is not None:
                # WLAN-Einrichtung: Kamerabild mit Hinweis bzw. Hotspot-Daten; schreiben nur bei Änderung
                v, img = wlan.lcd_frame()
                if v != self._wlan_sent:
                    try:
                        os.write(self._fd, to_rgb565(img))
                        self._wlan_sent = v
                    except OSError:
                        os.close(self._fd)
                        self._fd = None
                self._gaze_sent = None
                self._last_idx = None
                time.sleep(0.05 if wlan.state == "qr" else 0.1)   # QR-Suche: Augen mit 20 Bildern/s
                continue
            self._wlan_sent = None
            if gaze is not None and self._gaze_frame(gaze):
                continue
            self._gaze_sent = None
            if not self.overlay:
                continue                    # pet ohne Atlas: nächste Runde schließt das FIFO
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
