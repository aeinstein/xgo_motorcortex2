"""JPEG für die Anzeige im Browser: Hardware-Kodierer des Pi (native/jpegfast.cpp, /dev/video31) statt cv2.imencode.

Am RiderPi kostete cv2.imencode 640x480 (Qualität 70) 4,5 ms CPU je gesendetem Bild. Der Kodierer wird beim ersten
Bild geöffnet und geprüft (Prüfbild mit drei verschiedenen Kanälen kodieren, mit OpenCV zurücklesen, muss nahe am
Original liegen: fängt vertauschte Farben und kaputte Bilder). Schlägt das fehl, fehlt die Bibliothek oder das Gerät,
oder scheitert er mehrmals in Folge, kodiert cv2. Auch die Rohaufzeichnung (rawrec.py) nutzt ihn (eigene Instanz,
q95; cv2-Rückfall q92 wie vorher), meta.jsonl nennt je Bild den Kodierer ("jpeg").
"""
from __future__ import annotations

import ctypes
import logging
import threading

import cv2
import numpy as np

from . import fastlib

log = logging.getLogger(__name__)

_JPEG_ABI = 1


class Jpeg:
    MAX_FEHLER = 3          # so viele Fehler in Folge, dann bleibt es bei cv2

    def __init__(self, native: bool = True, dev: str = "/dev/video31"):
        self._lock = threading.Lock()
        self._dev = dev
        self._lib = None
        self._h = None                  # Handle des geöffneten Kodierers
        self._dims = None               # (w, h) des Handles
        self._out = None
        self._fehler = 0
        self.native = False             # True, sobald der Kodierer geprüft läuft
        self.zuletzt = ""               # Kodierer des letzten Bildes: "hw<q>" oder "cv<q>"
        if native:
            lib = fastlib.load()
            if fastlib.check_abi(lib, "jpeg_abi", _JPEG_ABI):
                vp, i = ctypes.c_void_p, ctypes.c_int
                lib.jpeg_open.restype = vp
                lib.jpeg_open.argtypes = [ctypes.c_char_p, i, i, i, ctypes.c_char_p, i]
                lib.jpeg_encode.restype = i
                lib.jpeg_encode.argtypes = [vp, vp, i, i, vp, i, i]
                lib.jpeg_close.restype = None
                lib.jpeg_close.argtypes = [vp]
                self._lib = lib

    def encode(self, img: np.ndarray, quality: int, quality_cv: int | None = None) -> bytes | None:
        """JPEG-Bytes des BGR-Bilds, None bei Fehler. quality_cv: Qualität im cv2-Rückfall (sonst quality);
        der Hardware-Kodierer wird ab ~95 nicht genauer, libjpeg erreicht dieselbe Treue schon tiefer."""
        if self._lib is not None and img.ndim == 3 and img.shape[2] == 3 and img.dtype == np.uint8:
            with self._lock:
                jpg = self._hw(np.ascontiguousarray(img), quality)
            if jpg is not None:
                self.zuletzt = "hw%d" % quality
                return jpg
        q = int(quality if quality_cv is None else quality_cv)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
        self.zuletzt = "cv%d" % q
        return jpg.tobytes() if ok else None

    def _schliessen(self) -> None:
        if self._h is not None:
            self._lib.jpeg_close(self._h)
        self._h = self._dims = None

    def _oeffnen(self, w: int, h: int) -> bool:
        self._schliessen()
        err = ctypes.create_string_buffer(256)
        hdl = self._lib.jpeg_open(self._dev.encode(), w, h, 80, err, len(err))
        if not hdl:
            log.warning("JPEG-Kodierer %s: %s – kodiere mit OpenCV", self._dev, err.value.decode("utf-8", "replace"))
            self._lib = None
            return False
        self._h, self._dims = hdl, (w, h)
        self._out = np.empty(w * h * 3 // 2 + 65536, np.uint8)
        # Prüfbild: Verläufe, je Kanal anders, dazu etwas Rauschen
        gx, gy = np.meshgrid(np.linspace(0, 255, w), np.linspace(0, 255, h))
        rng = np.random.default_rng(1)
        probe = np.clip(np.stack([gx, gy, 255 - gx], -1) + rng.normal(0, 4, (h, w, 3)), 0, 255).astype(np.uint8)
        n = self._lib.jpeg_encode(hdl, probe.ctypes.data, probe.strides[0], 90, self._out.ctypes.data,
                                  len(self._out), 1000)
        zurueck = cv2.imdecode(self._out[:n], cv2.IMREAD_COLOR) if n > 0 else None
        if zurueck is None or zurueck.shape != probe.shape or \
                float(np.abs(zurueck.astype(np.int16) - probe).mean()) > 6.0:
            log.warning("JPEG-Kodierer %s besteht die Prüfung nicht (Rückgabe %d) – kodiere mit OpenCV", self._dev, n)
            self._schliessen()
            self._lib = None
            return False
        if not self.native:
            log.info("JPEG für die Anzeige im Hardware-Kodierer (%s, %dx%d)", self._dev, w, h)
        self.native = True
        return True

    def _hw(self, img: np.ndarray, quality: int) -> bytes | None:
        h, w = img.shape[:2]
        if self._lib is None:
            return None
        if self._dims != (w, h) and not self._oeffnen(w, h):
            return None
        n = self._lib.jpeg_encode(self._h, img.ctypes.data, img.strides[0], int(quality), self._out.ctypes.data,
                                  len(self._out), 500)
        if n > 0:
            self._fehler = 0
            return self._out[:n].tobytes()
        self._fehler += 1
        log.warning("JPEG-Kodierer: Fehler %d (%d in Folge)", n, self._fehler)
        self._schliessen()                      # Zustand unklar: beim nächsten Bild neu öffnen
        if self._fehler >= self.MAX_FEHLER:
            log.warning("JPEG-Kodierer %dx in Folge gescheitert – kodiere ab jetzt mit OpenCV", self._fehler)
            self._lib = None
            self.native = False
        return None

    def close(self) -> None:
        with self._lock:
            if self._lib is not None:
                self._schliessen()
