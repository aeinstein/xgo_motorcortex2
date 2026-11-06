"""FloorNet: selbst trainiertes Bodennetz im C++-Kern (native/floornet.cpp, Training unter tools/floornet/).

Liefert je Spalte die Vollbildzeile der Bodengrenze. Vorerst nur im Schattenbetrieb: Es rechnet mit und wird
im Overlay gezeigt, entscheidet aber nichts. Fehlt der C++-Kern oder besteht er die Selbstprüfung nicht, ist
es aus, das Kantenverfahren läuft unverändert.
"""
from __future__ import annotations

import ctypes
import logging
import os

import numpy as np

from . import fastlib

log = logging.getLogger(__name__)

_ABI = 1
NCOLS, NBINS = 80, 64
FULL_W, FULL_H = 640, 480
SELFTEST_MAX = 1e-3         # größter relativer Fehler der Logits gegen PyTorch


class FloorNet:
    def __init__(self, lib, handle):
        self._lib = lib
        self._h = handle

    @classmethod
    def load(cls, path: str) -> "FloorNet | None":
        """Gewichte laden und prüfen. None, wenn es nicht geht; der Grund steht im Log."""
        lib = fastlib.load()
        if lib is None or not fastlib.check_abi(lib, "fn_abi", _ABI):
            log.warning("[floornet] kein C++-Kern: aus")
            return None
        if not os.path.exists(path):
            log.warning("[floornet] %s fehlt: aus", path)
            return None
        lib.fn_new.restype = ctypes.c_void_p
        lib.fn_new.argtypes = [ctypes.c_char_p]
        lib.fn_free.restype = None
        lib.fn_free.argtypes = [ctypes.c_void_p]
        lib.fn_selftest.restype = ctypes.c_double
        lib.fn_selftest.argtypes = [ctypes.c_void_p]
        lib.fn_run.restype = None
        lib.fn_run.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
        h = lib.fn_new(path.encode())
        if not h:
            log.warning("[floornet] %s nicht lesbar oder falsche Version: aus", path)
            return None
        worst = lib.fn_selftest(h)
        if worst > SELFTEST_MAX:
            log.warning("[floornet] Selbstprüfung fehlgeschlagen (Fehler %.2g): aus", worst)
            lib.fn_free(h)
            return None
        log.info("[floornet] %s geladen, Selbstprüfung %.2g", path, worst)
        return cls(lib, h)

    def run(self, bgr: np.ndarray) -> np.ndarray:
        """Vollbild BGR (480 x 640 x 3, uint8) -> Grenzzeile je Spalte (Vollbild, float32, NCOLS)."""
        if bgr.shape != (FULL_H, FULL_W, 3) or bgr.dtype != np.uint8:
            raise ValueError("FloorNet braucht 640x480 BGR, nicht %s %s" % (bgr.shape, bgr.dtype))
        rows = np.empty(NCOLS, np.float32)
        self._lib.fn_run(self._h, bgr.ctypes.data, bgr.strides[0], rows.ctypes.data, None)
        return rows

    def logits(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Wie run, zusätzlich die Logits (NBINS x NCOLS). Zum Vergleich mit PyTorch."""
        rows = np.empty(NCOLS, np.float32)
        lg = np.empty((NBINS, NCOLS), np.float32)
        self._lib.fn_run(self._h, bgr.ctypes.data, bgr.strides[0], rows.ctypes.data, lg.ctypes.data)
        return rows, lg

    def close(self) -> None:
        if self._h:
            self._lib.fn_free(self._h)
            self._h = None
