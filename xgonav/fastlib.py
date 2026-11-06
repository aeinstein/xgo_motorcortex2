"""Optionaler C++-Kern xgonav/native/libxgofast.so (ctypes).

Enthält Ego-Motion (native/egofast.cpp), Kantenverfahren und Verkleinern des Bodenmodells
(native/floorfast.cpp), das Eintragen in die Raumkarte (native/gridfast.cpp) und FloorNet
(native/floornet.cpp). Gebaut wird am Rider mit `make -C xgonav/native`; fehlt die Bibliothek,
rechnen egomotion.py, freespace.py und gridmap.py in Python (gleiches Ergebnis), FloorNet ist dann aus.
Jede Funktionsgruppe hat eine eigene ABI-Nummer, die der Aufrufer prüft.
"""
from __future__ import annotations

import ctypes
import logging
import os

log = logging.getLogger(__name__)

PATH = os.path.join(os.path.dirname(__file__), "native", "libxgofast.so")
_cache: list = []


def load():
    """Bibliothek laden (einmal je Prozess). None, wenn nicht gebaut oder nicht ladbar.
    CDLL gibt das GIL während jedes Aufrufs frei."""
    if _cache:
        return _cache[0]
    lib = None
    if os.path.exists(PATH):
        try:
            lib = ctypes.CDLL(PATH)
        except OSError as e:
            log.warning("%s nicht ladbar: %s", PATH, e)
    _cache.append(lib)
    return lib


def check_abi(lib, func: str, want: int) -> bool:
    """True, wenn lib die Funktion `func` hat und sie die ABI-Nummer `want` meldet."""
    if lib is None:
        return False
    try:
        f = getattr(lib, func)
    except AttributeError:
        log.warning("%s: %s fehlt – neu bauen: make -C xgonav/native", PATH, func)
        return False
    f.restype = ctypes.c_int
    got = f()
    if got != want:
        log.warning("%s: %s = %d statt %d – neu bauen: make -C xgonav/native", PATH, func, got, want)
        return False
    return True
