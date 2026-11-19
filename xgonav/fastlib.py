"""Optionaler C++-Kern xgonav/native/libxgofast.so (ctypes).

Enthält Ego-Motion (native/egofast.cpp), das Entzerren des Kamerabilds (native/linsefast.cpp), das Eintragen in
die Raumkarte (native/gridfast.cpp), Planer, Frontier-Suche, Merkkarte, Wandabstand und Wegfolger-Ziel
(native/navfast.cpp, Anbindung in navfast.py; Bodengeometrie für beide Karten in native/xgo.h), FloorNet
(native/floornet.cpp), die Schallrichtung (native/ortfast.cpp, Anbindung in ortung.py) und die Stimmproben
(native/stimmfast.cpp, Anbindung in stimme.py); bei beiden numpy als Ersatz, gleich bis auf Rundung.
Gebaut wird am Rider mit `make -C xgonav/native`; fehlt die Bibliothek, rechnen egomotion.py, camera.py,
gridmap.py, planner.py, frontier.py und localmap.py in Python (gleiches Ergebnis), FloorNet fehlt dann: der
Rider ist blind und fährt nicht selbst. Jede Funktionsgruppe hat eine eigene ABI-Nummer, die der Aufrufer prüft.

XGONAV_FASTLIB=<Datei> lädt eine andere Bibliothek (Tests am PC: navfast.cpp und gridfast.cpp brauchen kein
OpenCV und lassen sich dort allein bauen, siehe docs/testing.md).

Die Kamera direkt über libcamera (native/camfast.cpp + linsefast.cpp) ist eine eigene Bibliothek libxgocam.so
(`make -C xgonav/native cam`, braucht libcamera-dev), damit libxgofast.so ohne libcamera baut; XGONAV_CAMLIB
lädt eine andere. Fehlt sie, nimmt camera.py picamera2.
"""
from __future__ import annotations

import ctypes
import logging
import os

log = logging.getLogger(__name__)

PATH = os.environ.get("XGONAV_FASTLIB") or os.path.join(os.path.dirname(__file__), "native", "libxgofast.so")
CAM_PATH = os.environ.get("XGONAV_CAMLIB") or os.path.join(os.path.dirname(__file__), "native", "libxgocam.so")
_cache: dict = {}


def load(path: str | None = None):
    """Bibliothek laden (einmal je Prozess und Pfad; ohne Pfad libxgofast.so). None, wenn nicht gebaut oder nicht
    ladbar. CDLL gibt das GIL während jedes Aufrufs frei."""
    path = path or PATH
    if path in _cache:
        return _cache[path]
    lib = None
    if os.path.exists(path):
        try:
            lib = ctypes.CDLL(path)
        except OSError as e:
            log.warning("%s nicht ladbar: %s", path, e)
    _cache[path] = lib
    return lib


def load_cam():
    """libxgocam.so (Kamera über libcamera) oder None."""
    return load(CAM_PATH)


def check_abi(lib, func: str, want: int, path: str | None = None) -> bool:
    """True, wenn lib die Funktion `func` hat und sie die ABI-Nummer `want` meldet."""
    if lib is None:
        return False
    path = path or PATH
    ziel = "cam" if path == CAM_PATH else ""
    try:
        f = getattr(lib, func)
    except AttributeError:
        log.warning("%s: %s fehlt – neu bauen: make -C xgonav/native %s", path, func, ziel)
        return False
    f.restype = ctypes.c_int
    got = f()
    if got != want:
        log.warning("%s: %s = %d statt %d – neu bauen: make -C xgonav/native %s", path, func, got, want, ziel)
        return False
    return True
