"""ctypes-Anbindung des Augengenerators (st7789/project/riderpi/src/augen*.c) für Tests und Vorschau am PC.

Am Rider zeichnet der Display-Daemon selbst; dies hier lädt nur libxgoaugen.so (make -C st7789/project/riderpi
augen-lib, gcc). Fehlt sie, baut laden() sie mit gcc in ein Zwischenverzeichnis; ohne gcc: None.

  XGONAV_AUGENLIB=/pfad/libxgoaugen.so   fertige Bibliothek nehmen
"""
from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "st7789", "project", "riderpi", "src")
ABI = 1                                  # augen.h AUGEN_ABI

FELDER = ["dx", "dy", "w", "h", "r", "rot", "sc", "top", "tslope", "bot", "bcurve", "pupil", "ppx", "ppy", "hl",
          "mix", "spin", "tear", "blink"]
ART = {"eye": 0, "heart": 1, "happy": 2, "closed": 3, "spiral": 4, "chev": 5}
ART_NAME = {v: k for k, v in ART.items()}
W, H = 320, 240


class Auge(ctypes.Structure):
    _fields_ = [(f, ctypes.c_float) for f in FELDER] + [("kind", ctypes.c_int32)]


class Bild(ctypes.Structure):
    _fields_ = [("tilt", ctypes.c_float), ("gx", ctypes.c_float), ("gy", ctypes.c_float), ("l", Auge), ("r", Auge)]


class Rechteck(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int32) for n in ("x0", "y0", "x1", "y1")]


def _bauen() -> str | None:
    gcc = shutil.which("gcc")
    if gcc is None:
        return None
    ziel = os.path.join(tempfile.mkdtemp(prefix="augen"), "libxgoaugen.so")
    quellen = [os.path.join(SRC, n) for n in ("augen.c", "augen_ausdruck.c", "augen_leben.c")]
    subprocess.run([gcc, "-O2", "-shared", "-fPIC", "-Wall", *quellen, "-lm", "-o", ziel], check=True)
    return ziel


def laden():
    """libxgoaugen.so mit Signaturen, oder None (keine Bibliothek, kein gcc)."""
    pfad = os.environ.get("XGONAV_AUGENLIB")
    if not pfad:
        fertig = os.path.join(ROOT, "st7789", "project", "riderpi", "libxgoaugen.so")
        pfad = fertig if os.path.isfile(fertig) else _bauen()
    if not pfad:
        return None
    lib = ctypes.CDLL(pfad)
    P = ctypes.POINTER
    lib.augen_abi.restype = ctypes.c_int
    lib.augen_raster.argtypes = [P(Bild), P(ctypes.c_uint8)]
    lib.augen_raster.restype = Rechteck
    lib.augen_raster565.argtypes = [P(Bild), P(ctypes.c_uint16), P(ctypes.c_uint16)]
    lib.augen_raster565.restype = Rechteck
    lib.augen_lut.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, P(ctypes.c_uint16)]
    lib.augen_ausdruck_n.restype = ctypes.c_int
    lib.augen_ausdruck_name.argtypes = [ctypes.c_int]
    lib.augen_ausdruck_name.restype = ctypes.c_char_p
    lib.augen_ausdruck_dauer.argtypes = [ctypes.c_int]
    lib.augen_ausdruck_dauer.restype = ctypes.c_float
    lib.augen_ausdruck_index.argtypes = [ctypes.c_char_p]
    lib.augen_ausdruck_bild.argtypes = [ctypes.c_int, ctypes.c_float, P(Bild)]
    lib.bild_neutral.argtypes = [P(Bild)]
    lib.leben_neu.argtypes = [ctypes.c_uint32]
    lib.leben_neu.restype = ctypes.c_void_p
    lib.leben_frei.argtypes = [ctypes.c_void_p]
    lib.leben_befehl.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double]
    lib.leben_bild.argtypes = [ctypes.c_void_p, ctypes.c_double, P(Bild)]
    lib.leben_ausdruck_laeuft.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.leben_farbe.argtypes = [ctypes.c_void_p, ctypes.c_double, P(ctypes.c_uint8)]
    if lib.augen_abi() != ABI:
        raise RuntimeError(f"libxgoaugen.so ABI {lib.augen_abi()} statt {ABI}")
    return lib


def aus_python(frame) -> Bild:
    """(global, links, rechts) aus tools/eyes.py -> Bild"""
    gl, pl, pr = frame
    b = Bild(tilt=gl.get("tilt", 0.0), gx=gl.get("gx", 0.0), gy=gl.get("gy", 0.0))
    for ziel, p in ((b.l, pl), (b.r, pr)):
        for f in FELDER:
            setattr(ziel, f, float(p[f]))
        ziel.kind = ART[p["kind"]]
    return b


def raster(lib, b: Bild) -> np.ndarray:
    """Abdeckung 0..1 (240 x 320) wie tools/eyes.py render()"""
    a = np.zeros((H, W), np.uint8)
    lib.augen_raster(ctypes.byref(b), a.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
    return a.astype(np.float32) / 255.0


def ausdruck(lib, name: str, t: float) -> Bild:
    b = Bild()
    lib.augen_ausdruck_bild(lib.augen_ausdruck_index(name.encode()), t, ctypes.byref(b))
    return b
