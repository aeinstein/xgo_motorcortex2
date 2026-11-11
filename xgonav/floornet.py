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

_ABI = 2
NCOLS, NBINS = 80, 64
FULL_W, FULL_H = 640, 480
SELFTEST_MAX = 1e-3         # größter relativer Fehler der Logits gegen PyTorch


class FloorNet:
    def __init__(self, lib, handle):
        self._lib = lib
        self._h = handle

    @classmethod
    def load(cls, path: str, outlier_px: float = 0.0, outlier_ratio: float = 0.3) -> "FloorNet | None":
        """Gewichte laden und prüfen. None, wenn es nicht geht; der Grund steht im Log.
        outlier_px > 0 nimmt einzelne Zacken heraus, siehe set_outlier."""
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
        lib.fn_set_outlier.restype = None
        lib.fn_set_outlier.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
        lib.fn_decode.restype = None
        lib.fn_decode.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
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
        log.info("[floornet] %s geladen, Selbstprüfung %.2g, Zacken ab %.0f px (Anteil %.2f)", path, worst,
                 outlier_px, outlier_ratio)
        net = cls(lib, h)
        net.set_outlier(outlier_px, outlier_ratio)
        return net

    def set_outlier(self, px: float, ratio: float) -> None:
        """Zacken herausnehmen (wie tools/floornet/net.decode): Weicht eine Spalte um mehr als px Vollbildzeilen
        vom Median der 7 Spalten um sie ab und liegt beim Medianwert mindestens ratio der Masse ihres eigenen
        Gipfels, gilt das beste Fach beim Medianwert. px = 0: aus."""
        self._lib.fn_set_outlier(self._h, float(px), float(ratio))

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

    def decode(self, logits: np.ndarray) -> np.ndarray:
        """Nur die Auswertung des Kerns auf fremden Logits (NBINS x NCOLS). Für Tests gegen net.decode."""
        lg = np.ascontiguousarray(logits, np.float32)
        if lg.shape != (NBINS, NCOLS):
            raise ValueError("Logits müssen %d x %d sein, nicht %s" % (NBINS, NCOLS, lg.shape))
        rows = np.empty(NCOLS, np.float32)
        self._lib.fn_decode(self._h, lg.ctypes.data, rows.ctypes.data)
        return rows

    def close(self) -> None:
        if self._h:
            self._lib.fn_free(self._h)
            self._h = None


class ZeilenGlaetter:
    """Grenzzeilen des Netzes über die Zeit beruhigen, ohne das Netz zu ändern: die letzten Ausgaben werden über die
    Bodenebene auf das aktuelle Bild umgerechnet (Pose und Nicken), je Spalte gilt der Median. Ein Ausreißer in einem
    einzelnen Bild verschwindet so, ein echtes neues Hindernis erscheint bei n = 3 ein Bild später. Erkennt nichts
    besser; dauerhafte Fehler bleiben. Reines numpy auf n x NCOLS Werten, keine Bilddaten.

    Gemittelt wird nie: Bei gerader Zahl gültiger Werte fällt der älteste weg, sonst entstünde eine Zeile, die kein
    Bild gemeldet hat. Spalten, zu denen eine alte Ausgabe nichts sagt (am Horizont, aus dem Bild gedreht), zählen
    für diese Ausgabe nicht mit."""

    MAX_DT = 0.5            # s: ältere Ausgaben verfallen
    MAX_SPRUNG_M = 0.3      # Posensprung zwischen zwei Ausgaben (Kartenabgleich): Verlauf verwerfen
    MAX_SPRUNG_RAD = 0.6
    MIN_VOR_M = 0.03        # umgerechneter Punkt muss so weit vor der Kamera liegen

    def __init__(self, n: int = 3):
        self.n = max(1, int(n))
        self._alt: list[tuple] = []         # (rows, pose, pitch_dev, t), neueste zuerst

    def reset(self) -> None:
        self._alt = []

    @staticmethod
    def umrechnen(rows: np.ndarray, pose0, pitch0: float, pose1, pitch1: float, geo) -> np.ndarray:
        """Grenzzeilen eines früheren Bildes (pose0, pitch0) -> Zeilen je Spalte im aktuellen Bild (pose1, pitch1).
        geo: CamGeometry (f, H, cx, tilt_deg, cam_h, cam_fwd). NaN, wo das frühere Bild zur Spalte nichts sagt."""
        rows = np.asarray(rows, np.float64)
        n = len(rows)
        breite = FULL_W / n
        x = (np.arange(n) + 0.5) * breite
        tilt = np.radians(geo.tilt_deg)
        beta = np.arctan2(rows - geo.H / 2, geo.f) - tilt - pitch0
        ok = beta > np.radians(0.5)
        with np.errstate(divide="ignore", invalid="ignore"):
            vor = np.where(ok, geo.cam_h / np.tan(beta), np.nan)       # nach vorn ab Kamerafußpunkt
            links = vor * (geo.cx - x) / geo.f
            # Welt: Pose ist die Radachse, die Kamera sitzt cam_fwd davor
            c0, s0 = np.cos(pose0[2]), np.sin(pose0[2])
            wx = pose0[0] + (vor + geo.cam_fwd) * c0 - links * s0
            wy = pose0[1] + (vor + geo.cam_fwd) * s0 + links * c0
            c1, s1 = np.cos(pose1[2]), np.sin(pose1[2])
            dx, dy = wx - pose1[0], wy - pose1[1]
            vor1 = dx * c1 + dy * s1 - geo.cam_fwd
            links1 = -dx * s1 + dy * c1
            ok &= vor1 > ZeilenGlaetter.MIN_VOR_M
            x1 = geo.cx - geo.f * links1 / vor1
            r1 = geo.H / 2 + geo.f * np.tan(np.arctan2(geo.cam_h, vor1) + tilt + pitch1)
        out = np.full(n, np.nan, np.float32)
        if not ok.any():
            return out
        x1, r1 = x1[ok], r1[ok]
        # je aktueller Spalte der nächstgelegene alte Punkt, höchstens eine Spaltenbreite entfernt (nicht interpolieren)
        d = np.abs(x1[None, :] - x[:, None])
        j = d.argmin(axis=1)
        nah = d[np.arange(n), j] <= breite
        out[nah] = r1[j[nah]]
        return out

    def glaetten(self, rows: np.ndarray, pose, pitch_dev: float, t: float, geo) -> np.ndarray:
        """Aktuelle Ausgabe aufnehmen und die beruhigte zurückgeben. pose = (x, y, th) der Radachse oder None."""
        rows = np.asarray(rows, np.float32)
        if self.n <= 1 or pose is None or geo is None:
            self.reset()
            return rows
        if self._alt:
            p, t0 = self._alt[0][1], self._alt[0][3]
            dth = abs((pose[2] - p[2] + np.pi) % (2 * np.pi) - np.pi)
            if (t - t0 > self.MAX_DT or t < t0 or np.hypot(pose[0] - p[0], pose[1] - p[1]) > self.MAX_SPRUNG_M
                    or dth > self.MAX_SPRUNG_RAD):
                self.reset()
        m = np.full((1 + len(self._alt), len(rows)), np.nan, np.float32)
        m[0] = rows
        for k, (r, p, pd, tk) in enumerate(self._alt):
            if t - tk <= self.MAX_DT:
                m[k + 1] = self.umrechnen(r, p, pd, pose, pitch_dev, geo)
        self._alt = [(rows.copy(), tuple(pose), float(pitch_dev), float(t))] + self._alt[:self.n - 2]
        if len(m) == 1:
            return rows
        da = np.isfinite(m)
        gerade = da.sum(axis=0) % 2 == 0
        for k in range(len(m) - 1, 0, -1):                             # bei gerader Zahl den ältesten gültigen streichen
            weg = gerade & da[k]
            m[k, weg] = np.nan
            da[k, weg] = False
            gerade &= ~weg
        m.sort(axis=0)                                                  # NaN ans Ende; Zahl gültiger Werte ist ungerade
        return m[(da.sum(axis=0) - 1) // 2, np.arange(m.shape[1])]
