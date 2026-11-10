"""Monokulare Freiraumschätzung: Boden per Farbmodell erkennen.

Idee (Ulrich & Nourbakhsh 2000, "appearance-based obstacle detection"): Der
Bereich direkt vor dem Roboter ist (fast immer) Boden. Dessen Farbverteilung
(H/S-Histogramm) wird gelernt; jedes Pixel bekommt per Rückprojektion eine
Boden-Wahrscheinlichkeit. Kanten (Texturen, Objektgrenzen) senken sie.
Pro Bildspalte wird von unten nach oben gesucht, wie weit der Boden reicht.

Läuft auf 160x120 in wenigen ms. Ergänzt wird es von der Time-to-Contact aus
der Ego-Motion (für Hindernisse, die zufällig bodenfarben sind).
"""
from __future__ import annotations

import logging
import math
import os
import threading
from dataclasses import dataclass, field

import ctypes

import cv2
import numpy as np

from . import fastlib
from .affinity import pin_thread
from .config import FreespaceCfg
from .geometry import CamGeometry
from .shading import ShadingModel

log = logging.getLogger(__name__)


@dataclass
class FreeSpace:
    sectors: list[float] = field(default_factory=list)  # 0 = blockiert direkt vor uns, 1 = frei bis Horizont
    columns: np.ndarray | None = None                   # pro Spalte, gleiche Normierung
    horizon_row: float = 0.0                            # in Freespace-Auflösung
    mask: np.ndarray | None = None                      # Boden-Maske (uint8) für Overlay
    valid: bool = False
    # je Spalte, gleiche Normierung wie columns: darunter ist die Farbe nicht messbar
    # (Linsenabschattung), dort gilt "unbekannt". None = überall messbar.
    known_from: np.ndarray | None = None
    # Nick (rad) und Kartenpose (x, y, th) der Aufnahme. Das Bodenmodell rechnet im eigenen Thread:
    # das Ergebnis kommt 1-3 Bilder später an, Entfernungen aus dem Nick dieses späteren Bildes
    # streuen um Dezimeter (Nicken ±3° bei 2,5 Hz). None = nicht bekannt (Tests, alte Aufrufer).
    pitch_dev: float | None = None
    pose: tuple | None = None
    # FloorNet: Grenzzeile je Netzspalte (Vollbild). Geht nur mit src = "net" in Entscheidungen ein.
    net_rows: np.ndarray | None = None
    # Woher columns und sectors stammen: "method" (Verfahren aus der Konfig) oder "net" (FloorNet).
    src: str = "method"
    # Bei src = "net": die Grenze des Verfahrens, nur zum Ansehen.
    shadow_columns: np.ndarray | None = None

    @property
    def center(self) -> float:
        if not self.sectors:
            return 1.0
        n = len(self.sectors)
        return min(self.sectors[n // 2 - (n % 2 == 0): n // 2 + 1])


def net_to_columns(rows: np.ndarray, horizon: float, H: int, W: int, scale: float) -> np.ndarray:
    """Grenzzeilen des Netzes (Vollbild) -> FreeSpace.columns (W Spalten, 0 = direkt vor uns, 1 = bis zum Horizont).
    Umkehrung von Zeile = H - Wert * (H - Horizont). Jede Spalte nimmt die Netzspalte, in der sie liegt: Glätten
    würde Grenzen erfinden."""
    idx = ((np.arange(W) + 0.5) * len(rows) / W).astype(int)
    return np.clip((H - np.asarray(rows, np.float32)[idx] / scale) / max(1.0, H - horizon), 0.0, 1.0).astype(np.float32)


def _p20(a: np.ndarray) -> float:
    """20-%-Quantil mit linearer Interpolation (wie np.percentile, aber ohne dessen Overhead).
    NaN-Spalten (unbekannt) zählen nicht; ganz unbekannt = 0.0 (nicht als frei melden)."""
    a = a[np.isfinite(a)]
    if not len(a):
        return 0.0
    s = np.sort(a)
    pos = 0.2 * (len(s) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    return float(s[lo] + (s[hi] - s[lo]) * (pos - lo))


def column_runs(columns) -> list[tuple[int, int]]:
    """Zusammenhängende Bereiche bekannter Spalten (Start, Ende exklusiv) – zum Zeichnen der
    Bodengrenze, die an unbekannten (NaN-)Spalten unterbrochen wird."""
    ok = np.isfinite(np.asarray(columns, float))
    runs, i, n = [], 0, len(ok)
    while i < n:
        if ok[i]:
            j = i
            while j < n and ok[j]:
                j += 1
            runs.append((i, j))
            i = j
        else:
            i += 1
    return runs


_EDGE_ABI = 2


class _EdgeParams(ctypes.Structure):         # = EdgeParams in native/floorfast.cpp
    _fields_ = [(n, ctypes.c_int) for n in ("H", "W", "k", "edge_thin", "use_log", "edge_glare", "glare_v",
                                            "glare_px", "glare_spread", "pad0", "pad1", "pad2")] + \
               [(n, ctypes.c_double) for n in ("step_thr", "seg_thr", "chroma_w", "glare_dl")] + \
               [("lut", ctypes.c_uint8 * 256)]


def _edge_lib():
    """C++-Kern mit passender Kanten-ABI, sonst None."""
    lib = fastlib.load()
    if not fastlib.check_abi(lib, "edge_abi", _EDGE_ABI):
        return None
    lib.edge_columns.restype = ctypes.c_int
    lib.edge_columns.argtypes = [ctypes.POINTER(_EdgeParams), ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
                                 ctypes.c_void_p, ctypes.c_double, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    lib.area_down.restype = ctypes.c_int
    lib.area_down.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                              ctypes.c_int, ctypes.c_void_p]
    return lib


_area_mode_cache: dict = {}


def _area_mode(lib, w: int, h: int, f: int):
    """Rundungsvariante von area_down, die am Gerät bitgleich zu cv2.resize(INTER_AREA) ist
    (Rauschbild + glattes Bild), sonst None. Einmal je Größe."""
    key = (w, h, f)
    if key not in _area_mode_cache:
        rng = np.random.default_rng(7)
        tests = [rng.integers(0, 256, (h, w, 3), dtype=np.uint8)]
        tests.append(cv2.GaussianBlur(tests[0], (0, 0), 3))
        mode = None
        for m in (0, 1):
            ok = True
            for img in tests:
                ref = cv2.resize(img, (w // f, h // f), interpolation=cv2.INTER_AREA)
                out = np.empty_like(ref)
                if lib.area_down(img.ctypes.data, w, h, img.strides[0], 3, f, m, out.ctypes.data) != 0 or \
                        not np.array_equal(out, ref):
                    ok = False
                    break
            if ok:
                mode = m
                break
        if mode is None:
            log.info("area_down weicht von cv2.resize ab – Bodenmodell verkleinert mit OpenCV")
        _area_mode_cache[key] = mode
    return _area_mode_cache[key]


class FreeSpaceEstimator:
    def __init__(self, cfg: FreespaceCfg, full_size: tuple[int, int], geo: CamGeometry | None = None):
        self.cfg = cfg
        self.geo = geo
        self.W = cfg.width
        self.H = int(round(full_size[1] * cfg.width / full_size[0]))
        self.scale = full_size[0] / cfg.width
        self.model: np.ndarray | None = None
        bins = cfg.hist_bins
        self._bins = [bins, bins]
        self._ranges = [0, 180, 0, 256]
        self.diag: dict = {}                        # Diagnose des letzten update() (Rohaufzeichnung)
        self._sgain: np.ndarray | None = None      # Korrektur der Linsenabschattung (H, W, 3)
        self._srel: np.ndarray | None = None       # zuverlässige Pixel (bool), alle Kanäle
        self._bgonly: np.ndarray | None = None     # nur Blau+Grün messbar -> Modell ohne Rot
        self.model_bg: np.ndarray | None = None
        self._reset_req = False                    # reset(): beim nächsten update() neu lernen
        if cfg.shading_file and not os.path.exists(cfg.shading_file):
            log.warning("Abschattungs-Korrektur %s fehlt – ohne Korrektur (tools/calibrate.py shading)",
                        cfg.shading_file)
        elif cfg.shading_file:
            m = ShadingModel.load(cfg.shading_file)
            self._sgain, self._srel = m.gain_and_reliable(self.W, self.H, cfg.shading_max_gain)
            if cfg.bg_model:
                # Blau/Grün bis shading_max_gain_bg korrigieren (Rot bleibt bei shading_max_gain)
                g_bg, _ = m.gain_and_reliable(self.W, self.H, cfg.shading_max_gain_bg)
                self._sgain[..., :2] = g_bg[..., :2]
                rel_bg = (1.0 / m.falloff(self.W, self.H)[..., :2] <= cfg.shading_max_gain_bg).all(axis=-1)
                self._bgonly = rel_bg & ~self._srel
                self._rel_all = self._srel        # für die Probe des Farbmodells
                self._srel = rel_bg               # ab hier: messbar mit einem der beiden Modelle
                self._bg_bins = [32, 4]       # Helligkeit grob: Boden wird zum Rand hin real dunkler
            # je Spalte: Zeilen von unten bis zum ersten zuverlässigen Pixel (H = keines)
            inv = self._srel[::-1]
            self._unknown_rows = np.where(inv.any(axis=0), inv.argmax(axis=0), self.H).astype(np.float32)
        # Konstant je Lauf, deshalb nur einmal: Probenmaske aus der Abschattung, Puffer,
        # Zeilenindizes für _column_free (am CM4 kosteten die Neuberechnungen ~1 ms je Bild).
        x0, y0, x1, y1 = self._sample_rect()
        self._smask_rel = None
        if self._srel is not None:
            rel = (self._rel_all if self._bgonly is not None else self._srel)[y0:y1, x0:x1]
            if rel.mean() > 0.2:          # sonst lieber die ganze Probe als fast keine
                self._smask_rel = rel.astype(np.uint8) * 255
        self._fbuf = np.empty((self.H, self.W, 3), np.float32)
        self._sel = None if self._smask_rel is None else self._smask_rel > 0   # Probenpixel (bool)
        self._idx16 = np.arange(self.H, dtype=np.int16)[:, None]
        self._thr_reach = (self.H - np.arange(self.H, dtype=np.float32))[:, None]  # Lauf bis oben
        self._y_base = (self.H - np.arange(self.H, dtype=np.float32)) * self.scale  # Zeilen-Unterkante
        self._k_erode = np.ones((1, 5), np.uint8)
        self._k3 = np.ones((3, 3), np.uint8)
        self._k5 = np.ones((5, 5), np.uint8)
        fast = _edge_lib() if cfg.native else None
        self._edge_nat = fast if cfg.method in ("kanten", "beide") else None
        # Verkleinern im C++-Kern, nur wenn am Gerät bitgleich zu cv2.resize (Prüfung beim Start)
        self._area = None
        fw, fh = int(full_size[0]), int(full_size[1])
        if fast is not None and fw % self.W == 0 and fw // self.W * self.H == fh:
            f = fw // self.W
            mode = _area_mode(fast, fw, fh, f)
            if mode is not None:
                self._area = (fast, fw, fh, f, mode)
        self._edges_x: dict[int, np.ndarray] = {}
        self._atan_rows = None                     # arctan2 je Zeile für _need_rows (unabhängig vom Nicken)
        self._srel_u8 = None if self._srel is None else np.ascontiguousarray(self._srel, dtype=np.uint8)
        self._edge_out = np.empty(self.W, np.float32)
        self._edge_ng = ctypes.c_int(0)
        self.edge_backend = "c++" if self._edge_nat is not None else "python"
        if fast is not None:
            log.info("Bodenmodell: C++-Kern %s (Kantenverfahren %s, Verkleinern %s)", fastlib.PATH,
                     "ja" if self._edge_nat is not None else "nein", "ja" if self._area is not None else "nein")

    def _downscale(self, bgr: np.ndarray) -> np.ndarray:
        """= cv2.resize(bgr, (W, H), INTER_AREA); im C++-Kern, wenn beim Start als bitgleich geprüft."""
        a = self._area
        if a is not None and bgr.dtype == np.uint8 and bgr.shape == (a[2], a[1], 3) and bgr.strides[1:] == (3, 1):
            out = np.empty((self.H, self.W, 3), np.uint8)
            if a[0].area_down(bgr.ctypes.data, a[1], a[2], bgr.strides[0], 3, a[3], a[4], out.ctypes.data) == 0:
                return out
        return cv2.resize(bgr, (self.W, self.H), interpolation=cv2.INTER_AREA)

    def _edge_params(self) -> _EdgeParams:
        """Parameter für den C++-Kern; je Aufruf neu (cfg kann sich zur Laufzeit ändern, ~10 µs)."""
        c = self.cfg
        p = _EdgeParams(H=self.H, W=self.W, k=c.edge_k, edge_thin=int(bool(c.edge_thin)),
                        use_log=int(c.edge_log_s > 0), edge_glare=int(bool(c.edge_glare)), glare_v=int(c.glare_v),
                        glare_px=int(c.glare_px), glare_spread=int(c.glare_spread),
                        step_thr=float(c.edge_step_thr), seg_thr=float(c.edge_seg_thr),
                        chroma_w=float(np.float32(c.edge_chroma_w)), glare_dl=float(c.glare_dl))
        if c.edge_log_s > 0:
            ctypes.memmove(p.lut, self._log_lut_for(c.edge_log_s).ctypes.data, 256)
        return p

    def _log_lut_for(self, s_val: float) -> np.ndarray:
        if getattr(self, "_log_lut_s", None) != s_val:
            s = float(s_val)
            x = np.arange(256, dtype=np.float64)
            self._log_lut = np.round(255.0 * np.log1p(x / s) / np.log1p(255.0 / s)).astype(np.uint8)
            self._log_lut_s = s_val
        return self._log_lut

    def reset(self) -> None:
        """Bodenmodell verwerfen und beim nächsten update() neu lernen (z. B. nach Neustart
        der Kamera oder geänderter Belichtung). Darf aus jedem Thread aufgerufen werden."""
        self._reset_req = True

    def _sample_rect(self):
        y1 = int(round(self.H * (1 - self.cfg.sample_bottom)))
        y0 = max(0, y1 - int(round(self.H * self.cfg.sample_rows)))
        w = int(self.W * self.cfg.sample_cols)
        x0 = (self.W - w) // 2
        return x0, y0, x0 + w, y1

    def update(self, bgr: np.ndarray, pitch_dev: float = 0.0, exclude_boxes=()) -> FreeSpace:
        """exclude_boxes: Boxen erkannter Objekte (Vollbild-Pixel). Dort wird die Bodenfarbe
        nicht gelernt; sonst lernt das Modell z. B. die Kleidung einer Person im
        Probenbereich als Boden und erkennt danach den echten Boden nicht mehr."""
        if self._reset_req:
            self._reset_req = False
            self.model = self.model_bg = None
            log.info("Bodenmodell zurückgesetzt, lernt neu")
        small = self._downscale(bgr)
        small0 = small                    # unkorrigiert: fürs Kantenverfahren (Verlauf hat keine Kante)
        x0, y0, x1, y1 = self._sample_rect()
        # Kantenverfahren: das Farbmodell braucht nur die Probe (unteres Bilddrittel) und den
        # untersten Streifen -> nur ab knapp über der Probe rechnen (4 Zeilen Rand für Weich-
        # zeichner, Probe bleibt exakt gleich). Zeilen darüber gelten in der Maske als Boden.
        ry0 = max(0, y0 - 4) if (self.cfg.method == "kanten" and self._bgonly is None) else 0
        if ry0:
            small = small[ry0:]
        if self._sgain is not None:
            # = clip(small * gain, 0, 255) -> uint8; Verstärkung >= 0, also nur oben begrenzen
            # (np.clip hat in der numpy-Version am Rider viel Overhead)
            buf = self._fbuf[ry0:]
            np.multiply(small, self._sgain[ry0:], out=buf)
            np.minimum(buf, 255, out=buf)
            small = buf.astype(np.uint8)
        small = cv2.GaussianBlur(small, (5, 5), 0)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        sample = hsv[y0 - ry0:y1 - ry0, x0:x1]
        smask = self._smask_rel           # konstant, in __init__ berechnet; nicht verändern
        occupied = 0.0
        if len(exclude_boxes):
            occ = np.zeros((y1 - y0, x1 - x0), np.uint8)
            for bx1, by1, bx2, by2 in exclude_boxes:
                cx1, cy1 = int(bx1 / self.scale) - x0 - 1, int(by1 / self.scale) - y0 - 1
                cx2, cy2 = int(bx2 / self.scale) - x0 + 2, int(by2 / self.scale) - y0 + 2
                if cx2 > 0 and cy2 > 0 and cx1 < occ.shape[1] and cy1 < occ.shape[0]:
                    occ[max(0, cy1):cy2, max(0, cx1):cx2] = 1
            occupied = float(occ.mean())
            if occupied > 0:
                free_px = (occ == 0).astype(np.uint8) * 255
                smask = free_px if smask is None else cv2.bitwise_and(smask, free_px)
        hist = cv2.calcHist([sample], [0, 1], smask, self._bins, self._ranges)
        # Nachbar-Bins mitnehmen: leicht anders schattierter Boden ist immer noch Boden
        hist = cv2.GaussianBlur(hist, (3, 3), 0)
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)

        updated, bpm = False, None
        dark = float(sample[..., 2].mean()) < self.cfg.min_learn_v
        if (occupied > 0.8 or dark) and self.model is not None:
            pass                          # Probe verdeckt oder (fast) schwarz: Modell nicht anfassen
        elif self.model is None:
            self.model = hist
            updated = True
        else:
            # Modell nur nachführen, wenn die Probe zum Modell passt (sonst steht
            # wahrscheinlich gerade ein Hindernis im Probenbereich)
            bp = cv2.calcBackProject([sample], [0, 1], self.model, self._ranges, 1)
            sel = self._sel if smask is self._smask_rel else (None if smask is None else smask > 0)
            bpm = float(bp[sel].mean()) if sel is not None and sel.any() else float(bp.mean())
            if bpm / 255.0 > self.cfg.floor_thresh * 2:
                self.model = cv2.addWeighted(self.model, 0.9, hist, 0.1, 0)
                updated = True

        prob = cv2.calcBackProject([hsv], [0, 1], self.model, self._ranges, 1).astype(np.float32) / 255.0
        # Graue Böden: Hue ist bei niedriger Sättigung Rauschen -> S-Kanal glätten hilft,
        # zusätzlich Helligkeitsähnlichkeit als schwacher Faktor
        v = hsv[..., 2].astype(np.float32)
        vsel = self._sel if smask is self._smask_rel else (None if smask is None else smask > 0)
        vref = float(np.median(sample[..., 2][vsel] if vsel is not None else sample[..., 2]))
        fac = np.abs(v - vref)            # = clip(1.2 - |v - vref| / 160, 0.2, 1.0), ohne np.clip
        fac /= -160.0
        fac += 1.2
        np.maximum(fac, 0.2, out=fac)
        np.minimum(fac, 1.0, out=fac)
        prob *= fac
        if self._bgonly is not None:
            self._apply_bg_model(small, prob, (x0, y0, x1, y1), smask, updated)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 40, 120)
        edges = cv2.dilate(edges, self._k3)
        np.multiply(prob, np.float32(self.cfg.edge_weight), out=prob, where=edges > 0)
        floor = (prob > self.cfg.floor_thresh).astype(np.uint8) * 255
        if self._srel is not None:
            floor[~self._srel[ry0:]] = 255      # Farbe nicht messbar: unbekannt, nie Hindernis
        floor = cv2.morphologyEx(floor, cv2.MORPH_OPEN, self._k3)
        floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, self._k5)
        if ry0:
            full = np.full((self.H, self.W), 255, np.uint8)
            full[ry0:] = floor
            floor = full

        hz_full = self.geo.horizon_row(pitch_dev) if self.geo else self.H * self.scale / 2
        horizon = hz_full / self.scale
        horizon = min(max(float(horizon), self.H * 0.2), self.H * 0.8)
        n_wall = 0
        kanten = self.cfg.method in ("kanten", "beide")
        n_fb = 0
        if kanten:
            cols = self._edge_columns(small0, floor, horizon, pitch_dev)
        if self.cfg.method != "kanten":
            cf = self._column_free(floor, horizon, pitch_dev)
            if self.cfg.horizon_wall:
                cf, n_wall = self._horizon_walls(floor, gray, cf, horizon)
            if not kanten:
                cols = cf
            else:
                # "beide": Wo die Kanten nichts finden (frei bis zum Horizont, z. B. bei
                # schwachem Kontrast im Dunkeln), entscheidet das Farbmodell.
                nothing = cols >= self.cfg.edge_fallback_free
                if self._srel is not None:
                    nothing &= self._srel[min(int(horizon) + 1, self.H - 1):].mean(axis=0) >= self.cfg.min_known_frac
                n_fb = int(np.count_nonzero(nothing & (cf < cols)))
                cols = np.where(nothing, np.minimum(cols, cf), cols).astype(np.float32)
        hz_i = int(horizon)
        n_unknown = 0
        if self._srel is not None and not kanten:
            # Spalten mit zu wenig messbarem Boden unter dem Horizont: unbekannt statt frei
            frac = self._srel[hz_i:].mean(axis=0)
            unknown = frac < self.cfg.min_known_frac
            n_unknown = int(unknown.sum())
            if n_unknown:
                cols = cols.copy()
                cols[unknown] = np.nan
        n = self.cfg.sectors
        edges_x = self._edges_x.get(n)
        if edges_x is None:                      # konstant je Sektorzahl (linspace kostete ~0,2 ms je Bild)
            edges_x = self._edges_x[n] = np.linspace(0, self.W, n + 1).astype(int)
        sectors = [_p20(cols[edges_x[i]:edges_x[i + 1]]) for i in range(n)]
        r_ff = max(hz_i, ry0)                     # Diagnose: nur berechnete Zeilen
        if self._srel is not None:
            rel_b = self._srel[r_ff:]
            n_b = int(np.count_nonzero(rel_b))
            ff = np.count_nonzero(np.logical_and(floor[r_ff:] > 0, rel_b)) / n_b if n_b else None
        else:
            below = floor[r_ff:]
            ff = float((below > 0).mean()) if below.size else None
        self.diag = {"updated": updated, "match": None if bpm is None else round(bpm / 255.0, 3),
                     "occupied": round(occupied, 2), "sample_px": int((smask > 0).sum()) if smask is not None else int(sample.shape[0] * sample.shape[1]),
                     "floor_frac": None if ff is None else round(float(ff), 3), "unknown_cols": n_unknown,
                     "wall_cols": n_wall, "fallback_cols": n_fb,
                     "glare_cols": getattr(self, "_n_glare", 0) if kanten else 0}
        known = None
        if self._srel is not None and not kanten:
            known = np.clip(self._unknown_rows / max(1.0, self.H - horizon), 0.0, 1.0)
            if n_unknown:
                known = np.where(np.isnan(cols), 1.0, known)
        return FreeSpace(sectors, cols, horizon, floor, True, known, pitch_dev)

    def _apply_bg_model(self, small: np.ndarray, prob: np.ndarray, rect, smask, updated: bool) -> None:
        """Boden-Wahrscheinlichkeit ohne Rotkanal für die Pixel, wo nur Blau+Grün messbar sind
        (schreibt in prob). Merkmale: Grünanteil G/(B+G) und Helligkeit (B+G)/2. Probe und
        Nachführung wie beim Farbmodell: gelernt nur, wenn dieses gerade gelernt hat."""
        b = small[..., 0].astype(np.float32)
        g = small[..., 1].astype(np.float32)
        s_ = b + g
        chroma = (g * 255.0 / np.maximum(s_, 1.0)).astype(np.uint8)
        light = (s_ * 0.5).astype(np.uint8)
        feat = cv2.merge([chroma, light])
        x0, y0, x1, y1 = rect
        hist = cv2.calcHist([feat[y0:y1, x0:x1]], [0, 1], smask, self._bg_bins, [0, 256, 0, 256])
        hist = cv2.GaussianBlur(hist, (3, 3), 0)
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        if self.model_bg is None:
            self.model_bg = hist
        elif updated:
            self.model_bg = cv2.addWeighted(self.model_bg, 0.9, hist, 0.1, 0)
        pb = cv2.calcBackProject([feat], [0, 1], self.model_bg, [0, 256, 0, 256], 1)
        m = self._bgonly
        prob[m] = pb[m].astype(np.float32) / 255.0

    def _column_free(self, floor: np.ndarray, horizon: float, pitch_dev: float = 0.0) -> np.ndarray:
        """Je Spalte: Anteil der Strecke bis zum Horizont, der vom unteren Rand aus frei ist.
        Gesucht wird der unterste gültige Nicht-Boden-Lauf: mindestens min_obst_rows Zeilen
        und (mit Geometrie) so hoch wie ein min_obst_height_m hoher Gegenstand am Fußpunkt,
        oder bis zum oberen Bildrand reichend."""
        H = self.H
        span = max(1.0, H - horizon)
        R = max(1, H - int(np.floor(horizon)))           # Fußpunkte nur unterhalb des Horizonts
        obst = floor[::-1] == 0                           # ganze Spalte, Index 0 = unterste Zeile
        # Länge des Laufs, der in jeder Zeile beginnt (rückwärts aufsummiert, auch über
        # den Horizont hinaus: ein Gegenstand endet nicht an der Horizontlinie).
        # int16 reicht (Werte <= H) und halbiert den Speicherverkehr gegenüber int32/64.
        idx = self._idx16
        # nächste Bodenzeile oberhalb (bzw. H): kumulatives Minimum von oben nach unten gelesen
        nxt = np.where(obst, np.int16(H), idx)
        nxt = np.minimum.accumulate(nxt[::-1], axis=0)[::-1]
        ob = obst[:R]                                     # ab hier nur Zeilen unter dem Horizont
        run = nxt[:R] - idx[:R]
        run[~ob] = 0
        start = ob.copy()
        start[1:] &= ~ob[:-1]
        need = self._need_rows(R, pitch_dev)
        # gültig: Lauf hoch genug (need) oder bis zum oberen Bildrand (Zeile + Lauf >= H);
        # beides als eine Schwelle: run >= min(need, H - Zeile)
        thr = np.minimum(need[:, None], self._thr_reach[:R])
        valid = start & (run >= thr)
        has = valid.any(axis=0)
        first = np.where(has, valid.argmax(axis=0), span).astype(np.float32)
        free = first / span
        np.maximum(free, 0.0, out=free)
        np.minimum(free, 1.0, out=free)
        # glätten (einzelne Ausreißerspalten)
        free = cv2.erode(free.reshape(1, -1), self._k_erode).ravel()
        return free

    def _need_rows(self, R: int, pitch_dev: float) -> np.ndarray:
        """Je Zeile (Index 0 = unterste): Mindesthöhe eines Hindernisses in Zeilen der
        Arbeitsauflösung – min_obst_rows bzw. so hoch wie ein min_obst_height_m hoher
        aufrechter Gegenstand mit Fußpunkt in dieser Zeile."""
        need = np.full(R, float(self.cfg.min_obst_rows), np.float32)
        h = self.cfg.min_obst_height_m
        if h > 0 and self.geo is not None:
            g = self.geo
            y_base = self._y_base[:R]                     # Unterkante der Zeile, Vollbild
            # = g.ground_dist_np(y_base, pitch_dev); arctan2 je Zeile hängt nicht vom Nicken ab
            key = (g.H, g.f)
            if self._atan_rows is None or self._atan_rows[0] != key:
                self._atan_rows = (key, np.arctan2(np.asarray(self._y_base, float) - g.H / 2, g.f))
            beta = self._atan_rows[1][:R] - math.radians(g.tilt_deg) - pitch_dev
            with np.errstate(divide="ignore", invalid="ignore"):
                d = g.cam_h / np.tan(beta)
            d[beta <= math.radians(0.5)] = np.nan
            off = math.radians(g.tilt_deg) + pitch_dev
            with np.errstate(invalid="ignore"):
                y_top = g.H / 2 + g.f * np.tan(np.arctan2(g.cam_h - h, d) + off)
                rows_h = (y_base - y_top) / self.scale
            need = np.maximum(need, np.where(np.isfinite(rows_h), rows_h, 0.0)).astype(np.float32)
        return need

    def _edge_columns(self, small: np.ndarray, floor: np.ndarray, horizon: float,
                      pitch_dev: float) -> np.ndarray:
        """Kantenverfahren: je Spalte von unten nach oben die erste Stufe, über der es
        anders aussieht als darunter. Grundlage ist das unkorrigierte Bild (Abschattung und
        Lichtverläufe sind weich und erzeugen keine Stufe).
          1. LAB, dünne helle/dunkle Linien (Fugen, Kabel) per Öffnen/Schließen entfernen.
          2. Stufe in Zeile r: Mittel der edge_k Zeilen darüber gegen die edge_k darunter.
          3. Hindernis, wenn die Stufe >= edge_step_thr ist und das Stück darüber über die
             Mindesthöhe (_need_rows) im Mittel >= edge_seg_thr vom Boden darunter abweicht
             (flache Flecken und Fugen sind kürzer).
        Zusätzlich: Ist schon der unterste Streifen laut Farbmodell kein Boden, steht das
        Hindernis direkt vor dem Rider (dort gibt es keine Stufe vom Boden aus)."""
        c = self.cfg
        H, W = self.H, self.W
        k = c.edge_k
        if self._edge_nat is not None:
            return self._edge_columns_native(small, floor, horizon, pitch_dev)
        lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
        if c.edge_thin:
            lab = cv2.morphologyEx(lab, cv2.MORPH_OPEN, self._k3)
            lab = cv2.morphologyEx(lab, cv2.MORPH_CLOSE, self._k3)
        l_lin = lab[..., 0]                               # lineare Helligkeit (Lichtschein-Prüfung)
        if c.edge_log_s > 0:
            lab = lab.copy()
            lab[..., 0] = cv2.LUT(np.ascontiguousarray(l_lin), self._log_lut_for(c.edge_log_s))
        # Kumulative Ganzzahl-Summen entlang der Zeilen: Summe beliebiger Zeilenfenster in O(1).
        # Gewichtung der Farbkanäle erst beim Vergleich (spart die Gleitkomma-Kopie, CM4 ~1 ms).
        cs = np.zeros((H + 1, W, 3), np.int32)
        np.cumsum(lab, axis=0, dtype=np.int32, out=cs[1:])
        wv = np.array([1.0, c.edge_chroma_w, c.edge_chroma_w], np.float32)
        hz = int(np.ceil(horizon))
        r = np.arange(max(hz, k), H - k)                 # Kandidatenzeilen (Fußpunkt unter Horizont)
        span = max(1.0, H - horizon)
        free = np.ones(W, np.float32)
        glare = None
        if r.size:
            sb = cs[r + k] - cs[r]                       # k Zeilen Boden unmittelbar unter der Stufe
            # Stufe (als k-fache Summe): |oben - unten| je Kanal, gewichtet
            step = np.abs((cs[r] - cs[r - k]) - sb).dot(wv)
            ri, ci = np.nonzero(step >= c.edge_step_thr * k)
            if ri.size:
                # Stück darüber nur an den (seltenen) Stufen prüfen: Mittel über die
                # Mindesthöhe (bis höchstens zum oberen Bildrand) gegen den Boden darunter
                need = self._need_rows(H, pitch_dev)[H - 1 - r]
                n = np.clip(np.ceil(need).astype(np.int32), k, None)
                rr = r[ri]
                top = np.maximum(rr - n[ri], 0)
                seg = (cs[rr, ci] - cs[top, ci]) / (rr - top)[:, None].astype(np.float32)
                segd = np.abs(seg - sb[ri, ci] / np.float32(k)).dot(wv)
                ok = segd >= c.edge_seg_thr
                if ok.any():
                    # unterste Treffer-Zeile je Spalte (größtes r)
                    best = np.full(W, -1, np.int32)
                    np.maximum.at(best, ci[ok], rr[ok])
                    has = best >= 0
                    free = np.where(has, (H - 1 - best) / span, 1.0).astype(np.float32)
                    if c.edge_glare:
                        glare = self._glare_cols(small, l_lin, best, has, hz, k)
        # Hindernis schon am unteren Rand (Farbmodell, nur messbare Pixel)
        nb = max(2, k)
        bot = floor[H - nb:] == 0
        if self._srel is not None:
            bot &= self._srel[H - nb:]
        bot_hit = bot.mean(axis=0) >= 0.6
        free[bot_hit] = 0.0
        np.clip(free, 0.0, 1.0, out=free)
        self._n_glare = 0
        if glare is not None:
            glare &= ~bot_hit
            self._n_glare = int(np.count_nonzero(glare))
            if self._n_glare:
                free[glare] = 1.0                    # vor dem Glätten: nicht auf Nachbarn übertragen
        free = cv2.erode(free.reshape(1, -1), self._k_erode).ravel()
        if self._n_glare:
            free[glare] = np.nan                     # Lichtschein: unbekannt (weder Hindernis noch frei)
        return free

    def _edge_columns_native(self, small: np.ndarray, floor: np.ndarray, horizon: float,
                             pitch_dev: float) -> np.ndarray:
        """_edge_columns im C++-Kern (native/floorfast.cpp), gleiche Rechnung."""
        small = np.ascontiguousarray(small)
        floor = np.ascontiguousarray(floor)
        need = np.ascontiguousarray(self._need_rows(self.H, pitch_dev), dtype=np.float32)
        out = self._edge_out
        rc = self._edge_nat.edge_columns(ctypes.byref(self._edge_params()), small.ctypes.data, floor.ctypes.data,
                                         None if self._srel_u8 is None else self._srel_u8.ctypes.data,
                                         need.ctypes.data, float(horizon), out.ctypes.data, ctypes.byref(self._edge_ng))
        if rc != 0:
            log.warning("Kantenverfahren (C++) fehlgeschlagen – weiter mit numpy")
            self._edge_nat = None
            self.edge_backend = "python"
            return self._edge_columns(small, floor, horizon, pitch_dev)
        self._n_glare = int(self._edge_ng.value)
        return out.copy()

    def _glare_cols(self, small: np.ndarray, l_lin: np.ndarray, best: np.ndarray, has: np.ndarray,
                    hz: int, k: int) -> np.ndarray:
        """Spalten, deren unterste Stufe ein Lichtschein ist (Spiegelung einer Lampe im Boden):
        über der Stufe heller als darunter und übersteuerte Pixel zwischen Stufe und knapp über
        dem Horizont (dort steht die Lampe)."""
        c = self.cfg
        W = self.W
        glare = np.zeros(W, bool)
        cols = np.nonzero(has & (best >= k))[0]
        if not cols.size:
            return glare
        b = best[cols]
        cl = np.zeros((self.H + 1, W), np.int32)
        np.cumsum(l_lin, axis=0, dtype=np.int32, out=cl[1:])
        l_above = (cl[b, cols] - cl[b - k, cols]) / k   # L in OpenCV-LAB 0..255, linear
        l_below = (cl[np.minimum(b + k, self.H), cols] - cl[b, cols]) / k
        brighter = l_above - l_below >= c.glare_dl
        if not brighter.any():
            return glare
        ch = cv2.split(small)
        vmax = cv2.max(cv2.max(ch[0], ch[1]), ch[2])     # numpy max(axis=2) ist ~10x langsamer
        sat = (vmax >= c.glare_v).astype(np.int32)
        csat = np.zeros((self.H + 1, W), np.int32)
        np.cumsum(sat, axis=0, out=csat[1:])
        top = max(0, hz - 12)
        n_sat = csat[b, cols] - csat[np.minimum(top, b), cols]
        glare[cols] = brighter & (n_sat >= c.glare_px)
        if glare.any() and c.glare_spread > 0:
            # Ränder des Scheins (dort nicht übersteuert): Nachbarspalten, die ebenfalls über der
            # Stufe heller sind und ihre Stufe auf ähnlicher Höhe haben, gehören dazu.
            bright = np.zeros(W, bool)
            bright[cols] = brighter
            rowv = np.full(W, -1000, np.int32)
            rowv[cols] = b
            for _ in range(c.glare_spread):
                grow = np.zeros(W, bool)
                grow[1:] |= glare[:-1] & (np.abs(rowv[1:] - rowv[:-1]) <= 3)
                grow[:-1] |= glare[1:] & (np.abs(rowv[:-1] - rowv[1:]) <= 3)
                grow &= bright & ~glare
                if not grow.any():
                    break
                glare |= grow
        # Nur Streifen, die beidseitig von anderen Spalten eingerahmt sind: Am Bildrand sieht eine
        # von einer Lampe außerhalb des Bildes angestrahlte Wand genauso aus (Sockelleiste = echtes
        # Hindernis), dort bleibt es beim Hindernis.
        if glare[0] or glare[-1]:
            d = np.diff(glare.astype(np.int8))
            if glare[0]:
                end = np.argmax(d == -1) + 1 if (d == -1).any() else W
                glare[:end] = False
            if glare[-1]:
                start = W - 1 - np.argmax((d == 1)[::-1]) if (d == 1).any() else 0
                glare[start:] = False
        return glare

    def _horizon_walls(self, floor: np.ndarray, gray: np.ndarray, cols: np.ndarray,
                       horizon: float) -> tuple[np.ndarray, int]:
        """Horizontregel: Echter Boden liegt nie über dem Horizont. Läuft eine als Boden
        erkannte Fläche in einer bis zum Horizont freien Spalte über den Horizont hinaus nach
        oben weiter, ist es eine Wand in Bodenfarbe (graue Wand über grauem Boden, RiderPi
        2026-11-04). Deren Fußpunkt ist die erste kräftige waagrechte Kante unterhalb des
        Horizonts (Sockelleiste); gesucht wird vom Horizont nach unten, also die entfernteste
        Kante zuerst – Fugen zwischen Rider und Wand liegen tiefer im Bild."""
        c = self.cfg
        H = self.H
        hz = int(np.floor(horizon))
        top = hz - c.horizon_margin_rows
        bot = max(0, top - c.horizon_band_rows)
        if top <= bot or hz >= H - 1:
            return cols, 0
        band = floor[bot:top] > 0                      # Streifen knapp über dem Horizont
        if self._srel is not None:
            band &= self._srel[bot:top]                # nicht messbar zählt dort nicht als Boden
        cand = (band.mean(axis=0) >= c.horizon_floor_frac) & (cols >= 0.98)
        if not cand.any():
            return cols, 0
        # Senkrechter Helligkeitssprung je Spalte, waagrecht leicht geglättet
        g = gray.astype(np.int16)
        gy = np.zeros((H, self.W), np.float32)
        gy[1:-1] = np.abs(g[2:] - g[:-2]) * 0.5
        gy = cv2.blur(gy, (3, 1))
        strong = gy[hz:] >= c.wall_edge_thr            # Zeilen ab dem Horizont abwärts
        has = strong.any(axis=0)
        row = hz + strong.argmax(axis=0)               # erste kräftige Kante unter dem Horizont
        hit = cand & has
        if not hit.any():
            return cols, 0
        span = max(1.0, H - horizon)
        val = np.clip((H - 1 - row) / span, 0.0, 1.0).astype(np.float32)
        out = cols.copy()
        out[hit] = np.minimum(out[hit], val[hit])
        return out, int(hit.sum())

    def adopt_net(self, fs: FreeSpace, rows: np.ndarray) -> None:
        """FloorNet entscheidet: columns und sectors aus seinen Zeilen, das Verfahren bleibt als Schatten erhalten."""
        fs.shadow_columns = fs.columns
        fs.columns = net_to_columns(rows, fs.horizon_row, self.H, self.W, self.scale)
        n = len(fs.sectors)
        ex = np.linspace(0, self.W, n + 1).astype(int)
        fs.sectors = [_p20(fs.columns[ex[i]:ex[i + 1]]) for i in range(n)]
        fs.known_from = None
        fs.src = "net"

    def block_boxes(self, fs: FreeSpace, boxes) -> None:
        """Erkannte Objekte als Hindernisse eintragen (Unterkante = Fußpunkt)."""
        if fs.columns is None:
            return
        span = max(1.0, self.H - fs.horizon_row)
        changed = False
        for (x1, y1, x2, y2) in boxes:
            c1 = int(max(0, x1 / self.scale))
            c2 = int(min(self.W, x2 / self.scale + 1))
            bottom = y2 / self.scale
            if bottom <= fs.horizon_row or c2 <= c1:
                continue
            val = float(np.clip((self.H - bottom) / span, 0.0, 1.0))
            fs.columns[c1:c2] = np.minimum(fs.columns[c1:c2], val)
            changed = True
        if not changed:
            return                        # Sektoren unverändert, nicht neu berechnen
        n = len(fs.sectors)
        ex = np.linspace(0, self.W, n + 1).astype(int)
        fs.sectors = [_p20(fs.columns[ex[i]:ex[i + 1]]) for i in range(n)]


class FreeSpaceWorker:
    """Bodenmodell in eigenem Thread, nach dem Muster des DetectorWorker: offer() legt den
    neuesten Auftrag ab (ein älterer, noch nicht begonnener wird ersetzt), poll() holt das
    neueste fertige Ergebnis. Nur dieser Thread ruft est.update() auf; block_boxes() und die
    Konstanten (W, H, scale) des Schätzers darf die Hauptschleife weiter nutzen, diag wird
    je Lauf durch ein neues dict ersetzt (nie verändert)."""

    def __init__(self, est: FreeSpaceEstimator, cpus=None, net=None, net_primary: bool = False):
        self.est = est
        self.net = net                     # FloorNet oder None
        self.net_primary = net_primary     # True: Das Netz entscheidet, das Verfahren rechnet im Schatten
        self.net_ms = 0.0
        self.cpus = list(cpus or [])
        self._cond = threading.Condition()
        self._job = None
        self._result: FreeSpace | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.busy = False
        self.ms = 0.0                      # gleitendes Mittel der Rechenzeit

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="freespace", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread:
            self._thread.join(timeout=2)
        if self.net is not None and not (self._thread and self._thread.is_alive()):
            self.net.close()

    def offer(self, bgr, pitch_dev: float, boxes, pose=None) -> None:
        """pose: Kartenpose zum Bild, wird dem Ergebnis mitgegeben."""
        with self._cond:
            self._job = (bgr, pitch_dev, list(boxes), pose)
            self._cond.notify()

    def poll(self) -> FreeSpace | None:
        with self._cond:
            r, self._result = self._result, None
            return r

    def _run(self) -> None:
        if self.cpus:
            pin_thread(self.cpus, "Bodenmodell")
        import time
        while not self._stop.is_set():
            with self._cond:
                while self._job is None and not self._stop.is_set():
                    self._cond.wait(0.1)
                if self._stop.is_set():
                    return
                job, self._job = self._job, None
            self.busy = True
            t0 = time.perf_counter()
            try:
                fs = self.est.update(*job[:3])
                fs.pose = job[3]
            except Exception:
                log.exception("Bodenmodell fehlgeschlagen")
                fs = None
            ms = (time.perf_counter() - t0) * 1000
            self.ms = 0.8 * self.ms + 0.2 * ms if self.ms else ms
            if fs is not None and self.net is not None:
                self._run_net(fs, job[0])
            self.busy = False
            if fs is not None:
                with self._cond:
                    self._result = fs

    def _run_net(self, fs: FreeSpace, bgr) -> None:
        """FloorNet rechnen. Ein Fehler darf das Bodenmodell nicht treffen: Dann ist das Netz aus, das Verfahren entscheidet."""
        import time
        t0 = time.perf_counter()
        try:
            rows = self.net.run(bgr)
            if self.net_primary:
                if not np.isfinite(rows).all():
                    raise ValueError("FloorNet liefert ungültige Zeilen")
                self.est.adopt_net(fs, rows)
            fs.net_rows = rows
        except Exception:
            log.exception("FloorNet fehlgeschlagen: aus, das Verfahren entscheidet")
            self.net = None
            return
        ms = (time.perf_counter() - t0) * 1000
        self.net_ms = 0.8 * self.net_ms + 0.2 * ms if self.net_ms else ms
