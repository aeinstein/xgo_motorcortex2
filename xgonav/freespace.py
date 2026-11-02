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
from dataclasses import dataclass, field

import cv2
import numpy as np

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

    @property
    def center(self) -> float:
        if not self.sectors:
            return 1.0
        n = len(self.sectors)
        return min(self.sectors[n // 2 - (n % 2 == 0): n // 2 + 1])


def _p20(a: np.ndarray) -> float:
    """20-%-Quantil mit linearer Interpolation (wie np.percentile, aber ohne dessen Overhead)."""
    s = np.sort(a)
    pos = 0.2 * (len(s) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    return float(s[lo] + (s[hi] - s[lo]) * (pos - lo))


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
        self._srel: np.ndarray | None = None       # zuverlässige Pixel (bool)
        if cfg.shading_file and not os.path.exists(cfg.shading_file):
            log.warning("Abschattungs-Korrektur %s fehlt – ohne Korrektur (tools/calibrate.py shading)",
                        cfg.shading_file)
        elif cfg.shading_file:
            m = ShadingModel.load(cfg.shading_file)
            self._sgain, self._srel = m.gain_and_reliable(self.W, self.H, cfg.shading_max_gain)
            # je Spalte: Zeilen von unten bis zum ersten zuverlässigen Pixel (H = keines)
            inv = self._srel[::-1]
            self._unknown_rows = np.where(inv.any(axis=0), inv.argmax(axis=0), self.H).astype(np.float32)

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
        small = cv2.resize(bgr, (self.W, self.H), interpolation=cv2.INTER_AREA)
        if self._sgain is not None:
            small = np.clip(small.astype(np.float32) * self._sgain, 0, 255).astype(np.uint8)
        small = cv2.GaussianBlur(small, (5, 5), 0)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        x0, y0, x1, y1 = self._sample_rect()
        sample = hsv[y0:y1, x0:x1]
        smask = None
        if self._srel is not None:
            rel = self._srel[y0:y1, x0:x1]
            if rel.mean() > 0.2:          # sonst lieber die ganze Probe als fast keine
                smask = rel.astype(np.uint8) * 255
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
        if occupied > 0.8 and self.model is not None:
            pass                          # Probe fast ganz verdeckt: Modell nicht anfassen
        elif self.model is None:
            self.model = hist
            updated = True
        else:
            # Modell nur nachführen, wenn die Probe zum Modell passt (sonst steht
            # wahrscheinlich gerade ein Hindernis im Probenbereich)
            bp = cv2.calcBackProject([sample], [0, 1], self.model, self._ranges, 1)
            bpm = float(bp[smask > 0].mean()) if smask is not None and (smask > 0).any() else float(bp.mean())
            if bpm / 255.0 > self.cfg.floor_thresh * 2:
                self.model = cv2.addWeighted(self.model, 0.9, hist, 0.1, 0)
                updated = True

        prob = cv2.calcBackProject([hsv], [0, 1], self.model, self._ranges, 1).astype(np.float32) / 255.0
        # Graue Böden: Hue ist bei niedriger Sättigung Rauschen -> S-Kanal glätten hilft,
        # zusätzlich Helligkeitsähnlichkeit als schwacher Faktor
        v = hsv[..., 2].astype(np.float32)
        vref = float(np.median(sample[..., 2][smask > 0] if smask is not None else sample[..., 2]))
        prob *= np.clip(1.2 - np.abs(v - vref) / 160.0, 0.2, 1.0)
        edges = cv2.Canny(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), 40, 120)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
        prob[edges > 0] *= self.cfg.edge_weight
        floor = (prob > self.cfg.floor_thresh).astype(np.uint8) * 255
        if self._srel is not None:
            floor[~self._srel] = 255      # Farbe nicht messbar: unbekannt, nie Hindernis
        floor = cv2.morphologyEx(floor, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        hz_full = self.geo.horizon_row(pitch_dev) if self.geo else self.H * self.scale / 2
        horizon = hz_full / self.scale
        horizon = float(np.clip(horizon, self.H * 0.2, self.H * 0.8))
        cols = self._column_free(floor, horizon, pitch_dev)
        n = self.cfg.sectors
        edges_x = np.linspace(0, self.W, n + 1).astype(int)
        sectors = [_p20(cols[edges_x[i]:edges_x[i + 1]]) for i in range(n)]
        hz_i = int(horizon)
        below = floor[hz_i:]
        if self._srel is not None:
            below = below[self._srel[hz_i:]]
        self.diag = {"updated": updated, "match": None if bpm is None else round(bpm / 255.0, 3),
                     "occupied": round(occupied, 2), "sample_px": int((smask > 0).sum()) if smask is not None else int(sample.shape[0] * sample.shape[1]),
                     "floor_frac": round(float((below > 0).mean()), 3) if below.size else None}
        known = None
        if self._srel is not None:
            known = np.clip(self._unknown_rows / max(1.0, self.H - horizon), 0.0, 1.0)
        return FreeSpace(sectors, cols, horizon, floor, True, known)

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
        # den Horizont hinaus: ein Gegenstand endet nicht an der Horizontlinie)
        idx = np.arange(H, dtype=np.int32)[:, None]
        # nächste Bodenzeile oberhalb (bzw. H): kumulatives Minimum von oben nach unten gelesen
        nxt = np.where(obst, H, idx)
        nxt = np.minimum.accumulate(nxt[::-1], axis=0)[::-1]
        run = np.where(obst, nxt - idx, 0).astype(np.int32)
        start = obst.copy()
        start[1:] &= ~obst[:-1]
        obst, run, start = obst[:R], run[:R], start[:R]
        need = np.full(R, float(self.cfg.min_obst_rows), np.float32)
        h = self.cfg.min_obst_height_m
        if h > 0 and self.geo is not None:
            g = self.geo
            r_idx = np.arange(R, dtype=np.float32)
            y_base = (H - r_idx) * self.scale            # Unterkante der Zeile, Vollbild
            d = g.ground_dist_np(y_base, pitch_dev)
            off = math.radians(g.tilt_deg) + pitch_dev
            with np.errstate(invalid="ignore"):
                y_top = g.H / 2 + g.f * np.tan(np.arctan2(g.cam_h - h, d) + off)
                rows_h = (y_base - y_top) / self.scale
            need = np.maximum(need, np.where(np.isfinite(rows_h), rows_h, 0.0)).astype(np.float32)
        reach = np.arange(R)[:, None] + run >= H           # Lauf reicht bis zum oberen Bildrand
        valid = start & ((run >= need[:, None]) | reach)
        has = valid.any(axis=0)
        first = np.where(has, valid.argmax(axis=0), span).astype(np.float32)
        free = np.clip(first / span, 0.0, 1.0)
        # glätten (einzelne Ausreißerspalten)
        free = cv2.erode(free.reshape(1, -1), np.ones((1, 5), np.uint8)).ravel()
        return free

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
