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

import cv2
import numpy as np

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

    @property
    def center(self) -> float:
        if not self.sectors:
            return 1.0
        n = len(self.sectors)
        return min(self.sectors[n // 2 - (n % 2 == 0): n // 2 + 1])


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
        small = cv2.resize(bgr, (self.W, self.H), interpolation=cv2.INTER_AREA)
        if self._sgain is not None:
            # = clip(small * gain, 0, 255) -> uint8; Verstärkung >= 0, also nur oben begrenzen
            # (np.clip hat in der numpy-Version am Rider viel Overhead)
            buf = self._fbuf
            np.multiply(small, self._sgain, out=buf)
            np.minimum(buf, 255, out=buf)
            small = buf.astype(np.uint8)
        small = cv2.GaussianBlur(small, (5, 5), 0)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        x0, y0, x1, y1 = self._sample_rect()
        sample = hsv[y0:y1, x0:x1]
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
        edges = cv2.Canny(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), 40, 120)
        edges = cv2.dilate(edges, self._k3)
        np.multiply(prob, np.float32(self.cfg.edge_weight), out=prob, where=edges > 0)
        floor = (prob > self.cfg.floor_thresh).astype(np.uint8) * 255
        if self._srel is not None:
            floor[~self._srel] = 255      # Farbe nicht messbar: unbekannt, nie Hindernis
        floor = cv2.morphologyEx(floor, cv2.MORPH_OPEN, self._k3)
        floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, self._k5)

        hz_full = self.geo.horizon_row(pitch_dev) if self.geo else self.H * self.scale / 2
        horizon = hz_full / self.scale
        horizon = min(max(float(horizon), self.H * 0.2), self.H * 0.8)
        cols = self._column_free(floor, horizon, pitch_dev)
        hz_i = int(horizon)
        n_unknown = 0
        if self._srel is not None:
            # Spalten mit zu wenig messbarem Boden unter dem Horizont: unbekannt statt frei
            frac = self._srel[hz_i:].mean(axis=0)
            unknown = frac < self.cfg.min_known_frac
            n_unknown = int(unknown.sum())
            if n_unknown:
                cols = cols.copy()
                cols[unknown] = np.nan
        n = self.cfg.sectors
        edges_x = np.linspace(0, self.W, n + 1).astype(int)
        sectors = [_p20(cols[edges_x[i]:edges_x[i + 1]]) for i in range(n)]
        if self._srel is not None:
            rel_b = self._srel[hz_i:]
            n_b = int(np.count_nonzero(rel_b))
            ff = np.count_nonzero(np.logical_and(floor[hz_i:] > 0, rel_b)) / n_b if n_b else None
        else:
            below = floor[hz_i:]
            ff = float((below > 0).mean()) if below.size else None
        self.diag = {"updated": updated, "match": None if bpm is None else round(bpm / 255.0, 3),
                     "occupied": round(occupied, 2), "sample_px": int((smask > 0).sum()) if smask is not None else int(sample.shape[0] * sample.shape[1]),
                     "floor_frac": None if ff is None else round(float(ff), 3), "unknown_cols": n_unknown}
        known = None
        if self._srel is not None:
            known = np.clip(self._unknown_rows / max(1.0, self.H - horizon), 0.0, 1.0)
            if n_unknown:
                known = np.where(np.isnan(cols), 1.0, known)
        return FreeSpace(sectors, cols, horizon, floor, True, known)

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
        need = np.full(R, float(self.cfg.min_obst_rows), np.float32)
        h = self.cfg.min_obst_height_m
        if h > 0 and self.geo is not None:
            g = self.geo
            y_base = self._y_base[:R]                     # Unterkante der Zeile, Vollbild
            d = g.ground_dist_np(y_base, pitch_dev)
            off = math.radians(g.tilt_deg) + pitch_dev
            with np.errstate(invalid="ignore"):
                y_top = g.H / 2 + g.f * np.tan(np.arctan2(g.cam_h - h, d) + off)
                rows_h = (y_base - y_top) / self.scale
            need = np.maximum(need, np.where(np.isfinite(rows_h), rows_h, 0.0)).astype(np.float32)
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

    def __init__(self, est: FreeSpaceEstimator, cpus=None):
        self.est = est
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

    def offer(self, bgr, pitch_dev: float, boxes) -> None:
        with self._cond:
            self._job = (bgr, pitch_dev, list(boxes))
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
                fs = self.est.update(*job)
            except Exception:
                log.exception("Bodenmodell fehlgeschlagen")
                fs = None
            ms = (time.perf_counter() - t0) * 1000
            self.ms = 0.8 * self.ms + 0.2 * ms if self.ms else ms
            self.busy = False
            if fs is not None:
                with self._cond:
                    self._result = fs
