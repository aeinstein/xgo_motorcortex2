"""Multi-Objekt-Tracker mit Ego-Motion-Kompensation.

Jeder Frame: Kamerabewegung (A) auf alle Tracks anwenden, dann Eigenbewegung
des Objekts vorhersagen (Alpha-Beta-Filter ~ stationärer Kalman). Kommen
Detektionen (verzögert, bereits per Ego-Motion auf den aktuellen Frame
verschoben), werden sie per IoU zugeordnet. So bleiben Tracks stabil, obwohl
das Bild wackelt und der Detektor nur ein paar FPS schafft.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, field

import numpy as np

from .config import TrackerCfg
from .egomotion import warp_box

_ids = itertools.count(1)


def iou(a, b) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


@dataclass
class Track:
    cls: str
    cx: float
    cy: float
    w: float
    h: float
    conf: float
    t_seen: float
    id: int = field(default_factory=lambda: next(_ids))
    vx: float = 0.0       # px/s Eigenbewegung im Bild (ohne Kamerabewegung)
    vy: float = 0.0
    vs: float = 0.0       # relative Größenänderung 1/s (Annäherung > 0)
    hits: int = 1
    clip: tuple = (False, False, False, False)  # links, oben, rechts, unten am Bildrand abgeschnitten
    # Breite der letzten Detektion (px), ungeglättet; 0 = links oder rechts am Bildrand abgeschnitten. w selbst ist
    # geglättet und zwischen Detektionen vorhergesagt: Sein Nachziehen sah für pet wie Annäherung aus (2026-11-13)
    w_meas: float = 0.0

    @property
    def box(self):
        return (self.cx - self.w / 2, self.cy - self.h / 2, self.cx + self.w / 2, self.cy + self.h / 2)

    def set_box(self, b) -> None:
        self.cx, self.cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        self.w, self.h = b[2] - b[0], b[3] - b[1]

    def vis_box(self, W: int, H: int):
        x1, y1, x2, y2 = self.box
        return (max(0.0, x1), max(0.0, y1), min(float(W), x2), min(float(H), y2))

    def lost_s(self, now: float) -> float:
        return now - self.t_seen


class Tracker:
    ALPHA = 0.6   # Positions-Gewicht der Messung
    BETA = 0.15

    def __init__(self, cfg: TrackerCfg, frame_size: tuple[int, int]):
        self.cfg = cfg
        self.W, self.H = frame_size
        self.tracks: list[Track] = []

    def predict(self, A: np.ndarray, dt: float) -> None:
        """Kamerabewegung + Objektbewegung auf alle Tracks anwenden."""
        for tr in self.tracks:
            b = warp_box(A, tr.box)
            tr.set_box(b)
            if dt > 0:
                tr.cx += tr.vx * dt
                tr.cy += tr.vy * dt
                k = float(np.exp(np.clip(tr.vs * dt, -0.2, 0.2)))
                tr.w *= k
                tr.h *= k
            # Geschwindigkeiten langsam abklingen lassen (ohne Messung)
            tr.vx *= 0.97
            tr.vy *= 0.97
            tr.vs *= 0.97

    def update(self, dets, now: float, lag: float = 0.0) -> None:
        """dets: Detection-Liste, bereits in Koordinaten des aktuellen Frames.
        lag: Alter der Detektionen in s (Objektbewegung seitdem wird ergänzt)."""
        pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, d in enumerate(dets):
                if d.cls != tr.cls:
                    continue
                db = self._shift(d.box, tr, lag)
                s = iou(tr.vis_box(self.W, self.H), db)
                if s < self.cfg.iou_match:
                    # Fallback bei schneller Bewegung: Zentrumsabstand relativ zur Größe
                    dcx, dcy = (db[0] + db[2]) / 2 - tr.cx, (db[1] + db[3]) / 2 - tr.cy
                    rel = np.hypot(dcx, dcy) / max(tr.h, tr.w, 1.0)
                    dh = abs((db[3] - db[1]) - tr.h) / max(tr.h, 1.0)
                    if rel < 0.5 and dh < 0.5:
                        s = 0.01 + 0.2 * (0.5 - rel)
                    else:
                        continue
                pairs.append((s, ti, di))
        pairs.sort(reverse=True)
        used_t, used_d = set(), set()
        for s, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            self._correct(self.tracks[ti], dets[di], now, lag)
        for di, d in enumerate(dets):
            if di not in used_d:
                tr = Track(d.cls, 0, 0, 0, 0, d.conf, now)
                tr.set_box(d.box)
                tr.clip = self._clip_flags(d.box)
                tr.w_meas = 0.0 if (tr.clip[0] or tr.clip[2]) else tr.w
                self.tracks.append(tr)
        self.prune(now)

    def prune(self, now: float) -> None:
        keep = []
        for tr in self.tracks:
            if tr.lost_s(now) > self.cfg.max_lost_s:
                continue
            if tr.cx < -tr.w or tr.cx > self.W + tr.w or tr.cy < -tr.h or tr.cy > self.H + tr.h:
                continue
            if tr.w < 2 or tr.h < 2:
                continue
            keep.append(tr)
        self.tracks = keep

    def confirmed(self, now: float, cls: str | None = None) -> list[Track]:
        return [t for t in self.tracks
                if t.hits >= self.cfg.min_hits and (cls is None or t.cls == cls)]

    def get(self, tid: int | None) -> Track | None:
        for t in self.tracks:
            if t.id == tid:
                return t
        return None

    # ------------------------------------------------------------------
    def _clip_flags(self, b):
        mx, my = 0.03 * self.W, 0.03 * self.H   # Randzone: durch Wackeln unsicher
        return (b[0] <= mx, b[1] <= my, b[2] >= self.W - mx, b[3] >= self.H - my)

    @staticmethod
    def _shift(box, tr: Track, lag: float):
        dx, dy = tr.vx * lag, tr.vy * lag
        return (box[0] + dx, box[1] + dy, box[2] + dx, box[3] + dy)

    def _correct(self, tr: Track, d, now: float, lag: float) -> None:
        flags = self._clip_flags(d.box)
        b = list(self._shift(d.box, tr, lag))
        pb = tr.box
        # abgeschnittene Kante: Vorhersage behalten, solange sie jenseits des Rands liegt
        if flags[0]: b[0] = min(b[0], pb[0])
        if flags[1]: b[1] = min(b[1], pb[1])
        if flags[2]: b[2] = max(b[2], pb[2])
        if flags[3]: b[3] = max(b[3], pb[3])
        tr.clip = flags
        mcx, mcy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        mw, mh = b[2] - b[0], b[3] - b[1]
        tr.w_meas = 0.0 if (flags[0] or flags[2]) else mw
        dt = max(0.05, now - tr.t_seen)
        # Skalierungsrate aus einer unbeschnittenen Ausdehnung
        rs = None
        if not (flags[0] or flags[2]) and tr.w > 2 and mw > 2:
            rs = float(np.log(mw / tr.w))
        elif not (flags[1] or flags[3]) and tr.h > 2 and mh > 2:
            rs = float(np.log(mh / tr.h))
        ex, ey = mcx - tr.cx, mcy - tr.cy
        tr.cx += self.ALPHA * ex
        tr.cy += self.ALPHA * ey
        tr.w += self.ALPHA * (mw - tr.w)
        tr.h += self.ALPHA * (mh - tr.h)
        tr.vx += self.BETA * ex / dt
        tr.vy += self.BETA * ey / dt
        if rs is not None:
            tr.vs += self.BETA * rs / dt
        # Plausibilitätsgrenzen
        lim = 2.0 * self.W
        tr.vx = float(np.clip(tr.vx, -lim, lim))
        tr.vy = float(np.clip(tr.vy, -lim, lim))
        tr.vs = float(np.clip(tr.vs, -1.5, 1.5))
        tr.conf = 0.5 * tr.conf + 0.5 * d.conf
        tr.t_seen = now
        tr.hits += 1
