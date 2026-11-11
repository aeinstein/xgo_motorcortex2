"""Kurzzeit-Hinderniskarte im Roboter-KS (Ursprung Radachse) + Bahnprüfung.

Die Kamera sieht nur ±31°. Beim Drehen um eine Ecke verschwindet das
Hindernis aus dem Bild, ist aber noch da. Deshalb werden die Bodengrenzen aus
der Freiraumschätzung als Punkte (x vorn, y links, Meter) gespeichert, mit der
Eigenbewegung (Kurs aus Ego-Motion, Strecke aus Fahrbefehl) mitgeführt und
nach einigen Sekunden vergessen.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np

from .freespace import FreeSpace
from .geometry import CamGeometry


class LocalMap:
    def __init__(self, geo: CamGeometry, fs_width: int, fs_height: int,
                 max_age_s: float = 6.0, max_range_m: float = 2.5, radius_m: float = 0.18,
                 confirm: int = 1, confirm_radius_m: float = 0.08):
        self.geo = geo
        self.fw, self.fh = fs_width, fs_height
        self.max_age = max_age_s
        self.max_range = max_range_m
        self.radius = radius_m
        self.confirm = max(1, int(confirm))
        self.confirm_radius = confirm_radius_m
        self.pts = np.zeros((0, 2))
        self._conf = np.zeros(0, np.int8)   # je Punkt: -1 ungeprüft, 0 unbestätigt, 1 bestätigt
        self._conf_t = None                 # gilt für dieses self.t (neu bei jeder Änderung der Punktmenge)
        self.t = np.zeros(0)
        self.obs = np.zeros(0, np.int64)     # Nummer der Beobachtung, aus der der Punkt stammt
        self._obs_n = 0

    def move(self, ds: float, dyaw: float) -> None:
        """Roboter fuhr ds Meter vor und drehte um dyaw (rad, links +)."""
        if not len(self.pts):
            return
        p = self.pts - np.array([ds, 0.0])
        c, s = math.cos(-dyaw), math.sin(-dyaw)
        self.pts = p @ np.array([[c, s], [-s, c]])

    def add_freespace(self, fs: FreeSpace, now: float, pitch_dev: float) -> None:
        """Bodengrenzen eintragen. Spalten mit NaN gelten als verdeckt (unbekannt)."""
        if fs is None or fs.columns is None:
            return
        sc = self.geo.W / self.fw
        span = self.fh - fs.horizon_row
        cols = np.asarray(fs.columns, float)
        # Entfernung, ab der die Spalte überhaupt etwas weiß (Linsenabschattung: darunter
        # ist die Farbe nicht messbar). Spalten ohne messbaren Boden gelten als verdeckt.
        known_from = None
        if fs.known_from is not None:
            kf = np.asarray(fs.known_from, float)
            cols = np.where(kf >= 0.98, np.nan, cols)
            known_from = self.geo.ground_dist_np((self.fh - kf * span) * sc, pitch_dev)
        # Entfernung der Bodengrenze je Spalte (NaN: am/über dem Horizont oder verdeckt)
        dist = self.geo.ground_dist_np((self.fh - cols * span) * sc, pitch_dev)
        self._clear_seen(cols, dist, sc, pitch_dev, known_from)
        idx = np.arange(0, len(cols), 2)
        v, d = cols[idx], dist[idx]
        with np.errstate(invalid="ignore"):
            ok = (v < 0.98) & np.isfinite(d) & (d <= self.max_range)
        if ok.any():
            i, d = idx[ok], d[ok]
            x_full = (i + 0.5) * sc
            y = d * (self.geo.cx - x_full) / self.geo.f       # d * tan(Peilung)
            # Ursprung = Radachse: d zählt ab Kamerafußpunkt, der cam_fwd davor liegt
            new_pts = self._thin(np.column_stack([d + self.geo.cam_fwd, y]))
            self.pts = np.vstack([self.pts, new_pts])
            self.t = np.concatenate([self.t, np.full(len(new_pts), now)])
            self.obs = np.concatenate([self.obs, np.full(len(new_pts), self._obs_n, np.int64)])
        self._obs_n += 1
        self._prune(now)

    def _clear_seen(self, cols: np.ndarray, dist: np.ndarray, sc: float, pitch_dev: float,
                    known_from: np.ndarray | None = None) -> None:
        """Alte Punkte entfernen, wo die neue Beobachtung freien Boden zeigt.

        Ohne Freiräumen bleiben falsch (zu nah) geschätzte Grenzen bis max_age liegen
        und rücken beim Vorwärtsfahren in die Bahn. Ein Punkt wird nur gelöscht, wenn er
        im Sichtfeld liegt, jenseits der unteren Bildkante, und deutlich vor der jetzt
        gemessenen Grenze dieser Spalte (Toleranz wächst mit der Entfernung, weil die
        Bodenentfernung bei Nickfehlern quadratisch schlechter wird). known_from: je
        Spalte die Entfernung, ab der der Boden messbar ist; davor wird nichts gelöscht."""
        if not len(self.pts):
            return
        n = len(cols)
        known = np.isfinite(cols)                       # NaN = verdeckt -> nichts löschen
        with np.errstate(invalid="ignore"):
            free = known & ((cols >= 0.98) | ~np.isfinite(dist))   # frei bis zum Horizont
        rng = np.where(free, self.max_range, np.minimum(dist, self.max_range))
        rng[~known] = np.nan
        near = self.geo.ground_dist(self.geo.H - 1, pitch_dev) or 0.0
        # Spalte und Entfernungen gelten ab Kamera, die Punkte ab Radachse
        x, y = self.pts[:, 0] - self.geo.cam_fwd, self.pts[:, 1]
        ahead = x > 1e-3
        col = np.full(len(x), -1)
        xf = self.geo.cx - self.geo.f * y[ahead] / x[ahead]
        col[ahead] = np.floor(xf / sc).astype(int)
        inside = ahead & (col >= 1) & (col < n - 1)   # Randspalten auslassen
        r = np.full(len(x), np.nan)
        r[inside] = rng[col[inside]]
        lo = np.full(len(x), near)
        if known_from is not None:
            kf = np.where(np.isfinite(known_from), known_from, np.inf)   # NaN: nie messbar
            lo[inside] = np.maximum(near, kf[col[inside]])
        with np.errstate(invalid="ignore"):
            drop = inside & np.isfinite(r) & (x > lo) & (x < r - (0.1 + 0.15 * r))
        if drop.any():
            self.pts, self.t, self.obs = self.pts[~drop], self.t[~drop], self.obs[~drop]

    def _prune(self, now: float) -> None:
        if not len(self.pts):
            return
        keep = (now - self.t < self.max_age) & (np.hypot(self.pts[:, 0], self.pts[:, 1]) < self.max_range * 1.5)
        if len(keep) > 3000:
            keep[: len(keep) - 3000] = False
        self.pts, self.t, self.obs = self.pts[keep], self.t[keep], self.obs[keep]

    @staticmethod
    def _distinct(near: np.ndarray, obs: np.ndarray) -> np.ndarray:
        """Je Zeile von near: Anzahl verschiedener obs-Werte unter den markierten Spalten."""
        if not near.shape[1]:
            return np.zeros(near.shape[0], np.int32)
        _, oid = np.unique(obs, return_inverse=True)
        onehot = np.zeros((len(obs), oid.max() + 1), np.float32)
        onehot[np.arange(len(obs)), oid] = 1.0
        return ((near.astype(np.float32) @ onehot) > 0).sum(axis=1).astype(np.int32)

    MIN_SPACING = 0.02   # m zwischen benachbarten Punkten einer Beobachtung

    @classmethod
    def _thin(cls, pts: np.ndarray) -> np.ndarray:
        """Nahe Grenzpunkte liegen nur Millimeter auseinander (jede 2. Bildspalte); für den
        Sicherheitsradius genügt ein Abstand von MIN_SPACING. Reihenfolge = Bildspalten."""
        if len(pts) < 2:
            return pts
        # Python-Floats statt numpy-Einzelzugriffe (gleiche Schleife, ~10x schneller)
        xs, ys = pts[:, 0].tolist(), pts[:, 1].tolist()
        lim = cls.MIN_SPACING ** 2
        keep = [0]
        qx, qy = xs[0], ys[0]
        for i in range(1, len(xs)):
            x, y = xs[i], ys[i]
            if (x - qx) ** 2 + (y - qy) ** 2 >= lim:
                keep.append(i)
                qx, qy = x, y
        return pts[keep]

    def confirmed_points(self, reach: float = math.inf) -> np.ndarray:
        """Punkte innerhalb reach, in deren Nähe Beobachtungen aus mindestens `confirm`
        Bildern liegen. Fehlalarme des Bodenmodells (Reflexe, Fugen) flackern, echte
        Hindernisse nicht. Exakt gezählt, aber nur im Nahbereich; die Punktdichte begrenzt
        add_freespace (MIN_SPACING), sonst wüchse der Aufwand quadratisch."""
        if not len(self.pts):
            return self.pts
        r = np.hypot(self.pts[:, 0], self.pts[:, 1])
        cand = r < reach
        if self.confirm <= 1 or not cand.any():
            return self.pts[cand]
        # Das Ergebnis je Punkt ändert sich nur mit der Punktmenge: move() verschiebt alle gleich. Der
        # Regelschritt fragt mehrmals je Takt, am RiderPi kostete das bei ~2000 Punkten ~27 ms CPU.
        if self._conf_t is not self.t or len(self._conf) != len(self.pts):
            self._conf_t, self._conf = self.t, np.full(len(self.pts), -1, np.int8)
        need = cand & (self._conf < 0)
        if need.any():
            nb = r < r[need].max() + self.confirm_radius
            C, P, O = self.pts[need], self.pts[nb], self.obs[nb]
            near = ((C[:, None, :] - P[None, :, :]) ** 2).sum(-1) < self.confirm_radius ** 2
            self._conf[need] = self._distinct(near, O) >= self.confirm
        return self.pts[cand & (self._conf == 1)]

    def confirmed_later(self, reach: float):
        """Funktion, die confirmed_points(reach) für den jetzigen Stand rechnet, ohne Zwischenspeicher: für den
        Planer-Thread. Die Arrays werden nie verändert, nur ersetzt, und die Rechnung bremst die Hauptschleife nicht."""
        pts, obs, confirm, rad = self.pts, self.obs, self.confirm, self.confirm_radius

        def run() -> np.ndarray:
            if not len(pts):
                return pts
            r = np.hypot(pts[:, 0], pts[:, 1])
            cand = r < reach
            if confirm <= 1 or not cand.any():
                return pts[cand]
            nb = r < reach + rad
            C, P = pts[cand], pts[nb]
            near = ((C[:, None, :] - P[None, :, :]) ** 2).sum(-1) < rad ** 2
            return C[LocalMap._distinct(near, obs[nb]) >= confirm]
        return run

    def arc_clearance(self, v_mps: float, w_rads: float, horizon_s: float = 1.0, steps: int = 10,
                      radius: float | None = None, hit_out: list | None = None) -> float:
        """Freie Strecke (m) entlang des Bogens, bevor ein (bestätigter) Punkt näher als radius kommt.
        Punkte, die schon jetzt innerhalb von radius liegen (z. B. Schrankseite neben dem Rider),
        blockieren nur, wenn die Bahn ihnen noch näher kommt – sonst käme der Rider nie wieder weg.
        radius: statt des eigenen (Engstelle, [map] radius_tight_m). hit_out: bekommt die Punkte (N x 2,
        Rider-KS), die die Bahn sperren."""
        if abs(v_mps) < 1e-3:
            return math.inf
        radius = self.radius if radius is None else radius
        pts = self.confirmed_points(abs(v_mps) * horizon_s + radius)
        if not len(pts):
            return math.inf
        lim = np.minimum(radius, np.hypot(pts[:, 0], pts[:, 1]) - ESCAPE_TOL_M)
        x = y = th = 0.0
        dt = horizon_s / steps
        s = 0.0
        for _ in range(steps):
            th += w_rads * dt
            x += v_mps * math.cos(th) * dt
            y += v_mps * math.sin(th) * dt
            s += abs(v_mps) * dt
            near = np.hypot(pts[:, 0] - x, pts[:, 1] - y) < lim
            if near.any():
                if hit_out is not None:
                    hit_out.append(pts[near])
                return s
        return math.inf

    def gap_center(self, ahead_m: float = 0.45, side_m: float = 0.30, max_width_m: float = 0.5) -> tuple | None:
        """Mitte (x, y im Rider-KS) zwischen dem nächsten bestätigten Punkt links und rechts voraus, None wenn
        eine Seite fehlt oder die Lücke breiter als max_width_m ist. Für die Durchfahrt enger Stellen: Die
        Merkkarte misst die Kanten jetzt, die Raumkarte liegt dort um Zentimeter daneben."""
        p = self.confirmed_points(ahead_m + side_m)
        if not len(p):
            return None
        p = p[(p[:, 0] > 0.0) & (p[:, 0] < ahead_m) & (np.abs(p[:, 1]) < side_m)]
        le, ri = p[p[:, 1] > 0], p[p[:, 1] <= 0]
        if not len(le) or not len(ri):
            return None
        a, b = le[np.argmin(le[:, 1])], ri[np.argmax(ri[:, 1])]
        if a[1] - b[1] > max_width_m:
            return None
        return float((a[0] + b[0]) / 2), float((a[1] + b[1]) / 2)

    def side_min(self) -> tuple[float, float]:
        """Nächstes Hindernis links/rechts (seitlich hinter dem Sichtfeld)."""
        if not len(self.pts):
            return math.inf, math.inf
        d = np.hypot(self.pts[:, 0], self.pts[:, 1])
        left = d[self.pts[:, 1] > 0]
        right = d[self.pts[:, 1] < 0]
        return (left.min() if len(left) else math.inf, right.min() if len(right) else math.inf)


def blocking_boxes(tracks, target_id, now: float, no_block_cls=()) -> list:
    """Boxen, die als Hindernis ins Freiraum-Ergebnis kommen: frisch gesehen, nicht das Ziel, keine große Möbelklasse."""
    return [t.box for t in tracks if t.id != target_id and t.lost_s(now) < 0.5 and t.cls not in no_block_cls]


MOVABLE = {"person", "dog", "cat", "bird", "horse", "sports ball", "bicycle", "car"}
ESCAPE_TOL_M = 0.01   # arc_clearance: Punkt im Radius blockiert erst, wenn die Bahn ihm >1 cm näher kommt


def mask_movable(free: FreeSpace, objects, W: int, H: int) -> FreeSpace:
    """Für die (statische) Karte: Spalten unter beweglichen Objekten ausblenden,
    sonst hinterlässt z. B. eine verfolgte Person eine Geisterspur auf dem Weg.
    NaN = verdeckt: dort wird weder eingetragen noch freigeräumt.
    objects: Folge von (Klasse, (x1, y1, x2, y2)) in Vollbild-Pixeln."""
    mov = [box for cls, box in objects if cls in MOVABLE]
    if not mov:
        return free
    cols = free.columns.copy()
    sc = W / len(cols)
    for x1, _, x2, _ in mov:
        x1, x2 = max(0.0, x1), min(float(W), x2)
        cols[max(0, int(x1 / sc) - 1): int(x2 / sc) + 2] = np.nan
    return dataclasses.replace(free, columns=cols)
