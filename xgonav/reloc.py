"""Kartenabgleich: Pose in der Raumkarte aus der Bodengrenze der letzten Sekunden.

Der Rider hat weder Lidar noch Kompass. Was er hat, ist die Bodengrenze je Bildspalte: ein kurzer
(bis 1,25 m), schmaler (92°) und verrauschter Scan. Über einige Sekunden gesammelt (ScanPuffer,
Roboter-KS) wird daraus ein Stück Umriss, das auf die Raumkarte gelegt wird:

  abgleich_lokal   kleines Fenster um die Odometrie-Pose (Drift nachziehen)
  suche_global     ganze Karte, alle Richtungen (Startpose nach Transport)

Gerechnet wird mit Rohpunkten, nicht mit einer zweiten Karte: Mit grid.confirm_dr_m belegt eine
Drehung auf der Stelle keine Zelle, die Punkte sind trotzdem da. Die Karte wird einmal zu einem
Feld (Nähe zur nächsten Wand) gemacht; je Richtung liefert eine Korrelation (cv2.matchTemplate)
die Wertung aller Verschiebungen auf einmal. Treffer auf bekannt freiem Boden und freie Strahlen
durch Wände kosten.

Beide melden lieber "unsicher" als eine falsche Lage: zu wenig Wand, passt nicht, gleitet (Flur),
mehrdeutig (gleich gute zweite Lage). Rechenzeit: nichts davon läuft in der Bildschleife.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import RelocCfg

NMS_M, NMS_DEG = 0.3, 10.0        # näher beieinander gilt als dieselbe Lage
SLIDE_DROP = 0.10                 # Lagen bis so viel unter der besten zählen als "fast gleich gut"
PRO_WINKEL = 20                   # globale Suche: so viele beste Zellen je Richtung merken


@dataclass
class Scan:
    """Punkte im Roboter-KS (Radachse, x vorn, y links, m)."""
    hits: np.ndarray                  # (N, 2) Bodengrenze
    w: np.ndarray                     # (N,) Gewicht
    free: np.ndarray                  # (M, 2) Proben auf den Freistrahlen

    @property
    def n(self) -> int:
        return len(self.hits)


@dataclass
class Ergebnis:
    ok: bool
    grund: str                        # "" oder warum unsicher
    pose: tuple                       # (x, y, th) im Karten-KS; bei ok False die beste gefundene
    guete: float = 0.0                # Wertung der Lage (1 = alle Treffer auf einer Wand)
    anteil: float = 0.0               # Anteil der Treffer an einer Wand der Karte (von denen im Bekannten)
    n: int = 0                        # ausgedünnte Treffer
    n_bekannt: int = 0                # davon in schon kartiertem Gebiet
    streu_m: float = 0.0              # Streuung der fast gleich guten Lagen (größte Achse)
    zweit: float = 0.0                # Wertung der zweitbesten Lage / beste (globale Suche)
    teil: bool = False                # nur quer zur Gleitrichtung korrigiert (Flur)
    kandidaten: list = field(default_factory=list)   # globale Suche: (Wertung, x, y, Grad) verfeinert


def scan_punkte(gm, fs, pitch_dev: float, cfg: RelocCfg):
    """Eine Beobachtung des Bodenmodells als Punkte im Roboter-KS: Treffer, Gewichte, Freiproben.
    Dieselben Strahlen wie GridMap.add_freespace (gleiche Grenzen)."""
    _, bear, dist, free_len, start, ok, hit = gm._rays(fs, pitch_dev)
    cb, sb = np.cos(bear), np.sin(bear)
    off = gm.geo.cam_fwd
    d = dist[hit]
    hits = np.column_stack([off + d * cb[hit], d * sb[hit]])
    w = 1.0 / (1.0 + (d / cfg.far_m) ** 2)
    if ok.any():
        a, b = start[ok], free_len[ok]
        steps = np.floor((b - a) / cfg.free_step_m).astype(np.int64) + 1
        ray = np.repeat(np.arange(len(a)), steps)
        k = np.arange(steps.sum()) - np.repeat(np.cumsum(steps) - steps, steps)
        s = a[ray] + k * cfg.free_step_m
        free = np.column_stack([off + s * cb[ok][ray], s * sb[ok][ray]])
    else:
        free = np.zeros((0, 2))
    return hits, w, free


def _in_ks(pts: np.ndarray, von: tuple, nach: tuple) -> np.ndarray:
    """Punkte aus dem Roboter-KS der Pose von in das der Pose nach."""
    if not len(pts):
        return pts
    c, s = math.cos(von[2]), math.sin(von[2])
    wx = von[0] + c * pts[:, 0] - s * pts[:, 1] - nach[0]
    wy = von[1] + s * pts[:, 0] + c * pts[:, 1] - nach[1]
    c, s = math.cos(nach[2]), math.sin(nach[2])
    return np.column_stack([c * wx + s * wy, -s * wx + c * wy])


def _duenn(pts: np.ndarray, res: float, w: np.ndarray | None = None):
    """Je Rasterzelle ein Punkt (Mittel der Lage und des Gewichts): oft gesehene Stellen zählen nicht mehrfach."""
    if not len(pts):
        return pts, w
    q = np.floor(pts / res).astype(np.int64)
    q -= q.min(axis=0)
    breite = int(q[:, 1].max()) + 1
    if int(q[:, 0].max()) < (1 << 62) // breite:
        # Zelle als eine Zahl: gleiche Gruppen in gleicher Reihenfolge wie zeilenweise (axis=0), aber das
        # Sortieren kostet einen Bruchteil (PC 2026-11-12, bis 6000 Punkte: 1,6 -> 0,2 ms im Mittel)
        _, inv, cnt = np.unique(q[:, 0] * breite + q[:, 1], return_inverse=True, return_counts=True)
    else:
        _, inv, cnt = np.unique(q, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    out = np.column_stack([np.bincount(inv, pts[:, 0]) / cnt, np.bincount(inv, pts[:, 1]) / cnt])
    return out, (np.bincount(inv, w) / cnt if w is not None else None)


class ScanPuffer:
    """Beobachtungen der letzten Sekunden mit der Pose ihrer Aufnahme. Die Posen dürfen driften:
    Gebraucht wird nur ihre Lage zueinander über das kurze Fenster."""

    def __init__(self, cfg: RelocCfg, window_s: float | None = None, window_m: float | None = None):
        self.cfg = cfg
        self.window_s = cfg.window_s if window_s is None else window_s
        self.window_m = cfg.window_m if window_m is None else window_m
        self._e: deque = deque()          # (t, pose, hits, w, free, Weg seit Beginn)
        self._weg = 0.0

    def clear(self) -> None:
        self._e.clear()
        self._weg = 0.0

    def __len__(self) -> int:
        return len(self._e)

    def add(self, t: float, pose: tuple, hits: np.ndarray, w: np.ndarray, free: np.ndarray) -> None:
        if self._e:
            p = self._e[-1][1]
            self._weg += math.hypot(pose[0] - p[0], pose[1] - p[1])
        self._e.append((t, tuple(pose), hits, w, free, self._weg))
        while len(self._e) > 1 and (t - self._e[0][0] > self.window_s or self._weg - self._e[0][5] > self.window_m):
            self._e.popleft()

    def transform(self, k: tuple) -> None:
        """Alle Posen starr verschieben (nach einer Korrektur der Kartenpose): Pose' = k ∘ Pose."""
        self._e = deque((t, anwenden(k, p), h, w, f, s) for t, p, h, w, f, s in self._e)

    def spanne(self) -> tuple[float, float]:
        """Zeit (s) und Drehung (rad, Summe der Beträge) über den Puffer."""
        if len(self._e) < 2:
            return 0.0, 0.0
        th = np.array([e[1][2] for e in self._e])
        return self._e[-1][0] - self._e[0][0], float(np.abs(np.diff(th)).sum())

    def scan(self, pose: tuple) -> Scan:
        """Alles im Roboter-KS der Pose (gleiches KS wie die Posen des Puffers), ausgedünnt."""
        if not self._e:
            return Scan(np.zeros((0, 2)), np.zeros(0), np.zeros((0, 2)))
        hits = np.vstack([_in_ks(h, p, pose) for _, p, h, _, _, _ in self._e])
        w = np.concatenate([e[3] for e in self._e])
        free = np.vstack([_in_ks(f, p, pose) for _, p, _, _, f, _ in self._e])
        hits, w = _duenn(hits, self.cfg.hit_res_m, w)
        free, _ = _duenn(free, self.cfg.free_res_m)
        return Scan(hits, w, free)


class Feld:
    """Referenz aus einem Stand der Raumkarte (Log-Odds lo): treffer = Nähe zur nächsten Wand (1 an
    der Wand, fällt mit sigma ab), auf bekannt freiem Boden fern jeder Wand negativ; kosten = was
    eine Freiprobe dort kostet (belegt 1, unbekannt unknown_weight, frei 0)."""

    def __init__(self, lo: np.ndarray, res: float, l_show: float, cfg: RelocCfg):
        self.res, self.n, self.cfg = res, lo.shape[0], cfg
        occ = lo > l_show
        frei = lo < -l_show
        self.n_occ = int(occ.sum())
        if self.n_occ:
            d = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3) * res
        else:
            d = np.full(lo.shape, 1e3, np.float32)
        self.wand = np.exp(-(d * d) / (2 * cfg.sigma_m ** 2)).astype(np.float32)
        self.treffer = self.wand - np.float32(cfg.miss_weight) * (frei & (d > 3 * cfg.sigma_m))
        self.kosten = np.where(occ, 1.0, np.where(frei, 0.0, cfg.unknown_weight)).astype(np.float32)
        self.frei = frei
        self.bekannt = frei | (self.wand >= 0.5)              # hier weiß die Karte, ob eine Wand da ist
        self._grob: dict = {}

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor(y / self.res + self.n / 2)), int(math.floor(x / self.res + self.n / 2))

    def grob(self, f: int):
        """Auf f×f Zellen vergröbert: treffer als Maximum (keine Lage wird übersehen), kosten als Mittel,
        dazu wo der Rider stehen kann (bekannt freier Boden, eine Zelle Rand)."""
        if f not in self._grob:
            m = self.n // f

            def block(a):
                return a[:m * f, :m * f].reshape(m, f, m, f)
            t = block(self.treffer).max(axis=(1, 3))
            k = block(self.kosten).mean(axis=(1, 3)).astype(np.float32)
            steh = cv2.dilate(block(self.frei).any(axis=(1, 3)).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            self._grob[f] = (np.ascontiguousarray(t), k, steh)
        return self._grob[f]


def _kern(pts: np.ndarray, w: np.ndarray, res: float, th: float, off: tuple = (0.0, 0.0)):
    """Punkte um th gedreht als kleines Zählbild (Gewichtssumme je Zelle) samt Lage seiner Ecke
    relativ zur Roboterzelle. off = Lage des Roboters in seiner Zelle relativ zur Zellmitte (m)."""
    c, s = math.cos(th), math.sin(th)
    ck = np.floor((c * pts[:, 0] - s * pts[:, 1] + off[0]) / res + 0.5).astype(np.int64)
    rk = np.floor((s * pts[:, 0] + c * pts[:, 1] + off[1]) / res + 0.5).astype(np.int64)
    r0, c0 = int(rk.min()), int(ck.min())
    h, b = int(rk.max()) - r0 + 1, int(ck.max()) - c0 + 1
    k = np.bincount((rk - r0) * b + (ck - c0), weights=w, minlength=h * b).reshape(h, b).astype(np.float32)
    return k, r0, c0


def _korrel(bild: np.ndarray, rand: float, k: np.ndarray, kr0: int, kc0: int,
            r_lo: int, r_hi: int, c_lo: int, c_hi: int) -> np.ndarray:
    """Summe Kern x Bild für alle Roboterzellen r_lo..r_hi, c_lo..c_hi (einschließlich). Außerhalb
    des Bildes gilt der Wert rand."""
    h, b = k.shape
    a0, a1, b0, b1 = r_lo + kr0, r_hi + kr0 + h, c_lo + kc0, c_hi + kc0 + b
    aus = np.full((a1 - a0, b1 - b0), rand, np.float32)
    s0, s1, t0, t1 = max(a0, 0), min(a1, bild.shape[0]), max(b0, 0), min(b1, bild.shape[1])
    if s0 < s1 and t0 < t1:
        aus[s0 - a0:s1 - a0, t0 - b0:t1 - b0] = bild[s0:s1, t0:t1]
    return cv2.matchTemplate(aus, k, cv2.TM_CCORR)


def _wertung(treffer, kosten, rand_k: float, res: float, scan: Scan, th: float, off: tuple,
             fenster: tuple, lam: float) -> np.ndarray:
    """Wertung je Roboterzelle des Fensters (r_lo, r_hi, c_lo, c_hi) für die Richtung th:
    gewichtetes Mittel des Felds an den Treffern minus lam x mittlere Kosten der Freiproben."""
    k, r0, c0 = _kern(scan.hits, scan.w, res, th, off)
    s = _korrel(treffer, 0.0, k, r0, c0, *fenster) / float(scan.w.sum())
    if lam > 0 and len(scan.free):
        k, r0, c0 = _kern(scan.free, np.ones(len(scan.free)), res, th, off)
        s -= lam * _korrel(kosten, rand_k, k, r0, c0, *fenster) / len(scan.free)
    return s


def _parabel(a: float, b: float, c: float) -> float:
    """Lage des Scheitels durch drei Werte an -1, 0, +1 (b ist der größte), in Schritten."""
    d = a - 2 * b + c
    return 0.0 if d >= -1e-12 else max(-0.5, min(0.5, 0.5 * (a - c) / d))


def _anteil(feld: Feld, scan: Scan, pose: tuple) -> tuple[float, int]:
    """Treffer in der Lage pose: Anteil (nach Gewicht) an einer Wand der Karte unter denen, die in
    kartiertem Gebiet liegen, und deren Zahl. Treffer im Unbekannten sagen nichts: Beim Erkunden
    sieht der Rider ständig Neues, das spricht nicht gegen die Lage."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x = pose[0] + c * scan.hits[:, 0] - s * scan.hits[:, 1]
    y = pose[1] + s * scan.hits[:, 0] + c * scan.hits[:, 1]
    r = np.floor(y / feld.res + feld.n / 2).astype(np.int64)
    q = np.floor(x / feld.res + feld.n / 2).astype(np.int64)
    ok = (r >= 0) & (r < feld.n) & (q >= 0) & (q < feld.n)
    an = np.zeros(len(x), bool)
    bek = np.zeros(len(x), bool)
    an[ok] = feld.wand[r[ok], q[ok]] >= 0.5
    bek[ok] = feld.bekannt[r[ok], q[ok]]
    return float((scan.w * an).sum() / max(1e-9, (scan.w * bek).sum())), int(bek.sum())


def abgleich_lokal(feld: Feld, scan: Scan, pose: tuple, cfg: RelocCfg, search_m: float | None = None,
                   search_deg: float | None = None, step_deg: float | None = None,
                   min_bekannt: float = 0.0) -> Ergebnis:
    """Beste Lage im Fenster ±search_m, ±search_deg um pose. Gleitet der Abgleich in einer Richtung
    (Flur: nur eine Wandrichtung in Sicht), wird nur quer dazu und im Kurs korrigiert (teil).
    min_bekannt: so viel vom Scan muss in kartiertem Gebiet liegen (globale Suche; beim laufenden
    Abgleich genügen min_points Treffer im Bekannten)."""
    search_m = cfg.search_m if search_m is None else search_m
    search_deg = cfg.search_deg if search_deg is None else search_deg
    step = math.radians(cfg.step_deg if step_deg is None else step_deg)
    if scan.n < cfg.min_points:
        return Ergebnis(False, "zu wenig Wand", tuple(pose), n=scan.n)
    if not feld.n_occ:
        return Ergebnis(False, "Karte leer", tuple(pose), n=scan.n)
    res = feld.res
    rg, cg = feld.cell(pose[0], pose[1])
    # Lage des Roboters in seiner Zelle: die Kandidaten liegen dann genau bei pose + k x res
    off = (pose[0] - (cg + 0.5 - feld.n / 2) * res, pose[1] - (rg + 0.5 - feld.n / 2) * res)
    kz = max(1, int(math.ceil(search_m / res)))
    ka = max(1, int(round(math.radians(search_deg) / step)))
    fenster = (rg - kz, rg + kz, cg - kz, cg + kz)
    winkel = pose[2] + np.arange(-ka, ka + 1) * step
    lam = cfg.free_weight
    v = np.stack([_wertung(feld.treffer, feld.kosten, cfg.unknown_weight, res, scan, th, off, fenster, lam)
                  for th in winkel])
    ia, ir, ic = (int(i) for i in np.unravel_index(int(np.argmax(v)), v.shape))
    best = float(v[ia, ir, ic])
    da = _parabel(v[ia - 1, ir, ic], best, v[ia + 1, ir, ic]) if 0 < ia < len(winkel) - 1 else 0.0
    dr = _parabel(v[ia, ir - 1, ic], best, v[ia, ir + 1, ic]) if 0 < ir < 2 * kz else 0.0
    dc = _parabel(v[ia, ir, ic - 1], best, v[ia, ir, ic + 1]) if 0 < ic < 2 * kz else 0.0
    dx, dy = (ic - kz + dc) * res, (ir - kz + dr) * res
    th = float(winkel[ia] + da * step)
    gefunden = (pose[0] + dx, pose[1] + dy, th)
    anteil, n_bek = _anteil(feld, scan, gefunden)
    e = Ergebnis(False, "", gefunden, guete=best, anteil=anteil, n=scan.n, n_bekannt=n_bek)
    if n_bek < max(cfg.min_points, min_bekannt * scan.n):
        e.grund = "zu wenig Bekanntes"
        return e
    if best <= 0 or anteil < cfg.min_hit_frac:
        e.grund = "passt nicht"
        return e
    # Streuung der fast gleich guten Lagen (über alle Richtungen): groß heißt, die Lage ist dort unbestimmt
    p = v.max(axis=0)
    rr, cc = np.nonzero(p >= best - SLIDE_DROP * best)
    if len(rr) > 2:
        ev, evec = np.linalg.eigh(np.cov(np.vstack([cc, rr]).astype(float) * res))
        e.streu_m = float(math.sqrt(max(ev[1], 0.0)))
        if e.streu_m > cfg.max_slide_m:
            if math.sqrt(max(ev[0], 0.0)) > cfg.max_slide_m:
                e.grund = "gleitet"
                return e
            e2 = evec[:, 0]                                 # gut bestimmte Richtung
            q = dx * e2[0] + dy * e2[1]                     # nur der Anteil quer zur Gleitrichtung
            e.pose = (pose[0] + q * e2[0], pose[1] + q * e2[1], th)
            e.teil, e.ok = True, True
            return e
    if ir in (0, 2 * kz) or ic in (0, 2 * kz) or ia in (0, len(winkel) - 1):
        e.grund = "am Rand des Suchfensters"
        return e
    e.ok = True
    return e


def _winkel_diff(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def _gleich(a: tuple, b: tuple, weit: float = 1.0) -> bool:
    return (math.hypot(a[0] - b[0], a[1] - b[1]) < weit * NMS_M
            and _winkel_diff(a[2], b[2]) < weit * math.radians(NMS_DEG))


def suche_global(feld: Feld, scan: Scan, cfg: RelocCfg, pause=None) -> Ergebnis:
    """Lage des Scans irgendwo in der Karte: grobes Raster über alle Richtungen, die besten Kandidaten
    fein nachgerechnet. Angenommen wird nur eine eindeutige Lage. pause() wird zwischen den Richtungen
    gerufen (im Thread: time.sleep(0), damit die Hauptschleife drankommt)."""
    leer = (0.0, 0.0, 0.0)
    if scan.n < cfg.min_points:
        return Ergebnis(False, "zu wenig Wand", leer, n=scan.n)
    if not feld.n_occ:
        return Ergebnis(False, "Karte leer", leer, n=scan.n)
    f = max(1, int(round(cfg.global_res_m / feld.res)))
    res = feld.res * f
    treffer, kosten, steh = feld.grob(f)
    rows, cols = np.flatnonzero(steh.any(axis=1)), np.flatnonzero(steh.any(axis=0))
    if not len(rows):
        return Ergebnis(False, "Karte leer", leer, n=scan.n)
    fenster = (int(rows[0]), int(rows[-1]), int(cols[0]), int(cols[-1]))
    steh = steh[fenster[0]:fenster[1] + 1, fenster[2]:fenster[3] + 1]
    m = treffer.shape[0]
    roh = []
    for th in np.arange(0.0, 2 * math.pi, math.radians(cfg.global_step_deg)):
        s = _wertung(treffer, kosten, cfg.unknown_weight, res, scan, th, (0.0, 0.0), fenster, cfg.free_weight)
        s[~steh] = -np.inf
        flach = s.ravel()
        k = min(PRO_WINKEL, flach.size)
        for i in np.argpartition(flach, -k)[-k:]:
            if np.isfinite(flach[i]):
                r, c = divmod(int(i), s.shape[1])
                roh.append((float(flach[i]), (fenster[2] + c + 0.5 - m / 2) * res,
                            (fenster[0] + r + 0.5 - m / 2) * res, float(th)))
        if pause is not None:
            pause()
    roh.sort(reverse=True)
    grob: list = []
    for k in roh:                                           # je Lage nur der beste Kandidat
        if not any(_gleich(k[1:], g[1:]) for g in grob):
            grob.append(k)
            if len(grob) >= cfg.global_top:
                break
    fein: list = []
    for k in grob:
        e = abgleich_lokal(feld, scan, k[1:], cfg, search_m=1.5 * res, search_deg=1.5 * cfg.global_step_deg,
                           min_bekannt=cfg.global_known_frac)
        fein.append(e)
        if pause is not None:
            pause()
    fein.sort(key=lambda e: e.guete, reverse=True)
    # Nachbarn der besten Lage (bis 0,6 m, 20°) sind dieselbe Mulde, keine zweite Lage
    fein = [e for i, e in enumerate(fein) if not any(_gleich(e.pose, g.pose, 2.0) for g in fein[:i])]
    if not fein or fein[0].guete <= 0:
        return Ergebnis(False, "passt nicht", leer, n=scan.n)
    # die beste Lage im vollen Fenster prüfen: erst dort zeigt sich Gleiten
    e = abgleich_lokal(feld, scan, fein[0].pose, cfg, min_bekannt=cfg.global_known_frac)
    e.kandidaten = [(round(g.guete, 3), round(g.pose[0], 2), round(g.pose[1], 2),
                     round(math.degrees(g.pose[2]) % 360, 1)) for g in fein]
    if len(fein) > 1:
        e.zweit = max(0.0, fein[1].guete) / fein[0].guete
    if e.teil or e.grund == "gleitet":
        e.ok, e.teil, e.grund = False, False, "gleitet"
    elif e.ok and e.zweit > cfg.unique_ratio:
        e.ok, e.grund = False, "mehrdeutig"
    return e


# ------------------------------------------------------------------ starre Transformationen
def anwenden(k: tuple, pose: tuple) -> tuple:
    """k ∘ pose: Pose im korrigierten Karten-KS (k = (kx, ky, kth))."""
    c, s = math.cos(k[2]), math.sin(k[2])
    return (k[0] + c * pose[0] - s * pose[1], k[1] + s * pose[0] + c * pose[1], pose[2] + k[2])


def korrektur(von: tuple, nach: tuple) -> tuple:
    """Die starre Transformation k mit k ∘ von = nach."""
    kth = nach[2] - von[2]
    c, s = math.cos(kth), math.sin(kth)
    return (nach[0] - (c * von[0] - s * von[1]), nach[1] - (s * von[0] + c * von[1]), kth)


def schritt(ist: tuple, soll: tuple, cfg: RelocCfg) -> tuple | None:
    """Gedämpfter Schritt von der Pose ist zur abgeglichenen Pose soll: Anteil gain, gedeckelt;
    None im Totband."""
    dx, dy = soll[0] - ist[0], soll[1] - ist[1]
    dth = (soll[2] - ist[2] + math.pi) % (2 * math.pi) - math.pi
    d = math.hypot(dx, dy)
    if d < cfg.deadband_m and abs(dth) < math.radians(cfg.deadband_deg):
        return None
    g = min(cfg.gain, cfg.max_step_m / max(d, 1e-9))
    lim = math.radians(cfg.max_step_deg)
    return (ist[0] + g * dx, ist[1] + g * dy, ist[2] + max(-lim, min(lim, cfg.gain * dth)))
