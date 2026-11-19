"""Abgleich am Startbild: Wo steht der Rider am Ende einer Heimfahrt wirklich, gemessen am Bild vom Start?

Die Koppelnavigation liegt nach einer Erkundung einige cm daneben (Fahrt 20261119-144940: 37 m, Ende 12 cm
neben dem Start, Kurs ~2,5° falsch). Das Bild vom Start sieht dieselben Möbel und Wände; daraus wird die
Lage des Riders gegen den Start gemessen und die Pose der Raumkarte korrigiert.

  Anker      Bild am Startpunkt (Pose = Start, Rider steht), Merkmale (SIFT) einmal berechnet.
  Tiefe      Bilder der geraden Schlussfahrt auf den Start zu (Home: Vorpunkt -> Start). Auf wenigen
             Dezimetern ist die Odometrie millimetergenau: Merkmale zwischen ihnen und dem letzten Bild
             werden trianguliert (3D-Punkte im Karten-KS), Median über mehrere Abstände ([startbild] basen).
  Lage       Merkmale Anker <-> letztes Bild mit 3D-Punkt: Lage des Ankers im Karten-KS. Gesucht sind x, y,
             Kurs und kleine Nick-/Roll-Reste; die Kamerahöhe ist fest (124 mm gemessen bei 90 mm Körperhöhe,
             [camera] cam_h_u8). Robuste Gauß-Newton-Iteration ab der Odometrie-Pose (Tukey, Grenze von
             40 px auf 4 px). Gegenprobe frei in 6 Freiheitsgraden (solvePnPRansac).
  Korrektur  k mit k ∘ (gemessene Anker-Pose) = (Anker-Pose laut Odometrie beim Start); GridMap.shift_pose(k).

Kameramodell wie geometry.CamGeometry: entzerrte Bilder, Lochkamera (f aus hfov, Hauptpunkt Bildmitte), Blick um
tilt_deg + pitch_dev gehoben, gerollt um den IMU-Roll (ohne ihn lagen die 3D-Punkte um den Roll des Endes
verdreht), Linse cam_fwd vor der Radachse.

Offline geprüft (tools/start_abgleich.py, 2026-11-19): Fahrt 144940 Ende gemessen 7,9 cm vor / 9,9 cm links /
−4,7° (Nutzer mit Zollstock: 5 / 11 cm; Odometrie −3 / +2 cm); 14 kurze Paare derselben Fahrt gegen die
Odometrie: Median 1,1 cm je Richtung, 0,3°; Ausreißer (5–15 cm) nur unter ~27 stimmigen Treffern.

Kosten am Pi: nichts im Fahrbetrieb. Anker: einmal Merkmale eines Bildes. Schlussfahrt: je 4 cm Weg ein
Graubild (~1 ms). Messung: Merkmale von 1 + len(basen) Bildern und die Rechnung, im eigenen Faden auf
[cpu] render_cpus, während der Rider steht (SIFT am CM4 geschätzt 0,3–1 s je Bild, nicht gemessen).
"""
from __future__ import annotations

import logging
import math
import queue
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .affinity import pin_thread
from .reloc import anwenden, korrektur

log = logging.getLogger(__name__)


@dataclass
class Ansicht:
    """Ein Bild mit allem, was die Kameralage braucht. pose = (x, y, Kurs rad) in der Raumkarte (Radachse)."""
    gray: np.ndarray                 # Graubild, Histogramm ausgeglichen
    pose: tuple
    roll_deg: float                  # IMU-Roll (+ = rechts tief), 0 ohne IMU
    pitch_dev: float                 # Nick gegen die Referenzlage (rad, + = schaut höher)
    f: float
    cx: float
    cy: float
    tilt_deg: float
    cam_h: float                     # m
    cam_fwd: float                   # m
    t: float = 0.0
    kp: list | None = None           # Merkmale (erst im Faden berechnet)
    desc: np.ndarray | None = None


def ansicht(bgr: np.ndarray, pose: tuple, geo, roll_deg: float, pitch_dev: float, t: float = 0.0) -> Ansicht:
    """Aus einem Kamerabild und der aktuellen Geometrie (geometry.CamGeometry). ~1 ms am CM4."""
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    return Ansicht(cv2.equalizeHist(g), tuple(float(v) for v in pose), float(roll_deg), float(pitch_dev),
                   geo.f, geo.cx, geo.H / 2, geo.tilt_deg, geo.cam_h, geo.cam_fwd, t)


def kamera(a: Ansicht, pose: tuple | None = None, dnick: float = 0.0, droll: float = 0.0):
    """K, R (Welt -> Kamera), C (Kamerazentrum). Welt: x vor, y links, z hoch (Raumkarte); Kamera (OpenCV):
    x rechts, y unten, z Blick. pose (x, y, Kurs rad) ersetzt a.pose; dnick/droll (rad) kommen dazu."""
    x, y, th = pose if pose is not None else a.pose
    e = math.radians(a.tilt_deg) + a.pitch_dev + dnick
    zc = np.array([math.cos(e) * math.cos(th), math.cos(e) * math.sin(th), math.sin(e)])
    xc = np.array([math.sin(th), -math.cos(th), 0.0])
    yc = np.cross(zc, xc)
    ro = math.radians(a.roll_deg) + droll
    xc, yc = math.cos(ro) * xc + math.sin(ro) * yc, -math.sin(ro) * xc + math.cos(ro) * yc
    R = np.stack([xc, yc, zc])
    C = np.array([x + a.cam_fwd * math.cos(th), y + a.cam_fwd * math.sin(th), a.cam_h])
    K = np.array([[a.f, 0, a.cx], [0, a.f, a.cy], [0, 0, 1.0]])
    return K, R, C


def merkmale(a: Ansicht, n: int = 3000) -> Ansicht:
    if a.kp is None:
        a.kp, a.desc = cv2.SIFT_create(n).detectAndCompute(a.gray, None)
    return a


def _paare(da, db, ratio: float = 0.8):
    if da is None or db is None or len(da) < 2 or len(db) < 2:
        return []
    return [m for m, n in (p for p in cv2.BFMatcher(cv2.NORM_L2).knnMatch(da, db, k=2) if len(p) == 2)
            if m.distance < ratio * n.distance]


def tiefe(ende: Ansicht, tiefen: list[Ansicht]) -> dict[int, np.ndarray]:
    """3D-Punkte (Karten-KS) je Merkmal des Endbildes, trianguliert gegen jedes Tiefenbild; Median je Merkmal."""
    K, R0, C0 = kamera(ende)
    P0 = K @ np.hstack([R0, (-R0 @ C0)[:, None]])
    pts: dict[int, list] = {}
    for b in tiefen:
        _, R1, C1 = kamera(b)
        P1 = K @ np.hstack([R1, (-R1 @ C1)[:, None]])
        m10 = _paare(b.desc, ende.desc)
        if len(m10) < 20:
            continue
        p1 = np.float32([b.kp[m.queryIdx].pt for m in m10]).T
        p0 = np.float32([ende.kp[m.trainIdx].pt for m in m10]).T
        X = cv2.triangulatePoints(P1, P0, p1, p0)
        X = (X[:3] / X[3]).T
        ok = np.ones(len(X), bool)
        for P, pp, R, C in ((P1, p1, R1, C1), (P0, p0, R0, C0)):
            xh = P @ np.hstack([X, np.ones((len(X), 1))]).T
            z = (R @ (X - C).T)[2]
            ok &= (np.linalg.norm(xh[:2] / xh[2] - pp, axis=0) < 2.0) & (z > 0.2) & (z < 8.0)
        v1, v0 = X - C1, X - C0
        cosw = np.sum(v1 * v0, 1) / (np.linalg.norm(v1, axis=1) * np.linalg.norm(v0, axis=1))
        ok &= np.degrees(np.arccos(np.clip(cosw, -1, 1))) > 1.0          # zu wenig Parallaxe: Tiefe unsicher
        for i in np.flatnonzero(ok):
            pts.setdefault(m10[i].trainIdx, []).append(X[i])
    return {k: np.median(np.array(v), 0) for k, v in pts.items()}


def _projiziere(a: Ansicht, q: np.ndarray, X: np.ndarray):
    K, R, C = kamera(a, (q[0], q[1], q[2]), q[3] if len(q) > 3 else 0.0, q[4] if len(q) > 4 else 0.0)
    xc = R @ (X - C).T
    return ((K @ xc)[:2] / xc[2]).T, xc[2]


def eben_lage(a: Ansicht, X: np.ndarray, uv: np.ndarray, c_px: float = 4.0, iters: int = 30):
    """Lage des Ankers (x, y, Kurs, dNick, dRoll) mit fester Kamerahöhe; Start an a.pose. Rückgabe (q, Fehler px)
    oder (None, Fehler)."""
    q = np.array(tuple(a.pose) + (0.0, 0.0), float)
    hs = (1e-4, 1e-4, 1e-4, 1e-5, 1e-5)

    def res(p):
        pr, z = _projiziere(a, p, X)
        r = (pr - uv).ravel()
        r[np.repeat(z <= 0.05, 2)] = 1e3
        return r
    # Robustgrenze von grob nach fein: Die Odometrie liegt am Anker einige cm und Grad daneben, das sind schon
    # mehr als c_px Pixel; mit der engen Grenze von Anfang an hätte kein Punkt Gewicht
    e = None
    for c in [40.0] * 5 + [20.0] * 5 + [10.0] * 5 + [c_px] * iters:
        r = res(q)
        e = np.hypot(r[0::2], r[1::2])
        w = np.where(e < c, (1 - (e / c) ** 2) ** 2, 0.0)
        if (w > 0).sum() < 6:
            return None, e
        w2 = np.repeat(w, 2)
        J = np.empty((len(r), 5))
        for k, h in enumerate(hs):
            p = q.copy()
            p[k] += h
            J[:, k] = (res(p) - r) / h
        A = J.T @ (J * w2[:, None])
        dq = -np.linalg.solve(A + 1e-9 * np.eye(5), J.T @ (w2 * r))
        q += dq
        if c == c_px and np.all(np.abs(dq) < (1e-5, 1e-5, 2e-5, 1e-5, 1e-5)):
            break
    e = np.hypot(*(_projiziere(a, q, X)[0] - uv).T)
    return q, e


@dataclass
class Ergebnis:
    ok: bool
    grund: str                                    # Text fürs Log und den Status
    k: tuple | None = None                        # Korrektur der Kartenpose (kx, ky, kth)
    anker_wahr: tuple | None = None               # gemessene Lage des Ankers im Karten-KS (x, y, rad)
    inl: int = 0
    rep_px: float | None = None
    dnick_deg: float = 0.0
    droll_deg: float = 0.0
    frei6: tuple | None = None                    # Gegenprobe frei (x, y, rad)
    n3d: int = 0
    ms: float = 0.0
    info: dict = field(default_factory=dict)


def messen(anker: Ansicht, ende: Ansicht, tiefen: list[Ansicht], cfg) -> Ergebnis:
    """Die ganze Rechnung (cfg = StartbildCfg). Merkmale fehlender Ansichten werden hier berechnet."""
    t0 = time.monotonic()
    for a in [anker, ende] + list(tiefen):
        merkmale(a, cfg.features)
    pts3 = tiefe(ende, tiefen)
    if len(pts3) < cfg.min_inl:
        return Ergebnis(False, "zu wenig Tiefe (%d Punkte aus %d Bildern)" % (len(pts3), len(tiefen)), n3d=len(pts3),
                        ms=1000 * (time.monotonic() - t0))
    mA0 = [m for m in _paare(anker.desc, ende.desc) if m.trainIdx in pts3]
    if len(mA0) < cfg.min_inl:
        return Ergebnis(False, "zu wenig Treffer mit dem Startbild (%d)" % len(mA0), n3d=len(pts3),
                        ms=1000 * (time.monotonic() - t0))
    X = np.float64([pts3[m.trainIdx] for m in mA0])
    uv = np.float64([anker.kp[m.queryIdx].pt for m in mA0])
    q, e = eben_lage(anker, X, uv)
    if q is None:
        return Ergebnis(False, "Lage nicht bestimmbar", n3d=len(pts3), ms=1000 * (time.monotonic() - t0))
    inl = e < 3.0
    erg = Ergebnis(False, "", inl=int(inl.sum()), rep_px=round(float(np.median(e[inl])), 2) if inl.any() else None,
                   dnick_deg=math.degrees(q[3]), droll_deg=math.degrees(q[4]), n3d=len(pts3),
                   anker_wahr=(float(q[0]), float(q[1]), float(q[2])))
    # Gegenprobe frei in 6 Freiheitsgraden
    K = kamera(anker)[0]
    okp, rvec, tvec, inl6 = cv2.solvePnPRansac(X, uv, K, None, reprojectionError=3.0, iterationsCount=500,
                                               confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if okp and inl6 is not None and len(inl6) >= 6:
        RA, _ = cv2.Rodrigues(rvec)
        CA = (-RA.T @ tvec).ravel()
        th6 = math.atan2(RA[2][1], RA[2][0])
        erg.frei6 = (CA[0] - anker.cam_fwd * math.cos(th6), CA[1] - anker.cam_fwd * math.sin(th6), th6)
    erg.k = korrektur(erg.anker_wahr, tuple(anker.pose))
    d_m = math.hypot(erg.k[0], erg.k[1])
    d_deg = abs(math.degrees(erg.k[2]))
    # Die Korrektur verschiebt das Ende; als Maß zählt, wie weit sich die aktuelle Pose bewegt
    neu = anwenden(erg.k, tuple(ende.pose))
    schub = math.hypot(neu[0] - ende.pose[0], neu[1] - ende.pose[1])
    erg.info = {"schub_m": round(schub, 3), "dreh_deg": round(math.degrees(erg.k[2]), 2)}
    if erg.inl < cfg.min_inl:
        erg.grund = "zu wenig stimmige Treffer (%d < %d)" % (erg.inl, cfg.min_inl)
    elif schub > cfg.max_m or d_deg > cfg.max_deg:
        erg.grund = "Korrektur unplausibel (%.2f m, %.1f°)" % (schub, d_deg)
    elif abs(erg.dnick_deg) > cfg.max_lage_deg or abs(erg.droll_deg) > cfg.max_lage_deg:
        erg.grund = "Nick/Roll passen nicht (%.1f° / %.1f°)" % (erg.dnick_deg, erg.droll_deg)
    elif erg.frei6 is None or math.hypot(erg.frei6[0] - erg.anker_wahr[0],
                                         erg.frei6[1] - erg.anker_wahr[1]) > cfg.agree_m:
        erg.grund = "Gegenprobe weicht ab"
    else:
        erg.ok = True
        erg.grund = "%.0f cm, %+.1f°, %d Treffer" % (100 * schub, math.degrees(erg.k[2]), erg.inl)
    erg.ms = 1000 * (time.monotonic() - t0)
    return erg


class StartAbgleich:
    """Anker halten und Messungen in einem eigenen Faden rechnen. App ruft anker_setzen/messen und je
    Regelschritt poll(); alles Rechnen läuft im Faden."""

    def __init__(self, cfg, cpus=None):
        self.cfg = cfg
        self.anker: Ansicht | None = None
        self.anker_bereit = False                 # Merkmale des Ankers berechnet
        self.anker_start: tuple | None = None     # Startpunkt der Karte, zu dem der Anker gehört
        self.letztes: Ergebnis | None = None
        self.n_ok = self.n_verworfen = 0
        self._cpus = cpus
        self._q: queue.Queue = queue.Queue(maxsize=2)
        self._out: queue.Queue = queue.Queue()
        self._busy = False
        self._th: threading.Thread | None = None

    def _faden(self) -> None:
        if self._th is None:
            self._th = threading.Thread(target=self._run, name="startbild", daemon=True)
            self._th.start()

    def _run(self) -> None:
        pin_thread(self._cpus, "startbild", quiet=True)
        while True:
            job = self._q.get()
            try:
                if job[0] == "anker":
                    merkmale(job[1], self.cfg.features)
                    self.anker_bereit = True
                    log.info("Startbild: Anker gespeichert (%d Merkmale)", len(job[1].kp or []))
                else:
                    self._out.put(messen(job[1], job[2], job[3], self.cfg))
            except Exception as e:                # nie den Faden verlieren; ohne Ergebnis bleibt es beim alten
                log.exception("Startbild: Rechnung fehlgeschlagen")
                if job[0] == "messen":
                    self._out.put(Ergebnis(False, "Fehler: %s" % e))
            finally:
                self._busy = False

    def anker_setzen(self, a: Ansicht, start: tuple) -> None:
        self.anker, self.anker_bereit, self.anker_start = a, False, tuple(start)
        self._faden()
        self._busy = True
        self._q.put(("anker", a))

    def anker_weg(self, grund: str) -> None:
        if self.anker is not None:
            log.info("Startbild: Anker verworfen (%s)", grund)
        self.anker, self.anker_bereit, self.anker_start = None, False, None

    def messen(self, ende: Ansicht, tiefen: list[Ansicht]) -> bool:
        if self.anker is None or self._busy:
            return False
        self._faden()
        self._busy = True
        self._q.put(("messen", self.anker, ende, list(tiefen)))
        return True

    def poll(self) -> Ergebnis | None:
        try:
            e = self._out.get_nowait()
        except queue.Empty:
            return None
        self.letztes = e
        if e.ok:
            self.n_ok += 1
        else:
            self.n_verworfen += 1
        log.info("Startbild: %s (%s, %.0f ms)", "Korrektur" if e.ok else "keine Korrektur", e.grund, e.ms)
        return e

    def info(self) -> dict:
        e = self.letztes
        return {"anker": self.anker_bereit, "ok": self.n_ok, "verworfen": self.n_verworfen,
                "letztes": None if e is None else {"ok": e.ok, "grund": e.grund, "inl": e.inl, "ms": round(e.ms),
                                                    "n3d": e.n3d, **e.info}}
