"""Eigenbewegung der Kamera aus dem Bild (optischer Fluss + RANSAC).

Liefert pro Frame die Ähnlichkeitstransformation prev->cur (volle Auflösung),
daraus Dreh-/Nickrate, eine Schärfe-Kennzahl und eine Time-to-Contact-
Schätzung (Expansion im Bildzentrum). Bewegte Objekte (Tracks) werden beim
Feature-Setzen ausmaskiert, RANSAC verwirft den Rest.

Warum nicht IMU? Der Treiber liefert Yaw nur mit ~1 Hz und keine Roll/Pitch-
Werte. Der Fluss misst außerdem direkt, was im Bild passiert (inkl. Rolling-
Shutter-Effekte) – genau das braucht der Tracker.
"""
from __future__ import annotations

import ctypes
import logging
import math
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from . import fastlib
from .config import EgoCfg

log = logging.getLogger(__name__)

I3 = np.eye(3, dtype=np.float64)

# ------------------------------------------------------------------ C++-Kern (optional)
NATIVE_PATH = fastlib.PATH
_NATIVE_ABI = 2
_FAST_NAMES = ("Grau", "Schärfe", "Merkmale")   # Bits von ego_fast_mask()


class _EgoParams(ctypes.Structure):          # = EgoParams in native/egofast.cpp
    _fields_ = [(n, ctypes.c_int) for n in ("pw", "ph", "full_w", "full_h", "lk_win", "lk_levels", "lk_iters",
                                            "lk_fb_check", "max_features", "min_features", "min_distance", "pad0")] + \
               [(n, ctypes.c_double) for n in ("s", "ransac_px", "quality", "eps")] + \
               [("K", ctypes.c_double * 9), ("Ki", ctypes.c_double * 9), ("Hd", ctypes.c_double * 27)]


class _EgoOut(ctypes.Structure):             # = EgoOut in native/egofast.cpp
    _fields_ = [(n, ctypes.c_int) for n in ("tracked", "inliers", "npts", "pad0")] + \
               [(n, ctypes.c_double) for n in ("sharp", "yaw", "pitch", "roll", "sc", "dx", "dy", "exp_inst")] + \
               [("A", ctypes.c_double * 9)]


def _native_lib():
    """C++-Kern mit passender Ego-ABI, sonst None (siehe fastlib.py)."""
    lib = fastlib.load()
    if not fastlib.check_abi(lib, "ego_abi", _NATIVE_ABI):
        return None
    lib.ego_new.restype = ctypes.c_void_p
    lib.ego_new.argtypes = [ctypes.POINTER(_EgoParams)]
    lib.ego_free.argtypes = [ctypes.c_void_p]
    lib.ego_step.restype = ctypes.c_int
    lib.ego_step.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                             ctypes.c_double, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(_EgoOut)]
    lib.ego_fast_mask.restype = ctypes.c_int
    lib.ego_fast_mask.argtypes = [ctypes.c_int, ctypes.c_int]
    i, p, d = ctypes.c_int, ctypes.c_void_p, ctypes.c_double
    lib.xg_gray_half.restype = lib.xg_sharpness.restype = lib.xg_good_features.restype = i
    lib.xg_gray_half.argtypes = [p, i, i, i, p]
    lib.xg_sharpness.argtypes = [p, i, i, i, ctypes.POINTER(d)]
    lib.xg_good_features.argtypes = [p, i, i, i, p, i, d, d, p, i]
    return lib


@dataclass
class EgoResult:
    idx: int
    t: float
    dt: float
    A: np.ndarray = field(default_factory=lambda: I3.copy())  # 3x3, prev -> cur
    dx: float = 0.0          # Verschiebung der Bildmitte, px (volle Auflösung)
    dy: float = 0.0
    yaw: float = 0.0         # rad pro Frame, + = Kamera dreht nach links
    pitch: float = 0.0       # rad pro Frame, + = Kamera nickt nach oben
    roll: float = 0.0        # rad pro Frame
    rot: float = 0.0         # = roll (Kompatibilität)
    scale: float = 1.0       # Expansion um die Bildmitte (Vorwärtsfahrt)
    yaw_rate: float = 0.0    # rad/s, >0 = Linksdrehung des Roboters
    pitch_rate: float = 0.0  # rad/s, >0 = Kamera nickt nach oben
    sharpness: float = 0.0
    inliers: int = 0
    ok: bool = False
    expansion: float = 0.0   # 1/s, geglättet (Zentrum); TTC = 1/expansion
    pitch_dev: float = 0.0   # rad, aktuelles Nicken ggü. geglättetem Verlauf (+ = schaut höher)

    @property
    def ttc(self) -> float:
        return 1.0 / self.expansion if self.expansion > 1e-3 else math.inf


def rot_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Kameradrehung (Kamera-KS: x rechts, y unten, z vorn). yaw + = links, pitch + = hoch."""
    cy, sy = math.cos(-yaw), math.sin(-yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return Ry @ Rx @ Rz


def _to3(M: np.ndarray) -> np.ndarray:
    A = I3.copy()
    A[:2] = M
    return A


class EgoMotion:
    def __init__(self, cfg: EgoCfg, full_size: tuple[int, int], hfov_deg: float):
        self.cfg = cfg
        self.full_w, self.full_h = full_size
        self.s = self.full_w / cfg.proc_width
        self.pw = cfg.proc_width
        self.ph = int(round(self.full_h / self.s))
        self.fx = (self.full_w / 2) / math.tan(math.radians(hfov_deg) / 2)
        f = self.fx / self.s  # Brennweite in Arbeitsauflösung
        self.K = np.array([[f, 0, self.pw / 2], [0, f, self.ph / 2], [0, 0, 1.0]])
        self.Ki = np.linalg.inv(self.K)
        self.prev_gray: np.ndarray | None = None
        self.prev_pts: np.ndarray | None = None
        self.prev_t: float | None = None
        self.history: deque[tuple[int, np.ndarray]] = deque(maxlen=90)
        self._exp = 0.0
        # Trajektorie für Stabilisierungsanzeige
        self._traj = np.zeros(3)     # yaw, pitch, roll (kumulativ, rad)
        self._smooth = np.zeros(3)
        self._sharp_hist: deque[float] = deque(maxlen=60)
        self.S = np.diag([self.s, self.s, 1.0])
        self.Si = np.diag([1 / self.s, 1 / self.s, 1.0])
        self.last: EgoResult | None = None
        # Für _fit_rotation: Homographien der Einheitsdrehungen (konstant, nicht je Bild neu)
        self._eps = 1e-3
        self._Hd = [self._model_h(*[self._eps if j == k else 0.0 for j in range(3)], 0.0) for k in range(3)]
        self._center = np.array([[self.pw / 2], [self.ph / 2]])
        self._nat = None
        self._lib = _native_lib() if getattr(cfg, "native", False) else None
        if self._lib is not None:
            c = cfg
            p = _EgoParams(pw=self.pw, ph=self.ph, full_w=int(self.full_w), full_h=int(self.full_h),
                           lk_win=c.lk_win, lk_levels=c.lk_levels, lk_iters=c.lk_iters,
                           lk_fb_check=int(bool(c.lk_fb_check)), max_features=c.max_features,
                           min_features=c.min_features, min_distance=c.min_distance,
                           s=self.s, ransac_px=c.ransac_px, quality=c.quality, eps=self._eps)
            p.K[:] = self.K.ravel().tolist()
            p.Ki[:] = self.Ki.ravel().tolist()
            p.Hd[:] = np.concatenate([h.ravel() for h in self._Hd]).tolist()
            self._nat = self._lib.ego_new(ctypes.byref(p))
            self._out = _EgoOut()
        self.backend = "c++" if self._nat else "python"
        if self._nat:
            # Abkürzungen nur, wenn beim Start bitgleich zu OpenCV geprüft (sonst rechnet dort OpenCV)
            fm = self._lib.ego_fast_mask(self.pw, self.ph)
            fast = [n for k, n in enumerate(_FAST_NAMES) if fm >= 0 and fm >> k & 1]
            log.info("Ego-Motion: C++-Kern %s (Abkürzungen: %s)", NATIVE_PATH, ", ".join(fast) or "keine")
        elif getattr(cfg, "native", False):
            log.info("Ego-Motion in Python (C++-Kern nicht gebaut: make -C xgonav/native)")

    def __del__(self):
        if getattr(self, "_nat", None):
            self._lib.ego_free(self._nat)
            self._nat = None

    # ---------------------------------------------------------------------
    def update(self, idx: int, t: float, bgr: np.ndarray, exclude_boxes=()) -> EgoResult:
        if self._nat:
            return self._update_native(idx, t, bgr, exclude_boxes)
        # Erst grau, dann verkleinern (1 statt 3 Kanäle; CM4 0,83 statt 1,21 ms) und Varianz
        # per meanStdDev statt numpy (0,9 statt 1,4 ms). Werte gleich bis auf Rundung.
        gray = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (self.pw, self.ph),
                          interpolation=cv2.INTER_AREA)
        sharp = float(cv2.meanStdDev(cv2.Laplacian(gray, cv2.CV_32F))[1][0, 0] ** 2)
        self._sharp_hist.append(sharp)
        dt = (t - self.prev_t) if self.prev_t is not None else 0.0
        res = EgoResult(idx=idx, t=t, dt=dt, sharpness=sharp)

        if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) >= 8 and dt > 0:
            self._track(gray, res)

        self.history.append((idx, res.A))
        self._update_traj(res)
        self._replenish(gray, exclude_boxes)
        self.prev_gray = gray
        self.prev_t = t
        self.last = res
        return res

    def _update_native(self, idx: int, t: float, bgr: np.ndarray, exclude_boxes) -> EgoResult:
        """Wie update(), Fluss/Modell/Merkmale im C++-Kern (eine Pyramide je Bild)."""
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.strides[1:] != (3, 1):
            bgr = np.ascontiguousarray(bgr, dtype=np.uint8)
        boxes = np.ascontiguousarray(np.asarray(list(exclude_boxes), np.float64).reshape(-1, 4))
        dt = (t - self.prev_t) if self.prev_t is not None else 0.0
        o = self._out
        rc = self._lib.ego_step(self._nat, bgr.ctypes.data, bgr.shape[1], bgr.shape[0], bgr.strides[0], dt,
                                boxes.ctypes.data if len(boxes) else None, len(boxes), ctypes.byref(o))
        if rc != 0:
            log.warning("Ego-Motion (C++) fehlgeschlagen – weiter in Python")
            self._lib.ego_free(self._nat)
            self._nat = None
            self.backend = "python"
            return self.update(idx, t, bgr, exclude_boxes)
        res = EgoResult(idx=idx, t=t, dt=dt, sharpness=o.sharp)
        self._sharp_hist.append(o.sharp)
        if o.tracked:
            res.A = np.array(o.A, np.float64).reshape(3, 3)
            res.inliers = o.inliers
            res.ok = o.inliers >= 10
            res.yaw, res.pitch, res.roll, res.rot = o.yaw, o.pitch, o.roll, o.roll
            res.scale = 1.0 + o.sc
            res.dx, res.dy = o.dx, o.dy
            res.yaw_rate = o.yaw / dt
            res.pitch_rate = o.pitch / dt
            self._exp = 0.7 * self._exp + 0.3 * min(o.exp_inst, 5.0)
            res.expansion = self._exp
        self.history.append((idx, res.A))
        self._update_traj(res)
        self.prev_t = t
        self.last = res
        return res

    def sharpness_median(self) -> float:
        # je Bild aufgerufen: bei 60 Werten ist sorted() ~10x schneller als np.median (gleicher Wert)
        v = sorted(self._sharp_hist)
        n = len(v)
        if not n:
            return 0.0
        return float(v[n // 2]) if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2

    # ---------------------------------------------------------------------
    def _track(self, gray: np.ndarray, res: EgoResult) -> None:
        # Hinweis: Vorab gebaute Pyramiden (buildOpticalFlowPyramid) nimmt die Python-Bindung
        # von calcOpticalFlowPyrLK nicht an; sie werden also bei jedem Aufruf neu gebaut.
        c = self.cfg
        lk = dict(winSize=(c.lk_win, c.lk_win), maxLevel=c.lk_levels,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, c.lk_iters, 0.03))
        p0 = self.prev_pts
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None, **lk)
        if p1 is None:
            self.prev_pts = None
            return
        good = st.ravel() == 1
        if c.lk_fb_check:
            # Vorwärts-Rückwärts-Prüfung: verdoppelt die Kosten, verwirft unsichere Punkte
            p0r, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, p1, None, **lk)
            e = (p0 - p0r).reshape(-1, 2)
            good &= (st2.ravel() == 1) & (e[:, 0] * e[:, 0] + e[:, 1] * e[:, 1] < 1.0)
        a, b = p0[good].reshape(-1, 2), p1[good].reshape(-1, 2)
        if len(a) < 8:
            self.prev_pts = b.reshape(-1, 1, 2) if len(b) else None
            return
        M, inl = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC,
                                             ransacReprojThreshold=self.cfg.ransac_px,
                                             maxIters=200, confidence=0.98)
        if M is None:
            self.prev_pts = b.reshape(-1, 1, 2)
            return
        inl = inl.ravel().astype(bool)
        q, inl = self._fit_rotation(a, b, inl)
        yaw, pitch, roll, sc = q
        Hs = self._model_h(yaw, pitch, roll, sc)
        A = self.S @ Hs @ self.Si
        res.A = A
        res.inliers = int(inl.sum())
        res.ok = res.inliers >= 10
        res.yaw, res.pitch, res.roll, res.rot = yaw, pitch, roll, roll
        res.scale = 1.0 + sc
        c = np.array([self.full_w / 2, self.full_h / 2, 1.0])
        cc = A @ c
        res.dx, res.dy = float(cc[0] / cc[2] - c[0]), float(cc[1] / cc[2] - c[1])
        res.yaw_rate = yaw / res.dt
        res.pitch_rate = pitch / res.dt
        self._expansion(a[inl], b[inl], res)
        # nur Inlier weiterverfolgen (Außenseiter = bewegte Objekte)
        self.prev_pts = b[inl].reshape(-1, 1, 2).astype(np.float32)

    def _model_h(self, yaw: float, pitch: float, roll: float, sc: float) -> np.ndarray:
        """Homographie (Arbeitsauflösung): Drehung K R^T K^-1, dann Expansion um die Mitte."""
        Hr = self.K @ rot_matrix(yaw, pitch, roll).T @ self.Ki
        cx, cy = self.pw / 2, self.ph / 2
        E = np.array([[1 + sc, 0, -sc * cx], [0, 1 + sc, -sc * cy], [0, 0, 1.0]])
        return E @ Hr

    def _fit_rotation(self, a: np.ndarray, b: np.ndarray, inl: np.ndarray):
        """Lineares LS für (yaw, pitch, roll, expansion) auf RANSAC-Inliern, 2 Iterationen."""
        eps = self._eps
        n = len(a)
        P = np.empty((3, n))                                 # 3xN
        P[:2] = a.T
        P[2] = 1.0
        J = np.empty((n, 2, 4))                              # je Punkt (x, y) x 4 Parameter
        for k in range(3):
            Q = self._Hd[k] @ P
            J[:, :, k] = (((Q[:2] / Q[2]) - P[:2]) / eps).T
        J[:, :, 3] = (P[:2] - self._center).T
        J = J.reshape(2 * n, 4)                              # Zeilen x0,y0,x1,y1...
        d = (b - a).reshape(-1)
        m = inl.copy()
        q = np.zeros(4)
        thr2 = max(self.cfg.ransac_px, 1.0) ** 2
        for _ in range(2):
            if m.sum() < 6:
                break
            mm = np.repeat(m, 2)
            Jm = J[mm]
            # 4x4-Normalgleichungen statt lstsq (gut konditioniert, deutlich schneller)
            try:
                q = np.linalg.solve(Jm.T @ Jm, Jm.T @ d[mm])
            except np.linalg.LinAlgError:
                q, *_ = np.linalg.lstsq(Jm, d[mm], rcond=None)
            r = (J @ q - d).reshape(-1, 2)
            m = r[:, 0] * r[:, 0] + r[:, 1] * r[:, 1] < thr2
        return q, m

    def _expansion(self, a: np.ndarray, b: np.ndarray, res: EgoResult) -> None:
        """Skalierung nur im Bildzentrum -> Annäherung an frontales Hindernis."""
        cx, cy = self.pw / 2, self.ph / 2
        m = (np.abs(a[:, 0] - cx) < self.pw * 0.25) & (np.abs(a[:, 1] - cy) < self.ph * 0.3)
        inst = 0.0
        if m.sum() >= 6:
            Mc, _ = cv2.estimateAffinePartial2D(a[m], b[m], method=cv2.LMEDS)
            if Mc is not None:
                sc = math.hypot(Mc[0, 0], Mc[1, 0])
                inst = max(0.0, (sc - 1.0) / res.dt)
        self._exp = 0.7 * self._exp + 0.3 * min(inst, 5.0)
        res.expansion = self._exp

    def _replenish(self, gray: np.ndarray, exclude_boxes) -> None:
        n = 0 if self.prev_pts is None else len(self.prev_pts)
        if n >= self.cfg.min_features:
            return
        mask = np.full(gray.shape, 255, np.uint8)
        for (x1, y1, x2, y2) in exclude_boxes:
            cv2.rectangle(mask, (int(x1 / self.s), int(y1 / self.s)),
                          (int(x2 / self.s), int(y2 / self.s)), 0, -1)
        if n:
            for p in self.prev_pts.reshape(-1, 2):
                cv2.circle(mask, (int(p[0]), int(p[1])), self.cfg.min_distance, 0, -1)
        new = cv2.goodFeaturesToTrack(gray, maxCorners=self.cfg.max_features - n,
                                      qualityLevel=self.cfg.quality,
                                      minDistance=self.cfg.min_distance, mask=mask, blockSize=5)
        if new is None:
            return
        new = new.astype(np.float32)
        self.prev_pts = new if not n else np.vstack([self.prev_pts, new])

    def _update_traj(self, res: EgoResult) -> None:
        self._traj += (res.yaw, res.pitch, res.roll)
        a = self.cfg.smooth_alpha
        self._smooth = (1 - a) * self._smooth + a * self._traj
        res.pitch_dev = float(self._traj[1] - self._smooth[1])

    # ---------------------------------------------------------------------
    def transform_between(self, idx_from: int, idx_to: int) -> np.ndarray | None:
        """Komponierte Transformation von Frame idx_from nach idx_to (3x3)."""
        if idx_from == idx_to:
            return I3.copy()
        T = I3.copy()
        found = False
        for i, A in self.history:
            if idx_from < i <= idx_to:
                T = A @ T
                found = True
        if not found or not self.history or self.history[0][0] > idx_from + 1:
            return None
        return T

    def stab_matrix(self) -> np.ndarray:
        """3x3-Warp (volle Auflösung), der das Bild auf den geglätteten Verlauf dreht (Anzeige)."""
        dyaw, dpitch, droll = self._traj - self._smooth
        return self.S @ self._model_h(-dyaw, -dpitch, -droll, 0.0) @ self.Si


def warp_box(A: np.ndarray, box) -> tuple[float, float, float, float]:
    """Bbox (x1,y1,x2,y2) mit 3x3-Homographie transformieren.

    Über die Kantenmittelpunkte statt der Ecken: Bei Nicken spreizen sich weit
    außerhalb liegende Ecken (unsichtbare Box-Verlängerung) perspektivisch auf;
    die Mittelpunkte bilden Fußpunkt/Seiten exakt ab.
    """
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    pts = np.array([[x1, cy, 1], [x2, cy, 1], [cx, y1, 1], [cx, y2, 1]], dtype=np.float64).T
    q = A @ pts
    q = q[:2] / q[2]
    return float(q[0, 0]), float(q[1, 2]), float(q[0, 1]), float(q[1, 3])
