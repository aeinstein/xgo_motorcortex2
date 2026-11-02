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

import math
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import EgoCfg

I3 = np.eye(3, dtype=np.float64)


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

    # ---------------------------------------------------------------------
    def update(self, idx: int, t: float, bgr: np.ndarray, exclude_boxes=()) -> EgoResult:
        small = cv2.resize(bgr, (self.pw, self.ph), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        sharp = float(cv2.Laplacian(gray, cv2.CV_32F).var())
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

    def sharpness_median(self) -> float:
        return float(np.median(self._sharp_hist)) if self._sharp_hist else 0.0

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
            fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
            good &= (st2.ravel() == 1) & (fb < 1.0)
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
        if not hasattr(self, "_J"):
            self._J = {}
        eps = 1e-3
        P = np.hstack([a, np.ones((len(a), 1))]).T          # 3xN
        cols = []
        for k in range(3):
            q = [0.0, 0.0, 0.0]
            q[k] = eps
            Q = self._model_h(*q, 0.0) @ P
            cols.append(((Q[:2] / Q[2]) - P[:2]) / eps)       # 2xN
        cols.append(P[:2] - np.array([[self.pw / 2], [self.ph / 2]]))
        J = np.stack([c.T.reshape(-1) for c in cols], axis=1)  # (2N)x4, Zeilen x0,y0,x1,y1...
        d = (b - a).reshape(-1)
        m = inl.copy()
        q = np.zeros(4)
        for _ in range(2):
            mm = np.repeat(m, 2)
            if m.sum() < 6:
                break
            q, *_ = np.linalg.lstsq(J[mm], d[mm], rcond=None)
            r = (J @ q - d).reshape(-1, 2)
            err = np.hypot(r[:, 0], r[:, 1])
            m = err < max(self.cfg.ransac_px, 1.0)
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
