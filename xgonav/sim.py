"""Closed-Loop-Simulator zum Testen ohne Hardware.

Raycasting-Renderer (Raum + Kisten + Personen-Billboard), Kamera auf
balancierendem Roboter: Nicken (Pendeln + Beschleunigungsneigung), Kurszittern
und vertikale Bewegungsunschärfe. Die Fahrbefehle des SimRobot bewegen die
Kamera. Ein FakeDetector liefert Ground-Truth-Boxen mit realistischer Latenz.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera import FrameSource
from .config import CameraCfg
from .detector import Detection, DetectionBatch
from .robot import SimRobot


def _blob_tex(seed: int, base, w=256, h=128) -> np.ndarray:
    rng = np.random.default_rng(seed)
    t = np.full((h, w, 3), base, np.uint8)
    for _ in range(140):
        c = tuple(int(x) for x in np.clip(np.array(base) + rng.integers(-90, 90, 3), 0, 255))
        cv2.circle(t, (int(rng.integers(0, w)), int(rng.integers(0, h))), int(rng.integers(2, 12)), c, -1)
    for x in range(0, w, 64):  # vertikale Kanten (Türrahmen/Möbel)
        cv2.line(t, (x, 0), (x, h), (40, 40, 40), 2)
    return cv2.GaussianBlur(t, (3, 3), 0)


@dataclass
class Person:
    path: list[tuple[float, float]]
    speed: float = 0.35
    x: float = 0.0
    y: float = 0.0
    seg: int = 0
    pause_until: float = 0.0

    def __post_init__(self):
        self.x, self.y = self.path[0]

    def step(self, t: float, dt: float) -> None:
        if t < self.pause_until:
            return
        tx, ty = self.path[(self.seg + 1) % len(self.path)]
        dx, dy = tx - self.x, ty - self.y
        d = math.hypot(dx, dy)
        s = self.speed * dt
        if d <= s:
            self.x, self.y = tx, ty
            self.seg = (self.seg + 1) % len(self.path)
            self.pause_until = t + 1.5
        else:
            self.x += dx / d * s
            self.y += dy / d * s


@dataclass
class SimScene:
    size: float = 6.0
    boxes: list = field(default_factory=lambda: [(2.0, 3.6, 2.7, 4.3), (4.2, 0.9, 4.8, 2.6)])
    wall_h: float = 1.0
    box_h: float = 0.6

    def segments(self):
        s = self.size
        seg = [(0, 0, s, 0, 0), (s, 0, s, s, 0), (s, s, 0, s, 1), (0, s, 0, 0, 1)]
        for i, (x1, y1, x2, y2) in enumerate(self.boxes):
            k = 2
            seg += [(x1, y1, x2, y1, k), (x2, y1, x2, y2, k), (x2, y2, x1, y2, k), (x1, y2, x1, y1, k)]
        return np.array(seg, np.float64)

    def collides(self, x: float, y: float, r: float = 0.12) -> bool:
        if x < r or y < r or x > self.size - r or y > self.size - r:
            return True
        return any(x1 - r < x < x2 + r and y1 - r < y < y2 + r for (x1, y1, x2, y2) in self.boxes)


class SimWorld(FrameSource):
    def __init__(self, cfg: CameraCfg, robot: SimRobot, person: Person | None = None,
                 start=(1.0, 1.0, 0.0), shake: float = 1.0, render_scale: int = 2, fps: int = 20,
                 vx_mps: float = 1.0, mount_offset_mm: float = 45.0):
        super().__init__(cfg)
        self.robot = robot
        self.scene = SimScene()
        self.segs = self.scene.segments()
        self.person = person
        self.x, self.y, self.th = start
        self.shake = shake
        self.rs = render_scale
        self.fps = fps
        self.vx_mps = vx_mps
        self.mount = mount_offset_mm
        self.tex = [_blob_tex(1, (150, 160, 170)), _blob_tex(2, (120, 140, 120)), _blob_tex(3, (60, 90, 170))]
        self.collisions = 0
        self.dist_travelled = 0.0
        self.log: list[dict] = []
        self.truth: dict = {}
        self.truth_by_t: dict = {}
        self.coll_frames: list[int] = []
        self._t0 = None
        self._prev_v = 0.0
        self._acc_lp = 0.0
        self.pitch = 0.0
        self._pitch_prev = 0.0
        self._rng = np.random.default_rng(7)

    # ------------------------------------------------------------------
    def _grab(self) -> bool:
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
            self._last = now
        target = self._last + 1.0 / self.fps
        if target > now:
            time.sleep(target - now)
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        t = now - self._t0
        self._integrate(t, dt)
        img, box = self._render(t, dt)
        self.truth = {"t": now, "box": box, "pitch": self.pitch}
        self.truth_by_t[now] = self.truth
        if len(self.truth_by_t) > 200:
            self.truth_by_t.pop(next(iter(self.truth_by_t)))
        self._publish(img, now)
        return True

    def _integrate(self, t: float, dt: float) -> None:
        vn, wn = self.robot.command
        v = vn * self.vx_mps
        w = math.radians(wn * self.robot.cfg.vyaw_limit_dps)
        self.th += w * dt
        nx = self.x + v * math.cos(self.th) * dt
        ny = self.y + v * math.sin(self.th) * dt
        if self.scene.collides(nx, ny):
            if v != 0 and not getattr(self, "_in_coll", False):
                self.collisions += 1
                self.coll_frames.append(self._idx)
            self._in_coll = True
        else:
            self._in_coll = False
            self.dist_travelled += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
        if self.person:
            self.person.step(t, dt)
        acc = (v - self._prev_v) / max(dt, 1e-3)
        self._prev_v = v
        self._acc_lp = 0.7 * self._acc_lp + 0.3 * float(np.clip(acc, -2, 2))
        self.pitch = self.pitch_truth(t)
        self.yaw_jit = self.shake * math.radians(0.8) * math.sin(2 * math.pi * 1.7 * t)
        cam_h = ((self.robot.height_mm or 85.0) + self.mount) / 1000.0
        self.cam_h = cam_h
        self.log.append({"t": t, "x": self.x, "y": self.y, "th": self.th, "v": v, "w": w,
                         "px": self.person.x if self.person else None,
                         "py": self.person.y if self.person else None})

    def pitch_truth(self, t: float) -> float:
        """Kameranicken (rad, + = hoch) zur Zeit t (s seit Start), zeitkontinuierlich:
        Balancier-Pendeln + Vibration + Neigung beim Beschleunigen."""
        return self.shake * (math.radians(3.0) * math.sin(2 * math.pi * 2.5 * t)
                             + math.radians(1.5) * math.sin(2 * math.pi * 6.1 * t + 1)
                             + math.radians(0.5) * math.sin(2 * math.pi * 9.3 * t + 2)
                             + math.radians(0.3) * math.sin(2 * math.pi * 13.1 * t + 3)
                             - math.radians(4.0) * self._acc_lp / 2)

    def _render(self, t: float, dt: float):
        W, H = self.cfg.width // self.rs, self.cfg.height // self.rs
        f = (W / 2) / math.tan(math.radians(self.cfg.hfov_deg) / 2)
        th = self.th + self.yaw_jit
        cols = np.arange(W) + 0.5
        lat = (W / 2 - cols) / f                      # tan(Winkel) links +
        fwd = np.array([math.cos(th), math.sin(th)])
        left = np.array([-math.sin(th), math.cos(th)])
        dirs = fwd[None] + lat[:, None] * left[None]  # Strahlen (nicht normiert, Tiefe=1)
        # Strahl-Segment-Schnitt
        p = np.array([self.x, self.y])
        a, b = self.segs[:, 0:2], self.segs[:, 2:4]
        e = b - a
        denom = dirs[:, None, 0] * e[None, :, 1] - dirs[:, None, 1] * e[None, :, 0]
        ap = a[None] - p[None, None]
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (ap[..., 0] * e[None, :, 1] - ap[..., 1] * e[None, :, 0]) / denom
            ss = (ap[..., 0] * dirs[:, None, 1] - ap[..., 1] * dirs[:, None, 0]) / denom
        valid = (tt > 1e-3) & (ss >= 0) & (ss <= 1) & np.isfinite(tt)
        tt = np.where(valid, tt, np.inf)
        k = tt.argmin(1)
        depth = tt[np.arange(W), k]                   # = Tiefe entlang optischer Achse
        seglen = np.linalg.norm(e, axis=1)
        u = ss[np.arange(W), k] * seglen[k]
        tex_id = self.segs[k, 4].astype(int)
        wall_h = np.where(tex_id == 2, self.scene.box_h, self.scene.wall_h)

        rows = np.arange(H) + 0.5
        beta = np.arctan2(rows - H / 2, f) - self.pitch   # Winkel unter Horizontal
        tb = np.tan(beta)
        img = np.empty((H, W, 3), np.uint8)
        img[:] = (200, 190, 180)                          # Decke/Himmel
        cam_h = self.cam_h
        # Wandhöhe je Pixel
        z = cam_h - depth[None, :] * tb[:, None]
        wall = (z >= 0) & (z <= wall_h[None, :])
        for tid in range(3):
            m = wall & (tex_id[None, :] == tid)
            if m.any():
                tx = self.tex[tid]
                ui = (u[None, :] * 100).astype(int) % tx.shape[1]
                vi = np.clip(((1 - z / wall_h[None, :]) * (tx.shape[0] - 1)).astype(int), 0, tx.shape[0] - 1)
                ui = np.broadcast_to(ui, z.shape)
                img[m] = tx[vi[m], ui[m]]
        # Boden
        with np.errstate(divide="ignore"):
            zf = np.where(tb > 1e-4, cam_h / tb, np.inf)  # Tiefe des Bodenpunkts je Zeile
        floor = zf[:, None] < depth[None, :]
        if floor.any():
            Z = np.broadcast_to(np.where(np.isfinite(zf), zf, 0.0)[:, None], (H, W))
            wx = self.x + Z * (fwd[0] + lat[None, :] * left[0])
            wy = self.y + Z * (fwd[1] + lat[None, :] * left[1])
            ix, iy = np.floor(wx * 12).astype(np.int64), np.floor(wy * 12).astype(np.int64)
            n = (((ix * 73856093) ^ (iy * 19349663)) & 15).astype(np.int16) - 8
            base = np.array([95, 120, 150], np.int16)  # Holzboden-ähnlich (BGR)
            fl = np.clip(base[None, None, :] + n[..., None], 0, 255).astype(np.uint8)
            img[floor] = fl[floor]
        box = None
        if self.person:
            box = self._draw_person(img, f, th, depth, W, H)
        img = cv2.resize(img, (self.cfg.width, self.cfg.height), interpolation=cv2.INTER_LINEAR)
        # Bewegungsunschärfe ~ Nickrate * Belichtungszeit (hier übertrieben, 8 ms)
        prate = (self.pitch - self._pitch_prev) / max(dt, 1e-3)
        self._pitch_prev = self.pitch
        blur_px = abs(prate) * 0.008 * (self.cfg.width / 2) / math.tan(math.radians(self.cfg.hfov_deg) / 2)
        if blur_px > 1.5:
            kz = int(min(15, blur_px)) | 1
            kern = np.zeros((kz, kz), np.float32)
            kern[:, kz // 2] = 1.0 / kz
            img = cv2.filter2D(img, -1, kern)
        if box is not None:
            box = tuple(v * self.rs for v in box)
        return img, box

    def _draw_person(self, img, f, th, depth, W, H):
        pr = self.person
        rel = np.array([pr.x - self.x, pr.y - self.y])
        Z = rel @ np.array([math.cos(th), math.sin(th)])
        X = rel @ np.array([-math.sin(th), math.cos(th)])
        if Z < 0.15:
            return None
        c = W / 2 - f * X / Z
        half = f * 0.25 / Z  # 0.50 m breit (Annahme im Code: 0.45 -> 10 % Bias)
        c1, c2 = int(math.floor(c - half)), int(math.ceil(c + half))

        def row(zw):
            return H / 2 + f * math.tan(math.atan2(self.cam_h - zw, Z) + self.pitch)

        r_top, r_bot = row(1.72), row(0.0)
        x1, x2 = max(0, c1), min(W, c2)
        y1, y2 = max(0, int(r_top)), min(H, int(math.ceil(r_bot)))
        if x2 <= x1 or y2 <= y1:
            return None
        vis = depth[x1:x2] > Z
        if not vis.any():
            return None
        hh = r_bot - r_top
        for yy in range(y1, y2):
            fr = (yy - r_top) / hh
            col = (150, 180, 220) if fr < 0.13 else (180, 60, 30) if fr < 0.55 else (50, 40, 40)
            seg = img[yy, x1:x2]
            seg[vis] = col
        vx = np.nonzero(vis)[0]
        bx1, bx2 = x1 + vx.min(), x1 + vx.max() + 1
        if (bx2 - bx1) < 0.3 * (c2 - c1):
            return None  # größtenteils verdeckt
        return (float(bx1), float(max(0, r_top)), float(bx2), float(min(H, r_bot)))


class SimImu:
    """IMU des Simulators: 50 Hz, Rauschen, absichtlich vertauschtes Vorzeichen,
    Grundneigung (ref) und Zeitstempelversatz – die App muss das selbst herausfinden."""

    def __new__(cls, *a, **k):
        from .imu import ImuSource

        class _SimImu(ImuSource):
            def __init__(self, world, cfg, sign=-1.0, ref_deg=2.5, t_err_ms=-8.0, noise_deg=0.05, hz=50.0):
                super().__init__(cfg)
                self.world, self.sgn, self.ref = world, sign, math.radians(ref_deg)
                self.t_err, self.noise, self.hz = t_err_ms / 1000, math.radians(noise_deg), hz
                self._seq = 0
                self._rng = np.random.default_rng(11)

            def _open(self) -> bool:
                return True

            def _run(self) -> None:
                while not self._stop.wait(1.0 / self.hz):
                    w = self.world
                    if w._t0 is None:
                        continue
                    now = time.monotonic()
                    raw = self.sgn * w.pitch_truth(now - w._t0) + self.ref + self.noise * self._rng.standard_normal()
                    self._seq += 1
                    self.add(self._seq, now + self.t_err, 0.0, raw)
                    self.rate_hz = self.hz

        return _SimImu(*a, **k)


class FakeDetector:
    """Ersetzt DetectorWorker: Ground Truth aus SimWorld, mit Inferenzlatenz und Rauschen."""

    def __init__(self, world: SimWorld, latency_s: float = 0.18, period_s: float = 0.2):
        self.world, self.lat, self.period = world, latency_s, period_s
        self.infer_ms = latency_s * 1000
        self.busy = False
        self._pending: list[tuple[float, DetectionBatch]] = []
        self._next = 0.0
        self._lock = threading.Lock()
        self._rng = np.random.default_rng(3)

    def start(self) -> bool:
        return True

    def close(self) -> None: ...

    def offer(self, frame, sharpness, median) -> None:
        now = time.monotonic()
        if now < self._next:
            return
        self._next = now + self.period
        tr = self.world.truth_by_t.get(frame.t, {})
        dets = []
        if tr.get("box") is not None:
            x1, y1, x2, y2 = tr["box"]
            n = self._rng.normal(0, 3, 4)
            dets.append(Detection((x1 + n[0], y1 + n[1], x2 + n[2], y2 + n[3]), "person", 0.85))
        with self._lock:
            self._pending.append((now + self.lat, DetectionBatch(frame.idx, frame.t, dets, self.lat * 1000)))

    def poll(self):
        now = time.monotonic()
        with self._lock:
            if self._pending and self._pending[0][0] <= now:
                return self._pending.pop(0)[1]
        return None
