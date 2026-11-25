"""Closed-Loop-Simulator zum Testen ohne Hardware.

Raycasting-Renderer (Raum + Kisten + Personen-Billboard), Kamera auf
balancierendem Roboter: Nicken (Pendeln + Beschleunigungsneigung), Kurszittern
und vertikale Bewegungsunschärfe. Die Fahrbefehle des SimRobot bewegen die
Kamera. Ein FakeDetector liefert Ground-Truth-Boxen mit realistischer Latenz, SimFloorNet die
Bodengrenze aus der Ground Truth (statt FloorNet, das nur echte Bilder kennt).
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
from .mathe import w_to_yaw_rate, wrap
from .robot import FW_DEADBAND, SimRobot

SIM_NET_COLS = 80          # Spalten der Bodengrenze wie FloorNet (floornet.NCOLS)


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
    cls: str = "person"       # Klasse für FakeDetector; für Katze oder Hund height_m/width_m anpassen
    height_m: float = 1.72
    width_m: float = 0.50
    loop: bool = True         # False: bleibt am letzten Punkt
    vanish: bool = False      # mit loop False: am letzten Punkt verschwinden (aus dem Raum, nicht mehr zu sehen)

    def __post_init__(self):
        self.x, self.y = self.path[0]

    def step(self, t: float, dt: float) -> None:
        if not self.loop and self.seg >= len(self.path) - 1:
            if self.vanish:
                self.x, self.y = -50.0, -50.0
            return
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


# Nicken des balancierenden Riders, gemessen an den IMU-Proben (92 Hz) von sieben Rohfahrten am RiderPi
# (2026-11-10/11, zusammen ~1060 s, Fenster 4 s). Standardabweichung des Nickens in Fahrt, Median / 90 %:
# 0,5–1,5 Hz 0,94 / 2,10°, 1,5–4 Hz 0,33 / 0,94°, 4–8 Hz 0,12 / 0,26°, 8–15 Hz 0,05 / 0,16°; im Stand etwa
# ein Viertel bis ein Sechstel davon (0,16 / 0,09 / 0,04 / 0,02°). Stärkste Linie bei 0,5–0,75 Hz, nicht bei
# 2,5 Hz. Die Werte hier liegen zwischen Median und 90 % (Sinus-Amplitude = 1,41 x Standardabweichung).
# Bis 2026-11-11 standen hier 3,0° bei 2,5 Hz, 1,5° bei 6,1 Hz, 0,5° bei 9,3 Hz und 0,3° bei 13,1 Hz, auch im
# Stand: bei 2,5 Hz etwa sechsmal, bei 6 Hz etwa neunmal so viel wie gemessen.
NICK = ((0.8, 1.0), (2.5, 0.6), (6.1, 0.2), (9.3, 0.1), (13.1, 0.05))     # (Hz, Grad) in Fahrt
STAND_FACTOR = 0.25                                                        # Anteil davon im Stand
# Neigung beim Beschleunigen: gemessen 8,9° je m/s² (r = 0,98, Radwege gegen IMU), Beschleunigung p95 0,40 m/s².
# Begrenzt, weil der Simulator ohne --dyn die Geschwindigkeit sprunghaft ändert (sonst 20° und mehr).
ACC_DEG = 9.0
ACC_CLIP = 0.5


@dataclass
class SimScene:
    size: float = 6.0
    boxes: list = field(default_factory=lambda: [(2.0, 3.6, 2.7, 4.3), (4.2, 0.9, 4.8, 2.6)])
    wall_h: float = 1.0
    box_h: float = 0.6
    robot_r: float = 0.12              # Kollisionsradius (rund); der echte Rider ist 11,5 cm breit, 13,3 cm lang
    # Fugen im Boden (x1, y1, x2, y2), unsichtbar: Die Radachse kommt nur mit Fahrwert >= rille_v darüber,
    # sonst stehen Rider und Räder (RiderPi 2026-11-23: Fuge vor der Fensterstufe, bei 0,23 fest)
    rillen: list = field(default_factory=list)
    rille_v: float = 0.35

    def kreuzt_rille(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """Schneidet der Weg der Radachse (x0, y0) -> (x1, y1) eine Fuge?"""
        def seite(ax, ay, bx, by, px, py):
            return (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        for ax, ay, bx, by in self.rillen:
            if (seite(ax, ay, bx, by, x0, y0) * seite(ax, ay, bx, by, x1, y1) <= 0
                    and seite(x0, y0, x1, y1, ax, ay) * seite(x0, y0, x1, y1, bx, by) <= 0
                    and (x0, y0) != (x1, y1)):
                return True
        return False

    def segments(self):
        s = self.size
        seg = [(0, 0, s, 0, 0), (s, 0, s, s, 0), (s, s, 0, s, 1), (0, s, 0, 0, 1)]
        for i, (x1, y1, x2, y2) in enumerate(self.boxes):
            k = 2
            seg += [(x1, y1, x2, y1, k), (x2, y1, x2, y2, k), (x2, y2, x1, y2, k), (x1, y2, x1, y1, k)]
        return np.array(seg, np.float64)

    def collides(self, x: float, y: float, r: float | None = None) -> bool:
        r = self.robot_r if r is None else r
        if x < r or y < r or x > self.size - r or y > self.size - r:
            return True
        # runder Roboter: Abstand zum nächsten Punkt der Kiste (an den Ecken nicht quadratisch,
        # sonst zählt ein Vorbeifahren mit 3 cm Luft an einer Ecke als Kollision)
        return any(math.hypot(x - min(max(x, x1), x2), y - min(max(y, y1), y2)) < r
                   for (x1, y1, x2, y2) in self.boxes)


SCENES = {
    "raum": SimScene,
    # zwei Zimmer, dazwischen eine Wand mit 0,8 m breiter Tür (Wände als flache Kisten)
    "zwei": lambda: SimScene(boxes=[(3.0, 0.0, 3.1, 2.6), (3.0, 3.4, 3.1, 6.0), (1.2, 4.4, 1.9, 5.1), (4.3, 0.9, 4.9, 1.8)]),
    # wie "zwei", aber die Tür nur 25 cm breit; Kollisionsradius wie der echte Rider (halbe Länge 7 cm)
    "spalt": lambda: SimScene(boxes=[(3.0, 0.0, 3.1, 2.875), (3.0, 3.125, 3.1, 6.0), (1.2, 4.4, 1.9, 5.1),
                                     (4.3, 0.9, 4.9, 1.8)], robot_r=0.07),
    # wie "raum", dazu eine Fuge quer durch den Raum (x = 3) und rings um die rechte Kiste eine Fuge, an der die
    # Radachse 0,12 m Luft zur Kiste hat (wie vor der Fensterstufe)
    # rille_v 0,5: explore.v 0,4 des Riders bleibt hängen, push_v 0,45 + Korrektur des Temporeglers kommt drüber
    "fuge": lambda: SimScene(rillen=[(3.0, 0.0, 3.0, 6.0)] + _ring((4.2, 0.9, 4.8, 2.6), 0.24), rille_v=0.5),
}


def _ring(box, d):
    """Vier Fugen im Abstand d um eine Kiste."""
    x1, y1, x2, y2 = box
    a, b, c, e = x1 - d, y1 - d, x2 + d, y2 + d
    return [(a, b, c, b), (c, b, c, e), (c, e, a, e), (a, e, a, b)]


def scene_floor(scene: SimScene, x: np.ndarray, y: np.ndarray, margin: float = 0.0) -> np.ndarray:
    """Liegt der Punkt auf echtem Boden (im Raum, nicht in einer Kiste)? margin: Abstand zu den
    Rändern, den er mindestens haben muss."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = (x >= margin) & (y >= margin) & (x <= scene.size - margin) & (y <= scene.size - margin)
    for x1, y1, x2, y2 in scene.boxes:
        ok &= ~((x >= x1 - margin) & (x <= x2 + margin) & (y >= y1 - margin) & (y <= y2 + margin))
    return ok


def map_quality(lo: np.ndarray, res: float, l_show: float, scene: SimScene, start: tuple, path=None) -> dict:
    """Kartenqualität gegen die Wahrheit. lo: Log-Odds, start: (x, y, th) der Karten-Null in der
    Welt, path: gefahrene Punkte (Welt, für die Abdeckung nahe am Weg)."""
    n = lo.shape[0]
    x0, y0, th0 = start
    c0, s0 = math.cos(th0), math.sin(th0)

    def to_world(r, c):
        mx, my = (c + 0.5 - n / 2) * res, (r + 0.5 - n / 2) * res
        return x0 + mx * c0 - my * s0, y0 + mx * s0 + my * c0

    r, c = np.nonzero(lo > l_show)
    ox, oy = to_world(r, c)
    seg = scene.segments()
    ax, ay, bx, by = seg[:, 0], seg[:, 1], seg[:, 2], seg[:, 3]
    dx, dy = bx - ax, by - ay
    if len(ox):
        t = np.clip(((ox[:, None] - ax) * dx + (oy[:, None] - ay) * dy) / (dx * dx + dy * dy), 0, 1)
        d = np.hypot(ax + t * dx - ox[:, None], ay + t * dy - oy[:, None]).min(axis=1)
        prec = float((d < 0.15).mean())
    else:
        prec = float("nan")
    r, c = np.nonzero(lo < -l_show)
    fx, fy = to_world(r, c)
    # Eine Zelle Toleranz: eine Zelle auf der Wand gilt nicht als "frei hinter der Wand"
    wrong = ~scene_floor(scene, fx, fy, -res) if len(fx) else np.zeros(0, bool)
    # Wahrer Boden auf einem Raster: wie viel davon kennt die Karte als frei?
    g = np.arange(res / 2, scene.size, res)
    gx, gy = np.meshgrid(g, g)
    fl = scene_floor(scene, gx, gy)
    gx, gy = gx[fl], gy[fl]
    wx, wy = gx - x0, gy - y0
    mx, my = wx * c0 + wy * s0, -wx * s0 + wy * c0
    cc = np.floor(mx / res + n / 2).astype(np.int64)
    rr = np.floor(my / res + n / 2).astype(np.int64)
    inside = (rr >= 0) & (rr < n) & (cc >= 0) & (cc < n)
    known = np.zeros(len(gx), bool)
    known[inside] = lo[rr[inside], cc[inside]] < -l_show
    # Wandpunkte alle 5 cm: Anteil mit belegter Zelle in <= 10 cm, gesamt und <= 2 m vom Weg
    pts = []
    for ax_, ay_, bx_, by_, _ in seg:
        k = max(2, int(math.hypot(bx_ - ax_, by_ - ay_) / 0.05))
        pts += [(ax_ + (bx_ - ax_) * i / k, ay_ + (by_ - ay_) * i / k) for i in range(k + 1)]
    pts = np.array(pts)
    if len(ox):
        step = max(1, len(pts) * len(ox) // 4_000_000)
        dd = np.hypot(pts[::step, None, 0] - ox[None, :], pts[::step, None, 1] - oy[None, :]).min(axis=1)
        cover = float((dd <= 0.10).mean())
    else:
        dd, step, cover = None, 1, float("nan")
    recall = float("nan")
    if path is not None and dd is not None and len(path):
        p = np.asarray(path, float)[::5]
        near = np.hypot(pts[::step, None, 0] - p[None, :, 0], pts[::step, None, 1] - p[None, :, 1]).min(axis=1) <= 2.0
        if near.any():
            recall = float((dd[near] <= 0.10).mean())
    return {"praezision_15cm": round(prec, 3), "abdeckung_10cm": round(recall, 3), "waende_10cm": round(cover, 3),
            "frei_aussen_m2": round(float(wrong.sum()) * res * res, 2),
            "frei_innen_m2": round(float((~wrong).sum()) * res * res, 2),
            "boden_bekannt": round(float(known.mean()), 3),
            "belegt_m2": round(len(ox) * res * res, 2)}


class SimWorld(FrameSource):
    def __init__(self, cfg: CameraCfg, robot: SimRobot, person: Person | None = None,
                 start=(1.0, 1.0, 0.0), shake: float = 1.0, render_scale: int = 2, fps: int = 20,
                 vx_mps: float = 1.0, mount_offset_mm: float = 45.0, dynamics: bool = False,
                 scene: SimScene | None = None):
        super().__init__(cfg)
        self.robot = robot
        self.dynamics = dynamics
        # dunkle Zonen (x0, y0, x1, y1, Restlicht 0..1): Dunkelangst prüfen (tests/sim_run.py --dunkel)
        self.dark_zones: list[tuple[float, float, float, float, float]] = []
        self._vf = self._wf = 0.0
        self.scene = scene or SimScene()
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
        self._mot = STAND_FACTOR
        self.pitch = 0.0
        self._pitch_prev = 0.0
        self._rng = np.random.default_rng(7)
        # Radwege (m) aus dem Fahrbefehl: drehen auch weiter, wenn der Rider an einer Wand
        # hängt (Schlupf) – wie echte Encoder. Spurweite wie am RiderPi.
        self.track_m = 0.090
        self.wheel_l = self.wheel_r = 0.0
        # Bodengrenze je Bild aus der Ground Truth (SimFloorNet): (Bild, Zeilen) der letzten Bilder
        self.floor_rows: list[tuple[np.ndarray, np.ndarray]] = []

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
        img, box, rows = self._render(t, dt)
        self.floor_rows = (self.floor_rows + [(img, rows)])[-8:]
        self.truth = {"t": now, "box": box, "pitch": self.pitch}
        self.truth_by_t[now] = self.truth
        if len(self.truth_by_t) > 200:
            self.truth_by_t.pop(next(iter(self.truth_by_t)))
        self._publish(img, now)
        return True

    def _integrate(self, t: float, dt: float) -> None:
        vn, wn = self.robot.command
        v = vn * self.vx_mps
        w = w_to_yaw_rate(wn, self.robot.cfg)
        if self.dynamics:
            v, w = self._dyn(v, w, wn, dt)
        self.th += w * dt
        nx = self.x + v * math.cos(self.th) * dt
        ny = self.y + v * math.sin(self.th) * dt
        if self.scene.rillen and abs(vn) < self.scene.rille_v and self.scene.kreuzt_rille(self.x, self.y, nx, ny):
            # Rad hängt in der Fuge: Rider und Encoder stehen (nur Drehen geht)
            v, self._vf, nx, ny = 0.0, 0.0, self.x, self.y
            self.rille_steps = getattr(self, "rille_steps", 0) + 1
        self.wheel_l += (v - w * self.track_m / 2) * dt
        self.wheel_r += (v + w * self.track_m / 2) * dt
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
        self._acc_lp = 0.7 * self._acc_lp + 0.3 * float(np.clip(acc, -ACC_CLIP, ACC_CLIP))
        # Im Stand pendelt der Rider nur etwa ein Viertel so stark wie in Fahrt; Übergang über ~0,5 s
        ziel = 1.0 if abs(v) > 0.03 or abs(w) > 0.2 else STAND_FACTOR
        self._mot += (ziel - self._mot) * min(1.0, dt / 0.5)
        self.pitch = self.pitch_truth(t)
        self.yaw_jit = self.shake * self._mot * math.radians(0.8) * math.sin(2 * math.pi * 1.7 * t)
        cam_h = ((self.robot.height_mm or 85.0) + self.mount) / 1000.0
        self.cam_h = cam_h
        self.log.append({"t": t, "x": self.x, "y": self.y, "th": self.th, "v": v, "w": w,
                         "px": self.person.x if self.person else None,
                         "py": self.person.y if self.person else None})

    # Trägheit wie am RiderPi (dynamics=True; Messfahrt 2026-11-05, docs/limitations.md):
    # Drehbefehle unter 6/127,5 ignoriert die Firmware, die Drehung läuft ~0,3 s nach
    # (10–17° Überschwingen nach dem Stopp), und beim Anfahren rollt der balancierende Rider
    # erst kurz zurück (nichtminimalphasig; nur wenn er schneller wird, beim Bremsen auf 0 lief er 2026-11-23
    # nur so weit wie die Verzögerung allein ergibt).
    # Fahrt 2026-11-23 (Räder gegen Fahrwert, 0,1-0,7 m/s): Anfahren und Langsamerwerden folgen mit ~1,2 s
    # (angepasst 1,5-2 s, davon ~0,15 s Fenster der Radgeschwindigkeit), nur Fahrwert 0 (unter dem Totband)
    # bremst in ~0,25 s.
    DYN_TAU_V, DYN_TAU_STOP, DYN_TAU_W, DYN_NMP_S, DYN_W_DEAD = 1.2, 0.25, 0.3, 0.4, 6 / 127.5

    # Fahrrahmen (Firmware-Patch 08-velocity, robot.vel): Der Sollwert ist schon gerampt (robot.command = Generator der
    # Firmware), die Firmware regelt Tempo und Drehrate darauf; ohne Totband. Annahme bis zur Messfahrt: Nachlauf
    # 0,15 s (Tempo) / 0,1 s (Drehen), wie im Modellkreis von test_velocity_ctrl.cpp. Ohne Zurückrollen: Das Modell
    # dafür (DYN_NMP_S x Änderung) gehört zu Sprüngen im Fahrwert; bei einer Rampe zöge es die ganze
    # Beschleunigungsphase lang 0,4 s x a ab, die Vorlage baut sich mit der Vorsteuerung aber allmählich auf.
    DYN_TAU_VEL_V, DYN_TAU_VEL_W = 0.15, 0.1

    def _dyn(self, v: float, w: float, wn: float, dt: float) -> tuple[float, float]:
        if getattr(self.robot, "vel", False):
            self._vf += (v - self._vf) * (1.0 - math.exp(-dt / self.DYN_TAU_VEL_V))
            self._wf += (w - self._wf) * (1.0 - math.exp(-dt / self.DYN_TAU_VEL_W))
            return self._vf, self._wf
        if abs(wn) < self.DYN_W_DEAD:
            w = 0.0
        prev = self._vf
        tau = self.DYN_TAU_STOP if abs(v) < FW_DEADBAND * self.vx_mps else self.DYN_TAU_V   # unter dem Totband = 0
        self._vf += (v - self._vf) * (1.0 - math.exp(-dt / tau))
        v_eff = self._vf
        if abs(self._vf) > abs(prev):
            v_eff -= self.DYN_NMP_S * (self._vf - prev) / max(dt, 1e-3)
        self._wf += (w - self._wf) * (1.0 - math.exp(-dt / self.DYN_TAU_W))
        return v_eff, self._wf

    def pitch_truth(self, t: float) -> float:
        """Kameranicken (rad, + = hoch) zur Zeit t (s seit Start), zeitkontinuierlich:
        Balancier-Pendeln + Vibration + Neigung beim Beschleunigen. Amplituden siehe NICK."""
        return self.shake * (self._mot * sum(math.radians(a) * math.sin(2 * math.pi * f * t + i)
                                             for i, (f, a) in enumerate(NICK))
                             - math.radians(ACC_DEG) * self._acc_lp)

    def _render(self, t: float, dt: float):
        W, H = self.cfg.width // self.rs, self.cfg.height // self.rs
        f = (W / 2) / math.tan(math.radians(self.cfg.hfov_deg) / 2)
        th = self.th + self.yaw_jit
        cols = np.arange(W) + 0.5
        c0 = W / 2                                    # Spalte der Fahrtrichtung
        self._c0 = c0
        lat = (c0 - cols) / f                         # tan(Winkel) links +
        fwd = np.array([math.cos(th), math.sin(th)])
        left = np.array([-math.sin(th), math.cos(th)])
        dirs = fwd[None] + lat[:, None] * left[None]  # Strahlen (nicht normiert, Tiefe=1)
        # Strahl-Segment-Schnitt
        # Kamera sitzt cam_fwd_mm vor der Radachse (self.x/y = Achse, dort dreht und kollidiert der Rider)
        off = getattr(self.cfg, "cam_fwd_mm", 0.0) / 1000.0
        cx, cy = self.x + off * math.cos(self.th), self.y + off * math.sin(self.th)
        self._cam_xy = (cx, cy)
        p = np.array([cx, cy])
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
            wx = cx + Z * (fwd[0] + lat[None, :] * left[0])
            wy = cy + Z * (fwd[1] + lat[None, :] * left[1])
            ix, iy = np.floor(wx * 12).astype(np.int64), np.floor(wy * 12).astype(np.int64)
            n = (((ix * 73856093) ^ (iy * 19349663)) & 15).astype(np.int16) - 8
            base = np.array([95, 120, 150], np.int16)  # Holzboden-ähnlich (BGR)
            fl = np.clip(base[None, None, :] + n[..., None], 0, 255).astype(np.uint8)
            img[floor] = fl[floor]
        box = None
        obst = ~floor
        if self.person:
            box = self._draw_person(img, f, th, depth, W, H, obst)
        # Bodengrenze je Spalte wie FloorNet: unterste Zeile, ab der von unten gesehen kein Boden mehr ist
        # (Vollbild); je Netzspalte die nächste (größte Zeile) ihrer Bildspalten
        run = np.where(obst.any(axis=0), np.argmax(obst[::-1], axis=0), H)
        grenze = (H - run).astype(np.float32) * self.rs
        rows = grenze.reshape(SIM_NET_COLS, -1).max(axis=1)
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
        if self.dark_zones:
            k = self.light_factor()
            if k < 0.999:
                img = cv2.convertScaleAbs(img, alpha=k)
        if box is not None:
            box = tuple(v * self.rs for v in box)
        return img, box, rows

    def _licht_bei(self, x: float, y: float) -> float:
        k = 1.0
        for x0, y0, x1, y1, rest in self.dark_zones:
            d = min(x - x0, x1 - x, y - y0, y1 - y)       # > 0 innen
            tief = min(max(d / 0.3, 0.0), 1.0)              # 30 cm Übergang
            k = min(k, 1.0 - tief * (1.0 - rest))
        return k

    def light_factor(self) -> float:
        """Licht im Bild: wo die Kamera steht und wohin sie schaut (1,5 m voraus); aus einer dunklen Ecke
        heraus in den hellen Raum ist es heller als in die Ecke hinein."""
        cx, cy = getattr(self, "_cam_xy", (self.x, self.y))
        ax, ay = cx + 1.5 * math.cos(self.th), cy + 1.5 * math.sin(self.th)
        return 0.35 * self._licht_bei(cx, cy) + 0.65 * self._licht_bei(ax, ay)

    def _draw_person(self, img, f, th, depth, W, H, obst=None):
        pr = self.person
        cx, cy = self._cam_xy
        rel = np.array([pr.x - cx, pr.y - cy])
        Z = rel @ np.array([math.cos(th), math.sin(th)])
        X = rel @ np.array([-math.sin(th), math.cos(th)])
        if Z < 0.15:
            return None
        c = self._c0 - f * X / Z
        half = f * pr.width_m / 2 / Z  # Person 0.50 m breit (Annahme im Code: 0.45 -> 10 % Bias)
        c1, c2 = int(math.floor(c - half)), int(math.ceil(c + half))

        def row(zw):
            return H / 2 + f * math.tan(math.atan2(self.cam_h - zw, Z) + self.pitch)

        r_top, r_bot = row(pr.height_m), row(0.0)
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
            if obst is not None:
                obst[yy, x1:x2] |= vis
        vx = np.nonzero(vis)[0]
        bx1, bx2 = x1 + vx.min(), x1 + vx.max() + 1
        if (bx2 - bx1) < 0.3 * (c2 - c1):
            return None  # größtenteils verdeckt
        return (float(bx1), float(max(0, r_top)), float(bx2), float(min(H, r_bot)))


class SimImu:
    """IMU des Simulators: 50 Hz, Rauschen, absichtlich vertauschtes Vorzeichen,
    Grundneigung (ref) und Zeitstempelversatz – die App muss das selbst herausfinden.
    yaw_scale: Kurs zählt jede Drehung so viel mal (Skalenfehler des Kreisels, für den Abgleich am Startbild)."""

    def __new__(cls, *a, **k):
        from .imu import ImuSource

        class _SimImu(ImuSource):
            def __init__(self, world, cfg, sign=-1.0, ref_deg=2.5, t_err_ms=-8.0, noise_deg=0.05, hz=50.0,
                         yaw_scale=1.0):
                super().__init__(cfg)
                self.yaw_scale = yaw_scale
                self._th_alt = self._kurs = None
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
                    # liegend ruht die Lage (die Simulation kennt das Absetzen nur über den Zustand)
                    p = w.pitch_truth(now - w._t0) if w.robot.state.balancing else math.radians(60.0)
                    raw = self.sgn * p + self.ref + self.noise * self._rng.standard_normal()
                    self._seq += 1
                    if self._th_alt is None:
                        self._kurs = w.th
                    else:
                        self._kurs += self.yaw_scale * wrap(w.th - self._th_alt)
                    self._th_alt = w.th
                    self.add(self._seq, now + self.t_err, 0.0, raw, wrap(self._kurs))
                    self.rate_hz = self.hz

        return _SimImu(*a, **k)


class SimWheels:
    """Rad-Encoder des Simulators: 50 Hz, 1024 Ticks/U aus den Radwegen von SimWorld,
    mit optionalem Maßstabsfehler (scale, z. B. 1.03 = Rad 3 % größer als angenommen)."""

    def __new__(cls, *a, **k):
        from .wheels import WheelOdom

        class _SimWheels(WheelOdom):
            def __init__(self, world, cfg, hz=50.0, scale=1.0):
                super().__init__(cfg)
                self.world, self.hz, self.scale = world, hz, scale
                self._seq = 0

            def _open(self) -> bool:
                return True

            def _run(self) -> None:
                while not self._stop.wait(1.0 / self.hz):
                    w = self.world
                    if w._t0 is None:
                        continue
                    k = self.scale / self.m_per_tick
                    self._seq += 1
                    self.add(self._seq, time.monotonic(), int(w.wheel_l * k), int(w.wheel_r * k),
                             int((time.monotonic() - w._t0) * 1000))
                    self.rate_hz = self.hz

        return _SimWheels(*a, **k)


class SimFloorNet:
    """Ersetzt FloorNet im Simulator: Bodengrenze je Netzspalte (Vollbildzeile) aus der Ground Truth des gerenderten
    Bildes, Wände, Kisten und Person. Fehlalarme gibt es nur mit Phantoms."""

    def __init__(self, world: "SimWorld"):
        self.world = world

    def run(self, bgr: np.ndarray) -> np.ndarray:
        for img, rows in reversed(self.world.floor_rows):
            if img is bgr:
                return rows.copy()
        raise ValueError("SimFloorNet: Bild unbekannt (zu alt?)")


class Phantoms:
    """Fehlalarme des Bodenmodells, die der Simulator sonst nicht kennt: n ortsfeste Flecken (wie
    Spiegelungen oder Lichtpfützen), die aus manchen Blickwinkeln und nur zeitweise eine Bodengrenze
    vortäuschen. Wickelt SimFloorNet.run ein (Zeilen im Vollbild)."""

    def __init__(self, world: "SimWorld", net: SimFloorNet, geo, n: int = 6, seed: int = 3, radius: float = 0.15,
                 on_prob: float = 0.5):
        self.world, self.net, self.geo = world, net, geo
        rng = np.random.default_rng(seed)
        pts = []
        while len(pts) < n:
            x, y = rng.uniform(0.4, world.scene.size - 0.4, 2)
            if scene_floor(world.scene, x, y, 0.3):
                pts.append((float(x), float(y)))
        self.pts = pts
        self.radius, self.on_prob = radius, on_prob
        # Fleck wirkt nur aus einem Blickwinkelbereich (Spiegelung)
        self.dirs = rng.uniform(-math.pi, math.pi, n)
        self._rng = rng
        self.hits = 0
        self._run = net.run
        net.run = self.run

    def run(self, bgr: np.ndarray) -> np.ndarray:
        rows = self._run(bgr)
        w, geo = self.world, self.geo
        n = len(rows)
        bear = geo.bearing((np.arange(n) + 0.5) * geo.W / n)
        pitch = w.pitch                                   # Nicken des Bildes, wie gerendert
        cx, cy = getattr(w, "_cam_xy", (w.x, w.y))      # Kamera, nicht Radachse
        for (px, py), d0 in zip(self.pts, self.dirs):
            dx, dy = px - cx, py - cy
            dist = math.hypot(dx, dy)
            if dist < 0.25 or dist > 2.0 or self._rng.random() > self.on_prob:
                continue
            seen_from = math.atan2(-dy, -dx)
            if abs(wrap(seen_from - d0)) > math.radians(70):
                continue
            ang = wrap(math.atan2(dy, dx) - w.th)
            half = math.asin(min(1.0, self.radius / dist))
            cols = np.nonzero(np.abs(bear - ang) < half)[0]
            if not len(cols):
                continue
            fwd = dist * math.cos(ang)
            if fwd <= 0.05:
                continue
            y = geo.H / 2 + geo.f * math.tan(math.atan2(geo.cam_h, fwd) + math.radians(geo.tilt_deg) + pitch)
            if 0.0 <= y < geo.H:
                rows[cols] = np.fmax(rows[cols], y)       # näher = weiter unten im Bild
                self.hits += 1
        return rows


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

    backend = "sim"

    def start(self) -> bool:
        return True

    def close(self) -> None: ...

    def classes(self) -> list[str]:
        return [self.world.person.cls if self.world.person is not None else "person"]

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
            cls = self.world.person.cls if self.world.person is not None else "person"
            dets.append(Detection((x1 + n[0], y1 + n[1], x2 + n[2], y2 + n[3]), cls, 0.85))
        with self._lock:
            self._pending.append((now + self.lat, DetectionBatch(frame.idx, frame.t, dets, self.lat * 1000)))

    def poll(self):
        now = time.monotonic()
        with self._lock:
            if self._pending and self._pending[0][0] <= now:
                return self._pending.pop(0)[1]
        return None
