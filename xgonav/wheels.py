"""Rad-Odometrie aus den Encodern der ESP32-Firmware (/proc/XGORider/wheels_log).

Der Treiber liest die Rad-Ticks (R-1.1.6+Patch, Register 0x0d bzw. 0x0c) mit ~50 Hz und
stempelt sie mit CLOCK_MONOTONIC: "seq t_ns ticks11 ticks21 fw_ms", 1024 Ticks je
Umdrehung, beide Räder vorwärts positiv, 11 = links, 21 = rechts.

Daraus: Strecke je Rad in Metern (fortlaufend, int32-Überlauf und ESP32-Neustarts
abgefangen), Strecke der Mitte und Drehung (rad, links +) zwischen zwei Zeitpunkten.
Ohne Treiberunterstützung (R-1.1.3) gibt es keine Raddaten; App fällt dann auf
"Fahrbefehl x vx_limit" zurück.
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from collections import deque

from .config import OdomCfg

log = logging.getLogger(__name__)


def _wrap32(d: int) -> int:
    """Differenz zweier int32-Zählerstände, über den Überlauf hinweg."""
    return (d + 2 ** 31) % 2 ** 32 - 2 ** 31


class WheelOdom:
    """Fortlaufende Radwege (m) mit Zeitstempel; Unterklassen liefern Rohproben."""

    def __init__(self, cfg: OdomCfg, history_s: float = 10.0):
        self.cfg = cfg
        self.m_per_tick = math.pi * cfg.wheel_diameter_m / cfg.ticks_per_rev
        self._lock = threading.Lock()
        self._hist: deque[tuple[float, float, float]] = deque()   # (t, s_links, s_rechts)
        self._history = history_s
        self._last: tuple[int, int, int] | None = None              # (ticks11, ticks21, fw_ms)
        self._sl = self._sr = 0.0
        self._last_seq = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.rate_hz = 0.0
        self.resets = 0                     # erkannte ESP32-Neustarts / Sprünge

    # ---- Lebenszyklus -------------------------------------------------
    def start(self) -> bool:
        if not self._open():
            return False
        self._thread = threading.Thread(target=self._run, name="wheels", daemon=True)
        self._thread.start()
        return True

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)

    def _run(self) -> None:
        n0, t0 = 0, time.monotonic()
        while not self._stop.wait(1.0 / self.cfg.poll_hz):
            try:
                n0 += self._poll()
            except OSError as e:
                log.warning("Raddaten nicht lesbar: %s", e)
                self._stop.wait(1.0)
            dt = time.monotonic() - t0
            if dt >= 2.0:
                self.rate_hz = n0 / dt
                n0, t0 = 0, time.monotonic()

    def _open(self) -> bool: ...
    def _poll(self) -> int: return 0

    # ---- Proben -------------------------------------------------------
    def add(self, seq: int, t: float, ticks_l: int, ticks_r: int, fw_ms: int = 0) -> None:
        with self._lock:
            if seq <= self._last_seq:
                return
            self._last_seq = seq
            if self._last is not None:
                dl, dr = _wrap32(ticks_l - self._last[0]), _wrap32(ticks_r - self._last[1])
                # ESP32-Neustart: Firmware-Zeit springt zurück (Ticks beginnen neu), oder ein
                # unplausibler Sprung (> max_jump_ticks zwischen zwei Proben) -> Delta verwerfen
                fw_back = fw_ms and self._last[2] and fw_ms + 1000 < self._last[2]
                if fw_back or abs(dl) > self.cfg.max_jump_ticks or abs(dr) > self.cfg.max_jump_ticks:
                    self.resets += 1
                    log.info("Raddaten: Sprung/Neustart erkannt (Δ %d/%d Ticks), Delta verworfen", dl, dr)
                else:
                    self._sl += dl * self.m_per_tick * self.cfg.scale_left
                    self._sr += dr * self.m_per_tick * self.cfg.scale_right
            self._last = (ticks_l, ticks_r, fw_ms)
            self._hist.append((t, self._sl, self._sr))
            while self._hist and self._hist[0][0] < t - self._history:
                self._hist.popleft()

    def latest(self) -> tuple[float, float, float] | None:
        """(t, s_links, s_rechts) der neuesten Probe."""
        with self._lock:
            return self._hist[-1] if self._hist else None

    def fresh(self, now: float) -> bool:
        p = self.latest()
        return p is not None and now - p[0] <= self.cfg.max_age_s

    def motion(self, a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float]:
        """Strecke der Radmitte (m, vorwärts +) und Drehung (rad, links +) von Probe a nach b."""
        dl, dr = b[1] - a[1], b[2] - a[2]
        return (dl + dr) / 2.0, (dr - dl) / self.cfg.track_m


def calibrate_scales(sl: float, sr: float, theta_ref: float, track_m: float,
                     dist_true: float | None = None) -> tuple[float, float]:
    """Faktoren je Rad aus einer Fahrt: sl/sr = gemessene Radwege (m, ohne Faktor),
    theta_ref = wahre Drehung (rad, links +; Kamera/IMU), dist_true = wahre Strecke der
    Mitte (m, Maßband) oder None = Mittelwert der Räder gilt als richtig. Löst
      (kl*sl + kr*sr) / 2 = D   und   (kr*sr - kl*sl) / track = theta_ref."""
    D = dist_true if dist_true is not None else (sl + sr) / 2
    if abs(sl) < 1e-6 or abs(sr) < 1e-6:
        raise ValueError("beide Räder müssen sich bewegt haben")
    kl = (D - theta_ref * track_m / 2) / sl
    kr = (D + theta_ref * track_m / 2) / sr
    return kl, kr


class ProcWheels(WheelOdom):
    def _open(self) -> bool:
        self.path = os.path.join(self.cfg.proc_dir, "wheels_log")
        if not os.path.exists(self.path):
            log.info("%s fehlt – keine Rad-Odometrie (Treiber/Firmware ohne Rad-Encoder)", self.path)
            return False
        try:
            with open(os.path.join(self.cfg.proc_dir, "serial_stats")) as f:
                stats = dict(ln.split() for ln in f if len(ln.split()) == 2)
            if stats.get("wheels") == "0":
                log.info("Firmware liefert keine Rad-Encoder – keine Rad-Odometrie")
                return False
        except OSError:
            pass
        return True

    def _poll(self) -> int:
        with open(self.path) as f:
            lines = f.read().splitlines()
        n = 0
        for ln in lines:
            p = ln.split()
            if len(p) < 4:
                continue
            seq = int(p[0])
            if seq <= self._last_seq:
                continue
            self.add(seq, int(p[1]) / 1e9, int(p[2]), int(p[3]), int(p[4]) if len(p) > 4 else 0)
            n += 1
        return n


def make_wheels(cfg: OdomCfg) -> WheelOdom | None:
    if not cfg.enabled or cfg.source == "command":
        return None
    return ProcWheels(cfg)
