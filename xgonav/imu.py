"""Absolute Nicklage aus der ESP32-IMU (/proc/XGORider/imu_log, Treiber-Patch 0002).

Warum: Die Bildbewegung liefert nur schnelle Nickänderungen. Die langsame
Neigung beim Beschleunigen bleibt unsichtbar, verfälscht aber die
Fußpunkt-Entfernung stark (Kamera ~13 cm hoch: 1° ≈ 40 % bei 2 m).

Vorzeichen und Zeitversatz zwischen IMU- und Kamerazeitstempeln werden nicht
angenommen, sondern laufend geschätzt: Nickrate aus dem Bild gegen Nickrate
der IMU für verschiedene Versätze korrelieren. Erst bei eindeutiger
Korrelation wird die IMU benutzt (ImuSync.reliable).
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from collections import deque

import numpy as np

from .config import ImuCfg

log = logging.getLogger(__name__)


class ImuSource:
    """Hält eine Historie (t [s, CLOCK_MONOTONIC], roll, pitch, yaw [rad]) und interpoliert."""

    def __init__(self, cfg: ImuCfg, history_s: float = 20.0):
        self.cfg = cfg
        self._lock = threading.Lock()
        # Historie als numpy-Puffer, Zeilen t, roll, pitch, yaw (Kurs rad, links +, stetig ohne ±180°-Sprünge);
        # gültig sind die Spalten _a.._b. Angehängt wird hinten. Ist der Puffer voll, zieht der gültige Teil in
        # einen neuen um: Der alte bleibt, wie er ist, herausgegebene Ausschnitte (arrays()) gelten weiter. Mit
        # Deques kopierte arrays() je Bild die ganze Historie Wert für Wert (20 s bei 100 Hz: 4000 Werte).
        self._buf = np.empty((4, 4096))
        self._a = self._b = 0
        self._y_last = None                             # letzter Rohwert (rad) zum Entfalten
        self._hist = history_s
        self._last_seq = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.rate_hz = 0.0
        self.paused = False             # Ruhemodus: nicht lesen, nur selten aufwachen
        self.idle = False               # abgesetzt: Strom und Lesen nur mit cfg.idle_hz

    def start(self) -> bool:
        if not self._open():
            return False
        self._thread = threading.Thread(target=self._run, name="imu", daemon=True)
        self._thread.start()
        return True

    def set_idle(self, on: bool, force: bool = False) -> None:
        """Abgesetzt (on) nur noch cfg.idle_hz, sonst volle Rate. force schreibt die Rate auch ohne Wechsel
        (nach dem Ruhemodus, der die Treiber-Einstellung selbst zurückschreibt)."""
        on = bool(on) and getattr(self.cfg, "idle_hz", 0) > 0
        if on == self.idle and not force:
            return
        self.idle = on
        self._set_stream(self.cfg.idle_hz if on else self.cfg.stream_hz)

    def _wait_s(self) -> float:
        if self.paused:
            return 1.0
        if self.idle:
            return 1.0 / self.cfg.idle_hz
        return getattr(self.cfg, "poll_s", 0.05)

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)

    def add(self, seq: int, t: float, roll: float, pitch: float, yaw: float | None = None) -> None:
        """yaw: Firmware-Kurs (rad, links +) oder None. Sprünge über ±π werden entfaltet."""
        with self._lock:
            if seq <= self._last_seq:
                return
            self._last_seq = seq
            buf, a, b = self._buf, self._a, self._b
            y_alt = float(buf[3, b - 1]) if b > a else None
            if yaw is None:
                y = y_alt if y_alt is not None else 0.0
            elif self._y_last is None:
                y = yaw
            else:
                d = (yaw - self._y_last + math.pi) % (2 * math.pi) - math.pi
                y = (y_alt if y_alt is not None else yaw) + d
            if yaw is not None:
                self._y_last = yaw
            if b == buf.shape[1]:
                n = b - a
                neu = np.empty((4, max(4096, 2 * n + 1024)))
                neu[:, :n] = buf[:, a:b]
                self._buf, a, b = neu, 0, n
                buf = neu
            buf[:, b] = (t, roll, pitch, y)
            b += 1
            alt = t - self._hist
            while a < b and buf[0, a] < alt:            # je Probe meist ein Schritt
                a += 1
            self._a, self._b = a, b

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        """Zeiten und Nick der Historie (Ausschnitte des Puffers, nicht verändern)."""
        with self._lock:
            return self._buf[0, self._a:self._b], self._buf[2, self._a:self._b]

    def pitch_at(self, t) -> np.ndarray | float | None:
        ts, ps = self.arrays()
        if len(ts) < 2:
            return None
        return np.interp(t, ts, ps)

    def latest_t(self) -> float | None:
        with self._lock:
            return float(self._buf[0, self._b - 1]) if self._b > self._a else None

    def latest_tilt(self) -> tuple[float, float, float] | None:
        """(t, roll, pitch in rad) der neuesten Probe, sonst None."""
        with self._lock:
            return tuple(self._buf[:3, self._b - 1].tolist()) if self._b > self._a else None

    def since(self, t0: float) -> list[tuple[float, float, float, float]]:
        """Proben nach t0 als (t, roll, pitch, yaw in rad), älteste zuerst (für die Rohaufzeichnung)."""
        with self._lock:
            a, b = self._a, self._b
            alt = np.flatnonzero(self._buf[0, a:b] <= t0)          # ab der letzten Probe bis t0 ist alles neu
            k = a + (int(alt[-1]) + 1 if len(alt) else 0)
            rows = self._buf[:, k:b].T.tolist()
        return list(map(tuple, rows))

    def latest_yaw(self) -> tuple[float, float] | None:
        """(t, Kurs rad links +) der neuesten Probe mit Kurs, sonst None."""
        with self._lock:
            if self._b <= self._a or self._y_last is None:
                return None
            return float(self._buf[0, self._b - 1]), float(self._buf[3, self._b - 1])

    def _run(self) -> None:
        n0, t0 = 0, time.monotonic()
        while not self._stop.wait(self._wait_s()):
            if self.paused:
                self.rate_hz = 0.0
                n0, t0 = 0, time.monotonic()
                continue
            try:
                n = self._poll()
            except Exception:
                log.exception("IMU lesen fehlgeschlagen")
                n = 0
            n0 += n
            if time.monotonic() - t0 > 2.0:
                self.rate_hz = n0 / (time.monotonic() - t0)
                n0, t0 = 0, time.monotonic()

    def _open(self) -> bool: ...
    def _poll(self) -> int: return 0
    def _set_stream(self, hz: int) -> None: ...


class ProcImu(ImuSource):
    def _open(self) -> bool:
        self.path = os.path.join(self.cfg.proc_dir, "imu_log")
        if not os.path.exists(self.path):
            log.warning("%s fehlt – IMU aus (Treiber-Patch 0002 einspielen)", self.path)
            return False
        try:
            with open(os.path.join(self.cfg.proc_dir, "settings", "imu_hz"), "w") as f:
                f.write(str(self.cfg.imu_hz))
        except OSError as e:
            log.warning("imu_hz nicht gesetzt: %s", e)
        sp = os.path.join(self.cfg.proc_dir, "settings", "imu_stream_hz")
        if os.path.exists(sp):
            try:
                with open(sp, "w") as f:
                    f.write(str(int(self.cfg.stream_hz)))
                if self.cfg.stream_hz:
                    log.info("IMU-Strom %d Hz angefordert", self.cfg.stream_hz)
            except OSError as e:
                log.warning("imu_stream_hz nicht gesetzt: %s", e)
        elif self.cfg.stream_hz:
            log.warning("Treiber kennt keinen IMU-Strom (settings/imu_stream_hz fehlt) – Abfrage wie bisher")
        return True

    def _set_stream(self, hz: int) -> None:
        """Stromrate im Treiber umstellen; nur wenn der Strom genutzt wird (stream_hz > 0) und der Treiber ihn kennt."""
        sp = os.path.join(self.cfg.proc_dir, "settings", "imu_stream_hz")
        if not self.cfg.stream_hz or not os.path.exists(sp):
            return
        try:
            with open(sp, "w") as f:
                f.write(str(int(hz)))
            log.info("IMU-Strom %d Hz (%s)", hz, "abgesetzt" if self.idle else "balanciert")
        except OSError as e:
            log.warning("imu_stream_hz nicht gesetzt: %s", e)

    def _poll(self) -> int:
        with open(self.path) as f:
            lines = f.read().splitlines()
        # Puffer ist nach seq geordnet: von hinten nur die neuen Zeilen nehmen (sonst je Lesen ~128 Zeilen parsen)
        new = []
        for ln in reversed(lines):
            p = ln.split()
            if len(p) != 5:
                continue
            if int(p[0]) <= self._last_seq:
                break
            new.append(p)
        for p in reversed(new):
            # Felder: seq t_ns roll pitch yaw (Milligrad)
            self.add(int(p[0]), int(p[1]) / 1e9, math.radians(int(p[2]) / 1000), math.radians(int(p[3]) / 1000),
                     math.radians(int(p[4]) / 1000))
        return len(new)


class ImuSync:
    """Schätzt Vorzeichen und Zeitversatz IMU <-> Kamera aus Nickraten."""

    def __init__(self, cfg: ImuCfg, window: int = 300):
        self.cfg = cfg
        self.dpv: deque[float] = deque(maxlen=window)   # Bild: Nickänderung pro Frame (rad, + = hoch)
        self.t0: deque[float] = deque(maxlen=window)
        self.t1: deque[float] = deque(maxlen=window)
        self.offsets = np.arange(-cfg.sync_range_ms, cfg.sync_range_ms + 1e-9, cfg.sync_step_ms) / 1000.0
        self.sign = float(cfg.pitch_sign) if cfg.pitch_sign else 0.0
        self.offset = cfg.time_offset_ms / 1000.0 if cfg.time_offset_ms is not None else 0.0
        self.corr = 0.0
        self.slope = 0.0
        self.reliable = bool(cfg.pitch_sign) and cfg.time_offset_ms is not None
        self._last_eval = 0.0
        self.fixed = self.reliable

    def add(self, t_prev: float, t_cur: float, dpitch_vis: float) -> None:
        self.t0.append(t_prev)
        self.t1.append(t_cur)
        self.dpv.append(dpitch_vis)

    def update(self, imu: ImuSource, now: float) -> None:
        if self.fixed or now - self._last_eval < 1.0 or len(self.dpv) < 60:
            return
        self._last_eval = now
        v = np.asarray(self.dpv)
        if np.std(v) < math.radians(0.05):     # Rider steht völlig ruhig: keine Information
            return
        ts, ps = imu.arrays()
        a, b = np.asarray(self.t0), np.asarray(self.t1)
        if len(ts) < 20:
            return
        # nur Frame-Paare, die für alle Versätze innerhalb der IMU-Historie liegen
        m = (a + self.offsets.min() >= ts[0]) & (b + self.offsets.max() <= ts[-1])
        if m.sum() < 60:
            return
        v, a, b = v[m], a[m], b[m]
        best = (0.0, 0.0, 0.0)
        for o in self.offsets:
            d = np.interp(b + o, ts, ps) - np.interp(a + o, ts, ps)
            if np.std(d) < 1e-6:
                continue
            c = float(np.corrcoef(v, d)[0, 1])
            if abs(c) > abs(best[0]):
                best = (c, o, float(np.dot(v, d) / np.dot(d, d)))
        c, o, k = best
        self.corr, self.slope = c, k
        ok = abs(c) >= self.cfg.min_corr and 0.5 < abs(k) < 2.0   # Steigung ~1: gleiche Einheit
        if ok:
            self.sign = 1.0 if c > 0 else -1.0
            self.offset = o
        if ok != self.reliable:
            log.info("IMU-Sync %s: Vorzeichen %+d, Versatz %+.0f ms, Korrelation %.2f, Steigung %.2f",
                     "zuverlässig" if ok else "unsicher", self.sign, o * 1000, c, k)
        self.reliable = ok

    def pitch_dev(self, imu: ImuSource, t_frame: float) -> float | None:
        """Absolute Nickabweichung der Kamera (rad, + = schaut höher) ggü. Referenzlage."""
        if not self.reliable:
            return None
        lt = imu.latest_t()
        if lt is None or t_frame + self.offset - lt > self.cfg.max_age_s:
            return None
        p = imu.pitch_at(t_frame + self.offset)
        if p is None:
            return None
        # pitch_ref_deg = Rohwert der IMU bei der Kamerakalibrierung (tools/calibrate.py wall)
        return self.sign * (float(p) - math.radians(self.cfg.pitch_ref_deg))


def make_imu(cfg: ImuCfg) -> ImuSource | None:
    if not cfg.enabled:
        return None
    return ProcImu(cfg)
