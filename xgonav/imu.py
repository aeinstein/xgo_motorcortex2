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
from dataclasses import dataclass

import numpy as np

from .config import ImuCfg
from .mathe import wrap

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
        self._t_last = None                             # Zeit der letzten Probe mit Kurs
        self.yaw_jumps = 0                              # verworfene Kurssprünge (ESP-Neustart, Lesefehler)
        self._hist = history_s
        self._last_seq = 0
        self._poll_lock = threading.Lock()              # Faden und Bildschleife (refresh) lesen nie gleichzeitig
        self._t_poll = 0.0                              # Zeit des letzten Lesens (monoton)
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

    def refresh(self) -> int:
        """Neueste Proben sofort einlesen (Bildschleife direkt vor imu_frame): ein Lesen je Bild statt alle 10 ms
        im Faden, und der Wert zum Bild ist so frisch wie möglich. Liest gerade der Faden, nichts tun (er holt
        dieselben Proben). Im Ruhemodus und abgesetzt (idle_hz) nicht. Rückgabe: Anzahl neuer Proben."""
        if self.paused or self.idle or not self._poll_lock.acquire(blocking=False):
            return 0
        try:
            self._t_poll = time.monotonic()
            return self._poll()
        except Exception:
            log.debug("IMU lesen (refresh) fehlgeschlagen", exc_info=True)
            return 0
        finally:
            self._poll_lock.release()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)

    def add(self, seq: int, t: float, roll: float, pitch: float, yaw: float | None = None) -> None:
        """yaw: Firmware-Kurs (rad, links +) oder None. Sprünge über ±π werden entfaltet, Änderungen mal
        cfg.yaw_scale_left bzw. yaw_scale_right gerechnet (Skalenfehler des Gyros je Drehrichtung). Eine Änderung
        schneller als cfg.yaw_jump_dps ist kein Drehen (ESP-Neustart: Kurs beginnt bei 0) und wird verworfen; der
        Kurs läuft vom alten Wert stetig weiter."""
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
                d = wrap(yaw - self._y_last)
                lim = getattr(self.cfg, "yaw_jump_dps", 0.0)
                gap = t - self._t_last
                if lim > 0 and (gap > 0.5 or abs(d) > math.radians(lim) * max(gap, 0.02)):
                    # Über eine Lücke (ESP-Neustart: ~1 s keine Probe, danach Kurs ab 0; Ruhemodus) ist die Drehung
                    # unbekannt; ein zu schneller Sprung ist kein Drehen. Beides nicht übernehmen.
                    self.yaw_jumps += 1
                    log.info("IMU: Kursänderung %.1f° nach %.0f ms verworfen (Lücke oder Sprung)", math.degrees(d),
                             gap * 1000)
                    d = 0.0
                y = (y_alt if y_alt is not None else yaw) + d * (self.cfg.yaw_scale_left if d > 0 else self.cfg.yaw_scale_right)
            if yaw is not None:
                self._y_last, self._t_last = yaw, t
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

    def at(self, t: float) -> tuple[float, float, float | None, float] | None:
        """(Roll, Nick, Kurs in rad, Zeit der neuesten Probe) zur Zeit t, in einem Zug unter der Sperre; None mit
        weniger als 2 Proben. Kurs ist entfaltet (stetig), None, solange der Treiber keinen liefert."""
        with self._lock:
            a, b = self._a, self._b
            if b - a < 2:
                return None
            ts = self._buf[0, a:b]
            y = float(np.interp(t, ts, self._buf[3, a:b])) if self._y_last is not None else None
            return (float(np.interp(t, ts, self._buf[1, a:b])), float(np.interp(t, ts, self._buf[2, a:b])),
                    y, float(ts[-1]))

    def latest_yaw(self) -> float | None:
        """Kurs der neuesten Probe (rad, links +, entfaltet: alle Drehungen aufsummiert); None ohne Kurs."""
        with self._lock:
            return float(self._buf[3, self._b - 1]) if self._b > self._a and self._y_last is not None else None

    def fw_kurs_deg(self, ziel: float) -> float | None:
        """Kurs ziel (rad, im Kurs dieser Historie wie latest_yaw, Kursfaktoren eingerechnet) als Kurs der Firmware (°,
        auf ±180 gewickelt): für das absolute Kursziel (robot.vel_kursabs). Rohwert und Historie der neuesten Probe
        gehören zusammen; der Abstand wird durch den Kursfaktor der Drehrichtung zurückgerechnet. None ohne Kurs."""
        with self._lock:
            if self._b <= self._a or self._y_last is None:
                return None
            y, roh = float(self._buf[3, self._b - 1]), self._y_last
        d = ziel - y
        d /= self.cfg.yaw_scale_left if d > 0 else self.cfg.yaw_scale_right
        x = math.degrees(roh + d)
        return x - 360.0 * math.floor((x + 180.0) / 360.0)

    def latest_t(self) -> float | None:
        with self._lock:
            return float(self._buf[0, self._b - 1]) if self._b > self._a else None

    def since(self, t0: float) -> list[tuple[float, float, float, float]]:
        """Proben nach t0 als (t, roll, pitch, yaw in rad), älteste zuerst (für die Rohaufzeichnung)."""
        with self._lock:
            a, b = self._a, self._b
            alt = np.flatnonzero(self._buf[0, a:b] <= t0)          # ab der letzten Probe bis t0 ist alles neu
            k = a + (int(alt[-1]) + 1 if len(alt) else 0)
            rows = self._buf[:, k:b].T.tolist()
        return list(map(tuple, rows))

    def _run(self) -> None:
        # Läuft die Bildverarbeitung, liest die Bildschleife je Bild selbst (refresh); der Faden liest dann nur,
        # wenn seit dem letzten Lesen mehr als eine Wartezeit vergangen ist (Bildverarbeitung aus, Anzeige im Web).
        # Rate aus dem Fortschritt der Sequenznummer, gleich wer gelesen hat.
        seq0, t0 = self._last_seq, time.monotonic()
        while not self._stop.wait(self._wait_s()):
            now = time.monotonic()
            if self.paused:
                self.rate_hz = 0.0
                seq0, t0 = self._last_seq, now
                continue
            if now - self._t_poll >= 0.9 * self._wait_s():
                with self._poll_lock:
                    self._t_poll = now
                    try:
                        self._poll()
                    except Exception:
                        log.exception("IMU lesen fehlgeschlagen")
            if now - t0 > 2.0:
                self.rate_hz = max(0, self._last_seq - seq0) / (now - t0)
                seq0, t0 = self._last_seq, now

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


@dataclass(frozen=True)
class ImuFrame:
    """IMU zur Bildzeit, einmal je Bild in App._step gerechnet und von allen genutzt (Geometrie, Bodenmodell,
    Karten, Kippen, Overlay, Weboberfläche). t_imu = Bildzeit + [imu] time_offset_ms."""
    t_imu: float
    ok: bool                    # Probe jung genug und Vorzeichen/Versatz bekannt
    frisch: bool                # Probe jung genug (max_age); Roll, Kurs und Rohwerte gelten auch ohne Sync
    pitch: float | None         # Nick gegen die Referenzlage (rad, + = Kamera schaut höher), wie pitch_dev
    roll_deg: float             # Roll gegen waagerecht (Grad, + = rechts tief); 0 aus/ungültig
    yaw: float | None           # Kurs (rad, links +, entfaltet)
    pitch_raw_deg: float | None  # Rohwerte der IMU (Grad)
    roll_raw_deg: float | None
    alter_ms: float | None      # Bildzeit+Versatz minus neueste Probe (> 0: Wert gehalten, < 0: interpoliert)


def imu_frame(sync: "ImuSync", imu: ImuSource, t_frame: float) -> ImuFrame:
    """IMU-Werte zur Bildzeit t_frame (CLOCK_MONOTONIC, SensorTimestamp). Ungültig (ok False), wenn die neueste
    Probe älter als max_age ist oder Vorzeichen/Versatz unbekannt; dann pitch None, roll_deg 0."""
    t = t_frame + sync.offset
    w = imu.at(t)
    if w is None:
        return ImuFrame(t, False, False, None, 0.0, None, None, None, None)
    r, p, y, lt = w
    alter = t - lt
    frisch = alter <= max_age(sync.cfg, imu)
    ok = frisch and sync.reliable
    pitch = sync.sign * (p - math.radians(sync.cfg.pitch_ref_deg)) if ok else None
    roll = math.degrees(r) if frisch and sync.cfg.use_roll else 0.0
    return ImuFrame(t, ok, frisch, pitch, roll, y if frisch else None, math.degrees(p), math.degrees(r),
                    alter * 1000.0)


def max_age(cfg, imu) -> float:
    """Höchstes Alter der neuesten IMU-Probe, bis zu dem ein Bild sie noch nutzt. Abgesetzt (imu.idle) liefert
    die IMU nur idle_hz und wird so oft gelesen: die Probe ist dann bis 2 Abstände alt, mit max_age_s (0,3 s) fiel
    sonst bei 5 Hz jedes vierte Bild auf Nick 0 zurück (Horizontlinie sprang im Liegen, 2026-11-17)."""
    lim = cfg.max_age_s
    hz = getattr(cfg, "idle_hz", 0)
    if getattr(imu, "idle", False) and hz > 0:
        lim = max(lim, 2.5 / hz)
    return lim


def make_imu(cfg: ImuCfg) -> ImuSource | None:
    if not cfg.enabled:
        return None
    return ProcImu(cfg)
