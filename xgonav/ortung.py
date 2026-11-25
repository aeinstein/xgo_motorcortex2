"""Schallrichtung aus den zwei Mikrofonen (Stereo, WM8960): Laufzeitunterschied links/rechts aus der Phase
des Kreuzspektrums (GCC-PHAT im Band f_lo..f_hi), daraus der Winkel.

Ablauf je Block (block Abtastwerte, 21 ms bei 48 kHz):
  Pegel (Kern.level, eine Addition je Abtastwert) gegen den nachgeführten Ruhepegel -> Tor
  Tor zu:  sonst nichts
  Tor auf: Block ins Kreuzspektrum (eine FFT), nach window_blocks oder wenn das Tor länger als hang_blocks
           zu war: Laufzeit suchen -> Schall(winkel_deg, tau_us, guete), nur ab min_blocks und min_quality
Die Mikrofone liegen akustisch nur ~4 mm auseinander (Laufzeit höchstens +-11 us, ein Abtastwert sind 21 us):
brauchbar wird die Richtung erst über ~1 s gemittelt, also für Sprache und Dauergeräusche, nicht für ein Klatschen.
Gleichbleibender Lärm hebt den Ruhepegel (noise_up) und schließt das Tor nach einigen Sekunden wieder.
Eigene Töne (aplay) werden über `stumm` ausgeblendet, dazu nachhall_s danach.
Stimmproben (stimme.Stimmen, Modus pet) hängen als `horcher` am selben Strom: arecord hält hw:CARD=wm8960soundcard,DEV=0 allein.

Zwei Mikrofone trennen nur links/rechts: vorn und hinten sind nicht zu unterscheiden, der Winkel gilt gegen
die Querachse (0 = Mitte, > 0 links wie w > 0). mic_dist_m und tau_offset_us stammen aus einer Messreihe im
Stehen (docs/audio.md); liegend und mit Wänden in der Nähe weicht die linke Seite ab.

Kosten am Pi: arecord als eigener Prozess; hier ein Thread, der je Block einmal aufwacht. Gerechnet wird im
C++-Kern (native/ortfast.cpp, GIL frei), ohne ihn in numpy (gleich bis auf Rundung). Nichts in der Bildschleife.
"""
from __future__ import annotations

import ctypes
import logging
import math
import shutil
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

import numpy as np

from . import fastlib

log = logging.getLogger(__name__)

_ORT_ABI = 1
SCHALL_MPS = 343.0
VERLAUF_N, VERLAUF_S = 12, 15.0      # so viele Meldungen, höchstens so alt, stehen im Status
_lib_cache: list = []


def _lib():
    if not _lib_cache:
        L = fastlib.load()
        if not fastlib.check_abi(L, "ort_abi", _ORT_ABI):
            L = None
        else:
            p, i, d = ctypes.c_void_p, ctypes.c_int, ctypes.c_double
            L.ort_new.restype = p
            L.ort_new.argtypes = [i, d, d, d, d, i, d]
            L.ort_free.restype = None
            L.ort_free.argtypes = [p]
            L.ort_reset.restype = None
            L.ort_reset.argtypes = [p]
            L.ort_level.restype = d
            L.ort_level.argtypes = [p, i, i]
            L.ort_add.restype = None
            L.ort_add.argtypes = [p, p]
            L.ort_solve.restype = i
            L.ort_solve.argtypes = [p, p]
        _lib_cache.append(L)
    return _lib_cache[0]


class _PyKern:
    """Dasselbe wie native/ortfast.cpp in numpy. pcm: int16, Form (n, 2)."""
    name = "numpy"

    def __init__(self, n: int, rate: float, f_lo: float, f_hi: float, tau_max: float, ns: int, beta: float):
        self.n, self.ns, self.beta, self.tau_max = n, ns, beta, tau_max
        k0 = max(1, math.ceil(f_lo * n / rate))
        k1 = min(n // 2 - 1, math.floor(f_hi * n / rate))
        if n < 8 or n & (n - 1) or k1 < k0 or ns < 1:
            raise ValueError("ortung: Blocklänge (Zweierpotenz) oder Band ungültig")
        self.k0, self.k1 = k0, k1
        self.win = 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(n) / n)
        self.taus = tau_max * (np.arange(2 * ns + 1) - ns) / ns
        ph = 2.0 * np.pi * np.outer(self.taus, np.arange(k0, k1 + 1) * rate / n)
        self.stc, self.sts = np.cos(ph), np.sin(ph)
        self.reset()

    def reset(self) -> None:
        self.s = np.zeros(self.k1 - self.k0 + 1, np.complex128)
        self.blocks = 0

    @staticmethod
    def level(pcm: np.ndarray, dec: int) -> float:
        dec = max(1, dec)
        m = len(pcm) // dec
        s = pcm[:m * dec].sum(axis=1, dtype=np.int64).reshape(m, dec).sum(axis=1)
        if m < 2:
            return 0.0
        d = np.diff(s)
        return float((d * d).sum()) / (m - 1)

    def add(self, pcm: np.ndarray) -> None:
        x = np.fft.rfft(pcm[:self.n].astype(np.float64) * self.win[:, None], axis=0)
        self.s += x[self.k0:self.k1 + 1, 0] * np.conj(x[self.k0:self.k1 + 1, 1])
        self.blocks += 1

    def solve(self) -> tuple[int, float, float]:
        """(Blöcke, Laufzeit s, Güte); leert den Speicher."""
        blocks, s = self.blocks, self.s
        self.reset()
        m = np.abs(s)
        ok = m > 0
        if blocks == 0 or not ok.any():
            return blocks, 0.0, 0.0
        w = np.zeros_like(s)
        w[ok] = s[ok] * m[ok] ** -self.beta
        norm = float(np.abs(w).sum())
        r = self.stc @ w.real + self.sts @ w.imag
        best = int(np.argmax(r))
        off, peak = 0.0, float(r[best])
        if 0 < best < len(r) - 1:
            den = r[best - 1] - 2.0 * r[best] + r[best + 1]
            if den < 0.0:
                off = 0.5 * (r[best - 1] - r[best + 1]) / den
                peak = float(r[best] - 0.25 * (r[best - 1] - r[best + 1]) * off)
        return blocks, float(self.tau_max * (best - self.ns + off) / self.ns), peak / norm


class _NativeKern:
    name = "native"

    def __init__(self, L, n: int, rate: float, f_lo: float, f_hi: float, tau_max: float, ns: int, beta: float):
        self._L, self.n = L, n
        self._h = L.ort_new(n, rate, f_lo, f_hi, tau_max, ns, beta)
        if not self._h:
            raise ValueError("ortung: Blocklänge (Zweierpotenz) oder Band ungültig")
        self._out = (ctypes.c_double * 2)()

    def __del__(self):
        if getattr(self, "_h", None):
            self._L.ort_free(self._h)
            self._h = None

    def reset(self) -> None:
        self._L.ort_reset(self._h)

    def level(self, pcm: np.ndarray, dec: int) -> float:
        return self._L.ort_level(pcm.ctypes.data, len(pcm), dec)

    def add(self, pcm: np.ndarray) -> None:
        self._L.ort_add(self._h, pcm.ctypes.data)

    def solve(self) -> tuple[int, float, float]:
        blocks = self._L.ort_solve(self._h, self._out)
        return blocks, self._out[0], self._out[1]


def make_kern(n: int, rate: float, f_lo: float, f_hi: float, tau_max: float, ns: int, beta: float,
              native: bool | None = None):
    """native=None: C++-Kern, wenn vorhanden; True/False erzwingt einen Weg (Tests)."""
    L = _lib() if native is not False else None
    if L is not None:
        return _NativeKern(L, n, rate, f_lo, f_hi, tau_max, ns, beta)
    if native:
        raise RuntimeError("ortung: C++-Kern fehlt")
    return _PyKern(n, rate, f_lo, f_hi, tau_max, ns, beta)


@dataclass(frozen=True)
class Schall:
    t: float                 # monoton, s (Ende des ausgewerteten Fensters)
    winkel_deg: float        # 0 = Mitte (vorn oder hinten), > 0 links, < 0 rechts
    tau_us: float            # Laufzeitunterschied, > 0: links zuerst
    guete: float             # 0..1
    pegel_db: float          # über dem Ruhepegel
    blocks: int
    dauer_s: float = 1.0     # Zeit vom ersten gesammelten Block bis zur Meldung (Pausen eingerechnet)


class Ortung:
    """Tor, Sammeln und Auswerten. feed() nimmt beliebig lange Stücke (Tests, tools/ortung.py); am Rider
    liest ein Thread von arecord, solange set_active(True) gilt."""

    def __init__(self, cfg, stumm: Callable[[], bool] | None = None, native: bool | None = None):
        self.cfg, self._stumm = cfg, stumm
        self.tau_lim = cfg.mic_dist_m / SCHALL_MPS
        self.kern = make_kern(cfg.block, cfg.rate, cfg.f_lo, cfg.f_hi, self.tau_lim * cfg.tau_margin,
                              cfg.steer_steps, cfg.phat_beta, native)
        self.dec = max(1, round(cfg.rate / 8000.0))          # Tor-Band etwa 0,7..3,5 kHz
        self._on = 10.0 ** (cfg.on_db / 10.0)
        self._off = 10.0 ** (min(cfg.off_db, cfg.on_db) / 10.0)
        self.noise: float | None = None
        self._warm = cfg.warm_blocks
        self._n = 0                                          # gesammelte Blöcke
        self._gap = 0                                        # Blöcke mit geschlossenem Tor seit dem letzten offenen
        self._t0 = 0.0                                       # Beginn des laufenden Sammelns
        self._peak = 0.0
        self._next_t = 0.0                                   # frühestens dann wieder sammeln
        self._mute_until = 0.0
        self.last: Schall | None = None                      # wird ersetzt, nie verändert
        self.verlauf: deque = deque(maxlen=VERLAUF_N)        # letzte Meldungen für die Anzeige (Radar im Web)
        self.n_events = self.n_schwach = 0
        self.fehler: str | None = None
        self._want = threading.Event()
        self._ende = False
        self._proc: subprocess.Popen | None = None
        self._killed = False
        self._thread: threading.Thread | None = None
        self.horcher = None                                  # stimme.Stimmen: bekommt Blöcke, solange es aufnimmt

    # ------------------------------------------------------------------ Rechnen
    def feed(self, pcm: np.ndarray, now: float) -> list[Schall]:
        """pcm: int16 (n, 2), n ein Vielfaches von block. now: Zeit am Ende des Stücks."""
        c, out = self.cfg, []
        nb = len(pcm) // c.block
        for i in range(nb):
            s = self._block(pcm[i * c.block:(i + 1) * c.block], now - (nb - 1 - i) * c.block / c.rate)
            if s is not None:
                out.append(s)
        return out

    def _block(self, pcm: np.ndarray, now: float) -> Schall | None:
        c = self.cfg
        lvl = self.kern.level(pcm, self.dec)
        if self.noise is None:
            self.noise = lvl
        if self._warm > 0:                                   # Ruhepegel lernen, noch nichts melden
            self._warm -= 1
            self.noise += 0.2 * (lvl - self.noise)
            return None
        offen = lvl > self.noise * (self._off if self._n else self._on) and lvl > c.min_level
        self.noise += (c.noise_up if lvl > self.noise else c.noise_down) * (lvl - self.noise)
        h = self.horcher
        if h is not None and h.aufnahme:                     # Stimmproben (pet): Pegel ist schon gerechnet
            h.block(pcm, lvl, self.noise, now, self._ist_stumm(now))
        if offen and self._n == 0 and (now < self._next_t or self._ist_stumm(now)):
            return None
        if offen:
            if self._n == 0:
                self._t0 = now - c.block / c.rate
            self.kern.add(pcm)
            self._n += 1
            self._gap = 0
            self._peak = max(self._peak, lvl)
            if self._n < c.window_blocks:
                return None
        if self._n == 0:
            return None
        if not offen:
            self._gap += 1
            if self._gap <= c.hang_blocks:                   # kurze Pause: weiter sammeln
                return None
        n, peak = self._n, self._peak
        self._n, self._peak, self._gap = 0, 0.0, 0
        if n < c.min_blocks or self._ist_stumm(now):
            self.kern.reset()
            return None
        _, tau, guete = self.kern.solve()
        self._next_t = now + c.min_interval_s
        if guete < c.min_quality:
            self.n_schwach += 1
            return None
        tau = c.sign * (tau - c.tau_offset_us * 1e-6)
        sinus = max(-1.0, min(1.0, tau / self.tau_lim))
        s = Schall(t=now, winkel_deg=math.degrees(math.asin(sinus)), tau_us=tau * 1e6, guete=guete,
                   pegel_db=10.0 * math.log10(peak / max(self.noise, 1e-9)), blocks=n, dauer_s=now - self._t0)
        self.last = s
        self.verlauf.append(s)
        self.n_events += 1
        return s

    def _ist_stumm(self, now: float) -> bool:
        if self._stumm is not None and self._stumm():
            self._mute_until = now + self.cfg.nachhall_s
        return now < self._mute_until

    def status(self, now: float | None = None) -> dict:
        s = self.last
        now = time.monotonic() if now is None else now
        return {"aktiv": self._want.is_set(), "kern": self.kern.name, "fehler": self.fehler, "n": self.n_events,
                "schwach": self.n_schwach,
                # [Alter s, Winkel °, Güte, Pegel dB], älteste zuerst
                "verlauf": [[round(now - v.t, 2), round(v.winkel_deg, 1), round(v.guete, 2), round(v.pegel_db, 1)]
                            for v in list(self.verlauf) if now - v.t <= VERLAUF_S],
                "letzter": None if s is None else {"alter_s": round(now - s.t, 2), "winkel_deg": round(s.winkel_deg, 1),
                                                   "tau_us": round(s.tau_us, 1), "guete": round(s.guete, 2),
                                                   "pegel_db": round(s.pegel_db, 1)},
                "stimme": self.horcher.status() if self.horcher is not None else None}

    # ------------------------------------------------------------------ Aufnahme (nur am Rider)
    def set_active(self, on: bool) -> None:
        """An: arecord läuft und der Thread rechnet. Aus: arecord beendet, der Thread schläft (kein Aufwachen)."""
        if on == self._want.is_set():
            return
        if on:
            self._want.set()
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="ortung", daemon=True)
                self._thread.start()
        else:
            self._want.clear()
            self._kill()

    def close(self) -> None:
        self._ende = True
        self._want.set()
        self._kill()

    def _kill(self) -> None:
        p = self._proc
        self._killed = True
        if p is not None and p.poll() is None:
            p.terminate()

    def _run(self) -> None:
        c = self.cfg
        nbytes = c.block * 4
        exe = shutil.which("arecord")
        if exe is None:
            self.fehler = "arecord fehlt"
            log.warning("Ortung aus: arecord fehlt")
            return
        cmd = [exe, "-q", "-D", c.device, "-f", "S16_LE", "-r", str(c.rate), "-c", "2", "-t", "raw"]
        while not self._ende:
            self._want.wait()
            if self._ende:
                break
            self._killed = False
            try:
                self._proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                              stderr=subprocess.DEVNULL, bufsize=nbytes)
            except OSError as e:
                self.fehler = str(e)
                log.warning("Ortung: arecord startet nicht: %s", e)
                time.sleep(c.retry_s)
                continue
            log.info("Ortung an (%s, %d Hz, Kern %s)", c.device, c.rate, self.kern.name)
            self.noise, self._warm, self._n, self._gap = None, c.warm_blocks, 0, 0
            self.kern.reset()
            gelesen = False
            while self._want.is_set() and not self._ende:
                data = self._proc.stdout.read(nbytes)        # blockiert, GIL frei
                if len(data) < nbytes:
                    break
                gelesen = True
                try:
                    self._block(np.frombuffer(data, dtype="<i2").reshape(-1, 2), time.monotonic())
                except Exception as e:                       # darf nie etwas anderes stören
                    log.warning("Ortung: %s", e)
            selbst = not self._killed and not self._ende     # arecord von selbst beendet (Gerät belegt?)
            self._kill()
            self._proc.stdout.close()
            self._proc.wait()
            self._proc = None
            if selbst:
                self.fehler = "arecord beendet" if gelesen else f"arecord liefert nichts ({c.device})"
                log.warning("Ortung: %s, neuer Versuch in %.0f s", self.fehler, c.retry_s)
                time.sleep(c.retry_s)
            else:
                self.fehler = None
                log.info("Ortung aus")
