"""Stimmproben im Modus pet: nachplappern wie ein Wellensittich.

Solange eine Person zu sehen ist (App setzt `aufnahme`), schneidet Stimmen.block() im Thread der Schallortung
Äußerungen mit (Sprecherkennung auf dem Pegel, den die Ortung ohnehin rechnet: Tor über dem Ruhepegel, Vorlauf,
Nachlauf). Eine fertige Äußerung geht an den eigenen Thread `stimme`:
  Stereo 48 kHz -> mono 16 kHz -> 13 MFCC je 10 ms (Mittelwert abgezogen), stille Ränder ab, Anteil stimmhafter
  Rahmen (spektral nicht flach und mit Grundton) -> sonst verworfen (Rauschen, Wasser, Klappern)
  DTW gegen alle Proben:  Treffer bei einer bestätigten -> jedes Mal nachplappern (drain_neu)
                          Treffer bei einer unbestätigten -> bestätigt (die lautere Aufnahme bleibt), Sittich-Filter,
                                                             Meldung an pet (drain_neu), das plappert sie gleich nach
                          kein Treffer -> neue unbestätigte Probe (höchstens pool_n, die älteste fliegt raus)
Ab max_confirmed bestätigten ersetzt eine neue Bestätigung die am seltensten gehörte (bei Gleichstand die am längsten
nicht gehörte); Zähler in bestaetigt/zaehler.json.

Dateien unter [stimme] dir: kandidaten/<Zeit>.wav (16 kHz roh, nach kandidat_max_min gelöscht), bestaetigt/NN_roh.wav und NN.wav (gefiltert, 48 kHz,
spielt emotion.Emotions.sound("stimme:NN")). Löschen = Verzeichnis leeren.

Kosten am Pi: je Audioblock (21 ms) nur bei `aufnahme` ein paar Vergleiche und das Merken des Blocks; je Äußerung
Merkmale und DTW im C++-Kern (native/stimmfast.cpp, GIL frei) im eigenen Thread, je Bestätigung einmal der Filter.
Ohne Kern dasselbe in numpy (gleich bis auf Rundung, deutlich langsamer). Nichts in der Bildschleife.
"""
from __future__ import annotations

import ctypes
import glob
import json
import logging
import math
import os
import threading
import time
import wave
from collections import deque
from dataclasses import dataclass

import numpy as np

from . import fastlib

log = logging.getLogger(__name__)

_STIMM_ABI = 5
RATE_IN, RATE, RATE_OUT = 48000, 16000, 48000
FRAME, HOP, NFFT, NMEL, NCEP = 400, 160, 512, 24, 13
FIR_N, FIR_FC = 31, 7000.0
MEL_LO, MEL_HI = 100.0, 7600.0
FLAT_LO, FLAT_HI = 300.0, 3000.0
LAG_LO, LAG_HI = 40, 228             # Grundton 400..70 Hz in Abtastwerten bei 16 kHz
WS_N, WS_HOP, WS_TOL = 480, 240, 160  # WSOLA: 30-ms-Stücke, Ausgabe-Takt 15 ms, Suche +-10 ms
VERFALL_TAKT_S = 5.0                 # so oft prüft der Thread stimme, ob Kandidaten verfallen sind
_lib_cache: list = []


def _lib():
    if not _lib_cache:
        L = fastlib.load()
        if not fastlib.check_abi(L, "st_abi", _STIMM_ABI):
            L = None
        else:
            p, i, d = ctypes.c_void_p, ctypes.c_int, ctypes.c_double
            L.st_mono16k.restype = i
            L.st_mono16k.argtypes = [p, i, p]
            L.st_new.restype = p
            L.st_new.argtypes = [d, d, d, d]
            L.st_free.restype = None
            L.st_free.argtypes = [p]
            L.st_mfcc.restype = i
            L.st_mfcc.argtypes = [p, p, i, p, i, p]
            L.st_dtw.restype = d
            L.st_dtw.argtypes = [p, i, p, i, i, d]
            L.st_dehne.restype = i
            L.st_dehne.argtypes = [p, i, d, p, i]
            L.st_tor.restype = i
            L.st_tor.argtypes = [p, i, d, d, p]
            L.st_sittich.restype = i
            L.st_sittich.argtypes = [p, i, d, d, d, d, d, d, d, p, i]
        _lib_cache.append(L)
    return _lib_cache[0]


# ---------------------------------------------------------------------- Tabellen (gleich in stimmfast.cpp)
def _fir() -> np.ndarray:
    j = np.arange(FIR_N)
    m = j - (FIR_N - 1) / 2
    k = 2.0 * FIR_FC / RATE_IN
    h = k * np.sinc(k * m) * (0.54 - 0.46 * np.cos(2.0 * np.pi * j / (FIR_N - 1)))
    return h / h.sum()


def _mel(f):
    return 2595.0 * np.log10(1.0 + f / 700.0)


def _melbank() -> np.ndarray:
    pts = np.linspace(_mel(MEL_LO), _mel(MEL_HI), NMEL + 2)
    hz = 700.0 * (10.0 ** (pts / 2595.0) - 1.0)
    f = np.arange(NFFT // 2 + 1) * RATE / NFFT
    fb = np.zeros((NMEL, NFFT // 2 + 1))
    for b in range(NMEL):
        lo, ce, hi = hz[b], hz[b + 1], hz[b + 2]
        up = (f > lo) & (f <= ce)
        dn = (f > ce) & (f < hi)
        fb[b, up] = (f[up] - lo) / (ce - lo)
        fb[b, dn] = (hi - f[dn]) / (hi - ce)
    return fb


def _dct() -> np.ndarray:
    n = np.arange(1, NCEP + 1)[:, None]
    m = np.arange(NMEL)[None, :]
    return np.cos(np.pi * n * (m + 0.5) / NMEL)


class _PyKern:
    """Dasselbe wie native/stimmfast.cpp in numpy."""
    name = "numpy"

    def __init__(self, flat_max: float, per_min: float = 0.5, trim_db: float = 12.0, laut_db: float = 10.0):
        self.flat_max, self.per_min = flat_max, per_min
        self.trim, self.laut = 10.0 ** (-trim_db / 10.0), 10.0 ** (-laut_db / 10.0)
        self.h = _fir()
        self.win = 0.54 - 0.46 * np.cos(2.0 * np.pi * np.arange(FRAME) / (FRAME - 1))
        rw = np.fft.irfft(np.abs(np.fft.rfft(self.win, NFFT)) ** 2, NFFT)
        self.rw = rw[LAG_LO:LAG_HI + 1] / rw[0]          # Autokorrelation des Fensters (Boersma)
        self.fb = _melbank()
        self.dct = _dct()
        self.k0 = math.ceil(FLAT_LO * NFFT / RATE)
        self.k1 = math.floor(FLAT_HI * NFFT / RATE)

    def mono16k(self, pcm: np.ndarray) -> np.ndarray:
        """pcm: int16 (n, 2) mit 48 kHz -> float64 mit 16 kHz (Tiefpass 7 kHz, jeder 3. Wert; Einheit wie int16)."""
        x = (pcm[:, 0].astype(np.float64) + pcm[:, 1]) * 0.5
        if len(x) < FIR_N:
            return np.zeros(0)
        return np.correlate(x, self.h, "valid")[::3].copy()

    def mfcc(self, x: np.ndarray) -> tuple[np.ndarray, float]:
        """(Merkmale (Rahmen, 13), Anteil stimmhafter unter den lauten Rahmen); stille Ränder abgeschnitten."""
        if len(x) < FRAME:
            return np.zeros((0, NCEP)), 0.0
        y = np.empty_like(x)
        y[0] = x[0]
        y[1:] = x[1:] - 0.97 * x[:-1]
        m = 1 + (len(y) - FRAME) // HOP
        idx = np.arange(m)[:, None] * HOP + np.arange(FRAME)[None, :]
        P = np.abs(np.fft.rfft(y[idx] * self.win, NFFT, axis=1)) ** 2
        e = P.sum(axis=1)
        if e.max() <= 0:
            return np.zeros((0, NCEP)), 0.0
        drin = e >= self.trim * e.max()                  # Ränder unter trim_db weg (Rauschen vor und nach dem Satz)
        a, b = int(np.argmax(drin)), m - int(np.argmax(drin[::-1]))
        P, laut = P[a:b], e[a:b] >= self.laut * e.max()  # stimmhaft gezählt nur bis laut_db unter dem lautesten
        band = P[:, self.k0:self.k1 + 1]
        flat = np.exp(np.log(band + 1e-10).mean(axis=1)) / (band.mean(axis=1) + 1e-10)
        r = np.fft.irfft(P, NFFT, axis=1)
        with np.errstate(divide="ignore", invalid="ignore"):
            per = (r[:, LAG_LO:LAG_HI + 1] / r[:, :1] / self.rw).max(axis=1)
        per = np.where(r[:, 0] > 0, per, 0.0)
        voiced = float(((flat < self.flat_max) & (per > self.per_min) & laut).sum()) / float(laut.sum())
        logE = np.log(np.maximum(P @ self.fb.T, 1e-10))
        c = logE @ self.dct.T
        return c - c.mean(axis=0), voiced

    @staticmethod
    def dtw(a: np.ndarray, b: np.ndarray, band: float) -> float:
        """Kosten je Schritt (Summe der Abstände durch na + nb) auf dem besten Pfad im Band, inf ohne Pfad."""
        na, nb = len(a), len(b)
        if na == 0 or nb == 0:
            return math.inf
        C = np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(axis=2))
        r = max(band * max(na, nb), 1.0)
        inf = math.inf
        prev = [inf] * nb
        for i in range(na):
            ce = i * (nb - 1) / (na - 1) if na > 1 else 0.0
            j0, j1 = max(0, math.ceil(ce - r)), min(nb - 1, math.floor(ce + r))
            cur = [inf] * nb
            ci = C[i]
            for j in range(j0, j1 + 1):
                if i == 0 and j == 0:
                    cur[j] = float(ci[0])
                    continue
                best = prev[j]
                if j > 0:
                    if prev[j - 1] < best:
                        best = prev[j - 1]
                    if cur[j - 1] < best:
                        best = cur[j - 1]
                if best < inf:
                    cur[j] = best + float(ci[j])
            prev = cur
        return prev[nb - 1] / (na + nb)

    @staticmethod
    def dehne(x: np.ndarray, alpha: float) -> np.ndarray:
        """Zeitlich alpha-mal so lang, Tonhöhe bleibt (WSOLA): je Ausgabe-Takt WS_HOP das 30-ms-Stück aus der Gegend
        k * WS_HOP / alpha (+-WS_TOL), das am besten an die natürliche Fortsetzung des vorigen passt; Hann, 50 %
        überlappt. Mit anschließend alpha-mal schnellerem Abspielen: alpha-mal höher bei gleichem Tempo."""
        n, N, Hs = len(x), WS_N, WS_HOP
        if n < N + Hs or alpha <= 0.0:
            return x.copy()
        win = 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(N) / N)
        L = int(math.floor(n * alpha + 0.5))             # wie lround in C++
        y = np.zeros(L + N)
        y[:N] += x[:N] * win
        prev, k = 0, 1
        while True:
            nom = int(math.floor(k * Hs / alpha + 0.5))
            if prev + Hs + N > n or nom + N > n or k * Hs + N > L + N:
                break
            tmpl = x[prev + Hs:prev + Hs + N]
            lo, hi = max(0, nom - WS_TOL), min(n - N, nom + WS_TOL)
            seg = x[lo:hi + N]
            r = np.array([float(np.dot(tmpl, seg[j:j + N])) for j in range(hi - lo + 1)])
            p = lo + int(np.argmax(r))
            y[k * Hs:k * Hs + N] += x[p:p + N] * win
            prev = p
            k += 1
        return y[:L].copy()

    @staticmethod
    def tor(x: np.ndarray, gate_db: float, knee_db: float) -> np.ndarray:
        """Rauschtor vor dem Sittich-Filter (16 kHz): Rahmenenergie (25 ms, 10 ms Takt) ab gate_db unter dem
        lautesten Rahmen auf knee_db weich zu, Verstärkung je Abtastwert linear zwischen den Rahmenmitten; Ränder ohne
        Verstärkung weg."""
        n = len(x)
        if n < FRAME:
            return x.copy()
        m = 1 + (n - FRAME) // HOP
        idx = np.arange(m)[:, None] * HOP + np.arange(FRAME)[None, :]
        e = (x[idx] * x[idx]).sum(axis=1)
        if e.max() <= 0.0:
            return np.zeros(0)
        ed = 10.0 * np.log10(np.maximum(e, 1e-30) / e.max())
        g = np.clip((ed + gate_db + knee_db) / max(knee_db, 1e-6), 0.0, 1.0)
        p = np.clip((np.arange(n) - FRAME / 2) / HOP, 0.0, m - 1)
        i0 = np.floor(p).astype(np.int64)
        i1 = np.minimum(i0 + 1, m - 1)
        fr = p - i0
        gs = g[i0] * (1.0 - fr) + g[i1] * fr
        nz = np.nonzero(gs > 0.0)[0]
        if len(nz) == 0:
            return np.zeros(0)
        return (x * gs)[nz[0]:nz[-1] + 1].copy()

    @staticmethod
    def sittich(x: np.ndarray, speed: float, lo: float, hi: float, vib_hz: float, vib_ms: float,
                drive: float, peak: float) -> np.ndarray:
        """16 kHz (Einheit egal) -> 48 kHz, Spitze `peak` (linear)."""
        n = len(x)
        if n < 2:
            return np.zeros(0)
        st = speed * RATE / RATE_OUT
        m = int(math.floor((n - 1) / st)) + 1
        pos = np.arange(m) * st
        i0 = np.minimum(np.floor(pos).astype(np.int64), n - 1)
        fr = pos - i0
        i1 = np.minimum(i0 + 1, n - 1)
        y = x[i0] * (1.0 - fr) + x[i1] * fr
        # Schnarren: Verzögerung D (1 + sin), linear dazwischen, vor dem Anfang still
        D = vib_ms * 1e-3 * RATE_OUT
        p = np.arange(m) - D * (1.0 + np.sin(2.0 * np.pi * vib_hz * np.arange(m) / RATE_OUT))
        ok = p >= 0.0
        q0 = np.minimum(np.floor(np.where(ok, p, 0.0)).astype(np.int64), m - 1)
        qf = np.where(ok, p, 0.0) - q0
        q1 = np.minimum(q0 + 1, m - 1)
        z = np.where(ok, y[q0] * (1.0 - qf) + y[q1] * qf, 0.0)
        for f0, hp in ((lo, True), (hi, False)):
            w0 = 2.0 * math.pi * f0 / RATE_OUT
            cw, al = math.cos(w0), math.sin(w0) / (2.0 * math.sqrt(0.5))
            if hp:
                b0, b1 = (1.0 + cw) / 2.0, -(1.0 + cw)
            else:
                b0, b1 = (1.0 - cw) / 2.0, 1.0 - cw
            a0 = 1.0 + al
            b0, b1, b2, a1, a2 = b0 / a0, b1 / a0, b0 / a0, -2.0 * cw / a0, (1.0 - al) / a0
            out = np.empty(m)
            x1 = x2 = y1 = y2 = 0.0
            for k in range(m):
                v = float(z[k])
                o = b0 * v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
                x2, x1, y2, y1 = x1, v, y1, o
                out[k] = o
            z = out
        mx = float(np.abs(z).max())
        if mx <= 0.0:
            return np.zeros(m)
        z = np.tanh(drive * (z / mx)) / math.tanh(drive)
        nf = min(m // 2, RATE_OUT // 100)
        if nf > 0:
            r = np.arange(nf) / nf
            z[:nf] *= r
            z[m - nf:] *= r[::-1]
        mx = float(np.abs(z).max())
        return z * (peak / mx) if mx > 0.0 else z


class _NativeKern:
    name = "native"

    def __init__(self, L, flat_max: float, per_min: float = 0.5, trim_db: float = 12.0, laut_db: float = 10.0):
        self._L = L
        self._h = L.st_new(flat_max, per_min, trim_db, laut_db)
        self._v = ctypes.c_double()

    def __del__(self):
        if getattr(self, "_h", None):
            self._L.st_free(self._h)
            self._h = None

    def mono16k(self, pcm: np.ndarray) -> np.ndarray:
        pcm = np.ascontiguousarray(pcm, dtype="<i2")
        out = np.empty(max(0, len(pcm) // 3 + 1))
        k = self._L.st_mono16k(pcm.ctypes.data, len(pcm), out.ctypes.data)
        return out[:k]

    def mfcc(self, x: np.ndarray) -> tuple[np.ndarray, float]:
        x = np.ascontiguousarray(x, dtype=np.float64)
        mx = max(1, len(x) // HOP + 1)
        out = np.empty((mx, NCEP))
        k = self._L.st_mfcc(self._h, x.ctypes.data, len(x), out.ctypes.data, mx, ctypes.byref(self._v))
        return out[:k].copy(), float(self._v.value)

    def dtw(self, a: np.ndarray, b: np.ndarray, band: float) -> float:
        a = np.ascontiguousarray(a, dtype=np.float64)
        b = np.ascontiguousarray(b, dtype=np.float64)
        return self._L.st_dtw(a.ctypes.data, len(a), b.ctypes.data, len(b), NCEP, band)

    def dehne(self, x: np.ndarray, alpha: float) -> np.ndarray:
        x = np.ascontiguousarray(x, dtype=np.float64)
        mx = int(math.floor(len(x) * alpha + 0.5)) + 1
        out = np.empty(mx)
        k = self._L.st_dehne(x.ctypes.data, len(x), alpha, out.ctypes.data, mx)
        return out[:k]

    def tor(self, x: np.ndarray, gate_db: float, knee_db: float) -> np.ndarray:
        x = np.ascontiguousarray(x, dtype=np.float64)
        out = np.empty(len(x))
        k = self._L.st_tor(x.ctypes.data, len(x), gate_db, knee_db, out.ctypes.data)
        return out[:k]

    def sittich(self, x, speed, lo, hi, vib_hz, vib_ms, drive, peak) -> np.ndarray:
        x = np.ascontiguousarray(x, dtype=np.float64)
        mx = int(len(x) * RATE_OUT / (RATE * speed)) + 2
        out = np.empty(mx)
        k = self._L.st_sittich(x.ctypes.data, len(x), speed, lo, hi, vib_hz, vib_ms, drive, peak, out.ctypes.data, mx)
        return out[:k]


def make_kern(flat_max: float, native: bool | None = None, per_min: float = 0.5, trim_db: float = 12.0,
              laut_db: float = 10.0):
    """native=None: C++-Kern, wenn vorhanden; True/False erzwingt einen Weg (Tests)."""
    L = _lib() if native is not False else None
    if L is not None:
        return _NativeKern(L, flat_max, per_min, trim_db, laut_db)
    if native:
        raise RuntimeError("stimme: C++-Kern fehlt")
    return _PyKern(flat_max, per_min, trim_db, laut_db)


def sittich_kette(kern, x: np.ndarray, c) -> np.ndarray:
    """Rohprobe (16 kHz) -> abspielfertig (48 kHz, Spitze peak_db): Rauschtor, um pitch_st Halbtöne höher bei
    unverändertem Tempo (dehnen, dann schneller abspielen), mal speed schneller, Sittich-Filter."""
    x = kern.tor(x, c.gate_db, c.knee_db)
    f = 2.0 ** (c.pitch_st / 12.0)
    if abs(f - 1.0) > 1e-6:
        x = kern.dehne(x, f)
    return kern.sittich(x, f * c.speed, c.band_lo, c.band_hi, c.vibrato_hz, c.vibrato_ms, c.drive,
                        10.0 ** (c.peak_db / 20.0))


# ---------------------------------------------------------------------- Dateien
def read_wav(path: str) -> tuple[np.ndarray, int]:
    """Mono oder Stereo int16 -> (float64 mono, Rate)."""
    with wave.open(path, "rb") as w:
        rate, ch = w.getframerate(), w.getnchannels()
        if w.getsampwidth() != 2:
            raise ValueError(f"{path}: nur 16 bit")
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return x, rate


def write_wav(path: str, x: np.ndarray, rate: int, scale: float = 1.0) -> None:
    pcm = np.clip(np.round(x * scale), -32768, 32767).astype("<i2")
    tmp = path + ".tmp"
    with wave.open(tmp, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm.tobytes())
    os.replace(tmp, path)


@dataclass
class Probe:
    name: str                        # Dateiname ohne .wav (Kandidat: Zeit, bestätigt: NN)
    x: np.ndarray                    # 16 kHz, Einheit int16, als int16 gespeichert
    feat: np.ndarray
    rms: float
    t: float = 0.0                   # Kandidat: angelegt (verfällt nach kandidat_max_min); bestätigt: zuletzt gehört
    n: int = 0                       # bestätigt: so oft gehört (Bestätigung + jedes Wiedererkennen)


class Stimmen:
    """Mitschnitt (block, im Ortungs-Thread), Lernen (eigener Thread) und Ablage."""

    def __init__(self, cfg, rate: int = RATE_IN, block: int = 1024, native: bool | None = None,
                 clock=time.time):
        self.cfg, self.clock = cfg, clock
        self.kern = make_kern(cfg.flat_max, native, cfg.per_min, cfg.trim_db, cfg.laut_db)
        self.ok = rate == RATE_IN
        if not self.ok:
            log.warning("Stimmen aus: Ortung läuft mit %d Hz, nötig sind %d", rate, RATE_IN)
        self.blk_s = block / float(rate)
        self.d_kand = os.path.join(cfg.dir, "kandidaten")
        self.d_best = os.path.join(cfg.dir, "bestaetigt")
        self._ids: tuple[str, ...] = tuple(self._ids_auf_platte())
        self.kandidaten: list[Probe] = []
        self.bestaetigt: list[Probe] = []
        self._geladen = False
        self._neu: deque = deque()
        self.n_segmente = self.n_verworfen = self.n_wieder = 0
        self.fehler: str | None = None
        self._aufnahme = False
        self._reset_tor()
        self._slot = None
        self._ev = threading.Event()
        self._ende = False
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ für App/pet
    @property
    def ids(self) -> tuple[str, ...]:
        """Bestätigte Proben, die abgespielt werden können (NN)."""
        return self._ids

    @property
    def voll(self) -> bool:
        return len(self._ids) >= self.cfg.max_confirmed

    @property
    def aufnahme(self) -> bool:
        return self._aufnahme

    @aufnahme.setter
    def aufnahme(self, on: bool) -> None:
        on = bool(on and self.ok and self.cfg.enabled)
        if on != self._aufnahme:
            self._reset_tor()
            self._aufnahme = on

    def drain_neu(self) -> list[str]:
        out = []
        while self._neu:
            out.append(self._neu.popleft())
        return out

    def datei(self, nn: str) -> str:
        return os.path.join(self.d_best, nn + ".wav")

    def status(self) -> dict:
        return {"aufnahme": self._aufnahme, "kern": self.kern.name, "bestaetigt": len(self._ids),
                "kandidaten": len(self.kandidaten) if self._geladen else None, "voll": self.voll,
                "segmente": self.n_segmente, "verworfen": self.n_verworfen, "wieder": self.n_wieder,
                "fehler": self.fehler}

    def close(self) -> None:
        self._ende = True
        self._ev.set()

    # ------------------------------------------------------------------ Mitschnitt (Ortungs-Thread)
    def _reset_tor(self) -> None:
        self._pre: deque = deque(maxlen=max(1, round(self.cfg.vad_pre_s / self.blk_s)))
        self._blocks: list | None = None
        self._laut = 0                 # Blöcke über dem Tor
        self._gap = 0
        self._still_noetig = False     # nach dem Abschneiden bei max_s erst wieder still werden

    def block(self, pcm: np.ndarray, lvl: float, noise: float, now: float, stumm: bool) -> None:
        """Ein Audioblock (int16 (n, 2)) mit Pegel und Ruhepegel aus der Ortung."""
        if not self._aufnahme:
            return
        c = self.cfg
        on = lvl > noise * 10.0 ** (c.vad_on_db / 10.0)
        an = lvl > noise * 10.0 ** (c.vad_off_db / 10.0)
        if stumm:                                       # eigener Ton: nichts davon behalten
            self._blocks, self._laut, self._gap = None, 0, 0
            self._pre.clear()
            return
        if self._blocks is None:
            if self._still_noetig:
                self._still_noetig = an
            elif on:
                self._blocks = list(self._pre) + [pcm.copy()]
                self._laut, self._gap = 1, 0
                self._pre.clear()
                return
            self._pre.append(pcm.copy())
            return
        self._blocks.append(pcm.copy())
        if an:
            self._laut += 1 + self._gap                 # Pausen im Sprechen zählen mit
            self._gap = 0
        else:
            self._gap += 1
        lang = self._laut * self.blk_s >= c.max_s
        if lang or self._gap * self.blk_s >= c.vad_hang_s:
            seg = self._blocks
            self._blocks, self._still_noetig = None, lang
            dauer = self._laut * self.blk_s
            self._laut = self._gap = 0
            if dauer >= c.min_s:
                self._submit(np.concatenate(seg))

    def _submit(self, pcm: np.ndarray) -> None:
        self.n_segmente += 1
        self._slot = pcm                                # nur die neueste zählt
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="stimme", daemon=True)
            self._thread.start()
        self._ev.set()

    def _run(self) -> None:
        while not self._ende:
            # Kandidaten verfallen auch ohne neue Äußerung; ohne Kandidaten schläft der Thread durch
            self._ev.wait(VERFALL_TAKT_S if self.kandidaten else None)
            self._ev.clear()
            seg, self._slot = self._slot, None
            if self._ende:
                break
            try:
                if seg is None:
                    self._verfall()
                    continue
                self.lerne(seg)
            except Exception as e:                      # darf nie etwas anderes stören
                self.fehler = str(e)
                log.warning("Stimmen: %s", e)

    # ------------------------------------------------------------------ Lernen (Thread stimme, Tests direkt)
    def lerne(self, pcm: np.ndarray) -> str:
        """Eine Äußerung (int16 (n, 2), 48 kHz). Rückgabe: "verworfen" | "wieder" | "kandidat" | "bestaetigt"."""
        c = self.cfg
        self.laden()
        self._verfall()
        x = self.kern.mono16k(pcm)
        feat, voiced = self.kern.mfcc(x)
        if len(feat) * HOP / RATE < 0.5 * c.min_s or voiced < c.voiced_min:
            self.n_verworfen += 1
            log.debug("Stimmen: verworfen (%.2f s, stimmhaft %.2f)", len(feat) * HOP / RATE, voiced)
            return "verworfen"
        art, probe, d = self._naechste(feat)
        if art == "bestaetigt":                         # wiedererkannt: jedes Mal nachplappern (Nutzer 2026-11-19)
            self.n_wieder += 1
            probe.n += 1
            probe.t = self.clock()
            self._zaehler_speichern()
            self._neu.append(probe.name)
            log.info("Stimmen: %s wieder gehört (Abstand %.1f, %d-mal)", probe.name, d, probe.n)
            return "wieder"
        rms = float(np.sqrt(np.mean(x * x))) if len(x) else 0.0
        if art == "kandidat":
            self._bestaetige(probe, Probe("", x, feat, rms), d)
            return "bestaetigt"
        jetzt = self.clock()
        name = time.strftime("%Y%m%d-%H%M%S", time.localtime(jetzt)) + "-%03d" % (int(jetzt * 1000) % 1000)
        while any(p.name == name for p in self.kandidaten) or os.path.exists(os.path.join(self.d_kand, name + ".wav")):
            name += "x"
        write_wav(os.path.join(self.d_kand, name + ".wav"), x, RATE)
        self.kandidaten.append(Probe(name, x.astype(np.int16), feat, rms, jetzt))
        while len(self.kandidaten) > c.pool_n:
            alt = self.kandidaten.pop(0)
            _rm(os.path.join(self.d_kand, alt.name + ".wav"))
        log.info("Stimmen: neue Probe %s (%.1f s, %d Proben, Abstand %.1f)", name, len(x) / RATE,
                 len(self.kandidaten), d)
        return "kandidat"

    def _naechste(self, feat: np.ndarray) -> tuple[str | None, Probe | None, float]:
        c, best = self.cfg, (None, None, math.inf)
        for art, liste in (("bestaetigt", self.bestaetigt), ("kandidat", self.kandidaten)):
            for p in liste:
                q = len(feat) / max(1, len(p.feat))
                if not 1.0 / c.len_ratio <= q <= c.len_ratio:
                    continue
                d = self.kern.dtw(feat, p.feat, c.dtw_band)
                if d < best[2]:
                    best = (art, p, d)
        if best[2] <= c.match_dist:
            return best
        return None, None, best[2]

    def _bestaetige(self, kand: Probe, neu: Probe, d: float) -> None:
        c = self.cfg
        keep = neu if neu.rms > kand.rms else kand
        frei = [f"{i:02d}" for i in range(1, c.max_confirmed + 1) if f"{i:02d}" not in self._ids]
        raus = ""
        if not frei:                                    # voll: die am seltensten gehörte fliegt raus (Nutzer 2026-11-19)
            alt = min(self.bestaetigt, key=lambda p: (p.n, p.t))
            raus = " statt %s (%d-mal gehört)" % (alt.name, alt.n)
            self.bestaetigt.remove(alt)
            self._ids = tuple(x for x in self._ids if x != alt.name)
            _rm(os.path.join(self.d_best, alt.name + "_roh.wav"))
            _rm(self.datei(alt.name))
            frei = [alt.name]
        nn = frei[0]
        x = keep.x.astype(np.float64)
        write_wav(os.path.join(self.d_best, nn + "_roh.wav"), x, RATE)
        self._filter(nn, x)
        self.kandidaten.remove(kand)
        _rm(os.path.join(self.d_kand, kand.name + ".wav"))
        self.bestaetigt.append(Probe(nn, keep.x.astype(np.int16), keep.feat, keep.rms, self.clock(), 1))
        self._ids = tuple(sorted(self._ids + (nn,)))
        self._zaehler_speichern()
        self._neu.append(nn)
        log.info("Stimmen: Probe %s bestätigt%s (Abstand %.1f, %d von %d)", nn, raus, d, len(self._ids),
                 c.max_confirmed)

    def _zaehler_speichern(self) -> None:
        """Wie oft und wann zuletzt je bestätigte Probe gehört (bestaetigt/zaehler.json), für das Ersetzen."""
        try:
            tmp = os.path.join(self.d_best, "zaehler.json.tmp")
            with open(tmp, "w") as f:
                json.dump({p.name: [p.n, p.t] for p in self.bestaetigt}, f)
            os.replace(tmp, os.path.join(self.d_best, "zaehler.json"))
        except OSError as e:
            log.warning("Stimmen: Zähler nicht gespeichert: %s", e)

    def _filter(self, nn: str, x: np.ndarray) -> None:
        write_wav(self.datei(nn), sittich_kette(self.kern, x, self.cfg), RATE_OUT, 32767.0)

    def _verfall(self) -> None:
        """Unbestätigte Proben älter als kandidat_max_min löschen."""
        grenze = self.clock() - self.cfg.kandidat_max_min * 60.0
        alt = [p for p in self.kandidaten if p.t < grenze]
        for p in alt:
            self.kandidaten.remove(p)
            _rm(os.path.join(self.d_kand, p.name + ".wav"))
        if alt:
            log.info("Stimmen: %d unbestätigte Proben verfallen", len(alt))

    def _ids_auf_platte(self) -> list[str]:
        out = []
        for f in sorted(glob.glob(os.path.join(self.d_best, "[0-9][0-9]_roh.wav"))):
            nn = os.path.basename(f)[:2]
            if len(out) < self.cfg.max_confirmed:
                out.append(nn)
        return out

    def laden(self) -> None:
        """Proben von der Platte (einmal, im Thread stimme): Merkmale neu, alte Kandidaten weg, fehlende Filter nach."""
        if self._geladen:
            return
        self._geladen = True
        os.makedirs(self.d_kand, exist_ok=True)
        os.makedirs(self.d_best, exist_ok=True)
        try:
            with open(os.path.join(self.d_best, "zaehler.json")) as f:
                zaehler = json.load(f)
        except (OSError, ValueError):
            zaehler = {}
        for nn in self._ids:
            try:
                x, rate = read_wav(os.path.join(self.d_best, nn + "_roh.wav"))
                feat, _ = self.kern.mfcc(x)
                n, t = zaehler.get(nn, [1, os.path.getmtime(os.path.join(self.d_best, nn + "_roh.wav"))])
                self.bestaetigt.append(Probe(nn, x.astype(np.int16), feat, float(np.sqrt(np.mean(x * x))), t, n))
                if not os.path.isfile(self.datei(nn)):
                    self._filter(nn, x)
            except Exception as e:
                log.warning("Stimmen: %s nicht lesbar: %s", nn, e)
        for f in sorted(glob.glob(os.path.join(self.d_kand, "*.wav"))):
            t = os.path.getmtime(f)
            if self.clock() - t > self.cfg.kandidat_max_min * 60.0:
                _rm(f)
                continue
            try:
                x, rate = read_wav(f)
                feat, _ = self.kern.mfcc(x)
                self.kandidaten.append(Probe(os.path.basename(f)[:-4], x.astype(np.int16), feat,
                                             float(np.sqrt(np.mean(x * x))), t))
            except Exception as e:
                log.warning("Stimmen: %s nicht lesbar: %s", f, e)
                _rm(f)
        while len(self.kandidaten) > self.cfg.pool_n:
            _rm(os.path.join(self.d_kand, self.kandidaten.pop(0).name + ".wav"))
        log.info("Stimmen: %d bestätigt, %d Proben (Kern %s)", len(self.bestaetigt), len(self.kandidaten),
                 self.kern.name)


def _rm(path: str) -> None:
    try:
        os.remove(path)
    except OSError:
        pass
