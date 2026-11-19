"""Stimmproben (xgonav/stimme.py): Mitschnitt, Vergleich, Ablage, Sittich-Filter, C++ gegen numpy."""
import dataclasses
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from xgonav import stimme  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.ortung import Ortung  # noqa: E402

R = stimme.RATE_IN


def aeusserung(f0s, dur=1.0, stretch=1.0, amp=8000.0, seed=0):
    """Stimmhafte Silbenfolge: Grundton entlang f0s (Hz), 14 Obertöne, Hann-Hülle; int16 (n, 2)."""
    rng = np.random.default_rng(seed)
    n = int(dur * stretch * R)
    t = np.arange(n) / R
    f0 = np.interp(t, np.linspace(0, t[-1], len(f0s)), f0s)
    ph = 2 * np.pi * np.cumsum(f0) / R
    x = sum(np.sin(h * ph) / h for h in range(1, 15)) * np.hanning(n) * amp + rng.normal(0, 30, n)
    return np.stack([x, x], 1).astype(np.int16)


def stille(s, seed=1):
    x = np.random.default_rng(seed).normal(0, 30, int(s * R))
    return np.stack([x, x], 1).astype(np.int16)


WORT_A = [120, 180, 140, 220]
WORT_B = [250, 150, 260, 120]


def cfg(tmp_path, **kw):
    return dataclasses.replace(Config.neutral().stimme, dir=str(tmp_path), **kw)


def stimmen(tmp_path, native=False, **kw):
    return stimme.Stimmen(cfg(tmp_path, **kw), native=native)


# ---- Mitschnitt --------------------------------------------------------------
def mitschnitt(st, *teile, stumm_ab=None):
    """Strom durch die Ortung (numpy), fertige Segmente statt an den Thread in eine Liste."""
    segs = []
    st._submit = segs.append
    o = Ortung(Config.neutral().ortung, stumm=(lambda: stumm_ab is not None and o_t[0] >= stumm_ab), native=False)
    o.horcher = st
    st.aufnahme = True
    pcm = np.concatenate(teile)
    blk = o.cfg.block
    o_t = [0.0]
    for i in range(len(pcm) // blk):
        o_t[0] = (i + 1) * blk / R
        o.feed(pcm[i * blk:(i + 1) * blk], o_t[0])
    return segs


def test_mitschnitt_eine_aeusserung(tmp_path):
    st = stimmen(tmp_path)
    segs = mitschnitt(st, stille(2.0), aeusserung(WORT_A, 1.5), stille(1.0))
    assert len(segs) == 1
    assert 1.0 <= len(segs[0]) / R <= 1.5 + st.cfg.vad_pre_s + st.cfg.vad_hang_s + 0.1


def test_mitschnitt_zu_kurz_verworfen(tmp_path):
    st = stimmen(tmp_path)
    assert mitschnitt(st, stille(2.0), aeusserung(WORT_A, 0.2), stille(1.0)) == []


def test_mitschnitt_bei_max_s_abgeschnitten(tmp_path):
    st = stimmen(tmp_path)
    lang = np.concatenate([aeusserung(WORT_A, 1.0, amp=8000.0 + 0 * k, seed=k) for k in range(5)])
    segs = mitschnitt(st, stille(2.0), lang, stille(1.0))
    assert len(segs) >= 1 and len(segs[0]) / R <= st.cfg.max_s + st.cfg.vad_pre_s + 0.1


def test_eigener_ton_wird_nicht_mitgeschnitten(tmp_path):
    st = stimmen(tmp_path)
    assert mitschnitt(st, stille(2.0), aeusserung(WORT_A, 1.5), stille(1.0), stumm_ab=2.0) == []


def test_aus_ohne_aufnahme(tmp_path):
    st = stimmen(tmp_path)
    segs = []
    st._submit = segs.append
    o = Ortung(Config.neutral().ortung, native=False)
    o.horcher = st                                     # aufnahme bleibt aus
    pcm = np.concatenate([stille(2.0), aeusserung(WORT_A, 1.5), stille(1.0)])
    o.feed(pcm[:len(pcm) // 1024 * 1024], len(pcm) / R)
    assert segs == []


# ---- Vergleich und Ablage ------------------------------------------------------
def test_dtw_gleiches_wort_gestreckt_trifft():
    k = stimme.make_kern(0.3, native=False)
    fa, va = k.mfcc(k.mono16k(aeusserung(WORT_A)))
    fb, _ = k.mfcc(k.mono16k(aeusserung(WORT_A, stretch=1.2, seed=3)))
    fc, _ = k.mfcc(k.mono16k(aeusserung(WORT_B, seed=4)))
    m = Config.neutral().stimme.match_dist
    assert va > 0.9
    assert k.dtw(fa, fb, 0.25) < m < k.dtw(fa, fc, 0.25)


def test_rauschen_ist_keine_stimme(tmp_path):
    st = stimmen(tmp_path)
    x = np.random.default_rng(5).normal(0, 3000, int(1.2 * R))
    assert st.lerne(np.stack([x, x], 1).astype(np.int16)) == "verworfen"


def test_bestaetigen_filtern_und_wieder_hoeren(tmp_path):
    st = stimmen(tmp_path)
    assert st.lerne(aeusserung(WORT_A)) == "kandidat"
    assert st.lerne(aeusserung(WORT_B, seed=4)) == "kandidat"
    assert st.lerne(aeusserung(WORT_A, stretch=1.1, amp=12000.0, seed=6)) == "bestaetigt"
    assert st.ids == ("01",) and st.drain_neu() == ["01"] and st.drain_neu() == []
    assert len(st.kandidaten) == 1
    assert os.path.isfile(tmp_path / "bestaetigt" / "01.wav") and os.path.isfile(tmp_path / "bestaetigt" / "01_roh.wav")
    y, rate = stimme.read_wav(str(tmp_path / "bestaetigt" / "01.wav"))
    assert rate == stimme.RATE_OUT
    assert abs(np.abs(y).max() / 32767.0 - 10 ** (st.cfg.peak_db / 20)) < 0.01
    roh, _ = stimme.read_wav(str(tmp_path / "bestaetigt" / "01_roh.wav"))
    assert 0.3 < len(y) / stimme.RATE_OUT <= len(roh) / stimme.RATE / st.cfg.speed + 0.01   # Tor kürzt die Ränder
    assert st.lerne(aeusserung(WORT_A, seed=7)) == "wieder"           # keine zweite Probe desselben Worts
    assert len(st.kandidaten) == 1 and st.drain_neu() == ["01"]       # wiedererkannt: wieder abspielen


def test_pool_verdraengt_die_aelteste(tmp_path):
    st = stimmen(tmp_path, pool_n=3)
    worte = [[100, 300], [300, 100], [120, 280, 120], [280, 110, 280]]
    for k, w in enumerate(worte):
        st.lerne(aeusserung(w, seed=10 + k))
    assert len(st.kandidaten) == 3 and len(os.listdir(tmp_path / "kandidaten")) == 3


def test_voll_lernt_nichts_mehr_und_neustart_laedt(tmp_path):
    st = stimmen(tmp_path, max_confirmed=1)
    st.lerne(aeusserung(WORT_A))
    assert st.lerne(aeusserung(WORT_A, seed=2)) == "bestaetigt"
    assert st.voll and st.drain_neu() == ["01"]
    st.aufnahme = True
    assert st.aufnahme                                                # voll: erkennt weiter
    assert st.lerne(aeusserung(WORT_B)) == "voll" and st.kandidaten == []
    assert st.lerne(aeusserung(WORT_A, seed=3)) == "wieder" and st.drain_neu() == ["01"]
    neu = stimmen(tmp_path, max_confirmed=1)                          # Neustart
    assert neu.ids == ("01",) and neu.voll
    neu2 = stimmen(tmp_path, max_confirmed=2)
    neu2.lerne(aeusserung(WORT_B, seed=9))
    assert len(neu2.bestaetigt) == 1 and len(neu2.kandidaten) == 1


def test_alte_kandidaten_beim_start_geloescht(tmp_path):
    st = stimmen(tmp_path)
    st.lerne(aeusserung(WORT_A))
    f = next((tmp_path / "kandidaten").iterdir())
    alt = st.clock() - (st.cfg.kandidat_max_min + 1) * 60
    os.utime(f, (alt, alt))
    neu = stimmen(tmp_path)
    neu.laden()
    assert neu.kandidaten == [] and not f.exists()


def test_kandidaten_verfallen_nach_einer_minute(tmp_path):
    uhr = [1000.0]
    st = stimme.Stimmen(cfg(tmp_path), native=False, clock=lambda: uhr[0])
    assert st.cfg.kandidat_max_min == 1.0
    assert st.lerne(aeusserung(WORT_A)) == "kandidat"
    uhr[0] += 50
    assert st.lerne(aeusserung(WORT_B, seed=4)) == "kandidat"
    uhr[0] += 20                                                      # WORT_A jetzt 70 s alt
    assert st.lerne(aeusserung(WORT_A, seed=5)) == "kandidat"         # nicht mehr bestätigt
    assert len(st.kandidaten) == 2 and len(os.listdir(tmp_path / "kandidaten")) == 2
    uhr[0] += 61
    st._verfall()                                                     # der Thread macht das alle 5 s
    assert st.kandidaten == [] and os.listdir(tmp_path / "kandidaten") == []


def test_thread_raeumt_ohne_neue_aeusserung_auf(tmp_path, monkeypatch):
    monkeypatch.setattr(stimme, "VERFALL_TAKT_S", 0.05)
    uhr = [1000.0]
    st = stimme.Stimmen(cfg(tmp_path), native=False, clock=lambda: uhr[0])
    st._submit(aeusserung(WORT_A))
    for _ in range(100):
        if st.kandidaten:
            break
        time.sleep(0.02)
    assert len(st.kandidaten) == 1
    uhr[0] += 61
    for _ in range(100):
        if not st.kandidaten:
            break
        time.sleep(0.02)
    st.close()
    assert st.kandidaten == [] and os.listdir(tmp_path / "kandidaten") == []


def test_gefaerbtes_rauschen_ist_keine_stimme(tmp_path):
    """Wasserrauschen (am Rider 2026-11-19 gelernt): Rauschen mit abfallendem Spektrum, schwankend laut."""
    st = stimmen(tmp_path)
    rng = np.random.default_rng(8)
    n = int(1.5 * R)
    x = np.cumsum(rng.normal(0, 1, n))                               # 1/f^2
    x = x - np.convolve(x, np.ones(48) / 48, "same")                  # tiefe Frequenzen weg
    x *= 6000.0 / x.std() * (1.0 + 0.5 * np.sin(2 * np.pi * 3 * np.arange(n) / R))
    pcm = np.stack([x, x], 1).astype(np.int16)
    assert st.lerne(pcm) == "verworfen"
    assert st.lerne(pcm) == "verworfen"


# ---- C++ gegen numpy ---------------------------------------------------------
@pytest.mark.skipif(stimme._lib() is None, reason="C++-Kern fehlt (XGONAV_FASTLIB)")
def test_native_gleich_numpy():
    P, N = stimme.make_kern(0.3, native=False), stimme.make_kern(0.3, native=True)
    pcm = np.concatenate([stille(0.3), aeusserung(WORT_A, 1.3), stille(0.3)])
    a, b = P.mono16k(pcm), N.mono16k(pcm)
    assert len(a) == len(b) and np.abs(a - b).max() < 1e-6
    fa, va = P.mfcc(a)
    fb, vb = N.mfcc(a)
    assert fa.shape == fb.shape and np.abs(fa - fb).max() < 1e-9 and va == vb
    fc, _ = P.mfcc(P.mono16k(aeusserung(WORT_B, seed=4)))
    assert abs(P.dtw(fa, fc, 0.25) - N.dtw(fa, fc, 0.25)) < 1e-9
    ya, yb = P.sittich(a, 1.6, 1000, 5000, 25, 0.3, 2, 0.7), N.sittich(a, 1.6, 1000, 5000, 25, 0.3, 2, 0.7)
    assert len(ya) == len(yb) and np.abs(ya - yb).max() < 1e-9
    ta, tb = P.tor(a, 12.0, 6.0), N.tor(a, 12.0, 6.0)
    assert len(ta) == len(tb) and np.abs(ta - tb).max() < 1e-6


def test_rauschtor_schneidet_rauschen_ab():
    k = stimme.make_kern(0.3, native=False)
    x = k.mono16k(np.concatenate([stille(0.5), aeusserung(WORT_A, 0.6), stille(0.8)]))
    y = k.tor(x, 12.0, 6.0)
    assert 0.4 * stimme.RATE < len(y) < 0.8 * stimme.RATE                # nur der Satz bleibt
    roh = x[: int(0.4 * stimme.RATE)]
    assert np.sqrt(np.mean(y[:200] ** 2)) <= np.sqrt(np.mean(x ** 2))   # Anfang leiser als der Durchschnitt
    assert np.abs(roh).max() > 0
