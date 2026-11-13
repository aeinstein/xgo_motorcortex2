"""Schallrichtung (xgonav/ortung.py): Laufzeit aus der Phase, Tor, Ruhepegel, C++-Kern gegen numpy."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from xgonav import ortung
from xgonav.config import Config
from xgonav.ortung import Ortung, make_kern

RATE = 48000
RAUSCHEN = 1500.0        # Effektivwert in LSB, etwa -27 dBFS wie am Rider gemessen (docs/audio.md)


def _cfg(**kw):
    c = Config.neutral().ortung
    # Grundtests mit weit auseinander liegenden Mikrofonen und kurzen Fenstern (Prüfsignale bis 110 us)
    c.mic_dist_m, c.tau_offset_us, c.f_lo, c.f_hi, c.tau_margin = 0.04, 0.0, 500.0, 5000.0, 1.25
    c.window_blocks, c.min_blocks, c.hang_blocks, c.min_interval_s = 6, 2, 0, 0.25
    c.noise_up, c.off_db = 0.01, c.on_db
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def _rauschen(rng, n):
    return rng.normal(0.0, RAUSCHEN, (n, 2))


def _quelle(rng, n, tau_s, pegel=8000.0, f=(300.0, 6000.0)):
    """Bandbegrenztes Rauschen; der rechte Kanal ist um tau_s verzögert (tau > 0: links zuerst)."""
    spec = np.fft.rfft(rng.normal(0.0, 1.0, n))
    fr = np.fft.rfftfreq(n, 1.0 / RATE)
    spec[(fr < f[0]) | (fr > f[1])] = 0.0
    li = np.fft.irfft(spec, n)
    re = np.fft.irfft(spec * np.exp(-2j * np.pi * fr * tau_s), n)
    x = np.stack([li, re], axis=1)
    return x * (pegel / x.std())


def _pcm(x):
    return np.clip(np.round(x), -32768, 32767).astype(np.int16)


def _lauf(o, rng, tau_s, dauer_s=0.4, stille_s=1.0, pegel=8000.0, t0=0.0):
    """Stille, dann ein Geräusch, dann Stille. Gibt die gemeldeten Ereignisse zurück."""
    b = o.cfg.block
    n0, n1 = int(stille_s * RATE) // b * b, int(dauer_s * RATE) // b * b
    x = _rauschen(rng, n0 + n1 + n0)
    x[n0:n0 + n1] += _quelle(rng, n1, tau_s, pegel)
    return o.feed(_pcm(x), t0 + len(x) / RATE)


@pytest.mark.parametrize("tau_us", [-100.0, -60.0, -20.0, 0.0, 35.0, 80.0, 110.0])
def test_laufzeit_und_seite(tau_us):
    o = Ortung(_cfg(), native=False)
    ev = _lauf(o, np.random.default_rng(1), tau_us * 1e-6)
    assert ev, "Geräusch nicht gemeldet"
    for s in ev:
        assert abs(s.tau_us - tau_us) < 6.0, (s.tau_us, tau_us)
        assert s.guete > 0.6
    if abs(tau_us) > 10:
        assert all((s.winkel_deg > 0) == (tau_us > 0) for s in ev)      # > 0 = links zuerst = links
    want = np.degrees(np.arcsin(tau_us * 1e-6 * ortung.SCHALL_MPS / o.cfg.mic_dist_m))
    assert abs(np.median([s.winkel_deg for s in ev]) - want) < 5.0


def test_sign_vertauscht():
    ev = _lauf(Ortung(_cfg(sign=-1), native=False), np.random.default_rng(2), 60e-6)
    assert ev and all(s.winkel_deg < 0 and s.tau_us < 0 for s in ev)


def test_stille_meldet_nichts():
    o = Ortung(_cfg(), native=False)
    rng = np.random.default_rng(3)
    assert o.feed(_pcm(_rauschen(rng, 10 * RATE // 1024 * 1024)), 10.0) == []
    assert o.n_events == 0


def test_lauter_ohne_richtung_wird_verworfen():
    """Lauter, aber auf beiden Kanälen unabhängig (keine gemeinsame Quelle): Güte zu klein."""
    o = Ortung(_cfg(), native=False)
    rng = np.random.default_rng(4)
    x = _rauschen(rng, 2 * 48 * 1024)
    x[48 * 1024:60 * 1024] *= 6.0
    assert o.feed(_pcm(x), 2.0) == []
    assert o.n_schwach >= 1


def test_dauerlaerm_schliesst_das_tor():
    """Gleichbleibender Lärm: erst Meldungen, dann hebt sich der Ruhepegel und es wird nichts mehr gerechnet."""
    o = Ortung(_cfg(), native=False)
    rng = np.random.default_rng(5)
    n0, n1 = 48 * 1024, 30 * RATE // 1024 * 1024
    x = _rauschen(rng, n0 + n1)
    x[n0:] += _quelle(rng, n1, 50e-6, 6000.0)
    ev = o.feed(_pcm(x), (n0 + n1) / RATE)
    assert ev and max(s.t for s in ev) < 15.0, [round(s.t, 1) for s in ev]
    assert len(ev) <= 15.0 / o.cfg.min_interval_s


def test_eigener_ton_stumm():
    laut = [True]
    o = Ortung(_cfg(), stumm=lambda: laut[0], native=False)
    assert _lauf(o, np.random.default_rng(6), 60e-6) == []
    laut[0] = False                                   # danach noch nachhall_s taub, dann wieder da
    assert _lauf(o, np.random.default_rng(7), 60e-6, t0=10.0)


def test_kurzes_knacken_verworfen():
    o = Ortung(_cfg(min_blocks=2), native=False)
    rng = np.random.default_rng(8)
    x = _rauschen(rng, 96 * 1024)
    x[48 * 1024 + 100:48 * 1024 + 140] += 20000.0     # ein Block
    assert o.feed(_pcm(x), 2.0) == []


def test_status():
    o = Ortung(_cfg(), native=False)
    assert o.status(0.0)["letzter"] is None and o.status(0.0)["aktiv"] is False
    _lauf(o, np.random.default_rng(9), 60e-6)
    st = o.status(3.0)
    assert st["kern"] == "numpy" and st["n"] >= 1 and st["letzter"]["winkel_deg"] > 0
    assert len(st["verlauf"]) == st["n"] and st["verlauf"][-1][1] == st["letzter"]["winkel_deg"]
    assert o.status(3.0 + ortung.VERLAUF_S)["verlauf"] == []          # zu alt: nicht mehr im Status


def test_native_gleich_numpy():
    """C++-Kern (eigene FFT) gegen numpy: Pegel bitgleich, Laufzeit und Güte bis auf Rundung."""
    if ortung._lib() is None:
        pytest.skip("C++-Kern fehlt (ortfast.cpp, siehe docs/testing.md)")
    c = _cfg()
    arg = (c.block, c.rate, c.f_lo, c.f_hi, c.mic_dist_m / ortung.SCHALL_MPS * c.tau_margin, c.steer_steps, c.phat_beta)
    a, b = make_kern(*arg, native=True), make_kern(*arg, native=False)
    rng = np.random.default_rng(10)
    for tau_us in (-140.0, -70.0, 0.0, 15.0, 90.0, 200.0):
        x = _pcm(_rauschen(rng, 5 * c.block) + _quelle(rng, 5 * c.block, tau_us * 1e-6, 5000.0))
        for i in range(5):
            blk = np.ascontiguousarray(x[i * c.block:(i + 1) * c.block])
            assert a.level(blk, 6) == b.level(blk, 6)
            a.add(blk)
            b.add(blk)
        ra, rb = a.solve(), b.solve()
        assert ra[0] == rb[0] == 5
        assert abs(ra[1] - rb[1]) < 1e-10 and abs(ra[2] - rb[2]) < 1e-9, (ra, rb)
    o1, o2 = Ortung(c, native=True), Ortung(c, native=False)
    e1, e2 = _lauf(o1, np.random.default_rng(11), 45e-6), _lauf(o2, np.random.default_rng(11), 45e-6)
    assert len(e1) == len(e2) > 0
    assert all(abs(p.tau_us - q.tau_us) < 1e-3 and p.t == q.t for p, q in zip(e1, e2))


def test_rider_werte_kleine_laufzeit():
    """Defaults des Riders: 3,8 mm wirksamer Abstand, Versatz -2,2 us, 1-s-Fenster. Quelle 45° links/rechts."""
    c = Config.neutral().ortung
    lim = c.mic_dist_m / ortung.SCHALL_MPS
    for winkel in (-45.0, 0.0, 45.0):
        o = Ortung(c, native=False)
        tau = lim * np.sin(np.radians(winkel)) + c.tau_offset_us * 1e-6      # so kommt es an den Kanälen an
        ev = _lauf(o, np.random.default_rng(20), tau, dauer_s=3.0, pegel=6000.0)
        assert len(ev) >= 2, winkel
        assert abs(np.median([s.winkel_deg for s in ev]) - winkel) < 8.0, [s.winkel_deg for s in ev]


def test_rider_werte_klatschen_reicht_nicht():
    c = Config.neutral().ortung
    assert _lauf(Ortung(c, native=False), np.random.default_rng(21), 5e-6, dauer_s=0.2) == []


def test_pausen_unterbrechen_nicht():
    """Drei Silben von 0,3 s mit 0,15 s Pause werden zu einer Auswertung (hang_blocks), ohne Nachlauf zu keiner."""
    def silben(c):
        o = Ortung(c, native=False)
        rng = np.random.default_rng(22)
        b = c.block
        x = _rauschen(rng, 5 * RATE // b * b)
        for k in range(3):
            a = RATE + k * int(0.45 * RATE) // b * b
            n = int(0.3 * RATE) // b * b
            x[a:a + n] += _quelle(rng, n, 6e-6, 6000.0)
        return o.feed(_pcm(x), len(x) / RATE)
    c = Config.neutral().ortung
    assert len(silben(c)) == 1
    c.hang_blocks = 0
    assert silben(c) == []
