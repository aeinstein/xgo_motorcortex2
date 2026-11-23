"""Stimmproben (xgonav/stimme.py): Mitschnitt, Vergleich, Ablage, Sittich-Filter, C++ gegen numpy."""
import dataclasses
import json
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


def test_voll_ersetzt_die_seltenste_und_neustart_laedt(tmp_path):
    worte = [[100, 300], [300, 100], [120, 280, 120]]
    uhr = [1000.0]
    st = stimme.Stimmen(cfg(tmp_path, max_confirmed=2), native=False, clock=lambda: uhr[0])
    for k in (0, 1):                                                  # zwei bestätigt
        st.lerne(aeusserung(worte[k], seed=20 + k))
        assert st.lerne(aeusserung(worte[k], seed=30 + k)) == "bestaetigt"
        uhr[0] += 1
    assert st.voll and st.drain_neu() == ["01", "02"]
    assert st.lerne(aeusserung(worte[0], seed=40)) == "wieder"          # 01 jetzt 2-mal, 02 1-mal
    neu = stimme.Stimmen(cfg(tmp_path, max_confirmed=2), native=False, clock=lambda: uhr[0])   # Neustart: Zähler bleiben
    neu.laden()
    assert {p.name: p.n for p in neu.bestaetigt} == {"01": 2, "02": 1}
    neu.drain_neu()
    neu.lerne(aeusserung(worte[2], seed=50))
    assert neu.lerne(aeusserung(worte[2], seed=51)) == "bestaetigt"     # voll: 02 (seltenste) wird ersetzt
    assert neu.ids == ("01", "02") and neu.drain_neu() == ["02"]
    assert neu.lerne(aeusserung(worte[1], seed=52)) == "kandidat"       # altes 02 ist vergessen
    assert neu.lerne(aeusserung(worte[2], seed=53)) == "wieder"


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
    for _ in range(500):                                              # bis 10 s (volle Testsuite: Last)
        if st.kandidaten:
            break
        time.sleep(0.02)
    assert len(st.kandidaten) == 1
    uhr[0] += 61
    for _ in range(500):
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
    da, db = P.dehne(ta, 2.0), N.dehne(ta, 2.0)
    assert len(da) == len(db) and np.abs(da - db).max() < 1e-6 * np.abs(da).max()


def test_rauschtor_schneidet_rauschen_ab():
    k = stimme.make_kern(0.3, native=False)
    x = k.mono16k(np.concatenate([stille(0.5), aeusserung(WORT_A, 0.6), stille(0.8)]))
    y = k.tor(x, 12.0, 6.0)
    assert 0.4 * stimme.RATE < len(y) < 0.8 * stimme.RATE                # nur der Satz bleibt
    roh = x[: int(0.4 * stimme.RATE)]
    assert np.sqrt(np.mean(y[:200] ** 2)) <= np.sqrt(np.mean(x ** 2))   # Anfang leiser als der Durchschnitt
    assert np.abs(roh).max() > 0


def test_oktave_hoeher_bei_gleichem_tempo(tmp_path):
    """pitch_st 12: ein 220-Hz-Ton kommt mit 440 Hz heraus, so lang wie vorher (Tor schneidet nichts ab)."""
    c = cfg(tmp_path)
    assert c.pitch_st == 12.0 and c.speed == 1.0
    k = stimme.make_kern(c.flat_max, native=False)
    n = int(1.0 * stimme.RATE)
    t = np.arange(n) / stimme.RATE
    x = 8000.0 * np.sin(2 * np.pi * 220.0 * t)
    y = stimme.sittich_kette(k, x, dataclasses.replace(c, band_lo=100.0, vibrato_ms=0.0, drive=0.01))
    assert abs(len(y) / stimme.RATE_OUT - 1.0) < 0.02
    spec = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    f = np.argmax(spec) * stimme.RATE_OUT / len(y)
    assert abs(f - 440.0) < 5.0


def test_loeschen_aus_dem_web(tmp_path):
    st = stimmen(tmp_path, max_confirmed=2)
    st.lerne(aeusserung(WORT_A, seed=60))
    assert st.lerne(aeusserung(WORT_A, seed=61)) == "bestaetigt"
    assert st.lerne(aeusserung(WORT_B, seed=62)) == "kandidat"
    kand = st.kandidaten[0].name
    with pytest.raises(ValueError):
        st.loeschen("../01")
    assert st.loeschen("05") is False
    assert st.loeschen(kand) is True and st.kandidaten == [] and os.listdir(tmp_path / "kandidaten") == []
    assert st.loeschen("01") is True and st.ids == () and st.bestaetigt == []
    assert sorted(os.listdir(tmp_path / "bestaetigt")) == ["zaehler.json"]
    assert (tmp_path / "bestaetigt" / "zaehler.json").read_text() == "{}"
    st.lerne(aeusserung(WORT_B, seed=63))                                # frei gewordener Platz wird wieder belegt
    assert st.lerne(aeusserung(WORT_B, seed=64)) == "bestaetigt" and st.ids == ("01",)
    neu = stimmen(tmp_path, max_confirmed=2)                             # ungeladen: nur Platte und ids
    assert neu.loeschen("01") is True and neu.ids == () and not (tmp_path / "bestaetigt" / "01.wav").exists()


# ---- Hochladen (Web) -----------------------------------------------------------
def hoch(f0s, seed=0, dur=1.0):
    """Wie der Browser liefert: 16 kHz mono, Einheit int16."""
    k = stimme.make_kern(0.3, native=False)
    return k.mono16k(aeusserung(f0s, dur, seed=seed))


def test_hochladen_fest_und_neustart(tmp_path):
    st = stimmen(tmp_path)
    assert st.hochladen(hoch(WORT_A)) == ("01", None)
    assert st.ids == ("01",) and st.drain_neu() == []                  # nicht gleich nachplappern
    d = tmp_path / "bestaetigt"
    assert (d / "01.wav").is_file() and (d / "01_roh.wav").is_file()
    assert (d / "fest.json").read_text() == '["01"]' and st.status()["fest"] == 1
    neu = stimmen(tmp_path)
    assert neu.status()["fest"] == 1
    assert neu.lerne(aeusserung(WORT_A, seed=7)) == "wieder"           # wird beim Mitschnitt wiedererkannt


def test_hochladen_ohne_pruefung(tmp_path):
    st = stimmen(tmp_path)
    x = np.random.default_rng(5).normal(0, 3000, int(1.2 * stimme.RATE))   # Rauschen: beim Mitschnitt verworfen
    assert st.hochladen(x)[0] == "01"
    with pytest.raises(ValueError):
        st.hochladen(np.zeros(100))


def test_hochgeladene_fallen_nie_raus(tmp_path):
    worte = [[100, 300], [300, 100], [120, 280, 120]]
    st = stimmen(tmp_path, max_confirmed=2)
    st.lerne(aeusserung(worte[0], seed=20))
    assert st.lerne(aeusserung(worte[0], seed=21)) == "bestaetigt"     # 01 gelernt
    assert st.hochladen(hoch(worte[1], seed=22)) == ("02", None)       # 02 fest
    st.lerne(aeusserung(worte[2], seed=23))
    assert st.lerne(aeusserung(worte[2], seed=24)) == "bestaetigt"     # voll: ersetzt 01, nie die feste 02
    assert st.ids == ("01", "02") and st._fest == {"02"}
    assert st.hochladen(hoch(worte[0], seed=25)) == ("01", "01")       # Upload ersetzt die gelernte
    assert st._fest == {"01", "02"}
    with pytest.raises(stimme.VollFehler):                              # alle fest
        st.hochladen(hoch(worte[2], seed=26))
    st.lerne(aeusserung([200, 90, 200], seed=27))
    assert st.lerne(aeusserung([200, 90, 200], seed=28)) == "voll"      # alle fest: nichts bestätigt
    assert st.ids == ("01", "02") and st.kandidaten == []
    assert st.loeschen("02") is True and st._fest == {"01"}
    assert (tmp_path / "bestaetigt" / "fest.json").read_text() == '["01"]'


def test_hochladen_ueber_die_schnittstelle(tmp_path):
    import io
    import types
    import urllib.error
    import urllib.request
    import wave
    from xgonav.web import serve
    st = stimmen(tmp_path)
    srv = serve(types.SimpleNamespace(cfg=None, stimmen=st), "127.0.0.1", 0, 5)
    url = "http://127.0.0.1:%d/stimme/hochladen" % srv.server_address[1]

    def post(body):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, data=body, method="POST"), timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def wav(x, rate):
        b = io.BytesIO()
        with wave.open(b, "wb") as w:
            w.setnchannels(1), w.setsampwidth(2), w.setframerate(rate)
            w.writeframes(np.clip(x, -32768, 32767).astype("<i2").tobytes())
        return b.getvalue()
    try:
        assert post(wav(hoch(WORT_A), stimme.RATE)) == (200, {"ok": True, "name": "01", "ersetzt": None})
        assert post(wav(hoch(WORT_B), 48000))[0] == 400
        assert post(b"RIFF....WAVEkaputt")[0] == 400
        assert post(b"RIFF\x24\x00\x00\x00WAVE" + b"kaputt" * 6)[0] == 400   # wave: RuntimeError beim Überspringen
        assert post(b"")[0] == 413                                       # leer; zu groß liest der Server gar nicht erst
        assert st.ids == ("01",)
        fest = "http://127.0.0.1:%d/stimme/fest?" % srv.server_address[1]
        for q, code in (("name=01&an=0", 200), ("name=05&an=1", 404), ("name=x1&an=1", 400)):
            try:
                with urllib.request.urlopen(urllib.request.Request(fest + q, data=b"", method="POST"), timeout=5) as r:
                    assert r.status == code
            except urllib.error.HTTPError as e:
                assert e.code == code
        assert st._fest == set()
    finally:
        srv.shutdown()


def test_hochladen_fertig_ohne_filter(tmp_path):
    st = stimmen(tmp_path)
    x48 = aeusserung(WORT_A, 2.0)[:, 0].astype(np.float64)              # 48 kHz mono, schon bearbeitet
    assert st.hochladen(x48, fertig=True) == ("01", None)
    y, rate = stimme.read_wav(str(tmp_path / "bestaetigt" / "01.wav"))
    assert rate == stimme.RATE_OUT and np.array_equal(y, np.round(x48))  # unverändert
    roh, rate = stimme.read_wav(str(tmp_path / "bestaetigt" / "01_roh.wav"))
    assert rate == stimme.RATE and abs(len(roh) / rate - 2.0) < 0.01
    assert st._fest == {"01"}


def test_schuetzen_und_loesen(tmp_path):
    worte = [[100, 300], [300, 100], [120, 280, 120], [280, 110, 280]]
    uhr = [1000.0]
    st = stimme.Stimmen(cfg(tmp_path, max_confirmed=2), native=False, clock=lambda: uhr[0])
    for k in (0, 1):
        st.lerne(aeusserung(worte[k], seed=70 + k))
        assert st.lerne(aeusserung(worte[k], seed=80 + k)) == "bestaetigt"
        uhr[0] += 1
    assert st.lerne(aeusserung(worte[1], seed=90)) == "wieder"          # 02 öfter gehört, 01 wäre dran
    assert st.festsetzen("01", True) is True
    assert (tmp_path / "bestaetigt" / "fest.json").read_text() == '["01"]'
    assert stimme.Stimmen(cfg(tmp_path, max_confirmed=2), native=False)._fest == {"01"}   # Neustart kennt ihn
    st.lerne(aeusserung(worte[2], seed=91))
    assert st.lerne(aeusserung(worte[2], seed=92)) == "bestaetigt"     # ersetzt 02, nie das geschützte 01
    assert st.lerne(aeusserung(worte[0], seed=93)) == "wieder"
    assert st.festsetzen("01", False) is True and st._fest == set()
    assert st.festsetzen("05", True) is False
    with pytest.raises(ValueError):
        st.festsetzen("../01", True)
