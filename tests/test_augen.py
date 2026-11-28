"""Augengenerator des Display-Daemons (st7789/project/riderpi/src/augen*.c) gegen tools/eyes.py.

Prüft: Ausdrücke (Parameter wie die Python-Funktionen), Raster (Abdeckung deckungsgleich mit render(), nicht
bitgleich: Kantenglättung über Abstand statt 4-fach Überabtasten), autonome Ebene (Lidschlag, Sakkade, Lid,
Vergenz, Befehle). Braucht libxgoaugen.so (XGONAV_AUGENLIB) oder gcc; am Windows-PC übersprungen.
"""
import ctypes
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import augenlib  # noqa: E402
import eyes  # noqa: E402

LIB = augenlib.laden()
pytestmark = pytest.mark.skipif(LIB is None, reason="libxgoaugen.so fehlt und kein gcc")


def _zeiten(T, n):
    return [T * i / (n - 1) for i in range(n)]


def test_abi_und_namen():
    assert LIB.augen_abi() == augenlib.ABI
    namen = [LIB.augen_ausdruck_name(i).decode() for i in range(LIB.augen_ausdruck_n())]
    assert namen == list(eyes.EMOTIONS)
    for i, n in enumerate(namen):
        assert LIB.augen_ausdruck_dauer(i) == pytest.approx(eyes.EMOTIONS[n][1])
    assert LIB.augen_ausdruck_index(b"gibtsnicht") == -1


@pytest.mark.parametrize("name", list(eyes.EMOTIONS))
def test_ausdruck_parameter_wie_eyes(name):
    """Jeder Ausdruck liefert dieselben Parameter wie die Python-Funktion (float statt double)."""
    fn, T = eyes.EMOTIONS[name]
    for t in _zeiten(T, 41):
        gl, pl, pr = fn(t, T)
        b = augenlib.ausdruck(LIB, name, t)
        for k in ("tilt", "gx", "gy"):
            assert getattr(b, k) == pytest.approx(gl.get(k, 0.0), abs=2e-3), (name, t, k)
        for seite, p in (("l", pl), ("r", pr)):
            a = getattr(b, seite)
            for f in augenlib.FELDER:
                assert getattr(a, f) == pytest.approx(float(p[f]), rel=1e-3, abs=2e-3), (name, t, seite, f)
            assert augenlib.ART_NAME[a.kind] == p["kind"], (name, t, seite)


@pytest.mark.parametrize("name", list(eyes.EMOTIONS))
def test_raster_deckungsgleich(name):
    """Abdeckung aus denselben Parametern wie tools/eyes.py render(): je Bild im Mittel < 0,5 %, summiert < 8 %
    der Augenfläche (ein geschlossenes Auge ist nur 7 px hoch, dort zählen die halb bedeckten Randzeilen viel),
    Fläche um höchstens 3 % verschieden. Gemessen 2026-11-28: im Mittel höchstens 0,13 %, summiert höchstens 5,4 %
    (sleep: Abstand zur flachen Halbellipse genähert), Fläche höchstens 2 %."""
    fn, T = eyes.EMOTIONS[name]
    schlecht = []
    for t in _zeiten(T, 7):
        frame = fn(t, T)
        py = eyes.render(frame)
        c = augenlib.raster(LIB, augenlib.aus_python(frame))
        d = np.abs(py - c)
        flaeche = max(1.0, float(py.sum()))
        rel, bias = float(d.sum()) / flaeche, float((py - c).sum()) / flaeche
        if d.mean() > 0.005 or rel > 0.08 or abs(bias) > 0.03:
            schlecht.append((round(t, 2), round(float(d.mean()), 4), round(rel, 3), round(bias, 3)))
    assert not schlecht, f"{name}: (t, mittlere Abweichung, summiert, Flächenfehler) {schlecht}"


def test_raster_nur_im_rechteck():
    """augen_raster schreibt nur im zurückgegebenen Rechteck; draußen ist auch Python leer."""
    b = augenlib.ausdruck(LIB, "grievance", 2.0)          # mit Träne
    a = np.full((240, 320), 7, np.uint8)
    r = LIB.augen_raster(ctypes.byref(b), a.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
    innen = np.zeros_like(a, bool)
    innen[r.y0:r.y1 + 1, r.x0:r.x1 + 1] = True
    assert (a[~innen] == 7).all()
    fn, T = eyes.EMOTIONS["grievance"]
    assert eyes.render(fn(2.0, T))[~innen].max() < 0.02


def test_lut_wie_eyes():
    lut = (ctypes.c_uint16 * 256)()
    LIB.augen_lut(0, 225, 210, lut)
    alpha = np.linspace(0, 1, 256, dtype=np.float32).reshape(16, 16)
    erwartet = np.frombuffer(eyes.to_rgb565(eyes.to_bgr(alpha, (0, 225, 210))), np.uint16)
    assert np.abs(np.array(lut[:], np.int32) - erwartet.astype(np.int32)).max() <= 0x0821   # je Kanal ±1 Stufe


# ---------------------------------------------------------------- autonome Ebene

class Leben:
    def __init__(self, seed=1):
        self.p = LIB.leben_neu(seed)
        self.t = 0.0

    def __del__(self):
        LIB.leben_frei(self.p)

    def befehl(self, zeile):
        return LIB.leben_befehl(self.p, zeile.encode(), self.t)

    def bild(self, dt=0.04):
        self.t += dt
        b = augenlib.Bild()
        art = LIB.leben_bild(self.p, self.t, ctypes.byref(b))
        return art, b


def test_aus_zeichnet_nichts_und_ausdruck_wie_vorgerendert():
    le = Leben()
    assert le.bild()[0] == 0
    assert le.befehl("ausdruck cute")
    t0 = le.t
    T = eyes.EMOTIONS["cute"][1]
    letztes = None
    while True:
        art, b = le.bild()
        if art == 0:
            break
        letztes = (le.t, b)
        ref = augenlib.ausdruck(LIB, "cute", min(le.t - t0, T))
        assert b.l.w == pytest.approx(ref.l.w) and b.tilt == pytest.approx(ref.tilt)
    tl, b = letztes
    assert tl - t0 >= T                                        # letztes Bild genau am Ende
    ende = augenlib.ausdruck(LIB, "cute", T)
    assert b.l.dy == pytest.approx(ende.l.dy) and b.l.mix == pytest.approx(ende.l.mix)


def test_befehle():
    le = Leben()
    for z in ("an", "aus", "blick 0.5 -0.2", "nah 0.4", "lid 0.3", "lid 0.3 2", "farbe rot", "farbe #102030 1",
              "rate 20", "blinzeln", "atmen 0", "ausdruck like", "ausdruck like 0.5", "abbrechen"):
        assert le.befehl(z), z
    for z in ("", "quatsch", "ausdruck gibtsnicht", "blick a b", "farbe 12345", "lid", "blick 1"):
        assert not le.befehl(z), z


def test_farbe_blendet_ueber():
    le = Leben()
    rgb = (ctypes.c_uint8 * 3)()
    LIB.leben_farbe(le.p, 0.0, rgb)
    assert tuple(rgb) == (0, 225, 210)
    le.befehl("farbe rot 1")
    LIB.leben_farbe(le.p, 0.5, rgb)
    assert 100 < rgb[0] < 160
    LIB.leben_farbe(le.p, 1.5, rgb)
    assert tuple(rgb) == tuple(eyes.PALETTE["rot"])


def test_lidschlag_rate():
    le = Leben(seed=7)
    le.befehl("an")
    le.befehl("rate 15")
    n, zu = 0, False
    for _ in range(int(600 / 0.04)):
        _, b = le.bild()
        jetzt_zu = b.l.blink > 0.6
        n += jetzt_zu and not zu
        zu = jetzt_zu
    assert 11 <= n / 10 <= 21               # 15/min + 10 % Doppelblinzler, über 10 min (gemessen 14–17)


def test_ohne_rate_kein_lidschlag_und_lid_folgt():
    le = Leben()
    le.befehl("an")
    le.befehl("rate 0")
    le.befehl("lid 0.5")
    werte = [le.bild()[1].l.blink for _ in range(50)]           # 2 s
    assert werte[-1] == pytest.approx(0.5, abs=0.01)
    assert max(werte) <= 0.5 + 1e-3


def test_sakkade_springt_kleines_folgt():
    le = Leben()
    le.befehl("an")
    le.befehl("rate 0")
    le.bild()
    le.befehl("blick 0.8 0")
    _, b = le.bild(0.04)
    _, b = le.bild(0.04)                                       # 80 ms: Sprung (≤ 62 ms) ist durch
    assert b.l.dx == pytest.approx(34 * 0.8, abs=2.0)
    le.befehl("blick 0.85 0")                                  # kleine Änderung: weich
    _, b = le.bild(0.04)
    assert 34 * 0.8 - 2 < b.l.dx < 34 * 0.85 + 0.5
    for _ in range(20):
        _, b = le.bild(0.04)
    assert b.l.dx == pytest.approx(34 * 0.85, abs=2.0)
    assert b.l.w < b.r.w                                       # Auge in Blickrichtung (rechts) breiter


def test_vergenz():
    le = Leben()
    le.befehl("an")
    le.befehl("rate 0")
    le.befehl("nah 0.3")
    for _ in range(40):
        _, b = le.bild()
    assert b.l.dx - b.r.dx == pytest.approx(2 * 9 * (0.3 / 0.45), abs=1.5)


def test_aus_setzt_blick_zurueck():
    """Am Rider 2026-11-28: Nach einem Test mit Blick nach links standen alle folgenden Ausdrücke links."""
    le = Leben()
    le.befehl("an")
    le.befehl("rate 0")
    le.befehl("blick -0.5 0.2")
    le.befehl("nah 0.3")
    le.befehl("lid 0.5")
    for _ in range(30):
        le.bild()
    le.befehl("aus")
    le.befehl("an")
    _, b = le.bild()
    assert abs(b.l.dx) < 1.5 and abs(b.r.dx) < 1.5 and abs(b.l.dy) < 1.5 and b.l.blink == pytest.approx(0.0)


def test_steht_ohne_atmen():
    """Ohne Atmen und ohne Bewegung meldet leben_bild 3 (steht): der Daemon zeichnet erst beim nächsten Ereignis."""
    le = Leben()
    le.befehl("an")
    le.befehl("rate 0")
    le.befehl("atmen 0")
    arten = [le.bild()[0] for _ in range(100)]
    assert arten[-1] == 3 and arten.count(3) > 50
    le.befehl("atmen 1")
    assert le.bild()[0] in (1, 2)


def test_ausdruck_ueber_lebenden_augen():
    le = Leben()
    le.befehl("an")
    le.befehl("rate 0")
    le.befehl("blick -0.5 0")
    for _ in range(10):
        le.bild()
    le.befehl("ausdruck like")
    t0 = le.t
    T = eyes.EMOTIONS["like"][1]
    while le.t - t0 < T / 2:
        art, b = le.bild()
    assert art == 1 and augenlib.ART_NAME[b.l.kind] == "heart"
    ref = augenlib.ausdruck(LIB, "like", le.t - t0)
    assert b.l.dx == pytest.approx(ref.l.dx - 34 * 0.5, abs=2.5)   # Blick bleibt dazu
    while LIB.leben_ausdruck_laeuft(le.p, le.t):
        art, b = le.bild()
    art, b = le.bild()
    assert art in (1, 2) and b.l.mix == pytest.approx(0.0)


def test_kosten_bericht(capsys):
    """Nur Bericht: Rasterzeit je Bild am Rechner, der den Test laufen lässt."""
    import time
    lut = (ctypes.c_uint16 * 256)()
    LIB.augen_lut(0, 225, 210, lut)
    bild = np.zeros(320 * 240, np.uint16)
    zeiger = bild.ctypes.data_as(ctypes.POINTER(ctypes.c_uint16))
    for name, t in (("aufwachen", 4.4), ("like", 1.4), ("vertigo", 1.2), ("grievance", 1.5)):
        b = augenlib.ausdruck(LIB, name, t)
        t0 = time.perf_counter()
        for _ in range(50):
            r = LIB.augen_raster565(ctypes.byref(b), lut, zeiger)
        us = (time.perf_counter() - t0) / 50 * 1e6
        with capsys.disabled():
            print(f"\n  {name:10s} {us:7.0f} µs  {(r.x1 - r.x0 + 1) * (r.y1 - r.y0 + 1)} px")
    assert math.isfinite(us)
