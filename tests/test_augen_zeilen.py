"""xgonav/augen.py: Zeilen an den Augengenerator im Display-Daemon (nur Geändertes) und Emotions über ihn.
Statt des FIFO eine gewöhnliche Datei (läuft auch am Windows-PC)."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from xgonav.augen import Augen  # noqa: E402
from xgonav.config import EmotionCfg  # noqa: E402
from xgonav.emotion import Emotions  # noqa: E402


def _zeilen(pfad, ab=0):
    with open(pfad, encoding="ascii") as f:
        return f.read().splitlines()[ab:]


def test_leben_sendet_nur_geaendertes(tmp_path):
    f = tmp_path / "augen"
    f.write_text("")
    a = Augen(str(f))
    assert a.ok
    assert a.leben(0.0, 0.0, 0.0, 0.0, 15.0, None)
    assert _zeilen(f) == ["an", "farbe tuerkis 0.4", "rate 15.0", "lid 0.000", "nah 0.00", "blick 0.000 0.000"]
    n = len(_zeilen(f))
    a.leben(0.01, 0.0, 0.01, 0.01, 15.0, None)               # unter den Schwellen
    assert _zeilen(f, n) == []
    a.leben(0.3, -0.2, 0.5, 0.4, 6.0, "rot")
    assert _zeilen(f, n) == ["farbe rot 0.4", "rate 6.0", "lid 0.500", "nah 0.40", "blick 0.300 -0.200"]
    n = len(_zeilen(f))
    a.aus()
    assert _zeilen(f, n) == ["aus", "farbe tuerkis"]
    n = len(_zeilen(f))
    a.aus()
    assert _zeilen(f, n) == []
    a.leben(0.3, -0.2, 0.5, 0.4, 6.0, None)
    assert _zeilen(f, n)[0] == "an"


def test_ausdruck_mit_farbe(tmp_path):
    f = tmp_path / "augen"
    f.write_text("")
    a = Augen(str(f))
    assert a.ausdruck("like", "rosa")
    assert a.ausdruck("cute")
    assert _zeilen(f) == ["farbe rosa 0.2", "ausdruck like", "farbe tuerkis 0.2", "ausdruck cute"]


def test_ohne_daemon_nicht_verfuegbar(tmp_path):
    a = Augen(str(tmp_path / "fehlt"))
    assert not a.ok
    assert not a.leben(0, 0, 0, 0, 15, None)
    assert not a.ausdruck("cute")


class _Generator:
    def __init__(self, ok=True):
        self.ok_, self.calls = ok, []

    def ausdruck(self, name, farbe=None):
        self.calls.append((name, farbe))
        return self.ok_


def test_emotions_ueber_generator(tmp_path):
    g = _Generator()
    cfg = EmotionCfg(dir=str(tmp_path / "keine_bilder"), sound=False)
    e = Emotions(cfg, str(tmp_path / "lcd0"), augen=g)
    assert e.enabled                                           # ohne vorgerenderte Bilder, nur mit Generator
    e.farbe = "gelb"
    e.play("cute")
    assert g.calls == [("cute", "gelb")]
    e2 = Emotions(cfg, str(tmp_path / "lcd0"))
    assert not e2.enabled                                      # ohne Generator und ohne Bilder: aus
