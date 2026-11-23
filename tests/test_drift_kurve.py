"""tools/drift_kurve.py: Paarsuche (Wiederkehr an eine Stelle) und Kennzahlen an einer synthetischen Fahrt."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.drift_kurve import bewegung, kennzahlen, paare_suchen  # noqa: E402


def _fahrt():
    """25 Hz: 2 m geradeaus (0,4 m/s), 180° drehen (90 °/s), 2 m zurück, 180° drehen, 2 m hin."""
    ms, t, x, th = [], 0.0, 0.0, 0.0
    dt = 0.04

    def add():
        ms.append({"t": t, "pose": [x, 0.0, math.degrees(th)], "balancing": True,
                   "odom": {"widersprueche": 0, "schlupf": False}})
    for abschnitt in ("vor", "dreh", "vor", "dreh", "vor"):
        n = 125 if abschnitt == "vor" else 50
        for _ in range(n):
            if abschnitt == "vor":
                x += 0.4 * dt * math.cos(th)
            else:
                th += math.radians(90.0) * dt
            t += dt
            add()
    return ms


def test_paare_finden_wiederkehr():
    ms = _fahrt()
    b = bewegung(ms)
    paare = paare_suchen(b, nah_m=0.3, nah_deg=20.0, abstand_s=10.0, takt_s=8.0)
    assert paare, "Wiederkehr an den Start der dritten Geraden nicht gefunden"
    for kand, j in paare:
        assert b["v"][j] > 0.05 and b["w"][j] < 20.0          # j in gerader Fahrt
        assert np.all(b["t"][kand] <= b["t"][j] - 10.0)
        d = np.hypot(b["p"][kand, 0] - b["p"][j, 0], b["p"][kand, 1] - b["p"][j, 1])
        assert np.all(d < 0.3)


def test_kennzahlen():
    ms = _fahrt()
    b = bewegung(ms)
    z = kennzahlen(b, 0, len(ms) - 1)
    assert abs(z["weg_m"] - 6.0) < 0.05
    assert abs(z["dreh_deg"] - 360.0) < 1.0
    assert abs(z["v_mittel"] - 0.4) < 0.02
    assert z["fahrdreh_s"] < 0.3                                # nur die Übergänge (Glättung 0,25 s)
