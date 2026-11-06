"""web/rider-model.bin (aus tools/step_to_model.py) passt zum Körper-KS der Lage-Anzeige."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from step_to_model import GRUPPEN, modell_lesen  # noqa: E402


def _gruppe(p, d, gr, name):
    grenzen = np.cumsum([0] + [k for _, k in gr])
    i = [g[0] for g in GRUPPEN].index(name)
    return p[d[grenzen[i]:grenzen[i + 1]].ravel()]


def test_rider_model_file():
    pfad = ROOT / "web" / "rider-model.bin"
    assert pfad.stat().st_size < 250 * 1024
    p, d, gr = modell_lesen(pfad)
    assert len(gr) == len(GRUPPEN) and sum(k for _, k in gr) == len(d)
    assert d.max() < len(p)
    lo, hi = p.min(0), p.max(0)
    assert abs(lo[2]) < 0.05 and 12 < hi[2] < 14                 # Boden bei 0, knapp 13 cm hoch
    assert abs(lo[1] + hi[1]) < 0.2 and 5 < hi[1] < 6.5          # Breite symmetrisch zur Mitte
    assert _gruppe(p, d, gr, "display")[:, 0].min() > 3          # Display vorn (x > 0)
    r = _gruppe(p, d, gr, "reifen")                              # Reifen beidseits, 6 cm Durchmesser, um die Achse
    assert r[:, 1].min() < -4 and r[:, 1].max() > 4
    assert abs(r[:, 0].min() + r[:, 0].max()) < 0.1 and abs(r[:, 2].max() - 6) < 0.1
