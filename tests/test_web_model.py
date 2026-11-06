"""web/rider-model.bin (aus tools/step_to_model.py) passt zum Körper-KS der Lage-Anzeige."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from step_to_model import GRUPPEN, beinlage, modell_lesen, sollstreckung  # noqa: E402


def _gruppe(p, d, gr, farbe):
    """Punkte aller Zeichengruppen einer Farbgruppe."""
    grenzen = np.cumsum([0] + [k for *_, k in gr])
    hexwert = dict((g[0], g[1][1:]) for g in GRUPPEN)[farbe]
    return np.concatenate([p[d[grenzen[i]:grenzen[i + 1]].ravel()] for i, g in enumerate(gr) if g[0] == hexwert])


def test_rider_model_file():
    pfad = ROOT / "web" / "rider-model.bin"
    assert pfad.stat().st_size < 300 * 1024
    p, d, gr, pv_l, pv_r = modell_lesen(pfad)
    assert sum(k for *_, k in gr) == len(d)
    assert d.max() < len(p)
    lo, hi = p.min(0), p.max(0)
    assert abs(lo[2]) < 0.05 and 12 < hi[2] < 14                 # Boden bei 0, knapp 13 cm hoch
    assert abs(lo[1] + hi[1]) < 0.2 and 5 < hi[1] < 6.5          # Breite symmetrisch zur Mitte
    assert _gruppe(p, d, gr, "display")[:, 0].min() > 3          # Display vorn (x > 0)
    r = _gruppe(p, d, gr, "reifen")                              # Reifen beidseits, 6 cm Durchmesser, um die Achse
    assert r[:, 1].min() < -4 and r[:, 1].max() > 4
    assert abs(r[:, 0].min() + r[:, 0].max()) < 0.1 and abs(r[:, 2].max() - 6) < 0.1
    assert {t for _, t, _ in gr} == set(range(9))                # Körper + je vier Beinteile
    assert all(np.allclose(pv_l[k], pv_r[k], atol=1e-3) for k in pv_r)  # Seiten gleich, Radachse wie im CAD
    assert abs(pv_r["W"][0]) < 0.01 and abs(pv_r["W"][1] - 3) < 0.01


def test_beinlage_passt_zur_firmware():
    """Abstand Servo-Radachse aus dem Viergelenk gegen die Streckung der Firmware, im Bereich des Reglers."""
    _, _, _, pv, _ = modell_lesen(ROOT / "web" / "rider-model.bin")
    S, W = pv["S"], pv["W"]
    for u8 in (55, 90, 128, 160, 200):
        e, e2 = sollstreckung(u8, 0)
        assert e == e2 and 52 < e < 74
        *_, koppel, rad = beinlage(pv, e)
        w2 = (W[0] + rad[2], W[1] + rad[3])
        assert abs(np.hypot(w2[0] - S[0], w2[1] - S[1]) * 10 - e) < 3      # mm
        assert abs(w2[0] - S[0]) < 0.4                                     # Achse bleibt unter dem Servo
    # Glieder bleiben verbunden: Knie A aus Servo-Glied und Koppel, B aus oberem Glied und Koppel
    an = lambda b, q: (b[0] * q[0] - b[1] * q[1] + b[2], b[1] * q[0] + b[0] * q[1] + b[3])
    for e in (45, 63, 81):
        g_s, g_o, k, _ = beinlage(pv, e)
        assert np.allclose(an(g_s, pv["A"]), an(k, pv["A"]), atol=1e-6)
        assert np.allclose(an(g_o, pv["B"]), an(k, pv["B"]), atol=1e-6)
    # Roll: rechts tief heißt links länger, Mitte bleibt
    li, re = sollstreckung(128, 10)
    assert li > re and abs((li + re) / 2 - sollstreckung(128, 0)[0]) < 1e-6
    # Begrenzungen wie Firmware: Regler 55..200, Streckung 45..81
    assert sollstreckung(255, 0) == sollstreckung(200, 0) and sollstreckung(0, 0) == sollstreckung(55, 0)
    assert sollstreckung(200, 20)[0] <= 81 and sollstreckung(55, -20)[0] >= 45


def test_gridmap_render_center():
    from xgonav.config import Config
    from xgonav.geometry import CamGeometry
    from xgonav.gridmap import GridMap
    m = GridMap(Config().grid, CamGeometry(640, 480, 91.0, 1.39, 43.0, 85.0), 160, 120)
    rot = lambda img: np.argwhere((img[:, :, 2] > 180) & (img[:, :, 1] < 80))
    img, mx, my, kante = m.render(200, 4.0)                      # ohne Mitte: Rider mittig
    assert (mx, my) == (0.0, 0.0) and abs(kante - 4.0) < 1e-9
    assert np.abs(rot(img).mean(0) - 100).max() < 10
    # Mitte 1 m vor (oben = +x) und 1 m links (links = +y) vom Rider: Rider im Bild unten rechts
    img, mx, my, _ = m.render(200, 4.0, (1.0, 1.0))
    assert (mx, my) == (1.0, 1.0)
    z, s = rot(img).mean(0)
    assert abs(z - 150) < 10 and abs(s - 150) < 10
    assert abs(m.render(200, 4.03)[3] - 4.05) < 1e-9             # Kante auf ganze Zellen (5 cm)
    assert m.image(120).shape == (120, 120, 3)
