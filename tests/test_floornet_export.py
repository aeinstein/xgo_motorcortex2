"""FloorNet-Export: Dateiaufbau passend zu xgonav/native/floornet.cpp. Braucht torch (nur auf dem GPU-Rechner)."""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "floornet"))

import export as X  # noqa: E402
import net as N  # noqa: E402


def _net():
    torch.manual_seed(0)
    m = N.FloorNet()
    m.train()
    for _ in range(3):
        m(torch.rand(4, 3, N.IN_H, N.IN_W) - 0.5)
    return m.eval()


def _float_count() -> int:
    """Anzahl Gewichte, gerechnet wie load_net in floornet.cpp."""
    n = 12 * 3 * 9 + 12                                                      # Stem
    cin = 12
    for c in (24, 32, 32, 48, 48, 48):
        n += cin * 9 + c * cin + c                                           # tiefenweise, 1x1 + Bias
        cin = c
    n += 32 * 48 * 11 + 32                                                   # Kopf 1x1
    n += 32 * 3 + 32                                                         # Kopf 1x3
    n += N.NBINS * 32 + N.NBINS                                              # Ausgabe
    return n


def test_datei_hat_die_groesse_die_der_kern_erwartet(tmp_path):
    p = tmp_path / "n.bin"
    X.write(_net(), str(p))
    d = p.read_bytes()
    want = 4 + 12 + 4 * N.NBINS + 4 * _float_count() + 4 * N.NBINS * N.NCOLS + 4
    assert len(d) == want
    assert d[:4] == b"FNET" and d[-4:] == b"ENDE"
    assert struct.unpack("<3I", d[4:16]) == (X.VERSION, N.NCOLS, N.NBINS)
    assert np.frombuffer(d[16:16 + 4 * N.NBINS], "<f4") == pytest.approx(N.bin_centers().astype(np.float32))


def test_erster_stem_wert_und_logits_stehen_an_ihrer_stelle(tmp_path):
    m = _net()
    p = tmp_path / "n.bin"
    logits = X.write(m, str(p))
    d = p.read_bytes()
    ops = N.fold(m)
    first = np.frombuffer(d, "<f4", 12 * 3 * 9, 16 + 4 * N.NBINS)
    assert first == pytest.approx(ops[0]["w"].numpy().ravel())
    stored = np.frombuffer(d, "<f4", N.NBINS * N.NCOLS, len(d) - 4 - 4 * N.NBINS * N.NCOLS).reshape(N.NBINS, N.NCOLS)
    assert (stored == logits).all()


def test_testbild_und_eingabe():
    f = X.test_frame()
    assert f.shape == (480, 640, 3) and f.dtype == np.uint8
    # Gleiche Formel wie im C++-Kern, einzeln nachgerechnet
    x, y, c = 100, 200, 1
    assert f[y, x, c] == ((x * 3 + y * 5 + c * 40) ^ ((x // 32) * (y // 24) * 17)) & 255
    t = X.to_input(f)
    assert t.shape == (1, 3, N.IN_H, N.IN_W)
    # Eingabepixel (0, 0): Mittel der vier Vollbildpixel ab Zeile CROP_Y
    want = f[N.CROP_Y:N.CROP_Y + 2, 0:2, 0].astype(float).mean() / 255.0 - 0.5
    assert t[0, 0, 0, 0].item() == pytest.approx(want, abs=1e-6)
