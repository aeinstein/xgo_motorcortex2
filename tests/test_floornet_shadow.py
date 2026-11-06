"""FloorNet im Schattenbetrieb: darf das Bodenmodell nicht stören und entscheidet nichts."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav import fastlib  # noqa: E402
from xgonav.app import Snapshot, render  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.floornet import NCOLS, FloorNet  # noqa: E402
from xgonav.freespace import FreeSpace, FreeSpaceWorker  # noqa: E402


class _Est:
    """Bodenmodell, das immer dasselbe liefert."""

    def update(self, *a):
        return FreeSpace(sectors=[0.5] * 5, columns=np.full(160, 0.5, np.float32), valid=True)


class _Net:
    def __init__(self, fail: bool = False):
        self.fail = fail
        self.closed = False

    def run(self, bgr):
        if self.fail:
            raise RuntimeError("Absicht")
        return np.full(NCOLS, 300.0, np.float32)

    def close(self):
        self.closed = True


def _einmal(net):
    w = FreeSpaceWorker(_Est(), net=net)
    w.start()
    w.offer(np.zeros((480, 640, 3), np.uint8), 0.0, [])
    t0 = time.monotonic()
    fs = None
    while fs is None and time.monotonic() - t0 < 2.0:
        fs = w.poll()
        time.sleep(0.005)
    w.close()
    return w, fs


def test_ohne_netz_bleibt_alles_beim_alten():
    w, fs = _einmal(None)
    assert fs is not None and fs.net_rows is None and w.net_ms == 0.0


def test_netz_haengt_seine_zeilen_ans_ergebnis():
    net = _Net()
    w, fs = _einmal(net)
    assert fs.net_rows.tolist() == [300.0] * NCOLS
    assert fs.columns[0] == pytest.approx(0.5)                               # Entscheidungsgrundlage unberührt
    assert w.net_ms > 0.0
    assert net.closed


def test_fehler_im_netz_laesst_das_bodenmodell_leben():
    w, fs = _einmal(_Net(fail=True))
    assert fs is not None and fs.valid and fs.net_rows is None
    assert w.net is None


def test_overlay_zeichnet_die_netzlinie():
    frame = np.zeros((480, 640, 3), np.uint8)
    fs = FreeSpace(net_rows=np.full(NCOLS, 300.0, np.float32))
    img = render(Snapshot(frame=frame, free=fs), text=False)
    magenta = (img[:, 320] == (255, 0, 255)).all(axis=1)
    assert magenta[300]
    assert not magenta[200]


def test_schatten_ist_ab_werk_aus():
    assert Config().freespace.shadow_net == ""


def test_fehlende_bibliothek_oder_datei_gibt_kein_netz(tmp_path, monkeypatch):
    monkeypatch.setattr(fastlib, "load", lambda: None)
    assert FloorNet.load(str(tmp_path / "gibtsnicht.bin")) is None


def test_falsche_bildgroesse_wird_abgelehnt():
    n = FloorNet(None, None)
    with pytest.raises(ValueError):
        n.run(np.zeros((240, 320, 3), np.uint8))
