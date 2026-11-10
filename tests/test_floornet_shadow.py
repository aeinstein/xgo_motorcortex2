"""FloorNet im Schatten und als Entscheider: Ein Fehler im Netz darf das Bodenmodell nie treffen."""
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
from xgonav.freespace import FreeSpace, FreeSpaceEstimator, FreeSpaceWorker, net_to_columns  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402


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


class _EstNetz(_Est):
    """Wie _Est, mit den Maßen des echten Bodenmodells (160 x 120, Maßstab 4)."""
    W, H, scale = 160, 120, 4.0

    def update(self, *a):
        return FreeSpace(sectors=[0.5] * 5, columns=np.full(160, 0.5, np.float32), horizon_row=60.0, valid=True)

    adopt_net = FreeSpaceEstimator.adopt_net


def _einmal_primaer(net):
    w = FreeSpaceWorker(_EstNetz(), net=net, net_primary=True)
    w.start()
    w.offer(np.zeros((480, 640, 3), np.uint8), 0.0, [])
    t0 = time.monotonic()
    fs = None
    while fs is None and time.monotonic() - t0 < 2.0:
        fs = w.poll()
        time.sleep(0.005)
    w.close()
    return w, fs


def test_net_to_columns_ist_umkehrbar_und_begrenzt():
    rows = np.linspace(200.0, 479.0, NCOLS, dtype=np.float32)
    cols = net_to_columns(rows, 60.0, 120, 160, 4.0)
    assert cols.shape == (160,) and cols.dtype == np.float32
    # Über dem Horizont (Zeile 240 im Vollbild) heißt frei bis dorthin
    assert cols[0] == 1.0
    # Nur Spalten, die das Netz wirklich geliefert hat: je zwei gleiche, keine erfundenen Zwischenwerte
    assert cols[100] == cols[101] and cols[100] != cols[102]
    # Umkehrung wie in LocalMap und GridMap: Zeile = (H - Wert * Spanne) * Maßstab
    back = (120 - cols[60:] * (120 - 60.0)) * 4.0
    assert back == pytest.approx(rows[30:].repeat(2), abs=1e-3)


def test_netz_entscheidet_und_verfahren_bleibt_als_schatten():
    w, fs = _einmal_primaer(_Net())
    assert fs.src == "net" and w.net is not None
    want = (120 - 300.0 / 4.0) / 60.0
    assert fs.columns == pytest.approx(want)
    assert fs.sectors == pytest.approx([want] * 5)
    assert fs.shadow_columns == pytest.approx(0.5)


def test_fehler_im_netz_laesst_das_verfahren_entscheiden():
    w, fs = _einmal_primaer(_Net(fail=True))
    assert w.net is None and fs.src == "method" and fs.shadow_columns is None
    assert fs.columns == pytest.approx(0.5) and fs.net_rows is None


def test_ungueltige_zeilen_schalten_das_netz_ab():
    class _NaN(_Net):
        def run(self, bgr):
            return np.full(NCOLS, np.nan, np.float32)

    w, fs = _einmal_primaer(_NaN())
    assert w.net is None and fs.src == "method" and fs.columns == pytest.approx(0.5)


def test_overlay_zeigt_verfahren_gelb_und_netz_magenta():
    frame = np.zeros((480, 640, 3), np.uint8)
    fs = FreeSpace(sectors=[0.5] * 5, columns=net_to_columns(np.full(NCOLS, 300.0, np.float32), 60.0, 120, 160, 4.0),
                   horizon_row=60.0, mask=np.zeros((120, 160), np.uint8), valid=True, src="net",
                   shadow_columns=np.full(160, 0.5, np.float32), net_rows=np.full(NCOLS, 300.0, np.float32))
    img = render(Snapshot(frame=frame, free=fs), text=False)
    assert (img[:, 320] == (255, 0, 255)).all(axis=1)[300]
    assert (img[:, 320] == (0, 255, 255)).all(axis=1)[int((120 - 0.5 * 60) * 4)]


def test_primaer_ist_ab_werk_das_verfahren():
    assert Config().freespace.primary == "method" and Config().grid.net_row_bias == 0.0


def test_raumkarte_nimmt_den_versatz_der_quelle():
    cfg = Config()
    cfg.grid.row_bias, cfg.grid.net_row_bias, cfg.grid.native = 8.0, 0.0, False
    geo = CamGeometry.from_cfg(cfg)
    gefunden = {}
    for src in ("method", "net"):
        gm = GridMap(cfg.grid, geo, 160, 120)
        for _ in range(5):
            fs = FreeSpace(sectors=[0.3] * 5, columns=np.full(160, 0.3, np.float32), horizon_row=60.0, valid=True, src=src)
            gm.add_freespace(fs, 0.0, (0.0, 0.0, 0.0))
        occ = gm.occupied_cells(0.0)
        gefunden[src] = float(np.median(occ[:, 0]))
    # Positiver row_bias legt die Grenze höher im Bild, also weiter weg; der Netzversatz 0 nicht
    assert gefunden["method"] > gefunden["net"] + 0.02
