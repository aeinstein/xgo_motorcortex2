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

    cfg = Config().freespace
    geo = None
    _net_mask = None
    diag: dict = {}
    updates = 0

    def update(self, *a):
        type(self).updates += 1
        return FreeSpace(sectors=[0.5] * 5, columns=np.full(160, 0.5, np.float32), horizon_row=60.0, valid=True)

    from_net = FreeSpaceEstimator.from_net
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


def test_netz_entscheidet_allein_und_verfahren_rechnet_nicht():
    _EstNetz.updates = 0
    w, fs = _einmal_primaer(_Net())
    assert fs.src == "net" and w.net is not None
    want = (120 - 300.0 / 4.0) / 60.0
    assert fs.columns == pytest.approx(want)
    assert fs.sectors == pytest.approx([want] * 5)
    assert fs.horizon_row == 60.0 and fs.mask.shape == (120, 160) and fs.valid
    assert _EstNetz.updates == 0 and w.ms == 0.0 and w.est.diag == {"src": "net"}


def test_fehler_im_netz_laesst_das_verfahren_entscheiden():
    w, fs = _einmal_primaer(_Net(fail=True))
    assert w.net is None and fs.src == "method"
    assert fs.columns == pytest.approx(0.5) and fs.net_rows is None


def test_ungueltige_zeilen_schalten_das_netz_ab():
    class _NaN(_Net):
        def run(self, bgr):
            return np.full(NCOLS, np.nan, np.float32)

    w, fs = _einmal_primaer(_NaN())
    assert w.net is None and fs.src == "method" and fs.columns == pytest.approx(0.5)


def test_overlay_zeigt_nur_das_netz_wenn_es_entscheidet():
    frame = np.zeros((480, 640, 3), np.uint8)
    fs = FreeSpace(sectors=[0.5] * 5, columns=net_to_columns(np.full(NCOLS, 300.0, np.float32), 60.0, 120, 160, 4.0),
                   horizon_row=60.0, mask=np.zeros((120, 160), np.uint8), valid=True, src="net",
                   net_rows=np.full(NCOLS, 300.0, np.float32))
    img = render(Snapshot(frame=frame, free=fs), text=False)
    assert (img[:, 320] == (255, 0, 255)).all(axis=1)[300]
    assert not (img == (0, 255, 255)).all(axis=2).any()          # keine gelbe Linie
    # Entscheidet das Verfahren, bleibt seine Linie gelb und das Netz (Schatten) magenta
    fs = FreeSpace(sectors=[0.5] * 5, columns=np.full(160, 0.5, np.float32), horizon_row=60.0,
                   mask=np.zeros((120, 160), np.uint8), valid=True, net_rows=np.full(NCOLS, 300.0, np.float32))
    img = render(Snapshot(frame=frame, free=fs), text=False)
    assert (img[:, 320] == (255, 0, 255)).all(axis=1)[300]
    assert (img[:, 320] == (0, 255, 255)).all(axis=1)[int((120 - 0.5 * 60) * 4)]


def test_primaer_ist_ab_werk_das_verfahren():
    assert Config().freespace.primary == "method"


# ---- ZeilenGlaetter: Kante über die Zeit beruhigen ----

def _geo():
    from xgonav.geometry import CamGeometry
    return CamGeometry(640, 480, 91.8, tilt_deg=2.0, mount_offset_mm=0.0, body_height_mm=110.0, cam_fwd_mm=30.0)


def _zeilen(geo, punkte, pose, pitch=0.0):
    """Weltpunkte am Boden -> Grenzzeilen (NCOLS) aus Sicht von pose; Spalten ohne Punkt 300."""
    rows = np.full(NCOLS, 300.0, np.float32)
    for wx, wy in punkte:
        dx, dy = wx - pose[0], wy - pose[1]
        vor = dx * np.cos(pose[2]) + dy * np.sin(pose[2]) - geo.cam_fwd
        links = -dx * np.sin(pose[2]) + dy * np.cos(pose[2])
        x = geo.cx - geo.f * links / vor
        if vor > 0.05 and 0 <= x < 640:
            rows[int(x // 8)] = geo.H / 2 + geo.f * np.tan(np.arctan2(geo.cam_h, vor) + np.radians(geo.tilt_deg) + pitch)
    return rows


def test_glaetter_nimmt_einzelnen_ausreisser_im_stand_heraus():
    from xgonav.floornet import ZeilenGlaetter
    geo, g = _geo(), ZeilenGlaetter(3)
    ruhig = np.full(NCOLS, 350.0, np.float32)
    pose = (1.0, 2.0, 0.5)
    assert np.array_equal(g.glaetten(ruhig, pose, 0.0, 0.0, geo), ruhig)        # erstes Bild: unverändert
    g.glaetten(ruhig, pose, 0.0, 0.083, geo)
    zacke = ruhig.copy()
    zacke[40] = 470.0
    assert np.allclose(g.glaetten(zacke, pose, 0.0, 0.166, geo), ruhig, atol=0.5)
    # bleibt die Stufe, gilt sie ab dem zweiten Bild
    assert g.glaetten(zacke, pose, 0.0, 0.25, geo)[40] == pytest.approx(470.0, abs=0.5)


def test_worker_glaettet_nur_auf_wunsch_und_haelt_die_rohen_zeilen():
    class _Folge(_Net):
        def __init__(self):
            super().__init__()
            self.k = 0

        def run(self, bgr):
            self.k += 1
            return np.full(NCOLS, 460.0 if self.k == 3 else 350.0, np.float32)      # drittes Bild: Ausreißer

    def lauf(n):
        est = _EstNetz()
        est.geo = _geo()
        w = FreeSpaceWorker(est, net=_Folge(), net_primary=True, net_smooth_n=n)
        w.start()
        out = []
        for k in range(3):
            w.offer(np.zeros((480, 640, 3), np.uint8), 0.0, [], (0.0, 0.0, 0.0), 0.083 * k)
            t0, fs = time.monotonic(), None
            while fs is None and time.monotonic() - t0 < 2.0:
                fs = w.poll()
                time.sleep(0.005)
            out.append(fs)
        w.close()
        return out

    roh = lauf(0)[2]
    assert roh.net_rows[0] == 460.0 and roh.net_rows_raw is None
    glatt = lauf(3)
    assert glatt[0].net_rows[0] == 350.0
    assert glatt[2].net_rows == pytest.approx(350.0, abs=0.5) and glatt[2].net_rows_raw[0] == 460.0
    assert glatt[2].columns == pytest.approx(glatt[1].columns, abs=0.01) and glatt[2].src == "net"


def test_glaetter_mittelt_nie_bei_zwei_werten():
    from xgonav.floornet import ZeilenGlaetter
    geo, g = _geo(), ZeilenGlaetter(3)
    a, b = np.full(NCOLS, 350.0, np.float32), np.full(NCOLS, 400.0, np.float32)
    g.glaetten(a, (0, 0, 0), 0.0, 0.0, geo)
    assert np.array_equal(g.glaetten(b, (0, 0, 0), 0.0, 0.083, geo), b)         # 2 Werte: der aktuelle, kein Mittel


@pytest.mark.parametrize("pose1,pitch1", [((0.0, 0.0, 0.2), 0.0), ((0.1, 0.0, 0.0), 0.0), ((0.05, 0.02, -0.1), 0.02)])
def test_glaetter_rechnet_feste_bodenpunkte_auf_ihre_zeile_um(pose1, pitch1):
    from xgonav.floornet import ZeilenGlaetter
    geo = _geo()
    punkte = [(0.6 + 0.004 * i, -0.4 + 0.008 * i) for i in range(100)]          # schräge Kante am Boden
    alt = _zeilen(geo, punkte, (0.0, 0.0, 0.0))
    neu = _zeilen(geo, punkte, pose1, pitch1)
    um = ZeilenGlaetter.umrechnen(alt, (0.0, 0.0, 0.0), 0.0, pose1, pitch1, geo)
    ziel = np.isfinite(um) & (neu != 300.0)
    assert ziel.sum() >= 15
    # Spaltenraster 8 px: an einer schrägen Kante bis zu einer Spalte Versatz, also einige Zeilen
    assert np.median(np.abs(um[ziel] - neu[ziel])) < 3.0


def test_glaetter_setzt_bei_posensprung_luecke_und_fehlender_pose_zurueck():
    from xgonav.floornet import ZeilenGlaetter
    geo = _geo()
    a, b = np.full(NCOLS, 350.0, np.float32), np.full(NCOLS, 420.0, np.float32)
    for pose, t in (((2.0, 0.0, 0.0), 0.166), ((0.0, 0.0, 0.0), 2.0), (None, 0.166)):
        g = ZeilenGlaetter(3)
        g.glaetten(a, (0.0, 0.0, 0.0), 0.0, 0.0, geo)
        g.glaetten(a, (0.0, 0.0, 0.0), 0.0, 0.083, geo)
        assert np.array_equal(g.glaetten(b, pose, 0.0, t, geo), b)
    assert Config().freespace.net_smooth_n == 0                                  # ab Werk aus
    g = ZeilenGlaetter(1)                                                        # n = 1: aus
    g.glaetten(a, (0.0, 0.0, 0.0), 0.0, 0.0, geo)
    assert np.array_equal(g.glaetten(b, (0.0, 0.0, 0.0), 0.0, 0.083, geo), b)
