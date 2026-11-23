"""Kopf je Bild im WebSocket (xgonav/web.py ws_kopf) und Horizontlinie im Overlay."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.geometry import CamGeometry  # noqa: E402


def test_ws_kopf_horizontzeilen():
    """row_stand/row_imu: Normallage gegen den reinen IMU-Nick des Bildes, ohne IMU NaN (Browser verschiebt dann nicht)."""
    from xgonav.app import Snapshot
    from xgonav.imu import ImuFrame
    from xgonav.web import _hz_rows
    geo = CamGeometry(640, 480, 91.8, 0.0, 43.0, 85.0)
    fr = lambda p: ImuFrame(0.0, True, True, p, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert all(math.isnan(v) for v in _hz_rows(Snapshot(), geo))
    st, imu = _hz_rows(Snapshot(imu=fr(0.02)), geo)
    assert st == geo.horizon_row(0.0) and imu == geo.horizon_row(0.02) and imu > st


def test_render_horizont_mit_roll():
    """Overlay: cyan Horizontlinie ungeklemmt durch (cx, Zeile), bei Roll + rechts höher."""
    from xgonav.app import Snapshot, render
    frame = np.zeros((480, 640, 3), np.uint8)
    img = render(Snapshot(frame=frame), text=False, horizont=(430.0, 5.0, 320.0))
    cyan = (img[..., 0] > 120) & (img[..., 2] == 0)         # geglättet (LINE_AA): Blau stark, Rot 0
    ys = lambda x: 300 + np.flatnonzero(cyan[300:, x])          # oben Text und Nickmarke (Zeile 240)
    assert abs(ys(320).mean() - 430) <= 1                         # nicht auf 0,8 * 480 = 384 geklemmt
    k = math.tan(math.radians(5.0))
    assert abs(ys(20).mean() - (430 + 300 * k)) <= 1.5             # links tiefer
    assert abs(ys(620).mean() - (430 - 300 * k)) <= 1.5            # rechts höher
    assert render(Snapshot(frame=frame), text=False, horizont=(float("nan"), 0.0, 320.0)) is not None


def test_ws_kopf_imu_je_bild():
    """/ws?nick=2: Kopf aus dem IMU-Wert des Bildes (Snapshot.imu), 44 Byte; ohne gültige IMU Zustand 0."""
    import struct
    from xgonav.app import Snapshot
    from xgonav.imu import ImuFrame
    from xgonav.web import ws_kopf
    geo = CamGeometry(640, 480, 91.8, 0.0, 43.0, 85.0)
    fr = ImuFrame(1.0, True, True, math.radians(1.0), -2.0, math.radians(30.0), -3.0, 1.0, 4.0)
    k = ws_kopf(Snapshot(idx=7, imu=fr, nickrate=(5.0, 4.5)), geo, (250.0, -2.0, 320.0))
    assert len(k) == 44
    v = struct.unpack("!I10f", k)
    assert v[0] == 7 and v[3] == -2.0 and v[4] == 320.0
    assert v[2] == pytest.approx(geo.horizon_row(math.radians(1.0)), abs=1e-3)
    assert v[5] == pytest.approx(1.0) and v[6] == pytest.approx(30.0) and v[7] == 4.0
    assert (v[8], v[9], v[10]) == (5.0, 4.5, 1.0)
    ohne = struct.unpack("!I10f", ws_kopf(Snapshot(idx=8, imu=ImuFrame(1.0, False, True, None, 0.0, None, 0.0, 0.0, 400.0)),
                                          geo, (250.0, 0.0, 320.0)))
    assert ohne[10] == 0.0 and math.isnan(ohne[5]) and math.isnan(ohne[2])
    keine = struct.unpack("!I10f", ws_kopf(Snapshot(idx=9), geo, (250.0, 0.0, 320.0)))
    assert math.isnan(keine[10])


def test_ws_kopf_mit_pose():
    """/ws?nick=3: dieselben 10 Werte, dazu Horizontzeile bei cx und Kartenpose (Planerbahn im Browser); ohne Karte NaN."""
    import struct
    from xgonav.app import Snapshot
    from xgonav.web import ws_kopf
    geo = CamGeometry(640, 480, 91.8, 0.0, 43.0, 85.0)
    k = ws_kopf(Snapshot(idx=3), geo, (251.5, 1.0, 320.0), (1.5, -0.25, 0.5))
    assert len(k) == 60
    v = struct.unpack("!I14f", k)
    alt = struct.unpack("!I10f", ws_kopf(Snapshot(idx=3), geo, (251.5, 1.0, 320.0)))
    assert all(a == b or (math.isnan(a) and math.isnan(b)) for a, b in zip(v[:11], alt))
    assert v[11:] == (251.5, 1.5, -0.25, 0.5)
    ohne = struct.unpack("!I14f", ws_kopf(Snapshot(idx=3), geo, (251.5, 1.0, 320.0), None))
    assert ohne[11] == 251.5 and all(math.isnan(x) for x in ohne[12:])
