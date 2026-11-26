"""3D-Ansicht des Simulators (tools/sim3d.py): Umrechnung Karte -> Welt, Kartenausschnitt, Zeilenformat."""
import base64
import json
import math
import sys
import types

sys.path.insert(0, ".")
import numpy as np

from tools.sim3d import Feed, Hub, map_image, map_to_world
from xgonav.sim import SimScene


def test_map_to_world_dreht_und_verschiebt():
    (x, y), = map_to_world((1.0, 2.0, math.pi / 2), [(1.0, 0.0)])
    assert abs(x - 1.0) < 1e-9 and abs(y - 3.0) < 1e-9


def test_map_image_ausschnitt():
    lo = np.zeros((100, 100), np.float32)
    lo[40, 60] = 2.0           # belegt
    lo[45, 55] = -2.0          # frei
    m = map_image(lo, 0.7, 0.05, pad=2)
    assert (m["r0"], m["c0"], m["rows"], m["cols"]) == (38, 53, 10, 10)
    z = np.frombuffer(base64.b64decode(m["zellen"]), np.uint8).reshape(10, 10)
    assert z[40 - 38, 60 - 53] == 2 and z[45 - 38, 55 - 53] == 1 and z.sum() == 3
    assert map_image(np.zeros((10, 10), np.float32), 0.7, 0.05) is None


def test_feed_zeilen():
    gm = types.SimpleNamespace(x=0.5, y=0.0, th=0.1, plan=[(1.0, 0.0)], route=[], res=0.05,
                               lo=np.zeros((20, 20), np.float32), cfg=types.SimpleNamespace(l_show=0.7))
    gm.lo[3, 4] = 2.0
    world = types.SimpleNamespace(scene=SimScene(), person=None, dark_zones=[], collisions=0, pitch=0.01,
                                  cam_h=0.13, x=1.0, y=1.2, th=0.2,
                                  log=[{"t": 0.0, "x": 1.0, "y": 1.2, "th": 0.2, "v": 0, "w": 0, "px": None, "py": None}])
    app = types.SimpleNamespace(cfg=types.SimpleNamespace(camera=types.SimpleNamespace(hfov_deg=91.8)),
                                _cmd=types.SimpleNamespace(status="x"), behavior=types.SimpleNamespace(name="explore"),
                                gridmap=gm, snapshot=types.SimpleNamespace(frame=None, idx=0))
    hub = Hub()
    f = Feed(world, app, "explore", hub=hub)
    f._emit(f.head())
    f._emit(f.tick(0.0))
    k, t = (json.loads(s) for s in hub.lines)
    assert k["typ"] == "kopf" and k["size"] == 6.0 and k["start"] == [1.0, 1.2, 0.2]
    assert t["typ"] == "takt" and "karte" in t and "bild" not in t
    assert abs(t["schaetz"][0] - (1.0 + 0.5 * math.cos(0.2))) < 1e-3 and abs(t["schaetz"][2] - 0.3) < 1e-6
    assert len(t["plan"]) == 1
