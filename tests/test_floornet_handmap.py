"""FloorNet: Handlinien über die Pose auf die anderen Bilder einer Fahrt."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "floornet"))

import handmap as H  # noqa: E402
import label as L  # noqa: E402
from pick import pick  # noqa: E402
from xgonav.config import Config  # noqa: E402
from xgonav.gridmap import GridMap  # noqa: E402


def _cfg():
    c = Config.neutral()
    c.camera.hfov_deg, c.camera.tilt_deg, c.camera.mount_offset_mm = 88.5, 5.18, 43.0
    c.grid.native = False
    return c


def _geo():
    return L.make_geo(_cfg())


def _wall(geo, d, pitch=0.0):
    """Zeilen einer Wand d Meter voraus (gleicher Vorwärtsabstand in allen Spalten)."""
    return np.full(L.NCOLS, float(L.ground_row(geo, d, pitch)), np.float32)


def test_linie_trifft_von_anderer_pose_die_richtige_zeile():
    geo = _geo()
    gm = GridMap(_cfg().grid, geo, 160, 120)
    hit, free = np.zeros(gm.lo.shape, np.int32), np.zeros(gm.lo.shape, np.int32)
    H.draw_line(gm, hit, free, (0.0, 0.0, 0.0), 0.0, _wall(geo, 1.0), np.ones(L.NCOLS, bool), geo)
    gm.lo[:] = np.clip(H.L_HIT * hit - H.L_FREE * free, -H.L_MAX, H.L_MAX)
    a = np.array([0.0])
    rows, kinds = L.cast_labels(gm, geo, np.array([0.3]), a, a, a)
    c = L.NCOLS // 2
    assert kinds[0, c] == L.EXACT
    assert L.row_dist(geo, rows[0, c], 0.0) == pytest.approx(0.7, abs=0.06)


def test_unter_der_linie_ist_boden_und_dahinter_nichts_bekannt():
    geo = _geo()
    gm = GridMap(_cfg().grid, geo, 160, 120)
    hit, free = np.zeros(gm.lo.shape, np.int32), np.zeros(gm.lo.shape, np.int32)
    H.draw_line(gm, hit, free, (0.0, 0.0, 0.0), 0.0, _wall(geo, 1.0), np.ones(L.NCOLS, bool), geo)
    r, k = gm.cell(0.5, 0.0)
    assert free[r, k] > 0 and hit[r, k] == 0
    r, k = gm.cell(1.0 + 3 * gm.res, 0.0)
    assert free[r, k] == 0 and hit[r, k] == 0


def test_ferne_linie_gibt_keinen_treffer_nur_freistrahl():
    geo = _geo()
    gm = GridMap(_cfg().grid, geo, 160, 120)
    hit, free = np.zeros(gm.lo.shape, np.int32), np.zeros(gm.lo.shape, np.int32)
    H.draw_line(gm, hit, free, (0.0, 0.0, 0.0), 0.0, _wall(geo, H.R_OCC + 1.0), np.ones(L.NCOLS, bool), geo)
    assert hit.sum() == 0 and free.sum() > 0


def test_unsichere_spalten_zaehlen_nicht():
    geo = _geo()
    gm = GridMap(_cfg().grid, geo, 160, 120)
    hit, free = np.zeros(gm.lo.shape, np.int32), np.zeros(gm.lo.shape, np.int32)
    H.draw_line(gm, hit, free, (0.0, 0.0, 0.0), 0.0, _wall(geo, 1.0), np.zeros(L.NCOLS, bool), geo)
    assert hit.sum() == 0 and free.sum() == 0


def _fahrt(tmp_path):
    """Drei Bilder geradeaus: 0 mit Handlinie, 1 ohne (Person links im Bild), 2 unbrauchbar."""
    geo = _geo()
    run = tmp_path / "bilder"
    run.mkdir()
    metas = []
    for i in range(3):
        f = "%06d.jpg" % i
        (run / f).write_bytes(b"x")
        tracks = [{"cls": "person", "box": [0.0, 0.0, 80.0, 400.0]}] if i == 1 else []
        metas.append({"idx": i, "t": float(i), "file": f, "tracks_all": tracks})
    (run / "meta.jsonl").write_text("\n".join(json.dumps(m) for m in metas), encoding="utf-8")
    z = {"files": np.array(["%06d.jpg" % i for i in range(3)]), "t": np.arange(3.0), "x": np.array([0.0, 0.3, 0.6]),
         "y": np.zeros(3), "th": np.zeros(3), "rate": np.zeros(3), "pitch": np.zeros(3, np.float32),
         "row": np.zeros((3, L.NCOLS), np.float32), "kind": np.ones((3, L.NCOLS), np.uint8),
         "kanten_row": np.zeros((3, 160), np.float32), "lo": np.zeros((2, 2), np.float16), "run": "t", "wohnung": "a",
         "geom": "{}", "source_dir": str(run)}
    p = tmp_path / "t.npz"
    np.savez(p, **z)
    return str(p), geo


def test_label_run_ueberträgt_und_respektiert_person_und_unbrauchbar(tmp_path, monkeypatch):
    npz, geo = _fahrt(tmp_path)
    monkeypatch.setattr(L, "make_geo", lambda cfg: geo)
    own = _wall(geo, 1.0)
    ok = np.ones(L.NCOLS, bool)
    ok[0] = False
    d = H.label_run(npz, [("000000.jpg", own, ok)], {"000002.jpg"}, _cfg())
    assert d["hand"].tolist() == [True, False, False]
    # Eigenes Bild: die Linie selbst, unsichere Spalte zählt nicht
    assert d["row"][0, 40] == pytest.approx(own[40])
    assert d["kind"][0, 0] == L.IGNORE and d["kind"][0, 40] == L.EXACT
    # Anderes Bild: Wand ist näher, Spalten mit Person zählen nicht
    assert d["kind"][1, 40] == L.EXACT
    assert L.row_dist(geo, d["row"][1, 40], 0.0) == pytest.approx(0.7, abs=0.06)
    assert d["kind"][1, 0] == L.IGNORE
    # Unbrauchbar bleibt draußen
    assert (d["kind"][2] == L.IGNORE).all()


def test_pick_mit_kleinerer_mindestabdeckung():
    x = np.arange(5) * 0.1
    z = np.zeros(5)
    kind = np.zeros((5, 10), np.uint8)
    kind[:, :4] = 1                     # 40 % der Spalten beschriftet
    assert len(pick(x, z, z, z, kind)) == 0
    assert len(pick(x, z, z, z, kind, min_labeled=0.3)) == 5
