"""FloorNet: Auswahl des Prüfsatzes und Auswertung der Handlinie."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "floornet"))

import eval_hand as E  # noqa: E402
import make_testset as M  # noqa: E402


def test_linie_wird_je_spalte_interpoliert():
    rows, ok = E.polyline_rows([[0, 100], [639, 300]], ncols=80)
    assert ok.all()
    assert rows[0] == pytest.approx(100 + 200 * 4 / 639, abs=0.01)
    assert rows[-1] == pytest.approx(100 + 200 * 636 / 639, abs=0.01)
    assert (np.diff(rows) > 0).all()


def test_unsicherer_abschnitt_zaehlt_nicht():
    rows, ok = E.polyline_rows([[0, 300], [320, 300, 1], [639, 300]], ncols=80)
    assert ok[:40].all() and not ok[40:].any()
    assert np.isfinite(rows).all()


def test_zu_kurze_linie_zaehlt_gar_nicht():
    rows, ok = E.polyline_rows([[10, 300]], ncols=80)
    assert not ok.any() and np.isnan(rows).all()


def test_nur_fertige_und_brauchbare_bilder(tmp_path):
    p = tmp_path / "b.json"
    p.write_text(json.dumps({"bilder": {
        "a/1.jpg": {"pts": [[0, 300], [639, 300]], "fertig": True},
        "a/2.jpg": {"pts": [[0, 300], [639, 300]], "fertig": False},
        "a/3.jpg": {"pts": [], "fertig": True, "unbrauchbar": True},
    }}), encoding="utf-8")
    assert list(E.load_hand(str(p))) == ["a/1.jpg"]


def test_fehlerquoten_je_bild():
    err = np.array([[0.2, -0.2, 0.0, np.nan], [0.0, 0.0, 0.0, 0.0]])
    m = np.array([[True, True, True, False], [True, True, False, False]])
    n, fr, bl = E.rates(err, m, 0.15)
    assert n.tolist() == [3.0, 2.0] and fr.tolist() == [1.0, 0.0] and bl.tolist() == [1.0, 0.0]


def test_verhaeltnis_stimmt_und_intervall_umschliesst_es():
    n = np.full(40, 10.0)
    a = np.ones(40)            # Quote 10 %
    b = np.full(40, 2.0)       # Quote 20 %
    r, lo, hi = E.ratio_ci(a, b, n, n)
    assert r == pytest.approx(0.5)
    assert lo <= r <= hi


def test_auswahl_nimmt_verschiedene_blicke_und_keine_schnelle_drehung():
    x = np.arange(200) * 0.01             # 2 m in 1-cm-Schritten: Kandidaten alle 10 cm
    y, th = np.zeros(200), np.zeros(200)
    rate = np.zeros(200)
    rate[50:60] = 90.0
    got = M.choose(x, y, th, rate, 5, np.random.default_rng(1))
    assert len(got) == 5 and (np.diff(got) >= 10).all()
    assert not ((got >= 50) & (got < 60)).any()
    alle = M.choose(x, y, th, rate, 1000, np.random.default_rng(1))    # weniger Kandidaten als verlangt
    assert 15 <= len(alle) <= 20


def test_laufangabe():
    assert M.parse_runs("explore9:80,unten1:130") == [("explore9", 80), ("unten1", 130)]


def test_werkzeug_liegt_neben_dem_skript():
    assert (Path(M.__file__).with_name("beschriften.html")).exists()


def test_schnitt_legt_zeitfenster_als_lauf_an(tmp_path):
    import schnitt as S
    src, dst = tmp_path / "fahrt", tmp_path / "fahrt_b"
    src.mkdir()
    with open(src / "meta.jsonl", "w", encoding="utf-8") as f:
        for i in range(10):
            (src / ("%06d.jpg" % i)).write_bytes(b"jpg%d" % i)
            f.write(json.dumps({"idx": i, "t": 100.0 + i, "file": "%06d.jpg" % i}) + "\n")
        f.write('{"idx": 10, "t": 1')                    # abgeschnittene letzte Zeile
    assert S.schneide(str(src), str(dst), 3.0, 6.0) == 3
    lines = [json.loads(x) for x in (dst / "meta.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [m["idx"] for m in lines] == [3, 4, 5]
    assert (dst / "000004.jpg").read_bytes() == b"jpg4" and not (dst / "000006.jpg").exists()
    with pytest.raises(FileExistsError):
        S.schneide(str(src), str(dst), 0.0, 2.0)          # nie überschreiben


def test_hoehengruppen_und_kamerahoehe():
    body, drop = E.body_heights({"85": ["a/1.jpg"], "107.1": ["a/2.jpg", "b/3.jpg"], "unsicher": ["a/4.jpg"]})
    assert body == {"a/1.jpg": 85.0, "a/2.jpg": 107.1, "b/3.jpg": 107.1} and drop == {"a/4.jpg"}
    ch = E.cam_heights(0.128, ["a/1.jpg", "a/2.jpg", "a/9.jpg"], body)
    assert ch == pytest.approx([0.128, 0.128 + 0.0221, 0.128])
