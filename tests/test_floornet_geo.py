"""FloorNet: Labels aus Geometrie (tools/floornet/geo_label.py) an gerenderten Punktwolken, ohne MoGe und torch."""
import math
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools", "floornet"))

import comfy_label as C  # noqa: E402
import geo_label as G  # noqa: E402

W, H = 640, 480
F = 310.0
CAM_H = 0.128
A = math.radians(5.18)          # tilt_deg am Rider, Nick 0
BIG = 50.0


def render(kisten: list, a: float = A, cam_h: float = CAM_H, skala: float = 1.0) -> np.ndarray:
    """Punktwolke wie MoGe (Kamerakoordinaten x rechts, y unten, z vorn; nichts getroffen = inf) für achsparallele
    Kisten in Weltkoordinaten (X rechts, Y oben, Z vorn, Boden Y = 0): je Kiste (x0, x1, y0, y1, z0, z1)."""
    u, v = np.meshgrid(np.arange(W) + 0.0, np.arange(H) + 0.0)
    r = np.stack([(u - W / 2) / F, (v - H / 2) / F, np.ones_like(u)], -1)
    ca, sa = math.cos(a), math.sin(a)
    dw = np.stack([r[..., 0], -r[..., 1] * ca + sa, r[..., 1] * sa + ca], -1)      # Richtung in Weltkoordinaten
    o = np.array([0.0, cam_h, 0.0])
    t_best = np.full((H, W), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for k in kisten:
            lo, hi = np.array(k[0::2]), np.array(k[1::2])
            t1, t2 = (lo - o) / dw, (hi - o) / dw
            tn = np.nanmax(np.minimum(t1, t2), -1)
            tf = np.nanmin(np.maximum(t1, t2), -1)
            hit = (tn <= tf) & (tf > 0) & (tn > 1e-6)
            t_best = np.where(hit & (tn < t_best), tn, t_best)
    P = r * t_best[..., None] * skala          # Strahl r hat z = 1, t ist also die Tiefe
    return P


def soll_zeile(z: float, a: float = A, cam_h: float = CAM_H) -> float:
    return H / 2 + F * math.tan(math.atan2(cam_h, z) + a)


BODEN = (-BIG, BIG, -1.0, 0.0, -1.0, BIG)
WAND15 = (-BIG, BIG, -1.0, 3.0, 1.5, 2.0)


def spalten(xmax_m: float, z: float) -> np.ndarray:
    """Streifen, deren Bodenstrahl bei Entfernung z sicher innerhalb |X| < xmax_m liegt."""
    u = (np.arange(G.NCOLS) + 0.5) * W / G.NCOLS
    return np.abs((u - W / 2) / F * z) < xmax_m - 0.08


def label(P, a=A, cam_h=CAM_H):
    return G.label_bild(P, a, cam_h, F, G.Regeln(), None)


def hochbiegen(P: np.ndarray, ab: int = 380, oben_m: float = 0.09) -> np.ndarray:
    """Wie DA3 in explore11: der nahe Boden biegt sich unter Zeile ab nach oben, am Bildrand um oben_m."""
    P = P.copy()
    w = np.clip((np.arange(H) - ab) / (H - 1 - ab), 0, None)[:, None, None]
    return P - w * oben_m * G.unten(A)


def test_bent_up_floor_at_bottom_edge_counts_as_floor():
    P = hochbiegen(render([BODEN, WAND15]))
    rows, _, k = G.label_bild(P, A, CAM_H, F, G.Regeln(), None)
    assert np.abs(rows - soll_zeile(1.5)).max() < 1.5
    assert (k["klasse"][470:] == 1).all()
    rows0, _, _ = G.label_bild(P, A, CAM_H, F, G.Regeln(rand=0), None)
    assert (rows0 == H).all()                                  # ohne die Regel: rot unten, Linie am Bildrand


def test_flat_box_top_at_bottom_edge_stays_obstacle():
    kiste = (-BIG, BIG, 0.0, 0.08, 0.0, 0.30)                 # 8 cm hoch, vom Rider bis 30 cm, unten im Bild nur Decke
    rows, _, k = G.label_bild(render([BODEN, kiste, WAND15]), A, CAM_H, F, G.Regeln(), None)
    assert (rows == H).all()
    assert (k["klasse"][470:] == 2).all()


def test_frei_oben_matches_mask_rows():
    rng = np.random.default_rng(4)
    for _ in range(20):
        m = np.zeros((H, W), bool)
        for c in range(G.NCOLS):                       # je Streifen Boden von unten, dazu Lücken und Inseln
            top = rng.integers(150, H + 1)
            m[top:, c * 8:(c + 1) * 8] = True
            for _ in range(rng.integers(0, 4)):
                y = rng.integers(150, H)
                m[y:y + rng.integers(1, 12), c * 8:(c + 1) * 8] ^= True
        np.testing.assert_array_equal(G.frei_oben(m, 6), C.mask_rows(m, 6))


def test_wall_gives_its_foot_row():
    rows, eb, _ = label(render([BODEN, WAND15]))
    assert eb["art"] == "ebene" and eb["winkel"] < 0.5 and abs(eb["s"] - 1.0) < 0.01
    assert np.abs(rows - soll_zeile(1.5)).max() < 1.5


@pytest.mark.parametrize("skala", [0.8, 1.3, 1.7, 2.5])     # MoGe am Rider: Kamera 1,2-1,7x zu hoch
def test_wrong_model_scale_is_corrected_by_camera_height(skala):
    rows, eb, _ = label(render([BODEN, WAND15], skala=skala))
    assert eb["art"] == "ebene" and abs(eb["s"] - 1 / skala) < 0.01
    assert np.abs(rows - soll_zeile(1.5)).max() < 1.5


def test_box_low_sofa_edge_and_table():
    box = (-0.3, 0.3, -1.0, 0.3, 0.8, 1.0)
    sofa = (0.45, BIG, 0.10, 0.45, 0.6, 1.2)             # Kante 10 cm über dem Boden, Boden darunter sichtbar
    sofa_wand = (0.45, BIG, -1.0, 0.45, 1.2, 1.3)
    tisch = (-BIG, -0.45, 0.70, 0.75, 0.5, 1.0)          # 70 cm: Rider passt darunter
    rows, _, karten = label(render([BODEN, WAND15, box, sofa, sofa_wand, tisch]))
    u = (np.arange(G.NCOLS) + 0.5) * W / G.NCOLS
    mitte = spalten(0.3, 1.0)
    assert np.abs(rows[mitte] - soll_zeile(0.8)).max() < 1.5
    rechts = (u - W / 2) / F * 0.6 > 0.45 + 0.08         # Sofakante: Fußpunkt bei 0,6 m, nicht die Wand bei 1,2 m
    assert rechts.sum() > 5 and np.abs(rows[rechts] - soll_zeile(0.6)).max() < 1.5
    assert karten["fuss"][rechts].all()
    links = (u - W / 2) / F * 0.8 < -0.45 - 0.08         # unter dem Tisch hindurch bis zur Wand
    assert links.sum() > 5 and np.abs(rows[links] - soll_zeile(1.5)).max() < 1.5


def test_below_plane_counts_as_floor_no_drop():
    """Kein Absturz (Nutzer 2026-11-20): eine Stufe nach unten bei 1 m ist Boden, die Grenze liegt an der Wand."""
    oben = (-BIG, BIG, -1.0, 0.0, -1.0, 1.0)
    unten = (-BIG, BIG, -1.0, -0.18, 1.0, BIG)
    wand = (-BIG, BIG, -1.0, 3.0, 2.5, 3.0)
    rows, eb, karten = label(render([oben, unten, wand]))
    assert eb["art"] == "ebene"
    assert (rows < soll_zeile(1.0) - 5).all()
    assert not np.isin(karten["klasse"], [3]).any()


def test_pitch_from_imu_is_used_and_plane_refits_small_error():
    a_wahr = A + math.radians(2.0)
    P = render([BODEN, WAND15], a=a_wahr)
    rows, eb, _ = label(P, a=A)                          # IMU-Nick 2° daneben: Ebene korrigiert das
    assert eb["art"] == "ebene" and abs(eb["winkel"] - 2.0) < 0.3
    assert np.abs(rows - soll_zeile(1.5, a=a_wahr)).max() < 1.5


def test_wall_filling_frame_uses_fallback_scale():
    nah = (-BIG, BIG, -1.0, 3.0, 0.12, 0.2)              # Wand 12 cm vor der Linse: kaum Boden im Bild
    P = render([BODEN, nah], skala=1.2)
    rows, eb, _ = G.label_bild(P, A, CAM_H, F, G.Regeln(), 1 / 1.2)
    assert (rows >= soll_zeile(0.25)).all()               # Grenze ganz unten, nichts frei


@pytest.mark.parametrize("bad", [np.inf, np.nan])
def test_invalid_points_are_not_floor(bad):
    P = render([BODEN, WAND15])
    P[300:, 300:340] = bad                               # Loch in der Schätzung (Glanz): unten kein Boden
    rows, _, _ = label(P)
    c = slice(38, 42)                                    # Streifen ganz im Loch
    assert (rows[c] >= H - 1).all()


class FakeMoge:
    """Statt MoGe: jedes Bild zeigt Boden und Wand bei 1,5 m, Modellmaß 20 % zu groß."""

    def __init__(self, *a, **k):
        pass

    def punkte(self, bilder, fov_deg, ids=None):
        assert abs(fov_deg - 91.8) < 1e-6
        return np.stack([render([BODEN, WAND15], skala=1.2) for _ in bilder])


class FakeMogeWand(FakeMoge):
    """Wie FakeMoge, aber das zweite Bild jedes Pakets zeigt eine Wand 12 cm vor der Linse (keine eigene Ebene)."""

    def punkte(self, bilder, fov_deg, ids=None):
        P = super().punkte(bilder, fov_deg)
        if len(P) > 1:
            P[1] = render([BODEN, (-BIG, BIG, -1.0, 3.0, 0.12, 0.2)], skala=1.2)
        return P


def _fahrt(tmp_path, n=4):
    import cv2
    import json
    src = tmp_path / "data" / "fahrt1"
    src.mkdir(parents=True)
    rng = np.random.default_rng(1)
    with open(src / "meta.jsonl", "w") as fo:
        for i in range(n):
            name = "%06d.jpg" % (2 * i)
            cv2.imwrite(str(src / name), rng.integers(60, 200, (H, W, 3), dtype=np.uint8))
            fo.write(json.dumps({"file": name, "t": i * 0.1, "pose": [0.1 * i, 0.0, 0.0], "pitch_dev": 0.0,
                                 "balancing": True, "height_mm": 85}) + "\n")
    return src


def test_fahrten_writes_label_file(tmp_path, monkeypatch, capsys):
    src = _fahrt(tmp_path)
    monkeypatch.setattr(G, "Moge", FakeMoge)
    out = tmp_path / "geo"
    monkeypatch.setattr(sys, "argv", ["geo_label.py", "fahrten", "--runs", "fahrt1", "--data", str(tmp_path / "data"),
                                      "--out", str(out), "--bilder", "1", "--jedes", "2"])
    assert G.main() == 0
    with np.load(out / "npz" / "fahrt1.npz") as d:      # schließen, sonst ersetzt Windows die Datei unten nicht
        assert list(d["files"]) == ["000000.jpg", "000002.jpg", "000004.jpg", "000006.jpg"]
        assert np.abs(d["row"] - soll_zeile(1.5)).max() < 1.5
        assert str(d["source_dir"]) == str(src)
    assert len(list((out / "bilder").iterdir())) == 1
    # Fortsetzen: nichts neu rechnen
    monkeypatch.setattr(G, "Moge", None)
    assert G.main() == 0
    assert len(open(out / "rows.jsonl").read().splitlines()) == 4


def test_fahrten_drops_frames_without_own_plane(tmp_path, monkeypatch, capsys):
    import json
    _fahrt(tmp_path)
    monkeypatch.setattr(G, "Moge", FakeMogeWand)
    out = tmp_path / "geo"
    monkeypatch.setattr(sys, "argv", ["geo_label.py", "fahrten", "--runs", "fahrt1", "--data", str(tmp_path / "data"),
                                      "--out", str(out), "--jedes", "2"])
    assert G.main() == 0
    arten = {e["id"]: e["art"] for e in map(json.loads, open(out / "rows.jsonl").read().splitlines())}
    assert arten["fahrt1/000002.jpg"] == "ersatz"
    with np.load(out / "npz" / "fahrt1.npz") as d:
        assert list(d["files"]) == ["000000.jpg", "000004.jpg", "000006.jpg"]


def test_depth_maps_from_da3_give_same_line(tmp_path):
    """--tiefe: Tiefenkarte (z, Modellmaß 0,7) + conf aus da3_tiefe.py ergibt dieselbe Grenze wie die Punktwolke;
    Pixel unter conf_min sind ungültig (Wand reicht bis über den Bildrand)."""
    P = render([BODEN, WAND15], skala=0.7)
    t = np.where(np.isfinite(P[..., 2]), P[..., 2], 0.0)
    conf = np.ones((H, W), np.float32)
    conf[:, :W // 8] = 0.1
    os.makedirs(tmp_path / "fahrt1")
    np.savez_compressed(tmp_path / "fahrt1" / "000001.npz", tiefe=t.astype(np.float16), conf=conf.astype(np.float16))
    fov = math.degrees(2 * math.atan(W / 2 / F))
    Q = G.Tiefe(str(tmp_path), conf_min=0.5).punkte([np.zeros((H, W, 3), np.uint8)], fov, ["fahrt1/000001.jpg"])[0]
    assert np.isinf(Q[:, :W // 8]).all() and np.isfinite(Q[:, W // 8:]).all()
    rows, eb, _ = label(Q)
    assert eb["art"] == "ebene" and abs(eb["s"] - 1 / 0.7) < 0.01
    rechts = np.arange(G.NCOLS) >= G.NCOLS // 8 + 1
    assert np.abs(rows[rechts] - soll_zeile(1.5)).max() < 1.5


def test_filled_border_of_lens_table_continues_line_flat():
    """Rechts aufgefüllte Pixel der Entzerrung (Spalten ab 616, unten erst ab 627): keine Punkte daraus, die Streifen
    dort übernehmen waagrecht die Zeile des letzten echten Streifens."""
    erg = np.zeros((H, W), bool)
    erg[:, 616:] = True
    erg[400:, 616:627] = False
    P = render([BODEN, WAND15])
    P[erg] = (0.0, -0.05, 0.2)                    # Unsinn wie ein wiederholter Bildrand: nah und über dem Boden
    rows, eb, _ = G.label_bild(P, A, CAM_H, F, G.Regeln(), None, erg)
    assert eb["art"] == "ebene"
    assert np.all(rows[76:] == rows[75]) and abs(rows[75] - soll_zeile(1.5)) < 1.5     # ein Streifen Sicherheit


def test_filled_mask_from_rider_lens_table():
    from xgonav.config import Config
    erg = G.ergaenzt_maske(Config())
    assert erg is not None and not erg[:, :300].any()
    assert 10 <= erg[440].sum() <= 16 and 20 <= erg[280].sum() <= 30          # rechter Rand: unten 13, Bodenlinie ~25


def test_rider_lens_table_marks_only_right_edge_strips():
    from xgonav.config import Config
    erg = G.ergaenzt_maske(Config())
    rows = np.arange(G.NCOLS, dtype=np.float32)
    out = G.waagrecht(rows, erg, H / 2 + F * math.tan(A))
    geaendert = np.flatnonzero(out != rows)
    assert 3 <= len(geaendert) <= 5 and geaendert.min() >= 75 and np.all(out[geaendert] == out[geaendert.min() - 1])
