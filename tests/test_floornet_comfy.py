"""FloorNet: Umwandeln der ComfyUI-Zeilen in Label-Dateien (tools/floornet/comfy_npz.py)."""
import json
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools", "floornet"))

import comfy_batch as B  # noqa: E402
import comfy_npz as Z  # noqa: E402
import hand_npz as HN  # noqa: E402
import inseln as I  # noqa: E402
import stage2 as S2  # noqa: E402
import stage3 as S3  # noqa: E402
import stage3_npz as SN  # noqa: E402

GUT = np.full(80, 280.0, np.float32)


def test_inseln_counts_known_mask():
    m = np.zeros((480, 640), bool)
    m[300:480, :] = True                    # Hauptfläche, berührt den unteren Rand
    m[350:360, 100:110] = False             # Loch 100 px
    m[100:105, 200:210] = True              # Insel 50 px über der Grenze
    m[200:230, 400:430] = True              # Insel 900 px
    r = I.analyse(m)
    assert r["haupt"] and sorted(r["inseln"]) == [50, 900] and r["loecher"] == [100]
    assert r["lage"] == ["ueber", "ueber"]
    assert [I.area_class(a) for a in (49, 50, 199, 200, 999, 1000)] == [0, 1, 1, 2, 2, 3]


def test_inseln_without_bottom_and_border_gap_is_no_hole():
    m = np.zeros((480, 640), bool)
    m[100:200, 100:200] = True              # berührt den unteren Rand nicht
    r = I.analyse(m)
    assert not r["haupt"] and r["inseln"] == [10000] and r["loecher"] == []
    # Nicht-Grün am Bildrand ist kein Loch, auch wenn sonst alles grün ist
    v = np.ones((480, 640), bool)
    v[0:10, 0:10] = False
    r = I.analyse(v)
    assert r["haupt"] and r["inseln"] == [] and r["loecher"] == []
    assert I.analyse(np.zeros((480, 640), bool))["inseln"] == []


def test_g0h_removes_green_and_keeps_gray_brightness():
    img = np.array([[[50, 200, 100], [0, 200, 0], [0, 0, 0], [255, 255, 255]]], np.uint8)     # BGR
    got = B.g0h_image(img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(int)
    assert (got[..., 1] == 0).all()
    assert (np.abs(got[..., [0, 2]].max(axis=2) - gray) <= 1).all()     # größerer Kanal = Grauwert
    assert abs(int(got[0, 0, 0]) * 2 - int(got[0, 0, 2])) <= 2          # Verhältnis B : R bleibt 1 : 2
    assert got[0, 1, 0] == got[0, 1, 2] == gray[0, 1]                   # rein grün: R = B = Grau
    assert got[0, 2].max() == 0


def test_gray_png_drops_color(tmp_path):
    img = np.zeros((8, 8, 3), np.uint8)
    img[:, :] = (10, 200, 30)               # Grünstich
    p = tmp_path / "a.png"
    cv2.imwrite(str(p), img)
    got = cv2.imdecode(np.frombuffer(B.gray_png(str(p)), np.uint8), cv2.IMREAD_UNCHANGED)
    assert got.shape == (8, 8) and (got == cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)).all()


def test_floor_mask_keeps_dark_green_but_not_near_black():
    img = np.zeros((480, 640, 3), np.uint8)
    img[:160, :] = (0, 255, 0)              # hell
    img[160:320, :] = (0, 40, 0)            # Schatten auf dem Boden
    img[320:, :] = (0, 20, 0)               # fast schwarz, Farbton nur Rauschen
    m = S2.C.floor_mask(img)
    assert m[80, 300] and m[240, 300] and not m[400, 300]


def test_stage3_rows_swallow_thin_keep_shoe_and_reach_build_run(tmp_path):
    m = np.zeros((480, 640), bool)
    m[300:, :] = True                       # Boden ab Zeile 300
    m[:, 83:85] = False                     # 2 px breit senkrecht: verschluckt
    m[400:, 240:296] = False                # Schuh, 7 Streifen breit: bleibt
    stage3 = tmp_path / "s3" / "r"
    stage3.mkdir(parents=True)
    cv2.imwrite(str(stage3 / "000001.png"), S3.from_bottom(m).astype(np.uint8) * 255)
    r = SN.rows_of(str(stage3 / "000001.png"))
    assert r[10] == 300 and r[0] == 300
    assert (r[30:37] == 480).all()          # am Fuß blockiert: unten schwarz, die Spalte ist ganz schwarz
    assert r[29] == 300 and r[37] == 300
    assert r.dtype == np.float32

    runs, bad = SN.collect(str(tmp_path / "s3"), 1)
    assert bad == 0 and list(runs["r"]) == ["000001.jpg"]
    src = tmp_path / "fahrt"
    src.mkdir()
    cv2.imwrite(str(src / "000001.jpg"), np.full((48, 64, 3), 120, np.uint8))
    (src / "meta.jsonl").write_text(json.dumps({"file": "000001.jpg", "t": 1.0, "balancing": True, "pose": [1.0, 2.0, 0.0], "pitch_dev": 0.1}) + "\n", encoding="utf-8")
    d, count = Z.build_run("r", runs["r"], "{}", str(src), "a")
    assert count == {} and list(d["files"]) == ["000001.jpg"] and d["row"][0, 10] == 300 and d["kind"].all()


def test_stage3_stops_at_first_black_from_bottom():
    m = np.array([[1, 1, 0, 1],
                  [1, 0, 1, 1],
                  [1, 1, 1, 0],
                  [1, 1, 1, 1]], bool)
    want = np.array([[1, 0, 0, 0],
                     [1, 0, 1, 0],
                     [1, 1, 1, 0],
                     [1, 1, 1, 1]], bool)
    assert (S3.from_bottom(m) == want).all()
    leer = m.copy()
    leer[-1, 0] = False                     # schwarz am unteren Rand: Die Spalte ist ganz schwarz
    assert not S3.from_bottom(leer)[:, 0].any()


def test_floor_mask_green_lead_rejects_tint():
    img = np.zeros((480, 640, 3), np.uint8)
    img[:240, :] = (5, 70, 5)               # gemalt, dunkel: Vorsprung 65
    img[240:, :] = (16, 32, 7)              # Grünstich der Kamera (BGR): Vorsprung 16
    m = S2.C.floor_mask(img, 30)
    assert m[100, 300] and not m[400, 300]


def test_stage2_index_lists_only_complete_frames(tmp_path):
    s1, s2, data = tmp_path / "s1", tmp_path / "s2", tmp_path / "data"
    for d, names in ((s1 / "r", ["a.png", "b.png", "d.png"]), (s2 / "r", ["a.png", "b.png", "c.png", "d.png"]), (data / "r", ["a.jpg", "c.jpg", "d.jpg"])):
        d.mkdir(parents=True)
        for n in names:
            (d / n).write_bytes(b"x")
    # b ohne Original, c ohne Stage1
    assert S2.write_index(str(s1), str(s2), str(data)) == 2
    assert (s2 / "index.js").read_text(encoding="utf-8") == 'window.BILDER = ["r/a", "r/d"];\n'


def test_stage2_writes_floor_mask_and_skips_existing(tmp_path):
    src = tmp_path / "s1" / "r"
    src.mkdir(parents=True)
    img = np.zeros((480, 640, 3), np.uint8)
    img[240:, :] = (0, 255, 0)
    cv2.imwrite(str(src / "a.png"), img)
    jobs, n = S2.jobs_for(str(tmp_path / "s1"), str(tmp_path / "s2"))
    assert n == 1 and len(jobs) == 1
    assert S2.make_mask(jobs[0]) == "neu"
    got = cv2.imread(jobs[0][1], cv2.IMREAD_GRAYSCALE) > 0
    assert (got == S2.C.floor_mask(img)).all() and got[300, 300] and not got[100, 300]
    assert S2.jobs_for(str(tmp_path / "s1"), str(tmp_path / "s2"))[0] == []


def test_edit_archive_moves_comfy_file_and_finds_missing(tmp_path):
    ok, png = cv2.imencode(".png", np.full((8, 8, 3), 90, np.uint8))
    buf = png.tobytes()
    src = tmp_path / "comfy.png"
    src.write_bytes(buf)
    edits = str(tmp_path / "edits")
    a = B.edit_path(edits, "r/000001.jpg")
    assert a == os.path.join(edits, "r", "000001.png")
    B.keep_edit(buf, str(src), a)
    assert not src.exists() and open(a, "rb").read() == buf
    # ComfyUIs Datei nicht erreichbar: die abgeholten Bytes zählen
    b = B.edit_path(edits, "r/000002.jpg")
    B.keep_edit(buf, str(tmp_path / "fehlt.png"), b)
    assert open(b, "rb").read() == buf and not os.path.exists(b + ".tmp")
    # Abgebrochene Datei gilt als fehlend, Reihenfolge bleibt
    c = B.edit_path(edits, "r/000003.jpg")
    with open(c, "wb") as fo:
        fo.write(buf[:10])
    assert B.missing_edits(["r/000004.jpg", "r/000001.jpg", "r/000003.jpg", "r/000002.jpg"], edits) == ["r/000004.jpg", "r/000003.jpg"]


def test_hand_npz_keeps_only_lined_frames():
    n = 4
    z = {"files": np.array(["a.jpg", "b.jpg", "c.jpg", "d.jpg"]), "t": np.arange(n, dtype=float), "x": np.zeros(n), "y": np.zeros(n),
         "th": np.zeros(n), "rate": np.zeros(n), "pitch": np.zeros(n, np.float32), "kanten_row": np.zeros((n, 160), np.float32),
         "row": np.zeros((n, 80), np.float32), "kind": np.zeros((n, 80), np.uint8), "lo": np.zeros((2, 2)), "run": np.array("r"),
         "geom": np.array("{}"), "source_dir": np.array("s"), "wohnung": np.array("a")}
    ok = np.ones(80, bool)
    ok[:5] = False
    d = HN.build_run(z, {"c.jpg": (GUT, ok), "fehlt.jpg": (GUT, ok)})
    assert list(d["files"]) == ["c.jpg"] and d["hand"].all() and "lo" not in d
    assert d["row"][0, 0] == 280.0 and d["kind"][0, 0] == HN.IGNORE and d["kind"][0, 5] == HN.EXACT
    assert d["kanten_row"].shape == (1, 160)


def test_frame_reason_filters():
    assert Z.frame_reason(GUT, True, 80.0) == ""
    assert Z.frame_reason(GUT, False, 80.0) == "liegt"
    assert Z.frame_reason(GUT, True, 5.0) == "dunkel"
    kipp = GUT.copy()
    kipp[:20] = 90.0
    assert Z.frame_reason(kipp, True, 80.0) == "kippt"
    # Einzelne Spalte über dem Bereich ist kein Kippen
    einzeln = GUT.copy()
    einzeln[5] = 90.0
    assert Z.frame_reason(einzeln, True, 80.0) == ""
    zack = GUT.copy()
    zack[10:30:2] = 400.0
    assert Z.frame_reason(zack, True, 80.0) == "zerrissen"


def test_load_rows_skips_cut_last_line(tmp_path):
    p = tmp_path / "rows.jsonl"
    p.write_text(json.dumps({"id": "a/1.jpg", "rows": [280.0] * 80, "s": 1.0}) + "\n" + '{"id": "a/2.jp', encoding="utf-8")
    r = Z.load_rows(str(p))
    assert list(r) == ["a"] and list(r["a"]) == ["1.jpg"]


def test_build_run_fields(tmp_path):
    src = tmp_path / "fahrt"
    src.mkdir()
    hell = np.full((48, 64, 3), 120, np.uint8)
    frames, lines = {}, []
    for i, (bal, img) in enumerate([(True, hell), (False, hell), (True, hell // 40)]):
        name = "%06d.jpg" % i
        cv2.imwrite(str(src / name), img)
        frames[name] = GUT.copy()
        lines.append(json.dumps({"file": name, "t": float(i), "balancing": bal, "pose": [1.0 + i, 2.0, 90.0], "pitch_dev": 0.25}))
    frames["000000.jpg"][3] = 100.0
    (src / "meta.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    d, count = Z.build_run("x", frames, "{}", str(src), "a")
    assert count == {"liegt": 1, "dunkel": 1}
    assert list(d["files"]) == ["000000.jpg"]
    assert d["row"].shape == (1, 80) and d["kanten_row"].shape == (1, Z.KANTEN_COLS)
    # Spalte über dem Bereich des Netzes zählt nicht, der Rest ist exakt
    assert d["kind"][0, 3] == Z.IGNORE and d["kind"][0, 4] == Z.EXACT
    assert abs(d["th"][0] - np.pi / 2) < 1e-9 and abs(d["pitch"][0] - 0.25) < 1e-6


def test_kette_waehlt_nach_helligkeit_und_verlinkt_dunkle(tmp_path):
    hell = np.full((480, 640, 3), 61, np.uint8)
    hell[:137] = 0                                   # oben dunkel zählt nicht
    dunkel = np.full((480, 640, 3), 59, np.uint8)
    dunkel[:137] = 255                               # helles Fenster oben macht die Szene nicht hell
    cv2.imwrite(str(tmp_path / "h.png"), hell)
    cv2.imwrite(str(tmp_path / "d.png"), dunkel)
    assert B.kette(str(tmp_path / "h.png")) == ("farbe", 61.0)
    assert B.kette(str(tmp_path / "d.png")) == ("g0h", 59.0)
    assert B.kette(str(tmp_path / "d.png"), 50)[0] == "farbe"
    # Dunkle Bilder: vorhandene Bearbeitung übernehmen, fehlende oder kaputte nicht
    alt, neu = str(tmp_path / "alt"), str(tmp_path / "neu")
    os.makedirs(os.path.join(alt, "r"))
    cv2.imwrite(B.edit_path(alt, "r/000001.jpg"), hell)
    with open(B.edit_path(alt, "r/000002.jpg"), "wb") as fo:
        fo.write(b"kaputt")
    assert B.link_edit(B.edit_path(alt, "r/000001.jpg"), B.edit_path(neu, "r/000001.jpg"))
    assert (cv2.imread(B.edit_path(neu, "r/000001.jpg")) == hell).all() and os.path.exists(B.edit_path(alt, "r/000001.jpg"))
    assert not B.link_edit(B.edit_path(alt, "r/000002.jpg"), B.edit_path(neu, "r/000002.jpg"))
    assert not B.link_edit(B.edit_path(alt, "r/000003.jpg"), B.edit_path(neu, "r/000003.jpg"))
    assert B.missing_edits(["r/000001.jpg", "r/000002.jpg"], neu) == ["r/000002.jpg"]
