"""FloorNet: Umwandeln der ComfyUI-Zeilen in Label-Dateien (tools/floornet/comfy_npz.py)."""
import json
import os
import sys

import cv2
import numpy as np
import pytest

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

    runs, bad, weg = SN.collect(str(tmp_path / "s3"), 1, str(tmp_path))
    assert bad == 0 and list(runs["r"]) == ["000001.jpg"] and weg == {}
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
    assert (s2 / "index.js").read_text(encoding="utf-8") == 'window.BILDER = ["r/a", "r/d"];\nwindow.AUS = {};\n'


def _fahrt_mit_posen(data, posen):
    (data / "r").mkdir(parents=True, exist_ok=True)
    with open(data / "r" / "meta.jsonl", "w", encoding="utf-8") as f:
        for i, p in enumerate(posen):
            f.write(json.dumps({"file": "%06d.jpg" % i, **({"pose": p} if p else {})}) + "\n")
    return ["%06d" % i for i in range(len(posen))]


def test_stehend_keeps_first_and_every_move_or_turn(tmp_path):
    # steht, kriecht 1 cm je Bild, dreht auf der Stelle über die 360°-Grenze, ein Bild ohne Pose, steht wieder
    posen = [[0, 0, 0], [0.001, 0, 0.5], [0.01, 0, 0], [0.02, 0, 0], [0.03, 0, 0], [0.04, 0, 0],
             [0.04, 0, 179], [0.04, 0, -179], [0.04, 0, -176.5], None, [0.04, 0, -176.5], [0.045, 0, -176]]
    stems = _fahrt_mit_posen(tmp_path, posen)
    aus = S2.stehend(S2.posen(str(tmp_path), "r"), stems, 0.03, 3.0)
    #  0 bleibt, 1-3 zu nah, 4 (3 cm) bleibt, 5 zu nah, 6 (179°) bleibt, 7 (2° weiter) zu nah, 8 (4,5°) bleibt,
    #  9 ohne Pose bleibt, 10 bleibt (nach der Lücke neu), 11 zu nah
    assert sorted(aus) == ["000001", "000002", "000003", "000005", "000007", "000011"]
    assert S2.stehend(S2.posen(str(tmp_path), "r"), stems, 0.0, 0.0) == set()
    assert S2.stehend({}, stems) == set()                      # Fahrt ohne Posen: nichts fällt weg


def test_hand_liste_last_line_wins(tmp_path):
    p = tmp_path / "aus_hand.txt"
    p.write_text("r/000001\nr/000002\n-r/000001\n\nr/000003\n-r/000003\nr/000003\n", encoding="utf-8")
    assert S2.hand_liste(str(p)) == {"r/000002", "r/000003"}
    assert S2.hand_liste(str(tmp_path / "fehlt.txt")) == set() and S2.hand_liste("") == set()


def test_aussortiert_hand_first_then_next_frame_moves_up(tmp_path):
    stems = _fahrt_mit_posen(tmp_path, [[0, 0, 0]] * 3 + [[1, 0, 0]])
    ids = ["r/" + s for s in stems]
    assert S2.aussortiert(ids, str(tmp_path)) == {"r/000001": "steht", "r/000002": "steht"}
    hand = tmp_path / "aus_hand.txt"
    hand.write_text("r/000000\n", encoding="utf-8")
    # das behaltene Bild der Standfolge ist untauglich: das nächste rückt nach
    assert S2.aussortiert(ids, str(tmp_path), str(hand)) == {"r/000000": "hand", "r/000002": "steht"}


def test_stages_drop_sorted_out_frames_everywhere(tmp_path):
    data = tmp_path / "data"
    stems = _fahrt_mit_posen(data, [[0, 0, 0], [0, 0, 0], [1, 0, 0], [2, 0, 0]])
    s1, s2, s3 = tmp_path / "s1", tmp_path / "s2", tmp_path / "s3"
    for d in (s1 / "r", s2 / "r", s3 / "r"):
        d.mkdir(parents=True)
    m = np.zeros((480, 640), np.uint8)
    m[300:, :] = 255
    for s in stems:
        (s1 / "r" / (s + ".png")).write_bytes(b"x")
        (data / "r" / (s + ".jpg")).write_bytes(b"x")
        cv2.imwrite(str(s2 / "r" / (s + ".png")), m)
        cv2.imwrite(str(s3 / "r" / (s + ".png")), m)
    hand = s1 / S2.HAND
    hand.write_text("r/000003\n", encoding="utf-8")
    assert S2.write_index(str(s1), str(s2), str(data), str(hand), 0.03, 3.0) == 2
    js = (s2 / "index.js").read_text(encoding="utf-8")
    assert '"r/000001": "steht"' in js and '"r/000003": "hand"' in js and js.count("r/000002") == 1
    # Stage3 und die Label-Dateien lesen die Regeln aus info.json der Stufe davor
    (s2 / "info.json").write_text(json.dumps({"aus_hand": str(hand), "steht_m": 0.03, "steht_grad": 3.0}), encoding="utf-8")
    r = S3.aus_regeln(str(s2))
    assert r == {"hand": str(hand), "d_m": 0.03, "w_grad": 3.0}
    assert S3.aus_regeln(str(tmp_path / "leer")) == {"hand": "", "d_m": 0.0, "w_grad": 0.0}
    (s3 / "info.json").write_text((s2 / "info.json").read_text(encoding="utf-8"), encoding="utf-8")
    runs, bad, weg = SN.collect(str(s3), 1, str(data))
    assert bad == 0 and list(runs["r"]) == ["000000.jpg", "000002.jpg"] and weg == {"r": 2}


def test_stage3_npz_geometrie_aus_konfig_und_koerperhoehe_der_fahrt(tmp_path, monkeypatch, capsys):
    from xgonav.config import Config
    cfg = Config()
    cfg.camera.cam_h_u8 = [109.4, 0.1126]
    g = json.loads(SN.geometrie(cfg, 90.0))
    assert g["W"] == 640 and g["f"] == pytest.approx(320 / np.tan(np.radians(cfg.camera.hfov_deg) / 2))
    assert g["cam_h"] == pytest.approx(0.1238, abs=0.0005)                       # 109,4 + 0,1126 * 127,5
    assert json.loads(SN.geometrie(cfg, None))["cam_h"] != g["cam_h"] or cfg.behavior.height_cruise_mm == 90.0
    # ganzer Lauf ohne Dateien von label.py: Körperhöhe aus meta.jsonl, Quelle aus --data
    data, s3 = tmp_path / "data", tmp_path / "s3"
    (data / "r").mkdir(parents=True)
    (s3 / "r").mkdir(parents=True)
    with open(data / "r" / "meta.jsonl", "w", encoding="utf-8") as f:
        for i in range(3):
            cv2.imwrite(str(data / "r" / ("%06d.jpg" % i)), np.full((480, 640, 3), 120, np.uint8))
            m = np.zeros((480, 640), np.uint8)
            m[300:] = 255
            cv2.imwrite(str(s3 / "r" / ("%06d.png" % i)), m)
            f.write(json.dumps({"file": "%06d.jpg" % i, "balancing": True, "height_mm": 100.0, "pose": [float(i), 0.0, 0.0]}) + "\n")
    assert SN.koerperhoehe(str(data / "r")) == 100.0
    monkeypatch.setattr(sys, "argv", ["stage3_npz.py", str(s3), "--out", str(tmp_path / "out"), "--data", str(data), "--jobs", "1"])
    assert SN.main() == 0
    d = np.load(tmp_path / "out" / "r.npz")
    assert len(d["files"]) == 3 and str(d["source_dir"]) == os.path.abspath(str(data / "r"))
    assert json.loads(str(d["geom"]))["cam_h"] == pytest.approx(json.loads(SN.geometrie(Config(), 100.0))["cam_h"])
    assert d["row"][0] == pytest.approx(300.0)


def test_betrachter_server_appends_valid_ids_only(tmp_path):
    import threading
    import urllib.error
    import urllib.request
    from functools import partial
    from http.server import ThreadingHTTPServer

    import betrachter_server as BS

    (tmp_path / "stage1_hd" / "r").mkdir(parents=True)
    (tmp_path / "stage1_hd" / "r" / "000001.png").write_bytes(b"x")
    (tmp_path / "geheim.txt").write_text("x", encoding="utf-8")
    srv = ThreadingHTTPServer(("127.0.0.1", 0), partial(BS.Handler, directory=str(tmp_path)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:%d/" % srv.server_address[1]

    def post(d, typ="application/json", pfad="aus"):
        req = urllib.request.Request(url + pfad, json.dumps(d).encode(), {"Content-Type": typ})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    try:
        assert post({"ansicht": "hd", "id": "r/000001", "aus": True}) == 200
        assert post({"ansicht": "hd", "id": "r/000001", "aus": False}) == 200
        assert post({"ansicht": "hd", "id": "r/000001"}) == 200
        assert post({"ansicht": "hd", "id": "r/000002"}) == 400                 # Bild gibt es nicht
        assert post({"ansicht": "hd", "id": "../geheim"}) == 400
        assert post({"ansicht": "hd", "id": "r/000001\n-x/1"}) == 400
        assert post({"ansicht": "x", "id": "r/000001"}) == 400
        assert post(["r/000001"]) == 400
        assert post({"ansicht": "hd", "id": "r/000001"}, typ="text/plain") == 400
        assert post({"ansicht": "hd", "id": "r/000001"}, pfad="stage1_hd/aus_hand.txt") == 400
        with urllib.request.urlopen(url + "stage1_hd/aus_hand.txt", timeout=5) as r:        # Dateien wie bisher
            assert r.read().decode() == "r/000001\n-r/000001\nr/000001\n" and r.headers["Cache-Control"] == "no-store"
        assert post({"ansicht": "hd", "id": "r/000001"}, pfad="gesichtet") == 200           # gesichtet: eigene Liste
        assert post({"ansicht": "hd", "id": "r/000001", "aus": False}, pfad="gesichtet") == 200     # kein Zurückholen
        assert post({"ansicht": "hd", "id": "r/000002"}, pfad="gesichtet") == 400
        assert post({"ansicht": "x", "id": "r/000001"}, pfad="gesichtet") == 400
        with urllib.request.urlopen(url + "stage1_hd/gesichtet.txt", timeout=5) as r:
            assert r.read().decode() == "r/000001\nr/000001\n" and r.headers["Cache-Control"] == "no-store"
        # Sammelmeldung: untaugliche IDs fallen weg, der Rest wird geschrieben
        req = urllib.request.Request(url + "gesichtet", json.dumps({"ansicht": "hd", "ids": ["r/000001", "r/000002", "../x", 5]}).encode(),
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            assert json.loads(r.read()) == {"ok": True, "fehler": "", "n": 1, "ausgelassen": 3}
        assert post({"ansicht": "x", "ids": ["r/000001"]}, pfad="gesichtet") == 400
        assert post({"ansicht": "hd", "ids": ["r/000001"] * 501}, pfad="gesichtet") == 400
        assert (tmp_path / "stage1_hd" / "gesichtet.txt").read_text(encoding="utf-8") == "r/000001\n" * 3
        # Verbindung bleibt offen; Stage1-Bilder dürfen im Zwischenspeicher bleiben, fehlende nicht
        import http.client
        c = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
        for pfad, code, cache in (("/stage1_hd/r/000001.png", 200, "max-age=86400"), ("/stage1_hd/r/000002.png", 404, None),
                                  ("/geheim.txt", 200, None)):
            c.request("GET", pfad)
            r = c.getresponse()
            r.read()
            assert (r.status, r.getheader("Cache-Control"), r.version) == (code, cache, 11)
        c.close()
    finally:
        srv.shutdown()
        srv.server_close()
    assert S2.hand_liste(str(tmp_path / "stage1_hd" / S2.HAND)) == {"r/000001"}


def test_ids_fahrt_every_second_without_standing(tmp_path, capsys, monkeypatch):
    import ids_fahrt as IF

    stems = _fahrt_mit_posen(tmp_path, [[0, 0, 0]] * 6 + [[1, 0, 0], [1, 0, 0], [2, 0, 0]])
    for s in stems:
        (tmp_path / "r" / (s + ".jpg")).write_bytes(b"x")
    monkeypatch.setattr(sys, "argv", ["ids_fahrt.py", "r", "--data", str(tmp_path)])
    assert IF.main() == 0
    assert capsys.readouterr().out.split() == ["r/000000.jpg", "r/000006.jpg", "r/000008.jpg"]


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


def test_betrachter_server_neu_malen(tmp_path, monkeypatch):
    """POST /neu ruft das Werkzeug als eigenen Prozess (hier ein Ersatz) und reicht Seed und Labelzeile durch."""
    import sys
    import threading
    import urllib.error
    import urllib.request
    from functools import partial
    from http.server import ThreadingHTTPServer

    import betrachter_server as BS

    (tmp_path / "stage1_p11" / "r").mkdir(parents=True)
    (tmp_path / "stage1_p11" / "r" / "000001.png").write_bytes(b"x")
    ersatz = tmp_path / "ersatz.py"
    ersatz.write_text("import json, sys\nprint('Rauschen')\n"
                      "print(json.dumps({'id': sys.argv[-1], 'seed': 7, 'rows': [1.0, 2.0], 'args': sys.argv[1:]}))\n", encoding="utf-8")
    monkeypatch.setattr(BS, "NEU_CMD", [sys.executable, str(ersatz)])
    srv = ThreadingHTTPServer(("127.0.0.1", 0), partial(BS.Handler, directory=str(tmp_path)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:%d/neu" % srv.server_address[1]

    def post(d):
        req = urllib.request.Request(url, json.dumps(d).encode(), {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    try:
        code, d = post({"ansicht": "p11", "id": "r/000001"})
        assert code == 200 and d["ok"] and d["seed"] == 7 and d["rows"] == [1.0, 2.0]
        assert d["args"] == ["--dir", str(tmp_path), "--prompt", "", "p11", "r/000001"]
        code, d = post({"ansicht": "p11", "id": "r/000001", "prompt": "t2"})
        assert code == 200 and d["args"][2:4] == ["--prompt", "t2"]
        assert post({"ansicht": "p11", "id": "r/000001", "prompt": "../x"})[0] == 400
        assert post({"ansicht": "p11", "id": "r/000002"})[0] == 400             # Bild gibt es nicht: kein Prozess
        assert post({"ansicht": "p11", "id": "../x/1"})[0] == 400
        ersatz.write_text("import sys\nsys.exit('kaputt')\n", encoding="utf-8")
        code, d = post({"ansicht": "p11", "id": "r/000001"})
        assert code == 400 and "kaputt" in d["fehler"]
    finally:
        srv.shutdown()
        srv.server_close()
