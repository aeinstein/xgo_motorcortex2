"""Gesichtserkennung (xgonav/gesicht.py): Ausrichten, Vergleich, Anlernen, Detektor-Nachlauf, Tracker, pet-Begrüßung.
Die echten Netze (YuNet, MobileFaceNet/ncnn) nur, wenn die Modelle da sind."""
import base64
import json
import os
import sys
import time
import types
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from xgonav.config import Config, GesichtCfg, PetCfg  # noqa: E402
from xgonav.detector import Detection, DetectorWorker  # noqa: E402
from xgonav.gesicht import SEITE, VORLAGE, Gesichter, GesichtFehler, ausrichten, kopf_box  # noqa: E402
from xgonav.pet import PetBrain, Seen, Sense  # noqa: E402
from xgonav.tracker import Tracker  # noqa: E402


def einheit(*v):
    v = np.asarray(v, np.float32)
    return v / np.linalg.norm(v)


def fake(tmp_path, gesicht_fuer=lambda img: einheit(1, 0, 0), **kw):
    """Gesichter mit Ersatznetzen: das Gesicht füllt die Bildmitte, das Merkmal liefert gesicht_fuer(Ausschnitt)."""
    def finder(img):
        h, w = img.shape[:2]
        s = 0.6 * min(w, h)
        x0, y0 = (w - s) / 2, (h - s) / 2
        pts = VORLAGE / SEITE * s + (x0, y0)
        return [((x0, y0, s, s), pts, 0.9)]
    cfg = GesichtCfg(dir=str(tmp_path / "gesichter"), **kw)
    g = Gesichter(cfg, finder=finder, netz=gesicht_fuer)
    assert g.start()
    return g


def test_ausrichten_bildet_vorlage_ab():
    muster = np.zeros((SEITE, SEITE, 3), np.uint8)
    cv2.circle(muster, (38, 52), 6, (255, 255, 255), -1)
    cv2.rectangle(muster, (40, 85), (72, 95), (0, 0, 255), -1)
    # dasselbe Muster doppelt so groß, verschoben
    gross = np.zeros((400, 400, 3), np.uint8)
    gross[50:50 + 2 * SEITE, 30:30 + 2 * SEITE] = cv2.resize(muster, (2 * SEITE, 2 * SEITE), interpolation=cv2.INTER_NEAREST)
    al = ausrichten(gross, VORLAGE * 2 + (30, 50))
    assert al.shape == (SEITE, SEITE, 3)
    assert np.mean(np.abs(al.astype(int) - muster.astype(int))) < 8


def test_kopf_box():
    assert kopf_box((100, 50, 200, 450), 640, 480) == (90, 40, 210, 250)       # stehend: obere Hälfte
    x1, y1, x2, y2 = kopf_box((100, 200, 400, 400), 640, 480)                   # sitzend/breit: ganze Höhe
    assert (y1, y2) == (170, 400)
    assert kopf_box((0, 0, 4, 4), 640, 480) is None


def test_wer_schwelle(tmp_path):
    g = fake(tmp_path, match_sim=0.5)
    g.merkmale = {"anna": np.stack([einheit(1, 0, 0), einheit(0, 1, 0)]), "bob": einheit(0, 0, 1)[None]}
    assert g.wer(einheit(0.1, 1, 0))[0] == "anna"               # Maximum über ihre Fotos
    assert g.wer(einheit(0, 0.2, 1))[0] == "bob"
    name, sim = g.wer(einheit(1, 1, 1))                          # alle ~0,58 -> über 0,5
    assert name is not None and sim == pytest.approx(0.577, abs=1e-3)
    g.cfg.match_sim = 0.7
    assert g.wer(einheit(1, 1, 1))[0] is None


def test_anlernen_pruefen_loeschen(tmp_path):
    vec = {"v": einheit(1, 0, 0)}
    g = fake(tmp_path, gesicht_fuer=lambda img: vec["v"], min_face_px=20)
    img = np.full((300, 300, 3), 128, np.uint8)
    assert g.pruefen(img, (0, 0, 300, 600)) is None              # noch niemand angelernt: gar nicht erst prüfen
    nr, jpg = g.anlernen(img, "anna")
    assert nr == "00" and jpg[:2] == b"\xff\xd8"
    assert g.anlernen(img, "anna")[0] == "01"
    assert (tmp_path / "gesichter" / "anna" / "01.jpg").is_file()
    with pytest.raises(GesichtFehler):
        g.anlernen(img, "../x")
    g2 = fake(tmp_path)                                          # von der Platte
    assert g2.merkmale["anna"].shape == (2, 3)
    assert g.pruefen(img, (0, 0, 300, 600))[0] == "anna"
    vec["v"] = einheit(0, 1, 0)
    assert g.pruefen(img, (0, 0, 300, 600))[0] is None
    g.cfg.min_face_px = 500
    assert g.pruefen(img, (0, 0, 300, 600)) is None              # Gesicht zu klein
    assert g.loeschen("anna", "00") and g.merkmale["anna"].shape == (1, 3)
    assert not g.loeschen("anna", "07")
    assert g.loeschen("anna") and "anna" not in g.merkmale
    assert not (tmp_path / "gesichter" / "anna").exists()


def test_gruss_je_person(tmp_path):
    g = fake(tmp_path, min_face_px=20)
    img = np.full((300, 300, 3), 128, np.uint8)
    assert not g.gruss_setzen("anna", "cute", "")                 # noch nicht angelernt
    g.anlernen(img, "anna")
    assert g.gruss("anna") == {}
    assert not g.gruss_setzen("anna", "cu te", "")
    assert not g.gruss_setzen("anna", "", "../x")
    assert not g.gruss_setzen("anna", "", "", "../rot")
    assert g.gruss_setzen("anna", "cute", "stimme:03", "rot")
    assert g.gruss("anna") == {"emotion": "cute", "sound": "stimme:03", "augen": "rot"}
    assert fake(tmp_path).gruss("anna") == g.gruss("anna")          # von der Platte
    assert g.gruss_setzen("anna", "", "-") and g.gruss("anna") == {"sound": "-"}
    assert g.gruss_setzen("anna") and not (tmp_path / "gesichter" / "anna" / "gruss.json").exists()
    g.gruss_setzen("anna", "cute", "")
    assert g.loeschen("anna", "00")                                 # letztes Foto weg: Begrüßung auch
    assert not (tmp_path / "gesichter" / "anna").exists()


def test_detektor_prueft_nur_unbenannte_und_begrenzt(tmp_path):
    calls = []

    class G:
        cfg = GesichtCfg(max_hz=2.0)

        def pruefen(self, bgr, box):
            calls.append(box)
            return ("anna", 0.8)
    d = DetectorWorker(Config.neutral().detector)
    d.gesichter = G()
    img = np.zeros((480, 640, 3), np.uint8)
    dets = [Detection((0, 0, 100, 300), "person", 0.9), Detection((300, 0, 400, 300), "person", 0.9),
            Detection((500, 0, 600, 100), "chair", 0.9)]
    d._faces(img, dets, named=[(300, 0, 400, 300)])
    assert calls == [(0, 0, 100, 300)] and dets[0].who == "anna" and dets[1].who is None
    d._faces(img, dets, named=[])                                # zu früh: max_hz
    assert len(calls) == 1
    d._face_t -= 1.0
    d._faces(img, dets, named=[])
    assert len(calls) == 3


def test_tracker_behaelt_namen():
    tr = Tracker(Config.neutral().tracker, (640, 480))
    tr.update([Detection((100, 100, 200, 400), "person", 0.9, "anna", 0.7)], 0.0)
    assert tr.tracks[0].who == "anna"
    tr.update([Detection((102, 100, 202, 400), "person", 0.9)], 0.2)   # ohne Prüfung: Name bleibt
    assert len(tr.tracks) == 1 and tr.tracks[0].who == "anna" and tr.tracks[0].who_sim == 0.7


def test_pet_begruesst_erkannte():
    b = PetBrain(PetCfg(seek_m=0.0, roam_m=0.0, faint=False, follow_m=0.0, greet_kurz_s=20.0, greet_voll_s=60.0),
                 Config.neutral().robot, seed=1)
    b.trust = 0.3

    def lauf(t0, t1, who):
        evs, t = [], t0
        while t < t1:
            b.update(Sense(t=t, dt=0.05, seen=[Seen(1, "person", 2.0, 0.0, False, 77.0, t, who=who)]))
            evs += b.drain()
            t += 0.05
        return evs
    gruss = ("sound", "traellern", False)          # in 2 m (Zone mitte) trällert er sonst nicht
    assert gruss not in lauf(0.0, 2.0, None) and b.trust < 1.0
    evs = lauf(2.0, 4.0, "anna")
    assert ("emotion", "like", False) in evs and evs.count(gruss) == 1 and b.trust == 1.0
    assert gruss not in lauf(4.0, 30.0, "anna")                  # sie bleibt: kein zweites Mal
    lauf(30.0, 100.0, None)                                      # 70 s ohne sie erkannt
    assert gruss in lauf(100.0, 102.0, "anna")
    # eigene Begrüßung je Person (Web), "-" = nichts, None = Standard
    b.gruss_fuer = lambda n: {"bob": {"emotion": "cute", "sound": "stimme:03"}, "eva": {"emotion": "-"}}.get(n, {})
    evs = lauf(102.0, 104.0, "bob")
    assert ("emotion", "cute", False) in evs and ("sound", "stimme:03", False) in evs and gruss not in evs
    evs = lauf(104.0, 106.0, "eva")
    assert ("emotion", "like", False) not in evs and evs.count(gruss) == 1


def test_pet_begruesst_liegend():
    b = PetBrain(PetCfg(seek_m=0.0, roam_m=0.0, faint=False, follow_m=0.0, greet_voll_s=60.0),
                 Config.neutral().robot, seed=1)
    b.gruss_fuer = lambda n: {"anna": {"augen": "rot"}}.get(n, {})

    def lauf(t0, t1, who):
        evs, t = [], t0
        while t < t1:
            b.update(Sense(t=t, dt=0.05, balancing=False, seen=[Seen(1, "person", 2.0, 0.0, False, 77.0, t, who=who)]))
            evs += b.drain()
            t += 0.05
        return evs
    gruss = ("sound", "traellern", False)
    evs = lauf(0.0, 2.0, "anna")
    assert b.state == "liegt"
    assert ("emotion", "like", False) in evs and evs.count(gruss) == 1 and b.augen == "rot"
    assert ("sound", "gefunden", False) not in evs                # Begrüßung statt allgemeinem Aufmerken
    assert gruss not in lauf(2.0, 30.0, "anna")


class Gehirn:
    """PetBrain mit Wanduhr (Sense.uhr) und Gedächtnis; lauf() sammelt Ereignisse und Gesten."""

    def __init__(self, gedaechtnis=None, balancing=True, **kw):
        from xgonav.gedaechtnis import Gedaechtnis
        self.cfg = PetCfg(seek_m=0.0, roam_m=0.0, faint=False, follow_m=0.0, **kw)
        self.b = PetBrain(self.cfg, Config.neutral().robot, seed=1)
        self.g = self.b.gedaechtnis = gedaechtnis if gedaechtnis is not None else Gedaechtnis("", self.cfg)
        self.t, self.balancing = 0.0, balancing

    def lauf(self, s, who=None, uhr=None, tid=1, fremd_n=0, dist=2.0):
        """s Sekunden; uhr: Wanduhr am Anfang (dann mitlaufend), None = wie t."""
        evs, gesten = [], set()
        t1 = self.t + s
        u0, t0 = uhr, self.t
        while self.t < t1:
            seen = [] if who is False else [Seen(tid, "person", dist, 0.0, False, 77.0, self.t, who=who,
                                                 fremd_n=fremd_n)]
            self.b.update(Sense(t=self.t, dt=0.05, balancing=self.balancing, seen=seen,
                                uhr=None if u0 is None else u0 + (self.t - t0)))
            evs += self.b.drain()
            if self.b._gesture is not None:
                gesten.add(self.b._gesture[0])
            self.t += 0.05
        return evs, gesten


def test_begruessung_nach_abwesenheit():
    """Nutzer 2026-11-28: ab 5 min kurz (nur Ton), ab 1 h voll, ab 8 h Freudenausbruch; die Uhr ist die Wanduhr."""
    k = Gehirn()
    like, ton, lacht = ("emotion", "like", False), ("sound", "traellern", False), ("emotion", "laugh", False)
    u = 1_000_000.0
    evs, _ = k.lauf(2.0, "anna", u)                              # noch nie gesehen: voll
    assert like in evs and evs.count(ton) == 1 and lacht not in evs
    assert k.g.personen["anna"]["begegnungen"] == 1
    evs, _ = k.lauf(2.0, "anna", u + 2.0 + 120.0)                # 2 min weg: nichts
    assert ton not in evs and like not in evs
    evs, _ = k.lauf(2.0, "anna", u + 4.0 + 120.0 + 400.0)        # fast 7 min weg: kurz, nur der Ton
    assert ton in evs and like not in evs
    evs, _ = k.lauf(2.0, "anna", u + 6.0 + 520.0 + 7200.0)       # 2 h: voll
    assert like in evs and ton in evs and lacht not in evs
    evs, gesten = k.lauf(5.0, "anna", u + 8.0 + 7720.0 + 9 * 3600.0)   # 9 h: Ausbruch
    assert like in evs and lacht in evs and evs.count(ton) == 2 and "wackeln" in gesten
    assert k.g.personen["anna"]["begegnungen"] == 4
    assert k.b.info()["vertrauen_von"] == "anna"


def test_ausbruch_liegend_ohne_wackeln():
    k = Gehirn(balancing=False)
    k.g.gesehen("anna", 0.0)
    evs, gesten = k.lauf(5.0, "anna", 10 * 3600.0)
    assert ("emotion", "laugh", False) in evs and not gesten and k.b.state == "liegt"


def test_gedaechtnis_uebersteht_neustart(tmp_path):
    from xgonav.gedaechtnis import Gedaechtnis
    pfad = str(tmp_path / "gedaechtnis.json")
    k = Gehirn(Gedaechtnis(pfad, PetCfg()))
    u = 5_000_000.0
    evs, _ = k.lauf(2.0, "anna", u)
    assert ("emotion", "like", False) in evs
    k.b.trust = 0.7                                              # als hätte sie ihn erschreckt
    k.lauf(0.1, "anna", u + 2.0)
    assert k.g.speichern(0.0) and os.path.isfile(pfad)
    # Neustart: neues Gedächtnis aus der Datei, neues Gehirn; 1 min später keine Begrüßung, Vertrauen wie zuvor
    k2 = Gehirn(Gedaechtnis(pfad, PetCfg()))
    evs, _ = k2.lauf(2.0, "anna", u + 62.0)
    assert ("emotion", "like", False) not in evs and ("sound", "traellern", False) not in evs
    assert k2.b.trust == pytest.approx(0.7) and k2.g.personen["anna"]["begegnungen"] == 1


def test_vertrauen_je_person_nachtragend():
    k = Gehirn()
    k.lauf(1.0, None)
    assert k.b.trust == pytest.approx(0.3) and k.b.info()["vertrauen_von"] is None
    k.lauf(1.0, "anna")
    assert k.b.trust == 1.0                                      # angelernt: trust_bekannt
    k.b._frighten(Sense(t=k.t, dt=0.05), 0.1)                    # Schreck, während sie das Gegenüber ist
    k.lauf(0.1, "anna")
    assert k.g.vertrauen("anna") == pytest.approx(0.95)
    k.lauf(1.0, None, tid=2)                                     # jemand ohne Namen: Vertrauen der Sitzung
    assert k.b.trust == pytest.approx(0.3)
    k.lauf(1.0, "anna")
    assert k.b.trust == pytest.approx(0.95)                      # sie bleibt bei 0,95


def test_fremde_vorsichtig():
    k = Gehirn()
    doubt = ("emotion", "doubt", False)
    # in 4 m (Zone fern: dort kein doubt der Zone mitte)
    evs, _ = k.lauf(1.0, None, fremd_n=1, dist=4.0)              # erst eine Prüfung ohne Treffer: unbekannt
    assert k.b.trust == pytest.approx(0.3) and doubt not in evs
    evs, _ = k.lauf(1.0, None, fremd_n=2, dist=4.0)
    assert k.b.trust == pytest.approx(0.1) and evs.count(doubt) == 1 and k.b.info()["vertrauen_von"] == "fremd"
    evs, _ = k.lauf(3.0, None, fremd_n=3, dist=4.0)              # derselbe Track: kein zweites doubt
    assert doubt not in evs
    evs, _ = k.lauf(1.0, None, tid=7, fremd_n=2, dist=4.0)       # ein anderer Fremder
    assert evs.count(doubt) == 1 and k.b.trust == pytest.approx(0.1)
    nah = Seen(7, "person", 0.6, 0.0, False, 77.0, k.t)
    assert k.b._zone_of(nah) == "zunah"                         # mehr Abstand als bei 0,3
    k.b.trust = 0.3
    assert k.b._zone_of(nah) == "nah"


def test_tracker_zaehlt_fremde():
    tr = Tracker(Config.neutral().tracker, (640, 480))
    tr.update([Detection((100, 100, 200, 400), "person", 0.9, None, 0.2)], 0.0)
    assert tr.tracks[0].fremd_n == 1
    tr.update([Detection((102, 100, 202, 400), "person", 0.9)], 0.2)    # keine Prüfung: zählt nicht
    tr.update([Detection((104, 100, 204, 400), "person", 0.9, None, 0.3)], 0.4)
    assert tr.tracks[0].fremd_n == 2 and tr.tracks[0].who is None
    tr.update([Detection((106, 100, 206, 400), "person", 0.9, "anna", 0.6)], 0.6)
    tr.update([Detection((108, 100, 208, 400), "person", 0.9, None, 0.3)], 0.8)   # Name bleibt, zählt nicht
    assert tr.tracks[0].who == "anna" and tr.tracks[0].fremd_n == 2


def test_schnittstelle_hochladen(tmp_path):
    from xgonav.web import serve
    g = fake(tmp_path)
    srv = serve(types.SimpleNamespace(cfg=None, gesichter=g), "127.0.0.1", 0, 5)
    base = "http://127.0.0.1:%d" % srv.server_address[1]

    def req(pfad, body=b"", method="POST"):
        try:
            r = urllib.request.Request(base + pfad, data=body if method == "POST" else None, method=method)
            with urllib.request.urlopen(r, timeout=5) as a:
                return a.status, json.loads(a.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())
    try:
        ok, jpg = cv2.imencode(".jpg", np.full((200, 200, 3), 100, np.uint8))
        code, j = req("/gesicht/hochladen?name=anna", jpg.tobytes())
        assert code == 200 and j["nr"] == "00" and base64.b64decode(j["jpg"])[:2] == b"\xff\xd8"
        assert req("/gesicht/hochladen?name=a/b", jpg.tobytes())[0] == 400
        assert req("/gesicht/hochladen?name=anna", b"kein bild")[0] == 400
        assert req("/gesicht/hochladen?name=anna")[0] == 413
        assert req("/gesicht/gruss?name=anna&emotion=cute&sound=stimme:01") == (200, {"ok": True, "gruss": {"emotion": "cute", "sound": "stimme:01"}})
        assert req("/gesicht/gruss?name=anna&emotion=cute&sound=stimme:01&augen=Rot")[0] == 400
        assert req("/gesicht/gruss?name=bob&emotion=cute")[0] == 400
        assert req("/gesicht", method="GET") == (200, {"on": True, "namen": {"anna": 1},
                                                    "gruesse": {"anna": {"emotion": "cute", "sound": "stimme:01"}}, "ms": 0.0})
        assert req("/gesicht/loeschen?name=anna&nr=00")[0] == 200
        assert req("/gesicht/loeschen?name=anna&nr=00")[0] == 404
    finally:
        srv.shutdown()


def test_augenfarbe_solange_sie_da_ist():
    b = PetBrain(PetCfg(seek_m=0.0, roam_m=0.0, faint=False, follow_m=0.0, eye_hold_s=20.0),
                 Config.neutral().robot, seed=1)
    b.gruss_fuer = lambda n: {"anna": {"augen": "rot"}}.get(n, {})
    farben = []

    def lauf(t0, t1, who):
        t = t0
        while t < t1:
            b.update(Sense(t=t, dt=0.05, seen=[Seen(1, "person", 2.0, 0.0, False, 77.0, t, who=who)]))
            farben.append((t, b.augen, b.drain()))
            t += 0.05
    lauf(0.0, 2.0, None)
    assert b.augen is None
    lauf(2.0, 4.0, "bob")                                        # ohne eigene Farbe
    assert b.augen is None
    lauf(4.0, 6.0, "anna")
    assert b.augen == "rot"
    # die Begrüßungs-Emotion kommt erst, wenn die Farbe schon steht
    t_emo = next(t for t, f, evs in farben if ("emotion", "like", False) in evs and t >= 4.0)
    assert all(f == "rot" for t, f, _ in farben if 4.0 <= t <= t_emo)
    lauf(6.0, 20.0, None)                                        # kurz weg: bleibt
    assert b.augen == "rot"
    lauf(20.0, 27.0, None)                                       # > 20 s ohne sie
    assert b.augen is None


def test_emotion_in_farbe(tmp_path):
    from xgonav.emotion import Emotions
    for d in ("emo/like", "emo_rot/like"):
        (tmp_path / d).mkdir(parents=True)
    e = Emotions.__new__(Emotions)
    e.cfg = types.SimpleNamespace(dir=str(tmp_path / "emo"))
    e.farbe, e._proc, e.fifo, e.augen = None, None, "/tmp/kein", None
    gespielt = []
    import xgonav.emotion as em
    alt = em.subprocess.Popen
    em.subprocess.Popen = lambda cmd, **kw: gespielt.append(cmd[-1]) or types.SimpleNamespace(poll=lambda: 0)
    try:
        e._play_display("like")
        e.farbe = "rot"
        e._play_display("like")
        e.farbe = "lila"                                          # nicht gerendert: Standard
        e._play_display("like")
    finally:
        em.subprocess.Popen = alt
    assert gespielt == [str(tmp_path / "emo" / "like"), str(tmp_path / "emo_rot" / "like"), str(tmp_path / "emo" / "like")]


MODELLE = ("models/face_detection_yunet_2023mar.onnx", "models/mobilefacenet_ncnn/model.ncnn.param")


@pytest.mark.skipif(not all(os.path.isfile(ROOT / m) for m in MODELLE), reason="Modelle fehlen (tools/export_face.sh)")
def test_echte_modelle_laden(tmp_path):
    pytest.importorskip("ncnn")
    cfg = GesichtCfg(model=str(ROOT / "models/mobilefacenet_ncnn"),
                     det_model=str(ROOT / "models/face_detection_yunet_2023mar.onnx"), dir=str(tmp_path))
    g = Gesichter(cfg)
    assert g.start()
    v = g._merkmal(np.full((SEITE, SEITE, 3), 128, np.uint8))
    assert v.shape == (512,) and abs(float(np.linalg.norm(v)) - 1.0) < 1e-4
    with pytest.raises(GesichtFehler):
        g.anlernen(np.full((300, 300, 3), 128, np.uint8), "leer")     # graues Bild: kein Gesicht
