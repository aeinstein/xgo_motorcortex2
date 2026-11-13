"""Modus pet (xgonav/pet.py): reine Logik mit synthetischen Verläufen."""
import dataclasses
import importlib.util
import sys
import wave
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from xgonav.behaviors import World, make_behavior  # noqa: E402
from xgonav.config import BehaviorCfg, Config, PetCfg, RobotCfg  # noqa: E402
from xgonav.emotion import LCD_FRAME_BYTES, load_gaze  # noqa: E402
from xgonav.geometry import CamGeometry  # noqa: E402
from xgonav.pet import GAZE_BINS, GAZE_CENTER, LID_OPEN, LID_SHUT, Pet, PetBrain, Seen, Sense  # noqa: E402
from xgonav.robot import SimRobot  # noqa: E402
from xgonav.tracker import Track  # noqa: E402


def brain(seed=1, **kw):
    return PetBrain(PetCfg(**kw), Config.neutral().robot, seed=seed)


def seen(cls="person", dist=1.0, t=0.0, bearing=0.0, top_free=False, vs=None, id_=1, conf=0.9, hits=99, tall=False,
         width=None):
    """width: Boxbreite; Standard passend zum Abstand (0,5 m breit), vs=0: Box wächst nicht."""
    if width is None:
        width = 100.0 if vs == 0 or not dist else 155.0 / dist
    return Seen(id_, cls, dist, bearing, top_free, width, t, conf, hits, tall)


def run(b, t0, t1, who=None, dt=0.05, **sense):
    """who(t) -> Liste; Rückgabe: (Zeit danach, alle Acts, alle Ereignisse)."""
    acts, evs, t = [], [], t0
    while t < t1:
        acts.append(b.update(Sense(t=t, dt=dt, seen=who(t) if who else [], **sense)))
        evs += b.drain()
        t += dt
    return t, acts, evs


def walk(d0, speed, cls="person", start=0.0, period=0.25, vs=None):
    """Ziel kommt mit speed m/s auf ihn zu, neue Detektion alle period s."""
    def f(t):
        ts = start + int((t - start) / period) * period
        return [seen(cls, max(0.05, d0 - speed * (ts - start)), ts, vs=vs)]
    return f


# ---- Fahren ----------------------------------------------------------------
@pytest.mark.parametrize("cls,speed", [("person", 3.0), ("dog", 1.0), ("cat", 0.3)])
def test_budget_null_faehrt_nie(cls, speed):
    b = brain()
    _, acts, _ = run(b, 0.0, 6.0, walk(2.4, speed, cls), floor_ok=True)
    assert all(a.v == 0.0 for a in acts)


def test_grenzen_aus_der_konfiguration_nicht_aufhebbar():
    b = brain(retreat_m=5.0, creep_m=9.0, roll_deg=40.0, duck_mm=40.0)
    assert b.retreat_m == 0.30 and b.creep_m == 1.0 and b.roll_mag == 12.0
    assert b.duck_mm == 70.0                 # Minimum 60 + 10 mm


def test_standard_faehrt_nicht_und_setzt_nicht_ab():
    d = PetCfg()
    assert d.retreat_m == 0 and d.creep_m == 0 and not d.faint


def test_ruck_zurueck_bleibt_im_budget():
    b = brain(retreat_m=0.12)
    who = walk(2.4, 2.0)
    t, moved, n, ds = 0.0, 0.0, 0, 0.0
    while t < 3.0:
        a = b.update(Sense(t=t, seen=who(t), ds=ds))
        ds = a.v * 1.5 * 0.05                # fährt, was befohlen ist
        moved += abs(ds)
        n += a.v < 0
        t += 0.05
    assert n > 0 and moved <= 0.12 and b.n_retreat == 1


def test_raeder_begrenzen_den_impuls():
    """Fährt er schneller als gedacht (vx_limit falsch), stoppen ihn die Räder."""
    b = brain(retreat_m=0.12)
    who = walk(2.4, 2.0)
    t, moved, ds = 0.0, 0.0, 0.0
    while t < 3.0:
        a = b.update(Sense(t=t, seen=who(t), ds=ds))
        ds = a.v * 6.0 * 0.05                # viermal so schnell
        moved += abs(ds)
        t += 0.05
    assert 0.0 < moved <= 0.12 + 0.03        # höchstens ein Schritt zu viel


def test_ruck_ohne_raeder_faehrt_nicht():
    b = brain(retreat_m=0.12)
    _, acts, _ = run(b, 0.0, 3.0, walk(2.4, 2.0), ds_measured=False)
    assert all(a.v == 0.0 for a in acts)


def creep_run(**kw):
    b = brain(creep_m=0.2)
    t, moved, ds = 0.0, 0.0, 0.0
    while t < 10.0:
        a = b.update(Sense(t=t, seen=[seen("cat", 2.0, t, vs=0.0)], ds=ds, **kw))
        ds = a.v * 1.5 * 0.05
        moved += abs(ds)
        t += 0.05
    return moved


def test_katze_schleicht_nur_mit_bodenmodell_und_budget():
    assert creep_run() == 0.0                # floor_ok fehlt
    assert creep_run(floor_ok=True, ds_measured=False) == 0.0
    assert 0.15 < creep_run(floor_ok=True) <= 0.2


def test_katze_nah_genug_nicht_weiter():
    b = brain(creep_m=0.5)
    assert b.update(Sense(t=0.0, floor_ok=True, seen=[seen("cat", 0.7, 0.0)])).v == 0.0


# ---- Ziele und Zonen -------------------------------------------------------
def test_vorrang_hund_katze_person():
    all3 = [seen("person", 1.0, id_=1, vs=0), seen("cat", 1.0, id_=2, vs=0), seen("dog", 1.9, id_=3, vs=0)]
    b = brain()
    b.update(Sense(t=0.0, seen=all3))
    assert b.target.cls == "dog"
    b = brain()
    b.update(Sense(t=0.0, seen=all3[:2]))
    assert b.target.cls == "cat"


@pytest.mark.parametrize("kw", [dict(hits=2), dict(conf=0.4)])
def test_unsicheres_tier_zaehlt_nicht(kw):
    b = brain()
    b.update(Sense(t=0.0, seen=[seen("dog", 1.0, id_=3, vs=0, **kw), seen("person", 1.0, id_=1, vs=0)]))
    assert b.target.cls == "person"


@pytest.mark.parametrize("d,top,zone,emo", [
    (4.0, False, "fern", None), (2.0, False, "mitte", "doubt"), (1.0, False, "nah", "cute"),
    (1.2, True, "knien", "like"), (0.2, False, "zunah", "shy")])
def test_zonen(d, top, zone, emo):
    b = brain(gestures=False)
    _, _, evs = run(b, 0.0, 2.0, lambda t: [seen(dist=d, t=t, top_free=top, vs=0.0)])
    assert b.zone == zone
    if emo:
        assert ("emotion", emo, False) in evs


def test_kopf_schief_und_zurueck():
    b = brain()
    rolls, t = [], 0.0
    while t < 3.0:
        b.update(Sense(t=t, seen=[seen(dist=2.0, t=t, vs=0.0)]))
        rolls.append(b.roll_deg)
        t += 0.05
    assert max(abs(r) for r in rolls) == 8.0 and rolls[-1] == 0.0 and b.rolled


def test_gesten_aus_keine_bewegung():
    b = brain(gestures=False)
    _, acts, _ = run(b, 0.0, 5.0, walk(2.4, 3.0))
    assert all(a.v == 0 and a.w == 0 for a in acts)
    assert b.roll_deg == 0.0 and not b.rolled
    assert all(a.height_mm == b.base_h for a in acts)


# ---- Schreck ---------------------------------------------------------------
def test_schreck_bei_schneller_annaeherung():
    b = brain()
    _, acts, evs = run(b, 0.0, 2.0, walk(2.4, 2.0))
    assert ("emotion", "surprised", True) in evs and ("sound", "quiek", True) in evs
    assert any(a.height_mm == 75.0 for a in acts)


def test_langsam_kein_schreck():
    b = brain()
    _, _, evs = run(b, 0.0, 4.0, walk(2.4, 0.3))
    assert ("sound", "quiek", True) not in evs


def test_box_wird_nicht_groesser_kein_schreck():
    """Nur der Fußpunkt springt: Das allein zählt nicht."""
    b = brain()
    _, _, evs = run(b, 0.0, 2.0, walk(2.4, 2.0, vs=0.0))
    assert ("sound", "quiek", True) not in evs


def test_katze_erschreckt_nicht():
    b = brain()
    _, _, evs = run(b, 0.0, 2.0, walk(1.5, 2.0, "cat"))
    assert ("sound", "quiek", True) not in evs and ("sound", "gurren", False) in evs


def test_weiter_rennen_wird_zum_heftigen_schreck():
    """Erst knapp über der Schwelle, dann rennt er weiter: Ohnmacht, obwohl der Auslöser schwach war."""
    b = brain(faint=True)
    def who(t):
        ts = int(t / 0.2) * 0.2
        d = 1.55 - (1.0 * ts if ts < 0.6 else 0.6 + 4.0 * (ts - 0.6))
        return [seen(dist=max(d, 0.1), t=ts)]
    t = 0.0
    while b.state not in ("ab",) and t < 2.0:
        b.update(Sense(t=t, seen=who(t)))
        t += 0.05
    assert b.state == "ab" and b.n_startle == 1


def test_zu_weit_weg_kein_schreck():
    b = brain()
    _, _, evs = run(b, 0.0, 0.5, walk(6.0, 3.0))
    assert ("sound", "quiek", True) not in evs


def test_rauschen_im_fusspunkt_kein_schreck():
    """Langsames Gehen mit Fußpunkt-Rauschen; die Box wächst nur langsam (3 px Rauschen wie im Simulator)."""
    import random
    rng = random.Random(7)
    for _ in range(30):
        b = brain(seed=rng.randint(0, 999))
        noise = {}
        def who(t):
            ts = int(t / 0.25) * 0.25
            d = 1.6 - 0.3 * ts
            n = noise.setdefault(ts, rng.gauss(0.0, 0.15))
            w = noise.setdefault(("w", ts), 155.0 / d + rng.gauss(0.0, 3.0))
            return [seen(dist=d + n, t=ts, width=w)]
        _, _, evs = run(b, 0.0, 3.5, who)
        assert ("sound", "quiek", True) not in evs


def test_fusspunkt_schnell_box_langsam_kein_schreck():
    """Fußpunkt-Gerade 1,3 m/s (Nicken), Boxbreite wächst wie bei 0,9 m/s: Es zählt der kleinere Wert
    (Simulator 2026-11-13: Person mit 0,3 m/s, Fußpunkt sprang 2,24 -> 1,53 m in 0,46 s)."""
    b = brain()
    def who(t):
        ts = int(t / 0.2) * 0.2
        return [seen(dist=2.2 - 1.3 * ts, t=ts, width=155.0 / (2.2 - 0.9 * ts))]
    run(b, 0.0, 1.4, who)
    assert b.n_startle == 0


def test_seitlich_abgeschnittene_box_kein_schreck():
    """Wer seitlich ins Bild läuft, dessen abgeschnittene Box wächst von selbst: Die Breite zählt dann nicht."""
    b = brain()
    def who(t):
        ts = int(t / 0.25) * 0.25
        s = seen(dist=2.4 - 2.0 * ts, t=ts)
        return [dataclasses.replace(s, side_free=False)]
    run(b, 0.0, 2.0, who)
    assert b.n_startle == 0


def test_track_merkt_gemessene_breite():
    """w_meas = Breite der letzten Detektion, ungeglättet; 0, wenn links/rechts am Rand abgeschnitten."""
    from xgonav.config import TrackerCfg
    from xgonav.detector import Detection
    from xgonav.tracker import Tracker
    tk = Tracker(TrackerCfg(), (640, 480))
    tk.update([Detection((300, 100, 340, 400), "person", 0.9)], 0.0)
    tr = tk.tracks[0]
    assert tr.w_meas == 40
    tk.update([Detection((290, 100, 350, 400), "person", 0.9)], 0.2)
    assert tr.w_meas == 60 and tr.w < 60                 # geglättet zieht w nach, w_meas nicht
    tk.update([Detection((0, 100, 60, 400), "person", 0.9)], 0.4)    # neuer Track am linken Rand
    assert tk.tracks[-1] is not tr and tk.tracks[-1].w_meas == 0.0


def test_vertrauen_verschiebt_schwelle():
    b = brain()
    _, _, evs = run(b, 0.0, 2.0, walk(2.4, 0.9))
    assert ("sound", "quiek", True) in evs
    b = brain(trust_start=1.0)
    _, _, evs = run(b, 0.0, 2.0, walk(2.4, 0.9))
    assert ("sound", "quiek", True) not in evs


def test_gewoehnung_keine_ohnmacht_durch_leichte_schrecks():
    b = brain(faint=True)
    t, faint, n = 0.0, False, 0
    for _ in range(10):
        who, t0 = walk(2.4, 1.2, start=t), t
        while t < t0 + 6.0:
            b.update(Sense(t=t, seen=who(t) if t < t0 + 1.0 else []))
            faint = faint or b.state == "ab"
            n += any(e[1] in ("quiek", "surprised", "doubt") for e in b.drain())
            t += 0.05
    assert n > 0 and not faint and b.n_faint == 0


def test_messpause_eigene_bewegung_loest_keinen_schreck_aus():
    b = brain()
    b.update(Sense(t=0.0, seen=[seen(dist=2.0, t=0.0, vs=0.0)]))
    b._blind_until = 5.0
    _, _, evs = run(b, 0.05, 4.9, walk(2.4, 3.0))
    assert not any(e[1] == "quiek" for e in evs)


# ---- Ohnmacht und Aufstehen ------------------------------------------------
def bis_nap(b, **sense):
    """Heftiger Schreck bis in die Ohnmacht; Rückgabe: Zeit des Hinlegens."""
    t, who = 0.0, walk(2.4, 4.0)
    while b.state != "ab" and t < 3.0:
        b.update(Sense(t=t, seen=who(t), **sense))
        t += 0.05
    assert b.state == "ab"
    while not b.want_set_down and t < 6.0:
        b.update(Sense(t=t, **sense))
        t += 0.05
    b.want_set_down = False
    b.update(Sense(t=t, balancing=False, fw_mode=254, **sense))
    assert b.state == "nap"
    return t


def lauf(b, t0, t1, **kw):
    """Liegend bis t1 oder bis zum Aufstehwunsch; Rückgabe: Zeit, gesehene Lidstellungen je Zustand."""
    base = dict(balancing=False, fw_mode=254, tilt=(0.0, 60.0))
    base.update(kw)
    t, lids = t0, []
    while t < t1 and not b.want_stand_up:
        b.update(Sense(t=t, **base))
        lids.append((b.state, b.gaze[1], b.gaze[0]))
        t += 0.05
    return t, lids


def test_ohnmacht_augen_langsam_auf_dann_aufspringen():
    b = brain(faint=True)
    t0 = bis_nap(b)
    t, lids = lauf(b, t0, t0 + 9.5)
    assert not b.want_stand_up and b.state == "nap"       # 10 s liegen
    assert all(l[1] == LID_SHUT for l in lids[-100:])     # Augen zu
    t, lids = lauf(b, t, t0 + 20.0)
    assert b.want_stand_up and b.state == "hoch"
    assert t - t0 >= 10.0 + 3.0 + 1.5                     # Augen auf, Luft rein, Warnton
    auf = [l[1] for l in lids if l[0] == "lugen"]
    assert auf == sorted(auf, reverse=True)               # gehen nur auf, nie wieder zu
    assert auf[0] == LID_SHUT and auf[-1] == LID_OPEN
    assert len(set(auf)) == 5                             # langsam über alle Stellungen
    blick = {l[2] for l in lids if l[0] == "lugen"}
    assert min(blick) < GAZE_CENTER < max(blick)                    # schaut sich um
    b.want_stand_up = False
    b.update(Sense(t=t, balancing=True))
    assert b.state == "settle"
    b.update(Sense(t=t + 7.0, balancing=True))
    assert b.state == "wach"


def test_aufrappeln_nach_ohnmacht():
    b = brain(faint=True)
    t0 = bis_nap(b)
    t, _ = lauf(b, t0, t0 + 20.0)
    assert b.want_stand_up
    b.drain()
    b.update(Sense(t=t, balancing=True))
    assert ("sound", "aufrappeln", True) in b.drain()


def test_sturz_nur_augen_schrei_kommt_aus_emotion():
    """Firmware 0 (Sturz): vertigo ohne Ohnmachtston; abgesetzt von außen (254) wie bisher mit Ton."""
    for fw, snd in ((0, []), (254, ["ohnmacht"])):
        b = brain()
        run(b, 0.0, 0.5)
        _, _, evs = run(b, 0.5, 1.5, balancing=False, fw_mode=fw)
        assert ("emotion", "vertigo", True) in evs
        assert [e[1] for e in evs if e[0] == "sound"] == snd


def test_jemand_im_bild_augen_wieder_zu_und_wartet():
    """Luft nicht rein: stellt sich tot und schaut erst danach wieder."""
    b = brain(faint=True)
    t0 = bis_nap(b)
    t, _ = lauf(b, t0, t0 + 12.0)
    assert b.state == "lugen"
    t, lids = lauf(b, t, t + 10.0, seen=[seen("person", None, t, vs=0)])
    assert not b.want_stand_up and all(l[1] == LID_SHUT for l in lids[1:])
    # weg: wieder Augen auf, frühestens nach eyes_open_s Warnton
    t2, lids = lauf(b, t, t + 2.9)
    assert not b.want_stand_up and b.state == "lugen" and lids[-1][1] < LID_SHUT
    t3, _ = lauf(b, t2, t + 8.0)
    assert b.want_stand_up


def test_auch_ferne_oder_unsichere_box_ist_nicht_rein():
    b = brain(faint=True)
    t0 = bis_nap(b)
    lauf(b, t0, t0 + 62.0, seen=[seen("dog", None, 0.0, vs=0, conf=0.36, hits=1)])
    assert not b.want_stand_up and b.state == "liegt" and b.why_down == "Luft nicht rein"


def test_warnton_abgebrochen_wenn_jemand_kommt():
    b = brain(faint=True)
    t0 = bis_nap(b)
    t, _ = lauf(b, t0, t0 + 13.5)
    assert b.state == "warnen"
    lauf(b, t, t + 0.5, seen=[seen("person", None, t, vs=0)])
    assert b.state == "lugen" and not b.want_stand_up


@pytest.mark.parametrize("stoerung", [
    dict(tilt=(0.0, 20.0)),                 # hochgehoben
    dict(tilt=None),                        # IMU weg
    dict(ds=0.02),                          # Räder bewegt
    dict(ds_measured=False),                # Räder weg
    dict(fw_mode=0),                        # umgefallen
    dict(fw_mode=250),                      # Akku leer
    dict(fw_mode=253),                      # Zustand gewechselt
    dict(fw_mode=None),                     # Zustand unbekannt
])
def test_steht_nach_stoerung_nie_von_selbst_auf(stoerung):
    b = brain(faint=True)
    t0 = bis_nap(b)
    t, _ = lauf(b, t0, t0 + 3.0)
    lauf(b, t, t0 + 70.0, **stoerung)
    assert not b.want_stand_up and b.state == "liegt"
    lauf(b, t0 + 70.0, t0 + 90.0)           # auch nicht, wenn danach alles normal aussieht
    assert not b.want_stand_up and b.state == "liegt"


@pytest.mark.parametrize("sperre", [dict(battery=15), dict(battery=None)])
def test_akku_verhindert_aufstehen(sperre):
    b = brain(faint=True)
    t0 = bis_nap(b)
    lauf(b, t0, t0 + 62.0, **sperre)
    assert not b.want_stand_up and b.state == "liegt"


def test_fremd_abgesetzt_steht_nie_von_selbst_auf():
    b = brain(faint=True)
    b.update(Sense(t=0.0))
    b.update(Sense(t=0.05, balancing=False, fw_mode=254))
    assert b.state == "liegt"
    lauf(b, 0.1, 100.0)
    assert not b.want_stand_up


def test_liegend_gestartet_steht_nie_von_selbst_auf():
    b = brain(faint=True)
    lauf(b, 0.0, 100.0)
    assert not b.want_stand_up and b.state == "liegt"


def test_von_aussen_aufgestellt_waehrend_nap():
    b = brain(faint=True)
    t0 = bis_nap(b)
    t, _ = lauf(b, t0, t0 + 3.0)
    b.update(Sense(t=t, balancing=True))
    assert b.state == "settle" and not b.want_stand_up


def test_absetzen_kommt_nicht_an():
    b = brain(faint=True)
    b.update(Sense(t=0.0, seen=[seen("dog", 0.4, 0.0)]))
    assert b.state == "ab"
    run(b, 0.05, 5.0)
    assert b.state == "wach"


def test_hund_nah_ohnmacht_nur_wenn_erlaubt():
    b = brain(faint=True)
    b.update(Sense(t=0.0, seen=[seen("dog", 0.4, 0.0)]))
    assert b.state == "ab"
    b = brain()
    b.update(Sense(t=0.0, seen=[seen("dog", 0.4, 0.0)]))
    assert b.state == "schreck"


@pytest.mark.parametrize("kw,sense", [(dict(faint_max=0), {}), ({}, dict(battery=15))])
def test_keine_ohnmacht_bei_limit_oder_akku(kw, sense):
    b = brain(faint=True, **kw)
    b.update(Sense(t=0.0, seen=[seen("dog", 0.4, 0.0)], **sense))
    assert b.state == "schreck"


def test_aufstehen_fehlgeschlagen_kein_zweiter_versuch():
    b = brain(faint=True)
    t0 = bis_nap(b)
    t, _ = lauf(b, t0, t0 + 20.0)
    assert b.want_stand_up
    b.want_stand_up = False
    lauf(b, t, t + 30.0)
    assert not b.want_stand_up and b.state == "liegt" and b.why_down == "Aufstehen fehlgeschlagen"


# ---- Sonstiges -------------------------------------------------------------
def test_liegend_nur_ausdruck():
    b = brain()
    t, acts, evs = 0.0, [], []
    while t < 5.0:
        acts.append(b.update(Sense(t=t, balancing=False, fw_mode=254, seen=[seen(dist=None, t=t, bearing=0.5)])))
        evs += b.drain()
        t += 0.05
    assert all(a.v == 0 and a.w == 0 and a.height_mm is None for a in acts)
    assert ("sound", "gefunden", False) in evs and b.gaze[0] > GAZE_CENTER      # links vom Rider = rechts im Bild


def test_liegend_reagiert_nur_beim_auftauchen():
    b = brain()
    da = lambda t: [seen(dist=None, t=t, vs=0.0)]
    def zaehle(t0, t1, who):
        t, _, evs = run(b, t0, t1, who, balancing=False, fw_mode=254)
        return t, sum(e == ("sound", "gefunden", False) for e in evs)
    t, n = zaehle(0.0, 60.0, da)
    assert n == 1                            # eine Minute durchgehend zu sehen
    # Kurzer Aussetzer des Detektors zählt nicht als weg
    t, n = zaehle(t, t + 1.0, None)
    t, n2 = zaehle(t, t + 20.0, da)
    assert n + n2 == 0
    # Wirklich weg und wieder da: noch einmal
    t, _ = zaehle(t, t + 15.0, None)
    t, n = zaehle(t, t + 20.0, da)
    assert n == 1


def test_nach_langem_liegen_aufgestellt_bleibt_er_stehen():
    b = brain()
    t, _, _ = run(b, 0.0, b.cfg.timeout_s + 60.0, dt=0.5, balancing=False, fw_mode=254)
    t, _, evs = run(b, t, t + b.cfg.settle_s + 5.0)
    assert b.state == "wach" and not b.want_set_down and not b.want_idle
    assert ("emotion", "sleep", False) not in evs


def test_blick_gleitet_stufenweise():
    b = brain(gestures=False)
    t, blick = 0.0, []
    while t < 2.0:
        bearing = -0.8 if t < 1.0 else 0.8
        b.update(Sense(t=t, seen=[seen(dist=2.0, t=t, bearing=bearing, vs=0.0)]))
        blick.append(b.gaze[0])
        t += 0.05
    assert max(abs(x - y) for x, y in zip(blick, blick[1:])) == 1   # nie mehr als eine Stufe je Regelschritt
    assert set(blick) == set(range(GAZE_BINS))                      # alle Stufen, ganz links bis ganz rechts


def test_lidschlag_oeffnet_und_schliesst_stufenweise():
    b = brain()
    t, lider = 0.0, []
    while t < 8.0:
        b.update(Sense(t=t))
        lider.append(b.gaze[1])
        t += 0.05
    assert max(abs(x - y) for x, y in zip(lider, lider[1:])) == 1


def test_guck_guck():
    b = brain(gestures=False)
    who = lambda t: [seen(dist=1.0, t=t, vs=0.0)]
    t, _, _ = run(b, 0.0, 3.0, who)
    t, _, _ = run(b, t, t + 1.0)
    _, _, evs = run(b, t, t + 1.0, who)
    assert ("sound", "kichern", False) in evs


def test_person_weg_traurig():
    b = brain(gestures=False)
    t, _, _ = run(b, 0.0, 3.0, lambda t: [seen(dist=1.0, t=t, vs=0.0)])
    _, _, evs = run(b, t, t + 4.0)
    assert ("emotion", "sad", False) in evs and ("sound", "verloren", False) in evs


def test_anstupsen():
    b = brain(gestures=False)
    t, _, _ = run(b, 0.0, 2.0)
    _, _, evs = run(b, t, t + 1.0, ds=0.01)
    assert ("sound", "hey", False) in evs


def test_anstupsen_ohne_raeder_keine_aussage():
    b = brain(gestures=False)
    t, _, _ = run(b, 0.0, 2.0, ds_measured=False)
    _, _, evs = run(b, t, t + 1.0, ds=0.01, ds_measured=False)
    assert ("sound", "hey", False) not in evs


def test_doesen_und_aufwachen():
    b = brain(bored_s=2.0, bored_repeat_s=100.0, doze_s=5.0, timeout_s=80.0, gestures=False)
    t, _, evs = run(b, 0.0, 3.0)
    assert ("sound", "seufzen", False) in evs and b.det_hz == 4.0
    t, _, _ = run(b, t, 6.0)
    assert b.det_hz == 1.0 and b.gaze[1] != LID_OPEN
    run(b, t, t + 0.2, lambda t: [seen(dist=2.0, t=t, vs=0.0)])
    assert b.det_hz == 4.0


def test_niemand_setzt_ab_und_idle():
    b = brain(doze_s=5.0, timeout_s=8.0, gestures=False)
    run(b, 0.0, 9.0)
    assert b.want_set_down and b.want_idle and b.state == "aus"


def test_blick_folgt_und_blinzelt():
    b = brain(gestures=False)
    run(b, 0.0, 1.0, lambda t: [seen(dist=2.0, t=t, bearing=-0.8, vs=0.0)])
    assert b.gaze[0] == 0                    # rechts vom Rider = links im Bild
    lids, t = set(), 1.0
    while t < 12.0:
        b.update(Sense(t=t))
        lids.add(b.gaze[1])
        t += 0.05
    assert lids == {0, 1, 2, 3, 4}              # Lidschlag über alle Stellungen


def test_config_und_make_behavior():
    p = make_behavior("pet", Config.neutral().behavior, Config.neutral().robot, petcfg=PetCfg(faint=True))
    assert p.name == "pet" and p.pcfg.faint and p.want_stand_up is False
    c = Config.neutral()
    c.apply({"pet": {"retreat_m": 0.12}})
    assert c.pet.retreat_m == 0.12
    with pytest.raises(KeyError):
        c.apply({"pet": {"gibtsnicht": 1}})


def test_step_aus_world():
    cfg = Config.neutral()
    geo = CamGeometry.from_cfg(cfg)
    pet = make_behavior("pet", cfg.behavior, cfg.robot, petcfg=cfg.pet)
    tr = Track("person", 320, 200, 100, 200, 0.9, 1.0)
    w = World(t=1.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[tr], free=None, balancing=True,
              yaw=0.0, cmd_v=0.0, geo=geo, battery=90)
    cmd = pet.step(w)
    assert pet.brain.target.dist == pytest.approx(geo.ground_dist(300.0), abs=1e-6)
    assert cmd.v == 0.0 and cmd.target_id == tr.id
    # liegend stimmt die Bodengeometrie nicht
    tr.h = 300
    s = Pet.sense_tracks(dataclasses.replace(w, balancing=False))
    assert s[0].dist is None and s[0].tall


def test_robot_stand_up_nur_aus_dem_liegen():
    r = SimRobot(Config.neutral().robot)
    assert r.stand_up() is False and not r.writes
    r.set_down()
    assert not r.state.balancing and r.state.mode == 254
    assert r.stand_up() is True and r.writes[-1][1:] == ("action", 253)


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_augen_atlas(tmp_path):
    assert load_gaze(str(tmp_path / "fehlt")) is None
    assert load_tool("eyes").write_gaze(tmp_path / "g", (0, 225, 210)) == GAZE_BINS * 5
    atlas = load_gaze(str(tmp_path / "g"))
    assert len(atlas) == GAZE_BINS * 5 and all(len(v) == LCD_FRAME_BYTES for v in atlas.values())
    assert len(set(atlas.values())) == GAZE_BINS * 5    # jede Stellung sieht anders aus
    (tmp_path / "g" / "g3_l0.raw").write_bytes(b"kurz")
    assert load_gaze(str(tmp_path / "g")) is None


def test_augen_atlas_im_stil_der_emotionen():
    eyes = load_tool("eyes")
    neutral = eyes.render((dict(), eyes.neutral(), eyes.neutral()))
    # Geradeaus offen ist genau das Bild, mit dem jede Emotion beginnt und endet: kein Sprung beim Wechsel
    assert np.array_equal(eyes.gaze_alpha(GAZE_CENTER, 0), neutral)
    # Keine Pupille: Das Auge ist ein volles Rechteck, die Fläche ändert sich mit dem Blick kaum
    for g in (0, GAZE_BINS - 1):
        assert abs(eyes.gaze_alpha(g, 0).sum() / neutral.sum() - 1.0) < 0.02


def test_toene(tmp_path):
    snd = load_tool("sounds")
    for name, fn in snd.PET.items():
        p = tmp_path / f"pet_{name}_1.wav"
        snd.write(str(p), fn(1.0))
        with wave.open(str(p)) as f:
            assert 0.1 < f.getnframes() / f.getframerate() < 2.0, name
