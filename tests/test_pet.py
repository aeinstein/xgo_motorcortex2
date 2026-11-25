"""Modus pet (xgonav/pet.py): reine Logik mit synthetischen Verläufen."""
import dataclasses
import math
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
from xgonav.pet import GAZE_BINS, GAZE_CENTER, LID_OPEN, LID_SHUT, Heard, Pet, PetBrain, Seen, Sense  # noqa: E402
from xgonav.robot import SimRobot  # noqa: E402
from xgonav.tracker import Track  # noqa: E402


def brain(seed=1, **kw):
    kw.setdefault("seek_m", 0.0)             # Standard am RiderPi: ohne Grenze; die Tests geben den Weg selbst vor
    kw.setdefault("roam_m", 0.0)
    kw.setdefault("faint", False)            # Standard am RiderPi: an; die Tests schalten es selbst ein
    kw.setdefault("faint_level", 1.0)        # Standard am RiderPi: 0,8; die Tests rechnen mit 1,0
    kw.setdefault("seek_home", False)        # Standard: nach dem Suchen zurück zum Start; eigene Tests dafür
    kw.setdefault("follow_m", 0.0)           # Standard am RiderPi: folgt jeder Person (follow-Routine); eigene Tests
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


def test_standard_ruckt_nicht_und_faellt_am_rider_um():
    d = PetCfg()
    assert d.retreat_m == 0 and d.creep_m == 0 and d.faint          # RiderPi: Ohnmacht an
    assert Config.neutral().pet.faint                                 # Simulator: wie am Rider (seit 2026-11-23)


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
        ds = a.v * b.rcfg.vx_limit * 0.05
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


# ---- Suchen und streifen ---------------------------------------------------
class Kin:
    """Fährt, was befohlen ist (neutral: 1,5 m/s und 360°/s je Einheit), Kurs und Weg wie World.yaw/ds."""

    def __init__(self, b):
        self.b, self.x, self.y, self.yaw, self.ds, self.moved, self.far = b, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.acts, self.evs = [], []

    def run(self, t0, t1, who=None, dt=0.05, **sense):
        sense.setdefault("floor_ok", True)
        sense.setdefault("free_center", 1.0)
        t = t0
        while t < t1:
            a = self.b.update(Sense(t=t, dt=dt, seen=who(t) if who else [], ds=self.ds, yaw=self.yaw, **sense))
            self.acts.append(a)
            self.evs += [e[1] for e in self.b.drain()]
            self.yaw += math.radians(a.w * 360.0) * dt
            self.ds = a.v * 1.5 * dt
            self.x += self.ds * math.cos(self.yaw)
            self.y += self.ds * math.sin(self.yaw)
            self.moved += abs(self.ds)
            self.far = max(self.far, math.hypot(self.x, self.y))
            t += dt
        return t


def weg_nach(t_weg, dist=1.6, bearing=0.3):
    """Person steht da, bis t_weg; danach ist sie weg."""
    return lambda t: [seen("person", dist, t, bearing=bearing, vs=0)] if t < t_weg else []


def test_suchen_ohne_budget_dreht_nur():
    k = Kin(brain())
    k.run(0.0, 30.0, weg_nach(3.0))
    assert k.moved == 0.0 and all(a.v == 0.0 for a in k.acts)
    assert any(a.w != 0.0 for a in k.acts[200:])           # schaut sich um
    assert k.b.n_seek == 1 and "verloren" in k.evs and k.b.task is None
    assert abs(math.degrees(k.yaw)) > 200                   # einmal rundum (Hinterherschauen zählt mit)


def test_suchen_faehrt_zur_letzten_stelle():
    k = Kin(brain(seek_m=1.0))
    k.run(0.0, 40.0, weg_nach(3.0, dist=1.6, bearing=0.3))
    # 1,6 m entfernt, 0,4 m davor halten: 1,2 m, das Budget 1,0 m begrenzt
    assert 0.6 < k.moved <= 1.0
    assert all(a.v >= 0.0 for a in k.acts)                  # nie rückwärts
    assert "verloren" in k.evs and k.b.n_found == 0          # danach (ab 33 s) streift er, Budget 0


def test_suchen_nah_genug_faehrt_nicht():
    k = Kin(brain(seek_m=1.0))
    k.run(0.0, 30.0, weg_nach(3.0, dist=0.5))
    assert k.moved == 0.0 and k.b.n_seek == 1


def test_suchen_findet_person_wieder():
    k = Kin(brain(seek_m=1.0))
    k.run(0.0, 9.0, weg_nach(3.0))
    assert k.b.task == "suchen" and k.b.phase == "fahren"
    k.run(9.0, 12.0, lambda t: [seen("person", 1.0, t, vs=0)])
    assert k.b.task is None and k.b.n_found == 1 and "gefunden" in k.evs
    assert all(a.v == 0.0 for a in k.acts[-40:])


@pytest.mark.parametrize("sense", [dict(ds_measured=False), dict(floor_ok=False), dict(free_center=None),
                                   dict(battery=30), dict(battery=None)])
def test_suchen_ohne_voraussetzung_faehrt_nicht(sense):
    k = Kin(brain(seek_m=1.0, roam_m=2.0))
    k.run(0.0, 120.0, weg_nach(3.0), **sense)
    assert k.moved == 0.0


def test_suchen_haelt_bei_blockierter_bahn():
    k = Kin(brain(seek_m=1.0))
    k.run(0.0, 30.0, weg_nach(3.0), path_blocked=True)
    assert k.moved == 0.0 and k.b.n_seek == 1 and "verloren" in k.evs


def test_streifen_im_radius_und_budget():
    k = Kin(brain(seed=3, roam_m=1.5, roam_radius_m=0.8))
    k.run(0.0, 175.0)                                        # vor doze_s
    assert 0.5 < k.moved <= 1.5
    assert k.far <= 0.8 + 0.15
    assert all(a.v >= 0.0 for a in k.acts)
    assert k.b.state == "wach"


def test_streifen_doesen_beendet_aufgabe():
    k = Kin(brain(roam_m=1.5))
    k.run(0.0, 200.0)
    assert k.b._dozing and k.b.task is None
    assert all(a.v == 0.0 and a.w == 0.0 for a in k.acts[-200:])


def test_streifen_freiraum_klein_faehrt_nicht():
    k = Kin(brain(roam_m=1.5))
    k.run(0.0, 175.0, free_center=0.3)
    assert k.moved == 0.0


def test_ohne_gesten_kein_umschauen():
    k = Kin(brain(gestures=False, seek_m=1.0, roam_m=1.5))
    k.run(0.0, 175.0, weg_nach(3.0))
    assert all(a.v == 0.0 and a.w == 0.0 for a in k.acts)


def test_such_grenzen_negativ_ohne_grenze():
    b = brain(seek_m=-1.0, roam_m=-1.0, roam_radius_m=-1.0)
    assert b.seek_m == b.roam_m == b.roam_radius == math.inf
    assert brain(seek_m=9.0).seek_m == 9.0                       # keine feste Obergrenze mehr
    d = PetCfg()
    assert d.seek_m < 0 and d.roam_m < 0 and d.roam_radius_m < 0   # RiderPi: ohne Grenze
    n = Config.neutral().pet
    assert n.seek_m == d.seek_m and n.roam_m == d.roam_m          # Simulator: wie am Rider (seit 2026-11-23)


def test_ohne_grenze_sucht_bis_zur_stelle_und_streift_bis_doesen():
    k = Kin(brain(seek_m=-1.0, roam_m=-1.0, roam_radius_m=-1.0))
    k.run(0.0, 30.0, weg_nach(3.0, dist=2.4, bearing=0.3))
    assert 1.8 < k.moved <= 2.05                                   # 2,4 m - 0,4 m davor
    k.run(30.0, 200.0)
    assert k.moved > 2.5 and k.b._dozing and k.b.task is None


def test_verdeckter_fusspunkt_verschiebt_letzte_stelle_nicht():
    b = brain()
    k = Kin(b)
    k.run(0.0, 1.0, lambda t: [seen("person", 2.0, t, vs=0)])
    k.run(1.0, 1.5, lambda t: [seen("person", 10.6, t, vs=0)])    # Fußpunkt hinter einer Kiste
    assert abs(math.hypot(*b._last_pos) - 2.0) < 1e-6


# ---- Hören ------------------------------------------------------------------
def hoeren(b, t0, t1, quelle=None, who=None, db=14.0, takt=1.0, ab=None, bis=None, dt=0.05, fahrbar=False, **sense):
    """Regelschritte mit idealer Drehung und Fahrt. quelle: Weltwinkel der Schallquelle in Grad (None = still).
    Wie die Ortung: alle takt s eine Meldung über das letzte 1-s-Fenster, Peilung nach vorn gefaltet, dazu das
    Alter. Rückgabe: (t, yaw°, Weg m, Acts, Ereignisse)."""
    t, yaw, weg, acts, evs = t0, 0.0, 0.0, [], []
    ab = t0 if ab is None else ab
    bis = t1 if bis is None else bis
    heard, naechste, ds = None, ab + takt, 0.0
    if fahrbar:
        sense = dict(floor_ok=True, free_center=1.5, **sense)
    while t < t1:
        q = quelle(t) if callable(quelle) else quelle                # Funktion: Weltwinkel je Zeit, None = still
        if q is not None and ab <= t <= bis and t >= naechste:
            naechste = t + takt
            rel = math.radians(q) - yaw
            heard = Heard(key=t, bearing=math.asin(math.sin(rel)), db=db, age=0.0)
        if heard is not None:
            heard = dataclasses.replace(heard, age=t - heard.key)
        a = b.update(Sense(t=t, dt=dt, seen=who(t) if who else [], yaw=yaw, ds=ds, heard=heard, **sense))
        acts.append(a)
        evs += b.drain()
        yaw += math.radians(a.w * b.rcfg.vyaw_limit_dps) * dt
        ds = a.v * b.rcfg.vx_limit * dt
        weg += abs(ds)
        t += dt
    return t, math.degrees(yaw), weg, acts, evs


def wdiff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


@pytest.mark.parametrize("quelle", [60.0, -70.0, 150.0, -120.0, 35.0])
def test_horchen_dreht_zur_quelle(quelle):
    """Vorn seitlich: eine Drehung. Hinten: Die Peilung zeigt nach vorn gespiegelt, nach dem Nachlauschen dreht er herum."""
    b = brain(call_events=99)                                    # ohne Rufen: Das dreht ihn danach weiter
    _, yaw, weg, acts, evs = hoeren(b, 0.0, 40.0, quelle)
    assert weg == 0.0 and all(a.v == 0.0 for a in acts)
    assert wdiff(yaw, quelle) < 25.0, yaw
    assert ("sound", "horch", False) in evs
    assert b.n_heard >= 3


def test_horchen_stille_endet_fragend():
    """Ohne hear_seek: Ein Geräusch, zu dem er sich dreht, danach nichts mehr: schaut fragend und gibt auf."""
    b = brain(hear_seek=False)
    _, yaw, _, _, evs = hoeren(b, 0.0, 20.0, 60.0, bis=2.5)
    assert wdiff(yaw, 60.0) < 20.0
    assert b.task is None and ("emotion", "doubt", False) in evs and b.n_seek == 0


def test_horchen_stille_sucht_dort():
    """Ein Geräusch von der Seite, danach still, niemand zu sehen: dreht sich hin und fährt dorthin suchen."""
    b = brain(seek_m=-1.0, call_m=1.0)
    _, _, weg, _, _ = hoeren(b, 0.0, 30.0, 60.0, bis=2.5, fahrbar=True)
    assert b.n_called == 1 and b.n_seek == 1 and 0.3 < weg <= 1.05, weg


def test_horchen_beim_streifen_sucht():
    """Am Rider 2026-11-19: beim Streifen gehört, stehen geblieben, dann still. Er schaut sich um und lauscht
    dabei; ruft es in einer Pause, fährt er dorthin. Bleibt es still, endet das Suchen nach einer Runde."""
    def lauf(q):
        b = brain(seek_m=-1.0, call_m=1.0)
        _, _, weg, _, _ = hoeren(b, 0.0, 70.0, q, fahrbar=True)
        return b, weg
    b, weg = lauf(lambda t: 70.0 if 33.0 <= t <= 34.5 else None)
    assert b.n_seek == 1 and b.n_called == 0 and weg == 0.0 and b.task is None
    b, weg = lauf(lambda t: 70.0 if 33.0 <= t <= 34.5 or 41.0 <= t <= 44.5 else None)
    assert b.n_seek == 2 and b.n_called == 1 and weg > 0.3, weg


def test_horchen_nicht_waehrend_eigener_drehung():
    """Fenster, in denen er sich selbst gedreht hat, liefern keine Richtung."""
    b = brain()
    b._move_t = 100.0                                            # dreht „gerade“
    b.update(Sense(t=0.0, yaw=0.0))
    for k in range(1, 6):
        b.update(Sense(t=float(k), yaw=0.0, heard=Heard(key=float(k), bearing=1.0, db=15.0)))
    assert b.task is None and b.n_heard == 5


def test_horchen_sperre_nicht_bei_klarer_seite():
    """In der Sperrzeit nach einem Horchen dreht er nur, wenn das Geräusch ganz klar von der Seite kommt."""
    for grad, dreht in ((30.0, False), (60.0, True), (-60.0, True)):
        b = brain()
        b.update(Sense(t=0.0, yaw=0.0))
        b._cool["horchen"] = 100.0                               # eben erst gehorcht
        b.update(Sense(t=1.0, yaw=0.0, heard=Heard(key=1.0, bearing=math.radians(grad), db=15.0)))
        assert (b.task == "horchen") == dreht, grad
        if dreht:
            assert b.phase == "drehen" and b._look_dir == math.copysign(1.0, grad)
    b = brain(hear_side_deg=90.0)                                # Sperre gilt immer
    b.update(Sense(t=0.0, yaw=0.0))
    b._cool["horchen"] = 100.0
    b.update(Sense(t=1.0, yaw=0.0, heard=Heard(key=1.0, bearing=math.radians(80.0), db=15.0)))
    assert b.task is None


def test_horchen_aus():
    b = brain(hear=False)
    _, yaw, _, acts, evs = hoeren(b, 0.0, 15.0, 60.0)
    assert yaw == 0.0 and b.n_heard == 0 and not any(e[1] == "horch" for e in evs)


def test_horchen_zu_leise():
    b = brain()
    _, yaw, _, _, _ = hoeren(b, 0.0, 15.0, 60.0, db=5.0)
    assert yaw == 0.0 and b.n_heard == 0


def test_horchen_findet_person():
    """Taucht beim Hindrehen jemand im Bild auf, endet das Horchen mit Freude; danach redet er mit."""
    b = brain()
    wer = lambda t: [seen("person", 2.0, t)] if t > 6.0 else []
    _, _, _, _, evs = hoeren(b, 0.0, 9.0, 60.0, who=wer)
    assert ("sound", "gefunden", False) in evs and b.task is None
    assert b._voice_t > 0


def test_geraeusch_schreck():
    """Nach Stille plötzlich laut von der Seite: quiekt, duckt sich kurz und dreht sich schnell hin."""
    b = brain(call_events=99)
    _, yaw, _, acts, evs = hoeren(b, 0.0, 12.0, -80.0, db=26.0, ab=3.0)
    assert ("sound", "quiek", True) in evs and ("emotion", "surprised", True) in evs
    assert any(a.height_mm == b.duck_mm for a in acts)
    assert max(abs(a.w) for a in acts) > b.rcfg.w_of(b.cfg.look_w)
    assert wdiff(yaw, -80.0) < 25.0
    assert b.n_startle == 0 and b.state == "wach"                # kein Ruck zurück, keine Ohnmacht


def test_rufen_ohne_budget_schaut_sich_um():
    """Stimme von vorn (oder genau hinten), niemand zu sehen: Ohne Fahrbudget schaut er sich um, fährt nie."""
    b = brain()
    _, _, weg, acts, _ = hoeren(b, 0.0, 14.0, 0.0)
    assert b.n_called == 1 and b.n_seek == 1 and weg == 0.0
    assert any(a.w != 0.0 for a in acts)                         # Rundumblick


def test_rufen_faehrt_hoechstens_call_m():
    b = brain(seek_m=-1.0, call_m=1.0)
    _, _, weg, acts, _ = hoeren(b, 0.0, 40.0, 0.0, fahrbar=True)
    assert b.n_called >= 1
    assert 0.3 < weg <= 1.0 * b.n_called + 0.05, weg
    b = brain(seek_m=0.4, call_m=1.0)                            # seek_m bleibt die Obergrenze
    _, _, weg, _, _ = hoeren(b, 0.0, 20.0, 0.0, bis=6.0, fahrbar=True)
    assert weg <= 0.4 + 0.05
    b = brain(seek_m=-1.0, call_m=-1.0)                          # ohne Grenze: je Ruf eine Etappe
    _, _, weg, _, _ = hoeren(b, 0.0, 60.0, 0.0, bis=6.0, fahrbar=True)
    assert 1.5 < weg <= 3.0 + 0.05, weg
    b = brain(seek_m=-1.0, call_m=1.0)                           # ohne Bodenmodell nie
    _, _, weg, _, _ = hoeren(b, 0.0, 20.0, 0.0)
    assert weg == 0.0


def test_stimmen_wecken_beim_doesen():
    b = brain()
    t, _, _ = run(b, 0.0, b.cfg.doze_s + 2.0)
    assert b._dozing
    _, _, _, _, evs = hoeren(b, t, t + 4.0, 30.0)
    assert not b._dozing and b.det_hz == b.cfg.active_hz
    assert ("emotion", "surprised", False) in evs


def test_mitreden_nach_sprechpause():
    """Person im Bild spricht (drei Meldungen) und macht eine Pause: Er antwortet einmal, erst nach der Pause."""
    b = brain()
    wer = lambda t: [seen("person", 1.2, t)]
    t, _, _, _, evs = hoeren(b, 0.0, 6.0, 0.0, who=wer, bis=3.2)
    brabbel = [e for e in evs if e[1] == "brabbeln"]
    assert len(brabbel) == 1 and b.n_talk == 1
    assert b.roll_deg != 0.0 or b.rolled                         # Kopf schief
    _, _, _, _, evs = hoeren(b, t, t + 3.0, None, who=wer)       # weiter Stille: nicht noch einmal
    assert not [e for e in evs if e[1] == "brabbeln"]


def test_mitreden_nicht_ohne_gegenueber():
    b = brain(call_events=99)
    _, _, _, _, evs = hoeren(b, 0.0, 3.0, None)
    b.update(Sense(t=3.0, heard=Heard(key=3.0, bearing=1.2, db=15.0), balancing=False))    # liegend: keine Richtung
    b.update(Sense(t=4.0, heard=Heard(key=4.0, bearing=1.2, db=15.0), balancing=False))
    for k in range(60):
        b.update(Sense(t=4.05 + 0.05 * k, balancing=False))
        evs += b.drain()
    assert not [e for e in evs if e[1] == "brabbeln"]


def test_luft_nicht_rein_solange_jemand_spricht():
    b = brain()
    b._nap_ref = (0.0, 0.0, 254)
    b._ear_last_t = 9.0
    assert not b._air_clear(Sense(t=10.0, balancing=False))
    assert b._air_clear(Sense(t=9.0 + b.cfg.clear_s + 0.1, balancing=False))


def test_neue_toene_werden_erzeugt():
    spec = importlib.util.spec_from_file_location("sounds", ROOT / "tools" / "sounds.py")
    snd = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(snd)
    for name in ("horch", "brabbeln"):
        for k in snd.VARIANTS:
            y = snd.PET[name](k)
            assert 0.2 < len(y) / snd.RATE < 1.5 and np.abs(y).max() > 0.1


def test_rufen_nicht_endlos():
    """Dauerquelle vorn (Fernseher): Er sucht einmal, dann erst nach call_repeat_s wieder."""
    b = brain()
    hoeren(b, 0.0, 40.0, 0.0)
    assert b.n_called == 1


def test_komm_her_faehrt_zur_sprechenden_person():
    """Person in 2,5 m spricht: Er fährt auf sie zu und hält come_stop_m vor ihr. Ohne Budget oder Bodenmodell nie."""
    def lauf(b, t1=40.0, fahrbar=True, d0=2.5):
        pos = {"x": 0.0}
        def wer(t):
            return [seen("person", max(0.05, d0 - pos["x"]), t, vs=0)]
        t, acts, evs, heard, nxt, ds = 0.0, [], [], None, 1.0, 0.0
        extra = dict(floor_ok=True, free_center=1.5) if fahrbar else {}
        while t < t1:
            if t >= nxt:
                nxt, heard = t + 1.0, Heard(key=t, bearing=0.0, db=15.0)
            if heard is not None:
                heard = dataclasses.replace(heard, age=t - heard.key)
            a = b.update(Sense(t=t, seen=wer(t), ds=ds, heard=heard, **extra))
            acts.append(a)
            evs += b.drain()
            ds = a.v * b.rcfg.vx_limit * 0.05
            pos["x"] += ds
            t += 0.05
        return pos["x"], acts, evs
    b = brain(come_m=-1.0)
    weg, acts, evs = lauf(b)
    assert b.n_come >= 1 and 1.3 < weg < 2.5 - b.cfg.come_stop_m + 0.25, weg
    assert b.n_startle == 0 and ("sound", "kichern", False) in evs
    b = brain(come_m=0.5)
    assert lauf(b)[0] <= 0.5 * b.n_come + 0.05
    b = brain(come_m=0.0)
    assert lauf(b)[0] == 0.0 and b.n_come == 0
    b = brain(come_m=-1.0)
    assert lauf(b, fahrbar=False)[0] == 0.0
    b = brain(come_m=-1.0)                                       # schon nah: bleibt stehen
    assert lauf(b, d0=0.6)[0] == 0.0                             # come_stop_m 0,4 + 0,3


def test_horchen_unterbricht_streifen():
    """Beim Streifen dreht er fast durchgehend, die Richtung taugt dann nicht: Auf ein Geräusch hin bleibt er
    stehen, lauscht und dreht sich mit der nächsten Meldung hin."""
    b = brain(call_events=99)
    _, yaw, weg, acts, evs = hoeren(b, 0.0, 34.0)                # 30 s niemand: streift (dreht sich um)
    assert b.task == "streifen"
    b2 = brain(call_events=99)
    _, yaw, weg, acts, evs = hoeren(b2, 0.0, 60.0, 70.0, ab=33.0)
    assert weg == 0.0 and wdiff(yaw, 70.0) < 25.0, yaw
    assert b2.task == "horchen" and ("sound", "horch", False) in evs


def test_horchen_dreht_bei_erster_meldung():
    b = brain(call_events=99)
    _, yaw, _, _, _ = hoeren(b, 0.0, 6.0, 60.0, bis=1.2)         # nur eine Meldung
    assert wdiff(yaw, 60.0) < 20.0


def test_nach_dem_suchen_zum_start():
    """Gerufen, dort niemand: Er sucht, dann fährt er zurück zum Start (seek_home), aber nur mit Startbild; die Fahrt
    selbst macht Pet wie home (Phase extern), das Gehirn wartet auf heim_ende."""
    b = brain(seek_m=-1.0, call_m=1.0, seek_home=True)
    _, _, weg, _, _ = hoeren(b, 0.0, 60.0, 60.0, bis=2.5, fahrbar=True)
    assert b.n_called == 1 and weg > 0.3 and b.task is None          # ohne Startbild: bleibt dort
    b = brain(seek_m=-1.0, call_m=1.0, seek_home=True)
    t, _, _, acts, _ = hoeren(b, 0.0, 40.0, 60.0, bis=2.5, fahrbar=True, anker=True)
    assert b.task == "heim" and b.phase == "extern" and acts[-1].v == 0.0
    b.heim_ende("am Start")
    hoeren(b, t, t + 0.2, None, fahrbar=True, anker=True)
    assert b.task != "heim"                                          # danach allein: streift wie sonst


def test_planer_kein_weg_rufziel_gegenrichtung():
    """Rufen: Findet der Planer vorn keinen Weg, gilt das Ziel hinten (vorn und hinten klingen gleich); findet er
    auch dort keinen, hält er und schaut sich um."""
    b = brain(seek_m=-1.0, call_m=1.0)
    hoeren(b, 0.0, 5.0, 0.0, fahrbar=True)                       # Stimme von vorn -> Rufen
    assert b.task == "suchen" and b.phase == "fahren" and b._goal[0] > 0.3, b._goal
    x0 = b.x
    fahr = dict(floor_ok=True, free_center=1.5, yaw=b.yaw)
    b.update(Sense(t=5.05, nav_fail=True, **fahr))
    assert b.phase == "fahren" and b._goal[0] < x0 - 0.3, b._goal
    b.update(Sense(t=5.10, nav_fail=True, **fahr))
    assert b.phase == "schauen"


def test_planer_wegpunkt_lenkt():
    """Mit Planer lenkt er auf den Wegpunkt, nicht auf das Ziel; solange der Weg gerechnet wird, steht er."""
    b = brain(seek_m=-1.0, call_m=1.0)
    hoeren(b, 0.0, 5.0, 0.0, fahrbar=True)
    fahr = dict(floor_ok=True, free_center=1.5, yaw=b.yaw)
    a = b.update(Sense(t=5.05, nav_pending=True, **fahr))
    assert a.v == 0.0 and a.w == 0.0
    a = b.update(Sense(t=5.10, nav=(b.x, b.y + 1.0), **fahr))    # Weg biegt links ab
    assert a.v == 0.0 and a.w > 0.0                               # erst ausrichten, nach links


def test_pet_planer_umrechnung_und_umweg():
    """Pet: Ziel des Gehirns in Kartenkoordinaten und zurück; ein Umweg zum Rufziel gilt als kein Weg."""
    p = Pet(BehaviorCfg(), Config.neutral().robot, PetCfg(seek_m=-1.0, call_m=1.0))
    b = p.brain
    b.x, b.y, b.yaw = 1.0, 2.0, 0.5
    w = World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[], free=None, balancing=True, yaw=0.5, cmd_v=0.0, pose=(-3.0, 0.5, 0.5 + math.pi / 2))
    vor = (1.0 + math.cos(0.5), 2.0 + math.sin(0.5))
    q = p._to_grid(w, vor)                                         # 1 m voraus im Gehirn = 1 m voraus in der Karte
    assert abs(q[0] - (-3.0 + math.cos(0.5 + math.pi / 2))) < 1e-9 and abs(q[1] - (0.5 + math.sin(0.5 + math.pi / 2))) < 1e-9
    r = p._to_brain(w, q)
    assert abs(r[0] - vor[0]) < 1e-9 and abs(r[1] - vor[1]) < 1e-9
    b._goal_flip = (1.0, 2.0)
    p._nav_goal = p._nav_req = (0.0, 2.0)
    p.set_path([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)])   # 6 m statt 2 m
    assert p._nav_fail and p.path is None
    p._nav_fail = False
    p._nav_req = p._nav_goal
    p.set_path([(0.0, 0.0), (0.2, 1.0), (0.0, 2.0)])
    assert not p._nav_fail and p.path is not None


# ---- Nachplappern (stimme.Stimmen) -----------------------------------------
def test_neu_bestaetigte_probe_einmal_nachplappern():
    b = brain()
    wer = lambda t: [seen("person", 1.2, t)]
    run(b, 0.0, 1.0, who=wer)
    assert b.hoert_zu                                            # Person zu sehen: App schneidet mit
    _, _, evs = run(b, 1.0, 1.05, who=wer, stimmen=(("01",), ["01"]))
    _, _, evs2 = run(b, 1.05, 3.0, who=wer, stimmen=(("01",), []))
    alle = [e for e in evs + evs2 if e[1].startswith("stimme:")]
    assert alle == [("sound", "stimme:01", True)] and b.n_parrot == 1


def test_hoert_nicht_zu_ohne_person():
    b = brain()
    run(b, 0.0, 1.0, who=lambda t: [seen("person", 1.2, t)])
    run(b, 1.0, 1.0 + b.cfg.parrot_listen_s + 0.5)
    assert not b.hoert_zu


def test_mitreden_spielt_gelernte_probe_mit_sperre():
    """Mit parrot_p = 1 kommt beim Mitreden die Probe, beim nächsten Mal (innerhalb parrot_repeat_s) Brabbeln."""
    b = brain(parrot_p=1.0, talk_repeat_s=0.0)
    wer = lambda t: [seen("person", 1.2, t)]
    t, _, _, _, evs = hoeren(b, 0.0, 6.0, 0.0, who=wer, bis=3.2, stimmen=(("01", "02"), []))
    assert [e[1] for e in evs if e[1].startswith("stimme:") or e[1] == "brabbeln"] in (["stimme:01"], ["stimme:02"])
    _, _, _, _, evs = hoeren(b, t, t + 6.0, 0.0, who=wer, ab=t, bis=t + 3.2, stimmen=(("01", "02"), []))
    assert [e[1] for e in evs if e[1].startswith("stimme:") or e[1] == "brabbeln"] == ["brabbeln"]


def test_ohne_proben_immer_brabbeln():
    b = brain(parrot_p=1.0)
    _, _, _, _, evs = hoeren(b, 0.0, 6.0, 0.0, who=lambda t: [seen("person", 1.2, t)], bis=3.2)
    assert [e[1] for e in evs if e[1] == "brabbeln" or e[1].startswith("stimme:")] == ["brabbeln"]


def test_liegend_mitschneiden_und_nachplappern():
    b = brain()
    wer = lambda t: [seen("person", None, t)]
    run(b, 0.0, 1.0, who=wer, balancing=False)
    assert b.state == "liegt" and b.hoert_zu
    _, _, evs = run(b, 1.0, 1.05, who=wer, balancing=False, stimmen=(("01",), ["01"]))
    _, _, evs2 = run(b, 1.05, 3.0, who=wer, balancing=False, stimmen=(("01",), []))
    evs += evs2
    assert [e for e in evs if e[1].startswith("stimme:")] == [("sound", "stimme:01", True)]
    run(b, 3.0, 3.0 + b.cfg.parrot_listen_s + 0.5, balancing=False)
    assert not b.hoert_zu


# ---- Folgen (Nutzer 2026-11-22: sobald er sie sieht, genau die Routine des Modus follow) ------------
def _steht(dist=2.0, bis=1e9, wide=False):
    def f(t):
        if t >= bis:
            return []
        s = seen("person", dist, t, vs=0)
        s.wide = wide
        return [s]
    return f


def test_folgen_sobald_gesehen_ohne_vertrauen():
    k = Kin(brain(trust_start=0.1, follow_m=-1.0))
    k.run(0.0, 1.0, _steht(2.0))
    assert k.b.task == "folgen" and k.b.phase == "folgen" and k.b.n_follow == 1
    assert k.b.det_hz is None                                  # Detektor wie im Modus follow
    assert all(a.v == 0.0 for a in k.acts)                     # gefahren wird von Pet (behaviors.Follow)


def test_folgen_ende_durch_follow_dann_wieder_pet():
    k = Kin(brain(follow_m=-1.0, follow_repeat_s=3.0))
    k.run(0.0, 1.0, _steht(2.0, bis=0.5))
    assert k.b.task == "folgen"
    k.run(1.0, 20.0)                                           # verloren: das Gehirn wartet auf follow
    assert k.b.task == "folgen"
    k.b.folgen_ende("nicht wiedergefunden")
    k.run(20.0, 20.1)
    assert k.b.task is None and k.b.det_hz == k.b.cfg.active_hz and "verloren" in k.evs
    k.run(20.1, 22.0, _steht(2.0))                             # Sperre follow_repeat_s
    assert k.b.n_follow == 1
    k.run(22.0, 24.0, _steht(2.0))
    assert k.b.n_follow == 2


def test_folgen_ohne_budget_oder_boden_faehrt_nicht():
    for kw, sense in ((dict(follow_m=0.0), {}), (dict(follow_m=-1.0), dict(floor_ok=False)),
                      (dict(follow=False, follow_m=-1.0), {})):
        k = Kin(brain(**kw))
        k.run(0.0, 3.0, _steht(2.0), **sense)
        assert k.b.n_follow == 0 and k.moved == 0.0


def test_breite_person_kein_folgen_vertrauen_brabbeln():
    """Box breiter als hoch (liegt/sitzt am Boden): nicht folgen, Vertrauen steigt, alle wide_babble_s brabbeln."""
    b = brain(follow_m=-1.0, trust_start=0.3)
    k = Kin(b)
    k.run(0.0, b.cfg.wide_babble_s * 2 + 1.0, _steht(1.0, wide=True))
    assert b.n_follow == 0 and b.task is None and k.moved == 0.0
    assert b.trust > 0.3 and k.evs.count("brabbeln") >= 2
    k.run(20.0, 21.0, _steht(1.0))                             # steht wieder: folgt
    assert b.task == "folgen"


def test_pet_folgt_mit_follow_routine_auch_rueckwaerts():
    """Pet fährt die Aufgabe folgen mit behaviors.Follow: Abstand behavior.follow_dist_m, rückwärts bei zu nah;
    breite Personen sieht follow nicht; follow wartet -> Aufgabe endet."""
    from xgonav.behaviors import Follow
    cfg = Config.neutral()
    p = Pet(cfg.behavior, cfg.robot, dataclasses.replace(cfg.pet, follow_m=-1.0), seed=1, hcfg=cfg.home)
    geo = CamGeometry(640, 480, 91.8)
    y2 = 470.0                                                 # Fußpunkt weit unten: sehr nah
    tr = Track("person", 320, y2 - 150, 100, 300, 0.9, 0.0)
    assert tr.box[3] == pytest.approx(y2) and geo.ground_dist(y2, 0.0) < cfg.behavior.follow_dist_m
    p.brain.task = "folgen"
    vs = []
    for i in range(20):
        tr.t_seen = 0.05 * i
        w = World(t=0.05 * i, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[tr], free=None, balancing=True,
                  yaw=0.0, cmd_v=0.0, geo=geo)
        c = p._folgen(w)
        vs.append(c.v)
    assert isinstance(p.follow_drive, Follow) and c.status.startswith("verfolge")
    assert min(vs) == pytest.approx(-cfg.robot.v_of(cfg.behavior.backup_v))   # zu nah: setzt zurück wie follow
    breit = Track("person", 320, 400, 300, 120, 0.9, 1.0)
    w = World(t=1.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[breit], free=None, balancing=True,
              yaw=0.0, cmd_v=0.0, geo=geo)
    assert not p._folgen(w).status.startswith("verfolge")    # liegt/sitzt: follow sieht sie nicht
    p.follow_drive.phase = "warten"
    p._folgen(w)
    assert p.brain._folgen_ende == "nicht wiedergefunden"
    p.brain.task = None
    assert p._folgen(w) is None and p.follow_drive is None


def test_personensuche_lost_home_faehrt_heim():
    b = brain(trust_start=0.3, seek_m=-1.0, seek_home=True, lost_home=True)
    _, _, _, acts, _ = hoeren(b, 0.0, 60.0, None, who=_steht(1.6, bis=3.0), fahrbar=True, anker=True)
    assert any("heim/extern" in a.status for a in acts)


# ---- Rufsuche über die Öffnungen (Nutzer 2026-11-22) -----------------------
def test_rufsuche_ueber_oeffnungen_und_lauter():
    """Ruf von vorn (~0°, wie aus dem Nebenraum), Karte da: Öffnung anfordern, hinfahren, umschauen und horchen;
    nach call_openings Öffnungen bleibt er (seek_home aus)."""
    b = brain(seek_m=-1.0, call_m=-1.0, call_openings=2)
    t, acts = 0.0, []
    while t < 20.0 and b.phase != "oeffnung":
        t, _, _, a, _ = hoeren(b, t, t + 0.5, 0.0, fahrbar=True, karte=True, ab=0.0)
        acts += a
    assert b.task == "suchen" and b.phase == "oeffnung" and b._ruf is not None
    assert all(a.v == 0.0 for a in acts)                       # ohne Öffnung fährt er nicht geradeaus los
    b.oeffnung((1.0, 0.0))
    t, weg, t1 = t + 0.0, 0.0, t + 80.0
    while t < t1 and not (b.phase == "oeffnung" and b._ruf and b._ruf["n"] == 1):
        t, _, d, _, _ = hoeren(b, t, t + 0.5, 0.0, fahrbar=True, karte=True, db=20.0, ab=0.0)
        weg += d
    assert weg > 0.5 and b._ruf["n"] == 1 and b.phase == "oeffnung"
    assert b._ruf["lauter"]                                    # 20 dB statt 14 dB beim Ruf
    b.oeffnung(None)                                           # keine Öffnung mehr: bleibt dort
    hoeren(b, t, t + 0.2, None, fahrbar=True, karte=True)
    assert b.task not in ("suchen", "heim") and b.n_oeffnungen == 1   # bleibt dort (allein: streift wie sonst)


def test_rufsuche_ohne_karte_wie_bisher():
    b = brain(seek_m=-1.0, call_m=1.0)
    hoeren(b, 0.0, 12.0, 0.0, fahrbar=True)
    assert b._ruf is None and b.n_called >= 1


def test_pet_oeffnung_vom_planer_ans_gehirn():
    from xgonav.frontier import ExplorePlan
    cfg = Config.neutral()
    p = Pet(cfg.behavior, cfg.robot, cfg.pet, seed=1, hcfg=cfg.home, ecfg=cfg.explore)
    w = World(t=1.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[], free=None, balancing=True, yaw=0.0,
              cmd_v=0.0, pose=(0.0, 0.0, 0.0), pose_ok=True)
    b = p.brain
    b.task, b.phase, b._ruf = "suchen", "oeffnung", {"pegel": 10, "best": 10, "n": 0, "dbs": [], "lauter": False}
    p._w = w
    assert p.want_plan(1.0) and not p.want_plan(1.1)           # ein Auftrag je Öffnung
    class Pl:
        def request_fn(self, fn, *a, **k):
            self.avoid = a[5]
            return 7
    pl = Pl()
    p.request_plan(pl, None, w, None)
    p.set_path(ExplorePlan(path=[(0.0, 0.0), (1.0, 0.5)], goal=(1.0, 0.5), frontier=(1.2, 0.6)))
    assert b._oeff_result is not None and p.f.path and p._oeff_besucht == [(1.2, 0.6)]
    g = b._oeff_result[0]
    assert math.hypot(g[0] - b.x - 1.0, g[1] - b.y - 0.5) < 1e-6


def test_rufsuche_extern_gefahren_gibt_nicht_vorschnell_auf():
    """Am Rider 2026-11-22: Weg durch die Tür nach 3 s verworfen (Stillstand, Geradeaus-Bahnprüfung). Fährt Pet den
    Weg, zählen nur Ankunft, Budget, Zeit und festgefahren (nav_stuck)."""
    b = brain(seek_m=-1.0, call_m=-1.0)
    b.task, b._ruf = "suchen", {"pegel": 10, "best": 10, "n": 1, "dbs": [], "lauter": False}
    b._task_left = math.inf
    b._goal = (3.0, 0.0)
    b._phase_set("fahren", 0.0)
    b._drive_t_max = 100.0
    t = 0.0
    while t < 10.0:                                            # steht (Bahn blockiert, kein Fortschritt): bleibt dran
        a = b.update(Sense(t=t, floor_ok=True, free_center=1.0, path_blocked=True, nav_extern=True, nav=(1.0, 0.0)))
        t += 0.05
    assert b.phase == "fahren" and a.v == 0.0                  # Fahrbefehl kommt von Pet
    b.update(Sense(t=t, floor_ok=True, free_center=1.0, nav_extern=True, nav_stuck=True, nav=(1.0, 0.0)))
    assert b.phase == "schauen" and b._fails == 1


def test_pet_faehrt_rufsuche_mit_wegfolger():
    cfg = Config.neutral()
    p = Pet(cfg.behavior, cfg.robot, dataclasses.replace(cfg.pet, seek_m=-1.0, call_m=-1.0), seed=1,
            hcfg=cfg.home, ecfg=cfg.explore)
    b = p.brain
    import types
    free = types.SimpleNamespace(valid=True, center=1.0)      # Bodengrenze gültig, vorn frei
    w = World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[], free=free, balancing=True, yaw=0.0,
              cmd_v=0.0, pose=(0.0, 0.0, 0.0), pose_ok=True, map_ok=True, ds_measured=True, battery=100)
    b.update(Sense(t=0.0))
    b.task, b._ruf = "suchen", {"pegel": 10, "best": 10, "n": 1, "dbs": [], "lauter": False}
    b._task_left = math.inf
    b._goal = (2.0, 0.0)
    b._phase_set("fahren", 0.0)
    b._drive_t_max = 100.0
    p._nav_goal = p._to_grid(w, b._goal)
    p.f.set_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    c = p.step(w)
    assert c.v > 0 and "Rufsuche" in c.status


def test_gerufen_und_gefunden_kommt_her():
    """Am Rider 2026-11-22: Rufer gefunden, blieb stehen und suchte wieder. Jetzt: hinfahren (kommen)."""
    b = brain(seek_m=-1.0, call_m=-1.0, come_m=-1.0)
    b.update(Sense(t=0.0, floor_ok=True, free_center=1.0))
    b.task, b._ruf = "suchen", {"pegel": 10, "best": 10, "n": 1, "dbs": [], "lauter": False}
    b._task_left = math.inf
    b._phase_set("schauen", 0.0)
    b.update(Sense(t=0.1, floor_ok=True, free_center=1.0, seen=[seen("person", 2.5, 0.1, vs=0)]))
    assert b.task == "kommen" and b.n_found == 1 and b.n_come == 1
    b2 = brain(seek_m=-1.0)                                    # Personensuche (nicht gerufen): wie bisher
    b2.update(Sense(t=0.0, floor_ok=True, free_center=1.0))
    b2._task_start("suchen", Sense(t=0.0), None)
    b2.update(Sense(t=0.1, floor_ok=True, free_center=1.0, seen=[seen("person", 2.5, 0.1, vs=0)]))
    assert b2.task is None and b2.n_come == 0


def test_augen_schlitze_beim_fixieren():
    """Nutzer 2026-11-22: Hat er jemanden fixiert, werden die Augen zu Schlitzen."""
    b = brain(eyes_fix_s=1.5, eyes_fix_lid=2)
    b._next_blink = 1e9                                         # kein Lidschlag im Test
    t, _, _ = run(b, 0.0, 1.0, lambda t: [seen("person", 2.0, t, vs=0)])
    assert b.gaze[1] == LID_OPEN
    t, _, _ = run(b, t, 3.0, lambda t: [seen("person", 2.0, t, vs=0)])
    assert b.gaze[1] == 2
    run(b, t, t + 0.2, None)                                    # weg: wieder offen
    assert b.gaze[1] == LID_OPEN


def test_kommen_ohne_planerweg_faehrt_direkt():
    """Am Rider 2026-11-22 endete Kommen sofort mit „kein Weg“: Sie ist zu sehen, dann geradeaus."""
    b = brain(come_m=-1.0)
    b.update(Sense(t=0.0, floor_ok=True, free_center=1.0))
    tg = seen("person", 2.0, 0.0, vs=0)
    b.target = tg
    b._come_start(Sense(t=0.0, floor_ok=True, free_center=1.0), tg, "Test")
    a = None
    for k in range(10):
        a = b.update(Sense(t=0.05 * (k + 1), floor_ok=True, free_center=1.0, nav_fail=True,
                           seen=[seen("person", 2.0, 0.05 * (k + 1), vs=0)]))
    assert b.task == "kommen" and a.v > 0


def _pet_mit_karte(task="streifen", goal=(0.8, 0.0), pose=(0.0, 0.0, 0.0)):
    cfg = Config.neutral()
    p = Pet(cfg.behavior, cfg.robot, dataclasses.replace(cfg.pet, seek_m=-1.0, roam_m=-1.0), seed=1,
            hcfg=cfg.home, ecfg=cfg.explore)
    b = p.brain
    import types
    free = types.SimpleNamespace(valid=True, center=1.0)
    w = World(t=0.0, dt=0.05, W=640, H=480, hfov=1.6, ego=None, tracks=[], free=free, balancing=True, yaw=0.0,
              cmd_v=0.0, pose=pose, pose_ok=pose is not None, map_ok=True, ds_measured=True, battery=100)
    b.update(Sense(t=0.0))
    b.task, b._task_left, b._roam_left, b._home = task, math.inf, math.inf, (0.0, 0.0)
    b._goal = goal
    b._phase_set("fahren", 0.0)
    b._drive_t_max = 100.0
    return cfg, p, b, w


def test_pet_streifen_faehrt_geplanten_weg_mit_wegfolger():
    """Seit 2026-11-23 fährt pet jeden geplanten Weg mit dem PathFollower (Tempo pet.drive_v), nicht nur die Rufsuche."""
    cfg, p, b, w = _pet_mit_karte()
    p._nav_goal = p._to_grid(w, b._goal)
    p.f.set_path([(0.0, 0.0), (0.4, 0.0), (0.8, 0.0)])
    c = p.step(w)
    assert "streifen (" in c.status and 0 < c.v <= cfg.robot.v_of(cfg.pet.drive_v) + 1e-9
    # Weg hinter ihm: dreht auf der Stelle mit dem Drehregler von home (höchstens home.turn_w_max)
    p.f.set_path([(0.0, 0.0), (-0.4, 0.0), (-0.8, 0.0)])
    b._goal = (-0.8, 0.0)
    p._nav_goal = p._to_grid(w, b._goal)
    c = p.step(w)
    assert c.v == 0.0 and "drehe" in c.status
    assert 0 < abs(c.w) <= cfg.robot.w_of(cfg.home.turn_w_max) + 1e-9


def test_pet_gehirn_haelt_bei_festgefahren_nicht_bei_bahnpruefung():
    b = brain(roam_m=-1.0)
    b.update(Sense(t=0.0, floor_ok=True, free_center=1.0))
    b.task, b._task_left, b._roam_left, b._home = "streifen", math.inf, math.inf, (0.0, 0.0)
    b._goal = (0.8, 0.0)
    b._phase_set("fahren", 0.0)
    b._drive_t_max = 100.0
    t = 0.0
    while t < 6.0:                                             # blockiert und kein Fortschritt: bleibt dran
        a = b.update(Sense(t=t, floor_ok=True, free_center=0.1, path_blocked=True, nav_extern=True, nav=(0.5, 0.0)))
        t += 0.05
    assert b.phase == "fahren" and a.v == 0.0 and a.w == 0.0
    b.update(Sense(t=t, floor_ok=True, free_center=1.0, nav_extern=True, nav_stuck=True, nav=(0.5, 0.0)))
    assert b.phase == "schauen" and b._fails == 1


def test_pet_ohne_karte_faehrt_das_gehirn():
    cfg, p, b, w = _pet_mit_karte(pose=None)
    c = p.step(w)
    assert abs(c.v - cfg.robot.v_of(cfg.pet.drive_v)) < 1e-9 and "streifen (" not in c.status


def test_dreh_w_gemeinsamer_drehregler():
    from xgonav.behaviors import dreh_w
    r = RobotCfg()
    assert dreh_w(0.0, 1.5, 0.4, r) == 0.0
    assert dreh_w(math.radians(5), 1.5, 0.4, r, deadband_deg=8.0) == 0.0
    klein = math.degrees(1.5 * math.radians(2)) / r.vyaw_limit_dps                    # keine Untergrenze (seit 2026-11-25)
    assert dreh_w(math.radians(2), 1.5, 0.4, r) == pytest.approx(klein)
    assert dreh_w(-math.radians(2), 1.5, 0.4, r) == pytest.approx(-klein)
    assert dreh_w(math.radians(2), 1.5, 0.4, r, w_min=0.06) == pytest.approx(0.06)    # w_min nur noch auf Wunsch
    assert dreh_w(math.radians(170), 1.5, 0.4, r) == pytest.approx(r.w_of(0.4))       # Obergrenze je Modus (Faktor)
    assert dreh_w(math.radians(40), 1.5, 0.8, r) == pytest.approx(math.degrees(1.5 * math.radians(40)) / r.vyaw_limit_dps)
    assert dreh_w(math.radians(2), 1.5, 0.4, r, w_min=0.0) == pytest.approx(math.degrees(1.5 * math.radians(2)) / r.vyaw_limit_dps)
