"""Modus pet: Der Rider reagiert wie ein kleines Wesen auf Menschen, Katzen und Hunde.

PetBrain ist reine Logik (keine Prozesse, keine Dateien, Zeit von außen, testbar). Pet übersetzt
World -> Sense und Act -> Command. Töne, Emotionen, Roll und Aufstehen laufen über Attribute, die
App._pet_tick abholt, nicht über Command.extra: safety() baut bei einem liegenden Rider einen neuen
Command, ein Aufstehwunsch ginge dort verloren.

Rechenzeit am Pi: nur im Regelschritt (20 Hz), Arithmetik über wenige Tracks, pro Bild nichts.

Sicherheit: Fahren nur innerhalb der Budgets retreat_m / creep_m (Standard 0) und nur mit Rädern.
Vorwärts zusätzlich nur mit gültigem Bodenmodell und Merkkarte, weil safety() ohne Bodenmodell
„frei“ annimmt. Von selbst aufgestanden wird nur, wenn er sich selbst abgesetzt hat und seitdem
nichts an ihm geändert wurde (siehe _nap).
"""
from __future__ import annotations

import collections
import logging
import math
import random
from dataclasses import dataclass, field

from .behaviors import Behavior, Command, World, clamp
from .config import PetCfg, RobotCfg

log = logging.getLogger(__name__)

RETREAT_CAP_M, CREEP_CAP_M, ROLL_CAP_DEG = 0.30, 1.0, 12.0
GAZE_BINS = 7                        # Blickrichtungen im Augen-Atlas (tools/eyes.py --gaze)
GAZE_CENTER = GAZE_BINS // 2
GAZE_RANGE = math.radians(45.0)      # Peilung bei voll ausgelenktem Blick
GAZE_LIDS = 5                        # Lidstellungen im Atlas, 0 = offen .. 4 = zu
LID_OPEN, LID_HALF, LID_SHUT = 0, 2, 4
# Firmware-Zustände, aus denen er nie von selbst aufsteht (docs/LED.md)
BAD_FW_MODES = {0, 250, 251, 252}


@dataclass
class Seen:
    id: int
    cls: str
    dist: float | None               # m; 0 = Fußpunkt unten abgeschnitten (sehr nah); None = unbekannt
    bearing: float                   # rad, links +
    top_free: bool                   # Box oben nicht am Bildrand
    width: float                     # Boxbreite (px), hängt anders als der Fußpunkt nicht am Nicken
    t_seen: float
    conf: float = 1.0
    hits: int = 99
    tall: bool = False               # groß im Bild (zählt auch im Liegen, wo die Bodengeometrie nicht stimmt)


@dataclass
class Sense:
    t: float
    dt: float = 0.05
    balancing: bool = True
    battery: int | None = 100
    seen: list = field(default_factory=list)
    ds: float = 0.0                  # m seit dem letzten Schritt
    ds_measured: bool = True         # ds stammt von den Rädern, nicht aus dem Fahrbefehl
    tilt: tuple | None = (0.0, 0.0)  # (roll, pitch) in Grad, None = keine frische IMU
    fw_mode: int | None = 1
    floor_ok: bool = False           # Bodenmodell gültig und Merkkarte vorhanden
    height_mm: float | None = None


@dataclass
class Act:
    v: float = 0.0
    w: float = 0.0
    height_mm: float | None = None
    status: str = ""
    target_id: int | None = None


class PetBrain:
    def __init__(self, cfg: PetCfg, rcfg: RobotCfg, seed: int | None = None):
        self.cfg, self.rcfg = cfg, rcfg
        self.rng = random.Random(seed)
        # Grenzen hart hier, nicht in der Konfiguration: Eine falsche TOML darf sie nicht aufheben
        self.retreat_m = clamp(cfg.retreat_m, 0.0, RETREAT_CAP_M)
        self.creep_m = clamp(cfg.creep_m, 0.0, CREEP_CAP_M)
        self.roll_mag = clamp(cfg.roll_deg, 0.0, ROLL_CAP_DEG)
        self.duck_mm = max(cfg.duck_mm, rcfg.height_min_mm + 10.0)
        self.state = "start"
        self.trust = clamp(cfg.trust_start, 0.0, 1.0)
        self.fright = 0.0
        self.habit = 0.0
        self.events: list[tuple[str, str, bool]] = []      # (emotion|sound, Name, dringend)
        self._queue: list[tuple[float, str, str, bool]] = []
        self.gaze = (GAZE_CENTER, LID_OPEN)
        self.roll_deg = 0.0
        self.rolled = False                  # Roll wurde benutzt: beim Verlassen neu ausrichten
        self.det_hz = cfg.active_hz
        self.want_set_down = self.want_stand_up = self.want_idle = False
        self.base_h: float | None = None
        self.zone: str | None = None
        self.target: Seen | None = None
        self.n_retreat = self.n_faint = self.n_startle = 0
        self.why_down = ""
        self._until = 0.0
        self._state_t = 0.0
        self._cool: dict[str, float] = {}
        self._hist: collections.deque = collections.deque()   # (t_seen, Abstand) des Ziels
        self._hist_id: int | None = None
        self._blind_until = 0.0
        self._gesture: tuple[str, float, float] | None = None   # Name, Start, Ende
        self._duck_until = 0.0
        self._retreat_left = 0.0
        self._retreat_t = -1e9
        self._retreat_until = 0.0
        self._creep_s = 0.0                  # Zeitimpuls des Schleichens, Rest dieser Begegnung
        self._esc: tuple[float, float] | None = None   # (Zeit, Abstand) beim Auslösen des Schrecks
        self._creep_left = self.creep_m
        self._cat_t = -1e9
        self._zone_cand: str | None = None
        self._zone_t = 0.0
        self._person_t = -1e9
        self._person_since: float | None = None
        self._person_bearing = 0.0
        self._gone_done = True
        self._seen_t = None                  # letzte Begegnung (irgendein Ziel)
        self._bored_t = 0.0
        self._dozing = False
        self._nudge: collections.deque = collections.deque()
        self._move_t = -1e9
        self._faints: collections.deque = collections.deque()
        self._down_sent = False
        self._nap_t0 = 0.0
        self._nap_ref = None                 # (roll, pitch, Firmware-Zustand) in Ruhe
        self._nap_moved = 0.0
        self._peek_t = 0.0                   # Augen gehen seit dann auf
        self._clear_t: float | None = None   # niemand im Bild seit dann
        self._next_blink = 0.0
        self._blink_until = 0.0
        self._next_glance = 0.0

    # ---- Ausgabe ----------------------------------------------------------
    def _react(self, t: float, emotion: str | None = None, sound: str | None = None, urgent: bool = False) -> None:
        """Dringendes sofort, sonst nach kurzer Zufallspause: starre Reflexe wirken mechanisch."""
        at = t if urgent else t + self.rng.uniform(0.2, 0.6)
        if emotion:
            self._queue.append((at, "emotion", emotion, urgent))
        if sound:
            self._queue.append((at, "sound", sound, urgent))

    def _flush(self, t: float) -> None:
        keep = []
        for ev in self._queue:
            if ev[0] <= t:
                self.events.append(ev[1:])
            else:
                keep.append(ev)
        self._queue = keep

    def drain(self) -> list[tuple[str, str, bool]]:
        ev, self.events = self.events, []
        return ev

    def _ready(self, key: str, t: float, repeat_s: float) -> bool:
        if t < self._cool.get(key, -1e9):
            return False
        self._cool[key] = t + repeat_s
        return True

    def _set_state(self, name: str, t: float) -> None:
        if name != self.state:
            log.info("pet: %s -> %s", self.state, name)
        self.state, self._state_t = name, t

    # ---- Hauptschritt -----------------------------------------------------
    def update(self, s: Sense) -> Act:
        c = self.cfg
        if self.base_h is None:
            self.base_h = s.height_mm if s.height_mm is not None else c.height_mm
        self.fright *= math.exp(-c.fright_decay * s.dt)
        self.habit *= math.exp(-s.dt / max(c.habit_tau_s, 1e-3))
        if self.state == "start":
            self._seen_t = s.t
            self._bored_t = -1e9
            self._set_state("wach" if s.balancing else "liegt", s.t)
        elif not s.balancing and self.state in ("wach", "schreck", "settle"):
            # nicht von ihm selbst: gefallen oder von außen abgesetzt. Bleibt liegen, nie von selbst hoch.
            self.why_down = "von außen"
            self._set_state("liegt", s.t)
            self._gesture = None
            self.roll_deg = 0.0
            self._react(s.t, "vertigo", "ohnmacht", urgent=True)
        self.target = self._pick(s)
        act = {"wach": self._wach, "schreck": self._schreck, "ab": self._ab, "nap": self._nap,
               "lugen": self._lugen, "warnen": self._warnen, "hoch": self._hoch, "settle": self._settle, "liegt": self._liegt,
               "aus": self._aus}[self.state](s)
        if act.v != 0.0 or act.w != 0.0:
            self._move_t = s.t
        if act.height_mm is None and s.balancing and self.state in ("wach", "schreck", "settle"):
            act.height_mm = self.base_h
        act.target_id = self.target.id if self.target is not None else None
        act.status = "pet: %s%s, Vertrauen %.2f" % (self.state, " " + self.zone if self.zone else "", self.trust)
        self._update_gaze(s)
        self._flush(s.t)
        return act

    def _pick(self, s: Sense) -> Seen | None:
        """Hund vor Katze vor Person, je das nächste. Tiere erst nach mehreren sicheren Treffern.
        Für die Ohnmacht zählt dagegen jede Box (_air_clear): Beim Aufspringen lieber einmal zu vorsichtig."""
        c = self.cfg
        best = None
        for rank, cls in enumerate(("dog", "cat", "person")):
            cand = [x for x in s.seen if x.cls == cls
                    and (cls == "person" or (x.conf >= c.animal_conf and x.hits >= c.animal_hits))]
            if cand:
                best = min(cand, key=lambda x: x.dist if x.dist is not None else 1e9)
                break
        return best

    # ---- Wach -------------------------------------------------------------
    def _wach(self, s: Sense) -> Act:
        c, t = self.cfg, s.t
        tg = self.target
        act = Act()
        self._measure(s, tg)
        if tg is not None:
            if self._dozing:
                self._dozing = False
                self.det_hz = c.active_hz
                self._react(t, "surprised", "gefunden")
            self._seen_t = t
            self._bored_t = -1e9
        elif self._lonely(s):
            return act
        if tg is not None and tg.cls == "dog" and self._dog(s, tg):
            return self._schreck_act() if self.state == "schreck" else Act()
        if self._startle_check(s, tg):
            return self._schreck_act() if self.state == "schreck" else Act()
        if tg is not None and tg.cls == "cat":
            self._cat(s, tg, act)
        person = next((x for x in s.seen if x.cls == "person"), None)
        if tg is not None and tg.cls == "person":
            self._person(s, tg, act)
        self._person_presence(s, person)
        self._nudge_check(s)
        self._apply_gesture(s, act, tg)
        return act

    def _lonely(self, s: Sense) -> bool:
        """Niemand in Sicht: Langeweile, Dösen, nach timeout_s absetzen. True: nichts weiter zu tun."""
        c, t = self.cfg, s.t
        alone = t - self._seen_t
        if alone >= c.timeout_s:
            log.info("pet: %.0f s niemand, setze ab (Akku)", alone)
            self.want_set_down = self.want_idle = True
            self._set_state("aus", t)
            self._react(t, "sleep", "seufzen")
            return True
        if alone >= c.doze_s:
            if not self._dozing:
                self._dozing = True
                self.det_hz = c.doze_hz
                self.zone = None
                self._react(t, "sleep", "seufzen")
            return True
        if alone >= c.bored_s and t >= self._bored_t + c.bored_repeat_s:
            self._bored_t = t
            self._react(t, None, "seufzen")
            if c.gestures:
                self._gesture = ("schauen", t, t + 2.4)
        return False

    # ---- Abstand und Schreck ----------------------------------------------
    def _measure(self, s: Sense, tg: Seen | None) -> None:
        """Abstandsverlauf des Ziels, nur aus neuen Detektionen; nie während eigener Bewegung."""
        if tg is None or tg.id != self._hist_id or s.t < self._blind_until:
            self._hist.clear()
            self._hist_id = tg.id if tg is not None else None
            if tg is None or s.t < self._blind_until:
                return
        if tg.dist is None or tg.dist <= 0.0:
            return
        if not self._hist or tg.t_seen > self._hist[-1][0]:
            self._hist.append((tg.t_seen, tg.dist, tg.width))
        while self._hist and self._hist[0][0] < tg.t_seen - self.cfg.startle_window_s:
            self._hist.popleft()

    def approach_speed(self) -> float | None:
        """m/s auf ihn zu (Geradenanpassung), None bei zu wenigen Messungen."""
        h = self._hist
        if len(h) < self.cfg.startle_min_samples or h[-1][0] - h[0][0] < 0.25:
            return None
        n = len(h)
        mt = sum(p[0] for p in h) / n
        md = sum(p[1] for p in h) / n
        den = sum((p[0] - mt) ** 2 for p in h)
        if den <= 0:
            return None
        return -sum((p[0] - mt) * (p[1] - md) for p in h) / den

    def box_growth(self) -> float:
        """Wachstum der Boxbreite 1/s über das Fenster. Track.vs taugt hier nicht: Es reagiert träge, und aus
        dieser Höhe ist die Personenbox ab 1,9 m oben abgeschnitten."""
        h = self._hist
        if len(h) < 2 or h[0][2] <= 0 or h[-1][2] <= 0 or h[-1][0] <= h[0][0]:
            return 0.0
        return math.log(h[-1][2] / h[0][2]) / (h[-1][0] - h[0][0])

    def _startle_check(self, s: Sense, tg: Seen | None) -> bool:
        c, t = self.cfg, s.t
        # Nur Personen: Eine Katze zieht ihn an, Hunde haben ihre eigene Regel. Kleine Tiere rauschen auch stärker.
        if tg is None or tg.cls != "person" or tg.dist is None or tg.dist > c.startle_max_m or t < self._blind_until:
            return False
        speed = self.approach_speed()
        # Wer mit v auf ihn zukommt, dessen Box wächst mit v/d je Sekunde; Rauschen im Fußpunkt tut das nicht
        mean_d = sum(p[1] for p in self._hist) / max(len(self._hist), 1)
        if speed is None or self.box_growth() < c.startle_vs * speed / max(mean_d, 0.2):
            return False
        # Vertrauen 0,5 = Grundwert; wer ihm vertraut, darf schneller kommen
        thr = c.startle_mps * (0.7 + 0.6 * self.trust)
        if speed < thr or not self._ready("schreck", t, c.startle_cooldown_s):
            return False
        x = clamp((speed - thr) / max(c.startle_heavy_mps - thr, 1e-3), 0.0, 1.0)
        self._esc = (tg.t_seen, tg.dist)      # kommt er weiter so schnell, wird es heftig (_schreck)
        self._frighten(s, (0.4 + 1.4 * x) * (1.0 - self.trust) * 0.5 ** self.habit)
        return True

    def _frighten(self, s: Sense, m: float, force_faint: bool = False) -> None:
        """Schreck der Stärke m auf den Pegel addieren und darauf reagieren."""
        c, t = self.cfg, s.t
        self.fright += m
        self.habit += 1.0
        self.n_startle += 1
        self.trust = clamp(self.trust - c.trust_loss, 0.0, 1.0)
        log.info("pet: Schreck %.2f, Pegel %.2f, Vertrauen %.2f", m, self.fright, self.trust)
        if (force_faint or self.fright >= c.faint_level) and self._can_faint(s):
            self._faint(s)
            return
        if m < c.startle_min and not force_faint:
            self._react(t, "doubt", None, urgent=True)
            return
        self._react(t, "surprised", "quiek", urgent=True)
        self._start_schreck(s)

    def _start_schreck(self, s: Sense) -> None:
        c, t = self.cfg, s.t
        self._gesture = None
        self.roll_deg = 0.0
        self._until = t + c.startle_s
        self._blind_until = t + c.startle_s + 1.5
        self._retreat_left = 0.0
        if self.retreat_m > 0 and s.ds_measured and c.gestures and t - self._retreat_t >= c.retreat_cooldown_s:
            self._retreat_left = self.retreat_m
            self._retreat_t = t
            self._retreat_until = t + self._pulse_s(self.retreat_m, c.retreat_v, self.rcfg.reverse_gain)
            self.n_retreat += 1
        self._set_state("schreck", t)

    def _pulse_s(self, budget_m: float, v: float, gain: float = 1.0) -> float:
        """So lange darf v anliegen, damit der gesamte Weg samt Nachlauf im Budget bleibt."""
        return self.cfg.pulse_margin * budget_m / max(abs(v) * gain * self.rcfg.vx_limit, 1e-3)

    def _schreck_act(self) -> Act:
        """Erster Schritt der Reaktion: gleich abducken, gefahren wird ab dem nächsten."""
        return Act(height_mm=self.duck_mm if self.cfg.gestures else None)

    def _schreck(self, s: Sense) -> Act:
        c, t = self.cfg, s.t
        act = self._schreck_act()
        # Ausgelöst wird beim ersten Überschreiten der Schwelle, da ist die Geschwindigkeit noch unterschätzt.
        # Rennt jemand weiter auf ihn zu, ist es der heftige Schreck.
        tg = self.target
        if self._esc is not None and tg is not None and tg.dist and tg.t_seen - self._esc[0] >= 0.3:
            speed = (self._esc[1] - tg.dist) / (tg.t_seen - self._esc[0])
            self._esc = None
            if speed >= c.startle_heavy_mps:
                self.fright += 1.1 * (1.0 - self.trust) * 0.5 ** max(self.habit - 1.0, 0.0)
                log.info("pet: heftiger Schreck, %.1f m/s, Pegel %.2f", speed, self.fright)
                if self.fright >= c.faint_level and self._can_faint(s):
                    self._faint(s)
                    return Act()
        if self._retreat_left > 0:
            if not s.ds_measured:
                self._retreat_left = 0.0
            else:
                self._retreat_left -= abs(s.ds)
                if self._retreat_left <= 0.0 or t >= min(self._retreat_until, self._state_t + c.retreat_max_s):
                    self._retreat_left = 0.0
                else:
                    act.v = -c.retreat_v
        if t >= self._until:
            self._set_state("wach", t)
        return act

    # ---- Tiere ------------------------------------------------------------
    def _dog(self, s: Sense, tg: Seen) -> bool:
        """True: Reaktion gestartet, der Rest des Schritts entfällt."""
        c, t = self.cfg, s.t
        if tg.dist is not None and tg.dist <= c.dog_close_m:
            if self._ready("hund_nah", t, c.startle_cooldown_s):
                self._frighten(s, 1.5, force_faint=True)
                return True
        if tg.dist is not None and tg.dist <= c.dog_fear_m and self._ready("hund", t, c.retreat_cooldown_s):
            self.trust = clamp(self.trust - c.trust_loss, 0.0, 1.0)
            self._react(t, "grievance", "wimmern", urgent=True)
            self._start_schreck(s)
            return True
        return False

    def _cat(self, s: Sense, tg: Seen, act: Act) -> None:
        c, t = self.cfg, s.t
        if t - self._cat_t > 30.0:
            self._creep_left = self.creep_m                  # neue Begegnung
            self._creep_s = self._pulse_s(self.creep_m, c.creep_v)
        self._cat_t = t
        if self._ready("katze", t, c.animal_repeat_s):
            self._react(t, "like", "gurren")
        self._creep_left -= abs(s.ds)
        if (self._creep_left > 0 and self._creep_s > 0 and c.gestures and s.floor_ok and s.ds_measured
                and tg.dist is not None and tg.dist > c.cat_stop_m and abs(tg.bearing) < math.radians(15.0)):
            self._creep_s -= s.dt
            act.v = c.creep_v

    # ---- Person -----------------------------------------------------------
    def _zone_of(self, tg: Seen) -> str | None:
        c = self.cfg
        if tg.dist is None:
            return None
        if tg.dist < c.kneel_max_m and tg.top_free:
            return "knien"
        close = c.zone_close_m * (1.3 - 0.6 * self.trust)
        if tg.dist < close:
            return "zunah"
        if tg.dist < c.zone_mid_m:
            return "nah"
        return "mitte" if tg.dist < c.zone_far_m else "fern"

    def _person(self, s: Sense, tg: Seen, act: Act) -> None:
        c, t = self.cfg, s.t
        z = self._zone_of(tg)
        if z != self._zone_cand:
            self._zone_cand, self._zone_t = z, t
        elif z is not None and z != self.zone and t - self._zone_t >= c.zone_hold_s:
            self.zone = z
            if self._ready("zone_" + z, t, c.zone_repeat_s):
                self._enter_zone(z, t, tg)
        if self.zone in ("nah", "knien"):
            self.trust = clamp(self.trust + c.trust_gain * s.dt, 0.0, 1.0)
        if self.zone == "zunah" and c.gestures:
            if self._duck_until < t:
                self._blind_until = max(self._blind_until, t + 1.5)  # Höhe ändert die Bodengeometrie
            self._duck_until = t + 0.5
            act.height_mm = self.duck_mm

    def _enter_zone(self, z: str, t: float, tg: Seen) -> None:
        c = self.cfg
        if z == "fern":
            self._react(t, None, "gefunden")
        elif z == "mitte":
            self._react(t, "doubt", None)
            # Während der Geste misst er nicht: nicht, wenn gerade jemand schnell näherkommt
            speed = self.approach_speed()
            if c.gestures and self.roll_mag > 0 and (speed is None or speed < 0.5 * c.startle_mps):
                self._gesture = ("kopf", t, t + c.tilt_s)
                self.roll_deg = self.rng.choice((-1.0, 1.0)) * self.roll_mag
                self.rolled = True
                self._blind_until = max(self._blind_until, t + c.tilt_s + 0.5)
        elif z == "nah":
            self._react(t, "cute", "traellern")
            if c.gestures:
                self._gesture = ("wackeln", t, t + c.wiggle_s)
        elif z == "knien":
            self._react(t, "like", "traellern")
        elif z == "zunah":
            self._react(t, "shy", None)

    def _person_presence(self, s: Sense, person: Seen | None) -> None:
        """Guck-guck und Hinterherschauen."""
        c, t = self.cfg, s.t
        if person is not None:
            gone = t - self._person_t
            long_seen = self._person_since is not None and self._person_t - self._person_since >= 2.0
            if long_seen and 0.5 <= gone < c.peek_s and self._ready("guckguck", t, 10.0):
                self._react(t, "laugh", "kichern")
            if gone >= c.gone_s or self._person_since is None:
                self._person_since = t
            self._person_t = t
            self._person_bearing = person.bearing
            self._gone_done = False
        elif not self._gone_done and t - self._person_t >= c.gone_s:
            self._gone_done = True
            self.zone = None
            self._react(t, "sad", "verloren")

    def _nudge_check(self, s: Sense) -> None:
        """Radweg, obwohl er seit 1 s keinen Fahrbefehl hat: jemand hat ihn angestupst."""
        c, t = self.cfg, s.t
        self._nudge.append((t, s.ds))
        while self._nudge and self._nudge[0][0] < t - 0.5:
            self._nudge.popleft()
        if (s.ds_measured and t - self._move_t > 1.0 and abs(sum(p[1] for p in self._nudge)) >= c.nudge_m
                and self._ready("anstupsen", t, 5.0)):
            self.trust = clamp(self.trust - 0.02, 0.0, 1.0)
            self._react(t, "embarrassed", "hey")

    def _apply_gesture(self, s: Sense, act: Act, tg: Seen | None) -> None:
        c, t = self.cfg, s.t
        g = self._gesture
        if g is not None and t >= g[2]:
            if g[0] == "kopf":
                self.roll_deg = 0.0
            self._gesture = g = None
        if not c.gestures:
            return
        if g is not None:
            period = 0.2 if g[0] == "wackeln" else 0.6
            if g[0] != "kopf":
                sign = 1.0 if int((t - g[1]) / period) % 2 == 0 else -1.0
                act.w = sign * (c.wiggle_w if g[0] == "wackeln" else c.turn_min_w)
            return
        if tg is not None:
            act.w = self._turn_to(tg.bearing)
        elif not self._gone_done and t - self._person_t >= 0.5:
            act.w = math.copysign(c.turn_min_w, self._person_bearing)    # schaut hinterher

    def _turn_to(self, err: float) -> float:
        c = self.cfg
        if abs(err) < math.radians(c.turn_deadband_deg):
            return 0.0
        w = math.degrees(c.turn_kp * err) / self.rcfg.vyaw_limit_dps
        return math.copysign(clamp(abs(w), c.turn_min_w, c.turn_w), w)

    # ---- Ohnmacht ---------------------------------------------------------
    def _can_faint(self, s: Sense) -> bool:
        c, t = self.cfg, s.t
        while self._faints and self._faints[0] < t - c.faint_window_s:
            self._faints.popleft()
        return (c.faint and len(self._faints) < c.faint_max
                and s.battery is not None and s.battery > c.battery_low)

    def _faint(self, s: Sense) -> None:
        t = s.t
        self._faints.append(t)
        self.n_faint += 1
        self.fright = 0.0
        self._gesture = None
        self.roll_deg = 0.0
        self._down_sent = False
        self._react(t, "vertigo", "ohnmacht", urgent=True)
        self._set_state("ab", t)

    def _ab(self, s: Sense) -> Act:
        """Kurz die Reaktion zeigen, dann absetzen und warten, bis er liegt."""
        t = s.t
        if not self._down_sent and t - self._state_t >= 0.8:
            self._down_sent = True
            self.want_set_down = True
        if self._down_sent and not s.balancing:
            self._nap_t0, self._nap_ref, self._nap_moved = t, None, 0.0
            self._set_state("nap", t)
        elif t - self._state_t > 4.0:
            log.warning("pet: Absetzen kam nicht an")
            self._set_state("wach", t)
        return Act()

    def _nap_disturbed(self, s: Sense) -> str | None:
        """Grund, nie mehr von selbst aufzustehen. Prüfungen, die fehlen, zählen wie eine Störung."""
        c, t = self.cfg, s.t
        if s.fw_mode in BAD_FW_MODES:
            return "Firmware-Zustand %s" % s.fw_mode
        if t - self._nap_t0 < c.nap_rest_s:
            return None                                      # fällt noch in die Liegelage
        if s.tilt is None or s.fw_mode is None or not s.ds_measured:
            return "IMU, Räder oder Zustand unbekannt"
        if self._nap_ref is None:
            self._nap_ref = (s.tilt[0], s.tilt[1], s.fw_mode)
            self._nap_moved = 0.0
            return None
        self._nap_moved += abs(s.ds)
        if max(abs(s.tilt[0] - self._nap_ref[0]), abs(s.tilt[1] - self._nap_ref[1])) > c.nap_tilt_deg:
            return "Lage geändert (hochgehoben?)"
        if self._nap_moved > c.nap_moved_m:
            return "Räder bewegt"
        if s.fw_mode != self._nap_ref[2]:
            return "Firmware-Zustand %s statt %s" % (s.fw_mode, self._nap_ref[2])
        return None

    def _air_clear(self, s: Sense) -> bool:
        """Ist die Luft rein? Niemand im Bild, egal wie weit weg: Im Liegen stimmt die Bodengeometrie nicht,
        und wer ihn erschreckt hat, soll weg sein. Dazu Akku bekannt und die Liegelage als Referenz."""
        return (not s.seen and s.battery is not None and s.battery > self.cfg.battery_low
                and self._nap_ref is not None)

    def _nap_check(self, s: Sense) -> bool:
        """Gemeinsam für nap und warnen. False: Zustand ist schon gewechselt."""
        if s.balancing:                                      # von außen aufgestellt
            self._set_state("settle", s.t)
            self._blind_until = s.t + self.cfg.settle_s
            return False
        why = self._nap_disturbed(s)
        if why:
            log.warning("pet: stehe nicht von selbst auf: %s", why)
            self.why_down = why
            self._set_state("liegt", s.t)
            self._react(s.t, "sad", None)
            return False
        return True

    def _nap(self, s: Sense) -> Act:
        c, t = self.cfg, s.t
        if not self._nap_check(s):
            return Act()
        if t - self._nap_t0 >= c.nap_s:
            self._peek_t, self._clear_t = t, None
            self._set_state("lugen", t)
        return Act()

    def _lugen(self, s: Sense) -> Act:
        """Augen langsam auf, sich umschauen. Ist jemand im Bild, gehen sie wieder zu (stellt sich tot) und
        fangen danach von vorn an. Erst wenn die Luft clear_s lang rein ist, kommt der Warnton."""
        c, t = self.cfg, s.t
        if not self._nap_check(s):
            return Act()
        if t - self._nap_t0 >= c.nap_max_s:
            log.warning("pet: nach %.0f s ist die Luft nicht rein, bleibe liegen", t - self._nap_t0)
            self.why_down = "Luft nicht rein"
            self._set_state("liegt", t)
            self._react(t, "sad", None)
        elif not self._air_clear(s):
            self._clear_t = None
            self._peek_t = t
        else:
            if self._clear_t is None:
                self._clear_t = t
            if t - self._clear_t >= c.clear_s and t - self._peek_t >= c.eyes_open_s:
                self._react(t, None, "aufwachen", urgent=True)
                self._set_state("warnen", t)
        return Act()

    def _warnen(self, s: Sense) -> Act:
        c, t = self.cfg, s.t
        if not self._nap_check(s):
            return Act()
        if not self._air_clear(s):
            self._peek_t, self._clear_t = t, None
            self._set_state("lugen", t)
        elif t - self._state_t >= c.standup_warn_s:
            self.want_stand_up = True
            self._set_state("hoch", t)
        return Act()

    def _hoch(self, s: Sense) -> Act:
        c, t = self.cfg, s.t
        if s.balancing:
            self._react(t, "cute", None)
            self._blind_until = t + c.settle_s
            self._set_state("settle", t)
        elif t - self._state_t > c.standup_timeout_s:
            log.warning("pet: Aufstehen fehlgeschlagen, kein zweiter Versuch")
            self.why_down = "Aufstehen fehlgeschlagen"
            self._set_state("liegt", t)
        return Act()

    def _settle(self, s: Sense) -> Act:
        """Aufschwung, Nivellier-Impuls und Standreferenz abwarten; Höhe steht nach dem Aufstehen auf 128."""
        if s.t - self._state_t >= self.cfg.settle_s:
            self._set_state("wach", s.t)
        return Act()

    def _liegt(self, s: Sense) -> Act:
        """Nur Ausdruck. Bleibt liegen, bis ihn jemand aufstellt."""
        if s.balancing:
            self._blind_until = s.t + self.cfg.settle_s
            self._set_state("settle", s.t)
        elif self.target is not None and self._ready("liegend", s.t, 10.0):
            self._react(s.t, "surprised", "gefunden")
        return Act()

    def _aus(self, s: Sense) -> Act:
        return Act()

    # ---- Augen ------------------------------------------------------------
    def _update_gaze(self, s: Sense) -> None:
        t = s.t
        gx, lid = self.gaze
        if self.state == "lugen" and self.target is None:
            # schaut langsam nach links und rechts
            gx = GAZE_CENTER + int(round(GAZE_CENTER * 0.8 * math.sin(2 * math.pi * (t - self._peek_t) / 3.0)))
        elif self.target is not None:
            # Links vom Rider ist für den, der ihn anschaut, rechts im Bild
            raw = GAZE_CENTER + clamp(self.target.bearing / GAZE_RANGE, -1.0, 1.0) * GAZE_CENTER
            if abs(raw - gx) > 0.6:                          # Hysterese, sonst zittert der Blick
                gx = int(round(raw))
        elif t >= self._next_glance:
            self._next_glance = t + self.rng.uniform(3.0, 7.0)
            gx = clamp(GAZE_CENTER + self.rng.choice((-2, -1, 0, 0, 1, 2)), 0, GAZE_BINS - 1)
        if t >= self._next_blink:
            self._blink_until = t + 0.14
            self._next_blink = t + self.rng.uniform(2.0, 6.0)
        if self.state in ("nap", "ab", "aus"):
            lid = LID_SHUT
        elif self.state == "lugen":
            x = clamp((t - self._peek_t) / max(self.cfg.eyes_open_s, 1e-3), 0.0, 1.0)
            lid = LID_SHUT if self.target is not None else int(round(LID_SHUT * (1.0 - x)))
        elif self._dozing or self.state == "liegt":
            lid = LID_HALF if t >= self._blink_until else LID_SHUT
        else:
            lid = LID_SHUT if t < self._blink_until else LID_OPEN
        self.gaze = (int(gx), lid)

    def info(self) -> dict:
        return {"zustand": self.state, "zone": self.zone, "vertrauen": round(self.trust, 2),
                "schreck": round(self.fright, 2), "ziel": self.target.cls if self.target else None,
                "schrecks": self.n_startle, "ruck": self.n_retreat, "ohnmacht": self.n_faint, "liegt_weil": self.why_down or None}


class Pet(Behavior):
    name = "pet"

    def __init__(self, cfg, rcfg: RobotCfg, pcfg: PetCfg | None = None, seed: int | None = None):
        super().__init__(cfg, rcfg)
        self.pcfg = pcfg or PetCfg()
        self.brain = PetBrain(self.pcfg, rcfg, seed)
        self.want_set_down = False           # App._step
        self.want_stand_up = False           # App._pet_tick
        self.want_idle = False
        self.gaze = self.brain.gaze          # lcd.py liest das aus seinem Thread (Tupel, atomar ersetzt)

    @staticmethod
    def sense_tracks(w: World) -> list[Seen]:
        out = []
        for tr in w.tracks:
            if tr.cls not in ("person", "cat", "dog") or tr.lost_s(w.t) >= 0.5:
                continue
            x1, y1, x2, y2 = tr.box
            dist = None
            if w.balancing and w.geo is not None:        # liegend stimmt die Bodengeometrie nicht
                dist = 0.0 if tr.clip[3] else w.geo.ground_dist(y2, w.pitch_dev)
            out.append(Seen(tr.id, tr.cls, dist, w.bearing(tr.cx), not tr.clip[1] and y1 > 0.03 * w.H, tr.w,
                            tr.t_seen, tr.conf, tr.hits, tr.clip[3] or tr.h > 0.5 * w.H))
        return out

    def step(self, w: World) -> Command:
        b = self.brain
        s = Sense(t=w.t, dt=w.dt, balancing=w.balancing, battery=w.battery, seen=self.sense_tracks(w),
                  ds=w.ds, ds_measured=w.ds_measured, tilt=w.tilt, fw_mode=w.fw_mode,
                  floor_ok=w.map_ok and w.free is not None and bool(w.free.valid), height_mm=w.height_mm)
        act = b.update(s)
        if b.want_set_down:
            b.want_set_down, self.want_set_down = False, True
        if b.want_stand_up:
            b.want_stand_up, self.want_stand_up = False, True
        if b.want_idle:
            b.want_idle, self.want_idle = False, True
        self.gaze = b.gaze
        return Command(act.v, act.w, act.height_mm, act.status, act.target_id)

    def info(self) -> dict:
        return self.brain.info()
