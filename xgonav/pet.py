"""Modus pet: Der Rider reagiert wie ein kleines Wesen auf Menschen, Katzen und Hunde.

PetBrain ist reine Logik (keine Prozesse, keine Dateien, Zeit von außen, testbar). Pet übersetzt
World -> Sense und Act -> Command. Töne, Emotionen, Roll und Aufstehen laufen über Attribute, die
App._pet_tick abholt, nicht über Command.extra: safety() baut bei einem liegenden Rider einen neuen
Command, ein Aufstehwunsch ginge dort verloren.

Rechenzeit am Pi: nur im Regelschritt (20 Hz), Arithmetik über wenige Tracks, pro Bild nichts.

Aufgaben (Zustand wach, self.task): "suchen" fährt nach dem Verschwinden einer Person zu ihrer letzten Stelle
und schaut sich um, "streifen" schaut sich bei Langeweile um und fährt in Etappen in die freieste Richtung,
nie weiter als roam_radius_m vom Platz, an dem er aufstand. Die Stellen rechnet er selbst aus Rädern und
Kurs (Sense.ds, Sense.yaw), unabhängig von der Raumkarte. Jedes Ziel im Bild beendet die Aufgabe.

Sicherheit: Fahren nur innerhalb der Budgets retreat_m / creep_m (Standard 0), seek_m / roam_m (RiderPi: ohne Grenze)
und nur mit Rädern.
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

from .behaviors import Behavior, Command, World, _wrap, clamp
from .config import PetCfg, RobotCfg

log = logging.getLogger(__name__)

RETREAT_CAP_M, CREEP_CAP_M, ROLL_CAP_DEG = 0.30, 1.0, 12.0
NACHLAUF_S = 0.9                     # nach dem Stoppbefehl rollt er noch ~0,9 s x Tempo (PetCfg.pulse_margin)
CLOSE_M = 0.3                        # Fußpunkt unten abgeschnitten: so nah etwa
# Fußpunkt verdeckt (Kiste, Sofa): der Abstand wird riesig (Simulator 2026-11-13: 10,6 m statt 3,1 m). Darüber
# verworfen, bis dahin gekappt; die Richtung stimmt trotzdem, und gefahren wird ohnehin nur seek_m.
POS_DROP_M, POS_CAP_M = 5.0, 3.5
STALL_S, STALL_M = 2.5, 0.02         # Fahrt befohlen, aber so lange kaum Weg: Bahnprüfung bremst, Etappe aus
GAZE_BINS = 11                       # Blickrichtungen im Augen-Atlas (tools/eyes.py --gaze): 10 Stufen, ungerade für eine exakte Mitte
GAZE_CENTER = GAZE_BINS // 2
BLINK_S = 0.4                        # Lidschlag über alle Lidstellungen; 20 Hz Regelschritt: je 50 ms eine Stufe
GAZE_RANGE = math.radians(45.0)      # Peilung bei voll ausgelenktem Blick
GAZE_LIDS = 5                        # Lidstellungen im Atlas, 0 = offen .. 4 = zu
LID_OPEN, LID_HALF, LID_SHUT = 0, 2, 4
# Firmware-Zustände, aus denen er nie von selbst aufsteht (docs/LED.md)
BAD_FW_MODES = {0, 250, 251, 252}


def _limit(v: float) -> float:
    """Weg/Radius aus der Konfiguration: 0 = aus, negativ = ohne Grenze."""
    return math.inf if v < 0 else v


@dataclass
class Seen:
    id: int
    cls: str
    dist: float | None               # m; 0 = Fußpunkt unten abgeschnitten (sehr nah); None = unbekannt
    bearing: float                   # rad, links +
    top_free: bool                   # Box oben nicht am Bildrand
    width: float                     # Boxbreite (px) der letzten Detektion, hängt anders als der Fußpunkt nicht am Nicken
    t_seen: float
    conf: float = 1.0
    hits: int = 99
    tall: bool = False               # groß im Bild (zählt auch im Liegen, wo die Bodengeometrie nicht stimmt)
    side_free: bool = True           # Box links/rechts nicht am Bildrand: nur dann taugt die Breite


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
    yaw: float = 0.0                 # Kurs rad, links + (World.yaw)
    free_center: float | None = None  # Freiraum im Zentrum (FreeSpace.center), None = Bodenmodell ungültig
    path_blocked: bool = False       # geradeaus mit drive_v laut Merkkarte blockiert (nur während einer Fahrt)


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
        self._lying_seen_t = -1e9             # liegend zuletzt jemanden gesehen
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
        self._blink_t = -1e9
        self._gaze_goal = GAZE_CENTER         # dorthin gleitet der Blick, eine Stufe je Regelschritt
        self._next_glance = 0.0
        # Aufgaben suchen/streifen
        self.seek_m = _limit(cfg.seek_m)
        self.roam_m = _limit(cfg.roam_m)
        self.roam_radius = _limit(cfg.roam_radius_m)
        self.task: str | None = None         # "suchen" | "streifen"
        self.phase = ""                      # "drehen" | "fahren" | "schauen"
        self.n_seek = self.n_found = 0
        self.task_m = 0.0                    # in Aufgaben gefahren (gesamt)
        self.x = self.y = 0.0                # eigene Stelle aus Rädern und Kurs (m)
        self.yaw = 0.0
        self._home: tuple[float, float] | None = None   # Platz, an dem er (wieder) aufstand
        self._last_pos: tuple[float, float] | None = None   # letzte Stelle der Person
        self._goal: tuple[float, float] | None = None
        self._yaw_goal = 0.0
        self._phase_t = 0.0
        self._task_left = 0.0                # Restbudget der laufenden Aufgabe
        self._roam_left = self.roam_m        # Restbudget bis zur nächsten Begegnung
        self._roam_t = -1e9                  # zuletzt gestreift (Start)
        self._look_dir = 1.0
        self._look_i = 0
        self._looks: list[tuple[float, float | None]] = []   # (Kurs, Freiraum) je Blick
        self._stall: tuple[float, float] | None = None    # (Zeit, Weg) seit dem letzten Fortschritt
        self._fails = 0                      # Etappen in Folge, die nicht ankamen
        self._drive_t_max = 0.0

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
        if name != "wach" and self.task is not None:
            self._task_reset(t)                      # Schreck, Ohnmacht, abgesetzt: Aufgabe vorbei
        self.state, self._state_t = name, t

    def _odom(self, s: Sense) -> None:
        """Eigene Stelle aus Rädern und Kurs; liegend zählt nichts (getragen, Aufstehen rollt vor)."""
        if s.balancing and s.ds:
            self.x += s.ds * math.cos(s.yaw)
            self.y += s.ds * math.sin(s.yaw)
            if self.task is not None:
                self._task_left -= abs(s.ds)
                self.task_m += abs(s.ds)
                if self.task == "streifen":
                    self._roam_left -= abs(s.ds)
        self.yaw = s.yaw

    # ---- Hauptschritt -----------------------------------------------------
    def update(self, s: Sense) -> Act:
        c = self.cfg
        if self.base_h is None:
            self.base_h = s.height_mm if s.height_mm is not None else c.height_mm
        self.fright *= math.exp(-c.fright_decay * s.dt)
        self.habit *= math.exp(-s.dt / max(c.habit_tau_s, 1e-3))
        self._odom(s)
        if self.state == "start":
            self._seen_t = s.t
            self._bored_t = -1e9
            self._home = (self.x, self.y) if s.balancing else None
            self._set_state("wach" if s.balancing else "liegt", s.t)
        elif not s.balancing and self.state in ("wach", "schreck", "settle"):
            # nicht von ihm selbst: gefallen oder von außen abgesetzt. Bleibt liegen, nie von selbst hoch.
            self.why_down = "von außen"
            self._set_state("liegt", s.t)
            self._gesture = None
            self.roll_deg = 0.0
            # Sturz (Firmware 0): den Schrei spielt emotion.py in allen Modi, hier nur die Augen
            self._react(s.t, "vertigo", None if s.fw_mode == 0 else "ohnmacht", urgent=True)
        self.target = self._pick(s)
        act = {"wach": self._wach, "schreck": self._schreck, "ab": self._ab, "nap": self._nap,
               "lugen": self._lugen, "warnen": self._warnen, "hoch": self._hoch, "settle": self._settle, "liegt": self._liegt,
               "aus": self._aus}[self.state](s)
        if act.v != 0.0 or act.w != 0.0:
            self._move_t = s.t
        if act.height_mm is None and s.balancing and self.state in ("wach", "schreck", "settle"):
            act.height_mm = self.base_h
        act.target_id = self.target.id if self.target is not None else None
        act.status = "pet: %s%s%s, Vertrauen %.2f" % (self.state, " " + self.zone if self.zone else "",
                                                     " %s/%s" % (self.task, self.phase) if self.task else "", self.trust)
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
            if self.task is not None:
                self._task_found(s)
            self._seen_t = t
            self._bored_t = -1e9
            self._roam_left = self.roam_m            # nach einer Begegnung darf er wieder streifen
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
        if self.task is not None:
            self._task_step(s, act)
            return act
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
                self._task_reset(t)
                self._react(t, "sleep", "seufzen")
            return True
        if (c.look and c.gestures and self.task is None and alone >= c.roam_after_s
                and t >= self._roam_t + c.roam_repeat_s):
            self._roam_t = t
            self._look_dir = self.rng.choice((-1.0, 1.0))
            self._task_start("streifen", s)
        if alone >= c.bored_s and t >= self._bored_t + c.bored_repeat_s and self.task is None:
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
            # Seitlich abgeschnitten: Breite 0 = unbrauchbar (wer ins Bild läuft, dessen Box „wächst“ von selbst)
            self._hist.append((tg.t_seen, tg.dist, tg.width if tg.side_free else 0.0))
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
        h = [p for p in self._hist if p[2] > 0]          # nur ganze Breiten (nicht am Bildrand abgeschnitten)
        if len(h) < 2 or h[-1][0] - h[0][0] < 0.25:
            return 0.0
        return math.log(h[-1][2] / h[0][2]) / (h[-1][0] - h[0][0])

    def _startle_check(self, s: Sense, tg: Seen | None) -> bool:
        c, t = self.cfg, s.t
        # Nur Personen: Eine Katze zieht ihn an, Hunde haben ihre eigene Regel. Kleine Tiere rauschen auch stärker.
        if tg is None or tg.cls != "person" or tg.dist is None or tg.dist > c.startle_max_m or t < self._blind_until:
            return False
        speed = self.approach_speed()
        if speed is None:
            return False
        # Wer mit v auf ihn zukommt, dessen Box wächst mit v/d je Sekunde. Der Fußpunkt allein rauscht aus 12,8 cm
        # Höhe stark (2 m: 1° Nicken oder 3 px Box = 0,3-0,5 m), die Gerade über ~4 Messungen ergab so für eine Person
        # mit 0,3 m/s 1,0 m/s (Simulator 2026-11-13). Es zählt der kleinere Wert aus Gerade und Boxwachstum x Abstand.
        mean_d = sum(p[1] for p in self._hist) / max(len(self._hist), 1)
        speed = min(speed, self.box_growth() * max(mean_d, 0.2) / max(c.startle_vs, 1e-3))
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
            if s.balancing and person.dist is not None and (person.dist <= POS_DROP_M or self._last_pos is None):
                d = min(person.dist, POS_CAP_M) if person.dist > 0 else CLOSE_M
                a = self.yaw + person.bearing
                self._last_pos = (self.x + d * math.cos(a), self.y + d * math.sin(a))
        elif not self._gone_done and t - self._person_t >= c.gone_s:
            self._gone_done = True
            self.zone = None
            if c.look and c.gestures and self.task is None:
                self._react(t, "doubt", None)
                self._look_dir = 1.0 if self._person_bearing >= 0 else -1.0   # zur Seite, wo sie verschwand
                self._task_start("suchen", s, self._seek_goal())
            else:
                self._react(t, "sad", "verloren")

    # ---- Aufgaben: suchen und streifen --------------------------------------
    def _seek_goal(self) -> tuple[float, float] | None:
        """seek_stop_m vor der letzten Stelle der Person; None = schon nah genug, nur umschauen."""
        if self._last_pos is None:
            return None
        dx, dy = self._last_pos[0] - self.x, self._last_pos[1] - self.y
        d = math.hypot(dx, dy)
        go = d - self.cfg.seek_stop_m
        if go < 0.15:
            return None
        return self.x + dx / d * go, self.y + dy / d * go

    def _stop_m(self) -> float:
        return NACHLAUF_S * self.cfg.drive_v * self.rcfg.vx_limit

    def _no_drive(self, s: Sense, budget: float) -> str | None:
        """Grund, nicht zu fahren, oder None."""
        c = self.cfg
        if budget <= self._stop_m() + 0.05:
            return "Budget"
        if not (c.gestures and s.balancing):
            return "Gesten aus" if not c.gestures else "liegt"
        if not s.ds_measured:
            return "keine Räder"
        if not s.floor_ok or s.free_center is None:
            return "Bodenmodell"
        if s.battery is None or s.battery < c.roam_battery:
            return "Akku"
        return None

    def _may_drive(self, s: Sense, budget: float) -> bool:
        return self._no_drive(s, budget) is None

    def _task_start(self, kind: str, s: Sense, goal: tuple[float, float] | None = None) -> None:
        self.task = kind
        self._task_left = self.seek_m if kind == "suchen" else self._roam_left
        self._looks, self._fails = [], 0
        if kind == "suchen":
            self.n_seek += 1
        if goal is not None and self._may_drive(s, self._task_left):
            log.info("pet: %s, fahre zur letzten Stelle", kind)
            self._goal = goal
            self._phase_set("fahren", s.t)
        else:
            log.info("pet: %s, schaue mich um", kind)
            self._phase_set("schauen", s.t)

    def _phase_set(self, name: str, t: float) -> None:
        if self.phase == "fahren" and name != "fahren":
            self._move_t = max(self._move_t, t + NACHLAUF_S)     # Nachlauf ist kein Anstupsen
        self.phase, self._phase_t = name, t
        self._stall = None
        self._yaw_goal = None
        if name == "fahren":
            rest = math.hypot(self._goal[0] - self.x, self._goal[1] - self.y)
            self._drive_t_max = t + 4.0 + 2.0 * rest / max(self.cfg.drive_v * self.rcfg.vx_limit, 1e-3)

    def _task_reset(self, t: float) -> None:
        if self.task is None:
            return
        self._phase_set("", t)
        if self.task == "streifen":
            self._roam_t = t
        self.task, self._goal = None, None

    def _task_found(self, s: Sense) -> None:
        """Ziel im Bild: Aufgabe vorbei. Fuhr er gerade, misst er erst nach dem Nachlauf (Abstand rauscht)."""
        t = s.t
        if self.phase == "fahren":
            self._blind_until = max(self._blind_until, t + NACHLAUF_S + 0.5)
        if self.task == "suchen":
            self.n_found += 1
            self._react(t, "like", "gefunden")
        else:
            self._react(t, "surprised", "gefunden")
        log.info("pet: %s: gefunden", self.task)
        self._task_reset(t)

    def _task_end(self, s: Sense, why: str) -> None:
        log.info("pet: %s endet: %s", self.task, why)
        if self.task == "suchen":
            self._react(s.t, "sad", "verloren")
        self._task_reset(s.t)

    def _task_step(self, s: Sense, act: Act) -> None:
        {"fahren": self._drive, "schauen": self._look}[self.phase](s, act)

    def _drive(self, s: Sense, act: Act) -> None:
        """Zur Stelle self._goal: erst ausrichten, dann langsam fahren. Halten (dann umschauen), wenn der Rest
        im Nachlauf liegt, das Budget nicht mehr reicht, die Bahn blockiert ist oder er nicht vorankommt."""
        c, t = self.cfg, s.t
        dx, dy = self._goal[0] - self.x, self._goal[1] - self.y
        rest = math.hypot(dx, dy)
        err = _wrap(math.atan2(dy, dx) - self.yaw)
        stop = self._stop_m()
        why = None
        if rest <= stop:
            why = "angekommen"
        elif self._no_drive(s, self._task_left):
            why = self._no_drive(s, self._task_left)
        elif s.path_blocked:
            why = "Bahn blockiert"
        elif t > self._drive_t_max:
            why = "Zeit"
        elif self.task == "streifen" and self._home is not None and \
                math.hypot(self.x - self._home[0], self.y - self._home[1]) > self.roam_radius + 0.1:
            why = "Radius"
        if why is None and abs(err) > math.radians(30.0):
            act.w = self._turn_to(err)                       # auf der Stelle ausrichten
            self._stall = None
            return
        if why is None and (s.free_center or 0.0) < 0.5 * c.roam_free_min:
            why = "Freiraum %.2f" % (s.free_center or 0.0)
        if why is None:
            if self._stall is None or self.task_m - self._stall[1] >= STALL_M:
                self._stall = (t, self.task_m)
            elif t - self._stall[0] > STALL_S:
                why = "kommt nicht voran"
        if why is not None:
            log.info("pet: %s: halte (%s, Rest %.2f m)", self.task, why, rest)
            if why != "angekommen":
                self._fails += 1
            self._phase_set("schauen", t)
            return
        act.v = c.drive_v
        act.w = self._turn_to(err)

    def _look(self, s: Sense, act: Act) -> None:
        """Rundumblick in look_steps Blicken: still stehen (Detektor, Bodenmodell), Freiraum merken, weiterdrehen."""
        c, t = self.cfg, s.t
        if self._yaw_goal is not None:
            err = _wrap(self._yaw_goal - self.yaw)
            if abs(err) > math.radians(6.0) and t - self._phase_t < 8.0:
                w = math.degrees(c.turn_kp * abs(err)) / self.rcfg.vyaw_limit_dps
                act.w = math.copysign(clamp(w, c.turn_min_w, max(c.look_w, c.turn_min_w)), err)
                return
            self._yaw_goal, self._phase_t = None, t
            return
        if t - self._phase_t < c.look_pause_s:
            return
        self._looks.append((self.yaw, s.free_center))
        if len(self._looks) < max(c.look_steps, 1):
            self._yaw_goal, self._phase_t = self.yaw + self._look_dir * math.radians(c.look_step_deg), t
            return
        if self.task == "suchen":
            self._task_end(s, "niemand gefunden")
            return
        self._next_leg(s)

    def _next_leg(self, s: Sense) -> None:
        """Streifen: freieste Richtung aus dem Rundumblick, Etappe auf den Radius gekürzt. Draußen: zurück."""
        c = self.cfg
        budget = min(self._task_left, self._roam_left)
        if self._fails >= 3 or not self._may_drive(s, budget):
            self._task_end(s, "nichts mehr zu fahren")
            return
        hx, hy = self._home if self._home is not None else (self.x, self.y)
        leg = min(c.roam_leg_m, budget - self._stop_m())
        best, goal = -1e9, None
        for a, free in self._looks:
            if free is None or free < c.roam_free_min:
                continue
            ex, ey = self.x + leg * math.cos(a), self.y + leg * math.sin(a)
            r = math.hypot(ex - hx, ey - hy)
            if r > self.roam_radius:
                continue
            score = free - (0.3 * r / max(self.roam_radius, 0.1) if math.isfinite(self.roam_radius) else 0.0)                 + self.rng.uniform(0.0, 0.2)
            if score > best:
                best, goal = score, (ex, ey)
        if goal is None and math.hypot(self.x - hx, self.y - hy) > 0.2:
            # nichts Freies im Radius: zum Platz zurück (dreht erst, fährt nur bei Freiraum)
            d = math.hypot(hx - self.x, hy - self.y)
            k = min(leg, d) / d
            goal = (self.x + (hx - self.x) * k, self.y + (hy - self.y) * k)
        if goal is None:
            self._task_end(s, "keine freie Richtung")
            return
        self._looks = []
        self._goal = goal
        self._phase_set("fahren", s.t)

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
            self._react(t, "cute", "aufrappeln", urgent=True)
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
            self._seen_t = s.t                 # Die Zeit im Liegen zählt nicht als allein, sonst setzt er sich gleich wieder ab
            self._home, self._last_pos = (self.x, self.y), None   # womöglich woanders hingestellt
            self._set_state("wach", s.t)
        return Act()

    def _liegt(self, s: Sense) -> Act:
        """Nur Ausdruck. Bleibt liegen, bis ihn jemand aufstellt."""
        if s.balancing:
            self._blind_until = s.t + self.cfg.settle_s
            self._set_state("settle", s.t)
        elif self.target is not None:
            # Nur beim Auftauchen: Wer die ganze Zeit zu sehen ist, löst nichts mehr aus. Kurze Aussetzer des Detektors
            # zählen nicht als weg (gone_s), die Abklingzeit fängt das Kommen und Gehen an der Bildkante ab.
            if s.t - self._lying_seen_t >= self.cfg.gone_s and self._ready("liegend", s.t, 10.0):
                self._react(s.t, "surprised", "gefunden")
            self._lying_seen_t = s.t
        return Act()

    def _aus(self, s: Sense) -> Act:
        return Act()

    # ---- Augen ------------------------------------------------------------
    def _update_gaze(self, s: Sense) -> None:
        t = s.t
        goal = self._gaze_goal
        if self.state == "lugen" and self.target is None:
            # schaut langsam nach links und rechts
            goal = GAZE_CENTER + int(round(GAZE_CENTER * 0.8 * math.sin(2 * math.pi * (t - self._peek_t) / 3.0)))
        elif self.phase == "schauen" and self.target is None and self._yaw_goal is None:
            # beim Umschauen im Stehen: Blick wandert, beim Drehen schaut er in Drehrichtung
            goal = GAZE_CENTER + int(round(GAZE_CENTER * 0.8 * math.sin(2 * math.pi * (t - self._phase_t) / 2.0)))
        elif self.task is not None and self.target is None:
            goal = GAZE_CENTER + (int(round(GAZE_CENTER * 0.6 * self._look_dir)) if self.phase == "schauen" else 0)
        elif self.target is not None:
            # Links vom Rider ist für den, der ihn anschaut, rechts im Bild
            raw = GAZE_CENTER + clamp(self.target.bearing / GAZE_RANGE, -1.0, 1.0) * GAZE_CENTER
            if abs(raw - goal) > 0.6:                        # Hysterese, sonst zittert der Blick
                goal = int(round(raw))
        elif t >= self._next_glance:
            self._next_glance = t + self.rng.uniform(3.0, 7.0)
            goal = GAZE_CENTER + self.rng.choice((-3, -1, 0, 0, 1, 3))
        self._gaze_goal = goal
        gx = self.gaze[0]
        gx += (goal > gx) - (goal < gx)                      # gleitet, springt nicht
        if t >= self._next_blink:
            self._blink_t = t
            self._next_blink = t + BLINK_S + self.rng.uniform(2.0, 6.0)
        x = (t - self._blink_t) / BLINK_S
        closing = 1.0 - abs(2.0 * x - 1.0) if 0.0 <= x < 1.0 else 0.0     # 0 -> 1 -> 0 über den Lidschlag
        if self.state in ("nap", "ab", "aus"):
            lid = LID_SHUT
        elif self.state == "lugen":
            x = clamp((t - self._peek_t) / max(self.cfg.eyes_open_s, 1e-3), 0.0, 1.0)
            lid = LID_SHUT if self.target is not None else int(round(LID_SHUT * (1.0 - x)))
        elif self._dozing or self.state == "liegt":
            lid = LID_HALF + int(round((LID_SHUT - LID_HALF) * closing))
        else:
            lid = LID_OPEN + int(round((LID_SHUT - LID_OPEN) * closing))
        self.gaze = (int(gx), lid)

    def info(self) -> dict:
        return {"zustand": self.state, "zone": self.zone, "vertrauen": round(self.trust, 2),
                "schreck": round(self.fright, 2), "ziel": self.target.cls if self.target else None,
                "schrecks": self.n_startle, "ruck": self.n_retreat, "ohnmacht": self.n_faint, "liegt_weil": self.why_down or None,
                "aufgabe": "%s/%s" % (self.task, self.phase) if self.task else None, "suchen": self.n_seek,
                "gefunden": self.n_found, "gefahren_m": round(self.task_m, 2)}


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

    @property
    def drives(self) -> bool:
        """Fährt gerade eine Aufgabe: Nur dann rechnet App.path_blocked für pet (eine Bahnprüfung je Regelschritt)."""
        return self.brain.phase == "fahren"

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
            wm = getattr(tr, "w_meas", tr.w)             # gemessen, nicht geglättet/vorhergesagt
            out.append(Seen(tr.id, tr.cls, dist, w.bearing(tr.cx), not tr.clip[1] and y1 > 0.03 * w.H, wm or tr.w,
                            tr.t_seen, tr.conf, tr.hits, tr.clip[3] or tr.h > 0.5 * w.H,
                            wm > 0 and x1 > 0.03 * w.W and x2 < 0.97 * w.W))
        return out

    def step(self, w: World) -> Command:
        b = self.brain
        s = Sense(t=w.t, dt=w.dt, balancing=w.balancing, battery=w.battery, seen=self.sense_tracks(w),
                  ds=w.ds, ds_measured=w.ds_measured, tilt=w.tilt, fw_mode=w.fw_mode,
                  floor_ok=w.map_ok and w.free is not None and bool(w.free.valid), height_mm=w.height_mm,
                  yaw=w.yaw, free_center=w.free.center if w.free is not None and w.free.valid else None,
                  path_blocked=w.path_blocked)
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
