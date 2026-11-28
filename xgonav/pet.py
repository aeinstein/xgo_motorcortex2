"""Modus pet: Der Rider reagiert wie ein kleines Wesen auf Menschen, Katzen und Hunde.

PetBrain ist reine Logik (keine Prozesse, keine Dateien, Zeit von außen, testbar). Pet übersetzt
World -> Sense und Act -> Command. Töne, Emotionen, Roll und Aufstehen laufen über Attribute, die
App._pet_tick abholt, nicht über Command.extra: safety() baut bei einem liegenden Rider einen neuen
Command, ein Aufstehwunsch ginge dort verloren.

Rechenzeit am Pi: nur im Regelschritt (control_hz), Arithmetik über wenige Tracks, pro Bild nichts.

Aufgaben (Zustand wach, self.task): "suchen" fährt nach dem Verschwinden einer Person zu ihrer letzten Stelle
und schaut sich um, "streifen" schaut sich bei Langeweile um und fährt in Etappen in die freieste Richtung,
nie weiter als roam_radius_m vom Platz, an dem er aufstand. Die Stellen rechnet er selbst aus Rädern und
Kurs (Sense.ds, Sense.yaw), unabhängig von der Raumkarte. Jedes Ziel im Bild beendet die Aufgabe.

Hören (Sense.heard, Meldungen der Schallortung): "horchen" dreht sich zu einem Geräusch (Phase "drehen"), lauscht
nach ("lauschen") und dreht weiter, wenn es wieder von der Seite kommt (dann lag die Quelle hinten: zwei
Mikrofone trennen vorn und hinten nicht). Sieht er eine Person und sie spricht ihn an, fährt er auf sie zu
(Aufgabe "kommen", come_m, hält come_stop_m vor ihr). Hält eine Stimme von vorn an und ist niemand zu sehen, sucht er dort
(Rufen, call_m). Plötzlich laut nach Stille: zuckt zusammen und dreht sich schnell hin. Dösend wecken ihn
Stimmen; liegend nach einer Ohnmacht ist die Luft nicht rein, solange jemand spricht. Sieht er jemanden und
der macht eine Sprechpause, antwortet er (Mitreden). Während er selbst dreht oder fährt, zählt keine Richtung.
Nachplappern (Sense.stimmen, stimme.Stimmen): eine neu bestätigte Stimmprobe spielt er einmal gleich nach, beim
Mitreden mit parrot_p eine gelernte statt des Brabbelns. hoert_zu sagt der App, wann sie mitschneiden darf.

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
from dataclasses import dataclass, field, replace

from .behaviors import Behavior, Command, Follow, Home, PathFollower, World, _wrap, clamp, dreh_auf_kurs, dreh_w, fahrzeit_s
from .mathe import polar
from .config import ExploreCfg, HomeCfg, PetCfg, RobotCfg
from .innen import Akku, Innenleben
from .planner import path_length

log = logging.getLogger(__name__)

RETREAT_CAP_M, CREEP_CAP_M, ROLL_CAP_DEG = 0.30, 1.0, 12.0
NACHLAUF_S = 0.9                     # nach dem Stoppbefehl rollt er noch ~0,9 s x Tempo (PetCfg.pulse_margin)
CLOSE_M = 0.3                        # Fußpunkt unten abgeschnitten: so nah etwa
# Fußpunkt verdeckt (Kiste, Sofa): der Abstand wird riesig (Simulator 2026-11-13: 10,6 m statt 3,1 m). Darüber
# verworfen, bis dahin gekappt; die Richtung stimmt trotzdem, und gefahren wird ohnehin nur seek_m.
POS_DROP_M, POS_CAP_M = 5.0, 3.5
STALL_S, STALL_M = 2.5, 0.02         # Fahrt befohlen, aber so lange kaum Weg: Bahnprüfung bremst, Etappe aus
CALL_LEG_M = 3.0                     # Rufen ohne Grenze: je Ruf so weit auf die Stimme zu
CALL_FREE_M = 1.0                    # Rufziel: vorn weniger frei (Raumkarte) -> hinten, wenn dort mehr Platz
HOME_NEAR_M = 0.3                    # näher am Start: nach dem Suchen nicht mehr hinfahren
WIEDERSEHEN_S = 20.0                 # Wiedersehensfreude erst nach so langem Alleinsein
AKKU_HEIM_MAX_S = 180.0              # erschöpft zum Körbchen: länger nicht (Pose verloren), dann an Ort und Stelle
GAZE_BINS = 11                       # Blickrichtungen im Augen-Atlas (tools/eyes.py --gaze): 10 Stufen, ungerade für eine exakte Mitte
GAZE_CENTER = GAZE_BINS // 2
BLINK_S = 0.4                        # Lidschlag über alle Lidstellungen; bei 24 Hz Regelschritt je ~42 ms eine Stufe
GAZE_RANGE = math.radians(45.0)      # Peilung bei voll ausgelenktem Blick
GAZE_LIDS = 5                        # Lidstellungen im Atlas, 0 = offen .. 4 = zu
AUGEN_RATE, AUGEN_RATE_MUEDE = 15.0, 6.0   # Lidschläge je Minute im Augengenerator: wach, dösend/liegend
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
    wide: bool = False               # Box breiter als hoch: Person liegt oder sitzt am Boden (Nutzer 2026-11-22)
    who: str | None = None           # erkanntes Gesicht ([gesicht], Name am Track)


@dataclass
class Heard:
    key: float                       # Zeit der Meldung (eindeutig je Meldung)
    bearing: float                   # rad, links +; vorn/hinten nicht unterscheidbar (|bearing| <= 90°)
    db: float                        # über dem Ruhepegel
    age: float = 0.0                 # s seit dem Ende des ausgewerteten Fensters
    window_s: float = 1.0            # Länge des ausgewerteten Fensters


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
    heard: Heard | None = None       # letzte Schallmeldung (bleibt stehen, bis die nächste kommt)
    stimmen: tuple | None = None     # (bestätigte Stimmproben NN, neu bestätigte NN) aus stimme.Stimmen
    # Planer über die Raumkarte (Pet.step, nur mit Raumkarte): nächster Punkt auf dem Weg zum Fahrziel in
    # eigenen Koordinaten; None = ohne Planer geradeaus. nav_pending: Weg wird noch gerechnet, nav_fail: keiner.
    nav: tuple | None = None
    nav_pending: bool = False
    nav_fail: bool = False
    # Startbild am Startpunkt aufgenommen (World.anker): nur dann fährt er zum Start (Nutzer 2026-11-22)
    anker: bool = False
    # Raumkarte mit bestätigter Pose: Rufsuche über die Öffnungen der Karte (Pet wählt sie, Planer-Thread)
    karte: bool = False
    # Abstand zum Startpunkt der Raumkarte (m; gesetzt beim Aufspringen/Absetzen), None = keine Karte
    heim_m: float | None = None
    # Geplanter Weg (alle Aufgaben seit 2026-11-23, vorher nur Rufsuche): Pet fährt ihn mit dem Wegfolger von home
    # (Engstellen, Anschub, Neuplanen); das Gehirn prüft nur Ankunft, Budget, Zeit, Radius und ob Pet festgefahren
    # meldet (nav_stuck). Geradeaus-Bahnprüfung und Stillstand zählen dann nicht: Am Rider verwarf das 2026-11-22
    # den Weg durch die Tür nach 3 s.
    nav_extern: bool = False
    nav_stuck: bool = False
    stunde: float | None = None      # Uhrzeit (h, Ortszeit) fürs Innenleben, nur am Rider; None = unbekannt


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
        # Augengenerator im Display-Daemon (xgonav/augen.py): (Blick x, Blick y je -1..1, Grundlid 0..1,
        # Abstand m, Lidschläge/min); stufenlos, Lidschlag und Sakkaden macht der Daemon selbst
        self.augen_ziel = (0.0, 0.0, 0.0, 0.0, AUGEN_RATE)
        self.roll_deg = 0.0
        self.rolled = False                  # Roll wurde benutzt: beim Verlassen neu ausrichten
        self.det_hz = cfg.active_hz
        self.want_set_down = self.want_stand_up = self.want_idle = False
        self.want_ruhe: str | None = None    # Akku: absetzen und Ruhemodus (App._pet_ruhe), Grund
        # Innenleben (xgonav/innen.py): Akku in Stufen, Triebe als Faktoren auf Zeiten, Tempo, Gesten, Augen, Höhe
        self.akku = Akku(cfg)
        self.innen = Innenleben(cfg)
        self._akku_stufe: str | None = None  # Stufe im letzten Schritt (Reaktionen nur beim Wechsel)
        self._akku_heim = False              # erschöpft: Heimfahrt läuft (nichts lenkt sie ab)
        self._akku_heim_versucht = False
        self._akku_heim_t0 = 0.0
        self._akku_det = False               # Detektor wegen des Akkus auf doze_hz
        self._ruhe_at: float | None = None   # dann want_ruhe setzen (erst noch gähnen)
        self._ruhe_grund = ""
        self.neugier = 0.0                   # Bereitschaft zu streifen (info, Augen)
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
        self._seek_ear = False               # sucht nach einem Geräusch ohne Richtung: lauscht beim Umschauen
        self._ear_again = False              # Horchen kam aus diesem Umschauen: Rufen ohne Sperre, kein zweites Mal
        self._goal_flip: tuple[float, float] | None = None   # Stimme vorn oder hinten: Spiegelpunkt fürs Ziel
        self._faints: collections.deque = collections.deque()
        self._down_sent = False
        self._nap_t0 = 0.0
        self._nap_ref = None                 # (roll, pitch, Firmware-Zustand) in Ruhe
        self._nap_moved = 0.0
        self._peek_t = 0.0                   # Augen gehen seit dann auf
        self._clear_t: float | None = None   # niemand im Bild seit dann
        self._next_blink = 0.0
        self._fix: tuple | None = None       # (Ziel-Id, seit) solange der Blick auf demselben Ziel ruht
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
        # Hören
        self.n_heard = self.n_talk = self.n_called = self.n_come = 0
        self.come_m = _limit(cfg.come_m)
        self.follow_m = _limit(cfg.follow_m)
        self._seek_person = False            # Suche gilt einer verschwundenen Person (nicht dem Horchen)
        self._seek_gerufen = False           # Suche nach einem Ruf/Geräusch: gefunden -> hinfahren
        self._heim_result: str | None = None # Heimfahrt (Pet fährt sie wie home): Ergebnis, sobald fertig
        # Rufsuche (Nutzer 2026-11-22): Rufe aus einem anderen Raum kommen am Rider mit ~0° an, egal wohin er schaut
        # (Hall, Mikros 4 mm auseinander). Statt der Richtung: Öffnungen der Raumkarte anfahren, dort umschauen und
        # horchen, lauter = weiter so. self._ruf: {"pegel": Pegel beim Ruf, "best": lautester im Stand, "n": Öffnungen,
        # "dbs": Pegel beim Umschauen hier, "lauter": letzte Öffnung war lauter}; None = keine Rufsuche
        self._ruf: dict | None = None
        self._oeff_result: tuple | None = None   # Pet: (Ziel im Gehirn-KS oder None) für die Phase "oeffnung"
        self._db_hist: collections.deque = collections.deque(maxlen=12)   # (t, Pegel dB) im Stand gehört
        self.n_oeffnungen = 0
        self.n_follow = 0
        # Folgen (Nutzer 2026-11-22): Aufgabe "folgen", Phase "extern"; gefahren wird mit behaviors.Follow (Pet),
        # das Gehirn prüft nur, ob es weitergehen darf. Ergebnis von Pet (folgen_ende), sobald follow aufgibt.
        self._folgen_ende: str | None = None
        self._wide_babble_t = -1e9           # zuletzt einer liegenden/sitzenden Person zugebrabbelt
        self._come_lost_t: float | None = None
        self._ev: tuple[Heard, bool] | None = None    # neue Meldung dieses Schritts, Richtung brauchbar?
        self._ear_key: float | None = None
        self._ear_last_t = -1e9              # letzte Meldung (laut genug)
        self._ear_gap = 1e9                  # Stille vor der letzten Meldung
        self._ear_dirs: collections.deque = collections.deque()   # (t, Peilung) brauchbarer Meldungen
        self._ear_turns = 0
        self._ear_front_n = 0
        self._ear_flipped = False
        self._ear_fast = False
        self._ear_gaze: tuple[float, float] | None = None   # (bis wann, Peilung): Augen wandern zum Geräusch
        self._talk_n = 0                     # Meldungen in Folge (eine Äußerung)
        self._voice_t = -1e9                 # zuletzt eine Stimme von vorn gehört oder jemanden nach dem Horchen gesehen
        self._flinch_until = 0.0
        # Nachplappern (stimme.Stimmen): gelernte Proben, fällige Wiederholungen neu bestätigter, Abspielzähler
        self.hoert_zu = False                # App schneidet Stimmproben mit, solange das gilt
        self._parrot_person_t = -1e9         # zuletzt eine Person im Bild (auch liegend)
        self._who_t: dict[str, float] = {}   # Name -> zuletzt erkannt (Begrüßung)
        self.gruss_fuer = None               # Name -> {emotion, sound, augen} je Person (gesicht.Gesichter.gruss), von App
        self.augen: str | None = None        # Augenfarbe der zuletzt erkannten Person (tools/eyes.py PALETTE)
        self._augen_t = -1e9
        self.n_parrot = 0
        self._parrot_ids: tuple = ()
        self._parrot_due: list[tuple[float, str]] = []
        self._parrot_n: dict[str, int] = {}

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

    # ---- Innenleben (xgonav/innen.py) ---------------------------------------
    def _innen(self, s: Sense) -> None:
        """Akku glätten, Triebe fortschreiben; ein paar Floats je Regelschritt."""
        if self._ruhe_at is not None and s.t >= self._ruhe_at:
            self.want_ruhe, self._ruhe_at = self._ruhe_grund, None
        self.akku.update(s.t, s.battery)
        tg = self.target                                     # aus dem letzten Schritt
        person_da = any(x.cls == "person" for x in s.seen)
        person_nah = person_da and (self.zone in ("mitte", "nah", "knien")
                                    or (self._talk_n >= 2 and tg is not None and tg.cls == "person"))
        dyaw = _wrap(s.yaw - self.yaw) if s.balancing else 0.0
        self.innen.update(s.dt, self.akku.stufe, s.ds if s.balancing else 0.0, dyaw, person_da, person_nah,
                          self.fright, self.trust, s.stunde)
        alone = s.t - self._seen_t if self._seen_t is not None else 0.0
        self.neugier = clamp(alone / max(1e-3, self.cfg.roam_after_s * self.innen.f_streifen()), 0.0, 1.0)

    @property
    def dudeln_ok(self) -> bool:
        return self.innen.dudeln_ok

    def _akku_tick(self, s: Sense) -> None:
        """Akku-Stufen (Nutzer 2026-11-28): hungrig: Detektor langsamer (betteln/knurren in _wach); erschöpft: mit
        Startbild heim, dann absetzen + Ruhemodus; leer: sofort. Jede Reaktion einmal je Stufe."""
        c, t = self.cfg, s.t
        stufe = self.akku.stufe
        neu = stufe != self._akku_stufe
        if neu and self._akku_stufe is not None:
            log.info("pet: Akku %s (%.0f %%)", stufe, self.akku.wert if self.akku.wert is not None else -1)
        self._akku_stufe = stufe
        if self._akku_heim and t - self._akku_heim_t0 > AKKU_HEIM_MAX_S:
            log.warning("pet: Heimfahrt zum Körbchen dauert zu lange")
            self._task_reset(t)
        if self._ruhe_at is not None or self.want_ruhe is not None or not c.akku_ruhe:
            return
        if stufe == "leer":
            self._react(t, "sleep", "seufzen", urgent=True)
            self._ruhe(s, "Akku leer")
            return
        if stufe != "erschoepft":
            self._akku_heim = False
            return
        if self._akku_heim:
            if self.task == "heim":
                return                                       # unterwegs zum Körbchen
            self._akku_heim = False
            self._ruhe(s, "erschöpft, Heimfahrt vorbei")
            return
        self._react(t, "sad", "gaehnen", urgent=True)
        if self.state == "wach" and c.akku_heim and not self._akku_heim_versucht and s.balancing:
            self._akku_heim_versucht = True
            if self.task is not None:
                self._task_reset(t)
            if self._go_home(s, akku=True):
                log.info("pet: erschöpft, fahre zum Körbchen (Startpunkt)")
                self._akku_heim, self._akku_heim_t0 = True, t
                return
        self._ruhe(s, "erschöpft", nach_s=2.5)              # Gähnen noch sehen lassen

    def _akku_detektor(self) -> None:
        """Ab hungrig: Detektor ohne Ziel auf doze_hz (Rechenzeit ist Strom); mit Ziel wie sonst, damit Schreck und
        Folgen schnell bleiben. Wieder satt: zurück. Folgen (det_hz None) bleibt unberührt."""
        c = self.cfg
        if self.akku.stufe in ("hungrig", "erschoepft", "leer"):
            if self.det_hz is not None:
                self.det_hz = c.active_hz if self.target is not None else c.doze_hz
                self._akku_det = True
        elif self._akku_det:
            self._akku_det = False
            if self.det_hz is not None and not self._dozing:
                self.det_hz = c.active_hz

    def _ruhe(self, s: Sense, grund: str, nach_s: float = 0.0) -> None:
        """Still werden (Augen zu); nach nach_s App: absetzen und Ruhemodus (want_ruhe)."""
        log.warning("pet: %s -> absetzen, Ruhemodus", grund)
        self._set_state("aus", s.t)
        self._ruhe_grund, self._ruhe_at = grund, s.t + nach_s
        if nach_s <= 0.0:
            self.want_ruhe, self._ruhe_at = grund, None

    def _hunger(self, s: Sense, tg: Seen | None) -> None:
        """hungrig: bei einer Person betteln, allein knurrt der Magen."""
        c, t = self.cfg, s.t
        if tg is not None and tg.cls == "person" and self.zone in ("mitte", "nah", "knien"):
            if self._ready("betteln", t, c.bettel_repeat_s):
                log.info("pet: hungrig, bettelt")
                self._react(t, "grievance", "wimmern")
                if c.gestures and self.roll_mag > 0 and self.task is None:
                    self._gesture = ("kopf", t, t + c.tilt_s)
                    self.roll_deg = self.rng.choice((-1.0, 1.0)) * self.roll_mag
                    self.rolled = True
        elif tg is None and self._ready("knurren", t, c.knurr_repeat_s):
            self._react(t, None, "knurren")

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
        if self.augen is not None and s.t - self._augen_t > c.eye_hold_s:
            log.info("pet: Augen wieder Standard")
            self.augen = None
        self._innen(s)
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
        self._ev = self._hear_new(s)
        self._parrot_new(s)
        if self.state in ("wach", "liegt"):
            self._akku_tick(s)
        act = {"wach": self._wach, "schreck": self._schreck, "ab": self._ab, "nap": self._nap,
               "lugen": self._lugen, "warnen": self._warnen, "hoch": self._hoch, "settle": self._settle, "liegt": self._liegt,
               "aus": self._aus}[self.state](s)
        if act.v != 0.0 or act.w != 0.0:
            self._move_t = s.t
        if act.height_mm is None and s.balancing and self.state in ("wach", "schreck", "settle"):
            # müde etwas tiefer (Energie 1: genau base_h), nie unter duck_mm
            minus = self.innen.hoehe_minus()
            act.height_mm = (self.base_h if minus <= 0.0 or self.base_h is None
                             else max(min(self.duck_mm, self.base_h), self.base_h - minus))
        self._akku_detektor()
        act.target_id = self.target.id if self.target is not None else None
        act.status = "pet: %s%s%s, Vertrauen %.2f" % (self.state, " " + self.zone if self.zone else "",
                                                     " %s/%s" % (self.task, self.phase) if self.task else "", self.trust)
        self._update_gaze(s)
        # Mitschneiden wach und liegend (zum Ausprobieren ohne Balancieren); die Mikrofone taugen liegend für den
        # Pegel, nur die Richtung nicht
        if any(x.cls == "person" for x in s.seen):
            self._parrot_person_t = s.t
        self.hoert_zu = bool(c.parrot and self.state in ("wach", "liegt")
                             and s.t - self._parrot_person_t <= c.parrot_listen_s)
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
        if tg is None and self._dozing and self._ev is not None and self._talk_n >= 2:
            log.info("pet: Stimmen wecken ihn")
            self._dozing = False
            self.det_hz = c.active_hz
            self._seen_t = t                         # Stimmen zählen als Gesellschaft
            self._react(t, "surprised", "horch")
        if tg is not None:
            allein = t - self._person_t                      # seit der letzten Person (Tiere zählen nicht)
            if self._dozing:
                self._dozing = False
                self.det_hz = c.active_hz
                self._react(t, "surprised", "gefunden")
            if (self.task is not None and self.task not in ("kommen", "folgen")
                    and not (self.task == "heim" and self._akku_heim)):   # erschöpft heim: nichts lenkt ab
                if self.task == "horchen":
                    self._voice_t = t                # er hat hingehört und jemanden gefunden: redet mit
                self._task_found(s)
            if (tg.cls == "person" and tg.who is None and allein >= WIEDERSEHEN_S
                    and self.innen.gesellig >= c.gesellig_freude and self._ready("wiedersehen", t, 60.0)):
                # Wiedersehensfreude: er hat sie vermisst (Geselligkeit, xgonav/innen.py). like bleibt den bekannten
                # Gesichtern vorbehalten (_greet), deshalb cute
                log.info("pet: Wiedersehen nach %.0f s (Sehnsucht %.2f)", allein, self.innen.gesellig)
                self.innen.wiedersehen()
                self._react(t, "cute", "traellern")
                if c.gestures and self.task is None:
                    self._gesture = ("wackeln", t, t + c.wiggle_s)
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
        self._greet(s)
        if tg is not None and tg.cls == "person":
            self._person(s, tg, act)
        self._person_presence(s, person)
        if self.akku.stufe == "hungrig":
            self._hunger(s, tg)
        self._nudge_check(s)
        self._hear_wach(s, tg)
        self._follow_check(s, tg)
        if self.task == "kommen":
            self._come_step(s, tg)
        if t < self._flinch_until and c.gestures:
            act.height_mm = self.duck_mm
        if self.task is not None:
            self._task_step(s, act)
            return act
        self._apply_gesture(s, act, tg)
        return act

    def _greet(self, s: Sense) -> bool:
        """Erkanntes Gesicht ([gesicht]): begrüßen und sofort voll vertrauen; erneut erst nach greet_repeat_s ohne sie.
        Emotion, Ton und Augenfarbe je Person (Web, gruss.json), sonst greet_emotion/greet_sound/Türkis; "-" = nichts.
        Die Augenfarbe gilt, solange die Person erkannt wird, und noch eye_hold_s danach (update). Wach und liegend.
        -> True, wenn in diesem Schritt jemand begrüßt wurde."""
        c, t = self.cfg, s.t
        gruss = False
        for x in s.seen:
            if not x.who:
                continue
            g = self.gruss_fuer(x.who) if self.gruss_fuer is not None else {}
            if g.get("augen"):
                if self.augen != g["augen"]:
                    log.info("pet: Augen %s (%s)", g["augen"], x.who)
                self.augen, self._augen_t = g["augen"], t      # vor der Begrüßung: sie läuft schon in der Farbe
            last = self._who_t.get(x.who)
            self._who_t[x.who] = t
            if last is None or t - last >= c.greet_repeat_s:
                emo = g.get("emotion") or c.greet_emotion
                ton = g.get("sound") or c.greet_sound
                log.info("pet: %s erkannt – Begrüßung (%s, %s)", x.who, emo or "-", ton or "-")
                self._react(t, emo if emo and emo != "-" else None, ton if ton and ton != "-" else None)
                self.trust = 1.0
                gruss = True
        return gruss

    # ---- Nachplappern (stimme.Stimmen) --------------------------------------
    def _parrot_new(self, s: Sense) -> None:
        """Neu bestätigte Proben einmal nachplappern (nach parrot_delay_s, dringend: er hat es eben gelernt)."""
        c, t = self.cfg, s.t
        if s.stimmen is not None:
            ids, neu = s.stimmen
            self._parrot_ids = tuple(ids)
            for nn in neu:
                self._parrot_due.append((t + c.parrot_delay_s, nn))
        if not self._parrot_due:
            return
        due = [x for x in self._parrot_due if x[0] <= t]
        self._parrot_due = [x for x in self._parrot_due if x[0] > t]
        for _, nn in due:
            if c.parrot and self.state not in ("aus", "nap"):
                self._play_parrot(t, nn, urgent=True)

    def _play_parrot(self, t: float, nn: str, urgent: bool = False) -> None:
        log.info("pet: plappere Probe %s nach", nn)
        self.n_parrot += 1
        self._parrot_n[nn] = self._parrot_n.get(nn, 0) + 1
        self._cool["nachplappern"] = t + self.cfg.parrot_repeat_s
        self._react(t, None, "stimme:" + nn, urgent=urgent)

    def _babble_sound(self, t: float) -> str:
        """Beim Mitreden: mit parrot_p eine gelernte Probe (die seltener gespielte), sonst brabbeln."""
        c, ids = self.cfg, self._parrot_ids
        if (not c.parrot or not ids or t < self._cool.get("nachplappern", -1e9)
                or self.rng.random() >= c.parrot_p):
            return "brabbeln"
        wenig = min(self._parrot_n.get(nn, 0) for nn in ids)
        nn = self.rng.choice([x for x in ids if self._parrot_n.get(x, 0) == wenig])
        self.n_parrot += 1
        self._parrot_n[nn] = wenig + 1
        self._cool["nachplappern"] = t + c.parrot_repeat_s
        log.info("pet: plappere Probe %s", nn)
        return "stimme:" + nn

    # ---- Hören --------------------------------------------------------------
    def _hear_new(self, s: Sense) -> tuple[Heard, bool] | None:
        """Neue Schallmeldung dieses Schritts oder None. Zweiter Wert: Richtung brauchbar (balancierend und das
        ausgewertete Fenster begann nach der letzten eigenen Bewegung)."""
        c, h = self.cfg, s.heard
        if not c.hear or h is None or h.key == self._ear_key:
            return None
        self._ear_key = h.key
        if h.age > 1.5 or h.db < c.hear_min_db:
            return None
        self._ear_gap = s.t - self._ear_last_t
        self._ear_last_t = s.t
        self._talk_n = self._talk_n + 1 if self._ear_gap <= c.talk_pause_s + 1.0 else 1
        self.n_heard += 1
        ok = s.balancing and s.t - h.age - h.window_s - 0.2 > self._move_t
        if ok:
            self._db_hist.append((s.t, h.db))
        return h, ok

    def _hear_wach(self, s: Sense, tg: Seen | None) -> None:
        c, t = self.cfg, s.t
        if not c.hear:
            return
        # Mitreden: jemand ist da (zu sehen oder eben von vorn gehört) und macht eine Pause
        da = (tg is not None and tg.cls == "person") or t - self._voice_t < 10.0
        still = t - self._ear_last_t
        if (c.talk and da and self.task is None and self._gesture is None and self._talk_n >= c.talk_min_events
                and c.talk_pause_s <= still < c.talk_pause_s + 2.0 and self._ready("mitreden", t, c.talk_repeat_s)):
            self._talk_n = 0
            self.n_talk += 1
            self._voice_t = t
            self.trust = clamp(self.trust + 0.01, 0.0, 1.0)
            self._react(t, None, self._babble_sound(t))
            if c.gestures and self.roll_mag > 0:
                self._gesture = ("kopf", t, t + c.tilt_s)
                self.roll_deg = self.rng.choice((-1.0, 1.0)) * self.roll_mag
                self.rolled = True
                self._blind_until = max(self._blind_until, t + c.tilt_s + 0.5)
        # Komm her: Er sieht die Person, sie ist nicht nah und spricht
        if (self._ev is not None and tg is not None and tg.cls == "person" and self.task is None and s.balancing
                and tg.dist is not None and tg.dist > c.come_stop_m + 0.3 and self._talk_n >= c.come_events
                and t >= self._blind_until and self._may_drive(s, self.come_m)
                and self._ready("kommen", t, c.come_repeat_s)):
            self._come_start(s, tg, "Person in %.1f m spricht" % tg.dist)
        if self._ev is None or tg is not None or self.task in ("suchen", "horchen", "kommen", "folgen", "heim") or not s.balancing:
            return
        h, ok = self._ev
        front = math.radians(c.hear_front_deg)
        # Schreck: nach Stille plötzlich laut, niemand zu sehen
        if (h.db >= c.hear_startle_db and self._ear_gap >= c.hear_quiet_s and ok
                and self._ready("hoerschreck", t, 20.0)):
            log.info("pet: Schreck durch Geräusch (+%.0f dB, %.0f°)", h.db, math.degrees(h.bearing))
            self._react(t, "surprised", "quiek", urgent=True)
            self._flinch_until = t + 0.6
            self._blind_until = max(self._blind_until, t + 2.0)       # Höhe ändert die Bodengeometrie
            self._ear_dirs.clear()
            self._hear_start(s, h.bearing, fast=True)
            return
        frei = self._cool.get("horchen", -1e9) <= t
        if not ok:
            # Er war selbst in Bewegung (Streifen dreht und fährt fast durchgehend): stutzt, bleibt stehen und
            # lauscht; die nächste Meldung im Stand gibt die Richtung
            if self.task == "streifen" and frei:
                if self._ready("horch", t, c.hear_repeat_s):
                    self._react(t, None, "horch")
                self._ear_dirs.clear()
                self._hear_start(s, 0.0, listen=True)
            return
        while self._ear_dirs and self._ear_dirs[0][0] < t - 4.0:
            self._ear_dirs.popleft()
        self._ear_dirs.append((t, h.bearing))
        self._ear_gaze = (t + 2.5, h.bearing)                         # die Augen wandern zuerst
        if len(self._ear_dirs) < 2 and self._ready("horch", t, c.hear_repeat_s):
            self._react(t, None, "horch")
        if not frei and abs(h.bearing) < math.radians(c.hear_side_deg):
            return                                                    # Sperrzeit; ganz klar von der Seite gilt sie nicht
        if abs(h.bearing) >= front:                                   # klar von der Seite: gleich hindrehen
            self._ear_dirs.clear()
            self._hear_start(s, h.bearing)
        elif c.hear_seek or (len(self._ear_dirs) >= 2 and abs(self._ear_dirs[-2][1]) < front):
            self._ear_dirs.clear()                                    # zweimal von vorn: lauschen, evtl. Rufen
            self._hear_start(s, h.bearing)

    # ---- Folgen ---------------------------------------------------------------
    def _follow_check(self, s: Sense, tg: Seen | None) -> None:
        """Sieht er eine Person (nicht liegend/sitzend), folgt er ihr mit der Routine des Modus follow (Pet fährt
        behaviors.Follow: Abstand, Tempo, Rückwärts, Höhen, Suche an der letzten Stelle). Nutzer 2026-11-22: ohne
        Vertrauensschwelle, sobald er sie sieht."""
        c, t = self.cfg, s.t
        if tg is None or tg.cls != "person" or tg.wide or self.task is not None:
            return
        if not (c.follow and s.balancing and t >= self._blind_until and self._may_drive(s, self.follow_m)
                and self._ready("folgen", t, c.follow_repeat_s)):
            return
        log.info("pet: folge wie follow (Person %s, Vertrauen %.2f)",
                 "in %.1f m" % tg.dist if tg.dist is not None else "ohne Abstand", self.trust)
        self.n_follow += 1
        self.task = "folgen"
        self._task_left = self.follow_m
        self._looks, self._fails = [], 0
        self._folgen_ende = None
        self.det_hz = None                                           # Detektor wie im Modus follow
        self._phase_set("folgen", t)

    def folgen_ende(self, grund: str) -> None:
        """Pet: follow hat aufgegeben (Person nicht wiedergefunden). Das Gehirn beendet die Aufgabe im nächsten Schritt."""
        self._folgen_ende = grund

    def _follow_step(self, s: Sense, act: Act) -> None:
        """Gefahren wird von Pet (behaviors.Follow); hier nur, ob es weitergehen darf."""
        t = s.t
        tg = self.target
        self._blind_until = max(self._blind_until, t + 1.0)          # eigene Fahrt ist kein Schreck
        if self._folgen_ende is not None:
            grund, self._folgen_ende = self._folgen_ende, None
            self._gone_done, self.zone = True, None
            self._react(t, "sad", "verloren")
            self._task_end(s, grund)
            if self.cfg.seek_home and self.cfg.lost_home:      # wie nach der Suche einer verschwundenen Person
                self._go_home(s)
            return
        if tg is not None and tg.cls != "person":
            self._task_end(s, "Tier im Bild")
            return
        if tg is not None and tg.wide:
            self._task_end(s, "sitzt oder liegt")
            return
        why = self._no_drive(s, self._task_left)
        if why is not None:
            self._task_end(s, why)

    def _come_start(self, s: Sense, tg: Seen, why: str) -> None:
        log.info("pet: komme (%s)", why)
        self.n_come += 1
        self.task = "kommen"
        self._task_left = self.come_m
        self._looks, self._fails, self._come_lost_t = [], 0, None
        self._goal = (self.x, self.y)
        self._come_goal(tg)
        self._react(s.t, "cute", "traellern")         # like (Herzaugen) nur für bekannte Gesichter (_greet)
        self._phase_set("fahren", s.t)
        self._drive_t_max = s.t + 30.0                       # das Ziel wandert mit ihr; Hänger fangen STALL_S bzw. nav_stuck ab

    def _come_goal(self, tg: Seen) -> bool:
        """Ziel come_stop_m vor der Person. False: schon nah genug."""
        d = min(tg.dist, POS_CAP_M) if tg.dist > 0 else CLOSE_M
        go = d - self.cfg.come_stop_m
        if go <= 0.0:
            return False
        a = self.yaw + tg.bearing
        self._goal = polar(self.x, self.y, go, a)
        return True

    def _come_step(self, s: Sense, tg: Seen | None) -> None:
        """Ziel der Person nachführen. Verschwindet sie, endet die Aufgabe (dann greift das Suchen)."""
        t = s.t
        self._blind_until = max(self._blind_until, t + 1.0)          # eigene Fahrt ist kein Schreck
        if tg is not None and tg.cls == "person" and tg.dist is not None:
            self._come_lost_t = None
            if not self._come_goal(tg):
                self._task_end(s, "angekommen")
        elif tg is not None and tg.cls != "person":
            self._task_end(s, "Tier im Bild")
        else:
            if self._come_lost_t is None:
                self._come_lost_t = t
            elif t - self._come_lost_t > 1.0:
                self._task_end(s, "Person nicht mehr zu sehen")

    def _hear_start(self, s: Sense, bearing: float, fast: bool = False, listen: bool = False) -> None:
        """listen: Richtung noch unbekannt, nur stehen bleiben und lauschen."""
        if self.task is not None:
            self._task_reset(s.t)                                     # Streifen ist vorbei
        if listen:
            log.info("pet: horche, bleibe stehen")
        else:
            log.info("pet: horche, Geräusch bei %.0f°", math.degrees(bearing))
        self._ev = None                                               # diese Meldung ist verbraucht
        self.task = "horchen"
        self._looks, self._fails = [], 0
        self._ear_turns, self._ear_front_n, self._ear_flipped, self._ear_fast = 0, 0, False, fast
        if listen:
            self._phase_set("lauschen", s.t)
        elif abs(bearing) >= math.radians(self.cfg.hear_front_deg):
            self._hear_turn(s, bearing)
        else:
            self._ear_front_n = 1
            self._phase_set("lauschen", s.t)

    def _hear_turn(self, s: Sense, ang: float) -> None:
        self._ear_turns += 1
        self._look_dir = 1.0 if ang >= 0 else -1.0
        self._phase_set("drehen", s.t)
        self._yaw_goal = self.yaw + ang

    def _hear_end(self, s: Sense, why: str) -> None:
        log.info("pet: horchen endet: %s", why)
        self._cool["horchen"] = s.t + self.cfg.hear_repeat_s
        self._task_reset(s.t)

    def _hear_drehen(self, s: Sense, act: Act) -> None:
        c, t = self.cfg, s.t
        err = _wrap(self._yaw_goal - self.yaw)
        w = dreh_auf_kurs(err, 6.0, t - self._phase_t, 8.0, c.turn_kp, c.turn_w if self._ear_fast else c.look_w,
                          self.rcfg) if c.gestures else None
        if w is not None:
            act.w = w
            return
        self._ear_fast = False
        self._phase_set("lauschen", t)

    def _hear_lauschen(self, s: Sense, act: Act) -> None:
        """Steht und wartet auf die nächste Meldung. Von vorn: richtig gedreht (oder die Quelle liegt genau
        hinten). Wieder von der Seite: Die Quelle lag hinten, also um den Rest herumdrehen."""
        c, t = self.cfg, s.t
        if self._ev is not None and self._ev[1]:
            b = self._ev[0].bearing
            self._phase_t = t
            if abs(b) < math.radians(c.hear_front_deg):
                self._ear_front_n += 1
                self._voice_t = t
                if self._ear_front_n >= c.call_events:
                    if self._call_ok(t):
                        self._hear_call(s)
                    else:
                        self._hear_end(s, "Stimme vorn, eben erst gesucht")
            elif self._ear_turns < c.hear_turns:
                ang = b
                if not self._ear_flipped and self._ear_turns > 0:
                    self._ear_flipped = True
                    ang = math.copysign(math.pi - abs(b), b)
                self._ear_front_n = 0
                self._hear_turn(s, ang)
            else:
                self._react(t, "doubt", None)
                if c.hear_seek:
                    self._hear_seek(s, "Richtung bleibt unklar", known=False)
                else:
                    self._hear_end(s, "Richtung bleibt unklar")
            return
        if t - self._phase_t > c.hear_listen_s:
            if self._ear_front_n == 0:
                self._react(t, "doubt", None)
            why = "still" if self._ear_front_n == 0 else "Stimme vorn, niemand zu sehen"
            if c.hear_seek:
                self._hear_seek(s, why, known=self._ear_front_n > 0 or self._ear_turns > 0)
            else:
                self._hear_end(s, why)

    def _hear_seek(self, s: Sense, why: str, known: bool) -> None:
        """Gehört, niemand zu sehen: suchen statt aufgeben. Richtung bekannt (er hat sich hingedreht oder die
        Stimme kam von vorn): dorthin fahren wie beim Rufen. Unbekannt (nur gestutzt, oder vorn/hinten unklar):
        umschauen und in den Pausen weiterhorchen; die nächste brauchbare Meldung führt dann dorthin."""
        t = s.t
        again = self._ear_again                                       # schon einmal rundum gelauscht
        if not self._call_ok(t):
            self._hear_end(s, why + ", eben erst gesucht")
            return
        if known:
            log.info("pet: horchen endet: %s, niemand zu sehen -> suche dort", why)
            self._hear_call(s)
            return
        if again:
            self._hear_end(s, why)
            return
        log.info("pet: horchen endet: %s, niemand zu sehen -> schaue mich um und lausche", why)
        self._cool["horchen"] = t + self.cfg.hear_repeat_s
        self._task_reset(t)
        self._task_start("suchen", s)
        self._seek_ear = True
        self._seek_gerufen = True

    def _call_ok(self, t: float) -> bool:
        """Rufen erlaubt? Nach dem Umschauen-und-Lauschen einmal ohne Sperre (die Sperre lief schon an)."""
        if self._ear_again:
            self._ear_again = False
            self._cool["rufen"] = t + self.cfg.call_repeat_s
            return True
        return self._ready("rufen", t, self.cfg.call_repeat_s)

    def _hear_call(self, s: Sense) -> None:
        """Rufen: Die Stimme kommt weiter von vorn (oder genau von hinten) und niemand ist zu sehen. Sucht in
        diese Richtung: fährt bis call_m, wenn er darf, und schaut sich dort um (findet auch den, der hinten steht)."""
        c = self.cfg
        self.n_called += 1
        self._cool["horchen"] = s.t + c.hear_repeat_s
        seite = self._ear_turns > 0                   # zum Geräusch gedreht: die Richtung ist klar (er schaut hin)
        self._task_reset(s.t)
        budget = min(self.seek_m, _limit(c.call_m))
        if s.karte and c.call_openings > 0 and budget > self._stop_m() + 0.05:
            self._ruf_start(s, budget, seite)
            return
        goal = None
        if budget > self._stop_m() + 0.05:
            go = min(budget, CALL_LEG_M) - self._stop_m()
            goal = polar(self.x, self.y, go, self.yaw)
        log.info("pet: gerufen, suche vorn (Budget %.1f m)", budget)
        self._react(s.t, "like", "horch")
        self._task_start("suchen", s, goal)
        self._seek_gerufen = True
        self._task_left = min(self._task_left, budget)
        if goal is not None:
            self._goal_flip = (self.x, self.y)        # vorn und hinten klingen gleich: findet der Planer vorn
                                                      # keinen Weg, gilt hinten

    # ---- Rufsuche über die Öffnungen der Raumkarte ------------------------------
    def _ruf_start(self, s: Sense, budget: float, seite: bool = False) -> None:
        """seite: Er hat sich zum Geräusch gedreht; Öffnungen in Blickrichtung zuerst (Pet merkt sie beim ersten Auftrag)."""
        t = s.t
        pegel = max((db for tt, db in self._db_hist if t - tt <= 6.0), default=0.0)
        log.info("pet: gerufen (%.0f dB), suche über die Öffnungen der Karte%s (Budget %.1f m)", pegel,
                 ", in Blickrichtung zuerst" if seite else "", budget)
        self._react(t, "like", "horch")
        self.task = "suchen"
        self.n_seek += 1
        self._task_left = budget
        self._looks, self._fails, self._goal_flip = [], 0, None
        self._ruf = {"pegel": pegel, "best": pegel, "n": 0, "dbs": [], "lauter": False, "seite": seite}
        self._oeff_result = None
        self._phase_set("oeffnung", t)

    def oeffnung(self, goal: tuple | None, weg_m: float | None = None) -> None:
        """Pet: nächste Öffnung (Ziel im Gehirn-KS) oder None = keine mehr erreichbar; weg_m: Länge des Wegs."""
        self._oeff_result = (goal, weg_m)

    def _ruf_oeffnung(self, s: Sense, act: Act) -> None:
        """Wartet auf die Öffnung vom Planer, dann hinfahren."""
        t = s.t
        if self._oeff_result is None:
            if t - self._phase_t > 8.0:
                self._task_end(s, "Planer antwortet nicht")
            return
        (goal, weg_m), self._oeff_result = self._oeff_result, None
        if goal is None:
            self._task_end(s, "keine Öffnung mehr")
            return
        if not self._may_drive(s, self._task_left):
            self._task_end(s, "darf nicht fahren")
            return
        self.n_oeffnungen += 1
        self._ruf["n"] += 1
        self._ruf["dbs"] = []
        self._goal = goal
        self._looks = []
        luft = math.hypot(goal[0] - self.x, goal[1] - self.y)
        log.info("pet: Rufsuche: Öffnung %d, %.1f m (Weg %.1f m)", self._ruf["n"], luft, weg_m or luft)
        self._phase_set("fahren", t)
        if weg_m is not None:                                # Zeit nach dem Weg, nicht nach der Luftlinie
            v = max(self.rcfg.v_of(self.cfg.drive_v * self.innen.f_tempo()) * self.rcfg.vx_limit, 1e-3)
            self._drive_t_max = max(self._drive_t_max, t + 8.0 + 2.5 * weg_m / v)

    def _ruf_umgeschaut(self, s: Sense) -> None:
        """An der Öffnung rundum geschaut und gehorcht: lauter -> von hier weiter, sonst die nächste Öffnung."""
        c, r = self.cfg, self._ruf
        lvl, vorher = max(r["dbs"], default=None), r["best"]
        r["lauter"] = lvl is not None and lvl >= vorher + c.call_louder_db
        if r["lauter"]:
            r["best"] = lvl
        log.info("pet: Rufsuche: an Öffnung %d %s (lautester Ruf %s, bisher %.0f dB)", r["n"],
                 "lauter" if r["lauter"] else "nicht lauter", "-" if lvl is None else "%.0f dB" % lvl, vorher)
        if r["n"] >= c.call_openings:
            self._task_end(s, "niemand gefunden")
            return
        self._looks, self._oeff_result = [], None
        self._phase_set("oeffnung", s.t)

    def _lonely(self, s: Sense) -> bool:
        """Niemand in Sicht: Langeweile, Dösen, nach timeout_s absetzen. True: nichts weiter zu tun."""
        c, t = self.cfg, s.t
        if self._akku_heim:
            return False                     # erschöpft auf dem Heimweg: weder dösen noch absetzen
        alone = t - self._seen_t
        fz, fs = self.innen.f_zeit(), self.innen.f_streifen()   # Energie 1: beide genau 1 (wie ohne Innenleben)
        if alone >= c.timeout_s * fz:
            log.info("pet: %.0f s niemand, setze ab (Akku)", alone)
            self.want_set_down = self.want_idle = True
            self._set_state("aus", t)
            self._react(t, "sleep", "seufzen")
            return True
        if alone >= c.doze_s * fz:
            if not self._dozing:
                self._dozing = True
                self.det_hz = c.doze_hz
                self.zone = None
                self._task_reset(t)
                self._react(t, "sleep", "seufzen")
            return True
        if (c.look and c.gestures and self.task is None and alone >= c.roam_after_s * fs
                and t >= self._roam_t + c.roam_repeat_s * fs):
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
            self._retreat_until = t + self._pulse_s(self.retreat_m, c.retreat_v)
            self.n_retreat += 1
        self._set_state("schreck", t)

    def _pulse_s(self, budget_m: float, v: float) -> float:
        """So lange darf v (Faktor) anliegen, damit der gesamte Weg samt Nachlauf im Budget bleibt."""
        return self.cfg.pulse_margin * budget_m / max(abs(self.rcfg.v_of(v)) * self.rcfg.vx_limit, 1e-3)

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
                    act.v = -self.rcfg.v_of(c.retreat_v)
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
            act.v = self.rcfg.v_of(c.creep_v)

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
        if self.zone in ("nah", "knien") or tg.wide:
            self.trust = clamp(self.trust + c.trust_gain * s.dt, 0.0, 1.0)
        if tg.wide:
            self._wide_babble(s)
        if self.zone == "zunah" and c.gestures:
            if self._duck_until < t:
                self._blind_until = max(self._blind_until, t + 1.5)  # Höhe ändert die Bodengeometrie
            self._duck_until = t + 0.5
            act.height_mm = self.duck_mm

    def _wide_babble(self, s: Sense) -> None:
        """Person liegt oder sitzt am Boden (Box breiter als hoch): nicht folgen, hinschauen (_apply_gesture), Vertrauen
        (in _person) und alle wide_babble_s brabbeln, dazu Kopf schief wie beim Mitreden (Nutzer 2026-11-22)."""
        c, t = self.cfg, s.t
        if self.task is not None or self._gesture is not None or t - self._wide_babble_t < c.wide_babble_s:
            return
        self._wide_babble_t = t
        self.n_talk += 1
        self._react(t, None, self._babble_sound(t))
        if c.gestures and self.roll_mag > 0:
            self._gesture = ("kopf", t, t + c.tilt_s)
            self.roll_deg = self.rng.choice((-1.0, 1.0)) * self.roll_mag
            self.rolled = True
            self._blind_until = max(self._blind_until, t + c.tilt_s + 0.5)

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
            self._react(t, "cute", "traellern")           # like nur für bekannte Gesichter (Nutzer 2026-11-28)
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
                self._last_pos = polar(self.x, self.y, d, a)
        elif not self._gone_done and t - self._person_t >= c.gone_s:
            self._gone_done = True
            self.zone = None
            if c.look and c.gestures and self.task is None:
                self._react(t, "doubt", None)
                self._look_dir = 1.0 if self._person_bearing >= 0 else -1.0   # zur Seite, wo sie verschwand
                self._task_start("suchen", s, self._seek_goal())
                self._seek_person = True
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
        return NACHLAUF_S * self.rcfg.v_of(self.cfg.drive_v) * self.rcfg.vx_limit

    def _no_drive(self, s: Sense, budget: float, akku_min: int | None = None) -> str | None:
        """Grund, nicht zu fahren, oder None. akku_min: Akkugrenze statt roam_battery (erschöpft heim)."""
        c = self.cfg
        if budget <= self._stop_m() + 0.05:
            return "Budget"
        if not (c.gestures and s.balancing):
            return "Gesten aus" if not c.gestures else "liegt"
        if not s.ds_measured:
            return "keine Räder"
        if not s.floor_ok or s.free_center is None:
            return "Bodenmodell"
        if s.battery is None or s.battery < (c.roam_battery if akku_min is None else akku_min):
            return "Akku"
        return None

    def _may_drive(self, s: Sense, budget: float) -> bool:
        return self._no_drive(s, budget) is None

    def _task_start(self, kind: str, s: Sense, goal: tuple[float, float] | None = None) -> None:
        self.task = kind
        self._task_left = {"suchen": self.seek_m, "heim": math.inf}.get(kind, self._roam_left)
        self._looks, self._fails = [], 0
        self._goal_flip = None
        if kind == "suchen":
            self.n_seek += 1
        if goal is not None and self._may_drive(s, self._task_left):
            log.info("pet: %s, fahre %s", kind, "zum Start" if kind == "heim" else "zur letzten Stelle")
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
            self._drive_t_max = t + fahrzeit_s(rest, self.cfg.drive_v * self.innen.f_tempo(), self.rcfg, 1e-3)

    def _task_reset(self, t: float) -> None:
        self._seek_ear = self._ear_again = False
        self._seek_person = False
        self._seek_gerufen = False
        self._heim_result = None
        self._ruf = self._oeff_result = None
        self._goal_flip = None
        if self.task is None:
            return
        self._phase_set("", t)
        if self.task == "folgen":
            self._cool["folgen"] = t + self.cfg.follow_repeat_s      # neu folgen erst nach der Pause, ab dem Ende
            if self.det_hz is None:
                self.det_hz = self.cfg.doze_hz if self._dozing else self.cfg.active_hz
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
            self._react(t, "cute", "gefunden")
        else:
            self._react(t, "surprised", "gefunden")
        log.info("pet: %s: gefunden", self.task)
        gerufen = self.task == "suchen" and (self._ruf is not None or self._seek_gerufen)
        self._task_reset(t)
        # Er wurde gerufen und hat jemanden gefunden: hinfahren wie bei „Komm her“, ohne zweiten Zuruf und ohne
        # Sperre (am Rider 2026-11-22 fand er den Rufer, blieb stehen und suchte, sobald er aus dem Bild war)
        tg = self.target
        if (gerufen and tg is not None and tg.cls == "person" and tg.dist is not None
                and tg.dist > self.cfg.come_stop_m + 0.3 and self._may_drive(s, self.come_m)):
            self._come_start(s, tg, "gerufen und gefunden, %.1f m" % tg.dist)

    def _task_end(self, s: Sense, why: str) -> None:
        log.info("pet: %s endet: %s", self.task, why)
        kind, person = self.task, self._seek_person
        if kind == "suchen":
            self._react(s.t, "sad", "verloren")
        elif kind == "kommen" and why == "angekommen":
            self._react(s.t, "cute", "kichern")
        self._task_reset(s.t)
        # Heimfahrt nur mit Startbild (Abgleich am Start gegen die Kartendrift); nach der Suche einer verschwundenen Person bleibt er dort und horcht (Nutzer 2026-11-22, lost_home)
        if kind == "suchen" and self.cfg.seek_home and (self.cfg.lost_home or not person):
            self._go_home(s)

    def _go_home(self, s: Sense, akku: bool = False) -> bool:
        """Alles abgesucht: zurück zum Start (wo er aufstand), wie explore am Ende. Der Weg kommt vom Planer.
        akku: erschöpft, fährt auch unter roam_battery (bis akku_leer). True: Heimfahrt läuft."""
        if s.heim_m is not None:
            d = s.heim_m                                     # Kartenstart (Aufspringen/Absetzen), dorthin fährt Pet
        elif self._home is None:
            return False
        else:
            d = math.hypot(self._home[0] - self.x, self._home[1] - self.y)
        if d <= max(HOME_NEAR_M, self._stop_m() + 0.1):
            return False
        why = ("kein Startbild" if not s.anker
               else self._no_drive(s, math.inf, self.cfg.akku_leer if akku else None))
        if why is not None:
            log.info("pet: bleibe hier, %.1f m vom Start (%s)", d, why)
            return False
        # Gefahren wird wie im Modus home (Pet.step: Planer, Vorpunkt, gerade Schlussfahrt, Abgleich am Startbild)
        log.info("pet: heim, fahre zum Start (%.1f m) wie home", d)
        self.task, self._task_left, self._heim_result = "heim", math.inf, None
        self._looks, self._fails, self._goal_flip = [], 0, None
        self._phase_set("extern", s.t)
        return True

    def heim_ende(self, why: str) -> None:
        """Pet: Heimfahrt fertig oder aufgegeben."""
        self._heim_result = why

    def _heim_extern(self, s: Sense, act: Act) -> None:
        """Heimfahrt läuft in Pet (wie home); hier nur Ende und Fahrverbot prüfen."""
        why = self._heim_result or self._no_drive(s, math.inf, self.cfg.akku_leer if self._akku_heim else None)
        if why is not None:
            self._task_end(s, why)

    def _task_step(self, s: Sense, act: Act) -> None:
        {"fahren": self._drive, "schauen": self._look, "drehen": self._hear_drehen,
         "lauschen": self._hear_lauschen, "folgen": self._follow_step, "extern": self._heim_extern,
         "oeffnung": self._ruf_oeffnung}[self.phase](s, act)

    def _drive(self, s: Sense, act: Act) -> None:
        """Zur Stelle self._goal: erst ausrichten, dann langsam fahren. Halten (dann umschauen), wenn der Rest
        im Nachlauf liegt, das Budget nicht mehr reicht, die Bahn blockiert ist oder er nicht vorankommt."""
        c, t = self.cfg, s.t
        dx, dy = self._goal[0] - self.x, self._goal[1] - self.y
        rest = math.hypot(dx, dy)
        if s.nav is not None:                                # Planer: auf den nächsten Punkt des Wegs lenken
            dx, dy = s.nav[0] - self.x, s.nav[1] - self.y
        err = _wrap(math.atan2(dy, dx) - self.yaw)
        stop = self._stop_m()
        why = None
        if rest > stop and s.nav_fail and self._goal_flip is not None:
            # Rufen: vorn kein Weg laut Planer, die Stimme kann genauso von hinten kommen
            ox, oy = self._goal_flip
            self._goal = (2.0 * ox - self._goal[0], 2.0 * oy - self._goal[1])
            self._goal_flip = None
            log.info("pet: %s: dort kein Weg, versuche die Gegenrichtung", self.task)
            self._phase_set("fahren", t)
            return
        if s.nav_extern:                                     # Pet fährt den geplanten Weg (PathFollower)
            why = ("angekommen" if rest <= stop else "festgefahren" if s.nav_stuck else
                   self._no_drive(s, self._task_left) or ("Zeit" if t > self._drive_t_max else None)
                   or ("Radius" if self._out_of_radius() else None))
            if why is not None:
                self._halt(s, why, rest)
            return                                           # gefahren wird in Pet
        if rest <= stop:
            why = "angekommen"
        elif s.nav_fail and self.task != "kommen":           # kommen: sie ist zu sehen, dann geradeaus
            why = "kein Weg"
        elif s.nav_pending:
            self._stall = None                               # Weg wird gerechnet: stehen, das zählt nicht
            return
        elif self._no_drive(s, self._task_left):
            why = self._no_drive(s, self._task_left)
        elif s.path_blocked:
            why = "Bahn blockiert"
        elif t > self._drive_t_max:
            why = "Zeit"
        elif self._out_of_radius():
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
            self._halt(s, why, rest)
            return
        act.v = self.rcfg.v_of(c.drive_v * self.innen.f_tempo())   # müde langsamer (Energie 1: drive_v)
        act.w = self._turn_to(err)

    def _out_of_radius(self) -> bool:
        return (self.task == "streifen" and self._home is not None
                and math.hypot(self.x - self._home[0], self.y - self._home[1]) > self.roam_radius + 0.1)

    def _halt(self, s: Sense, why: str, rest: float) -> None:
        """Fahrt zur Stelle endet: kommen und Ankunft daheim beenden die Aufgabe, sonst umschauen."""
        log.info("pet: %s: halte (%s, Rest %.2f m)", self.task, why, rest)
        if self.task == "kommen" or (self.task == "heim" and why == "angekommen"):
            self._task_end(s, "am Start" if self.task == "heim" else why)
            return
        if why != "angekommen":
            self._fails += 1
        self._phase_set("schauen", s.t)

    def _look(self, s: Sense, act: Act) -> None:
        """Rundumblick in look_steps Blicken: still stehen (Detektor, Bodenmodell), Freiraum merken, weiterdrehen."""
        c, t = self.cfg, s.t
        if self._yaw_goal is not None:
            w = dreh_auf_kurs(_wrap(self._yaw_goal - self.yaw), 6.0, t - self._phase_t, 8.0, c.turn_kp, c.look_w,
                              self.rcfg)
            if w is not None:
                act.w = w
                return
            self._yaw_goal, self._phase_t = None, t
            return
        if self._ruf is not None:
            if self._ev is not None and self._ev[1]:
                self._ruf["dbs"].append(self._ev[0].db)              # im Stand gehört: Pegel zählt, Richtung nicht
        elif self._seek_ear and self._ev is not None and self._ev[1]:
            h = self._ev[0]                                           # im Stand gehört: Richtung brauchbar
            log.info("pet: suchen: Stimme bei %.0f°", math.degrees(h.bearing))
            self._react(t, None, "horch")
            self._hear_start(s, h.bearing)
            self._ear_again = True
            return
        if t - self._phase_t < (max(c.look_pause_s, c.hear_listen_s) if self._seek_ear or self._ruf is not None
                                else c.look_pause_s):
            return
        self._looks.append((self.yaw, s.free_center))
        if len(self._looks) < max(c.look_steps, 1):
            self._yaw_goal, self._phase_t = self.yaw + self._look_dir * math.radians(c.look_step_deg), t
            return
        if self.task == "suchen" and self._ruf is not None:
            self._ruf_umgeschaut(s)
            return
        if self.task == "suchen":
            self._task_end(s, "niemand gefunden")
            return
        if self.task == "heim":                              # Halt auf dem Heimweg: umgeschaut, neu planen
            if self._fails >= 3 or not self._may_drive(s, self._task_left):
                self._task_end(s, "kein Weg zum Start")
            else:
                self._looks = []
                self._phase_set("fahren", t)
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
            ex, ey = polar(self.x, self.y, leg, a)
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

    def roll_zurueck(self) -> bool:
        """App beim Fahrbefehl: Kopfneigen beenden, Roll 0. True, wenn er schräg gestellt war (dann neu ausrichten)."""
        if not self.rolled:
            return False
        if self._gesture is not None and self._gesture[0] == "kopf":
            self._gesture = None
        self.roll_deg = 0.0
        self.rolled = False
        return True

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
                act.w = sign * self.rcfg.w_of((c.wiggle_w if g[0] == "wackeln" else c.slow_w) * self.innen.f_tempo())
            return
        if tg is not None:
            act.w = self._turn_to(tg.bearing)
        elif not self._gone_done and t - self._person_t >= 0.5:
            act.w = math.copysign(self.rcfg.w_of(c.slow_w), self._person_bearing)    # schaut hinterher

    def _turn_to(self, err: float) -> float:
        c = self.cfg
        return dreh_w(err, c.turn_kp, c.turn_w, self.rcfg, c.turn_deadband_deg)

    # ---- Ohnmacht ---------------------------------------------------------
    def _can_faint(self, s: Sense) -> bool:
        c, t = self.cfg, s.t
        while self._faints and self._faints[0] < t - c.faint_window_s:
            self._faints.popleft()
        return (c.faint and len(self._faints) < c.faint_max
                and s.battery is not None and s.battery > c.battery_low and self.akku.stufe not in ("erschoepft", "leer"))

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
                and self.akku.stufe not in ("erschoepft", "leer")
                and self._nap_ref is not None and s.t - self._ear_last_t >= self.cfg.clear_s)   # und niemand spricht

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
        """Nur Ausdruck. Bleibt liegen, bis ihn jemand aufstellt. Bekannte Gesichter begrüßt er auch liegend."""
        if s.balancing:
            self._blind_until = s.t + self.cfg.settle_s
            self._set_state("settle", s.t)
            return Act()
        gruss = self._greet(s)
        if self.target is not None:
            # Nur beim Auftauchen: Wer die ganze Zeit zu sehen ist, löst nichts mehr aus. Kurze Aussetzer des Detektors
            # zählen nicht als weg (gone_s), die Abklingzeit fängt das Kommen und Gehen an der Bildkante ab.
            # Eine Begrüßung ersetzt das allgemeine Aufmerken.
            if not gruss and s.t - self._lying_seen_t >= self.cfg.gone_s and self._ready("liegend", s.t, 10.0):
                self._react(s.t, "surprised", "gefunden")
            self._lying_seen_t = s.t
        elif self._ev is not None and self._talk_n >= 2 and self._ready("liegend_hoeren", s.t, 15.0):
            self._react(s.t, "doubt", "horch")         # liegend taugt die Richtung nicht: nur aufmerken
        return Act()

    def _aus(self, s: Sense) -> Act:
        return Act()

    # ---- Augen ------------------------------------------------------------
    def _update_gaze(self, s: Sense) -> None:
        t = s.t
        goal = self._gaze_goal
        xf = None                            # stufenlos für den Augengenerator (-1..1); None: aus goal
        if self.state == "lugen" and self.target is None:
            # schaut langsam nach links und rechts
            xf = 0.8 * math.sin(2 * math.pi * (t - self._peek_t) / 3.0)
            goal = GAZE_CENTER + int(round(GAZE_CENTER * xf))
        elif self.phase == "schauen" and self.target is None and self._yaw_goal is None:
            # beim Umschauen im Stehen: Blick wandert, beim Drehen schaut er in Drehrichtung
            xf = 0.8 * math.sin(2 * math.pi * (t - self._phase_t) / 2.0)
            goal = GAZE_CENTER + int(round(GAZE_CENTER * xf))
        elif self.task == "horchen" and self.target is None:
            # beim Hindrehen in Drehrichtung, beim Lauschen geradeaus
            xf = 0.8 * self._look_dir if self.phase == "drehen" else 0.0
            goal = GAZE_CENTER + int(round(GAZE_CENTER * xf))
        elif self.task is not None and self.target is None:
            xf = 0.6 * self._look_dir if self.phase == "schauen" else 0.0
            goal = GAZE_CENTER + int(round(GAZE_CENTER * xf))
        elif self.target is None and self._ear_gaze is not None and t < self._ear_gaze[0]:
            xf = clamp(self._ear_gaze[1] / GAZE_RANGE, -1.0, 1.0)
            goal = GAZE_CENTER + int(round(xf * GAZE_CENTER))
        elif self.target is not None:
            # Links vom Rider ist für den, der ihn anschaut, rechts im Bild
            xf = clamp(self.target.bearing / GAZE_RANGE, -1.0, 1.0)     # Generator: ohne Hysterese, er glättet selbst
            raw = GAZE_CENTER + xf * GAZE_CENTER
            if abs(raw - goal) > 0.6:                        # Hysterese, sonst zittert der Blick
                goal = int(round(raw))
        elif t >= self._next_glance:
            # neugierig, darf aber nicht streifen (Akku, Boden): Blicke wandern öfter
            self._next_glance = t + self.rng.uniform(3.0, 7.0) * (0.5 if self.neugier >= 1.0 else 1.0)
            goal = GAZE_CENTER + self.rng.choice((-3, -1, 0, 0, 1, 3))
        self._gaze_goal = goal
        if xf is None:
            xf = (goal - GAZE_CENTER) / GAZE_CENTER
        gx = self.gaze[0]
        gx += (goal > gx) - (goal < gx)                      # gleitet, springt nicht
        # Fixieren: dieselbe Person, Blick angekommen (±1 Stufe), seit eyes_fix_s -> Schlitze
        tg = self.target
        if tg is not None and tg.cls == "person" and abs(gx - goal) <= 1:
            if self._fix is None or self._fix[0] != tg.id:
                self._fix = (tg.id, t)
        else:
            self._fix = None
        fix = (self.cfg.eyes_fix_s > 0 and self._fix is not None and t - self._fix[1] >= self.cfg.eyes_fix_s)
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
            base = clamp(self.cfg.eyes_fix_lid, LID_OPEN, LID_SHUT) if fix else LID_OPEN
            base = max(base, int(round(self.innen.lid_muede() * LID_SHUT)))   # müde: schwerere Lider
            lid = base + int(round((LID_SHUT - base) * closing))
        self.gaze = (int(gx), int(lid))
        self.augen_ziel = self._augen_ziel(s, xf, fix)

    def _augen_ziel(self, s: Sense, xf: float, fix: bool) -> tuple:
        """Augengenerator: Blick x/y, Grundlid ohne Lidschlag (den macht der Daemon), Abstand fürs Schielen,
        Lidschlagrate. Lid wie der Atlas: Stufe / LID_SHUT."""
        tg = self.target
        yf, nah = 0.0, 0.0
        if tg is not None:
            if tg.dist is not None:
                nah = tg.dist if tg.dist > 0 else CLOSE_M          # 0: Fußpunkt abgeschnitten, sehr nah
            if tg.cls == "person" and not tg.wide:
                # Kopf oben aus dem Bild: hochschauen; sonst nah etwas, fern kaum
                yf = -0.6 if not tg.top_free else (-0.3 if 0 < nah < 1.5 else -0.1)
            elif tg.cls != "person":
                yf = 0.3 if 0 < nah < 1.0 else 0.15                  # Tier am Boden
        muede = self._dozing or self.state == "liegt"
        if self.state in ("nap", "ab", "aus"):
            lid = 1.0
        elif self.state == "lugen":
            x = clamp((s.t - self._peek_t) / max(self.cfg.eyes_open_s, 1e-3), 0.0, 1.0)
            lid = 1.0 if tg is not None else 1.0 - x
        elif muede:
            lid = LID_HALF / LID_SHUT
        else:
            lid = clamp(self.cfg.eyes_fix_lid, LID_OPEN, LID_SHUT) / LID_SHUT if fix else 0.0
        if lid < 1.0:
            lid = max(lid, self.innen.lid_muede())          # müde: Lider schwerer (Energie 1: nichts)
        rate = 0.0 if lid >= 1.0 else (AUGEN_RATE_MUEDE if muede else self.innen.rate())
        return (float(xf), float(yf), float(lid), float(nah), rate)

    def info(self) -> dict:
        return {"zustand": self.state, "zone": self.zone, "vertrauen": round(self.trust, 2),
                "akku": self.akku.info(), **self.innen.info(), "neugier": round(self.neugier, 2),
                "schreck": round(self.fright, 2), "ziel": self.target.cls if self.target else None,
                "schrecks": self.n_startle, "ruck": self.n_retreat, "ohnmacht": self.n_faint, "liegt_weil": self.why_down or None,
                "aufgabe": "%s/%s" % (self.task, self.phase) if self.task else None, "suchen": self.n_seek, "folgen": self.n_follow,
                "gefunden": self.n_found, "gefahren_m": round(self.task_m, 2),
                "gehoert": self.n_heard, "gerufen": self.n_called, "mitgeredet": self.n_talk, "gekommen": self.n_come,
                "nachgeplappert": self.n_parrot, "proben": len(self._parrot_ids)}


def _wide(tr) -> bool:
    """Box breiter als hoch: Person liegt oder sitzt am Boden (Nutzer 2026-11-22)."""
    x1, y1, x2, y2 = tr.box
    return x2 - x1 > y2 - y1


class Pet(Behavior):
    name = "pet"

    # Weg zum Rufziel höchstens so viel länger als die Luftlinie, sonst gilt die Richtung als zugestellt
    DETOUR_K, DETOUR_ADD_M = 1.5, 0.5

    def __init__(self, cfg, rcfg: RobotCfg, pcfg: PetCfg | None = None, seed: int | None = None,
                 hcfg: HomeCfg | None = None, sbcfg=None, ecfg: ExploreCfg | None = None):
        super().__init__(cfg, rcfg)
        self.sbcfg = sbcfg
        self.e = ecfg or ExploreCfg()
        # Rufsuche: Öffnungen der Raumkarte (frontier.next_goal im Planer-Thread), besuchte nicht noch einmal
        self._oeff_of = None                 # brain._ruf, zu dem _oeff_besucht gehört
        self._oeff_dir: float | None = None  # Rufsuche nach klarer Peilung: gehörte Richtung (rad, Karten-KS)
        self._oeff_besucht: list = []
        self._oeff_req = False               # laufender Auftrag ist eine Öffnungssuche
        self._oeff_asked = False
        self._nav_stucks = 0                 # festgefahren auf dem geplanten Weg (PathFollower.stuck), je Ziel
        self._w: World | None = None
        self.home_drive: Home | None = None  # Heimfahrt wie Modus home (Startbild-Abgleich), solange sie läuft
        self.pcfg = pcfg or PetCfg()
        self.brain = PetBrain(self.pcfg, rcfg, seed)
        self.want_set_down = False           # App._step
        self.want_stand_up = False           # App._pet_tick
        self.want_idle = False
        self.want_ruhe: str | None = None   # App._pet_tick: absetzen und Ruhemodus (Akku)
        self.gaze = self.brain.gaze          # lcd.py liest das aus seinem Thread (Tupel, atomar ersetzt)
        self.augen_ziel = self.brain.augen_ziel   # ebenso, für den Augengenerator
        self.augen: str | None = None        # Augenfarbe (brain.augen): lcd.py und App -> Emotions.farbe
        # Folgen: genau die Routine des Modus follow (Nutzer 2026-11-22), solange die Aufgabe folgen läuft
        self.follow_drive: Follow | None = None
        self._f_seek = self._f_found = 0     # Suchfahrten/Wiederfunde früherer Folgen (info)
        # Planer (App._plan_tick, nur mit Raumkarte): Fahrziele des Gehirns in Kartenkoordinaten
        self.h = hcfg or HomeCfg()
        # Jeder geplante Weg des Gehirns (kommen, streifen, suchen, Rufsuche, heim ohne Startbild) wird hiermit
        # gefahren, im Tempo von pet (Nutzer 2026-11-23: ein Fahrweg für alle)
        self.f = PathFollower(cfg, rcfg, self.h, v=self.pcfg.drive_v)
        self._pending = False
        self._nav_goal: tuple[float, float] | None = None
        self._nav_req = None
        self._nav_fail = False
        self._nav_clear = False              # Umgebung des Ziels freiräumen (Start: driftet)
        self._t_plan = -1e9
        self._home_of = None                 # brain._home, zu dem _home_g gehört
        self._home_g: tuple[float, float] | None = None
        self._ray_for = None                 # Rufziel, für das der Strahl schon geprüft ist

    @property
    def drives(self) -> bool:
        """Fährt gerade eine Aufgabe: Nur dann rechnet App.path_blocked für pet (eine Bahnprüfung je Regelschritt)."""
        return self.brain.phase == "fahren"           # Folgen: Bahnprüfung der App wie im Modus follow

    @property
    def _drive(self):
        """Eingebettetes Verhalten, das gerade fährt und plant (Heimfahrt oder Folgen), sonst None."""
        return self.home_drive if self.home_drive is not None else self.follow_drive

    @property
    def path(self):
        d = self._drive
        return d.path if d is not None else self.f.path

    @property
    def ziel_anzeige(self):
        """Erwartete Stelle der verlorenen Person beim Folgen (Raumkarte), sonst None."""
        return self.follow_drive.ziel_anzeige if self.follow_drive is not None else None

    @property
    def pending(self) -> bool:
        d = self._drive
        return d.pending if d is not None else self._pending

    @pending.setter
    def pending(self, v: bool) -> None:
        d = self._drive
        if d is not None:
            d.pending = v
        else:
            self._pending = v

    # App._startbild_tick / _startbild_messen: Messung am Start, solange die Heimfahrt läuft
    @property
    def want_messung(self) -> bool:
        return self.home_drive is not None and self.home_drive.want_messung

    @want_messung.setter
    def want_messung(self, v: bool) -> None:
        if self.home_drive is not None:
            self.home_drive.want_messung = v

    @property
    def messung(self):
        return self.home_drive.messung if self.home_drive is not None else None

    @messung.setter
    def messung(self, erg) -> None:
        if self.home_drive is not None:
            self.home_drive.messung = erg

    # --- Planung (von App bedient, wie home/explore) ---
    def want_plan(self, t: float) -> bool:
        if self._drive is not None:
            return self._drive.want_plan(t)
        b = self.brain
        if b.phase == "oeffnung" and b._ruf is not None:
            if self.pending or self._oeff_asked:
                return False
            self.pending = self._oeff_asked = True
            return True
        if self.pending or self._nav_goal is None or self._nav_fail:
            return False
        if self.f.stuck:
            self._nav_stucks += 1                        # nach zwei Neuplanungen gilt er als festgefahren
        if self.f.path is None or t - self._t_plan > self.h.replan_s or self.f.stuck:
            self.pending, self._t_plan, self.f.stuck = True, t, False
            return True
        return False

    def request_plan(self, planner, grid, w: World, obstacles) -> int:
        if self._drive is not None:
            return self._drive.request_plan(planner, grid, w, obstacles)
        if self.brain.phase == "oeffnung":
            from .frontier import next_goal
            self._oeff_req = True
            r = self.e.avoid_r_m
            pc = self.pcfg
            prefer = ((self._oeff_dir, pc.call_dir_s_per_deg)
                      if self._oeff_dir is not None and pc.call_dir_s_per_deg > 0 else None)
            return planner.request_fn(next_goal, grid, w.pose, self.h, self.e, obstacles,
                                      [(x, y, r) for x, y in self._oeff_besucht], None, None, True,
                                      prefer=prefer, min_m=max(0.0, pc.call_min_m))
        self._oeff_req = False
        if self.brain._goal_flip is not None and self._ray_for is not self._nav_goal:
            self._call_ray(grid, w)
        self._nav_req = self._nav_goal
        return planner.request(w.pose[:2], self._nav_goal, keep=self.f.path, obstacles=obstacles,
                               clear_goal=self._nav_clear)

    def set_path(self, path) -> None:
        if self._drive is not None:
            self._drive.set_path(path)
            return
        if self._oeff_req:
            self._oeff_req, self.pending, self._oeff_asked = False, False, False
            self._oeff_ergebnis(path)
            return
        self.pending = False
        if self._nav_req is not self._nav_goal:          # Ziel inzwischen ein anderes
            return
        b = self.brain
        if path is not None and b._goal_flip is not None and len(path) >= 2:
            luft = math.hypot(path[-1][0] - path[0][0], path[-1][1] - path[0][1])
            if path_length(path) > self.DETOUR_K * luft + self.DETOUR_ADD_M:
                log.info("pet: Weg zum Rufziel %.1f m statt %.1f m: zugestellt", path_length(path), luft)
                path = None
        if path is None:
            if self.f.path is None or b._goal_flip is not None:
                self.f.clear()
                self._nav_fail = True
            return                                       # sonst alten Weg behalten
        self.f.set_path(path)

    def _free_m(self, grid, x: float, y: float, a: float, L: float) -> float:
        """Freie Strecke (m) von (x, y) in Richtung a, höchstens L, mit inflate_m Abstand zur Wand der Raumkarte.
        Nur einmal je Rufziel, höchstens L/res Abfragen (3 m: 60)."""
        r, d = self.h.inflate_m, 0.0
        while d + grid.res <= L:
            e = d + grid.res
            if grid.wall_dist(*polar(x, y, e, a), r) < r:
                return d
            d = e
        return L

    def _call_ray(self, grid, w: World) -> None:
        """Rufziel vor dem Planen: Steht vorn nach kurzer Strecke eine Wand und ist hinten mehr Platz, gilt
        hinten (vorn und hinten klingen gleich). Das Ziel endet vor der Wand; sonst plant der Planer durch
        Unbekanntes um sie herum (Simulator 2026-11-19: Umweg 4,7 statt 2,6 m, erst nach 0,7 m erkannt)."""
        b = self.brain
        x, y, _ = w.pose
        gx, gy = self._nav_goal
        L = math.hypot(gx - x, gy - y)
        if L < 0.05:
            return
        a = math.atan2(gy - y, gx - x)
        vor = self._free_m(grid, x, y, a, L)
        if vor < min(L, CALL_FREE_M):
            hin = self._free_m(grid, x, y, a + math.pi, L)
            if hin > vor + 0.3:
                log.info("pet: vorn nach %.1f m Wand, hinten %.1f m frei: suche hinten", vor, hin)
                a, vor = a + math.pi, hin
                b._goal_flip = None
        if vor < L or b._goal_flip is None:
            g = polar(x, y, vor, a)
            b._goal = self._to_brain(w, g)
            self._nav_goal = g
        self._ray_for = self._nav_goal

    def _frame(self, w: World):
        """Gehirn -> Karte: Drehung und Lage aus den beiden Posen dieses Schritts."""
        b = self.brain
        gx, gy, gth = w.pose
        d = gth - b.yaw
        return b.x, b.y, gx, gy, math.cos(d), math.sin(d)

    def _to_grid(self, w: World, p):
        bx, by, gx, gy, c, s = self._frame(w)
        dx, dy = p[0] - bx, p[1] - by
        return gx + c * dx - s * dy, gy + s * dx + c * dy

    def _to_brain(self, w: World, p):
        bx, by, gx, gy, c, s = self._frame(w)
        dx, dy = p[0] - gx, p[1] - gy
        return bx + c * dx + s * dy, by - s * dx + c * dy

    def _nav(self, w: World) -> dict:
        """Wegpunkt für das Gehirn; ohne Raumkarte nichts (dann fährt es geradeaus wie bisher)."""
        b = self.brain
        if w.pose is None:
            return {}
        if b._home is not self._home_of:                 # Start in Kartenkoordinaten festhalten (Abgleich
            self._home_of = b._home                      # verschiebt die Karte, nicht das Gehirn)
            self._home_g = self._to_grid(w, b._home) if b._home is not None else None
        if b.phase == "oeffnung" and b._oeff_result is not None:
            return {}                                    # Öffnung eben gefunden, Weg schon übernommen
        if b.phase != "fahren" or b._goal is None:
            self._nav_goal, self._nav_fail = None, False
            self.f.clear()
            return {}
        heim = b.task == "heim" and self._home_g is not None
        g = self._home_g if heim else self._to_grid(w, b._goal)
        if self._nav_goal is None or math.hypot(g[0] - self._nav_goal[0], g[1] - self._nav_goal[1]) > 0.10:
            # Ziel freiräumen: Start (driftet) und Kommen (dort steht die Person, die Karte sieht oft Belegtes)
            self._nav_goal, self._nav_fail, self._nav_clear = g, False, heim or b.task == "kommen"
            self._nav_stucks = 0
            self._t_plan = -1e9
            self.f.clear()
        if self.f.path:
            return {"nav": self._to_brain(w, self.f.target(w.pose[0], w.pose[1]))}
        if self._nav_fail:
            return {"nav_fail": True}
        return {"nav_pending": True}

    def _pfad_faehrt(self, w: World) -> bool:
        """Das Gehirn fährt zu einer Stelle und der Planer hat einen Weg: Pet fährt ihn mit dem PathFollower."""
        return self.brain.phase == "fahren" and bool(self.f.path) and w.pose is not None

    def _oeff_ergebnis(self, res) -> None:
        """Antwort der Öffnungssuche (frontier.ExplorePlan oder None) ans Gehirn; Weg gleich übernehmen."""
        b, w = self.brain, self._w
        if b.phase != "oeffnung" or w is None or w.pose is None:
            return
        if res is None or getattr(res, "path", None) is None:
            b.oeffnung(None)
            return
        self._oeff_besucht.append(res.frontier)
        self._nav_stucks = 0
        g = tuple(res.goal)
        b.oeffnung(self._to_brain(w, g), path_length(res.path))
        # _nav() findet im nächsten Schritt dasselbe Ziel und behält den Weg
        self._nav_goal, self._nav_req, self._nav_fail, self._nav_clear = g, g, False, False
        self._ray_for = g
        self._t_plan = w.t
        self.f.set_path(res.path)

    def _folgen(self, w: World) -> Command | None:
        """Aufgabe folgen des Gehirns mit behaviors.Follow fahren, genau wie im Modus follow: Abstand
        behavior.follow_dist_m, Tempo, Rückwärts bis backup_v, Höhen, verloren zur letzten Stelle und umschauen
        (Nutzer 2026-11-22). Liegende/sitzende Personen sieht follow nicht (die übernimmt das Gehirn). Meldet
        follow „warte“ (oder sucht ohne Karte länger als search_timeout_s), endet die Aufgabe."""
        b = self.brain
        if b.task != "folgen":
            if self.follow_drive is not None:
                self._f_seek += self.follow_drive.n_seek
                self._f_found += self.follow_drive.n_found
            self.follow_drive = None
            return None
        if self.follow_drive is None:
            self.follow_drive = Follow(self.cfg, self.rcfg, "person", hcfg=self.h, seek=True)
            self.f.clear()
        fd = self.follow_drive
        tracks = [t for t in w.tracks if not (t.cls == "person" and _wide(t))]
        c = fd.step(replace(w, tracks=tracks) if len(tracks) != len(w.tracks) else w)
        if fd.phase == "warten" or (fd.phase == "" and fd.search_start is not None
                                    and w.t - fd.search_start > self.cfg.search_timeout_s):
            b.folgen_ende("nicht wiedergefunden")
        return c

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
                            wm > 0 and x1 > 0.03 * w.W and x2 < 0.97 * w.W, _wide(tr), getattr(tr, "who", None)))
        return out

    def step(self, w: World) -> Command:
        b = self.brain
        self._w = w
        if b._ruf is not self._oeff_of:                  # neue Rufsuche: alle Öffnungen wieder offen
            self._oeff_of, self._oeff_besucht, self._oeff_asked = b._ruf, [], False
            # klare Peilung: Blickrichtung jetzt (er hat sich eben hingedreht) im Karten-KS, gilt für die ganze Rufsuche
            self._oeff_dir = (w.pose[2] if b._ruf is not None and b._ruf.get("seite") and w.pose is not None
                              else None)
        heard = None
        if w.sound is not None:
            sch, age = w.sound
            heard = Heard(sch.t, math.radians(sch.winkel_deg), sch.pegel_db, age, getattr(sch, "dauer_s", 1.0))
        s = Sense(t=w.t, dt=w.dt, balancing=w.balancing, battery=w.battery, seen=self.sense_tracks(w),
                  ds=w.ds, ds_measured=w.ds_measured, tilt=w.tilt, fw_mode=w.fw_mode,
                  floor_ok=w.map_ok and w.free is not None and bool(w.free.valid), height_mm=w.height_mm,
                  yaw=w.yaw, free_center=w.free.center if w.free is not None and w.free.valid else None,
                  path_blocked=w.path_blocked, heard=heard, stimmen=w.stimmen,
                  anker=w.anker and w.pose is not None and w.goal is not None,
                  karte=w.pose is not None and w.pose_ok,
                  heim_m=math.hypot(w.goal[0] - w.pose[0], w.goal[1] - w.pose[1])
                  if w.pose is not None and w.goal is not None else None,
                  nav_extern=self._pfad_faehrt(w), nav_stuck=self._nav_stucks >= 2, stunde=w.stunde, **self._nav(w))
        act = b.update(s)
        heim = self._heim(w)
        if heim is None:
            fc = self._folgen(w)
            if fc is not None:                           # Folgen: Befehl, Höhe und Ziel ganz von follow
                fc.status = "%s | %s" % (act.status, fc.status) if act.status else fc.status
                self._flags(b)
                return fc
        if heim is None and self._pfad_faehrt(w):
            # Geplanter Weg: wie home abfahren (Engstellen, Lücke, Anschub); Bahnprüfung der App bremst ohnehin
            self.f.v = self.pcfg.drive_v * b.innen.f_tempo()        # müde langsamer (Energie 1: drive_v)
            heim = self.f.drive(w, "Rufsuche" if b._ruf is not None else b.task, slow_end=True)
        self._flags(b)
        if heim is not None:
            return Command(heim.v, heim.w, heim.height_mm if act.height_mm is None else act.height_mm,
                           "%s | %s" % (act.status, heim.status), act.target_id)
        return Command(act.v, act.w, act.height_mm, act.status, act.target_id)

    def _flags(self, b: PetBrain) -> None:
        if b.want_set_down:
            b.want_set_down, self.want_set_down = False, True
        if b.want_stand_up:
            b.want_stand_up, self.want_stand_up = False, True
        if b.want_idle:
            b.want_idle, self.want_idle = False, True
        if b.want_ruhe:
            b.want_ruhe, self.want_ruhe = None, b.want_ruhe
        self.gaze = b.gaze
        self.augen_ziel = b.augen_ziel
        self.augen = b.augen

    def _heim(self, w: World) -> Command | None:
        """Heimfahrt des Gehirns (Aufgabe heim, Phase extern) mit behaviors.Home fahren: Planer, Vorpunkt, gerade
        Schlussfahrt, Messung am Startbild (App) und Korrektur der Kartenpose. Der Start ist der Kartenstart, den
        App beim Aufstehen in pet setzt (Nutzer 2026-11-22)."""
        b = self.brain
        if b.task != "heim" or b.phase != "extern" or w.pose is None or w.goal is None:
            self.home_drive = None
            return None
        if self.home_drive is None:
            self.home_drive = Home(self.cfg, self.rcfg, self.h, self.sbcfg)
            self.f.clear()
        hd = self.home_drive
        c = hd.step(w)
        if hd.done:
            b.heim_ende("am Start")
        elif hd.gave_up:
            b.heim_ende("kein Weg zum Start")
        return c

    def info(self) -> dict:
        fd = self.follow_drive
        return {**self.brain.info(), "folgen_suche": self._f_seek + (fd.n_seek if fd else 0),
                "folgen_wieder": self._f_found + (fd.n_found if fd else 0)}
