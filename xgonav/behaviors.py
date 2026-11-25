"""Verhalten: Folgen, Erkunden, Anfahren, Heimfahrt – plus Sicherheitsschicht.

Alle Verhalten arbeiten auf einem gemeinsamen Weltzustand und liefern einen
normierten Fahrbefehl (v, w) und eine gewünschte Körperhöhe. Die
Sicherheitsschicht hat immer das letzte Wort.
"""
from __future__ import annotations

import collections
import logging
import math
import random
from dataclasses import dataclass, field

import numpy as np

from . import navfast
from .config import BehaviorCfg, ExploreCfg, HomeCfg, PatrolCfg, RobotCfg
from .egomotion import EgoResult
from .freespace import FreeSpace
from .mathe import clamp, polar, wrap as _wrap, yaw_rate_to_w
from .geometry import CamGeometry
from .robot import stopp_text
from .tracker import Track

log = logging.getLogger(__name__)


@dataclass
class World:
    t: float
    dt: float
    W: int
    H: int
    hfov: float                       # rad
    ego: EgoResult
    tracks: list[Track]               # bestätigte Tracks
    free: FreeSpace | None
    balancing: bool
    yaw: float                        # Kurs (rad, links +): Koppelnavigation (IMU, sonst Räder, sonst Bild)
    cmd_v: float                      # aktuell gefahrener Befehl (nach Rampe)
    geo: CamGeometry | None = None
    pitch_dev: float = 0.0            # hochfrequentes Nicken in rad (aus Ego-Motion)
    pose: tuple | None = None         # (x, y, th) in der Raumkarte (m, rad), None = keine Karte
    pose_ok: bool = False             # Pose in der Raumkarte bestätigt (GridMap.pose_ok)
    goal: tuple | None = None         # Startpunkt (x, y, th) in der Raumkarte
    # Geradeaus mit cruise_v laut Merkkarte blockiert (gleiche Prüfung wie App._arc_check).
    # Erkunden braucht das: sonst will es weiter vorwärts, die Bahnprüfung setzt v auf 0 und
    # der Rider steht vor dem Hindernis, ohne wegzudrehen (RiderPi 2026-11-04).
    path_blocked: bool = False
    battery: int | None = None        # % oder None (unbekannt)
    # Für den Modus pet (xgonav/pet.py), sonst nicht gefüllt
    ds: float = 0.0                   # Strecke seit dem letzten Regelschritt (m)
    ds_measured: bool = False         # ds stammt von den Rädern, nicht aus dem Fahrbefehl
    tilt: tuple | None = None         # (roll, pitch) in Grad aus frischer IMU
    fw_mode: int | None = None        # /proc/XGORider/state (robot.FW_ZUSTAND; 254 liegt, 0 gefallen)
    height_mm: float | None = None    # Höhen-Sollwert
    sound: tuple | None = None        # (ortung.Schall, Alter s) der letzten Schallmeldung, nur für pet
    stimmen: tuple | None = None      # (bestätigte Stimmproben NN, neu bestätigte NN), nur für pet
    map_ok: bool = False              # Merkkarte vorhanden
    # Bodengrenze fehlt (FloorNet nicht geladen oder ausgefallen): safety() lässt nichts fahren
    blind: bool = False
    # Engstellen (PathFollower): wall_fn(x, y, reach_m) = Abstand zur nächsten belegten Zelle der Raumkarte
    # (inf ab reach_m), gap_fn() = Mitte der Lücke voraus laut Merkkarte (x, y im Rider-KS) oder None
    wall_fn: object = None
    gap_fn: object = None
    # Abgleich am Startbild (startbild.py): Anker zum aktuellen Startpunkt vorhanden
    anker: bool = False
    # Radgeschwindigkeit (m/s) über ~0,2 s, None ohne frische Raddaten; clear_fn() = freie Strecke geradeaus laut
    # Merkkarte (m, inf frei). Beide für den Anschub (PathFollower), clear_fn nur beim Auslösen gerufen.
    v_rad: float | None = None
    clear_fn: object = None
    # Tempo (m/s), auf das der Rider gerade hinbeschleunigt: Sollwert des Generators der Firmware (Fahrrahmen; None
    # auf dem Byteweg). Für das Bremsprofil: Beim Anfahren liegt es über v_rad, die Räder legen noch zu.
    v_erw: float | None = None
    # path_free_fn(pts, pose, tight) = freie Strecke (m) entlang des Wegs pts (N x 2, Welt-m, ab der Pose) laut
    # Merkkarte, inf frei; tight: mit dem Radius der Engstelle. Für das Tempo des Wegfolgers (home.free_slow).
    path_free_fn: object = None
    # arc_free_fn(v, w, tight) = freie Strecke (m) auf dem Bogen des Fahrbefehls (normiert) laut Merkkarte bis
    # zur Reichweite der Merkkarte (2,5 m), wie die Bahnprüfung (gleicher Bogen, gleicher Radius), inf frei
    arc_free_fn: object = None

    def bearing(self, x: float) -> float:
        """Bildspalte -> Winkel rad (links positiv)."""
        f = (self.W / 2) / math.tan(self.hfov / 2)
        cx = self.geo.cx if self.geo is not None else self.W / 2     # Spalte der Fahrtrichtung
        return -math.atan2(x - cx, f)


@dataclass
class Command:
    v: float = 0.0
    w: float = 0.0
    height_mm: float | None = None
    status: str = ""
    target_id: int | None = None
    extra: dict = field(default_factory=dict)


def dreh_w(err: float, kp: float, w_max: float, rcfg: RobotCfg, deadband_deg: float = 0.0,
           w_min: float | None = None) -> float:
    """Gemeinsamer Drehregler für alle Drehungen (PathFollower, Servo, pet, Dunkelangst; seit 2026-11-23):
    Soll-Drehrate kp * err (rad/s) als normierter Drehwert, Betrag höchstens w_max (Faktor: Anteil von robot.max_w),
    mindestens w_min (absolut, Standard 0: die Firmware regelt auch kleine Drehraten, Patch 0032), im Totband
    (Grad) 0."""
    if err == 0.0 or abs(err) < math.radians(deadband_deg):
        return 0.0
    lo = 0.0 if w_min is None else w_min
    return math.copysign(clamp(yaw_rate_to_w(kp * abs(err), rcfg), lo, max(rcfg.w_of(w_max), lo)), err)


# ---------------------------------------------------------------------------
class Behavior:
    name = "base"

    def __init__(self, cfg: BehaviorCfg, rcfg: RobotCfg):
        self.cfg = cfg
        self.rcfg = rcfg

    def step(self, w: World) -> Command:
        raise NotImplementedError

    def zurueck(self, hmm: float | None, status: str = "setze zurück") -> Command:
        """Rückwärts mit behavior.backup_v (blind: nur kurz, Dauer bestimmt der Aufrufer)."""
        return Command(-self.rcfg.v_of(self.cfg.backup_v), 0.0, hmm, status)


class Idle(Behavior):
    name = "idle"

    def step(self, w: World) -> Command:
        return Command(0.0, 0.0, None, "idle")


class Manual(Behavior):
    """Handsteuerung aus der Weboberfläche. Der Browser schickt Sollwerte, solange eine
    Fahrtaste gedrückt ist; bleiben sie länger als HOLD_S aus (Verbindung weg, Taste los),
    steht der Rider. Bodenmodell und Bahnprüfung gelten hier bewusst nicht (wie bei der
    früheren Fernsteuerung), wohl aber Sturz-Erkennung, Begrenzung und Watchdogs."""
    name = "manual"
    HOLD_S = 0.5

    def __init__(self, cfg, rcfg):
        super().__init__(cfg, rcfg)
        self._v = self._w = 0.0
        self._t = -1e9
        self.checked = False      # letzter Sollwert verlangt Bodengrenze + Bahnprüfung (/drive?sicher=1, MCP)
        self._turn: dict | None = None
        self.turn_info: dict | None = None   # /status turn: soll_deg, gedreht_deg, fertig, grund

    def set(self, v: float, w: float, now: float, checked: bool = False) -> None:
        """v, w: Faktoren -1..1 (Anteil von robot.max_v/max_w), seit 2026-11-23. Bricht ein /turn ab."""
        self._v, self._w, self._t = clamp(v, -1.0, 1.0), clamp(w, -1.0, 1.0), now
        self.checked = checked
        if self._turn is not None:
            self.turn_ende("abgebrochen")

    def turn(self, deg: float, w: float, tol_deg: float, now: float, yaw: float, timeout_s: float) -> None:
        """Auf der Stelle um deg drehen (+ = links) mit höchstens w (Anteil von max_w), geregelt über den Kurs
        (App.yaw) mit dreh_auf_kurs; danach turn_settle_s Nachlauf mitzählen. Ein Sollwert (set) bricht ab."""
        self._v = self._w = 0.0
        self._turn = {"soll": math.radians(deg), "summe": 0.0, "y": yaw, "w": w, "tol": tol_deg, "t0": now,
                      "timeout": timeout_s, "ende": None}
        self.turn_info = {"soll_deg": round(deg, 1), "gedreht_deg": 0.0, "fertig": False, "grund": None}

    def turn_ende(self, grund: str) -> None:
        if self.turn_info is not None and not self.turn_info["fertig"]:
            self.turn_info.update(fertig=True, grund=self.turn_info["grund"] or grund)
        self._turn = None

    def _turn_step(self, yaw: float | None, now: float) -> Command:
        t = self._turn
        if yaw is None:
            self.turn_ende("kein Kurs")
            return Command(0.0, 0.0, None, "Handsteuerung – drehen: kein Kurs")
        t["summe"] += _wrap(yaw - t["y"])
        t["y"] = yaw
        self.turn_info["gedreht_deg"] = round(math.degrees(t["summe"]), 1)
        if t["ende"] is None:
            w = dreh_auf_kurs(t["soll"] - t["summe"], t["tol"], now - t["t0"], t["timeout"], self.cfg.turn_to_kp,
                              t["w"], self.rcfg)
            if w is not None:
                return Command(0.0, w, None, "Handsteuerung – drehe %+.0f° von %+.0f°" % (
                    math.degrees(t["summe"]), math.degrees(t["soll"])))
            t["ende"] = now
            ok = abs(t["soll"] - t["summe"]) <= math.radians(t["tol"])
            self.turn_info["grund"] = "angekommen" if ok else "Zeit abgelaufen"
        if now - t["ende"] >= self.cfg.turn_settle_s:       # Nachlauf mitgezählt
            self.turn_ende(self.turn_info["grund"])
        return Command(0.0, 0.0, None, "Handsteuerung – gedreht %+.0f°" % math.degrees(t["summe"]))

    def step(self, wd: World | None, yaw: float | None = None, now: float | None = None) -> Command:
        import time as _time                          # braucht keine Weltdaten (auch ohne Kamera)
        now = _time.monotonic() if now is None else now
        if self._turn is not None:
            return self._turn_step(yaw, now)
        if now - self._t > self.HOLD_S:
            return Command(0.0, 0.0, None, "Handsteuerung – steht")
        return Command(self.rcfg.v_of(self._v), self.rcfg.w_of(self._w), None,
                       "Handsteuerung v=%+.2f w=%+.2f" % (self._v, self._w))


class Servo:
    """Visuelles Servoing auf einen Track: Drehen auf Bildmitte, Abstand über Fußpunkt."""

    def __init__(self, cfg: BehaviorCfg, rcfg: RobotCfg, want_dist: float):
        self.cfg, self.rcfg, self.want = cfg, rcfg, want_dist
        self._prev_err: float | None = None
        self._derr = 0.0
        self._dist: float | None = None
        self._ddot = 0.0
        self._P = None
        self._t_meas = None

    # Kalman-Filter für (Abstand, Fluchtgeschwindigkeit des Ziels). Vorhersage mit der
    # eigenen Fahrt, Messung nur bei neuer Detektion: dazwischen wird die Box nur per
    # Rotationsmodell verschoben, die Entfernung stünde still, während der Rider fährt,
    # und ein Differenzenquotient würde vor allem diese Sprünge messen.
    Q_ACC = 0.6          # m/s², Beschleunigungsrauschen des Ziels (Gehen, Abbiegen, Stehenbleiben)
    R_REL = 0.25         # relative Messunsicherheit der Entfernung
    R_ABS = 0.05         # m

    def reset(self) -> None:
        self._prev_err = None
        self._derr = 0.0
        self._dist = None
        self._ddot = 0.0
        self._P = None
        self._t_meas = None

    def _kf(self, dt: float, own: float, z: float | None) -> None:
        if self._dist is None:
            if z is not None:
                self._dist, self._ddot = z, 0.0
                self._P = [[(self.R_REL * z + self.R_ABS) ** 2, 0.0], [0.0, 0.3 ** 2]]
            return
        P = self._P
        if dt > 0:
            # Vorhersage: d += (v_ziel - v_eigen) dt
            self._dist += (self._ddot - own) * dt
            q = self.Q_ACC ** 2
            p00 = P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1] + q * dt ** 4 / 4
            p01 = P[0][1] + dt * P[1][1] + q * dt ** 3 / 2
            p11 = P[1][1] + q * dt * dt
            P = [[p00, p01], [p01, p11]]
        if z is not None:
            r = (self.R_REL * z + self.R_ABS) ** 2
            s = P[0][0] + r
            k0, k1 = P[0][0] / s, P[0][1] / s
            y = z - self._dist
            self._dist += k0 * y
            self._ddot += k1 * y
            P = [[(1 - k0) * P[0][0], (1 - k0) * P[0][1]],
                 [P[0][1] - k1 * P[0][0], P[1][1] - k1 * P[0][1]]]
        self._ddot = clamp(self._ddot, -1.0, 1.0)
        self._dist = max(0.0, self._dist)
        self._P = P

    def __call__(self, wd: World, tr: Track) -> tuple[float, float, float, float | None]:
        err = wd.bearing(tr.cx)                       # rad, + = Ziel links
        if self._prev_err is not None and wd.dt > 0:
            d = (err - self._prev_err) / wd.dt
            self._derr = 0.7 * self._derr + 0.3 * d
        self._prev_err = err
        rate = self.cfg.kp_turn * err + self.cfg.kd_turn * self._derr
        w_cmd = dreh_w(rate, 1.0, 1.0, self.rcfg, w_min=0.0)   # PD: dem bewegten Ziel nach, ohne Untergrenze

        z = None
        if wd.geo and tr.t_seen != self._t_meas:      # nur neue Detektionen messen
            self._t_meas = tr.t_seen
            z, _how = wd.geo.object_dist(tr.box, tr.cls, wd.pitch_dev, tr.clip)
        self._kf(wd.dt, wd.cmd_v * self.rcfg.vx_limit, z)
        v = 0.0
        if self._dist is not None:
            e = self._dist - self.want
            if abs(e) > self.cfg.dist_deadband:
                v = self.cfg.kp_dist * (e - math.copysign(self.cfg.dist_deadband, e))
            # Vorsteuerung: mit der Geschwindigkeit des Ziels mitfahren (nur wenn nicht zu nah)
            if e > -self.cfg.dist_deadband:
                v += self.cfg.ff_dist * max(0.0, self._ddot) / self.rcfg.vx_limit
        v = clamp(v, -self.rcfg.v_of(self.cfg.backup_v), self.rcfg.max_v)
        # erst ausrichten, dann fahren
        if v > 0:
            v *= clamp(1.0 - abs(err) / math.radians(25), 0.0, 1.0)
        return v, w_cmd, err, self._dist


class Follow(Behavior):
    """Ziel per Servo folgen. Verloren: mit Raumkarte (seek) zur letzten bekannten Stelle fahren (Planer wie home,
    App._plan_tick) und dort einmal umschauen, dann warten; ohne Karte oder bestätigte Pose auf der Stelle zur
    Seite der letzten Sichtung drehen wie bisher."""
    name = "follow"
    PLAN_FAILS = 2                # so oft kein Weg in Folge: dort umschauen, wo er steht

    def __init__(self, cfg, rcfg, target_class: str | None = None, want_dist: float | None = None,
                 hcfg: HomeCfg | None = None, seek: bool = False):
        super().__init__(cfg, rcfg)
        self.cls = target_class or cfg.target_class
        self.servo = Servo(cfg, rcfg, want_dist if want_dist is not None else cfg.follow_dist_m)
        self.target: int | None = None
        self.last_seen_t = -1e9
        self.last_side = 1.0
        self.search_start: float | None = None
        # Suche nach der Karte
        self.seek = seek and cfg.lost_seek
        self.h = hcfg or HomeCfg()
        self.f = PathFollower(cfg, rcfg, self.h)
        self.last_pos: tuple[float, float] | None = None   # letzte Stelle des Ziels in der Raumkarte (m)
        self._spur: collections.deque = collections.deque()  # (t, x, y) des Ziels in der Raumkarte, lost_vel_s lang
        self.erwartet: tuple[float, float] | None = None   # erwartete Stelle (Anzeige in der Raumkarte)
        self.phase = ""               # "" | "fahren" | "umschauen" | "warten"
        self._goal: tuple[float, float] | None = None
        self._req = None
        self.pending = False
        self._t_plan = -1e9
        self._fails = 0
        self._stucks = 0
        self._drive_until = 0.0
        self._umschau = Umschau()
        self.n_seek = self.n_found = 0     # Suchfahrten zur letzten Stelle / danach wiedergefunden

    @property
    def path(self):
        return self.f.path

    # --- Planung (von App bedient, wie home) ---
    def want_plan(self, t: float) -> bool:
        if self.pending or self.phase != "fahren" or self._goal is None:
            return False
        if self.f.path is None or t - self._t_plan > self.h.replan_s or self.f.stuck:
            self._stucks += self.f.stuck
            self.pending, self._t_plan, self.f.stuck = True, t, False
            return True
        return False

    def request_plan(self, planner, grid, w: World, obstacles) -> int:
        self._req = self._goal
        # Dort stand eben das Ziel: Die Karte kann an der Stelle unsicher sein
        return planner.request(w.pose[:2], self._goal, keep=self.f.path, obstacles=obstacles, clear_goal=True)

    def set_path(self, path) -> None:
        self.pending = False
        if self._req is not self._goal or self.phase != "fahren":
            return                                       # Ziel inzwischen ein anderes oder Suche vorbei
        if path is None:
            if self._goal != self.last_pos and self.last_pos is not None and self.f.path is None:
                # zur erwarteten Stelle kein Weg (Wand, Unbekanntes): dann zur zuletzt gesehenen
                log.info("follow: zur erwarteten Stelle kein Weg, fahre zur letzten Stelle (%.2f, %.2f)", *self.last_pos)
                self._goal = self.erwartet = self.last_pos
                self._t_plan = -1e9
                return
            self._fails += 1
            return                                       # alten Weg behalten, falls vorhanden
        self._fails = 0
        self.f.set_path(path)

    @property
    def ziel_anzeige(self) -> tuple[float, float] | None:
        """Erwartete Stelle des verlorenen Ziels, solange er dorthin fährt oder sich dort umschaut (Raumkarte)."""
        return self.erwartet if self.phase in ("fahren", "umschauen") else None

    def _vorhersage(self, w: World) -> tuple[float, float]:
        """Wohin das Ziel gegangen sein dürfte: letzte Stelle + Tempo (aus _spur) x lost_lead_s, gekappt."""
        c = self.cfg
        lx, ly = self.last_pos
        sp = self._spur
        if len(sp) < 2 or sp[-1][0] - sp[0][0] < 0.3:
            return lx, ly
        dt = sp[-1][0] - sp[0][0]
        vx, vy = (sp[-1][1] - sp[0][1]) / dt, (sp[-1][2] - sp[0][2]) / dt
        v = math.hypot(vx, vy)
        if v < c.lost_lead_min_mps:
            return lx, ly
        d = min(v * c.lost_lead_s, c.lost_lead_max_m)
        return lx + vx / v * d, ly + vy / v * d

    def _seek_reset(self) -> None:
        self.erwartet = None
        self.phase, self._goal, self._req = "", None, None
        self.f.clear()
        self._fails = self._stucks = 0
        self._umschau.reset()

    def _look(self, w: World, why: str) -> None:
        log.info("follow: %s verloren, %s: schaue mich um", self.cls, why)
        self.phase = "umschauen"
        self.f.clear()
        self._umschau.reset()

    def _seek_step(self, w: World) -> Command | None:
        """Suche nach der Raumkarte; None = nicht möglich (alter Suchweg)."""
        if not (self.seek and self.last_pos is not None and w.pose is not None and w.pose_ok):
            return None
        x, y, th = w.pose
        c = self.cfg
        if self.phase == "":
            lx, ly = self.last_pos
            d = math.hypot(lx - x, ly - y)
            ex, ey = self.erwartet = self._vorhersage(w)
            d = math.hypot(ex - x, ey - y)
            if d < c.lost_arrive_m:
                self._look(w, "an der erwarteten Stelle")
            else:
                self.phase, self._goal = "fahren", self.erwartet
                self.n_seek += 1
                self._fails = self._stucks = 0
                self._t_plan = -1e9
                self._drive_until = w.t + fahrzeit_s(d, self.f.v, self.rcfg)
                log.info("follow: %s verloren, fahre zur erwarteten Stelle (%.2f, %.2f), %.2f m (zuletzt gesehen "
                         "(%.2f, %.2f))", self.cls, ex, ey, d, lx, ly)
        if self.phase == "fahren":
            gx, gy = self._goal
            d = math.hypot(gx - x, gy - y)
            if d < c.lost_arrive_m:
                self._look(w, "angekommen")
            elif self._fails >= self.PLAN_FAILS or self._stucks >= 2 or w.t > self._drive_until:
                self._look(w, "kein Weg" if self._fails >= self.PLAN_FAILS else "festgefahren" if self._stucks >= 2 else "Zeit um")
            elif self.f.path is None:
                return Command(0, 0, c.height_cruise_mm, "suche %s: plane Weg zur erwarteten Stelle" % self.cls)
            else:
                cmd = self.f.drive(w, "suche %s: zur erwarteten Stelle %.2f m" % (self.cls, d), slow_end=True)
                cmd.height_mm = c.height_search_mm if abs(cmd.v) < 1e-6 else cmd.height_mm
                return cmd
        if self.phase == "umschauen":
            summe = self._umschau.schritt(th)
            if summe < math.radians(c.lost_look_deg):
                return Command(0.0, self.last_side * self.rcfg.w_of(c.lost_look_w), c.height_search_mm,
                               "suche %s: schaue mich um %.0f°" % (self.cls, math.degrees(summe)))
            log.info("follow: %s nicht wiedergefunden, warte", self.cls)
            self.phase = "warten"
        return Command(0, 0, c.height_cruise_mm, "Ziel verloren – warte")

    def pick(self, w: World) -> Track | None:
        cands = [t for t in w.tracks if t.cls == self.cls]
        cur = next((t for t in cands if t.id == self.target), None)
        if cur is not None and cur.lost_s(w.t) < self.cfg.lost_search_s:
            return cur
        if not cands:
            return None
        # größte (nächste) Instanz, Bildmitte bevorzugt
        best = max(cands, key=lambda t: t.h / w.H - 0.3 * abs(w.bearing(t.cx)) / math.radians(45))
        if best.id != self.target:
            self.servo.reset()
        self.target = best.id
        return best

    def step(self, w: World) -> Command:
        tr = self.pick(w)
        if tr is not None and tr.lost_s(w.t) < self.cfg.lost_search_s:
            self.search_start = None
            self.last_seen_t = w.t
            v, wc, err, dist = self.servo(w, tr)
            self.last_side = 1.0 if err >= 0 else -1.0
            if self.phase:
                log.info("follow: %s wiedergefunden (%s)", self.cls, self.phase)
                self.n_found += self.phase != "warten"
                self._seek_reset()
            if self.seek and w.pose is not None and w.pose_ok and w.balancing and dist is not None:
                x, y, th = w.pose
                d, a = min(dist, self.cfg.lost_seek_max_m), th + err
                self.last_pos = polar(x, y, d, a)
                sp = self._spur
                sp.append((w.t, *self.last_pos))
                while sp and sp[0][0] < w.t - self.cfg.lost_vel_s:
                    sp.popleft()
            fast = abs(v) > 0.6 * self.rcfg.max_v
            hmm = self.cfg.height_fast_mm if fast else self.cfg.height_cruise_mm
            st = "verfolge #%d %s (%+.0f°, %s)" % (tr.id, tr.cls, math.degrees(err),
                                                  "%.2f m" % dist if dist is not None else "? m")
            return Command(v, wc, hmm, st, tr.id, extra={"dist": dist, "err": err})
        # verloren -> in Richtung der letzten Sichtung suchen
        if self.search_start is None:
            self.search_start = w.t
            self.servo.reset()
        c = self._seek_step(w)
        if c is not None:
            return c
        if w.t - self.search_start > self.cfg.search_timeout_s:
            return Command(0, 0, self.cfg.height_cruise_mm, "Ziel verloren – warte")
        return Command(0.0, self.last_side * self.rcfg.w_of(self.cfg.search_w), self.cfg.height_search_mm,
                       "suche %s (%s)" % (self.cls, "links" if self.last_side > 0 else "rechts"))


class Explore(Behavior):
    """Reaktives Erkunden: in Richtung des größten Freiraums, bei Blockade
    zurücksetzen und um einen zufälligen Winkel wegdrehen (Kurs aus Ego-Motion)."""
    name = "explore"
    BRAKE_S = 0.6

    def __init__(self, cfg, rcfg):
        super().__init__(cfg, rcfg)
        self.state = "cruise"
        self.t_state = 0.0
        self.turn_goal = 0.0
        self.turn_dir = 1.0
        self.rng = random.Random()

    def _enter(self, s: str, t: float) -> None:
        self.state, self.t_state = s, t

    def step(self, w: World) -> Command:
        fs = w.free
        center = fs.center if fs and fs.valid else 1.0
        blocked = (center < self.cfg.blocked_free or w.path_blocked
                   or (w.ego.ttc < self.cfg.ttc_avoid_s and w.cmd_v > 0.03))
        hmm = self.cfg.height_cruise_mm

        if self.state == "cruise":
            if blocked:
                self._enter("brake", w.t)
                return Command(0, 0, hmm, "Hindernis – stopp")
            v = self.rcfg.v_of(self.cfg.cruise_v) * clamp(center / self.cfg.slow_free, 0.3, 1.0)
            wc = 0.0
            if fs and fs.valid:
                n = len(fs.sectors)
                # Zielrichtung: Sektoren mit Freiraum gewichtet, Mitte leicht bevorzugt
                angs = [w.hfov * (0.5 - (i + 0.5) / n) for i in range(n)]
                scores = [fs.sectors[i] - 0.15 * abs(angs[i]) for i in range(n)]
                best = max(range(n), key=lambda i: scores[i])
                if fs.sectors[best] - center > 0.1:
                    wc = yaw_rate_to_w(1.5 * angs[best], self.rcfg)
            return Command(v, wc, hmm, "erkunde (frei %.2f)" % center)

        if self.state == "brake":
            # Der Rider braucht ~0,6 s zum Anhalten (2026-11-05); direkt von vor auf zurück
            # brächte ihn stark ins Nicken
            if w.t - self.t_state < self.BRAKE_S:
                return Command(0, 0, hmm, "Hindernis – stopp")
            self._enter("backup", w.t)

        if self.state == "backup":
            if w.t - self.t_state < 0.8:
                return self.zurueck(hmm, "zurücksetzen")
            if fs and fs.valid:
                n = len(fs.sectors)
                left, right = sum(fs.sectors[: n // 2]), sum(fs.sectors[(n + 1) // 2:])
                self.turn_dir = 1.0 if left >= right else -1.0
            else:
                self.turn_dir = self.rng.choice((-1.0, 1.0))
            self.turn_goal = w.yaw + self.turn_dir * math.radians(self.rng.uniform(60, 140))
            self._enter("turn", w.t)

        if self.state == "turn":
            rem = (self.turn_goal - w.yaw) * self.turn_dir
            if (rem <= 0 and center > self.cfg.slow_free and not w.path_blocked) or w.t - self.t_state > 6.0:
                self._enter("cruise", w.t)
                return Command(0, 0, hmm, "weiter")
            return Command(0.0, self.turn_dir * self.rcfg.w_of(self.cfg.turn_w), hmm,
                           "drehe weg (%+.0f° offen)" % math.degrees(rem * self.turn_dir))
        return Command(0, 0, hmm, self.state)


class Approach(Behavior):
    """Erkunden, bis ein Objekt der Zielklasse gesehen wird; dann anfahren und davor stoppen.
    Angekommen ist er, wenn der geschätzte Abstand höchstens approach_dist_m + dist_deadband beträgt und er
    ausgerichtet ist (ARRIVE_S lang), oder wenn er vor dem nahen Ziel blockiert ist: Das Verhalten will vor,
    Sicherheitsschicht oder Bahnprüfung halten ihn an (gefahrener Befehl ~0 seit BLOCK_S). Sonst stünde er dort
    für immer mit „verfolge …“, weil das Verhalten selbst nie v = 0 befiehlt."""
    name = "approach"
    ARRIVE_S = 1.0
    ARRIVE_DEG = 8.0
    BLOCK_S = 3.0
    BLOCK_NEAR_M = 0.3            # blockiert zählt als angekommen bis so viel über dem Wunschabstand
    LOST_EXPLORE_S = 4.0          # so lange suchen, dann wieder erkunden

    def __init__(self, cfg, rcfg, target_class: str | None = None):
        super().__init__(cfg, rcfg)
        self.cls = target_class or cfg.target_class
        self.follow = Follow(cfg, rcfg, self.cls, cfg.approach_dist_m)
        self.explore = Explore(cfg, rcfg)
        self.arrived_since: float | None = None
        self.blocked_since: float | None = None
        self.done = False
        self.result = ""

    def _finish(self, status: str) -> Command:
        self.done, self.result = True, status
        return Command(0, 0, self.cfg.height_search_mm, status)

    def step(self, w: World) -> Command:
        if self.done:
            return Command(0, 0, self.cfg.height_search_mm, self.result)
        seen = any(t.cls == self.cls for t in w.tracks)
        if not seen and self.follow.search_start is None and self.follow.target is None:
            self.arrived_since = self.blocked_since = None
            c = self.explore.step(w)
            c.status = "suche %s: %s" % (self.cls, c.status)
            return c
        c = self.follow.step(w)
        dist, err = c.extra.get("dist"), c.extra.get("err")
        if c.target_id is not None and dist is not None:
            want = self.cfg.approach_dist_m
            near = dist <= want + self.cfg.dist_deadband and abs(err) < math.radians(self.ARRIVE_DEG)
            pushing = c.v > 0.02 and abs(w.cmd_v) < 0.01 and dist < want + self.BLOCK_NEAR_M
            self.arrived_since = (w.t if self.arrived_since is None else self.arrived_since) if near else None
            self.blocked_since = (w.t if self.blocked_since is None else self.blocked_since) if pushing else None
            if self.arrived_since is not None and w.t - self.arrived_since >= self.ARRIVE_S:
                return self._finish("angekommen bei %s (%.2f m)" % (self.cls, dist))
            if self.blocked_since is not None and w.t - self.blocked_since >= self.BLOCK_S:
                return self._finish("vor %s blockiert (%.2f m)" % (self.cls, dist))
        else:
            self.arrived_since = self.blocked_since = None
        if self.follow.search_start is not None and w.t - self.follow.search_start > self.LOST_EXPLORE_S:
            # länger nicht gesehen -> wieder erkunden; beim Wiederfinden mit frischem Abstand anfangen
            self.follow.target = None
            self.follow.search_start = None
            self.follow.servo.reset()
        return c


STUCK_PROGRESS_M = 0.3    # so viel näher ans Ziel, dann zählt früheres Festfahren nicht mehr


class Annaeherung:
    """Fortschritt zum Ziel (home, Heimweg von explore): n = Festfahren in Folge ohne Annäherung; STUCK_PROGRESS_M
    näher setzt zurück. Aufgeben nach max_fail Mal festgefahren und give_up_s ohne Annäherung."""

    def __init__(self, h: HomeCfg):
        self.h = h
        self.reset()

    def reset(self) -> None:
        self.n, self.best, self.t_best = 0, math.inf, None

    def schritt(self, dist: float, t: float) -> bool:
        """Abstand jetzt eintragen; True = aufgeben."""
        if dist < self.best - STUCK_PROGRESS_M:
            self.best, self.n, self.t_best = dist, 0, t
        return self.n >= self.h.max_fail and t - self.t_best > self.h.give_up_s

    def nah(self, dist: float) -> bool:
        """Festgefahren nahe am Ziel (near_m): Steht dort etwas, kommt er nie ganz hin – gilt als angekommen."""
        return self.n >= 1 and dist < self.h.near_m


class Umschau:
    """Beim Umschauen gedrehter Winkel (rad), aus dem Kurs je Schritt aufsummiert."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.th, self.summe = None, 0.0

    def schritt(self, th: float) -> float:
        if self.th is not None:
            self.summe += abs(_wrap(th - self.th))
        self.th = th
        return self.summe


def dreh_auf_kurs(err: float, tol_deg: float, laeuft_s: float, timeout_s: float, kp: float, w_max: float,
                  rcfg: RobotCfg) -> float | None:
    """Drehwert auf der Stelle zum Zielkurs (err rad, dreh_w), None = angekommen (|err| ≤ tol) oder Zeit um."""
    if abs(err) > math.radians(tol_deg) and laeuft_s < timeout_s:
        return dreh_w(err, kp, w_max, rcfg)
    return None


def fahrzeit_s(d: float, f: float, rcfg: RobotCfg, v_min_mps: float = 0.02) -> float:
    """Zeitgrenze für eine Fahrt über d m mit Tempo-Faktor f: 4 s + doppelte Fahrzeit."""
    return 4.0 + 2.0 * d / max(rcfg.v_of(f) * rcfg.vx_limit, v_min_mps)


class PathFollower:
    """Weg abfahren (Pure Pursuit) mit der am RiderPi erprobten Lenkung; für home, explore, patrol.
    Beobachtet den Fortschritt: stuck wird wahr, wenn der Rider STUCK_S lang weder 5 cm vorankam noch
    beim Drehen auf der Stelle 10° gutmachte. Die Luftlinie zum Ziel taugt dafür nicht: Jeder Umweg
    entfernt sich erst und galt so nach 3 s als festgefahren."""
    STUCK_S = 3.0
    # Vorwärts befohlen, aber die Räder stehen: er drückt gegen etwas. Am RiderPi 2026-11-06 stand er so
    # 4-5 s. Normales Anfahren braucht dort bis 5 cm im Mittel 1 s, höchstens 1,8 s; 1,5 s löste einmal
    # auf freier Strecke aus.
    PRESS_V, PRESS_M, PRESS_S = 0.08, 0.02, 2.5
    PRESS_AFTER_PUSH_S = 1.0

    def __init__(self, cfg: BehaviorCfg, rcfg: RobotCfg, hcfg: HomeCfg, v: float | None = None):
        self.cfg, self.rcfg, self.h = cfg, rcfg, hcfg
        self.v = hcfg.v if v is None else v      # Tempo auf freier Strecke (explore: explore.v), Anteil von max_v
        self.path: list[tuple[float, float]] | None = None
        self._pa, self._pa_of = None, None       # self.path als Feld für den C++-Kern
        self.stuck = False
        self.pressing = False
        self._press: tuple | None = None
        self._push_until: float | None = None    # Anschub läuft bis dahin
        self._push_n = 0                          # Anschübe seit dem letzten echten Fortschritt
        self._push_at: tuple[float, float] | None = None
        self._push_from: tuple[float, float] = (0.0, 0.0)
        self._push_budget = 0.0                   # Strecke (m), die der laufende Anschub höchstens fährt
        self._v_hoch_t = -math.inf               # zuletzt Räder >= kante_v
        self._kante: tuple | None = None         # (x, y, t): Räder standen schlagartig still
        self._turning = False
        self._anchor: tuple[float, float] | None = None
        self._t_prog = 0.0
        self._err_ref = 0.0
        self.tight_t = -math.inf                  # zuletzt in einer Engstelle gefahren (Zeit)
        self._kn, self._kn_of = (), None          # Ecken des Wegs über home.turn_enter_deg (Index), je Weg einmal

    def set_path(self, path) -> None:
        self.path = list(path) if path else None

    def clear(self) -> None:
        self.path = None
        self.stuck = False
        self.pressing = False
        self._press = None
        self._push_until = None     # _push_n bleibt: erst 0,3 m Fortschritt erlaubt neue Anschübe
        self._kante = None
        self._anchor = None

    def target(self, x: float, y: float, lookahead_m: float | None = None) -> tuple[float, float]:
        """Punkt lookahead_m entlang des Wegs ab der Projektion der Pose."""
        p = self.path
        if len(p) == 1:
            return p[0]
        rest = self.h.lookahead_m if lookahead_m is None else lookahead_m
        L = navfast.lib() if self.h.native else None
        if L is not None:
            if self._pa_of is not p:                   # Weg einmal als Feld, bis ein neuer kommt
                self._pa, self._pa_of = np.ascontiguousarray(p, np.float64).reshape(-1, 2), p
            return navfast.path_target(L, self._pa, x, y, rest)
        best, bi, bt = math.inf, 0, 0.0
        for i in range(len(p) - 1):
            ax, ay = p[i]
            bx, by = p[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0) if L2 > 1e-12 else 0.0
            ex, ey = ax + t * dx - x, ay + t * dy - y
            d = math.sqrt(ex * ex + ey * ey)            # ausgeschrieben statt math.hypot: rundet wie der C++-Kern
            if d < best:
                best, bi, bt = d, i, t
        i, t = bi, bt
        while i < len(p) - 1:
            ax, ay = p[i]
            bx, by = p[i + 1]
            L = math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay))
            left = (1.0 - t) * L
            if left >= rest and L > 1e-9:
                t += rest / L
                return ax + t * (bx - ax), ay + t * (by - ay)
            rest -= left
            i, t = i + 1, 0.0
        return p[-1]

    @staticmethod
    def free_ahead_m(w: World) -> float | None:
        """Freier Boden geradeaus laut Bodenmodell (m ab Kamerafußpunkt), inf bis zum Horizont, None unbekannt."""
        fs = w.free
        if fs is None or not fs.valid or w.geo is None:
            return None
        c = fs.center
        if c >= 0.98:
            return math.inf
        g = w.geo
        d = g.ground_dist(g.H - c * (g.H - g.horizon_row(w.pitch_dev)), w.pitch_dev)
        return math.inf if d is None else d

    def _watch_press(self, w: World) -> None:
        t, cmd_v = w.t, w.cmd_v
        x, y = w.pose[0], w.pose[1]
        c = self.cfg
        if self._push_at is not None and math.hypot(x - self._push_at[0], y - self._push_at[1]) > 0.3:
            self._push_n, self._push_at = 0, None      # wieder richtig vorangekommen
        # Schneller Anschub: Räder liefen eben noch und stehen schlagartig, obwohl vorwärts befohlen war (Fuge,
        # Kante). safety()/Bahnprüfung senken den Fahrwert danach oft sofort (Stufe dahinter), dann käme die
        # Drückerkennung unten nie auf ihre PRESS_S (RiderPi 2026-11-23, Fuge vor der Fensterstufe).
        vr = w.v_rad
        if c.kante_v > 0 and vr is not None:
            if vr >= c.kante_v:
                self._v_hoch_t = t
            k = self._kante
            if k is None:
                if vr < 0.03 and cmd_v > self.PRESS_V and t - self._v_hoch_t <= c.kante_fenster_s:
                    self._kante = (x, y, t)
            elif math.hypot(x - k[0], y - k[1]) >= self.PRESS_M or vr > 0.05 or t - k[2] > c.kante_s + 1.0:
                self._kante = None
            elif t - k[2] >= c.kante_s:
                self._kante = None
                if self._start_push(w, "Räder stehen schlagartig"):
                    return
        if cmd_v <= self.PRESS_V:
            self._press = None
        elif self._press is None or math.hypot(x - self._press[0], y - self._press[1]) >= self.PRESS_M:
            self._press = (x, y, t)
        elif t - self._press[2] > self.PRESS_S:
            self._press = None
            free = self.free_ahead_m(w)
            # Etwas Niedriges hält die Räder (Rille, Kante), vorn ist frei: kurz kräftig anschieben
            if not (free is not None and free >= c.push_free_m and self._start_push(w, "Räder stehen")):
                self.stuck = self.pressing = True

    def _start_push(self, w: World, grund: str) -> bool:
        """Anschub beginnen, wenn noch einer frei ist und vorn Platz ist: Strecke = freier Boden (Bodenmodell)
        und freie Bahn (Merkkarte) minus push_rest_m, höchstens push_max_m."""
        c = self.cfg
        if self._push_n >= c.push_n:
            return False
        free = self.free_ahead_m(w)
        if free is None:
            return False
        if w.clear_fn is not None:
            free = min(free, w.clear_fn())
        budget = min(c.push_max_m, free - c.push_rest_m)
        if budget < c.push_min_m:
            return False
        x, y = w.pose[0], w.pose[1]
        self._push_n += 1
        self._push_at = self._push_from = (x, y)
        self._push_budget = budget
        self._push_until = w.t + c.push_s
        self._anchor = None                    # die Zeit am Hindernis zählt nicht als Festfahren
        log.info("Anschub %d: %s, vorn %.2f m frei, höchstens %.2f m", self._push_n, grund, free, budget)
        return True

    def press_step(self, w: World, label: str) -> Command | None:
        """Drücken ohne Fortschritt prüfen; während eines Anschubs dessen Fahrbefehl, sonst None.
        Der Anschub (extra anschub) übergeht Freiraum/TTC in safety() und die Strecke der Bahnprüfung; begrenzt
        ist er durch seine Strecke: Weg + Radtempo x push_brake_s (Bremsweg) bleibt unter _push_budget."""
        if self._push_until is None:
            self._watch_press(w)
        if self._push_until is not None:
            c = self.cfg
            # nur vorwärts gezählt: beim Anfahren rollt der Rider erst kurz zurück
            th = w.pose[2]
            moved = max(0.0, (w.pose[0] - self._push_from[0]) * math.cos(th) + (w.pose[1] - self._push_from[1]) * math.sin(th))
            ahead = moved + max(0.0, w.v_rad or 0.0) * c.push_brake_s
            if w.t < self._push_until and ahead < self._push_budget:
                return Command(self.rcfg.v_of(c.push_v), 0.0, c.height_cruise_mm, "%s – Anschub %d" % (label, self._push_n),
                               extra={"anschub": True})
            # Danach genügt 1 s ohne Fortschritt für den nächsten Schritt: Anfahren ist dann vorbei
            self._push_until = None
            self._press = (w.pose[0], w.pose[1], w.t - self.PRESS_S + self.PRESS_AFTER_PUSH_S)
            if ahead >= self._push_budget:
                # Strecke erreicht: voll bremsen (nur 0 setzt der Rider schnell um)
                return Command(0.0, 0.0, c.height_cruise_mm, "%s – Anschub %d: %.2f m, bremse" % (
                    label, self._push_n, moved), extra={"bremsen": True})
        return None

    def _watch(self, t: float, x: float, y: float, err: float) -> None:
        if self._anchor is None:
            self._anchor, self._t_prog, self._err_ref = (x, y), t, abs(err)
            return
        moved = math.hypot(x - self._anchor[0], y - self._anchor[1]) >= 0.05
        turned = self._turning and abs(err) < self._err_ref - math.radians(10)
        if moved or turned:
            self._anchor, self._t_prog, self._err_ref = (x, y), t, abs(err)
        elif t - self._t_prog > self.STUCK_S:
            self.stuck = True
            self._anchor = None

    def _proj(self, x: float, y: float) -> tuple[int, float]:
        """Projektion der Pose auf den Weg (mindestens 2 Punkte): (Index des Stücks, Anteil darauf)."""
        p = self.path
        best, bi, bt = math.inf, 0, 0.0
        for i in range(len(p) - 1):
            ax, ay = p[i]
            bx, by = p[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0) if L2 > 1e-12 else 0.0
            d = math.hypot(ax + t * dx - x, ay + t * dy - y)
            if d < best:
                best, bi, bt = d, i, t
        return bi, bt

    def _ecken(self) -> tuple:
        """Indizes der Wegpunkte, an denen der Weg um mehr als home.turn_enter_deg abknickt (je Weg einmal)."""
        p = self.path
        if self._kn_of is not p:
            lim = math.radians(self.h.turn_enter_deg)
            kn = []
            for i in range(1, len(p) - 1):
                a = math.atan2(p[i][1] - p[i - 1][1], p[i][0] - p[i - 1][0])
                b = math.atan2(p[i + 1][1] - p[i][1], p[i + 1][0] - p[i][0])
                if abs(_wrap(b - a)) > lim:
                    kn.append(i)
            self._kn, self._kn_of = tuple(kn), p
        return self._kn

    def knick_m(self, x: float, y: float, weiter=None) -> float:
        """Weg (m) bis zur ersten Ecke über home.turn_enter_deg voraus, inf ohne. weiter: der nächste Weg bei
        Weiterfahrt ohne Halt; knickt er am Wegende so stark ab, zählt das Ende als Ecke."""
        p = self.path
        if p is None or len(p) < 2:
            return math.inf
        bi, bt = self._proj(x, y)
        s = (1.0 - bt) * math.hypot(p[bi + 1][0] - p[bi][0], p[bi + 1][1] - p[bi][1])
        for i in range(bi + 1, len(p)):
            if i in self._ecken():
                return s
            if i < len(p) - 1:
                s += math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
        if weiter is not None and len(weiter) >= 2:
            ex, ey = p[-1]
            a = math.atan2(ey - p[-2][1], ex - p[-2][0])
            q = next((q for q in weiter if math.hypot(q[0] - ex, q[1] - ey) >= 0.15), None)
            if q is not None and abs(_wrap(math.atan2(q[1] - ey, q[0] - ex) - a)) > math.radians(self.h.turn_enter_deg):
                return s
        return math.inf

    def ahead_pts(self, x: float, y: float) -> list:
        """Pose und die Wegpunkte voraus (für die freie Strecke entlang des Wegs)."""
        p = self.path
        if len(p) < 2:
            return [(x, y), p[0]]
        bi, _ = self._proj(x, y)
        return [(x, y)] + list(p[bi + 1:])

    def rest_m(self, x: float, y: float) -> float:
        """Restweg (m) entlang des Wegs ab der Projektion der Pose bis zum Ende."""
        p = self.path
        if len(p) == 1:
            return math.hypot(p[0][0] - x, p[0][1] - y)
        bi, bt = self._proj(x, y)
        ax, ay = p[bi]
        bx, by = p[bi + 1]
        rest = (1.0 - bt) * math.hypot(bx - ax, by - ay)
        for i in range(bi + 1, len(p) - 1):
            rest += math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
        return rest

    def end_v(self, w: World, x: float, y: float) -> float:
        """Höchster Fahrwert vor dem Wegende (home.end_*): Bremsweg-Profil auf den Restweg, um die Trägheit
        (Radtempo x end_lag_s) vorausgerechnet."""
        return self.brems_v(w, self.rest_m(x, y))

    def brems_v(self, w: World, s: float) -> float:
        """Höchster Fahrwert, um nach s Metern zu stehen (home.end_*), mindestens end_min_v."""
        h, rc = self.h, self.rcfg
        v_ist = w.v_rad if w.v_rad is not None else w.cmd_v * rc.vx_limit
        v_erw = getattr(w, "v_erw", None)
        if v_erw is not None:
            # beschleunigt er noch, fährt er während end_lag_s schneller als jetzt (RiderPi 2026-11-24: Fahrwert
            # 0,28 -> 0,10, Räder trotzdem 0,29 -> 0,40 m/s)
            v_ist = max(v_ist, v_erw)
        s = max(0.0, s - max(0.0, v_ist) * h.end_lag_s)
        return max(rc.v_of(h.end_min_v), math.sqrt(2.0 * h.end_decel_mps2 * s) / rc.vx_limit)

    def drive(self, w: World, label: str, slow_end: bool = True, weiter=None) -> Command:
        """Fahrbefehl entlang des Wegs; label steht vorn im Status. slow_end: zum Wegende hin rechtzeitig
        abbremsen (end_v; aus nur, wenn er ohne Halt weiterfährt). Ohne: Mit explore.v 0,30 rollte er über den
        Ankunftsradius hinaus und pendelte am Startpunkt hin und her (Simulator 2026-11-11, ~60 s).
        weiter: der nächste Weg bei Weiterfahrt ohne Halt (für das Bremsen vor einer scharfen Ecke)."""
        hmm = self.cfg.height_cruise_mm
        x, y, th = w.pose
        push = self.press_step(w, label)
        if push is not None:
            return push
        # Engstelle (Wand der Raumkarte nah): dem Weg eng folgen und erst ausrichten, dann fahren
        tm = self.h.tight_m
        tight = tm > 0 and w.wall_fn is not None and any(
            w.wall_fn(px, py, tm) < tm for px, py in [(x, y)] + [self.target(x, y, d) for d in (0.15, 0.3, 0.45)])
        gap = w.gap_fn() if tight and w.gap_fn is not None else None
        if tight:
            self.tight_t = w.t
        tx, ty = self.target(x, y, self.h.tight_lookahead_m if tight else None)
        err = _wrap(math.atan2(ty - y, tx - x) - th)
        mitte = False
        knick = None                                    # Weg bis zur nächsten Ecke, einmal je Schritt
        if gap is not None and self.h.tight_center_knick_m > 0:
            knick = self.knick_m(x, y, None if slow_end else weiter)
            if knick <= self.h.tight_center_knick_m:
                gap = None                              # der Weg biegt gleich ab: um die Ecke, nicht durch die Lücke
        if gap is not None:
            # Lücke voraus in Wegrichtung: auf ihre gemessene Mitte zielen statt auf den Weg der Raumkarte
            eg = math.atan2(gap[1], max(gap[0], 0.10))
            if abs(eg - err) < math.radians(self.h.tight_center_max_deg):
                err, mitte = eg, True
        # Auf der Stelle drehen nur bei großem Fehler (Hysterese), sonst in der Kurve weiterfahren
        if self._turning:
            self._turning = abs(err) > math.radians(self.h.tight_turn_exit_deg if tight else self.h.turn_exit_deg)
        else:
            self._turning = abs(err) > math.radians(self.h.tight_turn_enter_deg if tight else self.h.turn_enter_deg)
        self._watch(w.t, x, y, err)
        if self._turning:
            wc = dreh_w(err, self.h.kp_turn, self.h.turn_w_max, self.rcfg)
            return Command(0.0, wc, hmm, "%s – drehe (%+.0f°)" % (label, math.degrees(err)), extra={"tight": tight})
        wc = clamp(yaw_rate_to_w(self.h.kp_turn * err, self.rcfg), -1.0, 1.0)     # Kurve: Lenken beim Fahren
        if abs(wc) < self.rcfg.w_of(self.h.min_w) and abs(err) > math.radians(self.h.min_w_err_deg):
            wc = math.copysign(self.rcfg.w_of(self.h.min_w), err)
        v = self.rcfg.v_of(self.v) * clamp(1.0 - abs(err) / math.radians(90), 0.3, 1.0)
        if tight:
            v *= self.h.tight_v
        if slow_end:
            v = min(v, self.end_v(w, x, y))
        frei_txt, info = "", None
        if self.h.free_slow:
            # Tempo nach freier Strecke: vor dem ersten Hindernis der Merkkarte entlang des Wegs und vor einer
            # scharfen Ecke so bremsen wie am Wegende (home.free_slow)
            if knick is None:
                knick = self.knick_m(x, y, None if slow_end else weiter)
            weg = bogen = math.inf
            if w.path_free_fn is not None:
                weg = w.path_free_fn(self.ahead_pts(x, y), w.pose, tight)
            if w.arc_free_fn is not None and v > 0:
                # derselbe Bogen wie die Bahnprüfung, nur weiter voraus: Sie bremste im Simulator, während der Weg
                # frei war, weil der Bogen ein paar Zentimeter neben dem Weg an einer Kante vorbeiführt
                bogen = w.arc_free_fn(v, wc, tight)
            frei = min(knick, weg - self.h.free_rest_m, bogen - self.h.free_rest_m)
            # für die Rohaufnahme (meta.jsonl "frei"): welche Strecke begrenzt, None = frei
            info = {k: (None if s == math.inf else round(s, 2)) for k, s in
                    (("knick", knick), ("weg", weg), ("bogen", bogen), ("v_erw", w.v_erw if w.v_erw is not None
                                                                      else math.inf))}
            if frei < math.inf:
                vf = self.brems_v(w, frei)
                if vf < v:
                    v, frei_txt = vf, ", frei %.2fm" % max(0.0, frei)
        return Command(v, wc, hmm, "%s (%+.0f°%s%s)" % (label, math.degrees(err), ", Lücke" if mitte else ", eng" if tight else "",
                                                       frei_txt), extra={"tight": tight, "frei": info})

    def ausrichten(self, err: float, label: str = "am Start – richte aus") -> Command | None:
        """Am Ziel auf den Endkurs drehen (home.align_heading, align_tol_deg); None = steht schon richtig."""
        if self.h.align_heading and abs(err) > math.radians(self.h.align_tol_deg):
            return self.align(err, label)
        return None

    def align(self, err: float, label: str) -> Command:
        """Auf der Stelle auf einen Endkurs drehen (err rad)."""
        wc = dreh_w(err, self.h.kp_turn, self.h.turn_w_max, self.rcfg)
        return Command(0.0, wc, self.cfg.height_cruise_mm, "%s (%+.0f°)" % (label, math.degrees(err)))


class Home(Behavior):
    """Zurück zum Startpunkt der Raumkarte: Weg plant planner.py (App stößt an, sobald
    want_plan() es verlangt), gefahren wird mit PathFollower. Sicherheitsschicht und
    Bahnprüfung gelten wie bei explore. Am Ziel optional auf den Startkurs drehen.

    Mit Startbild ([startbild] enabled, Anker vorhanden: World.anker) endet die Fahrt anders: Weg zum Vorpunkt
    vor_m vor dem Start, dort auf den Startkurs drehen, gerade auf den Start fahren (App merkt sich dabei die
    Tiefenbilder), stehen und messen (App: startbild.StartAbgleich, Korrektur der Kartenpose), bei mehr als
    nach_m Rest einmal nachfahren, auf den Startkurs drehen. Hängt die Schlussfahrt, geht es ohne Startbild
    weiter wie bisher."""
    name = "home"
    SB_RUHE_S = 0.5               # vor dem Messbild so lange stehen (Nicken abklingen lassen)

    def __init__(self, cfg, rcfg, hcfg: HomeCfg | None = None, sbcfg=None):
        super().__init__(cfg, rcfg)
        self.h = hcfg or HomeCfg()
        self.sb = sbcfg if sbcfg is not None and sbcfg.enabled else None
        self.phase = "weg"            # mit Startbild: weg -> dreh -> gerade -> messen -> fein -> ende
        self.t_phase = 0.0
        self.ohne_sb = False          # Startbild aufgegeben: weiter wie ohne
        self.want_messung = False     # App: jetzt messen
        self.messung = None           # App: startbild.Ergebnis
        self.nach_n = 0
        self._fein_d: float | None = None
        self._fein_fahrt = False
        self._end_vz = 0.0
        self._gemessen = False        # Messung für diesen Halt schon angefordert
        self._grund = "–"             # Ergebnis der Messung (Status am Ende)
        self.f = PathFollower(cfg, rcfg, self.h, v=self.h.start_v)
        self.pending = False
        self.t_plan = -1e9
        self.fails = 0
        self.done = False
        self.gave_up = False
        self.keep_ok = False          # darf der Planer den bisherigen Weg behalten? (nicht nach Festfahren)
        self.an = Annaeherung(self.h)  # Festfahren in Folge ohne Annäherung ans Ziel

    @property
    def path(self):
        return self.f.path

    # --- Planung (von App bedient) ---
    def want_plan(self, t: float) -> bool:
        if self.pending or self.done or self.gave_up or self.phase != "weg":
            return False
        if self.f.path is None or t - self.t_plan > self.h.replan_s or self.f.stuck:
            self.keep_ok = self.f.path is not None and not self.f.stuck
            self.an.n += self.f.stuck
            self.pending, self.t_plan, self.f.stuck = True, t, False
            return True
        return False

    def _mit_sb(self, w: World) -> bool:
        return self.sb is not None and not self.ohne_sb and (w.anker or self.phase != "weg")

    def _vorpunkt(self, goal: tuple) -> tuple[float, float]:
        gx, gy, gth = goal
        return gx - self.sb.vor_m * math.cos(gth), gy - self.sb.vor_m * math.sin(gth)

    def request_plan(self, planner, grid, w: World, obstacles) -> int:
        ziel = self._vorpunkt(w.goal) if self._mit_sb(w) else w.goal[:2]
        return planner.request(w.pose[:2], ziel, keep=self.f.path if self.keep_ok else None, obstacles=obstacles)

    def set_path(self, path: list[tuple[float, float]] | None) -> None:
        self.pending = False
        if path is None:
            self.fails += 1
            if self.fails >= self.h.max_fail:
                self.gave_up = True
            return                                  # alten Weg behalten, falls vorhanden
        self.fails = 0
        self.f.set_path(path)

    def step(self, w: World) -> Command:
        hmm = self.cfg.height_cruise_mm
        if w.pose is None or w.goal is None:
            return Command(0, 0, hmm, "keine Raumkarte – stopp")
        x, y, th = w.pose
        gx, gy, gth = w.goal
        dist = math.hypot(gx - x, gy - y)
        if self.done:
            return Command(0, 0, hmm, "am Start angekommen")
        if self._mit_sb(w):
            c = self._startbild(w, x, y, th, gx, gy, gth, dist)
            if c is not None:
                return c
            vx, vy = self._vorpunkt(w.goal)
            return self._weg(w, math.hypot(vx - x, vy - y), "zum Vorpunkt")
        if dist < self.h.arrive_m:
            c = self.f.ausrichten(_wrap(gth - th))
            if c is not None:
                return c
            self.done = True
            return Command(0, 0, hmm, "am Start angekommen")
        # Steht das Ziel selbst hinter einem Hindernis, findet der Planer immer denselben Weg und die
        # Bahnprüfung bremst ihn immer wieder (RiderPi 2026-11-06, Klebemarke am Startpunkt): aufhören
        if self.an.nah(dist):
            self.done = True
            return Command(0, 0, hmm, "am Start angekommen (%.2f m daneben)" % dist)
        return self._weg(w, dist, "zum Start")

    def _weg(self, w: World, dist: float, label: str) -> Command:
        """Dem geplanten Weg folgen (zum Start oder zum Vorpunkt); Festfahren und Aufgeben wie gehabt."""
        hmm = self.cfg.height_cruise_mm
        if self.an.schritt(dist, w.t):
            self.gave_up = True
        if self.gave_up:
            return Command(0, 0, hmm, "kein Weg zum Start – stopp")
        if self.f.path is None:
            return Command(0, 0, hmm, "plane Weg %s" % label)
        return self.f.drive(w, "%s %.2f m" % (label, dist))

    # --- Schluss mit Startbild ---
    def _setze(self, phase: str, w: World) -> None:
        self.phase, self.t_phase = phase, w.t

    def _startbild(self, w: World, x, y, th, gx, gy, gth, dist) -> Command | None:
        """Phasen nach dem Weg; None = noch auf dem Weg zum Vorpunkt."""
        sb, hmm = self.sb, self.cfg.height_cruise_mm
        if self.phase == "weg":
            vx, vy = self._vorpunkt((gx, gy, gth))
            dv = math.hypot(vx - x, vy - y)
            # am Vorpunkt, oder festgefahren in seiner Nähe (Hindernis darauf): von hier aus weiter
            if dv >= sb.vor_tol_m and not self.an.nah(dv):
                return None
            self.f.clear()
            self._setze("dreh", w)
        if self.phase == "dreh":
            err = _wrap(gth - th)
            if abs(err) > math.radians(3.0):
                return self.f.align(err, "Startbild: richte aus")
            self._setze("gerade", w)
        if self.phase == "gerade":
            ux, uy = math.cos(gth), math.sin(gth)
            rest = (gx - x) * ux + (gy - y) * uy           # bis zum Start entlang des Startkurses
            if rest <= 0.01:
                self._setze("messen", w)
                return Command(0, 0, hmm, "Startbild: stehe")
            v_ms = max(self.rcfg.v_of(sb.gerade_v) * self.rcfg.vx_limit, 0.02)
            if w.t - self.t_phase > 3.0 * (sb.vor_m + 0.2) / v_ms + 5.0:
                # kommt nicht voran (Bahnprüfung, Hindernis am Start): ohne Startbild zu Ende
                log.info("Startbild: Schlussfahrt hängt, weiter ohne Abgleich")
                self.ohne_sb = True
                self._setze("weg", w)
                return Command(0, 0, hmm, "Startbild: Schlussfahrt hängt")
            soll = math.atan2(gy - y, gx - x) if rest > 0.10 else gth   # auf den Start zielen, zuletzt Kurs halten
            err = _wrap(soll - th)
            wm = self.rcfg.w_of(self.h.turn_w_max)
            wc = clamp(yaw_rate_to_w(self.h.kp_turn * err, self.rcfg), -wm, wm)
            return Command(self.rcfg.v_of(sb.gerade_v), wc, hmm, "Startbild: gerade zum Start %.2f m" % rest)
        if self.phase == "messen":
            if self.messung is None:
                if not self._gemessen and w.t - self.t_phase >= self.SB_RUHE_S:
                    self.want_messung, self._gemessen = True, True
                if w.t - self.t_phase > sb.timeout_s + self.SB_RUHE_S:
                    log.info("Startbild: keine Messung in %.0f s, ohne Abgleich fertig", sb.timeout_s)
                    self._grund = "keine Messung"
                    self._setze("ende", w)
                return Command(0, 0, hmm, "Startbild: messe")
            erg, self.messung, self._gemessen = self.messung, None, False
            self._grund = erg.grund
            # Die App hat die Pose schon korrigiert; dist ist der Rest danach
            if erg.ok and dist > sb.nach_m and self.nach_n < 1:
                self.nach_n += 1
                self._fein_d, self._fein_fahrt = None, False
                self._setze("fein", w)
            else:
                self._setze("ende", w)
        if self.phase == "fein":
            if dist < 0.015 or (self._fein_d is not None and self._fein_fahrt and dist > self._fein_d + 0.005):
                self._setze("ende", w)
            else:
                err = _wrap(math.atan2(gy - y, gx - x) - th)
                if not self._fein_fahrt and abs(err) > math.radians(3.0):
                    return self.f.align(err, "Startbild: drehe zum Start")
                self._fein_fahrt = True
                self._fein_d = dist if self._fein_d is None else min(self._fein_d, dist)
                wm = self.rcfg.w_of(self.h.turn_w_max)
                wc = clamp(yaw_rate_to_w(self.h.kp_turn * err, self.rcfg), -wm, wm)
                if w.t - self.t_phase > 20.0:
                    self._setze("ende", w)
                else:
                    return Command(0.75 * self.rcfg.v_of(sb.gerade_v), wc, hmm, "Startbild: nachfahren %.2f m" % dist)
        # ende: auf den Startkurs drehen (enger als sonst, sb.end_deg), fertig; über den Kurs hinaus gedreht oder
        # nach end_s: stehen bleiben statt pendeln
        err = _wrap(gth - th)
        vz = math.copysign(1.0, err)
        ueber = self._end_vz != 0.0 and vz != self._end_vz
        if (self.h.align_heading and abs(err) > math.radians(sb.end_deg) and not ueber
                and w.t - self.t_phase < sb.end_s):
            self._end_vz = vz
            return self.f.align(err, "am Start – richte aus")
        self.done = True
        return Command(0, 0, hmm, "am Start angekommen (Startbild: %s)" % self._grund)


class MapExplore(Behavior):
    """Raum kartieren: erst ein Rundumblick, dann Frontier für Frontier anfahren (frontier.py), bis
    nichts Unbekanntes mehr erreichbar ist; am Ende Karte speichern und zum Start. Festfahren: erst neu
    planen, dann zurücksetzen und das Ziel sperren; ohne Ziel kurz reaktiv fahren (Explore), zuletzt
    aufhören. Sicherheitsschicht und Bahnprüfung gelten wie überall."""
    name = "explore"
    NO_PATH_RETRY_S = 1.0

    def __init__(self, cfg, rcfg, hcfg: HomeCfg | None = None, ecfg: ExploreCfg | None = None):
        super().__init__(cfg, rcfg)
        self.h, self.e = hcfg or HomeCfg(), ecfg or ExploreCfg()
        self.f = PathFollower(cfg, rcfg, self.h, v=self.e.v)
        self.wander = Explore(cfg, rcfg)
        self.state = "scan"
        self.pending = False
        self.keep_ok = False
        self.t_plan = -1e9
        self.t0: float | None = None
        self.until = 0.0
        self.frontier: tuple | None = None
        self.goal: tuple | None = None
        self.visited: list = []            # angefahren, Frontier blieb: nicht noch einmal
        self.blocked_goals: list = []      # festgefahren: gesperrt bis zur nächsten Befreiung
        self.gap_blocked = False           # eben an einer Lücke gesperrt (App: Merkkarte -> Raumkarte)
        self.clusters = self.avoided = 0
        self.no_goal = 0
        self.stucks = 0
        self.t_goal = 0.0
        self.wanders = 0
        self.home_fails = 0
        self.home_an = Annaeherung(self.h)  # Heimweg: Festfahren ohne Annäherung
        self.reason = ""
        self.save_map = False              # App speichert die Karte, sobald das gesetzt ist
        self.done = False
        self._scan = Umschau()             # Rundumblick am Anfang
        self._cleared = False
        self._face: tuple | None = None
        self._known_best = 0.0
        self._t_growth: float | None = None
        self._xy: tuple | None = None      # letzte Pose (für das Nachplanen kurz vor dem Ziel)
        self.tight_goals = 0               # angefahrene Ziele, die nur in Engstellen lagen
        self.rechecks = 0                  # so oft am Start noch einmal nach Zielen gesucht
        self._home_d0: float | None = None # Entfernung zum Start, als der Heimweg begann
        self._goal_tight = False
        # Übernächstes Ziel: kurz vor dem Ziel im Voraus geplant (ab dem Ziel, ohne dessen Stelle), damit er
        # beim Erreichen ohne Halt weiterfährt
        self._req = "ziel"                 # Art des laufenden Auftrags: "ziel" oder "next"
        self._next = None                  # ExplorePlan
        self._next_t = -1e9
        self._next_for: tuple | None = None
        self._near = False

    @property
    def path(self):
        return self.f.path

    def info(self) -> dict:
        """Für /status und die Weboberfläche."""
        return {"zustand": self.state, "stellen": self.clusters, "gesperrt": self.avoided, "befreit": self.wanders,
                "ziel": None if self.frontier is None else [round(self.frontier[0], 2), round(self.frontier[1], 2)],
                "ende": self.reason}

    # --- Planung (von App bedient) ---
    def want_plan(self, t: float) -> bool:
        if self.pending or self.state not in ("ziel", "heim"):
            return False
        if self.f.path is None and t - self.t_plan < self.NO_PATH_RETRY_S:
            return False                            # ohne Weg nicht im Regeltakt nachfragen
        every = self.h.replan_s
        near = (self.state == "ziel" and self.goal is not None and self._xy is not None and self.f.path is not None
                and not self.f.stuck
                and math.hypot(self.goal[0] - self._xy[0], self.goal[1] - self._xy[1]) < self.e.roll_m)
        if near:
            every = min(every, self.e.roll_every_s)     # kurz vor dem Ziel: es wandert mit der Grenze mit
        if self.f.path is None or self.f.stuck or t - self.t_plan > every:
            self.keep_ok = self.f.path is not None and not self.f.stuck
            self.pending, self.t_plan = True, t
            # kurz vor dem Ziel abwechselnd: Ziel nachführen, übernächstes Ziel vorausplanen
            self._req = "next" if near and self._req == "ziel" else "ziel"
            return True
        return False

    def request_plan(self, planner, grid, w: World, obstacles) -> int:
        from .frontier import next_goal
        if self.state == "heim":
            self.home_an.n += self.f.stuck
            self.f.stuck = False
            return planner.request(w.pose[:2], w.goal[:2], keep=self.f.path if self.keep_ok else None, obstacles=obstacles)
        if not self._cleared:
            grid.clear_here(self.e.start_free_r_m)  # wo er steht, ist Boden; die Kamera sieht dort nichts
            self._cleared = True
        r = self.e.avoid_r_m
        avoid = [(x, y, r) for x, y in self.visited + self.blocked_goals]
        if self._req == "next" and self.goal is not None and self.frontier is not None and self.f.path:
            # ab dem Ziel, mit dem Kurs der Ankunft, ohne die Stelle, die er gerade anfährt
            p = self.f.path
            tha = math.atan2(p[-1][1] - p[-2][1], p[-1][0] - p[-2][0]) if len(p) >= 2 else w.pose[2]
            self._next_for = self.frontier
            return planner.request_fn(next_goal, grid, (self.goal[0], self.goal[1], tha), self.h, self.e, obstacles,
                                      avoid + [(self.frontier[0], self.frontier[1], r)], None, None,
                                      self.tight_goals < self.e.tight_max)
        self._req = "ziel"
        return planner.request_fn(next_goal, grid, w.pose, self.h, self.e, obstacles, avoid,
                                  self.frontier, self.f.path if self.keep_ok else None,
                                  self.tight_goals < self.e.tight_max)

    def set_path(self, res) -> None:
        self.pending = False
        if self.state == "heim":
            if hasattr(res, "frontier"):             # Antwort auf die letzte Zielsuche: veraltet
                self.t_plan = -1e9
                return
            if res is None:
                self.home_fails += 1
                if self.home_fails >= self.h.max_fail:
                    self._finish("kein Weg zum Start")
                return
            self.home_fails = 0
            self.f.set_path(res)
            return
        if self.state != "ziel" or isinstance(res, list):
            return
        if self._req == "next":                      # Vorausplanung: nur merken
            self._req = "ziel"
            ok = (res is not None and res.path is not None and self.frontier is not None and self._next_for is not None
                  and math.hypot(self.frontier[0] - self._next_for[0], self.frontier[1] - self._next_for[1]) <= self.e.keep_dist_m)
            self._next, self._next_t = (res, self.t_plan) if ok else (None, -1e9)
            return
        if res is not None and res.known_m2 > self._known_best + self.e.min_growth_m2:
            self._known_best, self._t_growth = res.known_m2, self.t_plan
        if res is None or res.path is None:          # None: Planer-Ausnahme
            self.clusters, self.avoided = (res.clusters, res.avoided) if res is not None else (self.clusters, self.avoided)
            self.f.clear()
            self.frontier = self.goal = None
            self.no_goal += 1
            return
        self.no_goal = 0
        self.clusters, self.avoided = res.clusters, res.avoided
        moved = math.inf if self.frontier is None else math.hypot(res.frontier[0] - self.frontier[0], res.frontier[1] - self.frontier[1])
        if moved > self.e.keep_dist_m:
            log.info("Erkunden: neues Ziel (%.2f, %.2f)%s statt %s, Pose (%.2f, %.2f), %d Stellen, %d gesperrt",
                     res.frontier[0], res.frontier[1], " eng" if res.tight else "",
                     "keins" if self.frontier is None else "(%.2f, %.2f)" % self.frontier,
                     self._xy[0] if self._xy else math.nan, self._xy[1] if self._xy else math.nan,
                     res.clusters, res.avoided)
            self.stucks = 0                         # neues Ziel
            self.t_goal = self.t_plan
            self.f.clear()
            self.tight_goals += self._goal_tight    # das vorige lag in einer Engstelle: zählt
            self._next = None
        elif moved > 0.15:
            self.t_goal = self.t_plan               # das Ziel wandert mit der Grenze: kein Festfahren
        self._goal_tight = res.tight
        self.frontier, self.goal = res.frontier, res.goal
        self.f.set_path(res.path)

    def neu_planen(self) -> None:
        """Die Karte hat eine neue Sperre (Dunkelzone): Ziel und Weg verwerfen, sofort neu suchen."""
        self.f.clear()
        self.frontier = self.goal = None
        self._next = None
        self.stucks = 0
        self.keep_ok = False
        self.t_plan = -1e9

    # --- Abschluss ---
    def _end(self, why: str, w: World) -> None:
        """Erkunden beendet: Karte speichern, zum Start, falls gewünscht und möglich."""
        self.reason = why
        self.save_map = True
        self.f.clear()
        self.frontier = self.goal = None
        self.t_plan = -1e9
        if self.e.return_home and w.goal is not None:
            self.state = "heim"
        else:
            self._finish(why)

    def _finish(self, why: str = "") -> None:
        self.state = "fertig"
        self.done = True
        self.save_map = True
        if why and why != self.reason:
            self.reason = "%s, %s" % (self.reason, why) if self.reason else why
        self.f.clear()

    def step(self, w: World) -> Command:
        hmm = self.cfg.height_cruise_mm
        e = self.e
        if w.pose is None:
            return Command(0, 0, hmm, "keine Raumkarte – stopp")
        x, y, th = w.pose
        self._xy = (x, y)
        if self.t0 is None:
            self.t0 = w.t
        if self.state not in ("heim", "fertig"):
            if w.t - self.t0 > e.max_s:
                self._end("Zeitgrenze", w)
            elif w.battery is not None and 0 < w.battery <= e.min_battery:
                self._end("Akku", w)

        if self.state == "scan":
            summe = self._scan.schritt(th)
            if summe >= 2 * math.pi:
                self.state = "ziel"
                return Command(0, 0, hmm, "Rundumblick fertig")
            return Command(0.0, self.rcfg.w_of(e.scan_w), hmm, "Rundumblick %.0f°" % math.degrees(summe))
        if self.state == "face":
            # Zur Frontier drehen: sie kann seitlich oder hinter ihm liegen, das Bild zeigt nur ±44°
            fx, fy = self._face
            err = _wrap(math.atan2(fy - y, fx - x) - th)
            if abs(err) < math.radians(15) or w.t >= self.until or math.hypot(fx - x, fy - y) < 0.05:
                self.state, self.until = "settle", w.t + e.settle_s
                return Command(0, 0, hmm, "schaue")
            return self.f.align(err, "schaue zur Stelle")
        if self.state == "settle":
            if w.t >= self.until:
                self.state = "ziel"
            return Command(0, 0, hmm, "schaue")
        if self.state == "backup":
            if w.t >= self.until:
                self.state = "ziel"
                return Command(0, 0, hmm, "weiter")
            return self.zurueck(hmm)
        if self.state == "wander":
            if w.t >= self.until:
                self.state = "ziel"
                self.wander.state = "cruise"
                return Command(0, 0, hmm, "weiter")
            # Auch beim reaktiven Befreien prüfen, ob er nur drückt (RiderPi 2026-11-17: 15 s gegen eine Rille)
            push = self.f.press_step(w, "befreie mich")
            if push is not None:
                return push
            if self.f.pressing:
                self.f.stuck = self.f.pressing = False
                self.wander.state = "cruise"
                self.state, self.until = "backup", w.t + 0.8
                return Command(0, 0, hmm, "befreie mich: hänge fest")
            c = self.wander.step(w)
            c.status = "befreie mich: " + c.status
            return c
        if self.state == "fertig":
            return Command(0, 0, hmm, self._final_status())
        if self.state == "heim":
            if w.goal is None:
                self._finish()
                return Command(0, 0, hmm, self._final_status())
            gx, gy, gth = w.goal
            dist = math.hypot(gx - x, gy - y)
            if self._home_d0 is None:
                self._home_d0 = dist
            if dist < self.h.arrive_m:
                if (self.reason == "Karte fertig" and self.rechecks < e.recheck_max
                        and self._home_d0 >= e.recheck_min_m):
                    # Auf dem Heimweg kam Neues in die Karte: noch einmal suchen, erst dann ist sie fertig
                    self.rechecks += 1
                    self.reason = ""
                    self.state, self._home_d0 = "ziel", None
                    self.no_goal = self.stucks = self.home_fails = 0
                    self.home_an.reset()
                    self.frontier = self.goal = None
                    self.f.clear()
                    self.t_plan, self._t_growth = -1e9, w.t
                    return Command(0, 0, hmm, "am Start – suche noch einmal")
                c = self.f.ausrichten(_wrap(gth - th))
                if c is not None:
                    return c
                self._finish()
                return Command(0, 0, hmm, self._final_status())
            if self.home_an.nah(dist):
                self._finish("am Start, %.2f m daneben" % dist)
                return Command(0, 0, hmm, self._final_status())
            if self.home_an.schritt(dist, w.t):
                self._finish("Start nicht erreicht, %.2f m entfernt" % dist)
                return Command(0, 0, hmm, self._final_status())
            if self.f.path is None:
                return Command(0, 0, hmm, "Karte fertig, plane Weg zum Start")
            return self.f.drive(w, "Karte fertig, zum Start %.2f m" % dist, slow_end=True)

        # --- ziel ---
        if self._t_growth is None:
            self._t_growth = w.t
        if w.t - self._t_growth > e.no_growth_s:
            self._end("Karte fertig", w)            # es kommt kaum noch etwas dazu
            return Command(0, 0, hmm, self._final_status())
        if self.no_goal >= e.no_goal_limit:
            self.no_goal = 0
            if self.blocked_goals or self.clusters > self.avoided or not self.visited:
                # Es gibt noch Stellen, nur gerade nicht anfahrbar (oder noch gar kein Ziel gefunden,
                # etwa wegen Fehlalarmen rundherum): woanders hin und noch einmal versuchen
                if self.wanders >= e.max_wander:
                    self._end("festgefahren", w)
                    return Command(0, 0, hmm, "festgefahren – Stopp")
                self.wanders += 1
                self.blocked_goals.clear()
                self.state, self.until = "wander", w.t + e.wander_s
                return Command(0, 0, hmm, "keine Stelle erreichbar, befreie mich")
            self._end("Karte fertig", w)
            return Command(0, 0, hmm, self._final_status())
        timed_out = self.frontier is not None and w.t - self.t_goal > e.goal_timeout_s
        if self.f.stuck or timed_out:
            self.f.stuck = False
            # Kontakt oder festgefahren in einer Engstelle/Lücke (explore.gap_block_s): gleich sperren statt erst
            # neu zu planen. Neu planen führte durch dieselbe Lücke (RiderPi 2026-11-23, Stühle: 3 s „Bahn 0,17 m“)
            in_gap = self.e.gap_block_s > 0 and w.t - self.f.tight_t <= self.e.gap_block_s
            self.stucks = 2 if timed_out or self.f.pressing or in_gap else self.stucks + 1
            self.f.pressing = False
            if self.stucks == 1:
                self.keep_ok = False
                self.f.set_path(None)               # sofort neu planen, ohne den alten Weg
                return Command(0, 0, hmm, "komme nicht weiter, plane neu")
            if self.frontier is not None:
                self.blocked_goals.append((self.frontier[0], self.frontier[1]))
                if in_gap and self.goal is not None:
                    # auch den Haltepunkt: Nachbarstellen hinter derselben Lücke fallen mit (RiderPi 2026-11-24)
                    self.blocked_goals.append((self.goal[0], self.goal[1]))
            # an einer Lücke gesperrt: App trägt die Hindernisse der Merkkarte ringsum in die Raumkarte ein
            self.gap_blocked = in_gap
            self.stucks = 0
            self.frontier = self.goal = None
            self.f.clear()
            self.state, self.until = "backup", w.t + 0.8
            return Command(0, 0, hmm, "Ziel gesperrt")
        if self.f.path is None:
            return Command(0, 0, hmm, "suche Ziel (%d Stellen)" % self.clusters)
        if math.hypot(self.goal[0] - x, self.goal[1] - y) < e.reach_m:
            self.visited.append((self.frontier[0], self.frontier[1]))
            nxt, self._next = self._next, None
            seen = abs(_wrap(math.atan2(self.frontier[1] - y, self.frontier[0] - x) - th)) < math.radians(e.pass_view_deg)
            if nxt is not None and seen and w.t - self._next_t < e.pass_age_s:
                # Die Stelle liegt im Bild und das nächste Ziel steht schon fest: ohne Halt weiter
                log.info("Erkunden: (%.2f, %.2f) erreicht bei (%.2f, %.2f), ohne Halt weiter zu (%.2f, %.2f)%s",
                         self.frontier[0], self.frontier[1], x, y, nxt.frontier[0], nxt.frontier[1],
                         " eng" if nxt.tight else "")
                self.tight_goals += self._goal_tight
                self._goal_tight = nxt.tight
                self.frontier, self.goal = nxt.frontier, nxt.goal
                self.stucks, self.t_goal, self.t_plan = 0, w.t, w.t
                self.f.clear()
                self.f.set_path(nxt.path)
                return self.f.drive(w, "erkunde (%d Stellen)" % self.clusters)
            log.info("Erkunden: (%.2f, %.2f) erreicht bei (%.2f, %.2f), halte und schaue",
                     self.frontier[0], self.frontier[1], x, y)
            self._face = self.frontier
            self.frontier = self.goal = None
            self.f.clear()
            self.state, self.until = "face", w.t + 6.0
            return Command(0, 0, hmm, "Ziel erreicht")
        # Steht das nächste Ziel schon fest, mit vollem Tempo über das Ziel; sonst davor abbremsen
        slow_end = self._next is None or w.t - self._next_t >= e.pass_age_s
        return self.f.drive(w, "erkunde (%d Stellen)" % self.clusters, slow_end=slow_end,
                            weiter=None if slow_end else self._next.path)

    def _final_status(self) -> str:
        offen = self.avoided
        if self.reason == "Karte fertig":
            return "Karte fertig (%d Stellen nicht erreicht)" % offen if offen else "Karte fertig"
        return "Erkunden beendet: %s" % self.reason


class Patrol(Behavior):
    """Streife: Wegpunkte der Route der Reihe nach anfahren (auf Wunsch im Kreis), am Ende oder bei wenig Akku
    zum Start. Festgefahren: erst neu planen, dann zurücksetzen und den Punkt auslassen; bleibt in einer Runde
    keiner übrig, zum Start. Bahnprüfung und Sicherheitsschicht gelten wie überall; die Karte ändert sich nicht."""
    name = "patrol"
    NO_PATH_RETRY_S = 1.0

    def __init__(self, cfg, rcfg, hcfg: HomeCfg | None = None, pcfg: PatrolCfg | None = None):
        super().__init__(cfg, rcfg)
        self.h, self.p = hcfg or HomeCfg(), pcfg or PatrolCfg()
        self.f = PathFollower(cfg, rcfg, self.h)
        self.points: list[tuple[float, float]] = []
        self.loop = False
        self.idx = 0
        self.lap = 1
        self.state = "fahrt"
        self.pending = False
        self.keep_ok = False
        self.t_plan = -1e9
        self.t_point: float | None = None
        self.until = 0.0
        self.fails = self.stucks = 0
        self.reached_lap = 0                 # in dieser Runde erreichte Punkte
        self.reached = 0
        self.skipped: list[int] = []         # ausgelassene Punkte (1-basiert), je Runde neu
        self.reason = ""
        self.done = False
        self.want_set_down = False

    def set_route(self, points, loop: bool) -> None:
        self.points, self.loop = [tuple(p) for p in points], loop

    @property
    def path(self):
        return self.f.path

    def info(self) -> dict:
        return {"zustand": self.state, "punkt": self.idx + 1, "von": len(self.points), "runde": self.lap,
                "erreicht": self.reached, "ausgelassen": list(self.skipped), "ende": self.reason}

    # --- Planung (von App bedient) ---
    def want_plan(self, t: float) -> bool:
        if self.pending or self.state not in ("fahrt", "heim"):
            return False
        if self.f.path is None and t - self.t_plan < self.NO_PATH_RETRY_S:
            return False
        if self.f.path is None or self.f.stuck or t - self.t_plan > self.h.replan_s:
            self.keep_ok = self.f.path is not None and not self.f.stuck
            self.pending, self.t_plan = True, t
            return True
        return False

    def request_plan(self, planner, grid, w: World, obstacles) -> int:
        keep = self.f.path if self.keep_ok else None
        if self.state == "heim":
            return planner.request(w.pose[:2], w.goal[:2], keep=keep, obstacles=obstacles)
        return planner.request(w.pose[:2], self.points[self.idx], keep=keep, obstacles=obstacles, clear_goal=False)

    def set_path(self, res) -> None:
        self.pending = False
        if res is None:
            self.fails += 1
            return
        self.fails = 0
        self.f.set_path(res)

    # --- Ablauf ---
    def _go_home(self, why: str) -> None:
        self.reason = why
        self.state = "heim"
        self.f.clear()
        self.stucks = self.fails = 0
        self.t_plan = -1e9

    def _finish(self, why: str) -> None:
        self.state = "fertig"
        self.done = True
        self.reason = (self.reason + ", " + why) if self.reason else why
        self.f.clear()

    def _next(self, w: World) -> None:
        """Nächster Punkt; am Ende der Route nächste Runde oder zum Start."""
        self.idx += 1
        self.t_point = w.t
        self.stucks = self.fails = 0
        self.f.clear()
        self.t_plan = -1e9
        if self.idx < len(self.points):
            return
        if self.reached_lap == 0:
            self._go_home("kein Punkt erreichbar")
        elif self.loop:
            self.idx, self.reached_lap = 0, 0
            self.lap += 1
            self.skipped = []
        else:
            self._go_home("Runde fertig")

    def _skip(self, w: World) -> Command:
        self.skipped.append(self.idx + 1)
        self.state, self.until = "backup", w.t + 0.8
        self.f.clear()
        return Command(0, 0, self.cfg.height_cruise_mm, "Punkt %d ausgelassen" % (self.idx + 1))

    def step(self, w: World) -> Command:
        hmm = self.cfg.height_cruise_mm
        if w.pose is None:
            return Command(0, 0, hmm, "keine Raumkarte – stopp")
        x, y, _ = w.pose
        if self.state == "fertig":
            return Command(0, 0, hmm, "Streife beendet: %s" % self.reason)
        if self.t_point is None:
            self.t_point = w.t
        if w.battery is not None and w.battery > 0:
            if w.battery <= self.p.down_battery:
                self._finish("Akku leer")
                self.want_set_down = True
                return Command(0, 0, hmm, "Streife beendet: Akku leer, setze ab")
            if w.battery <= self.p.return_battery and self.state != "heim":
                self._go_home("Akku")
        if self.state == "schau":
            if w.t >= self.until:
                self.state = "fahrt"
                self._next(w)
            return Command(0, 0, hmm, "Streife: schaue")
        if self.state == "backup":
            if w.t < self.until:
                return self.zurueck(hmm)
            self.state = "fahrt"
            self._next(w)
            return Command(0, 0, hmm, "weiter")
        if self.state == "heim":
            if w.goal is None:
                self._finish("kein Startpunkt")
                return Command(0, 0, hmm, "Streife beendet: %s" % self.reason)
            dist = math.hypot(w.goal[0] - x, w.goal[1] - y)
            stuck = self.f.stuck
            if stuck:
                self.f.stuck = self.f.pressing = False
                self.stucks += 1
                self.keep_ok = False
                self.f.set_path(None)
            if dist < self.h.arrive_m or (self.stucks >= 1 and dist < self.h.near_m):
                self._finish("am Start" if dist < self.h.arrive_m else "am Start, %.2f m daneben" % dist)
            elif self.stucks >= 3 * self.h.max_fail or self.fails >= 3 * self.h.max_fail:
                self._finish("Start nicht erreicht, %.2f m entfernt" % dist)
            if self.state == "fertig":
                return Command(0, 0, hmm, "Streife beendet: %s" % self.reason)
            if self.f.path is None:
                return Command(0, 0, hmm, "Streife: plane Weg zum Start")
            return self.f.drive(w, "Streife: zum Start %.2f m" % dist)
        # --- fahrt ---
        px, py = self.points[self.idx]
        dist = math.hypot(px - x, py - y)
        arrived = dist < self.p.reach_m
        if self.f.stuck:
            pressing = self.f.pressing
            self.f.stuck = self.f.pressing = False
            self.stucks = 2 if pressing else self.stucks + 1
            arrived = arrived or dist < self.h.near_m
            if not arrived and self.stucks < 2:
                self.keep_ok = False
                self.f.set_path(None)
                return Command(0, 0, hmm, "komme nicht weiter, plane neu")
            if not arrived:
                return self._skip(w)
        if arrived:
            self.reached += 1
            self.reached_lap += 1
            self.state, self.until = "schau", w.t + self.p.pause_s
            self.f.clear()
            return Command(0, 0, hmm, "Streife: Punkt %d erreicht" % (self.idx + 1))
        if w.t - self.t_point > self.p.point_timeout_s or self.fails >= self.h.max_fail:
            return self._skip(w)
        if self.f.path is None:
            return Command(0, 0, hmm, "Streife: plane Weg zu Punkt %d" % (self.idx + 1))
        return self.f.drive(w, "Streife: Punkt %d/%d, Runde %d, %.2f m" % (self.idx + 1, len(self.points), self.lap, dist))


def make_behavior(name: str, cfg: BehaviorCfg, rcfg: RobotCfg, target: str | None = None,
                  hcfg: HomeCfg | None = None, ecfg: ExploreCfg | None = None, has_map: bool = True,
                  pcfg: PatrolCfg | None = None, petcfg=None, sbcfg=None) -> Behavior:
    """has_map: Raumkarte vorhanden; sonst erkundet explore nur reaktiv."""
    def pet():
        from .pet import Pet          # pet.py importiert dieses Modul
        return Pet(cfg, rcfg, petcfg, hcfg=hcfg, sbcfg=sbcfg, ecfg=ecfg)
    return {"idle": lambda: Idle(cfg, rcfg),
            "pet": pet,
            "manual": lambda: Manual(cfg, rcfg),
            "follow": lambda: Follow(cfg, rcfg, target, hcfg=hcfg, seek=has_map),
            "explore": lambda: MapExplore(cfg, rcfg, hcfg, ecfg) if has_map else Explore(cfg, rcfg),
            "approach": lambda: Approach(cfg, rcfg, target),
            "home": lambda: Home(cfg, rcfg, hcfg, sbcfg),
            "patrol": lambda: Patrol(cfg, rcfg, hcfg, pcfg)}[name]()


# ---------------------------------------------------------------------------
def safety(cmd: Command, w: World, cfg: BehaviorCfg) -> Command:
    """Letzte Instanz: liegt/gefallen -> Stopp; Hindernis/TTC -> keine Vorwärtsfahrt."""
    if not w.balancing:
        return Command(0, 0, None, stopp_text(w.fw_mode), cmd.target_id)
    if w.blind:
        return Command(0, 0, None, "BLIND – keine Bodengrenze (FloorNet), Stopp", cmd.target_id)
    # Anschub des Wegfolgers (extra anschub): Freiraum und TTC sehen die Kante/Stufe, an der er hängt; seine
    # Strecke ist auf den freien Boden davor begrenzt (PathFollower._start_push)
    if cmd.v > 0 and not cmd.extra.get("anschub"):
        fs = w.free
        center = fs.center if fs and fs.valid else 1.0
        # Beim Folgen steht das Ziel selbst im Bild -> dessen Box nicht als Hindernis
        if center < cfg.blocked_free * 0.6:
            cmd.v = 0.0
            cmd.status += " | Freiraum!"
        elif center < cfg.slow_free:
            cmd.v *= clamp(center / cfg.slow_free, 0.25, 1.0)
        if w.ego.ttc < cfg.ttc_stop_s and w.cmd_v > 0.03:
            cmd.v = 0.0
            cmd.status += " | TTC %.1fs!" % w.ego.ttc
    return cmd
