"""Verhalten: Folgen, Erkunden, Anfahren – plus Sicherheitsschicht.

Alle Verhalten arbeiten auf einem gemeinsamen Weltzustand und liefern einen
normierten Fahrbefehl (v, w) und eine gewünschte Körperhöhe. Die
Sicherheitsschicht hat immer das letzte Wort.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .config import BehaviorCfg, RobotCfg
from .egomotion import EgoResult
from .freespace import FreeSpace
from .geometry import CamGeometry
from .tracker import Track


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
    vis_yaw: float                    # integrierter Kurs aus dem Bild (rad, links +)
    cmd_v: float                      # aktuell gefahrener Befehl (nach Rampe)
    geo: CamGeometry | None = None
    pitch_dev: float = 0.0            # hochfrequentes Nicken in rad (aus Ego-Motion)

    def bearing(self, x: float) -> float:
        """Bildspalte -> Winkel rad (links positiv)."""
        f = (self.W / 2) / math.tan(self.hfov / 2)
        return -math.atan2(x - self.W / 2, f)


@dataclass
class Command:
    v: float = 0.0
    w: float = 0.0
    height_mm: float | None = None
    status: str = ""
    target_id: int | None = None
    extra: dict = field(default_factory=dict)


def yaw_rate_to_w(rate_rad_s: float, rcfg: RobotCfg) -> float:
    return math.degrees(rate_rad_s) / rcfg.vyaw_limit_dps


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ---------------------------------------------------------------------------
class Behavior:
    name = "base"

    def __init__(self, cfg: BehaviorCfg, rcfg: RobotCfg):
        self.cfg = cfg
        self.rcfg = rcfg

    def step(self, w: World) -> Command:
        raise NotImplementedError


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

    def set(self, v: float, w: float, now: float) -> None:
        self._v, self._w, self._t = clamp(v, -1.0, 1.0), clamp(w, -1.0, 1.0), now

    def step(self, wd: World | None) -> Command:      # braucht keine Weltdaten (auch ohne Kamera)
        import time as _time
        if _time.monotonic() - self._t > self.HOLD_S:
            return Command(0.0, 0.0, None, "Handsteuerung – steht")
        return Command(self._v, self._w, None, "Handsteuerung v=%+.2f w=%+.2f" % (self._v, self._w))


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
        w_cmd = yaw_rate_to_w(rate, self.rcfg)

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
        v = clamp(v, -self.cfg.backup_v, self.rcfg.max_v)
        # erst ausrichten, dann fahren
        if v > 0:
            v *= clamp(1.0 - abs(err) / math.radians(25), 0.0, 1.0)
        return v, w_cmd, err, self._dist


class Follow(Behavior):
    name = "follow"

    def __init__(self, cfg, rcfg, target_class: str | None = None, want_dist: float | None = None):
        super().__init__(cfg, rcfg)
        self.cls = target_class or cfg.target_class
        self.servo = Servo(cfg, rcfg, want_dist if want_dist is not None else cfg.follow_dist_m)
        self.target: int | None = None
        self.last_seen_t = -1e9
        self.last_side = 1.0
        self.search_start: float | None = None

    def pick(self, w: World) -> Track | None:
        cands = [t for t in w.tracks if t.cls == self.cls]
        cur = next((t for t in cands if t.id == self.target), None)
        if cur is not None and cur.lost_s(w.t) < self.cfg.lost_search_s:
            return cur
        if not cands:
            return None
        # größte (nächste) Instanz, Bildmitte bevorzugt
        best = max(cands, key=lambda t: t.h / w.H - 0.3 * abs(t.cx - w.W / 2) / w.W)
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
            fast = abs(v) > 0.6 * self.rcfg.max_v
            hmm = self.cfg.height_fast_mm if fast else self.cfg.height_cruise_mm
            st = "verfolge #%d %s (%+.0f°, %s)" % (tr.id, tr.cls, math.degrees(err),
                                                  "%.2f m" % dist if dist is not None else "? m")
            return Command(v, wc, hmm, st, tr.id)
        # verloren -> in Richtung der letzten Sichtung suchen
        if self.search_start is None:
            self.search_start = w.t
            self.servo.reset()
        if w.t - self.search_start > self.cfg.search_timeout_s:
            return Command(0, 0, self.cfg.height_cruise_mm, "Ziel verloren – warte")
        return Command(0.0, self.last_side * self.cfg.search_w, self.cfg.height_search_mm,
                       "suche %s (%s)" % (self.cls, "links" if self.last_side > 0 else "rechts"))


class Explore(Behavior):
    """Reaktives Erkunden: in Richtung des größten Freiraums, bei Blockade
    zurücksetzen und um einen zufälligen Winkel wegdrehen (Kurs aus Ego-Motion)."""
    name = "explore"

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
        blocked = center < self.cfg.blocked_free or (w.ego.ttc < self.cfg.ttc_avoid_s and w.cmd_v > 0.03)
        hmm = self.cfg.height_cruise_mm

        if self.state == "cruise":
            if blocked:
                self._enter("backup", w.t)
                return Command(0, 0, hmm, "Hindernis – stopp")
            v = self.cfg.cruise_v * clamp(center / self.cfg.slow_free, 0.3, 1.0)
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

        if self.state == "backup":
            if w.t - self.t_state < 0.8:
                return Command(-self.cfg.backup_v, 0, hmm, "zurücksetzen")
            if fs and fs.valid:
                n = len(fs.sectors)
                left, right = sum(fs.sectors[: n // 2]), sum(fs.sectors[(n + 1) // 2:])
                self.turn_dir = 1.0 if left >= right else -1.0
            else:
                self.turn_dir = self.rng.choice((-1.0, 1.0))
            self.turn_goal = w.vis_yaw + self.turn_dir * math.radians(self.rng.uniform(60, 140))
            self._enter("turn", w.t)

        if self.state == "turn":
            rem = (self.turn_goal - w.vis_yaw) * self.turn_dir
            if (rem <= 0 and center > self.cfg.slow_free) or w.t - self.t_state > 6.0:
                self._enter("cruise", w.t)
                return Command(0, 0, hmm, "weiter")
            return Command(0.0, self.turn_dir * self.cfg.search_w * 1.5, hmm,
                           "drehe weg (%+.0f° offen)" % math.degrees(rem * self.turn_dir))
        return Command(0, 0, hmm, self.state)


class Approach(Behavior):
    """Erkunden, bis ein Objekt der Zielklasse gesehen wird; dann anfahren und davor stoppen."""
    name = "approach"

    def __init__(self, cfg, rcfg, target_class: str | None = None):
        super().__init__(cfg, rcfg)
        self.cls = target_class or cfg.target_class
        self.follow = Follow(cfg, rcfg, self.cls, cfg.approach_dist_m)
        self.explore = Explore(cfg, rcfg)
        self.arrived_since: float | None = None
        self.done = False

    def step(self, w: World) -> Command:
        if self.done:
            return Command(0, 0, self.cfg.height_search_mm, "angekommen bei %s" % self.cls)
        seen = any(t.cls == self.cls for t in w.tracks)
        if not seen and self.follow.search_start is None and self.follow.target is None:
            c = self.explore.step(w)
            c.status = "suche %s: %s" % (self.cls, c.status)
            return c
        c = self.follow.step(w)
        if c.target_id is not None and abs(c.v) < 1e-3 and abs(c.w) < 0.02:
            self.arrived_since = self.arrived_since or w.t
            if w.t - self.arrived_since > 1.0:
                self.done = True
        else:
            self.arrived_since = None
        if self.follow.search_start is not None and w.t - self.follow.search_start > 4.0:
            # länger nicht gesehen -> wieder erkunden
            self.follow.target = None
            self.follow.search_start = None
        return c


def make_behavior(name: str, cfg: BehaviorCfg, rcfg: RobotCfg, target: str | None = None) -> Behavior:
    return {"idle": lambda: Idle(cfg, rcfg),
            "manual": lambda: Manual(cfg, rcfg),
            "follow": lambda: Follow(cfg, rcfg, target),
            "explore": lambda: Explore(cfg, rcfg),
            "approach": lambda: Approach(cfg, rcfg, target)}[name]()


# ---------------------------------------------------------------------------
def safety(cmd: Command, w: World, cfg: BehaviorCfg) -> Command:
    """Letzte Instanz: gefallen -> Stopp; Hindernis/TTC -> keine Vorwärtsfahrt."""
    if not w.balancing:
        return Command(0, 0, None, "GEFALLEN – Stopp", cmd.target_id)
    if cmd.v > 0:
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
