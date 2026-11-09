"""Verhalten: Folgen, Erkunden, Anfahren, Heimfahrt – plus Sicherheitsschicht.

Alle Verhalten arbeiten auf einem gemeinsamen Weltzustand und liefern einen
normierten Fahrbefehl (v, w) und eine gewünschte Körperhöhe. Die
Sicherheitsschicht hat immer das letzte Wort.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .config import BehaviorCfg, ExploreCfg, HomeCfg, PatrolCfg, RobotCfg
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
    yaw: float                        # Kurs (rad, links +): Räder, sonst Bild ([odom] yaw_source)
    cmd_v: float                      # aktuell gefahrener Befehl (nach Rampe)
    geo: CamGeometry | None = None
    pitch_dev: float = 0.0            # hochfrequentes Nicken in rad (aus Ego-Motion)
    pose: tuple | None = None         # (x, y, th) in der Raumkarte (m, rad), None = keine Karte
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
    fw_mode: int | None = None        # /proc/XGORider/state
    height_mm: float | None = None    # Höhen-Sollwert
    map_ok: bool = False              # Merkkarte vorhanden

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

        if self.state == "brake":
            # Der Rider braucht ~0,6 s zum Anhalten (2026-11-05); direkt von vor auf zurück
            # brächte ihn stark ins Nicken
            if w.t - self.t_state < self.BRAKE_S:
                return Command(0, 0, hmm, "Hindernis – stopp")
            self._enter("backup", w.t)

        if self.state == "backup":
            if w.t - self.t_state < 0.8:
                return Command(-self.cfg.backup_v, 0, hmm, "zurücksetzen")
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
            return Command(0.0, self.turn_dir * self.cfg.turn_w, hmm,
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


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


STUCK_PROGRESS_M = 0.3    # so viel näher ans Ziel, dann zählt früheres Festfahren nicht mehr


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

    def __init__(self, cfg: BehaviorCfg, rcfg: RobotCfg, hcfg: HomeCfg):
        self.cfg, self.rcfg, self.h = cfg, rcfg, hcfg
        self.path: list[tuple[float, float]] | None = None
        self.stuck = False
        self.pressing = False
        self._press: tuple | None = None
        self._turning = False
        self._anchor: tuple[float, float] | None = None
        self._t_prog = 0.0
        self._err_ref = 0.0

    def set_path(self, path) -> None:
        self.path = list(path) if path else None

    def clear(self) -> None:
        self.path = None
        self.stuck = False
        self.pressing = False
        self._press = None
        self._anchor = None

    def target(self, x: float, y: float) -> tuple[float, float]:
        """Punkt lookahead_m entlang des Wegs ab der Projektion der Pose."""
        p = self.path
        if len(p) == 1:
            return p[0]
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
        rest = self.h.lookahead_m
        i, t = bi, bt
        while i < len(p) - 1:
            ax, ay = p[i]
            bx, by = p[i + 1]
            L = math.hypot(bx - ax, by - ay)
            left = (1.0 - t) * L
            if left >= rest and L > 1e-9:
                t += rest / L
                return ax + t * (bx - ax), ay + t * (by - ay)
            rest -= left
            i, t = i + 1, 0.0
        return p[-1]

    def _watch_press(self, t: float, x: float, y: float, cmd_v: float) -> None:
        if cmd_v <= self.PRESS_V:
            self._press = None
        elif self._press is None or math.hypot(x - self._press[0], y - self._press[1]) >= self.PRESS_M:
            self._press = (x, y, t)
        elif t - self._press[2] > self.PRESS_S:
            self.stuck = self.pressing = True
            self._press = None

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

    def drive(self, w: World, label: str) -> Command:
        """Fahrbefehl entlang des Wegs; label steht vorn im Status."""
        hmm = self.cfg.height_cruise_mm
        x, y, th = w.pose
        self._watch_press(w.t, x, y, w.cmd_v)
        tx, ty = self.target(x, y)
        err = _wrap(math.atan2(ty - y, tx - x) - th)
        # Auf der Stelle drehen nur bei großem Fehler (Hysterese), sonst in der Kurve weiterfahren
        if self._turning:
            self._turning = abs(err) > math.radians(self.h.turn_exit_deg)
        else:
            self._turning = abs(err) > math.radians(self.h.turn_enter_deg)
        self._watch(w.t, x, y, err)
        wc = clamp(yaw_rate_to_w(self.h.kp_turn * err, self.rcfg), -1.0, 1.0)
        if self._turning:
            if abs(wc) < self.cfg.search_w:          # kleine Werte ignoriert die Firmware
                wc = math.copysign(self.cfg.search_w, err)
            return Command(0.0, wc, hmm, "%s – drehe (%+.0f°)" % (label, math.degrees(err)))
        if abs(wc) < self.h.min_w and abs(err) > math.radians(self.h.min_w_err_deg):
            wc = math.copysign(self.h.min_w, err)
        v = self.h.v * clamp(1.0 - abs(err) / math.radians(90), 0.3, 1.0)
        return Command(v, wc, hmm, "%s (%+.0f°)" % (label, math.degrees(err)))

    def align(self, err: float, label: str) -> Command:
        """Auf der Stelle auf einen Endkurs drehen (err rad)."""
        wc = clamp(yaw_rate_to_w(self.h.kp_turn * err, self.rcfg), -1.0, 1.0)
        if abs(wc) < self.cfg.search_w:              # kleine Werte ignoriert die Firmware
            wc = math.copysign(self.cfg.search_w, err)
        return Command(0.0, wc, self.cfg.height_cruise_mm, "%s (%+.0f°)" % (label, math.degrees(err)))


class Home(Behavior):
    """Zurück zum Startpunkt der Raumkarte: Weg plant planner.py (App stößt an, sobald
    want_plan() es verlangt), gefahren wird mit PathFollower. Sicherheitsschicht und
    Bahnprüfung gelten wie bei explore. Am Ziel optional auf den Startkurs drehen."""
    name = "home"

    def __init__(self, cfg, rcfg, hcfg: HomeCfg | None = None):
        super().__init__(cfg, rcfg)
        self.h = hcfg or HomeCfg()
        self.f = PathFollower(cfg, rcfg, self.h)
        self.pending = False
        self.t_plan = -1e9
        self.fails = 0
        self.done = False
        self.gave_up = False
        self.keep_ok = False          # darf der Planer den bisherigen Weg behalten? (nicht nach Festfahren)
        self.stuck_n = 0              # Festfahren in Folge ohne Annäherung ans Ziel
        self.best_dist = math.inf
        self.t_best = None

    @property
    def path(self):
        return self.f.path

    # --- Planung (von App bedient) ---
    def want_plan(self, t: float) -> bool:
        if self.pending or self.done or self.gave_up:
            return False
        if self.f.path is None or t - self.t_plan > self.h.replan_s or self.f.stuck:
            self.keep_ok = self.f.path is not None and not self.f.stuck
            self.stuck_n += self.f.stuck
            self.pending, self.t_plan, self.f.stuck = True, t, False
            return True
        return False

    def request_plan(self, planner, grid, w: World, obstacles) -> int:
        return planner.request(w.pose[:2], w.goal[:2], keep=self.f.path if self.keep_ok else None, obstacles=obstacles)

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
        if dist < self.h.arrive_m:
            err = _wrap(gth - th)
            if self.h.align_heading and abs(err) > math.radians(self.h.align_tol_deg):
                return self.f.align(err, "am Start – richte aus")
            self.done = True
            return Command(0, 0, hmm, "am Start angekommen")
        # Steht das Ziel selbst hinter einem Hindernis, findet der Planer immer denselben Weg und die
        # Bahnprüfung bremst ihn immer wieder (RiderPi 2026-11-06, Klebemarke am Startpunkt): aufhören
        if self.stuck_n >= 1 and dist < self.h.near_m:
            self.done = True
            return Command(0, 0, hmm, "am Start angekommen (%.2f m daneben)" % dist)
        if dist < self.best_dist - STUCK_PROGRESS_M:
            self.best_dist, self.stuck_n, self.t_best = dist, 0, w.t
        if self.stuck_n >= self.h.max_fail and w.t - self.t_best > self.h.give_up_s:
            self.gave_up = True
        if self.gave_up:
            return Command(0, 0, hmm, "kein Weg zum Start – stopp")
        if self.f.path is None:
            return Command(0, 0, hmm, "plane Weg zum Start")
        return self.f.drive(w, "zum Start %.2f m" % dist)


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
        self.f = PathFollower(cfg, rcfg, self.h)
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
        self.clusters = self.avoided = 0
        self.no_goal = 0
        self.stucks = 0
        self.t_goal = 0.0
        self.wanders = 0
        self.home_fails = 0
        self.home_stuck = 0
        self.home_best = math.inf
        self.home_t_best = None
        self.reason = ""
        self.save_map = False              # App speichert die Karte, sobald das gesetzt ist
        self.done = False
        self._scan_th: float | None = None
        self._scan_sum = 0.0
        self._cleared = False
        self._face: tuple | None = None
        self._known_best = 0.0
        self._t_growth: float | None = None

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
        if self.f.path is None or self.f.stuck or t - self.t_plan > self.h.replan_s:
            self.keep_ok = self.f.path is not None and not self.f.stuck
            self.pending, self.t_plan = True, t
            return True
        return False

    def request_plan(self, planner, grid, w: World, obstacles) -> int:
        from .frontier import next_goal
        if self.state == "heim":
            self.home_stuck += self.f.stuck
            self.f.stuck = False
            return planner.request(w.pose[:2], w.goal[:2], keep=self.f.path if self.keep_ok else None, obstacles=obstacles)
        if not self._cleared:
            grid.clear_here(self.e.start_free_r_m)  # wo er steht, ist Boden; die Kamera sieht dort nichts
            self._cleared = True
        r = self.e.avoid_r_m
        avoid = [(x, y, r) for x, y in self.visited + self.blocked_goals]
        return planner.request_fn(next_goal, grid, w.pose, self.h, self.e, obstacles, avoid,
                                  self.frontier, self.f.path if self.keep_ok else None)

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
        if self.frontier is None or math.hypot(res.frontier[0] - self.frontier[0], res.frontier[1] - self.frontier[1]) > self.e.keep_dist_m:
            self.stucks = 0                         # neues Ziel
            self.t_goal = self.t_plan
            self.f.clear()
        self.frontier, self.goal = res.frontier, res.goal
        self.f.set_path(res.path)

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
        if self.t0 is None:
            self.t0 = w.t
        if self.state not in ("heim", "fertig"):
            if w.t - self.t0 > e.max_s:
                self._end("Zeitgrenze", w)
            elif w.battery is not None and 0 < w.battery <= e.min_battery:
                self._end("Akku", w)

        if self.state == "scan":
            if self._scan_th is not None:
                self._scan_sum += abs(_wrap(th - self._scan_th))
            self._scan_th = th
            if self._scan_sum >= 2 * math.pi:
                self.state = "ziel"
                return Command(0, 0, hmm, "Rundumblick fertig")
            return Command(0.0, e.scan_w, hmm, "Rundumblick %.0f°" % math.degrees(self._scan_sum))
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
            return Command(-self.cfg.backup_v, 0.0, hmm, "setze zurück")
        if self.state == "wander":
            if w.t >= self.until:
                self.state = "ziel"
                self.wander.state = "cruise"
                return Command(0, 0, hmm, "weiter")
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
            if dist < self.h.arrive_m:
                err = _wrap(gth - th)
                if self.h.align_heading and abs(err) > math.radians(self.h.align_tol_deg):
                    return self.f.align(err, "am Start – richte aus")
                self._finish()
                return Command(0, 0, hmm, self._final_status())
            if self.home_stuck >= 1 and dist < self.h.near_m:
                self._finish("am Start, %.2f m daneben" % dist)
                return Command(0, 0, hmm, self._final_status())
            if dist < self.home_best - STUCK_PROGRESS_M:
                self.home_best, self.home_stuck, self.home_t_best = dist, 0, w.t
            if self.home_stuck >= self.h.max_fail and w.t - self.home_t_best > self.h.give_up_s:
                self._finish("Start nicht erreicht, %.2f m entfernt" % dist)
                return Command(0, 0, hmm, self._final_status())
            if self.f.path is None:
                return Command(0, 0, hmm, "Karte fertig, plane Weg zum Start")
            return self.f.drive(w, "Karte fertig, zum Start %.2f m" % dist)

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
            self.stucks = 2 if timed_out or self.f.pressing else self.stucks + 1   # Kontakt: gleich zurück
            self.f.pressing = False
            if self.stucks == 1:
                self.keep_ok = False
                self.f.set_path(None)               # sofort neu planen, ohne den alten Weg
                return Command(0, 0, hmm, "komme nicht weiter, plane neu")
            if self.frontier is not None:
                self.blocked_goals.append((self.frontier[0], self.frontier[1]))
            self.stucks = 0
            self.frontier = self.goal = None
            self.f.clear()
            self.state, self.until = "backup", w.t + 0.8
            return Command(0, 0, hmm, "Ziel gesperrt")
        if self.f.path is None:
            return Command(0, 0, hmm, "suche Ziel (%d Stellen)" % self.clusters)
        if math.hypot(self.goal[0] - x, self.goal[1] - y) < e.reach_m:
            self.visited.append((self.frontier[0], self.frontier[1]))
            self._face = self.frontier
            self.frontier = self.goal = None
            self.f.clear()
            self.state, self.until = "face", w.t + 6.0
            return Command(0, 0, hmm, "Ziel erreicht")
        return self.f.drive(w, "erkunde (%d Stellen)" % self.clusters)

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
                return Command(-self.cfg.backup_v, 0.0, hmm, "setze zurück")
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
                  pcfg: PatrolCfg | None = None, petcfg=None) -> Behavior:
    """has_map: Raumkarte vorhanden; sonst erkundet explore nur reaktiv."""
    def pet():
        from .pet import Pet          # pet.py importiert dieses Modul
        return Pet(cfg, rcfg, petcfg)
    return {"idle": lambda: Idle(cfg, rcfg),
            "pet": pet,
            "manual": lambda: Manual(cfg, rcfg),
            "follow": lambda: Follow(cfg, rcfg, target),
            "explore": lambda: MapExplore(cfg, rcfg, hcfg, ecfg) if has_map else Explore(cfg, rcfg),
            "approach": lambda: Approach(cfg, rcfg, target),
            "home": lambda: Home(cfg, rcfg, hcfg),
            "patrol": lambda: Patrol(cfg, rcfg, hcfg, pcfg)}[name]()


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
