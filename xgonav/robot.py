"""Bewegungsschicht über /proc/XGORider (Kernelmodul xgo-drv).

Konventionen
------------
* ``drive(v, w)``: normiert auf den Vollausschlag der Firmware, Bereich [-1, 1].
  v > 0 vorwärts, w > 0 Linksdrehung (CCW), wie im RC-Client (128 + step).
* Sollwerte werden im Treiber/ESP32 *gehalten*. Deshalb gibt es hier
  - eine Rampe (Beschleunigungsbegrenzung, wichtig für den Balancierer),
  - einen Keepalive (periodisches Neuschreiben, für den Kernel-Watchdog),
  - einen Userspace-Watchdog: kommt länger kein drive(), wird gestoppt.
* Ohne Kernel-Patch hilft gegen SIGKILL/OOM nur der Kernel-Watchdog nicht;
  dann bleibt der letzte Sollwert stehen. Deshalb den Patch einspielen.
"""
from __future__ import annotations

import collections
import logging
import math
import os
import threading
import time
from dataclasses import dataclass

from .config import RobotCfg
from .fahrrahmen import W_LEVEL_OFF_DPS, Achse, grenzen_text, rahmen_text, zustand

log = logging.getLogger(__name__)

NEUTRAL = 128
# Höhe in der Firmware (xgorider2_rl main.cpp, action_engine.cpp): Solange der Rider nicht balanciert, setzt
# sie den Sollwert in jedem Takt auf 0; beim Aufstehen nimmt sie die Mittelstellung 128. Gefahren wird nur
# 55..200. /proc/XGORider/height zeigt dagegen den zuletzt geschriebenen Wert, auch über das Aufstehen hinweg.
FW_HEIGHT_STANDUP_U8 = 128
FW_HEIGHT_MIN_U8, FW_HEIGHT_MAX_U8 = 55, 200
# Totband der Firmware um 128 (rider_motion.cpp decode: 5 % des Bereichs); darunter fährt er nicht an
FW_DEADBAND = 0.05
# Aktionen der Firmware, die selbst fahren oder drehen (action_engine.cpp tick): je Abschnitt
# (ab Takt, forward u8, turn u8), dazu der Takt, in dem die Aktion endet. Ein Takt ist jeder 21. Durchlauf
# der Firmware-Schleife (260–333 Hz, also 63–81 ms; RobotCfg.action_tick_s ist die lange Grenze).
# Die Aktion setzt denselben Sollwert, den speed_x/speed_z schreiben: Jeder Keepalive hält sie an.
FW_ACTIONS: dict[int, tuple[tuple[tuple[int, int, int], ...], int]] = {
    3: (((0, 192, 128), (8, 64, 128)), 20),
    4: (((0, 176, 208), (14, 176, 48), (28, 96, 208), (44, 96, 48)), 60),
    5: (((0, 128, 32), (46, 128, 208)), 100),
}
FW_TICK_FAST = 0.75          # kürzester Takt als Anteil von action_tick_s (333 gegen 260 Hz, mit Reserve)
FW_ACTION_AHEAD_S = 0.3      # so weit voraus prüft die App die Abschnitte einer Aktion
FW_ACTION_RESERVE_S = 0.3    # Keepalive bleibt so lange über das gerechnete Ende hinaus aus


def action_cmds(n: int, elapsed_s: float, tick_s: float) -> list[tuple[float, float]]:
    """Fahrbefehle (v, w normiert) der Abschnitte von Aktion n, die elapsed_s nach dem Start laufen können
    oder in den nächsten FW_ACTION_AHEAD_S beginnen. Mehrere, weil die Taktdauer nur ungefähr bekannt ist."""
    segs, end = FW_ACTIONS[n]
    lo = elapsed_s / tick_s
    hi = (elapsed_s + FW_ACTION_AHEAD_S) / (tick_s * FW_TICK_FAST)
    out = []
    for i, (start, fwd, turn) in enumerate(segs):
        stop = segs[i + 1][0] if i + 1 < len(segs) else end
        if start <= hi and stop > lo:
            out.append((u8_to_norm(fwd), u8_to_norm(turn)))
    return out


def norm_to_u8(x: float) -> int:
    x = max(-1.0, min(1.0, x))
    return int(round(NEUTRAL + x * 127))


def u8_to_norm(u: int) -> float:
    return (u - NEUTRAL) / 127.0


# Zustand der Firmware (/proc/XGORider/state, xgorider2_rl main.cpp requestMode): abgesetzt (Aktion 254,
# StopReason::Manual, LEDs orange) und gekippt (0, StopReason::Tilt, LEDs rot) sind verschiedene Zustände.
FW_ZUSTAND: dict[int, str] = {1: "balanciert", 254: "liegt", 0: "gefallen", 253: "bereit",
                              250: "akku", 251: "aktorfehler", 252: "fehler"}


def lage(mode: int | None) -> str | None:
    """Firmware-Zustand -> Wort (FW_ZUSTAND); None = unbekannt."""
    if mode is None:
        return None
    return FW_ZUSTAND.get(mode, f"zustand {mode}")


def stopp_text(mode: int | None) -> str:
    """Begründung des Stopps, wenn er nicht balanciert: abgesetzt ist kein Sturz."""
    return "LIEGT – Stopp" if mode == 254 else "GEFALLEN – Stopp"


# StopReason der Firmware (runtime_guard.h), in der Reihenfolge der Aufzählung
FW_STOPGRUND = ("keiner", "Start", "Befehl", "Kalibrierung", "Kalibrierung ungültig", "IMU fehlt", "Aktoren fehlen",
                "gekippt", "Akku", "Taktlücke", "Reglerausgang ungültig", "RL", "freier Fall")
FW_SHUTDOWN_LEDS_OFF = 233   # Firmware 0028: 234 = LEDs blau (Pi fährt herunter), 233 = normal
LIFT_NAH_S = 3.0      # Auslösung so kurz vor dem Lesen gehört zu diesem Sturz


def lift_text(line: str) -> str | None:
    """Zeile aus /proc/XGORider/lift (Register 0x19, Firmware 0027, Aufbau main.cpp fillLiftDiag) -> Logtext;
    None, wenn der Treiber einen Fehler meldet (alte Firmware) oder die Zeile nicht passt."""
    try:
        kopf, _, rest = line.partition(":")
        d = bytes(int(x, 16) for x in rest.split())
    except ValueError:
        return None
    if not kopf.startswith("0x19") or len(d) != 32:
        return None
    u16 = lambda a: int.from_bytes(d[a:a + 2], "little")
    s16 = lambda a: int.from_bytes(d[a:a + 2], "little", signed=True)
    u32 = lambda a: int.from_bytes(d[a:a + 4], "little")
    grund = lambda g: FW_STOPGRUND[g] if g < len(FW_STOPGRUND) else f"Grund {g}"
    jetzt, n = u32(0), u16(4)
    stopp = "kein Stopp aus dem Balancieren" if u16(30) == 65535 else \
        f"letzter Stopp aus dem Balancieren: {grund(d[29])} vor {u16(30) / 100:.2f} s"
    if n == 0:
        return f"Hebeschutz: seit Start nie ausgelöst; {stopp}"
    alter = ((jetzt - u32(8)) & 0xFFFFFFFF) / 1000
    ende = {0: "greift noch", 1: "frei gegeben", 2: f"durch Stopp ({grund(d[28])})"}.get(d[6], f"Ende {d[6]}")
    griff = "griff" if d[7] & 2 else "nur Detektor (Hebeschutz aus)"
    wann = "VOR DIESEM STURZ" if alter <= LIFT_NAH_S else "nicht bei diesem Sturz"
    return (f"Hebeschutz {wann}: {griff} vor {alter:.2f} s, {u16(24)} Takte, {ende}; Rad {s16(12)}/{s16(14)} "
            f"(roh {s16(16)}/{s16(18)}, Spitze {s16(26)}), Befehl {s16(20) / 10:.1f}, Nick {s16(22) / 100:.2f}°; "
            f"{n}× seit Start; {stopp}")


@dataclass
class RobotState:
    balancing: bool = True      # Kurs kommt allein aus dem IMU-Strom (App.kurs_deg), nicht aus /proc yaw
    battery: int | None = None
    buttons: int = 0
    t: float = 0.0
    mode: int | None = None    # Wert aus /proc/XGORider/state (1 = balanciert, FW_ZUSTAND)

    @property
    def lage(self) -> str | None:
        return lage(self.mode)


class SpeedCtrl:
    """Geschwindigkeitsregelung über die Räder (RobotCfg.speed_ctrl): stellt den gesendeten Fahrwert nach,
    bis die Räder so schnell drehen wie verlangt. step() läuft im Regelschritt der App und liefert die
    Korrektur (normiert), die Robot beim Senden addiert."""

    def __init__(self, cfg: RobotCfg):
        self.cfg = cfg
        self.trim = 0.0
        self.v_soll = 0.0                    # m/s, über das Fenster gemittelt
        self.v_ist: float | None = None      # m/s aus den Rädern, None = keine frischen Raddaten
        self._soll: collections.deque[tuple[float, float]] = collections.deque()   # (t, gesendeter Sollwert)
        self._lag = 0.0                      # Sollwert hinter dem Trägheitsmodell (normiert)
        self._last_t: float | None = None
        self._dir = 0

    def step(self, now: float, v_nominal: float, v_ist: float | None) -> float:
        """v_nominal: Fahrwert nach Rampe und Faktoren, ohne Korrektur (normiert). v_ist: Radgeschwindigkeit
        (m/s) über dasselbe Fenster oder None (keine frischen Raddaten, nicht balancierend)."""
        c = self.cfg
        dt = min(0.2, now - self._last_t) if self._last_t is not None else 0.0
        self._last_t = now
        # Soll so träge wie der Rider (speed_ctrl_lag_s) und über dasselbe Fenster wie das Ist gemittelt:
        # sonst zählt jedes Anfahren als Fehler und die Korrektur lädt sich dabei auf
        tau = c.speed_ctrl_lag_s
        self._lag += (v_nominal - self._lag) * ((1.0 - math.exp(-dt / tau)) if tau > 0 else 1.0)
        h = self._soll
        h.append((now, self._lag))
        while h[0][0] < now - c.speed_ctrl_window_s:
            h.popleft()
        self.v_soll = sum(v for _, v in h) / len(h) * c.vx_limit
        self.v_ist = v_ist
        d = 1 if v_nominal > 0 else -1 if v_nominal < 0 else 0     # auch für numpy-Werte
        if v_ist is None or abs(v_nominal) < FW_DEADBAND or d != self._dir:
            self.trim = 0.0
        else:
            self.trim += (self.v_soll - v_ist) / c.vx_limit * dt / c.speed_ctrl_ti_s
            self.trim = max(-c.speed_ctrl_max, min(c.speed_ctrl_max, self.trim))
        self._dir = d
        return self.trim

    @property
    def v_modell(self) -> float:
        """Tempo (m/s), das der Rider laut Trägheitsmodell (speed_ctrl_lag_s) gerade anstrebt: Er setzt einen
        neuen Fahrwert erst nach 1-2 s um und beschleunigt so lange auf den alten hin."""
        return self._lag * self.cfg.vx_limit


class Robot:
    """Gemeinsame Logik (Rampe, Watchdog). Unterklassen implementieren _write_*."""

    def __init__(self, cfg: RobotCfg):
        self.cfg = cfg
        self.v_trim = 0.0                    # App: Korrektur der Geschwindigkeitsregelung (SpeedCtrl), sonst 0
        self._tempo = 1.0
        self.tempo = getattr(cfg, "tempo", 1.0)
        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._brake = False                  # Vollbremsung verlangt: Fahrwert ohne Rampe auf 0
        self._cur = (0.0, 0.0)
        self._sent: tuple[int, int] | None = None
        self._last_cmd_t = 0.0
        self._last_send_t = 0.0
        self._last_ramp_t = time.monotonic()
        self._height_lock = threading.Lock()
        self._height_mm: float | None = None      # gültiger Sollwert, auch von außen gesetzt (Web)
        self._height_cmd_mm: float | None = None  # zuletzt von xgonav selbst geschrieben
        self._height_text: str | None = None      # zuletzt gelesener Inhalt von /proc/.../height
        self._height_down = False                 # lag seit dem letzten Aufstehen
        self._seen_down = False              # erst nach einer Liegephase zählt Balancieren als Aufstehen
        self._balancing_since: float | None = None
        self._level_on_t: float | None = None
        self._standup_levelled = False
        self._level_request = False
        self._roll_cmd = 0                   # zuletzt geschriebener Rollbefehl (set_roll)
        self._drive_level_on = False         # Nivellieren beim Fahren gerade an
        self._straight_since: float | None = None
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        self.state = RobotState()
        self.ruhe_tick_s: float | None = None   # Ruhemodus: nur so oft Zustand lesen, nichts schreiben
        # Aktion der Firmware, die selbst fährt (FW_ACTIONS): (n, Start, Ende). Solange sie läuft, schreibt
        # _send nichts; nach action_abort bis zum Ende in jedem Durchlauf Stillstand.
        self._act: tuple[int, float, float] | None = None
        self._act_abort = False
        self._act_lock = threading.Lock()       # Start der Aktion gegen einen gerade laufenden Keepalive
        # Fahrrahmen (fahrrahmen.py): _setup schaltet ihn ein, wenn Treiber und Firmware ihn kennen. _gen_v/_gen_w
        # rechnen den Sollwert der Firmware nach (m/s bzw. °/s); _cur ist dann deren Sollwert, normiert.
        self.vel = False
        self._gen_v, self._gen_w = Achse(), Achse()
        self._frame = ""                        # Text des aktuellen Rahmens
        self._frame_sent: str | None = None
        self._limits_t = -1e9

    @property
    def v_max_mps(self) -> float:
        return self.cfg.max_v * self.cfg.vx_limit

    @property
    def w_max_dps(self) -> float:
        return self.cfg.max_w * self.cfg.vyaw_limit_dps

    def _write_limits(self, now: float) -> None:
        """v_max/w_max und Wächter an die Firmware (Fahrrahmen); periodisch, ein ESP-Neustart verliert sie."""
        c = self.cfg
        try:
            self._write("vel_limits", grenzen_text(self.v_max_mps, self.w_max_dps, c.vel_timeout_ms, c.fw_speed_unit_mm))
        except OSError as e:
            log.error("vel_limits nicht geschrieben: %s", e)
        self._limits_t = now

    # ---- öffentliche API -------------------------------------------------
    def start(self) -> None:
        self._setup()
        self._thread = threading.Thread(target=self._loop, name="robot", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._emergency_stop()
        self._teardown()

    TEMPO_MIN = 0.05

    @property
    def tempo(self) -> float:
        """Anteil, mit dem jeder Fahrbefehl nach der Begrenzung malgenommen wird (Temporegler im Web)."""
        return self._tempo

    @tempo.setter
    def tempo(self, f: float) -> None:
        f = float(f)
        if not (self.TEMPO_MIN <= f <= 1.0):
            raise ValueError(f"Tempo {f} außerhalb {self.TEMPO_MIN}..1")
        self._tempo = f

    def drive(self, v: float, w: float, brake: bool = False) -> None:
        """brake (nur mit v = 0): Fahrwert sofort auf 0 statt über die Rampe; der Rider bremst nur auf 0 schnell,
        Zwischenwerte der Rampe setzt er erst nach ~1-2 s um (Bahnprüfung, Fahrt 2026-11-23)."""
        # Erst begrenzen, dann Tempo: v/w der Modi sind Faktor x max_v (RobotCfg.v_of), gesendet wird
        # Faktor x tempo x max_v; Verhältnis v/w und damit die geprüfte Bahn bleiben gleich.
        v = max(-self.cfg.max_v, min(self.cfg.max_v, v)) * self._tempo
        w0 = w
        w = max(-self.cfg.max_w, min(self.cfg.max_w, w)) * self._tempo
        # Drehen auf der Stelle: Das Tempo darf einen Drehwert ab turn_min_w nicht unter diese Untergrenze
        # drücken, sonst steht er im Totband der Firmware (Fahrt 20261124-134052: Tempo 0,75, gesendet 0,045,
        # „drehe (-12°)“ ohne Drehung bis „Ziel gesperrt“, viermal hintereinander)
        lo = self.cfg.turn_min_w
        if not self.vel and v == 0.0 and lo > 0 and abs(w0) >= lo and abs(w) < lo:   # Fahrrahmen: kein Totband
            w = math.copysign(lo, w)
        with self._lock:
            self._target = (v, w)
            self._brake = brake and v == 0.0
            self._last_cmd_t = time.monotonic()

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def set_down(self) -> None:
        """Absetzen (Firmware-Aktion 254)."""
        self.stop()
        self.action_abort()
        self._write("action", 254)

    @property
    def action(self) -> tuple[int, float, float] | None:
        """Laufende, nicht abgebrochene Fahr-Aktion der Firmware: (n, Start, Ende; time.monotonic) oder None."""
        a = self._act
        return None if a is None or self._act_abort else a

    def action_start(self, n: int) -> float:
        """Fahr-Aktion der Firmware (FW_ACTIONS) starten: Keepalive und Kernel-Watchdog aus, bis sie nach
        Rechnung vorbei ist (die Firmware setzt den Fahrwert am Ende selbst auf Stillstand). Ob die Bahn frei
        ist, prüft die App vorher und in jedem Regelschritt (App._action_tick). Rückgabe: Ende."""
        _, end = FW_ACTIONS[n]
        self.stop()
        self._gen_reset()                        # Fahrrahmen: die Aktion beendet den Rahmenweg der Firmware
        with self._act_lock:
            now = time.monotonic()
            t_end = now + end * self.cfg.action_tick_s + FW_ACTION_RESERVE_S
            self._kernel_watchdog(0)             # schriebe sonst nach 1 s ohne Keepalive selbst 128/128
            self._act, self._act_abort = (n, now, t_end), False
            try:
                self._write("action", n)
            except OSError:
                self._action_end()
                raise
        log.info("Aktion %d der Firmware: Keepalive für %.1f s aus", n, t_end - now)
        return t_end

    def action_abort(self) -> None:
        """Laufende Fahr-Aktion anhalten: Die Firmware kennt dafür über /proc keinen Befehl, deshalb schreibt
        _send bis zu ihrem Ende in jedem Durchlauf Stillstand (ihr Fahrwert gilt dann höchstens 20 ms)."""
        if self._act is not None and not self._act_abort:
            self._act_abort = True
            log.warning("Aktion %d der Firmware abgebrochen", self._act[0])

    def _action_end(self) -> None:
        self._act, self._act_abort = None, False
        self._sent = None                        # nächster Durchlauf schreibt sofort wieder
        self._gen_reset()                        # Fahrrahmen: die Aktion hat den Rahmenweg der Firmware beendet
        self._kernel_watchdog(self.cfg.kernel_watchdog_ms)

    def _kernel_watchdog(self, ms: int) -> None:
        if self._read_text("settings/motion_timeout_ms") is not None:      # Treiber ohne den Eintrag: nichts
            self.write_setting("motion_timeout_ms", ms)

    def _action_tick(self, now: float) -> bool:
        """Im Durchlauf der Schleife; True, solange die Aktion ungestört läuft (dann schreibt _send nichts)."""
        a = self._act
        if a is None:
            return False
        if now >= a[2]:
            self._action_end()
            return False
        if not self.state.balancing:
            self.action_abort()
        if self._act_abort:
            self._sent = None                    # Stillstand in jedem Durchlauf neu schreiben
            return False
        return True

    def read_setting(self, name: str) -> str | None:
        """Treiber-Einstellung /proc/XGORider/settings/<name> lesen; None, wenn es sie nicht gibt."""
        return self._read_text("settings/" + name)

    def write_setting(self, name: str, value: int | str) -> bool:
        try:
            self._write("settings/" + name, value)
            return True
        except OSError as e:
            log.warning("settings/%s nicht gesetzt: %s", name, e)
            return False

    def stand_up(self) -> bool:
        """Aufspringen aus dem Liegen (Firmware-Aktion 253); er rollt dabei ~20 cm vor. Nur vom Modus pet
        nach dessen Prüfungen benutzt; an einen balancierenden Rider nie."""
        if self.state.balancing:
            return False
        self.stop()
        self._write("action", 253)
        return True

    def set_roll(self, deg: float) -> None:
        """Rollwinkel (Grad, −20..20); nicht während eines Nivellier-Impulses, der setzt ihn selbst."""
        if self._level_on_t is None and self.state.balancing:
            self._roll_cmd = int(round(max(-20.0, min(20.0, deg))))
            self._write("roll", self._roll_cmd)

    def set_height(self, mm: float) -> None:
        """Schreibt nur, wenn sich der eigene Sollwert ändert. Eine Höhe von außen (Web) bleibt so stehen,
        bis das Verhalten eine andere will; die Geometrie folgt ihr über height_mm."""
        span = self.cfg.height_max_mm - self.cfg.height_min_mm
        limit_mm = self.cfg.height_min_mm + span * min(self.cfg.height_limit_u8, 255) / 255
        mm = max(self.cfg.height_min_mm, min(limit_mm, mm))
        with self._height_lock:
            if self._height_cmd_mm is not None and abs(mm - self._height_cmd_mm) < 1.0:
                return
            u = int(round(255 * (mm - self.cfg.height_min_mm) / span))
            self._write("height", u)
            self._height_cmd_mm = mm
            self._height_mm = self.height_u8_to_mm(u)

    def height_u8_to_mm(self, u: int) -> float:
        """Sollwert u8 -> Körperhöhe (mm), mit der Begrenzung der Firmware."""
        u = max(FW_HEIGHT_MIN_U8, min(FW_HEIGHT_MAX_U8, u))
        return self.cfg.height_min_mm + (self.cfg.height_max_mm - self.cfg.height_min_mm) * u / 255

    def _height_tick(self) -> None:
        """Nach dem Aufstehen gilt die Mittelstellung der Firmware, nicht der alte Wert in /proc. Das
        Verhalten schreibt seine Höhe danach neu (sonst fuhr er mit 90 mm, während die Geometrie 85 annahm)."""
        if not self.state.balancing:
            self._height_down = True
            return
        if not self._height_down:
            return
        self._height_down = False
        with self._height_lock:
            self._height_mm = self.height_u8_to_mm(FW_HEIGHT_STANDUP_U8)
            self._height_cmd_mm = None
        log.info("Aufgestanden: Körperhöhe %.1f mm (Mittelstellung der Firmware)", self._height_mm)

    @property
    def height_mm(self) -> float | None:
        """Gültiger Höhen-Sollwert (mm), None solange keiner bekannt ist. Das ist der Sollwert, nicht die
        gemessene Beinstellung: Nach einer Änderung brauchen die Beine noch etwa eine Sekunde."""
        return self._height_mm

    @property
    def command(self) -> tuple[float, float]:
        """Aktuell (nach Rampe) gesendeter Sollwert, normiert."""
        return self._cur

    @property
    def v_nominal(self) -> float:
        """Fahrwert nach der Rampe, ohne die Korrektur der Geschwindigkeitsregelung. Mal vx_limit ist das die
        verlangte Geschwindigkeit."""
        return self._cur[0]

    def with_trim(self, v: float) -> float:
        """v plus Korrektur der Geschwindigkeitsregelung. Unter dem Totband der Firmware bleibt v, wie es
        ist (Stillstand bleibt 128), und die Korrektur kehrt die Richtung nie um."""
        t = self.v_trim
        if self.vel or t == 0.0 or abs(v) < FW_DEADBAND:   # Fahrrahmen: Firmware regelt das Tempo selbst
            return v
        return max(0.0, v + t) if v > 0 else min(0.0, v + t)

    # ---- interne Schleife (50 Hz) ---------------------------------------
    def _loop(self) -> None:
        next_poll = 0.0
        while not self._stop_evt.wait(self.ruhe_tick_s or 0.02):
            if self.ruhe_tick_s:
                # Ruhemodus: liegt, kein Keepalive und keine Rampe; Zustand und Akku fürs Web
                self._poll_state()
                self._height_tick()
                self._level_tick(time.monotonic())
                self._action_tick(time.monotonic())
                continue
            now = time.monotonic()
            with self._lock:
                target = self._target
                stale = now - self._last_cmd_t > self.cfg.stall_timeout_s
            if stale:
                target = (0.0, 0.0)
            if now >= next_poll:
                self._poll_state()
                self._height_tick()
                self._level_tick(now)
                if self.vel and now - self._limits_t >= self.cfg.vel_limits_s:
                    self._write_limits(now)
                next_poll = now + 0.2
            if not self.state.balancing:
                target = (0.0, 0.0)
                if self.vel:
                    self._gen_reset(resend=False)   # Firmware beendet den Rahmenweg bei jedem Stopp
            if self._act is not None and self._act_abort:
                target = (0.0, 0.0)
                self._cur = (0.0, 0.0)
                if self.vel:
                    self._gen_reset()
            self._ramp(target, now)
            with self._act_lock:
                if not self._action_tick(now):
                    self._send(now)

    @property
    def levelled(self) -> bool:
        """Balanciert und die Roll-Kalibrierung nach dem Aufstehen (Nivellier-Impuls) ist vorbei bzw. nicht vorgesehen;
        kein Impuls läuft oder steht an. Startpunkt und Startbild warten darauf (Nutzer 2026-11-22)."""
        if not self.state.balancing or self._level_on_t is not None or self._level_request:
            return False
        return not (self.cfg.level_after_standup and self._seen_down and not self._standup_levelled)

    def request_level(self) -> bool:
        """Neu ausrichten (Knopf im Web); False, wenn er nicht balanciert."""
        if not self.state.balancing:
            return False
        if self._level_on_t is None:
            self._level_request = True
        return True

    def _level_tick(self, now: float) -> None:
        """Nivellier-Impuls: Rollbefehl auf 0, Nivellierung level_pulse_s lang an, wieder aus.
        Auslöser sind request_level und (einmal je Aufstehen) level_after_standup; nie während des
        Aufschwungs, der höchstens 2 s dauert. Startet xgonav, während er schon steht, ist das
        kein Aufstehen."""
        if not self.state.balancing:
            self._seen_down = True
            self._balancing_since = self._level_on_t = None
            self._standup_levelled = self._level_request = False
            self._drive_level_on, self._straight_since, self._roll_cmd = False, None, 0   # Firmware setzt beim Absetzen zurück
            return
        if self._balancing_since is None:
            self._balancing_since = now
        try:
            if self._level_on_t is not None:
                if now - self._level_on_t >= self.cfg.level_pulse_s:
                    if not self.cfg.level_always:
                        self._write("roll_balance", 0)
                    self._level_on_t = None
                    self._drive_level_on = self.cfg.level_always
                return
            standup = self.cfg.level_after_standup and self._seen_down and not self._standup_levelled
            if (standup or self._level_request) and now - self._balancing_since >= 2.5:
                self._write("roll", 0)
                self._roll_cmd = 0
                self._write("roll_balance", 1)
                self._level_on_t = now
                self._level_request = False
                self._standup_levelled = True
                self._straight_since = None
                return
            self._drive_level(now)
        except OSError as e:
            log.error("Nivellierung fehlgeschlagen: %s", e)

    def _drive_level(self, now: float) -> None:
        """Nivellieren beim Fahren (robot.level_while_driving): an nach level_drive_delay_s Geradeausfahrt, aus beim
        Drehen, Anhalten, Rollbefehl oder einer Aktion der Firmware. Der Trimm bleibt danach stehen (Patch 0012)."""
        if self.cfg.level_always:
            if not self._drive_level_on:
                self._write("roll_balance", 1)
                self._drive_level_on = True
                log.info("Nivellierung dauerhaft an (robot.level_always)")
            return
        v, w = self._cur
        straight = (self.cfg.level_while_driving and self._roll_cmd == 0 and self._act is None
                    and abs(v) >= FW_DEADBAND and abs(w) <= self.cfg.level_drive_w_max)
        if not straight:
            self._straight_since = None
        elif self._straight_since is None:
            self._straight_since = now
        on = straight and now - self._straight_since >= self.cfg.level_drive_delay_s
        if on != self._drive_level_on:
            self._write("roll_balance", 1 if on else 0)
            self._drive_level_on = on
            log.debug("Nivellieren beim Fahren %s", "an" if on else "aus")

    def _ramp(self, target: tuple[float, float], now: float) -> None:
        dt = max(1e-3, now - self._last_ramp_t)
        self._last_ramp_t = now
        if self.vel:
            self._ramp_vel(target, dt)
            return
        v, w = self._cur
        if self._brake and target[0] == 0.0:
            v = 0.0                          # Vollbremsung (drive(brake=True)): ohne Rampe
        dv = max(-self.cfg.accel_v * dt, min(self.cfg.accel_v * dt, target[0] - v))
        dw = max(-self.cfg.accel_w * dt, min(self.cfg.accel_w * dt, target[1] - w))
        # Bremsen darf schneller gehen als Beschleunigen
        if abs(target[0]) < abs(v):
            dv = max(-3 * self.cfg.accel_v * dt, min(3 * self.cfg.accel_v * dt, target[0] - v))
        self._cur = (v + dv, w + dw)

    def _ramp_vel(self, target: tuple[float, float], dt: float) -> None:
        """Fahrrahmen: Ziel in m/s und °/s, Beschleunigung je nach Richtung; der Generator rechnet nach, was die
        Firmware daraus macht (_cur), der Rahmen selbst trägt nur Ziel und Grenzen."""
        c = self.cfg
        tv, tw = target[0] * c.vx_limit, target[1] * c.vyaw_limit_dps
        g = self._gen_v
        if self._brake and target[0] == 0.0:
            a, j = c.vel_brake_mps2, c.vel_brake_jerk_mps3
        elif tv * g.x >= 0 and abs(tv) > abs(g.x):
            a, j = c.vel_accel_mps2, c.vel_jerk_mps3          # schneller in derselben Richtung (oder aus dem Stand)
        else:
            a, j = c.vel_decel_mps2, c.vel_jerk_mps3          # langsamer oder Richtungswechsel
        g.schritt(tv, a, j, dt)
        self._gen_w.schritt(tw, c.vel_alpha_dps2, c.vel_jerk_w_dps3, dt)
        self._cur = (g.x / c.vx_limit, self._gen_w.x / c.vyaw_limit_dps)
        self._frame = rahmen_text(tv, a, j, tw, c.vel_alpha_dps2, c.vel_jerk_w_dps3, c.fw_speed_unit_mm)
        self._frame_w = tw

    def _gen_reset(self, resend: bool = True) -> None:
        """Firmware hat den Rahmenweg beendet (Stopp, Aktion, nicht balancierend): Generator neu ab 0. resend:
        nächster Durchlauf schreibt den Rahmen sofort (nicht im Liegen, sonst schriebe er in jedem Durchlauf)."""
        self._gen_v, self._gen_w = Achse(), Achse()
        if resend:
            self._frame_sent = None

    def _send_vel(self, now: float) -> None:
        keepalive = now - self._last_send_t > self.cfg.keepalive_s
        if self._frame == self._frame_sent and not keepalive:
            return
        try:
            self._write("vel", self._frame)
            self._frame_sent = self._frame
            self._last_send_t = now
            if (self.cfg.level_always and self.state.balancing
                    and abs(getattr(self, "_frame_w", 0.0)) > W_LEVEL_OFF_DPS):
                self._write("roll_balance", 1)               # Firmware schaltet sie beim Drehen ab
        except OSError as e:
            log.error("Schreiben fehlgeschlagen: %s", e)

    def _send(self, now: float) -> None:
        # Abbruch einer Firmware-Aktion bleibt bei den Bytes: 128/128 in jedem Durchlauf (hält die Aktion an)
        if self.vel and not (self._act is not None and self._act_abort):
            self._send_vel(now)
            return
        # die Geschwindigkeitsregelung wirkt nach max_v;
        # begrenzt wird nur noch auf den Bereich der Firmware (robot.max_sent entfiel 2026-11-23, Nutzer)
        m = 1.0
        v = max(-m, min(m, self.with_trim(self.v_nominal)))
        u = (norm_to_u8(v), norm_to_u8(max(-m, min(m, self._cur[1]))))
        keepalive = now - self._last_send_t > self.cfg.keepalive_s
        prev = self._sent
        if prev == u and not keepalive:
            return
        try:
            if prev is None or keepalive or u[0] != prev[0]:
                self._write("speed_x", u[0])
            if prev is None or keepalive or u[1] != prev[1]:
                self._write("speed_z", u[1])
                if self.cfg.level_always and self.state.balancing and abs(u[1] - NEUTRAL) > 16:
                    self._write("roll_balance", 1)   # Firmware schaltet sie beim Drehbefehl ab
            self._sent = u
            self._last_send_t = now
        except OSError as e:
            log.error("Schreiben fehlgeschlagen: %s", e)

    def _emergency_stop(self) -> None:
        self._cur = (0.0, 0.0)
        for name in ("speed_x", "speed_z"):
            try:
                self._write(name, NEUTRAL)
            except OSError:
                pass

    # ---- Hardware-Hooks ---------------------------------------------------
    def _setup(self) -> None: ...
    def _read_text(self, name: str) -> str | None: return None
    def _teardown(self) -> None: ...
    def _write(self, name: str, value: int | str) -> None: raise NotImplementedError
    def _poll_state(self) -> None: ...


class ProcRobot(Robot):
    """Echter Rider über das Kernelmodul."""

    def __init__(self, cfg: RobotCfg):
        super().__init__(cfg)
        self.base = cfg.proc_dir
        self._fds: dict[str, int] = {}
        self.has_kernel_watchdog = False

    def _path(self, name: str) -> str:
        return os.path.join(self.base, name)

    def _setup(self) -> None:
        if not os.path.isdir(self.base):
            raise RuntimeError(f"{self.base} fehlt – ist xgo-drv geladen?")
        # Treiber vor 2026-11-17 hat noch settings/force_yaw (Kurs halten per Impulsen, kämpft gegen jede Drehung) -> aus
        if os.path.exists(self._path("settings/force_yaw")):
            self._write_raw("settings/force_yaw", "0")
        wd = self._path("settings/motion_timeout_ms")
        if os.path.exists(wd):
            self._write_raw("settings/motion_timeout_ms", str(self.cfg.kernel_watchdog_ms))
            self.has_kernel_watchdog = True
            log.info("Kernel-Motion-Watchdog aktiv (%d ms)", self.cfg.kernel_watchdog_ms)
        else:
            log.warning("Kernel ohne motion_timeout_ms – bei Absturz fährt der Rider weiter! "
                        "Treiber aus kernel_module/ bauen und laden.")
        self._emergency_stop()
        self._vel_setup()
        # Firmware 0028: LEDs aus dem Shutdown-Blau zurück (beim Herunterfahren des Pi setzt xgo-shutdown-led 234);
        # ältere Firmware kennt 233 nicht als Aktion und tut nichts
        self._write("action", FW_SHUTDOWN_LEDS_OFF)
        self._led_start()

    def _vel_setup(self) -> None:
        """Fahrrahmen einschalten, wenn robot.vel_frame, der Treiber /proc/XGORider/vel hat und die Firmware auf 0x1a
        antwortet (ab 0029); sonst bleibt es bei speed_x/speed_z."""
        self.vel = False
        if not self.cfg.vel_frame:
            return
        if not os.path.exists(self._path("vel")):
            log.warning("Treiber ohne /proc/XGORider/vel – Fahrbefehle als Bytes (speed_x/speed_z)")
            return
        st = zustand(self._read_text("vel") or "")
        if st is None:
            log.warning("Firmware antwortet nicht auf den Fahrrahmen (0x1a, vor Patch 0029) – Fahrbefehle als Bytes")
            return
        self._write_limits(time.monotonic())
        self.vel = True
        c = self.cfg
        log.info("Fahrrahmen aktiv: v_max %.2f m/s, w_max %.0f °/s, a %.2f/%.2f m/s², Wächter %d ms, %.2f mm/s je Einheit",
                 self.v_max_mps, self.w_max_dps, c.vel_accel_mps2, c.vel_decel_mps2, c.vel_timeout_ms, c.fw_speed_unit_mm)

    def gespeicherte_led(self) -> int:
        """Im Web gespeicherte LED-Helligkeit (0..255), sonst led_default."""
        try:
            with open(self.cfg.led_file) as f:
                return max(0, min(255, int(f.read().strip())))
        except (OSError, ValueError):
            return self.cfg.led_default

    def _led_start(self) -> None:
        # Einmal beim Start: kurz volle Helligkeit als Zeichen, dann der gespeicherte Wert (setzt ihn auch nach
        # einem ESP-Neustart wieder, den rc.local nicht sieht). Kein Aufwand je Bild.
        if not os.path.exists(self._path("settings/led_brightness")):
            log.info("Treiber ohne settings/led_brightness – LED-Helligkeit bleibt")
            return
        n = self.gespeicherte_led()
        if self.cfg.led_start_s <= 0 or not self.write_setting("led_brightness", 255):
            self.write_setting("led_brightness", n)
            return
        t = threading.Timer(self.cfg.led_start_s, self.write_setting, ("led_brightness", n))
        t.daemon = True
        t.start()

    def _teardown(self) -> None:
        for fd in self._fds.values():
            try:
                os.close(fd)
            except OSError:
                pass
        self._fds.clear()

    def _write_raw(self, name: str, text: str) -> None:
        # Ein write() pro Wert; die Handler parsen den ganzen Puffer je Aufruf.
        fd = os.open(self._path(name), os.O_WRONLY)
        try:
            os.write(fd, text.encode())
        finally:
            os.close(fd)

    def _write(self, name: str, value: int | str) -> None:
        self._write_raw(name, str(value))

    def _read_text(self, name: str) -> str | None:
        try:
            with open(self._path(name)) as f:
                return f.read().strip()
        except OSError:
            return None

    def _read_int(self, name: str) -> int | None:
        try:
            with open(self._path(name)) as f:
                return int(f.read().strip() or 0)
        except (OSError, ValueError):
            return None

    def _poll_state(self) -> None:
        st = self._read_int("state")
        alt, self.state.mode = self.state.mode, st
        if st is not None:
            bal = st == 1
            if st != alt:
                if bal:
                    log.warning("Rider balanciert")
                elif st == 254:
                    log.warning("Rider liegt (abgesetzt)")
                elif st == 0:
                    log.warning("Rider GEFALLEN – Stopp")
                else:
                    log.warning("Rider %s (Zustand %d)", lage(st), st)
                if alt == 1 and st != 254:
                    self._lift_nach_sturz()
            self.state.balancing = bal
        self.state.battery = self._read_int("battery")
        self.state.buttons = self._read_int("buttons") or 0
        self._poll_height()
        self.state.t = time.monotonic()

    def _lift_nach_sturz(self) -> None:
        """Nach einem Sturz einmal das Protokoll des Hebeschutzes lesen (Firmware 0027, Treiber /proc/XGORider/lift)
        und loggen. Eigener Faden: das Lesen fragt den ESP32 (bis 0,1 s), die Schleife schreibt derweil weiter Stopps."""
        pfad = self._path("lift")
        if not os.path.exists(pfad):
            return

        def lesen() -> None:
            try:
                with open(pfad) as f:
                    text = lift_text(f.read().strip())
            except OSError:
                return
            if text:
                log.warning("%s", text)

        threading.Thread(target=lesen, name="lift", daemon=True).start()

    def _poll_height(self) -> None:
        """Höhen-Sollwert zurücklesen: Das Web (api.php) schreibt ihn direkt, ohne xgonav. Am 2026-11-07
        rechnete die Geometrie deshalb eine ganze Fahrt mit 128 statt 151 mm Kamerahöhe. Übernommen wird nur
        ein neu geschriebener Wert: Nach dem Aufstehen ist der alte Inhalt veraltet (_height_tick)."""
        with self._height_lock:                 # nicht zwischen Schreiben und Merken in set_height lesen
            try:
                with open(self._path("height")) as f:
                    text = f.read().strip()
            except OSError:
                return
            if not text.isdigit() or text == self._height_text:
                return                          # leer, solange seit dem Laden des Treibers keiner schrieb
            first = self._height_text is None
            self._height_text = text
            mm = self.height_u8_to_mm(int(text))
            if self._height_mm is not None and abs(mm - self._height_mm) < 0.5:
                return
            self._height_mm = mm
        if first:
            log.info("Körperhöhe laut Treiber: %.1f mm (u8 %s)", mm, text)
        elif self._height_cmd_mm is None or abs(mm - self._height_cmd_mm) >= 1.0:
            log.warning("Körperhöhe von außen: %.1f mm (u8 %s), Geometrie folgt", mm, text)


class SimRobot(Robot):
    """Für Replay/Tests: integriert Kurs aus w und protokolliert Kommandos."""

    def __init__(self, cfg: RobotCfg, log_cmds: bool = False):
        super().__init__(cfg)
        self.log_cmds = log_cmds
        self.writes: list[tuple[float, str, int | str]] = []
        self._up_at: float | None = None     # Aufspringen läuft, balanciert ab dann
        self.state.mode = 1
        self.settings: dict[str, str] = {}   # Treiber-Einstellungen (settings/<name>), für Tests vorbelegbar
        self.vel = cfg.vel_frame             # wie eine Firmware ab 0029 mit passendem Treiber

    def _setup(self) -> None:
        if self.vel:
            self._write_limits(time.monotonic())

    def _read_text(self, name: str) -> str | None:
        if name.startswith("settings/"):
            return self.settings.get(name[9:])
        return None

    def _write(self, name: str, value: int | str) -> None:
        self.writes.append((time.monotonic(), name, value))
        if name.startswith("settings/"):
            self.settings[name[9:]] = str(value)
        if len(self.writes) > 10000:
            del self.writes[:5000]
        if self.log_cmds:
            log.info("SIM %s <- %s", name, value)
        if name == "action" and value == 254:
            self.state.balancing, self.state.mode = False, 254
            self._up_at = None
        elif name == "action" and value == 253 and not self.state.balancing:
            self._up_at = time.monotonic() + 1.0

    def _poll_state(self) -> None:
        now = time.monotonic()
        if self._up_at is not None and now >= self._up_at:
            self.state.balancing, self.state.mode = True, 1
            self._up_at = None
        self.state.battery = 100
        self.state.t = now


def make_robot(kind: str, cfg: RobotCfg) -> Robot:
    if kind == "proc":
        return ProcRobot(cfg)
    if kind == "sim":
        return SimRobot(cfg, log_cmds=False)
    raise ValueError(kind)
