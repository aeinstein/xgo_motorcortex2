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

import logging
import math
import os
import threading
import time
from dataclasses import dataclass

from .config import RobotCfg
from .fahrrahmen import (CAP_KAPPA, CAP_KURS, CAP_KURSABS, CAP_STRECKE, KURSABS_VORAUS_DEG, W_LEVEL_OFF_DPS, Achse,
                         Bogen, Kurs, Strecke, bogen_schritt, kursabs_schritt, kursabs_text, wrap180,
                         bogen_text, grenzen_text, kurs_empfang, kurs_schritt, kurs_text, rahmen_text, strecke_empfang,
                         strecke_messen, strecke_schritt, strecke_text, zustand)

log = logging.getLogger(__name__)

NEUTRAL = 128
# Höhe in der Firmware (xgorider2_rl main.cpp, action_engine.cpp): Solange der Rider nicht balanciert, setzt
# sie den Sollwert in jedem Takt auf 0; beim Aufstehen nimmt sie die Mittelstellung 128. Gefahren wird nur
# 55..200. /proc/XGORider/height zeigt dagegen den zuletzt geschriebenen Wert, auch über das Aufstehen hinweg.
FW_HEIGHT_STANDUP_U8 = 128
FW_CURVE_LEAN_ON = 232          # Firmware 07-balance: Einlegen in Kurven an (231 aus), nach ESP-Neustart aus
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
FW_SHUTDOWN_LEDS_OFF = 233   # Firmware-Patch 06-leds: 234 = LEDs blau (Pi fährt herunter), 233 = normal
LIFT_NAH_S = 3.0      # Auslösung so kurz vor dem Lesen gehört zu diesem Sturz


def lift_text(line: str) -> str | None:
    """Zeile aus /proc/XGORider/lift (Register 0x19, Firmware-Patch 03-diagnostics, Aufbau main.cpp fillLiftDiag) -> Logtext;
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


class Robot:
    """Gemeinsame Logik (Rampe, Watchdog). Unterklassen implementieren _write_*."""

    def __init__(self, cfg: RobotCfg):
        self.cfg = cfg
        self._tempo = 1.0
        self.tempo = getattr(cfg, "tempo", 1.0)
        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._brake = False                  # Vollbremsung verlangt (Fahrrahmen: vel_brake statt vel_decel)
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
        # Krümmungsbetrieb (Fähigkeitsbit der Firmware und robot.vel_kappa): drive(kappa=…) schickt κ statt w
        self.caps = 0
        self.bogen = False
        self._target_k: float | None = None
        self._kappa_ziel: float | None = None      # κ des letzten Bogenrahmens: ein Halt bremst auf diesem Bogen
        # Strecke mit Halt (Fähigkeitsbit und robot.vel_strecke): drive(strecke=Rest m) schickt die Reststrecke mit
        self.strecke_an = False
        self._target_strecke: float | None = None
        self._strecke, self._strecke_q = Strecke(), None
        self._bogen = Bogen()
        # Kursziel (Fähigkeitsbit und robot.vel_kurs): drive(kurs=(Restfehler rad, w_max Anteil)) schickt ein Ziel Δψ,
        # die Firmware dreht mit Bremsvorausschau darauf. Neues Ziel (Wechselbit) nur, wenn sich das verlangte Ziel
        # gegen das laufende um mehr als KURS_NEU_DEG verschiebt. _gen_winkel: ∫ w_soll dt wie der Generator der Firmware.
        self.kurs_an = False
        self._target_kurs: tuple[float, float] | None = None
        self._kurs = Kurs()
        self._kurs_seq = False
        self._kurs_ziel = self._kurs_dpsi = 0.0
        self._gen_winkel = 0.0
        self._w_lim: float | None = None        # °/s: w_max in vel_limits während eines Kursziels (Obergrenze des Modus)
        # Absolutes Kursziel (Fähigkeitsbit bit4 und robot.vel_kursabs, 2026-11-26): drive(kurs=(Restfehler, w, Ziel im
        # Kurs der Firmware °)) schickt jeden Durchlauf nur das Ziel; ohne Ziel (kein IMU-Strom) gilt das Kursziel Δψ.
        # Der Nachbau dreht auf das Ziel im Winkel des Generators (_target_kurs[3], beim drive() aus dem Restfehler).
        self.kursabs_an = False
        self._kurs_abs = False
        self._kursabs_off = 0.0
        self._frame = ""                        # Text des aktuellen Rahmens
        self._frame_sent: str | None = None
        self._limits_t = -1e9
        self._vel_neu_t = time.monotonic()      # letzte Nachfrage, ob der Fahrrahmen (wieder) geht

    @property
    def v_max_mps(self) -> float:
        return self.cfg.max_v * self.cfg.vx_limit

    @property
    def w_max_dps(self) -> float:
        return self.cfg.max_w * self.cfg.vyaw_limit_dps

    @property
    def _w_max_jetzt(self) -> float:
        """w_max, das gerade in vel_limits steht: im Kursziel die Obergrenze des Modus (die Firmware dreht dort mit w_max)."""
        return self.w_max_dps if self._w_lim is None else min(self.w_max_dps, self._w_lim)

    def _write_limits(self, now: float) -> None:
        """v_max/w_max und Wächter an die Firmware (Fahrrahmen); periodisch, ein ESP-Neustart verliert sie."""
        c = self.cfg
        try:
            # mit absolutem Kursziel 18 Byte (alpha/Drehruck dafür); jedes 0x1b gilt ganz, 14 Byte nähmen sie weg
            abs_ = self.kursabs_an
            self._write("vel_limits", grenzen_text(self.v_max_mps, self._w_max_jetzt, c.vel_timeout_ms, c.fw_speed_unit_mm,
                                                  c.vel_ki, c.vel_i_max_deg, c.vel_kv,
                                                  c.vel_alpha_dps2 if abs_ else None, c.vel_jerk_w_dps3 if abs_ else None))
        except OSError as e:
            log.error("vel_limits nicht geschrieben: %s", e)
        self._limits_t = now

    def _caps_setup(self, caps: int) -> None:
        """Fähigkeitsbyte der Firmware (0x1a lesen, Byte 1): Krümmungsbetrieb nur mit Bit und robot.vel_kappa. Ältere
        Firmware (0) führte einen Krümmungsrahmen als Drehrate aus."""
        self.caps = caps
        self.bogen = bool(self.cfg.vel_kappa and caps & CAP_KAPPA)
        if self.cfg.vel_kappa and not self.bogen:
            log.warning("robot.vel_kappa an, aber die Firmware meldet keinen Krümmungsbetrieb (Fähigkeiten %d) – "
                        "Drehratenweg", caps)
        elif self.bogen:
            log.info("Krümmungsbetrieb aktiv (Fähigkeiten %d)", caps)
        self.kurs_an = bool(self.cfg.vel_kurs and caps & CAP_KURS)
        if self.cfg.vel_kurs and not self.kurs_an:
            log.warning("robot.vel_kurs an, aber Firmware oder Treiber melden kein Kursziel (Fähigkeiten %d) – "
                        "Drehratenweg", caps)
        elif self.kurs_an:
            log.info("Kursziel aktiv (Fähigkeiten %d)", caps)
        self.strecke_an = bool(self.cfg.vel_strecke and caps & CAP_STRECKE)
        if self.cfg.vel_strecke and not self.strecke_an:
            log.warning("robot.vel_strecke an, aber Firmware oder Treiber melden keine Strecke (Fähigkeiten %d)", caps)
        elif self.strecke_an:
            log.info("Strecke mit Halt aktiv (Fähigkeiten %d)", caps)
        abs_alt = self.kursabs_an
        self.kursabs_an = bool(self.cfg.vel_kursabs and caps & CAP_KURSABS)
        if self.cfg.vel_kursabs and not self.kursabs_an:
            log.warning("robot.vel_kursabs an, aber Firmware oder Treiber melden kein absolutes Kursziel (Fähigkeiten %d)",
                        caps)
        elif self.kursabs_an:
            log.info("Absolutes Kursziel aktiv (Fähigkeiten %d)", caps)
        if self.kursabs_an != abs_alt and self.vel:
            self._write_limits(time.monotonic())       # alpha/Drehruck dazu (18 Byte) bzw. wieder weg

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

    def drive(self, v: float, w: float, brake: bool = False, kappa: float | None = None,
              kurs: tuple[float, float] | None = None, strecke: float | None = None) -> None:
        """brake: Vollbremsung, beim Langsamerwerden mit robot.vel_brake statt vel_decel (Fahrrahmen; auch auf ein
        kleineres Tempo als 0, so rollt er weich bis an die Grenze der Bahnprüfung). Auf dem Byteweg ohne Wirkung.
        kappa: Bahnkrümmung (1/m, + = links) zum Befehl; mit self.bogen geht sie statt w an die Firmware (w = κ · v_soll
        dort, Grenzen begrenzen v statt w), sonst zählt w. kurs: (Restfehler zum Zielkurs in rad, + = links; höchster
        normierter Drehwert wie w, z. B. RobotCfg.w_of(Anteil)[, Ziel im Kurs der Firmware in ° oder None]) beim Drehen
        auf einen Kurs; mit self.kurs_an dreht die Firmware selbst darauf (Δψ), mit self.kursabs_an und Ziel auf den
        absoluten Kurs. strecke: Reststrecke (m) bis zum Halt, vom Aufrufer laufend nachgerechnet; mit
        self.strecke_an bremst die Firmware so, dass er dort steht (Stufe 3)."""
        # Erst begrenzen, dann Tempo: v/w der Modi sind Faktor x max_v (RobotCfg.v_of), gesendet wird
        # Faktor x tempo x max_v; Verhältnis v/w und damit die geprüfte Bahn bleiben gleich.
        v = max(-self.cfg.max_v, min(self.cfg.max_v, v)) * self._tempo
        w = max(-self.cfg.max_w, min(self.cfg.max_w, w)) * self._tempo
        with self._lock:
            self._target = (v, w)
            self._target_k = kappa if self.bogen else None
            self._target_kurs = None
            self._target_strecke = max(0.0, strecke) if strecke is not None and self.strecke_an else None
            if kurs is not None and (self.kurs_an or self.kursabs_an):
                fw = kurs[2] if len(kurs) > 2 else None
                if fw is None and self.KURSABS_OHNE_IMU and self.kursabs_an:
                    fw = math.inf                         # Simulator: Ziel im Winkel des Generators
                err = max(-KURSABS_VORAUS_DEG, min(KURSABS_VORAUS_DEG, math.degrees(kurs[0])))
                self._target_kurs = (kurs[0], max(0.0, min(self.cfg.max_w, kurs[1])) * self._tempo
                                     * self.cfg.vyaw_limit_dps, fw, self._gen_winkel + err)
                if fw is None and not self.kurs_an:
                    self._target_kurs = None              # absolut ohne Kurs der Firmware: Drehratenweg
            self._brake = brake
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
        return self.u8_to_mm(max(FW_HEIGHT_MIN_U8, min(FW_HEIGHT_MAX_U8, u)))

    def u8_to_mm(self, u: float) -> float:
        """Höhenbyte-Skala -> Körperhöhe (mm) ohne Begrenzung (gestellte Höhe aus dem Strom, 0..255)."""
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
        if getattr(self, "_ist_da", False):
            return                              # Höhe kommt aus dem Strom
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
                kappa = self._target_k
                kurs = self._target_kurs
                strecke = self._target_strecke
                stale = now - self._last_cmd_t > self.cfg.stall_timeout_s
            if stale:
                target = (0.0, 0.0)
            if now >= next_poll:
                self._poll_state()
                self._height_tick()
                self._level_tick(now)
                if self.vel and now - self._limits_t >= self.cfg.vel_limits_s:
                    self._write_limits(now)
                self._vel_nachpruefen(now)
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
            if stale or not self.state.balancing or (self._act is not None and self._act_abort):
                kappa = kurs = strecke = None       # Stopp von hier: Drehratenweg wie bisher
            self._ramp(target, now, kappa, kurs, strecke)
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
        Drehen, Anhalten, Rollbefehl oder einer Aktion der Firmware. Der Trimm bleibt danach stehen (Firmware-Patch 07-balance)."""
        if self.cfg.level_always:
            if not self._drive_level_on:
                # Nivellieren auf den scheinbaren Schwerevektor = Einlegen in Kurven (Firmware 07-balance, Aktion 232;
                # nach jedem ESP-Neustart aus, wirkt nur beim Nivellieren). Entweder-oder (Nutzer 2026-11-27): solange
                # das gilt, nimmt die Firmware keinen Roll von außen (Web); ohne level_always geht der Roll wieder.
                self._write("roll_balance", 1)
                self._write("action", FW_CURVE_LEAN_ON)
                self._drive_level_on = True
                log.info("Nivellierung dauerhaft an mit Einlegen in Kurven (robot.level_always)")
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

    def _ramp(self, target: tuple[float, float], now: float, kappa: float | None = None,
              kurs: tuple[float, float] | None = None, strecke: float | None = None) -> None:
        dt = max(1e-3, now - self._last_ramp_t)
        self._last_ramp_t = now
        if self.vel:
            self._ramp_vel(target, dt, kappa if self.bogen else None, kurs if self.kurs_an else None,
                           strecke if self.strecke_an else None)
            return
        self._cur = target                   # Byteweg: die Firmware filtert selbst (Tiefpass, 5er-Schritt)

    # Neues Ziel (Wechselbit) nur, wenn keins läuft oder das laufende angekommen ist und der Aufrufer noch mehr als
    # KURS_NEU_DEG daneben sieht (er will woanders hin). Nie während die Firmware dreht: Der Restfehler kommt vom Kurs
    # in xgonav, der dem Generator nachläuft (am Rider 2026-11-25 über 20° bei 90 °/s); jede Schwelle darauf schob das
    # Ziel weiter (2°: 98° statt 90°, 20°: 105°). Den Rest nach der Ankunft holt der Kurs-Regler der Firmware auf,
    # solange Kursziel-Rahmen kommen (3c9f6fc0).
    KURS_NEU_DEG = 15.0
    KURSABS_OHNE_IMU = False   # SimRobot: absolutes Kursziel ohne Kurs der Firmware (Winkel des Generators ist der Kurs)

    STRECKE_STUFE_M = 0.005   # Reststrecke im Rahmen auf 5 mm gerundet: sonst jeder Durchlauf ein neuer Rahmen
    STRECKE_NEU_M = 0.10      # Rest wächst um mehr: neues Ziel, sonst bleibt der kleinere

    def _ramp_vel(self, target: tuple[float, float], dt: float, kappa: float | None = None,
                  kurs: tuple[float, float] | None = None, strecke: float | None = None) -> None:
        """Fahrrahmen: Ziel in m/s und °/s, Beschleunigung je nach Richtung; der Generator rechnet nach, was die
        Firmware daraus macht (_cur), der Rahmen selbst trägt nur Ziel und Grenzen. kappa: Krümmungsrahmen
        (bogen_schritt wie velocity_cmd.h tickKappa, w = κ · v_soll). kurs: Kursziel-Rahmen (kurs_schritt wie
        tickCourse); w_max des Modus geht dabei über vel_limits."""
        c = self.cfg
        tv, tw = target[0] * c.vx_limit, target[1] * c.vyaw_limit_dps
        g = self._gen_v
        schneller = tv * g.x >= 0 and abs(tv) > abs(g.x)
        if self._brake and not schneller:
            a, j = c.vel_brake_mps2, c.vel_brake_jerk_mps3
        elif schneller:
            a, j = c.vel_accel_mps2, c.vel_jerk_mps3          # schneller in derselben Richtung (oder aus dem Stand)
        else:
            a, j = c.vel_decel_mps2, c.vel_jerk_mps3          # langsamer oder Richtungswechsel
        # w_max über vel_limits: vor dem Kursziel-Rahmen schreiben (hier, _send folgt) und während eines Kursziels nie
        # senken: die Firmware klemmt w sofort auf w_max, ein Sprung ohne alpha (Firmware-Sitzung 2026-11-25). Also beim
        # Start nicht unter die laufende Drehrate, danach nur anheben; zurück auf das volle w_max erst nach dem Kursziel.
        abs_ = kurs is not None and self.kursabs_an and kurs[2] is not None
        w_lim = None
        if kurs is not None and not abs_:
            w_lim = max(kurs[1], abs(self._gen_w.x))
            if self._kurs.an and self._w_lim is not None:
                w_lim = max(w_lim, self._w_lim)
            w_lim = min(w_lim, self.w_max_dps)
        if w_lim != self._w_lim:
            self._w_lim = w_lim
            self._write_limits(time.monotonic())
        if kurs is None and self._kurs.an:
            kurs_empfang(self._kurs, None)            # anderer Rahmen beendet das Kursziel
            self._kurs_abs = False
        if (kappa is None and kurs is None and target == (0.0, 0.0) and self._bogen.an and abs(g.x) > 1e-6
                and self._kappa_ziel is not None):
            # Halt nach einer Bogenfahrt: auf demselben Bogen bremsen (w = κ · v_soll) statt w sofort auf 0 und geradeaus
            # (Rider 2026-11-25); steht er, gilt wieder der Drehratenweg (w ist dann schon 0, kein Sprung)
            kappa = self._kappa_ziel
        self._kappa_ziel = kappa if kurs is None else None
        st = self._strecke
        sq = None
        if strecke is not None and kurs is None:
            # Strecke mit Halt: Nachbau wie die Firmware, gemessenes Tempo = Sollwert (der Pi kennt das Radtempo hier
            # nicht) und ohne Nachlauf (der Nachbau rollt nicht nach; mit lag hielte er 0,25 s × v zu früh); jeder neue
            # Rahmen setzt den Rest
            sq = round(strecke / self.STRECKE_STUFE_M) * self.STRECKE_STUFE_M
            if st.an and self._strecke_q is not None and self._strecke_q < sq < self._strecke_q + self.STRECKE_NEU_M:
                # nur kleiner werden lassen: ein größerer Rest weckt die Firmware nach dem Halt wieder auf (Pendeln der
                # Odometrie beim Balancieren, Rider 2026-11-25); erst ein Sprung über STRECKE_NEU_M ist ein neues Ziel
                sq = self._strecke_q
            strecke_messen(st, g.x, 1.0 if tv >= 0 else -1.0, 1000.0, dt)
            st.lag = 0.0
            if not st.an or sq != self._strecke_q:
                strecke_empfang(st, sq * 1000.0)
                self._strecke_q = sq
        elif st.an:
            strecke_empfang(st, None)
            self._strecke_q = None
        if abs_:
            # absolutes Kursziel: nur das Ziel, jeden Durchlauf; Fahrachse bremst auf 0 (velocity_cmd.h tickCourseAbs)
            fw = kurs[2] if math.isfinite(kurs[2]) else None
            if not (self._kurs.an and self._kurs_abs):
                self._kurs, self._kurs_abs = Kurs(an=True), True
                # Versatz Kurs der Firmware -> Winkel des Generators einmal beim Start; danach folgt der Nachbau dem
                # absoluten Ziel (sonst schöbe der nachlaufende Restfehler sein Ziel in jedem Durchlauf weiter)
                self._kursabs_off = wrap180(fw - kurs[3]) if fw is not None else 0.0
            ziel = kurs[3] if fw is None else kurs[3] + wrap180(fw - self._kursabs_off - kurs[3])
            g.schritt(0.0, a, j, dt)
            self._bogen.an = False
            kursabs_schritt(self._gen_w, self._kurs, ziel, self._gen_winkel, c.vel_alpha_dps2, c.vel_jerk_w_dps3,
                            min(kurs[1], self.w_max_dps), self.w_max_dps, dt)
            self._frame = kursabs_text(kurs[3] if fw is None else fw, min(kurs[1], self.w_max_dps))
            self._frame_w = self._gen_w.x
        elif kurs is not None:
            err = math.degrees(kurs[0])
            k = self._kurs
            if self._kurs_abs:
                k.an, self._kurs_abs = False, False   # relativ nach absolut: immer neues Ziel (wie die Firmware)
            if not k.an or (k.done and abs(err) > self.KURS_NEU_DEG):
                self._kurs_seq = not self._kurs_seq
                self._kurs_dpsi = round(err * 10.0) / 10.0    # wie im Rahmen (0,1°)
                self._kurs_ziel = self._gen_winkel + self._kurs_dpsi
                kurs_empfang(k, self._kurs_dpsi, self._kurs_seq)
            g.schritt(tv, a, j, dt)
            self._bogen.an = False
            kurs_schritt(self._gen_w, k, c.vel_alpha_dps2, c.vel_jerk_w_dps3, self._w_max_jetzt, dt)
            self._frame = kurs_text(self._kurs_seq, tv, a, j, self._kurs_dpsi, c.vel_alpha_dps2, c.vel_jerk_w_dps3,
                                    c.fw_speed_unit_mm)
            self._frame_w = self._gen_w.x
        elif kappa is not None:
            kq = round(kappa * 1000.0) / 1000.0       # wie im Rahmen (0,001 1/m)
            gw = self._gen_w
            gw.x = bogen_schritt(g, self._bogen, gw.x, tv, kq, a, j, c.vel_alpha_dps2, c.vel_jerk_w_dps3,
                                 self.v_max_mps, self.w_max_dps, dt, st=st if sq is not None else None)
            gw.a = 0.0
            if sq is not None:
                self._frame = strecke_text(tv, a, j, None, kq, c.vel_alpha_dps2, c.vel_jerk_w_dps3, c.fw_speed_unit_mm, sq)
            else:
                self._frame = bogen_text(tv, a, j, kq, c.vel_alpha_dps2, c.vel_jerk_w_dps3, c.fw_speed_unit_mm)
            self._frame_w = gw.x
        else:
            self._bogen.an = False
            if sq is not None:
                strecke_schritt(g, st, tv, a, j, dt, 1000.0)
                self._frame = strecke_text(tv, a, j, tw, None, c.vel_alpha_dps2, c.vel_jerk_w_dps3, c.fw_speed_unit_mm, sq)
            else:
                g.schritt(tv, a, j, dt)
                self._frame = rahmen_text(tv, a, j, tw, c.vel_alpha_dps2, c.vel_jerk_w_dps3, c.fw_speed_unit_mm)
            self._gen_w.schritt(tw, c.vel_alpha_dps2, c.vel_jerk_w_dps3, dt)
            self._frame_w = tw
        self._gen_winkel += self._gen_w.x * dt
        self._cur = (g.x / c.vx_limit, self._gen_w.x / c.vyaw_limit_dps)

    def _gen_reset(self, resend: bool = True) -> None:
        """Firmware hat den Rahmenweg beendet (Stopp, Aktion, nicht balancierend): Generator neu ab 0. resend:
        nächster Durchlauf schreibt den Rahmen sofort (nicht im Liegen, sonst schriebe er in jedem Durchlauf)."""
        self._gen_v, self._gen_w = Achse(), Achse()
        self._bogen = Bogen()
        self._kappa_ziel = None
        self._kurs = Kurs()
        self._kurs_abs = False
        self._strecke, self._strecke_q = Strecke(), None
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
        # begrenzt wird nur noch auf den Bereich der Firmware (robot.max_sent entfiel 2026-11-23, Nutzer)
        m = 1.0
        v = max(-m, min(m, self._cur[0]))
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
    def _vel_nachpruefen(self, now: float) -> None: ...


class ProcRobot(Robot):
    """Echter Rider über das Kernelmodul."""

    def __init__(self, cfg: RobotCfg):
        super().__init__(cfg)
        self.base = cfg.proc_dir
        self._fds: dict[str, int] = {}
        self.has_kernel_watchdog = False
        self._ist_t = -1.0                       # Körperhöhe aus dem Strom: zuletzt gelesen (monotonic)
        self._ist_mm: float | None = None
        self._ist_da = False                     # Strom hat schon eine Höhe geliefert

    def _path(self, name: str) -> str:
        return os.path.join(self.base, name)

    IST_CACHE_S = 0.01

    @property
    def height_mm(self) -> float | None:
        """Körperhöhe (mm). Sobald Treiber und Firmware sie im Strom gemeldet haben (/proc/XGORider/body_height,
        caps bit5, seit 2026-11-27), nur noch die gestellte Höhe (Bein A + B)/2 der Firmware, auch beim Einlegen in
        Kurven und nach dem Aufstehen; None, solange keine kommt (liegt, Strom steht). Vorher (alter Treiber oder
        alte Firmware) der Sollwert wie bisher."""
        now = time.monotonic()
        if now - self._ist_t >= self.IST_CACHE_S:
            self._ist_t = now
            try:
                with open(self._path("body_height")) as f:
                    p = f.read().split()
            except OSError:
                p = []
            self._ist_mm = self.u8_to_mm(int(p[1]) / 100.0) if len(p) == 2 else None
            if self._ist_mm is not None and not self._ist_da:
                self._ist_da = True
                log.info("Körperhöhe aus dem Strom: %.1f mm (Beine gestellt), Sollwert zählt nicht mehr", self._ist_mm)
        return self._ist_mm if self._ist_da else self._height_mm

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
        # Firmware-Patch 06-leds: LEDs aus dem Shutdown-Blau zurück (beim Herunterfahren des Pi setzt xgo-shutdown-led 234);
        # ältere Firmware kennt 233 nicht als Aktion und tut nichts
        self._write("action", FW_SHUTDOWN_LEDS_OFF)
        self._led_start()

    VEL_NEU_S = 5.0          # so oft nachsehen, ob die Firmware den Fahrrahmen (wieder) kann, solange es Bytes sind

    def _vel_nachpruefen(self, now: float) -> None:
        """Fahrrahmen nachträglich einschalten: Antwortete die Firmware beim Start nicht (Flashen, ESP-Neustart, USB),
        blieb xgonav bis zum eigenen Neustart bei den Bytes (Rider 2026-11-25). Alle VEL_NEU_S still nachfragen.
        Kosten am Pi: eine Abfrage an den ESP32 je 5 s, nur solange der Fahrrahmen aus ist."""
        if self.vel or not self.cfg.vel_frame or now - self._vel_neu_t < self.VEL_NEU_S:
            return
        self._vel_neu_t = now
        if not os.path.exists(self._path("vel")) or zustand(self._read_text("vel") or "") is None:
            return
        log.warning("Firmware antwortet jetzt auf den Fahrrahmen – schalte von Bytes um")
        self._gen_reset()
        self._vel_setup()

    def _vel_setup(self) -> None:
        """Fahrrahmen einschalten, wenn robot.vel_frame, der Treiber /proc/XGORider/vel hat und die Firmware auf 0x1a
        antwortet (Firmware-Patch 08-velocity); sonst bleibt es bei speed_x/speed_z."""
        self.vel = False
        if not self.cfg.vel_frame:
            return
        if not os.path.exists(self._path("vel")):
            log.warning("Treiber ohne /proc/XGORider/vel – Fahrbefehle als Bytes (speed_x/speed_z)")
            return
        st = zustand(self._read_text("vel") or "")
        if st is None:
            log.warning("Firmware antwortet nicht auf den Fahrrahmen (0x1a, Firmware ohne Patch 08-velocity) – Fahrbefehle als Bytes")
            return
        self._write_limits(time.monotonic())
        self.vel = True
        self._caps_setup(st["caps"])
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
        """Nach einem Sturz einmal das Protokoll des Hebeschutzes lesen (Firmware-Patch 03-diagnostics, Treiber /proc/XGORider/lift)
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
            log.warning("Körperhöhe von außen: %.1f mm (u8 %s), %s", mm, text,
                        "Geometrie nimmt die Höhe aus dem Strom" if getattr(self, "_ist_da", False) else "Geometrie folgt")


class SimRobot(Robot):
    """Für Replay/Tests: integriert Kurs aus w und protokolliert Kommandos.

    proc_dir (nur virtueller Rider, tools/sim_rider.py): ein gewöhnliches Verzeichnis spielt /proc/XGORider für
    die Weboberfläche. Jedes Schreiben landet auch in der Datei (wie /proc den letzten Wert zeigt); Höhe und
    Aktion, die von außen (api.php) geschrieben werden, übernimmt _poll_state wie Treiber und Firmware; state und
    battery stehen dort für live.php. Ersetzt wird immer per tmp + rename (Web und Simulator laufen als
    verschiedene Nutzer, das Verzeichnis gehört beiden)."""

    PROC_DATEIEN = {"height": "", "roll": "0", "action": "", "state": "1", "battery": "100"}

    def __init__(self, cfg: RobotCfg, log_cmds: bool = False, proc_dir: str | None = None):
        super().__init__(cfg)
        self.log_cmds = log_cmds
        self.writes: list[tuple[float, str, int | str]] = []
        self._up_at: float | None = None     # Aufspringen läuft, balanciert ab dann
        self.state.mode = 1
        self.settings: dict[str, str] = {}   # Treiber-Einstellungen (settings/<name>), für Tests vorbelegbar
        self.vel = cfg.vel_frame             # wie eine Firmware mit 08-velocity und passendem Treiber
        self.KURSABS_OHNE_IMU = True         # absolutes Kursziel: Winkel des Generators ist der Kurs der Firmware
        self.proc_dir = proc_dir
        self._proc_seen: dict[str, tuple[str, int]] = {}   # zuletzt gesehener Inhalt + mtime je Datei
        self._state_text: str | None = None
        if proc_dir:
            for sub in ("leds", "settings"):
                os.makedirs(os.path.join(proc_dir, sub), exist_ok=True)
            for name, text in self.PROC_DATEIEN.items():
                self._proc_schreiben(name, text)
            # steht schon (state 1): Mittelstellung der Firmware nach dem Aufstehen, wie am Rider
            self._proc_schreiben("height", str(FW_HEIGHT_STANDUP_U8))
            self._height_text = str(FW_HEIGHT_STANDUP_U8)
            self._height_mm = self.height_u8_to_mm(FW_HEIGHT_STANDUP_U8)

    def _proc_schreiben(self, name: str, text: str) -> None:
        p = os.path.join(self.proc_dir, name)
        tmp = "%s.%d.tmp" % (p, os.getpid())
        try:
            with open(tmp, "w") as f:
                f.write(text + "\n" if text else "")
            os.replace(tmp, p)
            self._proc_seen[name] = (text, os.stat(p).st_mtime_ns)
        except OSError as e:
            log.warning("Sim-/proc %s: %s", name, e)

    def _proc_neu(self, name: str) -> str | None:
        """Inhalt, falls jemand anderes die Datei seit dem letzten Blick geschrieben hat, sonst None."""
        p = os.path.join(self.proc_dir, name)
        try:
            mt = os.stat(p).st_mtime_ns
            with open(p) as f:
                text = f.read().strip()
        except OSError:
            return None
        alt = self._proc_seen.get(name)
        self._proc_seen[name] = (text, mt)
        if alt is not None and alt == (text, mt):
            return None
        return text

    def _setup(self) -> None:
        if self.vel:
            self._write_limits(time.monotonic())
            self._caps_setup(CAP_KAPPA | CAP_KURS | CAP_STRECKE | CAP_KURSABS)   # Firmware mit Bahnbefehlen bis Stufe 3
                                                                                # und absolutem Kursziel

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
        if self.proc_dir and (name in self.PROC_DATEIEN or name.startswith("leds/")):
            self._proc_schreiben(name, str(value))
            if name == "height":
                self._height_text = str(value)      # eigener Wert, nicht als „von außen“ übernehmen
        self._aktion(name, value)

    def _aktion(self, name: str, value: int | str) -> None:
        if name == "action" and value == 254:
            self.state.balancing, self.state.mode = False, 254
            self._up_at = None
        elif name == "action" and value == 253 and not self.state.balancing:
            self._up_at = time.monotonic() + 1.0

    def _proc_von_aussen(self) -> None:
        """Was die Weboberfläche direkt nach /proc schreibt: Höhe (wie ProcRobot._poll_height, Geometrie folgt) und
        Aktionen (253 aufstehen, 254 absetzen; die anderen kennt der Simulator nicht)."""
        text = self._proc_neu("height")
        if text is not None and text.isdigit():
            with self._height_lock:
                if text != self._height_text:
                    self._height_text = text
                    self._height_mm = self.height_u8_to_mm(int(text))
                    log.info("Körperhöhe von außen: %.1f mm (u8 %s)", self._height_mm, text)
        text = self._proc_neu("action")
        if text is not None and text.lstrip("-").isdigit():
            self.writes.append((time.monotonic(), "action", int(text)))
            self._aktion("action", int(text))

    def _poll_state(self) -> None:
        now = time.monotonic()
        if self.proc_dir:
            self._proc_von_aussen()
        if self._up_at is not None and now >= self._up_at:
            self.state.balancing, self.state.mode = True, 1
            self._up_at = None
        self.state.battery = 100
        self.state.t = now
        if self.proc_dir:
            st = "1" if self.state.balancing else str(self.state.mode)
            if st != self._state_text:
                self._state_text = st
                self._proc_schreiben("state", st)


def make_robot(kind: str, cfg: RobotCfg) -> Robot:
    if kind == "proc":
        return ProcRobot(cfg)
    if kind == "sim":
        return SimRobot(cfg, log_cmds=False)
    raise ValueError(kind)
