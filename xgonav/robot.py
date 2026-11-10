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
import os
import threading
import time
from dataclasses import dataclass

from .config import RobotCfg

log = logging.getLogger(__name__)

NEUTRAL = 128
# Höhe in der Firmware (xgorider2_rl main.cpp, action_engine.cpp): Solange der Rider nicht balanciert, setzt
# sie den Sollwert in jedem Takt auf 0; beim Aufstehen nimmt sie die Mittelstellung 128. Gefahren wird nur
# 55..200. /proc/XGORider/height zeigt dagegen den zuletzt geschriebenen Wert, auch über das Aufstehen hinweg.
FW_HEIGHT_STANDUP_U8 = 128
FW_HEIGHT_MIN_U8, FW_HEIGHT_MAX_U8 = 55, 200


def norm_to_u8(x: float) -> int:
    x = max(-1.0, min(1.0, x))
    return int(round(NEUTRAL + x * 127))


def u8_to_norm(u: int) -> float:
    return (u - NEUTRAL) / 127.0


@dataclass
class RobotState:
    balancing: bool = True
    yaw_deg: float | None = None
    battery: int | None = None
    buttons: int = 0
    t: float = 0.0
    mode: int | None = None    # Wert aus /proc/XGORider/state (1 = balanciert)


class Robot:
    """Gemeinsame Logik (Rampe, Watchdog). Unterklassen implementieren _write_*."""

    def __init__(self, cfg: RobotCfg):
        self.cfg = cfg
        self.forward_gain = 1.0              # App: manual_forward_gain in der Handsteuerung, sonst 1
        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
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
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        self.state = RobotState()
        self.ruhe_tick_s: float | None = None   # Ruhemodus: nur so oft Zustand lesen, nichts schreiben

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

    def drive(self, v: float, w: float) -> None:
        v = max(-self.cfg.max_v, min(self.cfg.max_v, v))
        w = max(-self.cfg.max_w, min(self.cfg.max_w, w))
        with self._lock:
            self._target = (v, w)
            self._last_cmd_t = time.monotonic()

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def set_down(self) -> None:
        """Absetzen (Firmware-Aktion 254)."""
        self.stop()
        self._write("action", 254)

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
            self._write("roll", int(round(max(-20.0, min(20.0, deg)))))

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

    # ---- interne Schleife (50 Hz) ---------------------------------------
    def _loop(self) -> None:
        next_poll = 0.0
        while not self._stop_evt.wait(self.ruhe_tick_s or 0.02):
            if self.ruhe_tick_s:
                # Ruhemodus: liegt, kein Keepalive und keine Rampe; Zustand und Akku fürs Web
                self._poll_state()
                self._height_tick()
                self._level_tick(time.monotonic())
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
                next_poll = now + 0.2
            if not self.state.balancing:
                target = (0.0, 0.0)
            self._ramp(target, now)
            self._send(now)

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
            return
        if self._balancing_since is None:
            self._balancing_since = now
        try:
            if self._level_on_t is not None:
                if now - self._level_on_t >= self.cfg.level_pulse_s:
                    self._write("roll_balance", 0)
                    self._level_on_t = None
                return
            standup = self.cfg.level_after_standup and self._seen_down and not self._standup_levelled
            if (standup or self._level_request) and now - self._balancing_since >= 2.5:
                self._write("roll", 0)
                self._write("roll_balance", 1)
                self._level_on_t = now
                self._level_request = False
                self._standup_levelled = True
        except OSError as e:
            log.error("Nivellierung fehlgeschlagen: %s", e)

    def _ramp(self, target: tuple[float, float], now: float) -> None:
        dt = max(1e-3, now - self._last_ramp_t)
        self._last_ramp_t = now
        v, w = self._cur
        dv = max(-self.cfg.accel_v * dt, min(self.cfg.accel_v * dt, target[0] - v))
        dw = max(-self.cfg.accel_w * dt, min(self.cfg.accel_w * dt, target[1] - w))
        # Bremsen darf schneller gehen als Beschleunigen
        if abs(target[0]) < abs(v):
            dv = max(-3 * self.cfg.accel_v * dt, min(3 * self.cfg.accel_v * dt, target[0] - v))
        self._cur = (v + dv, w + dw)

    def _send(self, now: float) -> None:
        v = self._cur[0]
        v *= self.cfg.reverse_gain if v < 0 else self.forward_gain
        u = (norm_to_u8(v), norm_to_u8(self._cur[1]))
        keepalive = now - self._last_send_t > self.cfg.keepalive_s
        prev = self._sent
        if prev == u and not keepalive:
            return
        try:
            if prev is None or keepalive or u[0] != prev[0]:
                self._write("speed_x", u[0])
            if prev is None or keepalive or u[1] != prev[1]:
                self._write("speed_z", u[1])
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
        # force_yaw kämpft gegen jede Drehung -> aus
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
        self.state.mode = st
        if st is not None:
            bal = st == 1
            if bal != self.state.balancing:
                log.warning("Rider %s", "balanciert" if bal else "GEFALLEN – Stopp")
            self.state.balancing = bal
        yaw = self._read_int("yaw")
        if yaw is not None:
            self.state.yaw_deg = float(yaw)
        self.state.battery = self._read_int("battery")
        self.state.buttons = self._read_int("buttons") or 0
        self._poll_height()
        self.state.t = time.monotonic()

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
        self._yaw = 0.0
        self._last_t = time.monotonic()
        self._up_at: float | None = None     # Aufspringen läuft, balanciert ab dann
        self.state.mode = 1
        self.settings: dict[str, str] = {}   # Treiber-Einstellungen (settings/<name>), für Tests vorbelegbar

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
        self._yaw += u8_to_norm(norm_to_u8(self._cur[1])) * self.cfg.vyaw_limit_dps * (now - self._last_t)
        self._last_t = now
        self.state.yaw_deg = self._yaw
        self.state.battery = 100
        self.state.t = now


def make_robot(kind: str, cfg: RobotCfg) -> Robot:
    if kind == "proc":
        return ProcRobot(cfg)
    if kind == "sim":
        return SimRobot(cfg, log_cmds=False)
    raise ValueError(kind)
