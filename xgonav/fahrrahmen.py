"""Fahrrahmen der Firmware (Patches 0029–0032, ARBEITSPLAN.md Punkt 4, Stufe 8): Sollwerte in echten Einheiten.

Statt der Bytes speed_x/speed_z (wirken wie eine Kraft) schreibt Robot je Änderung einen Rahmen nach
/proc/XGORider/vel: Tempo-Soll, Beschleunigung und Ruck, Drehrate, Drehbeschleunigung und Drehruck. Die Firmware
rampt selbst (S-Kurve) und regelt Tempo (PI + Vorsteuerung) und Drehrate. v_max, w_max und ihr Wächter gehen
beim Start (und alle paar Sekunden neu) nach /proc/XGORider/vel_limits; ohne sie verwirft sie jeden Rahmen.

Einheiten: Die Radgeschwindigkeit der Firmware ist nicht echte mm/s, sondern ~3,9 mm/s je Einheit
(robot.fw_speed_unit_mm, gemessen 2026-11-25 mit tools/fw_einheit.py). Alles Fahren (v, a, Ruck,
v_max) wird damit umgerechnet. Drehen ist seit 0032 in echten °/s (Kursziel läuft mit w·dt gegen den IMU-Kurs).

Achse.schritt ist der Sollwertgenerator der Firmware (velocity_cmd.h approach) in Python: Robot rechnet so am Pi
denselben Sollwert, den die Firmware gerade anfährt (robot.command, Simulator, Bahnprüfung). Kosten am Pi: zwei
Wurzeln je Durchlauf der 50-Hz-Schleife, ein /proc-Schreiben nur bei Änderung oder Keepalive.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

W_LEVEL_OFF_DPS = 16.0 / 127.5 * 300.0     # wie das Drehbyte (|Byte-128| > 16): Firmware schaltet 0x61 ab


@dataclass
class Achse:
    """Sollwert x und Soll-Beschleunigung a einer Achse (Fahren: m/s, m/s²; Drehen: °/s, °/s²)."""
    x: float = 0.0
    a: float = 0.0

    def schritt(self, ziel: float, amax: float, jmax: float, dt: float) -> None:
        """Wie velocity_cmd.h approach: a ≤ amax, Ruck ≤ jmax (0 = ohne Grenze), rastet am Ziel ein."""
        if amax <= 0:
            self.x, self.a = ziel, 0.0
            return
        if dt <= 0:
            return
        dv = ziel - self.x
        if jmax <= 0:
            if abs(dv) <= amax * dt:
                self.x, self.a = ziel, 0.0
            else:
                self.a = math.copysign(amax, dv)
                self.x += self.a * dt
            return
        js = jmax * dt
        h = 0.5 * js
        a_des = 0.0 if dv == 0 else math.copysign(min(amax, math.sqrt(h * h + 2.0 * jmax * abs(dv)) - h), dv)
        self.a += max(-js, min(js, a_des - self.a))
        nx = self.x + self.a * dt
        if (abs(self.a) <= js) if dv == 0 else ((ziel - nx) * dv <= 0):
            self.x, self.a = ziel, 0.0
        else:
            self.x = nx


def _rund(x: float) -> int:
    """Kaufmännisch runden (vom Nullpunkt weg; Pythons round rundet 0,5 auf gerade, -0,05 °/s würde 0)."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def _fw(x_si: float, unit_mm: float) -> int:
    """m/s (bzw. m/s², m/s³) -> Firmware-Einheiten."""
    return _rund(x_si * 1000.0 / unit_mm)


def rahmen_text(v_mps: float, a_mps2: float, j_mps3: float, w_dps: float, alpha_dps2: float, jw_dps3: float,
                unit_mm: float) -> str:
    """Text für /proc/XGORider/vel (kernel_module/xgo-vel.h): "V A J W ALPHA JW" in Firmware-Einheiten, W in °/s."""
    clamp = lambda x, lo, hi: max(lo, min(hi, x))
    v = clamp(_fw(v_mps, unit_mm), -32768, 32767)
    a = clamp(_fw(abs(a_mps2), unit_mm), 0, 32767)
    j = clamp(_fw(abs(j_mps3), unit_mm), 0, 65535)
    w10 = clamp(_rund(w_dps * 10.0), -32768, 32767)
    al = clamp(_rund(abs(alpha_dps2)), 0, 65535)
    jw = clamp(_rund(abs(jw_dps3)), 0, 65535)
    w = ("-" if w10 < 0 else "") + "%d.%d" % (abs(w10) // 10, abs(w10) % 10)
    return f"{v} {a} {j} {w} {al} {jw}"


def grenzen_text(v_max_mps: float, w_max_dps: float, timeout_ms: int, unit_mm: float,
                 stock_scale: float | None = None, ki: float | None = None, i_max_deg: float | None = None) -> str:
    """Text für /proc/XGORider/vel_limits: "V_MAX W_MAX TIMEOUT_MS" (V_MAX in Firmware-Einheiten, W_MAX in °/s).

    Mit stock_scale (Firmware 0035, 10 Byte) dazu "EINHEIT_UM VORFAKTOR_PROMILLE": die Einheit braucht die
    Vorsteuerung der Firmware, der Vorfaktor verkleinert das Soll für den Bestandsregler (robot.vel_stock_scale).
    Mit ki und i_max_deg (Firmware 0037, 14 Byte) dazu "KI_E4 IMAX_CENTI": Integral des Tempo-Reglers in ° Soll-Nick
    je (Firmware-Einheit Fehler · s) und seine Grenze in °.
    """
    v = max(0, min(65535, _fw(v_max_mps, unit_mm)))
    w10 = max(0, min(65535, _rund(w_max_dps * 10.0)))
    t = f"{v} {w10 // 10}.{w10 % 10} {max(0, min(65535, int(timeout_ms)))}"
    if stock_scale is None:
        return t
    um = max(1, min(65535, _rund(unit_mm * 1000.0)))
    c = max(100, min(2000, _rund(stock_scale * 1000.0)))
    if ki is None or i_max_deg is None:
        return f"{t} {um} {c}"
    k = max(1, min(10000, _rund(ki * 10000.0)))
    im = max(1, min(1000, _rund(i_max_deg * 100.0)))
    return f"{t} {um} {c} {k} {im}"


def zustand(text: str) -> dict | None:
    """Gelesenes /proc/XGORider/vel ("aktiv beschl waechter grenzen v w v_max w_max timeout") -> dict; None bei Fehler."""
    f = text.split()
    if len(f) != 9 or f[0] == "Fehler":
        return None
    try:
        return {"aktiv": f[0] == "1", "beschl": f[1] == "1", "waechter": f[2] == "1", "grenzen": f[3] == "1",
                "v": int(f[4]), "w": float(f[5]), "v_max": int(f[6]), "w_max": float(f[7]), "timeout_ms": int(f[8])}
    except ValueError:
        return None
