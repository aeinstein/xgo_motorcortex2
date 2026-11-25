"""Fahrrahmen der Firmware (Firmware-Patch 08-velocity, ARBEITSPLAN.md Punkt 4, Stufe 8): Sollwerte in echten Einheiten.

Statt der Bytes speed_x/speed_z (wirken wie eine Kraft) schreibt Robot je Änderung einen Rahmen nach
/proc/XGORider/vel: Tempo-Soll, Beschleunigung und Ruck, Drehrate, Drehbeschleunigung und Drehruck. Die Firmware
rampt selbst (S-Kurve) und regelt Tempo (PI + Vorsteuerung) und Drehrate. v_max, w_max und ihr Wächter gehen
beim Start (und alle paar Sekunden neu) nach /proc/XGORider/vel_limits; ohne sie verwirft sie jeden Rahmen.

Einheiten: Die Radgeschwindigkeit der Firmware ist nicht echte mm/s, sondern ~3,9 mm/s je Einheit
(robot.fw_speed_unit_mm, gemessen 2026-11-25 mit tools/fw_einheit.py). Alles Fahren (v, a, Ruck,
v_max) wird damit umgerechnet. Drehen ist in echten °/s (Kursziel läuft mit w·dt gegen den IMU-Kurs).

Achse.schritt ist der Sollwertgenerator der Firmware (velocity_cmd.h approach) in Python: Robot rechnet so am Pi
denselben Sollwert, den die Firmware gerade anfährt (robot.command, Simulator, Bahnprüfung). Kosten am Pi: zwei
Wurzeln je Durchlauf der 50-Hz-Schleife, ein /proc-Schreiben nur bei Änderung oder Keepalive.

Krümmungsbetrieb (ARBEITSPLAN.md Punkt 5 Stufe 1, Flag bit1): Der Rahmen trägt statt der Drehrate die Bahnkrümmung κ,
die Firmware rechnet w = κ · v_soll und begrenzt die Fahrachse statt w (bogen_schritt = velocity_cmd.h tickKappa).
So bleibt der Bogen beim Anfahren und Bremsen derselbe. Geprüft gegen die Vektoren der Firmware-Host-Tests
(firmware/tests/vectors/, tests/test_fahrrahmen.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

W_LEVEL_OFF_DPS = 16.0 / 127.5 * 300.0     # wie das Drehbyte (|Byte-128| > 16): Firmware schaltet 0x61 ab
KDEG = 57.29578                            # ° je rad wie in velocity_cmd.h (kDeg)
KAPPA_MAX = 32.767                         # 1/m, Bereich des i16-Felds (0,001 1/m)
CAP_KAPPA = 1                              # Fähigkeitsbyte (0x1a lesen, Byte 1): bit0 Krümmungsbetrieb


def _clamp_abs(x: float, m: float) -> float:
    return max(-m, min(m, x))


def _min_grenze(a: float, b: float) -> float:
    """Kleinere von zwei Grenzen, 0 = ohne Grenze (velocity_cmd.h minLimit)."""
    return b if a <= 0 else (a if b <= 0 else min(a, b))


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

    def beschleunigen(self, a_cmd: float, jmax: float, vmax: float, dt: float) -> None:
        """Beschleunigungsbetrieb wie velocity_cmd.h accelerate: a mit Ruck auf a_cmd, Tempo bis ±vmax; a_cmd = 0 hält
        das Tempo (Beschleunigung baut sich mit dem Ruck ab)."""
        if dt <= 0:
            return
        if a_cmd != 0:
            self.schritt(math.copysign(vmax, a_cmd), abs(a_cmd), jmax, dt)
            return
        if jmax <= 0:
            self.a = 0.0
        else:
            js = jmax * dt
            self.a -= max(-js, min(js, self.a))
        self.x = _clamp_abs(self.x + self.a * dt, vmax)


@dataclass
class Bogen:
    """Zustand des Krümmungsbetriebs: an = läuft (sonst Übernahme beim nächsten Schritt), kappa = nachgeführtes κ (1/m)."""
    an: bool = False
    kappa: float = 0.0


def bogen_schritt(fwd: Achse, b: Bogen, w_jetzt: float, v_ziel: float, k_ziel: float, a: float, j: float,
                  alpha: float, jw: float, v_max: float, w_max: float, dt: float, m: float = 1.0,
                  beschl: bool = False) -> float:
    """Ein Takt Krümmungsbetrieb wie velocity_cmd.h tickKappa; gibt w (°/s) zurück.

    fwd: Fahrachse in Einheiten u (Pi: m/s, also m = 1; Firmware: Radeinheit, m = unit_mm/1000 m/s je Einheit).
    w_jetzt: laufende Drehrate (°/s) für die Übernahme aus dem Drehratenweg. a, j: Grenzen der Fahrachse (u/s², u/s³;
    beim Wächter a schon ersetzt und v_ziel = 0), alpha, jw: °/s², °/s³ (0 = ohne Grenze). beschl: Beschleunigungsbetrieb
    (a mit Vorzeichen). Die Fahrachse wird so begrenzt, dass w = κ·v nie gekappt werden muss: |v| ≤ w_max/|κ|,
    a ≤ alpha/|κ|, Ruck ≤ jw/|κ|; ein neues κ läuft mit |dκ/dt| ≤ alpha/|v| nach.
    """
    k_ziel = _clamp_abs(k_ziel, KAPPA_MAX)
    if not b.an:                              # Übernahme: κ aus dem laufenden w/v, sonst spränge w
        vm = fwd.x * m
        b.kappa = _clamp_abs(w_jetzt / KDEG / vm, KAPPA_MAX) if abs(vm) > 1e-3 else k_ziel
        b.an = True
    vm, al = abs(fwd.x * m), alpha / KDEG
    if al <= 0 or vm < 1e-6:
        b.kappa = k_ziel
    else:
        b.kappa += _clamp_abs(k_ziel - b.kappa, al / vm * dt)
    kk = max(abs(b.kappa), abs(k_ziel)) * m
    v_lim = min(v_max, w_max / KDEG / kk) if kk > 0 else v_max
    a_lim = alpha / KDEG / kk if kk > 0 and alpha > 0 else 0.0
    j_lim = jw / KDEG / kk if kk > 0 and jw > 0 else 0.0
    if beschl:
        aa = _min_grenze(abs(a), a_lim)
        fwd.beschleunigen(0.0 if a == 0 else math.copysign(aa, a), _min_grenze(j, j_lim), v_lim, dt)
    else:
        fwd.schritt(_clamp_abs(v_ziel, v_lim), _min_grenze(abs(a), a_lim), _min_grenze(j, j_lim), dt)
    km = abs(b.kappa) * m
    fwd.x = _clamp_abs(fwd.x, min(v_max, w_max / KDEG / km) if km > 0 else v_max)
    return b.kappa * m * fwd.x * KDEG


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


def bogen_text(v_mps: float, a_mps2: float, j_mps3: float, kappa: float, alpha_dps2: float, jw_dps3: float,
               unit_mm: float) -> str:
    """Krümmungsrahmen für /proc/XGORider/vel: "kap V A J K ALPHA JW", K = κ in 0,001 1/m (+ = links). Nur senden, wenn
    die Firmware CAP_KAPPA meldet (ältere führte K als Drehrate in 0,1 °/s aus)."""
    t = rahmen_text(v_mps, a_mps2, j_mps3, 0.0, alpha_dps2, jw_dps3, unit_mm).split()
    t[3] = str(max(-32767, min(32767, _rund(kappa * 1000.0))))
    return "kap " + " ".join(t)


def grenzen_text(v_max_mps: float, w_max_dps: float, timeout_ms: int, unit_mm: float,
                 ki: float, i_max_deg: float, kv: float) -> str:
    """Text für /proc/XGORider/vel_limits (Firmware-Patch 08-velocity, 14 Byte):
    "V_MAX W_MAX TIMEOUT_MS EINHEIT_UM KI_E4 IMAX_CENTI KV_E4". V_MAX in Firmware-Einheiten, W_MAX in °/s, Einheit
    in µm/s je Firmware-Einheit (für die Vorsteuerung atan(a/g)), ki in ° Soll-Nick je (Einheit Fehler · s), i_max
    in °, kv in ° Soll-Nick je Einheit gemessenes Tempo.
    """
    v = max(0, min(65535, _fw(v_max_mps, unit_mm)))
    w10 = max(0, min(65535, _rund(w_max_dps * 10.0)))
    um = max(1, min(65535, _rund(unit_mm * 1000.0)))
    k = max(1, min(10000, _rund(ki * 10000.0)))
    im = max(1, min(1000, _rund(i_max_deg * 100.0)))
    kvx = max(0, min(2000, _rund(kv * 10000.0)))
    return f"{v} {w10 // 10}.{w10 % 10} {max(0, min(65535, int(timeout_ms)))} {um} {k} {im} {kvx}"


def zustand(text: str) -> dict | None:
    """Gelesenes /proc/XGORider/vel ("aktiv beschl waechter grenzen v w v_max w_max timeout [kappa caps]") -> dict;
    None bei Fehler. Alter Treiber (9 Felder): kappa False, caps 0."""
    f = text.split()
    if len(f) not in (9, 11) or f[0] == "Fehler":
        return None
    try:
        return {"aktiv": f[0] == "1", "beschl": f[1] == "1", "waechter": f[2] == "1", "grenzen": f[3] == "1",
                "v": int(f[4]), "w": float(f[5]), "v_max": int(f[6]), "w_max": float(f[7]), "timeout_ms": int(f[8]),
                "kappa": len(f) == 11 and f[9] == "1", "caps": int(f[10]) if len(f) == 11 else 0}
    except ValueError:
        return None
