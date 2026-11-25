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
CAP_KURS = 2                               # bit1 Kursziel
CAP_STRECKE = 4                            # bit2 Strecke mit Halt
STRECKE_ANKUNFT_MM = 5.0                   # wirksamer Rest für „erreicht“ (kDistArrive)
STRECKE_LAG_TAU_S = 0.5                    # Abklingen des Nachlaufs im Stand (kDistLagTau)
KURS_ANKUNFT_DEG = 0.5                     # Restwinkel für „angekommen“ (kCourseArrive)
STOP_SIM_MAX = 1000                        # Takte der Bremswinkel-Vorausschau (kStopSimMax)
WATCHDOG_ALPHA = 180.0                     # °/s², Wächter ohne alpha im Rahmen (kWatchdogAlpha)


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
                  beschl: bool = False, st: "Strecke | None" = None) -> float:
    """Ein Takt Krümmungsbetrieb wie velocity_cmd.h tickKappa; gibt w (°/s) zurück.

    fwd: Fahrachse in Einheiten u (Pi: m/s, also m = 1; Firmware: Radeinheit, m = unit_mm/1000 m/s je Einheit).
    w_jetzt: laufende Drehrate (°/s) für die Übernahme aus dem Drehratenweg. a, j: Grenzen der Fahrachse (u/s², u/s³;
    beim Wächter a schon ersetzt und v_ziel = 0), alpha, jw: °/s², °/s³ (0 = ohne Grenze). beschl: Beschleunigungsbetrieb
    (a mit Vorzeichen). Die Fahrachse wird so begrenzt, dass w = κ·v nie gekappt werden muss: |v| ≤ w_max/|κ|,
    a ≤ alpha/|κ|, Ruck ≤ jw/|κ|; ein neues κ läuft mit |dκ/dt| ≤ alpha/|v| nach. st: laufende Strecke (Bogen bis zum
    Haltepunkt, strecke_schritt mit den κ-Grenzen; beim Wächter None).
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
    elif st is not None and st.an:
        strecke_schritt(fwd, st, _clamp_abs(v_ziel, v_lim), _min_grenze(abs(a), a_lim), _min_grenze(j, j_lim), dt,
                        m * 1000.0)
    else:
        fwd.schritt(_clamp_abs(v_ziel, v_lim), _min_grenze(abs(a), a_lim), _min_grenze(j, j_lim), dt)
    km = abs(b.kappa) * m
    fwd.x = _clamp_abs(fwd.x, min(v_max, w_max / KDEG / km) if km > 0 else v_max)
    return b.kappa * m * fwd.x * KDEG


@dataclass
class Kurs:
    """Zustand des Kursziels (ARBEITSPLAN.md Punkt 5 Stufe 2, velocity_cmd.h course_*): an = läuft, seq = Wechselbit
    des laufenden Ziels, done = angekommen, rest = Restwinkel (°)."""
    an: bool = False
    seq: bool = False
    done: bool = False
    rest: float = 0.0


def kurs_empfang(k: Kurs, dpsi_deg: float | None, seq: bool = False) -> None:
    """Rahmen angenommen (velocity_cmd.h receive): dpsi None = kein Kursziel-Rahmen (beendet es). Ein neues Ziel nur,
    wenn keins lief oder das Wechselbit anders ist; gleiche Rahmen halten nur den Wächter wach."""
    if dpsi_deg is None:
        k.an = False
    elif not k.an or seq != k.seq:
        k.an, k.seq, k.rest, k.done = True, seq, dpsi_deg, False


def stop_winkel(ax: Achse, alpha: float, jw: float, dt: float) -> float:
    """Bremsweg einer Achse aus (x, a), mit demselben approach auf 0 vorausgerechnet (stopAngle/stopDistance):
    Drehachse in °, Fahrachse in Einheit von x mal s."""
    s, d = Achse(ax.x, ax.a), 0.0
    for _ in range(STOP_SIM_MAX):
        if s.x == 0 and s.a == 0:
            break
        s.schritt(0.0, alpha, jw, dt)
        d += s.x * dt
    return d


@dataclass
class Strecke:
    """Strecke mit Halt (ARBEITSPLAN.md Punkt 5 Stufe 3, velocity_cmd.h dist_*): an = läuft, done = erreicht, rest =
    Reststrecke (mm), lag = Nachlauf (mm, was der Rider dem Soll hinterherfährt und noch kommt)."""
    an: bool = False
    done: bool = False
    rest: float = 0.0
    lag: float = 0.0


def strecke_empfang(st: Strecke, s_mm: float | None) -> None:
    """Rahmen angenommen (receive): jeder Strecken-Rahmen setzt den Rest neu (kein Wechselbit), der Nachlauf läuft
    weiter und beginnt beim Einschalten bei 0; ein Rahmen ohne Strecke (s_mm None) beendet sie."""
    if s_mm is None:
        st.an, st.done, st.rest, st.lag = False, False, 0.0, 0.0
        return
    if not st.an:
        st.lag = 0.0
    st.an, st.rest = True, s_mm
    st.done = s_mm - st.lag <= STRECKE_ANKUNFT_MM


def strecke_messen(st: Strecke, v_soll_vorher: float, v_meas: float, richtung: float, unit_mm: float, dt: float) -> None:
    """Je Takt vor dem Generator (tick): gemessene Radstrecke vom Rest ab, Nachlauf = ∫(v_soll − v_meas) in
    Fahrtrichtung; im Stand (Soll 0, Ist < 1 Einheit) klingt er ab. v in Radeinheiten, richtung ±1 (Rahmen-v)."""
    if not st.an or dt <= 0:
        return
    st.rest -= v_meas * unit_mm * dt * richtung
    st.lag += (v_soll_vorher - v_meas) * unit_mm * dt * richtung
    if v_soll_vorher == 0 and abs(v_meas) < 1.0:
        st.lag *= math.exp(-dt / STRECKE_LAG_TAU_S)


def strecke_schritt(fwd: Achse, st: Strecke, v_ziel: float, amax: float, jmax: float, dt: float, unit_mm: float) -> None:
    """Fahrachse auf Strecke (approachDist): Schritt Richtung v_ziel nur, wenn der Bremsweg danach noch in den
    wirksamen Rest (Rest − Nachlauf, mm) passt, sonst bremsen; erreicht bei ≤ 5 mm, dann auf 0. Kosten am Pi: bis
    ~20 approach-Schritte je Durchlauf bei dt 0,02 s, nur bei laufender Strecke."""
    rest = st.rest - st.lag
    if rest <= STRECKE_ANKUNFT_MM:
        st.done = True
    if st.done:
        fwd.schritt(0.0, amax, jmax, dt)
        return
    if dt <= 0:
        return
    go = Achse(fwd.x, fwd.a)
    go.schritt(v_ziel, amax, jmax, dt)
    if rest - abs(go.x) * dt * unit_mm - abs(stop_winkel(go, amax, jmax, dt)) * unit_mm >= 0:
        fwd.x, fwd.a = go.x, go.a
    else:
        fwd.schritt(0.0, amax, jmax, dt)


def kurs_schritt(turn: Achse, k: Kurs, alpha: float, jw: float, w_max: float, dt: float,
                 waechter: bool = False) -> None:
    """Drehachse eines Takts im Kursziel (velocity_cmd.h tickCourse; die Fahrachse läuft wie im Geschwindigkeitsbetrieb
    beim Aufrufer). Mit Vorausschau: weiter auf ±w_max, solange der Bremswinkel danach noch in den Rest passt, sonst
    bremsen; ohne alpha springt w. Danach w auf w_max geklemmt, Rest −= w·dt, angekommen bei w = a = 0, |Rest| < 0,5°.
    Kosten am Pi: bis ~20 approach-Schritte je Durchlauf bei dt 0,02 s (nur während eines Kursziels)."""
    if waechter:
        turn.schritt(0.0, alpha if alpha > 0 else WATCHDOG_ALPHA, jw, dt)
    elif dt > 0:
        if k.done:
            turn.schritt(0.0, alpha, jw, dt)
        elif alpha <= 0:
            turn.x, turn.a = math.copysign(min(w_max, abs(k.rest) / dt), k.rest), 0.0
        else:
            sg = 1.0 if k.rest >= 0 else -1.0
            go = Achse(turn.x, turn.a)
            go.schritt(sg * w_max, alpha, jw, dt)
            if (k.rest - go.x * dt - stop_winkel(go, alpha, jw, dt)) * sg >= 0:
                turn.x, turn.a = go.x, go.a
            else:
                turn.schritt(0.0, alpha, jw, dt)
    turn.x = _clamp_abs(turn.x, w_max)
    k.rest -= turn.x * dt
    if not k.done and not waechter and turn.x == 0 and turn.a == 0 and abs(k.rest) < KURS_ANKUNFT_DEG:
        k.done = True


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


def kurs_text(seq: bool, v_mps: float, a_mps2: float, j_mps3: float, dpsi_deg: float, alpha_dps2: float,
              jw_dps3: float, unit_mm: float) -> str:
    """Kursziel-Rahmen für /proc/XGORider/vel: "kurs S V A J D ALPHA JW", S = Wechselbit (0/1, umschalten = neues
    Ziel), D = Δψ in ° (eine Nachkommastelle, + = links). Nur senden, wenn die Firmware CAP_KURS meldet."""
    return "kurs %d " % (1 if seq else 0) + rahmen_text(v_mps, a_mps2, j_mps3, dpsi_deg, alpha_dps2, jw_dps3, unit_mm)


def strecke_text(v_mps: float, a_mps2: float, j_mps3: float, w_dps: float | None, kappa: float | None,
                 alpha_dps2: float, jw_dps3: float, unit_mm: float, s_m: float) -> str:
    """Strecke mit Halt (15 Byte): "dist V A J W ALPHA JW S" bzw. mit Krümmung "kapdist V A J K ALPHA JW S", S =
    Reststrecke in mm (0..65535). Jeder Rahmen setzt den Rest neu: der Pi muss ihn laufend nachrechnen."""
    s_mm = max(0, min(65535, _rund(s_m * 1000.0)))
    if kappa is not None:
        return "kapdist " + bogen_text(v_mps, a_mps2, j_mps3, kappa, alpha_dps2, jw_dps3, unit_mm)[4:] + " %d" % s_mm
    return "dist " + rahmen_text(v_mps, a_mps2, j_mps3, w_dps or 0.0, alpha_dps2, jw_dps3, unit_mm) + " %d" % s_mm


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
    """Gelesenes /proc/XGORider/vel ("aktiv beschl waechter grenzen v w v_max w_max timeout [kappa caps [kurs
    erreicht [strecke]]]") -> dict; None bei Fehler. Älterer Treiber (9, 11, 13 Felder): fehlende Felder False/0. caps
    nimmt nur die Bits, die der Treiber auch schreiben kann (11 Felder: nur Krümmung, 13: ohne Strecke)."""
    f = text.split()
    if len(f) not in (9, 11, 13, 14) or f[0] == "Fehler":
        return None
    try:
        caps = int(f[10]) if len(f) >= 11 else 0
        if len(f) == 11:
            caps &= CAP_KAPPA                # Treiber ohne Kursrahmen
        elif len(f) == 13:
            caps &= CAP_KAPPA | CAP_KURS     # Treiber ohne Streckenrahmen
        return {"aktiv": f[0] == "1", "beschl": f[1] == "1", "waechter": f[2] == "1", "grenzen": f[3] == "1",
                "v": int(f[4]), "w": float(f[5]), "v_max": int(f[6]), "w_max": float(f[7]), "timeout_ms": int(f[8]),
                "kappa": len(f) >= 11 and f[9] == "1", "caps": caps,
                "kurs": len(f) >= 13 and f[11] == "1", "erreicht": len(f) >= 13 and f[12] == "1",
                "strecke": len(f) == 14 and f[13] == "1"}
    except ValueError:
        return None
