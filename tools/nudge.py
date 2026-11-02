"""Rider von Hand per Software manövrieren (z. B. zum Ausrichten für die Kalibrierung).

  python3 tools/nudge.py yaw                  # aktuellen Kurs anzeigen
  python3 tools/nudge.py turn 10              # 10° nach links drehen (IMU-geregelt), -10 = rechts
  python3 tools/nudge.py move 1.0             # 1 s langsam vorwärts, -1.0 = rückwärts (ungeregelt)
  Option --invert: Drehrichtung von speed_z umkehren, falls 'turn' falsch herum dreht.

Braucht Treiber-Patch 0002 (/proc/XGORider/imu) für 'turn'. Nach jedem Befehl wird
explizit Stillstand geschrieben; zusätzlich greift der Kernel-Watchdog (motion_timeout_ms).
Annahme: Kurs der IMU steigt bei Linksdrehung (beim Aufrichten beobachtet, nicht kalibriert).
"""
from __future__ import annotations

import os
import sys
import time

PROC = "/proc/XGORider"
NEUTRAL = 128


def write(name: str, value: int) -> None:
    fd = os.open(os.path.join(PROC, name), os.O_WRONLY)
    try:
        os.write(fd, str(int(value)).encode())
    finally:
        os.close(fd)


def read(name: str) -> str:
    with open(os.path.join(PROC, name)) as f:
        return f.read().strip()


def balancing() -> bool:
    return read("state") == "1"


def yaw_deg() -> float:
    # Format: seq t_ns roll pitch yaw (Milligrad)
    return int(read("imu").split()[4]) / 1000.0


def stop() -> None:
    write("speed_x", NEUTRAL)
    write("speed_z", NEUTRAL)


def u8(x: float) -> int:
    x = max(-1.0, min(1.0, x))
    return int(round(NEUTRAL + x * 127))


def turn(deg: float, invert: bool, timeout: float = 12.0) -> None:
    """Zweistufig, weil der Rider nach dem Stoppbefehl deutlich nachdreht (Rampe der
    Firmware, Schwung, Balancier-Drift; beobachtet 10-17°):
    1. grob: proportional drehen, schon bei COARSE_STOP Restwinkel stoppen, ausschwingen;
    2. fein: kurze Impulse mit Mindestgeschwindigkeit, nach jedem Impuls ausschwingen und
       neu messen; die Impulslänge passt sich an die beobachtete Wirkung an."""
    COARSE_STOP, TOL, W_MIN = 10.0, 2.0, 0.06   # kleinere Befehle ignoriert die Firmware
    y0 = yaw_deg()
    target = y0 + deg
    sgn = -1.0 if invert else 1.0
    t0 = time.monotonic()
    checked = False

    def settle(secs: float = 0.8) -> float:
        stop()
        time.sleep(secs)
        return target - yaw_deg()

    try:
        # --- Stufe 1: grob
        while abs(target - yaw_deg()) > COARSE_STOP:
            if not balancing():
                print("ABBRUCH: Rider balanciert nicht")
                return
            y = yaw_deg()
            err = target - y
            if time.monotonic() - t0 > timeout:
                print("ABBRUCH: Zeitlimit")
                break
            # nach 0,6 s prüfen, ob sich der Kurs in die gewünschte Richtung bewegt
            if not checked and time.monotonic() - t0 > 0.6:
                checked = True
                moved = y - y0
                if abs(moved) > 1.5 and moved * deg < 0:
                    stop()
                    print(f"ABBRUCH: dreht falsch herum ({moved:+.1f}°). Mit --invert erneut versuchen.")
                    return
            w = max(-0.10, min(0.10, 0.008 * err))
            if abs(w) < W_MIN:
                w = W_MIN if w > 0 else -W_MIN
            write("speed_z", u8(sgn * w))
            time.sleep(0.05)
        err = settle(1.0)
        # --- Stufe 2: fein, höchstens 5 Impulse
        gain = 25.0        # erwartete Wirkung in ° pro Sekunde Impuls, wird nachgeführt
        for _ in range(5):
            if abs(err) < TOL or not balancing():
                break
            dur = min(0.4, max(0.08, abs(err) / gain))
            w = W_MIN if err > 0 else -W_MIN
            t1 = time.monotonic()
            while time.monotonic() - t1 < dur:
                write("speed_z", u8(sgn * w))
                time.sleep(0.04)
            before = err
            err = settle(1.0)
            moved = abs(before - err) if (before - err) * before > 0 else 0.0
            if moved > 0.3:
                gain = max(5.0, min(80.0, moved / dur))
    finally:
        stop()
    time.sleep(0.3)
    y1 = yaw_deg()
    print(f"Kurs {y0:.1f}° -> {y1:.1f}° (gedreht {y1 - y0:+.1f}°, Ziel {deg:+.1f}°)")


def move(secs: float, v: float = 0.08) -> None:
    v = v if secs > 0 else -v
    secs = abs(secs)
    y0 = yaw_deg()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < secs:
            if not balancing():
                print("ABBRUCH: Rider balanciert nicht")
                return
            write("speed_x", u8(v))
            time.sleep(0.05)
    finally:
        stop()
    time.sleep(0.5)
    # Annahme 1 Firmware-Einheit = 1 m/s, Vollausschlag = 1,5 Einheiten (unkalibriert)
    print(f"{'vorwärts' if v > 0 else 'rückwärts'} {secs:.1f} s mit v={v:+.2f} "
          f"(laut Annahme ~{abs(v) * 1.5 * secs * 100:.0f} cm), Kursänderung {yaw_deg() - y0:+.1f}°")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    cmd = args[0] if args else ""
    if cmd == "yaw":
        print(f"Kurs {yaw_deg():.1f}°, balanciert: {balancing()}")
    elif cmd == "turn" and len(args) == 2:
        turn(float(args[1]), "--invert" in sys.argv)
    elif cmd == "move" and len(args) == 2:
        move(float(args[1]))
    else:
        print(__doc__)
