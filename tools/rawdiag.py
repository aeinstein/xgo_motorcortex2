#!/usr/bin/env python3
"""Rohwerte der IMU aus Register 0x15 mitschreiben (Firmware-Patch 0020), auf dem Rider als root.

    python3 rawdiag.py 60 /root/roh.csv        # 60 s lang, so schnell wie raw_reg antwortet (~50 Hz)

Das Register liefert laufende Summen über alle Proben der Regelschleife (~300 Hz). Je Zeile steht das
Mittel zwischen zwei Lesungen, also ohne Abtastfehler: Beschleunigung in m/s^2 (Achsen wie
ImuState::acceleration: x quer, y längs, z hoch), Kreisel in °/s (Roll, Nick, Gier; ohne
Nullpunktabzug), Radticks (11 = links), dazu die Zeile aus /proc/XGORider/imu (fusionierte Lage).

Spalten: t_ns (CLOCK_MONOTONIC nach der Lesung), t_us (Messzeit der letzten Probe am ESP32, entfaltet),
proben (Proben seit der vorigen Zeile), ax, ay, az, g_roll, g_nick, g_gier, w11, w21, imu

Kostet am Pi nur während des Mitschnitts Rechenzeit (ein Python-Prozess, zwei /proc-Zugriffe je Zeile).
raw_reg hat nur eine Adresse: nicht gleichzeitig mit imudiag.py oder anderen raw_reg-Lesern laufen lassen.
"""
import struct
import sys
import time

RAW = "/proc/XGORider/raw_reg"
IMU = "/proc/XGORider/imu"
K_ACC = 19.6 / 32768.0        # m/s^2 je Zählwert
K_GYRO = 2000.0 / 32768.0     # °/s je Zählwert
T_STEP_US = 64                # Register: micros() >> 6


def lesen():
    with open(RAW) as f:
        text = f.read()
    kopf, _, daten = text.partition(":")
    teile = daten.split()
    if "Fehler" in daten or len(teile) != 32:
        return None
    return bytes(int(x, 16) for x in teile)


def s32(d):   # Differenz zweier u32-Summen als Zahl mit Vorzeichen
    d &= 0xFFFFFFFF
    return d - (1 << 32) if d & 0x80000000 else d


def s16(d):
    d &= 0xFFFF
    return d - (1 << 16) if d & 0x8000 else d


def main():
    dauer = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    ziel = sys.argv[2] if len(sys.argv) > 2 else "/root/rawdiag.csv"
    with open(RAW, "w") as f:
        f.write("0x15 32")
    alt = None
    t_us = 0
    w = [0, 0]
    zeilen = fehler = 0
    ende = time.monotonic() + dauer
    with open(ziel, "w") as out:
        out.write("t_ns,t_us,proben,ax,ay,az,g_roll,g_nick,g_gier,w11,w21,imu\n")
        while time.monotonic() < ende:
            b = lesen()
            t_ns = time.monotonic_ns()
            if b is None:
                fehler += 1
                continue
            n, t16 = struct.unpack_from("<HH", b, 0)
            summen = struct.unpack_from("<6I", b, 4)
            w16 = struct.unpack_from("<2H", b, 28)
            if alt is None:
                alt = (n, t16, summen, w16)
                continue
            dn = (n - alt[0]) & 0xFFFF
            if not dn:          # noch keine neue Probe: nichts verrechnen, alter Stand bleibt Bezug
                continue
            t_us += ((t16 - alt[1]) & 0xFFFF) * T_STEP_US
            w[0] += s16(w16[0] - alt[3][0])
            w[1] += s16(w16[1] - alt[3][1])
            d = [s32(summen[i] - alt[2][i]) / dn for i in range(6)]
            alt = (n, t16, summen, w16)
            try:
                with open(IMU) as f:
                    imu = ";".join(f.read().split())
            except OSError:
                imu = ""
            out.write("%d,%d,%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%d,%s\n" % (
                t_ns, t_us, dn, d[0] * K_ACC, d[1] * K_ACC, d[2] * K_ACC,
                d[3] * K_GYRO, d[4] * K_GYRO, d[5] * K_GYRO, w[0], w[1], imu))
            zeilen += 1
    print("%d Zeilen in %.0f s (%.1f Hz), %d Lesefehler -> %s" % (zeilen, dauer, zeilen / dauer, fehler, ziel))


if __name__ == "__main__":
    main()
