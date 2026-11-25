#!/usr/bin/env python3
"""Einheit der Radgeschwindigkeit in der Firmware messen (ARBEITSPLAN.md Punkt 4, Stufe 8), auf dem Rider als root.

    sudo python3 /opt/xgonav/tools/fw_einheit.py 60 /root/fw_einheit.csv    # 60 s mitschreiben, dann auswerten
    python3 tools/fw_einheit.py --auswerten fw_einheit.csv                   # am PC nachrechnen

Fährt selbst NICHT. Gefahren wird währenddessen über xgonav (MCP rider: fahren), geradeaus, je Abschnitt einige
Sekunden mit gleichem Tempo. Mitgeschrieben werden ~20-mal je Sekunde:
  * Register 0x0e (Firmware-Patch 03-diagnostics, über /proc/XGORider/raw_reg): gefilterte Radgeschwindigkeit 11/21 in
    Firmware-Einheiten (Eingang des Geschwindigkeitsreglers), gefilterter Fahrbefehl (x10), Flags (bit0 fährt)
  * /proc/XGORider/wheels: Radticks (1024 je Umdrehung) -> echte Geschwindigkeit aus dem Raddurchmesser
  * aus 0x0e auch Soll-Nick und gemessener Nick + Bias (°, Eingang des Nick-Reglers), Balance-Ausgang,
    Korrektur und Modus des Positionshaltens; dazu der IMU-Nick aus /proc/XGORider/imu (°) — für die Frage, ob
    er beim Fahren mit einem Nick-Versatz schneller wird (ARBEITSPLAN.md, Messfahrten 2026-11-25)
Auswertung: In Abschnitten mit |v| >= --min-mps und ruhigem Tempo ist Faktor = echte mm/s / Firmware-Einheit
(Median, Streuung, Ausgleichsgerade durch 0). Dazu Firmware-Soll gegen Firmware-Ist: Regelt die Firmware, stehen
beide gleich; fährt sie viel schneller als verlangt, wirkt der Befehl wie eine Kraft.

Spalten: t (s, CLOCK_MONOTONIC), fw_ms, fw_v11, fw_v21 (Einheiten), fw_cmd (Einheiten), faehrt (0/1), ticks11, ticks21,
odo_mms (mm/s, über --fenster gemittelt; leer bis genug Proben), nick_soll, nick_ist (°, 0x0e), balance,
halt_korr, halt_modus (0x0e), imu_nick (°), fw_turn (°/s, gefilterter Drehsollwert aus 0x0e; auf dem Rahmenweg der
Sollwert des Generators), imu_yaw (°, Kurs aus /proc/XGORider/imu) — für die Drehrate auf dem Rahmenweg

Kosten am Pi: nur während des Mitschnitts, ein Python-Prozess, je Zeile zwei /proc-Zugriffe (0x0e = eine Anfrage
an den ESP32, ~4 ms auf der Leitung). raw_reg hat nur eine Adresse: nicht gleichzeitig mit rawdiag.py laufen lassen.
"""
from __future__ import annotations

import argparse
import collections
import csv
import math
import statistics
import struct
import sys
import time

RAW = "/proc/XGORider/raw_reg"
IMU = "/proc/XGORider/imu"
WHEELS = "/proc/XGORider/wheels"
TICKS_PER_REV = 1024


def diag_0e(text: str) -> dict | None:
    """Zeile von raw_reg ("0x0e 32: 00 11 ...") -> Felder; None bei Fehler oder anderer Adresse."""
    kopf, _, rest = text.partition(":")
    if not kopf.strip().startswith("0x0e") or "Fehler" in rest:
        return None
    b = bytes(int(x, 16) for x in rest.split())
    if len(b) < 32:
        return None
    ms, v11, v21 = struct.unpack_from("<Hhh", b, 0)
    fwd10, = struct.unpack_from("<h", b, 28)
    bal, soll100, ist100, korr10 = struct.unpack_from("<hhhh", b, 14)
    turn10, = struct.unpack_from("<h", b, 30)
    return {"fw_ms": ms, "fw_v11": v11, "fw_v21": v21, "fw_cmd": fwd10 / 10.0, "faehrt": b[23] & 1,
            "nick_soll": soll100 / 100.0, "nick_ist": ist100 / 100.0, "balance": bal,
            "halt_korr": korr10 / 10.0, "halt_modus": b[22], "fw_turn": turn10 / 10.0}


def imu_nick(text: str) -> float | None:
    """/proc/XGORider/imu (\"seq t_ns roll pitch yaw\", Milligrad) -> Nick in °."""
    f = text.split()
    return int(f[3]) / 1000.0 if len(f) >= 5 else None


def imu_yaw(text: str) -> float | None:
    """/proc/XGORider/imu -> Kurs in ° (Milligrad, + = links)."""
    f = text.split()
    return int(f[4]) / 1000.0 if len(f) >= 5 else None


def wheels(text: str) -> tuple[int, int, int] | None:
    """/proc/XGORider/wheels ("seq t_ns ticks_l ticks_r fw_ms") -> (t_ns, ticks11, ticks21)."""
    f = text.split()
    if len(f) < 4:
        return None
    return int(f[1]), int(f[2]), int(f[3])


class Odometrie:
    """Echte Geschwindigkeit (mm/s) aus den Radticks über ein Zeitfenster, Mittel beider Räder."""

    def __init__(self, durchmesser_m: float, fenster_s: float):
        self.mm_je_tick = math.pi * durchmesser_m * 1000.0 / TICKS_PER_REV
        self.fenster_ns = int(fenster_s * 1e9)
        self.h: collections.deque[tuple[int, int, int]] = collections.deque()

    def neu(self, t_ns: int, t11: int, t21: int) -> float | None:
        if self.h and t_ns == self.h[-1][0]:
            return self.wert()
        self.h.append((t_ns, t11, t21))
        while len(self.h) > 2 and self.h[-1][0] - self.h[1][0] >= self.fenster_ns:
            self.h.popleft()
        return self.wert()

    def wert(self) -> float | None:
        if len(self.h) < 2:
            return None
        (ta, a11, a21), (tb, b11, b21) = self.h[0], self.h[-1]
        dt = (tb - ta) / 1e9
        if dt < 0.5 * self.fenster_ns / 1e9:
            return None
        d11 = ((b11 - a11 + 2**31) % 2**32) - 2**31          # Ticks laufen über (int32)
        d21 = ((b21 - a21 + 2**31) % 2**32) - 2**31
        return (d11 + d21) / 2.0 * self.mm_je_tick / dt


def auswerten(rows: list[dict], min_mps: float = 0.08, ruhe: float = 0.15) -> dict:
    """Faktor echte mm/s je Firmware-Einheit aus ruhigen Fahrabschnitten. ruhe: höchste relative Änderung der
    Odometrie gegen die Zeile ±5 davor/danach (Anfahren und Bremsen raus, dort hinkt die Odometrie nach)."""
    gut = []
    for i in range(5, len(rows) - 5):
        r = rows[i]
        o = r.get("odo_mms")
        if o is None or not r["faehrt"] or abs(o) < min_mps * 1000:
            continue
        nb = [rows[k].get("odo_mms") for k in (i - 5, i + 5)]
        if any(x is None or abs(x - o) > ruhe * abs(o) for x in nb):
            continue
        fw = (r["fw_v11"] + r["fw_v21"]) / 2.0
        if fw == 0 or (fw > 0) != (o > 0):
            continue
        gut.append((o, fw, r["fw_cmd"]))
    if len(gut) < 10:
        return {"n": len(gut), "faktor": None}
    q = [o / fw for o, fw, _ in gut]
    sxy = sum(o * fw for o, fw, _ in gut)
    sxx = sum(fw * fw for _, fw, _ in gut)
    qs = sorted(q)
    soll_ist = [fw / c for _, fw, c in gut if c != 0]
    return {"n": len(gut), "faktor": statistics.median(q), "faktor_gerade": sxy / sxx,
            "p10": qs[len(qs) // 10], "p90": qs[(9 * len(qs)) // 10],
            "odo_min": min(o for o, _, _ in gut), "odo_max": max(o for o, _, _ in gut),
            "ist_je_soll": statistics.median(soll_ist) if soll_ist else None}


def bericht(e: dict) -> str:
    if e["faktor"] is None:
        return "zu wenig ruhige Fahrabschnitte (%d Zeilen); länger gleichmäßig fahren, |v| >= 0,08 m/s" % e["n"]
    t = ("Faktor %.2f mm/s je Firmware-Einheit (Median, p10 %.2f, p90 %.2f; Gerade durch 0: %.2f), %d Zeilen, "
         "Tempo %.0f..%.0f mm/s" % (e["faktor"], e["p10"], e["p90"], e["faktor_gerade"], e["n"], e["odo_min"], e["odo_max"]))
    if e["ist_je_soll"] is not None:
        t += "\nFirmware-Ist / Firmware-Soll: %.2f (1 = Firmware regelt das Tempo, deutlich > 1 = Befehl wirkt wie eine Kraft)" % e["ist_je_soll"]
    return t


def lesen_csv(pfad: str) -> list[dict]:
    rows = []
    with open(pfad, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({"t": float(r["t"]), "fw_v11": int(r["fw_v11"]), "fw_v21": int(r["fw_v21"]),
                         "fw_cmd": float(r["fw_cmd"]), "faehrt": int(r["faehrt"]),
                         "odo_mms": float(r["odo_mms"]) if r["odo_mms"] else None})
    return rows


def mitschreiben(sek: float, pfad: str, durchmesser_m: float, fenster_s: float, hz: float) -> list[dict]:
    with open(RAW, "w") as f:
        f.write("0x0e 32")
    odo = Odometrie(durchmesser_m, fenster_s)
    rows, fehler = [], 0
    ende = time.monotonic() + sek
    with open(pfad, "w", newline="") as out:
        w = csv.writer(out)
        w.writerow(["t", "fw_ms", "fw_v11", "fw_v21", "fw_cmd", "faehrt", "ticks11", "ticks21", "odo_mms",
                    "nick_soll", "nick_ist", "balance", "halt_korr", "halt_modus", "imu_nick", "fw_turn", "imu_yaw"])
        try:
            while time.monotonic() < ende:
                t0 = time.monotonic()
                with open(RAW) as f:
                    d = diag_0e(f.read())
                with open(WHEELS) as f:
                    wl = wheels(f.read())
                try:
                    with open(IMU) as f:
                        txt = f.read()
                    nk, yw = imu_nick(txt), imu_yaw(txt)
                except (OSError, ValueError):
                    nk = yw = None
                if d is None or wl is None:
                    fehler += 1
                else:
                    o = odo.neu(*wl)
                    r = dict(d, t=t0, ticks11=wl[1], ticks21=wl[2], odo_mms=o)
                    rows.append(r)
                    w.writerow([f"{t0:.4f}", d["fw_ms"], d["fw_v11"], d["fw_v21"], f"{d['fw_cmd']:.1f}", d["faehrt"],
                                wl[1], wl[2], "" if o is None else f"{o:.1f}",
                                f"{d['nick_soll']:.2f}", f"{d['nick_ist']:.2f}", d["balance"], f"{d['halt_korr']:.1f}",
                                d["halt_modus"], "" if nk is None else f"{nk:.3f}", f"{d['fw_turn']:.1f}",
                                "" if yw is None else f"{yw:.3f}"])
                    if len(rows) % int(hz) == 0:
                        print("fw %5.0f/%5.0f Soll %6.1f  Räder %s mm/s" % (d["fw_v11"], d["fw_v21"], d["fw_cmd"],
                              "-" if o is None else f"{o:6.0f}"), flush=True)
                time.sleep(max(0.0, 1.0 / hz - (time.monotonic() - t0)))
        except KeyboardInterrupt:
            print("abgebrochen, werte aus")
    print("%d Zeilen, %d Lesefehler, %s" % (len(rows), fehler, pfad))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("sek", nargs="?", type=float, default=60.0)
    ap.add_argument("pfad", nargs="?", default="/root/fw_einheit.csv")
    ap.add_argument("--auswerten", metavar="CSV", help="nur eine vorhandene Aufnahme auswerten")
    ap.add_argument("--durchmesser", type=float, default=0.060, help="Raddurchmesser m (wheels.wheel_diameter_m)")
    ap.add_argument("--fenster", type=float, default=0.3, help="Mittelungsfenster der Odometrie in s")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--min-mps", type=float, default=0.08)
    a = ap.parse_args()
    rows = lesen_csv(a.auswerten) if a.auswerten else mitschreiben(a.sek, a.pfad, a.durchmesser, a.fenster, a.hz)
    print(bericht(auswerten(rows, a.min_mps)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
