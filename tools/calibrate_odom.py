"""Rad-Odometrie kalibrieren: Faktor je Rad ([odom] scale_left / scale_right) und Spurweite (track_m).

Auf dem Rider, xgonav muss laufen (Bildverarbeitung an, Modus idle), der Rider balanciert.
Braucht wenig Platz: Die Rechnung vergleicht je Abschnitt den Kurs aus den Rädern mit IMU und
Kamera, die Bahn muss dafür nicht gerade sein. Ablauf von „messen“:

  1. Werkzeug starten, es wartet. Rider in Ruhe ausrichten (freie Richtung nach vorn).
  2. In der Weboberfläche „Hand“ drücken (keine Fahrtaste!) = Start. Ab da hält das Werkzeug
     den Kurs per IMU, das langsame Wegdrehen im Stand fällt weg.
  3. auf der Stelle links um --dreh-grad, dann zurück rechts (bestimmt die Spurweite)
  4. --hin-her mal --strecke-cm vor und zurück zum Start, Kurs gehalten (Unterschied der Räder);
     Strecke nach Radweg, nicht nach Zeit, Platzbedarf also nur Strecke + ~15 cm
  Abbruch jederzeit: in der Oberfläche „Idle“/Stopp (Werkzeug sieht den Moduswechsel) oder Strg+C.

  python3 tools/calibrate_odom.py messen [--dreh-grad 360] [--strecke-cm 30] [--hin-her 2]
  python3 tools/calibrate_odom.py rechne [--ref gerade|kamera|imu|mittel] [--dist-cm X] [--spur-fest]
      --dist-cm: Maßband für den ersten Vorwärtsabschnitt (nur wenn er gerade war); ohne gilt
      der bisherige Mittelwert (scale_left + scale_right) / 2 aus der Konfiguration.

Ergebnis von Hand in config.toml eintragen ([odom]). Stillstand und Modus idle werden am Ende
immer geschrieben.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xgonav.config import Config  # noqa: E402
from xgonav.wheels import calibrate_odom  # noqa: E402

API = "http://127.0.0.1:8088"
PROC = "/proc/XGORider"
DT = 0.1                      # Regeltakt des Werkzeugs (Handmodus hält Sollwerte 0,5 s)
W_MIN = 0.06                  # kleinere Drehbefehle ignoriert die Firmware (nudge.py)
HOLD_KP, HOLD_TOL, HOLD_MAX = 0.01, 1.5, 0.10   # Kurshalten: w je Grad, Totband, Grenze


class Abbruch(Exception):
    pass


def post(path: str) -> dict:
    return json.load(urllib.request.urlopen(urllib.request.Request(API + path, method="POST"), timeout=2))


def status() -> dict:
    return json.load(urllib.request.urlopen(API + "/status", timeout=2))


def read_log(name: str) -> list[list[int]]:
    with open(f"{PROC}/{name}") as f:
        return [[int(x) for x in ln.split()] for ln in f if ln.strip()]


def wrap32(d: int) -> int:
    return (d + 2 ** 31) % 2 ** 32 - 2 ** 31


def wrap_rad(d: float) -> float:
    return (d + math.pi) % (2 * math.pi) - math.pi


class Recorder(threading.Thread):
    """Räder (wheels_log), IMU (imu_log) und Kamera-Kurs (status.yaw_vis_deg) mitschneiden;
    liefert nebenbei den aktuellen Zustand (Kurs, Radweg, Modus) für die Regelung."""

    def __init__(self, mpt: float, kl: float, kr: float):
        super().__init__(daemon=True)
        self.wheels, self.imu, self.cam = {}, {}, []
        self.mpt, self.kl, self.kr = mpt, kl, kr
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()
        self.yaw = None            # Grad, fortlaufend (IMU)
        self.pos = 0.0             # Weg der Mitte in m seit Start (Räder, mit Faktoren)
        self.mode, self.balancing = None, True
        self._yaw_last = self._w_last = None

    def run(self):
        while not self.stop_evt.is_set():
            self.poll()
            time.sleep(0.05)

    def poll(self):
        with self.lock:
            for r in read_log("wheels_log"):
                if r[0] in self.wheels:
                    continue
                self.wheels[r[0]] = r              # seq t_ns ticks11 ticks21 [fw_ms]
            for r in read_log("imu_log"):
                self.imu[r[0]] = r                 # seq t_ns roll pitch yaw (Milligrad)
            W = self.wheels[max(self.wheels)] if self.wheels else None
            I = self.imu[max(self.imu)] if self.imu else None
            if W is not None:
                if self._w_last is not None:
                    dl = wrap32(W[2] - self._w_last[2]) * self.mpt * self.kl
                    dr = wrap32(W[3] - self._w_last[3]) * self.mpt * self.kr
                    self.pos += (dl + dr) / 2
                self._w_last = W
            if I is not None:
                y = I[4] / 1000.0
                if self._yaw_last is None:
                    self.yaw = y
                else:
                    self.yaw += math.degrees(wrap_rad(math.radians(y - self._yaw_last)))
                self._yaw_last = y
        try:
            s = status()
            self.mode, self.balancing = s.get("mode"), bool(s.get("balancing"))
            if s.get("yaw_vis_deg") is not None:
                self.cam.append((time.monotonic_ns(), s["yaw_vis_deg"]))
        except OSError:
            pass


class Fahrer:
    """Fahrbefehle über den Handmodus von xgonav, mit Kurshalten per IMU."""

    def __init__(self, rec: Recorder, w_turn: float, halten: bool = True):
        self.rec, self.w_turn, self.halten_an = rec, w_turn, halten
        self.target = rec.yaw
        self.phasen: list[dict] = []

    def check(self):
        if self.rec.mode != "manual":
            raise Abbruch(f"Modus ist {self.rec.mode} statt manual (Stopp in der Oberfläche?)")
        if not self.rec.balancing:
            raise Abbruch("Rider balanciert nicht mehr")

    def hold_w(self) -> float:
        if not self.halten_an:
            return 0.0
        err = self.target - self.rec.yaw
        if abs(err) < HOLD_TOL:
            return 0.0
        return math.copysign(min(HOLD_MAX, max(W_MIN, HOLD_KP * abs(err))), err)

    def send(self, v: float, w: float):
        self.check()
        try:
            post(f"/drive?v={v:.3f}&w={w:.3f}")
        except urllib.error.HTTPError as e:      # 409: Modus nicht mehr manual
            raise Abbruch(f"Fahrbefehl abgelehnt ({e.code})")
        time.sleep(DT)

    def phase(self, name: str):
        t = time.monotonic_ns()
        if self.phasen:
            self.phasen[-1]["t1"] = t
        self.phasen.append({"name": name, "t0": t, "t1": None})
        print(f"  {name}")

    def halten(self, secs: float):
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            self.send(0.0, self.hold_w())

    def ruhe(self, secs: float):
        """Stillstand ohne Kurshalten (nach Drehungen ausschwingen lassen)."""
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            self.send(0.0, 0.0)

    def drehen(self, grad: float, timeout: float = 40.0):
        self.phase(f"drehen {grad:+.0f}°")
        y0 = self.rec.yaw
        coast = 12.0                                  # Nachdrehen nach dem Stopp (nudge: 10-17°)
        t_end = time.monotonic() + timeout
        while abs(y0 + grad - self.rec.yaw) > coast and (self.rec.yaw - y0) * grad < grad * grad:
            if time.monotonic() > t_end:
                raise Abbruch("Drehung dauert zu lange")
            self.send(0.0, math.copysign(self.w_turn, grad))
        self.ruhe(3.0)
        self.target = self.rec.yaw                    # neuer Haltekurs: wo er stehen geblieben ist
        print(f"    gedreht {self.rec.yaw - y0:+.1f}° (IMU)")

    def fahren(self, ziel_m: float, v: float, timeout: float | None = None):
        """Bis Radweg (seit Start) ziel_m erreicht, Kurs gehalten."""
        vor = ziel_m > self.rec.pos
        self.phase(f"fahren nach {ziel_m * 100:+.0f} cm")
        brems = 0.02                                  # Nachlaufen nach dem Stopp
        if timeout is None:                           # mindestens 5 cm/s erwartet
            timeout = 5.0 + abs(ziel_m - self.rec.pos) / 0.05
        t_end = time.monotonic() + timeout
        while (self.rec.pos < ziel_m - brems) if vor else (self.rec.pos > ziel_m + brems):
            if time.monotonic() > t_end:
                raise Abbruch("Fahrt dauert zu lange (Rad blockiert?)")
            self.send(v if vor else -v, self.hold_w())
        self.halten(2.5)
        print(f"    Radweg jetzt {self.rec.pos * 100:+.1f} cm, Kursfehler {self.rec.yaw - self.target:+.1f}°")


def messen(a, cfg) -> dict:
    s = status()
    if s["pipeline"] != "läuft":
        sys.exit("Bildverarbeitung läuft nicht (Kamera-Kurs fehlt) – in der Oberfläche starten")
    if not s.get("balancing"):
        sys.exit("Rider balanciert nicht – abgebrochen")
    if s["mode"] != "idle":
        sys.exit(f"xgonav ist im Modus {s['mode']}, erwartet idle – abgebrochen")
    if not Path(f"{PROC}/wheels_log").exists():
        sys.exit("keine Raddaten (/proc/XGORider/wheels_log) – Treiber/Firmware ohne Rad-Encoder")
    o = cfg.odom
    rec = Recorder(math.pi * o.wheel_diameter_m / o.ticks_per_rev, o.scale_left, o.scale_right)
    rec.poll()
    rec.start()
    print(f"bereit: Rider ausrichten, dann in der Oberfläche „Hand“ drücken (wartet {a.warten:.0f} s) …")
    t_end = time.monotonic() + a.warten
    while rec.mode != "manual":
        if time.monotonic() > t_end:
            rec.stop_evt.set()
            sys.exit("kein Start (Modus Hand) – nichts gefahren")
        if not rec.balancing:
            rec.stop_evt.set()
            sys.exit("Rider balanciert nicht – abgebrochen")
        time.sleep(0.05)
    f = Fahrer(rec, a.w, halten=not a.ohne_halten)
    start_yaw = rec.yaw
    print(f"Start, Kurs {start_yaw:.1f}° wird gehalten")
    t_start = time.monotonic_ns()
    fehler = None
    try:
        f.phase("halten")
        f.halten(2.0)
        if a.dreh_grad > 0:
            f.drehen(+a.dreh_grad)
            f.drehen(-a.dreh_grad)
            f.phase("zurück auf Startkurs")           # gefahren wird in die ausgerichtete Richtung
            f.target = start_yaw
            f.halten(4.0)
        for i in range(a.hin_her):
            f.fahren(a.strecke_cm / 100, a.v)
            if a.nur_vor:
                break
            f.fahren(0.0, a.v)
    except Abbruch as e:
        fehler = str(e)
        print(f"ABBRUCH: {e}")
    except KeyboardInterrupt:
        fehler = "Strg+C"
        print("ABBRUCH: Strg+C")
    finally:
        try:
            post("/drive?v=0&w=0")
        except OSError:
            pass
        time.sleep(0.3)
        try:
            post("/mode?name=idle")
        except OSError:
            pass
    if f.phasen:
        f.phasen[-1]["t1"] = time.monotonic_ns() + int(3e9)   # Ausschwingen gehört dazu
    print("Stillstand, Modus idle – Ausschwingen 3 s")
    time.sleep(3.0)
    rec.stop_evt.set()
    rec.join()
    res = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "v": a.v, "w": a.w, "fehler": fehler,
           "track_m": o.track_m, "scale_left": o.scale_left, "scale_right": o.scale_right,
           "wheel_diameter_m": o.wheel_diameter_m, "ticks_per_rev": o.ticks_per_rev,
           "t_start": t_start, "phasen": f.phasen,
           "wheels": [rec.wheels[k][:4] for k in sorted(rec.wheels)],
           "imu": [[r[0], r[1], r[4]] for r in (rec.imu[k] for k in sorted(rec.imu))],
           "cam": rec.cam}
    Path(a.out).write_text(json.dumps(res))
    print(f"gespeichert: {a.out}")
    return res


def abschnitte(res: dict) -> list[dict]:
    """Je Phase: Radwege roh (ohne Faktor), Drehung laut IMU und Kamera."""
    mpt = math.pi * res["wheel_diameter_m"] / res["ticks_per_rev"]
    W, I, C = res["wheels"], res["imu"], res["cam"]

    def cam_at(t):
        if not C:
            return None
        best = min(C, key=lambda c: abs(c[0] - t))
        return best[1] if abs(best[0] - t) < 3e8 else None

    out = []
    for p in res["phasen"]:
        t0, t1 = p["t0"], p["t1"]
        w = [r for r in W if t0 <= r[1] < t1]
        i = [r for r in I if t0 <= r[1] < t1]
        if len(w) < 5 or len(i) < 5:
            continue
        sl = sum(wrap32(b[2] - a[2]) for a, b in zip(w, w[1:])) * mpt
        sr = sum(wrap32(b[3] - a[3]) for a, b in zip(w, w[1:])) * mpt
        th_imu = sum(wrap_rad(math.radians((b[2] - a[2]) / 1000)) for a, b in zip(i, i[1:]))
        c0, c1 = cam_at(t0), cam_at(t1)
        th_cam = math.radians(c1 - c0) if c0 is not None and c1 is not None else None
        out.append({"name": p["name"], "sl": sl, "sr": sr, "imu": th_imu, "cam": th_cam})
    return out


def rechne(res: dict, a) -> None:
    track0, kl0, kr0 = res["track_m"], res["scale_left"], res["scale_right"]
    if res.get("fehler"):
        print(f"Achtung: Messung wurde abgebrochen ({res['fehler']}) – nur vollständige Abschnitte zählen")
    segs, dreh, fahrt = [], 0, 0
    print(f"{'Abschnitt':<26}{'links':>8}{'rechts':>8}  {'Räder':>7}{'IMU':>8}{'Kamera':>8}")
    for s in abschnitte(res):
        th_r = (kr0 * s["sr"] - kl0 * s["sl"]) / track0
        cam = f"{math.degrees(s['cam']):+7.1f}°" if s["cam"] is not None else "      –"
        print(f"{s['name']:<26}{s['sl']*100:+7.1f} {s['sr']*100:+7.1f}  {math.degrees(th_r):+6.1f}°"
              f"{math.degrees(s['imu']):+7.1f}°{cam}")
        ist_dreh = s["sl"] * s["sr"] < 0 and abs(s["sl"] - s["sr"]) > 0.05
        ist_fahrt = s["sl"] * s["sr"] > 0 and abs(s["sl"] + s["sr"]) > 0.10
        if not (ist_dreh or ist_fahrt):
            continue                                      # Stand: zu wenig Information
        ref = a.ref
        if (s["cam"] is None and ref != "gerade") or (ist_dreh and a.ref != "kamera"):
            ref = "imu"                                   # schnelle Drehung: Kamera unsicher
        th = {"kamera": s["cam"], "imu": s["imu"], "gerade": 0.0 if ist_fahrt else s["imu"],
              "mittel": (s["imu"] + (s["cam"] if s["cam"] is not None else s["imu"])) / 2}[ref]
        if ist_dreh and s["cam"] is not None and abs(s["cam"] - s["imu"]) > 0.05 * abs(s["imu"]):
            print(f"    (Kamera weicht bei der Drehung > 5 % von der IMU ab – IMU verwendet)")
        segs.append((s["sl"], s["sr"], th))
        dreh += ist_dreh
        fahrt += ist_fahrt
    if not segs:
        sys.exit("keine verwertbaren Abschnitte")
    fit_track = dreh > 0 and not a.spur_fest
    tape = None
    if a.dist_cm is not None:
        idx = next((k for k, s in enumerate(segs) if s[0] > 0 and s[1] > 0), None)
        if idx is None:
            sys.exit("--dist-cm: kein Vorwärtsabschnitt")
        tape = (idx, a.dist_cm / 100)
    mean = (kl0 + kr0) / 2
    kl, kr, track = calibrate_odom(segs, track0, mean_scale=mean, tape=tape, fit_track=fit_track)
    print(f"\n{fahrt} Fahrabschnitte, {dreh} Drehungen; Kursreferenz {a.ref}; Skala: "
          + (f"Maßband {a.dist_cm:.1f} cm" if tape else f"Mittelwert {mean:.4f} aus der Konfiguration"))
    print("Restfehler je Abschnitt (Kurs aus Rädern mit neuen Werten minus Referenz):")
    for sl, sr, th in segs:
        print(f"  {math.degrees((kr * sr - kl * sl) / track - th):+.2f}°")
    print("\nIn config.toml eintragen:\n  [odom]\n  scale_left = %.4f\n  scale_right = %.4f" % (kl, kr))
    if fit_track:
        print("  track_m = %.4f" % track)
    else:
        print("  (track_m bleibt %.4f – keine Drehung gemessen oder --spur-fest)" % track)
    if abs(kl - 1) > 0.08 or abs(kr - 1) > 0.08:
        print("WARNUNG: Faktor weicht mehr als 8 % ab – Messung prüfen (Schlupf, falsche Strecke?)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("was", choices=["messen", "rechne"])
    ap.add_argument("--v", type=float, default=0.08, help="Fahrbefehl (normiert), R-1.1.6: 0,08 ≈ 10 cm/s")
    ap.add_argument("--w", type=float, default=0.10, help="Drehbefehl auf der Stelle (normiert)")
    ap.add_argument("--dreh-grad", type=float, default=360.0, help="0 = nicht drehen")
    ap.add_argument("--strecke-cm", type=float, default=30.0)
    ap.add_argument("--hin-her", type=int, default=2, help="0 = nicht fahren")
    ap.add_argument("--nur-vor", action="store_true", help="nur einmal vor (für das Maßband)")
    ap.add_argument("--ohne-halten", action="store_true",
                    help="kein Kurshalten (Firmware-Yaw hat 2°/s-Totzone, sieht langsame Drift nicht)")
    ap.add_argument("--warten", type=float, default=300.0, help="s bis zum Start per „Hand“")
    ap.add_argument("--out", default="/tmp/odomcal.json")
    ap.add_argument("--in", dest="inp", default="/tmp/odomcal.json")
    ap.add_argument("--dist-cm", type=float, default=None, help="Maßband, erster Vorwärtsabschnitt")
    ap.add_argument("--ref", choices=["gerade", "kamera", "imu", "mittel"], default="gerade",
                    help="Kursreferenz für Fahrten: gerade = sichtbar geradeaus gefahren (Kurs 0); "
                         "Kamera-Kurs ist bei Vorwärtsfahrt verfälscht (2026-11-04: −8,9° auf 2 m) (Drehungen: IMU)")
    ap.add_argument("--spur-fest", action="store_true", help="track_m nicht anpassen")
    ap.add_argument("--config", default="/opt/xgonav/config.toml")
    a = ap.parse_args()
    cfg = Config.load(a.config if Path(a.config).exists() else None)
    if a.was == "messen":
        if not 0 < a.v <= 0.15:
            sys.exit("--v bitte zwischen 0 und 0.15 (getestete Geschwindigkeiten)")
        if not 0.06 <= a.w <= 0.15:
            sys.exit("--w bitte zwischen 0.06 und 0.15")
        if not 0 < a.strecke_cm <= 250:
            sys.exit("--strecke-cm bitte zwischen 0 und 250")
    res = messen(a, cfg) if a.was == "messen" else json.loads(Path(a.inp).read_text())
    rechne(res, a)


if __name__ == "__main__":
    main()
