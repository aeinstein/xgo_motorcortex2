#!/usr/bin/env python3
"""Rider über die laufende xgonav-Instanz steuern (lokale Schnittstelle 127.0.0.1:8088). Läuft AM RIDER als root.

  python3 /root/rider_nav.py zustand              # Akku, Lage, Modus, Bildverarbeitung, Standreferenz, Dienste
  python3 /root/rider_nav.py start                # Bildverarbeitung an (falls aus) und auf Bilder warten
  python3 /root/rider_nav.py bild /tmp/rn.jpg     # Rohbild speichern (--overlay: mit Overlay)
  python3 /root/rider_nav.py aufstehen            # nur wenn er liegt: 253, warten bis er balanciert + Kalibrierung
  python3 /root/rider_nav.py fahr 0.10 0 2.0      # Handsteuerung: v w Sekunden (normiert, v>0 vor, w>0 links)
  python3 /root/rider_nav.py dreh 90              # um Grad drehen (+ = links), Kurs aus /status yaw_deg
  python3 /root/rider_nav.py absetzen             # idle, 254, warten bis er liegt (beendet record_status.py)
  python3 /root/rider_nav.py ende                 # absetzen + Bildverarbeitung aus

Jeder Fahrbefehl endet mit explizitem Stillstand (auch bei Strg+C oder Fehler) und bricht ab, sobald der
Rider nicht mehr balanciert. Rechenzeit am Pi: nur HTTP-Abfragen mit 10 Hz während einer Fahrt, sonst nichts.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

B = "http://127.0.0.1:8088"
P = "/proc/XGORider"
HZ = 10.0                  # Sollwerte für manual (HOLD_S 0,5 s) mit 10 Hz nachschicken
MIN_AKKU = 20              # darunter keine Fahrt


def req(path: str, post: bool = False, timeout: float = 3.0) -> tuple[int, bytes]:
    r = urllib.request.Request(B + path, data=b"" if post else None, method="POST" if post else "GET")
    try:
        with urllib.request.urlopen(r, timeout=timeout) as f:
            return f.status, f.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def post(path: str) -> dict:
    code, body = req(path, post=True)
    try:
        d = json.loads(body or b"{}")
    except ValueError:
        d = {}
    if code != 200:
        raise RuntimeError("%s -> %d %s" % (path, code, d.get("error", body[:200])))
    return d


def status() -> dict:
    code, body = req("/status")
    if code != 200:
        raise RuntimeError("/status -> %d" % code)
    return json.loads(body)


def proc(name: str) -> str:
    with open(os.path.join(P, name)) as f:
        return f.read().strip()


def proc_write(name: str, value) -> None:
    fd = os.open(os.path.join(P, name), os.O_WRONLY)
    try:
        os.write(fd, str(value).encode())
    finally:
        os.close(fd)


def balanciert() -> bool:
    return proc("state") == "1"


def akku() -> int | None:
    try:
        return int(proc("battery").split()[0])
    except (OSError, ValueError, IndexError):
        return None


def dienst(name: str) -> str:
    r = subprocess.run(["systemctl", "is-active", name], capture_output=True, text=True)
    return r.stdout.strip() or "?"


def warte(bed, timeout: float, schritt: float = 0.1) -> bool:
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if bed():
            return True
        time.sleep(schritt)
    return bed()


# ----------------------------------------------------------------------
def cmd_zustand(_a) -> int:
    s = status()
    hz = s.get("horizon") or {}
    out = {"akku_%": akku(), "state": proc("state"), "balanciert": balanciert(),
           "modus": s.get("mode"), "pipeline": s.get("pipeline"), "frame": s.get("frame"),
           "status": s.get("status"), "height_mm": s.get("height_mm"), "yaw_deg": s.get("yaw_deg"),
           "yaw_src": s.get("yaw_src"), "odom": s.get("odom"),
           "standreferenz": hz.get("zustand") if hz else None,
           "xgonav.service": dienst("xgonav"), "rc-control.service": dienst("rc-control")}
    print(json.dumps(out, ensure_ascii=False, indent=1))
    return 0


def cmd_start(a) -> int:
    if dienst("xgonav") != "active":
        print("xgonav.service läuft nicht – nicht selbst starten, Nutzer fragen")
        return 2
    s = status()
    if s.get("pipeline") != "läuft":
        post("/pipeline?on=1")
        print("Bildverarbeitung gestartet, warte auf Bilder …")
    f0 = s.get("frame") or 0
    seen = []
    ok = warte(lambda: seen.append(status().get("frame") or 0) or (seen[-1] > f0 + 5), a.timeout, 0.3)
    if not ok:
        print("keine neuen Bilder nach %.0f s (frame %s)" % (a.timeout, seen[-1] if seen else "?"))
        return 1
    print("Bilder laufen (frame %d)" % seen[-1])
    return 0


def cmd_bild(a) -> int:
    code, body = req("/frame?raw=%d" % (0 if a.overlay else 1), timeout=5)
    if code != 200:
        print("kein Bild (%d): %s" % (code, body[:200]))
        return 1
    with open(a.pfad, "wb") as f:
        f.write(body)
    print("Bild gespeichert:", a.pfad, len(body), "Bytes")
    return 0


def cmd_aufstehen(a) -> int:
    if balanciert():
        print("steht schon (state 1) – kein Aufspringen")
        return 0
    k = akku()
    if k is not None and k < MIN_AKKU:
        print("Akku %d %% < %d %% – kein Aufstehen" % (k, MIN_AKKU))
        return 2
    if dienst("rc-control") == "active":
        print("rc-control.service läuft – erst stoppen (systemctl stop rc-control)")
        return 2
    s = status()
    if s.get("pipeline") != "läuft":
        print("Bildverarbeitung aus – erst 'start' (Kalibrierung braucht Bilder)")
        return 2
    if s.get("mode") != "idle":
        post("/stop")
    print("Aufspringen (253) …")
    proc_write("action", 253)
    if not warte(balanciert, a.timeout):
        print("balanciert nach %.0f s nicht (state %s) – Nutzer fragen" % (a.timeout, proc("state")))
        return 1
    t_auf = time.monotonic()
    print("balanciert, warte auf Kalibrierung …")
    # Firmware: Aufschwung <= 2 s, dann (level_after_standup) Nivellier-Impuls 3 s ab 2,5 s nach dem Aufstehen.
    # xgonav ([horizon]): Standreferenz settle_s 3 s + window_s 3 s im Stillstand, Zustand "gültig".
    def fertig() -> bool:
        if not balanciert():
            raise RuntimeError("liegt wieder (state %s)" % proc("state"))
        if time.monotonic() - t_auf < a.min_s:
            return False
        hz = status().get("horizon")
        return not hz or hz.get("zustand") == "gültig"
    try:
        ok = warte(fertig, a.kal_timeout, 0.3)
    except RuntimeError as e:
        print("Abbruch:", e)
        return 1
    hz = status().get("horizon") or {}
    if not ok:
        print("Kalibrierung nicht fertig nach %.0f s (Standreferenz %s, verworfen %s) – Nutzer fragen"
              % (a.kal_timeout, hz.get("zustand"), hz.get("verworfen")))
        return 1
    print("bereit: %.1f s nach dem Aufstehen, Standreferenz %s (ref %s°, Streuung %s°), Höhe %s mm"
          % (time.monotonic() - t_auf, hz.get("zustand", "—"), hz.get("ref_deg"), hz.get("std_deg"),
             status().get("height_mm")))
    return 0


def _bereit_zum_fahren() -> None:
    if not balanciert():
        raise RuntimeError("Rider balanciert nicht – erst aufstehen")
    k = akku()
    if k is not None and k < MIN_AKKU:
        raise RuntimeError("Akku %d %% < %d %%" % (k, MIN_AKKU))
    if status().get("mode") != "manual":
        post("/mode?name=manual")
        warte(lambda: status().get("mode") == "manual", 2.0)


def _halt() -> None:
    for _ in range(3):
        try:
            post("/drive?v=0&w=0")
        except Exception as e:      # noqa: BLE001 – Stillstand so oft wie möglich versuchen
            print("Stillstand per /drive fehlgeschlagen:", e)
        time.sleep(0.05)
    try:
        post("/stop")               # idle + robot.stop()
    except Exception as e:          # noqa: BLE001
        print("/stop fehlgeschlagen:", e)


def _pose() -> str:
    s = status()
    o = s.get("odom") or {}
    return "Kurs %s° (%s), Strecke %s m" % (s.get("yaw_deg"), s.get("yaw_src"), o.get("m"))


def cmd_fahr(a) -> int:
    v, w = max(-1.0, min(1.0, a.v)), max(-1.0, min(1.0, a.w))
    try:
        _bereit_zum_fahren()
        print("vorher:", _pose())
        t_end = time.monotonic() + a.sekunden
        while time.monotonic() < t_end:
            if not balanciert():
                raise RuntimeError("liegt (state %s)" % proc("state"))
            post("/drive?v=%.3f&w=%.3f" % (v, w))
            time.sleep(1.0 / HZ)
    except (RuntimeError, KeyboardInterrupt) as e:
        print("Abbruch:", e or "Strg+C")
        _halt()
        return 1
    _halt()
    time.sleep(0.5)
    print("nachher:", _pose())
    return 0


def cmd_dreh(a) -> int:
    try:
        _bereit_zum_fahren()
        y0 = status().get("yaw_deg")
        if y0 is None:
            raise RuntimeError("kein yaw_deg in /status (Bildverarbeitung an?)")
        print("vorher:", _pose(), "-> %+.1f° (Ziel %.1f°)" % (a.grad, y0 + a.grad))
        # Gedrehten Winkel aufsummieren (je Schritt auf ±180° umgerechnet), damit auch ±360° und mehr gehen,
        # egal ob yaw_deg durchläuft oder bei ±180° umspringt.
        gedreht, y_alt = 0.0, y0
        t_end = time.monotonic() + (a.timeout if a.timeout is not None else 10.0 + abs(a.grad) / 20.0)
        while True:
            if not balanciert():
                raise RuntimeError("liegt (state %s)" % proc("state"))
            if time.monotonic() > t_end:
                raise RuntimeError("Zeit abgelaufen")
            y = status().get("yaw_deg")
            gedreht += (y - y_alt + 180.0) % 360.0 - 180.0
            y_alt = y
            err = a.grad - gedreht
            if abs(err) <= a.tol:
                break
            w = math.copysign(min(a.w, max(0.08, 0.01 * abs(err))), err)
            post("/drive?v=0&w=%.3f" % w)
            time.sleep(1.0 / HZ)
    except (RuntimeError, KeyboardInterrupt) as e:
        print("Abbruch:", e or "Strg+C")
        _halt()
        return 1
    _halt()
    time.sleep(1.0)                 # Rider dreht nach dem Stopp etwas nach
    gedreht += (status().get("yaw_deg") - y_alt + 180.0) % 360.0 - 180.0
    print("nachher:", _pose(), "| gedreht %+.1f° (Soll %+.1f°)" % (gedreht, a.grad))
    return 0


def cmd_absetzen(a) -> int:
    try:
        post("/stop")
    except Exception as e:          # noqa: BLE001 – absetzen trotzdem
        print("/stop fehlgeschlagen:", e)
    if not balanciert():
        print("liegt schon")
        return 0
    print("Absetzen (254) …")
    proc_write("action", 254)
    if not warte(lambda: not balanciert(), a.timeout):
        print("balanciert nach %.0f s noch – Nutzer fragen" % a.timeout)
        return 1
    print("liegt (state %s), Akku %s %%" % (proc("state"), akku()))
    return 0


def cmd_ende(a) -> int:
    rc = cmd_absetzen(a)
    s = status()
    if s.get("pipeline") == "läuft":
        post("/pipeline?on=0")
        print("Bildverarbeitung aus")
    return rc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("zustand").set_defaults(f=cmd_zustand)
    p = sp.add_parser("start"); p.add_argument("--timeout", type=float, default=20.0); p.set_defaults(f=cmd_start)
    p = sp.add_parser("bild"); p.add_argument("pfad", nargs="?", default="/tmp/rider_nav.jpg")
    p.add_argument("--overlay", action="store_true"); p.set_defaults(f=cmd_bild)
    p = sp.add_parser("aufstehen")
    p.add_argument("--timeout", type=float, default=10.0, help="bis er balanciert")
    p.add_argument("--min-s", type=float, default=6.0, help="mindestens so lange nach dem Aufstehen warten")
    p.add_argument("--kal-timeout", type=float, default=30.0, help="bis die Standreferenz gültig ist")
    p.set_defaults(f=cmd_aufstehen)
    p = sp.add_parser("fahr"); p.add_argument("v", type=float); p.add_argument("w", type=float)
    p.add_argument("sekunden", type=float); p.set_defaults(f=cmd_fahr)
    p = sp.add_parser("dreh"); p.add_argument("grad", type=float)
    p.add_argument("--w", type=float, default=0.20, help="höchste Drehrate (normiert)")
    p.add_argument("--tol", type=float, default=3.0)
    p.add_argument("--timeout", type=float, default=None, help="Standard: 10 s + 1 s je 20°")
    p.set_defaults(f=cmd_dreh)
    for n, f in (("absetzen", cmd_absetzen), ("ende", cmd_ende)):
        p = sp.add_parser(n); p.add_argument("--timeout", type=float, default=5.0); p.set_defaults(f=f)
    a = ap.parse_args()
    return a.f(a)


if __name__ == "__main__":
    sys.exit(main())
