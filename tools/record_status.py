#!/usr/bin/env python3
"""Am Rider: jedes Overlay-Bild aus der laufenden xgonav-Instanz mitschneiden, dazu /status je Bild sowie
imu_log und wheels_log des Treibers. Liest nur, steuert nichts.
Endet nach SEKUNDEN oder sofort, wenn der Rider abgesetzt wird (/proc/XGORider/state != 1); liegt er beim
Start, beginnt sie nicht. --neben startet einen zweiten Leser, der mit der Aufnahme endet.

  python3 tools/record_status.py /tmp/aufnahme 60        # 60 s; danach scp/tar zum PC
  python tools/horizon_replay.py /pfad/aufnahme            # am PC: Nick gegen ein Band in Kamerahöhe

Ergebnis: <idx>.jpg (Overlay ohne Text) und daten.json mit
  frames  [{idx, t (Bildzeit, CLOCK_MONOTONIC), t_hol}]
  status  [{idx, t, h (horizon), cmd, pitch_deg, src, mode, ego, bal}]   (~50/s, mehrere je Bild möglich)
  imu     [[seq, t_ns, roll, pitch, yaw] (Milligrad, Strings)]
  raeder  [[seq, t_ns, ticks11, ticks21, fw_ms]]
Am CM4 schafft das alle Bilder bei 24 fps (2026-11-07: 1438 von 1439 in 60 s).
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import threading
import time
import urllib.request


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ziel")
    ap.add_argument("sekunden", type=float)
    ap.add_argument("--url", default="http://127.0.0.1:8088")
    ap.add_argument("--proc", default="/proc/XGORider")
    ap.add_argument("--neben", default="", help="Befehl, der zeitgleich läuft und mit der Aufnahme endet (SIGINT), "
                                                "z. B. \"python3 /root/imudiag.py 600 /tmp/diag.csv\"")
    a = ap.parse_args()
    B = a.url
    os.makedirs(a.ziel, exist_ok=True)
    stop = threading.Event()
    frames, stati, imu, raeder = [], [], {}, {}

    def bilder():
        last = -1
        while not stop.is_set():
            try:
                with urllib.request.urlopen(B + "/frame?after=%d" % last, timeout=2) as r:
                    body = r.read()
                    idx, t = int(r.headers["X-Frame"]), float(r.headers["X-Frame-T"])
            except Exception as e:
                print("Bild:", e, flush=True)
                time.sleep(0.1)
                continue
            if idx == last:
                continue
            last = idx
            with open(os.path.join(a.ziel, "%06d.jpg" % idx), "wb") as f:
                f.write(body)
            frames.append({"idx": idx, "t": t, "t_hol": time.monotonic()})

    def status():
        while not stop.is_set():
            try:
                s = json.loads(urllib.request.urlopen(B + "/status", timeout=2).read())
                stati.append({"idx": s.get("frame"), "t": s.get("frame_t"), "h": s.get("horizon"), "cmd": s.get("cmd"),
                              "pitch_deg": s.get("pitch_deg"), "src": s.get("pitch_src"), "mode": s.get("mode"),
                              "ego": s.get("ego"), "bal": s.get("balancing")})
            except Exception as e:
                print("Status:", e, flush=True)
            time.sleep(0.02)

    def logs():
        while not stop.is_set():
            for name, ziel in (("imu_log", imu), ("wheels_log", raeder)):
                try:
                    for ln in open(os.path.join(a.proc, name)).read().splitlines():
                        p = ln.split()
                        if p and p[0].isdigit():
                            ziel[int(p[0])] = p
                except OSError:
                    pass
            time.sleep(0.3)

    def balanciert() -> bool:
        try:
            return open(os.path.join(a.proc, "state")).read().strip() == "1"
        except OSError:
            return False

    # Vereinbarung mit dem Nutzer (2026-11-07): Absetzen beendet die Aufnahme, immer.
    if not balanciert():
        print("ABBRUCH: Rider balanciert nicht (abgesetzt), keine Aufnahme", flush=True)
        return 1
    neben = None
    if a.neben:
        neben = subprocess.Popen(shlex.split(a.neben))
        print("nebenher gestartet:", a.neben, flush=True)
    ths = [threading.Thread(target=f, daemon=True) for f in (bilder, status, logs)]
    for th in ths:
        th.start()
    t0 = time.monotonic()
    grund = "Zeit um"
    try:
        while time.monotonic() - t0 < a.sekunden:
            time.sleep(0.2)
            if not balanciert():
                grund = "abgesetzt"
                break
            if int((time.monotonic() - t0) * 5) % 5 == 0:
                print("%3.0f s: %d Bilder, Modus %s" % (time.monotonic() - t0, len(frames),
                                                       stati[-1]["mode"] if stati else "?"), flush=True)
    finally:
        stop.set()
        if neben is not None and neben.poll() is None:
            neben.send_signal(signal.SIGINT)        # SIGINT: Python-Leser schließen ihre Datei sauber
            try:
                neben.wait(timeout=5)
            except subprocess.TimeoutExpired:
                neben.kill()
    print("Ende nach %.1f s: %s" % (time.monotonic() - t0, grund), flush=True)
    for th in ths:
        th.join(timeout=3)
    with open(os.path.join(a.ziel, "daten.json"), "w") as f:
        json.dump({"frames": frames, "status": stati,
                   "imu": [imu[k] for k in sorted(imu)], "raeder": [raeder[k] for k in sorted(raeder)]}, f)
    print("fertig:", len(frames), "Bilder,", len(stati), "Status,", len(imu), "IMU,", len(raeder), "Rad", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
