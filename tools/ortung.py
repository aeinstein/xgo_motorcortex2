#!/usr/bin/env python3
"""Schallrichtung ansehen und den Mikrofonabstand messen (xgonav/ortung.py).

  python tools/ortung.py live [--config /etc/xgonav.toml]   am Rider: eigenes arecord, je Geräusch eine Zeile
  python tools/ortung.py wav aufnahme.wav                   Stereo-Aufnahme (arecord ... -c 2) auswerten

hw:CARD=wm8960soundcard,DEV=0 lässt sich nur einmal öffnen: läuft die Ortung in xgonav (Modus idle/pet), vorher den Modus wechseln
oder dort GET /status lesen ("ortung"). Mikrofonabstand: Geräusch genau von der Seite (90°) machen, die größte
Laufzeit |tau| ablesen -> mic_dist_m = |tau| x 343 m/s; hat links das falsche Vorzeichen: sign = -1.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
import wave

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from xgonav.config import Config  # noqa: E402
from xgonav.ortung import SCHALL_MPS, Ortung  # noqa: E402


def zeile(s) -> str:
    pos = int(round((1.0 - s.winkel_deg / 90.0) * 20))             # links steht links
    balken = "".join("|" if i == 20 else ("#" if i == pos else ".") for i in range(41))
    return (f"{s.t:8.2f} s  {balken}  {s.winkel_deg:+6.1f}°  tau {s.tau_us:+7.1f} us "
            f"(= {abs(s.tau_us) * 1e-6 * SCHALL_MPS * 100:4.1f} cm)  Güte {s.guete:.2f}  +{s.pegel_db:.0f} dB")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("was", choices=["live", "wav"])
    ap.add_argument("datei", nargs="?")
    ap.add_argument("--config")
    ap.add_argument("--set", action="append", default=[], metavar="name=wert", help="z. B. mic_dist_m=0.06")
    a = ap.parse_args()
    cfg = Config.load(a.config).ortung
    for kv in a.set:
        k, v = kv.split("=", 1)
        setattr(cfg, k, type(getattr(cfg, k))(v))
    if a.was == "wav":
        with wave.open(a.datei, "rb") as w:
            if w.getnchannels() != 2 or w.getsampwidth() != 2:
                sys.exit("erwartet: Stereo, 16 Bit")
            cfg.rate = w.getframerate()
            pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").reshape(-1, 2)
        o = Ortung(cfg)
        n = len(pcm) // cfg.block * cfg.block
        ev = o.feed(np.ascontiguousarray(pcm[:n]), n / cfg.rate)
        for s in ev:
            print(zeile(s))
        print(f"{len(ev)} Geräusche, {o.n_schwach} ohne klare Richtung verworfen, Kern {o.kern.name}")
        return 0
    o = Ortung(cfg)
    cmd = ["arecord", "-q", "-D", cfg.device, "-f", "S16_LE", "-r", str(cfg.rate), "-c", "2", "-t", "raw"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=cfg.block * 4)
    print(f"Kern {o.kern.name}, mic_dist_m {cfg.mic_dist_m} (Suche bis ±{o.tau_lim * cfg.tau_margin * 1e6:.0f} us); "
          f"Strg+C beendet", flush=True)
    t0 = time.monotonic()
    try:
        while True:
            data = p.stdout.read(cfg.block * 4)
            if len(data) < cfg.block * 4:
                sys.exit(f"arecord liefert nichts ({cfg.device} belegt?)")
            s = o._block(np.frombuffer(data, dtype="<i2").reshape(-1, 2), time.monotonic() - t0)
            if s is not None:
                print(zeile(s), flush=True)
    except KeyboardInterrupt:
        return 0
    finally:
        p.terminate()


if __name__ == "__main__":
    sys.exit(main())
