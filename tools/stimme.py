#!/usr/bin/env python3
"""Stimmproben am PC prüfen (xgonav/stimme.py).

  python tools/stimme.py sittich ein.wav aus.wav [--set speed=1.8,band_lo=1200]   Filter anhören
  python tools/stimme.py vergleich a.wav b.wav ...                                 DTW-Abstände als Tabelle
  python tools/stimme.py lernen ordner/*.wav                                       Ablauf wie am Rider, in einem tmp-Verzeichnis
  python tools/stimme.py strom aufnahme.wav [--aus ordner] [--set vad_hang_s=0.6]  lange Stereo-Aufnahme (48 kHz) wie am
                                                                                   Rider in Äußerungen schneiden, vergleichen
Aufnahme am Rider (Modus manual, sonst hält die Ortung hw:0,0):
  arecord -D hw:0,0 -f S16_LE -r 48000 -c 2 -d 120 /root/stimmen.wav

Eingaben: mono oder stereo, 16 oder 48 kHz (z. B. scp root@<rider>:/opt/xgonav/stimmen/kandidaten/*.wav .).
Treffer gelten bis [stimme] match_dist; die Tabelle zeigt, wo die Grenze für echte Stimmen liegen sollte.
"""
from __future__ import annotations

import argparse
import dataclasses
import os
import sys
import tempfile
import wave

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from xgonav import stimme  # noqa: E402
from xgonav.config import Config  # noqa: E402


def _cfg(setz: str | None):
    c = Config().stimme
    for kv in filter(None, (setz or "").split(",")):
        k, v = kv.split("=")
        c = dataclasses.replace(c, **{k: type(getattr(c, k))(v)})
    return c


def _stereo48(path: str) -> np.ndarray:
    x, rate = stimme.read_wav(path)
    if rate == stimme.RATE:                          # 16 kHz: dreifach auffüllen (linear), damit derselbe Weg rechnet
        t = np.arange(len(x) * 3) / 3.0
        x = np.interp(t, np.arange(len(x)), x)
    elif rate != stimme.RATE_IN:
        raise SystemExit(f"{path}: {rate} Hz, nur 16 oder 48 kHz")
    return np.stack([x, x], 1).clip(-32768, 32767).astype(np.int16)


def _strom(a, c, k) -> None:
    from xgonav.ortung import Ortung
    with wave.open(a.dateien[0], "rb") as w:
        if w.getframerate() != stimme.RATE_IN or w.getnchannels() != 2:
            raise SystemExit("strom: nur Stereo mit 48 kHz")
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").reshape(-1, 2)
    ocfg = Config().ortung
    o = Ortung(ocfg)
    st = stimme.Stimmen(dataclasses.replace(c, dir=tempfile.mkdtemp()), ocfg.rate, ocfg.block)
    segs, zeiten = [], []
    t = [0.0]
    st._submit = lambda seg: (segs.append(seg), zeiten.append(t[0]))
    o.horcher = st
    st.aufnahme = True
    b = ocfg.block
    for i in range(len(pcm) // b):
        t[0] = (i + 1) * b / stimme.RATE_IN
        o.feed(pcm[i * b:(i + 1) * b], t[0])
    F, names = [], []
    for i, (seg, t1) in enumerate(zip(segs, zeiten)):
        x = k.mono16k(seg)
        feat, voiced = k.mfcc(x)
        ok = voiced >= c.voiced_min and len(feat) * stimme.HOP / stimme.RATE >= 0.5 * c.min_s
        name = f"s{i:02d}@{t1 - len(seg) / stimme.RATE_IN:5.1f}s"
        print(f"{name}  {len(seg) / stimme.RATE_IN:.2f} s, Merkmale {len(feat) / 100:.2f} s, stimmhaft {voiced:.2f}"
              f"{'' if ok else '  verworfen'}")
        if a.aus:
            os.makedirs(a.aus, exist_ok=True)
            stimme.write_wav(os.path.join(a.aus, f"s{i:02d}.wav"), x, stimme.RATE)
        if ok:
            F.append(feat)
            names.append(f"s{i:02d}")
    print(" " * 4 + "".join(f"{n:>6s}" for n in names))
    for i, fi in enumerate(F):
        row = []
        for j, fj in enumerate(F):
            q = len(fi) / max(1, len(fj))
            if i == j:
                row.append("-")
            elif not 1 / c.len_ratio <= q <= c.len_ratio:
                row.append("L")
            else:
                d = k.dtw(fi, fj, c.dtw_band)
                row.append(f"{d:.1f}{'*' if d <= c.match_dist else ''}")
        print(f"{names[i]:4s}" + "".join(f"{r:>6s}" for r in row))
    print(f"* = Treffer (match_dist {c.match_dist}), L = Länge zu verschieden")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("was", choices=["sittich", "vergleich", "lernen", "strom"])
    ap.add_argument("--aus", help="strom: Äußerungen als WAV hierhin")
    ap.add_argument("dateien", nargs="+")
    ap.add_argument("--set", help="[stimme]-Werte, z. B. speed=1.8,match_dist=7")
    a = ap.parse_args()
    c = _cfg(a.set)
    k = stimme.make_kern(c.flat_max, per_min=c.per_min, trim_db=c.trim_db, laut_db=c.laut_db)
    print(f"Kern {k.name}")
    if a.was == "sittich":
        if len(a.dateien) != 2:
            raise SystemExit("sittich ein.wav aus.wav")
        x = k.mono16k(_stereo48(a.dateien[0]))
        x = k.tor(x, c.gate_db, c.knee_db)
        y = k.sittich(x, c.speed, c.band_lo, c.band_hi, c.vibrato_hz, c.vibrato_ms, c.drive, 10 ** (c.peak_db / 20))
        stimme.write_wav(a.dateien[1], y, stimme.RATE_OUT, 32767.0)
        print(f"{a.dateien[1]}: {len(y) / stimme.RATE_OUT:.2f} s")
    elif a.was == "vergleich":
        F = []
        for f in a.dateien:
            feat, voiced = k.mfcc(k.mono16k(_stereo48(f)))
            F.append(feat)
            print(f"{os.path.basename(f):30s} {len(feat) / 100:.2f} s  stimmhaft {voiced:.2f}")
        names = [os.path.basename(f)[:12] for f in a.dateien]
        print(" " * 13 + " ".join(f"{n:>12s}" for n in names))
        for i, fi in enumerate(F):
            row = []
            for j, fj in enumerate(F):
                q = len(fi) / max(1, len(fj))
                if i == j:
                    row.append("-")
                elif not 1 / c.len_ratio <= q <= c.len_ratio:
                    row.append("Länge")
                else:
                    d = k.dtw(fi, fj, c.dtw_band)
                    row.append(f"{d:.1f}{'*' if d <= c.match_dist else ''}")
            print(f"{names[i]:12s} " + " ".join(f"{r:>12s}" for r in row))
        print(f"* = Treffer (match_dist {c.match_dist})")
    elif a.was == "strom":
        _strom(a, c, k)
    else:
        with tempfile.TemporaryDirectory() as d:
            st = stimme.Stimmen(dataclasses.replace(c, dir=d))
            for f in a.dateien:
                print(f"{os.path.basename(f):30s} {st.lerne(_stereo48(f))}")
            print(f"bestätigt {st.ids}, Proben {len(st.kandidaten)}")


if __name__ == "__main__":
    main()
