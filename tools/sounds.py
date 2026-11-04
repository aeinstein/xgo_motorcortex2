"""Geräusche für xgonav erzeugen (WAV, 22050 Hz, mono, 16 bit):
  freude.wav – freudig: drei aufsteigende, hüpfende Zwitscher-Töne (Person erkannt)
  trauer.wav – traurig: zwei fallende "Wah"-Töne mit Vibrato, der zweite lang (Person verloren)

  python tools/sounds.py [ZIELVERZEICHNIS]      # Standard: sounds/
Nur numpy und die Standardbibliothek, läuft auch auf dem Rider.
"""
from __future__ import annotations

import os
import sys
import wave

import numpy as np

RATE = 22050


def _env(n: int, attack: float = 0.01, release: float = 0.08) -> np.ndarray:
    """Hüllkurve: weich ein- und ausblenden (kein Knacken)."""
    t = np.arange(n) / RATE
    dur = n / RATE
    e = np.ones(n)
    a = t < attack
    e[a] = t[a] / attack
    r = t > dur - release
    e[r] = np.clip((dur - t[r]) / release, 0.0, 1.0)
    return e


def _tone(f0: float, f1: float, dur: float, vib_hz: float = 0.0, vib_depth: float = 0.0,
          harm: float = 0.25) -> np.ndarray:
    """Ton mit Frequenzverlauf f0 -> f1 (exponentiell), optional Vibrato, etwas Oberton."""
    n = int(dur * RATE)
    t = np.arange(n) / RATE
    f = f0 * (f1 / f0) ** (t / dur)
    if vib_hz:
        f = f * (1.0 + vib_depth * np.sin(2 * np.pi * vib_hz * t))
    ph = 2 * np.pi * np.cumsum(f) / RATE
    y = np.sin(ph) + harm * np.sin(2 * ph) + 0.5 * harm * np.sin(3 * ph)
    return y * _env(n)


def _silence(dur: float) -> np.ndarray:
    return np.zeros(int(dur * RATE))


def freude() -> np.ndarray:
    # C6 -> E6 -> G6, jeweils kurz nach oben gezogen ("hüpfend"), letzter Ton etwas länger
    parts = []
    for f, d in ((1047, 0.09), (1319, 0.09), (1568, 0.18)):
        parts += [_tone(f, f * 1.12, d, harm=0.2), _silence(0.035)]
    return np.concatenate(parts)


def trauer() -> np.ndarray:
    # zwei fallende "Wah": G4 -> E4, dann E4 -> B3 lang mit langsamem Vibrato
    return np.concatenate([
        _tone(392, 330, 0.28, vib_hz=5.0, vib_depth=0.01, harm=0.35),
        _silence(0.06),
        _tone(330, 247, 0.75, vib_hz=5.5, vib_depth=0.02, harm=0.35),
    ])


def write(path: str, y: np.ndarray, gain: float = 0.6) -> None:
    y = y / max(1e-9, float(np.abs(y).max())) * gain
    pcm = (y * 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(pcm.tobytes())


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "sounds"
    os.makedirs(out, exist_ok=True)
    for name, fn in (("freude.wav", freude), ("trauer.wav", trauer)):
        p = os.path.join(out, name)
        y = fn()
        write(p, y)
        print(f"{p}: {len(y) / RATE:.2f} s")


if __name__ == "__main__":
    main()
