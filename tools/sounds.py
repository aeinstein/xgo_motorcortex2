"""Geräusche für xgonav erzeugen (WAV, 22050 Hz, mono, 16 bit):
  freude.wav – freudig: drei aufsteigende, hüpfende Zwitscher-Töne (Person erkannt)
  trauer.wav – traurig: zwei fallende "Wah"-Töne mit Vibrato, der zweite lang (Person verloren)
  pet_<Ereignis>_<n>.wav – Modus pet (xgonav/pet.py), je Ereignis drei Tonhöhen

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


# ---- Modus pet: je Ereignis Varianten (Faktor k auf die Tonhöhe), damit es nicht mechanisch klingt
def traellern(k: float) -> np.ndarray:
    parts = []
    for f, d in ((1319, 0.07), (1568, 0.07), (1319, 0.07), (1760, 0.20)):
        parts += [_tone(f * k, f * k * 1.06, d, vib_hz=18.0, vib_depth=0.015, harm=0.2), _silence(0.02)]
    return np.concatenate(parts)


def quiek(k: float) -> np.ndarray:
    return np.concatenate([_tone(1400 * k, 3200 * k, 0.09, harm=0.3), _tone(3200 * k, 2500 * k, 0.11, vib_hz=30.0,
                                                                            vib_depth=0.03, harm=0.3)])


def kichern(k: float) -> np.ndarray:
    parts = []
    for f in (1500, 1600, 1700, 1600, 1500):
        parts += [_tone(f * k, f * k * 0.92, 0.055, harm=0.25), _silence(0.04)]
    return np.concatenate(parts)


def seufzen(k: float) -> np.ndarray:
    return _tone(620 * k, 300 * k, 0.8, vib_hz=4.0, vib_depth=0.008, harm=0.4)


def wimmern(k: float) -> np.ndarray:
    return np.concatenate([_tone(880 * k, 700 * k, 0.32, vib_hz=9.0, vib_depth=0.03, harm=0.3), _silence(0.05),
                           _tone(800 * k, 560 * k, 0.5, vib_hz=9.0, vib_depth=0.04, harm=0.3)])


def gurren(k: float) -> np.ndarray:
    return np.concatenate([_tone(520 * k, 780 * k, 0.22, vib_hz=26.0, vib_depth=0.05, harm=0.35), _silence(0.03),
                           _tone(780 * k, 980 * k, 0.28, vib_hz=26.0, vib_depth=0.05, harm=0.35)])


def hey(k: float) -> np.ndarray:
    return np.concatenate([_tone(700 * k, 1000 * k, 0.09, harm=0.4), _silence(0.02), _tone(1000 * k, 640 * k, 0.14, harm=0.4)])


def ohnmacht(k: float) -> np.ndarray:
    return _tone(1800 * k, 150 * k, 1.0, vib_hz=7.0, vib_depth=0.02, harm=0.3)


def aufwachen(k: float) -> np.ndarray:
    """Warnung vor dem Aufspringen, muss gut zu hören sein und dauert standup_warn_s."""
    parts = []
    for f in (500, 700, 950, 1250):
        parts += [_tone(f * k, f * k * 1.1, 0.25, harm=0.3), _silence(0.08)]
    return np.concatenate(parts)


PET = {"traellern": traellern, "quiek": quiek, "kichern": kichern, "seufzen": seufzen, "wimmern": wimmern,
       "gurren": gurren, "hey": hey, "ohnmacht": ohnmacht, "aufwachen": aufwachen}
VARIANTS = (1.0, 0.92, 1.09)


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
    for name, fn in PET.items():
        for i, k in enumerate(VARIANTS, 1):
            write(os.path.join(out, f"pet_{name}_{i}.wav"), fn(k))
    print(f"{len(PET) * len(VARIANTS)} Töne für den Modus pet")


if __name__ == "__main__":
    main()
