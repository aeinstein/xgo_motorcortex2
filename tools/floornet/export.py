#!/usr/bin/env python3
"""FloorNet für den C++-Kern exportieren: Gewichte ohne BatchNorm plus Testvektor (GPU-Rechner oder PC mit torch).

  python tools/floornet/export.py lauf/floornet.pt --out models/floornet.bin

Aufbau der Datei (little endian, float32): "FNET", Version, Spalten, Fächer (je uint32); Mitten der Fächer;
Gewichte je Schicht wie net.fold (w, dann b, ohne b bei der tiefenweisen Faltung); erwartete Logits des
Testbilds; "ENDE". Passt dazu: xgonav/native/floornet.cpp.
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))

from net import CROP_H, CROP_Y, DOWN, IN_H, IN_W, NBINS, NCOLS, FloorNet, bin_centers, fold, run_ops  # noqa: E402

VERSION = 1


def test_frame() -> np.ndarray:
    """Künstliches Testbild, dasselbe wie test_frame in floornet.cpp: Verläufe und Kanten."""
    y, x, c = np.mgrid[0:480, 0:640, 0:3]
    return (((x * 3 + y * 5 + c * 40) ^ ((x // 32) * (y // 24) * 17)) & 255).astype(np.uint8)


def to_input(frame: np.ndarray) -> torch.Tensor:
    """Vollbild BGR -> Netzeingabe, gerechnet wie preprocess im C++-Kern: 2x2 mitteln, ohne Runden auf 8 Bit."""
    crop = frame[CROP_Y:CROP_Y + CROP_H, :IN_W * DOWN].astype(np.float32)
    x = crop.reshape(IN_H, DOWN, IN_W, DOWN, 3).mean(axis=(1, 3)) / 255.0 - 0.5
    return torch.as_tensor(x.astype(np.float32)).permute(2, 0, 1)[None]


def layer_arrays(ops: list[dict]) -> list[np.ndarray]:
    """Gewichte in Dateireihenfolge."""
    out = []
    for op in ops:
        if op["kind"] == "collapse":
            continue
        out.append(op["w"].detach().cpu().numpy())
        if op.get("b") is not None:
            out.append(op["b"].detach().cpu().numpy())
    return out


def write(net: FloorNet, path: str) -> np.ndarray:
    """Datei schreiben, Rückgabe: erwartete Logits des Testbilds."""
    if net.variant != {"dil": [1, 1], "extra": 0}:
        raise ValueError("Der C++-Kern kennt nur das Standardnetz, nicht die Variante %s" % net.variant)
    net = net.cpu().eval()
    ops = fold(net)
    logits = run_ops(ops, to_input(test_frame()))[0].numpy().astype(np.float32)
    with open(path, "wb") as f:
        f.write(b"FNET")
        f.write(struct.pack("<3I", VERSION, NCOLS, NBINS))
        f.write(bin_centers().astype("<f4").tobytes())
        for a in layer_arrays(ops):
            f.write(a.astype("<f4").tobytes())
        f.write(logits.astype("<f4").tobytes())
        f.write(b"ENDE")
    return logits


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    net = FloorNet()
    net.load_state_dict(torch.load(a.model, map_location="cpu"))
    write(net, a.out)
    print("%s: %d Byte" % (a.out, Path(a.out).stat().st_size))
    return 0


if __name__ == "__main__":
    sys.exit(main())
