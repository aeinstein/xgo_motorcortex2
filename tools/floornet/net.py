"""FloorNet: Spaltennetz f2_s wie in bench_kern.cpp gemessen (320x171, ~18 MMACs, ~30 kParam).

Ausgabe je Spalte (80): Klassenverteilung über 64 Zeilenfächer, oben fein (1 Zeile bei 2 m sind ~10 cm),
unten grob. Vorgaben des C++-Kerns: nach der tiefenweisen Faltung kein ReLU und kein Bias, Residuum vor dem
ReLU der 1x1-Schicht, Kanalzahlen durch 8 teilbar. BatchNorm gibt es nur im Training, fold() rechnet sie in
die Gewichte ein; was dabei nach der tiefenweisen Faltung als Konstante bleibt, wandert in den Bias der
folgenden 1x1-Schicht.

Varianten (2026-11-10, nur Training und Auswertung; der C++-Kern und export.py kennen nur das Standardnetz):
dil = waagerechte Dilatation der tiefenweisen Faltung in Block 5 und 6 (mehr seitlicher Kontext, gleiche Rechenzeit),
extra = weitere Residualblöcke auf 80 x 11 hinter Block 6 (je ~2 MMACs).
"""
from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

CROP_Y, CROP_H, DOWN = 137, 342, 2      # Ausschnitt in Vollbildzeilen, Verkleinerung
IN_W, IN_H = 320, 171
NCOLS, NBINS = 80, 64
ROW_TOP, ROW_BOTTOM = 205.0, 480.0      # Zeilenfächer (Vollbild); darunter zählt als letztes Fach
BIN_RATIO = 1.03                        # jedes Fach so viel breiter als das darüber: 1,5 px oben, ~10 px unten
NORM_LEVEL, NORM_MIN, NORM_MAX = 0.3, 0.5, 8.0      # Helligkeitsnormierung: Zielwert, Grenzen des Faktors


def prep(x: torch.Tensor, norm: bool) -> torch.Tensor:
    """Eingabe (0..1, N x 3 x IN_H x IN_W) -> Netz (-0.5..0.5). norm: Helligkeit über die untere Hälfte des
    Ausschnitts (meist Boden; oben blendet oft eine Lampe) auf einen festen Wert bringen."""
    if norm:
        m = x[:, :, IN_H // 2:, :].mean(dim=(1, 2, 3), keepdim=True)
        x = (x * (NORM_LEVEL / m.clamp_min(1e-3)).clamp(NORM_MIN, NORM_MAX)).clamp(0.0, 1.0)
    return x - 0.5


def bin_edges() -> np.ndarray:
    """Grenzen der Fächer (NBINS + 1 Werte, aufsteigend, Vollbildzeilen)."""
    w = BIN_RATIO ** np.arange(NBINS)
    return ROW_TOP + np.concatenate([[0.0], np.cumsum(w)]) * (ROW_BOTTOM - ROW_TOP) / w.sum()


def bin_centers() -> np.ndarray:
    e = bin_edges()
    return (e[:-1] + e[1:]) / 2


OUTLIER_PX, OUTLIER_RATIO, OUTLIER_COLS, OUTLIER_BINS = 25.0, 0.3, 7, 2     # wie xgonav/config.py und floornet.cpp


def decode(logits: torch.Tensor, outlier_px: float = 0.0, outlier_ratio: float = OUTLIER_RATIO) -> torch.Tensor:
    """Logits (N, NBINS, NCOLS) -> Vollbildzeile (N, NCOLS): bestes Fach mit Parabel, wie im C++-Kern.

    outlier_px > 0 nimmt einzelne Zacken heraus (am Rider [freespace] net_outlier_px): Weicht eine Spalte um mehr
    als outlier_px vom Median der OUTLIER_COLS Spalten um sie ab und liegt beim Medianwert mindestens outlier_ratio
    der Masse ihres eigenen Gipfels (je 2 * OUTLIER_BINS + 1 Fächer), gilt das beste Fach beim Medianwert."""
    row = _peak(logits)
    if outlier_px <= 0:
        return row
    h = OUTLIER_COLS // 2
    med = F.pad(row[:, None, :], (h, h), mode="replicate")[:, 0].unfold(1, OUTLIER_COLS, 1).median(dim=2).values
    centers = torch.as_tensor(bin_centers(), dtype=logits.dtype, device=logits.device)
    best = logits.argmax(dim=1)
    nb = (centers[None, :, None] - med[:, None, :]).abs().argmin(dim=1)
    p = torch.softmax(logits, dim=1).permute(0, 2, 1)                       # N, NCOLS, NBINS
    win = F.avg_pool1d(p, 2 * OUTLIER_BINS + 1, 1, OUTLIER_BINS).permute(0, 2, 1)     # Rand zählt als 0
    swap = ((row - med).abs() > outlier_px) & \
           (win.gather(1, nb[:, None]).squeeze(1) >= outlier_ratio * win.gather(1, best[:, None]).squeeze(1))
    bins = torch.arange(NBINS, device=logits.device)[None, :, None]
    near = _peak(logits.masked_fill((bins - nb[:, None, :]).abs() > OUTLIER_BINS, -1e9))
    return torch.where(swap, near, row)


def _peak(logits: torch.Tensor) -> torch.Tensor:
    best = logits.argmax(dim=1)
    i = best.clamp(1, NBINS - 2).unsqueeze(1)
    c, l, r = logits.gather(1, i).squeeze(1), logits.gather(1, i - 1).squeeze(1), logits.gather(1, i + 1).squeeze(1)
    den = l - 2 * c + r
    frac = torch.where(den < -1e-6, 0.5 * (l - r) / den, torch.zeros_like(den)).clamp(-0.5, 0.5)
    frac = torch.where((best == i.squeeze(1)), frac, torch.zeros_like(frac))
    pos = best.float() + frac
    centers = torch.as_tensor(bin_centers(), dtype=logits.dtype, device=logits.device)
    lo = pos.floor().long().clamp(0, NBINS - 1)
    hi = (lo + 1).clamp(0, NBINS - 1)
    t = pos - lo.float()
    return centers[lo] * (1 - t) + centers[hi] * t


class DS(nn.Module):
    """Tiefenweise 3x3 (ohne ReLU und Bias), dann 1x1 mit ReLU; Residuum vor dem ReLU."""

    def __init__(self, cin: int, cout: int, stride=(1, 1), residual: bool = False, dil: int = 1):
        super().__init__()
        self.dw = nn.Conv2d(cin, cin, 3, stride=stride, padding=(1, dil), dilation=(1, dil), groups=cin, bias=False)
        self.dil = dil
        self.bn_d = nn.BatchNorm2d(cin)
        self.pw = nn.Conv2d(cin, cout, 1)
        self.bn_p = nn.BatchNorm2d(cout)
        self.residual = residual
        self.stride = stride

    def forward(self, x):
        y = self.bn_p(self.pw(self.bn_d(self.dw(x))))
        return F.relu(y + x if self.residual else y)


class FloorNet(nn.Module):
    def __init__(self, dil=(1, 1), extra: int = 0):
        super().__init__()
        self.variant = {"dil": [int(d) for d in dil], "extra": int(extra)}
        self.stem = nn.Conv2d(3, 12, 3, stride=2, padding=1)
        self.bn_stem = nn.BatchNorm2d(12)
        self.blocks = nn.ModuleList([
            DS(12, 24, (2, 2)),
            DS(24, 32, (2, 1)), DS(32, 32, residual=True),
            DS(32, 48, (2, 1)), DS(48, 48, residual=True, dil=dil[0]), DS(48, 48, residual=True, dil=dil[1]),
        ] + [DS(48, 48, residual=True) for _ in range(extra)])
        self.head1 = nn.Conv2d(48 * 11, 32, 1)
        self.bn_h1 = nn.BatchNorm2d(32)
        self.head2 = nn.Conv2d(32, 32, (1, 3), padding=(0, 1), groups=32)
        self.bn_h2 = nn.BatchNorm2d(32)
        self.out = nn.Conv2d(32, NBINS, 1)

    def forward(self, x):
        x = F.relu(self.bn_stem(self.stem(x)))
        for b in self.blocks:
            x = b(x)
        n, c, h, w = x.shape
        x = x.reshape(n, c * h, 1, w)
        x = F.relu(self.bn_h1(self.head1(x)))
        x = F.relu(self.bn_h2(self.head2(x)))
        return self.out(x).squeeze(2)


def load_net(lauf: str, dev=None) -> "FloorNet":
    """Netz eines Laufs laden: floornet.pt, Variante aus netz.json (fehlt sie: Standardnetz)."""
    import json
    import os
    v = {}
    p = os.path.join(lauf, "netz.json")
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            v = json.load(f)
    net = FloorNet(tuple(v.get("dil", (1, 1))), int(v.get("extra", 0)))
    net.load_state_dict(torch.load(os.path.join(lauf, "floornet.pt"), map_location=dev or "cpu"))
    return net.to(dev) if dev is not None else net


def _fold(conv: nn.Conv2d, bn: nn.BatchNorm2d):
    """Faltung + BatchNorm -> Gewichte, Bias (Konstante je Ausgangskanal)."""
    g = bn.weight / torch.sqrt(bn.running_var + bn.eps)
    w = conv.weight * g.view(-1, *([1] * (conv.weight.dim() - 1)))
    c = bn.bias - bn.running_mean * g
    if conv.bias is not None:
        c = c + conv.bias * g
    return w, c


@torch.no_grad()
def fold(m: FloorNet) -> list[dict]:
    """Netz ohne BatchNorm als Liste von Operationen, in der Reihenfolge des C++-Kerns."""
    ops = []
    w, b = _fold(m.stem, m.bn_stem)
    ops.append({"kind": "stem", "w": w, "b": b, "stride": (2, 2), "relu": True})
    for bl in m.blocks:
        wd, cd = _fold(bl.dw, bl.bn_d)
        ops.append({"kind": "dw", "w": wd, "b": None, "stride": bl.stride, "relu": False, "dil": bl.dil})
        wp, bp = _fold(bl.pw, bl.bn_p)
        # Konstante der tiefenweisen Faltung durch die 1x1-Schicht schieben
        g = bl.bn_p.weight / torch.sqrt(bl.bn_p.running_var + bl.bn_p.eps)
        bp = bp + (bl.pw.weight.squeeze(-1).squeeze(-1) @ cd) * g
        ops.append({"kind": "pw", "w": wp, "b": bp, "relu": True, "res": bl.residual})
    ops.append({"kind": "collapse"})
    w, b = _fold(m.head1, m.bn_h1)
    ops.append({"kind": "pw", "w": w, "b": b, "relu": True, "res": False})
    w, b = _fold(m.head2, m.bn_h2)
    ops.append({"kind": "dw1d", "w": w, "b": b, "relu": True})
    ops.append({"kind": "pw", "w": m.out.weight.clone(), "b": m.out.bias.clone(), "relu": False, "res": False})
    return ops


@torch.no_grad()
def run_ops(ops: list[dict], x: torch.Tensor) -> torch.Tensor:
    """Gefaltetes Netz rechnen, zum Prüfen von fold() und als Referenz für den C++-Kern."""
    res = None
    for op in ops:
        k = op["kind"]
        if k == "stem":
            x = F.conv2d(x, op["w"], op["b"], stride=op["stride"], padding=1)
        elif k == "dw":
            res = x
            d = op.get("dil", 1)
            x = F.conv2d(x, op["w"], None, stride=op["stride"], padding=(1, d), dilation=(1, d), groups=x.shape[1])
        elif k == "pw":
            x = F.conv2d(x, op["w"], op["b"])
            if op.get("res"):
                x = x + res
        elif k == "collapse":
            n, c, h, w = x.shape
            x = x.reshape(n, c * h, 1, w)
        elif k == "dw1d":
            x = F.conv2d(x, op["w"], op["b"], padding=(0, 1), groups=x.shape[1])
        if op.get("relu"):
            x = F.relu(x)
    return x.squeeze(2)
