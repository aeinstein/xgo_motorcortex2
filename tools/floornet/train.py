#!/usr/bin/env python3
"""FloorNet trainieren (GPU), Labels aus label.py. Daten liegen als Fächer je Bild im Speicher der GPU.

  python tools/floornet/train.py --labels labels/ --out lauf1/ \\
      --train explore2,explore3,explore4,explore5,explore6,explore7,explore8 --val explore9,streife1,unten1

Mit --hand_labels und --hand_train kommen Fahrten mit Labels aus Handlinien dazu (handmap.py). Die Labels aus dem
Kantenverfahren lassen sich abschwächen: --far_atleast macht ferne Grenzen zu "mindestens", --cap_atleast begrenzt,
wie weit "mindestens" gilt.

Bericht je Prüffahrt: Abstand der Vorhersage zu den Labels in Metern je Entfernungsband, daneben dasselbe für
das Kantenverfahren. Die Labels stammen aus dem Kantenverfahren, der Vergleich begünstigt es also. Er zeigt,
ob das Netz die Grenze gelernt hat, nicht, ob es besser ist; das kann nur ein Prüfsatz von Hand zeigen.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F

sys.path.insert(0, str(Path(__file__).resolve().parent))

from net import CROP_H, CROP_Y, DOWN, IN_H, IN_W, NCOLS, FloorNet, bin_centers, decode, prep  # noqa: E402
from pick import pick  # noqa: E402

IGNORE, EXACT, ATLEAST = 0, 1, 2
MARGIN = 12                 # Eingabezeilen Vorrat oberhalb des Ausschnitts, für den simulierten Nick
BANDS = ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5))
FAR_M = 0.15                # Abweichung, ab der eine Grenze als falsch zählt (Plan, Schritt 3)


def _decode_one(args):
    path, top = args
    img = cv2.imread(path)
    if img is None:
        raise RuntimeError("Bild fehlt: " + path)
    return cv2.resize(img[top:CROP_Y + CROP_H], (IN_W, IN_H + MARGIN), interpolation=cv2.INTER_AREA)


def load_run(npz: str, cache: str, dist: float, yaw: float, pool, min_labeled: float = 0.6, only_hand: bool = False) -> dict:
    """Gewählte Bilder einer Fahrt: Fächer (uint8, N x H x W x 3) und Labels. Bilder liegen im Zwischenspeicher.
    only_hand: nur Bilder mit eigener Handlinie, ohne die über die Karte übertragenen."""
    d = np.load(npz, allow_pickle=False)
    run = str(d["run"])
    hand = d["hand"] if "hand" in d.files else np.zeros(len(d["x"]), bool)
    idx = np.nonzero(hand)[0] if only_hand else np.union1d(
        pick(d["x"], d["y"], d["th"], d["rate"], d["kind"], dist, yaw, min_labeled), np.nonzero(hand)[0]).astype(int)
    # Schlüssel aus den Dateinamen: Label-Dateien derselben Fahrt können verschiedene Bilder an gleicher Stelle haben
    path = os.path.join(cache, "%s_%s.npy" % (run, hashlib.md5(d["files"][idx].tobytes()).hexdigest()[:10]))
    top = CROP_Y - MARGIN * DOWN
    if os.path.exists(path):
        imgs = np.load(path)
    else:
        src = str(d["source_dir"])
        imgs = np.stack(pool.map(_decode_one, [(os.path.join(src, str(d["files"][i])), top) for i in idx], chunksize=32))
        os.makedirs(cache, exist_ok=True)
        np.save(path, imgs)
    if len(imgs) != len(idx):
        raise RuntimeError("Zwischenspeicher passt nicht zu den Labels: " + path)
    return {"run": run, "imgs": imgs, "row": d["row"][idx], "kind": d["kind"][idx], "kanten": d["kanten_row"][idx], "hand": hand[idx],
            "pitch": d["pitch"][idx], "files": d["files"][idx], "source": str(d["source_dir"]), "geom": json.loads(str(d["geom"]))}


def soften(run: dict, far_atleast: float = 0.0, cap_atleast: float = 0.0) -> None:
    """Labels aus dem Kantenverfahren abschwächen, wo sie nachweislich schief liegen (Vergleich mit Handlinien):
    Ferne Grenzen liegen oft zu nah (erste Zelle einer verschmierten Wand), also nur noch "mindestens".
    "Mindestens" läuft oft durch dunkle Hindernisse (Freistrahl), also nur noch bis cap_atleast Meter behaupten."""
    geo, pitch = run["geom"], run["pitch"]
    row, kind = run["row"].copy(), run["kind"].copy()
    with np.errstate(invalid="ignore"):
        d = row_dist(geo, row, pitch)
        if far_atleast > 0:
            kind[(kind == EXACT) & (d > far_atleast)] = ATLEAST
        if cap_atleast > 0:
            beta = np.arctan2(geo["cam_h"], cap_atleast) + math.radians(geo["tilt_deg"]) + pitch[:, None]
            want = (geo["H"] / 2 + geo["f"] * np.tan(beta)).astype(row.dtype)
            m = (kind == ATLEAST) & (d > cap_atleast)
            row[m] = np.broadcast_to(want, row.shape)[m]
    run["row"], run["kind"] = row, kind


def augment(x: torch.Tensor, row: torch.Tensor, kind: torch.Tensor, norm: bool = False):
    """x: Bilder (N, 3, H + MARGIN, W), uint8. Rückgabe: Eingabe fürs Netz und passende Labels."""
    n = len(x)
    dev = x.device
    # Nick simulieren: Ausschnitt verschieben (je Eingabezeile 2 Vollbildzeilen); unten wird der Rand wiederholt
    dy = torch.randint(-6, 7, (n,), device=dev)
    ys = (MARGIN + dy)[:, None] + torch.arange(IN_H, device=dev)[None, :]
    ys = ys.clamp(0, x.shape[2] - 1)
    x = x.gather(2, ys[:, None, :, None].expand(-1, 3, -1, IN_W))
    row = row - (dy * DOWN).float()[:, None]
    # Spiegeln
    flip = torch.rand(n, device=dev) < 0.5
    x = torch.where(flip[:, None, None, None], x.flip(-1), x)
    row = torch.where(flip[:, None], row.flip(-1), row)
    kind = torch.where(flip[:, None], kind.flip(-1), kind)
    x = x.float() / 255.0
    # Bewegungsunschärfe beim Drehen
    blur = torch.rand(n, device=dev) < 0.2
    if blur.any():
        x[blur] = F.avg_pool2d(x[blur], (1, 5), stride=1, padding=(0, 2))
    # Licht: Verstärkung, Gamma, Farbstich
    gain = torch.exp(torch.randn(n, 1, 1, 1, device=dev) * 0.45)
    gamma = torch.exp(torch.randn(n, 1, 1, 1, device=dev) * 0.2)
    tint = 1.0 + torch.randn(n, 3, 1, 1, device=dev) * 0.08
    x = (x.clamp(1e-3, 1.0) ** gamma) * gain * tint
    # Wenig Licht: abdunkeln, dann Rauschen. Es wächst nicht mit, also wird das Bild verrauschter wie am Rider
    # (dunkle Stellen im Wohnzimmer: Boden im Mittel 15/255)
    dark = torch.rand(n, 1, 1, 1, device=dev) < 0.35
    x = x * torch.where(dark, torch.exp(-0.5 - 2.0 * torch.rand(n, 1, 1, 1, device=dev)), torch.ones(n, 1, 1, 1, device=dev))
    x = x + torch.randn_like(x) * (torch.rand(n, 1, 1, 1, device=dev) * 0.02)
    return prep(x.clamp(0.0, 1.0), norm), row, kind


def column_loss(logits, row, kind, centers, sigma: float = 3.0, far_w: float = 0.5, atleast_w: float = 0.5):
    """EXACT: weiche Klassenziele um die Zeile, dazu Strafe für Masse hinter der Grenze (fälschlich frei).
    ATLEAST: Die Grenze liegt mindestens so weit weg. IGNORE zählt nicht."""
    logp = F.log_softmax(logits, dim=1)                     # (N, K, C)
    c = centers[None, :, None]
    r = row[:, None, :]
    q = torch.exp(-0.5 * ((c - r) / sigma) ** 2)
    q = q / q.sum(dim=1, keepdim=True).clamp_min(1e-6)
    ce = -(q * logp).sum(dim=1)
    far = ((c < r - 2 * sigma).float() * logp.exp()).sum(dim=1)
    least = -torch.logsumexp(logp + torch.log((c <= r + sigma).float().clamp_min(1e-9)), dim=1)
    ex, al = kind == EXACT, kind == ATLEAST
    loss = (ce + far_w * far) * ex.float() + atleast_w * least * al.float()
    return loss.sum() / (ex.float() + atleast_w * al.float()).sum().clamp_min(1.0)


def smooth_loss(logits, row, kind, centers, delta: float = 8.0):
    """Sprünge zwischen Nachbarspalten, die das Label nicht hat. Erwartete Zeile, Huber in Vielfachen von delta (px)."""
    want = (F.softmax(logits, dim=1) * centers[None, :, None]).sum(dim=1)
    both = (kind[:, 1:] == EXACT) & (kind[:, :-1] == EXACT)
    d = ((want[:, 1:] - want[:, :-1]) - (row[:, 1:] - row[:, :-1])) / delta
    return (F.huber_loss(d, torch.zeros_like(d), reduction="none") * both.float()).sum() / both.float().sum().clamp_min(1.0)


def near_frames(run: dict, near_m: float, share: float) -> np.ndarray:
    """Nahaufnahmen: Bilder, in denen mindestens dieser Anteil der Spalten eine Grenze näher als near_m hat."""
    with np.errstate(invalid="ignore"):
        return ((row_dist(run["geom"], run["row"], run["pitch"]) < near_m) & (run["kind"] == EXACT)).mean(axis=1) >= share


def row_dist(geo: dict, row, pitch):
    """Zeile -> Abstand nach vorn (m); am oder über dem Horizont unendlich."""
    beta = np.arctan2(row - geo["H"] / 2, geo["f"]) - math.radians(geo["tilt_deg"]) - pitch[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(beta > math.radians(0.5), geo["cam_h"] / np.tan(beta), np.inf)


def band_report(pred_row, label_row, kind, pitch, geo) -> dict:
    """Je Band: Anteil fälschlich frei (Vorhersage > FAR_M hinter der Grenze), fälschlich blockiert, Median |Fehler|."""
    with np.errstate(invalid="ignore"):
        dp = row_dist(geo, pred_row, pitch)
        dl = row_dist(geo, label_row, pitch)
        out = {}
        for lo, hi in BANDS:
            m = (kind == EXACT) & (dl >= lo) & (dl < hi) & np.isfinite(pred_row)
            n = int(m.sum())
            if n < 50:
                continue
            e = dp[m] - dl[m]
            out["%.1f-%.1f" % (lo, hi)] = {"n": n, "frei": float((e > FAR_M).mean()), "block": float((e < -FAR_M).mean()),
                                           "med_cm": float(np.median(np.abs(np.nan_to_num(e, posinf=9.0))) * 100)}
        # ATLEAST: Grenze darf weiter sein, aber nicht näher
        m = (kind == ATLEAST) & np.isfinite(pred_row)
        if m.sum() >= 50:
            out["mind"] = {"n": int(m.sum()), "block": float(((dp[m] - dl[m]) < -FAR_M).mean())}
    return out


@torch.no_grad()
def predict(net, run: dict, dev, norm: bool = False, bs: int = 256) -> np.ndarray:
    net.eval()
    out = []
    for i in range(0, len(run["imgs"]), bs):
        x = torch.as_tensor(run["imgs"][i:i + bs][:, MARGIN:], device=dev).permute(0, 3, 1, 2).float() / 255.0
        out.append(decode(net(prep(x, norm))).cpu().numpy())
    return np.concatenate(out)


def evaluate(net, runs: list[dict], dev, norm: bool = False) -> dict:
    res = {}
    for r in runs:
        pr = predict(net, r, dev, norm)
        idx = np.linspace(0, r["kanten"].shape[1] - 1, NCOLS).round().astype(int)
        res[r["run"]] = {"netz": band_report(pr, r["row"], r["kind"], r["pitch"], r["geom"]),
                         "kanten": band_report(r["kanten"][:, idx],r["row"], r["kind"], r["pitch"], r["geom"])}
    return res


def show(res: dict) -> str:
    lines = ["%-10s %-7s %s" % ("Fahrt", "", "  ".join("%-22s" % ("%s m" % b) for b in ("0.0-0.5", "0.5-1.0", "1.0-1.5", "1.5-2.5")))
             + " mind.blockiert"]
    for run, r in res.items():
        for who in ("netz", "kanten"):
            cells = []
            for b in ("0.0-0.5", "0.5-1.0", "1.0-1.5", "1.5-2.5"):
                v = r[who].get(b)
                cells.append("%-22s" % ("fr%4.1f bl%4.1f med%3.0fcm" % (100 * v["frei"], 100 * v["block"], v["med_cm"]) if v else "-"))
            mind = r[who].get("mind")
            lines.append("%-10s %-7s %s %s" % (run, who, "  ".join(cells), "%.1f" % (100 * mind["block"]) if mind else "-"))
    return "\n".join(lines)


def tiles(net, run: dict, out: str, dev, norm: bool = False, n: int = 12) -> None:
    """Grün Label, blau mindestens, gelb Kanten, magenta Netz."""
    pr = predict(net, run, dev, norm)
    picks = np.linspace(0, len(pr) - 1, n + 2)[1:-1].round().astype(int)
    idx = np.linspace(0, run["kanten"].shape[1] - 1, NCOLS).round().astype(int)
    xs = ((np.arange(NCOLS) + 0.5) * 640 / NCOLS).astype(int)
    rows = []
    for i in picks:
        img = cv2.imread(os.path.join(run["source"], str(run["files"][i])))
        img = np.clip(255 * (img / 255.0) ** 0.7, 0, 255).astype(np.uint8)
        for c, x in enumerate(xs):
            k = run["kanten"][i, idx[c]]
            if np.isfinite(k):
                cv2.circle(img, (x, int(k)), 2, (0, 255, 255), -1)
            col = {EXACT: (0, 255, 0), ATLEAST: (255, 128, 0)}.get(int(run["kind"][i, c]))
            if col:
                cv2.circle(img, (x, int(run["row"][i, c])), 2, col, -1)
        pts = np.stack([xs, np.round(pr[i])], axis=1).astype(np.int32)
        cv2.polylines(img, [pts], False, (255, 0, 255), 2)
        cv2.putText(img, "%s %s" % (run["run"], str(run["files"][i])[:6]), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        rows.append(cv2.resize(img, (480, 360)))
    while len(rows) % 4:
        rows.append(np.zeros_like(rows[0]))
    cv2.imwrite(out, np.vstack([np.hstack(rows[j:j + 4]) for j in range(0, len(rows), 4)]), [cv2.IMWRITE_JPEG_QUALITY, 85])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--train", required=True, help="Fahrten, mit Komma getrennt")
    ap.add_argument("--val", required=True)
    ap.add_argument("--cache", default=os.path.expanduser("~/floornet/cache"))
    ap.add_argument("--dist", type=float, default=0.03)
    ap.add_argument("--yaw", type=float, default=4.0)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--norm", type=int, default=0, help="1: Helligkeit über den Boden normieren")
    ap.add_argument("--hand_labels", default="", help="Verzeichnis von handmap.py")
    ap.add_argument("--hand_train", default="", help="Fahrten aus --hand_labels, mit Komma getrennt")
    ap.add_argument("--hand_repeat", type=int, default=8, help="so oft zählt ein Bild mit eigener Handlinie")
    ap.add_argument("--hand_min_labeled", type=float, default=0.3)
    ap.add_argument("--hand_only", type=int, default=0, help="1: nur Bilder mit eigener Handlinie, nichts über die Karte Übertragenes")
    ap.add_argument("--far_atleast", type=float, default=0.0)
    ap.add_argument("--cap_atleast", type=float, default=0.0)
    ap.add_argument("--far_w", type=float, default=0.5)
    ap.add_argument("--atleast_w", type=float, default=0.5)
    ap.add_argument("--near_repeat", type=int, default=1, help="so oft zählt eine Nahaufnahme")
    ap.add_argument("--near_m", type=float, default=0.5)
    ap.add_argument("--near_share", type=float, default=0.3)
    ap.add_argument("--smooth_w", type=float, default=0.0, help="Gewicht der Strafe für Sprünge zwischen Nachbarspalten")
    ap.add_argument("--dil", default="1,1", help="waagerechte Dilatation in Block 5 und 6 (net.py, Varianten)")
    ap.add_argument("--extra", type=int, default=0, help="weitere Residualblöcke auf 80 x 11")
    a = ap.parse_args()
    torch.manual_seed(a.seed)
    os.makedirs(a.out, exist_ok=True)
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    with Pool(12) as pool:
        train = [load_run(os.path.join(a.labels, r + ".npz"), a.cache, a.dist, a.yaw, pool) for r in a.train.split(",") if r]
        for r in train:
            soften(r, a.far_atleast, a.cap_atleast)
        train += [load_run(os.path.join(a.hand_labels, r + ".npz"), a.cache, a.dist, a.yaw, pool, a.hand_min_labeled, bool(a.hand_only))
                  for r in a.hand_train.split(",") if r]
        val = [load_run(os.path.join(a.labels, r + ".npz"), a.cache, a.dist, a.yaw, pool) for r in a.val.split(",") if r]
    print("Training:", {r["run"]: len(r["imgs"]) for r in train}, "Prüfung:", {r["run"]: len(r["imgs"]) for r in val}, flush=True)
    x = torch.as_tensor(np.concatenate([r["imgs"] for r in train]), device=dev).permute(0, 3, 1, 2)
    row = torch.as_tensor(np.concatenate([r["row"] for r in train]), device=dev)
    kind = torch.as_tensor(np.concatenate([r["kind"] for r in train]), device=dev)
    # Bilder mit eigener Handlinie mehrfach: Sie sind die einzige Quelle der Wahrheit
    hand = np.concatenate([r["hand"] for r in train])
    near = np.concatenate([near_frames(r, a.near_m, a.near_share) for r in train])
    order = torch.as_tensor(np.repeat(np.arange(len(hand)), np.maximum(np.where(hand, a.hand_repeat, 1), np.where(near, a.near_repeat, 1))), device=dev)
    print("Bilder %d, davon mit Handlinie %d, nah %d, je Epoche %d" % (len(hand), hand.sum(), near.sum(), len(order)), flush=True)
    centers = torch.as_tensor(bin_centers(), dtype=torch.float32, device=dev)

    net = FloorNet(tuple(int(d) for d in a.dil.split(",")), a.extra).to(dev)
    with open(os.path.join(a.out, "netz.json"), "w", encoding="utf-8") as f:
        json.dump(net.variant, f)
    print("Parameter:", sum(p.numel() for p in net.parameters()), flush=True)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * (len(order) // a.bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, a.lr, total_steps=steps, pct_start=0.1)
    t0 = time.time()
    for ep in range(a.epochs):
        net.train()
        perm = order[torch.randperm(len(order), device=dev)]
        total = 0.0
        for i in range(len(order) // a.bs):
            b = perm[i * a.bs:(i + 1) * a.bs]
            xb, rb, kb = augment(x[b], row[b], kind[b], bool(a.norm))
            lg = net(xb)
            loss = column_loss(lg, rb, kb, centers, far_w=a.far_w, atleast_w=a.atleast_w)
            if a.smooth_w:
                loss = loss + a.smooth_w * smooth_loss(lg, rb, kb, centers)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            total += loss.item()
        if (ep + 1) % 10 == 0 or ep == 0:
            print("Epoche %3d  Verlust %.3f  %.0f s" % (ep + 1, total / max(len(order) // a.bs, 1), time.time() - t0), flush=True)
    torch.save(net.state_dict(), os.path.join(a.out, "floornet.pt"))
    if not val:
        return 0
    res = evaluate(net, val + train[:1], dev, bool(a.norm))
    text = show(res)
    print(text, flush=True)
    with open(os.path.join(a.out, "bericht.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    with open(os.path.join(a.out, "bericht.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)
    for r in val:
        tiles(net, r, os.path.join(a.out, r["run"] + "_netz.jpg"), dev, bool(a.norm))
    return 0


if __name__ == "__main__":
    sys.exit(main())
