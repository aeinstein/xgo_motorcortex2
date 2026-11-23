"""Kartendrift offline aus Rohfahrten messen: Wo der Rider eine Stelle wieder besucht, misst das Startbild-Verfahren
(xgonav/startbild.messen, wie tools/start_abgleich.py) die Lage des späteren Bildes gegen das frühere. Der
Unterschied zur Odometrie (Kartenpose) ist die Drift, die sich zwischen beiden Bildern angesammelt hat. Dazu je
Paar Weg, Drehung, Dauer und Fahrweise dazwischen, um die Drift gegen das Tempo aufzutragen.
Läuft am PC oder auf kimmy, nicht am Rider (Kosten am Pi: keine).

  python tools/drift_kurve.py FAHRT [FAHRT ...] [--out DIR] [--max-paare 30] [--bilder]
  python tools/drift_kurve.py --bild DIR/drift.jsonl [...]        # nur Schaubild aus vorhandenen Ergebnissen

Paare: spätes Bild j in gerader Vorwärtsfahrt (dort gibt es Tiefenbilder), Kandidaten i laut Odometrie
< --nah-m auseinander, < --nah-deg anderer Kurs, mindestens --abstand-s früher, beide balancierend; gewählt wird
das i mit den meisten SIFT-Treffern zu j (Bildähnlichkeit statt Odometrie, sonst fielen gerade die gedrifteten
Wiederkehren heraus), mindestens --min-treffer. Ein neues j erst --takt-s nach dem letzten. Gemessen wird mit den Gütegrenzen von [startbild] (Treffer, Nick/Roll, Gegenprobe), aber
ohne die Plausibilitätsgrenze der Korrektur (max_m/max_deg weit), sonst fielen gerade große Driften heraus.
Ergebnis je Paar als Zeile in DIR/drift.jsonl; --bilder legt je Paar DIR/paar_<fahrt>_<i>_<j>.jpg an (früh | spät).
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from xgonav.config import Config  # noqa: E402
from tools.start_abgleich import abgleich, lade, tiefenbilder  # noqa: E402

NAH_M = 1.0
NAH_DEG = 45.0
ABSTAND_S = 10.0
TAKT_S = 8.0
V_FAHRT = 0.15        # m/s: ab hier zählt ein Bild als „fährt“
W_DREH = 40.0         # °/s: ab hier als „dreht“; beides zugleich = Drehen aus der Fahrt


def _dwrap(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def bewegung(ms: list[dict], glatt_s: float = 0.25) -> dict:
    """Je Bild Zeit, Pose, Geschwindigkeit (m/s) und Drehrate (°/s), über glatt_s geglättet."""
    t = np.array([m["t"] for m in ms], float)
    p = np.array([m["pose"] for m in ms], float)
    ds = np.r_[0.0, np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))]
    dth = np.r_[0.0, np.abs(_dwrap(np.diff(p[:, 2])))]
    s, th = np.cumsum(ds), np.cumsum(dth)
    v = np.zeros(len(t))
    w = np.zeros(len(t))
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > glatt_s:
            j += 1
        dt = t[i] - t[j]
        if dt > 0:
            v[i] = (s[i] - s[j]) / dt
            w[i] = (th[i] - th[j]) / dt
    bal = np.array([bool(m.get("balancing")) for m in ms])
    wid = np.array([float(((m.get("odom") or {}).get("widersprueche")) or 0) for m in ms])
    sch = np.array([bool((m.get("odom") or {}).get("schlupf")) for m in ms])
    return {"t": t, "p": p, "s": s, "th": th, "v": v, "w": w, "bal": bal, "wid": wid, "schlupf": sch}


def paare_suchen(b: dict, nah_m=NAH_M, nah_deg=NAH_DEG, abstand_s=ABSTAND_S, takt_s=TAKT_S,
                 min_weg_m: float = 1.0) -> list[tuple[int, int]]:
    """(i, j) mit i < j: Wiederkehr an eine Stelle. j nur in gerader Vorwärtsfahrt (Tiefenbilder davor), zu j das
    i mit dem ähnlichsten Blick (Abstand/nah_m + Kursunterschied/nah_deg). Dazwischen mindestens min_weg_m Weg
    (sonst stand er nur)."""
    t, p, bal = b["t"], b["p"], b["bal"]
    out, letzt = [], -1e9
    for j in range(len(t)):
        if not bal[j] or t[j] - letzt < takt_s or b["v"][j] < 0.05 or b["w"][j] > 20.0:
            continue
        kand = np.flatnonzero((t <= t[j] - abstand_s) & bal & (b["w"] < 30.0))
        if not len(kand):
            continue
        d = np.hypot(p[kand, 0] - p[j, 0], p[kand, 1] - p[j, 1])
        dk = np.abs(_dwrap(p[kand, 2] - p[j, 2]))
        sel = (d < nah_m) & (dk < nah_deg) & (b["s"][j] - b["s"][kand] >= min_weg_m)
        if sel.any():
            out.append((kand[sel], j))
            letzt = t[j]
    return out


class Treffer:
    """Bildähnlichkeit für die Paarwahl: Zahl der SIFT-Paare (Ratio-Test), Merkmale je Bild zwischengespeichert."""

    def __init__(self, d: str, ms: list[dict], n: int = 1500):
        self.d, self.ms, self.n, self.cache = d, ms, n, {}
        self.sift = cv2.SIFT_create(n)
        self.bf = cv2.BFMatcher(cv2.NORM_L2)

    def desc(self, k: int):
        if k not in self.cache:
            g = cv2.imread(os.path.join(self.d, self.ms[k]["file"]), 0)
            self.cache[k] = None if g is None else self.sift.detectAndCompute(cv2.equalizeHist(g), None)[1]
        return self.cache[k]

    def zahl(self, a: int, b: int) -> int:
        da, db = self.desc(a), self.desc(b)
        if da is None or db is None or len(da) < 2 or len(db) < 2:
            return 0
        return sum(1 for p in self.bf.knnMatch(da, db, k=2) if len(p) == 2 and p[0].distance < 0.8 * p[1].distance)

    def bestes(self, kand: np.ndarray, j: int, b: dict, max_n: int = 40) -> tuple[int, int]:
        """Unter den Kandidaten (zeitlich ausgedünnt auf höchstens max_n) das Bild mit den meisten Treffern zu j."""
        t = b["t"][kand]
        # je 0,5 s höchstens ein Kandidat
        wahl, letzt = [], -1e9
        for k, tk in zip(kand, t):
            if tk - letzt >= 0.5:
                wahl.append(int(k))
                letzt = tk
        if len(wahl) > max_n:
            wahl = [wahl[q] for q in np.linspace(0, len(wahl) - 1, max_n).round().astype(int)]
        z = [self.zahl(k, j) for k in wahl]
        q = int(np.argmax(z))
        return wahl[q], z[q]


def kennzahlen(b: dict, i: int, j: int) -> dict:
    sl = slice(i, j + 1)
    v, w = b["v"][sl], b["w"][sl]
    dt = np.diff(b["t"][sl], prepend=b["t"][i])
    fahrt = v > V_FAHRT
    return {"dauer_s": round(float(b["t"][j] - b["t"][i]), 1),
            "weg_m": round(float(b["s"][j] - b["s"][i]), 2),
            "dreh_deg": round(float(b["th"][j] - b["th"][i]), 0),
            "v_mittel": round(float(v[fahrt].mean()) if fahrt.any() else 0.0, 3),
            "v_p95": round(float(np.percentile(v, 95)), 3),
            "w_p95": round(float(np.percentile(w, 95)), 1),
            "fahrdreh_s": round(float(dt[fahrt & (w > W_DREH)].sum()), 2),
            "widersprueche": int(max(0.0, b["wid"][j] - b["wid"][i])),
            "schlupf_s": round(float(dt[b["schlupf"][sl]].sum()), 2)}


def bildpaar(d: str, ms: list[dict], i: int, j: int, text: str, pfad: str) -> None:
    ks = []
    for k in (i, j):
        im = cv2.imread(os.path.join(d, ms[k]["file"]))
        im = cv2.resize(im, (320, 240)) if im is not None else np.zeros((240, 320, 3), np.uint8)
        cv2.putText(im, ms[k]["file"], (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        ks.append(im)
    out = np.hstack(ks)
    cv2.putText(out, text, (5, 232), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(pfad, out)


def fahrt_messen(d: str, cfg, out_dir: str, max_paare: int, bilder: bool, args) -> list[dict]:
    ms = [m for m in lade(d) if "pose" in m and "file" in m]
    if len(ms) < 50:
        return []
    b = bewegung(ms)
    paare = paare_suchen(b, args.nah_m, args.nah_deg, args.abstand_s, args.takt_s)
    if len(paare) > max_paare:
        paare = [paare[k] for k in np.linspace(0, len(paare) - 1, max_paare).round().astype(int)]
    name = os.path.basename(d.rstrip("/\\"))
    tr = Treffer(d, ms)
    erg = []
    for kand, j in paare:
        if len(tiefenbilder(ms, j, cfg.startbild.basen)) < 2:
            continue
        i, n_tr = tr.bestes(kand, j, b)
        if n_tr < args.min_treffer:
            print("%s %s: kein ähnliches früheres Bild (%d Treffer)" % (name, ms[j]["file"], n_tr), flush=True)
            continue
        try:
            r = abgleich(d, i, j, cfg.startbild.basen, cfg=cfg, ms=ms)
        except cv2.error as e:  # kaputtes Bild o. ä.
            print("%s %d->%d Fehler %s" % (name, i, j, e), file=sys.stderr)
            continue
        z = {"fahrt": name, "i": ms[i]["file"], "j": ms[j]["file"], "ok": r["ok"], "grund": r["grund"],
             "inl": r["inl"], "odometrie": r["ende_odometrie"]}
        z.update(kennzahlen(b, i, j))
        if "ende_gemessen" in r:
            o, g = r["ende_odometrie"], r["ende_gemessen"]
            e = [round(g[0] - o[0], 3), round(g[1] - o[1], 3), round(g[2] - o[2], 2)]
            z.update({"gemessen": g, "drift": e, "drift_m": round(math.hypot(e[0], e[1]), 3)})
            if bilder:
                bildpaar(d, ms, i, j, "%s  Drift %.0f cm %+.1f deg  %.1f m %.0f deg  inl %d" % (
                    "ok" if r["ok"] else "nein", 100 * z["drift_m"], e[2], z["weg_m"], z["dreh_deg"], r["inl"]),
                    os.path.join(out_dir, "paar_%s_%s_%s.jpg" % (name, ms[i]["file"][:-4], ms[j]["file"][:-4])))
        print(json.dumps(z, ensure_ascii=False), flush=True)
        erg.append(z)
    return erg


def schaubild(zeilen: list[dict], pfad: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ok = [z for z in zeilen if z.get("ok")]
    if not ok:
        print("keine angenommenen Paare")
        return
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    v = np.array([z["v_p95"] for z in ok])
    for a, (x, y, xl, yl) in zip(ax, [
            ([z["weg_m"] for z in ok], [100 * z["drift_m"] for z in ok], "Weg zwischen den Bildern (m)", "Drift (cm)"),
            ([z["dreh_deg"] for z in ok], [abs(z["drift"][2]) for z in ok], "Drehung dazwischen (°)", "|Kursdrift| (°)"),
            (v, [100 * z["drift_m"] / max(z["weg_m"], 0.5) for z in ok], "Tempo p95 (m/s)", "Drift je m Weg (cm/m)")]):
        sc = a.scatter(x, y, c=v, cmap="viridis", s=18)
        a.set_xlabel(xl)
        a.set_ylabel(yl)
        a.grid(alpha=0.3)
    fig.colorbar(sc, ax=ax, label="Tempo p95 (m/s)")
    fig.savefig(pfad, dpi=110, bbox_inches="tight")
    print("Schaubild:", pfad)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fahrten", nargs="*")
    ap.add_argument("--out", default="drift_out")
    ap.add_argument("--max-paare", type=int, default=30)
    ap.add_argument("--bilder", action="store_true")
    ap.add_argument("--bild", nargs="*", help="nur Schaubild aus drift.jsonl-Dateien")
    ap.add_argument("--nah-m", type=float, default=NAH_M)
    ap.add_argument("--nah-deg", type=float, default=NAH_DEG)
    ap.add_argument("--abstand-s", type=float, default=ABSTAND_S)
    ap.add_argument("--takt-s", type=float, default=TAKT_S)
    ap.add_argument("--min-treffer", type=int, default=60, help="SIFT-Paare zum ähnlichsten früheren Bild")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.bild:
        zeilen = [json.loads(ln) for f in a.bild for ln in open(f, encoding="utf-8") if ln.strip()]
        schaubild(zeilen, os.path.join(a.out, "drift.png"))
        return
    cfg = Config()
    cfg.startbild = copy.copy(cfg.startbild)
    cfg.startbild.max_m, cfg.startbild.max_deg = 2.0, 45.0      # messen, nicht korrigieren: große Drift zulassen
    alle = []
    with open(os.path.join(a.out, "drift.jsonl"), "a", encoding="utf-8") as f:
        for d in a.fahrten:
            for z in fahrt_messen(d, cfg, a.out, a.max_paare, a.bilder, a):
                f.write(json.dumps(z, ensure_ascii=False) + "\n")
                alle.append(z)
    n_ok = sum(z["ok"] for z in alle)
    print("Paare %d, angenommen %d" % (len(alle), n_ok))
    if n_ok:
        schaubild(alle, os.path.join(a.out, "drift.png"))


if __name__ == "__main__":
    main()
