"""Vergleichsseite DA3 gegen MoGe-2 gegen Stage3 p11 auf explore10-12: je Bild alle Grenzen über Bild und Grünbild,
daneben die Klassen der gewählten DA3-Variante; dazu Tabelle je Entfernungsband und die Unruhe von Bild zu Bild.
Aufruf (kimmy, aus geo/code/tools/floornet):
  python da3_seite.py ~/floornet/da3 ~/floornet/geo_pruef_v2 ~/floornet/stage3_p11_pruef metric_f1 l11_f1 l11_f8
Schreibt <da3>/index.html (Bilder relativ zu ~/floornet, Betrachter-Server Port 8765) und <da3>/vergleich.txt."""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import comfy_label as C  # noqa: E402

DA3, MOGE, P11 = sys.argv[1:4]
VAR = sys.argv[4:]
RUNS = ("explore10", "explore11", "explore12")
WEIT = 0.15
NAH = 1.5


def zeilen(pfad):
    out = {}
    for z in open(os.path.join(pfad, "rows.jsonl"), encoding="utf-8"):
        e = json.loads(z)
        if e["art"] == "ebene":
            out[e["id"]] = e
    return out


lehrer = {"moge": zeilen(MOGE)}
for v in VAR:
    lehrer[v] = zeilen(os.path.join(DA3, "geo_" + v))

eintraege = []
for run in RUNS:
    p = np.load(os.path.join(P11, run + ".npz"), allow_pickle=False)
    geo = json.loads(str(p["geom"]))
    for i, f in enumerate(p["files"]):
        k = "%s/%s" % (run, f)
        if not all(k in L for L in lehrer.values()):      # nur Bilder, auf denen alle eine eigene Ebene haben
            continue
        pitch = np.array([p["pitch"][i]], np.float64)
        x = dict(k=k, run=run, f=str(f), rows={"p11": p["row"][i].astype(np.float32)})
        for n, L in lehrer.items():
            x["rows"][n] = np.array(L[k]["rows"], np.float32)
        x["d"] = {n: C.row_dist(geo, r[None], pitch)[0] for n, r in x["rows"].items()}
        x["s"] = {n: L[k]["s"] for n, L in lehrer.items()}
        eintraege.append(x)

aus = []
log = lambda s="": (print(s), aus.append(s))
log("Bilder, auf denen MoGe und alle DA3-Varianten eine eigene Ebene haben: %d" % len(eintraege))
for n, L in lehrer.items():
    s = np.array([e["s"] for e in L.values()])
    log("  %-10s Ebene in %4d Bildern, Skala Median %.3f (p10 %.3f, p90 %.3f)" % (
        n, len(L), np.median(s), np.percentile(s, 10), np.percentile(s, 90)))


def tabelle(sel, titel):
    log("\n== %s, %d Bilder (Band = Entfernung der p11-Linie; freier/enger = > %d cm weiter/näher als p11)" % (
        titel, len(sel), WEIT * 100))
    log("%-9s | " % "Band" + " | ".join("%-27s" % n for n in lehrer))
    tot = {n: np.zeros(3) for n in lehrer}
    for lo, hi in ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5)):
        z = "%3.1f-%3.1f m | " % (lo, hi)
        teile = []
        for n in lehrer:
            cnt = fr = bl = 0
            meds = []
            for x in sel:
                m = (x["d"]["p11"] >= lo) & (x["d"]["p11"] < hi)
                e = x["d"][n] - x["d"]["p11"]
                cnt += int(m.sum())
                fr += int(((e > WEIT) & m).sum())
                bl += int(((e < -WEIT) & m).sum())
                meds.append(np.abs(np.nan_to_num(e[m], posinf=9.0)))
            if hi <= 1.0:
                tot[n] += (cnt, fr, bl)
            med = float(np.median(np.concatenate(meds)) * 100) if cnt else float("nan")
            teile.append("fr %4.1f%% en %4.1f%% med %3.0fcm" % (100 * fr / max(cnt, 1), 100 * bl / max(cnt, 1), med))
        log(z + " | ".join("%-27s" % t for t in teile))
    log("unter 1 m  | " + " | ".join("%-27s" % ("fr %4.1f%% en %4.1f%%" % (
        100 * t[1] / max(t[0], 1), 100 * t[2] / max(t[0], 1))) for t in tot.values()))


tabelle(eintraege, "alle")
for run in RUNS:
    tabelle([x for x in eintraege if x["run"] == run], run)

# Unruhe: aufeinanderfolgende Prüfbilder derselben Fahrt (höchstens 4 Bilder Abstand), Median der Änderung der
# Grenzentfernung je Streifen, nur wo beide Bilder unter 1,5 m liegen. Springt ein Teppich an/aus, wächst das.
log("\n== Unruhe von Bild zu Bild (Änderung der Grenzentfernung je Streifen, beide Bilder < %.1f m)" % NAH)
paare = [(a, b) for a, b in zip(eintraege, eintraege[1:])
         if a["run"] == b["run"] and 0 < int(b["f"][:-4]) - int(a["f"][:-4]) <= 4]
for n in ["p11"] + list(lehrer):
    d = []
    for a, b in paare:
        m = (a["d"][n] < NAH) & (b["d"][n] < NAH)
        d.append(np.abs(b["d"][n] - a["d"][n])[m])
    d = np.concatenate(d)
    log("  %-10s %d Paare  Median %4.1f cm  p90 %5.1f cm  Sprünge > 15 cm %4.1f%%" % (
        n, len(paare), 100 * np.median(d), 100 * np.percentile(d, 90), 100 * np.mean(d > WEIT)))
open(os.path.join(DA3, "vergleich.txt"), "w", encoding="utf-8").write("\n".join(aus) + "\n")


def kennz(x, n):
    m = x["d"]["p11"] < NAH
    e = np.nan_to_num(x["d"][n] - x["d"]["p11"], posinf=9.0, neginf=-9.0)
    return round(float(np.median(np.abs(e[m]))) * 100) if m.sum() >= 8 else 0


daten = []
for x in eintraege:
    daten.append(dict(k=x["k"], run=x["run"], f=x["f"],
                      r={n: [round(float(v), 1) for v in r] for n, r in x["rows"].items()},
                      sc={n: kennz(x, n) for n in lehrer}))
html = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "da3_seite.html"), encoding="utf-8").read()
html = html.replace("/*DATEN*/[]", json.dumps(daten)).replace("/*VAR*/[]", json.dumps(VAR))
html = html.replace('/*MOGE*/"../geo_pruef_v2/"', json.dumps(os.path.relpath(MOGE, DA3).replace(os.sep, "/") + "/"))
open(os.path.join(DA3, "index.html"), "w", encoding="utf-8").write(html)
print("\nSeite:", os.path.join(DA3, "index.html"))
