#!/usr/bin/env python3
"""CAD-Modell des Riders (STEP) in ein kleines Modell für die Lage-Anzeige im Web umwandeln.

  pip install cadquery-ocp fast-simplification     # nur am PC, nicht auf dem Rider
  python tools/step_to_model.py --cache rider.npz [--step docs/Rider-Pi.STEP] [--tris 17000] [--vorschau v.png]

Das Einlesen dauert unter einer Minute, der Sichtbarkeitstest etwa anderthalb; das vernetzte
Modell liegt danach in --cache und wird wiederverwendet. Format der Ausgabe: siehe modell_lesen."""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MAGIC = b"XGRM"

# Das STEP enthält keine Farben. Solid-Nummern gelten für docs/Rider-Pi.STEP (Reihenfolge beim
# Einlesen); Gruppe 0 nimmt alles Übrige auf. Name, Farbe, Solids
GRUPPEN = (
    ("gehaeuse", "#6b7683", ()),
    ("beine", "#4a535e", (66, 672, 23, 629, 43, 649, 65, 671, 33, 639, 32, 638, 55, 661, 56, 662, 59, 665)),
    ("reifen", "#23272d", (52, 658)),
    ("nabe", "#3a414c", (73, 675, 22, 628)),
    ("rand", "#16191d", (115, 117)),
    ("display", "#3987e5", (116,)),
    ("kamera", "#e3e6ea", (112,)),
    ("stifte", "#2b3038", (1183, 1184, 1185, 1186)),
)


def step_lesen(pfad: Path, abw_mm: float = 0.1, winkel: float = 0.5):
    """STEP einlesen und vernetzen: Punkte (mm), Dreiecke, Solid-Nummer je Dreieck."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    t0 = time.time()
    r = STEPControl_Reader()
    if r.ReadFile(str(pfad)) != IFSelect_RetDone:
        raise SystemExit(f"STEP nicht lesbar: {pfad}")
    print(f"[step] gelesen, {time.time() - t0:.0f} s", flush=True)
    r.TransferRoots()
    form = r.OneShape()
    print(f"[step] übertragen, {time.time() - t0:.0f} s", flush=True)
    BRepMesh_IncrementalMesh(form, abw_mm, False, winkel, True)
    print(f"[step] vernetzt, {time.time() - t0:.0f} s", flush=True)

    punkte, dreiecke, solid_nr = [], [], []
    basis = 0
    n_solid = 0
    sx = TopExp_Explorer(form, TopAbs_SOLID)
    while sx.More():
        fx = TopExp_Explorer(sx.Current(), TopAbs_FACE)
        while fx.More():
            flaeche = TopoDS.Face(fx.Current())
            ort = TopLoc_Location()
            netz = BRep_Tool.Triangulation_s(flaeche, ort)
            if netz is not None:
                tr = ort.Transformation()
                n = netz.NbNodes()
                p = np.empty((n, 3))
                for i in range(n):
                    q = netz.Node(i + 1).Transformed(tr)
                    p[i] = q.X(), q.Y(), q.Z()
                d = np.empty((netz.NbTriangles(), 3), dtype=np.int64)
                for i in range(len(d)):
                    d[i] = netz.Triangle(i + 1).Get()
                d -= 1
                if flaeche.Orientation() == TopAbs_REVERSED:
                    d = d[:, ::-1]
                punkte.append(p)
                dreiecke.append(d + basis)
                solid_nr.append(np.full(len(d), n_solid, dtype=np.int32))
                basis += n
            fx.Next()
        n_solid += 1
        sx.Next()
    print(f"[step] {n_solid} Solids, {basis} Punkte, {sum(map(len, dreiecke))} Dreiecke, {time.time() - t0:.0f} s", flush=True)
    return np.concatenate(punkte), np.concatenate(dreiecke), np.concatenate(solid_nr)


def netz_laden(step: Path, cache: Path, abw_mm: float, winkel: float):
    if cache.exists():
        z = np.load(cache)
        return z["p"], z["d"], z["s"]
    p, d, s = step_lesen(step, abw_mm, winkel)
    np.savez_compressed(cache, p=p, d=d, s=s)
    return p, d, s


def ausrichten(p: np.ndarray, s: np.ndarray, d: np.ndarray) -> np.ndarray:
    """CAD (mm: -x vorn = Display/Kamera, y hoch, z = Radachse) -> Körper-KS der Anzeige (cm:
    x vorn, y links, z hoch), Ursprung am Boden unter der Radachse."""
    boden = p[:, 1].min()
    reifen = np.unique(s[(p[d][:, :, 1].min(1) - boden) < 0.5])          # berühren den Boden
    i = np.unique(d[np.isin(s, reifen)])
    mx = (p[i, 0].min() + p[i, 0].max()) / 2
    mz = (p[:, 2].min() + p[:, 2].max()) / 2
    return np.stack([-(p[:, 0] - mx), p[:, 2] - mz, p[:, 1] - boden], 1) / 10.0


def richtungen(n: int) -> np.ndarray:
    """n Blickrichtungen, gleichmäßig auf der Kugel (Fibonacci)."""
    k = np.arange(n) + 0.5
    z, w = 1 - 2 * k / n, np.pi * (1 + 5 ** 0.5) * k
    r = np.sqrt(1 - z * z)
    return np.stack([r * np.cos(w), r * np.sin(w), z], 1)


def sichtbar(p: np.ndarray, d: np.ndarray, n_sicht: int = 40, w: int = 1024) -> np.ndarray:
    """Pixel je Dreieck, summiert über alle Blickrichtungen (ID-Puffer, parallel projiziert,
    von hinten nach vorn gemalt). Null heißt: von außen nie zu sehen."""
    import cv2
    q = p - (p.min(0) + p.max(0)) / 2
    k = w / (2.02 * np.linalg.norm(q, axis=1).max())
    zaehler = np.zeros(len(d) + 1, dtype=np.int64)
    for v in richtungen(n_sicht):
        r = np.cross([0.31, 0.17, 0.93], v)
        r /= np.linalg.norm(r)
        u = np.cross(v, r)
        xy = np.round((np.stack([q @ r, q @ u], 1) * k + w / 2) * 16).astype(np.int32)
        bild = np.zeros((w, w), dtype=np.int32)
        tiefe = (q @ v)[d].mean(1)
        for i in np.argsort(tiefe):
            cv2.fillConvexPoly(bild, xy[d[i]], int(i) + 1, cv2.LINE_8, 4)
        zaehler += np.bincount(bild.ravel(), minlength=len(zaehler))
    return zaehler[1:]


def verschweissen(p: np.ndarray, d: np.ndarray, s: np.ndarray):
    """Gleiche Punkte je Solid zusammenlegen (die Flächen teilen sich im STEP keine Punkte)."""
    sv = np.zeros(len(p), dtype=np.int64)
    sv[d.ravel()] = np.repeat(s, 3)
    schluessel = np.column_stack([np.round(p * 1e4).astype(np.int64), sv])
    _, erste, inv = np.unique(schluessel, axis=0, return_index=True, return_inverse=True)
    return p[erste], inv.ravel()[d]


def nachbarn_dazu(d: np.ndarray, an: np.ndarray, n_punkte: int, ringe: int = 2) -> np.ndarray:
    """Auswahl um angrenzende Dreiecke erweitern: Winzige Dreiecke fehlen sonst im ID-Puffer."""
    an = an.copy()
    for _ in range(ringe):
        punkt = np.zeros(n_punkte, dtype=bool)
        punkt[d[an].ravel()] = True
        an |= punkt[d].any(1)
    return an


def gruppe_von(s: np.ndarray) -> np.ndarray:
    gr = np.zeros(s.max() + 1, dtype=np.int32)
    for g, (_, _, solids) in enumerate(GRUPPEN):
        gr[list(solids)] = g
    return gr[s]


def vereinfachen(p: np.ndarray, d: np.ndarray, s: np.ndarray, ziel: int):
    """Quadric-Decimation je Solid, damit Teile nicht ineinanderlaufen; Budget nach Fläche.
    Ausgabe nach Farbgruppen sortiert: Punkte, Dreiecke, Anzahl je Gruppe."""
    import fast_simplification
    fl = np.linalg.norm(np.cross(p[d[:, 1]] - p[d[:, 0]], p[d[:, 2]] - p[d[:, 0]]), axis=1) / 2
    gew = np.bincount(s, weights=fl) ** 0.6
    gr = gruppe_von(s)
    pp, dd, n, basis = [], [], [], 0
    for g in range(len(GRUPPEN)):
        n.append(0)
        for k in np.unique(s[gr == g]):
            u, inv = np.unique(d[s == k], return_inverse=True)
            q, f = p[u], inv.reshape(-1, 3)
            z = max(12, int(ziel * gew[k] / gew.sum()))
            if len(f) > z and GRUPPEN[g][0] != "reifen":   # Reifen bleiben rund und stehen auf dem Boden
                q, f = fast_simplification.simplify(q, f, target_count=z, agg=7)
            pp.append(q)
            dd.append(f + basis)
            n[g] += len(f)
            basis += len(q)
    return np.concatenate(pp), np.concatenate(dd), n


def schreiben(pfad: Path, p: np.ndarray, d: np.ndarray, n: list[int]) -> None:
    """Format siehe modell_lesen."""
    skala = float(np.abs(p).max()) / 32767
    kopf = struct.pack("<4sIIIf", MAGIC, len(p), len(d), len(GRUPPEN), skala)
    for (_, farbe, _), k in zip(GRUPPEN, n):
        kopf += bytes.fromhex(farbe[1:]) + b"\0" + struct.pack("<I", k)
    pos = np.round(p / skala).astype("<i2").tobytes()
    rest = b"\0" * (-len(pos) % 4)
    pfad.write_bytes(kopf + pos + rest + d.astype("<u2").tobytes())


def modell_lesen(pfad: Path):
    """Kopf: Magic, Punkte, Dreiecke, Gruppen (u32), Skala cm/Einheit (f32); je Gruppe RGB, 0, Dreiecke
    (u32). Dann Int16-Positionen (x vorn, y links, z hoch), aufgefüllt auf 4 Byte, Uint16-Indizes."""
    b = pfad.read_bytes()
    magic, n_p, n_d, n_g, skala = struct.unpack_from("<4sIIIf", b)
    assert magic == MAGIC
    o, gr = 20, []
    for _ in range(n_g):
        gr.append((b[o:o + 3].hex(), struct.unpack_from("<I", b, o + 4)[0]))
        o += 8
    p = np.frombuffer(b, "<i2", n_p * 3, o).reshape(-1, 3) * skala
    o += n_p * 6
    o += -o % 4
    d = np.frombuffer(b, "<u2", n_d * 3, o).reshape(-1, 3)
    return p, d, gr


def vorschau(pfad: Path, p: np.ndarray, d: np.ndarray, gr: list, w: int = 600) -> None:
    """Kontrollbild aus der fertigen Datei: vorn, links, oben, hinten rechts, Standardansicht.
    Von hinten nach vorn gemalt, Verdeckung daher stellenweise falsch; maßgeblich ist der Browser."""
    import cv2
    farben = np.concatenate([np.tile([int(h[4:6], 16), int(h[2:4], 16), int(h[:2], 16)], (k, 1)) for h, k in gr])
    n = np.cross(p[d[:, 1]] - p[d[:, 0]], p[d[:, 2]] - p[d[:, 0]])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    licht = np.array([0.5, 0.35, 1.0]) / np.linalg.norm([0.5, 0.35, 1.0])
    m = (p.min(0) + p.max(0)) / 2
    k = w / (1.05 * np.linalg.norm(p.max(0) - p.min(0)))
    kacheln = []
    for az, el, name in ((0, 2, "vorn"), (90, 2, "links"), (0, 88, "oben"), (220, 25, "hinten rechts"), (40, 28, "Standard")):
        a, e = np.radians(az), np.radians(el)
        v = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        r = np.cross([0, 0, 1], v)                     # Blick -v: rechts = -v x z = z x v
        r /= np.linalg.norm(r)
        u = np.cross(v, r)
        q = p - m
        xy = np.round(np.stack([w / 2 + k * q @ r, w / 2 - k * q @ u], 1) * 16).astype(np.int32)
        bild = np.full((w, w, 3), 38, np.uint8)
        vorz = np.sign(n @ v)                          # zum Betrachter gedreht wie im Shader
        hell = 0.42 + 0.58 * np.maximum(0, (n * vorz[:, None]) @ licht)
        for i in np.argsort((q @ v)[d].mean(1)):
            cv2.fillConvexPoly(bild, xy[d[i]], (farben[i] * hell[i]).tolist(), cv2.LINE_AA, 4)
        cv2.putText(bild, name, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        kacheln.append(bild)
    kacheln.append(np.full((w, w, 3), 38, np.uint8))
    cv2.imwrite(str(pfad), np.vstack([np.hstack(kacheln[:3]), np.hstack(kacheln[3:])]))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--step", type=Path, default=ROOT / "docs" / "Rider-Pi.STEP")
    ap.add_argument("--cache", type=Path, required=True, help="Zwischenspeicher (.npz) des vernetzten Modells")
    ap.add_argument("--out", type=Path, default=ROOT / "web" / "rider-model.bin")
    ap.add_argument("--tris", type=int, default=17000, help="Dreiecke im Ergebnis (Zielwert, wird etwas überschritten)")
    ap.add_argument("--abw", type=float, default=0.5, help="Vernetzung: Abweichung (mm), nur ohne vorhandenen Cache")
    ap.add_argument("--winkel", type=float, default=0.8, help="Vernetzung: Winkel (rad), nur ohne vorhandenen Cache")
    ap.add_argument("--min-px", type=int, default=300, help="Solids mit weniger sichtbaren Pixeln entfallen")
    ap.add_argument("--vorschau", type=Path, help="Kontrollbild (PNG)")
    a = ap.parse_args()

    p, d, s = netz_laden(a.step, a.cache, a.abw, a.winkel)
    p = ausrichten(p, s, d)
    px = sichtbar(p, d)
    je_solid = np.bincount(s, weights=px, minlength=s.max() + 1)
    print(f"[modell] sichtbar: {(px > 0).sum()} von {len(d)} Dreiecken, "
          f"{(je_solid >= a.min_px).sum()} von {len(je_solid)} Solids", flush=True)
    p, d = verschweissen(p, d, s)
    an = nachbarn_dazu(d, px > 0, len(p)) & (je_solid[s] >= a.min_px)
    d, s = d[an], s[an]
    p2, d2, n = vereinfachen(p, d, s, a.tris)
    schreiben(a.out, p2, d2, n)
    p3, d3, gr = modell_lesen(a.out)
    print(f"[modell] {a.out.name}: {len(p3)} Punkte, {len(d3)} Dreiecke, {a.out.stat().st_size / 1024:.0f} KB, "
          f"Maße {p3.min(0).round(2)} .. {p3.max(0).round(2)} cm")
    for (name, _, _), (_, k) in zip(GRUPPEN, gr):
        print(f"[modell]   {name}: {k}")
    if a.vorschau:
        vorschau(a.vorschau, p3, d3, gr)


if __name__ == "__main__":
    sys.exit(main())
