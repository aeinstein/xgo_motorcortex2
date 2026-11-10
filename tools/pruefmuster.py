#!/usr/bin/env python3
"""Prüfmuster zum Ausdrucken: Gitter aus 5 mm breiten Linien, verteilt auf DIN-A4-Blätter zum Zusammenkleben.

  python tools/pruefmuster.py pruefmuster.pdf                  # 5 x 4 Blätter hochkant = 1,00 m x 1,00 m
  python tools/pruefmuster.py muster.pdf --spalten 3 --reihen 2 --raster 40 --kachel 200x240

Je Blatt eine Kachel (Standard 200 x 250 mm, mittig auf dem Blatt). Die Linienmitten liegen im ganzen
Muster alle --raster mm; die Kachelkanten liegen mitten in den hellen Feldern, an den Stößen trifft also
nie Linie auf Linie längs, nur quer. Mit 100 % drucken ("tatsächliche Größe"), an der grauen Haarlinie
schneiden (oder nur links und oben schneiden und überlappend kleben), nach den Blattnamen legen:
Buchstabe = Spalte von links, Zahl = Reihe von oben. Schwarzanteil bei 50 mm Raster: 19 %.

Schreibt das PDF selbst (nur Rechtecke und Helvetica), braucht keine Bibliothek.
"""
from __future__ import annotations

import argparse
import sys

MM = 72.0 / 25.4
A4 = (210.0, 297.0)


def _seite(sp: int, re: int, spalten: int, reihen: int, kw: float, kh: float, raster: float, linie: float,
           skala: tuple[float, float] = (1.0, 1.0)) -> bytes:
    """Inhalt eines Blatts (PDF-Operatoren). sp/re ab 0, Reihe 0 = oben. skala (Breite, Höhe) vergrößert
    alles um die Blattmitte (Drucker, der die Seite verkleinert, auch je Richtung verschieden); die Maße
    im Text bleiben die gedruckten Sollmaße."""
    x0, y0 = (A4[0] / skala[0] - kw) / 2, (A4[1] / skala[1] - kh) / 2   # Kachel unten links (mm im Sollmaß)
    gx, gy = sp * kw, (reihen - 1 - re) * kh             # Kachel unten links im Muster (mm, y nach oben)
    o = []

    def rect(x, y, w, h):
        o.append("%.3f %.3f %.3f %.3f re" % (x * MM, y * MM, w * MM, h * MM))

    o.append("%.3f %.3f %.3f %.3f re W n" % (x0 * MM, y0 * MM, kw * MM, kh * MM))   # nur in der Kachel
    o.append("0 g")
    for g0, lo, breit, senkrecht in ((gx, x0, kw, True), (gy, y0, kh, False)):
        k = int((g0 - linie) // raster)
        while True:
            m = raster / 2 + k * raster                  # Linienmitte im Muster
            k += 1
            if m + linie / 2 <= g0:
                continue
            if m - linie / 2 >= g0 + breit:
                break
            p = lo + (m - g0) - linie / 2
            if senkrecht:
                rect(p, y0, linie, kh)
            else:
                rect(x0, p, kw, linie)
    o.append("f")
    seite = ["%.6f 0 0 %.6f 0 0 cm" % skala, "q"] + o + ["Q"]
    # Schnittkante: graue Haarlinie um die Kachel, Schnittmarken in den Rändern
    seite += ["0.6 G %.3f w" % (0.1 * MM), "%.3f %.3f %.3f %.3f re S" % (x0 * MM, y0 * MM, kw * MM, kh * MM),
              "0 G %.3f w" % (0.2 * MM)]
    for cx in (x0, x0 + kw):
        for a, b in ((y0 - 12, y0 - 2), (y0 + kh + 2, y0 + kh + 12)):
            seite.append("%.3f %.3f m %.3f %.3f l S" % (cx * MM, a * MM, cx * MM, b * MM))
    for cy in (y0, y0 + kh):
        for a, b in ((0.5, x0 - 1), (x0 + kw + 1, A4[0] / skala[0] - 0.5)):
            seite.append("%.3f %.3f m %.3f %.3f l S" % (a * MM, cy * MM, b * MM, cy * MM))
    name = "%s%d" % (chr(ord("A") + sp), re + 1)

    def text(x, y, size, s, grau=0.0):
        s = s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
        seite.append("BT /F1 %g Tf %g g %.3f %.3f Td (%s) Tj ET" % (size, grau, x * MM, y * MM, s))

    text(x0 + 4, y0 + kh + 14, 11, "Blatt %s" % name)
    text(x0 + 30, y0 + kh + 14, 8, "Spalte %d von %d (von links), Reihe %d von %d (von oben)   ·   mit 100 %% drucken"
         % (sp + 1, spalten, re + 1, reihen))
    text(x0 + 4, y0 + kh + 9.5, 8, "Linienmitten alle %g mm, Linie %g mm breit, Kachel %g x %g mm; an der grauen Linie schneiden"
         % (raster, linie, kw, kh), 0.3)
    text(x0 + 4, y0 - 9, 8, "Muster gesamt %g x %g mm   ·   Kachel beginnt bei x %g mm, y %g mm (von links unten)"
         % (spalten * kw, reihen * kh, gx, gy), 0.3)
    text(x0 + 1.5, y0 + kh - 4, 7, name, 0.55)           # klein im hellen Eckfeld, bleibt nach dem Schneiden
    return "\n".join(seite).encode("cp1252")


def pdf(spalten: int, reihen: int, kw: float, kh: float, raster: float, linie: float,
        skala: tuple[float, float] = (1.0, 1.0)) -> bytes:
    objs: list[bytes] = []                               # Objekt i+1

    def add(b: bytes) -> int:
        objs.append(b)
        return len(objs)

    add(b"")                                             # 1 Katalog
    add(b"")                                             # 2 Seitenbaum
    font = add(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>")
    kids = []
    for re in range(reihen):
        for sp in range(spalten):
            c = _seite(sp, re, spalten, reihen, kw, kh, raster, linie, skala)
            cid = add(b"<< /Length %d >>\nstream\n" % len(c) + c + b"\nendstream")
            kids.append(add(b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] /Contents %d 0 R "
                            b"/Resources << /Font << /F1 %d 0 R >> >> >>" % (A4[0] * MM, A4[1] * MM, cid, font)))
    objs[0] = b"<< /Type /Catalog /Pages 2 0 R >>"
    objs[1] = b"<< /Type /Pages /Count %d /Kids [%s] >>" % (len(kids), b" ".join(b"%d 0 R" % k for k in kids))
    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offs = []
    for i, b in enumerate(objs, 1):
        offs.append(len(out))
        out += b"%d 0 obj\n" % i + b + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    for o in offs:
        out += b"%010d 00000 n \n" % o
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, xref)
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ziel")
    ap.add_argument("--spalten", type=int, default=5)
    ap.add_argument("--reihen", type=int, default=4)
    ap.add_argument("--raster", type=float, default=50.0, help="Abstand der Linienmitten (mm)")
    ap.add_argument("--linie", type=float, default=5.0, help="Linienbreite (mm)")
    ap.add_argument("--kachel", default="200x250", help="bedruckte Fläche je Blatt, Breite x Höhe (mm)")
    ap.add_argument("--gedruckt", default="", metavar="BxH",
                    help="Drucker verkleinert: so groß (mm) kam die Kachel des Probedrucks heraus, z. B. 192.5x249; "
                         "das Muster wird je Richtung so vergrößert, dass der Druck wieder die Sollmaße hat")
    ap.add_argument("--probe-kachel", default="200x250", help="Kachel des Probedrucks im PDF (mm)")
    a = ap.parse_args()
    kw, kh = (float(v) for v in a.kachel.lower().split("x"))
    skala = (1.0, 1.0)
    if a.gedruckt:
        g, s = ([float(v) for v in t.lower().split("x")] for t in (a.gedruckt, a.probe_kachel))
        skala = (s[0] / g[0], s[1] / g[1])
    if kw * skala[0] > A4[0] or kh * skala[1] > A4[1]:
        raise SystemExit("Kachel größer als DIN A4")
    with open(a.ziel, "wb") as f:
        f.write(pdf(a.spalten, a.reihen, kw, kh, a.raster, a.linie, skala))
    print("%s: %d Blätter, Muster %g x %g mm, im PDF um %.4f / %.4f vergrößert, Rand im PDF %.1f / %.1f mm"
          % (a.ziel, a.spalten * a.reihen, a.spalten * kw, a.reihen * kh, skala[0], skala[1],
             (A4[0] - kw * skala[0]) / 2, (A4[1] - kh * skala[1]) / 2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
