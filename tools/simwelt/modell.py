#!/usr/bin/env python3
"""Modell des Erdgeschosses aus den Karten der Fahrten ab 2026-11-23 (Maße in m, abgelesen aus der gemeinsamen Karte).
Erzeugt welt.json: python modell.py > welt.json

Bezug: x nach Osten (Fensterfront), y nach Norden (Küche); Ursprung = Südwestecke des umschließenden Rechtecks.
"""
import json

H = 2.4          # Wandhöhe
T = []


def teil(name, x1, y1, x2, y2, z0=0.0, z1=H, fest=True, tex="foto", sichtbar=True):
    T.append({"name": name, "x1": x1, "y1": y1, "x2": x2, "y2": y2, "z0": z0, "z1": z1, "fest": fest, "tex": tex,
              "sichtbar": sichtbar})


# --- Wände und alles außerhalb (volle Höhe) ---
teil("Wand N Kueche", 0.0, 8.5, 10.0, 9.0)
teil("Wand W Kueche", 0.0, 5.95, 1.25, 9.0)
teil("Wand Kueche/Nische", 0.0, 5.95, 3.75, 6.4)
teil("Wand O Kueche", 4.45, 5.9, 10.0, 9.0)
teil("Wand W Nische", 0.0, 0.0, 0.6, 5.95)
teil("Wand S Nische", 0.0, 0.0, 3.5, 4.45)
teil("Wand hinter TV", 0.0, 0.0, 4.25, 3.0)
teil("Wand S", 0.0, 0.0, 10.0, 0.4)
teil("Wand O Fenster", 9.75, 0.0, 10.0, 9.0)
teil("Wand N Ost", 8.25, 5.3, 10.0, 5.9)
teil("Wand O Nord", 8.75, 4.2, 10.0, 5.9)
# --- Küche ---
teil("Zeile N", 1.25, 7.9, 4.45, 8.5, 0.0, 0.9)
teil("Zeile S", 1.25, 6.4, 3.75, 7.0, 0.0, 0.9)
# --- Hauptraum ---
teil("Schrankwand", 5.5, 5.3, 8.25, 5.9)
teil("Hocker", 8.0, 4.2, 8.75, 5.3, 0.0, 0.42)
teil("Podest O", 8.9, 0.4, 9.75, 4.2, 0.0, 0.3)
teil("Podest S", 8.6, 0.4, 8.9, 2.6, 0.0, 0.3)
teil("Pflanze Topf", 4.8, 5.25, 5.2, 5.65, 0.0, 0.4, tex="topf")
teil("Pflanze", 4.55, 5.0, 5.45, 5.9, 0.4, 1.5, fest=False, tex="pflanze")
teil("Luftreiniger", 4.45, 2.95, 4.7, 3.2, 0.0, 0.5)
# --- Wohnbereich ---
teil("TV-Schrank", 4.25, 0.4, 4.68, 3.0, 0.0, 0.55)
teil("Fernseher", 4.3, 1.2, 4.38, 2.4, 0.62, 1.3, fest=False)
teil("Couchtisch", 5.45, 2.05, 6.2, 2.85, 0.0, 0.45)
teil("Sofa Sitz", 6.3, 1.5, 8.0, 3.4, 0.0, 0.42)
teil("Sofa Sued", 5.45, 0.4, 7.3, 1.4, 0.0, 0.42)
teil("Sofa Sued Lehne", 5.45, 0.4, 7.3, 0.65, 0.0, 0.8)
teil("Mondkissen", 4.95, 0.45, 5.4, 0.9, 0.0, 0.45)
teil("Sofa Lehne O", 7.6, 1.6, 8.0, 3.4, 0.0, 0.8)
teil("Sofa Lehne N", 6.3, 3.1, 8.0, 3.4, 0.0, 0.8)
for k in range(8):                       # Holztreppe an der Südwand, steigt nach Osten
    teil("Treppe %d" % k, 7.4 + 0.15 * k, 0.4, 7.4 + 0.15 * (k + 1), 1.3, 0.0, 0.18 * (k + 1), tex="holz")
# --- Essnische ---
teil("Tischplatte", 1.5, 5.12, 3.1, 5.92, 0.72, 0.76, fest=False, tex="tisch")
for x, y in ((1.55, 5.17), (2.99, 5.17), (1.55, 5.81), (2.99, 5.81)):
    teil("Tischbein", x, y, x + 0.06, y + 0.06, 0.0, 0.72, tex="tisch")
STUEHLE = [(1.9, 4.95, "s"), (2.7, 4.95, "s"), (3.35, 5.5, "o"), (1.25, 5.5, "w")]
# Schräge Wand mit dunkler Tür zwischen Essnische und TV-Wand
# von der Raumseite gesehen läuft sie von (4.7, 3.0) links nach (3.5, 4.45) rechts
SCHRAEG = [{"name": "Schraege", "x1": 4.7, "y1": 3.0, "x2": 3.5, "y2": 4.45, "z0": 0.0, "z1": H}]
TEPPICHE = [{"name": "Teppich", "x1": 5.0, "y1": 0.9, "x2": 6.9, "y2": 3.3}]
json.dump({"size": 10.0, "size_y": 9.0, "wall_h": H, "start": [7.3, 4.3, 3.14159], "teile": T, "stuehle": STUEHLE,
           "schraeg": SCHRAEG, "teppiche": TEPPICHE}, __import__("sys").stdout, indent=1)
