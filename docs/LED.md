# XGO Rider — LED-Bedeutung (aus Firmware-Sicht)

Gilt für die laufende Firmware **R-1.1.6** (`Rider2-recovered-full`). Quelle: `board_support.cpp`
(`renderLeds`/`renderFaultLeds`), `led_status.h` (`ledFaultFor`), Modus-Zuordnung in `main.cpp`.
Die Stock-Firmware R-1.1.3 kann abweichen (nicht analysiert).

## Hardware
- **4 RGB-LEDs** (WS2812/NeoPixel) an **GPIO27** (RMT-Kanal 0). Index 0..3.
- Helligkeit intern ~0,5 skaliert; „voll" ist meist Wert 200–255.

## Prioritäts-Logik
Pro Frame wird zuerst `renderFaultLeds` versucht, sonst `renderLeds`:

1. **Fault-Farbe** wird gezeigt, **außer** wenn `robot_mode==1` (balanciert) oder `robot_mode==250`
   (Low-Batt) — oder kein Fault vorliegt.
2. Sonst normale Darstellung (Fahrtrichtung, Wellen-Animationen, Benutzer-LEDs, Low-Batt-Blinken).

Die Fault-Farbe hängt von `last_stop_reason` ab (`ledFaultFor(driving,last_stop)`), die normale
Darstellung von `mode` und den Fahr-/Aktionswerten. Beide Felder stehen in `!status`
(`mode`, `reason`, `last_stop_reason`, `driving`).

## Fehler-/Stopp-Farben (Rider steht NICHT, Servos i. d. R. freigegeben)

| Farbe | Muster | Bedeutung (LedFault ← Stopp-Grund) |
|------|--------|-------------------------------------|
| **Rot, dauerhaft** | konstant | **Umgefallen / Kippfehler** (Tilt). Auch `mode 0`. |
| **Rot, schnell blinkend** (~5 Hz) | 100 ms an/aus | **Freier Fall erkannt** (Fall) — gestoppt + Servos freigegeben *vor* dem Aufprall. |
| **Rot, langsam blinkend** (~1 Hz) | 500 ms an/aus | **Sonstiger Fehler** (Other): IMU-, Aktuator-, Timing-, ungültige-Regelausgabe-, Policy- oder Kalibrier-Fehler. Servos freigegeben. |
| **Bernstein/Orange** (R200 G70 B0), konstant | konstant | **Manuell gestoppt** (Manual) — per `!stop`, Action 0x3e=254, Schalter oder App. |

## Low-Battery

| Farbe | Muster | Bedeutung |
|------|--------|-----------|
| **Rot, blinkend** (Batterie-Takt) | an/aus mit `low_blink` | **Akku leer / Abschaltung steht bevor** (`mode 250`, `stop_required`). Eigener Blink-Takt, nicht die Fault-Rot-Muster oben. |

## Normalbetrieb (balancierend, `mode 1`)

| Farbe | Wann | Bedeutung |
|------|------|-----------|
| **Benutzer-LEDs** (Default: Regenbogen-Palette) | steht/balanciert, keine Fahrt | Ruhezustand. Farben per `!rgb`/Register 0x69–0x6c frei setzbar. |
| **Grün, richtungsabhängig** | fährt (|forward|>5 oder |turn|>5) | Fahrtrichtung: vorwärts → LEDs 0&3, rückwärts → 1&2, links/rechts-Drehung → jeweils zwei LEDs grün. |

## Aktions-Animationen (während laufender Aktion, `mode 1`)

| Farbe | Wann | Bedeutung |
|------|------|-----------|
| **Blau blinkend** (~1 Hz) | `pitch_trim_stage==2` | **Pitch-Trimmung** läuft (im Stand ausbalancieren). |
| **Blau↔Cyan-Verlauf** (animiert) | Roll-Welle aktiv | Roll-„Welle"-Aktion; Farbton folgt dem Roll-Sollwert. |
| **Grün↔Blau-Ton** | Höhen-Welle aktiv | Height-„Welle"-Aktion; Ton folgt der Höhe. |

## Modus → Situation (für `!status.mode`)

| mode | Situation | LED |
|-----:|-----------|-----|
| 1 | balanciert / fährt | Benutzer-LEDs bzw. grüne Richtung / Wellen-Animation |
| 0 | Kipp-/Sturz-Fehler, „gestrandet", Weglauf-Schutz | rot (Tilt=dauer, Fall=schnell blinkend) |
| 250 | Akku leer / Abschaltung | rot blinkend (Batterie-Takt) |
| 251 | Aktuator-Fehler / Flash-Stopp | rot langsam blinkend (Other) |
| 252 | Policy-/Regel-/Control-Fehler | rot langsam blinkend (Other) |
| 253 | nach Flash / bereit-aber-disarmed / neutral | keine Fault-Farbe (LEDs bleiben wie zuletzt / Benutzer-LEDs) |
| 254 | manueller Stopp | bernstein/orange |
| 255 | Beinkalibrierung läuft | (Kalibriermodus) |

## Kurzmerker
- **Rot** = Problem/gestoppt (Muster unterscheidet Ursache: dauer=Kippen, schnell=Fall, langsam=Fehler, Batterie-Takt=Akku).
- **Orange** = bewusst gestoppt (manuell).
- **Grün** = fährt (Richtung).
- **Blau** = Trimmung/Roll-Welle.
- **Bunt/ruhig** = steht entspannt (Benutzer-LEDs).

Zustand live ablesen: `!status` → `mode`, `driving`, `reason`, `last_stop_reason`. LEDs selbst
setzen (nur im Ruhezustand sichtbar): `!rgb <index 0..3> <r> <g> <b>` bzw. Register 0x69–0x6c.
