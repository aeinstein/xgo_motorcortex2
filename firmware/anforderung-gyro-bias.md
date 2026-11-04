# Anforderung: Gyro-Nullpunkt (Gier) nachführen – Rider dreht sich im Stand weg

Stand: 2026-11-04, Firmware R-1.1.6 „Rider2-recovered-full“ + Patches 0001–0003, RL-Regler aktiv.

## Befund am RiderPi

Rider balanciert im Stand, Fahrbefehle neutral (128/128). Messung über 30 s:

| Quelle | Änderung |
|---|---|
| Kamera (xgonav, Ego-Motion) | −77,8° (≈ −2,6°/s, Rechtsdrehung) |
| Rad 11 (links) | 0 Ticks |
| Rad 21 (rechts) | −640 Ticks (−11,8 cm) |
| Firmware-Yaw (0x0d) | −0,3° |

ESP32-Uptime zum Messzeitpunkt ~17 min. Aus Radweg und Kamerawinkel ergibt sich eine wirksame Spurweite von ≈ 87 mm.

## Erklärung aus dem Quelltext

- `gyro_bias[2]` wird nur einmal beim Boot gemessen (`control/rider_leg_imu.cpp` ~Z. 107: 250 Proben, einzige Bedingung |roll| < 4°, keine Prüfung auf Stillstand) und danach nie wieder.
- Der RL-Regler regelt per `yaw_integral` (`policy/policy.cpp` ~Z. 169) die Rate `gyro_z − gyro_bias[2]` auf den Sollwert 0. Ist der Bias falsch, dreht der Rider real mit genau dem Bias-Fehler.
- Der Yaw-Winkel integriert nur bei |Rate| > 2°/s (`rider_leg_imu.cpp` Z. 142) und sieht die Drift deshalb nicht.
- `heading_hold` (0x0A) wirkt nur im klassischen Regler (`addHeadingCorrection`), nicht mit aktivem RL-Regler.

## Gewünscht

### 1. Boot-Kalibrierung robuster

Zusätzlich Stillstand verlangen, z. B. Streuung von `gyro_z` über die 250 Proben < ~0,5°/s und |gyro| klein. Bei Bewegung neu beginnen, statt einen falschen Bias zu übernehmen.

### 2. Laufende Nachführung von `gyro_bias[2]`

a) **Disarmed/liegend**, Räder stehen, Beschleunigung ruhig: Bias langsam auf `gyro_z` ziehen (Tiefpass, τ ≈ 10 s).

b) **Balancierend, ohne Drehbefehl** (Turn-Byte im Totband, `filtered_turn` ≈ 0), Raddaten frisch:

```
ω_räder = Gierrate aus den Rad-Encodern
          (Differenz der Radgeschwindigkeiten / Spurweite; wirksam am RiderPi ≈ 87–90 mm)
gyro_bias[2] += dt/τ · ((gyro_z − gyro_bias[2]) − ω_räder),   τ ≈ 10–20 s
```

- Nur bei |ω_räder| < ~15°/s.
- Vorzeichen von ω_räder gegen `gyro_z` prüfen. Referenz am RiderPi: rechtes Rad rückwärts, linkes steht = Rechtsdrehung, Kamera-Kurs fällt.
- Gesamtänderung gegenüber dem Boot-Wert begrenzen (z. B. ±5°/s), Änderungsrate begrenzen.

c) **Abschaltbar:** Konsole `!biastrack on|off|status`, Default on.

### 3. Neu messen auf Kommando

- Konsole `!gyrocal`: liegend Mittel von `gyro_z` über 2 s; balancierend Mittel von `gyro_z − ω_räder` über 3 s (Rider soll dabei nicht fahren). Ergebnis ausgeben.
- Zusätzlich als Binärbefehl, damit der Pi-Treiber es auslösen kann (freies Register selbst wählen, z. B. Schreiben 0xb4 = 1).

### 4. Auslesbar (Diagnose für den Pi)

- Lese-Register (z. B. 0xb3), little endian:

| Feld | Typ |
|---|---|
| `gyro_bias_z` (°/s) | float |
| `gyro_bias_z_boot` (°/s) | float |
| `ω_räder` (°/s) | float |
| `gyro_z` korrigiert (°/s) | float |
| flags: bit0 Nachführung an, bit1 Nachführung gerade aktiv, bit2 Boot-Kalibrierung wegen Bewegung wiederholt | uint8 |

- Im `!status`-JSON: `"gyro_bias_z"`, `"gyro_bias_boot"`, `"biastrack"`.
- Register und Befehle in `debug_firmware/protocol.md` dokumentieren.

## Nicht ändern

Bestehende Register (insb. 0x0c, 0x0d, 0x62–0x68), Policy-Gewichte, Protokoll-Framing. Die 2°/s-Totzone des Yaw-Winkels erst einmal lassen.

## Abnahme am RiderPi

- 30 s balancierend im Stand mit neutralen Befehlen: Drehung laut Kamera/Rädern < 3° (< 0,1°/s); bisher 78°.
- Gleiches nach ≥ 30 min Betrieb (Temperaturdrift).
- Boot mit bewegtem Rider: Kalibrierung wiederholt sich, Drift bleibt klein.
- `!gyrocal` und das Lese-Register liefern plausible Werte; Nachführung lässt sich per `!biastrack off` abschalten (dann Verhalten wie bisher).
