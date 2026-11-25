# Arbeitsplan Architektur (ab 2026-11-24)

Arbeitsdokument, wird mit den folgenden Fragen fortgeschrieben.

## Zielbild

Kamera + FloorNet + IMU schreiben Merkkarte und Raumkarte. Das Gehirn (ein Modus) äußert einen Wunsch an den
Planer, der Planer fährt den Rider, nach der Sicherheitsprüfung.

## Plan

1. **Nur ein Gehirn mit Modus, jeder Weg läuft über den Planer.**
   Bei Dunkelangst wird der schnellste Weg zurück gefahren.
2. **Test: Anfahren an eine Wand.**
3. **Test: Fahren einer Bahn entlang.** Es geht um das Glätten der Kurven und das Vermeiden von Drehungen auf der
   Stelle, dem Hauptgrund der Drift.
4. **Fahrregler in die Firmware.** Heute wirken die Fahrbefehle wie eine Kraft. Künftig überträgt der Pi die
   Sollgeschwindigkeit (m/s) mit Sollbeschleunigung, beim Drehen die Drehrate in °/s (mit Sollbeschleunigung).
   Die Firmware regelt die Kraft selbst.
   Kosten am Pi: sinken (Geschwindigkeitsregelung `robot.speed_ctrl` am Pi entfiele, nur noch Sollwerte senden).

## Ist-Stand Fahrbefehl (für Punkt 4)

- Pi sendet `drive(v, w)` normiert [-1, 1] → u8 (128 = Stillstand, `robot.norm_to_u8`), Faktor × `tempo` × `max_v`;
  Umrechnung auf m/s nur über `robot.vx_limit` 1,8 m/s je Firmware-Einheit (gemessen 2026-11-05).
- Firmware: Geschwindigkeitsregler nur P, dazu 0015 (Integral, hält nur eine Untergrenze von 30 % des Befehls;
  Aktion 239/238). Die Firmware hält Sollwerte, Stopps müssen explizit geschrieben werden.
- Pi: zusätzlich `robot.speed_ctrl` (`xgonav/robot.py:140`), stellt den gesendeten Wert über die Räder nach.
- Drehen: `w` normiert, Untergrenze `robot.turn_min_w` wegen Totband; geregelt wird am Pi über `dreh_w`/IMU-Kurs.

## Ist-Stand Dunkelangst (Befund 2026-11-24)

- `App._dunkel_tick` (`xgonav/app.py:487`) sitzt im Regelschritt zwischen `safety()` und `_arc_check()` und
  ersetzt den Befehl des Verhaltens durch `Command(0, w)`: Drehen auf der Stelle mit eigenem Kursziel
  (`dreh_auf_kurs`), am Planer vorbei. Das ist ein zweites Gehirn.
- Ablauf (`xgonav/dunkel.py`): 2 s unter 7 → Angst 1,5 s → 180° drehen → 1,5 s prüfen (über 11 = weiter) →
  +90°, +180° → aufgeben (absetzen, Ruhemodus); 3-mal in 300 s → sofort aufgeben.
- Nur `explore` (`dunkel.zone_modes`): Dunkelzone 2 × 3 m voraus + `neu_planen()` löscht Ziel und Weg; deshalb
  war in dem Moment kein Weg zu sehen.
- Andere Modi (`home`, `patrol`, `follow`, `approach`, `pet`): keine Zone, kein Neuplanen; das Verhalten rechnet
  weiter, seine Befehle werden verworfen, danach fährt der alte Weg weiter (evtl. zurück ins Dunkle).

## Vorläufiger Plan Firmware-Session (Punkt 4)

### Ist in der Firmware (gelesen 2026-11-24, `firmware/build/xgorider2_rl/firmware/src`)

Am Rider läuft Env `rider2-firmware`: klassische Kaskade, RL-Policy aus (`Mode::Off`, reicht die Räder durch).
Pro Regeltakt (~3 ms) in `MotionController::step` → `control/rider_motion.cpp`:

| Stufe | Funktion | heute |
|---|---|---|
| Sollwert | `updateCommandTargets` | Byte 0x30 → ±`max_forward` 210, Byte 0x32 → ±`max_turn` 300, Totband 5 %; Tiefpass 0,95/Takt (~60 ms), vorwärts zusätzlich max. 5 Einheiten/Takt |
| Tempo | `updateBalance`, `speedPid` | PI gegen mittlere Radgeschwindigkeit (`filtered_speed_mm_s`), Integral bei 100 gekappt → praktisch nur P (0,11) → Soll-Nick; 0015 hält nur 30 % als Untergrenze. **Darum wirkt der Befehl wie eine Kraft:** bleibende Regelabweichung, Last bremst ihn |
| Nick | `pitchPid` | P 37,5 + Nickrate × (Höhe + 0,45) × 100 → Radmoment |
| Rad je Seite | `addWheelSpeedCorrection` | ab |v| ≥ 10: 2 × (Soll − Ist) je Rad, Drehanteil aus turn/180·π·0,04/0,003583 |
| Kurs | `addHeadingCorrection`, `headingPid` | Kursziel += filtered_turn × 0,005 je Takt, PID (3/0/0,5) gegen IMU-Kurs + Vorsteuerung −turn × 0,5 |
| Halten | `updatePositionHold` | Position halten im Stand (0008: auch beim Drehen) |

Die Einheiten heißen schon mm/s und °/s (`state.forward_mm_s`, `turn_deg_s`), sind aber nicht kalibriert
(widerspricht `robot.vx_limit` 1,8 → am Rider nachmessen).

### Ziel

Pi sendet `v_soll` (mm/s) + `a_max` (mm/s²) und `w_soll` (°/s) + `alpha_max` (°/s²). Die Firmware erreicht und hält
diese Werte selbst, auch gegen Last, ohne Pendeln.

### Stufen

1. **Schnittstelle:** neues Register (Vorschlag 0x1a, in `main.cpp`/`command_protocol` frei), ein Rahmen, atomar:
   `int16 v [mm/s]`, `uint16 a [mm/s²]`, `int16 w [0,1 °/s]`, `uint16 alpha [°/s²]`. Alte Bytes 0x30/0x32 bleiben;
   der zuletzt beschriebene Weg gilt. Sollwerte werden gehalten wie heute; zusätzlich Firmware-Wächter: kein neuer
   Rahmen für N ms → Rampe auf 0.
2. **Sollwertgenerator:** `v_ref` läuft mit `a_max` auf `v_soll` zu, `w_ref` mit `alpha_max` auf `w_soll`
   (ersetzt Tiefpass und 5er-Schritt). Beschleunigungsgrenze der Firmware unabhängig vom Pi.
3. **Tempo-Regler:** echter PI auf `v_ref` mit Anti-Windup (Integral nur, wenn der Nick-Sollwert nicht in der
   Grenze), plus Vorsteuerung Soll-Nick ≈ atan(a_ref / g) beim Anfahren/Bremsen. Ersetzt 0015.
   Gefahr wie 0015 v1 (Pendeln Periode ~5 s): Integral klein anfangen, im Host-Test gegen ein Pendelmodell prüfen.
4. **Drehraten-Regler:** PI auf Gierrate (`gyro z − bias`) gegen `w_ref` + Vorsteuerung; Kursziel integriert
   `w_ref` weiter, damit Kurshalten bei w = 0 bleibt. Kleine Drehraten trotz Haftreibung (heute `turn_min_w`).
5. **Halten:** v = 0 und w = 0 → Positions- und Kurshalten wie heute (0008/0010).
6. **Host-Tests** (`firmware/tests/`, auf kimmy): Sprungantworten v/w, Rampe, Last (konstante Kraft),
   Wächter, alter Weg bitgleich, wenn das neue Register nie beschrieben wird.
7. **Treiber:** `/proc/XGORider/vel` (Text `v a w alpha`) schreibt das Register; `driver-test/run.sh` + Emulator.
8. **xgonav:** `Robot.drive` sendet m/s und °/s; `robot.speed_ctrl`, `turn_min_w`, `vx_limit` entfallen
   (erst nach Messfahrt). Planer/PathFollower geben v/w in echten Einheiten.
9. **Messung am Rider** (mit Nutzer, je Schritt freigegeben): liegend Register lesen → Stand balancierend →
   v-Sprünge 0,1/0,2 m/s, Drehen 20/45 °/s; Rad-, IMU- und Kamera-Odometrie vergleichen.

Kosten am Pi: sinken (kein Temporegler, keine Drehregelung über den IMU-Kurs je Regelschritt; nur Sollwerte).
Kosten am ESP32: zwei PI-Regler + Rampe je Takt, vernachlässigbar gegen die Lagefusion.

### Stand

- **Stufe 1 (2026-11-25):** Patch `firmware/patches/0029-velocity-frame.patch` (`velocity_cmd.h`): 0x1a Rahmen
  13 B / lesen 12 B, 0x1b Grenzen + Wächter 6 B, Beschleunigungsbetrieb, 0xa0/0xa1 raus, Konsole `!vel …`.
  Vorerst gehen v/w (begrenzt, ohne Byte-Totband) in die alte Kaskade; a/Ruck gespeichert für Stufe 2.
  Gebaut (`out/app0.bin`, SHA 7e0094fd…, 1 262 624 Byte), alle Host-Tests auf kimmy grün (`test_velocity_cmd.cpp`
  neu). Nicht geflasht, Treiber und xgonav noch ohne den Rahmen.
- **Stufe 2 (2026-11-25):** Patch `0030-velocity-generator.patch`: S-Kurve je Achse (a, Ruck; alpha, Drehruck),
  ersetzt für den Rahmenweg Tiefpass und 5er-Schritt; Wächter bremst mit den Rahmengrenzen; Übernahme aus der
  Fahrt. Gebaut (SHA d5f01194…, 1 263 776 Byte), alle Host-Tests grün. Nicht geflasht.
  Für Stufe 4 gesehen: `addHeadingCorrection` erhöht das Kursziel je Takt um filtered_turn × 0,005, bei ~333 Hz
  also um ~1,67 × w je Sekunde (nicht 1 × w); dazu die Vorsteuerung −turn × 0,5. Welche Drehrate wirklich
  herauskommt, ist ungemessen.

### Zu klären


### Entschieden (Nutzer 2026-11-25)

- Höchste Beschleunigungen (`a_max`, `alpha_max`) werden in xgonav konfiguriert (Dataclass in `config.py`) und mit
  jedem Rahmen gesendet. (Vorschlag, nicht entschieden: Firmware klemmt zusätzlich auf eine feste Obergrenze.)
- Die Radgeschwindigkeit der Firmware (`filtered_speed_mm_s`) ist echte mm/s; `v_soll` wird in mm/s gesendet.
- Höchstwerte (größtes `a_max` ohne Kippen, größte Drehrate und Drehbeschleunigung) werden am Rider gemessen
  (Stufe 9), danach in `config.py` eingetragen.
- Register 0xa0/0xa1 (`max_forward`/`max_turn`, Bereich der Bytes 0x30/0x32) kommen raus. Der alte Byteweg
  rechnet fest mit 210 mm/s / 300 °/s, `automatic_lean` ebenso; die Konsolenbefehle `!maxforward`/`!maxturn`
  entfallen mit. Am Pi schreibt sie heute niemand.
- Ruckbegrenzung kommt rein (Sollwertgenerator Stufe 2: S-Kurve für v und w). Rahmen in Stufe 1 um Ruck
  (mm/s³, °/s³) erweitern; Höchstwerte wie `a_max` in xgonav, gemessen in Stufe 9.
- Beschleunigungsbetrieb: Rahmen ohne Geschwindigkeit, aber mit Beschleunigung → diese ausführen, positiv wie
  negativ (Beschleunigung also mit Vorzeichen), ohne Geschwindigkeitsregelung, begrenzt durch `v_max`.
  Folgt daraus: Rahmen braucht ein Kennzeichen „keine Geschwindigkeit“.
- `v_max`/`w_max` schreibt xgonav beim Start in die Firmware (eigenes Register, Vorschlag 0x1b), nicht je Rahmen.
  Nur fürs Fahren; Drehen braucht immer `w`.
- Der Temporegler bleibt ein Faktor auf alle Sollwerte (v und w), wie heute Faktor × `tempo` × `max_v`/`max_w`.

## Offene Fragen

(folgen)
