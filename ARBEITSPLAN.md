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
  Für Stufe 4 gesehen: `addHeadingCorrection` erhöht das Kursziel je Regeltakt um filtered_turn × 0,005; der
  Regeltakt ist 4 ms (`kControlPeriodUs`, 250 Hz; gemessen balancierend ~260 Hz), also ~1,25–1,3 × w je Sekunde
  (nicht 1 × w); dazu die Vorsteuerung −turn × 0,5. Welche Drehrate wirklich herauskommt, ist ungemessen.
- **Stufe 3 (2026-11-25):** Patch `0031-velocity-control.patch`: Soll-Nick = Original-P + Integral (ki 0,05 °/(mm/s·s),
  ±4°, Anti-Windup, steht beim Hebeschutz, klingt beim Halten ab) + Vorsteuerung atan(a_soll/g); 0015 dort aus.
  Zur Laufzeit `!velctl KI FF IMAX`. Gebaut (SHA 49578aae…, 1 265 568 Byte), alle Host-Tests grün. Im einfachen Modell
  (nicht der Rider): Last 2° → Original 17,5 mm/s Restfehler, mit Integral 0,3 mm/s; Vorsteuerung halbiert den
  Schleppfehler. ki und Vorsteuerung am Rider einstellen (Stufe 9). Nicht geflasht.
- **Stufe 4 (2026-11-25):** Patch `0032-turn-rate-control.patch`: Kursziel läuft genau mit w_soll·dt (statt
  0,005 je Takt = 1,25 × w), Kurs-PID wirkt damit als PI auf die Drehrate, Kurshalten bei w = 0 bleibt; langsames
  Drehen zählt ab 0,05 °/s als Bewegung. Zur Laufzeit `!turnctl KP KD FF`. Gebaut (SHA ca488061…, 1 266 192 Byte),
  alle Host-Tests grün. Im Modell (nicht der Rider): 30 °/s → 30,00 (Original 37,5); 0,5 °/s mit Haftreibung
  6,8° in 20 s, Nachlauf 3,2°. Nicht geflasht.
- **Stufe 7 (2026-11-25):** Treiber `/proc/XGORider/vel` (schreiben `V A J W ALPHA JW` oder `acc A J W ALPHA JW`,
  lesen = Zustand 0x1a) und `vel_limits` (`V_MAX W_MAX TIMEOUT_MS`), Text ↔ Bytes in `kernel_module/xgo-vel.h`
  (gemeinsam mit dem Treibertest); `vel` startet den Kernel-Wächter neu. `driver-test/run.sh … --vel` auf kimmy grün
  (auch mit `--faults --stream --tunnel`), Modul baut dort. Am Rider eingespielt 2026-11-25 12:04 (Neustart).
- **Geflasht 2026-11-25:** Firmware 0001–0032 (SHA ca488061…), liegend; 0x1a antwortet. Rahmenweg noch nicht benutzt.
- **Stufe 8 (2026-11-25):** xgonav `fahrrahmen.py` + `Robot`: `drive(v, w)` bleibt normiert; mit Treiber `vel` und
  Firmware ab 0029 schreibt Robot Rahmen (Ziel m/s = v × `vx_limit`, °/s = w × `vyaw_limit_dps`, an die Firmware
  ÷ `fw_speed_unit_mm`, gemessen 3,9) mit a/Ruck (`vel_accel` 0,5 / `vel_decel` 1,0 m/s², Ruck 2,5; Drehen 360 °/s², 1800 °/s³;
  Bremse 1,5 m/s²), `vel_limits` beim Start und alle 2 s; Rampe am Pi, `speed_ctrl` und `turn_min_w` in `drive()` aus.
  Python-Generator = Firmware-Generator, `robot.command` ist deren Sollwert (Simulator, `v_erw`). Simulator fährt
  standardmäßig den Rahmenweg (Nachlauf 0,15 / 0,1 s angenommen). Unit-Tests 569 grün (Bytewege-Tests auf
  `vel_frame = False`, neu `tests/test_fahrrahmen.py`). Am Rider eingespielt 11:27: dort noch Bytes (Treiber ohne `vel`).
  Treiber 12:04 eingespielt: xgonav fährt jetzt den Rahmenweg (Log „Fahrrahmen aktiv … 3.90 mm/s je Einheit“,
  `vel_limits` 231 180.0 500), gefahren (Messfahrt Rahmenweg oben). Offen: Überschwingen; `turn_min_w` in `behaviors.dreh_w`, `speed_ctrl` und Pi-Rampe
  nach der Messfahrt entfernen.

- **Messfahrt 2026-11-25** (Byteweg, `tools/fw_einheit.py`, MCP geprüft geradeaus): v 0,2 / 0,3 je 3,5 s,
  Encoder 395 / 683 mm (= Kartenstrecke 0,40 / 0,68 m). Stationär 152 mm/s bei Firmware-Ist 38,4, Soll 27,5;
  246 mm/s bei Ist 63,1, Soll 37,1. **Firmware-Einheit = 3,9 mm/s** (3,95 / 3,90; Auswertung Median 3,82,
  p10–p90 3,66–3,93, gemittelt über Anfahren). Firmware-Ist/Soll 1,4–1,7: der Byteweg überfährt sein Soll, die
  8,4 war der Faktor zum Soll, nicht zur Einheit. `fw_speed_unit_mm` = 3,9; Höchsttempo des Rahmens damit
  210 × 3,9 ≈ 0,82 m/s (v_max aus `vx_limit` × `max_v` = 0,9 m/s wird in der Firmware nicht geklemmt, fährt aber
  bei normierten Werten ≤ 0,5 nie dorthin).
- **Messfahrt Rahmenweg 2026-11-25** (Treiber `vel`, MCP geprüft geradeaus, `fw_rahmen.csv`): Soll kommt richtig an
  (v 0,2 → 46 Einheiten = 0,18 m/s, v 0,3 → 69 = 0,27 m/s, Rampe 0,4 s), aber das Ist schießt über: 46 → Spitze 83
  (≈ 0,32 m/s) nach 1,8 s, am Ende (3,5 s) 50; 69 → Spitze 132 (≈ 0,51 m/s) nach 2 s, am Ende 72. Strecke 0,85 /
  1,28 m statt ~0,6 / 0,9 m. Anhalten sauber. Deutung: der Bestandsregler fährt von sich aus 1,4–1,9 × Soll (wie im
  Byteweg), das Integral (ki 0,05, i_max 4°) holt das erst in ~2 s zurück. Dazu Einheitenfehler in 0031: die
  Vorsteuerung rechnet atan(a/9810 mm/s²) mit a in Firmware-Einheiten/s², also 3,9-mal zu klein (0,7° statt 2,9°).
- **Firmware 0035 (2026-11-25, Nutzer: Einheitenfehler beheben und b)):** 0x1b mit 10 Byte bringt Einheit (µm/s je
  Einheit) und Vorfaktor c; Vorsteuerung in echten mm/s², Speed-P und Radkorrektur des Originals sehen Soll × c
  (`robot.vel_stock_scale` 0,55 ≈ 1/1,85), das Integral das volle Soll. Treiber `vel_limits` mit 5 Feldern
  ("231 180.0 500 3900 550"). Host-Tests und Treibertest grün.
- **Geflasht 2026-11-25 ~12:26:** 0001–0035 (SHA a4d09455…, 1 267 872 Byte, `out/app0_0035.bin`), Treiber und
  xgonav eingespielt, Firmware nimmt die 10-Byte-Grenzen an. **Messfahrt (c 0,55):** v 0,2: Soll 46, Ist nach 0,5 s
  ~40, Spitze 60 (1,30 × statt 1,80 ×) nach ~2,5 s, am Ende 52; v 0,3: Soll 69, Spitze 90 (1,30 × statt 1,91 ×)
  nach ~2 s, am Ende ~80. Strecke 0,70 / 1,09 m (vorher 0,85 / 1,28). Schneller am Soll, Überschwingen auf ein
  Drittel; der Rest steigt langsam an (Bestandsweg) und das Integral holt ihn zu langsam zurück.
- **Messfahrt c 0,42 (2026-11-25, Flur zurück):** schlechter. v 0,2: 90 % erst nach 2,1 s (c 0,55: 0,9 s), dann
  stetiger Anstieg bis 64 (1,40 ×) bei 3,7 s, kein Rückgang; v 0,3: 90 % nach 1,8 s, Spitze 93 (1,35 ×) bei 3,3 s,
  Ende 1,23 ×. Strecke 0,57 / 0,96 m. Deutung: kein Fehler des Bestandswegs allein, sondern Windup des Integrals
  (0031): solange das Ist beim Anfahren hinterherläuft (Gegenschwung des Balancierens), läuft vel_i bis i_max 4° voll
  und treibt danach über das Soll; je kleiner c, desto länger der Nachlauf, desto mehr Windup. Vorschlag: Integral
  nur bei ruhendem Generator (a = 0) bzw. kleinerem Fehler, c zurück auf 0,55.
- **Firmware 0036 (Nutzer: so vorgehen):** Integral erst, wenn der Generator 0,6 s lang a = 0 hat; c wieder 0,55.
  Host-Tests grün, Build SHA 8b3e94e2… (`out/app0_0036.bin`).
- **Geflasht 0001–0036 (2026-11-25), Messfahrt c 0,55:** v 0,2: 90 % nach 1,35 s, dann Ist stetig 38 → 55 bis 3,5 s
  (Spitze 1,26 × bei 3,7 s, Ende 1,13 ×), Strecke 0,68 m. v 0,3 nach 2,4 s von der Bahnprüfung gebremst: bis dahin
  Ist 56 → 78 (1,13 ×). Die frühe Spitze ist weg, aber der Bestandsweg kriecht weiter nach oben, auch während das
  Integral läuft: ki 0,05 (Fehler −8 → −0,4°/s) ist dafür zu schwach. Windup war nur ein Teil.
- **Firmware 0037 (Nutzer: Weg 2):** ki und i_max gehen mit 0x1b (14 Byte) von xgonav an die Firmware
  (`robot.vel_ki` 0,15, `vel_i_max_deg` 4). Treiber `vel_limits` mit 7 Feldern, Textpuffer 48 Byte. Host-Tests und
  Treibertest grün, Build SHA aa2260dd… (`out/app0_0037.bin`).
- **Geflasht 0001–0037 (2026-11-25), Messfahrt c 0,55, ki 0,15:** Strecke 0,61 / 0,89 m (≈ Soll-Strecke). Verlauf
  v 0,2 (Soll 46): nach der Rampe 1 s Plateau bei 26–28 (≈ Soll × c, Integral wartet 0,6 s), dann zieht das Integral
  hoch auf 63 (1,37 ×) bei 2,5 s und wieder herunter auf 46–40: langsame Schwingung, Periode ~3 s. v 0,3 (Soll 69):
  Plateau 48, Spitze 86 (1,25 ×), Ende 66–70. Mittel der letzten Sekunde 1,14–1,16 ×. Deutung: ki 0,15 zu groß für
  den Nachlauf von ~1 s; das Plateau zeigt, dass der Bestandsweg hier etwa Soll × c fährt (c 0,55 zu klein).
- **Messfahrt c 0,8, ki 0,08 (2026-11-25, Flur zurück):** Strecke 0,61 / 0,92 m. v 0,2: wieder 1 s Plateau bei
  25–26 (nicht 46 × 0,8 = 37!), dann Anstieg bis 66 (1,42 ×) bei 3,1 s, letzte Sekunde 1,33 ×; v 0,3: 90 % nach
  1,45 s, Spitze 83 (1,20 ×), Ende 1,15 ×. Damit widerlegt: das Plateau hängt nicht an c, und der Anstieg ist mit
  halbem ki fast gleich steil und läuft über das Soll hinaus weiter, ist also nicht (nur) das Integral. Neue
  Vermutung: beim Fahren ein Nick-Versatz von ~2° nach vorn (P 0,11 °/Einheit hält 15–20 Einheiten Überschuss),
  der den Rider stetig schneller macht. Prüfen: Nick (IMU) und Soll-Nick beim Fahren mitschreiben.
- **Messfahrt mit Nick (2026-11-25, `tools/fw_einheit.py` schreibt jetzt Soll-/Ist-Nick aus 0x0e, Balance,
  Positionshalten und IMU-Nick mit; c 0,8, ki 0,08, gegenläufig gefahren):** Versatz bestätigt, aber anders als
  vermutet. Der Nick-Regler (P 37,5) erreicht seinen Soll-Nick beim Fahren nicht: Soll-Nick geht auf −2,9° (v 0,2)
  bzw. −3,9° (v 0,3), der gemessene bleibt 3–4° weiter vorn und nähert sich nur langsam. Der Balance-Ausgang wächst
  mit dem Tempo (−90 bei 46, −145…−185 bei 73–84): die Räder bekommen ein Tempo-artiges Kommando, und um es zu
  halten, braucht es eine Nick-Abweichung proportional zum Tempo. Am Ende beider Abschnitte nahe am Soll:
  Soll-Nick / Ist-Tempo = −2,2° / 46 und −3,3° / 73, also ~−0,05° je Firmware-Einheit. Diesen Anteil muss bisher
  das Integral lernen (dauert, schwingt) und i_max 4° begrenzt ihn auf ~80 Einheiten ≈ 0,31 m/s. Vorschlag:
  Tempo-Vorsteuerung des Soll-Nicks −kv · v_soll (kv ≈ 0,05 °/Einheit), Integral nur für den Rest.
- **Firmware 0038 (Nutzer: so vorgehen):** −kv·v_soll im Soll-Nick, kv über 0x1b (16 Byte); xgonav `vel_kv` 0,05,
  c wieder 1,0, i_max 6°, ki 0,08. Treiber `vel_limits` mit 8 Feldern.
- **Geflasht 0001–0038 (2026-11-25, SHA 1d01f150…), Messfahrt kv 0,05, c 1, ki 0,08:** schlechter. Erster Versuch
  von der Bahnprüfung (Kabel) nach 13 cm gebremst. v 0,2: nach der Rampe fällt das Ist von 22 auf 10 (Soll 46),
  90 % erst nach 2,4 s, dann Spitze 69 (1,50 ×), letzte Sekunde 1,37 ×; v 0,3: Ist fällt von 40 auf 26, 90 % nach
  2,5 s, Spitze 94 (1,37 ×). Ablauf aus Soll-Nick nachgerechnet: am Rampenende fallen atan(a/g) (≈ +2,9°) weg und
  −kv·v_soll (−2,3° / −3,5°) setzt voll ein, solange das Ist noch weit unter dem Soll ist -> er bremst; das Integral
  lädt sich in dieser Delle auf (+2,1° bei 1,8 s) und treibt danach über das Soll. Erklärt auch den Bestandsweg:
  Gleichgewicht 0,11·(s − v) = −0,05·v ergibt v = 1,83 · s (gemessen 1,8–1,9 ×).
- **Firmware 0039 (Nutzer: so vorgehen):** kv auf das gemessene Tempo (−kv·v_ist): gleicht genau den Bedarf aus,
  ohne Integral rechnerisch v = s, keine Delle; ki 0,02 nur für den Rest.
- **Geflasht 0001–0039 (2026-11-25, SHA cdb6fdb3…), Messfahrt kv 0,05 auf Ist, ki 0,02:** erstmals kein
  Überschwingen, ruhig, Soll-Nick ±0,5° statt Schwingen. Aber zu langsam: v 0,2 hält 30–34 statt 46 (letzte Sekunde
  0,79 ×), v 0,3 hält 48–56 statt 69 (0,81 ×), das Integral holt mit 0,02 kaum auf. Strecke 0,47 / 0,76 m.
  Gleichgewicht nachgerechnet (Soll-Nick ≈ 0 bei v_ist): kv ist zu groß, der Bedarf liegt eher bei 0,02–0,025° je
  Einheit (die −0,05 aus der Nick-Messfahrt enthielten das Integral). Nächster Schritt: kv 0,025, ki 0,03.
- **Messfahrt kv 0,025, ki 0,03 (2026-11-25, gegenläufig):** erstmals brauchbar. v 0,2 (Soll 46): 90 % nach 1,6 s,
  Spitze 48 (1,03 ×), letzte Sekunde 0,97 ×, Strecke 0,58 m. v 0,3 (Soll 69): 90 % nach 0,65 s, hält 72–74
  (1,05 ×), kriecht zum Ende auf 80 (letzte Sekunde 1,12 ×), Strecke 1,0 m. Soll-Nick beim Halten −0,5° bei 46,
  −2,5…−3,5° bei 73–80: der Bedarf wächst stärker als linear (bei 46 ~0,01, bei 75 ~0,035° je Einheit). Offen:
  höhere Tempi messen, kv ggf. tempoabhängig; Drehen auf dem Rahmenweg noch nicht gemessen.
- **Messfahrt v 0,4 / 0,5 (2026-11-25, kv 0,025, ki 0,03, ungeprüft über MCP, da geprüft nur bis 0,30;
  gegenläufig, 2,5 / 2 s):** v 0,4 (Soll 92 = 0,36 m/s): 90 % nach 1,8 s, Spitze 93 (1,01 ×), letzte 0,7 s 0,96 ×,
  Soll-Nick −1,5°, Strecke 0,79 m. v 0,5 (Soll 115 = 0,45 m/s): 90 % nach 1,0 s, Spitze 134 (1,16 ×), letzte 0,7 s
  1,11 ×, Soll-Nick −4,7°, Strecke 0,86 m. Beim Bremsen aus v 0,5 Soll-Nick bis −15°, ohne Sturz. Die
  Halteabschnitte sind kurz (1–1,7 s), also nicht eingeschwungen. Bedarf je Einheit uneinheitlich (0,017 bei 88,
  0,037 bei 128); v 0,3 und 0,5 kriechen etwas über, v 0,2 und 0,4 nicht.

### Zu klären


### Entschieden (Nutzer 2026-11-25)

- Höchste Beschleunigungen (`a_max`, `alpha_max`) werden in xgonav konfiguriert (Dataclass in `config.py`) und mit
  jedem Rahmen gesendet. (Vorschlag, nicht entschieden: Firmware klemmt zusätzlich auf eine feste Obergrenze.)
- ~~Die Radgeschwindigkeit der Firmware ist echte mm/s~~ (2026-11-25 widerlegt, Nutzer: so vorgehen): Messung
  2026-11-05 Befehl 0,15 → ~27 cm/s echt, Firmware-Soll dafür 32,1 Einheiten (0x0e zeigte am 2026-11-10 genau 32,1),
  also ~8,4 mm/s je Firmware-Einheit; 210 × 8,4 ≈ 1,8 m/s = `vx_limit`. xgonav bekommt `robot.fw_speed_unit_mm`
  (gemessen 3,9, siehe Messfahrt) und rechnet alles, was an die Firmware geht (v, a, Ruck, v_max), damit um. Faktor messen:
  `tools/fw_einheit.py` (0x0e + Rad-Odometrie mitschreiben, gefahren über MCP).
- Höchstwerte (größtes `a_max` ohne Kippen, größte Drehrate und Drehbeschleunigung) werden am Rider gemessen
  (Stufe 9), danach in `config.py` eingetragen.
- Register 0xa0/0xa1 (`max_forward`/`max_turn`, Bereich der Bytes 0x30/0x32) kommen raus. Der alte Byteweg
  rechnet fest mit 210 mm/s / 300 °/s (`automatic_lean` ist seit 0033 entfernt); die Konsolenbefehle `!maxforward`/`!maxturn`
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
