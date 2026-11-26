# Arbeitsplan Architektur

Ist-Stand und nächste Schritte. Verworfene Firmware-Ansätze: `firmware/README.md` (Holzwege).

## Zielbild

Kamera + FloorNet + IMU schreiben Merkkarte und Raumkarte. Das Gehirn (ein Modus) äußert einen Wunsch an den
Planer, der Planer fährt den Rider, nach der Sicherheitsprüfung.

## Plan

| # | Punkt | Stand |
|---|---|---|
| 1 | **Nur ein Gehirn mit Modus, jeder Weg läuft über den Planer.** Bei Dunkelangst wird der schnellste Weg zurück gefahren | offen (Befund unten) |
| 2 | **Test: Anfahren an eine Wand** | geradeaus geprüft gefahren 2026-11-25 (unten) |
| 3 | **Test: Fahren einer Bahn entlang.** Kurven glätten, Drehungen auf der Stelle vermeiden (Hauptgrund der Drift) | offen |
| 4 | **Fahrregler in die Firmware.** Pi sendet Sollgeschwindigkeit und Drehrate in echten Einheiten, die Firmware regelt | erledigt, gefahren (unten) |
| 5 | **Bahnbefehle für die Navigation.** Krümmung statt Drehrate, Drehen auf Kurs, Strecke mit Halt, Rückmeldung im Strom (Plan unten) | Stufe 0 fertig. Stufe 1: gefahren 2026-11-25 (κ 2, Firmware hält κ_soll exakt; Anfang enger, Halt jetzt auf dem Bogen), `vel_kappa` aus. Stufe 2: gefahren 2026-11-25 mit 81cb3335 (±90°/±180°, Sollwinkel auf 0,4°, IMU-Kurs 2 s nach Ende auf ≤ 1,4°). `vel_kappa`/`vel_kurs` seit 2026-11-25 Standard an. Stufe 3: gefahren 2026-11-25 von Hand, mit Firmware 33c685ea auf ≤ 3,3 cm; `vel_strecke` Standard an, genutzt von Handsteuerung und home am Schluss. home mit Krümmung + Kursziel gefahren: am Start 1 cm / 0,9° |

## Bahnbefehle (Punkt 5, Plan 2026-11-25)

Grundsatz: Karte, Planer, Bahnprüfung und `safety()` bleiben am Pi (Weg ändert sich mit jeder Beobachtung, der
ESP32 kennt keine Karte). Die Firmware bekommt nur Befehle, die eine **Bahn** statt einer Geschwindigkeit
beschreiben, und hält sie mit 260 Hz. Keine Wegpunktlisten in der Firmware.

### Befunde, die den Plan begründen

| Stelle | Befund |
|---|---|
| `velocity_cmd.h tick` / `Robot._ramp_vel` | v und w rampen getrennt (a/Ruck bzw. alpha/Drehruck). Beim Anfahren und Bremsen erreicht w sein Ziel früher als v: engerer Bogen als geplant, aus dem Stand kurz Drehen auf der Stelle |
| `App._arc_check` | Bremst die Bahnprüfung, senkt sie nur v (`cmd.v *= k` bzw. auf `erlaubt`), w bleibt: der gefahrene Bogen wird enger als der geprüfte (`arc_clearance(vl, wr)`) |
| `PathFollower.step` | Lenkt mit `w = kp_turn · err` (Drehrate), Kurventempo getrennt über `kurve_v`; Krümmung des Wegs liegt ohnehin vor (`kurve_v`, geglättete Bögen aus Punkt 3) |
| `dreh_auf_kurs`, `align`, `POST /turn` | Kursregelung am Pi mit 24 Hz über `dreh_w`; Überschwingen der Rate 1,2–1,4 × für 0,25 s (Punkt 4). Die Firmware führt ihr Kursziel schon mit w·dt gegen den IMU-Kurs |
| Wand (Punkt 2) | Bahnprüfung bremst voll, danach Kriechen mit `slow_free` bis an die Grenze; Haltepunkt nur auf ~2 cm genau |
| `fahrrahmen.Achse` | Pi rechnet den Sollwertgenerator der Firmware nach (Simulator, Bahnprüfung): zwei Stände derselben Logik |
| Rückwärts | 1,29–1,37 × Soll (vorwärts 1,03–1,07 ×): stimmt nicht, gilt κ = w/v rückwärts nicht |

### Stufen (je Stufe ein Thema, eine Fahrt; nächste erst nach Fahrt)

| Stufe | Inhalt | Firmware (Patch 08-velocity) | Treiber (`xgo-vel.h`) | xgonav | Kosten am Pi | Prüfung |
|---|---|---|---|---|---|---|
| 0 | Rückwärts symmetrisch, Höchstwerte | geflasht 2026-11-25: Tempo-Regler über vier Fassungen (unten „Messfahrt Stufe 0“), jetzt ec493c66: Original-P 0,11 + Vorsteuerung −0,05·v_soll + atan(a/g), ki 0,1 auf ±40 ab Ende der Rampe; ki/i_max/kv aus 0x1b ignoriert | – | `vel_kv`/`vel_ki`/`vel_i_max_deg` als wirkungslos gekennzeichnet (`config.py`, `docs/configuration.md`); a_max ohne Kippen, w/alpha-Grenzen messen → `config.py` | 0 | fertig 2026-11-25: Firmware 5abeb9a6 (kp 0,15 / ki 0,06 fest) geflasht, v ±0,2 und ±0,3 gefahren, Spitze 1,13–1,17 ×, rückwärts 1,00 × (unten) |
| 1 | **Krümmungsbetrieb** | Flag bit1 in 0x1a: Feld w = κ in 0,001 1/m (i16, ±32 1/m); im Takt w_soll = κ · v_soll · 180/π (aus dem **Sollwert**, nicht v_ist: Kurs = ∫κ·v_soll dt, rauschfrei, am Pi nachrechenbar; keine v_min-Schwelle, w → 0 von selbst). Bahntreue unter Grenzen: nicht w kappen, sondern v begrenzen: \|v\| ≤ w_max/\|κ\|, a ≤ alpha/\|κ\|, Ruck ≤ Drehruck/\|κ\|. Neues κ während der Fahrt mit \|dκ/dt\| ≤ alpha/\|v\| nachführen (Klothoide), sonst springt w um Δκ·v. Rückwärts gleiches κ (derselbe Kreis rückwärts). Positionshalten beim Drehen (07-balance) greift nur bei v = 0, verträglich. Ohne bit1 wie heute | Text um κ erweitern, `zustand` liest Flag und Fähigkeitsbyte | `Robot.drive(v, w)` bleibt; neu `drive_bogen(v, kappa)`. `PathFollower` lenkt mit κ (Pure Pursuit κ = 2·sin α / L statt `kp_turn·err`); `_arc_check` prüft und bremst bei festem κ (nur v sinkt, Bahn bleibt). `Achse.schritt` + Kopplung, v/a/Ruck-Begrenzung durch κ und κ-Nachführung in Python nachgebaut, gegen Prüfvektoren | sinkt leicht (kein Lenkregler je Schritt, eine Multiplikation im 50-Hz-Durchlauf) | Host-Test: Bahn aus Generator-Simulation liegt bei Anfahren/Bremsen auf dem Kreis (Abweichung < 1 cm auf 1 m); Simulator `--dyn` explore/home; am Rider Kreisbogen v 0,2, κ 2 1/m, Endpunkt gegen Karte |
| 2 | **Kursziel** | Flag bit2: Feld w = Δψ in 0,1°, relativ zum Generatorwinkel bei Empfang. Der Generator integriert seinen Winkel ∫w_soll dt selbst (= Änderung des Kursziels): Rest → w_ziel = sign · min(w_max, √(2·alpha·\|Rest\|)) → vorhandenes `approach` der Drehachse; angekommen bei \|Rest\| < 0,5° und w = 0 (Flag), danach hält der Kurs-PID | wie 1 | `dreh_auf_kurs`/`align`/`POST /turn` schreiben einmal Δψ statt 24 Hz `dreh_w`; Ankunft aus dem Flag statt aus dem Kursfehler | sinkt (keine Kursregelung je Schritt) | Host-Test Überschwingen < 1°; am Rider 90°/180° links/rechts, Endkurs gegen IMU-Kurs ±2° |
| 3 | **Strecke mit Halt** | Flag bit3, Rahmen 15 Byte mit Reststrecke s (mm, u16): Rest = s − Radstrecke (Encoder) seit Empfang, v_ziel = sign · min(v, √(2·a·Rest)), danach v = 0 und Positionshalten. Neue Rahmen setzen s neu (Pi schickt die Reststrecke nach) | wie 1 | Wegfolger am Wegende (`end_v`) und Bahnprüfung schicken s = min(Weg bis Ziel, `clear − Rest`) statt das Tempo zu senken; Kriechen mit `slow_free` entfällt | sinkt | Wand wie Punkt 2: Halt 20 cm ±1 cm aus v 0,2/0,3; Host-Test Halt auf ±5 mm |
| 2b | **Absolutes Kursziel** (neben Stufe 2, Nutzer 2026-11-26: Rahmen „etwas overengineered“, Stufe 2 bleibt Standard) | Flag bit5 allein, 5 Byte: Wunschkurs ψ_ziel im Kurs der Firmware + w_max; Rest je Takt = wrap180(ψ_ziel − ψ_ref), ψ_ref = Kursziel des Kurs-PID; jeder Rahmen setzt nur das Ziel; alpha/Drehruck aus 0x1b (18 Byte) | `kursabs PSI W`, Grenzen mit 9 Feldern, Zustand 15 Felder | `robot.vel_kursabs` (aus): Ziel aus dem IMU-Kurs zur Bildzeit, höchstens 170° voraus; Wechselbit, Neuzielen und w_max über `vel_limits` entfallen | sinkt (Logik weg, ein Rahmen je Durchlauf) | Host-Tests + 16 `kursabs_*`-Vektoren; am Rider wie Stufe 2 (±90°/±180°), dazu explore gegen den Stand mit Stufe 2 |
| 4 | **Rückmeldung** | Eigener Stromtyp **0x16** (0x14 bleibt unverändert), nur bei aktivem Rahmenweg, gleiche Rate und gleiches t_us wie die IMU-Probe, 12 Byte: u16 seq, u32 t_us, i16 v_soll, i16 w_soll (0,1 °/s), u8 Flags (Kurs erreicht, Strecke erreicht, Wächter, Sättigung = Soll-Nick an der Grenze), u8 reserviert; ~2 kB/s bei 100 Hz | Stromtyp 0x16 lesen, Felder in `/proc/XGORider/vel` | `Achse`-Nachbau nur noch für Simulator; Bahnprüfung und Anzeige nehmen den echten Sollwert; Sättigung senkt `kurve_a_mps2`/Tempo für den Rest der Fahrt | sinkt | Vergleich Nachbau ↔ Strom an einer Fahrt |

Stand Stufe 1 (2026-11-25): Firmware 6194db7 geflasht. Treiber `kap V A J K ALPHA JW` und Zustand mit `kappa caps`
(driver-test `--vel` grün auf kimmy; am Rider gebaut und geladen 2026-11-25 17:10, meldet caps 1). xgonav: `fahrrahmen.bogen_schritt` gleich allen 8
Vektoren (`test_firmware_vektoren`), `Robot.drive(kappa=)`, `robot.bogen` nur mit Fähigkeitsbit und `vel_kappa`;
Wegfolger setzt `Command.kappa` = Drehrate/Tempo (gleiche Lenkung wie bisher), `app.bogen_w` hält w = κ·v, wenn
safety()/Bahnprüfung v senken. Simulator (je ein Lauf, `--dyn`): home 60 s beide am Start, explore 200 s `--rider`
beide Karte fertig ohne Kollision (κ: 1 Stelle nicht erreicht, Posefehler 0,7 statt 2,7 cm).

Stand Stufe 2 (2026-11-25): Firmware ad2f52e geflasht (caps 3). Treiber `kurs S V A J D ALPHA JW`, Zustand mit
`kurs erreicht` (13 Felder; driver-test `--vel` grün auf kimmy). xgonav: `fahrrahmen.kurs_schritt`/`kurs_empfang`
gegen die 10 `kurs_*`-Vektoren (float gegen double: Vorausschau kippt an der Grenze einen Takt verschieden, höchstens
0,61 °/s, Rest 0,05°, „angekommen“ ±2 Takte, Endrest gleich). `Robot.drive(kurs=(Fehler, w))`: neues Ziel mit
Wechselbit nur bei > 2° Verschiebung (oder > 1° Rest nach Ankunft), w-Obergrenze des Modus über `vel_limits`.
Befehle mit `Command.kurs`: Wegfolger (Drehen, Ausrichten) und `POST /turn`; Dunkelangst und pet noch über `dreh_w`.
Simulator (je ein Lauf, `--dyn`): home 60 s mit Kursziel und mit Kursziel + Krümmung am Start (4–5 cm), explore 200 s
`--rider` beide an: Karte fertig in 110 s, keine Kollision, 1 Stelle nicht erreicht.

Stand Stufe 3 (2026-11-25): Firmware 335401c (ede94cd8) geflasht. Treiber `dist V A J W ALPHA JW S` und
`kapdist V A J K ALPHA JW S` (15 Byte, `vel_encode` liefert die Länge), Zustand 14 Felder mit `strecke` (driver-test
`--vel` grün auf kimmy). xgonav: `fahrrahmen.Strecke`/`strecke_empfang`/`strecke_messen`/`strecke_schritt` gegen alle 26
Vektoren (Rest und Nachlauf auf 0,01 mm); `Robot.drive(strecke=Rest m)` schickt den Rest auf 5 mm gerundet, Nachbau mit
v_meas = Sollwert (ohne Nachlauf); Handsteuerung `/drive?strecke=M` bzw. MCP `fahren` mit `strecke`, Reststrecke aus
`odom_m` ab dem ersten Regelschritt. Wegfolger und Bahnprüfung schicken noch keine Strecke. Außerdem: xgonav fragt alle
5 s nach, wenn der Fahrrahmen beim Start fehlte (Flashen), und schaltet dann um.

Stand Stufe 2b (2026-11-26): Firmware c7a97230 gebaut, alle Host-Tests grün, **nicht geflasht** (alte 28 Vektoren
bytegleich). Treiber `kursabs PSI W` (5 Byte), `vel_limits` mit 9 Feldern (18 Byte), Zustand 15 Felder (`abs_grenzen`:
Treiber hat zuletzt 18 Byte geschrieben); driver-test `--vel` und `--faults` grün auf kimmy; am Rider nicht gebaut.
xgonav: `fahrrahmen.kursabs_schritt` gleich allen 16 `kursabs_*`-Vektoren (Ankunft am Ende in float bis 6 Takte
später: Rest-w ~4e-5 °/s); `Robot` mit `vel_kursabs` und Fähigkeitsbit bit4: vel_limits mit alpha/Drehruck, je
Durchlauf `kursabs`, Nachbau folgt dem festen Ziel (Versatz Firmware-Kurs → Generatorwinkel einmal beim Start); ohne
IMU-Strom das Kursziel Δψ. `App._kurs_fw`: Ziel = IMU-Kurs zur Bildzeit des Regelschritts + Restfehler (±170°), über
`ImuSource.fw_kurs_deg` (Kursfaktoren zurückgerechnet) in den Rohkurs. Simulator home 60 s `--wheels` mit/ohne:
1–1,4 cm / ±1–1,4° gegen 1,8 cm / 0,1° (Hysterese 1° der Firmware). Nicht gefahren.

## Strecke mit Halt und home gefahren (2026-11-25)

Strecke von Hand (`fahrt.py` am Rider = MCP `fahren` mit `strecke`, `vel_strecke` nur für den Test; Mitschnitt
`runde9.csv`). Wahre Strecke aus den Radticks (π·60 mm/1024 × 1,008, am Maßband kalibriert), Pose stimmt damit überein.

| Fahrt | verlangt | gefahren (Ticks) | Ursache |
|---|---|---|---|
| v 0,2 vorwärts | 1,0 m | 0,93 m | Pi rechnete mit `odom_m` (Betrag, zählt das Pendeln beim Balancieren mit: 0,99 m) → behoben 044c0d7, vorzeichenbehaftet `odom_s_m` |
| v −0,2 rückwärts | 0,5 m | 0,63 m | Pendeln der Odometrie machte den Rest wieder größer, die Firmware fuhr in 4 Stößen wieder an → behoben e25e0ad (Rest nur fallend) |
| v 0,3 vorwärts | 1,0 m | 0,95 m | 5–6 cm zu früh |
| v −0,2 rückwärts (nach Fix) | 0,5 m | 0,59 m | 9 cm zu weit, kein Nachschub mehr |

- Firmware-Fehler: Der Nachlauf lag = ∫(v_soll − v_meas) summierte eine dauerhafte Tempoabweichung auf (vorwärts
  Anlauf zu langsam → zu früh; rückwärts schneller als Soll → zu spät). Fix 73f08ad (SHA 33c685ea): lag = 0,25 s × v_ist.

Nachfahrt mit 33c685ea (`runde10.csv`):

| Fahrt | verlangt | gefahren (Radticks) | Pose |
|---|---|---|---|
| v 0,3 vorwärts | 1,0 m | 1,033 m (+3,3 cm) | 1,032 m |
| v −0,2 rückwärts | 0,5 m | 0,520 m (+2,0 cm) | 0,521 m |
| v 0,2 vorwärts | 1,0 m | 0,998 m (−0,2 cm) | 1,007 m |

- Stufe 3 damit von Hand auf ≤ 3,3 cm; ohne Bremsung, kein Nachschub.

home mit `vel_kappa` + `vel_kurs` (Standard an), Startbild an, Weg ~1 m um eine Ecke:

| Lauf | Ergebnis |
|---|---|
| 1 | am Start 1 cm, aber 180° falsch: Auf der Geraden zum Start 5 cm überrollt, zum Nachfahren umgedreht; die Schlussausrichtung brach sofort ab, weil das Vorzeichen des Fehlers um ±180° durch Zittern kippte („übergedreht“). Behoben 13237ae (übergedreht nur bei Fehler < 90°) |
| 2 | am Start 1 cm, Kurs 0,9° daneben; auf der Geraden 8,7 cm überrollt, daher zweimal gedreht (nachfahren, ausrichten) |

- Umgesetzt 2026-11-25: Die Schlussfahrt zum Start trägt die Reststrecke (Strecke mit Halt, `vel_strecke` jetzt
  Standard an), steht er still mit ≤ 4 cm Rest, misst er; das Nachfahren nach dem Abgleich fährt rückwärts, wenn der
  Start ≤ 0,3 m hinter ihm liegt (`Home.RUECK_MAX_M`). In `Robot` darf der Rest innerhalb einer Fahrt nur fallen
  (Sprung > 10 cm = neues Ziel). Simulator home 60/90 s: 0,8 / 2,2 cm, −0,2 / −0,5°. Am Rider noch nicht gefahren.

Reihenfolge: 0 → 1 → 2 → 3 → 4. Stufe 1 ist der Kern für Punkt 3 (Bahn entlang), 2 und 3 ersetzen Pi-Regelschleifen,
4 räumt den doppelten Generator weg.

### Schnittstelle (Vertrag, von beiden Seiten gepflegt; abgestimmt mit der Firmware-Sitzung 2026-11-25)

Byte-Vertrag: Kopfkommentar in `firmware/src/velocity_cmd.h` und dieser Abschnitt.

- 0x1a schreiben bleibt 13 Byte, Flags bit0 wie heute; bit1 Krümmung, bit2 Kursziel (bit1 und bit2 schließen sich
  aus); bit3 nur mit 15 Byte (Reststrecke). Unbekannte Bits → Rahmen verwerfen, Zähler in 0x1a lesen.
- **Fähigkeitsbyte (Pflicht):** 0x1a lesen, Byte 1 (heute 0): bit0 Krümmung, bit1 Kursziel, bit2 Strecke, bit3 Strom
  0x16. Die geflashte Firmware prüft in `parseFrame` nur bit0 und führe einen κ-Rahmen als w in 0,1 °/s aus.
  xgonav schaltet `vel_kappa`/`vel_kurs`/`vel_strecke` deshalb nur ein, wenn das Bit gesetzt ist, sonst Warnung
  im Log und Drehratenweg.
- Einheiten: κ 0,001 1/m, Δψ 0,1° (relativ zum Generatorwinkel bei Empfang), s mm (echte mm, Umrechnung über
  `unit_mm` aus 0x1b in der Firmware).
- **Wechselbit (Ergänzung 2026-11-25, Stufe 2):** bit4 in 0x1a schreiben, nur mit bit2. Der Pi schickt Rahmen laufend
  nach (Wächter); ein Kursziel-Rahmen setzt nur dann ein neues Ziel (Rest = Δψ, erreicht = 0), wenn vorher kein
  Kursziel lief oder bit4 anders ist als im letzten Kursziel-Rahmen. Gleiche Rahmen = nur Wächter. Für ein neues Ziel
  bit4 umschalten (auch nach Ankunft). Verworfen: bit1+bit2, bit4 ohne bit2, bit3 (bis Stufe 3), unbekannte Bits.
- Kursziel: Feld v gilt wie im Geschwindigkeitsbetrieb (Drehen im Fahren = Bogen), alpha/Drehruck aus dem Rahmen,
  w_max aus 0x1b. Die Firmware bremst mit Vorausschau (Bremswinkel aus `approach` auf 0), angekommen bei w = 0,
  a = 0, |Rest| < 0,5°, danach hält der Kurs-PID. Wächter: v und w auf 0, Ziel bleibt offen, ein gleicher Rahmen
  setzt fort. Drehraten- oder Krümmungsrahmen beenden das Kursziel; aus der Kurve ins Kursziel ohne Sprung.
- **0x1a lesen, Flags (Ergänzung Stufe 2):** bit4 Krümmung läuft, bit5 Kursziel läuft, bit6 Kursziel erreicht;
  Fähigkeiten 0x03 mit Stufe 2. Prüfvektoren haben seitdem zwei Spalten mehr: `rest` (°) und `done`.
- **Strecke mit Halt (Ergänzung 2026-11-25, Stufe 3, Firmware 335401c / SHA ede94cd8):** 0x1a schreiben mit bit3 =
  15 Byte, Bytes 0–12 wie bisher, Byte 13 u16 Reststrecke s in echten mm (0…65535). Kein Wechselbit: jeder Rahmen setzt
  Rest = s neu, der Pi MUSS die Reststrecke aus seiner Odometrie nachrechnen (gleiches s immer wieder: kommt nie an).
  Verworfen: bit3 mit 13 Byte, 15 Byte ohne bit3, bit3 + bit0, bit3 + bit2; bit3 + bit1 erlaubt (Bogen bis zum
  Haltepunkt, Strecke = Bogenlänge der Encoder).
- Firmware je Takt: Rest −= v_meas · unit_mm · dt · dir (dir = Vorzeichen des Rahmen-v, v_meas = Mittel der gefilterten
  Radtempi). Nachlauf (Änderung 73f08ad / SHA 33c685ea): vf += (v_meas·dir − vf)·(1 − exp(−dt/0,1 s)), lag = 0,25 s ·
  vf · unit_mm (was er nach dem Soll noch fährt; gefahren vorwärts 0,23 s, rückwärts 0,31 s); vf und lag 0 beim
  Einschalten der Strecke, bei Rahmen ohne bit3 und bei cancel; kein Integral, kein Abklingen im Stand mehr (das
  Integral ∫(v_soll − v_meas) machte den Rest zu S − ∫v_soll: vorwärts 5–6 cm zu früh, rückwärts 9 cm zu weit).
  Wirksamer Rest = Rest − lag. Pi-Nachbau in `Robot` ohne lag (der Nachbau rollt nicht nach). Fahrachse: go =
  approach(v_ziel, a, Ruck) (mit κ-Grenzen im Krümmungsbetrieb); fwd = go, solange wirksamer Rest − |go|·dt·unit −
  |stopDistance(go)|·unit ≥ 0, sonst approach auf 0. Erreicht bei wirksamem Rest ≤ 5 mm (auch bei einem Rahmen mit
  s − lag ≤ 5), dann v → 0, Flag bleibt bis zu einem Rahmen mit größerem s. Wächter bremst, Strecke bleibt offen.
- 0x1a lesen: Flag-Byte bit7 = Strecke erreicht; Fähigkeiten 0x07. Prüfvektoren: alle haben vier Spalten mehr
  (`v_meas`, an tick übergeben, außer strecke_* 0; `dist_rest`, `dist_done`, `dist_lag`), `frame_hex` bei strecke_*
  15 Byte; 8 neue `strecke_*.csv` (Pi darin nachgebildet: alle 100 ms s = S − ∫v_meas·unit·dt).
- Alte Rahmen (bit1–3 = 0) verhalten sich exakt wie geflasht; xgonav schaltet Stufe für Stufe um
  (`[robot] vel_kappa`, `vel_kurs`, `vel_strecke`, Default aus bis zur Fahrt).
- Rückmeldung: Stromtyp 0x16 (Stufe 4), 0x14 bleibt unverändert.
- Prüfvektoren: Die Firmware-Host-Tests schreiben bei `XGO_VECTORS=1` (Lauf auf kimmy) für `approach` + Kopplung
  Tabellen (Eingaben, Sollwerte je Takt) nach `firmware/tests/vectors/*.csv`, die CSV werden committet;
  `tests/test_fahrrahmen.py` rechnet sie mit `Achse` nach (ESP32 float, Toleranz 1e-4).

- **Absolutes Kursziel (Ergänzung 2026-11-26, Stufe 2b, Firmware c7a97230, nicht geflasht):** 0x1a schreiben 5 Byte:
  0 u8 Flags = 0x20 (genau so, jedes weitere Bit → verworfen) | 1 i16 ψ_ziel in 0,01° (mod 360 genommen, der Pi schickt
  ±180) | 3 u16 w_max in 0,1 °/s (0 = aus 0x1b; wirksam min(Rahmen, 0x1b)). bit5 mit 13/15 Byte → verworfen.
  0x1b schreiben 14 **oder** 18 Byte: 14 u16 alpha °/s², 16 u16 Drehruck °/s³ (nur fürs absolute Kursziel, 0 = ohne
  Grenze). Jedes 0x1b gilt ganz: nach 14 Byte fehlen alpha/Drehruck, dann wird bit5 verworfen. Ältere Firmware verwirft
  18 Byte ganz (dann gar keine Grenzen): erst Fähigkeit lesen. 0x1a lesen: Fähigkeiten bit4 (kCaps 0x17); Flags bit5
  „Kurs läuft“, bit6 „erreicht“ wie beim Kursziel (nicht unterscheidbar).
- Firmware je Takt (4 ms): ψ_ref = Kursziel des Kurs-PID vor dem Takt (läuft mit w_soll·dt; zu Beginn der Ist-Kurs, beide
  abgewickelt, derselbe Float wie im Strom), Rest = wrap180(ψ_ziel − ψ_ref) ∈ [−180, 180) (kürzerer Weg, genau
  gegenüber rechts herum). Drehachse = Generator des Kursziels (`turnToRest`); geändertes Ziel oder w_max fließend aus
  dem Zustand (Umkehr mit alpha/Ruck, kleineres w_max mit alpha abgebaut; nur das w_max aus 0x1b kappt sofort).
  Fahrachse bremst auf 0 mit a/Ruck des letzten Rahmens mit Fahrachse (sonst 500 mm/s² ohne Ruck). Erreicht bei w = 0,
  a = 0 und |Rest| < 0,5° (auch vor dem Takt geprüft), danach Halte-Integral; wieder offen erst bei |Rest| > 1,0°
  (kCourseReopen: Wackeln ±0,5° bewegt nichts, eine Korrektur muss > 1° sein). Wächter wie beim Kursziel. Relativer
  Kursrahmen nach absolutem: immer neues Δψ; absoluter nach relativem: Neustart; jeder andere Rahmen, Byte, Stopp,
  Aktion beendet ihn. Mehr als 180°: Pi führt das Ziel nach, höchstens 170° vor dem eigenen Kurs.
- Prüfvektoren `kursabs_*.csv` (16): Spalten t_us, rx, frame_hex, v_max, w_max, timeout_ms, unit_mm, abs_alpha,
  abs_jerk, psi_ref (an tick übergeben; nach jedem Takt ref += w·0,004), v, w, a, timed_out, rest, done, course_on.

### Zusammenarbeit

| Wer | Besitzt | Liefert |
|---|---|---|
| Firmware-Sitzung | `firmware/` (Patch 08-velocity, Host-Tests, README) | Rahmen-Erweiterung, Host-Tests, Prüfvektoren, Build; Nutzer flasht |
| xgonav-Sitzung | `kernel_module/xgo-vel.h` + `driver-test/`, `xgonav/`, `tests/`, `docs/` | Treibertext, `Robot`, Wegfolger, Bahnprüfung, Simulator, Fahrten |

- Dieser Abschnitt ist die gemeinsame Wahrheit; Änderungen am Vertrag nur hier und mit Nachricht an die andere Sitzung.
- Commits auf main, jede Sitzung nur in ihren Pfaden; vor dem Commit `git pull --rebase`.
- Ablauf je Stufe: Vertrag hier festschreiben → Firmware baut + Host-Tests + Vektoren → xgonav + Treiber gegen
  `driver-test` mit erweitertem `esp32_emu.py` → Nutzer flasht → eine Fahrt → Ergebnis hier eintragen.

## Fahrweg (Punkt 4, Stand 2026-11-25)

- Firmware-Patch `08-velocity`: Rahmen 0x1a (v mm/s, a, Ruck, w, alpha, Drehruck), Grenzen 0x1b (v_max, w_max,
  Wächter, Einheit, ki, i_max, kv), Sollwertgenerator mit Ruckbegrenzung, Soll-Nick = Original-P + Integral
  (erst 0,6 s nach der Rampe) + atan(a/g) − kv·v_ist, Kursziel läuft mit w·dt. Einzelheiten `firmware/README.md`.
- Treiber: `/proc/XGORider/vel` und `vel_limits` (7 Felder), Text ↔ Bytes in `kernel_module/xgo-vel.h`.
- xgonav (`xgonav/fahrrahmen.py`, `Robot`): `drive(v, w)` bleibt normiert; mit Treiber und Firmware schreibt Robot
  Rahmen (m/s = v × `vx_limit`, °/s = w × `vyaw_limit_dps`, an die Firmware ÷ `fw_speed_unit_mm` 3,9), `vel_limits`
  beim Start und alle 2 s. `vel_kv`, `vel_ki`, `vel_i_max_deg` gehen mit, sind seit e93e525 aber wirkungslos (Firmware regelt mit festen Werten). Ohne Treiber/Firmware Bytes.
  `speed_ctrl`, `turn_min_w` und die Rampe am Pi sind entfernt.
- Kosten am Pi: gesunken (nur noch Sollwerte, keine Tempo- und Drehregelung je Regelschritt).

Gefahren 2026-11-25 (geprüft geradeaus und auf der Stelle):

| Fahrt | Ergebnis |
|---|---|
| v 0,2 / 0,3 (0,18 / 0,27 m/s) | 90 % nach 0,5–1 s, Spitze 1,08–1,10 ×, letzte Sekunde 1,03–1,07 × |
| v 0,4 / 0,5 (ungeprüft über MCP, Abschnitte nur 1–1,7 s) | 0,96 × / 1,11 ×; Bremsen aus 0,5: Soll-Nick bis −15°, kein Sturz |
| Drehen 36–108 °/s, Kurve v 0,2 + 54 °/s | Rate 0,96–1,01 ×, Winkel auf ±6°; kurzes Überschwingen der Rate nach der Rampe (1,2–1,4 × für 0,25 s) |

Offen: Bedarf an Soll-Nick wächst stärker als linear mit dem Tempo (kv ggf. tempoabhängig); höhere Tempi
eingeschwungen messen; Höchstwerte (a_max ohne Kippen, größte Drehrate/-beschleunigung) messen und in `config.py`.

## Messfahrt Stufe 0 (Punkt 5, 2026-11-25)

Mitschnitt `tools/fw_einheit.py` (0x0e + Encoder, 20 Hz), MCP `fahren` geprüft geradeaus, je 4,5 s, vorwärts und auf
derselben Spur zurück.

| Firmware | Ergebnis |
|---|---|
| e93e525/4aabe9f6 (Tempo nur kp 0,3 · (Soll − Ist), Klemme nur ±100°) | Sturz ~1 s nach dem Aufstehen: Räder im Aufschwung ±190 Einheiten → Soll-Nick 0,3 · 191 = 57°, vornüber. xgonav schickt sofort einen Rahmen v 0, damit war der Regler aktiv |
| f287c1a9 (kp 0,11 voll + Zuschlag bis 0,3 im ±20-Band) | Aufstehen ok (Nutzer über das Web). Fahren: Grenzzyklus ~2,5 Hz, Ist −10 … −87 bei Soll −35, Soll-Nick ±10°, rückwärts 1,2 × Soll; die Bahnprüfung bremste vorwärts an den Ist-Spitzen (230 mm/s). Wirksame Verstärkung ~0,22, stabil nur ≤ 0,135. Abgebrochen nach v ±0,2 (Tempo 75 %) |
| 2a3c9422 (Original-P 0,11, Vorsteuerung −0,05 · v_soll, ki 0,05 auf ±20) | ruhig, Soll-Nick ±2–4°, keine Bremsung; vorwärts zu langsam (Tabelle unten) |
| ec493c66 (wie 2a3c9422, Integral ab Ende der Rampe statt 0,6 s danach, ki 0,1 auf ±40) | im Mittel richtig in beide Richtungen, vorwärts langsames Pendeln (zweite Tabelle) |

2a3c9422, Tempo 100 % (`messfahrt0d.csv`):

| Soll | Ist ab 1 s (Firmware / Encoder) | 90 % nach | Spitze | Soll-Nick Mittel |
|---|---|---|---|---|
| v 0,2 vorwärts (46 Einh., 179 mm/s) | 0,78 × / 0,75 × | 3,1 s | 1,52 × | 0,0° |
| v 0,2 rückwärts | 1,00 × / 1,00 × | 0,5 s | 1,25 × | +2,1° |
| v 0,3 vorwärts (69 Einh., 269 mm/s) | 0,62 × / 0,60 × | 3,6 s | 1,43 × | +0,7° |
| v 0,3 rückwärts (3,5 s) | 0,91 × / 0,90 × | 2,3 s | 1,20 × | +2,3° |

- Rückwärts stimmt jetzt (vorher 1,29–1,37 ×). Schwingen behoben.
- Vorwärts zu langsam: Soll-Nick in der ersten Sekunde **positiv** (+1,3 … +1,9°, wie rückwärts) statt −0,05 · 69 ≈ −3,5°;
  erst das Integral zieht ihn nach ~3 s auf −2°, dann Überschwingen auf 1,4–1,5 ×. Verdacht Vorzeichen der
  Vorsteuerung oder ein Versatz nur vorwärts; an die Firmware-Sitzung gemeldet, offen.
- Aufstehen mit 2a3c9422 nicht mitgeschrieben (Nutzer stand ihn über das Web auf, bevor der Mitschnitt lief), kein Sturz.
- Befund der Firmware-Sitzung: kein Vorzeichenfehler. Am Soll-Nick +1,05° bei Soll 69 / Ist 28 rechnet P (+4,5°) +
  Vorsteuerung (−3,45°) richtig; vorwärts braucht der Rider aber ~1,5° mehr Nick als die Vorsteuerung liefert
  (rückwärts nicht). Das Integral holte das zu spät und zu langsam. Herkunft des einseitigen Versatzes offen.

ec493c66, Tempo 100 % (`messfahrt0e.csv`; erster Vorwärtsversuch nach 0,34 m von der Bahnprüfung gebremst, direkt
nach dem Start des Bodennetzes, wiederholt):

| Soll | Ist ab 1 s (Firmware / Encoder) | Bereich (Einh.) | 90 % nach | Spitze | Soll-Nick |
|---|---|---|---|---|---|
| v 0,2 vorwärts (46) | 1,04 × / 1,03 × | 23 … 64 | 1,6 s | 1,40 × | −3,9 … +1,5° |
| v 0,2 rückwärts | 1,03 × / 1,04 × | −51 … −40 | 0,5 s | 1,12 × | +1,1 … +2,8° |
| v 0,3 vorwärts (69, 4 s) | 1,04 × / 1,03 × | 34 … 96 | 1,8 s | 1,40 × | −5,7 … +1,7° |
| v 0,3 rückwärts (3,8 s) | 1,02 × / 1,01 × | −81 … −53 | 1,4 s | 1,17 × | +1,0 … +4,4° |

- Vorwärts im Mittel behoben (vorher 0,62–0,78 ×), kein schnelles Schwingen.
- Vorwärts langsames Pendeln, Periode ~4 s: bei +46 Ist 26 (1,0 s) → 66 (2,8 s, 1,43 ×) → 34 (4,3 s, 0,74 ×); bei
  +69 38 → 96 (2,5 s, 1,39 ×) → 58 (3,8 s). Das Integral überschießt beim Aufbau des Vorwärts-Mehrbedarfs.
  Rückwärts sauber. An die Firmware-Sitzung zur Behebung (Nutzer 2026-11-25); Vorschläge: Integral sanfter oder
  den Mehrbedarf vorwärts als feste Vorsteuerung.
- Aufstehen wieder vom Nutzer über das Web (18:27:03), nicht mitgeschrieben, kein Sturz.
- Stufe 1/2 erst fahren, wenn vorwärts ruhig ist.

Einstellen am Rider ohne Flashen (Nutzer 2026-11-25): Die Firmware-Sitzung setzt je Runde im Liegen über USB (COM4,
DTR/RTS aus, kein Neustart) `!velctl KI FF IMAX KP KF` im RAM; Nutzer steckt das Kabel dafür an und zieht es zum
Fahren ab. Mitschnitt ab vor dem Aufstehen (`runde1.csv`, `runde2.csv`). Befund der Firmware-Sitzung aus Runde 1:
Das Integral steigt nur auf 1,6–2,4°; das Überschießen kommt vom Ist-Nick, der dem Soll-Nick nachläuft (bei +69 ist
der Soll-Nick ab 1,4 s negativ, der Ist-Nick bis 2,1 s über 1,5°) – also mehr P statt weniger I.

| Runde | Werte (KI FF IMAX KP KF) | Aufschwung max Soll-Nick | +46 Mittel / Spitze | −46 | +69 | −69 |
|---|---|---|---|---|---|---|
| 1 | 0,06 1 8 0,11 0,05 | 24,8° | 1,04 × / 1,33 × | 1,05 × / 1,18 × | 1,00 × / 1,36 × | 1,02 × / 1,18 × |
| 2 | 0,06 1 8 0,15 0,05 | 22,0° | 0,98 × / 1,13 × | 1,01 × / 1,15 × | – | – |

- Runde 2 vorwärts: 90 % nach 1,6 s ab Rampenbeginn (Firmware-Sitzung zählt ab Soll erreicht: Spitze 1,09 ×), ab 2 s
  45…51 Einheiten, kein Unterschwingen, kein Pendeln; Soll-Nick gleichmäßig auf −1,6 … −2,2°. Rückwärts Streuung 2,0.
- ±69 mit kp 0,15 nicht gefahren: Akku nach −46 bei 18 %, abgebrochen.
- Runde 3 (Werte neu gesetzt, nur +69 bei 22 % Akku): ab 2,2 s 69…72 Einheiten, kein Überschießen; −69 wegen Akku nicht.
- Firmware 023a6ff (SHA 5abeb9a6, kp 0,15 / ki 0,06 fest) geflasht. Danach (`runde4.csv`): Aufschwung Soll-Nick 14,2°;
  +69 (4,5 s) Mittel 0,92 × (90 % nach 2,1 s), Spitze 1,15 ×; −69 1,00 ×, Spitze 1,17 ×, Streuung 2,8. Stufe 0 fertig.

## Kursziel gefahren (Punkt 5 Stufe 2, 2026-11-25)

`vel_kurs` nur für den Test in `/opt/xgonav/config.toml`, danach wieder heraus. MCP `drehen` (`POST /turn`, w 0,25 →
90 °/s), Mitschnitt `runde5.csv` (`fw_turn` = Sollwert des Generators, `imu_yaw` roh).

| Soll | Dauer | ∫ w_soll (Generator) | IMU-Kurs am Ende | 2 s danach |
|---|---|---|---|---|
| +90° | 1,41 s | 89,5° | 87,3° | 87,4° |
| −90° | 1,41 s | −89,8° | −86,3° | −86,7° |
| +180° | 2,36 s | 180,1° | 177,3° | 175,8° |
| −180° | 2,41 s | −180,0° | −177,1° | −176,6° |

- Glatt: Rampe auf 90 °/s, rechtzeitig gebremst, kein Überschwingen der Rate, Generator trifft Δψ.
- Erste Drehung davor 98° statt 90°: xgonav schob das Ziel nach, weil der Restfehler vom IMU-Kurs kommt, der dem
  Generator nachläuft (Schwelle 2°). Behoben 13399a0 (Schwelle 20°, Nachkorrektur erst 0,5 s nach Ankunft), Test
  `test_kurs_nachlaufender_imu_kurs_schiebt_ziel_nicht`.
- IMU-Kurs 2–4 % kürzer: kein Maßstabsfehler (Kurs-PID und Strom nutzen beide `imu.yaw_deg`). Original-Logik
  `addHeadingCorrection`: im Stand korrigiert der Kurs-PID nicht und setzt das Kursziel nach 200 ms auf den Ist-Kurs; der
  Nachlauf am Ende bleibt stehen. Firmware-Fix 33a7c5a (SHA 3c9f6fc0, `TurnControl::hold` solange Kursziel-Rahmen
  kommen), geflasht 2026-11-25. Pi-Seite dazu: Handdrehung schickt im Nachlauf (`turn_settle_s` 1 s) weiter das
  Kursziel, Nachkorrektur `KURS_NACH_*` entfernt (korrigierte doppelt).

Nachfahrt mit 3c9f6fc0 (`runde7.csv`; Handdrehung hält 1 s im Nachlauf):

| Soll | ∫ w_soll | IMU-Kurs am Ende | vorher ohne Halten |
|---|---|---|---|
| +90° | 90,2° | 88,6° | 87,3° |
| −90° | −90,1° | −87,8° | −86,3° |
| +180° | 179,9° | 178,5° | 177,3° |
| −180° | −180,4° | −178,6° | −177,1° |

- Rest 1,4–2,2° statt 2,7–3,5°: Das Halten holt nur teilweise auf (Haftreibung wie im Modell der Firmware-Sitzung).
- Davor noch einmal 105° statt 90°: Der Kurs in xgonav läuft dem Generator bei 90 °/s über 20° nach, auch 20° Schwelle
  schob das Ziel nach. Jetzt nie ein neues Ziel, solange die Firmware dreht; erst nach Ankunft bei > 15° Rest (69f33db).

Nachfahrt mit 81cb3335 (Kurs-Regler bekommt beim Halten im Stand ein Integral, ki 0,04 je Takt, Totband 0,3°; Ursache
des Rests: Kurs-PID ohne Integral, P 3 Einheiten je Grad bleibt unter ~3° unter der Haftreibung) und 2 s Nachlauf
(`behavior.turn_settle_s` 2,0, 9c72292), `runde8.csv`:

| Soll | ∫ w_soll | IMU-Kurs am Ende | IMU-Kurs 2 s danach | Kurs xgonav (MCP) |
|---|---|---|---|---|
| +90° | 89,7° | 85,8° | 88,6° | 88,8° |
| −90° | −89,6° | −85,4° | −88,9° | −90,2° |
| +180° | 179,6° | 176,3° | 179,9° | 180,4° |
| −180° | −179,9° | −176,2° | −179,0° | −180,5° |

- Ziel ±2° erreicht: Im Nachlauf holt das Integral den Rest auf (IMU 2 s danach ≤ 1,4° daneben, Kurs xgonav ≤ 1,2°).

## Bogen gefahren (Punkt 5 Stufe 1, 2026-11-25)

`/drive?bogen=1` bzw. MCP `fahren`/`folge` mit `bogen` setzt `Command.kappa` = w/v (Handsteuerung); `vel_kappa` nur für
den Test. Gefahren über `bogenfahrt.py` am Rider (gleicher Ablauf wie MCP `fahren`, geprüft; das MCP-Werkzeug in Claude
Code kannte `bogen` noch nicht, die Werkzeugliste lädt nur beim Verbinden). v 0,2 (46 Einh.), w 0,115 → κ 2 1/m, 4,5 s.

| Größe | Ergebnis |
|---|---|
| κ_soll = w_soll / v_soll | 2,00 über Rampe und Fahrt |
| κ mit v_ist | 2,7–2,9 in der ersten Sekunde (v_ist 33 statt 46, langsamer Anlauf vorwärts), danach 2,0–2,2 |
| gedreht / Strecke | 91,5° / 0,75 m (Kreis R 0,5: 90° auf 0,79 m) |
| Endpunkt | 0,42 m vor, 0,51 m links (Kreis: 0,50 / 0,50) |
| Halt | `/stop` (v 0, w 0 ohne κ) schaltete auf den Drehratenweg: w sofort 0, geradeaus gebremst |

- Halt behoben: Nach einem Bogenrahmen bremst `Robot` bei v 0, w 0 weiter auf demselben Bogen (Krümmungsrahmen mit
  v 0) bis zum Stand; Drehen auf der Stelle schaltet sofort um (`test_halt_nach_bogen_bremst_auf_dem_bogen`).
- Offen: Der engere Anfang folgt aus der Kopplung an v_soll (Entscheidung Firmware) und dem langsamen Anlauf vorwärts.

## Anfahren an eine Wand (Punkt 2, gefahren 2026-11-25)

MCP `fahren` geprüft geradeaus auf einen glatten Pfeiler, Abstand Vorderkante Rad bis Wand nachgemessen:

| Fahrt | Halt | gemessen |
|---|---|---|
| v 0,2 (0,18 m/s) aus ~2 m | Bahnprüfung „Bahn 0.09m“ | 20,5 cm |
| v 0,3, zweiter Abschnitt aus dem Stand ~55 cm vor der Wand | „Bahn 0.09m“ | 21 cm |
| v 0,3 (0,27 m/s) aus ~1,2 m, durchgehend | „Bahn 0.10m“ | 22,5 cm |

- Haltepunkt unabhängig vom Anfahrtempo (laut Karte dreimal x 2,58–2,59 m): Die Bahnprüfung bremst voll, danach
  kriecht er mit kleinem Soll (7–30 Einheiten, `slow_free` bzw. Bahn) bis an die Grenze; kein Kontakt.
- Bis 2026-11-25 setzte die Bahnprüfung beim Vollbremsen den Fahrbefehl auf 0 (Byteweg: nur 0 setzte der Rider
  schnell um): schlagartiger Halt (Ist 74 → 102 durch den Gegenschwung, Soll-Nick bis −18°), danach fuhr er langsam
  wieder an (Nutzer sah es an den LEDs). Jetzt auf dem Fahrrahmen: Bahnprüfung gibt das erlaubte Tempo vor,
  `drive(brake=True)` verzögert auch auf Tempo > 0 mit `vel_brake`. Wiederholt mit v 0,3 aus 1,2 m („Bahn 0.17m,
  bremse“): Soll 69 → 53 → 39 → 26 → 8 in 0,6 s, Ist-Spitze 78, Soll-Nick bis −8°, kein Wiederanfahren; Halt
  ~24 cm (das Ende fiel mit dem Ende des 5-s-Abschnitts zusammen; Nachrücken bis an die Grenze nicht geprüft,
  Akku 18 %).
- Nebenbefund: rückwärts v −0,2 fährt er 1,29–1,37 × Soll (Ist −59…−63 statt −46, Soll-Nick +4°); vorwärts
  1,03–1,07 ×. Sache der Firmware (Nutzer 2026-11-25), an die Firmware-Sitzung gemeldet; xgonav ändert nichts.

## Dunkelangst (Befund 2026-11-24, zu Punkt 1)

- `App._dunkel_tick` (`xgonav/app.py`) sitzt im Regelschritt zwischen `safety()` und `_arc_check()` und ersetzt den
  Befehl des Verhaltens durch `Command(0, w)`: Drehen auf der Stelle mit eigenem Kursziel (`dreh_auf_kurs`), am
  Planer vorbei. Das ist ein zweites Gehirn.
- Ablauf (`xgonav/dunkel.py`): 2 s unter 7 → Angst 1,5 s → 180° drehen → 1,5 s prüfen (über 11 = weiter) →
  +90°, +180° → aufgeben (absetzen, Ruhemodus); 3-mal in 300 s → sofort aufgeben.
- Nur `explore` (`dunkel.zone_modes`): Dunkelzone 2 × 3 m voraus + `neu_planen()` löscht Ziel und Weg.
- Andere Modi: keine Zone, kein Neuplanen; das Verhalten rechnet weiter, seine Befehle werden verworfen, danach
  fährt der alte Weg weiter (evtl. zurück ins Dunkle).

## Entschieden (Nutzer 2026-11-25)

- Höchste Beschleunigungen und Ruck konfiguriert xgonav (`config.py`) und sendet sie mit jedem Rahmen.
- `v_max`/`w_max` schreibt xgonav beim Start (0x1b), nicht je Rahmen.
- Firmware-Tempoeinheit = 3,9 mm/s (`robot.fw_speed_unit_mm`, gemessen mit `tools/fw_einheit.py`), nicht mm/s.
- Beschleunigungsbetrieb: Rahmen ohne Geschwindigkeit führt die Beschleunigung mit Vorzeichen aus, begrenzt durch `v_max`.
- 0xa0/0xa1 sind raus; der Byteweg rechnet fest mit 210 / 300.
- Der Temporegler bleibt ein Faktor auf alle Sollwerte (Faktor × `tempo` × `max_v`/`max_w`).
