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
| 5 | **Bahnbefehle für die Navigation.** Krümmung statt Drehrate, Drehen auf Kurs, Strecke mit Halt, Rückmeldung im Strom (Plan unten) | Vertrag mit der Firmware-Sitzung abgestimmt; Stufe 1 wartet auf Freigabe durch den Nutzer |

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
| 0 | Rückwärts symmetrisch, Höchstwerte | geflasht 2026-11-25 (e93e525): Tempo nur über Soll − Ist (kp 0,3, ki 0,15, i_max 8° fest); ki/i_max/kv aus 0x1b ignoriert | – | `vel_kv`/`vel_ki`/`vel_i_max_deg` als wirkungslos gekennzeichnet (`config.py`, `docs/configuration.md`); a_max ohne Kippen, w/alpha-Grenzen messen → `config.py` | 0 | Messfahrten v ±0,2/0,3, eingeschwungen ≥ 3 s (noch nicht gefahren) |
| 1 | **Krümmungsbetrieb** | Flag bit1 in 0x1a: Feld w = κ in 0,001 1/m (i16, ±32 1/m); im Takt w_soll = κ · v_soll · 180/π (aus dem **Sollwert**, nicht v_ist: Kurs = ∫κ·v_soll dt, rauschfrei, am Pi nachrechenbar; keine v_min-Schwelle, w → 0 von selbst). Bahntreue unter Grenzen: nicht w kappen, sondern v begrenzen: \|v\| ≤ w_max/\|κ\|, a ≤ alpha/\|κ\|, Ruck ≤ Drehruck/\|κ\|. Neues κ während der Fahrt mit \|dκ/dt\| ≤ alpha/\|v\| nachführen (Klothoide), sonst springt w um Δκ·v. Rückwärts gleiches κ (derselbe Kreis rückwärts). Positionshalten beim Drehen (07-balance) greift nur bei v = 0, verträglich. Ohne bit1 wie heute | Text um κ erweitern, `zustand` liest Flag und Fähigkeitsbyte | `Robot.drive(v, w)` bleibt; neu `drive_bogen(v, kappa)`. `PathFollower` lenkt mit κ (Pure Pursuit κ = 2·sin α / L statt `kp_turn·err`); `_arc_check` prüft und bremst bei festem κ (nur v sinkt, Bahn bleibt). `Achse.schritt` + Kopplung, v/a/Ruck-Begrenzung durch κ und κ-Nachführung in Python nachgebaut, gegen Prüfvektoren | sinkt leicht (kein Lenkregler je Schritt, eine Multiplikation im 50-Hz-Durchlauf) | Host-Test: Bahn aus Generator-Simulation liegt bei Anfahren/Bremsen auf dem Kreis (Abweichung < 1 cm auf 1 m); Simulator `--dyn` explore/home; am Rider Kreisbogen v 0,2, κ 2 1/m, Endpunkt gegen Karte |
| 2 | **Kursziel** | Flag bit2: Feld w = Δψ in 0,1°, relativ zum Generatorwinkel bei Empfang. Der Generator integriert seinen Winkel ∫w_soll dt selbst (= Änderung des Kursziels): Rest → w_ziel = sign · min(w_max, √(2·alpha·\|Rest\|)) → vorhandenes `approach` der Drehachse; angekommen bei \|Rest\| < 0,5° und w = 0 (Flag), danach hält der Kurs-PID | wie 1 | `dreh_auf_kurs`/`align`/`POST /turn` schreiben einmal Δψ statt 24 Hz `dreh_w`; Ankunft aus dem Flag statt aus dem Kursfehler | sinkt (keine Kursregelung je Schritt) | Host-Test Überschwingen < 1°; am Rider 90°/180° links/rechts, Endkurs gegen IMU-Kurs ±2° |
| 3 | **Strecke mit Halt** | Flag bit3, Rahmen 15 Byte mit Reststrecke s (mm, u16): Rest = s − Radstrecke (Encoder) seit Empfang, v_ziel = sign · min(v, √(2·a·Rest)), danach v = 0 und Positionshalten. Neue Rahmen setzen s neu (Pi schickt die Reststrecke nach) | wie 1 | Wegfolger am Wegende (`end_v`) und Bahnprüfung schicken s = min(Weg bis Ziel, `clear − Rest`) statt das Tempo zu senken; Kriechen mit `slow_free` entfällt | sinkt | Wand wie Punkt 2: Halt 20 cm ±1 cm aus v 0,2/0,3; Host-Test Halt auf ±5 mm |
| 4 | **Rückmeldung** | Eigener Stromtyp **0x16** (0x14 bleibt unverändert), nur bei aktivem Rahmenweg, gleiche Rate und gleiches t_us wie die IMU-Probe, 12 Byte: u16 seq, u32 t_us, i16 v_soll, i16 w_soll (0,1 °/s), u8 Flags (Kurs erreicht, Strecke erreicht, Wächter, Sättigung = Soll-Nick an der Grenze), u8 reserviert; ~2 kB/s bei 100 Hz | Stromtyp 0x16 lesen, Felder in `/proc/XGORider/vel` | `Achse`-Nachbau nur noch für Simulator; Bahnprüfung und Anzeige nehmen den echten Sollwert; Sättigung senkt `kurve_a_mps2`/Tempo für den Rest der Fahrt | sinkt | Vergleich Nachbau ↔ Strom an einer Fahrt |

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
- Alte Rahmen (bit1–3 = 0) verhalten sich exakt wie geflasht; xgonav schaltet Stufe für Stufe um
  (`[robot] vel_kappa`, `vel_kurs`, `vel_strecke`, Default aus bis zur Fahrt).
- Rückmeldung: Stromtyp 0x16 (Stufe 4), 0x14 bleibt unverändert.
- Prüfvektoren: Die Firmware-Host-Tests schreiben bei `XGO_VECTORS=1` (Lauf auf kimmy) für `approach` + Kopplung
  Tabellen (Eingaben, Sollwerte je Takt) nach `firmware/tests/vectors/*.csv`, die CSV werden committet;
  `tests/test_fahrrahmen.py` rechnet sie mit `Achse` nach (ESP32 float, Toleranz 1e-4).

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
