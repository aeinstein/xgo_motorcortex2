# Arbeitsplan Architektur

Ist-Stand und nächste Schritte. Verworfene Firmware-Ansätze: `firmware/README.md` (Holzwege).

## Zielbild

Kamera + FloorNet + IMU schreiben Merkkarte und Raumkarte. Das Gehirn (ein Modus) äußert einen Wunsch an den
Planer, der Planer fährt den Rider, nach der Sicherheitsprüfung.

## Plan

| # | Punkt | Stand |
|---|---|---|
| 1 | **Nur ein Gehirn mit Modus, jeder Weg läuft über den Planer.** Bei Dunkelangst wird der schnellste Weg zurück gefahren | offen (Befund unten) |
| 2 | **Test: Anfahren an eine Wand** | offen |
| 3 | **Test: Fahren einer Bahn entlang.** Kurven glätten, Drehungen auf der Stelle vermeiden (Hauptgrund der Drift) | offen |
| 4 | **Fahrregler in die Firmware.** Pi sendet Sollgeschwindigkeit und Drehrate in echten Einheiten, die Firmware regelt | erledigt, gefahren (unten) |

## Fahrweg (Punkt 4, Stand 2026-11-25)

- Firmware-Patch `08-velocity`: Rahmen 0x1a (v mm/s, a, Ruck, w, alpha, Drehruck), Grenzen 0x1b (v_max, w_max,
  Wächter, Einheit, ki, i_max, kv), Sollwertgenerator mit Ruckbegrenzung, Soll-Nick = Original-P + Integral
  (erst 0,6 s nach der Rampe) + atan(a/g) − kv·v_ist, Kursziel läuft mit w·dt. Einzelheiten `firmware/README.md`.
- Treiber: `/proc/XGORider/vel` und `vel_limits` (7 Felder), Text ↔ Bytes in `kernel_module/xgo-vel.h`.
- xgonav (`xgonav/fahrrahmen.py`, `Robot`): `drive(v, w)` bleibt normiert; mit Treiber und Firmware schreibt Robot
  Rahmen (m/s = v × `vx_limit`, °/s = w × `vyaw_limit_dps`, an die Firmware ÷ `fw_speed_unit_mm` 3,9), `vel_limits`
  beim Start und alle 2 s. Werte: `vel_kv` 0,025, `vel_ki` 0,03, `vel_i_max_deg` 6. Ohne Treiber/Firmware Bytes.
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
