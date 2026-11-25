# Firmware
- Kalibrierung durch LEDs anzeigen
  

# XGONAV

- C++-Kern `navfast.cpp` (Planer, Frontier-Suche, Merkkarte): am Rider seit 2026-11-12 eingespielt, gebaut und dort bitgleich getestet; noch nicht gefahren (`plan_ms` und Regelschritt in Fahrt messen).
- Regelschritt der Hauptschleife am Rider liegend 0,5 ms im Mittel (p95 1,3; vor `navfast` 1,7 / 4,3). Noch Python: `World`/`safety`/Verhalten, Rad-Historie, Statuswerte je Bild (`snap` 0,3 ms), Punkte des Kartenabgleichs (`reloc.scan_punkte`, nur mit `reloc.track`), Glätten der FloorNet-Kante (`ZeilenGlaetter`).
- Schallrichtung (`xgonav/ortung.py`, `docs/audio.md`): kalibriert 2026-11-13 balancierend (Testton 1 m, 8 Stellungen), liegend unbrauchbar. Offen: mit Sprache und beim Fahren (Motorgeräusch) prüfen, Karte „Schallradar“ im Browser ansehen, Hören in `pet` (horchen, Rufen, Schreck, Wecken, Mitreden; 2026-11-13) am Rider ansehen: erst liegend Töne, dann stehend Drehen, Rufen (`call_m`) nur mit Aufsicht.
- Idee (2026-11-23, verschoben): FloorNet-Eingang am Horizont ±30 Zeilen 1:1 statt 2:1. Nutzen vermutlich gering: das Band (beim Fahren Zeilen ~220–280) ist oben nur Wand, unten Boden ab ~1,25 m, Karte nutzt bis 1,5 m; Hauptfehler sind die Labels (Teppich an/aus). Kosten am Pi: senkrecht 1:1 (320 breit) ~+18 % MACs ≈ +2,4 ms, voll 640 breit ~+7 ms; dazu Training, Export, `floornet_kern.h`, `sichtfeld.py`. Horizont wandert mit Nick (~5,4 px/°), Bandgrenze läge mal auf Wand, mal auf Boden. Vorher prüfen: `eval_labels.py` Fehler je Entfernungsband (explore10–12), dann auf kimmy Obergrenze mit ganz 1:1 gegen p11; bringt das nichts, bringt das Band auch nichts.

## Aufräumen 2026-11-17: offen

Alles uncommittet, am Rider nicht eingespielt, nicht gefahren. Diese Punkte brauchen eine Fahrt oder Messung:

- Regelschritt 24 Hz statt ~12 Hz (`control_hz`, jedes Bild): am Rider einspielen, Schleifenzeiten vor/nach messen, erste Fahrt mit Beobachtung (die Regler waren bei ~12 Hz abgestimmt).
- Treiber neu bauen und einspielen (erst xgonav, dann Treiber); `serial_stats` balancierend und liegend prüfen.
- `/opt/xgonav/config.toml` am Rider: entfernte Schlüssel streichen (`[horizon]`, `turn_settle_*`, `weak_wall_l`, `pass_turn_max_deg`, reloc `scan_extra_deg`/`max_shift_m`/`tries`), sonst `KeyError` beim Start; `control_hz` prüfen.
- `freespace.net_smooth_n = 3`: Zacken evtl. seit IMU je Bild / 36094d2 weg; kostet ein Netzbild Verzögerung.
- `reloc.track`: Default an, am Rider aus; alle 10 s Kopie der 48-m-Karte.
- `home.turn_w_max = 0.25`: mit Firmware-Patch 04-attitude neu messen.
- Anschub `behavior.push_*` + Fahr-Integral der Firmware (07-balance): zwei Mechanismen für dasselbe (`robot.speed_ctrl` ist seit 2026-11-25 raus, die Firmware regelt das Tempo auf dem Fahrrahmen), nach einer Fahrt entscheiden.
- `freespace.restart_hold_s` (1 s ohne Freiraum nach dem Aufstehen): Anlass war das Farbmodell (seit 2026-11-18 entfernt), das Netz lernt nichts; prüfen, ob die Haltezeit noch nötig ist.
- Raumkarte im Web holt `map.png` je Sekunde (Anzeige kostet Pi-Zeit).
- `localmap.py`: zweimal `rebase` je Bodenergebnis in Python, Kandidat für C++.
- `tests/sim_run.py` `RIDER`: `max_v` 0,15 / `max_w` 0,30 entsprechen nicht mehr dem Rider (0,5 / 0,5); ändern heißt slow-Tests neu abstimmen.
- rc_server-Zertifikatsumzug in `deploy/install-all.sh` kann raus, sobald am Rider geprüft ist, dass das Zertifikat am neuen Ort liegt.
- Ungenutzte Registerkonstanten in `constants.h` bleiben als Registerübersicht (`GPIO_LCD_*`/`SPI_FREQ_MHZ` nutzt `st7789/`).

# Frontend
