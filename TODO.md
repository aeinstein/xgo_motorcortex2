# Firmware

# XGONAV

- C++-Kern `navfast.cpp` (Planer, Frontier-Suche, Merkkarte): am Rider seit 2026-11-12 eingespielt, gebaut und dort bitgleich getestet; noch nicht gefahren (`plan_ms` und Regelschritt in Fahrt messen).
- Regelschritt der Hauptschleife am Rider liegend 0,5 ms im Mittel (p95 1,3; vor `navfast` 1,7 / 4,3). Noch Python: `World`/`safety`/Verhalten, Rad-Historie, Statuswerte je Bild (`snap` 0,3 ms), Punkte des Kartenabgleichs (`reloc.scan_punkte`, nur mit `reloc.track`), Glätten der FloorNet-Kante (`ZeilenGlaetter`).
- Schallrichtung (`xgonav/ortung.py`, `docs/audio.md`): kalibriert 2026-11-13 balancierend (Testton 1 m, 8 Stellungen), liegend unbrauchbar. Offen: mit Sprache und beim Fahren (Motorgeräusch) prüfen, Karte „Schallradar“ im Browser ansehen, danach Reaktion in `pet` (zum Geräusch drehen; löst auch vorn/hinten auf).

# Frontend
