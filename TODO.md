# Firmware

# XGONAV

- C++-Kern `navfast.cpp` (Planer, Frontier-Suche, Merkkarte): am Rider seit 2026-11-12 eingespielt, gebaut und dort bitgleich getestet; noch nicht gefahren (`plan_ms` und Regelschritt in Fahrt messen).
- Regelschritt der Hauptschleife am Rider liegend 0,5 ms im Mittel (p95 1,3; vor `navfast` 1,7 / 4,3). Noch Python: `World`/`safety`/Verhalten, Rad-Historie, Statuswerte je Bild (`snap` 0,3 ms), Punkte des Kartenabgleichs (`reloc.scan_punkte`, nur mit `reloc.track`), Glätten der FloorNet-Kante (`ZeilenGlaetter`).

# Frontend
