# Firmware

# XGONAV

- C++-Kern `navfast.cpp` (Planer, Frontier-Suche, Merkkarte): am Rider seit 2026-11-12 eingespielt, gebaut und dort bitgleich getestet; noch nicht gefahren (`plan_ms` und Regelschritt in Fahrt messen).
- Regelschritt der Hauptschleife am Rider liegend 0,8 ms im Mittel (p95 1,9; vor `navfast` 1,7 / 4,3). Noch Python/numpy: Geometrie je Beobachtung (`ground_dist_np`, in `LocalMap.add_freespace`, `GridMap._rays`, `reloc.scan_punkte` je einmal gerechnet; 0,35 ms je Eintrag in die Merkkarte), `World`/`safety`/Verhalten, Rad-Historie, Statuswerte je Bild (`snap` 0,3 ms).

# Frontend
