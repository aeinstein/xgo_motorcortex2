# CLAUDE.md – firmware/

Firmware-Infos aus der Root-`CLAUDE.md`, hier gebündelt. Details, Registerformate und Flash-Historie: `README.md`.

## Zweck

`firmware/` baut die ESP32-Firmware R-1.1.6 aus der Referenz `LuwuDynamics/xgorider2_rl` plus Patches in `firmware/patches/`. Am RiderPi läuft die Env `rider2-firmware`.

## Befehle

```bash
firmware/build.sh [rider2-firmware]   # ESP32-Firmware bauen (PlatformIO, flasht NICHT) -> firmware/out/app0.bin

# Host-Tests (nach build.sh, im Repo-Root; genauer Aufruf steht im Kopf jeder tests/*.cpp)
S=firmware/build/xgorider2_rl/firmware/src
g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_turn_hold.cpp $S/control/rider_motion.cpp $S/control/pid.cpp -o t && ./t
```

## Regeln

- Firmware nie selbst flashen.
- Nur app0 (`0x10000`), niemals `0x310000–0x32ffff` (Beinnullpunkte, BLE-Name, Pitch-Trim) oder spiffs (`README.md`).
- Kein Vollbackup (`read-flash`) vor dem Flashen: der Nutzer hat rund 30 Sicherungen.
- Am echten Rider: Bewegungen nur mit Zustimmung und Beobachtung durch den Nutzer, Körperhöhe nie unter Minimum + 10 mm.

## Patches

| Patch | Inhalt | Schalter / Register | Host-Test |
|---|---|---|---|
| 0001 | Rad-Register 0x0c/0x0d | lesen 0x0c, 0x0d | |
| 0002 | Not-Aus | Aktion 254 | |
| 0003 | LED-Helligkeit (abgelöst durch 0016) | | |
| 0004 | Aufstehen aus der Liegelage | Aktion 253 | |
| 0005 | Drehung um die Senkrechte aus Nick- und Rollrate herausrechnen; seit 2026-11-07 mit den Beschleunigungswinkeln statt der Lageschätzung, plus 2,4 % Gier-Übersprechen im Nick-Kreisel | | `test_imu_yaw_coupling.cpp` |
| 0006 | Beschleunigungswinkel bei schneller Drehung einfrieren (Fliehkraft), sonst rollt der Rider beim Drehen weg | | `test_imu_yaw_coupling.cpp` |
| 0007 | Diagnose-Register (Reglergrößen) | 0x0e, am Pi über `raw_reg` | |
| 0008 | Positionshalten auch beim Drehen auf der Stelle | Aktion 241 an / 240 aus | `test_turn_hold.cpp` |
| 0009 | Gyro-z-Nullpunkt laufend nachführen (Räder/Stillstand), Gier-Totband 2 → 0,3 °/s | 0x0f | `test_yaw_bias.cpp` |
| 0010 | Halten beim Drehen per Default an | | |
| 0011 | Gier-Totband entfernt, Kurs erst nach der Startkalibrierung integriert | | |
| 0012 | Roll-Trimm aus dem Nivellieren bleibt stehen, auch über Höhenänderungen (vorher ~2° Roll nach jeder Höhenänderung) | | `test_roll_trim.cpp` |
| 0013 | Diagnose Lagefusion Nick/Roll, nur Anzeige | 0x10 | `test_imu_diag.cpp` |
| 0014 | IMU-/Rad-Datenstrom, Pakete Typ 0x14, 34 Byte | 0x11 = Rate in Hz | `test_imu_stream.cpp` |
| 0015 | Integral beim Fahren, sonst blieb er an leichten Steigungen trotz vollem Fahrbefehl stehen. Erste Fassung ließ die Geschwindigkeit pendeln (Periode ~5 s); jetzige hält nur eine Untergrenze von 30 % des Befehls | Aktion 239 an / 238 aus, Default an | `test_drive_integral.cpp` |
| 0016 | LED-Helligkeit als globaler Multiplikator zur Laufzeit, Default 32 = halb so hell wie bisher | 0x12 = n/256, am Pi `settings/led_brightness` | |
| 0017 | Nick-Fusion nimmt den ungefilterten Beschleunigungs-Nick (außer vor der Startkalibrierung und bei 0006-Einfrieren); vorher kamen Nickbewegungen ~6 % zu klein heraus (Vorfilter hinkt nach), im Stand stimmte der Winkel | | `test_pitch_fusion.cpp` |
| 0018 | Neustart des ESP32 auf Befehl, nur wenn er nicht balanciert; schickt vorher „Drehmoment aus“ an beide Beine. Setzt die Servos selbst nicht zurück; danach balanciert er von selbst los, sobald er aufrecht steht | Aktion 237, Konsole `!restart`, Web „ESP-Neustart“ | |
| 0019 | Startkalibrierung des Gier-Nullpunkts gilt nur im Stillstand (Spanne je Kreiselachse < 1,5 °/s, je Beschleunigungsachse < 0,5 m/s²), sonst von vorn; nach 20 verworfenen Fenstern (~15 s) wie im Original. Vorher stand der Nullpunkt nach bewegtem Einschalten um 12–20 °/s daneben, er drehte balancierend langsam | 0x10 Flags bit4..7 = verworfene Fenster | `test_gyro_boot_cal.cpp` |
| 0020 | Rohwerte der IMU als laufende Summen (Beschleunigung, Kreisel, Radticks) für das Sensormodell der neuen Lagefusion; nur Diagnose, die Lagerechnung bleibt gleich. Leser `tools/rawdiag.py` | 0x15, am Pi über `raw_reg` | `test_raw_diag.cpp` |
| 0021 | Lagefusion neu: Lage als Richtungsvektor („oben“ in Körperachsen), vom Kreisel exakt weitergedreht; Nullpunkte aller drei Kreisel aus der Startkalibrierung, Nick/Roll laufen nach (40 s; nach bewegter Kalibrierung 30 s lang schnell); Achsfehler Nick/Roll-Kreisel gegen Gier −1,24 % / +0,99 %; Beschleunigung zieht mit 8 s nach, Gewicht stufenlos nach Betrag und Drehrate, v·Gierrate abgezogen; Kurs aus der Drehrate um die Senkrechte. **Ersetzt die Rechnung von 0005, 0006 und der Umschaltung in 0017** (nichts wird mehr eingefroren). Nach außen geht der wahre Nick (~2° tiefer als bis 0020); Regler und Wächter bekommen Nick + Ausgleich (Default 2,05°). Register 0x10: Felder 10/16/24/28 und Flag bit0 neu belegt (`main.cpp`) | 0x16 schreiben = Ausgleich (Wert − 128) × 0,05°, Konsole `!balance <Byte>` | `test_lagefusion_replay.cpp` (echte Rohdaten, Referenz `tests/lagefusion_ref.py`), `test_imu_yaw_coupling.cpp`, `test_pitch_fusion.cpp`, `test_imu_diag.cpp` |
| 0022 | LEDs wabern in der Nutzerfarbe (weiche Welle, 1,2 s, 12–100 %), solange der Kreisel beim Start kalibriert, danach fest; Akku-Warnung, Modus 0 und Fehlerfarben gehen vor. Still liegend nur ~1,5 s, bewegt bis ~15 s, bei \|Roll\| ≥ 4° dauerhaft | | `test_led_calibrating.cpp` |

## Stand am Rider

0022 gebaut 2026-11-19 (SHA da789d48…), **nicht geflasht**, am Rider ungesehen; Host-Test grün. Rückfallstand `out/app0_0021.bin`. 0001–0021 seit 2026-11-18 ~17:40 geflasht (SHA bda32b82…). 0021 bisher nur liegend geprüft: Schleife 333 Hz wie vorher, Startkalibrierung im Stillstand angenommen (Nullpunkte Nick −1,15 / Roll −0,06 / Gier −0,77 °/s), Nick und Roll auf 0,13° / 0,06° beim Beschleunigungssensor, Kurs −0,04° in 12 s; Gefahren 2026-11-18 17:30 (Hand: Stand, 4 × ±90° im Stand bis 100 °/s, zwei Bremsdrehungen aus ~0,5 m/s mit 89 °/s): hält im Stand die Stelle auf 3 mm (Ausgleich 2,05° passt), Nick-Streuung 0,14–0,17°; Roll in den Bremsdrehungen ≤ 0,45° vom Stand (bis 0020: 3,7–5°), Nick 1,5–3 s danach +0,2° / −0,3° (vorher +3…+4°), Drehen im Stand Roll ≤ 0,74°. Kamera-Gegenprobe (xgonav-Sitzung, Fahrt 172730): Nick nach Bremsdrehungen und nach Drehen im Stand auf ±0,2° (vorher 1,5–2,7° über 3 s); im Bremsnick selbst ±1,8–2,1° für 0,5 s, dort trägt die Kamera nur ±1° (20 ms Belichtung). Offen: Schleife beim Balancieren 260 statt 282 Hz (Rechnung in double; auf float umstellen), xgonav-Nick-Bezug neu bestimmen, Erkundungsfahrt, Aufspringen mit 0021 (Nutzer musste wegen hängendem Bein neu starten, Ursache weiter offen). Rückfallstände `out/app0_0020.bin`, `out/app0_0019.bin`. 0001–0020 seit 2026-11-18 geflasht (0020: Register 0x15 liegend ausgelesen, Schleife 333 Hz, `rawdiag.py` liest 83 Hz; Nullpunkte liegend Nick −1,13 / Roll −0,03 / Gier −0,76 °/s, Betrag der Beschleunigung 9,97 m/s²; balancierend noch nicht mitgeschrieben). 0001–0019 seit 2026-11-17 19:34 geflasht (0017 seit 18:12, 0018 seit 19:20); 0019 nur liegend geprüft (kalibriert, kein Fenster verworfen, Nullpunkt −0,78 °/s), bewegtes Einschalten noch nicht ausprobiert; 0017 noch nicht gegen das Kamerabild geprüft und nicht balanciert, neues 0015 noch nicht gefahren, 0016 noch nicht ausprobiert. Seit 0017 scheiterten 3 von 4 Aufspringversuchen (19:09:56 Zustand 251 = Aktorfehler 0,2 s nach dem Start, linkes Bein hielt fest; Ursache offen, `README.md`). 0018 im Liegen über die Web-API ausprobiert (IMU-Strom ~1 s weg, kommt von selbst zurück), an einem festhaltenden Bein noch nicht.

## Schnittstelle zum Pi

- Treiber `xgo-drv` liefert aus dem Strom (0014) oder, ohne Strom, aus 0x0d (0x0c fragt er seit 2026-11-17 nicht mehr ab) `/proc/XGORider/wheels` (Encoder, 1024 Ticks/Umdrehung, Rad 11 = links); `xgonav/wheels.py` liest das (`sim_run.py --wheels`).
- `api.php` schreibt `action` direkt nach `/proc`; wer 254 schreibt, ohne vorher `POST /stop` zu rufen, lässt ihn nach einer Ohnmacht (Modus `pet`) wieder aufspringen.
- Körperhöhe: Nach jedem Aufstehen nimmt die Firmware ihre Mittelstellung 128 (≈ 90 mm), `/proc` zeigt dann noch den alten Wert (`robot.FW_HEIGHT_STANDUP_U8`); gefahren wird nur u8 55..200.
