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

## Stand am Rider

0001–0019 seit 2026-11-17 19:34 geflasht (0017 seit 18:12, 0018 seit 19:20); 0019 nur liegend geprüft (kalibriert, kein Fenster verworfen, Nullpunkt −0,78 °/s), bewegtes Einschalten noch nicht ausprobiert; 0017 noch nicht gegen das Kamerabild geprüft und nicht balanciert, neues 0015 noch nicht gefahren, 0016 noch nicht ausprobiert. Seit 0017 scheiterten 3 von 4 Aufspringversuchen (19:09:56 Zustand 251 = Aktorfehler 0,2 s nach dem Start, linkes Bein hielt fest; Ursache offen, `README.md`). 0018 im Liegen über die Web-API ausprobiert (IMU-Strom ~1 s weg, kommt von selbst zurück), an einem festhaltenden Bein noch nicht.

## Schnittstelle zum Pi

- Treiber `xgo-drv` liefert aus dem Strom (0014) oder, ohne Strom, aus 0x0d (0x0c fragt er seit 2026-11-17 nicht mehr ab) `/proc/XGORider/wheels` (Encoder, 1024 Ticks/Umdrehung, Rad 11 = links); `xgonav/wheels.py` liest das (`sim_run.py --wheels`).
- `api.php` schreibt `action` direkt nach `/proc`; wer 254 schreibt, ohne vorher `POST /stop` zu rufen, lässt ihn nach einer Ohnmacht (Modus `pet`) wieder aufspringen.
- Körperhöhe: Nach jedem Aufstehen nimmt die Firmware ihre Mittelstellung 128 (≈ 90 mm), `/proc` zeigt dann noch den alten Wert (`robot.FW_HEIGHT_STANDUP_U8`); gefahren wird nur u8 55..200.
