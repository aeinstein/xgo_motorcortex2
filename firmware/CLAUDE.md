# CLAUDE.md – firmware/

Details, Register und Holzwege: `README.md`.

## Zweck

`firmware/` baut die ESP32-Firmware R-1.1.6 aus der Referenz `LuwuDynamics/xgorider2_rl` plus 8 Themen-Patches in `firmware/patches/`. Am RiderPi läuft die Env `rider2-firmware`.

## Befehle

```bash
firmware/build.sh [rider2-firmware]   # bauen (PlatformIO, flasht NICHT) -> firmware/out/app0.bin, reproduzierbar
firmware/tests/run.sh [filter]        # Host-Tests (g++, nach build.sh); am PC kein g++: auf kimmy (~/fwtest)
```

## Regeln

- Firmware nie selbst flashen.
- Nur app0 (`0x10000`), niemals `0x310000–0x32ffff` (Beinnullpunkte, BLE-Name, Pitch-Trim) oder spiffs.
- Kein Vollbackup (`read-flash`) vor dem Flashen: der Nutzer hat rund 30 Sicherungen.
- Am echten Rider: Bewegungen nur mit Zustimmung und Beobachtung durch den Nutzer, Körperhöhe nie unter Minimum + 10 mm.
- Ein Thema = ein Patch. Änderungen gehen in den passenden Patch (im Build-Klon ändern, Patch neu erzeugen), kein neuer Patch je Fix. Jeder Patch muss für sich auf die vorigen bauen. Verworfenes kommt als eine Zeile unter „Holzwege“ in `README.md`, kein Verlauf in den Patches oder Kommentaren.

## Patches

| Patch | Inhalt | Register / Aktionen |
|---|---|---|
| 01-ble | eigener BLE-Dienst `RiderKerk`, Tunnel zum Pi, Konsole an den Kanal der Zeile | 0x17 schreiben, 0x18 |
| 02-pi-interface | Lage + Räder in einem Schnappschuss, IMU-/Radstrom | 0x0d, 0x11 |
| 03-diagnostics | Reglergrößen, IMU-Rohsummen, Hebeschutz-Protokoll | 0x0e, 0x15, 0x19 |
| 04-attitude | Lagefusion als Richtungsvektor, alle Kreisel-Nullpunkte, Startkalibrierung nur im Stillstand, Ausgleich 2,05° | |
| 05-actions | Not-Aus, Aufstehen, Neustart, Aktionstabelle, Akku-Warnung hebt sich auf, kein Selbststart nach Sturz | 254, 253, 237 |
| 06-leds | Helligkeit, Lauflicht beim Kalibrieren, LEDs Sache des Clients, Blau beim Herunterfahren | 0x12, 234/233 |
| 07-balance | Positionshalten beim Drehen, Roll-Trimm, Roll halten, Fahr-Integral, Einlegen in Kurven (aus) | 232/231 |
| 08-velocity | Fahrrahmen in mm/s und °/s, Sollwertgenerator, Krümmungsbetrieb, Kursziel, Geschwindigkeits- und Drehratenregler | 0x1a, 0x1b |

## Stand am Rider

Geflasht 2026-11-25 die Serie 01–08 mit Krümmungsbetrieb, Kursziel und Tempo-Regler P 0,11 + 0,04 im ±40-Band (kp 0,15) + Vorsteuerung −kf·v_soll + Integral ab Rampenende (Band 40, ki 0,06) und Kurs-PID hält während des Kursziels (SHA 3c9f6fc0…, Stufen 1+2 aus ARBEITSPLAN.md Punkt 5), davor 5abeb9a6… (Kursziel 2–4 % kurz), davor ec493c66… (kp 0,11, ki 0,1: vorwärts langsames Pendeln), davor 2a3c9422… (Integral 0,6 s nach der Rampe, ki 0,05, Band 20: vorwärts 0,62–0,78 ×), davor f287c1a9… (kp 0,3 im ±20-Band: steht auf, schwingt beim Fahren mit 2,5 Hz), davor 4aabe9f6… (gefährlich beim Aufstehen: kp 0,3 auf den ganzen Fehler, Sturz 2026-11-25), davor 2a7f213d… (nur Krümmung), davor 12556176… (Commit e93e525, Tempo nur über Soll − Ist: kv raus, kp 0,3, ki 0,15, i_max 8° in der Firmware, ki/i_max/kv aus 0x1b ignoriert); Rückfallstand `out/app0_0035neu.bin` (0001–0035, SHA ef2a2894…). Tempo-Regelung, Krümmungsbetrieb und Kursziel **noch nicht gefahren**. Diese Werte vorher im RAM per `!velctl` (USB, DTR/RTS aus) gefahren: +46 Spitze 1,09 ×, +69 0,98–1,04 ×, −46 sauber; −69 1,00 ×. Kursziel trifft im Generator, IMU-Kurs blieb mit 5abeb9a6 2–4 % kurz (Original-Logik gab den Kurs im Stand auf); mit 3c9f6fc0 noch nicht gefahren. Byte-Vertrag: Kopfkommentar `velocity_cmd.h` + ARBEITSPLAN.md Punkt 5, Änderungen an beiden Stellen und an die xgonav-Sitzung melden; Prüfvektoren `tests/vectors/*.csv` neu schreiben (`XGO_VECTORS=1 firmware/tests/run.sh velocity_cmd` auf kimmy). Einlegen in Kurven (232) war nie an; erst Vorzeichen in einer Kurve prüfen.

## Schnittstelle zum Pi

- Treiber `xgo-drv` liest Lage und Räder aus dem Strom (0x11) oder, ohne Strom, aus 0x0d; `/proc/XGORider/wheels` (1024 Ticks/Umdrehung, Rad 11 = links), `/proc/XGORider/lift` (0x19), Fahrrahmen über `vel`/`vel_limits` (0x1a/0x1b).
- BLE: Der Treiber meldet die IPv4 von `wlan0` alle 3 s auf 0x18 und reicht den Tunnel als `/proc/XGORider/ble` (nur root) an `xgo-ble` weiter (`docs/ble.md`). Aktion 254 über BLE direkt geschrieben hält xgonav nicht an: vorher `nav POST /stop` über den Tunnel.
- `api.php` schreibt `action` direkt nach `/proc`; wer 254 schreibt, ohne vorher `POST /stop` zu rufen, lässt ihn nach einer Ohnmacht (Modus `pet`) wieder aufspringen.
- Körperhöhe: Nach jedem Aufstehen nimmt die Firmware ihre Mittelstellung 128 (≈ 90 mm), `/proc` zeigt dann noch den alten Wert (`robot.FW_HEIGHT_STANDUP_U8`); gefahren wird nur u8 55..200.
