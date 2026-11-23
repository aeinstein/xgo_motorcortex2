---
name: fahrt
description: Neue Fahrt(en) vom Rider verarbeiten – nach kimmy und lokal kopieren, prüfen, am Rider löschen, für ComfyUI einreihen und auswerten (Statusverlauf, Bremsen, Steckenbleiben, Bilder, Bodengrenze des Rider-Netzes). Verwenden, sobald der Nutzer eine neue Fahrt/Aufzeichnung/Aufnahme meldet oder fragt, was bei einer Fahrt passiert ist.
---

# fahrt: neue Fahrt holen, einreihen, auswerten

Für **jede** neue Fahrt dasselbe, ohne Rückfrage (Nutzer 2026-11-23: „bitte immer für jede fahrt das gleiche
machen … kopieren, löschen, einreihen und analysieren“). Das lokale Repo und kimmy sind die Wahrheit; am Rider
bleibt nichts liegen, auch keine Sicherungen.

## 1. Holen, prüfen, löschen, einreihen (ein Aufruf)

```bash
bash .claude/skills/fahrt/fahrt_holen.sh "<scratchpad>/fahrt"
```

- Findet den Rider unter 192.168.53.110 oder 10.10.0.110 (Schlüssel `aeinstein-ed.pem`).
- Jede Fahrt in `/home/aeinstein/fahrt/2026*`: per tar nach kimmy `~/floornet/data/rider/fahrt/mit_imu/<fahrt>`
  plus Link `fahrt/<fahrt>` und gleichzeitig ins lokale Verzeichnis. Gelöscht wird am Rider nur, wenn Dateizahl
  und meta-md5 auf kimmy **und** lokal mit dem Rider übereinstimmen; sonst Zeile `FEHLER`, Fahrt bleibt.
- Läuft gerade eine Aufnahme (`rawrec`), bleibt die neueste Fahrt unangetastet.
- Sonstige Dateien dort (`karte*.npz`): nach kimmy `fahrt/ohne_imu`, md5 prüfen, löschen.
- Einreihen: `ids_fahrt.py` (jedes 2. Bild, ohne Standbilder) → `comfy/ids_laufNN.txt`, neuer
  `stage1_p11/laufNN.sh` (Standard-Prompt T1, Stage2/3, am Ende `sichten.sh` je Fahrt), wartet auf den zuletzt
  gestarteten laufenden `laufNN.sh`. Laufende Läufe nie abbrechen.

Danach dem Nutzer melden: Fahrten mit Bildzahl/Größe, „am Rider gelöscht“, laufNN und Bilderzahl. Läuft ComfyUI
gerade, Restzeit aus `tail -1 ~/floornet/stage1_p11/batchNN.log` des laufenden Laufs.

## 2. Auswerten (am PC, lokale Kopie)

```bash
.venv/Scripts/python .claude/skills/fahrt/analyse.py "<scratchpad>/fahrt/<fahrt>"
.venv/Scripts/python .claude/skills/fahrt/analyse.py "<scratchpad>/fahrt/<fahrt>" --spur --von 100 --bis 108
```

Erst die Übersicht (Statuswechsel, Ereignisse: „bremse“, „Freiraum!“, Sperrzone, „komme nicht weiter“; Stecken =
Fahrwert > 0,08 bei Rädern < 0,05 m/s; Nick-Spitzen), dann das Ereignisbild `<fahrt>_ereignisse.jpg` mit Read
ansehen, dann mit `--spur` die Sekunden vor der Stelle, nach der der Nutzer fragt. Dabei klären:

- **Fahrwert gegen Räder:** Folgt er dem Fahrwert? (Rider setzt kleinere Werte erst nach ~1–2 s um, 0 in ~0,3 s.)
  „Bahn …m, bremse“ = Vollbremsung der Bahnprüfung (seit 2026-11-23); wie weit danach bis Stillstand (Pose).
- **Wann tauchte das Hindernis in der Bahnprüfung auf** (erstes „Bahn …m“) und ab wann war es im Bild sichtbar?
  Eigene Montage aus Bildern im Abstand ~0,5–1 s davor (cv2, 320×240 je Kachel), mit Read ansehen.
- **Bodenmodell:** Sieht man das Hindernis im Bild, aber die Bahn meldet es spät/nie, die Bodengrenze des
  Rider-Netzes zeichnen (Schritt 3).

## 3. Bodengrenze des Rider-Netzes (kimmy, CPU)

Erst prüfen, dass `~/floornet/lauf_rider` auf das Netz zeigt, das am Rider läuft:
`ssh root@<rider> readlink -f /opt/xgonav/models/floornet.bin` → `…/p11j.bin` ⇒ `lauf_p11j`. Weicht der Link ab:
`ln -sfn lauf_<netz> ~/floornet/lauf_rider` und `nohup nice ~/floornet/sweep/sichten.sh --alle &` (Regel
„Sichten mit Rider-Netz“).

```bash
ssh … aeinstein@kimmy.intern.itnox.de 'cd ~/floornet && CUDA_VISIBLE_DEVICES= .venv/bin/python sweep/netzbild.py lauf_rider <fahrt> /tmp/netz.jpg 4740 4763 4783 4799'
scp … aeinstein@kimmy.intern.itnox.de:/tmp/netz.jpg "<scratchpad>/"
```

`netzbild.py` liegt auch hier im Skill (fehlt es auf kimmy: nach `~/floornet/sweep/` kopieren). Grüne Linie =
Bodengrenze je Spalte (ohne Roll-Ausgleich und Glättung des Riders). Bildnummern = Dateinamen der Fahrt.

## 4. Bericht an den Nutzer

Kurz, deutsch, mit Zeiten (s ab Fahrtbeginn) und Bildnummern: was passiert ist, Ursache (Regelung/Bahnprüfung/
Bodenmodell/Planung), Belege (Tabelle Fahrwert/Räder/Status oder Netzbild), Vorschlag samt Kosten am Pi. Nichts
am Rider ändern oder fahren ohne Auftrag. Fahrten nur nach Sichtung durch den Nutzer ins Training.
