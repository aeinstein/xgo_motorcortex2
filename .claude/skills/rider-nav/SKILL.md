---
name: rider-nav
description: Den echten XGO Rider (RiderPi) über die laufende xgonav-Instanz steuern – Bildverarbeitung starten, aus dem Liegen aufspringen (nach Platzprüfung), Kalibrierung abwarten, Fahr-/Drehbefehle, danach absetzen und Bildverarbeitung aus. Verwenden, wenn der Nutzer den Rider fahren, drehen, aufstehen oder absetzen lassen will.
---

# rider-nav: Rider über xgonav steuern

Gesteuert wird nur über die **laufende** xgonav-Instanz (lokale Schnittstelle `127.0.0.1:8088` am Rider) und
`/proc/XGORider/action` (253 aufspringen, 254 absetzen). Nie parallel direkt `speed_x`/`speed_z` schreiben
(`tools/nudge.py` nur bei gestopptem xgonav). Werkzeug: `rider_nav.py` in diesem Ordner, läuft am Rider als root.

## Zugang

```bash
R="ssh -o IdentitiesOnly=yes -i /c/Users/aeinstein.ITNOX/Documents/aeinstein-ed.pem root@192.168.53.110"
scp -o IdentitiesOnly=yes -i /c/Users/aeinstein.ITNOX/Documents/aeinstein-ed.pem \
    .claude/skills/rider-nav/rider_nav.py root@192.168.53.110:/root/rider_nav.py     # einmal je Sitzung/Änderung
$R python3 /root/rider_nav.py zustand
```

## Vorbedingungen (vor jedem Ablauf)

- **Freigabe:** Bewegung nur, wenn der Nutzer sie in diesem Gespräch angesagt hat und daneben steht. Hat er
  gesagt, dass der Rider lädt oder nicht bewegt werden darf: vorher fragen, auch für Aufstehen.
- `zustand` lesen: Akku ≥ 20 % (sonst nicht fahren), `xgonav.service` active (sonst Nutzer fragen, nicht selbst
  neu starten).
- Benutzt der Nutzer ihn gerade (Modus ≠ idle, z. B. manual/follow im Web): nicht eingreifen, fragen.
- Autonome Modi (`explore`, `home`, `patrol`, `approach`) nur nach Frage nach Treppen/Kanten (kein Absturzschutz).
  Dieser Skill fährt nur Handsteuerung (`fahr`, `dreh`).

## Ablauf

1. **Bildverarbeitung an, falls aus:** `$R python3 /root/rider_nav.py start` (wartet auf neue Bilder; nie
   `/pipeline` während eines Dienst-Neustarts schicken).
2. **Liegt er (`balanciert: false`): Platz prüfen.**
   - `$R python3 /root/rider_nav.py bild /tmp/rn.jpg`, dann per scp ins Scratchpad holen und mit Read ansehen.
   - Beim Aufspringen rollt er ~20 cm vor und schwingt hoch: vorne ≥ 40 cm frei, keine Kabel, Kanten, Stufen,
     Möbel direkt daneben. Hinten und seitlich sieht die Kamera nicht → ist das nicht eindeutig, Nutzer fragen.
   - Steht er schon: Schritt 2–3 überspringen (kein 253 an einen balancierenden Rider).
3. **Aufspringen + Kalibrierung abwarten:** `$R python3 /root/rider_nav.py aufstehen`
   - schreibt 253, wartet bis `state` = 1, dann mindestens 6 s (Aufschwung ≤ 2 s, Nivellier-Impuls 2,5–5,5 s)
     und bei `[horizon]` bis die Standreferenz `gültig` ist (3 s Einschwingen + 3 s Mittel im Stillstand).
   - In dieser Zeit nichts schicken. Körperhöhe steht danach auf Firmware-Mitte 128 (≈ 90 mm); Höhe nur auf
     Ansage ändern, nie unter Minimum + 10 mm (u8 ≥ 55 wird gefahren), danach kalibriert die Standreferenz neu.
4. **Fahrbefehle** (je einzeln, Ergebnis dem Nutzer melden):
   - `fahr V W SEK` – normiert, v > 0 vor, w > 0 links; schickt `/drive` mit 10 Hz (manual hält nur 0,5 s),
     endet mit Stillstand + idle. Richtwerte: v 0,08 ≈ 10 cm/s, v 0,12 ≈ 26 cm/s (nichtlinear); xgonav begrenzt auf
     `max_v`/`max_w` der Konfig (Beispiel 0,15/0,30). Rückwärts wirkt mit `reverse_gain`.
   - `dreh GRAD` – + = links, beliebig groß (auch `360`, `-360`, `720`): summiert die Kursänderung aus `yaw_deg`
     in `/status` auf (Quelle `yaw_src`, Räder bevorzugt), Toleranz 3°, Zeitlimit 10 s + 1 s je 20°. Er dreht nach
     dem Stopp etwas nach (gemessen bis ~5°); für „wieder auf Startkurs“ danach mit kleinem `dreh` korrigieren.
   - Handsteuerung prüft **kein** Bodenmodell und keine Bahn: Strecken klein halten, Richtung vorher ansagen.
   - Abbruch des Nutzers („stopp“, Tool abgelehnt): sofort `$R python3 /root/rider_nav.py absetzen` bzw. bei
     abgebrochenem SSH prüfen, ob `rider_nav.py` am Rider noch läuft (`pkill -f "[r]ider_nav.py"`, danach `/stop`).
5. **Danach immer:** `$R python3 /root/rider_nav.py ende` – idle, 254 (absetzen), wartet bis er liegt, dann
   Bildverarbeitung aus (Akku schonen). Nur weglassen, wenn der Nutzer ausdrücklich weiterfahren will.

## Aufnahme

- Mitschnitt nur, wenn gewünscht: nach dem Aufstehen im Hintergrund
  `$R "nohup python3 /opt/xgonav/tools/record_status.py /tmp/rn-aufnahme-$(date +%H%M%S) 900 > /tmp/rn-aufnahme.log 2>&1 &"`
  (liegt das Werkzeug dort nicht: aus `tools/record_status.py` per scp nach `/root/` und von dort starten).
- Rohbilder für die Auswertung (`meta.jsonl`, FloorNet) stattdessen über den Knopf „Aufzeichnen“ im Web bzw.
  `POST /record?on=1|0` (`xgonav/rawrec.py`, schreibt nur balancierend). Nach `ende` wieder ausschalten, wenn gewünscht.
- **Absetzen beendet die Aufnahme**: `record_status.py` endet selbst, sobald `state` ≠ 1, und startet nicht,
  wenn er liegt. Zweitleser nur über `--neben "…"`. Eigene Mitschnitte genauso an das Absetzen koppeln.
- Danach Aufnahme per `tar`/scp ins Scratchpad holen, am Rider nur löschen, wenn der Nutzer zustimmt.

## Kosten am Pi

`rider_nav.py` macht nur HTTP-Abfragen (`/status`, `/drive`) mit 10 Hz während einer Fahrt, sonst nichts; kein
Eingriff in die Bildpipeline. `bild` kodiert ein JPEG (einmalig). `record_status.py` holt jedes Overlay-Bild
(Rendern + JPEG je Bild in xgonav) – nur bei Bedarf.

## Schnittstelle (Referenz)

| Zweck | Aufruf am Rider |
|---|---|
| Zustand | `GET /status` (mode, pipeline, frame, yaw_deg, odom, height_mm, horizon.zustand); `/proc/XGORider/state` 1 = balanciert, `battery` % |
| Bildverarbeitung | `POST /pipeline?on=1` / `?on=0` (aus = idle + Stillstand) |
| Modus | `POST /mode?name=idle\|manual\|follow…`, `POST /stop` = idle + Stillstand |
| Handsteuerung | `POST /drive?v=&w=` nur im Modus manual, alle < 0,5 s wiederholen |
| Bild | `GET /frame?raw=1` (Rohbild), ohne raw mit Overlay |
| Aufspringen / Absetzen | `echo 253 > /proc/XGORider/action` / `echo 254 > …` |
| Neu ausrichten (Roll) | `POST /level` (nur balancierend) |
