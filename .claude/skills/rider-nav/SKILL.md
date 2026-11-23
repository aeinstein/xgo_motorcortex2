---
name: rider-nav
description: Den echten XGO Rider (RiderPi) über die laufende xgonav-Instanz steuern – über den MCP-Server rider (Status, Bild, Bildverarbeitung, Aufstehen, Drehen, Fahren geprüft oder von Hand, Abschnittsfolgen, Absetzen); SSH/rider_nav.py nur als Rückfall und für Aufnahmen. Verwenden, wenn der Nutzer den Rider fahren, drehen, aufstehen oder absetzen lassen will.
---

# rider-nav: Rider über xgonav steuern

Gesteuert wird nur über die **laufende** xgonav-Instanz. **Erster Weg: MCP-Server `rider`** (Werkzeuge
`mcp__rider__*`, `web/mcp.php`, `docs/mcp.md`; in Claude Code im Nutzerbereich eingetragen, Freigabestufe in
`/etc/xgonav/mcp.json`). Kürzer als SSH, keine Kopie des Werkzeugs nötig. Schemas bei Bedarf per
`ToolSearch("select:mcp__rider__status,…")` laden.

`mcp.php` läuft wie `rider_nav.py` am Rider selbst (php-fpm, `127.0.0.1:8088`, 10 Hz): gleiche Genauigkeit, kein
Grund für SSH. **Rückfall: SSH + `rider_nav.py`** (dieser Ordner, als root) nur, wenn MCP nicht antwortet, und für
Aufnahmen (bietet MCP nicht an).
Nie parallel direkt `speed_x`/`speed_z` schreiben (`tools/nudge.py` nur bei gestopptem xgonav).

## Zugang SSH (Rückfall, Aufnahmen, Einspielen)

Adresse wechselt: zuerst `192.168.53.110`, sonst `10.10.0.110` (antwortet der MCP-Server nicht, liegt es oft daran;
dann den Eintrag `claude mcp remove rider -s user` + `claude mcp add --transport http -s user rider https://<ip>/mcp.php`).

```bash
R="ssh -o IdentitiesOnly=yes -i /c/Users/aeinstein.ITNOX/Documents/aeinstein-ed.pem root@192.168.53.110"
scp -o IdentitiesOnly=yes -i /c/Users/aeinstein.ITNOX/Documents/aeinstein-ed.pem \
    .claude/skills/rider-nav/rider_nav.py root@192.168.53.110:/root/rider_nav.py     # einmal je Sitzung/Änderung
$R python3 /root/rider_nav.py zustand
```

## Vorbedingungen (vor jedem Ablauf)

- **Freigabe:** Bewegung nur, wenn der Nutzer sie in diesem Gespräch angesagt hat und daneben steht. Hat er
  gesagt, dass der Rider lädt oder nicht bewegt werden darf: vorher fragen, auch für Aufstehen. Die MCP-Stufe
  `fahren` ist keine Freigabe des Nutzers.
- `mcp__rider__status` lesen: Akku ≥ 20 % (sonst nicht fahren), `ruhemodus` (dann erst `mcp__rider__ruhemodus`
  mit `an: false`). Antwortet xgonav nicht: Nutzer fragen, nicht selbst neu starten.
- Benutzt der Nutzer ihn gerade (Modus ≠ idle, z. B. manual/follow im Web): nicht eingreifen, fragen.
- Meldet ein Werkzeug „Nicht freigegeben“: Stufe ist gesperrt, dem Nutzer sagen, nicht über SSH umgehen.

## Ablauf (MCP)

1. **Bildverarbeitung an, falls aus:** `mcp__rider__bildverarbeitung` mit `an: true` (wartet auf Bilder, ≤ 20 s).
2. **Liegt er (`balanciert: false`): Platz prüfen.**
   - `mcp__rider__bild` (Rohbild; `overlay: true` mit Bodengrenze) ansehen.
   - Beim Aufspringen rollt er ~20 cm vor und schwingt hoch: vorne ≥ 40 cm frei, keine Kabel, Kanten,
     Möbel direkt daneben. Hinten und seitlich sieht die Kamera nicht → ist das nicht eindeutig, Nutzer fragen.
   - Steht er schon: Schritt 2–3 überspringen (kein 253 an einen balancierenden Rider).
3. **Aufspringen + Kalibrierung abwarten:** `mcp__rider__aufstehen` (`/stop`, 253, wartet bis balanciert ≤ 8 s,
   dann 6 s Nivellieren; braucht Bildverarbeitung und Akku ≥ 20 %).
   - Danach steht er in `idle`. Soll er in einem Modus laufen (z. B. `pet`): `mcp__rider__modus` mit `name`.
   - Körperhöhe steht danach auf Firmware-Mitte 128 (≈ 90 mm); Höhe nur auf Ansage (`mcp__rider__hoehe`, u8
     61..200), nie unter Minimum + 10 mm. Kamerahöhe ≈ 109,4 + 0,1126·u8 mm.
4. **Bewegen** (je einzeln, Ergebnis mit Bild dem Nutzer melden):
   - `mcp__rider__drehen` `grad` −1080..1080 (+ = links), `w` Drehrate 0,16..1 (Anteil von max_w, Standard 0,5), `tol` 1..10°
     (Standard 3), geregelt in xgonav (`POST /turn`, seit 2026-11-23), danach Bild. Er dreht nach dem Stopp etwas nach (bis ~5°); für „wieder
     auf Startkurs“ mit kleinem `drehen` korrigieren.
   - `mcp__rider__rundblick` `bilder` 2..8: volle Drehung, je Schritt ein Bild, endet auf dem Startkurs.
   - `mcp__rider__fahren` `v`, `w`, `sekunden` 0,2..5: Standard **geprüft** (`/drive?sicher=1`: Bodengrenze,
     `safety()`, Bahnprüfung; |v| ≤ 0,30, |w| ≤ 0,60 (Anteile von max_v/max_w), braucht Bildverarbeitung und
     FloorNet). `geprueft: false` = Handsteuerung wie das Steuerkreuz im Web und früher `rider_nav.py fahr`
     (ohne Bodengrenze und Bahnprüfung, |v|, |w| ≤ 1 = max_v/max_w): nur, wenn der Nutzer es so
     will oder die Prüfung an einer Engstelle bremst und er zustimmt; Richtung vorher ansagen, Strecken klein.
     Seit 2026-11-23 sind v, w Anteile von max_v/max_w (0,5), gesendet v · tempo · max_v. Richtwerte bei Tempo 100 %:
     v 0,16 ≈ 10 cm/s, v 0,24 ≈ 26 cm/s (nichtlinear); Nenntempo ≈ v · 0,5 · 1,8 m/s.
     Endet mit Stillstand + idle, liefert Strecke, Pose vorher/nachher, Bremsmeldungen, Bild. Kleine Schritte,
     nach jedem das Bild ansehen, nicht rückwärts ins Ungesehene. Der Temporegler (`mcp__rider__tempo`) gilt auch hier.
   - `mcp__rider__folge` `abschnitte` [[v, w, s], …] (1..10, je 0..5 s, zusammen ≤ 30 s), `geprueft` wie oben:
     Abschnitte ohne Halt dazwischen (z. B. `[[0.26, 0, 3], [0, 0.5, 1]]`: fahren und direkt aus der Fahrt drehen).
   - `mcp__rider__ausrichten`: Roll auf 0° nivellieren (nur balancierend); `mcp__rider__neigen` Roll −20..20°.
   - Abbruch des Nutzers („stopp“, Tool abgelehnt): sofort `mcp__rider__stopp`, danach `mcp__rider__absetzen`.
5. **Danach immer:** `mcp__rider__absetzen` (wartet bis er liegt), dann `mcp__rider__bildverarbeitung` mit
   `an: false` (Akku schonen). Nur weglassen, wenn der Nutzer ausdrücklich weiterfahren will.

## Rückfall SSH / rider_nav.py (nur wenn MCP nicht antwortet)

`$R python3 /root/rider_nav.py zustand|start|bild|aufstehen|fahr V W SEK|folge V,W,SEK …|dreh GRAD|hoehe U8|absetzen|ende`
– dieselben Abläufe wie oben; `fahr`/`folge` fahren dort immer **ungeprüft**.

- Beginnt ein Wert mit Minus, `--` davorsetzen (`folge -- -0.26,0,2.5`, `dreh -- -90`), sonst liest der Aufruf
  ihn als Option und bricht ohne Bewegung ab. Optionen gehören vor das `--` (`dreh --w 0.5 -- -90`).
- `ende` = idle, 254, warten bis er liegt, Bildverarbeitung aus.
- Abgebrochenes SSH: prüfen, ob `rider_nav.py` am Rider noch läuft (`pkill -f "[r]ider_nav.py"`), danach
  `mcp__rider__stopp`.

## Aufnahme (SSH)

- Mitschnitt nur, wenn gewünscht: nach dem Aufstehen im Hintergrund
  `$R "nohup python3 /opt/xgonav/tools/record_status.py /tmp/rn-aufnahme-$(date +%H%M%S) 900 > /tmp/rn-aufnahme.log 2>&1 &"`
  (liegt das Werkzeug dort nicht: aus `tools/record_status.py` per scp nach `/root/` und von dort starten).
- Rohbilder für die Auswertung (`meta.jsonl`, FloorNet) stattdessen über die Checkbox „Aufzeichnen“ im Web bzw.
  `POST /record?on=1|0` (`xgonav/rawrec.py`, schreibt nur balancierend; MCP bietet das nicht an).
- **Absetzen beendet die Aufnahme**: `record_status.py` endet selbst, sobald `state` ≠ 1, und startet nicht,
  wenn er liegt. Zweitleser nur über `--neben "…"`. Eigene Mitschnitte genauso an das Absetzen koppeln.
- Danach Aufnahme per `tar`/scp ins Scratchpad holen, am Rider nur löschen, wenn der Nutzer zustimmt.

## Kosten am Pi

MCP: je Aufruf ein php-fpm-Lauf mit HTTP-Abfragen an xgonav; `fahren` schickt `/drive` mit 10 Hz, `bild` holt ein
Rohbild (`/frame?raw=1`, ein JPEG). `rider_nav.py` genauso (HTTP, 10 Hz beim Fahren). Kein Eingriff in die
Bildpipeline. `record_status.py` holt jedes Overlay-Bild (Rendern + JPEG je Bild in xgonav) – nur bei Bedarf.

## Schnittstelle von xgonav (Referenz, für rider_nav.py)

| Zweck | Aufruf am Rider |
|---|---|
| Zustand | `GET /status` (mode, pipeline, frame, yaw_deg, odom, height_mm); `/proc/XGORider/state` 1 = balanciert, `battery` % |
| Bildverarbeitung | `POST /pipeline?on=1` / `?on=0` (aus = idle + Stillstand) |
| Modus | `POST /mode?name=idle\|manual\|follow…`, `POST /stop` = idle + Stillstand |
| Handsteuerung | `POST /drive?v=&w=` (`&sicher=1`: mit Bodengrenze und Bahnprüfung) nur im Modus manual, alle < 0,5 s wiederholen |
| Bild | `GET /frame?raw=1` (Rohbild), ohne raw mit Overlay |
| Aufspringen / Absetzen | `echo 253 > /proc/XGORider/action` / `echo 254 > …` |
| Neu ausrichten (Roll) | `POST /level` (nur balancierend) |
