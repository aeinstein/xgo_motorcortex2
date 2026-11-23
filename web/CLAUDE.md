# CLAUDE.md – web/

Web-Infos aus der Root-`CLAUDE.md`, hier gebündelt. Englische Doku: `docs/web-interface.md`.

## Zweck und Aufbau

Weboberfläche des RiderPi in PHP (`web/index.php`, `web/api.php`) hinter nginx (`deploy/nginx-xgonav.conf`, eigener php-fpm-Pool `deploy/php-fpm-xgonav.conf`). Der Nutzer bevorzugt PHP fürs Web; Python nur für die Bildverarbeitung. Weitere Dateien: `geschichte.html`, `tagebuch/`, `rider-model.bin` (CAD-Modell).

## Am RiderPi

- nginx mit TLS auf 443, Oberfläche direkt unter `/` (`root /opt/xgonav/web`); Port 80 leitet dorthin um, alte Adressen `/xgonav/…` schreibt nginx intern auf `/…` um. Zertifikat und Schlüssel unter `/etc/xgonav/tls`.
- Bilder per WebSocket `/ws` direkt von xgonav, mit Bestätigung je Bild.
- Laufzeitdaten (CPU, WLAN, IMU, xgonav-Status) ohne Polling per WebSocket `/live` vom PHP-CLI-Dienst `tools/live.php` (`xgonav-live.service`, Port 8089; 1 s, IMU 0,3 s balancierend); schläft ohne Browser.
- Gegensprechen per WebSocket `/sprech` vom PHP-CLI-Dienst `tools/sprech.php` (`xgonav-sprech.service`, Port 8090, `www-data` + Gruppe `audio`): nur im Modus `manual`, 8 kHz mono in beide Richtungen über `arecord`/`aplay` auf `plughw:0,0`, nur während eines Gesprächs; Wandeln, Echo-Dämpfung, Pegel im Browser. Gemeinsame WebSocket-Helfer `tools/ws.inc.php`. Dazu „Kamera senden“: zweite WebSocket `/sprech?bild`, der Browser schickt fertige RGB565-Bilder 320×240, der Dienst reicht sie ins Display-FIFO `/tmp/lcd0` (Besitz über `/tmp/lcd0.owner` wie `tools/emotion.sh`, am Ende Emotion „cute“). Seit 2026-11-21 am Rider.
- Emotionen erzeugt `tools/eyes.py` nach `/opt/xgonav/emotions`.
- MCP-Server `mcp.php` (`docs/mcp.md`, `tests/test_mcp.py` braucht php): Werkzeuge für ein multimodales LLM (Bild, Karte, Status, drehen, fahren), zustandslos per POST, Freigabestufe `schauen|drehen|fahren` in `/etc/xgonav/mcp.json` (Standard schauen). `fahren`/`folge` fahren standardmäßig geprüft (`/drive?sicher=1`: `safety()` + Bahnprüfung, |v| ≤ 0,30, |w| ≤ 0,60), mit `geprueft: false` wie das Steuerkreuz (bis 1, nur auf Wunsch des Besitzers); v, w sind seit 2026-11-23 Anteile von `max_v`/`max_w`; `folge` = Abschnitte ohne Halt, `drehen` bis ±1080° mit Rate `w`; jede Bewegung endet mit `/stop`. Tests liefen am Rider (kein php am PC) in einem Wegwerf-venv. Neue Werkzeuge: Eintrag in `werkzeuge()` + Funktion `w_<name>`. Am Rider eingespielt 2026-11-20 (Stufe fahren; lesende Werkzeuge und Wecken geprüft), über MCP nie gefahren.
- Kachel „Samples“: gelernte Stimmen des Modus pet (`api.php?a=stimmen|stimme_wav` direkt von `/opt/xgonav/stimmen`, `stimme_spielen`/`stimme_loeschen` über xgonav `POST /stimme/…`), nur beim Aufklappen geladen, Wiedergabe im Browser.
- Einstellungen (Modal im Kasten „System“): `api.php?a=config|config_save|config_restore|restart`, Zeilen-Editor `konfig.inc.php`, Schema `config-schema.json` (nach Änderungen an `xgonav/config.py`: `python tools/config_schema.py`, sonst schlägt `test_config_schema_aktuell` fehl). Sicherungen in `/var/lib/xgonav/config-backup`, `config.toml` Gruppe video 0664, sudoers `deploy/sudoers-xgonav-restart`.

## Schnittstelle zu xgonav

- `xgonav/web.py` ist nur noch die lokale Schnittstelle (`web_host = 127.0.0.1`), ohne Authentifizierung.
- Sie liest nur aus `App.snapshot` (neues `Snapshot` je Frame, nie mutiert); `render()` zeichnet das Overlay erst auf Anfrage.

## Regeln

- `schaubild.html` (Datenfluss, Takte, Kosten, Korrekturen) bei jeder Änderung mitpflegen, die es betreffen könnte, ungefragt. Kein Generator: SVG-Text von Hand, Kastenzeilen ≤ ~38 Zeichen.

- Rechenzeit am Rider hat oberste Priorität: Anzeige und Komfort im Browser rechnen, nie in der Bildpipeline oder je Bild am CM4. In jedem Plan die Kosten am Pi nennen.
- `api.php` schreibt `action` direkt nach `/proc`; wer sonst 254 schreibt, ohne vorher `POST /stop` zu rufen, lässt den Rider nach einer Ohnmacht (Modus `pet`) wieder aufspringen.
- Aktionen 3–5 fahren bzw. drehen über den Fahr-Sollwert der Firmware; der Keepalive von xgonav hielte sie an. `api.php` gibt sie deshalb an xgonav `POST /action` (nur Modus `idle`, Keepalive und Kernel-Watchdog so lange aus, Bahnprüfung je Regelschritt, sonst Abbruch); am Rider nie gefahren.
- Höhenänderungen aus dem Web liest `Robot` alle 0,2 s aus `/proc/XGORider/height` mit; gefahren wird nur u8 55..200, Körperhöhe nie unter Minimum + 10 mm.

## Befehle

```bash
sudo ./deploy/install-all.sh                     # auf dem Rider: alles (Treiber, Display, rc.local, xgonav, Web)
python tools/step_to_model.py --cache rider.npz  # nur PC (pip install cadquery-ocp fast-simplification): docs/Rider-Pi.STEP -> web/rider-model.bin
```
