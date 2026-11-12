# CLAUDE.md – web/

Web-Infos aus der Root-`CLAUDE.md`, hier gebündelt. Englische Doku: `docs/web-interface.md`.

## Zweck und Aufbau

Weboberfläche des RiderPi in PHP (`web/index.php`, `web/api.php`) hinter nginx (`deploy/nginx-xgonav.conf`, eigener php-fpm-Pool `deploy/php-fpm-xgonav.conf`). Der Nutzer bevorzugt PHP fürs Web; Python nur für die Bildverarbeitung. Weitere Dateien: `geschichte.html`, `tagebuch/`, `rider-model.bin` (CAD-Modell).

## Am RiderPi

- nginx mit TLS auf 443 → `/xgonav/`; Port 80 und `/` leiten dorthin um. Zertifikat und Schlüssel unter `/etc/xgonav/tls`.
- Bilder per WebSocket `/xgonav/ws` direkt von xgonav, mit Bestätigung je Bild.
- Laufzeitdaten (CPU, WLAN, IMU, xgonav-Status) ohne Polling per WebSocket `/xgonav/live` vom PHP-CLI-Dienst `tools/live.php` (`xgonav-live.service`, Port 8089; 1 s, IMU 0,3 s balancierend); schläft ohne Browser.
- Emotionen erzeugt `tools/eyes.py` nach `/opt/xgonav/emotions`.
- Einstellungen (Modal im Kasten „System“): `api.php?a=config|config_save|config_restore|restart`, Zeilen-Editor `konfig.inc.php`, Schema `config-schema.json` (nach Änderungen an `xgonav/config.py`: `python tools/config_schema.py`, sonst schlägt `test_config_schema_aktuell` fehl). Sicherungen in `/var/lib/xgonav/config-backup`, `config.toml` Gruppe video 0664, sudoers `deploy/sudoers-xgonav-restart`.

## Schnittstelle zu xgonav

- `xgonav/web.py` ist nur noch die lokale Schnittstelle (`web_host = 127.0.0.1`), ohne Authentifizierung.
- Sie liest nur aus `App.snapshot` (neues `Snapshot` je Frame, nie mutiert); `render()` zeichnet das Overlay erst auf Anfrage.

## Regeln

- Rechenzeit am Rider hat oberste Priorität: Anzeige und Komfort im Browser rechnen, nie in der Bildpipeline oder je Bild am CM4. In jedem Plan die Kosten am Pi nennen.
- `api.php` schreibt `action` direkt nach `/proc`; wer sonst 254 schreibt, ohne vorher `POST /stop` zu rufen, lässt den Rider nach einer Ohnmacht (Modus `pet`) wieder aufspringen.
- Höhenänderungen aus dem Web liest `Robot` alle 0,2 s aus `/proc/XGORider/height` mit; gefahren wird nur u8 55..200, Körperhöhe nie unter Minimum + 10 mm.

## Befehle

```bash
sudo ./deploy/install-all.sh                     # auf dem Rider: alles (Treiber, Display, rc.local, xgonav, Web)
python tools/step_to_model.py --cache rider.npz  # nur PC (pip install cadquery-ocp fast-simplification): docs/Rider-Pi.STEP -> web/rider-model.bin
```
