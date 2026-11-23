<?php
/*
 * xgonav – Bedienoberfläche. Bild: WebSocket ws?nick=3 (nginx -> xgonav /ws, Bestätigung je Bild),
 * Raumkarte: map.png, Status und Befehle: api.php. Dazu Handsteuerung, Höhe, Aktionen, LEDs,
 * Emotionen, Herunterfahren. Keine Authentifizierung.
 *
 * Aufbau: Kopfleiste (fest oben: Sprache, Absetzen, Aufspringen, Aufstehen, Bildverarbeitung an/aus, Aufzeichnen,
 * Einstellungen, Ruhe, STOPP) – links Bild mit Steuerkreuz als Auflage und Kameraleiste, Modus, Kennzahlen, Verlauf
 * (eingeklappt) – rechts Lage, Schallradar, Extras (Anleitung, Schaubild, Herunterfahren, Aktionen, LEDs, Emotionen),
 * Raumkarte; Tempo in der Modusleiste.
 * Sprache: Deutsch steht im HTML, Englisch im Wörterbuch EN (nur im Browser umgeschaltet).
 */
declare(strict_types=1);
header('Cache-Control: no-store');

const EMOTION_RAW = '/opt/xgonav/emotions';             // tools/eyes.py, von deploy/install.sh erzeugt
$host = htmlspecialchars(gethostname() ?: 'rider', ENT_QUOTES);
$emotionen = [];
foreach (@scandir(EMOTION_RAW) ?: [] as $e) {
    if (preg_match('/^[A-Za-z]+$/', $e) && is_dir(EMOTION_RAW . '/' . $e)) {
        $emotionen[] = $e;
    }
}
// LED-Helligkeit n/256 (api.php speichert sie, rc.local und xgonav setzen sie), Anzeige in Prozent; ohne Datei 20 %
$ledN = @file_get_contents('/var/lib/xgonav/led_brightness');
$ledP = ($ledN !== false && is_numeric(trim($ledN))) ? (int)round(max(0, min(255, (int)trim($ledN))) * 100 / 255) : 20;
function h(string $s): string { return htmlspecialchars($s, ENT_QUOTES); }
?><!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark">
<title>xgonav – <?= $host ?></title>
<style>
:root{
  --bg:#0f1114; --karte:#181b20; --karte2:#1f232a; --rand:#2b3038; --rand2:#3a414c;
  --text:#e3e6ea; --leise:#8b94a1; --akzent:#4fa3ff; --gut:#3fb56b; --warn:#e0a63a; --gefahr:#e5484d;
  --r:8px;
}
*{box-sizing:border-box}
html{background:var(--bg)}
body{font:14px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif;color:var(--text);margin:0}
code{font-size:12px;color:#c7cdd6}

/* --- Kopfleiste: fest oben, damit STOPP ohne Scrollen erreichbar ist; die Höhe (bricht um) misst
   kopfHoehe() im Browser und schiebt den Inhalt per --kopf-h darunter */
header{position:fixed;top:0;left:0;right:0;z-index:10;display:flex;align-items:center;gap:10px;flex-wrap:wrap;
  padding:8px 12px;background:rgba(15,17,20,.92);backdrop-filter:blur(6px);border-bottom:1px solid var(--rand)}
body{padding-top:var(--kopf-h,52px)}
html{scroll-padding-top:var(--kopf-h,52px)}
@media (hover:hover){#rec.an:hover{background:#ff2b30;border-color:#ff6b6e}}
.kopf-knoepfe{display:flex;align-items:center;gap:6px;flex-wrap:wrap}
/* Symbolknöpfe im Header (immer nur Symbol, gleiche Höhe wie die Textknöpfe) */
.kopf-ic{font-size:15px;line-height:1;padding:3px 7px}
#rec{color:var(--gefahr)}
#rec.an{background:var(--gefahr);border-color:var(--gefahr);color:#fff}   /* Aufnahme läuft: weiß auf rot */
header h1{font-size:15px;font-weight:600;margin:0 4px 0 0;white-space:nowrap}
header h1 span{color:var(--leise);font-weight:400}
@media (max-width:640px){
  header{gap:6px 8px;padding:6px 8px}
  header h1{flex:1}
  header .pillen{order:3;flex:0 0 100%;gap:4px}
  .kopf-zeile2{order:4}
  header .pille{padding:1px 7px}
  .pille .lang{display:none}
}
.sprache{display:inline-flex;border:1px solid var(--rand2);border-radius:6px;overflow:hidden}
.sprache button{border:0;border-radius:0;padding:3px 8px;font-size:12px;background:var(--karte2);color:var(--leise)}
.sprache button.aktiv{background:var(--akzent);color:#04121f;font-weight:600}
.pillen{display:flex;gap:6px;flex-wrap:wrap;flex:1;min-width:0}
.pille{display:inline-flex;align-items:center;gap:5px;padding:2px 9px;border-radius:999px;font-size:12px;
  background:var(--karte2);border:1px solid var(--rand);color:var(--leise);white-space:nowrap;font-variant-numeric:tabular-nums}
.pille b{color:var(--text);font-weight:500}
.pille.gut{border-color:#2d6b45}.pille.gut b{color:#7ddc9c}
.pille.warn{border-color:#7a5a20}.pille.warn b{color:#f0c46a}
.pille.schlecht{border-color:#7a2a2d}.pille.schlecht b{color:#ff8a8d}
.punkt{width:8px;height:8px;border-radius:50%;background:var(--leise)}
.gut .punkt{background:var(--gut)}.schlecht .punkt{background:var(--gefahr)}
#meldung{background:#3a1517;color:#ffb3b5;border-bottom:1px solid #6b2427;padding:5px 12px;font-size:13px}
#meldung:empty{display:none}

/* --- Knöpfe */
/* Einheitsgröße aller Textknöpfe (wie „ESP-Neustart“); Ausnahmen: STOPP, Wecken, Steuerkreuz, Einstellungs-Navigation */
button{font:inherit;font-size:12px;padding:3px 8px;border-radius:6px;border:1px solid var(--rand2);background:var(--karte2);
  color:var(--text);touch-action:manipulation;cursor:pointer;transition:background-color .12s,border-color .12s,transform .05s}
/* Hover nur bei echter Maus: auf Touch-Geräten bliebe der Effekt sonst nach dem Tippen hängen */
@media (hover:hover){
  button:hover{background:#2a3038;border-color:#566070}
  button.aktiv:hover{background:#1f4166;border-color:var(--akzent)}
  #stop:hover{background:#ff2b30;border-color:#ff6b6e}
  .gefahr:hover{background:#4a1d20;border-color:#a33}
}
button:active{transform:translateY(1px);background:#333a44}
button:focus-visible,select:focus-visible,input:focus-visible,summary:focus-visible{outline:2px solid var(--akzent);outline-offset:1px}
button:disabled{cursor:not-allowed;opacity:.5}
button.aktiv{border-color:var(--akzent);background:#18324f;color:#fff}
#stop{background:var(--gefahr);border-color:var(--gefahr);color:#fff;font-weight:700;letter-spacing:.06em;padding:8px 22px;font-size:15px}
#stop:active{background:#b82e33}
/* --- Ruhemodus: deckt die Seite ab, nur Wecken; die Seite fragt dann nur noch alle 5 s den Status ab */
#ruhe-decke{position:fixed;inset:0;z-index:50;display:none;flex-direction:column;align-items:center;justify-content:center;
  gap:14px;background:rgba(10,11,13,.94);text-align:center;padding:16px}
#ruhe-decke.an{display:flex}
#ruhe-decke h2{margin:0;font-size:22px;font-weight:600}
#ruhe-decke p{margin:0;color:var(--leise);font-size:14px;max-width:420px}
#ruhe-decke button{padding:12px 34px;font-size:17px;font-weight:600}
.gefahr{border-color:#6b2a2d;color:#ffb3b5}
.knoepfe{display:flex;flex-wrap:wrap;gap:6px}
input,select{font:inherit;color:var(--text);background:var(--karte2);border:1px solid var(--rand2);border-radius:6px;padding:5px 8px}
input[type=range]{padding:0;border:0;background:none;accent-color:var(--akzent);width:100%;cursor:pointer;min-width:80px}
input[type=color]{width:100%;height:34px;padding:2px;cursor:pointer}
/* alle Auswahlboxen gleich groß, so hoch wie die Textknöpfe (12 px, 3 px oben/unten) */
select{cursor:pointer;font-size:12px;line-height:1.4;padding:2px 4px}

/* --- Raster */
main{display:grid;grid-template-columns:minmax(320px,640px) minmax(300px,1fr);gap:12px;padding:12px;align-items:start;max-width:1400px}
@media (max-width:900px){main{grid-template-columns:1fr;padding:8px;gap:8px}}
.spalte{display:grid;gap:12px;min-width:0}
@media (max-width:900px){.spalte{gap:8px}}

/* --- Karten */
.karte{background:var(--karte);border:1px solid var(--rand);border-radius:var(--r);padding:10px 12px}
.kopf{display:flex;flex-wrap:wrap;align-items:center;gap:8px;margin:0 0 8px;min-height:28px}
.kopf h2{font-size:13px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:#a9b4c2;margin:0;flex:1}
.leise{color:var(--leise);font-size:12px}
details.hilfe{margin-top:8px;font-size:12px;color:var(--leise)}
details.hilfe summary{cursor:pointer;list-style:none;display:inline-block;user-select:none}
details.hilfe summary::-webkit-details-marker{display:none}
details.hilfe summary::before{content:"i";display:inline-grid;place-items:center;width:14px;height:14px;margin-right:5px;
  border:1px solid currentColor;border-radius:50%;font:600 10px/1 system-ui,sans-serif;vertical-align:1px}
details.hilfe[open] summary{color:var(--text)}
details.hilfe p{margin:4px 0 0}

/* --- Bild und Modus */
#bild-rahmen{overflow:hidden;background:#000;border:1px solid var(--rand);border-radius:var(--r)}
.bild-fenster{overflow:hidden;position:relative}   /* „Bild festhalten“ verschiebt das <img>; die Leiste darunter bleibt frei */
/* Ein Raster für beide Regler (display:contents), Spalten je nach Breite des Bildrahmens: breit eine Zeile,
   enger Ist-Wert in eigener Zeile, schmal die Regler untereinander. Nichts darf unter seinen Inhalt schrumpfen. */
#bild-rahmen{container-type:inline-size}
.bild-leiste{display:grid;align-items:center;gap:6px 8px;padding:6px 10px;background:var(--karte);border-top:1px solid var(--rand);
  grid-template-columns:minmax(0,1fr)}
/* Kameraregler im Klappfeld: zwei Regler je Zeile, schmal untereinander */
.kamera-klapp>summary{cursor:pointer;user-select:none;display:flex;gap:8px;align-items:center;color:var(--leise);font-size:13px}
.kamera-klapp>summary .titel{color:var(--text)}
.kamera-klapp[open]>summary{margin-bottom:6px}
.kamera-raster{display:grid;align-items:center;gap:6px 8px;
  grid-template-columns:auto minmax(50px,1fr) auto auto auto minmax(50px,1fr) auto auto}
.kamera-raster .regler{display:contents}
.kamera-raster .regler:nth-child(even) label{margin-left:8px}
.kamera-raster .ausgleich{grid-column:1/-1;display:flex;gap:6px;align-items:center;justify-content:flex-end;flex-wrap:wrap}
.bild-leiste .w{white-space:nowrap;min-width:3.6em}
.bild-leiste input[type=range]{min-width:0}
.umschalt{font-size:15px;line-height:1;padding:3px 7px}   /* Symbol-Umschalter wie die Kopf-Symbole */
/* Anzeige-Umschalter als Auflage oben rechts im Kamerabild (wie das Steuerkreuz halbdurchsichtig) */
.bild-schalter{position:absolute;top:6px;right:6px;z-index:3;display:flex;gap:4px}
.bild-schalter .umschalt{background:rgba(15,17,20,.45);border-color:rgba(255,255,255,.3);color:#fff}
.bild-schalter .umschalt.aktiv{background:var(--akzent);border-color:var(--akzent);color:#04121f}
.bild-leiste .imu-bild{grid-column:1/-1;font-variant-numeric:tabular-nums}
.tempo-kopf{display:inline-flex;align-items:center;gap:6px}
.tempo-kopf label{color:var(--leise);font-size:13px}
.tempo-kopf input{width:120px;margin:0}
.tempo-kopf .w{font-variant-numeric:tabular-nums;text-align:right;font-size:13px;min-width:3.2em}
.bild-leiste .imu-bild.schlecht{color:#ff8a8d}
.sprech-zeile{margin-top:8px;display:flex;align-items:center;gap:8px;flex-wrap:wrap;font-size:13px}
.sprech-zeile label{color:var(--leise)}
.sprech-zeile meter{width:60px}
.sprech-zeile input[type=range]{width:90px}
.sprech-zeile video{width:64px;height:48px;object-fit:cover;transform:scaleX(-1);border-radius:4px;background:#000}
@container (max-width:560px){
  .kamera-raster{grid-template-columns:auto minmax(50px,1fr) auto auto}
  .kamera-raster .regler:nth-child(even) label{margin-left:0}
}
.info{display:inline-grid;place-items:center;width:15px;height:15px;border:1px solid currentColor;border-radius:50%;
  font:600 10px/1 system-ui,sans-serif;color:var(--leise);cursor:help}
#bild{width:100%;height:auto;aspect-ratio:4/3;object-fit:contain;display:block;background:#000}
#bild.aus{opacity:.35}
/* konfigurierte Bildmitte (Spalte W/2, Zeile H/2), wandert mit Verschieben/Drehen des Bildes */
#bild-mitte{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;overflow:visible}
#bild-mitte line,#bild-mitte circle{fill:none;vector-effect:non-scaling-stroke}
#bild-mitte .halo{stroke:#000;stroke-width:3;opacity:.6}
#bild-mitte .strich{stroke:#fff;stroke-width:1}
#bild-mitte .lot{stroke-dasharray:6 6}
#bild-mitte.aus{display:none}
/* Planerbahn im Kamerabild: projiziert im Browser (planImBild), gleiche CSS-Verschiebung/Drehung wie das <img> */
#bild-plan{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;overflow:visible}
#bild-plan polyline{fill:none;vector-effect:non-scaling-stroke;stroke-linejoin:round;stroke-linecap:round}
#bild-plan .halo{stroke:#000;stroke-width:5;opacity:.55}
#bild-plan .strich{stroke:#ffa53a;stroke-width:2.5}
#bild-plan circle{fill:#ffa53a;stroke:#000;stroke-width:1;vector-effect:non-scaling-stroke}
.modus{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.segment{display:inline-flex;border:1px solid var(--rand2);border-radius:7px;overflow:hidden;flex-wrap:wrap}
.segment button{border:0;border-radius:0;border-right:1px solid var(--rand2);margin:0}
.segment button:last-child{border-right:0}
.segment button.aktiv{background:var(--akzent);color:#04121f;font-weight:600}
#ziel{width:auto;max-width:150px}
.ziel-gruppe{flex-wrap:nowrap}
.ziel-gruppe #ziel{border:0;border-radius:0;margin:0}
@media (max-width:640px){
  .segment{display:grid;grid-template-columns:repeat(3,1fr);width:100%}
  .segment button{padding:3px 2px}
  .segment.ziel-gruppe{display:flex}
  .ziel-gruppe button{flex:1}
  #ziel{flex:2;max-width:none}
}

/* --- Kennzahlen */
.kacheln{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:6px}
.kachel{background:var(--karte2);border-radius:6px;padding:5px 8px;min-width:0}
.kachel .k{font-size:11px;color:var(--leise);text-transform:uppercase;letter-spacing:.03em}
.kachel .w{font-variant-numeric:tabular-nums;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.kachel.breit{grid-column:1/-1}
.kachel.breit .w{white-space:normal}

/* --- Fahren */
/* Aktionen klein in der Lage-Kachel */
/* Steuerkreuz als Auflage unten rechts im Kamerabild; nur die Knöpfe fangen Zeiger */
.pad{position:absolute;right:8px;bottom:8px;z-index:3;display:grid;grid-template-columns:repeat(3,42px);
  grid-template-rows:repeat(3,38px);gap:4px;user-select:none;-webkit-user-select:none;pointer-events:none}
.pad button{margin:0;padding:0;font-size:16px;border-radius:8px;pointer-events:auto;touch-action:none;
  background:rgba(15,17,20,.45);border:1px solid rgba(255,255,255,.3);color:#fff}
.pad button:hover{background:rgba(15,17,20,.65)}
.pad button.aktiv{background:var(--akzent);color:#04121f}
@media (pointer:coarse){.pad{grid-template-columns:repeat(3,52px);grid-template-rows:repeat(3,46px)}}
@container (max-width:420px){.pad{grid-template-columns:repeat(3,36px);grid-template-rows:repeat(3,32px);gap:3px;right:5px;bottom:5px}}
.regler{display:grid;grid-template-columns:auto 1fr 3.6em auto;gap:6px 8px;align-items:center}
.regler label{color:var(--leise);font-size:13px}
.regler .w{font-variant-numeric:tabular-nums;text-align:right;font-size:13px}
.regler .leer{width:0}
.knopfpaar{display:inline-flex;gap:4px}
.knopfpaar .ausrichten{font-size:14px;line-height:1;padding:3px 6px}
/* zweite Kopfzeile: Modi, Ziel, Bodenmodell */
.kopf-zeile2{flex:0 0 100%;display:flex;flex-wrap:wrap;gap:6px 14px;align-items:center;padding-top:6px;border-top:1px solid var(--rand)}
.fn-wahl{display:flex;gap:6px;align-items:center;margin-left:auto}   /* rechtsbündig */
.fn-wahl label{color:var(--leise);font-size:12px}
.fn-wahl select{min-width:0;max-width:220px}
#fn-text{font-size:12px}
#fn-text:empty{display:none}
.trenner{border:0;border-top:1px solid var(--rand);margin:10px 0}

/* --- Raumkarte */
/* Rahmen: volle Breite, Höhe höchstens 400 px (schmal quadratisch); darin die quadratische Karte in Kachelbreite,
   senkrecht mittig, oben und unten abgeschnitten. Maßstab: Ausschnitt (m) = Breite. */
.raumkarte{container-type:inline-size}
.karte-ausschnitt{display:flex;justify-content:flex-end;align-items:center;gap:6px;flex-wrap:wrap;margin:0 0 6px;font-size:12px;color:var(--leise)}
.karte-knoepfe{margin-right:auto}
#karte-rahmen{position:relative;width:100%;height:min(400px,100cqw);overflow:hidden;background:#808080;border-radius:6px;cursor:grab;user-select:none;-webkit-user-select:none}
#karte-rahmen.zieht{cursor:grabbing}
#karte-rahmen.setzt{cursor:crosshair}
#karte-flaeche{position:absolute;left:0;top:50%;width:100%;aspect-ratio:1;transform:translateY(-50%);pointer-events:none}
#karte-punkte{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}
/* Knöpfe nach Zweck gruppiert; während Route/Zone gesetzt wird, ist die ganze Gruppe gesperrt (fieldset) */
#karte-gruppen{border:0;margin:8px 0 0;padding:0;min-width:0;display:grid;gap:5px}
.gruppe{display:grid;grid-template-columns:7.5em 1fr;gap:8px;align-items:center}
.gruppe .g-name{font-size:12px;color:var(--leise)}
@media (max-width:420px){.gruppe{grid-template-columns:1fr;gap:3px}}
/* Auflagen im Kartenrahmen: Bearbeiten oben, Pose-Warnung unten – die Karte verrutscht dadurch nicht */
.karte-auflage{position:absolute;left:0;right:0;z-index:2;display:flex;flex-wrap:wrap;gap:5px;align-items:center;
  padding:6px 8px;font-size:12px;cursor:default;background:rgba(15,17,20,.88)}
.karte-auflage[hidden]{display:none}
#route-edit{top:0;border-bottom:1px solid var(--rand2)}
#route-hinweis{flex:1 1 100%;color:var(--text)}
#karte-pose-warn{bottom:0;margin:0;background:rgba(58,21,23,.94);color:#ffb3b5;border-top:1px solid #6b2427}
#karte-pose-warn strong{color:#fff}
/* Statuszeilen: Erkunden und Streife teilen sich eine Zeile (nie beide zugleich), Platz bleibt reserviert */
.karte-status{display:grid;margin-top:6px;font-size:12px;color:var(--leise)}
#karte-erkunden,#karte-streife{grid-area:2/1;min-height:1.4em}
.weg{visibility:hidden}
#karte-startbild{margin:6px 0 0;display:flex;gap:8px;align-items:flex-end;font-size:12px;color:var(--leise)}
#karte-startbild[hidden]{display:none}
#karte-startbild img{width:160px;height:auto;border:1px solid var(--rand2);border-radius:4px}
#karte{display:block;width:100%;height:100%;pointer-events:none;user-select:none;-webkit-user-select:none}

/* --- Extras */
details.extras>summary{cursor:pointer;list-style:none;display:flex;align-items:center;gap:8px;user-select:none}
details.extras>summary::-webkit-details-marker{display:none}
details.extras>summary :is(h2,h3){font-size:13px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:#a9b4c2;margin:0;flex:1}
details.extras>summary::after{content:"▸";color:var(--leise);transition:transform .15s}
details.extras[open]>summary::after{transform:rotate(90deg)}
details.extras[open]>summary{margin-bottom:8px}
.unter{font-size:12px;color:var(--leise);margin:10px 0 4px}
.unter:first-of-type{margin-top:0}
.leds{display:grid;grid-template-columns:repeat(2,52px);gap:6px}
.ledzeile{display:flex;gap:12px;align-items:center}

/* --- Verlauf (Canvas-Diagramme, Höhe aus data-h in CSS-Pixeln) */
.diag{display:block;width:100%;background:var(--karte2);border-radius:6px;touch-action:pan-y;cursor:crosshair}
.legende{display:flex;flex-wrap:wrap;gap:4px 12px;font-size:12px;color:var(--leise);margin:2px 0 4px;font-variant-numeric:tabular-nums}
.legende i{display:inline-block;width:10px;height:3px;border-radius:2px;vertical-align:middle;margin-right:4px}
.legende b{color:var(--text);font-weight:500}
.unter .rechts{float:right}
.lage3d{display:block;width:100%;background:var(--karte2);border-radius:6px;cursor:grab;touch-action:none}
.lage3d.zieht{cursor:grabbing}
.lage3d-rahmen{position:relative;user-select:none;-webkit-user-select:none}
.lage3d-gl{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
.unter button{margin-left:6px}
.radar{display:block;width:100%;max-width:340px;margin:0 auto;touch-action:pan-y}
.lage-flaeche{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:8px}
.kamhoehe{display:flex;flex-direction:column;align-items:center;gap:4px;height:230px;font-size:12px;color:var(--leise)}
.kamhoehe .w{color:var(--text);font-variant-numeric:tabular-nums;white-space:nowrap}
/* senkrecht, oben = hoch; touch-action: Ziehen verschiebt nicht die Seite */
#hoehe{writing-mode:vertical-lr;direction:rtl;flex:1 1 0;min-height:0;min-width:0;width:32px;touch-action:none}
#lagekarte>.regler{margin-top:8px}

/* --- Einstellungen (Modal): alles im Browser, am Rider nur Lesen/Schreiben der config.toml */
#einst{width:min(1100px,calc(100vw - 16px));height:min(88vh,900px);max-width:none;max-height:none;padding:0;
  background:var(--karte);color:var(--text);border:1px solid var(--rand2);border-radius:10px}
#einst[open]{display:flex;flex-direction:column}
#einst::backdrop{background:rgba(5,6,8,.7)}
.ei-kopf,.ei-fuss{display:flex;flex-wrap:wrap;gap:8px;align-items:center;padding:10px 12px;border-bottom:1px solid var(--rand)}
.ei-fuss{border-bottom:0;border-top:1px solid var(--rand)}
.ei-kopf h2{margin:0;font-size:15px;font-weight:600;flex:1;min-width:120px}
#ei-suche{width:min(260px,100%)}
.ei-kopf label{font-size:13px;color:var(--leise);display:flex;gap:5px;align-items:center;cursor:pointer}
.ei-rumpf{flex:1;min-height:0;display:grid;grid-template-columns:200px 1fr}
#ei-nav{overflow:auto;border-right:1px solid var(--rand);padding:6px 0}
#ei-nav .kat{font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:var(--leise);padding:8px 12px 3px}
#ei-nav button{display:flex;width:100%;border:0;border-radius:0;background:none;text-align:left;padding:5px 12px 5px 18px;font-size:13px;gap:6px}
#ei-nav button.aktiv{background:#18324f;color:#fff}
#ei-nav button .n{margin-left:auto;color:var(--leise);font-size:11px;font-variant-numeric:tabular-nums}
#ei-nav button .n.ae{color:var(--warn)}
#ei-felder{overflow:auto;padding:4px 14px 14px}
#ei-felder h3{font-size:13px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:#a9b4c2;margin:14px 0 2px}
#ei-felder .doc{font-size:12px;color:var(--leise);margin:0 0 6px}
.ei-feld{display:grid;grid-template-columns:minmax(150px,230px) minmax(0,1fr) auto;gap:2px 10px;align-items:center;
  padding:7px 0;border-bottom:1px solid var(--rand)}
.ei-feld .key{font:12.5px ui-monospace,Consolas,monospace;overflow-wrap:anywhere}
.ei-feld .marke{font:10.5px system-ui,sans-serif;padding:0 5px;border-radius:4px;margin-left:5px;vertical-align:1px}
.ei-feld .gesetzt{background:#22324a;color:#9cc7ff}
.ei-feld .geaendert{background:#4a3a17;color:#f0c46a}
.ei-feld input[type=text]{width:100%;font:13px ui-monospace,Consolas,monospace}
.ei-feld input.falsch{border-color:var(--gefahr);background:#2a1416}
.ei-feld input[type=checkbox]{width:18px;height:18px;accent-color:var(--akzent);justify-self:start}
.ei-feld .zurueck{padding:2px 8px;font-size:12px}
.ei-feld .hilfe{grid-column:1/-1;font-size:12px;color:var(--leise)}
.ei-feld .hilfe i{font-style:normal;color:#6f7885}
.ei-feld.gesperrt{opacity:.55}
#ei-info{flex:1;font-size:13px;color:var(--leise);min-width:160px}
#ei-info.warn{color:#f0c46a}
#ei-sich{padding:0 12px 10px;font-size:13px}
#ei-sich summary{cursor:pointer;color:var(--leise)}
#ei-sich ul{list-style:none;margin:6px 0 0;padding:0;max-height:22vh;overflow:auto}
#ei-sich li{display:flex;gap:8px;align-items:center;padding:3px 0;font-variant-numeric:tabular-nums}
@media (max-width:700px){
  .ei-rumpf{grid-template-columns:1fr;grid-template-rows:auto 1fr}
  #ei-nav{display:flex;overflow-x:auto;border-right:0;border-bottom:1px solid var(--rand);padding:0}
  #ei-nav .kat{display:none}
  #ei-nav button{width:auto;white-space:nowrap;padding:8px 10px}
  .ei-feld{grid-template-columns:minmax(0,1fr) auto}
  .ei-feld .key{grid-column:1/-1}
}
</style>
</head>
<body>
<header>
  <h1>xgonav <span><?= $host ?></span></h1>
  <div class="pillen">
    <span class="pille" id="p-verb"><span class="punkt"></span><b>…</b></span>
    <span class="pille" id="p-modus"><span class="lang">Modus </span><b>-</b></span>
    <span class="pille" id="p-zustand"><span class="lang">Zustand </span><b>-</b></span>
    <span class="pille" id="p-akku"><span class="lang">Akku </span><b>-</b></span>
    <span class="pille" id="p-rec" style="display:none" title="Rohaufzeichnung: schreibt nur, solange der Rider balanciert"><span class="lang">Aufzeichnung </span><b>-</b></span>
    <span class="pille" id="p-wlan" title="WLAN-Empfang (Pegel) und Durchsatz des Riders: ↓ empfangen, ↑ gesendet"><span class="lang">WLAN </span><b>-</b></span>
  </div>
  <div class="sprache" role="group" aria-label="Sprache">
    <button data-lang="de" onclick="sprache('de')" title="Deutsch">DE</button><button data-lang="en" onclick="sprache('en')" title="English">EN</button>
  </div>
  <!-- Kalibrier-Knöpfe gesperrt (2026-11-02): Speichern bei schlaffen Beinen hat die
       Beinnullpunkte im ESP32-Flash überschrieben. Erst mit geführtem Ablauf wieder freigeben. -->
  <div class="kopf-knoepfe">
    <button id="k-absetzen" class="gefahr kopf-ic" onclick="befehl('action', {n: 254})" aria-label="Absetzen" title="Absetzen (Aktion 254, Not-Stopp)"><span aria-hidden="true">⤓</span></button>
    <button id="k-aufspringen" class="kopf-ic" onclick="aufspringen()" aria-label="Aufspringen" title="Aufspringen (Aktion 253, !standup): Rider muss stehen und Platz haben"><span aria-hidden="true">⤒</span></button>
    <button id="k-aufstehen" class="kopf-ic" onclick="arm()" aria-label="Aufstehen" title="Aufstehen (Aktion 255): balanciert los, sobald er aufrecht steht"><span aria-hidden="true">↑</span></button>
    <button id="pip" class="kopf-ic" onclick="pipelineUmschalten()" aria-label="Bildverarbeitung starten" title="Bildverarbeitung (Kamera und Auswertung) starten"><span aria-hidden="true" id="pip-ic">▶</span></button>
    <button id="rec" class="kopf-ic" hidden onclick="aufzeichnungSetzen(!aktAufzeichnung)" aria-pressed="false" aria-label="Aufzeichnen" title="Aufzeichnen: Rohbilder und Metadaten am Rider speichern. Schreibt nur, solange er balanciert; Absetzen beendet die Aufnahme, jedes Aufstehen beginnt eine neue."><span aria-hidden="true">●</span></button>
  </div>
  <button id="einst-knopf" class="kopf-ic" onclick="einstOeffnen()" aria-label="Einstellungen" title="Alle Einstellungen aus config.toml ändern (mit Sicherung), xgonav neu starten"><span aria-hidden="true">⚙</span></button>
  <button id="ruhe-knopf" class="kopf-ic" onclick="ruheSchalten(true)" aria-label="Ruhe" title="Ruhemodus: absetzen, Bildverarbeitung aus, Abfragen aus, CPU-Takt runter, Schlafaugen am Display"><span aria-hidden="true">☾</span></button>
  <button id="stop" onclick="befehl('stop')" title="Stillstand, Modus idle">STOPP</button>
  <div class="kopf-zeile2">
      <div class="modus">
        <div class="segment">
          <button data-modus="idle" onclick="modus('idle')">Idle</button>
          <button data-modus="manual" onclick="modus('manual')">Hand</button>
          <button data-modus="follow" onclick="modus('follow')">Folgen</button>
          <button data-modus="explore" onclick="modus('explore')">Erkunden</button>
          <button data-modus="home" onclick="modus('home')" title="Weg über die Raumkarte zum Startpunkt">Zum Start</button>
          <button data-modus="patrol" onclick="streife()" title="Route auf der Raumkarte abfahren">Streife</button>
          <button data-modus="pet" onclick="modus('pet')" title="Reagiert auf Menschen, Katzen und Hunde">Haustier</button>
        </div>
        <div class="segment ziel-gruppe">
          <button data-modus="approach" onclick="modus('approach')">Anfahren</button>
          <select id="ziel" title="Ziel für Folgen/Anfahren (aktive YOLO-Klassen)" aria-label="Ziel"><option value="person">person</option></select>
        </div>
        <div class="tempo-kopf" title="Tempo aller Fahr- und Drehbefehle in allen Modi, bis zum Neustart von xgonav">
          <label for="tempo">Tempo</label>
          <input type="range" id="tempo" min="25" max="100" step="25" value="75">
          <span class="w" id="tempo-w"></span>
        </div>
      </div>
      <div class="fn-wahl">
        <label for="fn-wahl">Bodenmodell</label>
        <select id="fn-wahl" aria-label="Bodenmodell" title="FloorNet-Gewichte aus [freespace] net_dir"></select>
        <button id="fn-an" onclick="bodenmodellAktivieren()" title="xgonav lädt und prüft das Netz, stellt den Symlink [freespace] net um und tauscht das Bodenmodell ohne Neustart (nur Idle/Hand)">Aktivieren</button>
      </div>
      <span class="leise" id="fn-text"></span>
  </div>
</header>
<div id="ruhe-decke">
  <h2>Ruhemodus</h2>
  <p>Rider liegt, Bildverarbeitung und Abfragen sind aus, CPU-Takt gesenkt, am Display stehen die Schlafaugen. Akku <b id="ruhe-akku">-</b></p>
  <p id="ruhe-fehler"></p>
  <button onclick="ruheSchalten(false)" title="CPU-Takt und Abfragen wieder hoch; Bildverarbeitung bleibt aus, Rider bleibt liegen">Wecken</button>
</div>
<div id="meldung"></div>

<main>
<div class="spalte">
  <div id="bild-rahmen">
    <svg width="0" height="0" style="position:absolute" aria-hidden="true"><filter id="bild-filter" color-interpolation-filters="sRGB"><feComponentTransfer><feFuncR id="bf-r" type="linear" slope="1" intercept="0"/><feFuncG id="bf-g" type="linear" slope="1" intercept="0"/><feFuncB id="bf-b" type="linear" slope="1" intercept="0"/></feComponentTransfer><feColorMatrix id="bf-satt" type="saturate" values="1"/><feColorMatrix id="bf-ton" type="hueRotate" values="0"/></filter></svg>
    <div class="bild-fenster"><img id="bild" alt="" aria-label="Kamerabild" width="640" height="480"><svg id="bild-mitte" viewBox="0 0 640 480" preserveAspectRatio="xMidYMid meet" aria-hidden="true"><g class="halo"><line class="lot" x1="320" y1="0" x2="320" y2="480"/><line class="qx" x1="305" y1="240" x2="335" y2="240"/><circle cx="320" cy="240" r="8"/></g><g class="strich"><line class="lot" x1="320" y1="0" x2="320" y2="480"/><line class="qx" x1="305" y1="240" x2="335" y2="240"/><circle cx="320" cy="240" r="8"/></g></svg><svg id="bild-plan" viewBox="0 0 640 480" preserveAspectRatio="xMidYMid meet" aria-hidden="true"></svg>
      <div class="bild-schalter" role="group" aria-label="Anzeige im Bild">
        <button id="hz-stabil" class="umschalt" onclick="hzStabil(!bildStabil)" aria-pressed="false" aria-label="Bild am IMU-Nick festhalten" title="Bild am IMU-Nick festhalten: Bild senkrecht so verschieben, dass der IMU-Nick fest steht; stimmt die IMU, steht die Szene still"><span aria-hidden="true">⇕</span></button><button id="hz-roll" class="umschalt" onclick="hzRoll(!bildRollAn)" aria-pressed="false" aria-label="Roll ausgleichen" title="Roll ausgleichen: Bild um den IMU-Roll drehen; stimmt der Roll, stehen senkrechte Kanten senkrecht und die cyan Linie waagerecht"><span aria-hidden="true">⟲</span></button><button id="hz-plan" class="umschalt" onclick="hzPlan(!planAn)" aria-pressed="false" aria-label="Planerbahn" title="Planerbahn: geplanten Weg des Planers (Zum Start, Streife, Haustier-Suche) aus der Raumkarte ins Bild legen; im Browser gerechnet aus Kartenpose, Nick, Roll und Kamerahöhe"><span aria-hidden="true">↝</span></button><button id="hz-mitte" class="umschalt" onclick="hzMitte(!bildMitteAn)" aria-pressed="false" aria-label="Bildmitte" title="Bildmitte: konfigurierte Bildmitte einblenden, gestrichelt die Spalte der Fahrtrichtung (W/2), Kreis bei Zeile H/2"><span aria-hidden="true">⌖</span></button>
      </div>
      <div class="pad" title="Steuerkreuz im Bild gedrückt halten, oder Pfeiltasten/WASD. Fahren schaltet auf „Hand“ und ist auf max_v/max_w der Konfiguration begrenzt. Ohne Bodenmodell und Bahnprüfung; Sturz-Erkennung und Watchdogs bleiben aktiv.">
        <span></span><button data-fahrt="vor" aria-label="vor">▲</button><span></span>
        <button data-fahrt="links" aria-label="links">◀</button><span></span><button data-fahrt="rechts" aria-label="rechts">▶</button>
        <span></span><button data-fahrt="zurück" aria-label="zurück">▼</button><span></span>
      </div></div>
    <div class="bild-leiste">
      <details class="kamera-klapp" id="kamera-klapp">
        <summary><span class="titel">Kamera</span><span class="leise" id="belicht-ist">-</span></summary>
        <div class="kamera-raster">
          <div class="regler">
            <label for="belicht">Belichtung</label>
            <input type="range" id="belicht" min="1" max="33" value="15" step="1"
              oninput="belichtAnzeige(this.value * 1000)" onchange="belichtung(this.value * 1000)">
            <span class="w" id="belicht-w">-</span>
            <button id="belicht-auto" onclick="belichtung(0)">Auto</button>
          </div>
          <div class="regler">
            <label for="gain">Verstärkung</label>
            <input type="range" id="gain" min="1" max="63.9" value="8" step="0.5"
              oninput="gainAnzeige(this.value)" onchange="verstaerkung(this.value)">
            <span class="w" id="gain-w">-</span>
            <button id="gain-auto" onclick="verstaerkung(0)">Auto</button>
          </div>
          <div class="regler">
            <label for="hell">Helligkeit</label>
            <input type="range" id="hell" min="-0.5" max="0.5" value="0" step="0.05"
              oninput="bildAnzeige()" onchange="bildSetzen()">
            <span class="w" id="hell-w">-</span>
            <button onclick="bildSetzen(null, null, 0)">0</button>
          </div>
          <div class="regler">
            <label for="kontrast">Kontrast</label>
            <input type="range" id="kontrast" min="0.5" max="4" value="1" step="0.1"
              oninput="bildAnzeige()" onchange="bildSetzen()">
            <span class="w" id="kontrast-w">-</span>
            <button onclick="bildSetzen(1, null)">1,0</button>
          </div>
          <div class="regler">
            <label for="saettigung">Sättigung</label>
            <input type="range" id="saettigung" min="0.5" max="4" value="1" step="0.1"
              oninput="bildAnzeige()" onchange="bildSetzen()">
            <span class="w" id="saettigung-w">-</span>
            <button onclick="bildSetzen(null, 1)">1,0</button>
          </div>
          <div class="regler">
            <label for="farbton" title="Nur Anzeige im Browser (libcamera hat keinen Farbton-Regler): Kamera, FloorNet und Aufnahmen bleiben unverändert">Farbton</label>
            <input type="range" id="farbton" min="-180" max="180" value="0" step="5" oninput="farbtonSetzen(this.value)">
            <span class="w" id="farbton-w">-</span>
            <button onclick="farbtonSetzen(0)">0</button>
          </div>
          <span class="ausgleich"><label title="Helligkeit/Kontrast/Sättigung wirken an der Quelle (ISP) und damit auch auf FloorNet und die Aufnahmen. Für die Anzeige hier per Filter wieder herunterziehen (Näherung; abgeschnittene Lichter und Schatten bleiben weg)."><input type="checkbox" id="bild-ausgleich" onchange="bildAusgleich(this.checked)"> Im Browser ausgleichen</label><span class="info" tabindex="0" title="Gilt bis zum Neustart von xgonav. Kürzere Belichtung = weniger Bewegungsunschärfe, aber dunkler; mehr Verstärkung = heller, aber mehr Rauschen (Automatik nur bis 8; über ~8× wird der Rauschabstand nicht besser, nur das Bild heller). Dauerhaft über [camera] exposure_us / analogue_gain in config.toml.">i</span></span>
        </div>
      </details>
      <div class="imu-bild leise" id="imu-bild" title="IMU-Wert dieses Bildes (Bildzeit + [imu] time_offset_ms), mit dem xgonav Boden, Karten und die cyan Linie rechnet. Nickrate: aus dem Bild (Ego-Motion) gegen die IMU; weichen sie dauerhaft ab, stimmen Zeitversatz oder Brennweite nicht">-</div>
    </div>
  </div>
  <details class="karte extras" id="telefonkarte" ontoggle="telefonUmschalten()">
    <summary><h2>Telefon</h2><span class="leise" id="telefon-zustand">Nur im Modus Hand</span></summary>
    <div class="sprech-zeile">
      <button id="sprech" onclick="sprechUmschalten()" disabled title="Nur im Modus Hand">Sprechen</button>
      <label title="Mikrofon nur senden, solange die Sprechtaste oder die Leertaste gedrückt ist; der Rider ist dann still"><input type="checkbox" id="sprech-ptt" onchange="sprechPtt(this.checked)"> Drücken zum Sprechen</label>
      <button id="sprech-taste" hidden title="Gedrückt halten (oder Leertaste)">Sprechtaste</button>
      <span class="leise">Rider</span><meter id="sprech-rider" min="0" max="1" value="0"></meter>
      <span class="leise">ich</span><meter id="sprech-ich" min="0" max="1" value="0"></meter>
      <label class="leise" title="Lautstärke des Riders hier im Browser">hier <input type="range" id="sprech-laut" min="0" max="400" value="200" aria-label="Lautstärke hier"></label>
      <label class="leise" title="Verstärkung der eigenen Stimme zum Rider (mit weichem Begrenzer, kein Übersteuern)">Rider <input type="range" id="sprech-hin" min="100" max="1600" step="50" value="300" aria-label="Lautstärke am Rider"></label>
      <span class="leise" id="sprech-text"></span>
      <label title="Eigenes Kamerabild aufs Display des Riders (320×240, im Browser umgerechnet; 0,75 MB/s bei 5 Bildern/s, am Rider ~9 % eines Kerns, bei 10/s doppelt). Nur während eines Gesprächs."><input type="checkbox" id="sprech-bild" onchange="bildUmschalten(this.checked)" disabled> Kamera senden</label>
      <select id="sprech-fps" title="Bilder je Sekunde aufs Display (WLAN: je Bild 150 kB)" aria-label="Bilder je Sekunde"><option value="5" selected>5/s</option><option value="10">10/s</option><option value="15">15/s</option></select>
      <video id="sprech-vorschau" muted playsinline hidden></video>
      <span class="leise" id="bild-text"></span>
    </div>
  </details>
  <div class="karte">
    <div class="kopf"><h2>Laufzeit Infos</h2><span class="leise" id="rec-text"></span>
      <span class="leise" id="rec-platz" hidden title="Frei am Rider bis zur Abbruchgrenze (output.record_min_free_mb); Minuten aus der Schreibrate der letzten Aufnahme (~ = geschätzt)"></span>
</div>
    <div class="kacheln" id="kurz"></div>
  </div>
  <details class="karte extras" id="verlauf" ontoggle="verlaufUmschalten()">
    <summary><h2>Verlauf</h2><span class="leise">CPU · WLAN · Akku</span></summary>
    <div class="unter">CPU je Kern (bezogen auf Höchsttakt), letzte 10 min <span class="rechts" id="v-temp"></span></div>
    <div class="legende" id="l-cpu"></div>
    <canvas class="diag" id="d-cpu" data-h="140"></canvas>
    <div class="unter">WLAN-Durchsatz, letzte 10 min <span class="rechts" id="v-wlan"></span></div>
    <div class="legende" id="l-wlan"></div>
    <canvas class="diag" id="d-wlan" data-h="110"></canvas>
    <div class="unter">WLAN-Empfang (Pegel)</div>
    <canvas class="diag" id="d-pegel" data-h="90"></canvas>
    <div class="unter">Akku, 120 min (im Browser gespeichert) <span class="rechts"><span id="v-akku"></span><button onclick="akkuLeeren()">Leeren</button></span></div>
    <canvas class="diag" id="d-akku" data-h="100"></canvas>
  </details>
</div>

<div class="spalte">
  <details class="karte extras" id="lagekarte" ontoggle="lageUmschalten()">
    <summary><h2>Lage</h2><span class="leise">IMU live · ziehen: drehen · Doppelklick: zurück</span></summary>
    <div class="lage-flaeche">
      <div class="lage3d-rahmen">
        <canvas class="lage3d" id="d-lage" data-h="230" role="img" aria-label="Lage des Riders als 3D-Modell"></canvas>
        <canvas class="lage3d-gl" id="d-lage-gl" aria-hidden="true"></canvas>
      </div>
      <div class="kamhoehe">
        <label for="hoehe">Kamerahöhe</label>
        <input type="range" id="hoehe" min="61" max="205" value="128"
          oninput="hoeheWert(this.value)"
          onchange="befehl('height', {h: this.value})">
        <span class="w" id="hoehe-w" title="Sollwert u8 128">u8 128</span>
      </div>
    </div>
    <div class="regler">
      <label for="roll">Roll</label>
      <input type="range" id="roll" min="-20" max="20" value="0" step="1"
        oninput="document.getElementById('roll-w').textContent = this.value + '°'"
        onchange="befehl('roll', {deg: this.value})">
      <span class="w" id="roll-w">0°</span><span class="knopfpaar"><button onclick="rollNull()" title="Roll auf 0°">0</button><button class="ausrichten" onclick="ausrichten()" aria-label="Neu ausrichten" title="Neu ausrichten: Roll auf 0° und die Beine 3 s nivellieren (nur wenn er steht)">⚖</button></span>
    </div>
    <div class="legende" id="l-lage" title="Gier 0° = Kurs beim Start des ESP32; + = links. Roll + = rechts tief, Nick + = vorn tief"></div>
    <details class="hilfe" data-i18n-html="hilfe-lage"><summary>Kamerahöhe, Roll, Aufstehen und Absetzen</summary>
      <p>Kamerahöhe: Rechenwert der Bodengeometrie aus <code>cam_h_u8</code> (Linsenmitte über Boden, Sollwert u8
        im Tooltip), die Beine brauchen danach etwa 1 s. Der Wert entspricht der Messung mit dem Zollstock
        (Gitter am Boden 2026-11-10). Oberhalb von u8 200 ändert sich nichts
        mehr. Untere Grenze mit Abstand zum Minimum; in Folgen/Erkunden setzt xgonav die Höhe selbst.</p>
      <p>Roll: + = rechts.</p>
      <p>Balancieren (Firmware R-1.1.6): Nach einem ESP32-Neustart balanciert der Rider erst nach
        „Aufstehen“. Liegt er, wartet die Firmware, bis er aufrecht steht – dann laufen die Räder sofort an.
        „Aufspringen“ (Aktion 253, wie <code>!standup</code>) lässt ihn aufspringen – <b>Rider muss stehen und Platz haben</b>.
        „Absetzen“ ist ein Not-Stopp (Aktion 254, wie <code>!stop</code>): er setzt sich kontrolliert ab und
        bleibt aus bis zum nächsten Aufstehen.</p>
      <p>„Neu ausrichten“ (⚖ neben dem Roll-Regler): Die Beine haben eine Totzone, bei gleichem Befehl steht er bis zu 2,6° unterschiedlich
        schief. Der Knopf setzt Roll auf 0° und lässt die Firmware 3 s nivellieren; danach bleibt er so stehen. Das
        passiert auch einmal nach jedem Aufstehen. Beim Fahren kann er langsam wieder schief werden.</p>
    </details>
  </details>
  <details class="karte extras" id="radarkarte" ontoggle="radarUmschalten()">
    <summary><h2>Schallradar</h2><span class="leise" id="radar-zustand">-</span></summary>
    <canvas class="radar" id="d-radar" role="img" aria-label="Richtung der letzten Geräusche"></canvas>
    <div class="legende" id="l-radar"></div>
    <details class="hilfe" data-i18n-html="hilfe-radar"><summary>Strahl = Geräusch · Länge = Lautstärke · verblasst in 12 s</summary>
      <p>Richtung aus dem Laufzeitunterschied der zwei Mikrofone (<code>[ortung]</code>). Oben ist vorn; dreht
        sich der Rider, drehen Peilungen und Speichen mit (IMU-Gier), alte Strahlen zeigen weiter zur Quelle. Zwei
        Mikrofone trennen nur links und rechts: jeder Strahl hat deshalb einen gestrichelten Zwilling nach hinten,
        die Quelle liegt auf einem von beiden. Länge = Pegel über dem Ruhepegel, Deckkraft = Güte und Alter.
        Lauscht nur in den Modi aus <code>[ortung] modes</code>, nie im Ruhemodus; eigene Töne werden ausgeblendet.</p>
    </details>
  </details>
  <details class="karte extras" id="extras" ontoggle="extrasUmschalten()">
    <summary><h2>Extras</h2><span class="leise">System · Aktionen · LEDs · Emotionen</span></summary>
    <div class="unter">System</div>
    <div class="knoepfe">
      <button onclick="window.open('anleitung/index.html', '_blank')" title="Anleitung: Installation, erste Schritte, Oberfläche, Verhalten, FloorNet-Training (englisch, statisch, kostet am Pi nichts)">Anleitung</button>
      <button onclick="window.open('schaubild.html', '_blank')" title="Schaubild der Bildpipeline: Stufen, Korrekturen, Threads, Eingänge und Abnehmer (statisch, kostet am Pi nichts)">Schaubild</button>
      <button onclick="espNeustart()" title="Aktion 237: ESP32 neu starten, nur wenn er nicht balanciert">ESP-Neustart</button>
      <button id="aus" class="gefahr" onclick="herunterfahren()" title="Rider herunterfahren">Herunterfahren</button>
    </div>
    <div class="unter">Aktionen</div>
    <div class="knoepfe">
      <?php for ($i = 1; $i <= 6; $i++): ?><button onclick="befehl('action', {n: <?= $i ?>})"<?= $i >= 3 && $i <= 5 ? ' title="Fährt bzw. dreht selbst: nur im Modus idle, braucht Platz"' : '' ?>><?= $i ?></button><?php endfor; ?>
    </div>
    <div class="unter">LEDs</div>
    <div class="ledzeile">
      <div class="leds">
        <input type="color" id="led0" onchange="led(0, this.value)" aria-label="LED 0"><input type="color" id="led3" onchange="led(3, this.value)" aria-label="LED 3">
        <input type="color" id="led1" onchange="led(1, this.value)" aria-label="LED 1"><input type="color" id="led2" onchange="led(2, this.value)" aria-label="LED 2">
      </div>
      <div class="knoepfe">
        <button onclick="ledsAus()">Alle aus</button>
        <button id="strobe" onclick="strobe()">Strobe</button>
      </div>
    </div>
    <div class="regler" title="Gilt für alle LEDs, bleibt über Neustarts gespeichert">
      <label for="ledhell">Helligkeit</label>
      <input type="range" id="ledhell" min="0" max="100" value="<?= $ledP ?>" step="1"
        oninput="document.getElementById('ledhell-w').textContent = this.value + ' %'"
        onchange="befehl('led_brightness', {p: this.value}, true)">
      <span class="w" id="ledhell-w"><?= $ledP ?> %</span><span class="leer"></span>
    </div>
    <div class="unter">Emotionen (Display)</div>
    <div class="knoepfe">
      <?php foreach ($emotionen as $e): ?><button onclick="befehl('emotion', {name: '<?= h($e) ?>'})"><?= h($e) ?></button><?php endforeach; ?>
      <?php if (!$emotionen): ?><span class="leise">keine gefunden in <?= h(EMOTION_RAW) ?></span><?php endif; ?>
    </div>
  </details>

  <details class="karte extras raumkarte" id="raumkarte" ontoggle="raumkarteUmschalten()">
    <summary><h2>Raumkarte</h2><span class="leise">eingeklappt: kein Nachladen</span></summary>
    <div class="karte-ausschnitt"><div class="knoepfe karte-knoepfe">
        <button onclick="befehl('map_save')" title="Raumkarte jetzt speichern ([grid] file)">Speichern</button>
        <button id="karte-sperre" onclick="karteSperren()" title="Gesperrt ändern Fahrten die Karte nicht">Karte sperren</button>
        <button class="gefahr" onclick="karteZuruecksetzen()" title="Karte leeren; Route und Sperrzonen gehen verloren">Zurücksetzen</button>
      </div><label for="karte-view">Ausschnitt</label>
      <select id="karte-view" aria-label="Ausschnitt" onchange="karteView(this.value)"><option value="3">3 m</option><option value="6" selected>6 m</option><option value="10">10 m</option><option value="16">16 m</option><option value="24">24 m</option><option id="karte-view-frei" hidden></option></select>
    </div>
    <div id="karte-rahmen" title="ziehen: verschieben · Mausrad: zoomen · Doppelklick: zurück zum Rider">
      <div id="karte-flaeche">
        <img id="karte" alt="Raumkarte" width="320" height="320" draggable="false">
        <svg id="karte-punkte" viewBox="0 0 1 1" preserveAspectRatio="none"></svg>
      </div>
      <div class="karte-auflage" id="route-edit" hidden>
        <span id="route-hinweis"></span>
        <label id="route-kreis"><input type="checkbox" id="route-loop"> im Kreis</label>
        <button onclick="punktWeg()">Letzten weg</button>
        <button onclick="bearbeitenFertig()" class="aktiv">Übernehmen</button>
        <button onclick="bearbeitenEnde()">Abbrechen</button>
      </div>
      <p class="karte-auflage" id="karte-pose-warn" hidden>
        <span><strong>Pose unbekannt</strong><span id="karte-pose-grund"></span>. Die Karte wird nicht beschrieben, „Zum Start“, Streife und Erkunden sind gesperrt;
        Sperrzonen gelten an falscher Stelle.</span>
        <button onclick="poseBestaetigen('start')" title="Die Pose springt auf den Startpunkt">Steht am Startpunkt</button>
        <button onclick="poseBestaetigen('hier')" title="Der Rider wurde nicht versetzt">Pose stimmt</button>
      </p>
    </div>
    <div class="karte-status">
      <div id="karte-pose">-</div>
      <div id="karte-erkunden" class="weg"></div>
      <div id="karte-streife" class="weg"></div>
    </div>
    <figure id="karte-startbild" hidden title="Bild am Startpunkt (Anker des Abgleichs); nach dem Absetzen weg, neu nach dem Aufspringen">
      <img id="karte-startbild-img" alt="Startbild" width="160" height="120">
      <figcaption>Startbild</figcaption>
    </figure>
    <fieldset id="karte-gruppen">
      <div class="gruppe"><span class="g-name">Start · Pose</span><div class="knoepfe">
        <button onclick="befehl('map_start')" title="Startpunkt = aktuelle Pose">Start hier</button>
        <button id="karte-pose-weg" hidden onclick="befehl('map_pose', {aktion: 'verwerfen'})" title="Der Rider wurde versetzt: Die Karte wird nicht mehr beschrieben, bis der Standort bestätigt ist">Pose verwerfen</button>
      </div></div>
      <div class="gruppe"><span class="g-name">Route · Zonen</span><div class="knoepfe">
        <button onclick="bearbeiten('route')" title="Wegpunkte der Streife per Klick auf die Karte setzen">Route setzen</button>
        <button onclick="bearbeiten('zone')" title="Bereich, den der Rider nie befährt: Ecken anklicken">Sperrzone</button>
        <button onclick="zonenLoeschen()" title="Alle Sperrzonen löschen">Zonen löschen</button>
      </div></div>
    </fieldset>
    <details class="hilfe" data-i18n-html="hilfe-karte"><summary>grau unbekannt · hell frei · dunkel belegt · grün Weg · cyan Plan · blau Start · rot Rider</summary>
      <p>Maus: ziehen verschiebt, Mausrad zoomt um den Zeiger, Doppelklick zentriert wieder auf den Rider
        und folgt ihm.</p>
      <p>Ursprung = Startpose (oben = vorn beim Start). Strecke und Drehung aus den Rädern (auch bei
        gestoppter Bildverarbeitung); ohne Marker driftet die Karte mit der Zeit.</p>
      <p>„Erkunden“ kartiert den Raum: erst ein Rundumblick, dann fährt er die Grenzen zwischen bekanntem Boden und
        Unbekanntem an, bis nichts mehr erreichbar ist (oder Zeit- bzw. Akkugrenze). Danach speichert er die Karte und
        fährt zum Start.</p>
      <p>„Route setzen“: Punkte der Reihe nach auf die Karte klicken; magenta ist die gespeicherte Route. Jeder Punkt
        muss auf bekanntem, erreichbarem Boden liegen. „Sperrzone“: Ecken eines Bereichs anklicken, den er nie befahren
        soll, etwa eine Treppe (rot). Sie gilt für alle Modi außer der Handsteuerung, driftet aber mit der Karte: Sie
        ersetzt keine Absperrung. Route und Zonen gehören zu dieser Karte; nach „Zurücksetzen“ sind sie weg.</p>
      <p>„Streife“ fährt die Route ab und am Ende zum Start. Der Rider muss dafür am Startpunkt stehen. Ab 30 % Akku fährt
        er zurück, bei 15 % setzt er sich ab. Nach „Erkunden“ ist die Karte gesperrt, damit Fahrten sie nicht mehr ändern.</p>
      <p>„Zum Start“ plant einen Weg über die Karte und fährt ihn ab. Die Karte wird regelmäßig gespeichert
        und beim Start von xgonav geladen; der Rider muss dann wieder am Startpunkt stehen.</p>
    </details>
  </details>
</div>
</main>
<dialog id="einst" aria-labelledby="ei-titel">
  <div class="ei-kopf">
    <h2 id="ei-titel">Einstellungen</h2>
    <input id="ei-suche" type="search" placeholder="Suchen (Schlüssel, Text)" aria-label="Einstellungen durchsuchen" oninput="einstZeigen()">
    <label><input type="checkbox" id="ei-nurgesetzt" onchange="einstZeigen()">nur in config.toml gesetzte</label>
    <button onclick="einstSchliessen()" aria-label="Schließen">✕</button>
  </div>
  <div class="ei-rumpf">
    <nav id="ei-nav"></nav>
    <div id="ei-felder"></div>
  </div>
  <details id="ei-sich"><summary>Sicherungen</summary><ul id="ei-sich-liste"></ul></details>
  <div class="ei-fuss">
    <span id="ei-info">-</span>
    <button onclick="einstVerwerfen()" title="Nicht gespeicherte Änderungen verwerfen">Verwerfen</button>
    <button id="ei-speichern" onclick="einstSpeichern(false)">Speichern</button>
    <button id="ei-speichern-neu" onclick="einstSpeichern(true)" class="aktiv">Speichern und neu starten</button>
    <button onclick="einstNeustart()" title="xgonav neu starten: liest config.toml neu ein">Neu starten</button>
  </div>
</dialog>
<script>
// ---------------------------------------------------------------- Sprache (nur im Browser, am Rider keine Kosten)
// Deutsch steht im HTML und im Code; EN übersetzt über den deutschen Text als Schlüssel (Leerraum zusammengefasst).
// Feste Texte: Textknoten und title/aria-label/placeholder; lange Hilfen ganz über data-i18n-html (EN_HTML).
// Dynamische Texte: tr('…'). Was xgonav oder api.php als Freitext liefern (Status, Fehler), bleibt deutsch.
// ?i18n=debug listet fehlende Übersetzungen in der Konsole.
const EN = {
  'Modus': 'Mode', 'Zustand': 'State', 'Akku': 'Battery', 'Bildverarb.': 'Vision',
  'Rohaufzeichnung: schreibt nur, solange der Rider balanciert': 'Raw recording: writes only while the Rider is balancing',
  'Aufzeichnung': 'Recording', 'WLAN': 'Wi-Fi',
  'WLAN-Empfang (Pegel) und Durchsatz des Riders: ↓ empfangen, ↑ gesendet': 'Wi-Fi signal (level) and throughput of the Rider: ↓ received, ↑ sent',
  'Sprache': 'Language',
  'Alle Einstellungen aus config.toml ändern (mit Sicherung), xgonav neu starten': 'Edit all settings in config.toml (with backup), restart xgonav',
  'Einstellungen': 'Settings', 'Schaubild': 'Diagram', 'Anleitung': 'Manual', 'Anleitung: Installation, erste Schritte, Oberfläche, Verhalten, FloorNet-Training (englisch, statisch, kostet am Pi nichts)': 'Manual: install, quick start, UI, behaviours, FloorNet training (static, no cost on the Pi)', 'Schaubild der Bildpipeline: Stufen, Korrekturen, Threads, Eingänge und Abnehmer (statisch, kostet am Pi nichts)': 'Image pipeline diagram: stages, corrections, threads, inputs and consumers (static, no cost on the Pi)', 'Rider herunterfahren': 'Shut down the Rider', 'Herunterfahren': 'Shut down',
  'Ruhemodus: absetzen, Bildverarbeitung aus, Abfragen aus, CPU-Takt runter, Schlafaugen am Display':
    'Rest mode: sit down, vision off, polling off, CPU clock down, sleepy eyes on the display',
  'Ruhe': 'Rest', 'Stillstand, Modus idle': 'Stand still, mode idle', 'STOPP': 'STOP', 'Ruhemodus': 'Rest mode',
  'Rider liegt, Bildverarbeitung und Abfragen sind aus, CPU-Takt gesenkt, am Display stehen die Schlafaugen. Akku':
    'Rider is lying down, vision and polling are off, CPU clock lowered, sleepy eyes on the display. Battery',
  'CPU-Takt und Abfragen wieder hoch; Bildverarbeitung bleibt aus, Rider bleibt liegen': 'CPU clock and polling back up; vision stays off, Rider stays down',
  'Wecken': 'Wake up', 'Kamerabild': 'Camera image', 'Belichtung': 'Exposure', 'Verstärkung': 'Gain',
  'Kontrast': 'Contrast', 'Sättigung': 'Saturation', 'Im Browser ausgleichen': 'Compensate in browser',
  'Helligkeit': 'Brightness', 'Farbton': 'Hue', 'Kamera': 'Camera',
  'Nur Anzeige im Browser (libcamera hat keinen Farbton-Regler): Kamera, FloorNet und Aufnahmen bleiben unverändert':
    'Display in the browser only (libcamera has no hue control): camera, FloorNet and recordings stay unchanged',
  'Helligkeit/Kontrast/Sättigung wirken an der Quelle (ISP) und damit auch auf FloorNet und die Aufnahmen. Für die Anzeige hier per Filter wieder herunterziehen (Näherung; abgeschnittene Lichter und Schatten bleiben weg).':
    'Brightness/contrast/saturation act at the source (ISP), so they also reach FloorNet and the recordings. For the view here they are pulled back down by a filter (approximation; clipped highlights and shadows stay lost).',
  'Gilt bis zum Neustart von xgonav. Kürzere Belichtung = weniger Bewegungsunschärfe, aber dunkler; mehr Verstärkung = heller, aber mehr Rauschen (Automatik nur bis 8; über ~8× wird der Rauschabstand nicht besser, nur das Bild heller). Dauerhaft über [camera] exposure_us / analogue_gain in config.toml.':
    'Applies until xgonav restarts. Shorter exposure = less motion blur but darker; more gain = brighter but noisier (auto only up to 8; above ~8× the signal-to-noise ratio does not improve, the image just gets brighter). Permanently via [camera] exposure_us / analogue_gain in config.toml.',
  'Idle': 'Idle', 'Hand': 'Manual', 'Folgen': 'Follow', 'Erkunden': 'Explore', 'Anfahren': 'Approach',
  'Weg über die Raumkarte zum Startpunkt': 'Path over the room map to the start point', 'Zum Start': 'Home',
  'Route auf der Raumkarte abfahren': 'Drive the route on the room map', 'Streife': 'Patrol',
  'Reagiert auf Menschen, Katzen und Hunde': 'Reacts to people, cats and dogs', 'Haustier': 'Pet',
  'Ziel für Folgen/Anfahren (aktive YOLO-Klassen)': 'Target for follow/approach (active YOLO classes)', 'Ziel': 'Target',
  'Laufzeit Infos': 'Runtime info',
  'Rohbilder und Metadaten am Rider speichern. Schreibt nur, solange er balanciert; Absetzen beendet die Aufnahme, jedes Aufstehen beginnt eine neue.':
    'Store raw images and metadata on the Rider. Writes only while balancing; sitting down ends the recording, every stand-up starts a new one.',
  'Aufzeichnen': 'Record',
  'Frei am Rider bis zur Abbruchgrenze (output.record_min_free_mb); Minuten aus der Schreibrate der letzten Aufnahme (~ = geschätzt)': 'Free on the Rider down to the stop limit (output.record_min_free_mb); minutes from the write rate of the last recording (~ = estimated)', 'Verlauf': 'History', 'CPU · WLAN · Akku': 'CPU · Wi-Fi · Battery',
  'CPU je Kern (bezogen auf Höchsttakt), letzte 10 min': 'CPU per core (relative to max clock), last 10 min', 'WLAN-Durchsatz, letzte 10 min': 'Wi-Fi throughput, last 10 min',
  'WLAN-Empfang (Pegel)': 'Wi-Fi signal (level)', 'Akku, 120 min (im Browser gespeichert)': 'Battery, 120 min (stored in the browser)',
  'Schallradar': 'Sound radar', 'lauscht': 'listening', 'aus': 'off', 'links': 'left', 'rechts': 'right', 'Mitte': 'centre',
  'vorn': 'front', 'hinten': 'rear', 'Güte': 'quality', 'vor {s} s': '{s} s ago', 'noch kein Geräusch': 'no sound yet',
  'Richtung der letzten Geräusche': 'Direction of the latest sounds',
  'Leeren': 'Clear', 'Lage': 'Attitude', 'IMU live · ziehen: drehen · Doppelklick: zurück': 'IMU live · drag: rotate · double-click: reset',
  'Lage des Riders als 3D-Modell': 'Attitude of the Rider as a 3D model', 'Kamerahöhe': 'Camera height',
  'Roll': 'Roll', 'Roll auf 0°': 'Roll to 0°',
  'Gier 0° = Kurs beim Start des ESP32; + = links. Roll + = rechts tief, Nick + = vorn tief':
    'Yaw 0° = heading at ESP32 start; + = left. Roll + = right side down, pitch + = front down',
  'Tempo aller Fahr- und Drehbefehle in allen Modi, bis zum Neustart von xgonav': 'Speed of all drive and turn commands in all modes, until xgonav restarts',
  'Fahren': 'Drive', 'vor': 'forward', 'links': 'left', 'rechts': 'right', 'zurück': 'back', 'Tempo': 'Speed', 'Pfeiltasten/WASD': 'Arrow keys/WASD',
  'Aktion 255': 'Action 255', 'Aufstehen': 'Stand up',
  'Absetzen (Aktion 254, Not-Stopp)': 'Sit down (action 254, emergency stop)',
  'Aufspringen (Aktion 253, !standup): Rider muss stehen und Platz haben': 'Jump up (action 253, !standup): Rider must be upright and have room',
  'Aufstehen (Aktion 255): balanciert los, sobald er aufrecht steht': 'Stand up (action 255): starts balancing once it is upright',
  'Gedrückt halten: Maus, Touch, Pfeiltasten/WASD': 'Hold: mouse, touch, arrow keys/WASD',
  'Steuerkreuz im Bild gedrückt halten, oder Pfeiltasten/WASD. Fahren schaltet auf „Hand“ und ist auf max_v/max_w der Konfiguration begrenzt. Ohne Bodenmodell und Bahnprüfung; Sturz-Erkennung und Watchdogs bleiben aktiv.': 'Hold the arrow pad on the image, or arrow keys/WASD. Driving switches to “Manual” and is limited to max_v/max_w from the configuration. No floor model and no path check; fall detection and watchdogs stay active.',
  'Aktion 253 (!standup): Rider muss stehen und Platz haben': 'Action 253 (!standup): Rider must be upright and have room',
  'Aufspringen': 'Jump up', 'Roll auf 0° und die Beine 3 s nivellieren (nur wenn er steht)': 'Roll to 0° and level the legs for 3 s (only while standing)',
  'Neu ausrichten': 'Re-level',
  'Neu ausrichten: Roll auf 0° und die Beine 3 s nivellieren (nur wenn er steht)': 'Re-level: roll to 0° and level the legs for 3 s (only while standing)', 'Aktion 254, Not-Stopp': 'Action 254, emergency stop', 'Absetzen': 'Sit down',
  'Extras': 'Extras', 'Aktionen · LEDs · Emotionen': 'Actions · LEDs · Emotions',
  'System · Aktionen · LEDs · Emotionen': 'System · Actions · LEDs · Emotions', 'System': 'System',
  'Bildverarbeitung starten': 'Start vision', 'Bildverarbeitung stoppen': 'Stop vision', '(Kamera und Auswertung)': '(camera and processing)',
  'Aufzeichnung beenden': 'Stop recording', 'Bildverarbeitung (Kamera und Auswertung) starten': 'Start vision (camera and processing)',
  'Aufzeichnen: Rohbilder und Metadaten am Rider speichern. Schreibt nur, solange er balanciert; Absetzen beendet die Aufnahme, jedes Aufstehen beginnt eine neue.':
    'Record: store raw images and metadata on the Rider. Writes only while balancing; sitting down ends the recording, every stand-up starts a new one.',
  'Bildverarbeitung (Kamera und Auswertung) starten oder stoppen': 'Start or stop vision (camera and processing)', 'Aktionen': 'Actions', 'LEDs': 'LEDs',
  'Fährt bzw. dreht selbst: nur im Modus idle, braucht Platz': 'Drives or turns by itself: mode idle only, needs room',
  'Alle aus': 'All off', 'Strobe': 'Strobe', 'Gilt für alle LEDs, bleibt über Neustarts gespeichert': 'Applies to all LEDs, kept across restarts',
  'Helligkeit': 'Brightness', 'Emotionen (Display)': 'Emotions (display)', 'keine gefunden in': 'none found in',
  'Geplanten Weg des Planers (Zum Start, Streife, Haustier-Suche) aus der Raumkarte ins Bild legen; im Browser gerechnet aus Kartenpose, Nick, Roll und Kamerahöhe':
    'Overlay the planner path from the room map on the image (Home, Patrol, Pet search); computed in the browser from map pose, pitch, roll and camera height',
  'Planerbahn': 'Planner path', 'Anzeige im Bild': 'Image overlays',
  'Bild am IMU-Nick festhalten: Bild senkrecht so verschieben, dass der IMU-Nick fest steht; stimmt die IMU, steht die Szene still':
    'Pin image to IMU pitch: shift the image vertically so the IMU pitch stays fixed; if the IMU is right, the scene stands still',
  'Roll ausgleichen: Bild um den IMU-Roll drehen; stimmt der Roll, stehen senkrechte Kanten senkrecht und die cyan Linie waagerecht':
    'Level roll: rotate the image by the IMU roll; if the roll is right, vertical edges stand vertical and the cyan line is level',
  'Planerbahn: geplanten Weg des Planers (Zum Start, Streife, Haustier-Suche) aus der Raumkarte ins Bild legen; im Browser gerechnet aus Kartenpose, Nick, Roll und Kamerahöhe':
    'Planner path: overlay the planner path from the room map on the image (Home, Patrol, Pet search); computed in the browser from map pose, pitch, roll and camera height',
  'Bildmitte: konfigurierte Bildmitte einblenden, gestrichelt die Spalte der Fahrtrichtung (W/2), Kreis bei Zeile H/2':
    'Image centre: show the configured image centre, dashed the column of the driving direction (W/2), circle at row H/2',
  'Bild senkrecht so verschieben, dass der IMU-Nick fest steht: stimmt die IMU, steht die Szene still': 'Shift the image vertically so the IMU pitch stays fixed: if the IMU is right, the scene stands still',
  'Bild am IMU-Nick festhalten': 'Pin image to IMU pitch', 'Roll ausgleichen': 'Level roll', 'Bildmitte': 'Image centre',
  'Konfigurierte Bildmitte einblenden: gestrichelt die Spalte der Fahrtrichtung (W/2), Kreis bei Zeile H/2':
    'Show the configured image centre: dashed the column of the driving direction (W/2), circle at row H/2',
  'IMU zum Bild': 'IMU at frame', 'IMU fehlt – Nick nur aus dem Bild': 'IMU missing – pitch from image only',
  'ohne IMU': 'without IMU', 'keine IMU': 'no IMU', 'Kurs': 'Heading', 'Alter': 'Age', 'Nickrate Bild/IMU': 'Pitch rate image/IMU',
  'IMU-Wert dieses Bildes (Bildzeit + [imu] time_offset_ms), mit dem xgonav Boden, Karten und die cyan Linie rechnet. Nickrate: aus dem Bild (Ego-Motion) gegen die IMU; weichen sie dauerhaft ab, stimmen Zeitversatz oder Brennweite nicht': 'IMU value of this frame (frame time + [imu] time_offset_ms) that xgonav uses for floor, maps and the cyan line. Pitch rate: from the image (ego-motion) vs. IMU; a lasting difference means time offset or focal length are off',
  'Bild um den IMU-Roll drehen: stimmt der Roll, stehen senkrechte Kanten senkrecht und die cyan Linie waagerecht': 'Rotate the image by the IMU roll: if the roll is right, vertical edges stand vertical and the cyan line is level',
  'Raumkarte': 'Room map', 'Ausschnitt': 'View', 'eingeklappt: kein Nachladen': 'collapsed: no reloading', 'Karte': 'Map',
  'Raumkarte jetzt speichern ([grid] file)': 'Save the room map now ([grid] file)', 'Speichern': 'Save',
  'Gesperrt ändern Fahrten die Karte nicht': 'While locked, drives do not change the map', 'Karte sperren': 'Lock map', 'Karte entsperren': 'Unlock map',
  'Karte leeren; Route und Sperrzonen gehen verloren': 'Clear the map; route and no-go zones are lost', 'Zurücksetzen': 'Reset',
  'Start · Pose': 'Start · Pose', 'Startpunkt = aktuelle Pose': 'Start point = current pose', 'Start hier': 'Start here',
  'Der Rider wurde versetzt: Die Karte wird nicht mehr beschrieben, bis der Standort bestätigt ist':
    'The Rider was moved: the map is not written until the position is confirmed',
  'Pose verwerfen': 'Discard pose', 'Route · Zonen': 'Route · Zones', 'Startbild': 'Start image',
  'Bild am Startpunkt (Anker des Abgleichs); nach dem Absetzen weg, neu nach dem Aufspringen':
    'Image at the start point (anchor of the start image check); gone after setting down, new after standing up',
  'Wegpunkte der Streife per Klick auf die Karte setzen': 'Set patrol waypoints by clicking on the map', 'Route setzen': 'Set route',
  'Bereich, den der Rider nie befährt: Ecken anklicken': 'Area the Rider never enters: click its corners', 'Sperrzone': 'No-go zone',
  'Alle Sperrzonen löschen': 'Delete all no-go zones', 'Zonen löschen': 'Delete zones',
  'ziehen: verschieben · Mausrad: zoomen · Doppelklick: zurück zum Rider': 'drag: pan · wheel: zoom · double-click: back to the Rider',
  'im Kreis': 'loop', 'Letzten weg': 'Remove last', 'Übernehmen': 'Apply', 'Abbrechen': 'Cancel', 'Pose unbekannt': 'Pose unknown',
  '. Die Karte wird nicht beschrieben, „Zum Start“, Streife und Erkunden sind gesperrt; Sperrzonen gelten an falscher Stelle.':
    '. The map is not written; Home, Patrol and Explore are blocked; no-go zones apply at the wrong place.',
  'Die Pose springt auf den Startpunkt': 'The pose jumps to the start point', 'Steht am Startpunkt': 'At start point',
  'Der Rider wurde nicht versetzt': 'The Rider was not moved', 'Pose stimmt': 'Pose is correct',
  'Einstellungen durchsuchen': 'Search settings', 'Suchen (Schlüssel, Text)': 'Search (key, text)',
  'nur in config.toml gesetzte': 'only set in config.toml', 'Schließen': 'Close', 'Sicherungen': 'Backups',
  'Nicht gespeicherte Änderungen verwerfen': 'Discard unsaved changes', 'Verwerfen': 'Discard',
  'Speichern und neu starten': 'Save and restart', 'xgonav neu starten: liest config.toml neu ein': 'Restart xgonav: re-reads config.toml',
  'Neu starten': 'Restart',
  // dynamisch
  'Status': 'Status', 'Befehl v / w': 'Command v / w', 'Kurs': 'Heading', 'Kamera / Schleife': 'Camera / loop',
  'Schleife p95 / max': 'Loop p95 / max', 'Detektor': 'Detector', 'Nicken': 'Pitch', 'Gier / Nick': 'Yaw / pitch', 'TTC / Schärfe': 'TTC / sharpness',
  'Rundumblick': 'looking around', 'fährt zu einer Stelle': 'driving to a spot', 'schaut': 'looking', 'setzt zurück': 'backing up',
  'befreit sich': 'freeing itself', 'fährt zum Start': 'driving home', 'fertig': 'done', 'fährt': 'driving',
  'verbunden': 'connected', 'getrennt': 'disconnected', 'keine Verbindung': 'no connection',
  'keine Verbindung zum Rider': 'no connection to the Rider', 'keine Verbindung zu xgonav': 'no connection to xgonav',
  'balanciert': 'balancing', 'liegt': 'lying', 'gefallen': 'fallen', 'Akku leer': 'battery low',
  'Aktorfehler': 'actuator fault', 'Fehler': 'fault', 'läuft': 'running', 'gestoppt': 'stopped', 'Stoppen': 'Stop', 'Starten': 'Start',
  'Aufzeichnung aus': 'Stop recording', 'Bilder': 'frames', 'wartet: liegt': 'waiting: lying down',
  'wartet: Bildverarb. aus': 'waiting: vision off', 'wartet auf Bild': 'waiting for image', 'Aufzeichnung aus: ': 'Recording off: ',
  'Bodenmodell': 'Floor model', 'Aktivieren': 'Activate', 'FloorNet-Gewichte aus [freespace] net_dir': 'FloorNet weights from [freespace] net_dir',
  'xgonav lädt und prüft das Netz, stellt den Symlink [freespace] net um und tauscht das Bodenmodell ohne Neustart (nur Idle/Hand)':
    'xgonav loads and checks the net, repoints the symlink [freespace] net and swaps the floor model without restart (idle/manual only)',
  ' (geladen)': ' (loaded)', 'kein Netz geladen': 'no net loaded', 'kein Modell in ': 'no model in ',
  'net ist kein Symlink: Wechsel erst nach deploy/install.sh': 'net is not a symlink: switching only after deploy/install.sh',
  'Bodenmodell wechseln auf ': 'Switch floor model to ', '? Nur in Idle oder Hand; das Bodenmodell ist danach kurz aus.': '? Idle or manual only; the floor model is briefly off afterwards.',
  'lädt …': 'loading …',
  'YOLO aus': 'YOLO off', ' – YOLO ist aus ([detector] enabled)': ' – YOLO is off ([detector] enabled)',
  'Steht der Rider (aufgestanden) am Startpunkt, in Startrichtung? Die Pose springt dann dorthin. Sind Treppen und Kanten abgesperrt?':
    'Is the Rider (standing) at the start point, facing the start direction? The pose then jumps there. Are stairs and edges blocked off?',
  'Steht der Rider (aufgestanden) am Startpunkt, in Startrichtung? Die Pose springt dann dorthin.':
    'Is the Rider (standing) at the start point, facing the start direction? The pose then jumps there.',
  'Steht der Rider (aufgestanden) genau dort, wo ihn die Karte zeigt, in dieser Richtung?':
    'Is the Rider (standing) exactly where the map shows it, facing that direction?',
  'Sollwert u8 ': 'Setpoint u8 ',
  'Fehler ': 'Error ', 'ist ': 'actual ', 'Start ': 'home ', 'belegt ': 'occupied ',
  'Erkunden: ': 'Explore: ', ' Stellen offen': ' spots open', ' übergangen': ' skipped',
  'Streife: ': 'Patrol: ', 'Punkt ': 'point ', ' von ': ' of ', 'Runde ': 'round ', 'ausgelassen: ': 'skipped: ',
  'Ecken der Sperrzone anklicken: ': 'Click the corners of the no-go zone: ', 'Wegpunkte anklicken: ': 'Click waypoints: ',
  'Route nicht lesbar': 'Route not readable', 'Eine Sperrzone braucht mindestens 3 Ecken': 'A no-go zone needs at least 3 corners',
  'Alle Sperrzonen löschen?': 'Delete all no-go zones?',
  'Raumkarte zurücksetzen? Route und Sperrzonen gehen dabei verloren.': 'Reset the room map? Route and no-go zones will be lost.',
  'Lage kalibrieren? Der Rider muss balancieren und frei stehen; er regelt sich ein paar Sekunden auf der Stelle ein.':
    'Calibrate attitude? The Rider must balance and stand free; it settles on the spot for a few seconds.',
  'Aufspringen? Der Rider muss auf den Rädern stehen und rundherum Platz haben – er springt hoch und schwingt dabei aus.':
    'Jump up? The Rider must stand on its wheels with room all around – it jumps up and swings out.',
  'ESP32 neu starten? Geht nur, wenn der Rider nicht balanciert. Danach wie nach dem Einschalten: Steht er aufrecht, balanciert er von selbst los.':
    'Restart the ESP32? Only works while the Rider is not balancing. Afterwards as after power-on: once upright it starts balancing by itself.',
  'ESP-Neustart': 'ESP restart', 'Aktion 237: ESP32 neu starten, nur wenn er nicht balanciert': 'Action 237: restart the ESP32, only while it is not balancing',
  'Rider wirklich herunterfahren?': 'Really shut down the Rider?',
  'Kern ': 'Core ', '↓ empfangen': '↓ received', '↑ gesendet': '↑ sent', ' · Qualität ': ' · quality ', 'Pegel': 'Level',
  ' · Last ': ' · load ', 'Nick': 'Pitch', 'Gier': 'Yaw', 'keine IMU-Daten': 'no IMU data',
  // Einstellungen
  'Zahl erwartet': 'number expected', 'ganze Zahl erwartet': 'integer expected', 'Typ unbekannt': 'unknown type',
  'steht in config.toml': 'set in config.toml', 'gesetzt': 'set', 'wird Default': 'becomes default', 'geändert': 'changed',
  'Default': 'Default', 'Default aus config.py: ': 'Default from config.py: ', ' – entfernt die Zeile aus config.toml': ' – removes the line from config.toml',
  '(leer)': '(empty)', 'In config.toml nicht lesbar ({x}), nur per ssh änderbar.': 'Not readable in config.toml ({x}), change only via ssh.',
  'Default: ': 'Default: ', ' · in Datei: ': ' · in file: ', 'gesetzt / Felder': 'set / fields', ', ungespeichert geändert': ', changed, unsaved',
  'Nichts gefunden.': 'Nothing found.', 'Keine Einträge.': 'No entries.', 'lädt …': 'loading …',
  ' ist nicht schreibbar (Gruppe video, 0664: deploy/install.sh)': ' is not writable (group video, 0664: deploy/install.sh)',
  '{n} ungültige Eingabe(n)': '{n} invalid input(s)', '{n} ungespeicherte Änderung(en) – wirkt erst nach Neustart': '{n} unsaved change(s) – takes effect after restart',
  'Gespeichert. Wirkt erst nach Neustart von xgonav.': 'Saved. Takes effect after xgonav restarts.', 'Keine Änderungen. ': 'No changes. ',
  'Zurückspielen': 'Restore', 'noch keine': 'none yet',
  'config.toml durch die Sicherung {name} ersetzen? Der jetzige Stand wird vorher gesichert. Wirkt nach Neustart.':
    'Replace config.toml with backup {name}? The current state is backed up first. Takes effect after restart.',
  'Laden fehlgeschlagen: ': 'Loading failed: ', 'Alle ungespeicherten Änderungen verwerfen?': 'Discard all unsaved changes?',
  'In config.toml schreiben (vorher Sicherung):': 'Write to config.toml (backup first):', 'Danach xgonav neu starten.': 'Then restart xgonav.',
  '(Default ': '(default ', 'leer': 'empty',
  'xgonav neu starten? Er liest dabei config.toml neu ein; Bild und Status sind einige Sekunden weg.':
    'Restart xgonav? It re-reads config.toml; image and status are gone for a few seconds.',
  'Trotzdem neu starten? Laufende Fahrt, Aufzeichnung und Modus enden.': 'Restart anyway? Running drive, recording and mode will end.',
  'xgonav startet neu …': 'xgonav is restarting …',
  'Sprechen': 'Talk', 'Auflegen': 'Hang up', 'Drücken zum Sprechen': 'Push to talk', 'Sprechtaste': 'Talk button', 'ich': 'me',
  'Nur im Modus Hand': 'Manual mode only', 'Gedrückt halten (oder Leertaste)': 'Hold (or space bar)',
  'Mikrofon nur senden, solange die Sprechtaste oder die Leertaste gedrückt ist; der Rider ist dann still':
    'Send the microphone only while the talk button or space bar is held; the Rider is muted meanwhile',
  'Lautstärke des Riders hier im Browser': 'Rider volume here in the browser', 'Lautstärke': 'Volume',
  'Gegensprechen mit dem Rider: sein Mikrofon hier hören, ins eigene Mikrofon sprechen (8 kHz). Nur im Modus Hand.':
    'Intercom with the Rider: hear its microphone here, talk into your own (8 kHz). Manual mode only.',
  'verbinde …': 'connecting …', 'Gespräch läuft': 'talking', 'Gespräch beendet': 'call ended',
  'Mikrofon nicht freigegeben': 'microphone not allowed', 'Browser kann kein Mikrofon (HTTPS nötig)': 'browser has no microphone access (HTTPS needed)',
  'keine Verbindung zum Gegensprech-Dienst': 'no connection to the intercom service',
  'nur im Handmodus': 'manual mode only', 'besetzt (anderer Browser spricht)': 'busy (another browser is talking)',
  'Mikrofon belegt oder arecord beendet': 'microphone busy or arecord ended', 'aplay beendet (Lautsprecher belegt?)': 'aplay ended (speaker busy?)',
  'arecord/aplay starten nicht': 'arecord/aplay do not start',
  'xgonav nicht erreichbar (läuft der Dienst?)': 'xgonav not reachable (is the service running?)',
  'Kamera senden': 'Send camera', 'hier': 'here', 'Lautstärke hier': 'Volume here', 'Lautstärke am Rider': 'Volume on the Rider',
  'Verstärkung der eigenen Stimme zum Rider (mit weichem Begrenzer, kein Übersteuern)': 'Gain of your voice to the Rider (soft limiter, no clipping)', 'Telefon': 'Phone', 'bereit': 'ready', 'Bilder je Sekunde': 'Frames per second',
  'Eigenes Kamerabild aufs Display des Riders (320×240, im Browser umgerechnet; 0,75 MB/s bei 5 Bildern/s, am Rider ~9 % eines Kerns, bei 10/s doppelt). Nur während eines Gesprächs.':
    'Own camera image on the Rider display (320×240, converted in the browser; 0.75 MB/s at 5 frames/s, ~9 % of a core on the Rider, double at 10/s). Only during a call.',
  'Bilder je Sekunde aufs Display (WLAN: je Bild 150 kB)': 'Frames per second to the display (Wi-Fi: 150 kB per frame)',
  'Bild am Display': 'image on display', 'Kamera nicht freigegeben': 'camera not allowed', 'kein Gespräch': 'no call',
  'besetzt (anderes Bild läuft)': 'busy (another image is running)', 'Display-Daemon läuft nicht': 'display daemon not running',
};
const EN_HTML = {
  'hilfe-radar': `<summary>Ray = sound · length = loudness · fades in 12 s</summary>
      <p>Direction from the time difference between the two microphones (<code>[ortung]</code>). Up is forward;
        when the Rider turns, rays and spokes turn with it (IMU yaw), older rays keep pointing at the source.
        Two microphones only separate left and right: every ray therefore has a dashed twin to the rear, the
        source is on one of them. Length = level above the noise floor, opacity = quality and age. Listens only
        in the modes of <code>[ortung] modes</code>, never in rest mode; own sounds are ignored.</p>`,
  'hilfe-lage': `<summary>Camera height, roll, stand up and sit down</summary>
      <p>Camera height: value of the floor geometry from <code>cam_h_u8</code> (lens centre above floor, setpoint u8
        in the tooltip); the legs need about 1 s afterwards. The value matches the folding-rule measurement
        (floor grid 2026-11-10). Above u8 200 nothing changes
        any more. Lower limit keeps a margin to the minimum; in Follow/Explore xgonav sets the height itself.</p>
      <p>Roll: + = right.</p>
      <p>Balancing (firmware R-1.1.6): after an ESP32 restart the Rider balances only after
        “Stand up”. If it is lying, the firmware waits until it is upright – then the wheels start immediately.
        “Jump up” (action 253, like <code>!standup</code>) makes it jump up – <b>Rider must be upright and have room</b>.
        “Sit down” is an emergency stop (action 254, like <code>!stop</code>): it sits down in a controlled way and
        stays off until the next stand-up.</p>
      <p>“Re-level” (⚖ next to the roll slider): the legs have a dead band; with the same command it stands up to 2.6° differently
        tilted. The button sets roll to 0° and lets the firmware level for 3 s; it then stays like that. This
        also happens once after every stand-up. While driving it can slowly tilt again.</p>`,
  'hilfe-karte': `<summary>grey unknown · light free · dark occupied · green path · cyan plan · blue start · red Rider</summary>
      <p>Mouse: drag pans, the wheel zooms around the pointer, double-click centres on the Rider again
        and follows it.</p>
      <p>Origin = start pose (up = forward at start). Distance and rotation from the wheels (also with
        vision stopped); without markers the map drifts over time.</p>
      <p>“Explore” maps the room: first a look around, then it drives to the borders between known floor and
        unknown until nothing reachable is left (or a time or battery limit). Then it saves the map and
        drives home.</p>
      <p>“Set route”: click the points on the map in order; magenta is the saved route. Every point
        must lie on known, reachable floor. “No-go zone”: click the corners of an area it must never enter,
        e.g. stairs (red). It applies to all modes except manual control, but drifts with the map: it
        does not replace a physical barrier. Route and zones belong to this map; after “Reset” they are gone.</p>
      <p>“Patrol” drives the route and finally home. The Rider must stand at the start point for it. Below 30 % battery it
        returns, at 15 % it sits down. After “Explore” the map is locked so drives no longer change it.</p>
      <p>“Home” plans a path over the map and drives it. The map is saved regularly
        and loaded when xgonav starts; the Rider must then be at the start point again.</p>`,
};
let LANG = 'de';
try { if (localStorage.getItem('xgonav.lang') === 'en') LANG = 'en'; } catch (e) {}
const I18N_DEBUG = /[?&]i18n=debug\b/.test(location.search), i18nFehlt = new Set();
const i18nKey = s => s.replace(/\s+/g, ' ').trim();
function trEn(s) {
  if (s in EN) return EN[s];
  const k = i18nKey(s);
  if (k in EN) return EN[k];
  if (I18N_DEBUG && /[A-Za-zÄÖÜäöüß]{2}/.test(k) && !i18nFehlt.has(k)) { i18nFehlt.add(k); console.warn('i18n fehlt:', k); }
  return null;
}
// Übersetzen; Platzhalter {x} aus v
function tr(s, v) {
  let r = LANG === 'en' ? (trEn(s) ?? s) : s;
  if (v) for (const k in v) r = r.replace('{' + k + '}', v[k]);
  return r;
}
const LOCALE = () => LANG === 'en' ? 'en-GB' : 'de-DE';
// Ein Textknoten bzw. Attribut: Original merken, nur zurücksetzen, solange der Code es nicht inzwischen neu gesetzt hat
function i18nWert(halter, key, jetzt, setzen) {
  const o = halter._i18n ??= {};
  if (o[key] && jetzt === o[key].en) { if (LANG === 'de') { setzen(o[key].de); delete o[key]; } return; }
  delete o[key];
  if (LANG !== 'en') return;
  const en = trEn(jetzt);
  if (en == null) return;
  const neu = jetzt.replace(/^(\s*)[\s\S]*?(\s*)$/, (_, a, b) => a + en + b);
  o[key] = {de: jetzt, en: neu};
  setzen(neu);
}
function spracheAnwenden() {
  document.documentElement.lang = LANG;
  document.querySelectorAll('[data-lang]').forEach(b => b.classList.toggle('aktiv', b.dataset.lang === LANG));
  for (const el of document.querySelectorAll('[data-i18n-html]')) {
    el._de ??= el.innerHTML;
    const en = EN_HTML[el.dataset.i18nHtml];
    el.innerHTML = LANG === 'en' && en ? en : el._de;
  }
  const w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT, {acceptNode: n =>
    n.parentElement.closest('script,style,pre,[data-i18n-html]') ? NodeFilter.FILTER_REJECT : NodeFilter.FILTER_ACCEPT});
  for (let n; (n = w.nextNode());) if (n.nodeValue.trim()) i18nWert(n, 'text', n.nodeValue, v => { n.nodeValue = v; });
  for (const el of document.querySelectorAll('[title],[aria-label],[placeholder]')) {
    if (el.closest('[data-i18n-html]')) continue;
    for (const a of ['title', 'aria-label', 'placeholder']) {
      const v = el.getAttribute(a);
      if (v) i18nWert(el, a, v, x => el.setAttribute(a, x));
    }
  }
}
// Ursprünglicher (deutscher) Wert eines Attributs, auch wenn es gerade übersetzt angezeigt wird
const attrDe = (el, a) => el._i18n?.[a]?.de ?? el.getAttribute(a) ?? '';
function sprache(l) {
  LANG = l === 'en' ? 'en' : 'de';
  try { localStorage.setItem('xgonav.lang', LANG); } catch (e) {}
  spracheAnwenden();
  if (window.letzterStatus) statusZeigen(window.letzterStatus);
  zieleLaden();
  bodenmodellLaden();
  if (einst.d) { einstSicherungen(); einstZeigen(); }
  verlaufZeichnen();
}

// Kennzahlen-Kacheln; Modus, Akku und Zustand stehen in der Kopfleiste, Bildverarbeitung als Knopf ▶/■
const felder = [
  ['Status', s => s.status, true],
  ['Befehl v / w', s => s.cmd ? s.cmd.v.toFixed(2) + ' / ' + s.cmd.w.toFixed(2) : '-'],
  ['Kurs', s => s.yaw_imu != null ? Math.round(s.yaw_imu) + '°' : '-'],
  ['Kamera / Schleife', s => (s.fps ?? '-') + ' fps / ' + (s.loop_ms ?? '-') + ' ms'],
  ['Schleife p95 / max', s => s.loop_p95 != null ? s.loop_p95 + ' / ' + s.loop_max + ' ms' : '-'],
  ['Detektor', s => s.det_ms != null ? Math.round(s.det_ms) + ' ms' : '-'],
  ['Nicken', s => (s.pitch_src ?? '-') + ' ' + (s.pitch_deg != null ? s.pitch_deg.toFixed(1) + '°' : '')],
  ['Gier / Nick', s => s.ego ? s.ego.yaw_dps.toFixed(0) + ' / ' + s.ego.pitch_dps.toFixed(0) + ' °/s' : '-'],
  ['TTC / Schärfe', s => (s.ttc != null ? s.ttc.toFixed(1) + ' s' : '-') + ' / ' + (s.ego ? s.ego.sharp : '-')],
];
const ERKUNDEN = {scan: 'Rundumblick', ziel: 'fährt zu einer Stelle', face: 'schaut', settle: 'schaut', backup: 'setzt zurück',
  wander: 'befreit sich', heim: 'fährt zum Start', fertig: 'fertig'};
const MODI = {idle: 'Idle', manual: 'Hand', follow: 'Folgen', explore: 'Erkunden', approach: 'Anfahren', home: 'Zum Start', patrol: 'Streife', pet: 'Haustier'};
const STREIFE = {fahrt: 'fährt', schau: 'schaut', backup: 'setzt zurück', heim: 'fährt zum Start', fertig: 'fertig'};
const esc = t => String(t ?? '-').replace(/[<&]/g, c => c === '<' ? '&lt;' : '&amp;');
const $ = id => document.getElementById(id);
let aktModus = null, aktPipeline = null;
function zeige(text) { $('meldung').textContent = tr(text || ''); }
// Pille der Kopfleiste setzen: Wert und Bewertung (gut / warn / schlecht / neutral)
function pille(id, wert, art = '') {
  const p = $(id);
  p.querySelector('b').textContent = wert;
  p.className = 'pille' + (art ? ' ' + art : '');
}
// Firmware-Zustand (/proc/XGORider/state, robot.FW_ZUSTAND): abgesetzt (254, LEDs orange) ist kein Sturz (0, LEDs rot)
const ZUSTAND = {1: ['balanciert', 'gut'], 254: ['liegt', 'warn'], 0: ['gefallen', 'schlecht'], 253: ['bereit', 'warn'],
  250: ['Akku leer', 'schlecht'], 251: ['Aktorfehler', 'schlecht'], 252: ['Fehler', 'schlecht']};
function zustandPille(state, balancing) {
  if (state == null) {
    if (balancing == null) return pille('p-zustand', '-');
    return pille('p-zustand', tr(balancing ? 'balanciert' : 'liegt'), balancing ? 'gut' : 'warn');
  }
  const [t, art] = ZUSTAND[state] || ['Zustand ' + state, 'schlecht'];
  pille('p-zustand', tr(t), art);
}
// Verbindungsfehler verschwinden von selbst, sobald xgonav wieder antwortet (andere Meldungen bleiben)
const VERBINDUNG = /nicht erreichbar|keine Verbindung|no connection/;
function verbindungOk() {
  if (VERBINDUNG.test($('meldung').textContent)) zeige('');
  pille('p-verb', tr('verbunden'), 'gut');
}
function verbindungWeg() { pille('p-verb', tr('getrennt'), 'schlecht'); }
// Kamerabild über eine WebSocket-Verbindung. Jedes Bild wird sofort bestätigt, erst dann
// schickt xgonav das nächste (neueste): kein Rückstau wie beim MJPEG-Stream über schwaches
// WLAN (bis 20 s). Nachricht (nick=3, xgonav/web.py ws_kopf): 4 Byte Bildnummer, 10 x float32 row_stand,
// row_imu, Roll, cx, IMU-Nick, IMU-Kurs, Alter der IMU-Probe, Nickrate Bild, Nickrate IMU, Zustand, dann
// 4 x float32 Horizontzeile bei cx, Kartenpose x, y, th (Planerbahn); JPEG. Ein älteres xgonav schickt nur nick=2.
// Bei Abbruch nach 1 s neu verbinden.
// „Bild am IMU-Nick festhalten“ (Kameraleiste): Das Bild wird hier im Browser per CSS um row_stand − row_imu
// verschoben, sodass der IMU-Nick fest steht; die Bildpipeline rechnet dafür nichts: Normallage gegen den
// reinen IMU-Nick (ohne Bildanteil).
// „Roll ausgleichen“ (nick=2: dazu Roll und cx im Kopf): Das Bild wird um den Horizontpunkt bei cx um den
// IMU-Roll gedreht (CSS rotate, im Uhrzeigersinn = rechts tief ausgleichen). Stimmt der Roll, stehen senkrechte
// Kanten senkrecht und die cyan Linie liegt waagerecht. Am Pi rechnet dafür nichts zusätzlich.
let bildUrl = null, bildDy = 0, bildRoll = 0, bildCx = NaN, bildRowImu = NaN;
let bildStabil = false, bildRollAn = false, bildMitteAn = true;
// Symbol-Umschalter (Kameraleiste): an = blau hervorgehoben wie ein aktiver Knopf
function umschaltZeigen(id, an) {
  const b = $(id);
  b.classList.toggle('aktiv', an);
  b.setAttribute('aria-pressed', an ? 'true' : 'false');
}
try { bildStabil = localStorage.getItem('xgonav.stabil') === '1'; } catch (e) {}
try { bildRollAn = localStorage.getItem('xgonav.roll') === '1'; } catch (e) {}
try { bildMitteAn = localStorage.getItem('xgonav.mitte') !== '0'; } catch (e) {}
umschaltZeigen('hz-stabil', bildStabil);
umschaltZeigen('hz-roll', bildRollAn);
umschaltZeigen('hz-mitte', bildMitteAn);
// Konfigurierte Bildmitte (Spalte cx aus dem Bild-Kopf = W/2, Zeile H/2 wie geometry.py) als SVG über
// dem Bild; bekommt dieselbe CSS-Verschiebung/Drehung wie das <img>, steht also immer auf demselben Bildpunkt.
function bildMitte(W, H, transform, origin) {
  const s = $('bild-mitte');
  s.classList.toggle('aus', !bildMitteAn);
  if (!bildMitteAn) return;
  const x = Number.isFinite(bildCx) ? bildCx : W / 2, y = H / 2;
  s.setAttribute('viewBox', '0 0 ' + W + ' ' + H);
  for (const l of s.querySelectorAll('.lot')) { l.setAttribute('x1', x); l.setAttribute('x2', x); l.setAttribute('y2', H); }
  for (const l of s.querySelectorAll('.qx')) {
    l.setAttribute('x1', x - 15); l.setAttribute('x2', x + 15); l.setAttribute('y1', y); l.setAttribute('y2', y);
  }
  for (const c of s.querySelectorAll('circle')) { c.setAttribute('cx', x); c.setAttribute('cy', y); }
  s.style.transformOrigin = origin;
  s.style.transform = transform;
}
function bildVerschieben() {
  const b = $('bild'), H = b.naturalHeight || 480, W = b.naturalWidth || 640;
  const t = [];
  if (bildStabil && bildDy) t.push('translateY(' + (bildDy / H * 100).toFixed(3) + '%)');
  if (bildRollAn && bildRoll) {
    t.push('rotate(' + bildRoll.toFixed(2) + 'deg)');
    const ox = Number.isFinite(bildCx) ? bildCx / W * 100 : 50, oy = Number.isFinite(bildRowImu) ? bildRowImu / H * 100 : 50;
    b.style.transformOrigin = ox.toFixed(2) + '% ' + oy.toFixed(2) + '%';
  }
  b.style.transform = t.join(' ');
  bildMitte(W, H, b.style.transform, b.style.transformOrigin);
  planImBild(W, H, b.style.transform, b.style.transformOrigin);
}
// Planerbahn im Kamerabild. xgonav schickt nur Rohdaten: je Bild (nick=3) Horizontzeile bei cx und Kartenpose,
// 1×/s im Status Planpunkte (Karten-KS, m) und Kameradaten (f, Kamerahöhe, Linse vor der Achse). Gerechnet wird
// hier: Lochkamera über der Bodenebene (das Bild ist schon entzerrt), Nick aus der Horizontzeile wie die cyan
// Linie, Roll um (cx, Horizontzeile) wie freespace.roll_columns. Gerade Bodenstrecken bleiben im Bild gerade,
// deshalb genügt es, die Strecken vor der Kamera (z > Z_MIN) abzuschneiden. Am Pi kostet das nichts.
let bildPose = null, planAn = true;
try { planAn = localStorage.getItem('xgonav.plan') !== '0'; } catch (e) {}
umschaltZeigen('hz-plan', planAn);
function hzPlan(an) {
  planAn = an;
  umschaltZeigen('hz-plan', an);
  try { localStorage.setItem('xgonav.plan', an ? '1' : '0'); } catch (e) {}
  bildVerschieben();
}
function planImBild(W, H, transform, origin) {
  const svg = $('bild-plan'), s = window.letzterStatus, k = s && s.kamera, plan = s && s.plan, p = bildPose;
  const leer = () => { svg.innerHTML = ''; };
  if (!planAn || !k || !plan || plan.length < 2 || !p || ![p.row, p.x, p.y, p.th].every(Number.isFinite)) return leer();
  const Z_MIN = 0.05, f = k.f, h = k.cam_h_m, cx = Number.isFinite(bildCx) ? bildCx : W / 2;
  const e = Math.atan((p.row - k.H / 2) / f), ce = Math.cos(e), se = Math.sin(e);
  const ct = Math.cos(p.th), st = Math.sin(p.th);
  const r = Number.isFinite(bildRoll) ? bildRoll * Math.PI / 180 : 0, cr = Math.cos(r), sr = Math.sin(r);
  // Kartenpunkt -> Kamera: X vor dem Kamerafußpunkt, Y links, z entlang der optischen Achse
  const kam = ([wx, wy]) => {
    const dx = wx - p.x, dy = wy - p.y, X = ct * dx + st * dy - k.cam_fwd_m, Y = -st * dx + ct * dy;
    return {X, Y, z: X * ce - h * se};
  };
  const bild = q => {
    const u = cx - f * q.Y / q.z, v = k.H / 2 + f * (q.X * se + h * ce) / q.z;
    const du = u - cx, dv = v - p.row;
    return [cx + du * cr + dv * sr, p.row - du * sr + dv * cr];
  };
  const zuege = [];
  let zug = [];
  for (let i = 0; i + 1 < plan.length; i++) {
    let a = kam(plan[i]), b = kam(plan[i + 1]);
    if (a.z < Z_MIN && b.z < Z_MIN) { if (zug.length) { zuege.push(zug); zug = []; } continue; }
    const schnitt = (a, b) => { const t = (Z_MIN - a.z) / (b.z - a.z);
      return {X: a.X + t * (b.X - a.X), Y: a.Y + t * (b.Y - a.Y), z: Z_MIN}; };
    const ab = a.z < Z_MIN, bb = b.z < Z_MIN;
    if (ab) a = schnitt(a, b);
    if (bb) b = schnitt(a, b);
    if (ab || !zug.length) { if (zug.length) zuege.push(zug); zug = [bild(a)]; }
    zug.push(bild(b));
    if (bb) { zuege.push(zug); zug = []; }
  }
  if (zug.length) zuege.push(zug);
  const pkt = z => z.map(([x, y]) => x.toFixed(1) + ',' + y.toFixed(1)).join(' ');
  const ziel = kam(plan[plan.length - 1]);
  let html = zuege.map(z => '<polyline class="halo" points="' + pkt(z) + '"/>').join('')
           + zuege.map(z => '<polyline class="strich" points="' + pkt(z) + '"/>').join('');
  if (ziel.z >= Z_MIN) { const [x, y] = bild(ziel); html += '<circle cx="' + x.toFixed(1) + '" cy="' + y.toFixed(1) + '" r="5"/>'; }
  svg.setAttribute('viewBox', '0 0 ' + W + ' ' + H);
  svg.style.transformOrigin = origin;
  svg.style.transform = transform;
  svg.innerHTML = html;
}
// IMU-Wert des gezeigten Bildes (aus dem Bild-Kopf, nicht aus live.php): damit rechnet xgonav dieses Bild.
// Zählt Bilder ohne gültige IMU seit dem Laden (dann Nick nur aus dem Bild, rot).
let imuBilder = 0, imuOhne = 0;
function imuBildZeigen(w) {
  const el = $('imu-bild'), z = (v, n, e) => Number.isFinite(v) ? (v >= 0 ? '+' : '') + v.toFixed(n) + e : '-';
  imuBilder++;
  if (w.ok !== 1) imuOhne++;
  const ohne = imuOhne ? ' · ' + tr('ohne IMU') + ' ' + imuOhne + '/' + imuBilder : '';
  if (!Number.isFinite(w.ok)) { el.textContent = tr('keine IMU'); el.classList.add('schlecht'); return; }
  el.classList.toggle('schlecht', w.ok !== 1);
  el.textContent = (w.ok === 1 ? tr('IMU zum Bild') : tr('IMU fehlt – Nick nur aus dem Bild')) + ': '
    + tr('Nick') + ' ' + z(w.nick, 2, '°') + ' · ' + tr('Roll') + ' ' + z(w.roll, 2, '°') + ' · ' + tr('Kurs') + ' '
    + z(w.kurs, 1, '°') + ' · ' + tr('Alter') + ' ' + z(w.alter, 0, ' ms') + ' · ' + tr('Nickrate Bild/IMU') + ' '
    + z(w.rateBild, 1, '') + ' / ' + z(w.rateImu, 1, ' °/s') + ohne;
}
// Ruhemodus: kein Bild, keine Karte, keine Lage; Status nur alle 5 s (drosselt tools/live.php selbst)
let ruhe = false, bildWs = null, liveWs = null;
function ruheZeigen(an, akku) {
  $('ruhe-akku').textContent = akku != null ? akku + ' %' : '-';
  if (an === ruhe) return;
  ruhe = an;
  $('ruhe-decke').classList.toggle('an', an);
  if (an) {
    bildSoll(false);
  } else {
    $('ruhe-fehler').textContent = '';
    karteHolen();
  }
  liveImu();
}
async function ruheSchalten(an) {
  try {
    const r = await fetch('api.php?a=ruhe&on=' + (an ? '1' : '0'), {method: 'POST'});
    const j = await r.json();
    const f = j.error || (j.fehler && j.fehler.join(' · ')) || '';
    zeige(f);
    $('ruhe-fehler').textContent = f;
  } catch (e) { zeige('keine Verbindung'); }
  status();
}
// Bild-WebSocket nur bei laufender Bildverarbeitung (Zustand aus dem Status); gestoppt bleibt das
// letzte Bild abgedunkelt stehen und xgonav kodiert nichts.
let bildAn = false;
function bildSoll(an) {
  $('bild').classList.toggle('aus', !an);
  if (an === bildAn) return;
  bildAn = an;
  if (an) { if (!bildWs) bildVerbinden(); }
  else if (bildWs) bildWs.close();
}
function bildVerbinden() {
  if (!bildAn) return;
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host +
                           location.pathname.replace(/[^/]*$/, '') + 'ws?nick=3');
  ws.binaryType = 'arraybuffer';
  // Bild erst dekodieren, dann Bild und Verschiebung/Drehung im selben Schritt setzen: sonst zeichnet der
  // Browser das neue Bild einmal mit der alten Lage (beim ersten Bild ohne, im Liegen bis 160 px Sprung).
  let neuestes = 0;
  ws.onmessage = ev => {
    ws.send('ok');                                    // Bestätigung: nächstes Bild bitte
    const d = new DataView(ev.data), stand = d.getFloat32(4), imu = d.getFloat32(8), r = d.getFloat32(12);
    const cx = d.getFloat32(16), nr = ++neuestes;
    const w = {nick: d.getFloat32(20), kurs: d.getFloat32(24), alter: d.getFloat32(28),
               rateBild: d.getFloat32(32), rateImu: d.getFloat32(36), ok: d.getFloat32(40), roll: r};
    // nick=3: 14 Werte; ein älteres xgonav (nick=2) schickt das JPEG schon ab Byte 44 (FF D8)
    const mitPose = ev.data.byteLength > 60 && !(d.getUint8(44) === 0xFF && d.getUint8(45) === 0xD8);
    const pz = mitPose ? {row: d.getFloat32(44), x: d.getFloat32(48), y: d.getFloat32(52), th: d.getFloat32(56)} : null;
    const url = URL.createObjectURL(new Blob([new Uint8Array(ev.data, mitPose ? 60 : 44)], {type: 'image/jpeg'}));
    const vor = new Image();
    vor.src = url;
    vor.decode().then(() => {
      if (nr !== neuestes) { URL.revokeObjectURL(url); return; }   // inzwischen kam ein neueres
      bildDy = Number.isFinite(stand) && Number.isFinite(imu) ? stand - imu : 0;
      bildRoll = Number.isFinite(r) ? r : 0;
      bildCx = cx;
      bildRowImu = imu;
      bildPose = pz;
      $('bild').src = url;
      bildVerschieben();
      imuBildZeigen(w);
      if (bildUrl) URL.revokeObjectURL(bildUrl);
      bildUrl = url;
    }, () => URL.revokeObjectURL(url));
  };
  bildWs = ws;
  ws.onclose = () => { if (bildWs === ws) bildWs = null; if (bildAn) setTimeout(bildVerbinden, 1000); };
}
// Laufzeitdaten per WebSocket (tools/live.php): sys + xgonav-Status 1×/s in einem Paket,
// IMU + Räder alle 0,3 s, wenn er balanciert, sonst 1×/s. Kein Polling mehr.
function liveVerbinden() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host +
                           location.pathname.replace(/[^/]*$/, '') + 'live');
  ws.onopen = () => { liveWs = ws; liveImu(true); };
  ws.onmessage = ev => {
    const m = JSON.parse(ev.data);
    if (m.imu) imuVerarbeiten(m.imu);
    if (m.sys) {
      sysVerarbeiten(m.sys);
      statusZeigen(m.status, m.status_fehler);
    }
  };
  ws.onclose = () => {
    if (liveWs === ws) liveWs = null;
    hoeheAusStatus = false; zeige('keine Verbindung zum Rider'); verbindungWeg();
    setTimeout(liveVerbinden, 1000);
  };
}
// IMU nur, wenn die Lage-Karte oder das Schallradar (dreht mit der Gier mit) offen und der Tab sichtbar ist;
// since: nach Neuverbinden keine Doppelten
let liveImuAn = null;
function liveImu(neu = false) {
  const an = !ruhe && ($('lagekarte').open || $('radarkarte').open) && !document.hidden;
  if (!liveWs || (an === liveImuAn && !neu)) return;
  liveImuAn = an;
  liveWs.send('imu ' + (an ? 1 : 0) + (neu ? ' ' + imuSeq + ' ' + radSeq : ''));
}
document.addEventListener('visibilitychange', () => liveImu());
async function befehl(a, p = {}, still = false) {
  const q = new URLSearchParams({a, ...p});
  try {
    const r = await fetch('api.php?' + q, {method: 'POST'});
    const j = await r.json();
    if (!still || !r.ok) zeige(r.ok ? '' : (j.error || (tr('Fehler ') + r.status)));
    return r.ok;
  } catch (e) { zeige('keine Verbindung'); verbindungWeg(); return false; }
  finally { if (!still) status(); }
}
function modus(n) { return befehl('mode', {name: n, target: $('ziel').value}); }
// Ziele für Folgen/Anfahren: die Klassen, die YOLO am Rider meldet; einmal je Seitenaufruf und nach jedem Neustart
let zieleStart = null;
async function zieleLaden() {
  const sel = $('ziel');
  let d = null;
  try { const r = await fetch('api.php?a=detector'); if (r.ok) d = await r.json(); } catch (e) {}
  if (!d) return;
  let gemerkt = null;
  try { gemerkt = localStorage.getItem('xgonav.ziel'); } catch (e) {}
  const alt = sel.value;
  const klassen = d.classes || [];
  sel.innerHTML = klassen.length ? klassen.map(c => '<option value="' + esc(c) + '">' + esc(c) + '</option>').join('')
                                 : '<option value="">' + tr('YOLO aus') + '</option>';
  sel.disabled = !klassen.length;
  const wahl = [gemerkt, alt, d.default, 'person'].find(c => c && klassen.includes(c));
  if (wahl) sel.value = wahl;
  const hinweis = klassen.length ? '' : tr(' – YOLO ist aus ([detector] enabled)');
  document.querySelectorAll('[data-modus="follow"],[data-modus="approach"]').forEach(b => {
    b.dataset.titel ??= attrDe(b, 'title');
    b.title = (b.dataset.titel ? tr(b.dataset.titel) : '') + hinweis;
  });
}
// Bodenmodell (FloorNet): Liste nur auf Anfrage (Seitenaufruf, Neustart, nach dem Wechsel), am Rider nur ein Verzeichnis lesen
async function bodenmodellLaden() {
  let d = null;
  try { const r = await fetch('api.php?a=floornet'); if (r.ok) d = await r.json(); } catch (e) {}
  const sel = $('fn-wahl'), txt = $('fn-text');
  if (!d) { sel.disabled = $('fn-an').disabled = true; txt.textContent = ''; return; }
  const m = d.modelle || [];
  sel.innerHTML = m.map(x => '<option value="' + esc(x.name) + '">' + esc(x.name) + ' (' + x.kb + ' kB, ' +
    new Date(x.zeit * 1000).toLocaleDateString(LANG === 'en' ? 'en-GB' : 'de-DE') + ')' +
    (x.name === d.geladen ? tr(' (geladen)') : '') + '</option>').join('');
  if (d.aktiv && m.some(x => x.name === d.aktiv)) sel.value = d.aktiv;
  sel.disabled = !m.length;
  $('fn-an').disabled = !m.length || !d.ist_link;
  // Das geladene Netz steht nur in der Auswahl; hier bleiben Warnungen
  const warn = [];
  if (!d.geladen) warn.push(tr('kein Netz geladen'));
  else if (d.blind) warn.push(tr('blind'));
  if (!d.ist_link) warn.push(tr('net ist kein Symlink: Wechsel erst nach deploy/install.sh'));
  txt.textContent = !m.length ? tr('kein Modell in ') + (d.ordner || '-') : warn.join(' · ');
}
async function bodenmodellAktivieren() {
  const n = $('fn-wahl').value;
  if (!n || !confirm(tr('Bodenmodell wechseln auf ') + n + tr('? Nur in Idle oder Hand; das Bodenmodell ist danach kurz aus.'))) return;
  $('fn-an').disabled = true;
  $('fn-text').textContent = tr('lädt …');
  await befehl('floornet', {name: n});
  bodenmodellLaden();
}
$('ziel').addEventListener('change', e => { try { localStorage.setItem('xgonav.ziel', e.target.value); } catch (x) {} });
function streife() {
  if (!confirm(tr('Steht der Rider (aufgestanden) am Startpunkt, in Startrichtung? Die Pose springt dann dorthin. Sind Treppen und Kanten abgesperrt?'))) return;
  return befehl('mode', {name: 'patrol', target: 'person', bestaetigt: 1});
}
function poseBestaetigen(aktion) {
  const frage = aktion === 'start'
    ? 'Steht der Rider (aufgestanden) am Startpunkt, in Startrichtung? Die Pose springt dann dorthin.'
    : 'Steht der Rider (aufgestanden) genau dort, wo ihn die Karte zeigt, in dieser Richtung?';
  if (!confirm(tr(frage))) return;
  return befehl('map_pose', {aktion});
}
function pipelineUmschalten() { return befehl('pipeline', {on: aktPipeline === 'läuft' ? 0 : 1}); }
// Feste Kopfleiste: ihre Höhe (je nach Breite ein- oder mehrzeilig) als --kopf-h, damit nichts darunter verschwindet
function kopfHoehe() {
  document.documentElement.style.setProperty('--kopf-h', document.querySelector('header').offsetHeight + 'px');
}
if (window.ResizeObserver) new ResizeObserver(kopfHoehe).observe(document.querySelector('header'));
kopfHoehe();
let aktAufzeichnung = false;
function aufzeichnungSetzen(an) { return befehl('record', {on: an ? 1 : 0}); }
// Freier Platz mit 3 Stellen und Komma (987 MB, 1,23 GB, 12,3 GB), gerechnet im Browser
function platzText(mb) {
  const [w, e] = mb >= 1000 ? [mb / 1024, ' GB'] : [mb, ' MB'];
  const t = w >= 100 ? Math.round(w).toString() : w.toFixed(w >= 10 ? 1 : 2);
  return (LANG === 'en' ? t : t.replace('.', ',')) + e;
}
function platzZeigen(a) {
  const el = $('rec-platz');
  el.hidden = !a || a.frei_mb == null;
  if (el.hidden) return;
  const min = a.rate_mb_s > 0 ? Math.floor(a.frei_mb / a.rate_mb_s / 60) : null;
  el.textContent = platzText(a.frei_mb) + (min == null ? '' : ' · ' + (a.rate_gemessen ? '' : '~') + min + ' min');
}
function aufzeichnungZeigen(s) {
  const a = s.aufzeichnung;
  $('rec').hidden = !a;
  platzZeigen(a);
  if (!a) return;
  aktAufzeichnung = !!a.an;
  const rec = $('rec'), recText = tr(a.an ? 'Aufzeichnung beenden' : 'Aufzeichnen');
  rec.classList.toggle('an', aktAufzeichnung);
  rec.setAttribute('aria-pressed', aktAufzeichnung ? 'true' : 'false');
  rec.setAttribute('aria-label', recText);
  rec.title = recText + ': ' + tr('Rohbilder und Metadaten am Rider speichern. Schreibt nur, solange er balanciert; Absetzen beendet die Aufnahme, jedes Aufstehen beginnt eine neue.');
  const ordner = a.ordner ? a.ordner.split('/').pop() : '';
  let text = '', art = 'warn';
  if (a.laeuft) { text = a.bilder + ' ' + tr('Bilder'); art = 'schlecht'; }
  else if (a.an) text = tr(!s.balancing ? 'wartet: liegt' : (s.pipeline !== 'läuft' ? 'wartet: Bildverarb. aus' : 'wartet auf Bild'));
  pille('p-rec', text || '-', art);
  $('p-rec').style.display = a.an ? '' : 'none';
  const rt = $('rec-text');
  rt.textContent = a.fehler ? tr('Aufzeichnung aus: ') + a.fehler : (a.laeuft ? '● ' + text + ' · ' + ordner : text);
  rt.title = a.ordner || '';
}
// Kamerahöhe aus dem Sollwert u8: Gerade aus /status, gerechnet im Browser. Ohne xgonav bleibt nur u8.
let kamGerade = null, hoeheAusStatus = false;
function kamText(u) {
  u = Number(u);
  if (!kamGerade) return 'u8 ' + u;
  return (kamGerade[0] + kamGerade[1] * Math.max(55, Math.min(200, u))).toFixed(1) + ' mm';   // Grenzen der Firmware
}
function hoeheWert(u) {
  const el = $('hoehe-w');
  el.textContent = kamText(u);
  el.title = tr('Sollwert u8 ') + u;
}
function hoeheSetzen(u) {
  reglerSetzen('hoehe', u, kamText(u));
  $('hoehe-w').title = tr('Sollwert u8 ') + $('hoehe').value;
}
function hzRoll(an) {
  bildRollAn = an;
  umschaltZeigen('hz-roll', an);
  try { localStorage.setItem('xgonav.roll', an ? '1' : '0'); } catch (e) {}
  bildVerschieben();
}
function hzMitte(an) {
  bildMitteAn = an;
  umschaltZeigen('hz-mitte', an);
  try { localStorage.setItem('xgonav.mitte', an ? '1' : '0'); } catch (e) {}
  bildVerschieben();
}
function hzStabil(an) {
  bildStabil = an;
  umschaltZeigen('hz-stabil', an);
  try { localStorage.setItem('xgonav.stabil', an ? '1' : '0'); } catch (e) {}
  bildVerschieben();
}
// WLAN aus dem Live-Paket ("rx tx dBm Qualität"); Raten hier aus der Differenz
const wlan = [];            // [t_ms, ↓ kB/s, ↑ kB/s, dBm, Qualität]
let wlanVorher = null;
function wlanMerken(h) {
  const f = (h || '').split(' ').map(v => v === '-' ? null : Number(v));
  if (f.length !== 4) { pille('p-wlan', '-'); return; }
  const [rx, tx, dbm, q] = f, jetzt = Date.now();
  let ab = null, auf = null;
  if (wlanVorher && rx != null && tx != null && rx >= wlanVorher.rx && tx >= wlanVorher.tx) {
    const dt = (jetzt - wlanVorher.t) / 1000;               // Zählerrücksprung (Neustart): verwerfen
    if (dt > 0.2) { ab = (rx - wlanVorher.rx) / 1024 / dt; auf = (tx - wlanVorher.tx) / 1024 / dt; }
  }
  if (rx != null && tx != null) wlanVorher = {t: jetzt, rx, tx};
  if (ab != null) { wlan.push([jetzt, ab, auf, dbm, q]); abschneiden(wlan, jetzt - CPU_MS); }
  const kb = v => v == null ? '-' : v < 10 ? v.toFixed(1) : v.toFixed(0);
  pille('p-wlan', (dbm != null ? dbm + ' dBm' : '-') + (ab != null ? ' · ↓' + kb(ab) + ' ↑' + kb(auf) + ' kB/s' : ''),
        dbm == null ? '' : dbm >= -60 ? 'gut' : dbm >= -70 ? 'warn' : 'schlecht');
}
// Status einmalig per HTTP (Start, nach Befehlen); laufend kommt er mit dem Live-Paket
async function status() {
  try {
    const r = await fetch('api.php?a=status');
    const s = await r.json();
    statusZeigen(r.ok ? s : null, s.error || (tr('Fehler ') + r.status));
  } catch (e) { statusZeigen(null, 'keine Verbindung zu xgonav'); }
}
function statusZeigen(s, fehler) {
  try {
    if (!s) { hoeheAusStatus = false; bildSoll(false); lageKnoepfe(null); zeige(fehler || 'keine Verbindung zu xgonav'); verbindungWeg(); return; }
    verbindungOk();
    ruheZeigen(!!s.ruhe, s.battery);
    bildSoll(!s.ruhe && s.pipeline === 'läuft');
    aktModus = s.mode;
    aktPipeline = s.pipeline;
    if (s.start != null && s.start !== zieleStart) { zieleStart = s.start; zieleLaden(); bodenmodellLaden(); }   // nach Neustart neu
    window.letzterStatus = s;
    if (s.cam_h_gerade) kamGerade = s.cam_h_gerade;
    hoeheAusStatus = s.height_u8 != null;
    if (hoeheAusStatus) hoeheSetzen(s.height_u8);
    radarStatus(s.ortung);
    $('kurz').innerHTML = felder.map(([k, f, breit]) =>
      '<div class="kachel' + (breit ? ' breit' : '') + '"><div class="k">' + tr(k) + '</div><div class="w">' + esc(f(s)) + '</div></div>').join('');
    document.querySelectorAll('[data-modus]').forEach(b => b.classList.toggle('aktiv', b.dataset.modus === s.mode));
    sprechZeigen();
    pille('p-modus', tr(MODI[s.mode] || s.mode || '-'), s.mode && s.mode !== 'idle' ? 'warn' : '');
    zustandPille(s.state, s.balancing);
    lageKnoepfe(s.balancing);
    akkuMerken(s.battery);
    pille('p-akku', s.battery != null ? s.battery + ' %' : '-',
          s.battery == null ? '' : (s.battery < 20 ? 'schlecht' : s.battery < 40 ? 'warn' : 'gut'));
    const laeuft = s.pipeline === 'läuft';
    const pt = tr(laeuft ? 'Bildverarbeitung stoppen' : 'Bildverarbeitung starten');
    $('pip-ic').textContent = laeuft ? '■' : '▶';
    $('pip').setAttribute('aria-label', pt);
    $('pip').title = pt + ' ' + tr('(Kamera und Auswertung)');
    $('pip').classList.toggle('aktiv', !laeuft);
    aufzeichnungZeigen(s);
    if (s.grid) $('karte-pose').textContent =
      'x ' + s.grid.pose[0].toFixed(2) + ' m · y ' + s.grid.pose[1].toFixed(2) + ' m · ' + s.grid.pose[2].toFixed(0) + '° · ' + tr('Start ') + s.grid.zum_start_m.toFixed(2) + ' m · ' + tr('belegt ') + s.grid.belegt_m2 + ' m²';
    const ek = $('karte-erkunden');
    ek.classList.toggle('weg', !s.explore);
    if (s.explore) {
      const ende = s.explore.ende ? ' · ' + s.explore.ende : '';
      ek.textContent = tr('Erkunden: ') + tr(ERKUNDEN[s.explore.zustand] || s.explore.zustand) + ' · ' + s.explore.stellen + tr(' Stellen offen')
        + (s.explore.gesperrt ? ', ' + s.explore.gesperrt + tr(' übergangen') : '') + ende;
    }
    const st = $('karte-streife');
    st.classList.toggle('weg', !s.patrol);
    if (s.patrol) {
      const p = s.patrol;
      st.textContent = tr('Streife: ') + tr(STREIFE[p.zustand] || p.zustand) + ' · ' + tr('Punkt ') + p.punkt + tr(' von ') + p.von + ' · ' + tr('Runde ') + p.runde
        + (p.ausgelassen.length ? ' · ' + tr('ausgelassen: ') + p.ausgelassen.join(', ') : '') + (p.ende ? ' · ' + p.ende : '');
    }
    startbildZeigen(s.startbild);
    if (s.grid) {
      const poseWeg = s.grid.pose_ok === false;
      $('karte-pose-warn').hidden = !poseWeg;
      $('karte-pose-grund').textContent = s.grid.pose_grund ? ' (' + s.grid.pose_grund + ')' : '';
      $('karte-pose-weg').hidden = !s.grid.reloc;          // ohne Kartenabgleich nie (Konfig), sonst fest an seinem Platz
      $('karte-pose-weg').disabled = poseWeg;
      karteGesperrt = !!s.grid.gesperrt;
      const ks = $('karte-sperre');
      ks.textContent = tr(karteGesperrt ? 'Karte entsperren' : 'Karte sperren');
      ks.classList.toggle('aktiv', karteGesperrt);
    }
    belichtStatus(s.exposure);
    gainStatus(s.exposure);
    bildStatus(s.exposure);
    tempoStatus(s.tempo);
  } catch (e) { console.error(e); }
}

// Startbild: nur bei neuer Nummer laden (am Rider einmal je Anker kodiert); ohne Anker ganz ausblenden
let startbildN = 0;
function startbildZeigen(sb) {
  const n = (sb && sb.bild) || 0;
  if (n && n !== startbildN) $('karte-startbild-img').src = 'startbild.jpg?n=' + n;
  startbildN = n;
  $('karte-startbild').hidden = !n;
}

// --- Handsteuerung: solange gedrückt alle 200 ms senden; xgonav hält nach 0,5 s ohne Sollwert an
const gedrueckt = new Set();
let fahrtTimer = null;
// Temporegler: gilt in xgonav für alle Fahr- und Drehbefehle aller Modi (POST /tempo, Robot.drive nimmt jeden
// Befehl nach der Begrenzung auf max_v/max_w damit mal). Angezeigt wird der Wert aus /status.
const tempoRegler = $('tempo');
let tempoBewegt = false;
tempoRegler.addEventListener('pointerdown', () => tempoBewegt = true);
tempoRegler.addEventListener('pointerup', () => tempoBewegt = false);
tempoRegler.oninput = () => $('tempo-w').textContent = tempoRegler.value + ' %';
tempoRegler.onchange = () => { tempoBewegt = false; befehl('tempo', {f: (Number(tempoRegler.value) / 100).toFixed(2)}, true); };
tempoRegler.oninput();
function tempoStatus(t) {
  if (t == null || tempoBewegt || document.activeElement === tempoRegler) return;
  tempoRegler.value = Math.round(t * 100);                  // rastet auf 25/50/75/100
  $('tempo-w').textContent = Math.round(t * 100) + ' %';    // angezeigt wird der wirkliche Wert
}
// Handsteuerung schickt Vollausschlag, xgonav begrenzt auf max_v/max_w und nimmt mit dem Tempo mal
function sollwert() {
  const v = (gedrueckt.has('vor') ? 1 : 0) - (gedrueckt.has('zurück') ? 1 : 0);
  const w = (gedrueckt.has('links') ? 1 : 0) - (gedrueckt.has('rechts') ? 1 : 0);
  return {v: v.toFixed(3), w: w.toFixed(3)};
}
async function fahrtStart(r) {
  if (gedrueckt.has(r)) return;
  gedrueckt.add(r);
  document.querySelectorAll('[data-fahrt="' + r + '"]').forEach(b => b.classList.add('aktiv'));
  if (fahrtTimer) return;
  if (aktModus !== 'manual') { await modus('manual'); }
  const senden = () => befehl('drive', sollwert(), true);
  senden();
  fahrtTimer = setInterval(senden, 200);
}
function fahrtEnde(r) {
  gedrueckt.delete(r);
  document.querySelectorAll('[data-fahrt="' + r + '"]').forEach(b => b.classList.remove('aktiv'));
  if (gedrueckt.size === 0 && fahrtTimer) {
    clearInterval(fahrtTimer); fahrtTimer = null;
    befehl('drive', {v: 0, w: 0}, true);
  }
}
document.querySelectorAll('[data-fahrt]').forEach(b => {
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); fahrtStart(b.dataset.fahrt); });
  b.addEventListener('pointerup', () => fahrtEnde(b.dataset.fahrt));
  b.addEventListener('pointercancel', () => fahrtEnde(b.dataset.fahrt));
});
const tasten = {ArrowUp: 'vor', w: 'vor', ArrowDown: 'zurück', s: 'zurück', ArrowLeft: 'links', a: 'links', ArrowRight: 'rechts', d: 'rechts'};
// nicht in Eingabefeldern und nicht auf Reglern/Auswahl (dort steuern die Pfeiltasten das Element)
const eigeneTasten = e => ['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName);
document.addEventListener('keydown', e => { const r = tasten[e.key]; if (r && !eigeneTasten(e)) { e.preventDefault(); fahrtStart(r); } });
document.addEventListener('keyup', e => { const r = tasten[e.key]; if (r) fahrtEnde(r); });
window.addEventListener('blur', () => [...gedrueckt].forEach(fahrtEnde));

// --- Gegensprechen (nur Modus Hand): WebSocket /sprech zu tools/sprech.php, 8 kHz mono S16 in beide Richtungen.
// Abtastrate wandeln, Echo dämpfen und Pegel rechnet der Browser; am Rider laufen nur arecord und aplay.
const SPRECH_HZ = 8000;
const SPRECH_WORKLET = `
class Auf extends AudioWorkletProcessor {
  // Tiefpass 3,5 kHz (gefenstertes Sinc, Blackman, 127 Taps bei der Eingangsrate) vor dem Abtasten auf 8 kHz;
  // bis 2026-11-21 nur ein Kastenfilter, der ließ 4–24 kHz ins Sprachband falten (klang unklar). Danach
  // Hochpass 400 Hz (Butterworth 4. Ordnung, zwei Biquads bei 8 kHz): unter ~300 Hz macht der Rider-Lautsprecher
  // fast nur Oberwellen (gemessen 2026-11-21: 150 Hz +16 dB, 250 Hz +7 dB Klirren über dem Grundton), die Stimme
  // klang dadurch falsch; Verständlichkeit tragen die Oberwellen. Dann Verstärkung (Regler "Rider") und weicher
  // Begrenzer bis 0,9. (Gemessen mit Class-D-Boost 4, der selbst verzerrte; seit 2026-11-21 Boost 0, docs/audio.md.)
  constructor() {
    super(); this.schritt = sampleRate / ${SPRECH_HZ}; this.pos = 0;
    const N = 127, fc = 3500 / sampleRate; this.h = new Float32Array(N); let sum = 0;
    for (let i = 0; i < N; i++) {
      const m = i - (N - 1) / 2, w = 0.42 - 0.5 * Math.cos(2 * Math.PI * i / (N - 1)) + 0.08 * Math.cos(4 * Math.PI * i / (N - 1));
      this.h[i] = w * (m === 0 ? 2 * fc : Math.sin(2 * Math.PI * fc * m) / (Math.PI * m)); sum += this.h[i];
    }
    for (let i = 0; i < N; i++) this.h[i] /= sum;
    this.ring = new Float32Array(N); this.w = 0; this.gain = 3;
    this.hp = [0.5412, 1.3066].map(Q => {            // Güten der beiden Butterworth-Stufen (4. Ordnung)
      const w0 = 2 * Math.PI * 400 / ${SPRECH_HZ}, c = Math.cos(w0), al = Math.sin(w0) / (2 * Q), a0 = 1 + al;
      return {b0: (1 + c) / 2 / a0, b1: -(1 + c) / a0, b2: (1 + c) / 2 / a0, a1: -2 * c / a0, a2: (1 - al) / a0, x1: 0, x2: 0, y1: 0, y2: 0};
    });
    this.port.onmessage = e => { if (e.data && e.data.gain) this.gain = e.data.gain; };
    this.buf = new Int16Array(320); this.i = 0; this.q = 0;
  }
  process(ein) {
    const x = ein[0] && ein[0][0];
    if (!x) return true;
    const h = this.h, r = this.ring, N = h.length;
    for (let k = 0; k < x.length; k++) {
      r[this.w] = x[k]; this.w = (this.w + 1) % N; this.pos += 1;
      if (this.pos >= this.schritt) {                // nur an den Abtastpunkten falten: 8000 × 127 Mult./s
        this.pos -= this.schritt;
        let y = 0;
        for (let j = 0, idx = this.w; j < N; j++, idx = idx + 1 === N ? 0 : idx + 1) y += h[j] * r[idx];
        for (const f of this.hp) {
          const o = f.b0 * y + f.b1 * f.x1 + f.b2 * f.x2 - f.a1 * f.y1 - f.a2 * f.y2;
          f.x2 = f.x1; f.x1 = y; f.y2 = f.y1; f.y1 = o; y = o;
        }
        const v = Math.tanh(y * this.gain * 1.1) / 1.1;   // bis ~0,5 fast linear, höchstens 0,9
        this.q += v * v;
        this.buf[this.i++] = v * 32767;
        if (this.i === this.buf.length) {
          this.port.postMessage({pcm: this.buf.buffer, rms: Math.sqrt(this.q / this.buf.length)}, [this.buf.buffer]);
          this.buf = new Int16Array(320); this.i = 0; this.q = 0;
        }
      }
    }
    return true;
  }
}
class Ab extends AudioWorkletProcessor {
  constructor() {
    super(); this.ring = new Float32Array(${SPRECH_HZ}); this.r = 0; this.w = 0; this.n = 0; this.pos = 0;
    this.laeuft = false; this.schritt = ${SPRECH_HZ} / sampleRate;
    this.port.onmessage = e => {
      const d = new Int16Array(e.data);
      for (let k = 0; k < d.length; k++) {
        if (this.n === this.ring.length) { this.r = (this.r + 1) % this.ring.length; this.n--; }
        this.ring[this.w] = d[k] / 32768; this.w = (this.w + 1) % this.ring.length; this.n++;
      }
      if (this.n > ${SPRECH_HZ * 0.4}) {             // mehr als 0,4 s gepuffert: auf 0,12 s kürzen (nur das Neueste)
        const weg = this.n - ${SPRECH_HZ * 0.12}; this.r = (this.r + weg) % this.ring.length; this.n -= weg;
      }
    };
  }
  process(ein, aus) {
    const y = aus[0][0], L = this.ring.length;
    if (!this.laeuft && this.n >= ${SPRECH_HZ * 0.12}) this.laeuft = true;   // Jitterpuffer 120 ms
    for (let k = 0; k < y.length; k++) {
      if (!this.laeuft || this.n < 2) { y[k] = 0; this.laeuft = false; continue; }
      const a = this.ring[this.r], b = this.ring[(this.r + 1) % L];
      y[k] = a + (b - a) * this.pos;                 // linear hochwandeln
      this.pos += this.schritt;
      while (this.pos >= 1) { this.pos -= 1; this.r = (this.r + 1) % L; this.n--; }
    }
    return true;
  }
}
registerProcessor('sprech-auf', Auf);
registerProcessor('sprech-ab', Ab);
`;
let sprech = null;                // {ws, ctx, strom, ab, gain, an}
let sprechPttAn = false, sprechTaste = false, sprechRiderPegel = 0;

// Echo-Sperre: Das Rider-Mikrofon hört seinen Lautsprecher (gemessen −17 dBFS), unsere Stimme kommt also nach
// der Rundlaufzeit (WLAN, aplay-/arecord-Puffer, Jitterpuffer: ~0,5–1,5 s, schwankt) zurück; die alte Dämpfung
// (nur 300 ms nach dem eigenen Sprechen) kam dafür zu früh. Pegelverläufe in 40-ms-Fächern: gesendet (je
// Mikrofonrahmen) und empfangen (je Rider-Rahmen nach Ankunftszeit). Alle 1 s die beste Korrelation der
// Log-Pegel über 0–1,6 s Verzögerung, als Stimme in ein abklingendes Histogramm; dazu die Kopplung (Median
// empfangen/gesendet). Gedämpft auf ECHO_REST wird ein Rahmen, wenn ECHO_FENSTER Fächer um die
// Verzögerung herum laut gesprochen wurde und er nicht deutlich lauter ist als das erwartete Echo (sonst
// spricht drüben jemand: durchlassen). Ohne Schätzung: das ganze Fenster 0–1,7 s. Alles im Browser.
const ECHO_FACH = 40, ECHO_N = 128, ECHO_MAXLAG = 40, ECHO_FENSTER = 6, ECHO_REST = 0.05;
const echo = {
  gesendet: new Float32Array(ECHO_N), gFach: new Float64Array(ECHO_N).fill(-1),
  empfangen: new Float32Array(ECHO_N), eFach: new Float64Array(ECHO_N).fill(-1),
  stimmen: new Float32Array(ECHO_MAXLAG + 1), lag: -1, k: 0, naechste: 0, faktor: 1,
};
function echoReset() {
  echo.gFach.fill(-1); echo.eFach.fill(-1); echo.stimmen.fill(0); echo.lag = -1; echo.k = 0; echo.faktor = 1;
}
function echoEintragen(werte, faecher, fach, rms) {
  const i = fach % ECHO_N;
  if (faecher[i] !== fach) { faecher[i] = fach; werte[i] = 0; }
  werte[i] = Math.max(werte[i], rms);
}
function echoWert(werte, faecher, fach) { const i = ((fach % ECHO_N) + ECHO_N) % ECHO_N; return faecher[i] === fach ? werte[i] : 0; }
function echoMax(fach, b) {                            // lautestes Gesendetes in fach-b .. fach+b
  let m = 0; for (let d = -b; d <= b; d++) m = Math.max(m, echoWert(echo.gesendet, echo.gFach, fach + d)); return m;
}
function echoSchaetzen(jetzt) {
  const ende = jetzt - 1, n = 80;                        // letzte 3,2 s Empfang gegen Gesendetes davor
  const e = [], g = [];
  let eM = 0;
  for (let j = 0; j < n; j++) { const v = Math.log(1e-4 + echoWert(echo.empfangen, echo.eFach, ende - j)); e.push(v); eM += v / n; }
  let eV = 0; for (const v of e) eV += (v - eM) ** 2;
  let best = -1, bestR = 0;
  for (let L = 1; L <= ECHO_MAXLAG; L++) {
    let gM = 0; g.length = 0;
    for (let j = 0; j < n; j++) { const v = Math.log(1e-4 + echoWert(echo.gesendet, echo.gFach, ende - j - L)); g.push(v); gM += v / n; }
    let gV = 0, c = 0;
    for (let j = 0; j < n; j++) { gV += (g[j] - gM) ** 2; c += (g[j] - gM) * (e[j] - eM); }
    if (gV < n * 0.5) return;                            // zu wenig eigenes Sprechen im Fenster: Schätzung behalten
    const r = c / Math.sqrt(gV * eV + 1e-12);
    if (r > bestR) { bestR = r; best = L; }
  }
  for (let L = 0; L <= ECHO_MAXLAG; L++) echo.stimmen[L] *= 0.85;
  if (best >= 0 && bestR > 0.3) echo.stimmen[best] += bestR;
  let lag = -1, s = 0.6;                                 // erst nach ~2 übereinstimmenden Schätzungen
  for (let L = 0; L <= ECHO_MAXLAG; L++) {
    let v = 0; for (let d = -2; d <= 2; d++) v += echo.stimmen[L + d] || 0;
    if (v > s) { s = v; lag = L; }
  }
  echo.lag = lag;
  if (lag < 0) return;
  const q = [];                                          // Kopplung: Median empfangen/gesendet bei lautem Sprechen
  for (let j = 0; j < n; j++) {
    const gw = echoWert(echo.gesendet, echo.gFach, ende - j - lag);
    if (gw > 0.03) q.push(echoWert(echo.empfangen, echo.eFach, ende - j) / gw);
  }
  if (q.length >= 5) { q.sort((a, b) => a - b); echo.k = q[q.length >> 1]; }
}
// Faktor für einen eben angekommenen Rider-Rahmen (Pegel rms)
function echoFaktor(jetzt, rms) {
  if (echo.lag >= 0 && echo.k > 0) {
    const g = echoMax(jetzt - echo.lag, ECHO_FENSTER);
    return g > 0.02 && rms < 3 * echo.k * g + 0.02 ? ECHO_REST : 1;
  }
  for (let d = 0; d <= ECHO_MAXLAG + 2; d++) if (echoWert(echo.gesendet, echo.gFach, jetzt - d) > 0.02) return ECHO_REST;
  return 1;
}
try { sprechPttAn = localStorage.getItem('xgonav.sprechptt') === '1'; } catch (e) {}
$('sprech-ptt').checked = sprechPttAn;
$('sprech-taste').hidden = !sprechPttAn;
try { $('sprech-laut').value = localStorage.getItem('xgonav.sprechlaut') || 200; } catch (e) {}
try { $('sprech-hin').value = localStorage.getItem('xgonav.sprechhin2') || 300; } catch (e) {}   // neuer Schlüssel: Standard 300 %
function sprechHin() {
  if (sprech && sprech.auf) sprech.auf.port.postMessage({gain: Number($('sprech-hin').value) / 100});
}
$('sprech-hin').oninput = () => { sprechHin(); try { localStorage.setItem('xgonav.sprechhin2', $('sprech-hin').value); } catch (e) {} };
function telefonUmschalten() {
  try { localStorage.setItem('xgonav.telefon', $('telefonkarte').open ? '1' : '0'); } catch (e) {}
}
try { $('telefonkarte').open = localStorage.getItem('xgonav.telefon') === '1'; } catch (e) {}   // Standard: zu
function sprechText(t) { $('sprech-text').textContent = t ? tr(t) : ''; }
function sprechSendet() { return !sprechPttAn || sprechTaste; }
function sprechGain() {
  if (!sprech || !sprech.gain) return;
  const v = Number($('sprech-laut').value) / 100;
  const duck = sprechPttAn && sprechTaste;   // Sprechtaste gedrückt: Rider leiser; sonst dämpft die Echo-Sperre je Rahmen
  sprech.gain.gain.setTargetAtTime(duck ? v * 0.1 : v, sprech.ctx.currentTime, 0.03);
}
$('sprech-laut').oninput = () => { sprechGain(); try { localStorage.setItem('xgonav.sprechlaut', $('sprech-laut').value); } catch (e) {} };
function sprechPtt(an) {
  sprechPttAn = an; sprechTaste = false;
  $('sprech-taste').hidden = !an;
  try { localStorage.setItem('xgonav.sprechptt', an ? '1' : '0'); } catch (e) {}
  sprechGain();
}
function sprechTasteSetzen(an) {
  if (sprechTaste === an) return;
  sprechTaste = an;
  $('sprech-taste').classList.toggle('aktiv', an);
  sprechGain();
}
const tasteSp = $('sprech-taste');
tasteSp.addEventListener('pointerdown', e => { e.preventDefault(); tasteSp.setPointerCapture(e.pointerId); sprechTasteSetzen(true); });
tasteSp.addEventListener('pointerup', () => sprechTasteSetzen(false));
tasteSp.addEventListener('pointercancel', () => sprechTasteSetzen(false));
// Leertaste = Sprechtaste, nur bei laufendem Gespräch mit „Drücken zum Sprechen“ und nicht in Eingabefeldern
document.addEventListener('keydown', e => {
  if (e.code === 'Space' && sprech && sprechPttAn && !eigeneTasten(e)) { e.preventDefault(); sprechTasteSetzen(true); }
});
document.addEventListener('keyup', e => { if (e.code === 'Space') sprechTasteSetzen(false); });
window.addEventListener('blur', () => sprechTasteSetzen(false));
function sprechZeigen() {
  const an = !!(sprech && sprech.an), hand = aktModus === 'manual';
  const b = $('sprech');
  b.classList.toggle('aktiv', an);
  b.textContent = tr(sprech ? 'Auflegen' : 'Sprechen');
  b.disabled = !sprech && !hand;
  $('sprech-bild').disabled = !an;
  $('telefon-zustand').textContent = tr(an ? 'Gespräch läuft' : sprech ? 'verbinde …' : hand ? 'bereit' : 'Nur im Modus Hand');
  if (!an && bild) bildEnde();
  b.title = tr(hand || sprech ? 'Gegensprechen mit dem Rider: sein Mikrofon hier hören, ins eigene Mikrofon sprechen (8 kHz). Nur im Modus Hand.'
                              : 'Nur im Modus Hand');
}
async function sprechUmschalten() {
  if (sprech) { sprechEnde(); return; }
  if (aktModus !== 'manual') { sprechText('Nur im Modus Hand'); return; }
  if (!navigator.mediaDevices || !window.AudioWorkletNode) { sprechText('Browser kann kein Mikrofon (HTTPS nötig)'); return; }
  const s = sprech = {an: false};
  echoReset();
  sprechZeigen();
  sprechText('verbinde …');
  try {
    s.strom = await navigator.mediaDevices.getUserMedia({audio: {echoCancellation: true, noiseSuppression: true,
                                                                autoGainControl: true, channelCount: 1}});
    s.ctx = new AudioContext();
    const url = URL.createObjectURL(new Blob([SPRECH_WORKLET], {type: 'application/javascript'}));
    await s.ctx.audioWorklet.addModule(url);
    URL.revokeObjectURL(url);
  } catch (e) {
    if (sprech === s) sprechEnde(); sprechText('Mikrofon nicht freigegeben'); return;
  }
  if (sprech !== s) { sprechAufraeumen(s); return; }
  const auf = s.auf = new AudioWorkletNode(s.ctx, 'sprech-auf');
  sprechHin();
  s.ctx.createMediaStreamSource(s.strom).connect(auf);
  s.ab = new AudioWorkletNode(s.ctx, 'sprech-ab', {numberOfInputs: 0, outputChannelCount: [1]});
  s.gain = s.ctx.createGain();
  s.ab.connect(s.gain).connect(s.ctx.destination);
  sprechGain();
  const ws = s.ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/sprech');
  ws.binaryType = 'arraybuffer';
  auf.port.onmessage = e => {
    const {pcm, rms} = e.data;
    const senden = sprechSendet();
    echoEintragen(echo.gesendet, echo.gFach, Math.floor(performance.now() / ECHO_FACH), senden && s.an ? rms : 0);
    $('sprech-ich').value = senden ? Math.min(1, rms * 5) : 0;
    $('sprech-rider').value = sprechRiderPegel;
    sprechRiderPegel *= 0.7;
    if (senden && s.an && ws.readyState === 1 && ws.bufferedAmount < 16000) ws.send(pcm);
  };
  ws.onmessage = e => {
    if (typeof e.data === 'string') {
      let m = null; try { m = JSON.parse(e.data); } catch (e2) {}
      if (!m) return;
      s.an = m.sprech === 'an';
      sprechText(s.an ? 'Gespräch läuft' : (m.grund || 'aus'));
      if (s.an) s.ctx.resume();
      sprechZeigen();
      return;
    }
    const d = new Int16Array(e.data); let q = 0;                     // Pegel vor dem Weiterreichen (danach ist der Puffer weg)
    for (let k = 0; k < d.length; k++) q += d[k] * d[k];
    const rms = d.length ? Math.sqrt(q / d.length) / 32768 : 0;
    const fach = Math.floor(performance.now() / ECHO_FACH);
    echoEintragen(echo.empfangen, echo.eFach, fach, rms);
    if (fach >= echo.naechste) { echo.naechste = fach + 1000 / ECHO_FACH; echoSchaetzen(fach); }
    const f = echoFaktor(fach, rms), f0 = echo.faktor;             // über den Rahmen überblenden (kein Knacken)
    if (f !== 1 || f0 !== 1)
      for (let k = 0; k < d.length; k++) d[k] = Math.round(d[k] * (f0 + (f - f0) * (k + 1) / d.length));
    echo.faktor = f;
    sprechRiderPegel = Math.max(sprechRiderPegel, Math.min(1, 5 * rms * f));
    s.ab.port.postMessage(e.data, [e.data]);
  };
  ws.onclose = () => {
    if (sprech !== s) return;
    const grund = $('sprech-text').textContent;          // Grund vom Dienst stehen lassen
    sprechEnde();
    if (grund === tr('verbinde …')) sprechText('keine Verbindung zum Gegensprech-Dienst');
    else if (grund === tr('Gespräch läuft')) sprechText('Gespräch beendet');
    else $('sprech-text').textContent = grund;
  };
}
function sprechAufraeumen(s) {
  try { s.ws && s.ws.close(); } catch (e) {}
  try { s.strom && s.strom.getTracks().forEach(t => t.stop()); } catch (e) {}
  try { s.ctx && s.ctx.close(); } catch (e) {}
}
function sprechEnde() {
  const s = sprech;
  sprech = null;
  if (bild) bildEnde();
  if (s) sprechAufraeumen(s);
  sprechTasteSetzen(false);
  $('sprech-ich').value = 0; $('sprech-rider').value = 0;
  sprechText('');
  sprechZeigen();
}

// --- Bild aufs Rider-Display (nur während eines Gesprächs): eigene WebSocket /sprech?bild, je Rahmen ein
// fertiges Bild RGB565 LE 320×240 (153 600 B). Zuschneiden und Umrechnen hier, der Rider reicht nur durch.
const BILD_B = 320, BILD_H = 240;
let bild = null;                  // {ws, strom, timer, ctx, an}
try { $('sprech-fps').value = localStorage.getItem('xgonav.sprechfps') || '5'; } catch (e) {}
$('sprech-fps').onchange = () => {
  try { localStorage.setItem('xgonav.sprechfps', $('sprech-fps').value); } catch (e) {}
  if (bild && bild.an) bildTakt();
};
function bildText(t) { $('bild-text').textContent = t ? tr(t) : ''; }
function bildTakt() {
  clearInterval(bild.timer);
  bild.timer = setInterval(bildSenden, 1000 / Number($('sprech-fps').value || 5));
}
function bildSenden() {
  const b = bild, v = $('sprech-vorschau');
  if (!b || !b.an || b.ws.readyState !== 1 || b.ws.bufferedAmount > 0 || !v.videoWidth) return;   // höchstens ein Bild unterwegs
  const s = Math.max(BILD_B / v.videoWidth, BILD_H / v.videoHeight);          // mittig zuschneiden, ungespiegelt
  const sw = BILD_B / s, sh = BILD_H / s;
  b.ctx.drawImage(v, (v.videoWidth - sw) / 2, (v.videoHeight - sh) / 2, sw, sh, 0, 0, BILD_B, BILD_H);
  const d = b.ctx.getImageData(0, 0, BILD_B, BILD_H).data;
  const px = new Uint16Array(BILD_B * BILD_H);                                // Browser sind little-endian
  for (let i = 0, j = 0; i < px.length; i++, j += 4) px[i] = ((d[j] & 0xF8) << 8) | ((d[j + 1] & 0xFC) << 3) | (d[j + 2] >> 3);
  b.ws.send(px.buffer);
}
async function bildUmschalten(an) {
  if (!an) { bildEnde(); return; }
  if (bild || !sprech || !sprech.an) { $('sprech-bild').checked = !!bild; return; }
  const b = bild = {an: false};
  bildText('verbinde …');
  try {
    b.strom = await navigator.mediaDevices.getUserMedia({video: {width: {ideal: 320}, height: {ideal: 240}, facingMode: 'user'}});
  } catch (e) {
    if (bild === b) bildEnde(); bildText('Kamera nicht freigegeben'); return;
  }
  if (bild !== b) { b.strom.getTracks().forEach(t => t.stop()); return; }
  const v = $('sprech-vorschau');
  v.srcObject = b.strom; v.hidden = false;
  v.play().catch(() => {});
  const cv = document.createElement('canvas'); cv.width = BILD_B; cv.height = BILD_H;
  b.ctx = cv.getContext('2d', {willReadFrequently: true});
  const ws = b.ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/sprech?bild');
  ws.onmessage = e => {
    if (typeof e.data !== 'string') return;
    let m = null; try { m = JSON.parse(e.data); } catch (e2) {}
    if (!m) return;
    b.an = m.bild === 'an';
    bildText(b.an ? 'Bild am Display' : (m.grund || 'aus'));
    if (b.an) bildTakt();
  };
  ws.onclose = () => {
    if (bild !== b) return;
    const g = $('bild-text').textContent;
    bildEnde();
    $('bild-text').textContent = g === tr('Bild am Display') || g === tr('verbinde …') ? '' : g;
  };
}
function bildEnde() {
  const b = bild;
  bild = null;
  if (b) {
    clearInterval(b.timer);
    try { b.ws && b.ws.close(); } catch (e) {}
    try { b.strom && b.strom.getTracks().forEach(t => t.stop()); } catch (e) {}
  }
  const v = $('sprech-vorschau'); v.srcObject = null; v.hidden = true;
  $('sprech-bild').checked = false;
  bildText('');
}

// --- Belichtung: Regler in ms, xgonav rechnet in µs (0 = Automatik)
let belichtBewegt = false;      // Regler nicht überschreiben, solange er bedient wird
const belichtRegler = $('belicht');
belichtRegler.addEventListener('pointerdown', () => belichtBewegt = true);
belichtRegler.addEventListener('pointerup', () => belichtBewegt = false);
function belichtAnzeige(us) {
  $('belicht-w').textContent = us ? (us / 1000).toFixed(0) + ' ms' : tr('Auto');
}
function belichtStatus(e) {
  if (!e) return;
  belichtRegler.max = Math.floor(e.max_us / 1000);
  if (!belichtBewegt && document.activeElement !== belichtRegler) {
    if (e.soll_us) belichtRegler.value = Math.round(e.soll_us / 1000);
    belichtAnzeige(e.soll_us);
  }
  $('belicht-auto').classList.toggle('aktiv', e.soll_us === 0);
  $('belicht-ist').textContent = e.ist_us == null && e.gain == null ? '' :
    tr('ist ') + (e.ist_us != null ? (e.ist_us / 1000).toFixed(1) + ' ms' : '-') + ' · ' + (e.gain ?? '-') + '×';
}
function belichtung(us) { belichtBewegt = false; return befehl('exposure', {us}); }

// --- Verstärkung (0 = Automatik)
let gainBewegt = false;
const gainRegler = $('gain');
gainRegler.addEventListener('pointerdown', () => gainBewegt = true);
gainRegler.addEventListener('pointerup', () => gainBewegt = false);
function gainAnzeige(g) {
  $('gain-w').textContent = Number(g) ? Number(g).toFixed(1) + '×' : tr('Auto');
}
// Helligkeit/Kontrast/Sättigung an der Quelle (ISP, POST /bild); Ausgleich und Farbton nur für die Anzeige per
// SVG-Filter im Browser (kostet am Pi nichts). Ausgleich als Näherung der ISP-Kurve: aus = (ein - 0,5 - b)/k + 0,5.
const kontrastRegler = $('kontrast'), saettRegler = $('saettigung'), hellRegler = $('hell'), tonRegler = $('farbton');
let bildBewegt = false, bildIst = [1, 1, 0], bildAusgleichAn = true, farbton = 0;
try { bildAusgleichAn = localStorage.getItem('xgonav.bildausgleich') !== '0'; } catch (e) {}
try { farbton = Number(localStorage.getItem('xgonav.farbton')) || 0; } catch (e) {}
try { $('kamera-klapp').open = localStorage.getItem('xgonav.kameraklapp') === '1'; } catch (e) {}
$('kamera-klapp').addEventListener('toggle', e => {
  try { localStorage.setItem('xgonav.kameraklapp', e.target.open ? '1' : '0'); } catch (e2) {}
});
$('bild-ausgleich').checked = bildAusgleichAn;
[kontrastRegler, saettRegler, hellRegler].forEach(r => {
  r.addEventListener('pointerdown', () => bildBewegt = true);
  r.addEventListener('pointerup', () => bildBewegt = false);
});
function bildFilter() {
  const [k, s, b] = bildAusgleichAn ? bildIst : [1, 1, 0];
  const slope = 1 / k, icpt = 0.5 - (0.5 + b) / k;
  ['bf-r', 'bf-g', 'bf-b'].forEach(id => { $(id).setAttribute('slope', slope); $(id).setAttribute('intercept', icpt); });
  $('bf-satt').setAttribute('values', 1 / s);
  $('bf-ton').setAttribute('values', farbton);
  $('bild').style.filter = k !== 1 || s !== 1 || b !== 0 || farbton !== 0 ? 'url(#bild-filter)' : '';
}
function bildAnzeige() {
  $('kontrast-w').textContent = Number(kontrastRegler.value).toFixed(1);
  $('saettigung-w').textContent = Number(saettRegler.value).toFixed(1);
  const h = Number(hellRegler.value);
  $('hell-w').textContent = (h > 0 ? '+' : '') + h.toFixed(2);
  tonRegler.value = farbton;
  $('farbton-w').textContent = (farbton > 0 ? '+' : '') + farbton + '°';
}
function bildSetzen(k = null, s = null, b = null) {
  bildBewegt = false;
  if (k != null) kontrastRegler.value = k;
  if (s != null) saettRegler.value = s;
  if (b != null) hellRegler.value = b;
  bildAnzeige();
  return befehl('bild', {k: Number(kontrastRegler.value).toFixed(2), s: Number(saettRegler.value).toFixed(2),
                         b: Number(hellRegler.value).toFixed(2)});
}
function farbtonSetzen(h) {
  farbton = Number(h) || 0;
  try { localStorage.setItem('xgonav.farbton', String(farbton)); } catch (e) {}
  bildAnzeige();
  bildFilter();
}
function bildAusgleich(an) {
  bildAusgleichAn = an;
  try { localStorage.setItem('xgonav.bildausgleich', an ? '1' : '0'); } catch (e) {}
  bildFilter();
}
function bildStatus(e) {
  if (!e || e.contrast == null) return;
  bildIst = [e.contrast, e.saturation, e.brightness ?? 0];
  bildFilter();
  if (bildBewegt || [kontrastRegler, saettRegler, hellRegler].includes(document.activeElement)) return;
  kontrastRegler.value = e.contrast;
  saettRegler.value = e.saturation;
  hellRegler.value = e.brightness ?? 0;
  bildAnzeige();
}
bildAnzeige();
bildFilter();
function gainStatus(e) {
  if (!e || e.gain_soll == null) return;
  if (e.gain_max) gainRegler.max = e.gain_max;
  if (!gainBewegt && document.activeElement !== gainRegler) {
    if (e.gain_soll) gainRegler.value = e.gain_soll;
    gainAnzeige(e.gain_soll);
  }
  $('gain-auto').classList.toggle('aktiv', e.gain_soll === 0);
}
function verstaerkung(g) { gainBewegt = false; return befehl('gain', {g}); }

// --- LEDs
function led(i, c) { $('led' + i).value = c; return befehl('led', {i, c}, true); }
function ledsAus() { for (let i = 0; i < 4; i++) led(i, '#000000'); }
let strobeTimer = null, strobeLed = 0;
function strobe() {
  const b = $('strobe');
  if (strobeTimer) { clearInterval(strobeTimer); strobeTimer = null; b.classList.remove('aktiv'); ledsAus(); return; }
  b.classList.add('aktiv');
  strobeTimer = setInterval(() => { led(strobeLed, '#000000'); strobeLed = (strobeLed + 1) % 4; led(strobeLed, '#0000ff'); }, 100);
}

function ausrichten() {
  const r = $('roll');
  r.value = 0; r.oninput(); r._zeit = performance.now();   // xgonav setzt den Rollbefehl selbst auf 0
  return befehl('ausrichten');
}

function rollNull() {
  const r = $('roll');
  r.value = 0; r.oninput(); r._zeit = performance.now(); befehl('roll', {deg: 0});
}

// --- System
let kalibAn = false;
function kalibrierung() {
  kalibAn = !kalibAn;
  $('kalib').classList.toggle('aktiv', kalibAn);
  $('kalib').textContent = kalibAn ? 'Beine speichern' : 'Beine kalibrieren';
  befehl('calibration', {on: kalibAn ? 1 : 0});
}
// Nick-Trimm (Register 0x09): Firmware hält den Rider im Stand, bis er ruhig steht, und
// speichert die nötige Haltekorrektur. Vorher xgonav auf idle, damit nichts dazwischenfährt.
async function lageKalibrieren() {
  if (!confirm(tr('Lage kalibrieren? Der Rider muss balancieren und frei stehen; er regelt sich ein paar Sekunden auf der Stelle ein.'))) return;
  if (aktModus && aktModus !== 'idle' && !(await modus('idle'))) return;
  befehl('pitch_calibration');
}
// Kopfleiste: balanciert er, nur Absetzen; liegt er, nur Aufstehen/Aufspringen; unbekannt (keine Verbindung): alle
function lageKnoepfe(balanciert) {
  const b = balanciert === true, l = balanciert === false;
  $('k-absetzen').hidden = l;
  $('k-aufstehen').hidden = b;
  $('k-aufspringen').hidden = b;
}
// Aktion 255 = Arm (R-1.1.6): Räder laufen an, sobald der Rider aufrecht steht
function arm() {
  befehl('action', {n: 255});
}
// Aktion 253 = !standup: Rider springt auf und schwingt dabei aus – nur mit Nachfrage
function aufspringen() {
  if (confirm(tr('Aufspringen? Der Rider muss auf den Rädern stehen und rundherum Platz haben – er springt hoch und schwingt dabei aus.'))) befehl('action', {n: 253});
}
// Aktion 237 = ESP32 neu starten (Firmware-Patch 0018); die Firmware lehnt ab, solange er balanciert
function espNeustart() {
  if (confirm(tr('ESP32 neu starten? Geht nur, wenn der Rider nicht balanciert. Danach wie nach dem Einschalten: Steht er aufrecht, balanciert er von selbst los.'))) befehl('action', {n: 237});
}
function herunterfahren() {
  if (confirm(tr('Rider wirklich herunterfahren?'))) befehl('shutdown');
}

// ---------------------------------------------------------------- Einstellungen (Modal)
// Schema (web/config-schema.json aus xgonav/config.py), Werte und Sicherungen holt api.php?a=config einmal beim
// Öffnen; Formular, Suche und Prüfen laufen hier im Browser. Gespeichert werden nur Änderungen
// ({"abschnitt.key": Wert | null}, null = Zeile entfernen, dann gilt der Default aus config.py).
const einst = {d: null, felder: {}, aend: {}, falsch: {}, ab: null, neustart: false};
const eiName = (ab, f) => ab + '.' + f.key;
const eiGleich = (a, b) => JSON.stringify(a) === JSON.stringify(b);
function eiDatei(ab, f) { const w = einst.d.werte[ab]; return w && f.key in w ? {da: true, v: w[f.key]} : {da: false, v: f.default}; }
// Wert, der nach dem Speichern gilt
function eiWert(ab, f) {
  const n = eiName(ab, f);
  if (n in einst.aend) return einst.aend[n] === null ? f.default : einst.aend[n];
  return eiDatei(ab, f).v;
}
function eiText(f, v) {
  if (v === null || v === undefined) return '';
  if (f.typ === 'liste') return v.join(', ');
  return String(v);
}
// Eingabe lesen; wirft mit Meldung, wenn sie nicht zum Typ passt
function eiLesen(f, t) {
  t = t.trim();
  const zahl = s => {
    s = s.replace(',', '.');
    if (!/^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$/.test(s)) throw tr('Zahl erwartet');
    return Number(s);
  };
  if (f.typ === 'int') { if (!/^[-+]?\d+$/.test(t)) throw tr('ganze Zahl erwartet'); return Number(t); }
  if (f.typ === 'float') { if (t === '' && f.nullbar) return null; return zahl(t); }
  if (f.typ === 'str') return t;
  if (f.typ === 'liste') {
    if (t.replace(/^\[|\]$/g, '').trim() === '') return [];
    const teile = t.replace(/^\[|\]$/g, '').split(',').map(s => s.trim().replace(/^["']|["']$/g, ''));
    return f.elem === 'str' ? teile : teile.map(zahl);
  }
  throw tr('Typ unbekannt');
}
function eiSetzen(ab, f, v) {
  const n = eiName(ab, f), datei = eiDatei(ab, f);
  delete einst.falsch[n];
  if (v === null) {                                   // auf Default
    if (datei.da) einst.aend[n] = null; else delete einst.aend[n];
  } else if (eiGleich(v, datei.v) && (datei.da || eiGleich(v, f.default))) {
    delete einst.aend[n];                             // wie in der Datei bzw. Default ohne Eintrag
  } else {
    einst.aend[n] = v;
  }
  einstInfo();
}
function eiZeile(ab, f) {
  const n = eiName(ab, f), datei = eiDatei(ab, f);
  const unl = (einst.d.unlesbar[ab] || {})[f.key];
  const z = document.createElement('div');
  z.className = 'ei-feld' + (unl ? ' gesperrt' : '');
  const geaendert = n in einst.aend;
  const gesetzt = geaendert ? einst.aend[n] !== null : datei.da;
  z.innerHTML = '<div class="key">' + esc(ab === einst.d.schema.oben ? f.key : n) +
    (gesetzt ? '<span class="marke gesetzt" title="' + tr('steht in config.toml') + '">' + tr('gesetzt') + '</span>' : '') +
    (geaendert ? '<span class="marke geaendert">' + tr(einst.aend[n] === null ? 'wird Default' : 'geändert') + '</span>' : '') + '</div>';
  let inp;
  const v = eiWert(ab, f);
  if (f.typ === 'bool') {
    inp = document.createElement('input'); inp.type = 'checkbox'; inp.checked = !!v;
    inp.onchange = () => { eiSetzen(ab, f, inp.checked); einstZeigen(); };
  } else {
    inp = document.createElement('input'); inp.type = 'text'; inp.value = einst.falsch[n] ?? eiText(f, v);
    inp.spellcheck = false; inp.autocomplete = 'off';
    if (f.typ === 'int' || f.typ === 'float') inp.inputMode = 'decimal';
    if (n in einst.falsch) inp.classList.add('falsch');
    inp.onchange = () => {
      try { eiSetzen(ab, f, eiLesen(f, inp.value)); einstZeigen(); }
      catch (e) { einst.falsch[n] = inp.value; inp.classList.add('falsch'); inp.title = e; einstInfo(); }
    };
  }
  inp.setAttribute('aria-label', n);
  if (unl || !einst.d.schreibbar) inp.disabled = true;
  z.appendChild(inp);
  const zb = document.createElement('button');
  zb.className = 'zurueck'; zb.textContent = tr('Default');
  zb.title = tr('Default aus config.py: ') + (eiText(f, f.default) || tr('(leer)')) + tr(' – entfernt die Zeile aus config.toml');
  zb.disabled = !!unl || !einst.d.schreibbar || (!gesetzt && !(n in einst.falsch));
  zb.onclick = () => { eiSetzen(ab, f, null); einstZeigen(); };
  z.appendChild(zb);
  const h = document.createElement('div');
  h.className = 'hilfe';
  h.innerHTML = (unl ? '<b>' + esc(tr('In config.toml nicht lesbar ({x}), nur per ssh änderbar.', {x: unl})) + '</b> ' : '') + esc(f.hilfe || '') +
    ' <i>' + tr('Default: ') + esc(eiText(f, f.default) || tr('(leer)')) + (datei.da && !eiGleich(datei.v, eiWert(ab, f)) ? tr(' · in Datei: ') + esc(eiText(f, datei.v)) : '') + '</i>';
  z.appendChild(h);
  return z;
}
function einstNav() {
  const nav = $('ei-nav');
  nav.innerHTML = '';
  for (const k of einst.d.schema.kategorien) {
    const t = document.createElement('div'); t.className = 'kat'; t.textContent = k.name; nav.appendChild(t);
    for (const a of k.abschnitte) {
      const b = document.createElement('button');
      const nAend = Object.keys(einst.aend).filter(n => n.startsWith(a.name + '.')).length;
      const nGes = Object.keys(einst.d.werte[a.name] || {}).length;
      b.innerHTML = esc(a.name) + '<span class="n' + (nAend ? ' ae' : '') + '" title="' + tr('gesetzt / Felder') +
        (nAend ? tr(', ungespeichert geändert') : '') + '">' + (nAend ? nAend + '✎ ' : '') + nGes + '/' + a.felder.length + '</span>';
      b.classList.toggle('aktiv', !$('ei-suche').value.trim() && a.name === einst.ab);
      b.onclick = () => { einst.ab = a.name; $('ei-suche').value = ''; einstZeigen(); $('ei-felder').scrollTop = 0; };
      nav.appendChild(b);
    }
  }
  nav.querySelector('button.aktiv')?.scrollIntoView({block: 'nearest', inline: 'nearest'});
}
function einstZeigen() {
  if (!einst.d) return;
  const such = $('ei-suche').value.trim().toLowerCase(), nur = $('ei-nurgesetzt').checked;
  const box = $('ei-felder'), oben = box.scrollTop;
  box.innerHTML = '';
  let n = 0;
  for (const k of einst.d.schema.kategorien) for (const a of k.abschnitte) {
    if (!such && a.name !== einst.ab) continue;
    const felder = a.felder.filter(f => {
      const name = eiName(a.name, f);
      if (nur && !eiDatei(a.name, f).da && !(name in einst.aend)) return false;
      return !such || (name + ' ' + (f.hilfe || '')).toLowerCase().includes(such);
    });
    if (!felder.length && such) continue;
    const h = document.createElement('h3'); h.textContent = (such ? k.name + ' · ' : '') + '[' + a.name + ']'; box.appendChild(h);
    if (a.doc) { const p = document.createElement('p'); p.className = 'doc'; p.textContent = a.doc; box.appendChild(p); }
    for (const f of felder) { box.appendChild(eiZeile(a.name, f)); n++; }
  }
  if (!n) box.insertAdjacentHTML('beforeend', '<p class="leise">' + tr(such ? 'Nichts gefunden.' : 'Keine Einträge.') + '</p>');
  if (!such) box.scrollTop = oben;
  einstNav();
  einstInfo();
}
function einstInfo() {
  const nA = Object.keys(einst.aend).length, nF = Object.keys(einst.falsch).length, info = $('ei-info');
  let t, warn = false;
  // Mehrzahl: „(n)“/„(en)“ bzw. „(s)“ im Text
  const mz = (s, n) => LANG === 'en' ? s.replace(/\(s\)/g, n > 1 ? 's' : '') : s.replace(/\((e?n)\)/g, (_, e) => n > 1 ? e : '');
  if (!einst.d) t = tr('lädt …');
  else if (!einst.d.schreibbar) { t = einst.d.datei + tr(' ist nicht schreibbar (Gruppe video, 0664: deploy/install.sh)'); warn = true; }
  else if (nF) { t = mz(tr('{n} ungültige Eingabe(n)', {n: nF}), nF); warn = true; }
  else if (nA) t = mz(tr('{n} ungespeicherte Änderung(en) – wirkt erst nach Neustart', {n: nA}), nA);
  else if (einst.neustart) { t = tr('Gespeichert. Wirkt erst nach Neustart von xgonav.'); warn = true; }
  else t = tr('Keine Änderungen. ') + einst.d.datei;
  info.textContent = t;
  info.classList.toggle('warn', warn);
  $('ei-speichern').disabled = $('ei-speichern-neu').disabled = !nA || nF > 0 || !einst.d?.schreibbar;
}
function einstSicherungen() {
  const ul = $('ei-sich-liste');
  ul.innerHTML = '';
  const liste = einst.d.sicherungen;
  $('ei-sich').querySelector('summary').textContent = tr('Sicherungen') + ' (' + liste.length + ')';
  for (const s of liste) {
    const li = document.createElement('li');
    li.innerHTML = '<span>' + new Date(s.zeit * 1000).toLocaleString(LOCALE()) + '</span><code>' + esc(s.name) + '</code>';
    const b = document.createElement('button');
    b.textContent = tr('Zurückspielen');
    b.disabled = !einst.d.schreibbar;
    b.onclick = async () => {
      if (!confirm(tr('config.toml durch die Sicherung {name} ersetzen? Der jetzige Stand wird vorher gesichert. Wirkt nach Neustart.', {name: s.name}))) return;
      const j = await einstPost('config_restore', {name: s.name, hash: einst.d.hash});
      if (j) { einst.neustart = true; await einstLaden(); zeige(''); }
    };
    li.appendChild(b);
    ul.appendChild(li);
  }
  if (!liste.length) ul.innerHTML = '<li class="leise">' + tr('noch keine') + '</li>';
}
async function einstPost(a, p, body) {
  try {
    const r = await fetch('api.php?' + new URLSearchParams({a, ...p}), {method: 'POST', body,
      headers: body ? {'Content-Type': 'application/json'} : {}});
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { j.status = r.status; throw j; }
    return j;
  } catch (e) {
    if (e && e.status === 409 && e.grund) throw e;     // Neustart: Rückfrage beim Aufrufer
    alert((e && e.error) || tr('keine Verbindung'));
    return null;
  }
}
async function einstLaden() {
  try {
    const r = await fetch('api.php?a=config');
    const j = await r.json();
    if (!r.ok) throw j.error || (tr('Fehler ') + r.status);
    einst.d = j;
  } catch (e) { $('ei-info').textContent = tr('Laden fehlgeschlagen: ') + e; return false; }
  einst.aend = {}; einst.falsch = {};
  if (!einst.ab) einst.ab = eiErster();
  einstSicherungen();
  einstZeigen();
  return true;
}
function eiErster() { return einst.d.schema.kategorien[0].abschnitte[0].name; }
async function einstOeffnen() {
  $('einst').showModal();
  if (!Object.keys(einst.aend).length) { einst.d = null; einstInfo(); await einstLaden(); }
  else einstZeigen();                                 // ungespeicherte Änderungen bleiben beim Schließen erhalten
}
function einstSchliessen() { $('einst').close(); }
function einstVerwerfen() {
  if (Object.keys(einst.aend).length && !confirm(tr('Alle ungespeicherten Änderungen verwerfen?'))) return;
  einst.aend = {}; einst.falsch = {};
  einstZeigen();
}
async function einstSpeichern(neustarten) {
  const zeilen = [];
  for (const k of einst.d.schema.kategorien) for (const a of k.abschnitte) for (const f of a.felder) {
    const n = eiName(a.name, f);
    if (!(n in einst.aend)) continue;
    const alt = eiDatei(a.name, f);
    const dflt = tr('(Default ') + (eiText(f, f.default) || tr('leer')) + ')';
    zeilen.push(n + ': ' + (alt.da ? eiText(f, alt.v) || tr('(leer)') : dflt) + ' → ' +
      (einst.aend[n] === null ? dflt : eiText(f, einst.aend[n]) || tr('(leer)')));
  }
  if (!confirm(tr('In config.toml schreiben (vorher Sicherung):') + '\n\n' + zeilen.join('\n') + (neustarten ? '\n\n' + tr('Danach xgonav neu starten.') : ''))) return;
  const j = await einstPost('config_save', {hash: einst.d.hash}, JSON.stringify(einst.aend));
  if (!j) return;
  einst.neustart = true;
  await einstLaden();
  if (neustarten) await einstNeustart(true);
}
async function einstNeustart(ohneFrage = false) {
  if (!ohneFrage && !confirm(tr('xgonav neu starten? Er liest dabei config.toml neu ein; Bild und Status sind einige Sekunden weg.'))) return;
  let j;
  try { j = await einstPost('restart', {}); }
  catch (e) {
    if (!confirm(e.error + '.\n\n' + tr('Trotzdem neu starten? Laufende Fahrt, Aufzeichnung und Modus enden.'))) return;
    try { j = await einstPost('restart', {trotzdem: 1}); } catch (e2) { alert(e2.error); return; }
  }
  if (!j) return;
  einst.neustart = false;
  einstInfo();
  $('ei-info').textContent = tr('xgonav startet neu …');
  zeige('xgonav startet neu …');
  setTimeout(() => { if (/startet neu|is restarting/.test($('meldung').textContent)) zeige(''); }, 15000);
}

// Raumkarte: einmal pro Sekunde neu laden, nächstes Bild erst nach dem vorigen; eingeklappt oder Tab verborgen
// gar nicht (map.png rechnet am Rider), Aufklappen und Sichtbarwerden stoßen wieder an. Verschieben und Zoomen
// zeichnet der Server; bis das neue Bild da ist, verschiebt und skaliert CSS das alte.
// Karten-KS: oben = +x, links = +y (m). cx/cy null = Mitte folgt dem Rider.
const karte = {cx: null, cy: null, v: 6, bild: null};   // bild: Stand des angezeigten Bildes (X-Map-View)
let karteUrl = null, karteTimer = null, karteLaeuft = false, karteNochmal = false;
async function karteHolen() {
  clearTimeout(karteTimer);
  if (ruhe || !$('raumkarte').open || document.hidden) return;   // Wecken/Aufklappen/Sichtbar stößt wieder an
  if (karteLaeuft) { karteNochmal = true; return; }
  karteLaeuft = true;
  try {
    const mitte = karte.cx == null ? '' : '&cx=' + karte.cx.toFixed(3) + '&cy=' + karte.cy.toFixed(3);
    const r = await fetch('map.png?px=960&view=' + karte.v.toFixed(3) + mitte, {cache: 'no-store'});
    if (r.ok) {
      const [cx, cy, v, size] = (r.headers.get('X-Map-View') || '').split(',').map(Number);
      const url = URL.createObjectURL(await r.blob()), img = $('karte');
      img.onload = () => { karte.bild = size ? {cx, cy, v, size} : null; karteAnzeigen(); };
      img.src = url;
      if (karteUrl) URL.revokeObjectURL(karteUrl);
      karteUrl = url;
    }
  } catch (e) { /* xgonav weg */ }
  karteLaeuft = false;
  karteTimer = setTimeout(karteHolen, karteNochmal ? 0 : 1000);
  karteNochmal = false;
}
const karteAnstoss = () => { clearTimeout(karteTimer); karteTimer = setTimeout(karteHolen, 150); };
function karteAnzeigen() {
  const b = karte.bild, W = $('karte-rahmen').clientWidth;
  if (!b) { $('karte').style.transform = ''; return; }
  const cx = karte.cx == null ? b.cx : karte.cx, cy = karte.cy == null ? b.cy : karte.cy;
  $('karte').style.transform = 'translate(' + W * (cy - b.cy) / karte.v + 'px,' + W * (cx - b.cx) / karte.v + 'px) scale(' + b.v / karte.v + ')';
  punkteZeichnen();
}
// Route und Sperrzonen setzen: Klick auf die Karte legt einen Punkt in Karten-Koordinaten an (oben = +x, links = +y).
// Gespeichertes zeichnet der Server ins Bild, die noch nicht übernommenen Punkte liegen als SVG darüber.
const bearb = {art: null, pts: []};
let karteGesperrt = false;
async function routeHolen() {
  const r = await fetch('api.php?a=route', {cache: 'no-store'});
  if (!r.ok) throw new Error(tr('Route nicht lesbar'));
  return r.json();
}
const punktText = pts => pts.map(p => p[0].toFixed(3) + ',' + p[1].toFixed(3)).join(';');
const zonenText = zonen => zonen.map(punktText).join('|');
function punkteZeichnen() {
  const svg = $('karte-punkte'), b = karte.bild;
  if (!bearb.art || !b) { svg.innerHTML = ''; return; }
  const cx = karte.cx == null ? b.cx : karte.cx, cy = karte.cy == null ? b.cy : karte.cy;
  const q = bearb.pts.map(p => [0.5 - (p[1] - cy) / karte.v, 0.5 - (p[0] - cx) / karte.v]);
  const farbe = bearb.art === 'zone' ? '#dc2828' : '#c83cc8';
  const linie = q.map(p => p.join(',')).join(' ');
  const form = bearb.art === 'zone' ? 'polygon' : 'polyline';
  svg.innerHTML = '<' + form + ' points="' + linie + '" fill="none" stroke="' + farbe + '" stroke-width="2" vector-effect="non-scaling-stroke"/>'
    + q.map(p => '<circle cx="' + p[0] + '" cy="' + p[1] + '" r="0.012" fill="' + farbe + '"/>').join('');
}
function bearbeitenHinweis() {
  $('route-hinweis').textContent = tr(bearb.art === 'zone' ? 'Ecken der Sperrzone anklicken: ' : 'Wegpunkte anklicken: ') + bearb.pts.length;
}
async function bearbeiten(art) {
  let akt = {points: [], loop: false};
  if (art === 'route') {
    try { akt = await routeHolen(); } catch (e) { zeige(e.message); return; }
  }
  bearb.art = art;
  bearb.pts = art === 'route' ? akt.points.map(p => [p[0], p[1]]) : [];
  $('route-loop').checked = !!akt.loop;
  $('route-kreis').hidden = art !== 'route';
  $('karte-gruppen').disabled = true; document.querySelectorAll('.karte-knoepfe button').forEach(b => b.disabled = true); $('route-edit').hidden = false;
  $('karte-rahmen').classList.add('setzt');
  bearbeitenHinweis(); punkteZeichnen();
}
function bearbeitenEnde() {
  bearb.art = null; bearb.pts = [];
  $('karte-gruppen').disabled = false; document.querySelectorAll('.karte-knoepfe button').forEach(b => b.disabled = false); $('route-edit').hidden = true;
  $('karte-rahmen').classList.remove('setzt');
  punkteZeichnen();
}
function punktWeg() { bearb.pts.pop(); bearbeitenHinweis(); punkteZeichnen(); }
async function bearbeitenFertig() {
  let akt;
  try { akt = await routeHolen(); } catch (e) { zeige(e.message); return; }
  const p = {pts: punktText(akt.points), loop: akt.loop ? 1 : 0};
  if (bearb.art === 'route') {
    p.pts = punktText(bearb.pts); p.loop = $('route-loop').checked ? 1 : 0;
  } else {
    if (bearb.pts.length < 3) { zeige('Eine Sperrzone braucht mindestens 3 Ecken'); return; }
    p.zones = zonenText([...akt.zones, bearb.pts]);
  }
  if (await befehl('route', p)) { bearbeitenEnde(); karteAnstoss(); }
}
async function zonenLoeschen() {
  if (!confirm(tr('Alle Sperrzonen löschen?'))) return;
  let akt;
  try { akt = await routeHolen(); } catch (e) { zeige(e.message); return; }
  if (await befehl('route', {pts: punktText(akt.points), loop: akt.loop ? 1 : 0, nozones: 1})) karteAnstoss();
}
function karteZuruecksetzen() {
  if (confirm(tr('Raumkarte zurücksetzen? Route und Sperrzonen gehen dabei verloren.'))) befehl('map_reset');
}
function karteSperren() { return befehl('map_lock', {on: karteGesperrt ? 0 : 1}); }
// Ziehen darf kein System-Drag werden: Ist Text markiert, zöge der Browser sonst die Markierung mit und
// bräche die Bewegung per pointercancel ab.
function keinSystemDrag(el) {
  el.addEventListener('mousedown', e => { if (e.button === 0) e.preventDefault(); });
  el.addEventListener('dragstart', e => e.preventDefault());
}
function karteView(v) {     // Auswahlfeld
  karte.v = Number(v); karteAnzeigen(); karteAnstoss();
}
// Nur Maus/Stift: Auf dem Touchscreen bleibt Wischen über der Karte das Scrollen der Seite.
(() => {
  const el = $('karte-rahmen'), mitte = () => [karte.cx == null ? karte.bild.cx : karte.cx, karte.cy == null ? karte.bild.cy : karte.cy];
  const grenze = c => Math.max(-karte.bild.size, Math.min(karte.bild.size, c));
  let start = null, gezogen = false;
  const auflage = e => e.target.closest('.karte-auflage');    // Knöpfe über der Karte: kein Punkt, kein Ziehen
  el.addEventListener('mousedown', e => { if (!auflage(e) && e.button === 0) e.preventDefault(); });
  el.addEventListener('dragstart', e => e.preventDefault());
  // Klick (Maus oder Finger): Punkt setzen. Ziehen verschiebt weiter die Karte.
  el.addEventListener('click', e => {
    if (auflage(e)) return;
    if (!bearb.art || !karte.bild || gezogen) { gezogen = false; return; }
    const q = $('karte-flaeche').getBoundingClientRect(), [cx, cy] = mitte();   // quadratische Fläche, nicht der Rahmen
    const fx = (e.clientX - q.left) / q.width, fy = (e.clientY - q.top) / q.height;
    bearb.pts.push([cx + (0.5 - fy) * karte.v, cy + (0.5 - fx) * karte.v]);
    bearbeitenHinweis(); punkteZeichnen();
  });
  el.addEventListener('pointerdown', e => {
    if (e.pointerType === 'touch' || e.button !== 0 || !karte.bild || auflage(e)) return;
    start = {x: e.clientX, y: e.clientY, m: mitte()};
    gezogen = false;
    el.setPointerCapture(e.pointerId); el.classList.add('zieht');
  });
  el.addEventListener('pointermove', e => {
    if (!start) return;
    const dx = e.clientX - start.x, dy = e.clientY - start.y, W = el.clientWidth;
    if (!dx && !dy) return;
    if (Math.abs(dx) + Math.abs(dy) > 3) gezogen = true;
    if (!gezogen) return;
    karte.cx = grenze(start.m[0] + dy / W * karte.v); karte.cy = grenze(start.m[1] + dx / W * karte.v);
    karteAnzeigen();
  });
  const ende = () => { if (start) { start = null; el.classList.remove('zieht'); karteAnstoss(); } };
  el.addEventListener('pointerup', ende); el.addEventListener('pointercancel', ende);
  el.addEventListener('dblclick', e => { if (bearb.art || auflage(e)) return; karte.cx = karte.cy = null; karteAnzeigen(); karteAnstoss(); });
  el.addEventListener('wheel', e => {
    if (!karte.bild) return;
    e.preventDefault();
    const d = e.deltaY * (e.deltaMode === 1 ? 33 : e.deltaMode === 2 ? 400 : 1);
    const neu = Math.max(1, Math.min(karte.bild.size, karte.v * Math.exp(d * 0.0015)));
    if (neu === karte.v) return;
    // Der Punkt unter dem Mauszeiger bleibt stehen
    const q = $('karte-flaeche').getBoundingClientRect(), fx = (e.clientX - q.left) / q.width, fy = (e.clientY - q.top) / q.height, [cx, cy] = mitte();
    karte.cx = grenze(cx + (0.5 - fy) * (karte.v - neu)); karte.cy = grenze(cy + (0.5 - fx) * (karte.v - neu));
    karte.v = neu;
    const o = $('karte-view-frei');
    o.value = String(neu); o.textContent = neu.toFixed(1) + ' m'; o.hidden = false; $('karte-view').value = o.value;
    karteAnzeigen(); karteAnstoss();
  }, {passive: false});
})();
// --- Verlauf: Lage (IMU), CPU je Kern, Akku. Abfrage nur bei offener Karte, Akku immer
// (aus status()) und im localStorage, damit er ein Neuladen der Seite übersteht.
const FARBEN = ['#3987e5', '#d95926', '#199e70', '#c98500'];   // Kategorien, dunkel, geprüft (CVD)
const LAGE_MS = 5e3, CPU_MS = 600e3, AKKU_MS = 120 * 60e3, AKKU_TAKT_MS = 30e3, AKKU_KEY = 'xgonav.akku';
const lage = [];            // [t_ms (Treiberzeit), roll, pitch, gier] – Puffer zum Abspielen der 50-Hz-Proben
const cpu = [];             // [t_ms, [Kern0 %, Kern1 %, …]]
let imuSeq = 0, cpuVorher = null, cpuZeit = 0;
// Rad-Encoder aus wheels_log (gleiche Treiberzeit wie die Lage): [t_ms, Ticks links, Ticks rechts], fortlaufend
// über den int32-Überlauf; 1024 Ticks je Umdrehung, vorwärts +. radRoh = letzte Rohwerte zum Entfalten.
const radPuffer = [], RAD_TICKS = 1024;
let radSeq = 0, radRoh = null, radJetzt = null;

// --- Lage als 3D-Modell (eigene Projektion, keine Bibliothek): Ring und Kurslinie im Canvas 2D, darüber
// das CAD-Modell rider-model.bin per WebGL (tools/step_to_model.py); ohne das die Quader in MODELL. Körper-KS: x vorn,
// y links, z hoch (cm). Firmware: Roll + = rechts tief, Nick + = vorn tief, Gier + = links.
const LAGE_VERZUG = 400;    // ms hinter der neuesten Probe abspielen (Live-Paket alle 300 ms)
const ACHSE = [0, 0, 3.2];  // um die Radachse drehen: Räder bleiben am Boden
function quader(c, l, farbe) {
  const [x, y, z] = c, [a, b, h] = l.map(v => v / 2);
  const P = (i, j, k) => [x + i * a, y + j * b, z + k * h];
  return [
    {p: [P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1)], n: [1, 0, 0], farbe},
    {p: [P(-1, 1, -1), P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1)], n: [-1, 0, 0], farbe},
    {p: [P(1, 1, -1), P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1)], n: [0, 1, 0], farbe},
    {p: [P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1)], n: [0, -1, 0], farbe},
    {p: [P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1)], n: [0, 0, 1], farbe},
    {p: [P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1), P(-1, -1, -1)], n: [0, 0, -1], farbe},
  ];
}
function rad(c, r, breite, farbe, n = 18) {   // Zylinder, Achse y
  const [x, y, z] = c, f = [], au = [], in_ = [];
  for (let k = 0; k < n; k++) {
    const w = 2 * Math.PI * k / n;
    au.push([x + r * Math.cos(w), y + breite / 2, z + r * Math.sin(w)]);
    in_.push([x + r * Math.cos(w), y - breite / 2, z + r * Math.sin(w)]);
  }
  f.push({p: au.slice().reverse(), n: [0, 1, 0], farbe: '#3a414c'});
  f.push({p: in_, n: [0, -1, 0], farbe: '#3a414c'});
  for (let k = 0; k < n; k++) {
    const k2 = (k + 1) % n, w = 2 * Math.PI * (k + .5) / n;
    f.push({p: [au[k], au[k2], in_[k2], in_[k]], n: [Math.cos(w), 0, Math.sin(w)], farbe});
  }
  return f;
}
const MODELL = [
  ...quader([0, 0, 12], [9, 7, 7], '#6b7683'),           // Kopf/Körper, zwischen den Beinen
  ...quader([4.7, 0, 12.6], [0.5, 5.4, 4.6], '#3987e5'),  // Front (Display)
  ...quader([5.1, 0, 14.6], [0.6, 1.6, 1.1], '#e3e6ea'),  // Kamera
  ...quader([0, 4.2, 8.2], [1.4, 1.0, 10], '#4a535e'),    // Bein links (Radachse bis Kopfhöhe)
  ...quader([0, -4.2, 8.2], [1.4, 1.0, 10], '#4a535e'),   // Bein rechts
  ...rad([0, 6.2, 3.2], 3.2, 1.8, '#23272d'),             // Rad links (11)
  ...rad([0, -6.2, 3.2], 3.2, 1.8, '#23272d'),            // Rad rechts (21)
];
// CAD-Modell: Format siehe modell_lesen in tools/step_to_model.py. Flächennormalen beim Laden, daher ohne Indizes.
const MODELL_ACHSE = [0, 0, 3.0];    // Radachse im CAD (Raddurchmesser 60 mm)
const GL_VS = `attribute vec3 aP, aN, aC; attribute float aTeil; varying vec3 vN, vC;
uniform mat3 uR; uniform vec3 uA, uAchse, uPos, uRe, uHo, uVo; uniform vec4 uS; uniform vec2 uT; uniform vec4 uTeil[9];
void main() {
  vec4 b = uTeil[int(aTeil + 0.5)];     // cos, sin, tx, tz in der x/z-Ebene (Beinglieder)
  vec3 p = vec3(b.x * aP.x - b.y * aP.z + b.z, aP.y, b.y * aP.x + b.x * aP.z + b.w);
  vec3 q = uR * (p - uA) + uAchse - uPos;
  float x = dot(q, uRe), y = dot(q, uHo), z = dot(q, uVo);
  gl_Position = vec4(uS.x * x + uS.y * z, uS.z * y + uS.w * z, uT.x * z + uT.y, z);
  vN = uR * vec3(b.x * aN.x - b.y * aN.z, aN.y, b.y * aN.x + b.x * aN.z); vC = aC;
}`;
const GL_FS = `precision mediump float; varying vec3 vN, vC; uniform vec3 uL;
void main() {
  vec3 n = normalize(vN);
  if (!gl_FrontFacing) n = -n;
  gl_FragColor = vec4(vC * (0.42 + 0.58 * max(0.0, dot(n, uL))), 1.0);
}`;
let lageGL = null, lageGLVersucht = false;
function modellDekodieren(buf) {
  const k = new DataView(buf);
  if (k.getUint32(0, true) !== 0x32524758) throw new Error('kein Rider-Modell');
  const nP = k.getUint32(4, true), nD = k.getUint32(8, true), nG = k.getUint32(12, true), s = k.getFloat32(16, true);
  const o = 20 + 8 * nG + 80, pos = new Int16Array(buf, o, nP * 3), idx = new Uint16Array(buf, o + ((nP * 6 + 3) & ~3), nD * 3);
  const dreh = Array.from({length: 2}, (_, j) => Object.fromEntries(['S', 'P', 'A', 'B', 'W'].map((n, m) =>
    [n, [k.getFloat32(20 + 8 * nG + 40 * j + 8 * m, true), k.getFloat32(20 + 8 * nG + 40 * j + 8 * m + 4, true)]])));
  const a = new Float32Array(nD * 30);
  let z = 0, g = 0, rest = 0, c = null, teil = 0;
  for (let i = 0; i < nD; i++) {
    while (!rest) { const b = 20 + 8 * g++; c = [k.getUint8(b) / 255, k.getUint8(b + 1) / 255, k.getUint8(b + 2) / 255]; teil = k.getUint8(b + 3); rest = k.getUint32(b + 4, true); }
    rest--;
    const P = [0, 1, 2].map(j => { const v = idx[3 * i + j] * 3; return [pos[v] * s, pos[v + 1] * s, pos[v + 2] * s]; });
    const u = P[1].map((v, m) => v - P[0][m]), w = P[2].map((v, m) => v - P[0][m]);
    const n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
    for (const p of P) { a.set(p, z); a.set(n, z + 3); a.set(c, z + 6); a[z + 9] = teil; z += 10; }
  }
  return {daten: a, dreh};
}
// Beine: Sollwerte aus dem Treiber (Höhe u8, Roll °), Firmware meldet keine Stellung. Unbekannt: Standard 128.
// Abgesetzt (Zustand 254): Beine auf dem Minimum der Firmware, die Regler behalten ihren Wert.
const BEIN_MIN_MM = 45;
const beinSoll = {h: 128, r: 0, abgesetzt: false}, beinIst = {l: null, r: null, t: 0};
const grenze = (v, a, b) => Math.max(a, Math.min(b, v));
function beinStreckung(h, r) {    // mm links, rechts wie Firmware (main.cpp, updateLegTargets)
  const m = grenze(h, 55, 200) * 36 / 255, d = Math.sin(r / 57.3) * 43;
  return [grenze(m + d, 0, 36) + 45, grenze(m - d, 0, 36) + 45];
}
// Viergelenk S-A-B-P (cm, x/z): je Glied [cos, sin, tx, tz]: Servo-Glied, oberes Glied, Koppel, Rad. Wie beinlage im Konverter.
function beinTeile({S, P, A, B, W}, e, ticks = 0) {
  const abst = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1]);
  const um = (q, w) => { const c = Math.cos(w), s = Math.sin(w); return [c, s, q[0] - c * q[0] + s * q[1], q[1] - s * q[0] - c * q[1]]; };
  const w = Math.acos(abst(S, W) * 10 / 96) - Math.acos(e / 96), bS = um(S, w);
  const A2 = [bS[0] * A[0] - bS[1] * A[1] + bS[2], bS[1] * A[0] + bS[0] * A[1] + bS[3]];
  const ab = abst(A, B), pb = abst(P, B), dd = abst(P, A2), a = (pb * pb - ab * ab + dd * dd) / (2 * dd), h = Math.sqrt(Math.max(0, pb * pb - a * a));
  const mx = P[0] + a * (A2[0] - P[0]) / dd, mz = P[1] + a * (A2[1] - P[1]) / dd, ex = (A2[1] - P[1]) / dd, ez = -(A2[0] - P[0]) / dd;
  const B1 = [mx + h * ex, mz + h * ez], B3 = [mx - h * ex, mz - h * ez], B2 = abst(B1, B) <= abst(B3, B) ? B1 : B3;
  const oben = Math.atan2(B2[1] - P[1], B2[0] - P[0]) - Math.atan2(B[1] - P[1], B[0] - P[0]);
  const r = Math.atan2(B2[1] - A2[1], B2[0] - A2[0]) - Math.atan2(B[1] - A[1], B[0] - A[0]), c = Math.cos(r), s = Math.sin(r);
  const K = [c, s, A2[0] - c * A[0] + s * A[1], A2[1] - s * A[0] - c * A[1]];
  const W2 = [c * W[0] - s * W[1] + K[2], s * W[0] + c * W[1] + K[3]];
  // Rad: um die eigene Achse W drehen (Encoder), dann nach W2. Vorwärts rollen = vorn nach unten (x -> -z)
  const phi = -2 * Math.PI * (((ticks % RAD_TICKS) + RAD_TICKS) % RAD_TICKS) / RAD_TICKS, rc = Math.cos(phi), rs = Math.sin(phi);
  const Rad = [rc, rs, W2[0] - (rc * W[0] - rs * W[1]), W2[1] - (rs * W[0] + rc * W[1])];
  return {teile: [bS, um(P, oben), K, Rad], achse: W2};
}
async function modellLaden() {
  lageGLVersucht = true;
  try {
    const cv = $('d-lage-gl'), gl = cv.getContext('webgl', {antialias: true});
    if (!gl) throw new Error('kein WebGL');
    const r = await fetch('rider-model.bin');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const {daten, dreh} = modellDekodieren(await r.arrayBuffer());
    const prog = gl.createProgram();
    for (const [typ, text] of [[gl.VERTEX_SHADER, GL_VS], [gl.FRAGMENT_SHADER, GL_FS]]) {
      const sh = gl.createShader(typ); gl.shaderSource(sh, text); gl.compileShader(sh); gl.attachShader(prog, sh);
    }
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
    gl.useProgram(prog);
    gl.bindBuffer(gl.ARRAY_BUFFER, gl.createBuffer());
    gl.bufferData(gl.ARRAY_BUFFER, daten, gl.STATIC_DRAW);
    [['aP', 0, 3], ['aN', 12, 3], ['aC', 24, 3], ['aTeil', 36, 1]].forEach(([n, off, g]) => {
      const a = gl.getAttribLocation(prog, n); gl.enableVertexAttribArray(a); gl.vertexAttribPointer(a, g, gl.FLOAT, false, 40, off);
    });
    gl.enable(gl.DEPTH_TEST); gl.clearColor(0, 0, 0, 0);
    const u = {};
    for (const n of ['uR', 'uA', 'uAchse', 'uPos', 'uRe', 'uHo', 'uVo', 'uS', 'uT', 'uL', 'uTeil']) u[n] = gl.getUniformLocation(prog, n);
    gl.uniform3fv(u.uL, LICHT); gl.uniform3fv(u.uAchse, MODELL_ACHSE);
    const nah = 30, fern = 110;   // Kameraabstand 70 cm, das Modell reicht höchstens ±25 cm darum
    gl.uniform2f(u.uT, (fern + nah) / (fern - nah), -2 * fern * nah / (fern - nah));
    cv.addEventListener('webglcontextlost', () => { lageGL = null; });
    lageGL = {gl, u, n: daten.length / 10, dreh};
  } catch (e) { console.warn('Rider-Modell nicht geladen, zeichne Quader:', e); }
}
function modellZeichnen(K, R, B, H, dpr) {
  const {gl, u, n, dreh} = lageGL, cv = gl.canvas, b = Math.round(B * dpr), h = Math.round(H * dpr);
  if (cv.width !== b || cv.height !== h) { cv.width = b; cv.height = h; }
  gl.viewport(0, 0, b, h);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  if (!R) return;
  // Tiefpass wie die Firmware (~0,33 s), damit Reglerzüge nicht springen
  const jetzt = performance.now(), f = beinIst.l == null ? 1 : 1 - Math.exp(-(jetzt - beinIst.t) / 330);
  const [zL, zR] = beinSoll.abgesetzt ? [BEIN_MIN_MM, BEIN_MIN_MM] : beinStreckung(beinSoll.h, beinSoll.r);
  beinIst.l = beinIst.l == null ? zL : beinIst.l + (zL - beinIst.l) * f;
  beinIst.r = beinIst.r == null ? zR : beinIst.r + (zR - beinIst.r) * f;
  beinIst.t = jetzt;
  const [tL, tR] = radJetzt || [0, 0];
  const L = beinTeile(dreh[0], beinIst.l, tL), Re = beinTeile(dreh[1], beinIst.r, tR);
  gl.uniform4fv(u.uTeil, [1, 0, 0, 0, ...Re.teile.flat(), ...L.teile.flat()]);   // 0 Körper, 1..4 rechts, 5..8 links
  // Die Mitte der Radachsen liegt fest: Der Körper hebt und senkt sich, die Räder bleiben am Boden
  gl.uniform3f(u.uA, (L.achse[0] + Re.achse[0]) / 2, 0, (L.achse[1] + Re.achse[1]) / 2);
  gl.uniformMatrix3fv(u.uR, false, [R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2]]);
  gl.uniform3fv(u.uPos, K.pos); gl.uniform3fv(u.uRe, K.r); gl.uniform3fv(u.uHo, K.u); gl.uniform3fv(u.uVo, K.f);
  // wie kamera(): Pixel = (cx + k·x/z, cy − k·y/z), hier als Clip-Koordinaten mit w = z
  gl.uniform4f(u.uS, 2 * K.k / B, 2 * K.cx / B - 1, 2 * K.k / H, 1 - 2 * K.cy / H);
  gl.drawArrays(gl.TRIANGLES, 0, n);
}
function drehmatrix(roll, pitch, gier) {      // R = Rz(gier) · Ry(nick) · Rx(roll), Grad
  const g = Math.PI / 180, [cr, sr] = [Math.cos(roll * g), Math.sin(roll * g)];
  const [cp, sp] = [Math.cos(pitch * g), Math.sin(pitch * g)], [cy, sy] = [Math.cos(gier * g), Math.sin(gier * g)];
  return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
          [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
          [-sp, cp * sr, cp * cr]];
}
const mal = (R, v) => [0, 1, 2].map(i => R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2]);
const skal = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
// Kamera auf einer Kugel um die Modellmitte; Azimut/Elevation mit der Maus (Standard: vorn links oben)
const LAGE_SICHT = [40, 28];
let lageSicht = LAGE_SICHT.slice();
try { const v = JSON.parse(localStorage.getItem('xgonav.lage3d')); if (Array.isArray(v) && v.length === 2) lageSicht = v.map(Number); } catch (e) {}
function kamera(B, H) {
  const az = lageSicht[0] * Math.PI / 180, el = lageSicht[1] * Math.PI / 180, dist = 70, ziel = [0, 0, 5];
  const d = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
  const pos = d.map((v, i) => ziel[i] + v * dist), f = d.map(v => -v);
  const r = [f[1], -f[0], 0], rl = Math.hypot(r[0], r[1]); r[0] /= rl; r[1] /= rl;   // f × z
  const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
  const norm = p => {                                   // ohne Maßstab: [rechts, unten] / Tiefe
    const q = [p[0] - pos[0], p[1] - pos[1], p[2] - pos[2]], z = skal(q, f);
    return [skal(q, r) / z, -skal(q, u) / z, z];
  };
  // Rahmen für jeden Blickwinkel aus Ring (r 17) und einem Kasten um alle Lagen des Modells
  // (±13 cm um die Achse, 0..17 cm hoch, Beine ganz ausgefahren): 90 % der Fläche, mittig
  const rand = [];
  for (let k = 0; k < 36; k++) { const w = Math.PI * k / 18; rand.push(norm([17 * Math.cos(w), 17 * Math.sin(w), 0])); }
  for (const x of [-13, 13]) for (const y of [-13, 13]) for (const z of [0, 17]) rand.push(norm([x, y, z]));
  const xs = rand.map(v => v[0]), ys = rand.map(v => v[1]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const k = 0.9 * Math.min(B / (x1 - x0), H / (y1 - y0)), cx = B / 2 - k * (x0 + x1) / 2, cy = H / 2 - k * (y0 + y1) / 2;
  return {pos, f, r, u, k, cx, cy, proj: p => { const v = norm(p); return [cx + k * v[0], cy + k * v[1], v[2]]; }};
}
// Lage zur Treiberzeit t, linear zwischen den Proben (Gier ist fortlaufend, nicht gewickelt)
function lageBei(t) {
  if (!lage.length) return null;
  const i = suche(lage, t);
  if (i <= 0) return lage[0].slice(1);
  if (i >= lage.length) return lage[lage.length - 1].slice(1);
  const a = lage[i - 1], b = lage[i], f = (t - a[0]) / ((b[0] - a[0]) || 1);
  return [1, 2, 3].map(k => a[k] + (b[k] - a[k]) * f);
}
// Radticks zur Treiberzeit t, linear zwischen den Proben; null ohne Raddaten
function radBei(t) {
  if (!radPuffer.length) return null;
  const i = suche(radPuffer, t);
  if (i <= 0) return radPuffer[0].slice(1);
  if (i >= radPuffer.length) return radPuffer[radPuffer.length - 1].slice(1);
  const a = radPuffer[i - 1], b = radPuffer[i], f = (t - a[0]) / ((b[0] - a[0]) || 1);
  return [a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}
function farbe(hex, s) {
  const v = parseInt(hex.slice(1), 16);
  return 'rgb(' + [v >> 16, (v >> 8) & 255, v & 255].map(c => Math.round(c * s)).join(',') + ')';
}
const LICHT = (v => v.map(x => x / Math.hypot(...v)))([0.5, 0.35, 1]);
function lageZeichnen(w) {
  const cv = $('d-lage'), dpr = window.devicePixelRatio || 1, B = cv.clientWidth, H = Number(cv.dataset.h);
  if (!B) return;
  if (cv.width !== Math.round(B * dpr) || cv.height !== Math.round(H * dpr)) {
    cv.width = Math.round(B * dpr); cv.height = Math.round(H * dpr); cv.style.height = H + 'px';
  }
  const g = cv.getContext('2d'), K = kamera(B, H);
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, B, H);
  g.font = '11px system-ui,sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  // Boden: Kompassring, fest in der Welt (Gier dreht das Modell dagegen); 0° = Kurs beim ESP32-Start
  const ring = (r, n = 72) => Array.from({length: n + 1}, (_, k) => K.proj([r * Math.cos(2 * Math.PI * k / n), r * Math.sin(2 * Math.PI * k / n), 0]));
  g.strokeStyle = '#2b3038'; g.lineWidth = 1;
  for (const r of [7, 14]) { g.beginPath(); ring(r).forEach((p, k) => k ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1])); g.stroke(); }
  for (let a = 0; a < 360; a += 30) {
    const c = Math.cos(a * Math.PI / 180), s = Math.sin(a * Math.PI / 180);
    const p1 = K.proj([12.8 * c, 12.8 * s, 0]), p2 = K.proj([14 * c, 14 * s, 0]), pt = K.proj([15.8 * c, 15.8 * s, 0]);
    g.strokeStyle = a % 90 ? '#2b3038' : '#4a535e'; g.beginPath(); g.moveTo(p1[0], p1[1]); g.lineTo(p2[0], p2[1]); g.stroke();
    if (a % 90 === 0) { g.fillStyle = '#8b94a1'; g.fillText(a > 180 ? (a - 360) + '°' : a + '°', pt[0], pt[1]); }
  }
  if (!w) { if (lageGL) modellZeichnen(K, null, B, H, dpr); g.fillStyle = '#8b94a1'; g.fillText(tr('keine IMU-Daten'), B / 2, 16); return; }
  const R = drehmatrix(w[0], w[1], w[2]);
  const welt = p => { const q = mal(R, [p[0] - ACHSE[0], p[1] - ACHSE[1], p[2] - ACHSE[2]]); return [q[0] + ACHSE[0], q[1] + ACHSE[1], q[2] + ACHSE[2]]; };
  // Schatten/Kursanzeiger am Boden: Vorwärtsrichtung des Körpers, auf den Boden projiziert
  const vor = mal(R, [1, 0, 0]), vl = Math.hypot(vor[0], vor[1]) || 1;
  const a0 = K.proj([0, 0, 0]), a1 = K.proj([12 * vor[0] / vl, 12 * vor[1] / vl, 0]);
  g.strokeStyle = '#3987e5'; g.lineWidth = 2; g.beginPath(); g.moveTo(a0[0], a0[1]); g.lineTo(a1[0], a1[1]); g.stroke();
  if (lageGL) { modellZeichnen(K, R, B, H, dpr); return; }
  // Ersatz ohne WebGL/Datei. Flächen: drehen, Rückseiten weg, von hinten nach vorn zeichnen, Lambert-Schattierung
  const flaechen = [];
  for (const f of MODELL) {
    const p = f.p.map(welt), n = mal(R, f.n);
    if (skal([p[0][0] - K.pos[0], p[0][1] - K.pos[1], p[0][2] - K.pos[2]], n) >= 0) continue;
    const q = p.map(K.proj);
    flaechen.push({q, z: q.reduce((s, v) => s + v[2], 0) / q.length, s: 0.42 + 0.58 * Math.max(0, skal(n, LICHT)), farbe: f.farbe});
  }
  flaechen.sort((a, b) => b.z - a.z);
  g.lineJoin = 'round'; g.lineWidth = 0.6;
  for (const f of flaechen) {
    g.fillStyle = farbe(f.farbe, f.s); g.strokeStyle = farbe(f.farbe, f.s * 0.8);
    g.beginPath(); f.q.forEach((v, k) => k ? g.lineTo(v[0], v[1]) : g.moveTo(v[0], v[1])); g.closePath(); g.fill(); g.stroke();
  }
}
let lageOff = null, lageRaf = 0, lageN = 0;
function lageSchleife() {
  lageRaf = 0;
  if (!$('lagekarte').open || document.hidden) return;
  if (!lageGLVersucht) modellLaden();
  const t = lageOff == null ? null : performance.now() - lageOff - LAGE_VERZUG;
  const w = t == null ? null : lageBei(t);
  radJetzt = t == null ? null : radBei(t);
  lageZeichnen(w);
  if (w && lageN++ % 10 === 0) {
    const gier = ((w[2] + 180) % 360 + 360) % 360 - 180;
    $('l-lage').innerHTML = [[tr('Roll'), w[0], 1], [tr('Nick'), w[1], 1], [tr('Gier'), gier, 0]]
      .map(([n, v, nk]) => '<span>' + n + ' <b>' + (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(nk) + '°</b></span>').join('');
  }
  lageRaf = requestAnimationFrame(lageSchleife);
}
function lageStarten() { if (!lageRaf) lageRaf = requestAnimationFrame(lageSchleife); }
// Ansicht drehen: ziehen (Maus/Touch), Doppelklick = Standard; gemerkt im localStorage
(() => {
  const cv = $('d-lage'), merken = () => { try { localStorage.setItem('xgonav.lage3d', JSON.stringify(lageSicht.map(v => Math.round(v)))); } catch (e) {} };
  let start = null;
  keinSystemDrag(cv);
  cv.addEventListener('pointerdown', e => { start = [e.clientX, e.clientY, ...lageSicht]; cv.setPointerCapture(e.pointerId); cv.classList.add('zieht'); });
  cv.addEventListener('pointermove', e => {
    if (!start) return;
    lageSicht = [((start[2] - (e.clientX - start[0]) * 0.5) % 360 + 360) % 360,
                 Math.max(5, Math.min(85, start[3] + (e.clientY - start[1]) * 0.4))];
  });
  const ende = () => { if (start) { start = null; cv.classList.remove('zieht'); merken(); } };
  cv.addEventListener('pointerup', ende); cv.addEventListener('pointercancel', ende);
  cv.addEventListener('dblclick', () => { lageSicht = LAGE_SICHT.slice(); merken(); });
})();
document.addEventListener('visibilitychange', lageStarten);

function akkuLesen() {
  try { return JSON.parse(localStorage.getItem(AKKU_KEY)) || []; } catch (e) { return []; }
}
// Höchstens ein Punkt je 30 s; jedes Mal frisch lesen, damit mehrere Tabs sich nicht überschreiben
function akkuMerken(pct) {
  if (pct == null) return;
  const jetzt = Date.now(), a = akkuLesen().filter(p => p[0] > jetzt - AKKU_MS);
  if (a.length && jetzt - a[a.length - 1][0] < AKKU_TAKT_MS) return;
  a.push([jetzt, pct]);
  try { localStorage.setItem(AKKU_KEY, JSON.stringify(a)); } catch (e) { /* voll oder gesperrt */ }
}
function akkuLeeren() { localStorage.removeItem(AKKU_KEY); verlaufZeichnen(); }

function abschneiden(arr, grenze) {
  let i = 0;
  while (i < arr.length && arr[i][0] < grenze) i++;
  if (i) arr.splice(0, i);
}
// "schöne" Schrittweite für etwa n Teilstriche
function schritt(spanne, n) {
  const roh = spanne / n, p = Math.pow(10, Math.floor(Math.log10(roh))), f = roh / p;
  return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p;
}
// Erster Punkt mit t >= x (binäre Suche)
function suche(pkt, x) {
  let lo = 0, hi = pkt.length;
  while (lo < hi) { const m = (lo + hi) >> 1; if (pkt[m][0] < x) lo = m + 1; else hi = m; }
  return lo;
}
// Liniendiagramm. serien: [{name, farbe, pkt: [[t, y], …]}]; o: von, bis, ymin, ymax, minSpanne,
// einheit, luecke (ms, größere Abstände unterbrechen die Linie), xText(t), xs (Abstand der Zeitstriche,
// ms; ab bis rückwärts, wenn anker gesetzt, sonst auf Vielfache), nk (Nachkommastellen)
function linien(cv, serien, o) {
  const dpr = window.devicePixelRatio || 1, B = cv.clientWidth, H = Number(cv.dataset.h);
  if (!B) return;
  if (cv.width !== Math.round(B * dpr) || cv.height !== Math.round(H * dpr)) {
    cv.width = Math.round(B * dpr); cv.height = Math.round(H * dpr); cv.style.height = H + 'px';
  }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, B, H);
  const L = 38, R = 8, O = 6, U = 18, w = B - L - R, h = H - O - U;
  // y-Bereich: fest oder aus den sichtbaren Werten, mindestens minSpanne
  let lo = o.ymin ?? Infinity, hi = o.ymax ?? -Infinity;
  if (o.ymin == null || o.ymax == null)
    for (const s of serien) for (let i = suche(s.pkt, o.von); i < s.pkt.length; i++) {
      const y = s.pkt[i][1];
      if (o.ymin == null && y < lo) lo = y;
      if (o.ymax == null && y > hi) hi = y;
    }
  if (!isFinite(lo) || !isFinite(hi)) { lo = 0; hi = 1; }
  const fehlt = (o.minSpanne || 0) - (hi - lo);
  if (fehlt > 0) { if (o.ymin == null) lo -= fehlt / 2; if (o.ymax == null) hi += fehlt / 2; }
  const st = schritt(hi - lo || 1, h < 80 ? 2 : 3);
  if (o.ymin == null) lo = Math.floor(lo / st) * st;
  if (o.ymax == null) hi = Math.ceil(hi / st) * st;
  const X = t => L + (t - o.von) / (o.bis - o.von) * w, Y = y => O + (hi - y) / (hi - lo || 1) * h;
  const css = getComputedStyle(document.body);
  g.font = '11px system-ui,sans-serif';
  g.lineWidth = 1;
  // Raster und Achsen zurückhaltend
  g.strokeStyle = '#2b3038'; g.fillStyle = '#8b94a1'; g.textAlign = 'right'; g.textBaseline = 'middle';
  for (let y = lo; y <= hi + st / 1e6; y += st) {
    const py = Math.round(Y(y)) + .5;
    g.beginPath(); g.moveTo(L, py); g.lineTo(L + w, py); g.stroke();
    g.fillText((+y.toFixed(6)) + (o.einheit || ''), L - 4, py);
  }
  const xs = o.xs, t0 = o.anker ? o.bis - Math.floor((o.bis - o.von) / xs) * xs : Math.ceil(o.von / xs) * xs;
  g.textAlign = 'center'; g.textBaseline = 'top';
  for (let t = t0; t <= o.bis; t += xs) {
    const px = Math.round(X(t)) + .5;
    g.beginPath(); g.moveTo(px, O); g.lineTo(px, O + h); g.stroke();
    g.fillText(o.xText(t), Math.min(Math.max(px, L + 14), L + w - 14), O + h + 4);
  }
  // Linien, 2 px, an Lücken unterbrochen
  g.save(); g.beginPath(); g.rect(L, O, w, h); g.clip();
  g.lineWidth = 2; g.lineJoin = 'round'; g.lineCap = 'round';
  for (const s of serien) {
    g.strokeStyle = s.farbe; g.beginPath();
    let tv = null;
    for (let i = Math.max(0, suche(s.pkt, o.von) - 1); i < s.pkt.length; i++) {
      const [t, y] = s.pkt[i];
      if (tv == null || t - tv > o.luecke) g.moveTo(X(t), Y(y)); else g.lineTo(X(t), Y(y));
      tv = t;
    }
    g.stroke();
  }
  g.restore();
  // Fadenkreuz mit Werten am nächsten Punkt
  if (cv._hx != null && serien.some(s => s.pkt.length)) {
    const t = o.von + (cv._hx * B - L) / w * (o.bis - o.von);
    const zeilen = [o.xText(t)];
    for (const s of serien) {
      const i = Math.min(suche(s.pkt, t), s.pkt.length - 1);
      const p = i > 0 && Math.abs(s.pkt[i - 1][0] - t) < Math.abs(s.pkt[i][0] - t) ? s.pkt[i - 1] : s.pkt[i];
      if (p && Math.abs(p[0] - t) <= o.luecke) zeilen.push([s, p[1].toFixed(o.nk ?? 1) + (o.einheit || '')]);
    }
    const px = Math.round(Math.min(Math.max(X(t), L), L + w)) + .5;
    g.strokeStyle = '#8b94a1'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(px, O); g.lineTo(px, O + h); g.stroke();
    const bw = 118, bh = 8 + zeilen.length * 15, bx = px + 8 + bw > B ? px - 8 - bw : px + 8;
    g.fillStyle = 'rgba(15,17,20,.92)'; g.strokeStyle = '#3a414c';
    g.beginPath(); g.roundRect(bx, O + 2, bw, bh, 5); g.fill(); g.stroke();
    g.textAlign = 'left'; g.textBaseline = 'top';
    zeilen.forEach((z, k) => {
      const ty = O + 6 + k * 15;
      if (typeof z === 'string') { g.fillStyle = '#8b94a1'; g.fillText(z, bx + 7, ty); return; }
      g.fillStyle = z[0].farbe; g.fillRect(bx + 7, ty + 5, 10, 3);
      g.fillStyle = css.getPropertyValue('--text') || '#e3e6ea';
      g.fillText(z[0].name + '  ' + z[1], bx + 22, ty);
    });
  }
}
// Legende mit Farbe und letztem Wert: Identität nie nur über die Farbe
function legende(id, serien, einheit, nk = 1) {
  $(id).innerHTML = serien.map(s => {
    const p = s.pkt[s.pkt.length - 1];
    return '<span><i style="background:' + s.farbe + '"></i>' + esc(s.name) + ' <b>' +
           (p ? p[1].toFixed(nk) + einheit : '-') + '</b></span>';
  }).join('');
}
const uhr = t => new Date(t).toLocaleTimeString(LOCALE(), {hour: '2-digit', minute: '2-digit'});
function verlaufZeichnen() {
  if (!$('verlauf').open) return;
  // Lage: eigene Schleife (lageSchleife). CPU: Browserzeit
  const jetzt = Date.now(), n = cpu.length ? cpu[cpu.length - 1][1].length : 0;
  const cs = Array.from({length: n}, (_, k) =>
    ({name: tr('Kern ') + k, farbe: FARBEN[k % FARBEN.length], pkt: cpu.map(p => [p[0], p[1][k]])}));
  legende('l-cpu', cs, ' %', 0);
  linien($('d-cpu'), cs, {von: jetzt - CPU_MS, bis: jetzt, ymin: 0, ymax: 100, einheit: '%', luecke: 5e3,
                          xText: uhr, xs: 120e3, nk: 0});
  const ws = [{name: tr('↓ empfangen'), farbe: FARBEN[0], pkt: wlan.map(p => [p[0], p[1]])},
              {name: tr('↑ gesendet'), farbe: FARBEN[1], pkt: wlan.map(p => [p[0], p[2]])}];
  legende('l-wlan', ws, ' kB/s', 0);
  linien($('d-wlan'), ws, {von: jetzt - CPU_MS, bis: jetzt, ymin: 0, einheit: '', luecke: 5e3, minSpanne: 10,
                           xText: uhr, xs: 120e3, nk: 0});
  const wl = wlan.length ? wlan[wlan.length - 1] : null;
  $('v-wlan').textContent = wl ? (wl[3] != null ? wl[3] + ' dBm' : '') + (wl[4] != null ? tr(' · Qualität ') + wl[4] + '/70' : '') : '';
  linien($('d-pegel'), [{name: tr('Pegel'), farbe: FARBEN[2], pkt: wlan.filter(p => p[3] != null).map(p => [p[0], p[3]])}],
         {von: jetzt - CPU_MS, bis: jetzt, ymin: -90, ymax: -30, einheit: '', luecke: 5e3, xText: uhr, xs: 120e3, nk: 0});
  const a = akkuLesen().filter(p => p[0] > jetzt - AKKU_MS);
  $('v-akku').textContent = a.length ? a[a.length - 1][1] + ' %' : '';
  linien($('d-akku'), [{name: tr('Akku'), farbe: FARBEN[0], pkt: a}],
         {von: jetzt - AKKU_MS, bis: jetzt, ymin: 0, ymax: 100, einheit: '%', luecke: 5 * 60e3, xText: uhr,
          xs: 30 * 60e3, nk: 0});
}
// Offen/zu merken, damit der Verlauf nach dem Neuladen gleich weiterläuft
function verlaufUmschalten() {
  try { localStorage.setItem('xgonav.verlauf', $('verlauf').open ? '1' : '0'); } catch (e) {}
  if ($('verlauf').open) verlaufZeichnen();
}
function extrasUmschalten() {
  try { localStorage.setItem('xgonav.extras', $('extras').open ? '1' : '0'); } catch (e) {}
}
// Schallradar: Strahlen aus status.ortung.verlauf ([Alter s, Winkel °, Güte, Pegel dB]); gezeichnet wird nur im
// Browser und nur, solange die Karte offen ist. Zeiten werden beim Eintreffen auf die Browseruhr umgerechnet.
// Mitdrehen: je Peilung die IMU-Gier beim ersten Sehen merken (der Lage-Puffer hält nur LAGE_MS), beim Zeichnen
// um die Drehung seitdem ausgleichen; ohne IMU-Proben bleibt alles relativ zum Rider wie gemessen.
const RADAR_MS = 12000;
let radarStrahlen = [], radarAktiv = false, radarLaeuft = false, radarGierZuletzt = null;
function radarGier(t) {                                  // Gier ° (+ = links, fortlaufend) zur Browserzeit t
  if (lageOff == null || !lage.length) return null;
  const w = lageBei(t - lageOff);
  return w ? w[2] : null;
}
function radarWinkel(s, gJetzt) {                        // Peilung relativ zum Rider jetzt, ° (+ = links)
  return s.gier != null && gJetzt != null ? s.w + s.gier - gJetzt : s.w;
}
function radarStatus(o) {
  radarAktiv = !!(o && o.aktiv);
  const jetzt = performance.now();
  if (o && o.verlauf) {
    const alt = radarStrahlen;
    radarStrahlen = o.verlauf.map(v => {
      const s = {t: jetzt - v[0] * 1000, w: v[1], g: v[2], db: v[3]};
      const vor = alt.find(a => Math.abs(a.t - s.t) < 300 && a.w === s.w);
      s.gier = vor ? vor.gier : radarGier(s.t);
      return s;
    });
  }
  $('radar-zustand').textContent = !o ? '-' : o.fehler ? o.fehler : tr(o.aktiv ? 'lauscht' : 'aus');
  radarLegende(jetzt);
  radarStarten();
}
function radarLegende(jetzt) {
  const l = radarStrahlen.length ? radarStrahlen[radarStrahlen.length - 1] : null;
  const w = l ? ((radarWinkel(l, radarGier(jetzt)) + 180) % 360 + 360) % 360 - 180 : 0;
  $('l-radar').textContent = !l ? tr('noch kein Geräusch') :
    (Math.abs(w) < 3 ? tr('Mitte') : Math.abs(w).toFixed(0) + '° ' + tr(w > 0 ? 'links' : 'rechts')) +
    ' · ' + tr('Güte') + ' ' + l.g.toFixed(2) + ' · +' + l.db.toFixed(0) + ' dB · ' +
    tr('vor {s} s', {s: Math.max(0, (jetzt - l.t) / 1000).toFixed(0)});
}
function radarZeichnen(jetzt) {
  const cv = $('d-radar'), dpr = window.devicePixelRatio || 1, B = cv.clientWidth;
  if (!B) return;
  if (cv.width !== Math.round(B * dpr) || cv.height !== cv.width) {
    cv.width = cv.height = Math.round(B * dpr); cv.style.height = B + 'px';
  }
  const g = cv.getContext('2d'), cx = B / 2, cy = B / 2, R = B / 2 - 16;
  const css = getComputedStyle(document.documentElement), farbe = n => css.getPropertyValue(n).trim();
  const gruen = farbe('--gut'), rand = farbe('--rand2'), leise = farbe('--leise');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, B, B);
  g.fillStyle = farbe('--karte2');
  g.beginPath(); g.arc(cx, cy, R, 0, 2 * Math.PI); g.fill();
  g.strokeStyle = rand; g.lineWidth = 1;
  for (const f of [1 / 3, 2 / 3, 1]) { g.beginPath(); g.arc(cx, cy, R * f, 0, 2 * Math.PI); g.stroke(); }
  const gJetzt = radarGier(jetzt), dreh = gJetzt != null ? gJetzt * Math.PI / 180 : 0;
  radarGierZuletzt = gJetzt;
  for (let k = 0; k < 12; k++) {                         // Speichen alle 30°, fest in der Welt; Achsen kräftiger
    const a = k * Math.PI / 6 + dreh;
    g.globalAlpha = k % 3 ? 0.35 : 1;
    g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx + R * Math.sin(a), cy - R * Math.cos(a)); g.stroke();
  }
  g.globalAlpha = 1;
  // Strahlen: Winkel > 0 = links; Zwilling nach hinten gestrichelt (vorn/hinten nicht unterscheidbar)
  for (const s of radarStrahlen) {
    const alter = jetzt - s.t;
    if (alter > RADAR_MS) continue;
    const blass = (1 - alter / RADAR_MS) * Math.min(1, Math.max(0.35, s.g));
    const len = R * Math.min(1, Math.max(0.35, (s.db - 3) / 22)), w = radarWinkel(s, gJetzt) * Math.PI / 180;
    for (const hinten of [false, true]) {
      const a = hinten ? Math.PI + w : -w, x = cx + len * Math.sin(a), y = cy - len * Math.cos(a);
      const v = g.createLinearGradient(cx, cy, x, y);
      v.addColorStop(0, 'transparent'); v.addColorStop(1, gruen);
      g.globalAlpha = blass * (hinten ? 0.45 : 1);
      g.strokeStyle = v; g.lineWidth = hinten ? 2 : 3; g.lineCap = 'round';
      g.setLineDash(hinten ? [5, 6] : []);
      g.beginPath(); g.moveTo(cx, cy); g.lineTo(x, y); g.stroke();
      g.setLineDash([]);
      g.fillStyle = gruen;
      g.beginPath(); g.arc(x, y, hinten ? 3 : 4.5, 0, 2 * Math.PI); g.fill();
      if (!hinten && alter < 600) {                      // frisches Geräusch: Ring, der sich ausbreitet
        g.globalAlpha = blass * (1 - alter / 600);
        g.strokeStyle = gruen; g.lineWidth = 1.5;
        g.beginPath(); g.arc(x, y, 5 + 16 * alter / 600, 0, 2 * Math.PI); g.stroke();
      }
    }
  }
  g.globalAlpha = 1;
  g.fillStyle = farbe('--text');                         // Rider in der Mitte, Spitze nach vorn
  g.beginPath(); g.moveTo(cx, cy - 9); g.lineTo(cx + 6, cy + 6); g.lineTo(cx - 6, cy + 6); g.closePath(); g.fill();
  g.fillStyle = leise; g.font = '11px system-ui, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(tr('vorn'), cx, cy - R - 8); g.fillText(tr('hinten'), cx, cy + R + 8);
  g.fillText('L', cx - R - 8, cy); g.fillText('R', cx + R + 8, cy);
}
// Schleife nur bei offener Karte und nur, solange ein Strahl verblasst oder der Rider sich dreht
function radarSchleife(jetzt) {
  radarLaeuft = false;
  if (!$('radarkarte').open || document.hidden) return;
  const vorher = radarGierZuletzt;
  radarZeichnen(jetzt);
  const dreht = radarGierZuletzt != null && (vorher == null || Math.abs(radarGierZuletzt - vorher) > 0.2);
  if (dreht) radarLegende(jetzt);
  if (dreht || radarStrahlen.some(s => jetzt - s.t < RADAR_MS)) radarStarten();
}
function radarStarten() {
  if (radarLaeuft || !$('radarkarte').open) return;
  radarLaeuft = true;
  requestAnimationFrame(radarSchleife);
}
function radarUmschalten() {
  try { localStorage.setItem('xgonav.radar', $('radarkarte').open ? '1' : '0'); } catch (e) {}
  liveImu();                                             // Gier zum Mitdrehen
  radarStarten();
}
document.addEventListener('visibilitychange', radarStarten);
window.addEventListener('resize', radarStarten);
function raumkarteUmschalten() {
  try { localStorage.setItem('xgonav.raumkarte', $('raumkarte').open ? '1' : '0'); } catch (e) {}
  if ($('raumkarte').open) karteAnstoss(); else clearTimeout(karteTimer);
}
document.addEventListener('visibilitychange', () => { if (!document.hidden) karteAnstoss(); });
function lageUmschalten() {
  try { localStorage.setItem('xgonav.lage', $('lagekarte').open ? '1' : '0'); } catch (e) {}
  liveImu();
  if ($('lagekarte').open) lageStarten();
}
// Regler zeigen den Sollwert aus dem Treiber (auch wenn xgonav oder ein anderer Tab ihn setzt).
// Nicht während der Nutzer zieht und kurz danach: Die Antwort kann noch den alten Wert enthalten.
function reglerSetzen(id, wert, text) {
  const el = $(id);
  if (el._zieht || (el._zeit != null && performance.now() - el._zeit < 1000)) return;
  el.value = wert;
  $(id + '-w').textContent = text;
}
for (const id of ['hoehe', 'roll']) {
  const el = $(id), los = () => { el._zieht = false; el._zeit = performance.now(); };
  el.addEventListener('pointerdown', () => { el._zieht = true; });
  el.addEventListener('input', () => { el._zeit = performance.now(); });
  window.addEventListener('pointerup', () => { if (el._zieht) los(); });
  window.addEventListener('pointercancel', () => { if (el._zieht) los(); });
}
// IMU + Räder aus dem Live-Paket; der Server liefert je Verbindung nur neue Proben
function imuVerarbeiten(s) {
  if (s.seq < imuSeq) { lage.length = 0; imuSeq = 0; lageOff = null; }  // Treiber neu geladen
  for (const p of s.imu) { if (p[0] > imuSeq) { lage.push([p[1], p[2], p[3], p[4]]); imuSeq = p[0]; } }
  if (s.wseq != null) {
    if (s.wseq < radSeq) { radPuffer.length = 0; radSeq = 0; radRoh = null; }   // Treiber neu geladen
    const d32 = d => ((d + 2 ** 31) % 2 ** 32 + 2 ** 32) % 2 ** 32 - 2 ** 31;  // wie _wrap32 in wheels.py
    for (const p of s.wheels || []) {
      if (p[0] <= radSeq) continue;
      const v = radPuffer.length && radRoh ? [radPuffer[radPuffer.length - 1][1] + d32(p[2] - radRoh[0]), radPuffer[radPuffer.length - 1][2] + d32(p[3] - radRoh[1])] : [p[2], p[3]];
      radPuffer.push([p[1], ...v]); radRoh = [p[2], p[3]]; radSeq = p[0];
    }
    if (radPuffer.length) abschneiden(radPuffer, radPuffer[radPuffer.length - 1][0] - LAGE_MS);
  }
  if (lage.length) {
    abschneiden(lage, lage[lage.length - 1][0] - LAGE_MS);
    // Treiberzeit -> Browserzeit: kleinster Abstand (frischeste Probe), langsam nachführen
    const off = performance.now() - lage[lage.length - 1][0];
    lageOff = lageOff == null || off < lageOff || Math.abs(off - lageOff) > 2000 ? off : lageOff + (off - lageOff) * 0.02;
  }
  if (s.state != null) beinSoll.abgesetzt = s.state === 254;
  radarStarten();                                        // Drehung: Radar dreht mit
}
// CPU, Temperatur, Akku, WLAN, Beine: 1×/s aus dem Live-Paket (auch im Hintergrund, Verlauf bleibt lückenlos)
function sysVerarbeiten(s) {
  if (s.height != null) { beinSoll.h = s.height; if (!hoeheAusStatus) hoeheSetzen(s.height); }
  if (s.roll != null) { beinSoll.r = s.roll; reglerSetzen('roll', s.roll, s.roll + '°'); }
  beinSoll.abgesetzt = s.state === 254;
  if (s.state != null) zustandPille(s.state, s.state === 1);
  const jetzt = Date.now();
  // Auslastung auf den Höchsttakt bezogen: 100 % bei halbem Takt zählt als 50 %
  const takt = s.mhz && s.mhz[1] > 0 ? Math.min(1, s.mhz[0] / s.mhz[1]) : 1;
  if (cpuVorher && cpuVorher.length === s.cpu.length)
    cpu.push([jetzt, s.cpu.map((c, k) => {
      const dt = c[0] - cpuVorher[k][0];
      return dt > 0 ? takt * Math.max(0, Math.min(100, 100 * (1 - (c[1] - cpuVorher[k][1]) / dt))) : 0;
    })]);
  cpuVorher = s.cpu; cpuZeit = jetzt;
  abschneiden(cpu, jetzt - CPU_MS);
  $('v-temp').textContent = (s.temp_c != null ? s.temp_c.toFixed(1) + ' °C' : '') +
                            (s.mhz ? ' · ' + s.mhz[0] + '/' + s.mhz[1] + ' MHz' : '') +
                            (s.load != null ? tr(' · Last ') + s.load.toFixed(2) : '');
  akkuMerken(s.battery);
  wlanMerken(s.wlan);
  verlaufZeichnen();
}
// Fadenkreuz: Maus und Touch
document.querySelectorAll('.diag').forEach(cv => {
  cv.addEventListener('pointermove', e => { cv._hx = e.offsetX / cv.clientWidth; verlaufZeichnen(); });
  cv.addEventListener('pointerleave', () => { cv._hx = null; verlaufZeichnen(); });
});
window.addEventListener('resize', verlaufZeichnen);
spracheAnwenden();
status();
try {
  if (localStorage.getItem('xgonav.raumkarte') !== '0') $('raumkarte').open = true;   // Standard: offen
  if (localStorage.getItem('xgonav.verlauf') === '1') $('verlauf').open = true;
  if (localStorage.getItem('xgonav.lage') !== '0') $('lagekarte').open = true;   // Standard: offen
  if (localStorage.getItem('xgonav.extras') === '1') $('extras').open = true;
  if (localStorage.getItem('xgonav.radar') !== '0') $('radarkarte').open = true;   // Standard: offen
} catch (e) { $('lagekarte').open = true; $('raumkarte').open = true; }
liveVerbinden();
lageStarten();
</script>
</body>
</html>
