<?php
/*
 * xgonav – Bedienoberfläche. Bild: WebSocket ws?nick=1 (nginx -> xgonav /ws, Bestätigung je Bild),
 * Raumkarte: map.png, Status und Befehle: api.php. Enthält die Funktionen der früheren
 * Fernsteuerung (rc_server): Handsteuerung, Höhe, Aktionen, LEDs, Emotionen, Herunterfahren.
 * Keine Authentifizierung.
 *
 * Aufbau: Kopfleiste (immer sichtbar, mit STOPP) – links Bild, Modus, Kennzahlen, Kamera,
 * Verlauf (eingeklappt) – rechts Karten (Fahren, Raumkarte, Extras eingeklappt, System).
 */
declare(strict_types=1);
header('Cache-Control: no-store');

const RC = '/home/aeinstein/motorcortex/rc_server';
$host = htmlspecialchars(gethostname() ?: 'rider', ENT_QUOTES);
$emotionen = [];
foreach (@scandir(RC . '/emotions') ?: [] as $e) {
    if (preg_match('/^[A-Za-z]+$/', $e) && is_dir(RC . '/emotions/' . $e)) {
        $emotionen[] = $e;
    }
}
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

/* --- Kopfleiste: bleibt oben, damit STOPP ohne Scrollen erreichbar ist */
header{position:sticky;top:0;z-index:10;display:flex;align-items:center;gap:10px;flex-wrap:wrap;
  padding:8px 12px;background:rgba(15,17,20,.92);backdrop-filter:blur(6px);border-bottom:1px solid var(--rand)}
header h1{font-size:15px;font-weight:600;margin:0 4px 0 0;white-space:nowrap}
header h1 span{color:var(--leise);font-weight:400}
@media (max-width:640px){
  header{gap:6px 8px;padding:6px 8px}
  header h1{flex:1}
  header .pillen{order:3;flex:0 0 100%;gap:4px}
  header .pille{padding:1px 7px}
  .pille .lang{display:none}
}
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
button{font:inherit;font-size:14px;padding:6px 11px;border-radius:6px;border:1px solid var(--rand2);background:var(--karte2);
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
.gefahr{border-color:#6b2a2d;color:#ffb3b5}
.knoepfe{display:flex;flex-wrap:wrap;gap:6px}
input,select{font:inherit;color:var(--text);background:var(--karte2);border:1px solid var(--rand2);border-radius:6px;padding:5px 8px}
input[type=range]{padding:0;border:0;background:none;accent-color:var(--akzent);width:100%;cursor:pointer;min-width:80px}
input[type=color]{width:100%;height:34px;padding:2px;cursor:pointer}
select{cursor:pointer;padding:3px 6px}

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
#bild{width:100%;height:auto;aspect-ratio:4/3;object-fit:contain;display:block;background:#000}
.modus{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.segment{display:inline-flex;border:1px solid var(--rand2);border-radius:7px;overflow:hidden;flex-wrap:wrap}
.segment button{border:0;border-radius:0;border-right:1px solid var(--rand2);margin:0}
.segment button:last-child{border-right:0}
.segment button.aktiv{background:var(--akzent);color:#04121f;font-weight:600}
#ziel{width:96px}
@media (max-width:640px){
  .segment{display:grid;grid-template-columns:repeat(3,1fr);width:100%}
  .segment button{padding:7px 2px;font-size:13px}
  #ziel{flex:1}
}

/* --- Kennzahlen */
.kacheln{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:6px}
.kachel{background:var(--karte2);border-radius:6px;padding:5px 8px;min-width:0}
.kachel .k{font-size:11px;color:var(--leise);text-transform:uppercase;letter-spacing:.03em}
.kachel .w{font-variant-numeric:tabular-nums;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.kachel.breit{grid-column:1/-1}
.kachel.breit .w{white-space:normal}
details.roh summary{cursor:pointer;color:var(--leise);font-size:12px;margin-top:8px}
pre{background:#0b0d10;border-radius:6px;padding:8px;overflow:auto;max-height:40vh;font-size:12px;margin:6px 0 0}

/* --- Fahren */
.fahren{display:grid;grid-template-columns:auto 1fr;gap:14px;align-items:stretch}
@media (max-width:420px){.fahren{grid-template-columns:1fr}.pad{justify-self:center}}
.fahren-rechts{display:flex;flex-direction:column;gap:10px;justify-content:space-between;min-width:0}
.aktionen{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.aktionen button{margin:0;padding:8px 6px;min-height:40px}
.pad{display:grid;grid-template-columns:repeat(3,56px);grid-template-rows:repeat(3,52px);gap:5px;user-select:none;-webkit-user-select:none}
.pad button{margin:0;padding:0;font-size:20px;border-radius:8px}
@media (pointer:coarse){.pad{grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,58px)}}
.regler{display:grid;grid-template-columns:auto 1fr 3.6em auto;gap:6px 8px;align-items:center}
.regler label{color:var(--leise);font-size:13px}
.regler .w{font-variant-numeric:tabular-nums;text-align:right;font-size:13px}
.regler button{padding:3px 8px;font-size:12px}
.regler .leer{width:0}
.trenner{border:0;border-top:1px solid var(--rand);margin:10px 0}

/* --- Raumkarte */
#karte-rahmen{position:relative;max-width:400px;aspect-ratio:1;margin:0 auto;overflow:hidden;background:#808080;border-radius:6px;cursor:grab;user-select:none;-webkit-user-select:none}
#karte-rahmen.zieht{cursor:grabbing}
#karte-rahmen.setzt{cursor:crosshair}
#karte-punkte{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}
.route-leiste{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-top:6px}
#karte{display:block;width:100%;height:100%;pointer-events:none;user-select:none;-webkit-user-select:none}

/* --- Extras */
details.extras>summary{cursor:pointer;list-style:none;display:flex;align-items:center;gap:8px;user-select:none}
details.extras>summary::-webkit-details-marker{display:none}
details.extras>summary h2{font-size:13px;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:#a9b4c2;margin:0;flex:1}
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
.unter button{padding:1px 7px;font-size:12px;margin-left:6px}
.lage-flaeche{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:8px}
.kamhoehe{display:flex;flex-direction:column;align-items:center;gap:4px;height:230px;font-size:12px;color:var(--leise)}
.kamhoehe .w{color:var(--text);font-variant-numeric:tabular-nums;white-space:nowrap}
/* senkrecht, oben = hoch; touch-action: Ziehen verschiebt nicht die Seite */
#hoehe{writing-mode:vertical-lr;direction:rtl;flex:1 1 0;min-height:0;min-width:0;width:32px;touch-action:none}
#lagekarte>.regler{margin-top:8px}
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
    <span class="pille" id="p-pip"><span class="lang">Bildverarb. </span><b>-</b></span>
    <span class="pille" id="p-wlan" title="WLAN-Empfang (Pegel) und Durchsatz des Riders: ↓ empfangen, ↑ gesendet"><span class="lang">WLAN </span><b>-</b></span>
  </div>
  <button id="stop" onclick="befehl('stop')" title="Stillstand, Modus idle">STOPP</button>
</header>
<div id="meldung"></div>

<main>
<div class="spalte">
  <div id="bild-rahmen"><img id="bild" alt="" aria-label="Kamerabild" width="640" height="480"></div>
  <div class="karte">
    <div class="modus">
      <div class="segment">
        <button data-modus="idle" onclick="modus('idle')">Idle</button>
        <button data-modus="manual" onclick="modus('manual')">Hand</button>
        <button data-modus="follow" onclick="modus('follow')">Folgen</button>
        <button data-modus="explore" onclick="modus('explore')">Erkunden</button>
        <button data-modus="approach" onclick="modus('approach')">Anfahren</button>
        <button data-modus="home" onclick="modus('home')" title="Weg über die Raumkarte zum Startpunkt">Zum Start</button>
        <button data-modus="patrol" onclick="streife()" title="Route auf der Raumkarte abfahren">Streife</button>
      </div>
      <input id="ziel" value="person" title="Ziel für Folgen/Anfahren (COCO-Klasse)" aria-label="Ziel">
    </div>
  </div>
  <div class="karte">
    <div class="kopf"><h2>Bildverarbeitung</h2><button id="pip" onclick="pipelineUmschalten()">…</button></div>
    <div class="kacheln" id="kurz"></div>
    <details class="roh"><summary>Status (roh)</summary><pre id="roh"></pre></details>
  </div>
  <section class="karte">
    <div class="kopf"><h2>Kamera</h2><span class="leise" id="belicht-ist">-</span></div>
    <div class="regler">
      <label for="belicht">Belichtung</label>
      <input type="range" id="belicht" min="1" max="33" value="15" step="1"
        oninput="belichtAnzeige(this.value * 1000)" onchange="belichtung(this.value * 1000)">
      <span class="w" id="belicht-w">-</span>
      <button id="belicht-auto" onclick="belichtung(0)">Auto</button>
      <label for="gain">Verstärkung</label>
      <input type="range" id="gain" min="1" max="63.9" value="8" step="0.5"
        oninput="gainAnzeige(this.value)" onchange="verstaerkung(this.value)">
      <span class="w" id="gain-w">-</span>
      <button id="gain-auto" onclick="verstaerkung(0)">Auto</button>
    </div>
    <details class="hilfe"><summary>Gilt bis zum Neustart von xgonav</summary>
      <p>Kürzere Belichtung = weniger Bewegungsunschärfe, aber dunkler; mehr Verstärkung = heller, aber mehr
        Rauschen (Automatik nur bis 8; über ~8× wird der Rauschabstand nicht mehr besser, nur das Bild heller). Dauerhaft über <code>[camera] exposure_us</code> /
        <code>analogue_gain</code> in config.toml.</p>
    </details>
  </section>
  <details class="karte extras" id="verlauf" ontoggle="verlaufUmschalten()">
    <summary><h2>Verlauf</h2><span class="leise">CPU · WLAN · Akku</span></summary>
    <div class="unter">CPU je Kern, letzte 10 min <span class="rechts" id="v-temp"></span></div>
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
      <span class="w" id="roll-w">0°</span><button onclick="rollNull()" title="Roll auf 0°">0</button>
    </div>
    <div class="legende" id="l-lage" title="Gier 0° = Kurs beim Start des ESP32; + = links. Roll + = rechts tief, Nick + = vorn tief"></div>
    <details class="hilfe"><summary>Kamerahöhe und Roll</summary>
      <p>Kamerahöhe: Rechenwert der Bodengeometrie aus <code>cam_h_u8</code> (Linsenmitte über Boden, Sollwert u8
        im Tooltip), die Beine brauchen danach etwa 1 s. Enthalten ist die Korrektur ×0,921 aus der Bodenprobe; der
        Zollstock zeigt deshalb rund 8 % mehr, solange deren Ursache offen ist. Oberhalb von u8 200 ändert sich nichts
        mehr. Untere Grenze mit Abstand zum Minimum; in Folgen/Erkunden setzt xgonav die Höhe selbst.</p>
      <p>Roll: + = rechts.</p>
    </details>
  </details>
  <section class="karte">
    <div class="kopf"><h2>Fahren</h2></div>
    <div class="fahren">
      <div class="pad">
        <span></span><button data-fahrt="vor" aria-label="vor">▲</button><span></span>
        <button data-fahrt="links" aria-label="links">◀</button><span></span><button data-fahrt="rechts" aria-label="rechts">▶</button>
        <span></span><button data-fahrt="zurück" aria-label="zurück">▼</button><span></span>
      </div>
      <div class="fahren-rechts">
        <div class="regler">
          <label for="tempo">Tempo</label>
          <input type="range" id="tempo" min="5" max="100" value="60">
          <span class="w" id="tempo-w"></span><span class="leer"></span>
        </div>
        <div class="aktionen">
          <button onclick="arm()" title="Aktion 255">Aufstehen</button>
          <button onclick="aufspringen()" title="Aktion 253 (!standup): Rider muss stehen und Platz haben">Aufspringen</button>
          <button onclick="ausrichten()" title="Roll auf 0° und die Beine 3 s nivellieren (nur wenn er steht)">Neu ausrichten</button>
          <button class="gefahr" onclick="befehl('action', {n: 254})" title="Aktion 254, Not-Stopp">Absetzen</button>
        </div>
      </div>
    </div>
    <details class="hilfe"><summary>Gedrückt halten: Maus, Touch, Pfeiltasten/WASD</summary>
      <p>Fahren schaltet auf „Hand“ und ist auf max_v/max_w der Konfiguration begrenzt. Ohne Bodenmodell
        und Bahnprüfung; Sturz-Erkennung und Watchdogs bleiben aktiv.</p>
      <p>Balancieren (Firmware R-1.1.6): Nach einem ESP32-Neustart balanciert der Rider erst nach
        „Aufstehen“. Liegt er, wartet die Firmware, bis er aufrecht steht – dann laufen die Räder sofort an.
        „Aufspringen“ (Aktion 253, wie <code>!standup</code>) lässt ihn aufspringen – <b>Rider muss stehen und Platz haben</b>.
        „Absetzen“ ist ein Not-Stopp (Aktion 254, wie <code>!stop</code>): er setzt sich kontrolliert ab und
        bleibt aus bis zum nächsten Aufstehen.</p>
      <p>„Neu ausrichten“: Die Beine haben eine Totzone, bei gleichem Befehl steht er bis zu 2,6° unterschiedlich
        schief. Der Knopf setzt Roll auf 0° und lässt die Firmware 3 s nivellieren; danach bleibt er so stehen. Das
        passiert auch einmal nach jedem Aufstehen. Beim Fahren kann er langsam wieder schief werden.</p>
    </details>
  </section>

  <section class="karte" id="horizont" hidden>
    <div class="kopf"><h2>Horizont</h2><span class="leise" id="hz-zustand">-</span></div>
    <div class="leise" id="hz-werte">-</div>
    <div class="knoepfe" style="margin-top:6px">
      <button onclick="hzSchieben(-5)" title="Linie 5 px nach oben">−5</button>
      <button onclick="hzSchieben(-1)" title="Linie 1 px nach oben">−1</button>
      <span class="w" id="hz-zeile">-</span>
      <button onclick="hzSchieben(1)" title="Linie 1 px nach unten">+1</button>
      <button onclick="hzSchieben(5)" title="Linie 5 px nach unten">+5</button>
      <button id="hz-speichern" onclick="hzSpeichern()" title="Zeile der grünen Linie für die aktuelle Kamerahöhe speichern">Für diese Kamerahöhe speichern</button>
      <button onclick="hzAus()" title="Grüne Linie ausblenden">Linie aus</button>
      <label title="Bild senkrecht so verschieben, dass die cyan IMU-Linie fest steht"><input type="checkbox" id="hz-stabil" onchange="hzStabil(this.checked)"> Bild am IMU-Nick festhalten</label>
    </div>
    <div class="leise" style="margin-top:6px" id="hz-tabelle">-</div>
    <details class="hilfe"><summary>weiß Stand · cyan IMU · magenta Bild · orange IMU roh · grün Einstellen</summary>
      <p>Erst aufstehen lassen und nicht anfassen: Nach 3 s Einschwingen misst xgonav 3 s lang den mittleren
        Nickwinkel (Standreferenz). Danach die grüne Linie auf einen Bezug in Kamerahöhe schieben (Laserlinie,
        Klebeband an der Wand in Höhe der Kameramitte) und speichern. Das je Kamerahöhe wiederholen; dazwischen
        rechnet xgonav linear.</p>
      <p>Cyan ist der Horizont aus IMU minus Standreferenz. Magenta startet am IMU-Wert, folgt dann dem Bildnick
        (schnelle Änderungen) und wird spätestens nach 0,5 s wieder auf die IMU gesetzt; magenta gilt für die
        Entfernungen. Orange ist der rohe Nick des Treibers (neueste Probe, ohne Zeitabgleich zum Bild), links
        mit dem Rohwert in Grad. Nach jeder Höhenänderung und jedem Absetzen misst er die Standlage neu.</p>
      <p>„Bild am IMU-Nick festhalten“: Das Bild wird je Bild so verschoben, dass die cyan IMU-Linie fest auf der
        Standzeile liegt. Stimmt die IMU, steht die Szene still; wandert sie gegen die cyan Linie, driftet die IMU
        oder der Zeitabgleich. Schwarzer Rand oben/unten = Verschiebung.</p>
    </details>
  </section>

  <section class="karte">
    <div class="kopf"><h2>Raumkarte</h2>
      <select id="karte-view" aria-label="Ausschnitt" onchange="karteView(this.value)"><option value="3">3 m</option><option value="6" selected>6 m</option><option value="10">10 m</option><option value="16">16 m</option><option id="karte-view-frei" hidden></option></select>
      <button onclick="befehl('map_start')" title="Startpunkt = aktuelle Pose">Start hier</button>
      <button onclick="befehl('map_save')" title="Raumkarte jetzt speichern ([grid] file)">Speichern</button>
      <button onclick="befehl('map_reset')">Zurücksetzen</button>
    </div>
    <div id="karte-rahmen" title="ziehen: verschieben · Mausrad: zoomen · Doppelklick: zurück zum Rider">
      <img id="karte" alt="Raumkarte" width="320" height="320" draggable="false">
      <svg id="karte-punkte" viewBox="0 0 1 1" preserveAspectRatio="none"></svg>
    </div>
    <div class="leise" style="margin-top:6px" id="karte-pose">-</div>
    <div class="leise" id="karte-erkunden" hidden></div>
    <div class="leise" id="karte-streife" hidden></div>
    <div class="route-leiste" id="route-start">
      <button onclick="bearbeiten('route')" title="Wegpunkte der Streife per Klick auf die Karte setzen">Route setzen</button>
      <button onclick="bearbeiten('zone')" title="Bereich, den der Rider nie befährt: Ecken anklicken">Sperrzone</button>
      <button onclick="zonenLoeschen()" title="Alle Sperrzonen löschen">Zonen löschen</button>
      <button id="karte-sperre" onclick="karteSperren()" title="Gesperrt ändern Fahrten die Karte nicht">Karte sperren</button>
    </div>
    <div class="route-leiste" id="route-edit" hidden>
      <span id="route-hinweis" class="leise"></span>
      <label id="route-kreis"><input type="checkbox" id="route-loop"> im Kreis</label>
      <button onclick="punktWeg()">Letzten weg</button>
      <button onclick="bearbeitenFertig()" class="aktiv">Übernehmen</button>
      <button onclick="bearbeitenEnde()">Abbrechen</button>
    </div>
    <details class="hilfe"><summary>grau unbekannt · hell frei · dunkel belegt · grün Weg · cyan Plan · blau Start · rot Rider</summary>
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
  </section>

  <details class="karte extras">
    <summary><h2>Extras</h2><span class="leise">Aktionen · LEDs · Emotionen</span></summary>
    <div class="unter">Aktionen</div>
    <div class="knoepfe">
      <?php for ($i = 1; $i <= 6; $i++): ?><button onclick="befehl('action', {n: <?= $i ?>})"><?= $i ?></button><?php endfor; ?>
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
    <div class="unter">Emotionen (Display)</div>
    <div class="knoepfe">
      <?php foreach ($emotionen as $e): ?><button onclick="befehl('emotion', {name: '<?= h($e) ?>'})"><?= h($e) ?></button><?php endforeach; ?>
      <?php if (!$emotionen): ?><span class="leise">keine gefunden in <?= h(RC) ?>/emotions</span><?php endif; ?>
    </div>
  </details>

  <section class="karte">
    <div class="kopf"><h2>System</h2>
      <!-- Kalibrier-Knöpfe gesperrt (2026-11-02): Speichern bei schlaffen Beinen hat die
           Beinnullpunkte im ESP32-Flash überschrieben. Erst mit geführtem Ablauf wieder freigeben. -->
      <button id="aus" class="gefahr" onclick="herunterfahren()">Herunterfahren</button>
    </div>
  </section>
</div>
</main>
<script>
// Kennzahlen-Kacheln; Modus, Bildverarbeitung, Akku und Zustand stehen in der Kopfleiste
const felder = [
  ['Status', s => s.status, true],
  ['Befehl v / w', s => s.cmd ? s.cmd.v.toFixed(2) + ' / ' + s.cmd.w.toFixed(2) : '-'],
  ['Kurs', s => s.yaw_drv != null ? Math.round(s.yaw_drv) + '°' : '-'],
  ['Kamera / Schleife', s => (s.fps ?? '-') + ' fps / ' + (s.loop_ms ?? '-') + ' ms'],
  ['Schleife p95 / max', s => s.loop_p95 != null ? s.loop_p95 + ' / ' + s.loop_max + ' ms' : '-'],
  ['Detektor', s => s.det_ms != null ? Math.round(s.det_ms) + ' ms' : '-'],
  ['Nicken', s => (s.pitch_src ?? '-') + ' ' + (s.pitch_deg != null ? s.pitch_deg.toFixed(1) + '°' : '')],
  ['Gier / Nick', s => s.ego ? s.ego.yaw_dps.toFixed(0) + ' / ' + s.ego.pitch_dps.toFixed(0) + ' °/s' : '-'],
  ['TTC / Schärfe', s => (s.ttc != null ? s.ttc.toFixed(1) + ' s' : '-') + ' / ' + (s.ego ? s.ego.sharp : '-')],
];
const ERKUNDEN = {scan: 'Rundumblick', ziel: 'fährt zu einer Stelle', face: 'schaut', settle: 'schaut', backup: 'setzt zurück',
  wander: 'befreit sich', heim: 'fährt zum Start', fertig: 'fertig'};
const MODI = {idle: 'Idle', manual: 'Hand', follow: 'Folgen', explore: 'Erkunden', approach: 'Anfahren', home: 'Zum Start', patrol: 'Streife'};
const STREIFE = {fahrt: 'fährt', schau: 'schaut', backup: 'setzt zurück', heim: 'fährt zum Start', fertig: 'fertig'};
const esc = t => String(t ?? '-').replace(/[<&]/g, c => c === '<' ? '&lt;' : '&amp;');
const $ = id => document.getElementById(id);
let aktModus = null, aktPipeline = null;
function zeige(text) { $('meldung').textContent = text || ''; }
// Pille der Kopfleiste setzen: Wert und Bewertung (gut / warn / schlecht / neutral)
function pille(id, wert, art = '') {
  const p = $(id);
  p.querySelector('b').textContent = wert;
  p.className = 'pille' + (art ? ' ' + art : '');
}
// Verbindungsfehler verschwinden von selbst, sobald xgonav wieder antwortet (andere Meldungen bleiben)
const VERBINDUNG = /nicht erreichbar|keine Verbindung/;
function verbindungOk() {
  if (VERBINDUNG.test($('meldung').textContent)) zeige('');
  pille('p-verb', 'verbunden', 'gut');
}
function verbindungWeg() { pille('p-verb', 'getrennt', 'schlecht'); }
// Kamerabild über eine WebSocket-Verbindung. Jedes Bild wird sofort bestätigt, erst dann
// schickt xgonav das nächste (neueste): kein Rückstau wie beim MJPEG-Stream über schwaches
// WLAN (bis 20 s). Nachricht (nick=1): 4 Byte Bildnummer, 2 x float32 row_stand/row_imu, JPEG.
// Bei Abbruch nach 1 s neu verbinden.
// „Bild am IMU-Nick festhalten“: Das Bild wird hier im Browser per CSS um row_stand − row_imu verschoben,
// sodass die cyan IMU-Linie fest steht; die Bildpipeline rechnet dafür nichts.
let bildUrl = null, bildDy = 0;
let bildStabil = false;
try { bildStabil = localStorage.getItem('xgonav.stabil') === '1'; } catch (e) {}
function bildVerschieben() {
  const b = $('bild'), H = b.naturalHeight || 480;
  b.style.transform = bildStabil && bildDy ? 'translateY(' + (bildDy / H * 100).toFixed(3) + '%)' : '';
}
function bildVerbinden() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host +
                           location.pathname.replace(/[^/]*$/, '') + 'ws?nick=1');
  ws.binaryType = 'arraybuffer';
  $('bild').onload = bildVerschieben;               // Verschiebung erst mit dem zugehörigen Bild
  ws.onmessage = ev => {
    ws.send('ok');                                    // Bestätigung: nächstes Bild bitte
    const d = new DataView(ev.data), stand = d.getFloat32(4), imu = d.getFloat32(8);
    bildDy = Number.isFinite(stand) && Number.isFinite(imu) ? stand - imu : 0;
    const url = URL.createObjectURL(new Blob([new Uint8Array(ev.data, 12)], {type: 'image/jpeg'}));
    $('bild').src = url;
    if (bildUrl) URL.revokeObjectURL(bildUrl);
    bildUrl = url;
  };
  ws.onclose = () => setTimeout(bildVerbinden, 1000);
}
async function befehl(a, p = {}, still = false) {
  const q = new URLSearchParams({a, ...p});
  try {
    const r = await fetch('api.php?' + q, {method: 'POST'});
    const j = await r.json();
    if (!still || !r.ok) zeige(r.ok ? '' : (j.error || ('Fehler ' + r.status)));
    return r.ok;
  } catch (e) { zeige('keine Verbindung'); verbindungWeg(); return false; }
  finally { if (!still) status(); }
}
function modus(n) { return befehl('mode', {name: n, target: $('ziel').value.trim().toLowerCase()}); }
function streife() {
  if (!confirm('Steht der Rider (aufgestanden) am Startpunkt, in Startrichtung? Die Pose springt dann dorthin. Sind Treppen und Kanten abgesperrt?')) return;
  return befehl('mode', {name: 'patrol', target: 'person', bestaetigt: 1});
}
function pipelineUmschalten() { return befehl('pipeline', {on: aktPipeline === 'läuft' ? 0 : 1}); }
// Horizont im Stand ([horizon] enabled): grüne Linie einstellen und je Körperhöhe speichern
const HZ_ZUSTAND = {liegt: 'liegt – erst aufstehen', wartet: 'schwingt ein', misst: 'misst Standlage', 'gültig': 'Standreferenz gültig'};
let hzZeile = null, hzTabelleT = 0;
const zahlOder = (v, n = 1, e = '') => v == null ? '-' : v.toFixed(n) + e;
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
  el.title = 'Sollwert u8 ' + u;
}
function hoeheSetzen(u) {
  reglerSetzen('hoehe', u, kamText(u));
  $('hoehe-w').title = 'Sollwert u8 ' + $('hoehe').value;
}
function hzAnzeigen(h) {
  const box = $('horizont');
  box.hidden = !h;
  if (!h) return;
  $('hz-zustand').textContent = HZ_ZUSTAND[h.zustand] || h.zustand;
  $('hz-stabil').checked = bildStabil;
  $('hz-werte').textContent = 'Kamerahöhe ' + zahlOder((window.letzterStatus || {}).cam_h_mm, 1, ' mm') + ' · IMU im Stand ' + zahlOder(h.ref_deg, 2, '°')
    + ' (±' + zahlOder(h.std_deg, 2, '°') + ') · Zeile Stand ' + zahlOder(h.row_stand) + ' / IMU ' + zahlOder(h.row_imu)
    + ' / Bild ' + zahlOder(h.row_bild) + ' · Bild−IMU ' + zahlOder(h.bild_minus_imu_deg, 2, '°')
    + ' · IMU roh ' + zahlOder(h.imu_roh_deg, 2, '°')
    + ' · Ego ok ' + (h.ego_ok != null ? Math.round(h.ego_ok * 100) + ' %' : '-')
    + (h.row_cal == null ? ' · keine Tabelle: tilt_deg der Konfig' : (h.tabelle_deckt ? '' : ' · außerhalb der Tabelle'));
  $('hz-speichern').disabled = h.zustand !== 'gültig' || hzZeile == null;
  if (hzZeile == null && h.edit_row != null) hzZeile = h.edit_row;
  $('hz-zeile').textContent = hzZeile == null ? '-' : hzZeile.toFixed(0);
  if (Date.now() - hzTabelleT > 5000) hzTabelle();
}
async function hzTabelle() {
  hzTabelleT = Date.now();
  try {
    const r = await fetch('api.php?a=horizon');
    const j = await r.json();
    if (!r.ok) return;
    const el = $('hz-tabelle');
    el.innerHTML = j.punkte.length ? j.punkte.map(p => esc(p.kamera_mm.toFixed(1) + ' mm → Zeile ' + p.row.toFixed(1)
      + (p.ref_deg != null ? ' (IMU ' + p.ref_deg.toFixed(2) + '°)' : ''))
      + ' <button onclick="hzLoeschen(' + p.height_mm + ')" title="Eintrag löschen">×</button>').join('<br>')
      : 'Tabelle leer';
  } catch (e) {}
}
async function hzSchieben(d) {
  const h = (window.letzterStatus || {}).horizon || {};
  if (hzZeile == null) hzZeile = h.row_cal ?? h.row_stand ?? h.row_imu ?? 240;
  hzZeile = Math.max(0, Math.min(479, hzZeile + d));
  $('hz-zeile').textContent = hzZeile.toFixed(0);
  await befehl('horizon_edit', {row: hzZeile}, true);
}
async function hzSpeichern() {
  if (hzZeile == null) return;
  if (await befehl('horizon_cal', {row: hzZeile})) { hzTabelle(); }
}
function hzStabil(an) {
  bildStabil = an;
  try { localStorage.setItem('xgonav.stabil', an ? '1' : '0'); } catch (e) {}
  bildVerschieben();
}
async function hzAus() { hzZeile = null; await befehl('horizon_edit', {row: ''}, true); }
async function hzLoeschen(mm) {
  if (!confirm('Horizont für ' + mm + ' mm löschen?')) return;
  if (await befehl('horizon_delete', {h: mm})) hzTabelle();
}
// WLAN aus dem Header X-Wlan der Statusabfrage ("rx tx dBm Qualität"); Raten hier aus der Differenz
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
async function status() {
  try {
    const r = await fetch('api.php?a=status');
    wlanMerken(r.headers.get('X-Wlan'));
    const s = await r.json();
    if (!r.ok) { hoeheAusStatus = false; zeige(s.error || ('Fehler ' + r.status)); verbindungWeg(); return; }
    verbindungOk();
    aktModus = s.mode;
    aktPipeline = s.pipeline;
    window.letzterStatus = s;
    if (s.cam_h_gerade) kamGerade = s.cam_h_gerade;
    hoeheAusStatus = s.height_u8 != null;
    if (hoeheAusStatus) hoeheSetzen(s.height_u8);
    hzAnzeigen(s.horizon);
    $('kurz').innerHTML = felder.map(([k, f, breit]) =>
      '<div class="kachel' + (breit ? ' breit' : '') + '"><div class="k">' + k + '</div><div class="w">' + esc(f(s)) + '</div></div>').join('');
    $('roh').textContent = JSON.stringify(s, null, 1);
    document.querySelectorAll('[data-modus]').forEach(b => b.classList.toggle('aktiv', b.dataset.modus === s.mode));
    pille('p-modus', MODI[s.mode] || s.mode || '-', s.mode && s.mode !== 'idle' ? 'warn' : '');
    pille('p-zustand', s.balancing == null ? '-' : (s.balancing ? 'balanciert' : 'liegt'),
          s.balancing == null ? '' : (s.balancing ? 'gut' : 'warn'));
    akkuMerken(s.battery);
    pille('p-akku', s.battery != null ? s.battery + ' %' : '-',
          s.battery == null ? '' : (s.battery < 20 ? 'schlecht' : s.battery < 40 ? 'warn' : 'gut'));
    pille('p-pip', s.pipeline ?? '-', s.pipeline === 'läuft' ? 'gut' : (s.pipeline ? 'warn' : ''));
    const pip = $('pip');
    pip.textContent = s.pipeline === 'läuft' ? 'Stoppen' : 'Starten';
    pip.classList.toggle('aktiv', s.pipeline !== 'läuft');
    if (s.grid) $('karte-pose').textContent =
      'x ' + s.grid.pose[0].toFixed(2) + ' m · y ' + s.grid.pose[1].toFixed(2) + ' m · ' + s.grid.pose[2].toFixed(0) + '° · Start ' + s.grid.zum_start_m.toFixed(2) + ' m · belegt ' + s.grid.belegt_m2 + ' m²';
    const ek = $('karte-erkunden');
    ek.hidden = !s.explore;
    if (s.explore) {
      const ende = s.explore.ende ? ' · ' + s.explore.ende : '';
      ek.textContent = 'Erkunden: ' + ERKUNDEN[s.explore.zustand] + ' · ' + s.explore.stellen + ' Stellen offen'
        + (s.explore.gesperrt ? ', ' + s.explore.gesperrt + ' übergangen' : '') + ende;
    }
    const st = $('karte-streife');
    st.hidden = !s.patrol;
    if (s.patrol) {
      const p = s.patrol;
      st.textContent = 'Streife: ' + STREIFE[p.zustand] + ' · Punkt ' + p.punkt + ' von ' + p.von + ' · Runde ' + p.runde
        + (p.ausgelassen.length ? ' · ausgelassen: ' + p.ausgelassen.join(', ') : '') + (p.ende ? ' · ' + p.ende : '');
    }
    if (s.grid) {
      karteGesperrt = !!s.grid.gesperrt;
      const ks = $('karte-sperre');
      ks.textContent = karteGesperrt ? 'Karte entsperren' : 'Karte sperren';
      ks.classList.toggle('aktiv', karteGesperrt);
    }
    belichtStatus(s.exposure);
    gainStatus(s.exposure);
  } catch (e) { hoeheAusStatus = false; zeige('keine Verbindung zu xgonav'); verbindungWeg(); }
}

// --- Handsteuerung: solange gedrückt alle 200 ms senden; xgonav hält nach 0,5 s ohne Sollwert an
const gedrueckt = new Set();
let fahrtTimer = null;
const tempo = () => Number($('tempo').value) / 100;
$('tempo').oninput = () => $('tempo-w').textContent = $('tempo').value + ' %';
$('tempo').oninput();
function sollwert() {
  const t = tempo();
  const v = (gedrueckt.has('vor') ? t : 0) - (gedrueckt.has('zurück') ? t : 0);
  const w = (gedrueckt.has('links') ? t : 0) - (gedrueckt.has('rechts') ? t : 0);
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

// --- Belichtung: Regler in ms, xgonav rechnet in µs (0 = Automatik)
let belichtBewegt = false;      // Regler nicht überschreiben, solange er bedient wird
const belichtRegler = $('belicht');
belichtRegler.addEventListener('pointerdown', () => belichtBewegt = true);
belichtRegler.addEventListener('pointerup', () => belichtBewegt = false);
function belichtAnzeige(us) {
  $('belicht-w').textContent = us ? (us / 1000).toFixed(0) + ' ms' : 'Auto';
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
    'ist ' + (e.ist_us != null ? (e.ist_us / 1000).toFixed(1) + ' ms' : '-') + ' · ' + (e.gain ?? '-') + '×';
}
function belichtung(us) { belichtBewegt = false; return befehl('exposure', {us}); }

// --- Verstärkung (0 = Automatik)
let gainBewegt = false;
const gainRegler = $('gain');
gainRegler.addEventListener('pointerdown', () => gainBewegt = true);
gainRegler.addEventListener('pointerup', () => gainBewegt = false);
function gainAnzeige(g) {
  $('gain-w').textContent = Number(g) ? Number(g).toFixed(1) + '×' : 'Auto';
}
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
  if (!confirm('Lage kalibrieren? Der Rider muss balancieren und frei stehen; er regelt sich ein paar Sekunden auf der Stelle ein.')) return;
  if (aktModus && aktModus !== 'idle' && !(await modus('idle'))) return;
  befehl('pitch_calibration');
}
// Aktion 255 = Arm (R-1.1.6): Räder laufen an, sobald der Rider aufrecht steht
function arm() {
  befehl('action', {n: 255});
}
// Aktion 253 = !standup: Rider springt auf und schwingt dabei aus – nur mit Nachfrage
function aufspringen() {
  if (confirm('Aufspringen? Der Rider muss auf den Rädern stehen und rundherum Platz haben – er springt hoch und schwingt dabei aus.')) befehl('action', {n: 253});
}
function herunterfahren() {
  if (confirm('Rider wirklich herunterfahren?')) befehl('shutdown');
}

// Raumkarte: einmal pro Sekunde neu laden, nächstes Bild erst nach dem vorigen. Verschieben und Zoomen
// zeichnet der Server; bis das neue Bild da ist, verschiebt und skaliert CSS das alte.
// Karten-KS: oben = +x, links = +y (m). cx/cy null = Mitte folgt dem Rider.
const karte = {cx: null, cy: null, v: 6, bild: null};   // bild: Stand des angezeigten Bildes (X-Map-View)
let karteUrl = null, karteTimer = null, karteLaeuft = false, karteNochmal = false;
async function karteHolen() {
  clearTimeout(karteTimer);
  if (karteLaeuft) { karteNochmal = true; return; }
  karteLaeuft = true;
  try {
    const mitte = karte.cx == null ? '' : '&cx=' + karte.cx.toFixed(3) + '&cy=' + karte.cy.toFixed(3);
    const r = await fetch('map.png?px=480&view=' + karte.v.toFixed(3) + mitte, {cache: 'no-store'});
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
  if (!r.ok) throw new Error('Route nicht lesbar');
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
  $('route-hinweis').textContent = (bearb.art === 'zone' ? 'Ecken der Sperrzone anklicken: ' : 'Wegpunkte anklicken: ') + bearb.pts.length;
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
  $('route-start').hidden = true; $('route-edit').hidden = false;
  $('karte-rahmen').classList.add('setzt');
  bearbeitenHinweis(); punkteZeichnen();
}
function bearbeitenEnde() {
  bearb.art = null; bearb.pts = [];
  $('route-start').hidden = false; $('route-edit').hidden = true;
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
  if (!confirm('Alle Sperrzonen löschen?')) return;
  let akt;
  try { akt = await routeHolen(); } catch (e) { zeige(e.message); return; }
  if (await befehl('route', {pts: punktText(akt.points), loop: akt.loop ? 1 : 0, nozones: 1})) karteAnstoss();
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
  keinSystemDrag(el);
  // Klick (Maus oder Finger): Punkt setzen. Ziehen verschiebt weiter die Karte.
  el.addEventListener('click', e => {
    if (!bearb.art || !karte.bild || gezogen) { gezogen = false; return; }
    const q = el.getBoundingClientRect(), [cx, cy] = mitte();
    const fx = (e.clientX - q.left) / q.width, fy = (e.clientY - q.top) / q.height;
    bearb.pts.push([cx + (0.5 - fy) * karte.v, cy + (0.5 - fx) * karte.v]);
    bearbeitenHinweis(); punkteZeichnen();
  });
  el.addEventListener('pointerdown', e => {
    if (e.pointerType === 'touch' || e.button !== 0 || !karte.bild) return;
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
  el.addEventListener('dblclick', () => { if (bearb.art) return; karte.cx = karte.cy = null; karteAnzeigen(); karteAnstoss(); });
  el.addEventListener('wheel', e => {
    if (!karte.bild) return;
    e.preventDefault();
    const d = e.deltaY * (e.deltaMode === 1 ? 33 : e.deltaMode === 2 ? 400 : 1);
    const neu = Math.max(1, Math.min(karte.bild.size, karte.v * Math.exp(d * 0.0015)));
    if (neu === karte.v) return;
    // Der Punkt unter dem Mauszeiger bleibt stehen
    const q = el.getBoundingClientRect(), fx = (e.clientX - q.left) / q.width, fy = (e.clientY - q.top) / q.height, [cx, cy] = mitte();
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
const LAGE_VERZUG = 300;    // ms hinter der neuesten Probe abspielen (Abfrage alle 250 ms)
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
  if (!w) { if (lageGL) modellZeichnen(K, null, B, H, dpr); g.fillStyle = '#8b94a1'; g.fillText('keine IMU-Daten', B / 2, 16); return; }
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
    $('l-lage').innerHTML = [['Roll', w[0], 1], ['Nick', w[1], 1], ['Gier', gier, 0]]
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
const uhr = t => new Date(t).toLocaleTimeString('de-DE', {hour: '2-digit', minute: '2-digit'});
function verlaufZeichnen() {
  if (!$('verlauf').open) return;
  // Lage: eigene Schleife (lageSchleife). CPU: Browserzeit
  const jetzt = Date.now(), n = cpu.length ? cpu[cpu.length - 1][1].length : 0;
  const cs = Array.from({length: n}, (_, k) =>
    ({name: 'Kern ' + k, farbe: FARBEN[k % FARBEN.length], pkt: cpu.map(p => [p[0], p[1][k]])}));
  legende('l-cpu', cs, ' %', 0);
  linien($('d-cpu'), cs, {von: jetzt - CPU_MS, bis: jetzt, ymin: 0, ymax: 100, einheit: '%', luecke: 5e3,
                          xText: uhr, xs: 120e3, nk: 0});
  const ws = [{name: '↓ empfangen', farbe: FARBEN[0], pkt: wlan.map(p => [p[0], p[1]])},
              {name: '↑ gesendet', farbe: FARBEN[1], pkt: wlan.map(p => [p[0], p[2]])}];
  legende('l-wlan', ws, ' kB/s', 0);
  linien($('d-wlan'), ws, {von: jetzt - CPU_MS, bis: jetzt, ymin: 0, einheit: '', luecke: 5e3, minSpanne: 10,
                           xText: uhr, xs: 120e3, nk: 0});
  const wl = wlan.length ? wlan[wlan.length - 1] : null;
  $('v-wlan').textContent = wl ? (wl[3] != null ? wl[3] + ' dBm' : '') + (wl[4] != null ? ' · Qualität ' + wl[4] + '/70' : '') : '';
  linien($('d-pegel'), [{name: 'Pegel', farbe: FARBEN[2], pkt: wlan.filter(p => p[3] != null).map(p => [p[0], p[3]])}],
         {von: jetzt - CPU_MS, bis: jetzt, ymin: -90, ymax: -30, einheit: '', luecke: 5e3, xText: uhr, xs: 120e3, nk: 0});
  const a = akkuLesen().filter(p => p[0] > jetzt - AKKU_MS);
  $('v-akku').textContent = a.length ? a[a.length - 1][1] + ' %' : '';
  linien($('d-akku'), [{name: 'Akku', farbe: FARBEN[0], pkt: a}],
         {von: jetzt - AKKU_MS, bis: jetzt, ymin: 0, ymax: 100, einheit: '%', luecke: 5 * 60e3, xText: uhr,
          xs: 30 * 60e3, nk: 0});
}
let sysTimer = null;
// Offen/zu merken, damit der Verlauf nach dem Neuladen gleich weiterläuft
function verlaufUmschalten() {
  try { localStorage.setItem('xgonav.verlauf', $('verlauf').open ? '1' : '0'); } catch (e) {}
  if ($('verlauf').open) sysHolen();
}
function lageUmschalten() {
  try { localStorage.setItem('xgonav.lage', $('lagekarte').open ? '1' : '0'); } catch (e) {}
  if ($('lagekarte').open) { sysHolen(); lageStarten(); }
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
let sysErstes = true;       // einmal immer: Regler stimmen auch bei zugeklappten Karten
async function sysHolen() {
  clearTimeout(sysTimer);
  if (!sysErstes && !$('verlauf').open && !$('lagekarte').open) return; // beide zugeklappt: nichts abfragen
  sysErstes = false;
  try {
    const r = await fetch('api.php?a=sys&since=' + imuSeq + '&wsince=' + radSeq, {cache: 'no-store'});
    const s = await r.json();
    if (r.ok) {
      if (s.seq < imuSeq) { lage.length = 0; imuSeq = 0; lageOff = null; }  // Treiber neu geladen
      for (const p of s.imu) { lage.push([p[1], p[2], p[3], p[4]]); imuSeq = p[0]; }
      if (s.wseq != null) {
        if (s.wseq < radSeq) { radPuffer.length = 0; radSeq = 0; radRoh = null; }   // Treiber neu geladen
        const d32 = d => ((d + 2 ** 31) % 2 ** 32 + 2 ** 32) % 2 ** 32 - 2 ** 31;  // wie _wrap32 in wheels.py
        for (const p of s.wheels || []) {
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
      if (s.height != null) { beinSoll.h = s.height; if (!hoeheAusStatus) hoeheSetzen(s.height); }
      if (s.roll != null) { beinSoll.r = s.roll; reglerSetzen('roll', s.roll, s.roll + '°'); }
      beinSoll.abgesetzt = s.state === 254;
      const jetzt = Date.now();
      if (!cpuVorher || jetzt - cpuZeit >= 1000) {         // CPU, Temperatur, Akku: 1 Hz
        if (cpuVorher && cpuVorher.length === s.cpu.length)
          cpu.push([jetzt, s.cpu.map((c, k) => {
            const dt = c[0] - cpuVorher[k][0];
            return dt > 0 ? Math.max(0, Math.min(100, 100 * (1 - (c[1] - cpuVorher[k][1]) / dt))) : 0;
          })]);
        cpuVorher = s.cpu; cpuZeit = jetzt;
        abschneiden(cpu, jetzt - CPU_MS);
        $('v-temp').textContent = (s.temp_c != null ? s.temp_c.toFixed(1) + ' °C' : '') +
                                  (s.load != null ? ' · Last ' + s.load.toFixed(2) : '');
        akkuMerken(s.battery);
        wlanMerken(s.wlan);   // Verlauf aus derselben Abfrage wie die CPU: setInterval(status) drosselt
                              // der Browser im Hintergrund auf 1×/min, diese Kette nur auf 1×/s
        verlaufZeichnen();
      }
    }
  } catch (e) { /* Rider weg: beim nächsten Mal wieder */ }
  sysTimer = setTimeout(sysHolen, 250);                    // Lage live; IMU-Ring hält ~2,5 s
}
// Fadenkreuz: Maus und Touch
document.querySelectorAll('.diag').forEach(cv => {
  cv.addEventListener('pointermove', e => { cv._hx = e.offsetX / cv.clientWidth; verlaufZeichnen(); });
  cv.addEventListener('pointerleave', () => { cv._hx = null; verlaufZeichnen(); });
});
window.addEventListener('resize', verlaufZeichnen);
bildVerbinden();
karteHolen();
status();
if (localStorage.getItem('xgonav.verlauf') === '1') $('verlauf').open = true;
if (localStorage.getItem('xgonav.lage') !== '0') $('lagekarte').open = true;   // Standard: offen
sysHolen();
lageStarten();
setInterval(status, 700);
</script>
</body>
</html>
