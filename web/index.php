<?php
/*
 * xgonav – Bedienoberfläche. Bild: WebSocket ws (nginx -> xgonav /ws, Bestätigung je Bild),
 * Raumkarte: map.png, Status und Befehle: api.php. Enthält die Funktionen der früheren
 * Fernsteuerung (rc_server): Handsteuerung, Höhe, Aktionen, LEDs, Emotionen, Herunterfahren.
 * Keine Authentifizierung.
 *
 * Aufbau: Kopfleiste (immer sichtbar, mit STOPP) – links Bild, Modus, Kennzahlen –
 * rechts Karten (Fahren, Raumkarte, Kamera, Extras eingeklappt, System).
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
#bild{width:100%;height:auto;aspect-ratio:4/3;object-fit:contain;display:block;background:#000;border:1px solid var(--rand);border-radius:var(--r)}
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
.fahren{display:grid;grid-template-columns:auto 1fr;gap:14px;align-items:start}
@media (max-width:420px){.fahren{grid-template-columns:1fr}.pad{justify-self:center}}
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
#karte{width:100%;max-width:400px;height:auto;aspect-ratio:1;background:#808080;display:block;border-radius:6px;margin:0 auto}

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
  </div>
  <button id="stop" onclick="befehl('stop')" title="Stillstand, Modus idle">STOPP</button>
</header>
<div id="meldung"></div>

<main>
<div class="spalte">
  <img id="bild" alt="" aria-label="Kamerabild" width="640" height="480">
  <div class="karte">
    <div class="modus">
      <div class="segment">
        <button data-modus="idle" onclick="modus('idle')">Idle</button>
        <button data-modus="manual" onclick="modus('manual')">Hand</button>
        <button data-modus="follow" onclick="modus('follow')">Folgen</button>
        <button data-modus="explore" onclick="modus('explore')">Erkunden</button>
        <button data-modus="approach" onclick="modus('approach')">Anfahren</button>
        <button data-modus="home" onclick="modus('home')" title="Weg über die Raumkarte zum Startpunkt">Zum Start</button>
      </div>
      <input id="ziel" value="person" title="Ziel für Folgen/Anfahren (COCO-Klasse)" aria-label="Ziel">
    </div>
  </div>
  <div class="karte">
    <div class="kopf"><h2>Bildverarbeitung</h2><button id="pip" onclick="pipelineUmschalten()">…</button></div>
    <div class="kacheln" id="kurz"></div>
    <details class="roh"><summary>Status (roh)</summary><pre id="roh"></pre></details>
  </div>
</div>

<div class="spalte">
  <section class="karte">
    <div class="kopf"><h2>Fahren</h2>
      <button onclick="arm()" title="Aktion 255">Aufstehen</button>
      <button class="gefahr" onclick="befehl('action', {n: 254})" title="Aktion 254, Not-Stopp">Absetzen</button>
    </div>
    <div class="fahren">
      <div class="pad">
        <span></span><button data-fahrt="vor" aria-label="vor">▲</button><span></span>
        <button data-fahrt="links" aria-label="links">◀</button><span></span><button data-fahrt="rechts" aria-label="rechts">▶</button>
        <span></span><button data-fahrt="zurück" aria-label="zurück">▼</button><span></span>
      </div>
      <div class="regler">
        <label for="tempo">Tempo</label>
        <input type="range" id="tempo" min="5" max="100" value="60">
        <span class="w" id="tempo-w"></span><span class="leer"></span>
        <label for="hoehe">Höhe</label>
        <input type="range" id="hoehe" min="43" max="255" value="128"
          oninput="document.getElementById('hoehe-w').textContent = this.value"
          onchange="befehl('height', {h: this.value})">
        <span class="w" id="hoehe-w">128</span><span class="leer"></span>
        <label for="roll">Roll</label>
        <input type="range" id="roll" min="-20" max="20" value="0" step="1"
          oninput="document.getElementById('roll-w').textContent = this.value + '°'"
          onchange="befehl('roll', {deg: this.value})">
        <span class="w" id="roll-w">0°</span><button onclick="rollNull()" title="Roll auf 0°">0</button>
      </div>
    </div>
    <details class="hilfe"><summary>Gedrückt halten: Maus, Touch, Pfeiltasten/WASD</summary>
      <p>Fahren schaltet auf „Hand“ und ist auf max_v/max_w der Konfiguration begrenzt. Ohne Bodenmodell
        und Bahnprüfung; Sturz-Erkennung und Watchdogs bleiben aktiv. Höhe: untere Grenze mit Abstand zum
        Minimum; in Folgen/Erkunden setzt xgonav die Höhe selbst. Roll: + = rechts.</p>
      <p>Balancieren (Firmware R-1.1.6): Nach einem ESP32-Neustart balanciert der Rider erst nach
        „Aufstehen“. Liegt er, wartet die Firmware, bis er aufrecht steht – dann laufen die Räder sofort an.
        „Absetzen“ ist ein Not-Stopp (Aktion 254, wie <code>!stop</code>): er setzt sich kontrolliert ab und
        bleibt aus bis zum nächsten Aufstehen.</p>
    </details>
  </section>

  <section class="karte">
    <div class="kopf"><h2>Raumkarte</h2>
      <select id="karte-view" aria-label="Ausschnitt"><option value="3">3 m</option><option value="6" selected>6 m</option><option value="10">10 m</option><option value="16">16 m</option></select>
      <button onclick="befehl('map_start')" title="Startpunkt = aktuelle Pose">Start hier</button>
      <button onclick="befehl('map_save')" title="Raumkarte jetzt speichern ([grid] file)">Speichern</button>
      <button onclick="befehl('map_reset')">Zurücksetzen</button>
    </div>
    <img id="karte" alt="Raumkarte" width="320" height="320">
    <div class="leise" style="margin-top:6px" id="karte-pose">-</div>
    <details class="hilfe"><summary>grau unbekannt · hell frei · dunkel belegt · grün Weg · cyan Plan · blau Start · rot Rider</summary>
      <p>Ursprung = Startpose (oben = vorn beim Start). Strecke aus den Rädern, Drehung aus der Kamera;
        ohne Marker driftet die Karte mit der Zeit.</p>
      <p>„Zum Start“ plant einen Weg über die Karte und fährt ihn ab. Die Karte wird regelmäßig gespeichert
        und beim Start von xgonav geladen; der Rider muss dann wieder am Startpunkt stehen.</p>
    </details>
  </section>

  <section class="karte">
    <div class="kopf"><h2>Kamera</h2><span class="leise" id="belicht-ist">-</span></div>
    <div class="regler">
      <label for="belicht">Belichtung</label>
      <input type="range" id="belicht" min="1" max="33" value="15" step="1"
        oninput="belichtAnzeige(this.value * 1000)" onchange="belichtung(this.value * 1000)">
      <span class="w" id="belicht-w">-</span>
      <button id="belicht-auto" onclick="belichtung(0)">Auto</button>
      <label for="gain">Verstärkung</label>
      <input type="range" id="gain" min="1" max="16" value="8" step="0.5"
        oninput="gainAnzeige(this.value)" onchange="verstaerkung(this.value)">
      <span class="w" id="gain-w">-</span>
      <button id="gain-auto" onclick="verstaerkung(0)">Auto</button>
    </div>
    <details class="hilfe"><summary>Gilt bis zum Neustart von xgonav</summary>
      <p>Kürzere Belichtung = weniger Bewegungsunschärfe, aber dunkler; mehr Verstärkung = heller, aber mehr
        Rauschen (Automatik nur bis 8). Dauerhaft über <code>[camera] exposure_us</code> /
        <code>analogue_gain</code> in config.toml.</p>
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
const MODI = {idle: 'Idle', manual: 'Hand', follow: 'Folgen', explore: 'Erkunden', approach: 'Anfahren', home: 'Zum Start'};
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
// WLAN (bis 20 s). Nachricht: 4 Byte Bildnummer + JPEG. Bei Abbruch nach 1 s neu verbinden.
let bildUrl = null;
function bildVerbinden() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host +
                           location.pathname.replace(/[^/]*$/, '') + 'ws');
  ws.binaryType = 'arraybuffer';
  ws.onmessage = ev => {
    ws.send('ok');                                    // Bestätigung: nächstes Bild bitte
    const url = URL.createObjectURL(new Blob([new Uint8Array(ev.data, 4)], {type: 'image/jpeg'}));
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
function pipelineUmschalten() { return befehl('pipeline', {on: aktPipeline === 'läuft' ? 0 : 1}); }
async function status() {
  try {
    const r = await fetch('api.php?a=status');
    const s = await r.json();
    if (!r.ok) { zeige(s.error || ('Fehler ' + r.status)); verbindungWeg(); return; }
    verbindungOk();
    aktModus = s.mode;
    aktPipeline = s.pipeline;
    $('kurz').innerHTML = felder.map(([k, f, breit]) =>
      '<div class="kachel' + (breit ? ' breit' : '') + '"><div class="k">' + k + '</div><div class="w">' + esc(f(s)) + '</div></div>').join('');
    $('roh').textContent = JSON.stringify(s, null, 1);
    document.querySelectorAll('[data-modus]').forEach(b => b.classList.toggle('aktiv', b.dataset.modus === s.mode));
    pille('p-modus', MODI[s.mode] || s.mode || '-', s.mode && s.mode !== 'idle' ? 'warn' : '');
    pille('p-zustand', s.balancing == null ? '-' : (s.balancing ? 'balanciert' : 'liegt'),
          s.balancing == null ? '' : (s.balancing ? 'gut' : 'warn'));
    pille('p-akku', s.battery != null ? s.battery + ' %' : '-',
          s.battery == null ? '' : (s.battery < 20 ? 'schlecht' : s.battery < 40 ? 'warn' : 'gut'));
    pille('p-pip', s.pipeline ?? '-', s.pipeline === 'läuft' ? 'gut' : (s.pipeline ? 'warn' : ''));
    const pip = $('pip');
    pip.textContent = s.pipeline === 'läuft' ? 'Stoppen' : 'Starten';
    pip.classList.toggle('aktiv', s.pipeline !== 'läuft');
    if (s.grid) $('karte-pose').textContent =
      'x ' + s.grid.pose[0].toFixed(2) + ' m · y ' + s.grid.pose[1].toFixed(2) + ' m · ' + s.grid.pose[2].toFixed(0) + '° · Start ' + s.grid.zum_start_m.toFixed(2) + ' m · belegt ' + s.grid.belegt_m2 + ' m²';
    belichtStatus(s.exposure);
    gainStatus(s.exposure);
  } catch (e) { zeige('keine Verbindung zu xgonav'); verbindungWeg(); }
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

function rollNull() {
  const r = $('roll');
  r.value = 0; r.oninput(); befehl('roll', {deg: 0});
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
  if (confirm('Aufstehen (Arm)? Steht der Rider aufrecht, laufen die Räder sofort an. Nur mit Platz drumherum.'))
    befehl('action', {n: 255});
}
function herunterfahren() {
  if (confirm('Rider wirklich herunterfahren?')) befehl('shutdown');
}

// Raumkarte: einmal pro Sekunde neu laden, nächstes Bild erst nach dem vorigen
let karteUrl = null;
async function karteHolen() {
  try {
    const v = $('karte-view').value;
    const r = await fetch('map.png?px=480&view=' + v, {cache: 'no-store'});
    if (r.ok) {
      const url = URL.createObjectURL(await r.blob());
      $('karte').src = url;
      if (karteUrl) URL.revokeObjectURL(karteUrl);
      karteUrl = url;
    }
  } catch (e) { /* xgonav weg */ }
  setTimeout(karteHolen, 1000);
}
bildVerbinden();
karteHolen();
status();
setInterval(status, 700);
</script>
</body>
</html>
