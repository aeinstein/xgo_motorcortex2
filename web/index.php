<?php
/*
 * xgonav – Bedienoberfläche. Bild: stream (nginx -> xgonav /stream, MJPEG),
 * Status und Befehle: api.php. Enthält die Funktionen der früheren Fernsteuerung
 * (rc_server): Handsteuerung, Höhe, Aktionen, LEDs, Emotionen, Kalibrierung,
 * Herunterfahren. Keine Authentifizierung.
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
<title>xgonav – <?= $host ?></title>
<style>
body{font:14px system-ui,sans-serif;background:#111;color:#ddd;margin:12px}
/* feste 4:3-Fläche (Kamera 640x480), auch ohne Stream; Bild wird darin eingepasst */
#bild{width:100%;height:auto;aspect-ratio:4/3;object-fit:contain;box-sizing:border-box;border:1px solid #333;background:#000;display:block}
button{font-size:15px;margin:3px;padding:8px 14px;border-radius:4px;border:1px solid #555;background:#2a2a2a;color:#eee;
  touch-action:manipulation;cursor:pointer;transition:background-color .12s,border-color .12s,transform .05s}
/* Hover nur bei echter Maus: auf Touch-Geräten bliebe der Effekt sonst nach dem Tippen hängen */
@media (hover:hover){
  button:hover{background:#3a3a3a;border-color:#888}
  button.aktiv:hover{background:#2f5a3a;border-color:#6c6}
  #stop:hover{background:#e00;border-color:#f44}
  #aus:hover{background:#633;border-color:#a66}
}
button:active{transform:translateY(1px);background:#4a4a4a}
button:focus-visible{outline:2px solid #6af;outline-offset:1px}
button:disabled{cursor:not-allowed;opacity:.5}
button.aktiv{border-color:#4a4;background:#243}
#stop{background:#c00;color:#fff;font-weight:bold;border-color:#c00}
#stop:active{background:#a00}
#aus{background:#422;border-color:#844}
input[type=range],input[type=color],select{cursor:pointer}
.raster{display:grid;grid-template-columns:minmax(320px,640px) minmax(260px,1fr);gap:14px;align-items:start}
@media (max-width:900px){.raster{grid-template-columns:1fr}}
fieldset{border:1px solid #333;border-radius:6px;margin:0 0 10px;padding:6px 10px}
legend{color:#9ab}
.pad{display:grid;grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,52px);gap:4px;user-select:none}
.pad button{margin:0;padding:0;font-size:20px}
.leds{display:grid;grid-template-columns:repeat(2,48px);gap:6px;margin:6px 0}
input[type=color]{width:48px;height:36px;border:0;background:none}
input[type=range]{width:200px}
table{border-collapse:collapse}
td{padding:2px 10px 2px 0;vertical-align:top}
td:first-child{color:#999}
pre{background:#1b1b1b;padding:8px;overflow:auto;max-height:40vh}
#meldung{color:#e88;min-height:1.2em}
.klein{color:#888;font-size:12px}
</style>
</head>
<body>
<div class="raster">
<div>
  <img id="bild" alt="Kamerabild" width="640" height="480">
  <div style="margin-top:6px">
    <button id="stop" onclick="befehl('stop')">STOPP</button>
    <button data-modus="idle" onclick="modus('idle')">Idle</button>
    <button data-modus="manual" onclick="modus('manual')">Hand</button>
    <button data-modus="follow" onclick="modus('follow')">Folgen</button>
    <button data-modus="explore" onclick="modus('explore')">Erkunden</button>
    <button data-modus="approach" onclick="modus('approach')">Anfahren</button>
    Ziel: <input id="ziel" value="person" size="10">
  </div>
  <div>
    <button id="pip-aus" onclick="befehl('pipeline', {on: 0})">Bildverarbeitung stoppen</button>
    <button id="pip-an" onclick="befehl('pipeline', {on: 1})">Bildverarbeitung starten</button>
  </div>
  <div id="meldung"></div>
  <table id="kurz"></table>
  <details><summary>Status (roh)</summary><pre id="roh"></pre></details>
</div>
<div>
  <fieldset><legend>Handsteuerung</legend>
    <div class="pad">
      <span></span><button data-fahrt="vor">▲</button><span></span>
      <button data-fahrt="links">◀</button><span></span><button data-fahrt="rechts">▶</button>
      <span></span><button data-fahrt="zurück">▼</button><span></span>
    </div>
    Tempo <input type="range" id="tempo" min="5" max="100" value="60"> <span id="tempo-w"></span>
    <div class="klein">Gedrückt halten (Maus, Touch, Pfeiltasten/WASD). Schaltet auf „Hand“; begrenzt auf max_v/max_w
      der Konfiguration. Ohne Bodenmodell/Bahnprüfung, Sturz-Erkennung und Watchdogs bleiben.</div>
  </fieldset>
  <fieldset><legend>Raumkarte</legend>
    <img id="karte" alt="Raumkarte" width="320" height="320" style="width:100%;max-width:480px;height:auto;aspect-ratio:1;background:#808080;display:block">
    <div class="klein">Ausschnitt <select id="karte-view"><option>3</option><option selected>6</option><option>10</option><option>16</option></select> m ·
      grau unbekannt, hell frei, dunkel belegt, grün Weg, rot Rider. Pose: <span id="karte-pose">-</span></div>
    <button onclick="befehl('map_reset')">Karte zurücksetzen</button>
    <div class="klein">Ursprung = Startpose (oben = vorn beim Start). Strecke aus den Rädern, Drehung aus
      der Kamera; ohne Marker driftet die Karte mit der Zeit.</div>
  </fieldset>
  <fieldset><legend>Kamera</legend>
    Belichtung <input type="range" id="belicht" min="1" max="33" value="15" step="1"
      oninput="belichtAnzeige(this.value * 1000)" onchange="belichtung(this.value * 1000)">
    <span id="belicht-w">-</span>
    <button id="belicht-auto" onclick="belichtung(0)">Auto</button>
    <br>Verstärkung <input type="range" id="gain" min="1" max="16" value="8" step="0.5"
      oninput="gainAnzeige(this.value)" onchange="verstaerkung(this.value)">
    <span id="gain-w">-</span>
    <button id="gain-auto" onclick="verstaerkung(0)">Auto</button>
    <div class="klein">Ist: <span id="belicht-ist">-</span>. Kürzere Belichtung = weniger Bewegungsunschärfe, aber
      dunkler; mehr Verstärkung = heller, aber mehr Rauschen (Automatik nur bis 8). Gilt bis zum Neustart
      von xgonav; dauerhaft über <code>[camera] exposure_us</code> / <code>analogue_gain</code> in config.toml.</div>
  </fieldset>
  <fieldset><legend>Haltung</legend>
    Höhe <input type="range" id="hoehe" min="43" max="255" value="128"
      oninput="document.getElementById('hoehe-w').textContent = this.value"
      onchange="befehl('height', {h: this.value})"> <span id="hoehe-w">128</span>
    <div class="klein">Untere Grenze mit Abstand zum Minimum. In Folgen/Erkunden setzt xgonav die Höhe selbst.</div>
    Roll <input type="range" id="roll" min="-20" max="20" value="0" step="1"
      oninput="document.getElementById('roll-w').textContent = this.value + '°'"
      onchange="befehl('roll', {deg: this.value})"> <span id="roll-w">0°</span> <span class="klein">(+ = rechts)</span>
    <button onclick="rollNull()">0°</button>
  </fieldset>
  <fieldset><legend>Balancieren (Firmware R-1.1.6)</legend>
    <button onclick="arm()">Aufstehen (Arm)</button>
    <button onclick="befehl('action', {n: 254})">Absetzen (Disarm)</button>
    <div class="klein">Nach einem ESP32-Neustart balanciert der Rider erst nach „Aufstehen“. Liegt er,
      wartet die Firmware, bis er aufrecht steht – dann laufen die Räder sofort an. „Absetzen“ ist ein
      Not-Stopp (Aktion 254, wie <code>!stop</code>): er setzt sich kontrolliert ab und bleibt aus bis zum nächsten Aufstehen.</div>
  </fieldset>
  <fieldset><legend>Aktionen</legend>
    <?php for ($i = 1; $i <= 6; $i++): ?><button onclick="befehl('action', {n: <?= $i ?>})"><?= $i ?></button><?php endfor; ?>
  </fieldset>
  <fieldset><legend>LEDs</legend>
    <div class="leds">
      <input type="color" id="led0" onchange="led(0, this.value)"><input type="color" id="led3" onchange="led(3, this.value)">
      <input type="color" id="led1" onchange="led(1, this.value)"><input type="color" id="led2" onchange="led(2, this.value)">
    </div>
    <button onclick="ledsAus()">Alle aus</button>
    <button id="strobe" onclick="strobe()">Strobe</button>
  </fieldset>
  <fieldset><legend>Emotionen (Display)</legend>
    <?php foreach ($emotionen as $e): ?><button onclick="befehl('emotion', {name: '<?= h($e) ?>'})"><?= h($e) ?></button><?php endforeach; ?>
    <?php if (!$emotionen): ?><span class="klein">keine gefunden in <?= h(RC) ?>/emotions</span><?php endif; ?>
  </fieldset>
  <fieldset><legend>System</legend>
    <!-- Kalibrier-Knöpfe gesperrt (2026-11-02): Speichern bei schlaffen Beinen hat die
         Beinnullpunkte im ESP32-Flash überschrieben. Erst mit geführtem Ablauf wieder freigeben. -->
    <button id="aus" onclick="herunterfahren()">Herunterfahren</button>
  </fieldset>
</div>
</div>
<script>
const felder = [
  ['Modus', s => s.mode], ['Bildverarbeitung', s => s.pipeline], ['Status', s => s.status],
  ['Befehl v / w', s => s.cmd ? s.cmd.v.toFixed(2) + ' / ' + s.cmd.w.toFixed(2) : '-'],
  ['Akku', s => s.battery != null ? s.battery + ' %' : '-'],
  ['Zustand', s => s.balancing == null ? '-' : (s.balancing ? 'balanciert' : 'gefallen / geparkt')],
  ['Kurs', s => s.yaw_drv != null ? Math.round(s.yaw_drv) + '°' : '-'],
  ['Kamera / Schleife', s => (s.fps ?? '-') + ' fps / ' + (s.loop_ms ?? '-') + ' ms' +
    (s.loop_p95 != null ? ' (p95 ' + s.loop_p95 + ', max ' + s.loop_max + ')' : '')],
  ['Detektor', s => s.det_ms != null ? Math.round(s.det_ms) + ' ms' : '-'],
  ['Nicken', s => (s.pitch_src ?? '-') + ' ' + (s.pitch_deg != null ? s.pitch_deg.toFixed(1) + '°' : '')],
  ['Drehrate Gier / Nick', s => s.ego ? s.ego.yaw_dps.toFixed(0) + ' / ' + s.ego.pitch_dps.toFixed(0) + ' °/s' : '-'],
  ['TTC / Schärfe', s => (s.ttc != null ? s.ttc.toFixed(1) + ' s' : '-') + ' / ' + (s.ego ? s.ego.sharp : '-')],
];
let aktModus = null;
function zeige(text) { document.getElementById('meldung').textContent = text || ''; }
// Verbindungsfehler verschwinden von selbst, sobald xgonav wieder antwortet (andere Meldungen bleiben)
const VERBINDUNG = /nicht erreichbar|keine Verbindung/;
function verbindungOk() {
  if (VERBINDUNG.test(document.getElementById('meldung').textContent)) zeige('');
}
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
    document.getElementById('bild').src = url;
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
  } catch (e) { zeige('keine Verbindung'); return false; }
  finally { if (!still) status(); }
}
function modus(n) { return befehl('mode', {name: n, target: document.getElementById('ziel').value.trim().toLowerCase()}); }
async function status() {
  try {
    const r = await fetch('api.php?a=status');
    const s = await r.json();
    if (!r.ok) { zeige(s.error || ('Fehler ' + r.status)); return; }
    verbindungOk();
    aktModus = s.mode;
    document.getElementById('kurz').innerHTML = felder
      .map(([k, f]) => '<tr><td>' + k + '</td><td>' + String(f(s) ?? '-').replace(/[<&]/g, c => c === '<' ? '&lt;' : '&amp;') + '</td></tr>').join('');
    document.getElementById('roh').textContent = JSON.stringify(s, null, 1);
    document.querySelectorAll('[data-modus]').forEach(b => b.classList.toggle('aktiv', b.dataset.modus === s.mode));
    if (s.grid) document.getElementById('karte-pose').textContent =
      'x ' + s.grid.pose[0].toFixed(2) + ' m, y ' + s.grid.pose[1].toFixed(2) + ' m, ' + s.grid.pose[2].toFixed(0) + '°, belegt ' + s.grid.belegt_m2 + ' m²';
    belichtStatus(s.exposure);
    gainStatus(s.exposure);
    document.getElementById('pip-an').classList.toggle('aktiv', s.pipeline === 'läuft');
    document.getElementById('pip-aus').classList.toggle('aktiv', s.pipeline === 'gestoppt');
  } catch (e) { zeige('keine Verbindung zu xgonav'); }
}

// --- Handsteuerung: solange gedrückt alle 200 ms senden; xgonav hält nach 0,5 s ohne Sollwert an
const gedrueckt = new Set();
let fahrtTimer = null;
const tempo = () => Number(document.getElementById('tempo').value) / 100;
document.getElementById('tempo').oninput = () => document.getElementById('tempo-w').textContent = document.getElementById('tempo').value + ' %';
document.getElementById('tempo').oninput();
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
document.addEventListener('keydown', e => { const r = tasten[e.key]; if (r && e.target.tagName !== 'INPUT') { e.preventDefault(); fahrtStart(r); } });
document.addEventListener('keyup', e => { const r = tasten[e.key]; if (r) fahrtEnde(r); });
window.addEventListener('blur', () => [...gedrueckt].forEach(fahrtEnde));

// --- Belichtung: Regler in ms, xgonav rechnet in µs (0 = Automatik)
let belichtBewegt = false;      // Regler nicht überschreiben, solange er bedient wird
const belichtRegler = document.getElementById('belicht');
belichtRegler.addEventListener('pointerdown', () => belichtBewegt = true);
belichtRegler.addEventListener('pointerup', () => belichtBewegt = false);
function belichtAnzeige(us) {
  document.getElementById('belicht-w').textContent = us ? (us / 1000).toFixed(0) + ' ms' : 'Auto';
}
function belichtStatus(e) {
  if (!e) return;
  belichtRegler.max = Math.floor(e.max_us / 1000);
  if (!belichtBewegt && document.activeElement !== belichtRegler) {
    if (e.soll_us) belichtRegler.value = Math.round(e.soll_us / 1000);
    belichtAnzeige(e.soll_us);
  }
  document.getElementById('belicht-auto').classList.toggle('aktiv', e.soll_us === 0);
  document.getElementById('belicht-ist').textContent =
    (e.ist_us != null ? (e.ist_us / 1000).toFixed(1) + ' ms' : '-') + ', Verstärkung ' + (e.gain ?? '-');
}
function belichtung(us) { belichtBewegt = false; return befehl('exposure', {us}); }

// --- Verstärkung (0 = Automatik)
let gainBewegt = false;
const gainRegler = document.getElementById('gain');
gainRegler.addEventListener('pointerdown', () => gainBewegt = true);
gainRegler.addEventListener('pointerup', () => gainBewegt = false);
function gainAnzeige(g) {
  document.getElementById('gain-w').textContent = Number(g) ? Number(g).toFixed(1) + '×' : 'Auto';
}
function gainStatus(e) {
  if (!e || e.gain_soll == null) return;
  if (e.gain_max) gainRegler.max = e.gain_max;
  if (!gainBewegt && document.activeElement !== gainRegler) {
    if (e.gain_soll) gainRegler.value = e.gain_soll;
    gainAnzeige(e.gain_soll);
  }
  document.getElementById('gain-auto').classList.toggle('aktiv', e.gain_soll === 0);
}
function verstaerkung(g) { gainBewegt = false; return befehl('gain', {g}); }

// --- LEDs
function led(i, c) { document.getElementById('led' + i).value = c; return befehl('led', {i, c}, true); }
function ledsAus() { for (let i = 0; i < 4; i++) led(i, '#000000'); }
let strobeTimer = null, strobeLed = 0;
function strobe() {
  const b = document.getElementById('strobe');
  if (strobeTimer) { clearInterval(strobeTimer); strobeTimer = null; b.classList.remove('aktiv'); ledsAus(); return; }
  b.classList.add('aktiv');
  strobeTimer = setInterval(() => { led(strobeLed, '#000000'); strobeLed = (strobeLed + 1) % 4; led(strobeLed, '#0000ff'); }, 100);
}

function rollNull() {
  const r = document.getElementById('roll');
  r.value = 0; r.oninput(); befehl('roll', {deg: 0});
}

// --- System
let kalibAn = false;
function kalibrierung() {
  kalibAn = !kalibAn;
  document.getElementById('kalib').classList.toggle('aktiv', kalibAn);
  document.getElementById('kalib').textContent = kalibAn ? 'Beine speichern' : 'Beine kalibrieren';
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
    const v = document.getElementById('karte-view').value;
    const r = await fetch('map.png?px=480&view=' + v, {cache: 'no-store'});
    if (r.ok) {
      const url = URL.createObjectURL(await r.blob());
      document.getElementById('karte').src = url;
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
