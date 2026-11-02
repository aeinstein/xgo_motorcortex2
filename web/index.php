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
img{max-width:100%;border:1px solid #333;background:#000;min-height:120px;display:block}
button{font-size:15px;margin:3px;padding:8px 14px;border-radius:4px;border:1px solid #555;background:#2a2a2a;color:#eee;touch-action:manipulation}
button.aktiv{border-color:#4a4;background:#243}
#stop{background:#c00;color:#fff;font-weight:bold;border-color:#c00}
#aus{background:#422;border-color:#844}
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
  <img id="bild" src="stream" alt="Kamerabild">
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
  <fieldset><legend>Aktionen</legend>
    <?php for ($i = 1; $i <= 6; $i++): ?><button onclick="befehl('action', {n: <?= $i ?>})"><?= $i ?></button><?php endfor; ?>
    <button onclick="befehl('action', {n: 255})">Stopp</button>
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
    <button id="kalib" onclick="kalibrierung()">Kalibrierung</button>
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
  ['Kamera / Schleife', s => (s.fps ?? '-') + ' fps / ' + (s.loop_ms ?? '-') + ' ms'],
  ['Nicken', s => (s.pitch_src ?? '-') + ' ' + (s.pitch_deg ?? '')],
];
let aktModus = null;
function zeige(text) { document.getElementById('meldung').textContent = text || ''; }
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
    aktModus = s.mode;
    document.getElementById('kurz').innerHTML = felder
      .map(([k, f]) => '<tr><td>' + k + '</td><td>' + String(f(s) ?? '-').replace(/[<&]/g, c => c === '<' ? '&lt;' : '&amp;') + '</td></tr>').join('');
    document.getElementById('roh').textContent = JSON.stringify(s, null, 1);
    document.querySelectorAll('[data-modus]').forEach(b => b.classList.toggle('aktiv', b.dataset.modus === s.mode));
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
  document.getElementById('kalib').textContent = kalibAn ? 'Kalibrierung beenden' : 'Kalibrierung';
  befehl('calibration', {on: kalibAn ? 1 : 0});
}
function herunterfahren() {
  if (confirm('Rider wirklich herunterfahren?')) befehl('shutdown');
}

// Stream nach Abbruch (z. B. Neustart von xgonav) neu verbinden
document.getElementById('bild').onerror = function () {
  setTimeout(() => { this.src = 'stream?' + Date.now(); }, 2000);
};
status();
setInterval(status, 700);
</script>
</body>
</html>
