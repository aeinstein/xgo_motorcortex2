<?php
/*
 * WLAN eintragen: Netz wählen, Passwort, verbinden (api.php?a=wlan_connect -> sudo xgo-wlan connect).
 * Gedacht für den Hotspot der WLAN-Einrichtung (Tasten C+D halten, kein QR-Code: https://10.42.0.1/wlan.php),
 * geht aber auch im normalen Betrieb ("WLAN hinzufügen"). Bestehende Profile bleiben; das neue hat Vorrang.
 * Keine Rechenzeit am Rider außer beim Aufruf (nmcli).
 */
declare(strict_types=1);
header('Cache-Control: no-store');
?><!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rider WLAN</title>
<style>
  :root { --bg: #f4f4f2; --fg: #1d1d1b; --leise: #6b6b66; --rand: #cfcfca; --akzent: #1f6feb; --ok: #1a7f37; --fehler: #c62828; }
  @media (prefers-color-scheme: dark) {
    :root { --bg: #161615; --fg: #ececea; --leise: #9a9a94; --rand: #3a3a37; --akzent: #58a6ff; --ok: #3fb950; --fehler: #ff6b6b; }
  }
  body { margin: 0; background: var(--bg); color: var(--fg); font: 16px/1.4 system-ui, sans-serif; }
  main { max-width: 420px; margin: 0 auto; padding: 20px 16px; }
  h1 { font-size: 1.3rem; margin: 0 0 4px; }
  p.leise { color: var(--leise); margin: 0 0 18px; font-size: .9rem; }
  label { display: block; margin: 14px 0 4px; font-weight: 600; }
  input, select, button { width: 100%; box-sizing: border-box; font: inherit; padding: 10px; border: 1px solid var(--rand);
                          border-radius: 8px; background: transparent; color: inherit; }
  button { margin-top: 18px; background: var(--akzent); border-color: var(--akzent); color: #fff; font-weight: 600; }
  button:disabled { opacity: .5; }
  .zeile { display: flex; gap: 8px; align-items: center; margin-top: 6px; color: var(--leise); font-size: .9rem; }
  .zeile input { width: auto; }
  #meldung { margin-top: 16px; min-height: 1.4em; }
  .ok { color: var(--ok); } .fehler { color: var(--fehler); }
  #jetzt { color: var(--leise); font-size: .9rem; margin-bottom: 8px; }
</style>
</head>
<body>
<main>
  <h1>Rider ins WLAN</h1>
  <p class="leise">Das Netz wird zusätzlich gespeichert und bevorzugt. Über den Hotspot verbunden: Nach „Verbinden“ geht
     der Hotspot zu, das Ergebnis steht am Display des Riders.</p>
  <div id="jetzt">Verbunden: …</div>
  <label for="netze">Netze in der Nähe</label>
  <select id="netze"><option value="">(wird geladen …)</option></select>
  <label for="ssid">Netzname (SSID)</label>
  <input id="ssid" autocomplete="off" autocapitalize="none" spellcheck="false" maxlength="32">
  <label for="pw">Passwort</label>
  <input id="pw" type="password" autocomplete="off" autocapitalize="none" spellcheck="false" maxlength="63">
  <div class="zeile"><input type="checkbox" id="zeigen"><label for="zeigen" style="margin:0;font-weight:400">Passwort zeigen</label></div>
  <button id="los">Verbinden</button>
  <div id="meldung"></div>
</main>
<script>
const $ = id => document.getElementById(id);
function melde(text, klasse) { $('meldung').textContent = text; $('meldung').className = klasse || ''; }

async function status() {
  try {
    const r = await (await fetch('api.php?a=wlan_status')).json();
    $('jetzt').textContent = r.con ? `Verbunden: ${r.con === 'xgo-hotspot' ? 'Hotspot des Riders' : (r.ssid || r.con)}${r.ip ? ' · ' + r.ip : ''}`
                                   : 'Verbunden: –';
  } catch (e) { $('jetzt').textContent = ''; }
}

async function netze() {
  const sel = $('netze');
  try {
    const r = await (await fetch('api.php?a=wlan_scan')).json();
    sel.innerHTML = '';
    sel.add(new Option(r.netze.length ? '– auswählen –' : '(keine gefunden, Namen eintippen)', ''));
    for (const n of r.netze) {
      sel.add(new Option(`${n.ssid}  ·  ${n.signal} %${n.security ? '' : '  · offen'}`, n.ssid));
    }
  } catch (e) {
    sel.innerHTML = '';
    sel.add(new Option('(Liste nicht verfügbar)', ''));
  }
}

$('netze').onchange = () => { if ($('netze').value) { $('ssid').value = $('netze').value; $('pw').focus(); } };
$('zeigen').onchange = () => { $('pw').type = $('zeigen').checked ? 'text' : 'password'; };
$('los').onclick = async () => {
  const ssid = $('ssid').value, pw = $('pw').value;
  if (!ssid) { melde('Bitte einen Netznamen angeben.', 'fehler'); return; }
  if (pw && (pw.length < 8 || pw.length > 63)) { melde('Das Passwort hat 8 bis 63 Zeichen (leer nur bei offenen Netzen).', 'fehler'); return; }
  $('los').disabled = true;
  melde('Verbinde … (bis zu 45 s)');
  try {
    const resp = await fetch('api.php?a=wlan_connect', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                                         body: JSON.stringify({ ssid, pw }) });
    const r = await resp.json();
    if (r.hotspot) melde(`Der Rider wechselt jetzt in „${ssid}“, der Hotspot geht zu. Ergebnis und Adresse stehen am Display.`, 'ok');
    else if (resp.ok) melde(`Verbunden mit „${r.ssid || ssid}“${r.ip ? ', Adresse ' + r.ip : ''}.`, 'ok');
    else melde(r.error || 'Fehlgeschlagen.', 'fehler');
  } catch (e) {
    melde('Verbindung zum Rider getrennt. Läuft das Verbinden über den Hotspot, steht das Ergebnis am Display.', 'fehler');
  }
  $('los').disabled = false;
  status();
};
status();
netze();
</script>
</body>
</html>
