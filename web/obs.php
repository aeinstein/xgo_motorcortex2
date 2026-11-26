<?php
// Nur das Kamerabild, z. B. als OBS-Browserquelle (640 × 480): https://<rider>/obs.php?roh=1
// roh=1: Kamerabild ohne Overlay, sonst mit Overlay wie in der Oberfläche. Bilder über /ws mit Bestätigung
// je Bild (xgonav/web.py _ws); am Pi so teuer wie ein weiterer offener Tab der Oberfläche.
$roh = (($_GET['roh'] ?? '0') === '1') ? '?roh=1' : '';
header('Cache-Control: no-store');
?><!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rider Kamera</title>
<style>
  html, body { margin: 0; height: 100%; background: #000; overflow: hidden; }
  img { display: block; width: 100%; height: 100%; object-fit: contain; }
</style>
</head>
<body>
<img id="bild" alt="">
<script>
const bild = document.getElementById('bild');
const pfad = '<?= $roh ?>';
let url = null;
function verbinden() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws' + pfad);
  ws.binaryType = 'arraybuffer';
  ws.onmessage = (e) => {
    if (typeof e.data === 'string') return;
    const neu = URL.createObjectURL(new Blob([e.data.slice(4)], { type: 'image/jpeg' }));   // 4 Byte Bildnummer
    const alt = url;
    url = neu;
    bild.src = neu;
    if (alt) URL.revokeObjectURL(alt);
    ws.send('ok');
  };
  ws.onclose = () => setTimeout(verbinden, 1000);   // Bildverarbeitung aus/Neustart: letztes Bild bleibt stehen
}
verbinden();
</script>
</body>
</html>
