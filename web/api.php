<?php
/*
 * xgonav – Status und Befehle an die lokale xgonav-Schnittstelle weiterreichen.
 *
 *   GET  api.php?a=status
 *   POST api.php?a=mode&name=follow&target=person
 *   POST api.php?a=stop
 *   POST api.php?a=pipeline&on=0|1
 *   POST api.php?a=drive&v=-1..1&w=-1..1      (nur Modus manual, alle 0,2 s senden)
 *   POST api.php?a=led&i=0..3&c=%23rrggbb
 *   POST api.php?a=action&n=1..6|255
 *   POST api.php?a=height&h=43..255         (u8; unter 43 nicht, siehe Höhengrenzen)
 *   POST api.php?a=roll&deg=-20..20          (Grad, + = nach rechts; Treiber-Patch 0004: Register 0x36)
 *   POST api.php?a=exposure&us=0|100..33333     (Belichtung in µs, 0 = Automatik; nur bis zum Neustart)
 *   POST api.php?a=gain&g=0|1..16               (analoge Verstärkung, 0 = Automatik; nur bis zum Neustart)
 *   POST api.php?a=calibration&on=0|1       (Beinnullpunkte: 1 = Start, Motoren los; 0 = speichern)
 *   POST api.php?a=pitch_calibration         (Lage: Firmware balanciert im Stand aus und speichert den Trimm)
 *   POST api.php?a=emotion&name=...           (Bildfolge aufs Display)
 *   POST api.php?a=shutdown                   (sudo-Regel: www-data -> systemctl poweroff)
 *
 * Fahren geht über xgonav (sonst überschreibt dessen Stillstand die Befehle); LEDs,
 * Aktionen, Höhe, Roll und Kalibrierung schreibt PHP direkt nach /proc/XGORider.
 *
 * xgonav lauscht nur auf 127.0.0.1:8088 (config.toml: [output] web_host). Keine
 * Authentifizierung – nur im vertrauenswürdigen Netz betreiben.
 */
declare(strict_types=1);

const XGONAV = 'http://127.0.0.1:8088';
const MODES = ['idle', 'manual', 'follow', 'explore', 'approach'];
const PROC = '/proc/XGORider';
const RC = '/home/aeinstein/motorcortex/rc_server';     // Emotionen der früheren Fernsteuerung
const EMOTION = '/opt/xgonav/tools/emotion.sh';         // Player (RGB565-Rohbilder statt ffmpeg)
const EMOTION_RAW = '/opt/xgonav/emotions';             // von deploy/install.sh vorkonvertiert
const HEIGHT_MIN = 43;                                   // u8; ~10 mm über dem Minimum
const HEIGHT_MAX = 255;                                 // volle Höhe (Firmware 0..255)

header('Content-Type: application/json; charset=utf-8');
header('Cache-Control: no-store');

function antwort(int $code, string $body): never
{
    http_response_code($code);
    echo $body;
    exit;
}

function proc(string $datei, string $wert): never
{
    if (@file_put_contents(PROC . '/' . $datei, $wert) === false) {
        antwort(500, json_encode(['error' => 'Schreiben nach ' . $datei . ' fehlgeschlagen']));
    }
    antwort(200, json_encode(['ok' => true]));
}

function zahl(string $name, float $min, float $max): float
{
    $v = $_GET[$name] ?? '';
    if (!is_numeric($v) || (float)$v < $min || (float)$v > $max) {
        antwort(400, json_encode(['error' => "$name ungültig"]));
    }
    return (float)$v;
}

function xgonav(string $method, string $path): never
{
    $ctx = stream_context_create(['http' => [
        'method' => $method,
        'timeout' => 3,
        'ignore_errors' => true,          // auch 4xx/5xx-Antworten von xgonav durchreichen
        'header' => "Content-Length: 0\r\n",
    ]]);
    $body = @file_get_contents(XGONAV . $path, false, $ctx);
    if ($body === false) {
        antwort(502, json_encode(['error' => 'xgonav nicht erreichbar (läuft der Dienst?)']));
    }
    $code = 502;
    if (isset($http_response_header[0]) && preg_match('#^HTTP/\S+\s+(\d{3})#', $http_response_header[0], $m)) {
        $code = (int)$m[1];
    }
    antwort($code, $body);
}

$a = $_GET['a'] ?? '';
$post = ($_SERVER['REQUEST_METHOD'] ?? '') === 'POST';

if ($a === 'status') {
    xgonav('GET', '/status');
}
if (!$post) {
    antwort(405, json_encode(['error' => 'Befehle nur per POST']));
}
switch ($a) {
    case 'mode':
        $name = $_GET['name'] ?? '';
        if (!in_array($name, MODES, true)) {
            antwort(400, json_encode(['error' => 'unbekannter Modus']));
        }
        $target = $_GET['target'] ?? 'person';
        if (!preg_match('/^[a-z ]{1,30}$/', $target)) {    // COCO-Klassen: Kleinbuchstaben, Leerzeichen
            antwort(400, json_encode(['error' => 'ungültiges Ziel']));
        }
        xgonav('POST', '/mode?' . http_build_query(['name' => $name, 'target' => $target]));
    case 'stop':
        xgonav('POST', '/stop');
    case 'pipeline':
        $on = ($_GET['on'] ?? '1') === '0' ? '0' : '1';
        xgonav('POST', '/pipeline?on=' . $on);
    case 'drive':
        $v = zahl('v', -1, 1);
        $w = zahl('w', -1, 1);
        xgonav('POST', '/drive?' . http_build_query(['v' => $v, 'w' => $w]));
    case 'exposure':
        xgonav('POST', '/exposure?us=' . (int)zahl('us', 0, 1000000));   // Grenzen prüft xgonav
    case 'gain':
        xgonav('POST', '/gain?g=' . round(zahl('g', 0, 100), 2));          // Grenzen prüft xgonav
    case 'led':
        $i = (int)zahl('i', 0, 3);
        $c = $_GET['c'] ?? '';
        if (!preg_match('/^#[0-9a-fA-F]{6}$/', $c)) {
            antwort(400, json_encode(['error' => 'Farbe ungültig']));
        }
        proc("leds/$i", strtolower($c));
    case 'action':
        $n = (int)zahl('n', 1, 255);
        if ($n > 6 && $n !== 255) {
            antwort(400, json_encode(['error' => 'Aktion ungültig']));
        }
        proc('action', (string)$n);
    case 'height':
        proc('height', (string)(int)zahl('h', HEIGHT_MIN, HEIGHT_MAX));
    case 'roll':
        proc('roll', (string)(int)round(zahl('deg', -20, 20)));
    case 'calibration':
        proc('settings/calibration', ($_GET['on'] ?? '0') === '1' ? '1' : '0');
    case 'pitch_calibration':
        proc('settings/pitch_calibration', '1');
    case 'emotion':
        $name = $_GET['name'] ?? '';
        if (!preg_match('/^[A-Za-z]+$/', $name) || !is_dir(RC . '/emotions/' . $name)) {
            antwort(400, json_encode(['error' => 'Emotion unbekannt']));
        }
        if (!is_writable('/tmp/lcd0')) {    // FIFO des Display-Daemons, Gruppe video
            antwort(500, json_encode(['error' => 'Display nicht beschreibbar (www-data in Gruppe video?)']));
        }
        // laufende Emotion abbrechen (Player und ggf. ffmpeg-Fallback bzw. altes showEmotion)
        exec('pkill -u www-data -f "[e]motion.sh play|[f]fmpeg.*/tmp/lcd0" 2>/dev/null');
        // Vorkonvertierte Bilder, sonst spielt der Player die PNGs mit ffmpeg ab.
        $verz = is_dir(EMOTION_RAW . '/' . $name) ? EMOTION_RAW . '/' . $name : RC . '/emotions/' . $name;
        exec(sprintf('%s play %s > /dev/null 2>&1 &', escapeshellarg(EMOTION), escapeshellarg($verz)));
        antwort(200, json_encode(['ok' => true]));
    case 'shutdown':
        // Erst prüfen, ob poweroff erlaubt ist, dann "sleep" abspielen (blockiert ~4 s), dann aus.
        exec('sudo -n -l /usr/bin/systemctl poweroff 2>&1', $out, $rc);
        if ($rc === 0 && is_dir(EMOTION_RAW . '/sleep') && is_writable('/tmp/lcd0')) {
            exec('pkill -u www-data -f "[e]motion.sh play|[f]fmpeg.*/tmp/lcd0" 2>/dev/null');
            exec(sprintf('timeout 10 %s play %s > /dev/null 2>&1', escapeshellarg(EMOTION),
                         escapeshellarg(EMOTION_RAW . '/sleep')));
        }
        $out = [];
        exec('sudo -n /usr/bin/systemctl poweroff 2>&1', $out, $rc);
        antwort($rc === 0 ? 200 : 500, json_encode($rc === 0 ? ['ok' => true] : ['error' => 'Herunterfahren nicht erlaubt: ' . implode(' ', $out)]));
    default:
        antwort(400, json_encode(['error' => 'unbekannte Aktion']));
}
