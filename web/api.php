<?php
/*
 * xgonav – Status und Befehle an die lokale xgonav-Schnittstelle weiterreichen.
 *
 *   GET  api.php?a=status
 *   GET  api.php?a=sys&since=<seq>            (Verlauf: IMU-Proben nach seq, CPU je Kern roh, Temperatur, Akku;
 *                                              direkt aus /proc, läuft auch ohne xgonav)
 *   POST api.php?a=mode&name=follow&target=person
 *   POST api.php?a=map_reset                  (Raumkarte leeren, Pose und Start auf 0)
 *   POST api.php?a=map_start                  (Startpunkt = aktuelle Pose)
 *   POST api.php?a=map_save                   (Raumkarte sofort speichern, [grid] file)
 *   POST api.php?a=stop
 *   POST api.php?a=pipeline&on=0|1
 *   POST api.php?a=drive&v=-1..1&w=-1..1      (nur Modus manual, alle 0,2 s senden)
 *   POST api.php?a=led&i=0..3&c=%23rrggbb
 *   POST api.php?a=action&n=1..6|253|254|255 (R-1.1.6+Patch: 255 = Arm/aufstehen, 254 = Disarm/Not-Stopp,
 *                                            253 = Aufspringen aus dem Liegen (!standup); 1..6 Aktionen)
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
const MODES = ['idle', 'manual', 'follow', 'explore', 'approach', 'home'];
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

// Messwerte für die Verlaufsanzeige. CPU-Zeiten roh (Jiffies), die Auslastung rechnet der
// Browser aus der Differenz zweier Abfragen – so bleibt PHP zustandslos.
function sys(int $since): array
{
    $imu = [];
    $seq = 0;                                               // neueste seq; < since = Treiber neu geladen
    // imu_log: "seq t_ns roll pitch yaw" (Milligrad), letzte 128 Proben (~2,5 s bei 50 Hz)
    foreach (@file(PROC . '/imu_log', FILE_IGNORE_NEW_LINES) ?: [] as $z) {
        $p = explode(' ', $z);
        if (count($p) !== 5) {
            continue;
        }
        $seq = max($seq, (int)$p[0]);
        if ((int)$p[0] <= $since) {
            continue;
        }
        $imu[] = [(int)$p[0], round((int)$p[1] / 1e6, 1),
                  round((int)$p[2] / 1000, 1), round((int)$p[3] / 1000, 1), round((int)$p[4] / 1000, 1)];
    }
    $cpu = [];
    foreach (@file('/proc/stat', FILE_IGNORE_NEW_LINES) ?: [] as $z) {
        if (!preg_match('/^cpu\d+\s/', $z)) {
            continue;
        }
        $f = array_map('intval', array_slice(preg_split('/\s+/', trim($z)), 1, 8));
        $cpu[] = [array_sum($f), $f[3] + $f[4]];             // [gesamt, idle + iowait]
    }
    $temp = @file_get_contents('/sys/class/thermal/thermal_zone0/temp');
    $load = @file_get_contents('/proc/loadavg');
    $akku = @file_get_contents(PROC . '/battery');
    return [
        't' => (int)round(microtime(true) * 1000),
        'seq' => $seq,
        'imu' => $imu,
        'cpu' => $cpu,
        'temp_c' => is_numeric(trim((string)$temp)) ? round((int)$temp / 1000, 1) : null,
        'load' => $load ? (float)explode(' ', $load)[0] : null,
        'battery' => is_numeric(trim((string)$akku)) ? (int)$akku : null,
    ];
}

$a = $_GET['a'] ?? '';
$post = ($_SERVER['REQUEST_METHOD'] ?? '') === 'POST';

if ($a === 'status') {
    xgonav('GET', '/status');
}
if ($a === 'sys') {
    antwort(200, json_encode(sys((int)($_GET['since'] ?? 0))));
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
    case 'map_reset':
        xgonav('POST', '/map/reset');
    case 'map_start':
        xgonav('POST', '/map/start');
    case 'map_save':
        xgonav('POST', '/map/save');
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
        // 0 ist bei R-1.1.6 nur "keine Aktion" (kein Stopp) – nicht anbieten, sonst verwechselbar
        if ($n > 6 && $n !== 253 && $n !== 254 && $n !== 255) {
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
