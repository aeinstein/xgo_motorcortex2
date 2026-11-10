<?php
/*
 * xgonav – Status und Befehle an die lokale xgonav-Schnittstelle weiterreichen.
 *
 *   GET  api.php?a=status                     (einmalig, z. B. nach Befehlen; laufend kommen Status, CPU, WLAN
 *                                              und IMU per WebSocket /xgonav/live von tools/live.php)
 *   POST api.php?a=mode&name=follow&target=person
 *   POST api.php?a=map_reset                  (Raumkarte leeren, Pose und Start auf 0)
 *   POST api.php?a=map_start                  (Startpunkt = aktuelle Pose)
 *   POST api.php?a=map_save                   (Raumkarte sofort speichern, [grid] file)
 *   POST api.php?a=map_lock&on=0|1            (Karte sperren: Fahrten ändern sie nicht mehr)
 *   GET  api.php?a=route                      (Route und Sperrzonen)
 *   POST api.php?a=route&pts=x,y;x,y&loop=0|1[&zones=x,y;x,y;x,y|...|&nozones=1]   (Meter im Karten-KS)
 *   GET  api.php?a=horizon                    (Horizonttabelle und Standreferenz, [horizon] enabled)
 *   POST api.php?a=horizon_edit&row=0..479|   (Linie zum Einstellen im Bild, leer = aus)
 *   POST api.php?a=horizon_cal&row=0..479     (Zeile für die aktuelle Körperhöhe speichern, nur im Stand)
 *   POST api.php?a=horizon_delete&h=<mm>      (Eintrag dieser Körperhöhe löschen)
 *   POST api.php?a=stop
 *   POST api.php?a=pipeline&on=0|1
 *   POST api.php?a=ruhe&on=0|1                (Ruhemodus: xgonav /ruhe + sudo xgo-ruhe on|off für CPU-Takt
 *                                            und Display; Wecken startet nichts, Rider bleibt liegen)
 *   POST api.php?a=record&on=0|1              (Rohaufzeichnung; schreibt nur, solange der Rider balanciert)
 *   POST api.php?a=drive&v=-1..1&w=-1..1      (nur Modus manual, alle 0,2 s senden)
 *   POST api.php?a=led&i=0..3&c=%23rrggbb
 *   POST api.php?a=led_brightness&p=0..100    (Prozent; n = p·255/100 nach /proc und dauerhaft nach LED_FILE,
 *                                            das rc.local beim Booten und xgonav beim Start setzen)
 *   POST api.php?a=action&n=1..6|253|254|255 (R-1.1.6+Patch: 255 = Arm/aufstehen, 254 = Disarm/Not-Stopp,
 *                                            253 = Aufspringen aus dem Liegen (!standup); 1..6 Aktionen)
 *   POST api.php?a=height&h=61..255         (u8; darunter nicht, siehe Höhengrenzen)
 *   POST api.php?a=ausrichten               (Beine neu ausrichten, xgonav /level)
 *   POST api.php?a=roll&deg=-20..20          (Grad, + = nach rechts; Treiber-Patch 0004: Register 0x36)
 *   POST api.php?a=exposure&us=0|100..33333     (Belichtung in µs, 0 = Automatik; nur bis zum Neustart)
 *   POST api.php?a=gain&g=0|1..63.9             (analoge Verstärkung, 0 = Automatik; nur bis zum Neustart)
 *   POST api.php?a=calibration&on=0|1       (Beinnullpunkte: 1 = Start, Motoren los; 0 = speichern)
 *   POST api.php?a=pitch_calibration         (Lage: Firmware balanciert im Stand aus und speichert den Trimm)
 *   POST api.php?a=emotion&name=...           (Bildfolge aufs Display)
 *   POST api.php?a=shutdown                   (sudo-Regel: www-data -> systemctl poweroff)
 *   (sudo-Regel deploy/sudoers-xgonav-ruhe: www-data -> /usr/local/sbin/xgo-ruhe on|off)
 *
 * Fahren geht über xgonav (sonst überschreibt dessen Stillstand die Befehle); LEDs,
 * Aktionen, Höhe, Roll und Kalibrierung schreibt PHP direkt nach /proc/XGORider.
 *
 * xgonav lauscht nur auf 127.0.0.1:8088 (config.toml: [output] web_host). Keine
 * Authentifizierung – nur im vertrauenswürdigen Netz betreiben.
 */
declare(strict_types=1);

const XGONAV = 'http://127.0.0.1:8088';
const MODES = ['idle', 'manual', 'follow', 'explore', 'approach', 'home', 'patrol', 'pet'];
const PROC = '/proc/XGORider';
const EMOTION = '/opt/xgonav/tools/emotion.sh';         // Player (RGB565-Rohbilder statt ffmpeg)
const EMOTION_RAW = '/opt/xgonav/emotions';             // tools/eyes.py, von deploy/install.sh erzeugt
const HEIGHT_MIN = 61;                                   // u8; ~10 mm über dem Minimum, dazu ~4 mm fürs Ausrichten (ein Bein tiefer)
const LED_FILE = '/var/lib/xgonav/led_brightness';      // n/256 (Firmware-Patch 0016), deploy/install.sh legt den Ordner an
const HEIGHT_MAX = 205;                                 // u8; Firmware nähme 255, bei 235 Stopp 252 (2026-11-06, Ursache offen)

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

// xgonav aufrufen, ohne abzubrechen; HTTP-Code oder 0, wenn xgonav nicht läuft.
function xgonav_versuch(string $path): int
{
    $body = @file_get_contents(XGONAV . $path, false, stream_context_create(['http' => [
        'method' => 'POST', 'timeout' => 3, 'ignore_errors' => true, 'header' => "Content-Length: 0\r\n",
    ]]));
    if ($body === false || !isset($http_response_header[0])
        || !preg_match('#^HTTP/\S+\s+(\d{3})#', $http_response_header[0], $m)) {
        return 0;
    }
    return (int)$m[1];
}

// Ruhemodus, root-Teil (CPU-Takt, Display); true bei Erfolg.
function ruhe_system(bool $an, array &$fehler): bool
{
    exec('sudo -n /usr/local/sbin/xgo-ruhe ' . ($an ? 'on' : 'off') . ' 2>&1', $out, $rc);
    if ($rc !== 0) {
        $fehler[] = 'xgo-ruhe: ' . implode(' ', $out);
    }
    return $rc === 0;
}

$a = $_GET['a'] ?? '';
$post = ($_SERVER['REQUEST_METHOD'] ?? '') === 'POST';

if ($a === 'status') {
    xgonav('GET', '/status');
}
if ($a === 'route' && !$post) {
    xgonav('GET', '/route');
}
if ($a === 'horizon' && !$post) {
    xgonav('GET', '/horizon');
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
        xgonav('POST', '/mode?' . http_build_query(['name' => $name, 'target' => $target,
                                                    'bestaetigt' => ($_GET['bestaetigt'] ?? '') === '1' ? '1' : '0']));
    case 'map_reset':
        xgonav('POST', '/map/reset');
    case 'map_start':
        xgonav('POST', '/map/start');
    case 'map_save':
        xgonav('POST', '/map/save');
    case 'map_lock':
        xgonav('POST', '/map/lock?on=' . (($_GET['on'] ?? '1') === '0' ? '0' : '1'));
    case 'route':
        // Nur Zahlen und Trenner; Grenzen und Erreichbarkeit prüft xgonav
        $q = ['pts' => $_GET['pts'] ?? '', 'loop' => ($_GET['loop'] ?? '0') === '1' ? '1' : '0'];
        if (!preg_match('/^[-0-9.,;]{0,2000}$/', $q['pts'])) {
            antwort(400, json_encode(['error' => 'Route ungültig']));
        }
        if (isset($_GET['zones'])) {
            if (!preg_match('/^[-0-9.,;|]{0,2000}$/', $_GET['zones'])) {
                antwort(400, json_encode(['error' => 'Sperrzone ungültig']));
            }
            $q['zones'] = $_GET['zones'];
        } elseif (isset($_GET['nozones'])) {
            $q['nozones'] = '1';
        }
        xgonav('POST', '/route?' . http_build_query($q));
    case 'horizon_edit':
        $row = $_GET['row'] ?? '';
        xgonav('POST', '/horizon/edit?row=' . ($row === '' ? '' : round(zahl('row', 0, 479.9), 1)));
    case 'horizon_cal':
        xgonav('POST', '/horizon/cal?row=' . round(zahl('row', 0, 479.9), 1));
    case 'horizon_delete':
        xgonav('POST', '/horizon/delete?h=' . round(zahl('h', 0, 300), 1));
    case 'stop':
        xgonav('POST', '/stop');
    case 'ausrichten':
        xgonav('POST', '/level');
    case 'pipeline':
        $on = ($_GET['on'] ?? '1') === '0' ? '0' : '1';
        xgonav('POST', '/pipeline?on=' . $on);
    case 'ruhe':
        // Einschlafen: erst xgonav (setzt ab, Abfragen aus), dann CPU-Takt runter. Wecken umgekehrt: erst
        // CPU-Takt hoch. Beides wird versucht, auch wenn ein Teil fehlschlägt (xgonav läuft evtl. nicht).
        $an = ($_GET['on'] ?? '1') !== '0';
        $fehler = [];
        if ($an) {
            $code = xgonav_versuch('/ruhe?on=1');
            $sys = ruhe_system(true, $fehler);
        } else {
            $sys = ruhe_system(false, $fehler);
            $code = xgonav_versuch('/ruhe?on=0');
        }
        if ($code !== 200) {
            $fehler[] = $code === 0 ? 'xgonav nicht erreichbar' : "xgonav: HTTP $code";
        }
        antwort(200, json_encode(['ok' => $fehler === [], 'ruhe' => $an, 'xgonav' => $code === 200,
                                  'system' => $sys, 'fehler' => $fehler]));
    case 'record':
        xgonav('POST', '/record?on=' . (($_GET['on'] ?? '1') === '0' ? '0' : '1'));
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
    case 'led_brightness':
        $n = (int)round(zahl('p', 0, 100) * 255 / 100);
        // Erst speichern (über eine Hilfsdatei, damit rc.local/xgonav nie eine halbe Datei lesen), dann setzen
        $tmp = LED_FILE . '.neu';
        if (@file_put_contents($tmp, "$n\n") === false || !@rename($tmp, LED_FILE)) {
            antwort(500, json_encode(['error' => 'Speichern nach ' . LED_FILE . ' fehlgeschlagen (Ordner da, www-data in Gruppe video?)']));
        }
        proc('settings/led_brightness', (string)$n);
    case 'action':
        $n = (int)zahl('n', 1, 255);
        // 0 ist bei R-1.1.6 nur "keine Aktion" (kein Stopp) – nicht anbieten, sonst verwechselbar
        if ($n > 6 && $n !== 253 && $n !== 254 && $n !== 255) {
            antwort(400, json_encode(['error' => 'Aktion ungültig']));
        }
        if ($n === 254) {
            // Die Aktion geht direkt nach /proc, xgonav erfährt davon nichts. Der Modus pet stünde
            // nach einer Ohnmacht sonst von selbst wieder auf. Fehlschlag egal: dann läuft xgonav nicht.
            @file_get_contents(XGONAV . '/stop?nur=pet', false, stream_context_create(['http' => [
                'method' => 'POST', 'timeout' => 1, 'ignore_errors' => true, 'header' => "Content-Length: 0\r\n",
            ]]));
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
        if (!preg_match('/^[A-Za-z]+$/', $name) || !is_dir(EMOTION_RAW . '/' . $name)) {
            antwort(400, json_encode(['error' => 'Emotion unbekannt']));
        }
        if (!is_writable('/tmp/lcd0')) {    // FIFO des Display-Daemons, Gruppe video
            antwort(500, json_encode(['error' => 'Display nicht beschreibbar (www-data in Gruppe video?)']));
        }
        // laufende Emotion abbrechen (Player und ggf. ffmpeg-Fallback bzw. altes showEmotion)
        exec('pkill -u www-data -f "[e]motion.sh play|[f]fmpeg.*/tmp/lcd0" 2>/dev/null');
        exec(sprintf('%s play %s > /dev/null 2>&1 &', escapeshellarg(EMOTION), escapeshellarg(EMOTION_RAW . '/' . $name)));
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
