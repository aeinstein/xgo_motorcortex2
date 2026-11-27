<?php
/*
 * xgonav – Status und Befehle an die lokale xgonav-Schnittstelle weiterreichen.
 *
 *   GET  api.php?a=status                     (einmalig, z. B. nach Befehlen; laufend kommen Status, CPU, WLAN
 *                                              und IMU per WebSocket /live von tools/live.php)
 *   POST api.php?a=mode&name=follow&target=person
 *   POST api.php?a=map_reset                  (Raumkarte leeren, Pose und Start auf 0)
 *   POST api.php?a=map_start                  (Startpunkt = aktuelle Pose)
 *   POST api.php?a=map_save                   (Raumkarte sofort speichern, [grid] file)
 *   POST api.php?a=map_lock&on=0|1            (Karte sperren: Fahrten ändern sie nicht mehr)
 *   GET  api.php?a=route                      (Route und Sperrzonen)
 *   POST api.php?a=route&pts=x,y;x,y&loop=0|1[&zones=x,y;x,y;x,y|...|&nozones=1]   (Meter im Karten-KS)
 *   GET  api.php?a=detector                   (wählbare Ziele für Folgen/Anfahren: aktive YOLO-Klassen)
 *   GET  api.php?a=floornet                   (Bodenmodelle in [freespace] net_dir, aktives, geladenes)
 *   POST api.php?a=floornet&name=x.bin        (Bodenmodell wechseln: xgonav lädt und prüft es, stellt den Symlink
 *                                            [freespace] net um und tauscht das Netz ohne Neustart; nur idle/manual)
 *   POST api.php?a=stop
 *   POST api.php?a=pipeline&on=0|1
 *   POST api.php?a=ruhe&on=0|1                (Ruhemodus: xgonav /ruhe, Schlafaugen aufs Display (bleiben
 *                                            stehen), sudo xgo-ruhe on|off für CPU-Takt + WLAN-Stromsparen; Wecken startet
 *                                            nichts, Rider bleibt liegen)
 *   POST api.php?a=record&on=0|1              (Rohaufzeichnung; schreibt nur, solange der Rider balanciert)
 *   POST api.php?a=drive&v=-1..1&w=-1..1      (nur Modus manual, alle 0,2 s senden)
 *   POST api.php?a=tempo&f=0.05..1            (Temporegler: Anteil für alle Fahr-/Drehbefehle aller Modi)
 *   POST api.php?a=pursuit&on=0|1             (Fahrmodus Pursuit: Beschleunigung +50 %, Einlegen, Höhe sinkt mit dem Tempo)
 *   POST api.php?a=bild&k=0.5..4&s=0.5..4&b=-0.5..0.5  (Kontrast/Sättigung/Helligkeit im ISP; Grenzen prüft xgonav)
 *   POST api.php?a=led&i=0..3&c=%23rrggbb
 *   POST api.php?a=led_brightness&p=0..100    (Prozent; n = p·255/100 nach /proc und dauerhaft nach LED_FILE,
 *                                            das rc.local beim Booten und xgonav beim Start setzen)
 *   POST api.php?a=action&n=1..6|237|253|254|255 (R-1.1.6+Patch: 255 = Arm/aufstehen, 254 = Disarm/Not-Stopp,
 *                                            253 = Aufspringen aus dem Liegen (!standup), 237 = ESP32 neu
 *                                            starten (nur wenn er nicht balanciert); 1..6 Aktionen, davon
 *                                            3..5 fahren selbst: über xgonav /action, nur im Modus idle,
 *                                            Bahnprüfung bleibt)
 *   POST api.php?a=height&h=61..255         (u8; darunter nicht, siehe Höhengrenzen)
 *   POST api.php?a=ausrichten               (Beine neu ausrichten, xgonav /level)
 *   POST api.php?a=roll&deg=-20..20          (Grad, + = nach rechts; Treiber-Patch 0004: Register 0x36)
 *   POST api.php?a=exposure&us=0|100..max     (Belichtung in µs, 0 = Automatik; nur bis zum Neustart)
 *   POST api.php?a=gain&g=0|1..63.9             (analoge Verstärkung, 0 = Automatik; nur bis zum Neustart)
 *   POST api.php?a=calibration&on=0|1       (Beinnullpunkte: 1 = Start, Motoren los; 0 = speichern)
 *   POST api.php?a=pitch_calibration         (Lage: Firmware balanciert im Stand aus und speichert den Trimm)
 *   POST api.php?a=emotion&name=...           (Bildfolge aufs Display)
 *   POST api.php?a=shutdown                   (sudo-Regel: www-data -> systemctl poweroff)
 *   GET  api.php?a=wlan_status                (wlan0: Verbindung, SSID, IP; sudo xgo-wlan status)
 *   GET  api.php?a=wlan_scan                  (sichtbare Netze aus dem Cache; sudo xgo-wlan scan)
 *   POST api.php?a=wlan_connect               (Body JSON {"ssid", "pw"}: Profil anlegen und verbinden, web/wlan.php;
 *                                            im Hotspot sofort antworten, das Verbinden schließt ihn)
 *   POST api.php?a=wlan&on=0|1                (WLAN-Einrichtung in xgonav wie Tasten C+D: QR-Code; den Hotspot
 *                                            öffnet xgo-wlan-auto.service von selbst)
 *   (sudo-Regel deploy/sudoers-xgonav-wlan: www-data -> /usr/local/sbin/xgo-wlan connect|status|scan)
 *   (sudo-Regel deploy/sudoers-xgonav-ruhe: www-data -> /usr/local/sbin/xgo-ruhe on|off)
 *   GET  api.php?a=config                     (Einstellungen: Schema, Werte aus config.toml, Sicherungen)
 *   POST api.php?a=config_save&hash=<sha1>    (Body JSON {"abschnitt.key": Wert|null}; null = Default.
 *                                            Prüft mit Config.load, sichert nach CONFIG_BACKUP, wirkt nach Neustart)
 *   POST api.php?a=config_restore&name=<datei>&hash=<sha1>   (Sicherung zurückspielen; aktuelle wird vorher gesichert)
 *   POST api.php?a=restart[&trotzdem=1]       (xgonav neu starten; 409, wenn er fährt, balanciert oder aufzeichnet;
 *                                            im Ruhemodus nie. sudo-Regel deploy/sudoers-xgonav-restart)
 *
 * Fahren geht über xgonav (sonst überschreibt dessen Stillstand die Befehle); LEDs,
 * Aktionen, Höhe, Roll und Kalibrierung schreibt PHP direkt nach /proc/XGORider.
 *
 * xgonav lauscht nur auf 127.0.0.1:8088 (config.toml: [output] web_host). Keine
 * Authentifizierung – nur im vertrauenswürdigen Netz betreiben.
 */
declare(strict_types=1);

// Am Rider ohne Umgebungsvariablen. Virtueller Rider (tools/sim_rider.py, php-fpm-Pool auf kimmy): XGO_NAV =
// xgonav des Simulators, XGO_PROC = Verzeichnis statt /proc, XGO_SIM=1 sperrt alles, was den Rechner selbst träfe.
define('XGONAV', getenv('XGO_NAV') ?: 'http://127.0.0.1:8088');
const MODES = ['idle', 'manual', 'follow', 'explore', 'approach', 'home', 'patrol', 'pet'];
define('PROC', getenv('XGO_PROC') ?: '/proc/XGORider');
define('SIM', getenv('XGO_SIM') === '1');
const EMOTION = '/opt/xgonav/tools/emotion.sh';         // Player (RGB565-Rohbilder statt ffmpeg)
const EMOTION_RAW = '/opt/xgonav/emotions';             // tools/eyes.py, von deploy/install.sh erzeugt
const STIMMEN = '/opt/xgonav/stimmen';                 // [stimme] dir: bestaetigt/NN(_roh).wav, kandidaten/<Zeit>.wav
const HEIGHT_MIN = 61;                                   // u8; ~10 mm über dem Minimum, dazu ~4 mm fürs Ausrichten (ein Bein tiefer)
const LED_FILE = '/var/lib/xgonav/led_brightness';      // n/256 (Firmware-Patch 06-leds), deploy/install.sh legt den Ordner an
const HEIGHT_MAX = 255;                                 // u8; seit Firmware 5a80b9ea (2026-11-27) fährt jedes Bein nur bis zur Servogrenze, 255 = so hoch wie möglich
const CONFIG = '/opt/xgonav/config.toml';               // Gruppe video, 0664 (deploy/install.sh)
const CONFIG_SCHEMA = __DIR__ . '/config-schema.json';  // tools/config_schema.py
const CONFIG_BACKUP = '/var/lib/xgonav/config-backup';  // je Speichern eine Sicherung, die letzten 50 bleiben
const CONFIG_PRUEF = '/opt/xgonav/venv/bin/python';     // Config.load wie beim Start von xgonav

require __DIR__ . '/konfig.inc.php';

// Ortszeit des Riders für Sicherungsnamen und Vermerke (php-fpm nimmt sonst UTC)
$tz = trim((string)@file_get_contents('/etc/timezone'));
if ($tz !== '' && in_array($tz, timezone_identifiers_list(), true)) {
    date_default_timezone_set($tz);
}

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

function xgonav(string $method, string $path, int $timeout = 3, ?string $body = null, string $ctype = ''): never
{
    $http = [
        'method' => $method,
        'timeout' => $timeout,
        'ignore_errors' => true,          // auch 4xx/5xx-Antworten von xgonav durchreichen
        'header' => "Content-Length: 0\r\n",
    ];
    if ($body !== null) {                 // Body weiterreichen (Stimme hochladen)
        $http['header'] = 'Content-Length: ' . strlen($body) . "\r\n" . ($ctype !== '' ? "Content-Type: $ctype\r\n" : '');
        $http['content'] = $body;
    }
    $ctx = stream_context_create(['http' => $http]);
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

// Schlafaugen abspielen und warten (~4 s, höchstens 10 s); das letzte Bild (Augen zu) bleibt am Display stehen.
function schlafaugen(): void
{
    if (!is_dir(EMOTION_RAW . '/sleep') || !is_writable('/tmp/lcd0')) {
        return;
    }
    exec('pkill -u www-data -f "[e]motion.sh play|[f]fmpeg.*/tmp/lcd0" 2>/dev/null');
    exec(sprintf('timeout 10 %s play %s > /dev/null 2>&1', escapeshellarg(EMOTION),
                 escapeshellarg(EMOTION_RAW . '/sleep')));
}

// Ruhemodus, root-Teil (CPU-Takt, WLAN-Stromsparen); true bei Erfolg.
function ruhe_system(bool $an, array &$fehler): bool
{
    if (SIM) {
        return true;                         // virtueller Rider: kein Takt/WLAN des Rechners anfassen
    }
    exec('sudo -n /usr/local/sbin/xgo-ruhe ' . ($an ? 'on' : 'off') . ' 2>&1', $out, $rc);
    if ($rc !== 0) {
        $fehler[] = 'xgo-ruhe: ' . implode(' ', $out);
    }
    return $rc === 0;
}

// ---------------------------------------------------------------- WLAN (deploy/xgo-wlan)
const XGO_WLAN = '/usr/local/sbin/xgo-wlan';

// Helfer per sudo, stdin wird übergeben (Passwort nie in der Kommandozeile). [rc, Zeilen]
function wlan_helfer(string $cmd, string $stdin = ''): array
{
    $p = proc_open(['sudo', '-n', XGO_WLAN, $cmd], [0 => ['pipe', 'r'], 1 => ['pipe', 'w'], 2 => ['pipe', 'w']], $pipes);
    if (!is_resource($p)) {
        return [127, ['Helfer nicht startbar']];
    }
    fwrite($pipes[0], $stdin);
    fclose($pipes[0]);
    $out = stream_get_contents($pipes[1]) . stream_get_contents($pipes[2]);
    fclose($pipes[1]);
    fclose($pipes[2]);
    $rc = proc_close($p);
    return [$rc, array_values(array_filter(explode("\n", trim($out)), fn($z) => $z !== ''))];
}

function wlan_kv(array $zeilen): array
{
    $d = [];
    foreach ($zeilen as $z) {
        $teil = explode('=', $z, 2);
        if (count($teil) === 2) {
            $d[$teil[0]] = $teil[1];
        }
    }
    return $d;
}

// ---------------------------------------------------------------- Einstellungen (config.toml)
function konfig_schema(): array
{
    $s = json_decode((string)@file_get_contents(CONFIG_SCHEMA), true);
    if (!is_array($s)) {
        antwort(500, json_encode(['error' => 'config-schema.json fehlt (python tools/config_schema.py)']));
    }
    return $s;
}

function konfig_text(): string
{
    $t = @file_get_contents(CONFIG);
    if ($t === false) {
        antwort(500, json_encode(['error' => CONFIG . ' nicht lesbar']));
    }
    return $t;
}

function konfig_sicherungen(): array
{
    $liste = [];
    foreach (glob(CONFIG_BACKUP . '/config.toml.*') ?: [] as $p) {
        $liste[] = ['name' => basename($p), 'zeit' => filemtime($p), 'bytes' => filesize($p)];
    }
    usort($liste, fn($x, $y) => strcmp($y['name'], $x['name']));
    return $liste;
}

// Aktuellen Stand sichern; die ältesten über 50 löschen. Gibt den Namen zurück.
function konfig_sichern(string $text): string
{
    if (!is_dir(CONFIG_BACKUP) && !@mkdir(CONFIG_BACKUP, 02775)) {
        antwort(500, json_encode(['error' => CONFIG_BACKUP . ' nicht anlegbar (www-data in Gruppe video?)']));
    }
    $name = 'config.toml.' . date('Ymd-His');
    for ($n = 2; file_exists(CONFIG_BACKUP . "/$name"); $n++) {
        $name = 'config.toml.' . date('Ymd-His') . "-$n";
    }
    if (@file_put_contents(CONFIG_BACKUP . "/$name", $text) === false) {
        antwort(500, json_encode(['error' => 'Sicherung nicht schreibbar']));
    }
    foreach (array_slice(konfig_sicherungen(), 50) as $alt) {
        @unlink(CONFIG_BACKUP . '/' . $alt['name']);
    }
    return $name;
}

// Neuen Text mit Config.load prüfen (wie beim Start), sichern, schreiben. Gibt den Namen der Sicherung zurück.
function konfig_schreiben(string $alt, string $neu): string
{
    $kandidat = CONFIG_BACKUP . '/kandidat.toml';
    if (!is_dir(CONFIG_BACKUP)) {
        @mkdir(CONFIG_BACKUP, 02775);
    }
    if (@file_put_contents($kandidat, $neu) === false) {
        antwort(500, json_encode(['error' => 'Prüfdatei nicht schreibbar (' . CONFIG_BACKUP . ')']));
    }
    $py = 'import sys; sys.path.insert(0, "/opt/xgonav"); from xgonav.config import Config; Config.load(sys.argv[1])';
    exec(sprintf('%s -I -c %s %s 2>&1', escapeshellarg(CONFIG_PRUEF), escapeshellarg($py), escapeshellarg($kandidat)),
         $out, $rc);
    @unlink($kandidat);
    if ($rc !== 0) {
        antwort(400, json_encode(['error' => 'Prüfung mit Config.load fehlgeschlagen: ' . (end($out) ?: "rc $rc")]));
    }
    $name = konfig_sichern($alt);
    // Ordner gehört aeinstein: kein Umbenennen möglich, deshalb an Ort und Stelle (Sicherung liegt vor)
    if (@file_put_contents(CONFIG, $neu, LOCK_EX) === false) {
        antwort(500, json_encode(['error' => CONFIG . ' nicht schreibbar (Gruppe video, 0664?)']));
    }
    return $name;
}

// Vom Browser geladener Stand muss noch aktuell sein (sonst hat jemand per ssh oder ein zweiter Browser geändert)
function konfig_hash_pruefen(string $text): void
{
    if (($_GET['hash'] ?? '') !== sha1($text)) {
        antwort(409, json_encode(['error' => 'config.toml wurde inzwischen geändert – Einstellungen neu laden']));
    }
}

// Stimmprobe: Name (NN = bestätigt, Zeitname = Kandidat) -> Pfad; ungültig = 400
function stimme_pfad(string $name, bool $roh): string
{
    if (preg_match('/^\d\d$/', $name)) {
        return STIMMEN . '/bestaetigt/' . $name . ($roh ? '_roh' : '') . '.wav';
    }
    if (preg_match('/^\d{8}-\d{6}-\d{3}x*$/', $name)) {
        return STIMMEN . '/kandidaten/' . $name . '.wav';
    }
    antwort(400, json_encode(['error' => 'Probe ungültig']));
}

// Dauer in s aus dem WAV-Kopf (write_wav: 44 Byte Kopf, 16 bit)
function wav_dauer(string $f): ?float
{
    $h = @file_get_contents($f, false, null, 0, 44);
    if ($h === false || strlen($h) < 44 || substr($h, 0, 4) !== 'RIFF') {
        return null;
    }
    $k = unpack('vch/Vrate', substr($h, 22, 6));
    $n = (filesize($f) ?: 44) - 44;
    return $k['rate'] > 0 && $k['ch'] > 0 ? round($n / (2 * $k['ch'] * $k['rate']), 2) : null;
}

$a = $_GET['a'] ?? '';
$post = ($_SERVER['REQUEST_METHOD'] ?? '') === 'POST';

// Virtueller Rider: nichts, was den Rechner selbst herunterfährt, neu startet, sein WLAN ändert oder Programme startet
if (SIM && in_array($a, ['shutdown', 'restart', 'wlan_status', 'wlan_scan', 'wlan_connect', 'emotion', 'config_save', 'config_restore'], true)) {
    antwort(409, json_encode(['error' => 'im Simulator nicht verfügbar']));
}

if ($a === 'config' && !$post) {
    $text = konfig_text();
    [$werte, $fehler] = konfig_werte(konfig_zerlegen($text));
    antwort(200, json_encode(['schema' => konfig_schema(), 'werte' => (object)$werte, 'unlesbar' => (object)$fehler,
                              'hash' => sha1($text), 'datei' => CONFIG, 'schreibbar' => is_writable(CONFIG),
                              'sicherungen' => konfig_sicherungen()],
                             JSON_UNESCAPED_UNICODE | JSON_PRESERVE_ZERO_FRACTION));
}

if ($a === 'status') {
    xgonav('GET', '/status');
}
if ($a === 'route' && !$post) {
    xgonav('GET', '/route');
}
if ($a === 'detector' && !$post) {
    xgonav('GET', '/detector');
}
if ($a === 'floornet' && !$post) {
    xgonav('GET', '/floornet');
}
if ($a === 'stimmen' && !$post) {
    // Kachel „Samples“: direkt von der Platte, auch ohne xgonav (kostet nur beim Aufklappen)
    $zaehler = json_decode((string)@file_get_contents(STIMMEN . '/bestaetigt/zaehler.json'), true) ?: [];
    $fest = json_decode((string)@file_get_contents(STIMMEN . '/bestaetigt/fest.json'), true) ?: [];   // hochgeladene
    $best = [];
    foreach (glob(STIMMEN . '/bestaetigt/[0-9][0-9]_roh.wav') ?: [] as $f) {
        $nn = substr(basename($f), 0, 2);
        $sittich = STIMMEN . "/bestaetigt/$nn.wav";
        [$n, $t] = $zaehler[$nn] ?? [null, filemtime($f)];
        $best[] = ['name' => $nn, 'n' => $n, 'zuletzt' => $t, 'angelegt' => filemtime($f), 's' => wav_dauer($f),
                   'sittich' => is_file($sittich), 's_sittich' => is_file($sittich) ? wav_dauer($sittich) : null,
                   'fest' => in_array($nn, $fest, true)];
    }
    $kand = [];
    foreach (glob(STIMMEN . '/kandidaten/*.wav') ?: [] as $f) {
        $kand[] = ['name' => basename($f, '.wav'), 'zeit' => filemtime($f), 's' => wav_dauer($f)];
    }
    usort($kand, fn($x, $y) => $y['zeit'] <=> $x['zeit']);
    antwort(200, json_encode(['dir' => STIMMEN, 'bestaetigt' => $best, 'kandidaten' => $kand]));
}
if ($a === 'stimme_wav' && !$post) {
    $f = stimme_pfad($_GET['name'] ?? '', ($_GET['roh'] ?? '') === '1');
    if (!is_file($f)) {
        antwort(404, json_encode(['error' => 'Probe nicht vorhanden']));
    }
    header('Content-Type: audio/wav');
    header('Content-Length: ' . filesize($f));
    header('Cache-Control: no-cache');
    readfile($f);
    exit;
}
if ($a === 'wlan_status' && !$post) {
    [$rc, $z] = wlan_helfer('status');
    antwort($rc === 0 ? 200 : 500, json_encode($rc === 0 ? wlan_kv($z) : ['error' => end($z) ?: "rc $rc"],
                                               JSON_UNESCAPED_UNICODE));
}
if ($a === 'wlan_scan' && !$post) {
    [$rc, $z] = wlan_helfer('scan');
    $netze = [];
    foreach ($z as $zeile) {
        // SIGNAL:SECURITY:SSID, ":" in der SSID ist mit \ maskiert
        $f = preg_split('/(?<!\\\\):/', $zeile, 3);
        if (count($f) === 3 && $f[2] !== '') {
            $ssid = str_replace('\\:', ':', $f[2]);
            if (!isset($netze[$ssid]) || $netze[$ssid]['signal'] < (int)$f[0]) {
                $netze[$ssid] = ['ssid' => $ssid, 'signal' => (int)$f[0], 'security' => $f[1]];
            }
        }
    }
    usort($netze, fn($x, $y) => $y['signal'] <=> $x['signal']);
    antwort(200, json_encode(['netze' => array_values($netze)], JSON_UNESCAPED_UNICODE));
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
    case 'map_pose':
        $aktion = $_GET['aktion'] ?? '';
        if (!in_array($aktion, ['start', 'hier', 'verwerfen'], true)) {
            antwort(400, json_encode(['error' => 'unbekannte Aktion']));
        }
        xgonav('POST', '/map/pose?aktion=' . $aktion);
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
    case 'floornet':
        $name = $_GET['name'] ?? '';
        if (!preg_match('/^[A-Za-z0-9_.+-]{1,80}\.bin$/', $name)) {
            antwort(400, json_encode(['error' => 'Modellname ungültig']));
        }
        xgonav('POST', '/floornet?name=' . rawurlencode($name), 20);   // Laden + Selbstprüfung am CM4
    case 'stimme_spielen':
    case 'stimme_loeschen':
        $name = $_GET['name'] ?? '';
        stimme_pfad($name, false);                                   // nur prüfen
        xgonav('POST', '/stimme/' . ($a === 'stimme_spielen' ? 'spielen' : 'loeschen') . '?name=' . rawurlencode($name));
    case 'stimme_fest':
        // 📌 in der Kachel „Samples“: schützen (fällt nie raus) oder lösen
        $name = $_GET['name'] ?? '';
        if (!preg_match('/^\d\d$/', $name)) {
            antwort(400, json_encode(['error' => 'Probe ungültig']));
        }
        xgonav('POST', '/stimme/fest?name=' . $name . '&an=' . (($_GET['an'] ?? '') === '1' ? '1' : '0'));
    case 'stimme_hochladen':
        // Browser hat schon in WAV 16 kHz mono gewandelt (Kachel „Samples“); xgonav filtert und reiht ein
        $wav = (string)file_get_contents('php://input');
        // fertig=1: schon bearbeitet (48 kHz), ohne Sittich-Filter
        if (strlen($wav) < 44 || strlen($wav) > 1000 * 1024 || substr($wav, 0, 4) !== 'RIFF' || substr($wav, 8, 4) !== 'WAVE') {
            antwort(400, json_encode(['error' => 'keine WAV-Datei oder größer als 1000 KB']));
        }
        xgonav('POST', '/stimme/hochladen?fertig=' . (($_GET['fertig'] ?? '') === '1' ? '1' : '0'), 10, $wav, 'audio/wav');
    case 'stop':
        xgonav('POST', '/stop');
    case 'ausrichten':
        xgonav('POST', '/level');
    case 'pipeline':
        $on = ($_GET['on'] ?? '1') === '0' ? '0' : '1';
        xgonav('POST', '/pipeline?on=' . $on);
    case 'ruhe':
        // Einschlafen: erst xgonav (setzt ab, Abfragen aus, schreibt nicht mehr aufs Display), dann die
        // Schlafaugen (bleiben stehen; das Display lässt sich nicht abschalten), erst danach CPU-Takt runter.
        // Wecken umgekehrt: erst CPU-Takt hoch. Beides wird versucht, auch wenn ein Teil fehlschlägt
        // (xgonav läuft evtl. nicht).
        $an = ($_GET['on'] ?? '1') !== '0';
        $fehler = [];
        if ($an) {
            $code = xgonav_versuch('/ruhe?on=1');
            schlafaugen();
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
    case 'wlan':
        xgonav('POST', '/wlan?on=' . (($_GET['on'] ?? '1') === '0' ? '0' : '1'));
    case 'wlan_connect':
        $d = json_decode((string)file_get_contents('php://input'), true);
        $ssid = is_array($d) ? (string)($d['ssid'] ?? '') : '';
        $pw = is_array($d) ? (string)($d['pw'] ?? '') : '';
        if ($ssid === '' || strlen($ssid) > 32 || preg_match('/[\x00-\x1f]/', $ssid . $pw)) {
            antwort(400, json_encode(['error' => 'SSID ungültig']));
        }
        if ($pw !== '' && (strlen($pw) < 8 || strlen($pw) > 63)) {
            antwort(400, json_encode(['error' => 'Passwort muss 8 bis 63 Zeichen haben (oder leer für offene Netze)']));
        }
        $stdin = "$ssid\n$pw\n" . ($pw === '' ? 'NOPASS' : 'WPA') . "\n0\n";
        $vorher = wlan_kv(wlan_helfer('status')[1]);
        if (($vorher['con'] ?? '') === 'xgo-hotspot' && function_exists('fastcgi_finish_request')) {
            // Über den Hotspot verbunden: Das Umschalten trennt den Browser, Ergebnis steht am Display
            echo json_encode(['ok' => true, 'hotspot' => true]);
            fastcgi_finish_request();
            wlan_helfer('connect', $stdin);
            exit;
        }
        [$rc, $z] = wlan_helfer('connect', $stdin);
        antwort($rc === 0 ? 200 : 500, json_encode($rc === 0 ? ['ok' => true] + wlan_kv($z)
                                                             : ['error' => end($z) ?: "rc $rc"], JSON_UNESCAPED_UNICODE));
    case 'drive':
        $v = zahl('v', -1, 1);
        $w = zahl('w', -1, 1);
        xgonav('POST', '/drive?' . http_build_query(['v' => $v, 'w' => $w]));
    case 'exposure':
        xgonav('POST', '/exposure?us=' . (int)zahl('us', 0, 1000000));   // Grenzen prüft xgonav
    case 'gain':
        xgonav('POST', '/gain?g=' . round(zahl('g', 0, 100), 2));          // Grenzen prüft xgonav
    case 'bild':
        $q = ['k' => round(zahl('k', 0, 10), 2), 's' => round(zahl('s', 0, 10), 2)];
        if (isset($_GET['b'])) {
            $q['b'] = round(zahl('b', -1, 1), 2);                           // fehlt b: Helligkeit bleibt
        }
        xgonav('POST', '/bild?' . http_build_query($q));
    case 'pursuit':
        xgonav('POST', '/pursuit?on=' . (($_GET['on'] ?? '0') === '1' ? '1' : '0'));
    case 'tempo':
        xgonav('POST', '/tempo?f=' . round(zahl('f', 0.05, 1), 3));        // Anteil für alle Fahrbefehle
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
        if ($n > 6 && $n !== 237 && $n !== 253 && $n !== 254 && $n !== 255) {
            antwort(400, json_encode(['error' => 'Aktion ungültig']));
        }
        // 237 = ESP32 neu starten (Firmware-Patch 05-actions, die Firmware lehnt ab, solange er balanciert):
        // danach liegt er disarmed, pet darf ihn also ebenso wenig von selbst aufstellen wie nach 254.
        if ($n === 254 || $n === 237) {
            // Die Aktion geht direkt nach /proc, xgonav erfährt davon nichts. Der Modus pet stünde
            // nach einer Ohnmacht sonst von selbst wieder auf. Fehlschlag egal: dann läuft xgonav nicht.
            @file_get_contents(XGONAV . '/stop?nur=pet', false, stream_context_create(['http' => [
                'method' => 'POST', 'timeout' => 1, 'ignore_errors' => true, 'header' => "Content-Length: 0\r\n",
            ]]));
        }
        if ($n >= 3 && $n <= 5) {
            // 3–5 fahren bzw. drehen selbst über den Fahr-Sollwert der Firmware: Der Keepalive von xgonav
            // (128/128 alle 0,3 s) hielte sie an. xgonav startet sie deshalb selbst, setzt den Keepalive so
            // lange aus und prüft die Bahn (POST /action, 409 mit Grund). Läuft xgonav nicht: direkt.
            $ctx = stream_context_create(['http' => [
                'method' => 'POST', 'timeout' => 3, 'ignore_errors' => true, 'header' => "Content-Length: 0\r\n",
            ]]);
            $body = @file_get_contents(XGONAV . '/action?n=' . $n, false, $ctx);
            if ($body !== false) {
                $code = 502;
                if (isset($http_response_header[0]) && preg_match('#^HTTP/\S+\s+(\d{3})#', $http_response_header[0], $m)) {
                    $code = (int)$m[1];
                }
                antwort($code, $body);
            }
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
        if ($rc === 0) {
            schlafaugen();
        }
        $out = [];
        exec('sudo -n /usr/bin/systemctl poweroff 2>&1', $out, $rc);
        antwort($rc === 0 ? 200 : 500, json_encode($rc === 0 ? ['ok' => true] : ['error' => 'Herunterfahren nicht erlaubt: ' . implode(' ', $out)]));
    case 'config_save':
        $aend = json_decode((string)file_get_contents('php://input'), true);
        if (!is_array($aend) || $aend === [] || array_is_list($aend)) {
            antwort(400, json_encode(['error' => 'keine Änderungen']));
        }
        $fp = fopen(CONFIG_BACKUP . '.lock', 'c');
        if ($fp) flock($fp, LOCK_EX);             // zwei Browser gleichzeitig: nacheinander
        $alt = konfig_text();
        konfig_hash_pruefen($alt);
        try {
            $neu = konfig_aendern($alt, konfig_schema_felder(konfig_schema()), $aend, 'Web ' . date('Y-m-d'));
        } catch (KonfigFehler $e) {
            antwort(400, json_encode(['error' => $e->getMessage()]));
        }
        $name = konfig_schreiben($alt, $neu);
        antwort(200, json_encode(['ok' => true, 'sicherung' => $name, 'hash' => sha1($neu)]));
    case 'config_restore':
        $name = $_GET['name'] ?? '';
        if (!preg_match('/^config\.toml\.[0-9]{8}-[0-9]{6}(-[0-9]+)?$/', $name) || !is_file(CONFIG_BACKUP . "/$name")) {
            antwort(400, json_encode(['error' => 'Sicherung unbekannt']));
        }
        $fp = fopen(CONFIG_BACKUP . '.lock', 'c');
        if ($fp) flock($fp, LOCK_EX);
        $alt = konfig_text();
        konfig_hash_pruefen($alt);
        $neu = (string)file_get_contents(CONFIG_BACKUP . "/$name");
        $gesichert = konfig_schreiben($alt, $neu);
        antwort(200, json_encode(['ok' => true, 'sicherung' => $gesichert, 'hash' => sha1($neu)]));
    case 'restart':
        // Zustand vorher lesen: nicht mitten in einer Fahrt, beim Balancieren oder in einer Aufnahme neu starten,
        // außer der Nutzer bestätigt es (trotzdem=1). Im Ruhemodus nie: erst wecken (CPU-Takt).
        $body = @file_get_contents(XGONAV . '/status', false, stream_context_create(['http' => ['timeout' => 2]]));
        $s = $body === false ? null : json_decode($body, true);
        if (is_array($s)) {
            if (!empty($s['ruhe'])) {
                antwort(409, json_encode(['error' => 'Ruhemodus: erst wecken', 'ruhe' => true]));
            }
            $grund = [];
            if (($s['mode'] ?? 'idle') !== 'idle') $grund[] = 'Modus ' . $s['mode'];
            if (!empty($s['balancing'])) $grund[] = 'balanciert';
            if (!empty($s['aufzeichnung']['an'])) $grund[] = 'Aufzeichnung läuft';
            if ($grund && ($_GET['trotzdem'] ?? '') !== '1') {
                antwort(409, json_encode(['error' => 'Rider in Benutzung: ' . implode(', ', $grund), 'grund' => $grund]));
            }
        }
        exec('sudo -n /usr/bin/systemctl --no-block restart xgonav 2>&1', $out, $rc);
        antwort($rc === 0 ? 200 : 500, json_encode($rc === 0 ? ['ok' => true]
            : ['error' => 'Neustart nicht erlaubt: ' . implode(' ', $out) . ' (deploy/sudoers-xgonav-restart)']));
    default:
        antwort(400, json_encode(['error' => 'unbekannte Aktion']));
}
