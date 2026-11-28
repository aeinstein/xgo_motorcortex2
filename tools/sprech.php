<?php
/*
 * xgonav-sprech – Gegensprechen Browser ↔ Rider per WebSocket, nur im Modus manual (Hand).
 *
 * nginx reicht /sprech hierher durch (127.0.0.1:8090, systemd deploy/xgonav-sprech.service).
 * Rider-Mikrofon:  arecord (plughw:CARD=wm8960soundcard,DEV=0, 8 kHz, mono, S16_LE) → Binärrahmen je 40 ms (640 B) → Browser
 * Browser-Mikrofon: Binärrahmen (S16_LE, 8 kHz, mono)                        → aplay → Lautsprecher
 * Ohne Gespräch laufen weder arecord noch aplay; der Dienst schläft in stream_select (0 % CPU).
 * Die Ortung hält hw:CARD=wm8960soundcard,DEV=0 nur in [ortung] modes (idle, pet); in manual ist das Gerät frei. Wechselt der Modus
 * weg von manual, endet das Gespräch binnen STATUS_S und gibt das Gerät frei.
 * Nur ein Gespräch zugleich; ein zweiter Browser bekommt "besetzt".
 * Rechenarbeit (Abtastrate wandeln, Echo dämpfen, Pegel) macht der Browser.
 *
 * Server → Browser:  Text {"sprech": "an"|"aus", "grund": "..."}; Binär = Ton vom Rider
 * Browser → Server:  Binär = Ton zum Rider; Text "mic 0|1" (Rider-Mikro senden), "laut 0|1" (Lautsprecher)
 *
 * Bild aufs Display: zweite Verbindung GET /?bild (eigene TCP-Verbindung, große Rahmen halten den Ton nicht
 * auf), nur während eines Gesprächs. Je Binärrahmen ein fertiges Bild RGB565 LE 320×240 (153 600 B, im Browser
 * umgerechnet), durchgereicht ins FIFO des Display-Daemons. Besitz über FIFO.owner (PID): xgonav/lcd.py gibt das
 * FIFO dann frei, ein laufender Ausdruck des Augengenerators bricht ab. Startet jemand eine Emotion,
 * wird das laufende Bild fertig geschrieben und bis zu ihrem Ende nichts gezeigt. Am Ende läuft „cute“.
 * Server → Browser: Text {"bild": "an"|"aus", "grund": "..."}.
 *
 * Umgebung (für Tests am PC): XGO_SPRECH_PORT (8090), XGO_STATUS_URL (http://127.0.0.1:8088/status),
 * XGO_ARECORD, XGO_APLAY (ganze Befehlszeile, durch Leerzeichen getrennt), XGO_LCD_FIFO (/tmp/lcd0),
 * XGO_LCD_ENDE (Befehl nach dem Bild, Default emotion.sh play cute).
 */
declare(strict_types=1);

require __DIR__ . '/ws.inc.php';

const RATE = 8000;
const RAHMEN = 640;             // 40 ms bei 8 kHz, 16 bit, mono
const STATUS_S = 1.0;           // so oft den Modus prüfen, solange ein Gespräch läuft
const IN_MAX = 16384;           // größter Rahmen vom Browser
const AUS_MAX = 16000;          // ungesendet zum Browser (0,5 s): neue Töne verwerfen statt stauen
const LAUT_MAX = 8000;          // ungespielt für aplay (0,5 s): auf LAUT_REST kürzen, nur das Neueste zählt
const LAUT_REST = 3200;         // 0,2 s
const BILD = 320 * 240 * 2;     // ein Display-Bild RGB565
const BILD_WARTE_S = 0.3;       // nach dem Eintragen als Besitzer: laufende Emotion/lcd.py geben das FIFO frei
const OWNER_ALT_S = 10.0;       // wie emotion.active: ältere Besitzer-Einträge zählen nicht mehr

define('PORT', (int)(getenv('XGO_SPRECH_PORT') ?: 8090));
define('STATUS_URL', getenv('XGO_STATUS_URL') ?: 'http://127.0.0.1:8088/status');

function befehl(string $env, array $std): array
{
    $v = trim((string)getenv($env));
    return $v !== '' ? preg_split('/\s+/', $v) : $std;
}

define('ARECORD', befehl('XGO_ARECORD', ['arecord', '-q', '-D', 'plughw:CARD=wm8960soundcard,DEV=0', '-f', 'S16_LE', '-r', (string)RATE,
                                         '-c', '1', '-t', 'raw', '--buffer-time=100000']));
define('APLAY', befehl('XGO_APLAY', ['aplay', '-q', '-D', 'plughw:CARD=wm8960soundcard,DEV=0', '-f', 'S16_LE', '-r', (string)RATE,
                                     '-c', '1', '-t', 'raw', '--buffer-time=200000']));
define('LCD_FIFO', getenv('XGO_LCD_FIFO') ?: '/tmp/lcd0');
define('LCD_OWNER', LCD_FIFO . '.owner');
define('LCD_ENDE', getenv('XGO_LCD_ENDE') !== false ? (string)getenv('XGO_LCD_ENDE')
    : (is_file(__DIR__ . '/emotion.sh') ? escapeshellarg(__DIR__ . '/emotion.sh') . ' play cute' : ''));

if (function_exists('pcntl_signal')) {
    pcntl_signal(SIGPIPE, SIG_IGN);     // aplay oder Browser weg mitten im Schreiben: nicht den Dienst beenden
}

function log_(string $s): void
{
    fwrite(STDERR, "[sprech] $s\n");
}

$srv = @stream_socket_server('tcp://127.0.0.1:' . PORT, $errno, $errstr);
if (!$srv) {
    log_("Port " . PORT . " nicht verfügbar: $errstr");
    exit(1);
}
stream_set_blocking($srv, false);
log_("lauscht auf 127.0.0.1:" . PORT);

$clients = [];                  // id => [sock, in, out, ws, bild]
$gespraech = null;              // id des Browsers mit Gespräch
$bildId = null;                 // id der Bild-Verbindung
$lcd = null;                    // FIFO des Display-Daemons, offen solange wir Besitzer sind
$lcdBuf = '';                   // Rest des Bildes, das gerade ins FIFO geht (nie abbrechen: Bildgrenzen)
$lcdNext = null;                // neuestes wartendes Bild
$lcdAb = null;                  // Besitz eingetragen: ab dann öffnen; null = nicht eingetragen
$lcdGezeigt = false;            // mindestens ein Bild geschrieben (dann am Ende „cute“)
$rec = $play = null;            // [proc, pipe]
$recBuf = $playBuf = '';
$mic = $laut = true;
$naechsterStatus = 0.0;
$jetzt = fn(): float => hrtime(true) / 1e9;

function trennen(int $id): void
{
    global $clients, $gespraech, $bildId;
    if (!isset($clients[$id])) {
        return;
    }
    @fclose($clients[$id]['sock']);
    unset($clients[$id]);
    if ($gespraech === $id) {
        ende();
    }
    if ($bildId === $id) {
        bild_ende();
    }
}

function senden(int $id, string $daten = ''): void
{
    global $clients;
    if (!isset($clients[$id])) {
        return;
    }
    $c = &$clients[$id];
    $c['out'] .= $daten;
    if ($c['out'] === '') {
        return;
    }
    $w = @fwrite($c['sock'], $c['out']);
    if ($w === false) {
        unset($c);
        trennen($id);
        return;
    }
    $c['out'] = (string)substr($c['out'], $w);
    if (strlen($c['out']) > 4 * AUS_MAX) {
        unset($c);
        trennen($id);
    }
}

function melden(int $id, string $zustand, string $grund = ''): void
{
    global $clients;
    $art = !empty($clients[$id]['bild']) ? 'bild' : 'sprech';
    senden($id, ws_frame(0x1, json_encode([$art => $zustand, 'grund' => $grund], JSON_UNESCAPED_UNICODE)));
}

// Melden und Verbindung schließen (Close-Rahmen; was noch im Puffer liegt, geht blockierend kurz hinterher).
function abweisen(int $id, string $grund): void
{
    global $clients;
    melden($id, 'aus', $grund);
    senden($id, ws_frame(0x8, pack('n', 1000)));
    if (isset($clients[$id]) && $clients[$id]['out'] !== '') {
        stream_set_blocking($clients[$id]['sock'], true);
        @fwrite($clients[$id]['sock'], $clients[$id]['out']);
        $clients[$id]['out'] = '';
    }
    trennen($id);
}

function prozess(array $cmd, bool $lesen): ?array
{
    $spec = $lesen ? [0 => ['file', '/dev/null', 'r'], 1 => ['pipe', 'w'], 2 => ['file', '/dev/null', 'w']]
                   : [0 => ['pipe', 'r'], 1 => ['file', '/dev/null', 'w'], 2 => ['file', '/dev/null', 'w']];
    $p = @proc_open($cmd, $spec, $pipes);
    if (!is_resource($p)) {
        return null;
    }
    $pipe = $pipes[$lesen ? 1 : 0];
    stream_set_blocking($pipe, false);
    return [$p, $pipe];
}

function beenden(?array &$pr): void
{
    if ($pr === null) {
        return;
    }
    @fclose($pr[1]);
    @proc_terminate($pr[0]);
    @proc_close($pr[0]);
    $pr = null;
}

function beginn(int $id): void
{
    global $gespraech, $rec, $play, $recBuf, $playBuf, $mic, $laut, $naechsterStatus, $jetzt;
    $rec = prozess(ARECORD, true);
    $play = prozess(APLAY, false);
    if ($rec === null || $play === null) {
        beenden($rec);
        beenden($play);
        abweisen($id, 'arecord/aplay starten nicht');
        return;
    }
    $gespraech = $id;
    $recBuf = $playBuf = '';
    $mic = $laut = true;
    $naechsterStatus = $jetzt() + STATUS_S;
    log_('Gespräch an');
    melden($id, 'an');
}

function ende(): void
{
    global $gespraech, $rec, $play, $recBuf, $playBuf, $bildId;
    if ($gespraech === null) {
        return;
    }
    beenden($rec);
    beenden($play);
    $recBuf = $playBuf = '';
    $gespraech = null;
    log_('Gespräch aus');
    if ($bildId !== null) {
        abweisen($bildId, 'Gespräch beendet');
    }
}

// ------------------------------------------------------------------ Display
// Besitzer laut FIFO.owner: '' = frei (leer oder veraltet), 'wir', sonst 'fremd'.
function lcd_besitzer(): string
{
    $pid = trim((string)@file_get_contents(LCD_OWNER));
    if ($pid === '') {
        return '';
    }
    if ($pid === (string)getmypid()) {
        return 'wir';
    }
    $m = @filemtime(LCD_OWNER);
    return $m !== false && time() - $m < OWNER_ALT_S ? 'fremd' : '';
}

function lcd_schliessen(): void
{
    global $lcd;
    if ($lcd !== null) {
        @fclose($lcd);
        $lcd = null;
    }
}

// Je Schleifendurchlauf, solange eine Bild-Verbindung besteht: Besitz und nächstes Bild. Geschrieben wird in
// der select-Schleife, sobald das FIFO Platz hat; ein angefangenes Bild wird immer zu Ende geschrieben.
function lcd_takt(float $t): void
{
    global $lcd, $lcdBuf, $lcdNext, $lcdAb;
    if ($lcdBuf !== '' || $lcdNext === null) {             // nur mit wartendem Bild nachsehen (≈ Bildrate)
        return;
    }
    $wer = lcd_besitzer();
    if ($wer === 'fremd') {                                 // Emotion läuft: freigeben, Bilder verwerfen
        if ($lcd !== null || $lcdAb !== null) {
            log_('Display: Emotion läuft, Bild pausiert');
        }
        lcd_schliessen();
        $lcdAb = null;
        $lcdNext = null;
        return;
    }
    if ($lcdAb === null || $wer === '') {                   // eintragen, dann kurz warten
        @file_put_contents(LCD_OWNER, getmypid() . "\n");
        $lcdAb = $t + BILD_WARTE_S;
        return;
    }
    if ($t < $lcdAb) {
        return;
    }
    if ($lcd === null) {
        if (@filetype(LCD_FIFO) !== 'fifo') {               // kein Daemon: nicht in eine normale Datei schreiben
            $lcdNext = null;
            return;
        }
        $f = @fopen(LCD_FIFO, 'r+');                        // O_RDWR blockiert bei einem FIFO nie
        if ($f === false) {
            $lcdNext = null;
            return;
        }
        stream_set_blocking($f, false);
        stream_set_write_buffer($f, 0);
        $lcd = $f;
    }
    $lcdBuf = $lcdNext;
    $lcdNext = null;
}

// Bild-Verbindung vorbei: angefangenes Bild fertig schreiben (höchstens 1 s), FIFO zu, Besitz zurück, „cute“.
function bild_ende(): void
{
    global $bildId, $lcd, $lcdBuf, $lcdNext, $lcdAb, $lcdGezeigt;
    $bildId = null;
    $bis = hrtime(true) / 1e9 + 1.0;
    while ($lcd !== null && $lcdBuf !== '' && hrtime(true) / 1e9 < $bis) {
        $n = @fwrite($lcd, $lcdBuf);
        if ($n === false) {
            break;
        }
        $lcdBuf = (string)substr($lcdBuf, $n);
        if ($lcdBuf !== '') {
            usleep(5000);
        }
    }
    lcd_schliessen();
    $lcdBuf = '';
    $lcdNext = null;
    $lcdAb = null;
    if (lcd_besitzer() === 'wir') {
        @file_put_contents(LCD_OWNER, '');
        if ($lcdGezeigt && LCD_ENDE !== '') {
            exec(LCD_ENDE . ' > /dev/null 2>&1 &');
        }
    }
    $lcdGezeigt = false;
    log_('Bild aus');
}

// Modus prüfen: Fehlertext oder null, wenn xgonav in manual ist.
function nicht_hand(): ?string
{
    [, $fehler, $j] = xgonav_status(STATUS_URL);
    if ($j === null) {
        return $fehler;
    }
    if (!empty($j['ruhe'])) {
        return 'Ruhemodus';
    }
    return ($j['mode'] ?? '') === 'manual' ? null : 'nur im Handmodus';
}

while (true) {
    $r = [$srv];
    $w = [];
    foreach ($clients as $c) {
        $r[] = $c['sock'];
        if ($c['out'] !== '') {
            $w[] = $c['sock'];
        }
    }
    if ($rec !== null) {
        $r[] = $rec[1];
    }
    if ($play !== null && $playBuf !== '') {
        $w[] = $play[1];
    }
    if ($lcd !== null && $lcdBuf !== '') {
        $w[] = $lcd;
    }
    $e = null;
    $sek = $usek = null;                                    // ohne Gespräch: schlafen bis zur nächsten Verbindung
    if ($gespraech !== null) {
        $bis = $naechsterStatus;
        if ($lcdNext !== null && $lcdAb !== null) {
            $bis = min($bis, $lcdAb);
        }
        $dt = max(0.0, $bis - $jetzt());
        $sek = (int)$dt;
        $usek = (int)(($dt - $sek) * 1e6);
    }
    if (@stream_select($r, $w, $e, $sek, $usek) === false) {
        continue;                                           // Signal
    }
    foreach ($r as $s) {
        if ($s === $srv) {
            $neu = @stream_socket_accept($srv, 0);
            if ($neu) {
                stream_set_blocking($neu, false);
                $clients[(int)$neu] = ['sock' => $neu, 'in' => '', 'out' => '', 'ws' => false, 'bild' => false,
                                       'teil' => null];
            }
            continue;
        }
        if ($rec !== null && $s === $rec[1]) {              // Ton vom Rider-Mikrofon
            $d = @fread($s, 8192);
            if ($d === false || ($d === '' && feof($s))) {
                $id = $gespraech;
                ende();
                if ($id !== null) {
                    abweisen($id, 'Mikrofon belegt oder arecord beendet');
                }
                continue;
            }
            $recBuf .= $d;
            while (strlen($recBuf) >= RAHMEN) {
                $stueck = substr($recBuf, 0, RAHMEN);
                $recBuf = (string)substr($recBuf, RAHMEN);
                if ($mic && $gespraech !== null && strlen($clients[$gespraech]['out']) < AUS_MAX) {
                    senden($gespraech, ws_frame(0x2, $stueck));
                }
            }
            continue;
        }
        $id = (int)$s;
        if (!isset($clients[$id])) {
            continue;
        }
        $d = @fread($s, 65536);
        if ($d === false || ($d === '' && feof($s))) {
            trennen($id);
            continue;
        }
        $clients[$id]['in'] .= $d;
        if (!$clients[$id]['ws']) {
            $clients[$id]['bild'] = (bool)preg_match('#^GET\s+\S*\?bild\b#', $clients[$id]['in']);
            $antwort = ws_handshake($clients[$id]['in']);
            if ($antwort === null) {
                if (strlen($clients[$id]['in']) > 8192) {
                    trennen($id);
                }
                continue;
            }
            if ($antwort === '') {
                @fwrite($s, "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
                trennen($id);
                continue;
            }
            $clients[$id]['ws'] = true;
            senden($id, $antwort);
            if (!isset($clients[$id])) {
                continue;
            }
            if ($clients[$id]['bild']) {                    // Bild aufs Display: nur zum laufenden Gespräch
                if ($gespraech === null) {
                    abweisen($id, 'kein Gespräch');
                } elseif ($bildId !== null) {
                    abweisen($id, 'besetzt (anderes Bild läuft)');
                } elseif (@filetype(LCD_FIFO) !== 'fifo') {
                    abweisen($id, 'Display-Daemon läuft nicht');
                } else {
                    $bildId = $id;
                    $lcdGezeigt = false;
                    log_('Bild an');
                    melden($id, 'an');
                }
                continue;
            }
            if ($gespraech !== null) {
                abweisen($id, 'besetzt (anderer Browser spricht)');
                continue;
            }
            $grund = nicht_hand();
            if ($grund !== null) {
                abweisen($id, $grund);
                continue;
            }
            beginn($id);
            if (!isset($clients[$id])) {
                continue;
            }
        }
        $frames = ws_lesen($clients[$id]['in'], $clients[$id]['bild'] ? BILD : IN_MAX, $clients[$id]['teil']);
        if ($frames === false) {
            trennen($id);
            continue;
        }
        foreach ($frames as [$op, $daten]) {
            if (!isset($clients[$id])) {
                break;
            }
            if ($op === 0x8) {                              // Close: bestätigen und trennen
                @fwrite($s, ws_frame(0x8, ''));
                trennen($id);
            } elseif ($op === 0x9) {
                senden($id, ws_frame(0xA, $daten));
            } elseif ($op === 0x2 && $id === $bildId) {
                if (strlen($daten) === BILD) {
                    $lcdNext = $daten;                      // nur das Neueste zählt
                }
            } elseif ($op === 0x2 && $id === $gespraech && $laut && $play !== null) {
                $playBuf .= $daten;
                if (strlen($playBuf) > LAUT_MAX) {          // Rückstau: nur das Neueste spielen
                    $weg = strlen($playBuf) - LAUT_REST;
                    $playBuf = (string)substr($playBuf, $weg - $weg % 2);   // gerade Zahl verwerfen: Samples bleiben ganz
                }
            } elseif ($op === 0x1 && $id === $gespraech && preg_match('/^(mic|laut) ([01])$/', trim($daten), $m)) {
                if ($m[1] === 'mic') {
                    $mic = $m[2] === '1';
                } else {
                    $laut = $m[2] === '1';
                    if (!$laut) {                           // halbes Sample behalten, sonst kippt die Byte-Lage
                        $playBuf = strlen($playBuf) % 2 ? substr($playBuf, -1) : '';
                    }
                }
            }
        }
    }
    foreach ($w as $s) {
        if ($lcd !== null && $s === $lcd) {
            $n = @fwrite($s, $lcdBuf);
            if ($n === false) {                             // Daemon weg
                $lcdBuf = '';
                lcd_schliessen();
                continue;
            }
            $lcdBuf = (string)substr($lcdBuf, $n);
            if ($lcdBuf === '') {
                $lcdGezeigt = true;
                @touch(LCD_OWNER);                          // frisch halten (lcd.py: jünger als 10 s)
            }
            continue;
        }
        if ($play !== null && $s === $play[1]) {
            $n = @fwrite($s, $playBuf);
            if ($n === false) {
                $id = $gespraech;
                ende();
                if ($id !== null) {
                    abweisen($id, 'aplay beendet (Lautsprecher belegt?)');
                }
                continue;
            }
            $playBuf = (string)substr($playBuf, $n);
            continue;
        }
        if (isset($clients[(int)$s])) {
            senden((int)$s);
        }
    }
    if ($bildId !== null) {
        lcd_takt($jetzt());
    }
    // Modus weg von manual (oder xgonav weg): Gespräch beenden, Gerät frei für die Ortung
    if ($gespraech !== null && $jetzt() >= $naechsterStatus) {
        $grund = nicht_hand();
        $naechsterStatus = $jetzt() + STATUS_S;
        if ($grund !== null) {
            $id = $gespraech;
            ende();
            abweisen($id, $grund);
        }
    }
}
