<?php
/*
 * xgonav-live – Laufzeitdaten für die Weboberfläche per WebSocket statt Polling.
 *
 * nginx reicht /live hierher durch (127.0.0.1:8089, systemd deploy/xgonav-live.service).
 * Ohne verbundenen Browser schläft der Dienst in stream_select: keine /proc-Zugriffe, 0 % CPU.
 * Bewusst nicht unter web/: dort wäre jede .php-Datei per php-fpm aufrufbar.
 *
 * Server → Browser (Text-Frames, JSON; fällt ein IMU-Takt auf den sys-Takt, kommt beides in einem Frame):
 *   {"sys": {...}, "status": <xgonav /status oder null>, "status_fehler": "..."}   1×/s (Ruhemodus: alle 5 s)
 *   {"imu": {"seq", "imu": [[seq, t_ms, roll, pitch, gier], ...], "wseq",
 *            "wheels": [[seq, t_ms, ticks links, ticks rechts], ...], "state"}}  0,3 s balancierend, sonst 1 s
 * Browser → Server:
 *   "imu 0|1 [since wsince]"   IMU an/aus für diese Verbindung (Lage-Karte zu, Tab verdeckt), ab welcher seq
 *
 * Umgebung (für Tests am PC): XGO_PROC (Default /proc/XGORider), XGO_LIVE_PORT (8089),
 * XGO_STATUS_URL (http://127.0.0.1:8088/status).
 */
declare(strict_types=1);

require __DIR__ . '/ws.inc.php';                    // ws_frame, ws_lesen, ws_handshake, xgonav_status

const SYS_S = 1.0;          // CPU, WLAN, Akku, Beine, xgonav-Status
const RUHE_S = 5.0;         // dasselbe im Ruhemodus
const IMU_BAL_S = 0.3;      // IMU + Räder, wenn der Rider balanciert (imu_log hält 128 Proben)
const IMU_LIEGT_S = 1.0;    // sonst
const IN_MAX = 8192;        // Handshake bzw. Client-Nachrichten: der Browser schickt nur kurze Texte
const OUT_MAX = 1 << 18;    // so viel ungesendet: Verbindung hängt, trennen

define('PROC', rtrim(getenv('XGO_PROC') ?: '/proc/XGORider', '/'));
define('PORT', (int)(getenv('XGO_LIVE_PORT') ?: 8089));
define('STATUS_URL', getenv('XGO_STATUS_URL') ?: 'http://127.0.0.1:8088/status');

if (function_exists('pcntl_signal')) {
    pcntl_signal(SIGPIPE, SIG_IGN);     // Browser weg mitten im Schreiben: nicht den Dienst beenden
}

function lies(string $pfad): string
{
    return trim((string)@file_get_contents($pfad));
}

function zahl_oder_null(string $v): ?int
{
    return is_numeric($v) ? (int)$v : null;
}

// imu_log: "seq t_ns roll pitch yaw" (Milligrad), wheels_log: "seq t_ns ticks11 ticks21 fw_ms", 11 = links.
// Einmal je Takt gelesen, je Verbindung nach since gefiltert.
function imu_lesen(): array
{
    $imu = [];
    $seq = 0;                                               // neueste seq; < since = Treiber neu geladen
    foreach (@file(PROC . '/imu_log', FILE_IGNORE_NEW_LINES) ?: [] as $z) {
        $p = explode(' ', $z);
        if (count($p) !== 5) {
            continue;
        }
        $s = (int)$p[0];
        $seq = max($seq, $s);
        $imu[] = [$s, round((int)$p[1] / 1e6, 1),
                  round((int)$p[2] / 1000, 1), round((int)$p[3] / 1000, 1), round((int)$p[4] / 1000, 1)];
    }
    $rad = [];
    $wseq = 0;
    foreach (@file(PROC . '/wheels_log', FILE_IGNORE_NEW_LINES) ?: [] as $z) {
        $p = explode(' ', $z);
        if (count($p) !== 5) {
            continue;
        }
        $s = (int)$p[0];
        $wseq = max($wseq, $s);
        $rad[] = [$s, round((int)$p[1] / 1e6, 1), (int)$p[2], (int)$p[3]];
    }
    return ['seq' => $seq, 'imu' => $imu, 'wseq' => $wseq, 'wheels' => $rad];
}

// CPU-Zeiten roh (Jiffies), die Auslastung rechnet der Browser aus der Differenz.
function sys_lesen(): array
{
    $cpu = [];
    foreach (@file('/proc/stat', FILE_IGNORE_NEW_LINES) ?: [] as $z) {
        if (!preg_match('/^cpu\d+\s/', $z)) {
            continue;
        }
        $f = array_map('intval', array_slice(preg_split('/\s+/', trim($z)), 1, 8));
        $cpu[] = [array_sum($f), $f[3] + $f[4]];             // [gesamt, idle + iowait]
    }
    $temp = lies('/sys/class/thermal/thermal_zone0/temp');
    $load = lies('/proc/loadavg');
    // Takt in MHz [aktuell, höchst]; CM4: alle Kerne in einer Policy, cpu0 reicht. Höchsttakt einmal gelesen.
    static $max_khz = null;
    if ($max_khz === null) {
        $max_khz = zahl_oder_null(lies('/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq')) ?? 0;
    }
    $cur_khz = zahl_oder_null(lies('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq'));
    return [
        't' => (int)round(microtime(true) * 1000),
        'cpu' => $cpu,
        'temp_c' => is_numeric($temp) ? round((int)$temp / 1000, 1) : null,
        'load' => $load !== '' ? (float)explode(' ', $load)[0] : null,
        'mhz' => ($cur_khz !== null && $max_khz > 0) ? [intdiv($cur_khz, 1000), intdiv($max_khz, 1000)] : null,
        'battery' => zahl_oder_null(lies(PROC . '/battery')),
        // Sollwerte der Beine, null solange unbekannt (nichts geschrieben seit dem Laden, alter Treiber)
        'height' => zahl_oder_null(lies(PROC . '/height')),
        'roll' => zahl_oder_null(lies(PROC . '/roll')),
        'state' => zahl_oder_null(lies(PROC . '/state')),
        'wlan' => wlan(),
    ];
}

// WLAN: "rx_bytes tx_bytes dBm Qualität" von wlan0, '-' wenn unbekannt; Raten rechnet der Browser.
function wlan(): string
{
    $st = '/sys/class/net/wlan0/statistics/';
    $rx = lies($st . 'rx_bytes');
    $tx = lies($st . 'tx_bytes');
    $dbm = $q = '-';
    // /proc/net/wireless: "wlan0: 0000   44.  -66.  -256 ..." (Status, Qualität, Pegel dBm, Rauschen)
    foreach (@file('/proc/net/wireless', FILE_IGNORE_NEW_LINES) ?: [] as $z) {
        $f = preg_split('/\s+/', trim($z));
        if (($f[0] ?? '') === 'wlan0:' && count($f) >= 4) {
            $q = rtrim($f[2], '.');
            $dbm = rtrim($f[3], '.');
        }
    }
    $ok = fn(string $v) => preg_match('/^-?\d+$/', $v) ? $v : '-';
    return implode(' ', [$ok($rx), $ok($tx), $ok($dbm), $ok($q)]);
}

// xgonav /status: [JSON-Text oder null, Fehlertext oder null, Ruhemodus]. Der Text wird unverändert weitergereicht.
function status_holen(): array
{
    [$body, $fehler, $j] = xgonav_status(STATUS_URL);
    return [$body, $fehler, ($j['ruhe'] ?? false) === true];
}

$srv = @stream_socket_server('tcp://127.0.0.1:' . PORT, $errno, $errstr);
if (!$srv) {
    fwrite(STDERR, "[live] Port " . PORT . " nicht verfügbar: $errstr\n");
    exit(1);
}
stream_set_blocking($srv, false);
fwrite(STDERR, "[live] lauscht auf 127.0.0.1:" . PORT . ", Proc " . PROC . "\n");

$clients = [];              // id => [sock, in, out, ws, imu, since, wsince]
$naechsterSys = $naechsteImu = 0.0;
$ruhe = false;

$jetzt = fn(): float => hrtime(true) / 1e9;

function trennen(array &$clients, int $id): void
{
    @fclose($clients[$id]['sock']);
    unset($clients[$id]);
}

function senden(array &$clients, int $id, string $daten = ''): void
{
    $c = &$clients[$id];
    $c['out'] .= $daten;
    if ($c['out'] === '') {
        return;
    }
    $w = @fwrite($c['sock'], $c['out']);
    if ($w === false) {
        trennen($clients, $id);
        return;
    }
    $c['out'] = (string)substr($c['out'], $w);
    if (strlen($c['out']) > OUT_MAX) {
        trennen($clients, $id);
    }
}

while (true) {
    $ws = array_filter($clients, fn($c) => $c['ws']);
    $imuGewollt = !$ruhe && array_filter($ws, fn($c) => $c['imu']);
    $r = [$srv];
    $w = [];
    foreach ($clients as $k) {
        $r[] = $k['sock'];
        if ($k['out'] !== '') {
            $w[] = $k['sock'];
        }
    }
    $e = null;
    $sek = $usek = null;                                    // ohne Browser: schlafen bis zur nächsten Verbindung
    if ($ws) {
        $dt = max(0.0, min($naechsterSys, $imuGewollt ? $naechsteImu : INF) - $jetzt());
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
                $clients[(int)$neu] = ['sock' => $neu, 'in' => '', 'out' => '', 'ws' => false,
                                       'imu' => true, 'since' => 0, 'wsince' => 0];
            }
            continue;
        }
        $id = (int)$s;
        if (!isset($clients[$id])) {
            continue;
        }
        $d = @fread($s, 8192);
        if ($d === false || ($d === '' && feof($s))) {
            trennen($clients, $id);
            continue;
        }
        $c = &$clients[$id];
        $c['in'] .= $d;
        if (!$c['ws']) {
            $p = strpos($c['in'], "\r\n\r\n");
            if ($p === false) {
                unset($c);
                if (strlen($clients[$id]['in']) > IN_MAX) {
                    trennen($clients, $id);
                }
                continue;
            }
            $antwort = ws_handshake($c['in']);
            if ($antwort === '') {
                @fwrite($s, "HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
                unset($c);
                trennen($clients, $id);
                continue;
            }
            $c['ws'] = true;
            if (!$ws) {                                     // erster Browser: gleich ein volles Paket
                $naechsterSys = $naechsteImu = $jetzt();
            }
            $ws[$id] = $c;
            unset($c);
            senden($clients, $id, $antwort);
            if (!isset($clients[$id])) {
                continue;
            }
            $c = &$clients[$id];
        }
        $frames = ws_lesen($c['in'], IN_MAX);
        unset($c);
        if ($frames === false) {
            trennen($clients, $id);
            continue;
        }
        foreach ($frames as [$op, $daten]) {
            if (!isset($clients[$id])) {
                break;
            }
            if ($op === 0x8) {                              // Close: bestätigen und trennen
                @fwrite($s, ws_frame(0x8, ''));
                trennen($clients, $id);
            } elseif ($op === 0x9) {
                senden($clients, $id, ws_frame(0xA, $daten));
            } elseif ($op === 0x1 && preg_match('/^imu ([01])(?: (\d+) (\d+))?$/', trim($daten), $m)) {
                $clients[$id]['imu'] = $m[1] === '1';
                if (isset($m[2])) {
                    $clients[$id]['since'] = (int)$m[2];
                    $clients[$id]['wsince'] = (int)$m[3];
                }
                if ($m[1] === '1') {                        // gleich liefern; nach dem Wecken auch sys
                    $naechsteImu = min($naechsteImu, $jetzt());
                    if ($ruhe) {
                        $naechsterSys = $jetzt();
                    }
                }
            }
        }
    }
    foreach ($w as $s) {
        if (isset($clients[(int)$s])) {
            senden($clients, (int)$s);
        }
    }

    // Takte, nur mit verbundenem Browser
    $ws = array_filter($clients, fn($c) => $c['ws']);
    if (!$ws) {
        continue;
    }
    $t = $jetzt();
    $sysTeil = null;
    if ($t >= $naechsterSys) {
        [$status, $fehler, $ruhe] = status_holen();
        $sysTeil = '"sys":' . json_encode(sys_lesen()) . ',"status":' . ($status ?? 'null')
                 . ',"status_fehler":' . json_encode($fehler, JSON_UNESCAPED_UNICODE);
        $takt = $ruhe ? RUHE_S : SYS_S;
        $naechsterSys = max($naechsterSys + $takt, $t);
        if ($naechsterSys - $t > $takt) {
            $naechsterSys = $t + $takt;
        }
        $t = $jetzt();
    }
    $imu = null;
    if (!$ruhe && $t >= $naechsteImu && array_filter($ws, fn($c) => $c['imu'])) {
        $imu = imu_lesen();
        $imu['state'] = zahl_oder_null(lies(PROC . '/state'));
        $takt = $imu['state'] === 1 ? IMU_BAL_S : IMU_LIEGT_S;   // 1 = balanciert, wie robot._poll_state
        $naechsteImu = $naechsteImu + $takt > $t ? $naechsteImu + $takt : $t + $takt;
    } elseif ($t >= $naechsteImu) {
        $naechsteImu = $t + IMU_LIEGT_S;
    }
    if ($sysTeil === null && $imu === null) {
        continue;
    }
    foreach (array_keys($ws) as $id) {
        if (!isset($clients[$id])) {
            continue;
        }
        $c = &$clients[$id];
        $teile = $sysTeil !== null ? [$sysTeil] : [];
        if ($imu !== null && $c['imu']) {
            if ($imu['seq'] < $c['since']) {
                $c['since'] = 0;                            // Treiber neu geladen
            }
            if ($imu['wseq'] < $c['wsince']) {
                $c['wsince'] = 0;
            }
            $since = $c['since'];
            $wsince = $c['wsince'];
            $teile[] = '"imu":' . json_encode([
                'seq' => $imu['seq'],
                'imu' => array_values(array_filter($imu['imu'], fn($p) => $p[0] > $since)),
                'wseq' => $imu['wseq'],
                'wheels' => array_values(array_filter($imu['wheels'], fn($p) => $p[0] > $wsince)),
                'state' => $imu['state'],
            ]);
            $c['since'] = $imu['seq'];
            $c['wsince'] = $imu['wseq'];
        }
        unset($c);
        if ($teile) {
            senden($clients, $id, ws_frame(0x1, '{' . implode(',', $teile) . '}'));
        }
    }
}
