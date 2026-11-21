<?php
/*
 * WebSocket-Grundlagen für die PHP-CLI-Dienste hinter nginx (tools/live.php, tools/sprech.php).
 * Nur unfragmentierte Rahmen vom Browser; die Höchstlänge gibt jeder Dienst an ws_lesen mit.
 * Bewusst nicht unter web/: dort wäre jede .php-Datei per php-fpm aufrufbar.
 */
declare(strict_types=1);

const WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

function ws_frame(int $op, string $d): string
{
    $n = strlen($d);
    if ($n < 126) {
        return chr(0x80 | $op) . chr($n) . $d;
    }
    if ($n < 65536) {
        return chr(0x80 | $op) . chr(126) . pack('n', $n) . $d;
    }
    return chr(0x80 | $op) . chr(127) . pack('J', $n) . $d;
}

// Vollständige Client-Frames aus dem Puffer: [[opcode, daten], ...]; false = Protokollfehler oder > $max.
function ws_lesen(string &$buf, int $max): array|false
{
    $aus = [];
    while (($n = strlen($buf)) >= 2) {
        $b1 = ord($buf[1]);
        $len = $b1 & 0x7f;
        $o = 2;
        if ($len === 126) {
            if ($n < 4) {
                break;
            }
            $len = unpack('n', substr($buf, 2, 2))[1];
            $o = 4;
        } elseif ($len === 127) {                           // 64-bit-Länge (Bilder aufs Display)
            if ($n < 10) {
                break;
            }
            $len = unpack('J', substr($buf, 2, 8))[1];
            $o = 10;
        }
        if ($len > $max) {
            return false;
        }
        $maske = ($b1 & 0x80) !== 0;
        if ($n < $o + ($maske ? 4 : 0) + $len) {
            break;
        }
        $d = substr($buf, $o + ($maske ? 4 : 0), $len);
        if ($maske && $len) {
            $d ^= substr(str_repeat(substr($buf, $o, 4), intdiv($len, 4) + 1), 0, $len);
        }
        $aus[] = [ord($buf[0]) & 0x0f, $d];
        $buf = (string)substr($buf, $o + ($maske ? 4 : 0) + $len);
    }
    return $aus;
}

// HTTP-Kopf im Puffer: null = noch unvollständig, '' = kein WebSocket-Handshake, sonst die 101-Antwort.
// Der Kopf wird aus $buf entfernt.
function ws_handshake(string &$buf): ?string
{
    $p = strpos($buf, "\r\n\r\n");
    if ($p === false) {
        return null;
    }
    $kopf = substr($buf, 0, $p);
    $buf = (string)substr($buf, $p + 4);
    if (!preg_match('/^Sec-WebSocket-Key:\s*(\S+)/mi', $kopf, $m)) {
        return '';
    }
    return "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
        . 'Sec-WebSocket-Accept: ' . base64_encode(sha1($m[1] . WS_GUID, true)) . "\r\n\r\n";
}

// xgonav /status: [JSON-Text oder null, Fehlertext oder null, dekodiert oder null].
function xgonav_status(string $url, float $timeout = 0.5): array
{
    $body = @file_get_contents($url, false, stream_context_create(['http' => [
        'timeout' => $timeout, 'ignore_errors' => true,
    ]]));
    if ($body === false) {
        return [null, 'xgonav nicht erreichbar (läuft der Dienst?)', null];
    }
    $code = 0;
    if (isset($http_response_header[0]) && preg_match('#^HTTP/\S+\s+(\d{3})#', $http_response_header[0], $m)) {
        $code = (int)$m[1];
    }
    $j = json_decode($body, true);
    if ($code !== 200 || !is_array($j)) {
        return [null, is_array($j) && isset($j['error']) ? (string)$j['error'] : 'xgonav: Fehler ' . $code, null];
    }
    return [$body, null, $j];
}
