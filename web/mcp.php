<?php
/*
 * xgonav – MCP-Server (Model Context Protocol, Streamable HTTP) für ein multimodales LLM:
 * Kamerabild, Raumkarte und Zustand ansehen, sich umschauen (drehen, Höhe), fahren, Modi wählen.
 *
 *   POST https://<rider>/mcp.php      JSON-RPC 2.0 (initialize, ping, tools/list, tools/call), Antwort als JSON
 *
 * Zustandslos: keine Sitzung, kein SSE-Strom; jede Anfrage ist ein php-fpm-Aufruf. Ohne Anfragen kostet der
 * Server am Pi nichts. Je Aufruf: 1–3 lokale HTTP-Abfragen an xgonav (127.0.0.1:8088); ein Bild = ein JPEG
 * (Hardware-Kodierer), eine Karte = ein PNG, beides im Web-Thread von xgonav, nie in der Bildpipeline.
 * Bewegungen schicken wie die Weboberfläche Sollwerte mit 10 Hz (/drive), solange sie laufen.
 *
 * Freigabe in /etc/xgonav/mcp.json (fehlt die Datei: nur schauen):
 *   {"stufe": "schauen" | "drehen" | "fahren", "token": ""}
 *   schauen: Bild, Karte, Status, Bildverarbeitung an/aus, wecken, Belichtung, LEDs, Emotionen, Stopp
 *   drehen:  dazu aufstehen, absetzen, auf der Stelle drehen, Rundblick, Körperhöhe, Roll, ausrichten
 *   fahren:  dazu fahren (immer mit Bodengrenze und Bahnprüfung, /drive?sicher=1), Modi, Tempo, Karte, Route,
 *            Aktionen der Firmware
 *   token:   wenn gesetzt, verlangt der Server „Authorization: Bearer <token>“
 *
 * Nicht angeboten: Herunterfahren, Neustart, Einstellungen, WLAN, Kalibrierung, Bodenmodell
 * wechseln, Rohaufzeichnung (bleiben der Weboberfläche vorbehalten).
 *
 * Tests: tests/test_mcp.py (Umgebung XGO_MCP_XGONAV, XGO_PROC, XGO_MCP_CONF, XGO_MCP_SCHNELL).
 */
declare(strict_types=1);

define('XGONAV', getenv('XGO_MCP_XGONAV') ?: 'http://127.0.0.1:8088');
define('PROC', getenv('XGO_PROC') ?: '/proc/XGORider');
define('CONF', getenv('XGO_MCP_CONF') ?: '/etc/xgonav/mcp.json');
define('SCHNELL', (float)(getenv('XGO_MCP_SCHNELL') ?: 1.0));   // Tests: Wartezeiten teilen
const EMOTION = '/opt/xgonav/tools/emotion.sh';
const EMOTION_RAW = '/opt/xgonav/emotions';
const SPERRE = '/tmp/xgonav-mcp.lock';                  // eine Bewegung zur Zeit
const PROTOKOLLE = ['2025-06-18', '2025-03-26', '2024-11-05'];
const STUFEN = ['schauen' => 0, 'drehen' => 1, 'fahren' => 2];
const MODES = ['idle', 'manual', 'follow', 'explore', 'approach', 'home', 'patrol', 'pet'];
const MIN_AKKU = 20;                                    // darunter keine Bewegung
const HEIGHT_MIN = 61;                                  // u8 wie api.php: Minimum + 10 mm (+ Ausrichten)
const HEIGHT_MAX = 200;                                 // xgonav fährt 55..200
const HZ = 10.0;                                        // Sollwerte für manual (hält 0,5 s)
const V_MAX = 0.15;                                     // normiert, geprüft ohne manual_forward_gain: Nennwert v · vx_limit (1,8) m/s
const W_MAX = 0.30;
const FAHR_S_MAX = 5.0;                                 // längste Fahrt je Aufruf

class Fehler extends Exception
{
}

// ---------------------------------------------------------------- Zugriff auf xgonav und /proc
/** [HTTP-Code (0 = nicht erreichbar), Body, Header klein geschrieben] */
function http(string $method, string $path, float $timeout = 3.0): array
{
    $body = @file_get_contents(XGONAV . $path, false, stream_context_create(['http' => [
        'method' => $method, 'timeout' => $timeout, 'ignore_errors' => true, 'header' => "Content-Length: 0\r\n",
    ]]));
    $kopf = $http_response_header ?? [];
    if ($body === false || !isset($kopf[0]) || !preg_match('#^HTTP/\S+\s+(\d{3})#', $kopf[0], $m)) {
        return [0, '', []];
    }
    $h = [];
    foreach (array_slice($kopf, 1) as $z) {
        $t = explode(':', $z, 2);
        if (count($t) === 2) {
            $h[strtolower(trim($t[0]))] = trim($t[1]);
        }
    }
    return [(int)$m[1], $body, $h];
}

/** Befehl an xgonav; wirft mit dessen Fehlertext. */
function post(string $path, float $timeout = 3.0): array
{
    [$code, $body] = http('POST', $path, $timeout);
    if ($code === 0) {
        throw new Fehler('xgonav nicht erreichbar (läuft der Dienst?)');
    }
    $d = json_decode($body, true);
    if ($code !== 200) {
        throw new Fehler(is_array($d) && isset($d['error']) ? (string)$d['error'] : "xgonav: HTTP $code");
    }
    return is_array($d) ? $d : [];
}

function status(): array
{
    [$code, $body] = http('GET', '/status', 2.0);
    $d = $code === 200 ? json_decode($body, true) : null;
    if (!is_array($d)) {
        throw new Fehler($code === 0 ? 'xgonav nicht erreichbar (läuft der Dienst?)' : "xgonav /status: HTTP $code");
    }
    return $d;
}

function proc_lesen(string $name): ?string
{
    $v = @file_get_contents(PROC . '/' . $name);
    return $v === false ? null : trim($v);
}

function proc_schreiben(string $name, string $wert): void
{
    if (@file_put_contents(PROC . '/' . $name, $wert) === false) {
        throw new Fehler("Schreiben nach /proc/XGORider/$name fehlgeschlagen");
    }
}

function balanciert(): bool
{
    return proc_lesen('state') === '1';
}

function akku(): ?int
{
    $v = proc_lesen('battery');
    return $v !== null && preg_match('/^\d+/', $v, $m) ? (int)$m[0] : null;
}

function schlafen(float $s): void
{
    usleep((int)max(0.0, $s / SCHNELL * 1e6));
}

/** Bedingung bis zu $timeout Sekunden abfragen. */
function warte(callable $bed, float $timeout, float $schritt = 0.1): bool
{
    $ende = microtime(true) + $timeout / SCHNELL;
    while (microtime(true) < $ende) {
        if ($bed()) {
            return true;
        }
        schlafen($schritt);
    }
    return (bool)$bed();
}

// ---------------------------------------------------------------- Freigabe
function konf(): array
{
    static $k = null;
    if ($k === null) {
        $d = json_decode((string)@file_get_contents(CONF), true);
        $d = is_array($d) ? $d : [];
        $stufe = (string)($d['stufe'] ?? 'schauen');
        $k = ['stufe' => isset(STUFEN[$stufe]) ? $stufe : 'schauen', 'token' => (string)($d['token'] ?? '')];
    }
    return $k;
}

function braucht(string $stufe): void
{
    $ist = konf()['stufe'];
    if (STUFEN[$ist] < STUFEN[$stufe]) {
        throw new Fehler("Nicht freigegeben: Dieses Werkzeug braucht die Stufe „{$stufe}“, eingestellt ist „{$ist}“. "
            . 'Nur der Besitzer kann das am Rider ändern (' . CONF . ', {"stufe": "' . $stufe . '"}). '
            . 'Sag ihm das, statt es weiter zu versuchen.');
    }
}

// ---------------------------------------------------------------- Bewegung
/** Nur eine Bewegung zur Zeit (zweiter Aufruf scheitert sofort). */
function sperre()
{
    $fp = @fopen(SPERRE, 'c');
    if ($fp === false || !flock($fp, LOCK_EX | LOCK_NB)) {
        throw new Fehler('Es läuft schon eine Bewegung über MCP – abwarten oder „stopp“ rufen.');
    }
    return $fp;
}

$GLOBALS['faehrt'] = false;

function halt(): void
{
    for ($i = 0; $i < 2; $i++) {
        http('POST', '/drive?v=0&w=0', 1.0);
    }
    http('POST', '/stop', 1.0);                 // idle + Stillstand
    $GLOBALS['faehrt'] = false;
}

// Bricht die Anfrage ab (Zeitlimit, Fehler), darf kein Sollwert stehen bleiben; xgonav hält ohnehin nach 0,5 s an.
register_shutdown_function(function (): void {
    if ($GLOBALS['faehrt']) {
        halt();
    }
});

/** Vor jeder Bewegung: balanciert, Akku, niemand sonst fährt; schaltet auf manual. */
function fahrbereit(bool $mit_bild): array
{
    $s = status();
    if (!empty($s['ruhe'])) {
        throw new Fehler('Ruhemodus: erst wecken („ruhemodus“ mit an = false).');
    }
    if (!balanciert()) {
        throw new Fehler('Der Rider liegt. Erst „aufstehen“ (vorher mit „bild“ prüfen, ob vorne Platz ist).');
    }
    $a = akku();
    if ($a !== null && $a < MIN_AKKU) {
        throw new Fehler("Akku $a % < " . MIN_AKKU . ' %: keine Bewegung.');
    }
    $modus = (string)($s['mode'] ?? '');
    if ($modus !== 'idle' && $modus !== 'manual') {
        throw new Fehler("Modus „{$modus}“ läuft gerade. Erst „stopp“ rufen, wenn er wirklich beendet werden soll.");
    }
    if ($mit_bild && ($s['pipeline'] ?? '') !== 'läuft') {
        throw new Fehler('Die Bildverarbeitung ist aus: erst „bildverarbeitung“ mit an = true.');
    }
    if ($modus !== 'manual') {
        post('/mode?name=manual');
        warte(fn() => (status()['mode'] ?? '') === 'manual', 2.0);
    }
    return $s;
}

function kurs(array $s): ?float
{
    $y = $s['yaw_deg'] ?? $s['yaw_imu'] ?? null;
    return is_numeric($y) ? (float)$y : null;
}

/** Auf der Stelle drehen (+ = links), Kurs aus /status; gibt den gedrehten Winkel zurück. Vorher fahrbereit(). */
function drehe(float $grad, float $w_max = 0.25, float $tol = 3.0): float
{
    $y_alt = kurs(status());
    if ($y_alt === null) {
        throw new Fehler('Kein Kurs in /status (IMU-Strom oder Bildverarbeitung fehlt).');
    }
    $gedreht = 0.0;
    $ende = microtime(true) + (10.0 + abs($grad) / 20.0) / SCHNELL;
    $GLOBALS['faehrt'] = true;
    try {
        while (true) {
            if (!balanciert()) {
                throw new Fehler('Abbruch: Der Rider liegt.');
            }
            if (microtime(true) > $ende) {
                throw new Fehler(sprintf('Abbruch: Zeit abgelaufen nach %+.0f° von %+.0f°.', $gedreht, $grad));
            }
            $y = kurs(status());
            if ($y === null) {
                throw new Fehler('Abbruch: Kurs fehlt.');
            }
            $gedreht += fmod(fmod($y - $y_alt + 180.0, 360.0) + 360.0, 360.0) - 180.0;
            $y_alt = $y;
            $err = $grad - $gedreht;
            if (abs($err) <= $tol) {
                break;
            }
            $w = min($w_max, max(0.08, 0.01 * abs($err))) * ($err < 0 ? -1 : 1);
            post(sprintf('/drive?v=0&w=%.3f', $w));       // 409, sobald jemand „stopp“ ruft: Abbruch
            schlafen(1.0 / HZ);
        }
    } finally {
        for ($i = 0; $i < 2; $i++) {
            http('POST', '/drive?v=0&w=0', 1.0);
        }
        $GLOBALS['faehrt'] = false;
    }
    schlafen(1.0);                                         // er dreht nach dem Stopp etwas nach
    $y = kurs(status());
    if ($y !== null) {
        $gedreht += fmod(fmod($y - $y_alt + 180.0, 360.0) + 360.0, 360.0) - 180.0;
    }
    return $gedreht;
}

// ---------------------------------------------------------------- Inhalte
function text(string $t): array
{
    return ['type' => 'text', 'text' => $t];
}

function js($d): string
{
    return (string)json_encode($d, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES);
}

/** Kurzfassung des Zustands: das, was ein LLM zum Entscheiden braucht. */
function kurzstatus(array $s): array
{
    $g = $s['grid'] ?? null;
    $o = $s['ortung']['letzter'] ?? null;
    return [
        'modus' => $s['mode'] ?? null,
        'meldung' => $s['status'] ?? null,
        'bildverarbeitung' => ($s['pipeline'] ?? '') === 'läuft',
        'ruhemodus' => !empty($s['ruhe']),
        'balanciert' => balanciert(),
        'akku_prozent' => akku() ?? ($s['battery'] ?? null),
        'bild_nr' => $s['frame'] ?? null,
        'bilder_je_s' => $s['fps'] ?? null,
        'kurs_grad' => kurs($s),                                   // + = links, alle Drehungen summiert
        'nick_grad' => $s['pitch_deg'] ?? null,
        'koerperhoehe_mm' => $s['height_mm'] ?? null,
        'kamerahoehe_mm' => $s['cam_h_mm'] ?? null,
        'fahrbefehl' => $s['cmd'] ?? null,                         // normiert: v > 0 vor, w > 0 links
        'tempo' => $s['tempo'] ?? null,
        'strecke_m' => $s['odom']['m'] ?? null,
        'freiraum_sektoren' => $s['free'] ?? [],                   // links → rechts, 0 = zu, 1 = frei
        'bodengrenze' => $s['fs_src'] ?? null,                     // "blind" = kein Bodenmodell: fährt nicht selbst
        'kollision_in_s' => $s['ttc'] ?? null,
        'objekte' => $s['tracks'] ?? [],                           // nur in follow/approach/pet
        'ziel' => $s['target'] ?? null,
        'karte' => !is_array($g) ? null : [
            'pose_x_y_grad' => $g['pose'] ?? null, 'pose_bekannt' => $g['pose_ok'] ?? null,
            'start_x_y_grad' => $g['start'] ?? null, 'zum_start_m' => $g['zum_start_m'] ?? null,
            'frei_m2' => $g['frei_m2'] ?? null, 'belegt_m2' => $g['belegt_m2'] ?? null,
            'gesperrt' => $g['gesperrt'] ?? null, 'routenpunkte' => $g['route'] ?? null, 'sperrzonen' => $g['zonen'] ?? null,
        ],
        'erkunden' => $s['explore'] ?? null,
        'streife' => $s['patrol'] ?? null,
        'haustier' => $s['pet'] ?? null,
        'letztes_geraeusch' => $o,                                 // winkel_deg: + = links, vorn/hinten nicht unterscheidbar
        'freigabe' => konf()['stufe'],
    ];
}

/** Kamerabild als MCP-Inhalt samt Bildnummer; neuer als $nach, wenn angegeben. */
function bild_holen(bool $overlay, ?int $nach = null): array
{
    [$code, $body, $h] = http('GET', '/frame?raw=' . ($overlay ? '0' : '1') . ($nach !== null ? "&after=$nach" : ''), 5.0);
    if ($code !== 200) {
        throw new Fehler($code === 0 ? 'xgonav nicht erreichbar (läuft der Dienst?)'
            : 'Kein Bild da (Bildverarbeitung gerade erst gestartet?).');
    }
    return [['type' => 'image', 'data' => base64_encode($body), 'mimeType' => 'image/jpeg'], (int)($h['x-frame'] ?? -1)];
}

/** Frisches Bild + Kurzangaben; scheitert bei gestoppter Bildverarbeitung (das letzte Bild wäre alt). */
function blick(bool $overlay, string $vorweg = ''): array
{
    $s = status();
    if (($s['pipeline'] ?? '') !== 'läuft') {
        throw new Fehler('Die Bildverarbeitung ist aus, es gibt kein aktuelles Bild: erst „bildverarbeitung“ mit an = true.');
    }
    [$img, $nr] = bild_holen($overlay, isset($s['frame']) ? (int)$s['frame'] : null);
    $k = kurzstatus($s);
    $info = ['bild_nr' => $nr, 'kurs_grad' => $k['kurs_grad'], 'nick_grad' => $k['nick_grad'],
             'kamerahoehe_mm' => $k['kamerahoehe_mm'], 'balanciert' => $k['balanciert'], 'modus' => $k['modus'],
             'freiraum_sektoren' => $k['freiraum_sektoren'], 'pose_x_y_grad' => $k['karte']['pose_x_y_grad'] ?? null];
    if (!$k['balanciert']) {
        $info['hinweis'] = 'Der Rider liegt: Die Kamera schaut flach über den Boden nach vorn.';
    }
    return [text(($vorweg !== '' ? $vorweg . "\n" : '') . js($info)), $img];
}

// ---------------------------------------------------------------- Werkzeuge
function schema(array $props = [], array $pflicht = []): array
{
    return ['type' => 'object', 'properties' => (object)$props, 'required' => $pflicht, 'additionalProperties' => false];
}

/** name => [Stufe, Beschreibung, Schema, nur lesend] */
function werkzeuge(): array
{
    $zahl = fn(string $d, float $min, float $max) => ['type' => 'number', 'description' => $d, 'minimum' => $min, 'maximum' => $max];
    $ja = fn(string $d) => ['type' => 'boolean', 'description' => $d];
    return [
        'status' => ['schauen', 'Zustand des Riders als JSON: Modus, Meldung, balanciert/liegt, Akku, Kurs (Grad, + = links), '
            . 'Nick, Kamerahöhe, Freiraum je Sektor (links → rechts), Pose in der Raumkarte (x vorwärts ab Start, y links, Meter), '
            . 'erkannte Objekte, letztes Geräusch (Richtung), Freigabestufe dieses Servers. voll = true liefert alles, was xgonav meldet.',
            schema(['voll' => $ja('Vollständiges /status von xgonav statt der Kurzfassung')]), true],
        'bild' => ['schauen', 'Aktuelles Kamerabild (JPEG 640×480, entzerrt, Öffnungswinkel 91,8° waagrecht; Kamera schaut nach vorn, '
            . '~12 cm über dem Boden). Das ist dein Auge. overlay = true zeichnet ein, was der Rider selbst sieht: magenta = Bodengrenze '
            . '(bis dahin ist Boden frei), cyan = Horizont, Balken unten = Freiraum je Sektor, rot = Ziel, blau = Fahrbefehl. '
            . 'Für einen Blick in eine andere Richtung „drehen“ oder „rundblick“.',
            schema(['overlay' => $ja('Mit eingezeichneter Bodengrenze usw. (Standard: Rohbild)')]), true],
        'karte' => ['schauen', 'Raumkarte von oben als PNG: hell = frei, dunkel = Hindernis/Wand, grau = unbekannt, orange = unbestätigt; '
            . 'der Rider ist eingezeichnet. Oben = +x, links = +y (Meter, Ursprung = Start). Dazu Pose, Start, Route und Sperrzonen als JSON. '
            . 'Die Karte entsteht nur beim Fahren (explore oder von Hand) und kann Doppelwände enthalten.',
            schema(['ausschnitt_m' => $zahl('Kantenlänge des Ausschnitts in Metern (Standard 6)', 1, 48),
                    'px' => ['type' => 'integer', 'description' => 'Bildgröße (Standard 480)', 'minimum' => 120, 'maximum' => 960],
                    'mitte_x' => ['type' => 'number', 'description' => 'Mitte x in Metern (Standard: Rider); nur zusammen mit mitte_y'],
                    'mitte_y' => ['type' => 'number', 'description' => 'Mitte y in Metern']]), true],
        'erkennung' => ['schauen', 'Welche Objektklassen der Detektor (YOLO) kennt und in welchen Modi er läuft (nur follow, approach, pet); '
            . 'die Klassen sind die möglichen Ziele für „modus“ follow/approach.', schema(), true],
        'bildverarbeitung' => ['schauen', 'Kamera und Bildverarbeitung starten oder stoppen. Ohne sie gibt es kein Bild und keine Karte. '
            . 'An kostet Akku (CPU ~340 %): nach dem Schauen wieder ausschalten, wenn der Rider liegt. Aus heißt auch: Stillstand, Modus idle.',
            schema(['an' => $ja('true = starten (wartet auf die ersten Bilder), false = stoppen')], ['an']), false],
        'ruhemodus' => ['schauen', 'Ruhemodus des Riders: an = false weckt ihn (immer erlaubt; er bleibt liegen, die Bildverarbeitung '
            . 'bleibt aus – danach „bildverarbeitung“ an). an = true legt ihn schlafen: absetzen, Bildverarbeitung aus, CPU-Takt runter, '
            . 'Schlafaugen (~4 s; braucht die Freigabestufe drehen). Meldet „status“ ruhemodus = true, geht bis zum Wecken nichts anderes.',
            schema(['an' => $ja('false = wecken, true = schlafen legen')], ['an']), false],
        'kamera' => ['schauen', 'Belichtungszeit und Verstärkung der Kamera setzen (0 = Automatik), z. B. wenn das Bild zu dunkel ist. '
            . 'Gilt bis zum Neustart von xgonav; die Bodengrenze beginnt danach neu.',
            schema(['belichtung_us' => ['type' => 'integer', 'description' => 'µs, 0 = Automatik, sonst 100..Bildabstand', 'minimum' => 0],
                    'verstaerkung' => $zahl('0 = Automatik, sonst 1..63.9', 0, 63.9)]), false],
        'leds' => ['schauen', 'Die vier LEDs des Riders setzen (Index 0..3) – zum Zeigen, was du gerade tust oder meinst.',
            schema(['farben' => ['type' => 'array', 'description' => 'Bis zu 4 Farben "#rrggbb" für LED 0..3; "#000000" = aus',
                                 'items' => ['type' => 'string', 'pattern' => '^#[0-9a-fA-F]{6}$'], 'minItems' => 1, 'maxItems' => 4]],
                   ['farben']), false],
        'emotion' => ['schauen', 'Augen-Animation auf dem Display des Riders abspielen. Ohne name: Liste der vorhandenen Emotionen.',
            schema(['name' => ['type' => 'string', 'description' => 'Name der Emotion, z. B. happy']]), false],
        'stopp' => ['schauen', 'Sofort anhalten: Stillstand, Modus idle, bricht laufende Bewegungen und Modi ab. Immer erlaubt.',
            schema(), false],

        'aufstehen' => ['drehen', 'Aus dem Liegen aufspringen und balancieren (dauert ~10 s). Er rollt dabei ~20 cm vor und schwingt hoch: '
            . 'VORHER mit „bild“ prüfen, dass vorne mindestens 40 cm frei sind und kein Kabel da liegt; im Zweifel den '
            . 'Menschen fragen (nach hinten und zur Seite sieht die Kamera nichts). Braucht laufende Bildverarbeitung.', schema(), false],
        'absetzen' => ['drehen', 'Anhalten und hinlegen (Motoren aus). Spart Akku; immer tun, wenn du fertig bist, danach '
            . '„bildverarbeitung“ aus.', schema(), false],
        'drehen' => ['drehen', 'Auf der Stelle drehen, um woanders hinzuschauen: grad > 0 = nach links, < 0 = nach rechts. Geregelt über den '
            . 'IMU-Kurs (±3°, er dreht bis ~5° nach). Liefert danach das neue Kamerabild. Nur balancierend.',
            schema(['grad' => $zahl('Drehwinkel in Grad, + = links', -360, 360),
                    'bild' => $ja('Danach das Kamerabild mitliefern (Standard true)')], ['grad']), false],
        'rundblick' => ['drehen', 'Einmal im Kreis drehen und in gleichmäßigen Schritten je ein Kamerabild liefern (Standard 4 Bilder im '
            . 'Abstand von 90°, das deckt mit 91,8° Öffnungswinkel alles ab). Das erste Bild ist die Blickrichtung vor dem Drehen; am Ende '
            . 'steht er wieder ungefähr wie vorher. Dauert ~6 s je Schritt. Nur balancierend.',
            schema(['bilder' => ['type' => 'integer', 'description' => 'Anzahl der Bilder (Standard 4)', 'minimum' => 2, 'maximum' => 8],
                    'overlay' => $ja('Bilder mit eingezeichneter Bodengrenze')]), false],
        'hoehe' => ['drehen', 'Körperhöhe ändern und damit die Augenhöhe: u8 61 (tief) .. 200 (hoch), Mitte 128; Kamerahöhe ≈ 109 + 0,113·u8 mm. '
            . 'Nur balancierend und im Stand; danach 4 s ruhig stehen.',
            schema(['u8' => ['type' => 'integer', 'description' => 'Höhen-Sollwert', 'minimum' => HEIGHT_MIN, 'maximum' => HEIGHT_MAX]], ['u8']), false],
        'neigen' => ['drehen', 'Körper seitlich neigen (Roll in Grad, + = nach rechts) – Kopf schief legen. 0 = gerade. Nur balancierend.',
            schema(['grad' => $zahl('Roll in Grad', -20, 20)], ['grad']), false],
        'ausrichten' => ['drehen', 'Beine neu ausrichten (3 s), wenn er im Stand schief steht. Nur balancierend.', schema(), false],

        'fahren' => ['fahren', 'Ein Stück fahren: v > 0 vorwärts, v < 0 rückwärts (dahin sieht die Kamera nicht!), w > 0 Linkskurve; normiert. '
            . 'Nenntempo ≈ v · 1,8 m/s (v 0,10 ≈ 18 cm/s; sehr kleine v bewegen ihn evtl. nicht). Höchstens ' . FAHR_S_MAX . ' s je Aufruf, also in kleinen Schritten fahren und '
            . 'zwischendurch schauen. xgonav prüft jeden Schritt gegen Bodengrenze und Hinderniskarte und bremst selbst (Meldung „Freiraum!“ '
            . 'oder „Bahn“). '
            . 'Liefert Strecke, Meldungen und das Kamerabild danach.',
            schema(['v' => $zahl('Vorwärtsgeschwindigkeit, normiert', -V_MAX, V_MAX),
                    'w' => $zahl('Drehrate, normiert, + = links (Standard 0)', -W_MAX, W_MAX),
                    'sekunden' => $zahl('Dauer', 0.2, FAHR_S_MAX),
                    'bild' => $ja('Danach das Kamerabild mitliefern (Standard true)')], ['v', 'sekunden']), false],
        'modus' => ['fahren', 'Verhalten von xgonav wählen: idle (steht), follow (einem Ziel folgen), approach (Ziel suchen und anfahren), '
            . 'explore (Raum selbstständig erkunden und kartieren), home (zurück zum Start), patrol (Route abfahren, braucht bestaetigt), '
            . 'pet (Haustier: reagiert auf Menschen und Geräusche). Autonome Modi fahren von selbst weiter, bis du „stopp“ rufst: '
            . 'zwischendurch „status“ und „bild“ ansehen.',
            schema(['name' => ['type' => 'string', 'enum' => MODES],
                    'ziel' => ['type' => 'string', 'description' => 'Zielklasse für follow/approach (siehe „erkennung“), Standard person'],
                    'bestaetigt' => $ja('patrol: Der Mensch hat bestätigt, dass der Rider am Startpunkt steht')], ['name']), false],
        'tempo' => ['fahren', 'Temporegler: Anteil 0,05..1 für alle Fahr- und Drehbefehle aller Modi (bis zum Neustart).',
            schema(['anteil' => $zahl('0.05 .. 1', 0.05, 1)], ['anteil']), false],
        'karte_aendern' => ['fahren', 'Raumkarte verwalten: leeren (Karte weg, Pose = Start = 0), start_hier (Startpunkt = jetzige Pose), '
            . 'speichern, sperren/entsperren (gesperrt ändern Fahrten sie nicht), pose_start (Rider steht am Startpunkt), pose_hier '
            . '(angezeigte Pose stimmt), pose_verwerfen.',
            schema(['aktion' => ['type' => 'string', 'enum' => ['leeren', 'start_hier', 'speichern', 'sperren', 'entsperren',
                                                                'pose_start', 'pose_hier', 'pose_verwerfen']]], ['aktion']), false],
        'route' => ['fahren', 'Route für die Streife (patrol) und Sperrzonen setzen, Koordinaten in Metern im Karten-System (siehe „karte“). '
            . 'Ersetzt die bisherige Route; zonen weglassen = Sperrzonen bleiben, leere Liste = alle löschen.',
            schema(['punkte' => ['type' => 'array', 'description' => 'Routenpunkte [[x, y], …]',
                                 'items' => ['type' => 'array', 'items' => ['type' => 'number'], 'minItems' => 2, 'maxItems' => 2]],
                    'schleife' => $ja('Route als Rundkurs'),
                    'zonen' => ['type' => 'array', 'description' => 'Sperrzonen, je eine Liste von mindestens 3 Ecken [[x, y], …]',
                                'items' => ['type' => 'array', 'minItems' => 3,
                                            'items' => ['type' => 'array', 'items' => ['type' => 'number'], 'minItems' => 2, 'maxItems' => 2]]]],
                   ['punkte']), false],
        'aktion' => ['fahren', 'Eingebaute Bewegungsfolge der Firmware abspielen (1..6). 3 und 4 fahren, 5 dreht: nur im Modus idle, '
            . 'xgonav prüft dabei die Bahn. Nur balancierend.',
            schema(['n' => ['type' => 'integer', 'minimum' => 1, 'maximum' => 6]], ['n']), false],
    ];
}

function w_status(array $a): array
{
    $s = status();
    return [text(js(!empty($a['voll']) ? $s + ['mcp' => kurzstatus($s)] : kurzstatus($s)))];
}

function w_bild(array $a): array
{
    return blick(!empty($a['overlay']));
}

function w_karte(array $a): array
{
    $q = ['px' => (int)($a['px'] ?? 480), 'view' => (float)($a['ausschnitt_m'] ?? 6)];
    if (isset($a['mitte_x']) !== isset($a['mitte_y'])) {
        throw new Fehler('mitte_x und mitte_y nur zusammen.');
    }
    if (isset($a['mitte_x'])) {
        $q += ['cx' => (float)$a['mitte_x'], 'cy' => (float)$a['mitte_y']];
    }
    [$code, $png, $h] = http('GET', '/map.png?' . http_build_query($q), 5.0);
    if ($code !== 200) {
        $d = json_decode($png, true);
        throw new Fehler($code === 0 ? 'xgonav nicht erreichbar (läuft der Dienst?)' : (string)($d['error'] ?? "Karte: HTTP $code"));
    }
    $v = array_map('floatval', explode(',', $h['x-map-view'] ?? ''));
    $info = ['ausschnitt' => count($v) >= 3 ? ['mitte_x' => $v[0], 'mitte_y' => $v[1], 'kante_m' => $v[2]] : null,
             'achsen' => 'oben = +x, links = +y, Meter', 'karte' => kurzstatus(status())['karte']];
    [$rc, $rb] = http('GET', '/route');
    if ($rc === 200 && is_array($r = json_decode($rb, true))) {
        $info['route'] = ['punkte' => $r['points'] ?? [], 'schleife' => $r['loop'] ?? false, 'sperrzonen' => $r['zones'] ?? []];
    }
    return [text(js($info)), ['type' => 'image', 'data' => base64_encode($png), 'mimeType' => 'image/png']];
}

function w_erkennung(array $a): array
{
    [$code, $body] = http('GET', '/detector');
    if ($code !== 200) {
        throw new Fehler($code === 0 ? 'xgonav nicht erreichbar (läuft der Dienst?)' : "xgonav: HTTP $code");
    }
    return [text($body)];
}

function w_bildverarbeitung(array $a): array
{
    if (empty($a['an'])) {
        post('/pipeline?on=0');
        return [text('Bildverarbeitung gestoppt (Stillstand, Modus idle).')];
    }
    $s = status();
    if (($s['pipeline'] ?? '') !== 'läuft') {
        post('/pipeline?on=1');
    }
    $f0 = (int)($s['frame'] ?? 0);
    $ok = warte(fn() => (int)(status()['frame'] ?? 0) > $f0 + 5, 20.0, 0.3);
    if (!$ok) {
        throw new Fehler('Gestartet, aber nach 20 s kommen keine neuen Bilder. „status“ ansehen.');
    }
    return [text('Bildverarbeitung läuft, Bilder kommen.')];
}

function w_ruhemodus(array $a): array
{
    // Reihenfolge wie api.php: einschlafen erst xgonav, dann Schlafaugen, dann CPU-Takt; wecken umgekehrt.
    // Beide Teile werden versucht, auch wenn einer scheitert.
    $an = !empty($a['an']);
    $fehler = [];
    $system = function () use ($an, &$fehler): void {
        exec('sudo -n /usr/local/sbin/xgo-ruhe ' . ($an ? 'on' : 'off') . ' 2>&1', $out, $rc);
        if ($rc !== 0) {
            $fehler[] = 'xgo-ruhe: ' . implode(' ', $out);
        }
    };
    $xgonav = function () use ($an, &$fehler): void {
        [$code] = http('POST', '/ruhe?on=' . ($an ? '1' : '0'));
        if ($code !== 200) {
            $fehler[] = $code === 0 ? 'xgonav nicht erreichbar' : "xgonav: HTTP $code";
        }
    };
    if ($an) {
        braucht('drehen');                                   // setzt ihn ab
        $xgonav();
        if (is_dir(EMOTION_RAW . '/sleep') && is_writable('/tmp/lcd0')) {
            exec('pkill -u www-data -f "[e]motion.sh play|[f]fmpeg.*/tmp/lcd0" 2>/dev/null');
            exec(sprintf('timeout 10 %s play %s > /dev/null 2>&1', escapeshellarg(EMOTION), escapeshellarg(EMOTION_RAW . '/sleep')));
        }
        $system();
    } else {
        $system();
        $xgonav();
    }
    if ($fehler) {
        throw new Fehler(($an ? 'Schlafen' : 'Wecken') . ' nur teilweise: ' . implode('; ', $fehler));
    }
    return [text($an ? 'Der Rider schläft (liegt, Bildverarbeitung aus).'
                     : 'Der Rider ist wach. Er liegt, die Bildverarbeitung ist aus: als Nächstes „bildverarbeitung“ mit an = true.')];
}

function w_kamera(array $a): array
{
    $out = [];
    if (isset($a['belichtung_us'])) {
        $out += post('/exposure?us=' . (int)$a['belichtung_us']);
    }
    if (isset($a['verstaerkung'])) {
        $out += post('/gain?g=' . round((float)$a['verstaerkung'], 2));
    }
    if ($out === []) {
        $out = status()['exposure'] ?? [];                 // ohne Angaben: nur anzeigen
    }
    return [text(js($out))];
}

function w_leds(array $a): array
{
    $farben = $a['farben'] ?? null;
    if (!is_array($farben) || $farben === [] || count($farben) > 4) {
        throw new Fehler('farben: 1 bis 4 Einträge "#rrggbb".');
    }
    foreach (array_values($farben) as $i => $c) {
        if (!is_string($c) || !preg_match('/^#[0-9a-fA-F]{6}$/', $c)) {
            throw new Fehler('Farbe ungültig: "#rrggbb" erwartet.');
        }
        proc_schreiben("leds/$i", strtolower($c));
    }
    return [text('LEDs gesetzt.')];
}

function w_emotion(array $a): array
{
    $namen = array_map('basename', glob(EMOTION_RAW . '/*', GLOB_ONLYDIR) ?: []);
    $name = (string)($a['name'] ?? '');
    if ($name === '') {
        return [text(js(['emotionen' => $namen]))];
    }
    if (!preg_match('/^[A-Za-z]+$/', $name) || !in_array($name, $namen, true)) {
        throw new Fehler('Emotion unbekannt. Vorhanden: ' . implode(', ', $namen));
    }
    if (!is_writable('/tmp/lcd0')) {
        throw new Fehler('Display nicht beschreibbar.');
    }
    exec('pkill -u www-data -f "[e]motion.sh play|[f]fmpeg.*/tmp/lcd0" 2>/dev/null');
    exec(sprintf('%s play %s > /dev/null 2>&1 &', escapeshellarg(EMOTION), escapeshellarg(EMOTION_RAW . '/' . $name)));
    return [text("Emotion „{$name}“ läuft.")];
}

function w_stopp(array $a): array
{
    post('/stop');
    return [text('Stillstand, Modus idle.')];
}

function w_aufstehen(array $a): array
{
    if (balanciert()) {
        return [text('Er steht schon.')];
    }
    $s = status();
    if (!empty($s['ruhe'])) {
        throw new Fehler('Ruhemodus: erst wecken („ruhemodus“ mit an = false).');
    }
    $ak = akku();
    if ($ak !== null && $ak < MIN_AKKU) {
        throw new Fehler("Akku $ak % < " . MIN_AKKU . ' %: kein Aufstehen.');
    }
    if (($s['pipeline'] ?? '') !== 'läuft') {
        throw new Fehler('Die Bildverarbeitung ist aus: erst „bildverarbeitung“ an und mit „bild“ den Platz vorne prüfen.');
    }
    $fp = sperre();
    post('/stop');                                           // idle; pet stünde sonst mittendrin anders auf
    proc_schreiben('action', '253');
    if (!warte('balanciert', 8.0)) {
        throw new Fehler('Er balanciert nach 8 s nicht. Den Menschen fragen.');
    }
    // Aufschwung <= 2 s, dann Nivellier-Impuls bis ~5,5 s: so lange nichts schicken
    $t0 = microtime(true);
    while (microtime(true) - $t0 < 6.0 / SCHNELL) {
        if (!balanciert()) {
            throw new Fehler('Er ist wieder umgefallen. Den Menschen fragen.');
        }
        schlafen(0.3);
    }
    return [text('Er balanciert und ist bereit. Körperhöhe steht auf Mitte (~90 mm).')];
}

function w_absetzen(array $a): array
{
    http('POST', '/stop', 2.0);                              // auch ohne xgonav absetzen
    if (!balanciert()) {
        return [text('Er liegt schon.')];
    }
    proc_schreiben('action', '254');
    $ok = warte(fn() => !balanciert(), 5.0);
    return [text($ok ? 'Abgesetzt, er liegt.' : 'Absetzen geschrieben, er meldet aber noch „balanciert“.')];
}

function w_drehen(array $a): array
{
    $grad = (float)($a['grad'] ?? 0);
    if (abs($grad) > 360) {
        throw new Fehler('grad: -360 .. 360.');
    }
    $fp = sperre();
    fahrbereit(false);
    try {
        $ist = drehe($grad);
    } finally {
        halt();
    }
    $t = sprintf('Gedreht %+.1f° (Soll %+.1f°).', $ist, $grad);
    if (($a['bild'] ?? true) && (status()['pipeline'] ?? '') === 'läuft') {
        return blick(false, $t);
    }
    return [text($t)];
}

function w_rundblick(array $a): array
{
    $n = max(2, min(8, (int)($a['bilder'] ?? 4)));
    $overlay = !empty($a['overlay']);
    $fp = sperre();
    fahrbereit(true);
    $inhalt = [];
    $summe = 0.0;
    try {
        for ($i = 0; $i < $n; $i++) {
            $b = blick($overlay, sprintf('Bild %d von %d, Blickrichtung %+.0f° gegenüber dem Anfang (+ = links):', $i + 1, $n, $summe));
            array_push($inhalt, ...$b);
            // Soll = was bis hierher fehlt: Nachdrehen gleicht sich aus, am Ende steht er wie am Anfang
            $summe += drehe(360.0 * ($i + 1) / $n - $summe);
            schlafen(0.5);                                   // ausschwingen, sonst ist das Bild verwischt
        }
    } finally {
        halt();
    }
    $inhalt[] = text(sprintf('Rundblick fertig, insgesamt %+.0f° gedreht.', $summe));
    return $inhalt;
}

function w_hoehe(array $a): array
{
    $u = (int)($a['u8'] ?? -1);
    if ($u < HEIGHT_MIN || $u > HEIGHT_MAX) {
        throw new Fehler('u8: ' . HEIGHT_MIN . ' .. ' . HEIGHT_MAX . '.');
    }
    $fp = sperre();
    fahrbereit(false);
    http('POST', '/stop', 1.0);                              // Höhe nur im Stand (idle)
    proc_schreiben('height', (string)$u);
    schlafen(4.0);
    $s = status();
    return [text(js(['u8' => $u, 'kamerahoehe_mm' => $s['cam_h_mm'] ?? null, 'balanciert' => balanciert()]))];
}

function w_neigen(array $a): array
{
    $g = (int)round((float)($a['grad'] ?? 0));
    if (abs($g) > 20) {
        throw new Fehler('grad: -20 .. 20.');
    }
    if (!balanciert()) {
        throw new Fehler('Der Rider liegt.');
    }
    proc_schreiben('roll', (string)$g);
    return [text("Roll $g °.")];
}

function w_ausrichten(array $a): array
{
    post('/level');
    return [text('Richtet die Beine neu aus (3 s).')];
}

function w_fahren(array $a): array
{
    $v = (float)($a['v'] ?? 0);
    $w = (float)($a['w'] ?? 0);
    $sek = (float)($a['sekunden'] ?? 0);
    if (abs($v) > V_MAX || abs($w) > W_MAX || $sek < 0.2 || $sek > FAHR_S_MAX) {
        throw new Fehler(sprintf('Grenzen: |v| ≤ %.2f, |w| ≤ %.2f, sekunden 0.2 .. %.0f.', V_MAX, W_MAX, FAHR_S_MAX));
    }
    // Ein xgonav vor 2026-11-20 kennt sicher=1 nicht und führe ungeprüft (und mit manual_forward_gain)
    if (strpos(http('GET', '/')[1], 'sicher=') === false) {
        throw new Fehler('Das laufende xgonav kennt das geprüfte Fahren noch nicht (Neustart des Dienstes nötig). Dem Besitzer sagen.');
    }
    $fp = sperre();
    $s0 = fahrbereit(true);
    if (($s0['fs_src'] ?? null) === 'blind') {
        throw new Fehler('Keine Bodengrenze (FloorNet fehlt oder ist ausgefallen): Der Rider ist blind und fährt nicht.');
    }
    $meldungen = [];
    $GLOBALS['faehrt'] = true;
    try {
        $ende = microtime(true) + $sek / SCHNELL;
        while (microtime(true) < $ende) {
            if (!balanciert()) {
                throw new Fehler('Abbruch: Der Rider liegt.');
            }
            post(sprintf('/drive?v=%.3f&w=%.3f&sicher=1', $v, $w));    // 409, sobald jemand „stopp“ ruft
            $m = (string)(status()['status'] ?? '');
            foreach (['Freiraum!', 'Bahn', 'TTC', 'Sperrzone', 'BLIND', 'GEFALLEN'] as $wort) {
                if (strpos($m, $wort) !== false) {
                    $meldungen[$wort] = $m;
                }
            }
            schlafen(1.0 / HZ);
        }
    } finally {
        halt();
    }
    schlafen(0.5);
    $s1 = status();
    $info = ['strecke_m' => isset($s0['odom']['m'], $s1['odom']['m']) ? round($s1['odom']['m'] - $s0['odom']['m'], 2) : null,
             'kurs_vorher_grad' => kurs($s0), 'kurs_nachher_grad' => kurs($s1),
             'pose_vorher' => $s0['grid']['pose'] ?? null, 'pose_nachher' => $s1['grid']['pose'] ?? null,
             'gebremst' => array_values($meldungen)];
    if ($meldungen) {
        $info['hinweis'] = 'xgonav hat gebremst oder angehalten: Dort ist ein Hindernis oder kein Boden erkannt. Nicht erzwingen, anders fahren.';
    }
    if (($a['bild'] ?? true) && ($s1['pipeline'] ?? '') === 'läuft') {
        return blick(false, js($info));
    }
    return [text(js($info))];
}

function w_modus(array $a): array
{
    $name = (string)($a['name'] ?? '');
    if (!in_array($name, MODES, true)) {
        throw new Fehler('Modus unbekannt: ' . implode(', ', MODES));
    }
    $ziel = (string)($a['ziel'] ?? 'person');
    if (!preg_match('/^[a-z ]{1,30}$/', $ziel)) {
        throw new Fehler('ziel ungültig.');
    }
    post('/mode?' . http_build_query(['name' => $name, 'target' => $ziel, 'bestaetigt' => !empty($a['bestaetigt']) ? '1' : '0']));
    schlafen(0.3);
    $s = status();
    return [text(js(['modus' => $s['mode'] ?? null, 'meldung' => $s['status'] ?? null]))];
}

function w_tempo(array $a): array
{
    $f = (float)($a['anteil'] ?? 0);
    if ($f < 0.05 || $f > 1) {
        throw new Fehler('anteil: 0.05 .. 1.');
    }
    return [text(js(post('/tempo?f=' . round($f, 3))))];
}

function w_karte_aendern(array $a): array
{
    $wege = ['leeren' => '/map/reset', 'start_hier' => '/map/start', 'speichern' => '/map/save',
             'sperren' => '/map/lock?on=1', 'entsperren' => '/map/lock?on=0', 'pose_start' => '/map/pose?aktion=start',
             'pose_hier' => '/map/pose?aktion=hier', 'pose_verwerfen' => '/map/pose?aktion=verwerfen'];
    $aktion = (string)($a['aktion'] ?? '');
    if (!isset($wege[$aktion])) {
        throw new Fehler('aktion: ' . implode(', ', array_keys($wege)));
    }
    post($wege[$aktion]);
    return [text("Karte: $aktion erledigt.")];
}

function punkte(array $liste): string
{
    $t = [];
    foreach ($liste as $p) {
        if (!is_array($p) || count($p) !== 2 || !is_numeric($p[0]) || !is_numeric($p[1])) {
            throw new Fehler('Punkte als [x, y] in Metern.');
        }
        $t[] = round((float)$p[0], 3) . ',' . round((float)$p[1], 3);
    }
    return implode(';', $t);
}

function w_route(array $a): array
{
    if (!is_array($a['punkte'] ?? null)) {
        throw new Fehler('punkte fehlt.');
    }
    $q = ['pts' => punkte($a['punkte']), 'loop' => !empty($a['schleife']) ? '1' : '0'];
    if (isset($a['zonen'])) {
        if (!is_array($a['zonen'])) {
            throw new Fehler('zonen: Liste von Eckenlisten.');
        }
        if ($a['zonen'] === []) {
            $q['nozones'] = '1';
        } else {
            $q['zones'] = implode('|', array_map(fn($z) => punkte(is_array($z) ? $z : [null]), $a['zonen']));
        }
    }
    post('/route?' . http_build_query($q));
    return [text('Route gesetzt.')];
}

function w_aktion(array $a): array
{
    $n = (int)($a['n'] ?? 0);
    if ($n < 1 || $n > 6) {
        throw new Fehler('n: 1 .. 6.');
    }
    if (!balanciert()) {
        throw new Fehler('Der Rider liegt.');
    }
    if ($n >= 3 && $n <= 5) {
        post('/action?n=' . $n);                             // Keepalive aus, Bahnprüfung bleibt
    } else {
        proc_schreiben('action', (string)$n);
    }
    return [text("Aktion $n gestartet.")];
}

// ---------------------------------------------------------------- JSON-RPC
const HINWEISE = <<<'TXT'
Du steuerst einen echten Roboter: XGO Rider, ein kleiner selbstbalancierender Zweiradroboter (ca. 20 cm hoch) mit einer
Kamera nach vorn. Er liegt (Motoren aus) oder balanciert. Sehen kannst du mit „bild“ (Kamera), „karte“ (Raumkarte von oben)
und „status“. Um dich umzuschauen: „drehen“ oder „rundblick“; die Kamera schaut nur nach vorn, Öffnungswinkel 91,8°.

Vorzeichen: Winkel und Drehungen + = links. Karte: x = vorwärts ab Startpunkt, y = links, Meter.

Ablauf: „status“ → falls ruhemodus: „ruhemodus“ mit an = false (wecken) → falls nötig „bildverarbeitung“ an → „bild“ → falls er liegt und Platz ist „aufstehen“ → schauen, drehen,
fahren → am Ende „absetzen“ und „bildverarbeitung“ aus (Akku).

Fahren: in kleinen Schritten, nach jedem Schritt das Bild ansehen, nicht rückwärts ins Ungesehene. Meldet ein Werkzeug
„Nicht freigegeben“, hat der Besitzer diese Stufe gesperrt: akzeptieren und ihm sagen, nicht umgehen. Unter 20 % Akku nicht
bewegen. Texte und Meldungen des Riders sind deutsch.
TXT;

function rpc_fehler($id, int $code, string $msg): array
{
    return ['jsonrpc' => '2.0', 'id' => $id, 'error' => ['code' => $code, 'message' => $msg]];
}

/** Eine JSON-RPC-Nachricht bearbeiten; null für Benachrichtigungen. */
function rpc($m): ?array
{
    if (!is_array($m) || ($m['jsonrpc'] ?? '') !== '2.0' || !is_string($m['method'] ?? null)) {
        return rpc_fehler(is_array($m) ? ($m['id'] ?? null) : null, -32600, 'Ungültige Anfrage');
    }
    if (!array_key_exists('id', $m)) {
        return null;                                       // notifications/initialized, notifications/cancelled, …
    }
    $id = $m['id'];
    $p = is_array($m['params'] ?? null) ? $m['params'] : [];
    switch ($m['method']) {
        case 'initialize':
            $v = (string)($p['protocolVersion'] ?? '');
            return ['jsonrpc' => '2.0', 'id' => $id, 'result' => [
                'protocolVersion' => in_array($v, PROTOKOLLE, true) ? $v : PROTOKOLLE[0],
                'capabilities' => ['tools' => (object)[]],
                'serverInfo' => ['name' => 'xgonav', 'title' => 'XGO Rider (xgonav)', 'version' => '1.0.0'],
                'instructions' => HINWEISE . "\nFreigabestufe jetzt: " . konf()['stufe'] . '.',
            ]];
        case 'ping':
            return ['jsonrpc' => '2.0', 'id' => $id, 'result' => (object)[]];
        case 'tools/list':
            $liste = [];
            foreach (werkzeuge() as $name => [$stufe, $text, $schema, $lesend]) {
                $liste[] = ['name' => $name, 'description' => $text . ($stufe !== 'schauen' ? " [Freigabestufe: $stufe]" : ''),
                            'inputSchema' => $schema,
                            'annotations' => ['readOnlyHint' => $lesend, 'destructiveHint' => false, 'openWorldHint' => false]];
            }
            return ['jsonrpc' => '2.0', 'id' => $id, 'result' => ['tools' => $liste]];
        case 'tools/call':
            $name = (string)($p['name'] ?? '');
            $w = werkzeuge();
            if (!isset($w[$name])) {
                return rpc_fehler($id, -32602, "Unbekanntes Werkzeug: $name");
            }
            $arg = $p['arguments'] ?? [];
            try {
                braucht($w[$name][0]);
                $inhalt = ('w_' . $name)(is_array($arg) ? $arg : []);
                return ['jsonrpc' => '2.0', 'id' => $id, 'result' => ['content' => $inhalt, 'isError' => false]];
            } catch (Fehler $e) {
                return ['jsonrpc' => '2.0', 'id' => $id, 'result' => ['content' => [text($e->getMessage())], 'isError' => true]];
            }
        default:
            return rpc_fehler($id, -32601, 'Methode unbekannt: ' . $m['method']);
    }
}

function ende(int $code, ?string $body = null, array $kopf = []): never
{
    http_response_code($code);
    foreach ($kopf as $k) {
        header($k);
    }
    if ($body !== null) {
        header('Content-Type: application/json; charset=utf-8');
        echo $body;
    }
    exit;
}

header('Cache-Control: no-store');
$methode = $_SERVER['REQUEST_METHOD'] ?? '';
if ($methode !== 'POST') {
    // kein SSE-Strom (GET) und keine Sitzungen (DELETE): laut Spezifikation 405
    ende(405, js(['error' => 'MCP-Server von xgonav: JSON-RPC per POST (Streamable HTTP)']), ['Allow: POST']);
}
$token = konf()['token'];
if ($token !== '' && !hash_equals('Bearer ' . $token, (string)($_SERVER['HTTP_AUTHORIZATION'] ?? ''))) {
    ende(401, js(['error' => 'Token fehlt oder falsch']), ['WWW-Authenticate: Bearer']);
}
// Nur JSON: Ein Browser muss dafür erst anfragen (CORS-Preflight, hier 405); so kann keine fremde Webseite
// im Heimnetz den Rider über ein einfaches Formular-POST steuern.
if (stripos((string)($_SERVER['CONTENT_TYPE'] ?? ''), 'application/json') !== 0) {
    ende(415, js(['error' => 'Content-Type: application/json erwartet']));
}
set_time_limit(120);
ignore_user_abort(true);                                   // eine Bewegung endet immer mit Stillstand, auch ohne Zuhörer
$anfrage = json_decode((string)file_get_contents('php://input'), true);
if ($anfrage === null) {
    ende(400, js(rpc_fehler(null, -32700, 'Kein gültiges JSON')));
}
if (is_array($anfrage) && array_is_list($anfrage)) {       // Stapel (ältere Protokollversionen)
    $antworten = array_values(array_filter(array_map('rpc', $anfrage), fn($r) => $r !== null));
    $antworten === [] ? ende(202) : ende(200, js($antworten));
}
$antwort = rpc($anfrage);
$antwort === null ? ende(202) : ende(200, js($antwort));
