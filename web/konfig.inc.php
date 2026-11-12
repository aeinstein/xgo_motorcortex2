<?php
/*
 * Einstellungen aus der Weboberfläche: config.toml zeilenweise lesen und ändern (Kommentare bleiben),
 * Typen gegen web/config-schema.json prüfen (tools/config_schema.py aus xgonav/config.py).
 * Eingebunden von api.php; direkt aufgerufen tut die Datei nichts.
 *
 * Verstanden wird nur das Format, das config.toml am Rider hat: [abschnitt], schlüssel = wert  # kommentar,
 * Werte true/false, Zahl, "text", 'text' und einzeilige Listen. Mehrzeiliges gilt als unlesbar; solche
 * Schlüssel lassen sich hier nicht ändern (Fehlermeldung), der Rest schon.
 */
declare(strict_types=1);

if (realpath($_SERVER['SCRIPT_FILENAME'] ?? '') === __FILE__) {
    http_response_code(404);
    exit;
}

const KONFIG_OBEN = 'allgemein';          // Pseudo-Abschnitt für Schlüssel vor dem ersten [abschnitt]

final class KonfigFehler extends RuntimeException {}

// Schema: [abschnitt][schlüssel] => Feld (typ, elem, nullbar, default, hilfe)
function konfig_schema_felder(array $schema): array
{
    $felder = [];
    foreach ($schema['kategorien'] as $k) {
        foreach ($k['abschnitte'] as $a) {
            foreach ($a['felder'] as $f) {
                $felder[$a['name']][$f['key']] = $f;
            }
        }
    }
    return $felder;
}

// Wert einer TOML-Zeile ab $s[$i] lesen; gibt [Wert, Endposition] zurück oder wirft.
function konfig_wert_lesen(string $s, int &$i)
{
    $n = strlen($s);
    while ($i < $n && ($s[$i] === ' ' || $s[$i] === "\t")) $i++;
    if ($i >= $n) throw new KonfigFehler('Wert fehlt');
    $c = $s[$i];
    if ($c === '"') {
        $j = $i + 1;
        while ($j < $n && $s[$j] !== '"') {
            $j += $s[$j] === '\\' ? 2 : 1;
        }
        if ($j >= $n) throw new KonfigFehler('Text nicht geschlossen');
        $roh = substr($s, $i, $j - $i + 1);
        $v = json_decode($roh);            // TOML-Basisstrings: Escapes wie JSON
        if (!is_string($v)) throw new KonfigFehler("Text $roh nicht lesbar");
        $i = $j + 1;
        return $v;
    }
    if ($c === "'") {
        $j = strpos($s, "'", $i + 1);
        if ($j === false) throw new KonfigFehler('Text nicht geschlossen');
        $v = substr($s, $i + 1, $j - $i - 1);
        $i = $j + 1;
        return $v;
    }
    if ($c === '[') {
        $i++;
        $liste = [];
        while (true) {
            while ($i < $n && ($s[$i] === ' ' || $s[$i] === "\t")) $i++;
            if ($i >= $n) throw new KonfigFehler('mehrzeilige Liste');
            if ($s[$i] === ']') { $i++; return $liste; }
            $liste[] = konfig_wert_lesen($s, $i);
            while ($i < $n && ($s[$i] === ' ' || $s[$i] === "\t")) $i++;
            if ($i < $n && $s[$i] === ',') { $i++; continue; }
            if ($i < $n && $s[$i] === ']') { $i++; return $liste; }
            throw new KonfigFehler('Liste nicht lesbar');
        }
    }
    if (!preg_match('/\G(true|false|[-+]?(?:inf|nan)|[-+]?[0-9_]+(?:\.[0-9_]+)?(?:[eE][-+]?[0-9_]+)?)/', $s, $m, 0, $i)) {
        throw new KonfigFehler('Wert nicht lesbar');
    }
    $i += strlen($m[1]);
    $t = str_replace('_', '', $m[1]);
    if ($t === 'true') return true;
    if ($t === 'false') return false;
    if (preg_match('/^[-+]?[0-9]+$/', $t)) return (int)$t;
    return (float)$t;
}

/*
 * config.toml in Zeilen zerlegen. Jede Zeile: ['roh' => Text] und bei Schlüsselzeilen zusätzlich
 * abschnitt, key, vor (bis einschließlich "= "), wert, rest (Leerraum + Kommentar), fehler (unlesbar).
 */
function konfig_zerlegen(string $text): array
{
    $zeilen = [];
    $abschnitt = KONFIG_OBEN;
    foreach (preg_split('/\r?\n/', $text) as $roh) {
        $z = ['roh' => $roh];
        if (preg_match('/^\s*\[\s*([A-Za-z0-9_]+)\s*\]\s*(#.*)?$/', $roh, $m)) {
            $abschnitt = $m[1];
            $z['kopf'] = $abschnitt;
        } elseif (preg_match('/^(\s*([A-Za-z0-9_]+)\s*=\s*)(.*)$/', $roh, $m)) {
            $z += ['abschnitt' => $abschnitt, 'key' => $m[2], 'vor' => $m[1]];
            $i = 0;
            try {
                $z['wert'] = konfig_wert_lesen($m[3], $i);
                $rest = substr($m[3], $i);
                if (trim($rest) !== '' && !preg_match('/^\s*#/', $rest)) {
                    throw new KonfigFehler('Zeichen nach dem Wert');
                }
                $z['rest'] = $rest;
            } catch (KonfigFehler $e) {
                $z['fehler'] = $e->getMessage();
            }
        }
        $zeilen[] = $z;
    }
    return $zeilen;
}

// Gesetzte Werte: [abschnitt][key] => Wert; unlesbare Zeilen: [abschnitt][key] => Fehlertext
function konfig_werte(array $zeilen): array
{
    $werte = [];
    $fehler = [];
    foreach ($zeilen as $z) {
        if (!isset($z['key'])) continue;
        if (isset($z['fehler'])) {
            $fehler[$z['abschnitt']][$z['key']] = $z['fehler'];
        } else {
            $werte[$z['abschnitt']][$z['key']] = $z['wert'];
        }
    }
    return [$werte, $fehler];
}

// Wert aus dem Browser gegen das Feld prüfen und normieren; wirft mit lesbarer Meldung.
function konfig_pruefen(string $name, array $feld, $v)
{
    $typ = $feld['typ'];
    if ($typ === 'bool') {
        if (!is_bool($v)) throw new KonfigFehler("$name: Ja/Nein erwartet");
        return $v;
    }
    if ($typ === 'int') {
        if (is_float($v) && floor($v) === $v && abs($v) < 1e15) $v = (int)$v;
        if (!is_int($v)) throw new KonfigFehler("$name: ganze Zahl erwartet");
        return $v;
    }
    if ($typ === 'float') {
        if (!is_int($v) && !is_float($v)) throw new KonfigFehler("$name: Zahl erwartet");
        if (!is_finite((float)$v)) throw new KonfigFehler("$name: endliche Zahl erwartet");
        return (float)$v;
    }
    if ($typ === 'str') {
        if (!is_string($v)) throw new KonfigFehler("$name: Text erwartet");
        if (preg_match('/[\x00-\x1f]/', $v)) throw new KonfigFehler("$name: Steuerzeichen nicht erlaubt");
        return $v;
    }
    if ($typ === 'liste') {
        if (!is_array($v) || !array_is_list($v)) throw new KonfigFehler("$name: Liste erwartet");
        foreach ($v as $e) {
            if ($feld['elem'] === 'str' ? !is_string($e) : (!is_int($e) && !is_float($e))) {
                throw new KonfigFehler("$name: Liste aus " . ($feld['elem'] === 'str' ? 'Texten' : 'Zahlen') . ' erwartet');
            }
        }
        return $v;
    }
    throw new KonfigFehler("$name: Typ $typ unbekannt");
}

function konfig_toml(string $typ, $v): string
{
    if (is_bool($v)) return $v ? 'true' : 'false';
    if (is_int($v)) return $typ === 'float' ? $v . '.0' : (string)$v;
    if (is_float($v)) {
        $s = rtrim(rtrim(sprintf('%.12F', $v), '0'), '.');
        if (abs($v) >= 1e12 || ($v != 0.0 && abs($v) < 1e-6)) $s = sprintf('%.12e', $v);
        return preg_match('/[.eE]/', $s) ? $s : $s . '.0';
    }
    if (is_string($v)) return json_encode($v, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES);
    if (is_array($v)) return '[' . implode(', ', array_map(fn($e) => konfig_toml('', $e), $v)) . ']';
    throw new KonfigFehler('Wert nicht darstellbar');
}

/*
 * Änderungen anwenden: $aend = ["abschnitt.key" => Wert | null]; null entfernt die Zeile (dann gilt der
 * Default aus config.py). Gibt den neuen Text zurück. Vorhandene Zeilen behalten Einrückung und Kommentar,
 * neue Schlüssel kommen ans Ende ihres Abschnitts (fehlt er, wird er angehängt).
 */
function konfig_aendern(string $text, array $felder, array $aend, string $vermerk): string
{
    $zeilen = konfig_zerlegen($text);
    $neu = [];                                    // [abschnitt][key] => TOML-Wert für neue Zeilen
    foreach ($aend as $name => $v) {
        if (!preg_match('/^([A-Za-z0-9_]+)\.([A-Za-z0-9_]+)$/', (string)$name, $m) || !isset($felder[$m[1]][$m[2]])) {
            throw new KonfigFehler("$name: unbekannte Einstellung");
        }
        [$ab, $key] = [$m[1], $m[2]];
        $feld = $felder[$ab][$key];
        $toml = $v === null ? null : konfig_toml($feld['typ'], konfig_pruefen($name, $feld, $v));
        $gefunden = false;
        foreach ($zeilen as $i => $z) {
            if (($z['abschnitt'] ?? null) !== $ab || ($z['key'] ?? null) !== $key) continue;
            if (isset($z['fehler'])) {
                throw new KonfigFehler("$name: Zeile in config.toml nicht lesbar ({$z['fehler']}), nur per ssh ändern");
            }
            if ($gefunden || $toml === null) {
                unset($zeilen[$i]);               // doppelt (TOML-Fehler) oder auf Default zurück
            } else {
                $zeilen[$i]['roh'] = $z['vor'] . $toml . $z['rest'];
            }
            $gefunden = true;
        }
        if (!$gefunden && $toml !== null) {
            $neu[$ab][$key] = $toml;
        }
    }
    $zeilen = array_values($zeilen);
    foreach ($neu as $ab => $paare) {
        $einfuegen = [];
        foreach ($paare as $key => $toml) {
            $einfuegen[] = ['roh' => "$key = $toml   # $vermerk"];
        }
        // Ende des Abschnitts: letzte Schlüsselzeile vor dem nächsten Kopf (Leerzeilen/Kommentare danach bleiben)
        $start = null;
        $ende = null;
        foreach ($zeilen as $i => $z) {
            if (isset($z['kopf'])) {
                if ($start !== null) break;
                if ($z['kopf'] === $ab) { $start = $i; $ende = $i; }
                continue;
            }
            if ($start !== null && isset($z['key'])) $ende = $i;
        }
        if ($ab === KONFIG_OBEN) {
            // oberste Ebene: hinter den letzten Schlüssel vor dem ersten Kopf, sonst ganz an den Anfang
            $ende = null;
            foreach ($zeilen as $i => $z) {
                if (isset($z['kopf'])) break;
                if (isset($z['key'])) $ende = $i;
            }
            array_splice($zeilen, $ende === null ? 0 : $ende + 1, 0, $einfuegen);
        } elseif ($start === null) {
            while ($zeilen && trim(end($zeilen)['roh']) === '') array_pop($zeilen);
            array_push($zeilen, ['roh' => ''], ['roh' => "[$ab]"], ...$einfuegen);
            $zeilen[] = ['roh' => ''];
        } else {
            array_splice($zeilen, $ende + 1, 0, $einfuegen);
        }
    }
    return implode("\n", array_column($zeilen, 'roh'));
}
