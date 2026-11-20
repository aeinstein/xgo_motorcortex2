// Host-Test für Patch 0025: Bausteine des neuen BLE-Dienstes (pi_tunnel.h) und die Register 0x17/0x18.
// Tunnelrahmen (Aufbau, Stückeln und Zusammensetzen), Parser (0x17 mit 1..24 Byte, 0x18 mit 4 Byte, 0x13 und
// 0xb3 gibt es nicht mehr), Leseantworten 0x17/0x18, Ausgabepuffer, IP mit Zeitablauf, Zeilenpuffer der Konsole.
// Nach firmware/build.sh, im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_ble_tunnel.cpp $S/command_protocol.cpp -o t && ./t
#include "pi_tunnel.h"
#include "command_protocol.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace rider2;
using namespace rider2::protocol;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

// Rahmen byteweise in den Parser; true, wenn genau ein Befehl herauskommt.
static bool feedFrame(StreamParser& p, Source src, const std::uint8_t* f, std::size_t n, CommandEvent& e) {
    int got = 0;
    for (std::size_t i = 0; i < n; ++i) if (p.feed(src, f[i], 0, e)) ++got;
    return got == 1;
}

int main() {
    // --- Tunnelrahmen ESP32 -> Pi
    {
        const std::uint8_t data[] = {'a', 'b', 'c'};
        std::uint8_t f[tunnel::kMaxFrame];
        const std::size_t n = tunnel::buildFrame(tunnel::kData, data, 3, f, sizeof f);
        std::uint8_t sum = 0;
        for (std::size_t i = 2; i + 3 < n; ++i) sum = std::uint8_t(sum + f[i]);
        check(n == 12 && f[0] == 0x55 && f[1] == 0 && f[2] == 12 && f[3] == 0x15 && f[4] == 0x17 && f[5] == 0 &&
              !std::memcmp(f + 6, "abc", 3) && f[9] == std::uint8_t(~sum) && f[10] == 0 && f[11] == 0xaa,
              "Tunnelrahmen: Typ 0x15, Register 0x17, Kennung + Daten, Prüfsumme");
        check(tunnel::buildFrame(tunnel::kConnected, nullptr, 0, f, sizeof f) == 9 && f[5] == 1,
              "Meldung verbunden: 9 Byte, nur Kennung");
        std::uint8_t big[24] = {};
        check(tunnel::buildFrame(tunnel::kData, big, 24, f, sizeof f) == 0 &&
              tunnel::buildFrame(tunnel::kData, big, 23, f, sizeof f) == tunnel::kMaxFrame,
              "höchstens 23 Byte Daten je Rahmen (32 Byte Rahmen)");
    }
    // --- Stückeln wie loop() und Zusammensetzen wie der Pi
    {
        std::string msg = "17 wlan connect WPA 0 Mein%20Netz ";
        for (int i = 0; i < 63; ++i) msg += char('a' + i % 26);
        msg += "\n";
        std::string back;
        int frames = 0;
        for (std::size_t off = 0; off < msg.size(); off += tunnel::kMaxData) {
            const std::size_t k = std::min(tunnel::kMaxData, msg.size() - off);
            std::uint8_t f[tunnel::kMaxFrame];
            const std::size_t n = tunnel::buildFrame(tunnel::kData, reinterpret_cast<const std::uint8_t*>(msg.data()) + off, k, f, sizeof f);
            if (n != k + 9 || f[5] != tunnel::kData) fail = 1;
            back.append(reinterpret_cast<const char*>(f) + 6, n - 9);
            ++frames;
        }
        std::printf("       %zu Byte in %d Rahmen\n", msg.size(), frames);
        check(back == msg && frames == int((msg.size() + 22) / 23), "lange Zeile: gestückelt und wieder vollständig");
    }
    // --- Parser: Schreiben auf 0x17 / 0x18 vom Pi
    {
        StreamParser p;
        CommandEvent e;
        std::uint8_t f[40], pay[25] = {};
        pay[0] = tunnel::kData;
        for (int i = 1; i < 25; ++i) pay[i] = std::uint8_t(0x40 + i);
        std::size_t n = buildFrame(1, 0x17, pay, 24, f, sizeof f);
        const bool full = feedFrame(p, Source::Aux, f, n, e) && e.reg == 0x17 && e.length == 24 && !e.isRead() &&
                          !std::memcmp(e.data, pay, 24) && commandKind(e) == CommandKind::PiTunnel;
        n = buildFrame(1, 0x17, pay, 1, f, sizeof f);
        const bool one = feedFrame(p, Source::Aux, f, n, e) && e.length == 1;
        check(full && one, "Schreiben auf 0x17: 1..24 Byte angenommen");
        const auto before = p.stats();
        n = buildFrame(1, 0x17, pay, 25, f, sizeof f);
        check(!feedFrame(p, Source::Aux, f, n, e) && p.stats().bad_length == before.bad_length + 1,
              "25 Byte Nutzlast: abgelehnt (Rahmen zu lang)");

        const std::uint8_t ip[4] = {192, 168, 53, 110};
        n = buildFrame(1, 0x18, ip, 4, f, sizeof f);
        const bool ipok = feedFrame(p, Source::Aux, f, n, e) && e.reg == 0x18 && e.length == 4 && !std::memcmp(e.data, ip, 4) &&
                          commandKind(e) == CommandKind::PiIp;
        n = buildFrame(1, 0x18, ip, 3, f, sizeof f);
        const bool ipbad = !feedFrame(p, Source::Aux, f, n, e);
        check(ipok && ipbad, "Schreiben auf 0x18: genau 4 Byte");

        const std::uint8_t one_byte[1] = {1};
        const std::uint8_t name[4] = {'T', 'e', 's', 't'};
        n = buildFrame(1, 0x13, name, 4, f, sizeof f);
        const bool no13 = !feedFrame(p, Source::Aux, f, n, e);
        n = buildFrame(1, 0xb3, one_byte, 1, f, sizeof f);
        const bool nob3 = !feedFrame(p, Source::Ble, f, n, e);
        check(no13 && nob3, "0x13 (BLE-Name) und 0xb3 (Trace über BLE) gibt es nicht mehr");
    }
    // --- Leseantworten 0x17 (BLE verbunden) und 0x18 (IP)
    {
        StreamParser p;
        CommandEvent e;
        Telemetry t;
        t.ble_connected = 1;
        const std::uint8_t ip[4] = {10, 10, 0, 110};
        std::memcpy(t.pi_ip, ip, 4);
        std::uint8_t f[16], out[64];
        const std::uint8_t len1[1] = {1}, len4[1] = {4};
        std::size_t n = buildFrame(2, 0x18, len4, 1, f, sizeof f);
        bool ok = feedFrame(p, Source::Ble, f, n, e) && e.isRead();
        std::size_t m = buildReadResponse(e, t, out, sizeof out);
        ok = ok && m == 12 && out[3] == 0x12 && out[4] == 0x18 && !std::memcmp(out + 5, ip, 4);
        n = buildFrame(2, 0x17, len1, 1, f, sizeof f);
        ok = ok && feedFrame(p, Source::Usb, f, n, e);
        m = buildReadResponse(e, t, out, sizeof out);
        ok = ok && m == 9 && out[4] == 0x17 && out[5] == 1;
        check(ok, "Lesen: 0x18 = 4 Byte IP, 0x17 = 1 Byte BLE verbunden");
    }
    // --- Ausgabepuffer
    {
        std::uint8_t store[16];
        tunnel::ByteRing r;
        r.attach(store, sizeof store);
        const std::uint8_t a[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::uint8_t out[16];
        bool ok = r.push(a, 10) == 10 && r.size() == 10 && r.pop(out, 6) == 6 && !std::memcmp(out, a, 6);
        ok = ok && r.push(a, 10) == 10 && r.size() == 14;                       // läuft über das Pufferende
        ok = ok && r.pop(out, 16) == 14 && !std::memcmp(out, a + 6, 4) && !std::memcmp(out + 4, a, 10) && r.size() == 0;
        check(ok, "Ausgabepuffer: Reihenfolge bleibt über das Pufferende hinweg");
        ok = r.push(a, 10) == 10 && r.push(a, 10) == 6 && r.dropped() == 4 && r.space() == 0;
        r.clear();
        check(ok && r.size() == 0 && r.push(a, 3) == 3, "voll: Rest verworfen und gezählt; clear() leert");
        tunnel::ByteRing none;
        check(none.push(a, 3) == 0 && none.pop(out, 3) == 0, "ohne Speicher: nichts");
    }
    // --- IP des Pi mit Zeitablauf
    {
        tunnel::PiIp ip;
        const std::uint8_t a[4] = {192, 168, 53, 110}, b[4] = {10, 10, 0, 110}, zero[4] = {0, 0, 0, 0};
        bool ok = !ip.expire(50000) && ip.set(a, 1000) && !ip.set(a, 4000) && !std::memcmp(ip.ip, a, 4);
        ok = ok && !ip.expire(14000) && ip.set(b, 14000) && !std::memcmp(ip.ip, b, 4);
        check(ok, "IP: geändert gemeldet nur bei neuem Wert");
        ok = !ip.expire(24000) && ip.expire(24001) && !std::memcmp(ip.ip, zero, 4) && !ip.expire(30000);
        check(ok, "IP: 10 s ohne Meldung -> 0.0.0.0, einmal gemeldet");
        ok = ip.set(a, 0xfffffff0u) && !ip.expire(0x00000100u) && ip.expire(0xfffffff0u + 10001u);
        check(ok, "IP: Zeitablauf über den Überlauf von millis()");
        tunnel::PiIp z;
        check(!z.set(zero, 5) && !z.expire(20000), "IP 0.0.0.0 vom Pi: keine Änderung, auch nicht beim Ablauf");
    }
    // --- Zeilenpuffer der Konsole
    {
        tunnel::LineBuffer<8> l;
        auto feed = [&](const char* s) { int lines = 0; for (; *s; ++s) if (l.feed(std::uint8_t(*s))) ++lines; return lines; };
        bool ok = feed("!stop") == 0 && feed("\n") == 1 && !std::strcmp(l.text(), "!stop");
        ok = ok && feed("\r\n\n") == 0;                                          // leere Zeilen
        ok = ok && feed("!arm\r\n") == 1 && !std::strcmp(l.text(), "!arm");
        check(ok, "Zeilen: Ende mit \\n oder \\r\\n, leere Zeilen übergangen");
        ok = feed("!zulangezeile\n") == 0 && feed("!led 9\n") == 1 && !std::strcmp(l.text(), "!led 9");
        check(ok, "zu lange Zeile verworfen, die nächste gilt wieder");
        feed("!hal");
        l.reset();
        check(feed("!x\n") == 1 && !std::strcmp(l.text(), "!x"), "reset() verwirft die angefangene Zeile");
    }
    std::printf(fail ? "\nFEHLER\n" : "\nalles ok\n");
    return fail;
}
