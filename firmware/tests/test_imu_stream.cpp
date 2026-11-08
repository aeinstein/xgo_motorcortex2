// Host-Test für Patch 0014: IMU-/Rad-Datenstrom (imu_stream.h). Zeitsteuerung mit seq, Ring
// (voll -> verwerfen, Reihenfolge), Paketaufbau mit Rahmen/Prüfsumme und Rückparsen, Register 0x11.
// Nach firmware/build.sh, im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_stream.cpp $S/command_protocol.cpp -o t && ./t
#include "imu_stream.h"
#include "command_protocol.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace rider2;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}
static std::uint32_t u32(const std::uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | std::uint32_t(p[3]) << 24; }
static float f32(const std::uint8_t* p) { std::uint32_t b = u32(p); float f; std::memcpy(&f, &b, 4); return f; }

int main() {
    using namespace imustream;
    check(periodForHz(0) == 0 && periodForHz(100) == 10000 && periodForHz(50) == 20000 && periodForHz(250) == 4000 &&
          periodForHz(255) == 4000, "Sollabstand: 0 aus, 100 Hz 10 ms, 50 Hz 20 ms, >250 Hz -> 4 ms");

    // IMU wie am RiderPi alle 3 ms: 100 Hz im Mittel, Abstände 9 oder 12 ms
    Producer pr;
    int sent = 0, bad_gap = 0;
    std::uint32_t last = 0, first = 0;
    for (int k = 0; k < 1000; ++k) {             // 3 s
        const std::uint32_t t = 0xfffff000u + std::uint32_t(k) * 3000u;   // mit Überlauf von micros()
        if (pr.due(10000, t)) {
            if (sent) { const std::uint32_t d = t - last; if (d != 9000 && d != 12000) ++bad_gap; } else first = t;
            last = t; ++sent;
        }
    }
    const double hz = (sent - 1) / ((last - first) * 1e-6);
    std::printf("       3-ms-IMU, Soll 100 Hz: %d Pakete, %.2f Hz\n", sent, hz);
    check(sent >= 299 && sent <= 301 && pr.seq == sent && bad_gap == 0 && hz > 99.5 && hz < 100.5,
          "Zeitsteuerung: 100 Hz im Mittel bei 3-ms-IMU, Abstände 9/12 ms, über den micros()-Überlauf");
    check(!pr.due(0, 5000) && !pr.on, "aus: nichts fällig");
    check(pr.due(10000, 1000000000u) && !pr.due(10000, 1000003000u), "nach langer Pause neu aufsetzen statt nachholen");

    Ring<16> r;
    Sample s;
    int pushed = 0;
    for (int k = 0; k < 20; ++k) { s.seq = std::uint16_t(k); pushed += r.push(s); }
    check(pushed == 16, "Ring voll nach 16, Rest verworfen");
    bool order = true;
    for (int k = 0; k < 16; ++k) order = order && r.pop(s) && s.seq == k;
    check(order && !r.pop(s), "Reihenfolge erhalten, danach leer");

    Sample x;
    x.seq = 0xbeef; x.t_us = 0x12345678u; x.roll = -1.25f; x.pitch = 2.5f; x.yaw = 181.0f;
    x.w11 = -123456; x.w21 = 7654321;
    std::uint8_t out[64];
    const std::size_t n = buildPacket(x, out, sizeof out);
    check(n == kFrame && n == 34, "Paket 34 Byte");
    check(out[0] == 0x55 && out[1] == 0 && out[2] == 34 && out[3] == 0x14 && out[4] == 0x11 && out[32] == 0 && out[33] == 0xaa,
          "Rahmen 55 00 len 14 11 ... 00 AA");
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i < n - 2; ++i) sum = std::uint8_t(sum + out[i]);
    check(sum == 0xff, "Prüfsumme wie bei Antworten (Summe = FF)");
    const std::uint8_t* d = out + 5;
    check((d[0] | d[1] << 8) == 0xbeef && u32(d + 2) == 0x12345678u && f32(d + 6) == -1.25f && f32(d + 10) == 2.5f &&
          f32(d + 14) == 181.0f && std::int32_t(u32(d + 18)) == -123456 && std::int32_t(u32(d + 22)) == 7654321,
          "Inhalt little endian: seq, t_us, Roll/Nick/Kurs, Räder");
    check(buildPacket(x, out, 33) == 0, "zu kleiner Puffer -> 0");

    // Schreibbefehl auf 0x11 wird vom Parser angenommen
    protocol::StreamParser p;
    protocol::CommandEvent e;
    std::uint8_t frame[16];
    const std::uint8_t rate = 100;
    const std::size_t fn = protocol::buildFrame(1, 0x11, &rate, 1, frame, sizeof frame);
    bool got = false;
    for (std::size_t i = 0; i < fn; ++i) got = p.feed(protocol::Source::Aux, frame[i], 1000, e) || got;
    check(got && protocol::commandKind(e) == protocol::CommandKind::ImuStream && e.data[0] == 100,
          "Schreibbefehl 0x11 = 100 wird als imu_stream angenommen");
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
