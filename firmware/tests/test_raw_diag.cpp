// Host-Test 03-diagnostics: Register 0x15 (laufende Summen der IMU-Rohwerte). Prüft, dass updateImu
// jede Probe mit Vorzeichen und Achsen wie acceleration/gyro_deg_s aufsummiert, dass die Summen beim
// Überlauf richtige Differenzen geben, dass die Lagerechnung davon unberührt bleibt und dass 0x15 als
// 32-Byte-Antwort kommt.
// Aufruf: firmware/tests/run.sh test_raw_diag
#include "control/rider_leg_imu.h"
#include "command_protocol.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace rider::control;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}
static void put(std::uint8_t* p, int v) {
    const auto u = std::uint16_t(std::int16_t(v));
    p[0] = std::uint8_t(u >> 8); p[1] = std::uint8_t(u & 0xff);
}

int main() {
    ImuState s;
    std::uint32_t us = 1000;
    // Rohwörter AX, AY, AZ, GX, GY, GZ (Sensorachsen)
    auto step = [&](int ax, int ay, int az, int gx, int gy, int gz) {
        std::uint8_t raw[12];
        put(raw, ax); put(raw + 2, ay); put(raw + 4, az); put(raw + 6, gx); put(raw + 8, gy); put(raw + 10, gz);
        us += 3000;
        return updateImu(s, raw, 12, us, us / 1000, 1);
    };
    check(step(100, -200, 16000, 30, -40, 50), "updateImu nimmt die Probe");
    check(s.raw_samples == 1 && s.raw_us == us, "Zähler 1, Messzeit der Probe");
    check(std::int32_t(s.raw_accel_sum[0]) == -100 && std::int32_t(s.raw_accel_sum[1]) == 200 &&
          std::int32_t(s.raw_accel_sum[2]) == 16000, "Beschleunigung: [-AX, -AY, +AZ]");
    check(std::int32_t(s.raw_gyro_sum[0]) == 40 && std::int32_t(s.raw_gyro_sum[1]) == 30 &&
          std::int32_t(s.raw_gyro_sum[2]) == 50, "Kreisel: [-GY, +GX, +GZ]");
    // Zählwert und Gleitkommawert derselben Probe passen zusammen
    check(std::fabs(s.acceleration[1] - 200 * 19.6f / 32768.0f) < 1e-5f &&
          std::fabs(s.gyro_deg_s[0] - 40 * 2000.0f / 32768.0f) < 1e-4f, "Maßstab 19,6/32768 m/s^2 und 2000/32768 °/s");
    check(step(-32768, -32768, 32767, -32768, -32768, 32767), "Probe am Anschlag");
    check(std::int32_t(s.raw_accel_sum[0]) == -100 + 32768 && std::int32_t(s.raw_gyro_sum[0]) == 40 + 32768,
          "-(-32768) läuft nicht über");
    // Überlauf: 70000 Proben mit +AZ 32767 -> Summe > 2^31; Differenz zweier Lesungen bleibt richtig
    for (int k = 0; k < 70000; ++k) step(0, 0, 32767, 0, 0, 0);
    const std::uint32_t z0 = s.raw_accel_sum[2];
    const std::uint16_t n0 = s.raw_samples;
    for (int k = 0; k < 7; ++k) step(0, 0, -1234, 0, 0, 0);
    check(std::int32_t(s.raw_accel_sum[2] - z0) == 7 * -1234 && std::uint16_t(s.raw_samples - n0) == 7,
          "Differenz nach Überlauf: 7 Proben, Mittel -1234");

    // Die Lagerechnung bleibt gleich: zwei Zustände mit verschiedenen Summen-Startwerten liefern dieselbe Lage
    ImuState a, b;
    b.raw_accel_sum[1] = 0xfffffff0u; b.raw_samples = 65530;
    std::uint32_t t = 0;
    for (int k = 0; k < 3000; ++k) {
        std::uint8_t raw[12];
        put(raw, -300); put(raw + 2, -800 + (k % 7)); put(raw + 4, 16300); put(raw + 6, 20); put(raw + 8, -10); put(raw + 10, k > 2000 ? 900 : 5);
        t += 3000;
        updateImu(a, raw, 12, t, t / 1000, 1); updateImu(b, raw, 12, t, t / 1000, 1);
    }
    check(a.pitch_deg == b.pitch_deg && a.roll_deg == b.roll_deg && a.yaw_deg == b.yaw_deg, "Lage unabhängig von den Summen");

    // Register 0x15: 32 Byte, Inhalt aus Telemetry::rawdiag
    using namespace rider2::protocol;
    Telemetry tel;
    for (int i = 0; i < 32; ++i) tel.rawdiag[i] = std::uint8_t(i * 5 + 3);
    CommandEvent e; e.type = 2; e.reg = 0x15; e.length = 1; e.data[0] = 255;
    std::uint8_t frame[64];
    const size_t n = buildReadResponse(e, tel, frame, sizeof frame);
    check(n == 32 + 8 && frame[4] == 0x15 && !std::memcmp(frame + 5, tel.rawdiag, 32) && frame[n - 1] == 0xaa,
          "Antwort auf 0x15: 32 Byte aus rawdiag");
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
