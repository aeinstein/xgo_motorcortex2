// Host-Test für Patch 0013: Diagnose-Register 0x10 (Lagefusion). Prüft, dass updateImu die Zwischenwerte
// richtig ablegt (Beschleunigungswinkel; seit Patch 0021 Gewicht der Beschleunigung, Nullpunkte, Abzug der
// Querbeschleunigung statt Abzug 0005 / Einfrieren 0006) und 0x10 als 32-Byte-Antwort kommt.
// Nach firmware/build.sh, im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_diag.cpp $S/control/rider_leg_imu.cpp $S/command_protocol.cpp -o t && ./t
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
static void put(std::uint8_t* p, double v) {
    long r = std::lround(v);
    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;
    const auto u = std::uint16_t(std::int16_t(r));
    p[0] = std::uint8_t(u >> 8); p[1] = std::uint8_t(u & 0xff);
}

int main() {
    const double kg = 1.0 / (0.000030517578125 * 2000.0), ka = 1.0 / (0.000030517578125 * 19.6);
    ImuState s;
    std::uint32_t us = 0;
    const double tilt = 3.0 * M_PI / 180.0, roll = 2.0 * M_PI / 180.0;   // 3° Nick, 2° Roll
    auto step = [&](double gz) {
        std::uint8_t raw[12] = {};
        us += 2000;
        // Körperachsen: acceleration = [-AX, -AY, +AZ]; Nick = atan(acc[1]/acc[2]), Roll = atan(acc[0]/acc[2])
        put(raw + 0, -std::sin(roll) * 9.8 * ka);
        put(raw + 2, -std::sin(tilt) * 9.8 * ka);
        put(raw + 4, std::cos(tilt) * 9.8 * ka);
        put(raw + 6, 0); put(raw + 8, 0);
        put(raw + 10, gz * kg);
        updateImu(s, raw, 12, us, us / 1000, 1);
    };
    for (int k = 0; k < 3000; ++k) step(0);            // 6 s: Kalibrierung + Einschwingen
    const std::uint8_t n0 = s.diag_samples;
    step(0);
    const float exp_p = float(double(std::atan(s.acceleration[1] / s.acceleration[2])) * 57.2958);
    const float exp_r = float(double(std::atan(s.acceleration[0] / s.acceleration[2])) * 57.2958);
    check(s.diag_accel_pitch == exp_p && s.diag_accel_roll == exp_r, "Beschleunigungswinkel wie in updateImu");
    check(std::fabs(s.diag_accel_pitch - 3.0f) < 0.05f && std::fabs(s.diag_accel_roll - 2.0f) < 0.05f,
          "Beschleunigungswinkel 3° Nick / 2° Roll");
    // Patch 0021: Gewicht der Beschleunigung im Stand nahe 1 (Betrag 9,8 gegen kGravity 9,99), beim Drehen kleiner
    std::printf("       Gewicht im Stand %.3f\n", s.diag_weight);
    check(s.diag_weight > 0.8f && s.diag_weight <= 1.0f, "Gewicht der Beschleunigung im Stand > 0,8");
    check(s.bias_from_boot && std::fabs(s.gyro_bias[0]) < 0.05f && std::fabs(s.gyro_bias[1]) < 0.05f,
          "Nullpunkte Roll/Nick aus der Startkalibrierung (hier 0)");
    check(std::uint8_t(s.diag_samples - n0) == 1, "Probenzähler zählt je Probe");
    for (int k = 0; k < 200; ++k) step(80);             // 0,4 s Drehung mit 80 °/s
    std::printf("       Gewicht bei 80 °/s %.3f, Lage Nick %.2f° Roll %.2f°\n", s.diag_weight, s.pitch_deg, s.roll_deg);
    check(s.diag_weight > 0.2f && s.diag_weight < 0.5f, "Gewicht bei 80 °/s Drehung zwischen 0,2 und 0,5 (stufenlos, nichts eingefroren)");
    check(s.diag_lateral == 0.0f, "ohne frische Radticks kein Abzug der Querbeschleunigung");
    step(0);
    check(s.diag_weight > 0.8f, "nach der Drehung wieder volles Gewicht");

    // Register 0x10: 32 Byte, Inhalt aus Telemetry::imudiag
    using namespace rider2::protocol;
    Telemetry t;
    for (int i = 0; i < 32; ++i) t.imudiag[i] = std::uint8_t(i * 7 + 1);
    CommandEvent e; e.type = 2; e.reg = 0x10; e.length = 1; e.data[0] = 255;
    std::uint8_t frame[64];
    const size_t n = buildReadResponse(e, t, frame, sizeof frame);
    check(n == 32 + 8 && frame[4] == 0x10 && !std::memcmp(frame + 5, t.imudiag, 32) && frame[n - 1] == 0xaa,
          "Antwort auf 0x10: 32 Byte aus imudiag");
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
