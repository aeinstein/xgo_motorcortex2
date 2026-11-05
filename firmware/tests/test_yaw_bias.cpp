// Host-Test für Patch 0009: Gyro-z-Nullpunkt laufend nachführen (trackYawBias) + Totband 0,3 °/s.
// updateImu mit synthetischen Rohdaten (500 Hz, Rauschen ±0,15 °/s, Rohwert-Quantisierung), Encoder
// mit 100 Hz passend zur wahren Drehung. Fall wie am RiderPi 2026-11-05: Startkalibrierung bei
// 0,2 °/s Sensor-Nullpunkt, danach durch Erwärmung 0,68 °/s (Fehler 0,48 °/s).
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_yaw_bias.cpp $S/control/rider_leg_imu.cpp -o t && ./t
#include "control/rider_leg_imu.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

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

struct Sim {
    ImuState imu;
    YawBiasTracker tr;
    std::uint32_t us = 0;
    double true_yaw = 0, sensor_bias = 0.2, enc_yaw = 0, slip = 1.0;
    std::int32_t e11 = 0, e21 = 0;
    std::uint32_t rng = 12345;
    int n = 0;
    double noise() {   // gleichverteilt ±0,15 °/s, deterministisch
        rng = rng * 1664525u + 1013904223u;
        return (double(rng >> 8) / double(1u << 24) - 0.5) * 0.3;
    }
    // secs Sekunden mit wahrer Gierrate omega (°/s); tilt_amp_deg: Kippen um die Querachse (getragen)
    void run(double secs, double omega, YawReference ref, double tilt_amp_deg = 0) {
        const double kg = 1.0 / (0.000030517578125 * 2000.0), ka = 1.0 / (0.000030517578125 * 19.6);
        for (int k = 0; k < int(secs * 500); ++k) {
            us += 2000; ++n;
            true_yaw += omega * 0.002;
            enc_yaw += omega * slip * 0.002;
            const double tilt = tilt_amp_deg * M_PI / 180.0 * std::sin(2 * M_PI * 1.0 * n * 0.002);
            std::uint8_t raw[12] = {};
            put(raw + 0, 0);
            put(raw + 2, -std::sin(tilt) * 9.8 * ka);
            put(raw + 4, std::cos(tilt) * 9.8 * ka);
            put(raw + 6, 0); put(raw + 8, 0);
            put(raw + 10, (omega + sensor_bias + noise()) * kg);
            updateImu(imu, raw, 12, us, us / 1000, 1);
            if (n % 5 == 0) {   // Encoder mit 100 Hz
                const double ticks = enc_yaw / kWheelYawDegPerTick;
                e11 = std::int32_t(std::lround(-ticks / 2)); e21 = std::int32_t(std::lround(ticks / 2));
            }
            trackYawBias(imu, tr, ref, e11, e21, us);
        }
    }
    double biasErr() const { return sensor_bias - imu.gyro_bias[2]; }
    double yawErr() const { return imu.yaw_deg - true_yaw; }
};

int main() {
    // Startkalibrierung im Stillstand (Referenz Still würde sofort nachführen -> None)
    auto boot = [](Sim& s) { s.run(3.0, 0, YawReference::None); s.sensor_bias = 0.68; };
    {
        Sim s; boot(s);
        check(s.imu.calibrated && std::fabs(s.imu.gyro_bias[2] - 0.2) < 0.03, "Startkalibrierung trifft 0,2 °/s");
        s.run(60, 0, YawReference::None);
        std::printf("       ohne Nachführung, 60 s still, Nullpunkt 0,48 °/s daneben: Kurs %+.1f°\n", s.yawErr());
        check(s.yawErr() > 20, "ohne Nachführung läuft der Kurs weg (Totband 0,3 °/s verdeckt den Fehler nicht mehr)");
    }
    {
        Sim s; boot(s);
        s.run(60, 0, YawReference::Still);
        std::printf("       Still, 60 s: Nullpunktfehler %+.3f °/s, Kurs %+.2f°, %u Nachführungen\n", s.biasErr(), s.yawErr(), s.tr.updates);
        check(std::fabs(s.biasErr()) < 0.03, "Still: Nullpunkt nach 60 s auf 0,03 °/s");
        check(std::fabs(s.yawErr()) < 4.0, "Still: Kurs bleibt unter 4°");
        const double y = s.imu.yaw_deg; s.run(60, 0, YawReference::Still);
        check(std::fabs(s.imu.yaw_deg - y) < 0.3, "Still: danach 60 s Liegen < 0,3° Drift");
    }
    {
        Sim s; boot(s);   // balanciert, langsames Wegdrehen wie im Stand
        s.run(10, 1.0, YawReference::Wheels); s.run(10, 0, YawReference::Wheels);
        s.run(10, -1.5, YawReference::Wheels); s.run(15, 0.4, YawReference::Wheels);
        std::printf("       Räder, 45 s: Nullpunktfehler %+.3f °/s, Kurs %+.2f° (wahr %+.1f°)\n", s.biasErr(), s.yawErr(), s.true_yaw);
        check(std::fabs(s.biasErr()) < 0.05, "Räder: Nullpunkt nach 45 s auf 0,05 °/s");
        const double e0 = s.yawErr(); s.run(30, 0.8, YawReference::Wheels);
        check(std::fabs(s.yawErr() - e0) < 1.0, "Räder: danach 30 s langsames Drehen, Kursfehler wächst < 1°");
        // schnelle Drehung, Räder rutschen 10 %: kein Einfluss auf den Nullpunkt
        const double b0 = s.imu.gyro_bias[2]; s.slip = 1.1;
        for (int k = 0; k < 4; ++k) { s.run(3, k % 2 ? -80 : 80, YawReference::Wheels); s.run(0.5, 0, YawReference::Wheels); }
        check(std::fabs(s.imu.gyro_bias[2] - b0) < 0.02, "Räder: schnelle Drehungen mit Schlupf verschieben den Nullpunkt nicht");
    }
    {
        Sim s; boot(s);   // getragen: kippt ±5° mit 1 Hz und dreht langsam
        const double b0 = s.imu.gyro_bias[2];
        s.run(30, 1.0, YawReference::Still, 5.0);
        check(std::fabs(s.imu.gyro_bias[2] - b0) < 0.01 && s.tr.updates == 0, "Still: getragen (Kippen) -> keine Nachführung");
    }
    {
        Sim s; s.sensor_bias = 0.2; s.run(3.0, 0, YawReference::None);   // richtiger Nullpunkt
        s.run(20, 0.5, YawReference::None);
        std::printf("       0,5 °/s für 20 s (wahr 10°): Kurs %+.2f°\n", s.imu.yaw_deg - (s.true_yaw - 10));
        check(std::fabs(s.yawErr()) < 0.6, "Totband 0,3 °/s: langsames Drehen (0,5 °/s) wird mitgezählt (Original: 0°)");
    }
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
