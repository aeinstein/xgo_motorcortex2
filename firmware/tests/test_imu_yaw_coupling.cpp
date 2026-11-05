// Host-Test für die Patches 0005/0006: updateImu mit synthetischen Rohdaten. Sensor gekippt wie
// beim balancierenden RiderPi (Roll +2,2°, Nick -1,1°) und nicht auf der Drehachse (d vor/hinter,
// 2 cm seitlich: Fliehkraft auf den Beschleunigungssensor). 5 s Stillstand (Kalibrierung),
// 3 s Drehung um die Senkrechte mit 40 bzw. 85 °/s in beide Richtungen, 3 s Stillstand. Die
// wahre Lage ändert sich nicht.
//   Original: Nick bis 5,6° daneben (am Rider gemessen +2..3° bei 40 °/s)
//   0005 (Kippraten ohne Gier-Anteil): Fliehkraft verfälscht bei 85 °/s noch ~1°
//   0006 (Beschleunigungswinkel bei > 25 °/s eingefroren): Grenze 0,15°
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_imu_yaw_coupling.cpp $S/control/rider_leg_imu.cpp -o t && ./t
#include "control/rider_leg_imu.h"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <initializer_list>

using namespace rider::control;

static void put(std::uint8_t *p, double v) {
    long r = std::lround(v);
    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;
    const auto u = std::uint16_t(std::int16_t(r));
    p[0] = std::uint8_t(u >> 8); p[1] = std::uint8_t(u & 0xff);
}

int main() {
    int fail = 0;
    const double roll = 2.2 * M_PI / 180.0, pitch = -1.1 * M_PI / 180.0;
    // Hochachse im Sensor-KS (Rohachsen): Firmware-Roll = atan(-AX/AZ), Nick = atan(-AY/AZ)
    double ux = -std::tan(roll), uy = -std::tan(pitch), uz = 1.0;
    const double n = std::sqrt(ux * ux + uy * uy + uz * uz);
    ux /= n; uy /= n; uz /= n;
    const double g = 9.8;
    const double ka = 1.0 / (0.000030517578125 * 19.6);   // m/s^2 -> Rohwert (±2 g)
    const double kg = 1.0 / (0.000030517578125 * 2000.0); // °/s -> Rohwert (±2000 °/s)
    const double e = 0.02;                                 // m seitlich der Drehachse (Sensor-x)
    for (double d : {0.11, -0.11}) {                       // m vor/hinter der Drehachse (Sensor-y)
        for (double omega : {40.0, -40.0, 85.0, -85.0}) {
            ImuState s;
            double p0 = 0, r0 = 0, dp = 0, dr = 0;
            std::uint32_t us = 0;
            for (int k = 0; k < 11 * 200; ++k) {
                const double t = k * 0.005;
                const double w = (t >= 5.0 && t < 8.0) ? omega : 0.0;
                const double w2 = std::pow(w * M_PI / 180.0, 2);   // Zentripetal: -w^2 * r (zur Achse)
                std::uint8_t raw[12];
                put(raw + 0, (g * ux - w2 * e) * ka);  // Firmware: acc[0] = -AX, acc[1] = -AY, acc[2] = +AZ
                put(raw + 2, (g * uy - w2 * d) * ka);
                put(raw + 4, g * uz * ka);
                put(raw + 6, w * ux * kg);             // gyro[1] (Nick) = +GX
                put(raw + 8, w * uy * kg);             // gyro[0] (Roll) = -GY
                put(raw + 10, w * uz * kg);            // gyro[2] (Gier) = +GZ
                us += 5000;
                updateImu(s, raw, 12, us, std::uint32_t(t * 1000) + 1000, 1);
                if (k == 5 * 200 - 1) { p0 = s.pitch_deg; r0 = s.roll_deg; }
                if (t >= 5.0) {
                    if (std::fabs(s.pitch_deg - p0) > std::fabs(dp)) dp = s.pitch_deg - p0;
                    if (std::fabs(s.roll_deg - r0) > std::fabs(dr)) dr = s.roll_deg - r0;
                }
            }
            std::printf("d %+5.2f m, Drehung %+4.0f °/s: Lage vorher Roll %+5.2f° Nick %+5.2f° | "
                        "größte Abweichung Nick %+6.2f°, Roll %+6.2f°\n", d, omega, r0, p0, dp, dr);
            if (std::fabs(dp) > 0.15 || std::fabs(dr) > 0.15) fail = 1;
        }
    }
    std::printf(fail ? "FEHLER: Drehung verfälscht die Lageschätzung\n" : "ok\n");
    return fail;
}
