// Host-Test 04-attitude: Die Startkalibrierung der Kreisel-Nullpunkte gilt nur im Stillstand.
// updateImu mit synthetischen Rohdaten (500 Hz, Rauschen ±0,2 °/s je Kreiselachse wie am RiderPi im
// Liegen gemessen: Spanne 0,3-0,43 °/s). Fall: beim Einschalten bewegt, der Nullpunkt stand um 12-20 °/s daneben.
// Aufruf: firmware/tests/run.sh test_gyro_boot_cal
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
    std::uint32_t us = 0, rng = 12345;
    double sensor_bias = -0.8;   // °/s, wie am RiderPi
    int n = 0;
    double noise() {   // gleichverteilt ±0,2 °/s, deterministisch
        rng = rng * 1664525u + 1013904223u;
        return (double(rng >> 8) / double(1u << 24) - 0.5) * 0.4;
    }
    // Art der Bewegung: 0 still, 1 Gier ungleichmäßig (Hand), 2 nur Rollkreisel, 3 nur Beschleunigung
    void run(double secs, int art) {
        const double kg = 1.0 / (0.000030517578125 * 2000.0), ka = 1.0 / (0.000030517578125 * 19.6);
        for (int k = 0; k < int(secs * 500); ++k) {
            us += 2000; ++n;
            const double ph = 2 * M_PI * 0.7 * n * 0.002;
            const double gz = art == 1 ? 15.0 + 10.0 * std::sin(ph) : 0.0;
            const double gx = art == 2 ? 6.0 * std::sin(ph) : 0.0;
            const double ax = art == 3 ? 0.6 * std::sin(ph * 5) : 0.0;
            std::uint8_t raw[12] = {};
            put(raw + 0, ax * ka);
            put(raw + 2, 0);
            put(raw + 4, 9.8 * ka);
            put(raw + 6, noise() * kg);
            put(raw + 8, (gx + noise()) * kg);
            put(raw + 10, (gz + sensor_bias + noise()) * kg);
            updateImu(imu, raw, 12, us, us / 1000, 253);
        }
    }
    double biasErr() const { return imu.gyro_bias[2] - sensor_bias; }
};

int main() {
    {
        Sim s; s.run(1.2, 0);
        check(!s.imu.calibrated, "nach 1,2 s noch nicht kalibriert (Beginn bei 0,75 s, 251 Proben)");
        s.run(0.2, 0);
        std::printf("       still: Nullpunkt %+.3f °/s (Sensor %+.3f), verworfen %d\n", s.imu.gyro_bias[2], s.sensor_bias, s.imu.calibration_tries);
        check(s.imu.calibrated && s.imu.calibration_tries == 0, "still: erstes Fenster gilt, Zeitpunkt wie im Original");
        check(std::fabs(s.biasErr()) < 0.03, "still: Nullpunkt trifft");
    }
    {
        Sim s; s.run(2.5, 1);
        check(!s.imu.calibrated && s.imu.calibration_tries >= 2, "beim Einschalten bewegt (Gier 5..25 °/s): Fenster verworfen");
        s.run(1.2, 0);
        std::printf("       bewegt, dann still: Nullpunkt %+.3f °/s, verworfen %d\n", s.imu.gyro_bias[2], s.imu.calibration_tries);
        check(s.imu.calibrated, "danach still: kalibriert nach höchstens zwei weiteren Fenstern");
        check(std::fabs(s.biasErr()) < 0.03, "Nullpunkt trifft trotz Bewegung beim Einschalten");
    }
    {
        Sim s; s.run(2.5, 2);
        check(!s.imu.calibrated, "nur Kippen (Rollkreisel ±6 °/s, Gier ruhig): verworfen");
        s.run(1.2, 0);
        check(s.imu.calibrated && std::fabs(s.biasErr()) < 0.03, "danach still: kalibriert, Nullpunkt trifft");
    }
    {
        Sim s; s.run(2.5, 3);
        check(!s.imu.calibrated, "nur Erschütterung (Beschleunigung ±0,6 m/s^2): verworfen");
    }
    {
        Sim s; s.run(10.0, 1);
        check(!s.imu.calibrated, "10 s dauernd bewegt: noch nicht kalibriert");
        s.run(3.0, 1);
        std::printf("       dauernd bewegt: kalibriert %d nach %d verworfenen Fenstern, Nullpunkt %+.2f °/s\n",
                    int(s.imu.calibrated), s.imu.calibration_tries, s.imu.gyro_bias[2]);
        check(s.imu.calibrated && s.imu.calibration_tries == kCalibrationMaxTries,
              "dauernd bewegt: nach 20 Fenstern gilt die Messung wie im Original (startet trotzdem)");
    }
    std::printf(fail ? "FEHLER\n" : "alle Tests ok\n");
    return fail;
}
