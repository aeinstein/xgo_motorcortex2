// Host-Test für Patch 0021 (neue Lagefusion): echte Rohwerte vom RiderPi laufen durch updateImu.
// Prüfdaten firmware/tests/data/lagefusion_163849.txt (erzeugt von firmware/tests/lagefusion_ref.py aus den
// Mitschnitten von Register 0x15, 2026-11-18): 60 s liegend (Startkalibrierung), 1 s künstliches Aufrichten,
// dann die Messfahrt 163849 (60 s balancierend still, Drehen im Stand, geradeaus, Bremsdrehungen aus
// 0,5 m/s mit 90 °/s). Je Zeile die 12 Rohbytes, Radticks und die Lage aus der Python-Referenz.
//   1. updateImu liefert dieselbe Lage wie die Referenz (Nick/Roll auf 0,05°, Kurs auf 0,3°).
//   2. Startkalibrierung im Liegen setzt alle drei Nullpunkte (Nick-Kreisel RiderPi ~-1,15 °/s).
//   3. Bremsdrehungen: Roll bleibt innerhalb 1,5° um den Stand davor (bis 0020: bis 4,9° daneben).
//   4. Nach jeder Bewegung liegt die Lage wieder beim Beschleunigungssensor (3 s danach, auf 1°).
// Nach firmware/build.sh, im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_lagefusion_replay.cpp $S/control/rider_leg_imu.cpp -o t && ./t [Datei]
#include "control/rider_leg_imu.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

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

struct Row { double t; float pitch, roll, acc_pitch, acc_roll, gz; };

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "firmware/tests/data/lagefusion_163849.txt";
    std::FILE* f = std::fopen(path, "r");
    if (!f) { std::printf("FEHLER Prüfdaten fehlen: %s\n", path); return 1; }
    ImuState s;
    char line[256];
    double dp = 0, dr = 0, dy = 0;
    long n = 0, first_fresh = -1;
    float bias_at_cal[3] = {0, 0, 0};
    bool was_cal = false, boot = false;
    std::vector<Row> rows;      // nur die Fahrt
    double t_fresh0 = 0;
    while (std::fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        unsigned long now_us; int r[6], w11, w21, fresh; double rp, rr, ry;
        if (std::sscanf(line, "%lu %d %d %d %d %d %d %d %d %d %lf %lf %lf", &now_us, &r[0], &r[1], &r[2], &r[3], &r[4],
                        &r[5], &w11, &w21, &fresh, &rp, &rr, &ry) != 13) continue;
        std::uint8_t raw[12];
        for (int k = 0; k < 6; ++k) put(raw + 2 * k, r[k]);
        s.wheel_ticks[0] = w11; s.wheel_ticks[1] = w21; s.wheels_fresh = fresh != 0;
        updateImu(s, raw, 12, std::uint32_t(now_us), std::uint32_t(now_us / 1000 + 1000), 1);
        if (s.calibrated && !was_cal) {
            was_cal = true; boot = s.bias_from_boot;
            for (int k = 0; k < 3; ++k) bias_at_cal[k] = s.gyro_bias[k];
        }
        dp = std::fmax(dp, std::fabs(s.pitch_deg - rp));
        dr = std::fmax(dr, std::fabs(s.roll_deg - rr));
        dy = std::fmax(dy, std::fabs(s.yaw_deg - ry));
        if (fresh) {
            if (first_fresh < 0) { first_fresh = n; t_fresh0 = now_us * 1e-6; }
            rows.push_back({now_us * 1e-6 - t_fresh0, s.pitch_deg, s.roll_deg, s.diag_accel_pitch, s.diag_accel_roll,
                            s.gyro_deg_s[2] - s.gyro_bias[2]});
        }
        ++n;
    }
    std::fclose(f);
    std::printf("       %ld Zeilen, davon Fahrt %zu; größte Abweichung von der Referenz: Nick %.4f° Roll %.4f° Kurs %.3f°\n",
                n, rows.size(), dp, dr, dy);
    check(n > 20000 && rows.size() > 15000, "Prüfdaten vollständig");
    check(dp < 0.05 && dr < 0.05 && dy < 0.3, "updateImu = Python-Referenz (Nick/Roll 0,05°, Kurs 0,3°)");
    std::printf("       Startkalibrierung: Nullpunkte Roll %+.3f Nick %+.3f Gier %+.3f °/s, im Stillstand %d; am Ende Roll %+.3f Nick %+.3f\n",
                bias_at_cal[0], bias_at_cal[1], bias_at_cal[2], int(boot), s.gyro_bias[0], s.gyro_bias[1]);
    check(was_cal && boot, "Startkalibrierung im Liegen angenommen");
    check(std::fabs(bias_at_cal[1] + 1.14f) < 0.1f && std::fabs(bias_at_cal[0]) < 0.15f && std::fabs(bias_at_cal[2] + 0.77f) < 0.1f,
          "Nullpunkte aller drei Achsen gesetzt (Nick ~-1,14, Roll ~0, Gier ~-0,77 °/s)");
    check(std::fabs(s.gyro_bias[1] + 1.15f) < 0.25f && std::fabs(s.gyro_bias[0]) < 0.25f, "Nachführung hält die Nullpunkte über die Fahrt");

    // Abschnitte der Fahrt: Beginn in s ab Mitschnittbeginn (aus den Zeiten der Messfahrt, t0 = 76515,9 s)
    auto mean = [&](double a, double b, int what) {
        double sum = 0; long c = 0;
        for (const auto& x : rows) if (x.t >= a && x.t < b) {
            sum += what == 0 ? x.pitch : what == 1 ? x.roll : what == 2 ? x.acc_pitch : x.acc_roll; ++c; }
        return c ? sum / c : 0.0;
    };
    struct Seg { const char* name; double turn; bool check_roll; };
    const double t0 = 76515.866;   // Pi-Zeit der ersten Fahrtzeile
    const Seg segs[] = {{"Bremsdrehung links", 76679.81 - t0, true}, {"Bremsdrehung rechts", 76694.68 - t0, true},
                        {"erst bremsen, links", 76721.32 - t0, true}, {"erst bremsen, rechts", 76737.06 - t0, true},
                        {"Drehen im Stand links", 76603.7 - t0, true}, {"Drehen im Stand rechts", 76610.1 - t0, true}};
    for (const auto& g : segs) {
        const double before = mean(g.turn - 4.5, g.turn - 3.0, 1);
        double worst = 0, rate = 0;
        for (const auto& x : rows) if (x.t >= g.turn - 0.5 && x.t < g.turn + 3.0) {
            worst = std::fmax(worst, std::fabs(x.roll - before)); rate = std::fmax(rate, std::fabs(x.gz)); }
        const double end_p = mean(g.turn + 3.0, g.turn + 3.6, 0) - mean(g.turn + 3.0, g.turn + 3.6, 2);
        const double end_r = mean(g.turn + 3.0, g.turn + 3.6, 1) - mean(g.turn + 3.0, g.turn + 3.6, 3);
        char text[160];
        std::snprintf(text, sizeof text, "%s (bis %.0f °/s): Roll höchstens %.2f° vom Stand, danach Nick %+.2f° Roll %+.2f° zur Beschleunigung",
                      g.name, rate, worst, end_p, end_r);
        check(rate > 60 && worst < 1.5 && std::fabs(end_p) < 1.0 && std::fabs(end_r) < 1.0, text);
    }
    // Stand (60 s balancierend still, ab 3 s nach Mitschnittbeginn): Lage = Beschleunigung im Mittel
    const double sp = mean(3, 58, 0) - mean(3, 58, 2), sr = mean(3, 58, 1) - mean(3, 58, 3);
    std::printf("       Stand: Nick %+.2f° (Beschleunigung %+.2f°), Unterschied %+.2f°; Roll-Unterschied %+.2f°\n",
                mean(3, 58, 0), mean(3, 58, 2), sp, sr);
    check(std::fabs(sp) < 0.4 && std::fabs(sr) < 0.4, "im Stand kein Versatz mehr zwischen Lage und Beschleunigung (vorher ~2° im Nick)");
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
