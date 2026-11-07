// Host-Test für Patch 0012: Roll-Trimm aus dem Nivellieren bleibt stehen, auch über Höhenänderungen.
// RiderPi 2026-11-07: nach jeder Höhenänderung wieder ~2° Roll, weil das Rollziel nach dem Nivellieren auf
// den Rollbefehl abklang und nur die Totzone der Beinservos ihn gerade hielt.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_roll_trim.cpp $S/control/rider_leg_imu.cpp -o t && ./t
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

static const std::uint16_t ZA = 600, ZB = 400;   // Beinnullpunkte (beliebig, gleich für Referenz und Patch)

// Originalformel R-1.1.6 (vor 0012) für den Vergleich ohne Nivellieren
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (hi < v ? hi : v); }
static std::uint32_t ticks(float e) {
    const double angle = std::acos((double(e) * 0.5) / 48.0);
    return std::uint32_t(((1.1408376963350786 - angle) / 6.283185307179586) * 1024.0);
}
static void reference(LegState& s, float height, float roll) {
    s.roll_target_deg = float(double(s.roll_target_deg) * 0.95 + double(roll) * 0.050000000000000044);
    s.roll_target_deg = clampf(s.roll_target_deg, -20.0f, 20.0f);
    const double d = std::sin(double(s.roll_target_deg) / 57.3) * 43.0;
    s.extension_a = clampf(float(double(height) + d), 0.0f, 36.0f) + 45.0f;
    s.extension_b = clampf(float(double(height) - d), 0.0f, 36.0f) + 45.0f;
    s.servo_a = std::uint16_t(std::uint32_t(ZA) - ticks(s.extension_a));
    s.servo_b = std::uint16_t(std::uint32_t(ZB) + ticks(s.extension_b));
}

static void run(LegState& s, int n, float height, float roll, bool leveling = false, float inc = 0) {
    for (int k = 0; k < n; ++k) updateLegTargets(s, height, roll, leveling, inc, ZA, ZB);
}

int main() {
    {   // ohne Nivellieren: bitgleich zum Original, über Höhen- und Rollwechsel
        LegState p, r;
        bool same = true;
        const float seq[][2] = {{18, 0.078f}, {10, 0.078f}, {30, 0.078f}, {30, 3.0f}, {5, -4.0f}, {18, 0.078f}};
        for (const auto& hr : seq)
            for (int k = 0; k < 150; ++k) {
                updateLegTargets(p, hr[0], hr[1], false, 0, ZA, ZB);
                reference(r, hr[0], hr[1]);
                same = same && p.roll_target_deg == r.roll_target_deg && p.servo_a == r.servo_a &&
                       p.servo_b == r.servo_b && p.extension_a == r.extension_a && p.extension_b == r.extension_b;
            }
        check(same && p.roll_trim_deg == 0, "ohne Nivellieren bitgleich zum Original (900 Takte, Höhe/Roll wechselnd)");
    }
    {
        LegState s;
        run(s, 100, 18, 0.078f);
        run(s, 50, 18, 0.078f, true, -0.0416f);          // Nivellieren zieht das Ziel auf ~ -2°
        const float lvl = s.roll_target_deg;
        std::printf("       nach dem Nivellieren: Ziel %+.2f°, Trimm %+.2f°\n", lvl, s.roll_trim_deg);
        check(std::fabs(lvl + 2.0f) < 0.1f, "Nivellieren erreicht ~ -2°");
        run(s, 200, 18, 0.078f);                         // aus
        std::printf("       200 Takte später: Ziel %+.2f° (Original: ~+0,08°)\n", s.roll_target_deg);
        check(std::fabs(s.roll_target_deg - lvl) < 1e-4f, "nach dem Abschalten bleibt das Ziel stehen");
        const int diff18 = int(s.servo_a) + int(s.servo_b);
        run(s, 300, 30, 0.078f);                         // Höhe ändern
        check(std::fabs(s.roll_target_deg - lvl) < 1e-4f, "Höhe 18 -> 30: Rollziel bleibt beim Trimm");
        LegState ref; ref.roll_trim_deg = s.roll_trim_deg; ref.roll_target_deg = s.roll_target_deg;
        run(ref, 1, 30, 0.078f);
        check(s.servo_a == ref.servo_a && s.servo_b == ref.servo_b, "Höhe 30: Servos wie mit Trimm berechnet");
        LegState flat; run(flat, 300, 30, 0.078f);
        std::printf("       Servosumme bei 18: %d, Servos bei 30 mit/ohne Trimm: %u/%u bzw. %u/%u\n", diff18,
                    s.servo_a, s.servo_b, flat.servo_a, flat.servo_b);
        check(s.servo_a != flat.servo_a || s.servo_b != flat.servo_b, "Höhe 30: Trimm wirkt (Servos anders als ohne)");
        run(s, 300, 30, 3.0f);                           // Rollbefehl +3°
        check(std::fabs(s.roll_target_deg - (3.0f + s.roll_trim_deg)) < 1e-3f, "Rollbefehl +3°: Ziel = Befehl + Trimm");
        s = LegState{};                                  // Reset wie MotionController::reset (Aufstehen/Fallen)
        check(s.roll_trim_deg == 0 && s.roll_target_deg == 0, "Reset: Trimm 0");
    }
    {   // Grenze: Trimm höchstens ±10°
        LegState s;
        run(s, 600, 18, 0.0f, true, 0.1f);               // Ziel läuft bis +20° (Clamp des Originals)
        check(s.roll_target_deg == 20.0f && s.roll_trim_deg == 10.0f, "Trimm auf +10° begrenzt");
        run(s, 300, 18, 0.0f);
        check(std::fabs(s.roll_target_deg - 10.0f) < 1e-3f, "danach Ziel = Trimm 10°");
    }
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
