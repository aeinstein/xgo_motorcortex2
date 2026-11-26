// Host-Test 07-balance: In schnellen Kurven einlegen (control::updateCurveLean, MotionRequest::roll_target_deg).
// Ziel = -atan(v * Gierrate / g): Vorzeichen, Betrag, Grenzen (6°, Beine), Tiefpass + Rate, aus = zurück auf 0.
// Regelkreis wie test_roll_hold.cpp: Körper-Roll = Störung + Vorzeichen * Roll-Sollwert der Beine; mit Ziel geht
// der Körper-Roll auf das Ziel, der Roll-Trimm nimmt die Schräglage nicht auf, Ziel 0 ist bitgleich zu ohne.
// Aufruf: firmware/tests/run.sh test_curve_lean
#include "motion_controller.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

using rider::control::CurveLean;
using rider::control::updateCurveLean;
using rider::firmware::MotionController;
using rider::firmware::MotionInput;
using rider::firmware::MotionRequest;
using rider::firmware::MotionResult;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

static constexpr float kDt = 0.003f;   // Regeltakt ~333 Hz
static constexpr float kHeight = 18;   // Mittelstellung (Byte 128)

static float settle(float v, float w_deg_s, float height = kHeight, float seconds = 3.0f) {
    CurveLean s;
    for (int k = 0; k < int(seconds / kDt); ++k) updateCurveLean(s, true, v, w_deg_s, height, kDt);
    return s.target_deg;
}

static void testTarget() {
    const float want = -std::atan(0.5f * 90.0f * 0.017453292f / 9.81f) * 57.29578f;   // -4,57°
    const float a = settle(0.5f, 90);
    std::printf("       0,5 m/s, 90 °/s: Ziel %.3f° (erwartet %.3f°)\n", a, want);
    check(std::fabs(a - want) < 0.01f, "0,5 m/s bei 90 °/s: Ziel -atan(v*w/g) = -4,57°");
    check(settle(0.5f, -90) > 4.5f, "Drehung rechts: Ziel positiv");
    check(settle(-0.5f, 90) > 4.5f, "rückwärts: Vorzeichen dreht sich um");
    check(settle(0, 120) == 0.0f, "auf der Stelle (v = 0): kein Einlegen");
    check(std::fabs(settle(1.5f, 150) + rider::control::kLeanMaxDeg) < 1e-4f, "Betrag begrenzt auf 6°");
    const float low = settle(1.5f, 150, 15.0f);   // Platz 15 - 10 = 5 -> asin(5/43) = 6,68° > 6
    const float lower = settle(1.5f, 150, 12.0f); // Platz 2 -> asin(2/43) = 2,67°
    check(std::fabs(low + 6.0f) < 1e-4f && std::fabs(lower + 2.666f) < 0.01f, "Beingrenze: bei niedriger Höhe weniger Schräglage");
    check(settle(1.5f, 150, 9.0f) == 0.0f && settle(1.5f, 150, 27.0f) == 0.0f, "Bein näher als 10 an 0 bzw. 36: kein Einlegen");
    check(settle(NAN, 90) == 0.0f && settle(0.5f, NAN) == 0.0f, "ungültige Eingänge: Ziel 0");
}

static void testDynamics() {
    CurveLean s;
    float max_step = 0, prev = 0;
    for (int k = 0; k < 400; ++k) {
        updateCurveLean(s, true, 1.5f, 150, kHeight, kDt);
        max_step = std::fmax(max_step, std::fabs(s.target_deg - prev));
        prev = s.target_deg;
    }
    check(max_step <= rider::control::kLeanRateDegS * kDt + 1e-6f, "Sprung: Ziel ändert sich höchstens 20 °/s");
    const float before = s.target_deg;
    updateCurveLean(s, true, 1.5f, 150, kHeight, 0.0f);
    updateCurveLean(s, true, 1.5f, 150, kHeight, 0.5f);
    check(s.target_deg == before, "dt 0 oder > 0,1 s: keine Änderung");
    for (int k = 0; k < 1000; ++k) updateCurveLean(s, false, 1.5f, 150, kHeight, kDt);
    check(std::fabs(s.target_deg) < 1e-3f, "aus: Ziel läuft auf 0 zurück (3 s)");
}

struct Run {
    MotionController c;
    MotionResult r;
    float roll = 0;
};

static void step(Run& run, std::uint32_t k, bool roll_hold, float target, float disturbance, float sign) {
    MotionInput in;
    in.now_ms = 3000 + 3 * k; in.mode = 1; in.calibrated = true;
    in.roll_deg = run.roll; in.yaw_deg = 0.01f * float(k); in.speed11 = in.speed21 = 40;
    MotionRequest rq;
    rq.command.forward = 170; rq.command.turn = 200;
    rq.roll_hold = roll_hold; rq.roll_target_deg = target; rq.drive_integral = true; rq.turn_hold = true;
    run.r = run.c.step(in, rq);
    run.roll = disturbance + sign * run.r.legs.roll_target_deg;
}

static void testClosedLoop() {
    float sign = 0;
    for (float s : {-1.0f, 1.0f}) {
        Run r;
        for (std::uint32_t k = 0; k < 2000; ++k) step(r, k, true, 0, 2.0f, s);
        if (std::fabs(r.roll) < 0.2f) sign = s;
    }
    check(sign != 0, "Roll halten gleicht 2° aus (Vorzeichen der Beine bestimmt)");

    Run lean;
    for (std::uint32_t k = 0; k < 2000; ++k) step(lean, k, true, -4.5f, 2.0f, sign);
    check(std::fabs(lean.roll + 4.5f) < 0.2f, "mit Ziel -4,5°: Körper-Roll geht auf -4,5°");
    // Roll-Trimm = Beine - Rollbefehl - Einlegen = nur der Ausgleich der Störung, nicht die Schräglage
    Run upright;
    for (std::uint32_t k = 0; k < 2000; ++k) step(upright, k, true, 0, 2.0f, sign);
    std::printf("       Trimm aufrecht %.3f°, eingelegt %.3f°\n", upright.r.legs.roll_trim_deg, lean.r.legs.roll_trim_deg);
    check(std::fabs(lean.r.legs.roll_trim_deg - upright.r.legs.roll_trim_deg) < 0.2f, "Trimm enthält die Schräglage nicht");
    // Roll halten endet mitten in der Kurve: Beine laufen auf Rollbefehl + Trimm, also aufrecht, nicht schräg
    for (std::uint32_t k = 2000; k < 2600; ++k) step(lean, k, false, -4.5f, 2.0f, sign);
    check(std::fabs(lean.roll) < 0.3f, "Roll halten aus mitten in der Kurve: Rider richtet sich auf");

    // Ziel 0: bitgleich zum Stand ohne Einlegen (Ziel 0, Trimm - 0)
    Run a, b;
    bool same = true;
    for (std::uint32_t k = 0; k < 2000; ++k) {
        step(a, k, true, 0.0f, 2.0f, sign);
        step(b, k, true, -0.0f, 2.0f, sign);
        same = same && a.r.legs.roll_target_deg == b.r.legs.roll_target_deg && a.r.legs.servo_a == b.r.legs.servo_a &&
               a.r.legs.servo_b == b.r.legs.servo_b && a.r.legs.roll_trim_deg == b.r.legs.roll_trim_deg;
    }
    check(same, "Ziel 0 und -0: Beine bitgleich");
    // Ziel ohne Roll halten wirkt nicht
    Run c, d;
    for (std::uint32_t k = 0; k < 500; ++k) { step(c, k, false, -4.5f, 2.0f, sign); step(d, k, false, 0, 2.0f, sign); }
    check(c.r.legs.roll_target_deg == d.r.legs.roll_target_deg, "ohne Roll halten: Ziel wirkungslos");
}

// Schlangenlinie (Fahrt 20261126-201922: ±3°, Periode 1,7 s): mit Vorsteuerung folgt der Körper-Roll dem Ziel ohne
// den Verzug des Roll-Integrators; mit dem Tiefpass des Ziels zusammen bleibt der Verzug klein.
static float lagSeconds(const float* ref, const float* out, int n, float* gain) {
    int best = 0; float best_e = 1e30f;
    for (int sh = 0; sh < 200; ++sh) {
        float e = 0;
        for (int k = 200; k < n; ++k) { const float d = out[k] - ref[k - sh]; e += d * d; }
        if (e < best_e) { best_e = e; best = sh; }
    }
    float ro = 0, rr = 0;
    for (int k = 200; k < n; ++k) { ro += out[k] * ref[k - best]; rr += ref[k - best] * ref[k - best]; }
    *gain = ro / rr;
    return best * kDt;
}

static void testWeave(float sign) {
    constexpr int n = 2000;   // 6 s
    static float target[n], roll[n], want[n];
    Run r;
    for (std::uint32_t k = 0; k < 600; ++k) step(r, k, true, 0, 2.0f, sign);   // Störung ausgleichen
    CurveLean cl;
    for (int k = 0; k < n; ++k) {
        const float t = k * kDt;
        // v 0,9 m/s, Gierrate so, dass -atan(v·w/g) = 3°·sin(ωt)
        want[k] = 3.0f * std::sin(2.0f * 3.14159265f * t / 1.7f);
        const float w = -std::tan(want[k] / 57.29578f) * 9.81f / 0.9f * 57.29578f;
        target[k] = updateCurveLean(cl, true, 0.9f, w, kHeight, kDt);
        step(r, 600 + k, true, target[k], 2.0f, sign);
        roll[k] = r.roll;
    }
    float g1, g2;
    const float lag_pid = lagSeconds(target, roll, n, &g1);
    const float lag_all = lagSeconds(want, roll, n, &g2);
    std::printf("       Schlangenlinie: Roll hinter Ziel %.0f ms (Verstärkung %.2f), hinter -atan(v·w/g) %.0f ms (%.2f)\n",
                lag_pid * 1000, g1, lag_all * 1000, g2);
    check(lag_pid <= 0.015f && std::fabs(g1 - 1) < 0.1f, "Vorsteuerung: Körper-Roll folgt dem Ziel ohne Integrator-Verzug");
    check(lag_all <= 0.08f && std::fabs(g2 - 1) < 0.15f, "Schlangenlinie 1,7 s: gesamter Verzug höchstens 80 ms");
}

int main() {
    testTarget();
    testDynamics();
    testClosedLoop();
    testWeave(1.0f);   // Vorzeichen +1: nur damit ist Roll halten stabil (testClosedLoop)
    std::printf(fail ? "FEHLER\n" : "alles ok\n");
    return fail;
}
