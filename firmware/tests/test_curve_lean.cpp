// Host-Test 07-balance: In schnellen Kurven einlegen (control::updateCurveLean, MotionRequest::roll_target_deg).
// Ziel = -atan(v * Gierrate / g): Vorzeichen, Betrag, Grenze nur die Mechanik (24,74°), Höhe weicht aus, nur Tiefpass, aus = zurück auf 0.
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

static float settle(float v, float w_deg_s, float seconds = 3.0f) {
    CurveLean s;
    for (int k = 0; k < int(seconds / kDt); ++k) updateCurveLean(s, true, v, w_deg_s, kDt);
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
    // Fahrt 20261127-084257: 0,87 m/s, 180 °/s -> -atan(v·w/g) = -15,5°, ganz (Schräglage vor Höhe)
    const float want15 = -std::atan(0.87f * 180 * 0.017453292f / 9.81f) * 57.29578f;
    const float sharp = settle(0.87f, 180);
    std::printf("       0,87 m/s, 180 °/s: nötig %.2f°, Ziel %.2f°\n", want15, sharp);
    check(std::fabs(sharp - want15) < 0.01f, "scharfe Kurve: volles -atan(v·w/g) = -15,5° (keine Winkel- oder Höhengrenze)");
    check(std::fabs(settle(3.0f, 150) + rider::control::kLeanMaxDeg) < 1e-4f, "Betrag nur durch die Mechanik begrenzt (asin(18/43) = 24,74°)");
    check(settle(NAN, 90) == 0.0f && settle(0.5f, NAN) == 0.0f, "ungültige Eingänge: Ziel 0");
}

static void testDynamics() {
    CurveLean s;
    // Sprung auf -15,57° (Fahrt 084257): nur Tiefpass kLeanTau, keine Rate -> nach 3 kLeanTau (0,24 s) bei 95 %
    const float want = -std::atan(0.87f * 180 * 0.017453292f / 9.81f) * 57.29578f;
    updateCurveLean(s, true, 0.87f, 180, kDt);
    const float first = s.target_deg;
    for (int k = 1; k < 80; ++k) updateCurveLean(s, true, 0.87f, 180, kDt);
    std::printf("       Sprung auf %.2f°: erster Takt %.3f°, nach 0,24 s %.2f°\n", want, first, s.target_deg);
    check(std::fabs(first - want * kDt / rider::control::kLeanTau) < 1e-4f, "Sprung: erster Takt = Tiefpass (kein Sprung im Ziel)");
    check(s.target_deg / want > 0.94f, "keine Ratenbegrenzung: nach 3 kLeanTau über 94 % (mit 20 °/s waren es 0,24 s × 20 = 4,8°)");
    for (int k = 0; k < 400; ++k) updateCurveLean(s, true, 1.5f, 150, kDt);
    const float before = s.target_deg;
    updateCurveLean(s, true, 1.5f, 150, 0.0f);
    updateCurveLean(s, true, 1.5f, 150, 0.5f);
    check(s.target_deg == before, "dt 0 oder > 0,1 s: keine Änderung");
    for (int k = 0; k < 1000; ++k) updateCurveLean(s, false, 1.5f, 150, kDt);
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
        target[k] = updateCurveLean(cl, true, 0.9f, w, kDt);
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

// Schräglage vor Höhe (updateLegTargets mit lean_deg): passt die Beindifferenz nicht um die Höhe, rückt die Höhe zur
// Mitte und beide Beine führen die Schräglage ganz aus; ohne Einlegen bitgleich (Beine klemmen wie im Original).
static void testLegPriority() {
    using rider::control::LegState;
    using rider::control::updateLegTargets;
    constexpr std::uint16_t ZA = 2048, ZB = 2048;
    bool ok_full = true, ok_range = true, ok_shift = true;
    for (float h : {2.0f, 9.0f, 18.0f, 27.0f, 34.0f}) for (float lean : {-15.5f, 15.5f, -24.0f}) {
        LegState s;
        updateLegTargets(s, h, 0, true, 0, ZA, ZB, lean);
        const float a = s.extension_a - 45, b = s.extension_b - 45;
        const float want = float(std::sin(double(lean) / 57.3) * 43.0);
        ok_full = ok_full && std::fabs((a - b) * 0.5f - want) < 1e-3f;
        ok_range = ok_range && a >= 0 && a <= 36 && b >= 0 && b <= 36;
        ok_shift = ok_shift && std::fabs((a + b) * 0.5f - (h + s.height_shift)) < 1e-3f;   // gemeldete Höhe = (A + B) / 2
        if (std::fabs(want) <= std::fmin(h, 36 - h)) ok_full = ok_full && std::fabs((a + b) * 0.5f - h) < 1e-3f;   // passt: Höhe bleibt
    }
    check(ok_full && ok_range, "Schräglage vor Höhe: Beine führen die volle Differenz aus, Höhe rückt nur so weit wie nötig");
    check(ok_shift, "Höhe für Balance und Nick-Ausgleich: Sollhöhe + height_shift = (Bein A + Bein B) / 2");
    LegState o; o.roll_target_deg = 15;   // Roll halten ohne Einlegen bei Höhe 2: wie im Original klemmt ein Bein
    updateLegTargets(o, 2.0f, 0, true, 0, ZA, ZB, 0.0f);
    const float d15 = float(std::sin(15.0 / 57.3) * 43.0);
    check(o.height_shift == 0.0f && o.extension_b == 45.0f && std::fabs(o.extension_a - (45.0f + 2.0f + d15)) < 1e-3f, "ohne Einlegen: Höhe bleibt, Bein klemmt (Original)");
    LegState m;
    updateLegTargets(m, 18.0f, 0, true, 0, ZA, ZB, -24.0f);
    check(std::fabs(m.roll_target_deg + 24.0f) < 1e-4f, "mit Einlegen: Rollziel bis kLeanMaxDeg statt ±20°");
}

int main() {
    testTarget();
    testDynamics();
    testClosedLoop();
    testWeave(1.0f);   // Vorzeichen +1: nur damit ist Roll halten stabil (testClosedLoop)
    testLegPriority();
    std::printf(fail ? "FEHLER\n" : "alles ok\n");
    return fail;
}
