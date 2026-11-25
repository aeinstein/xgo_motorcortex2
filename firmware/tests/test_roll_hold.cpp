// Host-Test für Patch 0028: Roll laufend auf 0 halten (MotionRequest::roll_hold) und blaue LEDs beim Herunterfahren.
// Roll halten: dieselbe Beinrechnung wie die Nivellierung 0x61 (bitgleich), aber die Räder wie ohne Nivellierung
// (kein rollabhängiger Anteil der Kurskorrektur), und das auch bei vollem Drehbefehl. Dazu ein kleiner Regelkreis:
// Körper-Roll = Störung + Roll-Sollwert der Beine (Vorzeichen aus der Nivellierung selbst bestimmt) -> geht auf 0.
// Shutdown-LEDs: blau vor allem anderen (Fehlerfarbe, Akku, Fahren), aus = wie vorher.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_roll_hold.cpp $S/motion_controller.cpp $S/control/rider_motion.cpp $S/control/pid.cpp $S/control/rider_leg_imu.cpp $S/board_support.cpp -o t && ./t
#include "motion_controller.h"
#include "board_support.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

using rider::firmware::MotionController;
using rider::firmware::MotionInput;
using rider::firmware::MotionRequest;
using rider::firmware::MotionResult;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

struct Run {
    MotionController c;
    MotionResult r;
    float roll = 0;
};

// Ein Takt (3 ms) bei Fahrt mit Drehung; roll_deg = Störung + Vorzeichen * Roll-Sollwert der Beine (geschlossen)
static void step(Run& run, std::uint32_t k, bool leveling, bool roll_hold, float disturbance, float sign,
                 std::uint8_t turn) {
    MotionInput in;
    in.now_ms = 3000 + 3 * k; in.mode = 1; in.calibrated = true;
    in.roll_deg = run.roll; in.yaw_deg = 0.01f * float(k); in.speed11 = in.speed21 = 40;
    MotionRequest rq;
    rq.command.forward = 170; rq.command.turn = turn;
    rq.leveling = leveling; rq.roll_hold = roll_hold; rq.drive_integral = true; rq.turn_hold = true;
    run.r = run.c.step(in, rq);
    run.roll = disturbance + sign * run.r.legs.roll_target_deg;
}

static bool sameLegs(const MotionResult& a, const MotionResult& b) {
    return a.legs.roll_target_deg == b.legs.roll_target_deg && a.legs.servo_a == b.legs.servo_a &&
           a.legs.servo_b == b.legs.servo_b && a.legs.roll_trim_deg == b.legs.roll_trim_deg;
}
static bool sameWheels(const MotionResult& a, const MotionResult& b) {
    return a.wheels.id11 == b.wheels.id11 && a.wheels.id21 == b.wheels.id21;
}

static void testRollHold() {
    // Vorzeichen der Beine: mit welchem Zusammenhang Körper-Roll/Sollwert gleicht die Nivellierung aus?
    float sign = 0;
    for (float s : {-1.0f, 1.0f}) {
        Run lv;
        for (std::uint32_t k = 0; k < 2000; ++k) step(lv, k, true, false, 2.0f, s, 128);
        if (std::fabs(lv.roll) < 0.2f) sign = s;
    }
    check(sign != 0, "Nivellierung 0x61 gleicht 2° Roll im Stand aus (Vorzeichen bestimmt)");

    // Fahrt mit voller Drehung: Roll halten = Beine wie Nivellierung, Räder wie ohne
    Run off, lv, hold;
    bool legs_same = true;
    for (std::uint32_t k = 0; k < 2000; ++k) {
        step(off, k, false, false, 2.0f, sign, 220);
        step(lv, k, true, false, 2.0f, sign, 220);
        step(hold, k, false, true, 2.0f, sign, 220);
        legs_same = legs_same && sameLegs(hold.r, lv.r);
    }
    check(legs_same, "Roll halten: Beinrechnung bitgleich zur Nivellierung (2000 Takte, Drehbefehl 220)");
    check(std::fabs(hold.roll) < 0.2f, "Roll halten: 2° Störung beim Fahren mit Drehung auf < 0,2° ausgeglichen");
    check(std::fabs(off.roll - 2.0f - sign * off.r.command.roll) < 0.01f, "ohne: Roll bleibt bei 2° (+ Rollbyte 128 = 0,078°)");

    // Räder: Roll halten gegen dieselben Beine, aber ohne Nivellierung (gleicher Roll-Eingang, offen)
    Run a, b;
    bool same = true;
    for (std::uint32_t k = 0; k < 2000; ++k) {
        step(a, k, false, true, 2.0f, sign, 220);
        step(b, k, false, false, 2.0f, sign, 220);
        b.roll = a.roll;   // gleicher Eingang
        same = same && sameWheels(a.r, b.r);
    }
    check(same, "Roll halten: Räder bitgleich zu ohne Nivellierung (kein Anteil an der Kurskorrektur)");

    // Ausschalten: Sollwert läuft wie im Original auf Befehl + Trimm zurück, nichts springt
    Run c;
    for (std::uint32_t k = 0; k < 2000; ++k) step(c, k, false, true, 2.0f, sign, 128);
    const float trim = c.r.legs.roll_trim_deg;
    for (std::uint32_t k = 2000; k < 2300; ++k) step(c, k, false, false, 2.0f, sign, 128);
    check(std::fabs(c.r.legs.roll_target_deg - trim - c.r.command.roll) < 0.05f, "aus: Sollwert bleibt bei Rollbefehl + eingelerntem Trimm (0012)");
}

static void testShutdownLeds() {
    using namespace rider::board;
    LedControl c;
    LedPixels px{};
    check(!renderShutdownLeds(c, px), "Shutdown aus: nichts zu zeichnen");
    c.shutdown = true;
    c.robot_mode = 0; c.fault = LedFault::Tilt;
    bool blue = renderShutdownLeds(c, px);
    for (const auto& p : px) blue = blue && p.r == 0 && p.g == 0 && p.b == kShutdownColor.b;
    check(blue, "Shutdown an: alle LEDs blau, auch über der Kippfarbe");
    c.shutdown = false;
    LedPixels fp{};
    check(renderFaultLeds(c, 0, fp) && fp[0].r == 200, "Shutdown aus: Kippfarbe wieder rot");
}

int main() {
    testRollHold();
    testShutdownLeds();
    std::printf(fail ? "FEHLER\n" : "alles ok\n");
    return fail;
}
