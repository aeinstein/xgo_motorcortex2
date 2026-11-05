// Host-Test für Patch 0008: Positionshalten beim Drehen auf der Stelle.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_turn_hold.cpp $S/control/rider_motion.cpp $S/control/pid.cpp -o t && ./t
#include "control/rider_motion.h"
#include <cmath>
#include <cstdio>

using namespace rider::control;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

int main() {
    const std::uint8_t kTurn = 160;          // Drehbefehl außerhalb 113..143
    // Original (allow_turn=false): beim Drehen kein Halten
    {
        HoldState h;
        for (std::uint32_t t = 3000; t < 4000; t += 4) updatePositionHold(h, t, 1, 0.0f, kTurn, 0.0f, 0.0f);
        check(h.mode == 0 && h.correction == 0, "ohne Schalter: Drehen hält nicht (wie R-1.1.6)");
    }
    // Patch 0008: beim Drehen halten, Abweichung zurückregeln
    {
        HoldState h;
        for (std::uint32_t t = 3000; t < 3600; t += 4) updatePositionHold(h, t, 1, 0.0f, kTurn, 0.0f, 0.0f, true, true, 675.0f);
        check(h.mode == 2, "mit Schalter: nach 500 ms Halten auch beim Drehen");
        updatePositionHold(h, 3604, 1, 0.0f, kTurn, 0.05f, 0.0f, true, true, 675.0f);
        check(std::fabs(h.correction - (-0.05f * 675.0f)) < 1e-3f, "5 cm vorgerollt -> Korrektur -33,75 (zurück)");
        updatePositionHold(h, 3608, 1, 0.0f, kTurn, -0.05f, 0.0f, true, true, 675.0f);
        check(h.correction > 33.0f, "5 cm zurückgerollt -> Korrektur vorwärts");
        updatePositionHold(h, 3612, 1, 0.0f, kTurn, 0.25f, 0.0f, true, true, 675.0f);
        check(h.mode == 0, "mehr als 20 cm weggeschoben -> Halten aus (Schutz wie Original)");
        updatePositionHold(h, 3616, 1, 20.0f, kTurn, 0.25f, 0.0f, true, true, 675.0f);
        check(h.mode == 0 && h.correction == 0, "Fahrbefehl -> kein Halten");
    }
    // Stehen ohne Drehen: mit und ohne Schalter gleiches Verhalten (nur Rückführung/Verstärkung anders)
    {
        HoldState a, b;
        for (std::uint32_t t = 3000; t < 3600; t += 4) {
            updatePositionHold(a, t, 1, 0.0f, 128, 0.0f, 0.0f);
            updatePositionHold(b, t, 1, 0.0f, 128, 0.0f, 0.0f, true, true, 675.0f);
        }
        check(a.mode == 2 && b.mode == 2, "Stehen: beide halten");
    }
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
