// Host-Test für Patch 0015: Integral des Geschwindigkeitsfehlers beim Fahren.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_drive_integral.cpp $S/control/rider_motion.cpp $S/control/pid.cpp -o t && ./t
#include "control/rider_motion.h"
#include <cmath>
#include <cstdio>

using namespace rider::control;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

static float step(BalanceState& s, float target, float speed, bool holding, bool on) {
    Wheels w;
    updateBalance(s, target, speed, speed, 2.0f, 0.0f, 0.0f, 36.0f, holding, w, on);
    return s.pitch.target;
}

// Grobes Fahrmodell (RiderPi 2026-11-10, Register 0x0e): bei ~2,5 deg Vorlage über das Gleichgewicht
// steigt die Radgeschwindigkeit in ~1,5 s auf ~50 -> ~0,04 je Takt (3 ms) und Grad. "last" ist die
// Vorlage, die Steigung/Rollwiderstand fressen. Rückgabe: Geschwindigkeit nach n Takten, größte danach.
static void fahre(bool on, float last, float target, int n, float& end, float& peak, float& min_late) {
    BalanceState s;
    float v = 0;
    peak = 0; min_late = 1e9f;
    for (int k = 0; k < n; ++k) {
        const float p = step(s, target, v, false, on);
        v += 0.04f * (p - last);
        if (v > peak) peak = v;
        if (k > n / 2 && v < min_late) min_late = v;
    }
    end = v;
}

int main() {
    // Aus: bitgleich zum Original (Pitch-Soll = Ausgang des Geschwindigkeitsreglers)
    {
        BalanceState a, b;
        bool same = true;
        for (int k = 0; k < 2000; ++k) {
            const float t = (k / 500) % 2 ? 32.0f : 0.0f, v = 10.0f * std::sin(k * 0.01f);
            Wheels wa, wb;
            updateBalance(a, t, v, v, 2.0f, 0.1f, 0.01f, 36.0f, false, wa);
            updateBalance(b, t, v, v, 2.0f, 0.1f, 0.01f, 36.0f, false, wb, false);
            same = same && a.pitch.target == a.speed.output && a.pitch.target == b.pitch.target &&
                   wa.id11 == wb.id11 && wa.id21 == wb.id21 && b.drive_i == 0;
        }
        check(same, "aus: bitgleich zum Original");
    }
    // Stillstand bei vollem Befehl: Original bleibt bei ~3,5 deg, Patch legt bis 3 deg dazu
    {
        BalanceState a, b;
        float pa = 0, pb = 0, pb1 = 0;
        for (int k = 0; k < 1000; ++k) {
            pa = step(a, 32.0f, 0.0f, false, false);
            pb = step(b, 32.0f, 0.0f, false, true);
            if (k == 332) pb1 = pb;
        }
        std::printf("       Stillstand: Original %.2f deg, Patch %.2f deg (nach 1 s %.2f)\n", pa, pb, pb1);
        check(pa < 3.7f, "Original: höchstens ~3,6 deg Vorlage (I-Anteil wirkungslos)");
        check(std::fabs(pb - (pa + kDriveIMax)) < 0.01f, "Patch: genau kDriveIMax mehr, begrenzt");
        check(pb1 > pa + 2.5f, "Patch: nach ~1 s (333 Takte) fast ausgeschöpft");
    }
    // Ohne Fahrbefehl oder beim Halten: weich abklingen, nicht springen, kein Aufsummieren
    {
        BalanceState s;
        for (int k = 0; k < 1000; ++k) step(s, 32.0f, 0.0f, false, true);
        const float i0 = s.drive_i;
        step(s, 0.0f, 0.0f, false, true);
        check(s.drive_i > 0.98f * i0 && s.drive_i < i0, "Befehl weg: ein Takt nur x0,99");
        for (int k = 0; k < 333; ++k) step(s, 0.0f, 0.0f, false, true);
        check(s.drive_i < 0.05f * i0, "Befehl weg: nach ~1 s praktisch null");
        for (int k = 0; k < 2000; ++k) step(s, 0.0f, 0.0f, false, true);
        check(s.drive_i == 0, "Befehl weg: klingt auf exakt 0 ab (dann wieder Originalpfad)");
        for (int k = 0; k < 1000; ++k) step(s, 20.0f, 0.0f, true, true);
        check(s.drive_i == 0, "Positionshalten (Modus 2): kein Aufsummieren");
    }
    // Rückwärts symmetrisch
    {
        BalanceState s;
        float p = 0;
        for (int k = 0; k < 1000; ++k) p = step(s, -32.0f, 0.0f, false, true);
        check(std::fabs(s.drive_i + kDriveIMax) < 1e-4f && p < -6.0f, "rückwärts: -kDriveIMax");
    }
    // Fahrmodell: ebene Strecke, Last 3,6 deg (wie 2026-11-10 vor dem Start), zu große Last
    {
        float e, pk, ml;
        fahre(false, 0.0f, 32.0f, 3000, e, pk, ml);
        std::printf("       eben    Original: Ende %5.1f\n", e);
        fahre(true, 0.0f, 32.0f, 3000, e, pk, ml);
        std::printf("       eben    Patch:    Ende %5.1f, Spitze %5.1f\n", e, pk);
        check(std::fabs(e - 32.0f) < 1.0f && pk < 45.0f, "eben: Patch erreicht 32, Überschwingen < 40 %");
        fahre(false, 3.6f, 32.0f, 3000, e, pk, ml);
        std::printf("       Last    Original: Ende %5.1f\n", e);
        check(e < 2.0f, "Last 3,6 deg: Original bleibt stehen (wie am Rider)");
        fahre(true, 3.6f, 32.0f, 3000, e, pk, ml);
        std::printf("       Last    Patch:    Ende %5.1f, Spitze %5.1f, zweite Hälfte min %5.1f\n", e, pk, ml);
        // I am Anschlag (3 deg), Rest trägt P: 0,11 x (32 - v) = 0,6 -> v ~ 26,5
        check(e > 25.0f && ml > 25.0f && pk < 32.0f, "Last 3,6 deg: Patch fährt weiter (~27 statt 0), ohne Pumpen");
        fahre(true, 2.5f, 32.0f, 3000, e, pk, ml);
        std::printf("       Last 2,5 Patch:    Ende %5.1f, Spitze %5.1f, zweite Hälfte min %5.1f\n", e, pk, ml);
        check(std::fabs(e - 32.0f) < 1.0f && ml > 30.0f && pk < 45.0f, "Last 2,5 deg: Patch erreicht 32, ohne Pumpen");
        fahre(true, 7.5f, 32.0f, 3000, e, pk, ml);
        check(e < 1.0f, "Last 7,5 deg (> 3,5 + 3): Patch hilft nicht unbegrenzt");
    }
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
