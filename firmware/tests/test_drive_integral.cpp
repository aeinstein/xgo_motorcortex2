// Host-Test für Patch 0015: Integral des Geschwindigkeitsfehlers beim Fahren.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_drive_integral.cpp $S/control/rider_motion.cpp $S/control/pid.cpp -o t && ./t
#include "control/rider_motion.h"
#include <cmath>
#include <cstdio>
#include <initializer_list>

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
// Vorlage, die Steigung/Rollwiderstand fressen. traege > 0: die Neigung folgt dem Soll erst mit dieser
// Zeitkonstante (Takte), wie am echten Rider. Die Schwingung der ersten Fassung (volles Integral) am
// Rider bildet das Modell nur grob nach; es prüft die Regel, nicht die Abstimmung.
struct Fahrt { float end, peak, min_late, i_end, i_peak; };
static Fahrt fahre(bool on, float last, float target, int n, float traege = 0.0f) {
    BalanceState s;
    Fahrt f{0, 0, 1e9f, 0, 0};
    float v = 0, lage = 0;
    for (int k = 0; k < n; ++k) {
        const float p = step(s, target, v, false, on);
        lage = traege > 0 ? lage + (p - lage) / traege : p;
        v += 0.04f * (lage - last);
        if (v > f.peak) f.peak = v;
        if (std::fabs(s.drive_i) > f.i_peak) f.i_peak = std::fabs(s.drive_i);
        if (k > n / 2 && v < f.min_late) f.min_late = v;
    }
    f.end = v; f.i_end = s.drive_i;
    return f;
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
    // (0,3 x 32 x kDriveKi je Takt -> 3 deg nach ~1040 Takten = 3,1 s)
    {
        BalanceState a, b;
        float pa = 0, pb = 0, pb1 = 0;
        for (int k = 0; k < 1500; ++k) {
            pa = step(a, 32.0f, 0.0f, false, false);
            pb = step(b, 32.0f, 0.0f, false, true);
            if (k == 332) pb1 = pb;
        }
        std::printf("       Stillstand: Original %.2f deg, Patch %.2f deg (nach 1 s %.2f)\n", pa, pb, pb1);
        check(pa < 3.7f, "Original: höchstens ~3,6 deg Vorlage (I-Anteil wirkungslos)");
        check(std::fabs(pb - (pa + kDriveIMax)) < 0.01f, "Patch: genau kDriveIMax mehr, begrenzt");
        check(pb1 > pa + 0.8f && pb1 < pa + 1.1f, "Patch: nach ~1 s (333 Takte) rund 1 deg mehr");
    }
    // Rollt er schneller als kDriveMinFrac des Befehls, baut es ab und bleibt bei null (kein Bremsen)
    {
        BalanceState s;
        for (int k = 0; k < 1500; ++k) step(s, 32.0f, 0.0f, false, true);
        for (int k = 0; k < 300; ++k) step(s, 32.0f, 20.0f, false, true);
        check(s.drive_i > 1.9f && s.drive_i < 2.2f, "rollt mit 20 bei Befehl 32: baut ab (~1 deg in 0,9 s)");
        for (int k = 0; k < 1500; ++k) step(s, 32.0f, 20.0f, false, true);
        check(s.drive_i == 0 && s.pitch.target == s.speed.output, "rollt weiter: null, Originalpfad");
        for (int k = 0; k < 1500; ++k) step(s, 32.0f, 60.0f, false, true);
        check(s.drive_i == 0, "schneller als der Befehl: wird nicht negativ");
    }
    // Befehl umgekehrt: der Rest gegen die neue Richtung klingt ab, erst dann baut es neu auf
    {
        BalanceState s;
        for (int k = 0; k < 1500; ++k) step(s, 32.0f, 0.0f, false, true);
        const float i0 = s.drive_i;
        step(s, -32.0f, 0.0f, false, true);
        check(s.drive_i > 0.98f * i0 && s.drive_i < i0, "Befehl umgekehrt: ein Takt nur x0,99, kein Sprung");
        for (int k = 0; k < 3000; ++k) step(s, -32.0f, 0.0f, false, true);
        check(std::fabs(s.drive_i + kDriveIMax) < 1e-4f, "Befehl umgekehrt: danach -kDriveIMax");
    }
    // Ohne Fahrbefehl oder beim Halten: weich abklingen, nicht springen, kein Aufsummieren
    {
        BalanceState s;
        for (int k = 0; k < 1500; ++k) step(s, 32.0f, 0.0f, false, true);
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
        for (int k = 0; k < 1500; ++k) p = step(s, -32.0f, 0.0f, false, true);
        check(std::fabs(s.drive_i + kDriveIMax) < 1e-4f && p < -6.0f, "rückwärts: -kDriveIMax");
    }
    // Fahrmodell: ebene Strecke (auch mit träger Neigung), Last 3,6 deg (wie 2026-11-10 vor dem Start),
    // kleine Last, zu große Last
    {
        for (float traege : {0.0f, 100.0f}) {
            const Fahrt o = fahre(false, 0.0f, 32.0f, 3000, traege), p = fahre(true, 0.0f, 32.0f, 3000, traege);
            std::printf("       eben (Trägheit %3.0f) Original: Ende %5.1f, Spitze %5.1f | Patch: Ende %5.1f, Spitze %5.1f, I höchstens %.2f\n",
                        traege, o.end, o.peak, p.end, p.peak, p.i_peak);
            check(std::fabs(p.end - o.end) < 0.1f && p.i_end == 0, "eben: Patch endet wie das Original, Integral null");
            check(p.peak < o.peak + 2.0f && p.i_peak < 1.0f, "eben: Anfahren kaum anders (Spitze < +2, Integral < 1 deg)");
        }
        Fahrt f = fahre(false, 3.6f, 32.0f, 3000);
        std::printf("       Last 3,6 Original: Ende %5.1f\n", f.end);
        check(f.end < 2.0f, "Last 3,6 deg: Original bleibt stehen (wie am Rider)");
        f = fahre(true, 3.6f, 32.0f, 6000);
        std::printf("       Last 3,6 Patch:    Ende %5.1f, Spitze %5.1f, zweite Hälfte min %5.1f, I %.2f\n", f.end, f.peak, f.min_late, f.i_end);
        // Untergrenze 0,3 x 32 = 9,6: P trägt 0,11 x 22,4 = 2,5 deg, das Integral den Rest (1,1 deg)
        check(std::fabs(f.end - 9.6f) < 0.5f && f.min_late > 9.0f && f.peak < 12.0f,
              "Last 3,6 deg: Patch rollt mit der Untergrenze (~9,6 statt 0), ohne Pumpen");
        f = fahre(true, 3.6f, 32.0f, 6000, 100.0f);
        std::printf("       Last 3,6 Patch, träge: Ende %5.1f, Spitze %5.1f, zweite Hälfte min %5.1f\n", f.end, f.peak, f.min_late);
        check(std::fabs(f.end - 9.6f) < 0.5f && f.min_late > 8.5f && f.peak < 18.0f, "Last 3,6 deg, träge Neigung: schwingt sich ein");
        const Fahrt o = fahre(false, 1.0f, 32.0f, 3000);
        f = fahre(true, 1.0f, 32.0f, 3000);
        std::printf("       Last 1,0 Original: Ende %5.1f | Patch: Ende %5.1f, I %.2f\n", o.end, f.end, f.i_end);
        check(std::fabs(f.end - o.end) < 0.1f && f.i_end == 0, "Last 1 deg: über der Untergrenze, Patch wie Original");
        f = fahre(true, 7.5f, 32.0f, 3000);
        check(f.end < 1.0f, "Last 7,5 deg (> 3,5 + 3): Patch hilft nicht unbegrenzt");
    }
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
