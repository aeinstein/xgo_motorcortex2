// Host-Test 08-velocity: Drehratenregler des Fahrrahmens (control/rider_motion.h TurnControl).
// Prüft: aus = Original bitgleich, Kursziel läuft genau mit w·dt (Original 0,005 je Takt = 1,25 × w bei 4 ms),
// kp/kd/ff wirken. Dazu ein Modell (NICHT der Rider): Gierrate folgt −c·Raddifferenz mit 0,1 s Verzögerung, optional
// Haftreibung (unter 10 Radeinheiten keine Drehung). Damit: mittlere Drehrate = w_soll (Original 1,25 ×), nach w = 0
// steht der Kurs, langsames Drehen (0,5 °/s) kommt trotz Haftreibung voran. Kursziel (hold): ohne !heading hält der
// Kurs-PID nach der Drehung das Ziel und holt den Nachlauf auf; ohne hold gibt die Original-Logik den Kurs im Stand auf.
// Aufruf: firmware/tests/run.sh test_turn_ctrl
#include "control/rider_motion.h"
#include <cmath>
#include <cstdio>

using namespace rider::control;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

constexpr float kDt = 0.004f;

struct Sim {
    HeadingState h;
    float yaw = 0, rate = 0;
    std::uint32_t ms = 0;
    // ein Takt mit Soll-Drehrate w; tc == nullptr = Original; Rückgabe = Raddifferenz
    bool heading_hold = true;   // wie !heading; am Rider im Rahmenweg aus
    float step(float w, const TurnControl* tc, float stiction = 0, float c = 1.0f) {
        Wheels wh;
        addHeadingCorrection(h, wh, ms, 1, heading_hold, 0, w, w, yaw, false, 0, tc);
        const float d = wh.id11;
        const float drive = std::fabs(d) < stiction ? 0.0f : -c * d;
        rate += (drive - rate) * kDt / 0.1f;
        yaw += rate * kDt;
        ms += 4;
        return d;
    }
};

int main() {
    // --- aus = Original bitgleich
    {
        Sim a, b; TurnControl off; off.on = false; off.kp = 9; off.ff_gain = 2;   // Werte dürfen aus nicht wirken
        bool same = true;
        for (int i = 0; i < 3000; ++i) {
            const float w = i < 1000 ? 40.0f : (i < 2000 ? -15.0f : 0.0f);
            const float da = a.step(w, nullptr), db = b.step(w, &off);
            if (da != db || a.h.target != b.h.target || a.yaw != b.yaw) same = false;
        }
        check(same, "Rahmenweg aus: Raddifferenz, Kursziel und Kurs bitgleich zum Original");
    }
    // --- Kursziel läuft genau mit w·dt
    {
        Sim s; TurnControl tc; tc.on = true;
        for (int i = 0; i < 200; ++i) s.step(0, &tc);   // Kurs fängt sich
        const float t0 = s.h.target;
        for (int i = 0; i < 250; ++i) s.step(30, &tc);
        check(std::fabs((s.h.target - t0) - 30.0f) < 0.05f, "Kursziel: 30 °/s über 1 s -> +30° (genau w·dt)");
        Sim o; for (int i = 0; i < 200; ++i) o.step(0, nullptr);
        const float o0 = o.h.target;
        for (int i = 0; i < 250; ++i) o.step(30, nullptr);
        std::printf("       Original: Kursziel +%.1f° in 1 s bei 30 °/s\n", double(o.h.target - o0));
        check(std::fabs((o.h.target - o0) - 37.5f) < 0.1f, "Original: 0,005 je Takt = 37,5° (1,25 × w)");
    }
    // --- Modell: mittlere Drehrate
    {
        Sim s, o; TurnControl tc; tc.on = true;
        for (int i = 0; i < 1000; ++i) { s.step(30, &tc); o.step(30, nullptr); }   // 4 s
        const float ys = s.yaw, yo = o.yaw;
        for (int i = 0; i < 500; ++i) { s.step(30, &tc); o.step(30, nullptr); }    // 2 s messen
        const float rs = (s.yaw - ys) / 2.0f, ro = (o.yaw - yo) / 2.0f;
        std::printf("       Modell 30 °/s: neu %.2f °/s, Original %.2f °/s\n", double(rs), double(ro));
        check(std::fabs(rs - 30.0f) < 0.3f, "Modell: mittlere Drehrate = w_soll (±1 %)");
        check(ro > 36.0f, "Modell: Original dreht ~1,25 × zu schnell");
        // Stopp: Kurs steht
        for (int i = 0; i < 500; ++i) s.step(0, &tc);
        const float y_stop = s.yaw;
        for (int i = 0; i < 500; ++i) s.step(0, &tc);
        check(std::fabs(s.yaw - y_stop) < 0.2f, "Modell: nach w = 0 hält er den Kurs");
    }
    // --- Modell: langsam mit Haftreibung
    {
        Sim s; TurnControl tc; tc.on = true;
        for (int i = 0; i < 250; ++i) s.step(0, &tc, 10);
        const float y0 = s.yaw;
        for (int i = 0; i < 5000; ++i) s.step(0.5f, &tc, 10);   // 20 s
        const float moved = s.yaw - y0;
        std::printf("       Modell 0,5 °/s mit Haftreibung: %.2f° in 20 s (Soll 10°), Nachlauf %.2f°\n",
                    double(moved), double(s.h.target - s.yaw));
        check(moved > 6.0f && moved < 10.5f, "Modell: 0,5 °/s kommt trotz Haftreibung voran (Nachlauf ≤ ~4°)");
    }
    // --- kp/kd/ff wirken
    {
        Sim a, b; TurnControl t1; t1.on = true; TurnControl t2 = t1; t2.ff_gain = 1.0f;
        const float d1 = a.step(20, &t1), d2 = b.step(20, &t2);
        check(std::fabs((d2 - d1) - (-10.0f)) <= 1.0f, "ff: 0,5 mehr Vorsteuerung bei 20 °/s = 10 Radeinheiten mehr");
    }
    // --- Kursziel: nach der Drehung den Kurs halten (am Rider ohne !heading)
    {
        auto turn90 = [](bool hold, float stiction, float& lag_end, float& err_late, float& target_late) {
            Sim s; s.heading_hold = false; TurnControl tc; tc.on = true; tc.hold = hold;
            for (int i = 0; i < 100; ++i) s.step(0, &tc, stiction);
            const float t0 = s.h.target;
            for (int i = 0; i < 375; ++i) s.step(60, &tc, stiction);   // 1,5 s mit 60 °/s = 90°
            lag_end = (t0 + 90.0f) - s.yaw;
            for (int i = 0; i < 500; ++i) s.step(0, &tc, stiction);     // 2 s danach
            err_late = (t0 + 90.0f) - s.yaw; target_late = s.h.target - (t0 + 90.0f);
        };
        float l0, e0, g0, l1, e1, g1, l2, e2, g2;
        turn90(false, 0, l0, e0, g0); turn90(true, 0, l1, e1, g1); turn90(true, 10, l2, e2, g2);
        std::printf("       90°-Drehung: Nachlauf am Ende %.2f°; 2 s später ohne hold %.2f° (Ziel versetzt um %.2f°), mit hold %.2f°, mit hold + Haftreibung 10 %.2f°\n",
                    double(l1), double(e0), double(g0), double(e1), double(e2));
        check(std::fabs(g0) > 0.01f || std::fabs(e0) > 0.01f || l0 < 0.01f, "ohne hold: Original gibt das Kursziel im Stand auf (Ziel = Ist-Kurs)");
        check(std::fabs(g1) < 1e-3f && std::fabs(e1) < 0.3f, "mit hold: Kursziel bleibt, Kurs 2 s nach der Drehung auf < 0,3° am Ziel");
        check(std::fabs(e2) < 1.5f, "mit hold und Haftreibung 10: < 1,5° am Ziel (Rest, wo kp·Fehler unter der Haftung liegt)");
    }
    std::printf(fail ? "\nFEHLER\n" : "\nalle ok\n");
    return fail;
}
