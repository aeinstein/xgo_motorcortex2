// Host-Test für Patch 0031: Geschwindigkeitsregler des Fahrrahmens (control/rider_motion.h VelocityControl).
// Prüft am Regler: aus = Originalweg bitgleich (auch mit 0015), Vorsteuerung atan(a/g), Integral (Vorzeichen, Grenze,
// Anti-Windup, freeze, Abklingen beim Halten und nach dem Ende ohne Sprung), Rest von 0015 klingt ab.
// Dazu ein geschlossener Kreis mit einem einfachen Fahrmodell (NICHT der echte Rider): Beschleunigung
// g·tan(Nick − Last), Nick folgt dem Soll-Nick mit 0,1 s Verzögerung. Damit: bleibende Abweichung unter Last mit dem
// Original-P gegen ~0 mit Integral, kein Aufschaukeln, Vorsteuerung verkleinert den Schleppfehler beim Anfahren.
// 0035: Vorsteuerung mit unit_mm (a in Firmware-Einheiten/s²), stock_scale: Original-P sieht Soll × c, Integral
// das volle Soll, aus/1 = bitgleich.
// 0036: Integral erst, wenn der Generator kVelSettleS (0,6 s) lang a = 0 hat (kein Windup beim Anfahren); die
// Integral-Tests setzen dafür vel_settled vorab (settle()).
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_velocity_ctrl.cpp $S/control/rider_motion.cpp $S/control/pid.cpp -o t && ./t
#include "control/rider_motion.h"
#include "velocity_cmd.h"
#include <cmath>
#include <cstdio>

using namespace rider::control;
namespace V = rider2::velocity;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}
static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

constexpr float kDt = 0.004f;   // Regeltakt kControlPeriodUs
static void settle(BalanceState& b) { b.vel_settled = kVelSettleS; }   // Generator ruht schon (0036)

// Ein Reglertakt; gibt den Soll-Nick zurück.
static float step(BalanceState& b, float target, float speed, float pitch, bool holding, bool drive_int,
                  const VelocityControl* vc) {
    Wheels w;
    updateBalance(b, target, speed, speed, pitch, 0, 0, 18, holding, w, drive_int, vc);
    return b.pitch.target;
}

// Fahrmodell: v' = g·tan(θ − Last), θ folgt dem Soll mit tau.
struct Plant { float v = 0, theta = 0; };
static void plantStep(Plant& p, float theta_target, float load_deg, float tau = 0.1f) {
    p.theta += (theta_target - p.theta) * kDt / tau;
    p.v += V::clampAbs(kGravityMmS2 * std::tan((p.theta - load_deg) * 0.0174533f), 20000.0f) * kDt;
}

struct LoopResult { float err_end = 0, err_peak = 0, osc_late = 0; };
// Fährt target(t) mit Last; vc == nullptr = Originalweg ohne 0015.
static LoopResult loop(float seconds, float load_deg, VelocityControl* vc, float v_target, const V::Frame* frame = nullptr) {
    BalanceState b; Plant p; LoopResult r;
    V::State gs; V::Limits gl; gl.v_max = 600; gl.w_max = 300; gl.set = true;
    if (frame) { V::receive(gs, gl, *frame, 0); V::tick(gs, gl, 0); }
    float vmin = 1e9f, vmax = -1e9f;
    const int n = int(seconds / kDt);
    for (int i = 0; i < n; ++i) {
        float tgt = v_target, acc = 0;
        if (frame) { V::tick(gs, gl, std::uint32_t((i + 1) * kDt * 1e6f)); tgt = gs.v; acc = gs.fwd.a; }
        if (vc) { vc->accel = acc; vc->dt = kDt; }
        const float th = step(b, tgt, p.v, p.theta, false, false, vc);
        plantStep(p, th, load_deg);
        const float e = tgt - p.v;
        r.err_peak = std::max(r.err_peak, std::fabs(e));
        if (i >= n - int(2.0f / kDt)) { vmin = std::min(vmin, p.v); vmax = std::max(vmax, p.v); }
        r.err_end = e;
    }
    r.osc_late = vmax - vmin;
    return r;
}

int main() {
    // --- aus = Originalweg bitgleich (mit 0015)
    {
        BalanceState a, b; VelocityControl off; off.on = false;
        bool same = true; float v = 0;
        for (int i = 0; i < 2000; ++i) {
            const float tgt = i < 1000 ? 150.0f : -80.0f;
            const float ta = step(a, tgt, v, 0.5f, false, true, nullptr);
            const float tb = step(b, tgt, v, 0.5f, false, true, &off);
            if (ta != tb || a.drive_i != b.drive_i) same = false;
            v += (tgt - v) * 0.002f;
        }
        check(same && b.vel_i == 0, "Rahmenweg aus: Soll-Nick und 0015 bitgleich zum Original");
    }
    // --- Vorsteuerung
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.accel = 500; vc.ki = 0; vc.dt = kDt;
        const float th = step(b, 100, 100, 0, false, true, &vc);
        check(near(th, std::atan(500.0f / 9810.0f) * 57.29578f, 1e-3f), "Vorsteuerung: 500 mm/s² -> +2,92° ohne Fehler");
        vc.accel = -800; vc.ff_gain = 0.5f;
        const float th2 = step(b, 100, 100, 0, false, true, &vc);
        check(near(th2, -0.5f * std::atan(800.0f / 9810.0f) * 57.29578f, 1e-3f), "Vorsteuerung: Vorzeichen und Anteil ff_gain");
    }
    // --- Patch 0035: Einheit in der Vorsteuerung, Vorfaktor des Bestandswegs
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.ki = 0; vc.dt = kDt;
        vc.accel = 128.2f; vc.unit_mm = 3.9f;   // 0,5 m/s² in Einheiten von 3,9 mm/s
        const float th = step(b, 100, 100, 0, false, true, &vc);
        check(near(th, std::atan(500.0f / 9810.0f) * 57.29578f, 2e-3f), "0035: Vorsteuerung mit Einheit 3,9 -> 2,92° bei 0,5 m/s²");
        BalanceState p, q; VelocityControl one; one.on = true; one.dt = kDt;
        VelocityControl sc = one; sc.stock_scale = 0.55f;
        settle(p); settle(q);
        step(p, 100, 40, 0, false, true, &one); step(q, 100, 40, 0, false, true, &sc);
        BalanceState r; VelocityControl ref = one; step(r, 55, 40, 0, false, true, &ref);
        check(q.speed.output == r.speed.output, "0035: Original-P sieht Soll × 0,55");
        check(near(q.vel_i, p.vel_i, 1e-7f) && q.vel_i > 0, "0035: Integral rechnet mit dem vollen Soll");
        VelocityControl off = sc; off.on = false; BalanceState o1, o2;
        check(step(o1, 100, 40, 0, false, true, &off) == step(o2, 100, 40, 0, false, true, nullptr),
              "0035: Rahmenweg aus -> Vorfaktor wirkt nicht");
    }
    // --- Patch 0036: Integral erst bei ruhendem Generator
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.dt = kDt; vc.accel = 128;
        for (int i = 0; i < 500; ++i) step(b, 40, 0, 0, false, true, &vc);   // 2 s Rampe, Ist hängt 40 hinterher
        check(b.vel_i == 0 && b.vel_settled == 0, "0036: während der Rampe (a ≠ 0) kein Integral");
        vc.accel = 0;
        for (int i = 0; i < 149; ++i) step(b, 40, 0, 0, false, true, &vc);   // 0,596 s nach Rampenende
        check(b.vel_i == 0, "0036: bis 0,6 s nach Rampenende kein Integral");
        for (int i = 0; i < 26; ++i) step(b, 40, 0, 0, false, true, &vc);
        check(near(b.vel_i, 0.208f, 0.005f), "0036: danach wächst es (26 Takte × 0,05 × 40 × 0,004 = 0,208°)");
        vc.accel = -50; step(b, 40, 0, 0, false, true, &vc);
        const float frozen = b.vel_i;
        for (int i = 0; i < 100; ++i) step(b, 40, 0, 0, false, true, &vc);
        check(b.vel_i == frozen && b.vel_settled == 0, "0036: neue Rampe hält das Integral an (bleibt stehen)");
        VelocityControl off = vc; off.on = false; settle(b);
        step(b, 40, 0, 0, false, true, &off);
        check(b.vel_settled == 0, "0036: Rahmenweg aus setzt die Wartezeit zurück");
    }
    // --- Integral: wächst mit ki·Fehler·dt, Grenze, Vorzeichen
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.dt = kDt;   // ki 0,05
        settle(b);
        for (int i = 0; i < 250; ++i) step(b, 30, 0, 0, false, true, &vc);   // 1 s, 30 mm/s Fehler
        check(near(b.vel_i, 1.5f, 0.01f), "Integral: 30 mm/s über 1 s -> 1,5°");
        for (int i = 0; i < 1000; ++i) step(b, 30, 0, 0, false, true, &vc);
        check(near(b.vel_i, 4.0f, 1e-4f), "Integral: Grenze i_max 4°");
        for (int i = 0; i < 250; ++i) step(b, 0, 60, 0, false, true, &vc);
        check(near(b.vel_i, 1.0f, 0.01f), "Integral: negativer Fehler baut ab");
        check(b.drive_i == 0, "0015 ist auf dem Rahmenweg aus");
    }
    // --- Anti-Windup und freeze
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.dt = kDt;
        for (int i = 0; i < 250; ++i) step(b, 2000, 0, 0, false, true, &vc);   // P allein 220° > Grenze 100°
        check(b.vel_i == 0 && b.pitch.target == b.speed.output_limit, "Anti-Windup: in der Grenze wächst das Integral nicht");
        BalanceState c; vc.freeze = true;
        for (int i = 0; i < 250; ++i) step(c, 50, 0, 0, false, true, &vc);
        check(c.vel_i == 0, "freeze (Hebeschutz): Integral steht");
    }
    // --- Halten und Ende ohne Sprung
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.dt = kDt;
        settle(b);
        for (int i = 0; i < 250; ++i) step(b, 40, 0, 0, false, true, &vc);
        const float i0 = b.vel_i;
        for (int i = 0; i < 375; ++i) step(b, 0, 0, 0, true, true, &vc);   // 1,5 s halten
        check(near(b.vel_i, i0 * std::exp(-1.0f), 0.02f), "Halten: Integral klingt mit 1,5 s ab");
        BalanceState c; settle(c);
        for (int i = 0; i < 250; ++i) step(c, 40, 40 - 1e-3f, 0, false, true, &vc);
        for (int i = 0; i < 250; ++i) step(c, 40, 0, 0, false, true, &vc);
        const float before = step(c, 40, 40, 0, false, true, &vc);
        VelocityControl off; off.on = false; off.dt = kDt;
        const float after = step(c, 40, 40, 0, false, true, &off);
        check(std::fabs(after - before) < 0.05f, "Rahmenweg endet: kein Sprung im Soll-Nick");
        for (int i = 0; i < 250; ++i) step(c, 40, 40, 0, false, true, &off);
        check(std::fabs(c.vel_i) < 0.1f, "Rahmenweg endet: Integral nach 1 s unter 5 % (Zeitkonstante 0,3 s)");
    }
    // --- Rest von 0015 klingt beim Wechsel ab
    {
        BalanceState b;
        for (int i = 0; i < 1000; ++i) step(b, 200, 0, 0, false, true, nullptr);   // 0015 baut auf
        const float d0 = b.drive_i;
        VelocityControl vc; vc.on = true; vc.dt = kDt; vc.ki = 0;
        const float t0 = step(b, 200, 0, 0, false, true, &vc);
        check(d0 > 0.5f && b.drive_i > 0.9f * d0 && near(t0, b.speed.output + b.drive_i, 1e-3f), "Wechsel auf den Rahmenweg: 0015-Rest bleibt erst stehen");
        for (int i = 0; i < 1000; ++i) step(b, 200, 0, 0, false, true, &vc);
        check(b.drive_i == 0, "0015-Rest klingt ab (4 s)");
    }
    // --- geschlossener Kreis (Modell)
    {
        const LoopResult orig = loop(8, 2.0f, nullptr, 200);
        VelocityControl vc; vc.on = true;
        const LoopResult pi = loop(8, 2.0f, &vc, 200);
        std::printf("       Modell, Last 2°: Original Restfehler %.1f mm/s, mit Integral %.2f mm/s (Schwankung zuletzt %.2f)\n",
                    double(orig.err_end), double(pi.err_end), double(pi.osc_late));
        check(orig.err_end > 15.0f, "Modell: Original-P lässt unter Last ~18 mm/s Abweichung");
        check(std::fabs(pi.err_end) < 1.0f, "Modell: mit Integral nach 8 s < 1 mm/s");
        check(pi.osc_late < 1.0f, "Modell: kein Aufschaukeln");
        V::Frame f; f.v = 300; f.a = 500; f.jerk = 2500;
        VelocityControl ff; ff.on = true;
        VelocityControl noff; noff.on = true; noff.ff_gain = 0;
        const LoopResult with = loop(3, 0, &ff, 0, &f), without = loop(3, 0, &noff, 0, &f);
        std::printf("       Modell, Anfahren 0 -> 300 mm/s: Schleppfehler mit Vorsteuerung %.1f, ohne %.1f mm/s\n",
                    double(with.err_peak), double(without.err_peak));
        check(with.err_peak < 0.5f * without.err_peak, "Modell: Vorsteuerung halbiert den Schleppfehler beim Anfahren");
    }
    std::printf(fail ? "\nFEHLER\n" : "\nalle ok\n");
    return fail;
}
