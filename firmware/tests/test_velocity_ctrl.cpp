// Host-Test 08-velocity: Geschwindigkeitsregler des Fahrrahmens (control/rider_motion.h VelocityControl).
// Prüft am Regler: aus = Byteweg bitgleich (auch mit Fahr-Integral), Vorsteuerung atan(a·unit_mm/g), P allein auf
// Soll − Ist (Original-P + kp − 0,11 bis ±kVelBand beim Fahren, Original-P beim Halten, gleicher Fehler = gleicher
// Soll-Nick in beiden Richtungen, großer Fehler beim Aufschwung nur Original-P + 3,8°), Integral mit geklemmtem Fehler,
// Integral (Vorzeichen, Grenze, Anti-Windup, freeze, erst nach kVelSettleS mit a = 0, Abklingen beim Halten und
// nach dem Ende ohne Sprung), Rest des Fahr-Integrals klingt ab; die Integral-Tests setzen vel_settled vorab (settle()).
// Dazu ein geschlossener Kreis mit einem Fahrmodell, das die Messfahrten 2026-11-25 nachbildet (NICHT der echte
// Rider): Tempo in Firmware-Einheiten (3,9 mm/s), Beschleunigung g·tan(Nick − Last − Bedarf(v)) mit Nick-Bedarf
// 0,8° − 0,05°·v (damit läuft der Original-P auf ~1,8 × Soll, mit dem alten kv 0,025 rückwärts ~1,3 ×), Nick folgt
// dem Soll-Nick mit 0,1 s. Damit: beide Richtungen, Steigung ±2°, am Ende am Soll, kein Aufschaukeln.
// Aufruf: firmware/tests/run.sh test_velocity_ctrl
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
static void settle(BalanceState& b) { b.vel_settled = kVelSettleS; }   // Generator ruht schon

// Ein Reglertakt; gibt den Soll-Nick zurück.
static float step(BalanceState& b, float target, float speed, float pitch, bool holding, bool drive_int,
                  const VelocityControl* vc) {
    Wheels w;
    updateBalance(b, target, speed, speed, pitch, 0, 0, 18, holding, w, drive_int, vc);
    return b.pitch.target;
}

// Fahrmodell in Firmware-Einheiten: v' = g/Einheit · tan(θ − Last − (n0 − c·v)), θ folgt dem Soll mit tau.
constexpr float kUnit = 3.9f, kNeedN0 = 0.8f, kNeedC = 0.05f;
struct Plant { float v = 0, theta = 0; };
static void plantStep(Plant& p, float theta_target, float load_deg, float tau = 0.1f) {
    p.theta += (theta_target - p.theta) * kDt / tau;
    const float need = kNeedN0 - kNeedC * p.v;
    p.v += V::clampAbs(kGravityMmS2 / kUnit * std::tan((p.theta - load_deg - need) * 0.0174533f), 20000.0f) * kDt;
}

struct LoopResult { float err_end = 0, err_peak = 0, osc_late = 0, v_end = 0; };
// Fährt target(t) mit Last; vc == nullptr = Originalweg ohne Fahr-Integral.
static LoopResult loop(float seconds, float load_deg, VelocityControl* vc, float v_target, const V::Frame* frame = nullptr) {
    BalanceState b; Plant p; LoopResult r;
    V::State gs; V::Limits gl; gl.v_max = 600; gl.w_max = 300; gl.set = true;
    if (frame) { V::receive(gs, gl, *frame, 0); V::tick(gs, gl, 0); }
    float vmin = 1e9f, vmax = -1e9f;
    const int n = int(seconds / kDt);
    for (int i = 0; i < n; ++i) {
        float tgt = v_target, acc = 0;
        if (frame) { V::tick(gs, gl, std::uint32_t((i + 1) * kDt * 1e6f)); tgt = gs.v; acc = gs.fwd.a; }
        if (vc) { vc->accel = acc; vc->dt = kDt; vc->unit_mm = kUnit; }
        const float th = step(b, tgt, p.v, p.theta, false, false, vc);
        plantStep(p, th, load_deg);
        const float e = tgt - p.v;
        r.err_peak = std::max(r.err_peak, std::fabs(e));
        if (i >= n - int(2.0f / kDt)) { vmin = std::min(vmin, p.v); vmax = std::max(vmax, p.v); }
        r.err_end = e; r.v_end = p.v;
    }
    r.osc_late = vmax - vmin;
    return r;
}

int main() {
    // --- aus = Originalweg bitgleich (mit Fahr-Integral)
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
        check(same && b.vel_i == 0, "Rahmenweg aus: Soll-Nick und Fahr-Integral bitgleich zum Original");
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
    // --- Einheit in der Vorsteuerung
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.ki = 0; vc.dt = kDt;
        vc.accel = 128.2f; vc.unit_mm = 3.9f;   // 0,5 m/s² in Einheiten von 3,9 mm/s
        const float th = step(b, 100, 100, 0, false, true, &vc);
        check(near(th, std::atan(500.0f / 9810.0f) * 57.29578f, 2e-3f), "Vorsteuerung mit Einheit 3,9 -> 2,92° bei 0,5 m/s²");
    }
    // --- Integral erst bei ruhendem Generator
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.dt = kDt; vc.accel = 128;
        for (int i = 0; i < 500; ++i) step(b, 20, 0, 0, false, true, &vc);   // 2 s Rampe, Ist hängt 20 hinterher
        check(b.vel_i == 0 && b.vel_settled == 0, "während der Rampe (a ≠ 0) kein Integral");
        vc.accel = 0;
        for (int i = 0; i < 149; ++i) step(b, 20, 0, 0, false, true, &vc);   // 0,596 s nach Rampenende
        check(b.vel_i == 0, "bis 0,6 s nach Rampenende kein Integral");
        for (int i = 0; i < 26; ++i) step(b, 20, 0, 0, false, true, &vc);
        check(near(b.vel_i, 0.312f, 0.005f), "danach wächst es (26 Takte × 0,15 × 20 × 0,004 = 0,312°)");
        vc.accel = -50; step(b, 20, 0, 0, false, true, &vc);
        const float frozen = b.vel_i;
        for (int i = 0; i < 100; ++i) step(b, 20, 0, 0, false, true, &vc);
        check(b.vel_i == frozen && b.vel_settled == 0, "neue Rampe hält das Integral an (bleibt stehen)");
        VelocityControl off = vc; off.on = false; settle(b);
        step(b, 20, 0, 0, false, true, &off);
        check(b.vel_settled == 0, "Rahmenweg aus setzt die Wartezeit zurück");
    }
    // --- P allein auf Soll − Ist
    {
        VelocityControl vc; vc.on = true; vc.dt = kDt; vc.ki = 0;
        BalanceState a, b, c, d2;
        const float f = step(a, 60, 40, 0, false, true, &vc);
        check(near(f, 0.3f * 20, 0.02f), "Fahren: kp 0,3 × 20 Einheiten Fehler (Bandrand) -> 6°");
        const float r = step(b, -20, -40, 0, false, true, &vc);
        check(f == r, "gleicher Fehler, andere Richtung/anderes Tempo: gleicher Soll-Nick (kein Tempo-Modell)");
        check(step(c, 60, 60, 0, false, true, &vc) == 0.0f, "ohne Fehler kein Soll-Nick");
        const float h = step(d2, 60, 40, 0, true, true, &vc);
        check(near(h, d2.speed.output, 1e-6f) && near(h, 0.14f * 20, 0.02f), "Halten: Original-P 0,14");
        BalanceState g; const float big = step(g, 0, -190, 0, false, true, &vc);
        check(near(big, 0.11f * 190 + (0.3f - 0.11f) * kVelBand, 0.1f), "großer Fehler (Aufschwung, Räder 190): Original-P + 0,19 × 20 = 24,7°, nicht 57°");
    }
    // --- Integral: wächst mit ki·Fehler·dt, Grenze, Vorzeichen
    {
        BalanceState b; VelocityControl vc; vc.on = true; vc.dt = kDt;   // ki 0,15
        settle(b);
        for (int i = 0; i < 250; ++i) step(b, 20, 0, 0, false, true, &vc);   // 1 s, 20 Einheiten Fehler
        check(near(b.vel_i, 3.0f, 0.02f), "Integral: 20 Einheiten über 1 s -> 3°");
        for (int i = 0; i < 1000; ++i) step(b, 20, 0, 0, false, true, &vc);
        check(near(b.vel_i, 8.0f, 1e-4f), "Integral: Grenze i_max 8°");
        for (int i = 0; i < 250; ++i) step(b, 0, 20, 0, false, true, &vc);
        check(near(b.vel_i, 5.0f, 0.02f), "Integral: negativer Fehler baut ab (8° − 3° = 5°)");
        BalanceState o; settle(o);
        for (int i = 0; i < 250; ++i) step(o, 30, 0, 0, false, true, &vc);
        check(near(o.vel_i, 3.0f, 0.02f), "Integral: außerhalb des Bands nur mit dem Bandrand (Fehler 30 wie 20 -> 3° in 1 s)");
        check(b.drive_i == 0, "Fahr-Integral ist auf dem Rahmenweg aus");
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
        for (int i = 0; i < 250; ++i) step(b, 20, 0, 0, false, true, &vc);
        const float i0 = b.vel_i;
        for (int i = 0; i < 375; ++i) step(b, 0, 0, 0, true, true, &vc);   // 1,5 s halten
        check(near(b.vel_i, i0 * std::exp(-1.0f), 0.02f), "Halten: Integral klingt mit 1,5 s ab");
        BalanceState c; settle(c);
        for (int i = 0; i < 250; ++i) step(c, 40, 40 - 1e-3f, 0, false, true, &vc);
        for (int i = 0; i < 250; ++i) step(c, 20, 0, 0, false, true, &vc);
        const float i_start = c.vel_i;
        const float before = step(c, 40, 40, 0, false, true, &vc);
        VelocityControl off; off.on = false; off.dt = kDt;
        const float after = step(c, 40, 40, 0, false, true, &off);
        check(std::fabs(after - before) < 0.05f, "Rahmenweg endet: kein Sprung im Soll-Nick");
        for (int i = 0; i < 250; ++i) step(c, 40, 40, 0, false, true, &off);
        check(std::fabs(c.vel_i) < 0.05f * i_start, "Rahmenweg endet: Integral nach 1 s unter 5 % (Zeitkonstante 0,3 s)");
    }
    // --- Rest des Fahr-Integrals klingt beim Wechsel ab
    {
        BalanceState b;
        for (int i = 0; i < 1000; ++i) step(b, 200, 0, 0, false, true, nullptr);   // Fahr-Integral baut auf
        const float d0 = b.drive_i;
        VelocityControl vc; vc.on = true; vc.dt = kDt; vc.ki = 0;
        const float t0 = step(b, 200, 0, 0, false, true, &vc);
        check(d0 > 0.5f && b.drive_i > 0.9f * d0 && near(t0, b.speed.output + (vc.kp - b.speed.kp) * kVelBand + b.drive_i, 1e-3f), "Wechsel auf den Rahmenweg: Rest des Fahr-Integrals bleibt erst stehen");
        for (int i = 0; i < 1000; ++i) step(b, 200, 0, 0, false, true, &vc);
        check(b.drive_i == 0, "Rest des Fahr-Integrals klingt ab (4 s)");
    }
    // --- geschlossener Kreis (Modell der Messfahrten)
    {
        const LoopResult orig = loop(6, 0, nullptr, 46);
        std::printf("       Modell: Original-P bei Soll 46 -> %.2f × Soll\n", double(orig.v_end / 46));
        check(orig.v_end > 1.5f * 46, "Modell: Original-P läuft auf ~1,8 × Soll (wie am Rider)");
        float worst = 0;
        for (float s : {46.0f, 69.0f, -46.0f, -69.0f}) for (float load : {0.0f, 2.0f, -2.0f}) {
            V::Frame f; f.v = s; f.a = 128; f.jerk = 641;   // xgonav: 0,5 m/s², 2,5 m/s³ in Einheiten von 3,9 mm/s
            VelocityControl vc; vc.on = true;
            const LoopResult r = loop(6, load, &vc, 0, &f);
            const float q = r.v_end / s;
            std::printf("       Modell, Soll %+3.0f, Steigung %+.0f°: Ende %.3f × Soll, Spitze %.2f × Soll, Schwankung zuletzt %.2f\n",
                        double(s), double(load), double(q), double(1 + r.err_peak / std::fabs(s)), double(r.osc_late));
            worst = std::max(worst, std::max(std::fabs(q - 1), 0.5f * r.osc_late / std::fabs(s)));
        }
        check(worst < 0.03f, "Modell: vorwärts/rückwärts, eben und ±2° Steigung nach 6 s auf ±3 % am Soll, zuletzt < 6 % Schwankung");
    }
    std::printf(fail ? "\nFEHLER\n" : "\nalle ok\n");
    return fail;
}
