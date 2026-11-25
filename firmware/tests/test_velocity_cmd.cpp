// Host-Test 08-velocity: Fahrrahmen in mm/s und °/s (Register 0x1a/0x1b, velocity_cmd.h), 0xa0/0xa1 entfernt.
// Prüft: Rahmen- und Grenzen-Format (0x1b 14 Byte: Einheit, 0 = Standard, Bytes 8..13 ohne Bedeutung; andere Längen verworfen),
// Ablehnung ohne Grenzen, Begrenzung auf v_max/w_max, Beschleunigungsbetrieb (Vorzeichen, Grenze), Wächter,
// Abbruch, Byte-Gegenwert, Protokoll (Längen, Lesen 0x1a, 0xa0/0xa1 unbekannt) und dass der Rahmenweg in
// updateCommandTargets ohne Byte-Totband ankommt, der Byteweg aber unverändert bleibt.
// Sollwertgenerator (S-Kurve): Beschleunigung ≤ a, Ruck ≤ Ruckgrenze, kein Überschwingen, Dauer wie gerechnet,
// Umkehr, Beschleunigungsbetrieb mit Ruck, Wächter mit den Rahmengrenzen, Übernahme aus der Fahrt, Drehachse.
// Krümmungsbetrieb (bit1): Format, Fähigkeitsbyte, w = κ·v_soll, Bahn auf dem Kreis (Anfahren/Bremsen/rückwärts/
// Wächter), v/a durch w_max/alpha begrenzt, κ-Nachführung, Übernahme aus dem Drehratenweg. Kursziel (bit2): Ziel ±0,5°
// ohne Überschwingen, Wechselbit, Wächter, im Fahren, aus der Kurve; Prüfvektoren (XGO_VECTORS=1).
// Aufruf: firmware/tests/run.sh test_velocity_cmd
#include "velocity_cmd.h"
#include "command_protocol.h"
#include "control/rider_motion.h"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace V = rider2::velocity;
using namespace rider2::protocol;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}
static bool near(float a, float b, float tol = 1e-3f) { return std::fabs(a - b) <= tol; }

static void frameBytes(std::uint8_t* d, bool speed, int v, int a, int j, int w10, int alpha, int jw) {
    std::memset(d, 0, V::kFrameLen);
    d[0] = speed ? 0 : V::kFlagNoSpeed;
    V::put16(d + 1, std::uint16_t(std::int16_t(v))); V::put16(d + 3, std::uint16_t(std::int16_t(a)));
    V::put16(d + 5, std::uint16_t(j)); V::put16(d + 7, std::uint16_t(std::int16_t(w10)));
    V::put16(d + 9, std::uint16_t(alpha)); V::put16(d + 11, std::uint16_t(jw));
}
static V::Limits limits(int vmax, int wmax10, int timeout) {
    std::uint8_t d[V::kLimitsLen] = {};
    V::put16(d, std::uint16_t(vmax)); V::put16(d + 2, std::uint16_t(wmax10)); V::put16(d + 4, std::uint16_t(timeout));
    V::Limits l; V::parseLimits(d, sizeof d, l); return l;
}


// Läuft eine Achse über den Generator und misst Spitzen von Beschleunigung und Ruck sowie das Überschwingen.
struct Profile { float t_end = -1, a_peak = 0, j_peak = 0, x_min = 1e9f, x_max = -1e9f, a_snap = 0; bool monotonic = true; };
static Profile run(V::Axis& ax, float target, float amax, float jmax, float seconds, float dt = 0.003f) {
    Profile p; float a_prev = ax.a, x_prev = ax.x; const float dir = target >= ax.x ? 1.0f : -1.0f;
    for (float t = 0; t < seconds; t += dt) {
        V::approach(ax, target, amax, jmax, dt);
        p.a_peak = std::max(p.a_peak, std::fabs(ax.a));
        if (!(ax.x == target && ax.a == 0)) p.j_peak = std::max(p.j_peak, std::fabs(ax.a - a_prev) / dt);   // Einrasten zählt nicht
        if ((ax.x - x_prev) * dir < -1e-4f) p.monotonic = false;
        p.x_min = std::min(p.x_min, ax.x); p.x_max = std::max(p.x_max, ax.x);
        if (p.t_end < 0 && ax.x == target && ax.a == 0) { p.t_end = t + dt; p.a_snap = std::fabs(a_prev); }
        a_prev = ax.a; x_prev = ax.x;
    }
    return p;
}

static void generatorTests() {
    {   // 0 -> 400 mm/s, a 800, Ruck 4000: Dauer v/a + a/j = 0,5 + 0,2 = 0,7 s
        V::Axis ax; Profile p = run(ax, 400, 800, 4000, 2.0f);
        check(p.a_peak <= 800.5f, "S-Kurve: Beschleunigung ≤ a_max");
        check(p.j_peak <= 4000.0f * 1.01f, "S-Kurve: Ruck ≤ Ruckgrenze");
        check(p.monotonic && p.x_max <= 400.0f, "S-Kurve: monoton, kein Überschwingen");
        check(p.t_end > 0.66f && p.t_end < 0.76f, "S-Kurve: am Ziel nach ~0,7 s");
        check(p.a_snap <= 2.0f * 4000.0f * 0.003f, "S-Kurve: Einrasten mit höchstens zwei Ruckstufen Rest-a");
    }
    {   // kleiner Sprung, a_max nie erreicht: Dauer 2·sqrt(dv/j) = 0,141 s
        V::Axis ax; Profile p = run(ax, 20, 800, 4000, 1.0f);
        check(p.a_peak < 300 && p.t_end > 0.13f && p.t_end < 0.17f, "kleiner Sprung: Dreieck im a, ~0,14 s");
        check(p.x_max <= 20.0f, "kleiner Sprung: kein Überschwingen");
        check(p.a_snap <= 2.0f * 4000.0f * 0.003f, "kleiner Sprung: Einrasten mit höchstens zwei Ruckstufen Rest-a");
    }
    {   // Umkehr 300 -> -300
        V::Axis ax{300, 0}; Profile p = run(ax, -300, 600, 3000, 3.0f);
        check(p.monotonic && p.x_min >= -300.0f && ax.x == -300, "Umkehr 300 -> -300: monoton, kein Überschwingen");
        check(p.j_peak <= 3000.0f * 1.01f && p.a_peak <= 600.5f, "Umkehr: Grenzen eingehalten");
    }
    {   // Zieländerung mitten in der Beschleunigung: zurück auf 0 ohne Ruckspitze
        V::Axis ax; run(ax, 400, 800, 4000, 0.3f); Profile p = run(ax, 0, 800, 4000, 2.0f);
        check(p.j_peak <= 4000.0f * 1.01f && ax.x == 0, "Zieländerung während der Rampe: Ruck begrenzt, am Ziel");
        check(p.x_min >= -15.0f, "Zieländerung: höchstens kleiner Unterschwinger");
    }
    {   // ohne Ruckgrenze: Rampe mit a, ohne Grenzen: Sprung
        V::Axis ax; Profile p = run(ax, 300, 600, 0, 1.0f);
        check(p.t_end > 0.49f && p.t_end < 0.51f && p.a_peak <= 600.0f, "Ruck 0: lineare Rampe 0,5 s");
        V::Axis bx; V::approach(bx, 300, 0, 0, 0.003f);
        check(bx.x == 300, "a 0: Sprung");
    }
    {   // Beschleunigungsbetrieb mit Ruck: hoch bis v_max, a = 0 hält, negativ bis -v_max
        V::State s; V::Limits l = limits(400, 600, 0);
        V::Frame f; f.has_speed = false; f.a = 300; f.jerk = 2000;
        std::uint32_t t = 0; V::receive(s, l, f, t);
        V::tick(s, l, t);                                              // erster Takt: Start, dt = 0
        for (int i = 0; i < 50; ++i) { t += 3000; V::tick(s, l, t); }
        check(near(s.fwd.a, 300, 1.0f), "Beschleunigungsbetrieb: a nach 0,15 s mit Ruck 2000 auf 300");
        for (int i = 0; i < 1000; ++i) { t += 3000; V::tick(s, l, t); }
        check(s.v == 400 && s.fwd.a == 0, "Beschleunigungsbetrieb: rastet bei v_max ein");
        f.a = 0; V::receive(s, l, f, t);
        for (int i = 0; i < 100; ++i) { t += 3000; V::tick(s, l, t); }
        check(s.v == 400, "a = 0 hält das Tempo");
        f.a = -300; V::receive(s, l, f, t);
        float jp = 0, ap = s.fwd.a;
        for (int i = 0; i < 2000; ++i) { t += 3000; V::tick(s, l, t); if (s.v != -400) jp = std::max(jp, std::fabs(s.fwd.a - ap) / 0.003f); ap = s.fwd.a; }
        check(s.v == -400 && jp <= 2000.0f * 1.01f, "negativ bis -v_max, Ruck begrenzt");
    }
    {   // Wächter bremst mit den Grenzen des letzten Rahmens
        V::State s; V::Limits l = limits(500, 600, 100);
        V::Frame f; f.v = 300; f.a = 600; f.jerk = 3000;
        std::uint32_t t = 0; V::receive(s, l, f, t);
        for (int i = 0; i < 30; ++i) { t += 3000; V::tick(s, l, t); }   // 90 ms, Wächter noch nicht
        const float v_before = s.v;
        float jp = 0, ap = s.fwd.a;
        for (int i = 0; i < 1000; ++i) { t += 3000; V::tick(s, l, t); if (s.v != 0) jp = std::max(jp, std::fabs(s.fwd.a - ap) / 0.003f); ap = s.fwd.a; }
        check(v_before > 0 && s.timed_out && s.v == 0 && jp <= 3000.0f * 1.01f, "Wächter: weich auf 0 mit Rahmen-a und -Ruck");
    }
    {   // Übernahme aus der Fahrt und kleinere Grenzen
        V::State s; V::Limits l = limits(500, 600, 0);
        V::Frame f; f.v = 300; f.a = 500; f.jerk = 2500; f.w = 0; f.alpha = 100;
        V::receive(s, l, f, 0); V::tick(s, l, 0, 250.0f, -20.0f);
        check(s.v == 250 && s.w == -20, "erster Takt beginnt beim aktuellen Tempo und der Drehrate");
        V::Limits small = limits(100, 300, 0); V::tick(s, small, 3000);
        check(s.v == 100 && near(s.w, -20, 0.5f), "neue kleinere v_max gilt sofort");
    }
    {   // Drehachse: alpha und Drehruck
        V::State s; V::Limits l = limits(500, 900, 0);
        V::Frame f; f.w = 60; f.alpha = 120; f.jerk_w = 600;
        std::uint32_t t = 0; V::receive(s, l, f, t); V::tick(s, l, t);
        float ap = 0, peak = 0, jp = 0; bool reached = false; float t_end = 0;
        for (int i = 1; i < 1000; ++i) { t += 3000; V::tick(s, l, t);
            peak = std::max(peak, std::fabs(s.turn.a));
            if (s.w != 60) jp = std::max(jp, std::fabs(s.turn.a - ap) / 0.003f);
            ap = s.turn.a; if (!reached && s.w == 60) { reached = true; t_end = i * 0.003f; } }
        // Dauer 60/120 + 120/600 = 0,7 s
        check(reached && peak <= 120.5f && jp <= 600.0f * 1.01f && t_end > 0.66f && t_end < 0.76f, "Drehen: S-Kurve 0 -> 60 °/s in ~0,7 s");
    }
}


// --- Krümmungsbetrieb (bit1): Szenarien als Folge von Rahmen, je Takt ausgewertet. Mit XGO_VECTORS=1 schreibt jedes
// Szenario firmware/tests/vectors/<name>.csv (Lauf aus dem Repo-Root, run.sh), für den Nachbau am Pi
// (tests/test_fahrrahmen.py): je Takt t_us, rx (1 = in diesem Takt neuer Rahmen, vor dem Takt angenommen), die 13
// Rahmenbytes als Hex, Grenzen (v_max, w_max, Wächter, unit_mm) und danach v, w, a (Fahrachse), κ, timed_out,
// Restwinkel des Kursziels (°) und angekommen.
struct KEv { std::uint32_t t_us; std::uint8_t d[V::kFrameLen]; };
struct KStep { std::uint32_t t_us; float v, w, a, kappa; bool timed_out, kappa_on; float rest; bool done, course_on; };
static void kappaBytes(std::uint8_t* d, bool speed, int v, int a, int j, float kappa, int alpha, int jw) {
    frameBytes(d, speed, v, a, j, 0, alpha, jw);
    d[0] |= V::kFlagKappa; V::put16(d + 7, std::uint16_t(std::int16_t(std::lround(kappa * 1000.0f))));
}
static KEv kev(std::uint32_t t_us, const std::uint8_t* d) { KEv e; e.t_us = t_us; std::memcpy(e.d, d, V::kFrameLen); return e; }
static constexpr std::uint32_t kTickUs = 4000;   // Regeltakt am Rider
static std::vector<KStep> runScenario(const char* name, const V::Limits& l, const std::vector<KEv>& evs, float seconds) {
    V::State s; std::vector<KStep> out; std::size_t next = 0;
    FILE* csv = nullptr;
    if (const char* env = std::getenv("XGO_VECTORS"); env && *env == '1') {
        char path[256]; std::snprintf(path, sizeof path, "firmware/tests/vectors/%s.csv", name);
        csv = std::fopen(path, "w");
        if (!csv) std::printf("FEHLER Vektor %s nicht schreibbar\n", path);
        else std::fprintf(csv, "t_us,rx,frame_hex,v_max,w_max,timeout_ms,unit_mm,v,w,a,kappa,timed_out,rest,done\n");
    }
    const std::uint8_t* last = nullptr;
    for (std::uint32_t t = 0; t <= std::uint32_t(seconds * 1e6f); t += kTickUs) {
        bool rx = false;
        while (next < evs.size() && evs[next].t_us <= t) {
            V::Frame f; if (V::parseFrame(evs[next].d, V::kFrameLen, f)) V::receive(s, l, f, t);
            last = evs[next].d; rx = true; ++next;
        }
        V::tick(s, l, t);
        out.push_back({t, s.v, s.w, s.fwd.a, s.kappa, s.timed_out, s.kappa_on, s.course_rest, s.course_done, s.course_on});
        if (csv) {
            char hex[2 * V::kFrameLen + 1] = {};
            if (last) for (int i = 0; i < V::kFrameLen; ++i) std::snprintf(hex + 2 * i, 3, "%02x", last[i]);
            std::fprintf(csv, "%u,%d,%s,%.9g,%.9g,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%d\n", unsigned(t), rx ? 1 : 0, hex,
                         double(l.v_max), double(l.w_max), unsigned(l.timeout_ms), double(l.unit_mm),
                         double(s.v), double(s.w), double(s.fwd.a), double(s.kappa), s.timed_out ? 1 : 0,
                         double(s.course_rest), s.course_done ? 1 : 0);
        }
    }
    if (csv) std::fclose(csv);
    return out;
}
// Bahn aus v (Einheiten) und w (°/s): größter Abstand vom Kreis um (0, 1/κ) mit Radius 1/|κ|, in m.
static float circleError(const std::vector<KStep>& tr, float kappa, float m, std::size_t from = 0, std::size_t to = SIZE_MAX) {
    double x = 0, y = 0, psi = 0, worst = 0; const double r = 1.0 / kappa;
    for (std::size_t i = 1; i < tr.size() && i < to; ++i) {
        const double dt = (tr[i].t_us - tr[i - 1].t_us) * 1e-6;
        const double pm = psi + 0.5 * tr[i].w / V::kDeg * dt;   // Mittelpunktregel
        x += tr[i].v * m * dt * std::cos(pm); y += tr[i].v * m * dt * std::sin(pm);
        psi += tr[i].w / V::kDeg * dt;
        if (i >= from) worst = std::max(worst, std::fabs(std::hypot(x, y - r) - std::fabs(r)));
    }
    return float(worst);
}
static V::Limits kLimits(int timeout = 300) { V::Limits l = limits(600, 3000, timeout); l.unit_mm = 3.9f; return l; }

static void kappaTests() {
    const float m = 3.9e-3f;   // m/s je Einheit am Rider
    // Format
    {
        std::uint8_t d[V::kFrameLen]; kappaBytes(d, true, 46, 128, 641, 2.0f, 360, 1800);
        V::Frame f; check(V::parseFrame(d, sizeof d, f) && f.kappa_mode && near(f.kappa, 2.0f, 1e-6f) && f.w == 0,
                          "Krümmung: bit1, Feld w = κ 0,001 1/m");
        kappaBytes(d, true, 46, 128, 641, -32.767f, 360, 1800); V::parseFrame(d, sizeof d, f);
        check(near(f.kappa, -32.767f, 1e-5f), "κ-Bereich ±32,767 1/m");
        std::uint8_t u[V::kFrameLen]; frameBytes(u, true, 46, 0, 0, 100, 0, 0);
        u[0] = 8; check(!V::parseFrame(u, sizeof u, f), "unbekanntes Bit (bit3) -> verworfen");
        u[0] = 0x80; check(!V::parseFrame(u, sizeof u, f), "unbekanntes Bit (bit7) -> verworfen");
        u[0] = 0; check(V::parseFrame(u, sizeof u, f) && !f.kappa_mode && near(f.w, 10.0f, 1e-5f), "ohne bit1: w wie bisher");
        V::State s; V::Limits l = kLimits(); V::Frame k; k.kappa_mode = true; k.kappa = 1; k.v = 20;
        V::receive(s, l, k, 0); V::tick(s, l, 0);
        std::uint8_t g[V::kDiagLen]; V::fillDiag(g, s, l);
        check(g[1] == V::kCaps && (g[1] & 1) && (g[0] & 16), "0x1a lesen: Fähigkeitsbyte bit0 Krümmung, Flag bit4 Krümmungsbetrieb");
        V::cancel(s); V::fillDiag(g, s, l);
        check(!(g[0] & 16) && !s.kappa_on && g[1] == V::kCaps, "nach Abbruch kein Krümmungsbetrieb, Fähigkeiten bleiben");
    }
    // Kreis v 46 (0,18 m/s), κ 2 1/m: Anfahren, 2,5 s, Bremsen auf 0; Bahn liegt auf dem Kreis
    {
        std::uint8_t a[V::kFrameLen], b[V::kFrameLen];
        kappaBytes(a, true, 46, 128, 641, 2.0f, 360, 1800); kappaBytes(b, true, 0, 128, 641, 2.0f, 360, 1800);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 3000000; t += 100000) ev.push_back(kev(t, a));
        for (std::uint32_t t = 3000000; t < 5000000; t += 100000) ev.push_back(kev(t, b));
        const auto tr = runScenario("kappa_kreis", kLimits(), ev, 5.0f);
        bool coupled = true, alpha_ok = true; float wpk = 0;
        for (std::size_t i = 1; i < tr.size(); ++i) {
            if (std::fabs(tr[i].w - 2.0f * m * tr[i].v * V::kDeg) > 1e-4f) coupled = false;
            if (std::fabs(tr[i].w - tr[i - 1].w) > 360.0f * kTickUs * 1e-6f * 1.01f + 1e-4f) alpha_ok = false;
            wpk = std::max(wpk, std::fabs(tr[i].w));
        }
        const float err = circleError(tr, 2.0f, m);
        std::printf("       Kreis κ 2: größte Abweichung %.2f mm, w bis %.1f °/s, Ende v %.2f\n", double(err * 1000), double(wpk), double(tr.back().v));
        check(coupled, "Kreis: w = κ · v_soll · 180/π in jedem Takt");
        check(alpha_ok, "Kreis: Drehbeschleunigung ≤ alpha");
        check(err < 0.01f, "Kreis: Bahn beim Anfahren, Fahren und Bremsen < 1 cm vom Kreis");
        check(tr.back().v == 0 && tr.back().w == 0, "Kreis: steht am Ende (v = w = 0)");
    }
    // rückwärts mit gleichem κ: derselbe Kreis rückwärts (w < 0)
    {
        std::uint8_t a[V::kFrameLen]; kappaBytes(a, true, -46, 128, 641, 2.0f, 360, 1800);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 3000000; t += 100000) ev.push_back(kev(t, a));
        const auto tr = runScenario("kappa_rueck", kLimits(), ev, 3.0f);
        check(tr.back().w < 0 && near(tr.back().w, 2.0f * m * -46 * V::kDeg, 1e-3f), "rückwärts: w = κ·v < 0");
        check(circleError(tr, 2.0f, m) < 0.01f, "rückwärts: derselbe Kreis");
    }
    // Grenze: κ 10 1/m, w_max 60 °/s -> |v| ≤ w_max/|κ| = 26,85 Einheiten, w nie über w_max
    {
        V::Limits l = limits(600, 600, 300); l.unit_mm = 3.9f;
        std::uint8_t a[V::kFrameLen]; kappaBytes(a, true, 46, 128, 641, 10.0f, 360, 1800);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 3000000; t += 100000) ev.push_back(kev(t, a));
        const auto tr = runScenario("kappa_grenze", l, ev, 3.0f);
        float wpk = 0; for (const auto& x : tr) wpk = std::max(wpk, std::fabs(x.w));
        const float vlim = 60.0f / V::kDeg / (10.0f * m);
        check(near(tr.back().v, vlim, 0.01f) && wpk <= 60.0f + 1e-3f, "Grenze: v auf w_max/|κ| begrenzt, w ≤ w_max");
        check(circleError(tr, 10.0f, m) < 0.01f, "Grenze: Bahn bleibt auf dem Kreis (r 10 cm)");
    }
    // S-Kurve: κ 2 -> -2 bei v 46: κ läuft mit |dκ/dt| ≤ alpha/|v| nach, w ohne Sprung
    {
        std::uint8_t a[V::kFrameLen], b[V::kFrameLen];
        kappaBytes(a, true, 46, 128, 641, 2.0f, 360, 1800); kappaBytes(b, true, 46, 128, 641, -2.0f, 360, 1800);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 2000000; t += 100000) ev.push_back(kev(t, a));
        for (std::uint32_t t = 2000000; t < 4000000; t += 100000) ev.push_back(kev(t, b));
        const auto tr = runScenario("kappa_s", kLimits(), ev, 4.0f);
        bool rate_ok = true, smooth = true;
        for (std::size_t i = 1; i < tr.size(); ++i) {
            const float vm = std::fabs(tr[i - 1].v * m);
            if (vm > 1e-3f && std::fabs(tr[i].kappa - tr[i - 1].kappa) > 360.0f / V::kDeg / vm * kTickUs * 1e-6f * 1.001f + 1e-6f) rate_ok = false;
            if (tr[i].t_us > 1000000 && std::fabs(tr[i].w - tr[i - 1].w) > 360.0f * kTickUs * 1e-6f * 1.01f) smooth = false;
        }
        check(rate_ok, "S-Kurve: |dκ/dt| ≤ alpha/|v|");
        check(smooth, "S-Kurve: w ohne Sprung (Änderung je Takt ≤ alpha·dt)");
        check(near(tr.back().kappa, -2.0f, 1e-6f) && tr.back().w < 0, "S-Kurve: am Ende κ -2, Rechtskurve");
    }
    // Übernahme: aus einer Fahrt mit w (alter Rahmen) in den Krümmungsbetrieb ohne Sprung in w
    {
        std::uint8_t a[V::kFrameLen], b[V::kFrameLen];
        frameBytes(a, true, 46, 128, 641, 200, 360, 1800); kappaBytes(b, true, 46, 128, 641, 1.0f, 360, 1800);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 2000000; t += 100000) ev.push_back(kev(t, a));
        for (std::uint32_t t = 2000000; t < 4000000; t += 100000) ev.push_back(kev(t, b));
        const auto tr = runScenario("kappa_uebernahme", kLimits(), ev, 4.0f);
        const std::size_t k = 2000000 / kTickUs;
        check(!tr[k - 1].kappa_on && tr[k].kappa_on && std::fabs(tr[k].w - tr[k - 1].w) <= 360.0f * kTickUs * 1e-6f * 1.01f,
              "Übernahme: Wechsel w -> κ ohne Sprung in w (κ beginnt bei w/v, dann ≤ alpha·dt je Takt)");
        check(near(tr.back().kappa, 1.0f, 1e-6f) && near(tr.back().w, 1.0f * m * 46 * V::kDeg, 1e-3f), "Übernahme: am Ende κ 1");
    }
    // Wächter im Krümmungsbetrieb: bremst auf 0, w folgt κ·v, Bahn bleibt
    {
        std::uint8_t a[V::kFrameLen]; kappaBytes(a, true, 46, 128, 641, 2.0f, 360, 1800);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 1500000; t += 100000) ev.push_back(kev(t, a));
        const auto tr = runScenario("kappa_waechter", kLimits(300), ev, 3.5f);
        check(tr.back().timed_out && tr.back().v == 0 && tr.back().w == 0, "Wächter: steht, w = 0");
        check(circleError(tr, 2.0f, m) < 0.01f, "Wächter: Bremsen auf dem Kreis");
    }
    // Beschleunigungsbetrieb mit κ: w = κ·v, a ≤ alpha/|κ|
    {
        std::uint8_t a[V::kFrameLen]; kappaBytes(a, false, 0, 2000, 0, 5.0f, 360, 0);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 1000000; t += 100000) ev.push_back(kev(t, a));
        const auto tr = runScenario("kappa_accel", kLimits(), ev, 1.0f);
        const float alim = 360.0f / V::kDeg / (5.0f * m);
        bool ok = true; for (const auto& x : tr) if (std::fabs(x.a) > alim * 1.001f || std::fabs(x.w - 5.0f * m * x.v * V::kDeg) > 1e-3f) ok = false;
        check(ok && tr.back().v > 0, "Beschleunigungsbetrieb: a ≤ alpha/|κ|, w = κ·v");
    }
    // Drehratenweg als Bezug für den Nachbau (alte Rahmen unverändert): v 46 + w 20 °/s, dann 0
    {
        std::uint8_t a[V::kFrameLen], b[V::kFrameLen];
        frameBytes(a, true, 46, 128, 641, 200, 360, 1800); frameBytes(b, true, 0, 128, 641, 0, 360, 1800);
        std::vector<KEv> ev; for (std::uint32_t t = 0; t < 2000000; t += 100000) ev.push_back(kev(t, a));
        for (std::uint32_t t = 2000000; t < 3500000; t += 100000) ev.push_back(kev(t, b));
        const auto tr = runScenario("rate_bezug", kLimits(), ev, 3.5f);
        bool none = true; for (const auto& x : tr) if (x.kappa_on) none = false;
        check(none && tr[1999999 / kTickUs].w == 20.0f && tr.back().v == 0, "Drehratenweg: kein Krümmungsbetrieb, w 20 °/s wie bisher");
    }
}

// --- Kursziel (bit2, Wechselbit bit4)
static void courseBytes(std::uint8_t* d, bool seq, int v, float dpsi_deg, int alpha, int jw) {
    frameBytes(d, true, v, 128, 641, 0, alpha, jw);
    d[0] |= V::kFlagCourse | (seq ? V::kFlagCourseSeq : 0);
    V::put16(d + 7, std::uint16_t(std::int16_t(std::lround(dpsi_deg * 10.0f))));
}
struct Turn { float angle = 0, over = 0, wpk = 0, dw = 0; };
// gedrehter Winkel (∫w dt), Überschwingen über das Ziel, größte Drehrate und Änderung je Takt
static Turn turned(const std::vector<KStep>& tr, float goal, std::size_t from = 0) {
    Turn r; const float sg = goal >= 0 ? 1.0f : -1.0f;
    for (std::size_t i = from + 1; i < tr.size(); ++i) {
        r.angle += tr[i].w * kTickUs * 1e-6f;
        r.over = std::max(r.over, (r.angle - goal) * sg);
        r.wpk = std::max(r.wpk, std::fabs(tr[i].w)); r.dw = std::max(r.dw, std::fabs(tr[i].w - tr[i - 1].w));
    }
    return r;
}
static std::vector<KEv> repeat(const std::uint8_t* d, std::uint32_t from, std::uint32_t to) {
    std::vector<KEv> ev; for (std::uint32_t t = from; t < to; t += 100000) ev.push_back(kev(t, d)); return ev;
}

static void courseTests() {
    const V::Limits l = []{ V::Limits x = limits(600, 1000, 300); x.unit_mm = 3.9f; return x; }();   // w_max 100 °/s
    const float dw_max = 360.0f * kTickUs * 1e-6f * 1.01f + 1e-4f;
    // Format
    {
        std::uint8_t d[V::kFrameLen]; courseBytes(d, true, 0, 90.0f, 360, 1800);
        V::Frame f; check(V::parseFrame(d, sizeof d, f) && f.course_mode && f.course_seq && near(f.dpsi, 90.0f, 1e-4f) && f.w == 0 && !f.kappa_mode,
                          "Kursziel: bit2, Feld w = Δψ 0,1°, Wechselbit bit4");
        d[0] = V::kFlagCourse | V::kFlagKappa; check(!V::parseFrame(d, sizeof d, f), "Krümmung + Kursziel -> verworfen");
        d[0] = V::kFlagCourseSeq; check(!V::parseFrame(d, sizeof d, f), "Wechselbit ohne Kursziel -> verworfen");
        d[0] = 8; check(!V::parseFrame(d, sizeof d, f), "bit3 (Strecke, noch nicht) -> verworfen");
        check((V::kCaps & 2) != 0, "Fähigkeitsbyte bit1 Kursziel");
    }
    struct Case { const char* name; float goal; int alpha, jw; };
    for (const Case& c : {Case{"kurs_90", 90, 360, 1800}, Case{"kurs_m180", -180, 360, 1800}, Case{"kurs_klein", 2, 360, 1800},
                          Case{"kurs_steil", 45, 1000, 5000}, Case{"kurs_ohne_ruck", 90, 360, 0}}) {
        std::uint8_t d[V::kFrameLen]; courseBytes(d, false, 0, c.goal, c.alpha, c.jw);
        const auto tr = runScenario(c.name, l, repeat(d, 0, 5000000), 5.0f);
        const Turn r = turned(tr, c.goal);
        std::printf("       %s: gedreht %.3f°, Überschwingen %.3f°, w bis %.1f °/s, Ende Rest %.3f°\n",
                    c.name, double(r.angle), double(r.over), double(r.wpk), double(tr.back().rest));
        char what[160];
        std::snprintf(what, sizeof what, "%s: Ziel auf ±0,5°, Überschwingen < 1°, angekommen, w ≤ w_max, Drehbeschleunigung ≤ alpha", c.name);
        check(std::fabs(r.angle - c.goal) < 0.5f && r.over < 1.0f && tr.back().done && r.wpk <= 100.0f + 1e-3f
              && r.dw <= std::max(dw_max, float(c.alpha) * kTickUs * 1e-6f * 1.01f + 1e-4f) && tr.back().w == 0, what);
    }
    {   // gleiche Rahmen (gleiches Wechselbit) drehen nicht nochmal, neues Wechselbit setzt ein neues Ziel ab dem Generatorwinkel
        std::uint8_t a[V::kFrameLen], b[V::kFrameLen]; courseBytes(a, false, 0, 90, 360, 1800); courseBytes(b, true, 0, 45, 360, 1800);
        auto ev = repeat(a, 0, 600000); auto e2 = repeat(b, 600000, 5000000); ev.insert(ev.end(), e2.begin(), e2.end());
        const auto tr = runScenario("kurs_neu", l, ev, 5.0f);
        const std::size_t k = 600000 / kTickUs;
        const Turn first = turned(std::vector<KStep>(tr.begin(), tr.begin() + k), 90);
        const Turn r = turned(tr, 0);
        check(std::fabs(r.angle - (first.angle + 45)) < 0.5f && tr.back().done, "neues Wechselbit mitten im Drehen: Ziel = Winkel bei Empfang + 45°");
        const auto tk = runScenario("kurs_halten", l, repeat(a, 0, 6000000), 6.0f);
        check(std::fabs(turned(tk, 90).angle - 90) < 0.5f, "Wiederholung mit gleichem Wechselbit: nur einmal 90°");
    }
    {   // Wächter: bremst die Drehung, Ziel bleibt offen
        std::uint8_t a[V::kFrameLen]; courseBytes(a, false, 0, 180, 360, 1800);
        const auto tr = runScenario("kurs_waechter", l, repeat(a, 0, 500000), 3.0f);
        check(tr.back().timed_out && tr.back().w == 0 && !tr.back().done && tr.back().rest > 10, "Wächter: Drehung gebremst, nicht angekommen");
    }
    {   // im Fahren: v bleibt, Drehung auf 90° (Bogen)
        std::uint8_t a[V::kFrameLen]; courseBytes(a, false, 46, 90, 360, 1800);
        const auto tr = runScenario("kurs_fahrt", l, repeat(a, 0, 4000000), 4.0f);
        check(tr.back().v == 46 && tr.back().done && std::fabs(turned(tr, 90).angle - 90) < 0.5f, "im Fahren: v 46 bleibt, 90° gedreht");
    }
    {   // aus dem Krümmungsbetrieb (w ≠ 0) ins Kursziel, dann zurück auf den Drehratenweg
        std::uint8_t k[V::kFrameLen], c[V::kFrameLen], r[V::kFrameLen];
        kappaBytes(k, true, 46, 128, 641, 2.0f, 360, 1800); courseBytes(c, false, 0, -30, 360, 1800);
        frameBytes(r, true, 0, 128, 641, 0, 360, 1800);
        auto ev = repeat(k, 0, 2000000); auto e2 = repeat(c, 2000000, 5000000); auto e3 = repeat(r, 5000000, 6000000);
        ev.insert(ev.end(), e2.begin(), e2.end()); ev.insert(ev.end(), e3.begin(), e3.end());
        const auto tr = runScenario("kurs_aus_kappa", l, ev, 6.0f);
        const std::size_t a = 2000000 / kTickUs, b = 5000000 / kTickUs;
        float dwk = 0; for (std::size_t i = a; i < b; ++i) dwk = std::max(dwk, std::fabs(tr[i].w - tr[i - 1].w));
        const Turn rr = turned(std::vector<KStep>(tr.begin() + a - 1, tr.begin() + b), -30);
        check(dwk <= dw_max && std::fabs(rr.angle + 30) < 0.5f && tr[b - 1].done, "aus der Kurve ins Kursziel: ohne Sprung, −30° ab Empfang");
        check(!tr.back().course_on && tr.back().w == 0, "Drehratenweg beendet das Kursziel");
    }
    {   // Diagnose
        V::State s; V::Frame f; f.course_mode = true; f.dpsi = 0.2f; V::receive(s, l, f, 0); V::tick(s, l, 0); V::tick(s, l, 4000);
        std::uint8_t g[V::kDiagLen]; V::fillDiag(g, s, l);
        check((g[0] & 32) && (g[0] & 64) && g[1] == V::kCaps, "0x1a lesen: bit5 Kursziel, bit6 erreicht (Rest 0,2° < 0,5°)");
    }
}

int main() {
    generatorTests();
    kappaTests();
    courseTests();
    // --- Format
    {
        std::uint8_t d[V::kFrameLen]; frameBytes(d, true, -350, 800, 4000, -455, 120, 900);
        V::Frame f;
        check(V::parseFrame(d, V::kFrameLen, f), "Rahmen 13 Byte angenommen");
        check(f.has_speed && f.v == -350 && f.a == 800 && f.jerk == 4000, "v, a, Ruck gelesen");
        check(near(f.w, -45.5f) && f.alpha == 120 && f.jerk_w == 900, "w (0,1 °/s), alpha, Drehruck gelesen");
        check(!V::parseFrame(d, 12, f), "Rahmen mit 12 Byte verworfen");
        frameBytes(d, false, 0, -300, 0, 0, 0, 0); V::parseFrame(d, V::kFrameLen, f);
        check(!f.has_speed && f.a == -300, "Beschleunigungsbetrieb: Flag und negatives a");
        V::Limits l = limits(500, 900, 250);
        check(l.set && l.v_max == 500 && near(l.w_max, 90.0f) && l.timeout_ms == 250, "Grenzen gelesen");
    }
    // --- ohne Grenzen kein Rahmen
    {
        V::State s; V::Limits none; V::Frame f; f.v = 100;
        check(!V::receive(s, none, f, 0) && !s.active, "ohne 0x1b wird 0x1a verworfen");
    }
    // --- Begrenzung, kein Totband
    {
        V::State s; V::Limits l = limits(400, 600, 0);
        V::Frame f; f.v = 900; f.w = -80;
        V::receive(s, l, f, 1000); V::tick(s, l, 1000);
        check(near(s.v, 400) && near(s.w, -60), "v auf v_max, w auf w_max begrenzt");
        f.v = 3; f.w = 0.5f; V::receive(s, l, f, 4000); V::tick(s, l, 4000);
        check(near(s.v, 3) && near(s.w, 0.5f), "kleine Sollwerte kommen ohne Totband an");
    }
    // --- Beschleunigungsbetrieb
    {
        V::State s; V::Limits l = limits(300, 600, 0);
        V::Frame f; f.has_speed = false; f.a = 200; f.w = 10;
        std::uint32_t t = 0;
        V::receive(s, l, f, t);
        for (int i = 0; i <= 333; ++i, t += 3000) V::tick(s, l, t);   // ~1 s
        check(near(s.v, 200, 2.0f) && s.accel_mode, "a = +200 mm/s² über 1 s -> ~200 mm/s");
        for (int i = 0; i < 333; ++i, t += 3000) V::tick(s, l, t);
        check(near(s.v, 300), "begrenzt durch v_max");
        f.a = -500; V::receive(s, l, f, t);
        for (int i = 0; i < 400; ++i, t += 3000) V::tick(s, l, t);   // 1,2 s
        check(near(s.v, -300), "negativ beschleunigt bis -v_max");
        check(near(s.w, 10), "Drehen auch im Beschleunigungsbetrieb über w");
        f.has_speed = true; f.v = 50; f.a = 0; V::receive(s, l, f, t); V::tick(s, l, t);
        check(!s.accel_mode && near(s.v, 50), "Rahmen mit v beendet den Beschleunigungsbetrieb");
    }
    // --- Wächter
    {
        V::State s; V::Limits l = limits(500, 600, 200);
        V::Frame f; f.v = 250; f.w = 30;
        V::receive(s, l, f, 0); V::tick(s, l, 150000);
        check(near(s.v, 250) && !s.timed_out, "innerhalb des Wächters gehalten");
        V::tick(s, l, 201000);
        check(s.timed_out && s.active && s.v < 250 && s.v > 200, "nach 200 ms ohne Rahmen: bremst (Rampe)");
        std::uint32_t t = 201000;
        for (int i = 0; i < 200; ++i) { t += 3000; V::tick(s, l, t); }
        check(s.v == 0 && s.w == 0, "Wächter: nach 0,6 s mit 500 mm/s² / 180 °/s² auf 0");
        V::receive(s, l, f, 210000); V::tick(s, l, 210000);
        check(!s.timed_out && near(s.v, 250), "neuer Rahmen hebt den Wächter auf");
        V::Limits off = limits(500, 600, 0);
        V::tick(s, off, 5000000);
        check(near(s.v, 250), "Wächter 0 = aus: Sollwert wird gehalten (wie die Bytes)");
    }
    // --- Abbruch
    {
        V::State s; V::Limits l = limits(500, 600, 0);
        V::Frame f; f.v = 100; V::receive(s, l, f, 0); V::tick(s, l, 0);
        V::cancel(s);
        check(!s.active && s.v == 0 && s.w == 0, "cancel beendet den Rahmenweg");
    }
    // --- Byte-Gegenwert (Positionshalten: Drehen außerhalb 113..143)
    {
        check(V::toByte(0, V::kByteTurn) == 128, "0 -> Byte 128");
        check(V::toByte(300, V::kByteTurn) == 255 && V::toByte(-300, V::kByteTurn) == 1, "±300 °/s -> 255/1");
        check(V::toByte(34.0f, V::kByteTurn) <= 143 && V::toByte(37.0f, V::kByteTurn) > 143, "Grenze 113..143 bei ~35 °/s wie das Byte");
        check(V::toByte(1000, V::kByteForward) == 255, "über dem Bereich gesättigt");
    }
    // --- Diagnose 0x1a
    {
        V::State s; V::Limits l = limits(450, 725, 300);
        V::Frame f; f.v = -120; f.w = 12.3f; V::receive(s, l, f, 0); V::tick(s, l, 0);
        std::uint8_t d[V::kDiagLen]; V::fillDiag(d, s, l);
        check(d[0] == (1 | 8), "Flags: aktiv, Grenzen gesetzt");
        check(d[1] == V::kCaps, "Byte 1: Fähigkeiten");
        check(V::i16(d + 2) == -120 && V::i16(d + 4) == 123, "v und w x10 im Diagnose-Register");
        check(V::u16(d + 6) == 450 && V::u16(d + 8) == 725 && V::u16(d + 10) == 300, "Grenzen im Diagnose-Register");
    }
    // --- Protokoll
    {
        CommandEvent e; e.type = 0; e.reg = 0x1a; e.length = 13;
        check(validCommand(e) && commandKind(e) == CommandKind::Velocity, "0x1a mit 13 Byte gültig");
        e.length = 1; check(!validCommand(e), "0x1a mit 1 Byte ungültig");
        e.reg = 0x1b; e.length = 14; check(validCommand(e) && commandKind(e) == CommandKind::VelocityLimits, "0x1b mit 14 Byte gültig");
        e.length = 6; check(!validCommand(e), "0x1b mit 6 Byte ungültig (altes Format)");
        e.length = 16; check(!validCommand(e), "0x1b mit 16 Byte ungültig");
        e.length = 14;
        e.length = 6;
        e.reg = 0xa0; e.length = 1; check(!validCommand(e), "0xa0 (max_forward) entfernt");
        e.reg = 0xa1; check(!validCommand(e), "0xa1 (max_turn) entfernt");
        e.reg = 0x30; check(validCommand(e), "0x30 (Fahrbyte) bleibt");
        CommandEvent r; r.type = 2; r.reg = 0x1a; r.length = 1; r.data[0] = 12;
        Telemetry t; for (int i = 0; i < 12; ++i) t.veldiag[i] = std::uint8_t(i + 1);
        std::uint8_t out[64]; const std::size_t n = buildReadResponse(r, t, out, sizeof out);
        check(n == 12 + 8 && out[4] == 0x1a && out[5] == 1 && out[16] == 12, "0x1a lesen liefert 12 Byte Diagnose");
    }
    // --- Regelkette: Rahmenweg in updateCommandTargets, alter Weg unverändert
    {
        using namespace rider::control;
        CommandState a, b; MotionCommand c;
        c.forward = 133; c.turn = 125;           // im Byte-Totband (±5 %: 9,1 mm/s < 10,5; -5,9 °/s > -15)
        updateCommandTargets(a, c);
        check(a.forward == 0 && a.turn == 0, "alter Weg: Byte im Totband -> 0");
        c.velocity = true; c.velocity_forward = 8; c.velocity_turn = -4;
        updateCommandTargets(b, c);
        check(b.forward == 8 && b.turn == -4, "Rahmenweg: 8 mm/s / -4 °/s ohne Totband");
        check(b.filtered_forward == 8 && b.filtered_turn == -4, "Rahmenweg: kein Tiefpass, filtered = Sollwert");
        CommandState x, y; MotionCommand old; old.forward = 200; old.turn = 60;
        MotionCommand same = old; same.velocity_forward = 999; same.velocity_turn = 999;   // velocity = false
        for (int i = 0; i < 50; ++i) { updateCommandTargets(x, old); updateCommandTargets(y, same); }
        check(x.filtered_forward == y.filtered_forward && x.filtered_turn == y.filtered_turn && x.forward == y.forward,
              "velocity = false: bitgleich zum alten Weg");
    }
    {   // Einheit; Bytes 8..13 (ki, i_max, kv von xgonav) ohne Bedeutung
        const V::Limits z = limits(231, 1800, 500);
        check(z.unit_mm == 1.0f, "0x1b mit 0: Einheit 1");
        std::uint8_t d[V::kLimitsLen];
        V::put16(d, 231); V::put16(d + 2, 1800); V::put16(d + 4, 500); V::put16(d + 6, 3900);
        V::put16(d + 8, 300); V::put16(d + 10, 600); V::put16(d + 12, 250);
        V::Limits l; check(V::parseLimits(d, sizeof d, l), "0x1b 14 Byte angenommen");
        check(l.v_max == 231 && near(l.w_max, 180.0f, 1e-4f) && l.timeout_ms == 500, "0x1b: Grenzen");
        check(near(l.unit_mm, 3.9f, 1e-5f), "0x1b: Einheit 3,9 mm/s");
        V::Limits l2; V::put16(d + 8, 60000); V::put16(d + 10, 0); V::put16(d + 12, 60000); V::parseLimits(d, sizeof d, l2);
        check(l2.v_max == l.v_max && l2.w_max == l.w_max && l2.unit_mm == l.unit_mm, "Bytes 8..13 ändern nichts");
        check(!V::parseLimits(d, 6, l), "0x1b mit 6 Byte verworfen");
    }
    std::printf(fail ? "\nFEHLER\n" : "\nalle ok\n");
    return fail;
}
