// Host-Test für Patch 0029: Fahrrahmen in mm/s und °/s (Register 0x1a/0x1b, velocity_cmd.h), 0xa0/0xa1 entfernt.
// Prüft: Rahmen- und Grenzen-Format, Ablehnung ohne Grenzen, Begrenzung auf v_max/w_max, Beschleunigungsbetrieb
// (Vorzeichen, Grenze), Wächter, Abbruch, Byte-Gegenwert, Protokoll (Längen, Lesen 0x1a, 0xa0/0xa1 unbekannt) und
// dass der Rahmenweg in updateCommandTargets ohne Byte-Totband ankommt, der alte Weg aber unverändert bleibt.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_velocity_cmd.cpp $S/control/rider_motion.cpp $S/control/pid.cpp $S/command_protocol.cpp -o t && ./t
#include "velocity_cmd.h"
#include "command_protocol.h"
#include "control/rider_motion.h"
#include <cmath>
#include <cstdio>
#include <cstring>

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
    std::uint8_t d[V::kLimitsLen];
    V::put16(d, std::uint16_t(vmax)); V::put16(d + 2, std::uint16_t(wmax10)); V::put16(d + 4, std::uint16_t(timeout));
    V::Limits l; V::parseLimits(d, sizeof d, l); return l;
}

int main() {
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
        check(l.set && l.v_max == 500 && near(l.w_max, 90.0f) && l.timeout_ms == 250, "Grenzen 6 Byte gelesen");
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
        f.has_speed = true; f.v = 50; V::receive(s, l, f, t); V::tick(s, l, t);
        check(!s.accel_mode && near(s.v, 50), "Rahmen mit v beendet den Beschleunigungsbetrieb");
    }
    // --- Wächter
    {
        V::State s; V::Limits l = limits(500, 600, 200);
        V::Frame f; f.v = 250; f.w = 30;
        V::receive(s, l, f, 0); V::tick(s, l, 150000);
        check(near(s.v, 250) && !s.timed_out, "innerhalb des Wächters gehalten");
        V::tick(s, l, 201000);
        check(s.timed_out && s.v == 0 && s.w == 0 && s.active, "nach 200 ms ohne Rahmen: v = w = 0");
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
        check(V::i16(d + 2) == -120 && V::i16(d + 4) == 123, "v und w x10 im Diagnose-Register");
        check(V::u16(d + 6) == 450 && V::u16(d + 8) == 725 && V::u16(d + 10) == 300, "Grenzen im Diagnose-Register");
    }
    // --- Protokoll
    {
        CommandEvent e; e.type = 0; e.reg = 0x1a; e.length = 13;
        check(validCommand(e) && commandKind(e) == CommandKind::Velocity, "0x1a mit 13 Byte gültig");
        e.length = 1; check(!validCommand(e), "0x1a mit 1 Byte ungültig");
        e.reg = 0x1b; e.length = 6; check(validCommand(e) && commandKind(e) == CommandKind::VelocityLimits, "0x1b mit 6 Byte gültig");
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
        CommandState x, y; MotionCommand old; old.forward = 200; old.turn = 60;
        MotionCommand same = old; same.velocity_forward = 999; same.velocity_turn = 999;   // velocity = false
        for (int i = 0; i < 50; ++i) { updateCommandTargets(x, old); updateCommandTargets(y, same); }
        check(x.filtered_forward == y.filtered_forward && x.filtered_turn == y.filtered_turn && x.forward == y.forward,
              "velocity = false: bitgleich zum alten Weg");
    }
    std::printf(fail ? "\nFEHLER\n" : "\nalle ok\n");
    return fail;
}
