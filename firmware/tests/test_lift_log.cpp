// Host-Test für Patch 0027: Protokoll des Hebeschutzes (lift_log.h) und kein Selbststart aus Zustand 0.
// Prüft: Auslösung samt Messwerten festhalten, Dauer und betragsgrößtes Rohfeld, Ende frei gegeben bzw. durch
// Stopp, dass ein Stopp nichts löscht (anders als lift::State), den letzten Stopp aus dem Balancieren; dazu
// ActionEngine in Zustand 0: mit auto_recover = false bleibt er auch nach langem Aufrechtstehen liegen, mit true
// (Original R-1.1.6) geht er nach > 500 Takten auf 1.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_lift_log.cpp $S/action_engine.cpp -o t && ./t
#include "lift_log.h"
#include "lift_guard.h"
#include "action_engine.h"
#include <cstdint>
#include <cstdio>

namespace lift = rider2::lift;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

static lift::LogSample sample(std::uint32_t ms, float speed, std::int16_t raw11, std::int16_t raw21) {
    lift::LogSample s;
    s.ms = ms; s.speed11 = speed; s.speed21 = speed; s.raw11 = raw11; s.raw21 = raw21;
    s.forward = 45.0f; s.pitch = -0.5f;
    return s;
}

static void testLog() {
    lift::Log l;
    lift::logTick(l, false, sample(1000, 80, 100, -100));
    check(l.trips == 0 && !l.active, "ohne Auslösung nichts protokolliert");

    // Auslösung: Werte des ersten Takts, Dauer zählt, Rohspitze betragsgrößter Wert beider Räder
    lift::logTick(l, true, sample(1005, 320, 395, -390));
    lift::logTick(l, true, sample(1008, 330, 400, -520));
    lift::logTick(l, true, sample(1011, 310, 380, -380));
    check(l.trips == 1 && l.active && l.end == lift::LogEnd::Active, "Auslösung gezählt, greift noch");
    check(l.trip.ms == 1005 && l.trip.speed11 == 320 && l.trip.raw11 == 395 && l.trip.raw21 == -390,
          "Werte im Takt der Auslösung");
    check(l.trip.forward == 45.0f && l.trip.pitch == -0.5f, "Fahrbefehl und Nick der Auslösung");
    check(l.ticks == 3, "Dauer 3 Takte");
    check(l.raw_peak == -520, "Rohspitze -520 (betragsgrößter Wert, Vorzeichen bleibt)");

    // Stopp während er greift (Kippstopp): Ende = Stopp mit Grund, Werte bleiben, letzter Stopp beim Fahren
    lift::logStop(l, true, 7 /* Tilt */, 1100);
    check(!l.active && l.end == lift::LogEnd::Stopped && l.end_reason == 7, "Ende durch Kippstopp");
    check(l.trips == 1 && l.trip.ms == 1005 && l.ticks == 3 && l.raw_peak == -520, "Stopp löscht nichts");
    check(l.stops == 1 && l.stop_reason == 7 && l.stop_ms == 1100, "letzter Stopp aus dem Balancieren");

    // stop() läuft im Stillstand jeden Takt mit Startup: zählt nicht als Stopp aus dem Balancieren
    lift::logStop(l, false, 1 /* Startup */, 1110);
    check(l.stops == 1 && l.stop_reason == 7 && l.stop_ms == 1100, "Stopp im Stillstand überschreibt nichts");

    // zweite Auslösung, frei gegeben
    lift::logTick(l, true, sample(2000, 305, 380, -370));
    lift::logTick(l, false, sample(2003, 20, 10, -10));
    check(l.trips == 2 && l.end == lift::LogEnd::Released && l.ticks == 1 && l.trip.ms == 2000,
          "zweite Auslösung frei gegeben, Werte neu");
    lift::logStop(l, true, 9 /* Timing */, 2500);
    check(l.end == lift::LogEnd::Released && l.stops == 2 && l.stop_reason == 9,
          "Stopp nach der Freigabe ändert das Ende nicht");
}

// Detektor und Protokoll zusammen wie in main.cpp: der Detektor wird beim Stopp zurückgesetzt, das Protokoll nicht
static void testWithDetector() {
    lift::Config c;
    lift::State st;
    lift::Log l;
    lift::Input in;
    in.speed11 = in.speed21 = 350;
    for (int k = 0; k < 10; ++k) {
        st = lift::update(st, c, in);
        lift::logTick(l, st.active, sample(3000 + 3 * k, in.speed11, 430, -430));
    }
    check(st.active && l.trips == 1 && l.ticks == 10 - c.fast_ticks + 1, "Detektor löst nach fast_ticks aus");
    lift::logStop(l, true, 7, 3100);
    st = lift::State();   // wie stop() in main.cpp
    check(st.trips == 0 && l.trips == 1 && l.end == lift::LogEnd::Stopped, "nach dem Stopp: Detektor leer, Protokoll da");
}

static void testNoAutoRecover() {
    using rider2::ActionEngine;
    for (int recover = 0; recover < 2; ++recover) {
        ActionEngine e;
        e.state().auto_recover = recover != 0;
        e.requestMode(0, true);   // gefallen
        std::uint32_t t = 0;
        bool armed = false;
        for (int k = 0; k < 1000; ++k) { t += 3; armed = e.controlTick(t, 1.0f, 0.5f) || armed; }
        if (recover) check(armed && e.state().mode == 1, "Original: aufrecht nach > 500 Takten von selbst Zustand 1");
        else check(!armed && e.state().mode == 0, "0027: 1000 aufrechte Takte, bleibt in Zustand 0");
    }
    ActionEngine e;
    e.state().auto_recover = false;
    e.requestMode(0, true);
    e.requestMode(253, true);   // arm() in main.cpp: 255 bzw. 253 setzen 253, die Wartelogik dort startet dann
    check(e.state().mode == 253, "Befehl bewaffnet aus Zustand 0");
}

int main() {
    testLog();
    testWithDetector();
    testNoAutoRecover();
    std::printf(fail ? "FEHLER\n" : "alles ok\n");
    return fail;
}
