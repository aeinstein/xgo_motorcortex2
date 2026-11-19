// Host-Test für Patch 0024: gespeicherte Aktionen als Tabelle (action_engine.cpp, kActions).
// Prüft die Aktionen 1..6 an ihren Schaltpunkten (Werte wie vor dem Umbau; xgonav/robot.py FW_ACTIONS
// spiegelt 3..5), die Schleife (Register 0x03), unbekannte Nummern und den Abbruch.
// Mit -DTRACE gibt das Programm stattdessen den Zustand je Tick aus: so wurde die Tabelle am 2026-11-19
// gegen die alte switch-Fassung verglichen (gleiche Ausgabe; die TRACE-Fassung nutzt nur die alte API).
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_action_table.cpp $S/action_engine.cpp -o t && ./t
#include "action_engine.h"
#include <cstdint>
#include <cstdio>

using rider2::ActionEngine;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

static void dump(const char* tag, int k, ActionEngine& e) {
    const auto& s = e.state();
    const auto ev = e.takeEvents();
    std::printf("%s %3d id=%u f=%u t=%u h=%u r=%u rp=%u hp=%u rph=%g hph=%g hh=%d lv=%d hp=%d mode=%u loop=%u seq=%u wait=%u c=%u def=%d ev=%03x/%u\n",
                tag, k, s.action_id, s.command.forward, s.command.turn, s.command.height, s.command.roll,
                s.command.roll_period, s.command.height_period, double(s.roll_phase), double(s.height_phase),
                int(s.heading_hold), int(s.leveling), int(s.hold_protection), s.mode, s.loop, s.sequence_index,
                s.loop_wait_ticks, s.action_id ? e.counter(s.action_id) : 0, int(e.deferred()), unsigned(ev.flags), ev.action_id);
}

// Vollständige Zustandsfolge: jede Aktion einzeln, unbekannte Nummern, Abbruch, Wiederholung, Schleife.
static void trace() {
    std::uint32_t now = 1000;
    for (int id = 0; id <= 12; ++id) {
        ActionEngine e;
        e.state().mode = 1;
        e.state().command.roll_period = 77;
        e.state().command.height_period = 99;
        char tag[16];
        std::snprintf(tag, sizeof tag, "a%d", id);
        std::printf("%s req=%d\n", tag, int(e.request(std::uint8_t(id), now)));
        dump(tag, 0, e);
        for (int k = 1; k <= 130; ++k) {
            now += 70;
            if (k % 3 == 0) e.poll(now - 30);
            e.tick(now);
            dump(tag, k, e);
        }
    }
    {   // Abbruch mitten in Aktion 5, erneute Anforderung während einer laufenden Aktion, Wechsel der Aktion
        ActionEngine e;
        e.request(5, now);
        for (int k = 1; k <= 60; ++k) { now += 70; e.tick(now); if (k == 30) e.cancel(now); if (k == 40) e.request(4, now); if (k == 45) e.request(4, now); if (k == 50) e.request(3, now); dump("x", k, e); }
        e.request(200, now); dump("x", 61, e);
        e.request(255, now); dump("x", 62, e);
    }
    {   // Schleife über alle sechs
        ActionEngine e;
        e.setLoop(1, now);
        for (int k = 1; k <= 700; ++k) { now += 70; if (k % 2) e.poll(now - 10); e.tick(now); dump("l", k, e); }
        e.setLoop(0, now); dump("l", 701, e);
    }
}

#ifndef TRACE
// Aktion von vorn bis Tick `n` laufen lassen (Anforderung = Tick 0).
static ActionEngine run(int id, int n, std::uint32_t* now_out = nullptr) {
    ActionEngine e;
    std::uint32_t now = 1000;
    e.state().mode = 1;
    e.request(std::uint8_t(id), now);
    for (int k = 1; k <= n; ++k) { now += 70; e.tick(now); }
    if (now_out) *now_out = now;
    return e;
}
static bool neutral(const ActionEngine& e) {
    const auto& s = e.state();
    return s.action_id == 0 && s.command.forward == 128 && s.command.turn == 128 && s.command.roll == 128
           && s.command.height == 144 && s.roll_phase == 0 && s.height_phase == 0;
}
#endif

int main() {
#ifdef TRACE
    trace();
    return 0;
#else
    (void)trace;
    // Tabelle
    for (int id = 1; id <= 6; ++id) check(rider2::findAction(std::uint8_t(id)) != nullptr, "Aktion 1..6 in der Tabelle");
    check(!rider2::findAction(7) && !rider2::findAction(0) && !rider2::findAction(255), "7, 0, 255 nicht in der Tabelle");
    check(ActionEngine::supported(3) && ActionEngine::supported(255) && ActionEngine::supported(0) && !ActionEngine::supported(7)
          && !ActionEngine::supported(254), "supported(): Tabelle, 0 und 255");

    {   // 1: Rollwelle bei Höhe 112, Ende Tick 50
        auto e = run(1, 0);
        check(e.state().command.height == 112 && e.state().roll_phase == 10 && e.state().height_phase == 0, "1: Start Höhe 112, Rollwelle");
        check(e.takeEvents().has(rider2::ActionStarted), "1: Ereignis ActionStarted");
        e = run(1, 49); check(e.state().action_id == 1 && e.state().roll_phase == 10, "1: Tick 49 läuft noch");
        e = run(1, 50); check(neutral(e), "1: Tick 50 fertig, Befehle neutral");
        check(e.takeEvents().has(rider2::ActionFinished), "1: Ereignis ActionFinished");
    }
    {   // 2: Höhenwelle, Ende 40
        auto e = run(2, 0); check(e.state().height_phase == 10 && e.state().roll_phase == 0 && e.state().command.height == 144, "2: Start Höhenwelle");
        e = run(2, 39); check(e.state().action_id == 2, "2: Tick 39 läuft noch");
        e = run(2, 40); check(neutral(e), "2: Tick 40 fertig");
    }
    {   // 3: vor 192, ab 8 zurück 64, Ende 20
        auto e = run(3, 0); check(e.state().command.forward == 192 && e.state().command.turn == 128, "3: Start vor 192");
        e = run(3, 7); check(e.state().command.forward == 192, "3: Tick 7 noch vor");
        e = run(3, 8); check(e.state().command.forward == 64, "3: Tick 8 zurück 64");
        e = run(3, 19); check(e.state().command.forward == 64 && e.state().action_id == 3, "3: Tick 19 läuft noch");
        e = run(3, 20); check(neutral(e), "3: Tick 20 fertig");
    }
    {   // 4: Zickzack
        auto e = run(4, 0); check(e.state().command.forward == 176 && e.state().command.turn == 208, "4: Start 176/208");
        e = run(4, 14); check(e.state().command.forward == 176 && e.state().command.turn == 48, "4: Tick 14 176/48");
        e = run(4, 28); check(e.state().command.forward == 96 && e.state().command.turn == 208, "4: Tick 28 96/208");
        e = run(4, 44); check(e.state().command.forward == 96 && e.state().command.turn == 48, "4: Tick 44 96/48");
        e = run(4, 59); check(e.state().action_id == 4, "4: Tick 59 läuft noch");
        e = run(4, 60); check(neutral(e), "4: Tick 60 fertig");
    }
    {   // 5: Drehen 32, ab 46 208, Höhenwelle, Kurshalten aus, Ende 100, 50 ms später Kurshalten zurück
        auto e = run(5, 0);
        check(e.state().command.turn == 32 && e.state().height_phase == 10 && !e.state().heading_hold, "5: Start Drehen 32, Höhenwelle, Kurshalten aus");
        e = run(5, 46); check(e.state().command.turn == 208 && !e.state().heading_hold, "5: Tick 46 Drehen 208");
        std::uint32_t now = 0;
        e = run(5, 100, &now);
        check(neutral(e) && e.deferred() && !e.state().heading_hold, "5: Tick 100 fertig, Rückstellen des Kurshaltens steht aus");
        e.poll(now + 49); check(e.deferred(), "5: nach 49 ms noch ausstehend");
        e.poll(now + 50); check(!e.deferred(), "5: nach 50 ms zurückgestellt");
        check(e.takeEvents().has(rider2::DeferredRestore), "5: Ereignis DeferredRestore");
    }
    {   // 6: beide Wellen, Perioden 128, Ende 80
        ActionEngine e;
        e.state().command.roll_period = 77; e.state().command.height_period = 99;
        e.request(6, 1000);
        check(e.state().roll_phase == 10 && e.state().height_phase == 750 && e.state().command.roll_period == 128
              && e.state().command.height_period == 128, "6: Start beide Wellen, Perioden 128");
        auto f = run(6, 79); check(f.state().action_id == 6, "6: Tick 79 läuft noch");
        f = run(6, 80); check(neutral(f), "6: Tick 80 fertig");
    }
    {   // unbekannt, Abbruch
        ActionEngine e;
        check(!e.request(7, 1000) && e.state().action_id == 0 && e.takeEvents().has(rider2::ActionUnknown), "7: unbekannt, nichts läuft");
        check(!e.request(236, 1000) && e.state().action_id == 0, "236: unbekannt");
        e.request(4, 1000); e.tick(1070); e.cancel(1100);
        check(neutral(e) && e.takeEvents().has(rider2::ActionCancelled), "Abbruch: Befehle neutral");
    }
    {   // Schleife: 1, dann nach 30 Ticks Pause 2 …
        ActionEngine e;
        std::uint32_t now = 1000;
        e.setLoop(1, now);
        int seen[7] = {0}, order[8] = {0}, n = 0, last = 0;
        for (int k = 0; k < 1000 && n < 8; ++k) {
            now += 70; e.poll(now - 10); e.tick(now);
            const int id = e.state().action_id;
            if (id && id != last) { order[n++] = id; ++seen[id]; }
            last = id;
        }
        check(n == 8 && order[0] == 1 && order[1] == 2 && order[2] == 3 && order[3] == 4 && order[4] == 5 && order[5] == 6
              && order[6] == 1 && order[7] == 2, "Schleife: 1 2 3 4 5 6 1 2");
    }
    std::printf(fail ? "FEHLER\n" : "alles ok\n");
    return fail;
#endif
}
