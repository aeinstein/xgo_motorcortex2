// Host-Test 05-actions: Akku-Warnung hebt sich wieder auf.
// Ab <= 7 % zählt BatteryMonitor die Warnstufe hoch (ab 10 Zustand 250, keine Befehle mehr). Im Original
// blieb sie bis zum Stromlos-Machen stehen; hier ist sie weg, wenn der Akku 5 s lang ununterbrochen
// >= 15 % liegt. Kürzer oder darunter bleibt sie.
// Aufruf: firmware/tests/run.sh test_battery_recover
#include "board_support.h"
#include <cstdint>
#include <cstdio>

using namespace rider::board;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

// kleinster ADC-Wert, der mindestens p Prozent ergibt
static std::uint16_t adcFor(float p) {
    for (unsigned a = 0; a < 4096; ++a)
        if (batteryPercent(batteryVoltage(std::uint16_t(a))) >= p) return std::uint16_t(a);
    return 4095;
}

// n Abtastungen im Abstand von 3 ms (Schleife ~333 Hz)
static void feed(BatteryMonitor& m, std::uint16_t adc, std::uint32_t& t, std::uint32_t ms) {
    for (std::uint32_t end = t + ms; t < end; t += 3) m.update(adc, t);
}

// langsam von a nach b (das Original verwirft Sprünge > 0,1 V je 15er-Block nach 5 s Laufzeit)
static std::uint16_t cur = 0;
static void ramp(BatteryMonitor& m, std::uint16_t b, std::uint32_t& t, std::uint32_t ms) {
    const int a = cur;
    for (std::uint32_t t0 = t, end = t + ms; t < end; t += 3)
        m.update(std::uint16_t(a + (int(b) - a) * int(t - t0) / int(ms)), t);
    cur = b;
}

int main() {
    const std::uint16_t leer = adcFor(0.0f), voll = adcFor(90.0f), knapp = adcFor(12.0f);
    check(batteryPercent(batteryVoltage(leer)) <= 7.0f, "leerer Akku <= 7 %");

    BatteryMonitor m;
    std::uint32_t t = 0;
    feed(m, voll, t, 6000); cur = voll;
    check(m.state().low_stage == 0 && m.state().percent > 80, "voll: keine Warnung");

    ramp(m, leer, t, 30000); feed(m, leer, t, 8000);   // Original: 200 Abtastungen, dann alle 80 eine Stufe
    check(m.state().stop_required, "leer: Zustand 250 verlangt (Stufe >= 10)");

    ramp(m, knapp, t, 10000); feed(m, knapp, t, 20000);
    check(m.state().low_stage > 0, "12 %: Warnung bleibt (Hysterese)");

    ramp(m, voll, t, 2000);                             // 15 % kurz nach Rampenbeginn überschritten
    check(m.state().low_stage > 0, "erst ~2 s über 15 %: Warnung bleibt");
    ramp(m, knapp, t, 2000); feed(m, knapp, t, 1000);   // wieder darunter: Zeit beginnt neu
    ramp(m, voll, t, 2000); feed(m, voll, t, 1500);
    check(m.state().low_stage > 0, "nach Rückfall erst ~3 s darüber: Warnung bleibt");

    feed(m, voll, t, 3000);
    const auto& s = m.state();
    check(s.low_stage == 0 && !s.stop_required && !s.neutralize && !s.release_legs && !s.low_blink,
          "> 5 s über 15 %: Warnung aufgehoben");

    ramp(m, leer, t, 30000); feed(m, leer, t, 8000);
    check(m.state().stop_required, "wieder leer: Warnung kommt wieder");

    std::printf(fail ? "\nFEHLER\n" : "\nalle ok\n");
    return fail;
}
