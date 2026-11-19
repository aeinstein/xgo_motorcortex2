// Host-Test für Patch 0022 und 0023 (LEDs):
// 0022: Solange der Kreisel beim Start kalibriert, läuft ein heller Punkt in der Nutzerfarbe über die
//       vier LEDs im Kreis (0 -> 1 -> 2 -> 3, Umlauf 0,4 s, weich überblendet). Akku-Warnung, Modus 0
//       und Fehleranzeige haben Vorrang.
// 0023: Keine Farbverläufe der Roll-/Höhenwelle mehr; ohne eigenes Bild liefert renderLeds false
//       (updateLeds zeigt dann die Nutzerfarben).
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_led_calibrating.cpp $S/board_support.cpp -o t && ./t
#include "board_support.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace rider::board;

static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}

int main() {
    const Rgb pink = paletteColor(kUserLedPalette);   // Startfarbe aus begin()
    LedPixels user;
    user.fill(pink);
    const std::uint32_t T = kCalibrationSpinPeriodMs;

    // Lauflicht: Punkt steht bei t = i·T/4 genau auf LED i, die anderen glimmen
    bool spot = true;
    for (unsigned i = 0; i < 4; ++i) {
        const std::uint32_t t = i * T / 4;
        for (unsigned k = 0; k < 4; ++k)
            spot = spot && calibrationSpin(t, k) == (k == i ? 255 : kCalibrationSpinMin);
    }
    check(spot, "Punkt wandert 0 -> 1 -> 2 -> 3, je Viertelumlauf eine LED");
    check(calibrationSpin(T / 8, 0) == calibrationSpin(T / 8, 1) && calibrationSpin(T / 8, 0) > kCalibrationSpinMin
          && calibrationSpin(T / 8, 0) < 255, "auf halbem Weg leuchten beide Nachbarn gleich (überblendet)");
    check(calibrationSpin(T / 8, 2) == kCalibrationSpinMin && calibrationSpin(T / 8, 3) == kCalibrationSpinMin,
          "die gegenüberliegenden LEDs glimmen nur");
    int lo = 255, hi = 0, step = 0;
    bool periodic = true, one_max = true;
    for (std::uint32_t t = 0; t < 2 * T; t += 20) {
        int best = 0, bright = 0;
        for (unsigned k = 0; k < 4; ++k) {
            const int w = calibrationSpin(t, k);
            lo = std::min(lo, w); hi = std::max(hi, w);
            if (t) step = std::max(step, std::abs(w - int(calibrationSpin(t - 20, k))));
            periodic = periodic && w == calibrationSpin(t + T, k);
            best = std::max(best, w);
            if (w > 200) ++bright;
        }
        one_max = one_max && bright <= 1 && best >= 140;   // nie zwei volle, nie alles dunkel
    }
    std::printf("Faktor %d..%d, größter Schritt je 20 ms: %d\n", lo, hi, step);
    check(lo == kCalibrationSpinMin && hi == 255, "Faktor zwischen Minimum und voll");
    check(periodic, "Umlauf = kCalibrationSpinPeriodMs");
    check(one_max, "immer höchstens eine LED hell, nie alle dunkel");
    check(T == 400, "Umlauf 400 ms");
    check(step <= 75, "weich überblendet (≤ 75/256 je Bild)");
    check(calibrationSpin(123, 5) == calibrationSpin(123, 1), "LED-Index wird auf 0..3 begrenzt");

    // Kalibrierend im Startmodus 253: Nutzerfarbe mal Faktor, Farbton bleibt
    LedControl c;
    c.robot_mode = 253;
    c.gyro_calibrating = true;
    LedPixels out{};
    user[2] = Rgb{0, 200, 40};            // eine LED vom Client anders gesetzt
    check(renderLeds(c, 2 * T / 4, user, out), "kalibrierend: es wird gezeichnet");
    check(std::abs(int(out[2].g) - 200) <= 1 && std::abs(int(out[2].b) - 40) <= 1 && out[2].r == 0, "Punkt auf LED 2: volle Nutzerfarbe");
    check(out[0].r > 0 && out[0].r < pink.r / 4 && out[0].g == 0, "übrige LEDs: dunkel, Farbton bleibt, nie ganz aus");

    // Kalibriert: kein eigenes Bild
    c.gyro_calibrating = false;
    check(!renderLeds(c, 100, user, out), "kalibriert, Modus 253: renderLeds zeichnet nichts (Nutzerfarben)");

    // Vorrang: Akku schwach (250), Modus 0, Fehleranzeige
    c.gyro_calibrating = true;
    c.robot_mode = 250;
    c.low_blink = true;
    renderLeds(c, 0, user, out);
    check(out[0].r == 200 && out[0].b == 0, "Akku-Warnung geht vor");
    c.robot_mode = 0;
    renderLeds(c, 0, user, out);
    check(out[0].r == 200 && out[0].b == 0, "Modus 0 (rot) geht vor");
    c.robot_mode = 253;
    c.fault = LedFault::Tilt;
    check(renderFaultLeds(c, 0, out) && out[0].r == 200 && out[0].b == 0, "Fehleranzeige geht vor");

    // Fusions-Neustart beim Balancieren (Modus 1): Lauflicht statt Nutzerfarbe fest
    c.fault = LedFault::None;
    c.robot_mode = 1;
    renderLeds(c, 0, user, out);
    check(out[1].r < pink.r / 4 && std::abs(int(out[0].r) - int(pink.r)) <= 1, "Modus 1, nicht kalibriert: Lauflicht");

    // 0023: Balancieren zeigt die Nutzerfarben, beim Fahren Grün, beim Nick-Trimm Blau; keine Wellenfarben
    c.gyro_calibrating = false;
    check(renderLeds(c, 0, user, out) && out == user, "Modus 1 im Stand: Nutzerfarben");
    c.forward_request = 50;
    renderLeds(c, 0, user, out);
    check(out[0].g == 240 && out[3].g == 240 && out[1].g == 0 && out[0].r == 0, "Modus 1 vorwärts: Grün vorn");
    c.forward_request = 0;
    c.pitch_trim_stage = 2;
    renderLeds(c, 100, user, out);
    check(out[0].b == 128 && out[0].r == 0, "Nick-Trimm: blau");
    check(Rgb{1, 2, 3} == Rgb{1, 2, 3} && Rgb{1, 2, 3} != Rgb{1, 2, 4}, "Rgb-Vergleich");

    std::printf(fail ? "FEHLER\n" : "alles ok\n");
    return fail;
}
