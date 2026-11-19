// Host-Test für Patch 0022: Solange der Kreisel beim Start kalibriert, wabern die LEDs in der
// Nutzerfarbe (weiche Welle, 1,2 s); danach zeichnet renderLeds im Startmodus 253 nichts mehr
// (updateLeds zeigt dann einmal die Nutzerfarbe fest). Akku-Warnung und Fehleranzeige haben Vorrang.
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
    const Rgb pink = paletteColor(234);   // Startfarbe aus begin()
    LedPixels user;
    user.fill(pink);
    user[2] = Rgb{0, 200, 40};            // eine LED vom Pi anders gesetzt

    // Welle: Grenzen, Periode, stetig im 20-ms-Takt der LEDs
    int lo = 255, hi = 0, step = 0;
    for (std::uint32_t t = 0; t <= 2 * kCalibrationWavePeriodMs; t += 20) {
        const int w = calibrationWave(t);
        if (w < lo) lo = w;
        if (w > hi) hi = w;
        if (t) step = std::max(step, std::abs(w - int(calibrationWave(t - 20))));
    }
    std::printf("Welle %d..%d, größter Schritt je 20 ms: %d\n", lo, hi, step);
    check(lo == kCalibrationWaveMin && hi == 255, "Welle zwischen Minimum und voll");
    check(calibrationWave(0) == kCalibrationWaveMin, "Welle beginnt dunkel");
    check(calibrationWave(kCalibrationWavePeriodMs / 2) == 255, "voll nach halber Periode");
    check(calibrationWave(317) == calibrationWave(317 + kCalibrationWavePeriodMs), "Periode 1200 ms");
    check(step <= 14, "keine Sprünge (≤ 14/256 je Bild)");

    // Kalibrierend im Startmodus 253: Nutzerfarbe mal Welle, Farbton bleibt
    LedControl c;
    c.robot_mode = 253;
    c.gyro_calibrating = true;
    LedPixels out{};
    check(renderLeds(c, kCalibrationWavePeriodMs / 2, user, out), "kalibrierend: es wird gezeichnet");
    bool full = true;
    for (unsigned i = 0; i < 4; ++i)
        full = full && std::abs(int(out[i].r) - int(user[i].r)) <= 1 && std::abs(int(out[i].g) - int(user[i].g)) <= 1
               && std::abs(int(out[i].b) - int(user[i].b)) <= 1;
    check(full, "Wellenberg: volle Nutzerfarbe je LED");
    renderLeds(c, 0, user, out);
    check(out[0].r > 0 && out[0].r < pink.r / 4 && out[0].g == 0 && out[2].r == 0 && out[2].g > out[2].b,
          "Wellental: dunkel, Farbton bleibt, nie ganz aus");
    bool moves = false;
    LedPixels prev = out;
    for (std::uint32_t t = 20; t < kCalibrationWavePeriodMs; t += 20) {
        renderLeds(c, t, user, out);
        moves = moves || out[0].r != prev[0].r;
        prev = out;
    }
    check(moves, "Helligkeit ändert sich über die Periode");

    // Kalibriert: wie bisher nichts zeichnen (altes Bild bleibt stehen)
    c.gyro_calibrating = false;
    check(!renderLeds(c, 100, user, out), "kalibriert, Modus 253: renderLeds zeichnet nichts");

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

    // Fusions-Neustart beim Balancieren (Modus 1): wabert ebenfalls statt Nutzerfarbe fest
    c.fault = LedFault::None;
    c.robot_mode = 1;
    renderLeds(c, 0, user, out);
    check(out[0].r < pink.r / 4, "Modus 1, nicht kalibriert: wabert");

    std::printf(fail ? "FEHLER\n" : "alles ok\n");
    return fail;
}
