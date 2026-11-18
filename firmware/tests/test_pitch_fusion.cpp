// Host-Test für Patch 0017: Nick-Fusion ohne Nachhinken. updateImu mit synthetischen Rohdaten, Rider
// liegend von Hand genickt (reine Drehung um die Nickachse, keine Linearbeschleunigung), Abtastung 3 ms
// wie am RiderPi. 5 s Stillstand (Startkalibrierung), dann Sinus -37..+28° mit Periode 2, 5 und 12 s.
// Ausgewertet wie am Rider gemessen (2026-11-17): Winkeländerung je 0,1 s, wahr gegen Fusion, nur
// Paare mit 10..50 °/s.
//   vor 0017: 1,059 / 1,053 / 1,027 (Beschleunigungs-Nick mit ~0,1 s vorgefiltert, hinkt nach)
//   0017:     1,00 ± 0,01
// Dazu: gehaltene Stellungen stimmen auf 0,05°. Seit Patch 0021 (neue Lagefusion, Nullpunkte aller Achsen
// aus der Startkalibrierung) verschiebt ein Nick-Kreisel-Nullpunkt von -1,2 °/s den Winkel nicht mehr
// (bis 0020: ~1,8°). Drehungen um die Senkrechte prüft test_imu_yaw_coupling, echte Fahrdaten
// test_lagefusion_replay.
// Nach firmware/build.sh (Quellen mit Patches in firmware/build/), im Repo-Root:
//   S=firmware/build/xgorider2_rl/firmware/src
//   g++ -std=c++17 -ffp-contract=off -I $S firmware/tests/test_pitch_fusion.cpp $S/control/rider_leg_imu.cpp -o t && ./t
#include "control/rider_leg_imu.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <vector>

using namespace rider::control;

static const double kPi = 3.14159265358979323846;
static int fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "ok    " : "FEHLER", what);
    if (!ok) fail = 1;
}
static void put(std::uint8_t* p, double v) {
    long r = std::lround(v);
    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;
    const auto u = std::uint16_t(std::int16_t(r));
    p[0] = std::uint8_t(u >> 8); p[1] = std::uint8_t(u & 0xff);
}

struct Probe { double wahr, imu; };

// theta(t) in Grad ab t = 0 (nach 5 s Stillstand bei theta(0)); bias = Nullpunkt des Nick-Kreisels (°/s)
static std::vector<Probe> lauf(const std::function<double(double)>& theta, double dauer_s, double bias = 0.0) {
    const double dt = 0.003, g = 9.8;
    const double ka = 1.0 / (0.000030517578125 * 19.6);   // m/s^2 -> Rohwert (±2 g)
    const double kg = 1.0 / (0.000030517578125 * 2000.0); // °/s -> Rohwert (±2000 °/s)
    ImuState s;
    std::vector<Probe> out;
    std::uint32_t us = 0;
    double prev = theta(0.0);
    const int vor = int(5.0 / dt), n = vor + int(dauer_s / dt);
    for (int k = 0; k < n; ++k) {
        const double t = k < vor ? 0.0 : (k - vor) * dt;
        const double th = theta(t), rate = (th - prev) / dt;
        prev = th;
        std::uint8_t raw[12];
        // Firmware: acc[1] = -AY, acc[2] = +AZ, Nick = atan(acc[1]/acc[2]); gyro[1] = +GX, Nick -= gyro*dt
        put(raw + 0, 0.0);
        put(raw + 2, -g * std::sin(th * kPi / 180.0) * ka);
        put(raw + 4, g * std::cos(th * kPi / 180.0) * ka);
        put(raw + 6, (-rate + bias) * kg);
        put(raw + 8, 0.0);
        put(raw + 10, 0.0);
        us += 3000;
        updateImu(s, raw, 12, us, std::uint32_t(k * dt * 1000) + 1000, 1);
        if (k == vor - 1) check(s.calibrated, "Startkalibrierung nach 5 s Stillstand fertig");
        if (k >= vor) out.push_back({th, double(s.pitch_deg)});
    }
    return out;
}

// Von a nach b ab t0 mit fester Rate (°/s), davor a, danach b. Sprünge gingen über den Kreiselbereich.
static double rampe(double t, double t0, double a, double b, double rate) {
    if (t <= t0) return a;
    const double d = (t - t0) * rate;
    return d >= std::fabs(b - a) ? b : a + (b > a ? d : -d);
}

// Summe |Winkeländerung wahr| / Summe |Winkeländerung Fusion| über 0,1-s-Paare mit 10..50 °/s, ab 5 s
static double verhaeltnis(const std::vector<Probe>& o) {
    const int m = 33;   // 0,1 s
    double sw = 0, si = 0;
    for (std::size_t i = std::size_t(5.0 / 0.003); i + m < o.size(); i += m) {
        const double dw = o[i + m].wahr - o[i].wahr, di = o[i + m].imu - o[i].imu;
        const double r = std::fabs(dw) / 0.099;
        if (r >= 10.0 && r < 50.0) { sw += std::fabs(dw); si += std::fabs(di); }
    }
    return si > 0 ? sw / si : 0.0;
}

int main() {
    for (double periode : {2.0, 5.0, 12.0}) {
        const auto o = lauf([periode](double t) { return -4.5 + 32.5 * std::sin(2.0 * kPi * t / periode); }, 30.0);
        const double v = verhaeltnis(o);
        char text[96];
        std::snprintf(text, sizeof text, "Sinus Periode %4.1f s: wahr/Fusion %.4f (Grenze 1 ± 0,01)", periode, v);
        check(std::fabs(v - 1.0) < 0.01, text);
    }
    {   // drei gehaltene Stellungen wie in der Messung am Rider, dazwischen mit 60 °/s bewegt
        const auto o = lauf([](double t) { return t < 6 ? 26.86 : t < 12 ? rampe(t, 6, 26.86, -5.28, 60)
                                                                         : rampe(t, 12, -5.28, -32.60, 60); }, 18.0);
        double e = 0;
        for (double t : {5.9, 11.9, 17.9}) {
            const auto& p = o[std::size_t(t / 0.003)];
            if (std::fabs(p.imu - p.wahr) > e) e = std::fabs(p.imu - p.wahr);
        }
        char text[96];
        std::snprintf(text, sizeof text, "gehaltene Stellungen: größte Abweichung %.3f° (Grenze 0,05°)", e);
        check(e < 0.05, text);
    }
    {   // 30° in 0,3 s: kein Nachhinken mehr (vorher bis -1,4°)
        const auto o = lauf([](double t) { return rampe(t, 2, 0.0, 30.0, 100); }, 6.0);
        double e = 0;
        for (double t : {2.35, 2.5, 3.0, 4.0}) {
            const auto& p = o[std::size_t(t / 0.003)];
            if (std::fabs(p.imu - p.wahr) > e) e = std::fabs(p.imu - p.wahr);
        }
        char text[96];
        std::snprintf(text, sizeof text, "30° in 0,3 s: größte Abweichung danach %.3f° (Grenze 0,1°)", e);
        check(e < 0.1, text);
    }
    {   // Nick-Kreisel-Nullpunkt -1,2 °/s wie am RiderPi: seit 0021 kalibriert, kein Versatz mehr (vorher ~1,8°)
        const auto o = lauf([](double) { return 0.0; }, 30.0, -1.2);
        char text[96];
        std::snprintf(text, sizeof text, "Nullpunkt -1,2 °/s: Versatz %.3f° (Grenze 0,05°)", o.back().imu);
        check(std::fabs(o.back().imu) < 0.05, text);
    }
    {   // derselbe Nullpunkt, dabei 30° in 0,3 s genickt und 20 s gehalten: bleibt beim wahren Winkel
        const auto o = lauf([](double t) { return rampe(t, 2, 0.0, 30.0, 100); }, 22.0, -1.2);
        double e = 0;
        for (double t : {2.5, 5.0, 12.0, 21.9}) {
            const auto& p = o[std::size_t(t / 0.003)];
            if (std::fabs(p.imu - p.wahr) > e) e = std::fabs(p.imu - p.wahr);
        }
        char text[96];
        std::snprintf(text, sizeof text, "Nullpunkt -1,2 °/s, 30° genickt: größte Abweichung %.3f° (Grenze 0,1°)", e);
        check(e < 0.1, text);
    }
    {   // Startkalibrierung bewegt (Gierrate zappelt, 20 Fenster verworfen, dann angenommen): Nullpunkte Nick/Roll
        // unbekannt. Die schnelle Nachführung (30 s) holt den Nick-Nullpunkt von -1,2 °/s, der Nick steht richtig.
        const double kg = 1.0 / (0.000030517578125 * 2000.0), ka = 1.0 / (0.000030517578125 * 19.6);
        ImuState s;
        std::uint32_t us = 0;
        double worst_late = 0;
        int cal_at = -1;
        for (int k = 0; k < int(60.0 / 0.003); ++k) {
            std::uint8_t raw[12];
            put(raw + 0, 0.0); put(raw + 2, 0.0); put(raw + 4, 9.8 * ka);
            put(raw + 6, -1.2 * kg); put(raw + 8, 0.0);
            put(raw + 10, (s.calibrated ? 0.0 : (k / 20 % 2 ? 1.0 : -1.0)) * kg);
            us += 3000;
            updateImu(s, raw, 12, us, std::uint32_t(k * 3) + 1000, 1);
            if (s.calibrated && cal_at < 0) cal_at = k;
            if (cal_at >= 0 && k > cal_at + int(20.0 / 0.003) && std::fabs(s.pitch_deg) > worst_late) worst_late = std::fabs(s.pitch_deg);
        }
        char text[200];
        std::snprintf(text, sizeof text, "Kalibrierung bewegt angenommen nach %.1f s: Nick-Nullpunkt am Ende %+.3f °/s, Nick ab 20 s danach höchstens %.3f° (Grenze 0,3°)",
                      cal_at * 0.003, s.gyro_bias[1], worst_late);
        check(cal_at > 0 && !s.bias_from_boot && std::fabs(s.gyro_bias[1] + 1.2f) < 0.1f && worst_late < 0.3, text);
    }
    std::printf(fail ? "FEHLER\n" : "ok\n");
    return fail;
}
