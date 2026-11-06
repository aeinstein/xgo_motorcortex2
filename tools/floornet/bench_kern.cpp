// FloorNet, Schritt 0: Rechenzeit eines UNTRAINIERTEN Spaltennetzes am CM4 messen.
// Legt fest, wie groß das Netz sein darf, bevor irgendetwas trainiert wird. Die Gewichte sind
// Zufall, das Ergebnis sagt also nichts über die Qualität, nur über Zeit und Speicher.
//
//   g++ -O3 -mcpu=native -std=c++17 -o bench_kern bench_kern.cpp
//   ./bench_kern [--cpu 3] [--iter 200] [--hz 0] [--only f3_s] [--share 1] [--ops 0]
//
// Aufbau wie geplant: Bild 640x480 -> Ausschnitt ab knapp über dem Horizont -> f:1 verkleinern ->
// Stem (3x3, Schritt 2) -> Tiefenweise-Separable-Blöcke -> Zeilen zusammenlegen -> Spaltenkopf
// (1x1, 1x3, 1x1). Ausgabe je Spalte: Zeile als Klassenverteilung plus Art der Grenze.
// float32, ein Thread. Vor der Messung wird jede Schicht gegen eine einfache Referenzrechnung
// geprüft, damit kein fehlerhafter Kern schnell aussieht.
//
#if !defined(__aarch64__)
#error "Nur für den Rider (aarch64 mit NEON) gedacht"
#endif
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "../../xgonav/native/floornet_kern.h"

namespace {

using namespace fn_kern;


double now_ms() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

// Je Spalte: beste Zeile mit Parabel-Verfeinerung, Art der Grenze. Rückgabe: Prüfsumme.
float postprocess(const Tensor& out, int nr, std::vector<float>& rows, std::vector<int>& kinds) {
    float chk = 0.f;
    for (int x = 0; x < out.w; ++x) {
        int best = 0;
        float bv = out.row(0, 0)[x];
        for (int c = 1; c < nr; ++c) {
            const float v = out.row(c, 0)[x];
            if (v > bv) { bv = v; best = c; }
        }
        float frac = 0.f;
        if (best > 0 && best < nr - 1) {
            const float l = out.row(best - 1, 0)[x], r = out.row(best + 1, 0)[x];
            const float den = l - 2 * bv + r;
            if (den < -1e-6f) frac = 0.5f * (l - r) / den;
        }
        int kind = 0;
        float kv = out.row(nr, 0)[x];
        for (int c = 1; c < out.c - nr; ++c) {
            const float v = out.row(nr + c, 0)[x];
            if (v > kv) { kv = v; kind = c; }
        }
        rows[x] = best + frac;
        kinds[x] = kind;
        chk += rows[x] + kind;
    }
    return chk;
}

// ---- Netz bauen --------------------------------------------------------------------------------

struct Stage { int c, sy, sx, n; };

struct Spec {
    std::string name;
    int f;                    // Verkleinerung 640 -> 640/f
    int c0;                   // Kanäle des Stems
    std::vector<Stage> st;
    int head;                 // Kanäle im Spaltenkopf
};

std::vector<float> randn(std::mt19937& g, size_t n, float sigma) {
    std::normal_distribution<float> d(0.f, sigma);
    std::vector<float> v(n);
    for (float& x : v) x = d(g);
    return v;
}

Net build(const Spec& s, std::mt19937& g) {
    Net net;
    net.f = s.f;
    const int W = kFullW / s.f, H = kCropH / s.f;
    auto add = [&](int c, int h, int w) {
        net.t.emplace_back();
        net.t.back().init(c, h, w);
        return int(net.t.size()) - 1;
    };
    int cur = add(3, H, W);
    auto stride = [](int n, int st) { return (n + st - 1) / st; };
    auto make = [](Kind k, int in, int out, int sy, int sx, bool relu, std::vector<float> w, std::vector<float> b) {
        Op op;
        op.kind = k; op.in = in; op.out = out; op.sy = sy; op.sx = sx; op.relu = relu;
        op.w = std::move(w); op.b = std::move(b);
        return op;
    };

    // Stem: volle 3x3-Faltung, Schritt 2
    {
        const int h = stride(H, 2), w = stride(W, 2);
        const int o = add(s.c0, h, w);
        net.ops.push_back(make(kStem, cur, o, 2, 2, true, randn(g, size_t(s.c0) * 3 * 9, 0.33f), randn(g, s.c0, 0.05f)));
        net.macs += 27.0 * s.c0 * h * w;
        cur = o;
    }
    auto ds = [&](int c, int sy, int sx, bool residual) {
        const Tensor in = net.t[cur];
        const int src = cur, h = stride(in.h, sy), w = stride(in.w, sx);
        const int od = add(in.c, h, w);
        net.ops.push_back(make(kDw, cur, od, sy, sx, false, randn(g, size_t(in.c) * 9, 0.33f), {}));
        net.macs += 9.0 * in.c * h * w;
        const float sig = 1.f / std::sqrt(float(in.c));
        const int op = add(c, h, w);
        Op p = make(kPw, od, op, 1, 1, true, randn(g, size_t(c) * in.c, sig), randn(g, c, 0.05f));
        if (residual) p.res = src;
        pack_pw(p, in.c, c);
        net.macs += double(in.c) * c * h * w;
        net.ops.push_back(std::move(p));
        cur = op;
    };
    for (const Stage& st : s.st)
        for (int i = 0; i < st.n; ++i) {
            if (i == 0) ds(st.c, st.sy, st.sx, st.sy == 1 && st.sx == 1 && net.t[cur].c == st.c);
            else ds(st.c, 1, 1, true);
        }
    // Spaltenkopf
    {
        const Tensor in = net.t[cur];
        const int cc = in.c * in.h, w = in.w;
        const int oc = add(cc, 1, w);
        net.ops.push_back(make(kCollapse, cur, oc, 1, 1, false, {}, {}));
        const int o1 = add(s.head, 1, w);
        Op p1 = make(kPw, oc, o1, 1, 1, true, randn(g, size_t(s.head) * cc, 1.f / std::sqrt(float(cc))), randn(g, s.head, 0.05f));
        pack_pw(p1, cc, s.head);
        net.macs += double(cc) * s.head * w;
        net.ops.push_back(std::move(p1));
        const int o2 = add(s.head, 1, w);
        net.ops.push_back(make(kDw1d, o1, o2, 1, 1, true, randn(g, size_t(s.head) * 3, 0.5f), randn(g, s.head, 0.05f)));
        net.macs += 3.0 * s.head * w;
        const int co = net.nr + 3;
        const int o3 = add(co, 1, w);
        Op p2 = make(kPw, o2, o3, 1, 1, false, randn(g, size_t(co) * s.head, 1.f / std::sqrt(float(s.head))), randn(g, co, 0.05f));
        pack_pw(p2, s.head, co);
        net.macs += double(s.head) * co * w;
        net.ops.push_back(std::move(p2));
    }
    for (const Op& o : net.ops) net.params += double(o.w.size() + o.b.size());
    return net;
}

std::vector<std::vector<uint8_t>> make_frames(std::mt19937& g, int n) {
    // Glattes Rauschen, Werte im Bereich echter Bilder. Die Laufzeit hängt nicht vom Inhalt ab.
    std::vector<std::vector<uint8_t>> out(n);
    std::uniform_int_distribution<int> d(0, 255);
    for (auto& fr : out) {
        fr.resize(size_t(kFullW) * kFullH * 3);
        int a = d(g), b = d(g), c = d(g);
        for (size_t i = 0; i < fr.size(); i += 3) {
            a += (d(g) - a) >> 3;
            b += (d(g) - b) >> 3;
            c += (d(g) - c) >> 3;
            fr[i] = uint8_t(a);
            fr[i + 1] = uint8_t(b);
            fr[i + 2] = uint8_t(c);
        }
    }
    return out;
}

// Je Schicht: Referenz und schneller Weg auf derselben Eingabe, danach weiter mit dem schnellen.
bool verify(Net& net, const std::vector<uint8_t>& frame, double* worst) {
    preprocess(frame.data(), net.f, net.t[0]);
    bool ok = true;
    double w = 0;
    for (const Op& op : net.ops) {
        Tensor& out = net.t[op.out];
        const std::vector<float> before(out.base, out.base + out.size());
        ref_op(net, op);
        const std::vector<float> want(out.base, out.base + out.size());
        std::copy(before.begin(), before.end(), out.base);
        run_op(net, op);
        for (size_t j = 0; j < want.size(); ++j) {
            if (!std::isfinite(out.base[j])) ok = false;
            const double e = std::fabs(double(want[j]) - out.base[j]) / (1.0 + std::fabs(want[j]));
            w = std::max(w, e);
        }
        if (!border_clean(out) || !border_clean(net.t[op.in])) ok = false;
    }
    *worst = w;
    return ok && w < 1e-3;
}

}  // namespace

int main(int argc, char** argv) {
    int cpu = 3, iter = 200;
    double hz = 0;
    bool share = true, ops = false;
    std::string only;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string a = argv[i];
        if (a == "--cpu") cpu = std::atoi(argv[i + 1]);
        else if (a == "--iter") iter = std::atoi(argv[i + 1]);
        else if (a == "--hz") hz = std::atof(argv[i + 1]);
        else if (a == "--only") only = argv[i + 1];
        else if (a == "--share") share = std::atoi(argv[i + 1]) != 0;
        else if (a == "--ops") ops = std::atoi(argv[i + 1]) != 0;
        else { std::fprintf(stderr, "unbekannt: %s\n", a.c_str()); return 2; }
    }
    if (cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) std::perror("sched_setaffinity");
    }
    // Größen: t(iny), s(mall), m(edium), l(arge) je Eingabebreite. Ausgabe ~80 Spalten (f3: 54).
    const std::vector<Spec> specs = {
        {"f4_t", 4, 8, {{16, 2, 1, 1}, {24, 2, 1, 2}, {32, 1, 1, 2}}, 24},
        {"f4_s", 4, 12, {{24, 2, 1, 1}, {32, 2, 1, 2}, {48, 1, 1, 3}}, 32},
        {"f4_m", 4, 16, {{32, 2, 1, 2}, {48, 2, 1, 3}, {64, 1, 1, 4}}, 32},
        {"f3_t", 3, 8, {{16, 2, 2, 1}, {24, 2, 1, 2}, {32, 1, 1, 2}}, 24},
        {"f3_s", 3, 12, {{24, 2, 2, 1}, {32, 2, 1, 2}, {48, 1, 1, 3}}, 32},
        {"f3_m", 3, 16, {{32, 2, 2, 2}, {48, 2, 1, 3}, {64, 1, 1, 4}}, 32},
        {"f2_t", 2, 8, {{16, 2, 2, 1}, {24, 2, 1, 2}, {32, 2, 1, 2}}, 24},
        {"f2_s", 2, 12, {{24, 2, 2, 1}, {32, 2, 1, 2}, {48, 2, 1, 3}}, 32},
        {"f2_m", 2, 16, {{32, 2, 2, 2}, {48, 2, 1, 3}, {64, 2, 1, 4}}, 32},
    };
    std::mt19937 g(1234);
    const auto frames = make_frames(g, 4);
    std::printf("CPU %d, %d Durchläufe, Takt %s\n", cpu, iter, hz > 0 ? (std::to_string(hz) + " Hz").c_str() : "frei");
    if (!ops)
        std::printf("%-5s %9s %5s %7s %7s %7s %6s | %6s %6s %6s %6s | %8s  %s\n", "Netz", "Eingabe", "Spalt", "MMACs", "kParam",
                    "Gew.kB", "Puf.kB", "mean", "p50", "p95", "max", "MMAC/ms", "Vorverarb. ms (ref-Fehler)");
    int rc = 0;
    for (const Spec& s : specs) {
        if (!only.empty() && s.name != only) continue;
        Net net = build(s, g);
        assign_slots(net, share);
        double worst = 0;
        if (!verify(net, frames[0], &worst)) {
            std::printf("%-5s Referenz weicht ab: %.2e - nicht gemessen\n", s.name.c_str(), worst);
            rc = 1;
            continue;
        }
        if (ops) {
            // Zeit und MACs je Schichtart: 40 Durchläufe, jede Schicht einzeln gestoppt
            double tm[kKinds] = {}, mc[kKinds] = {};
            for (int it = 0; it < 40; ++it) {
                preprocess(frames[0].data(), net.f, net.t[0]);
                for (const Op& op : net.ops) {
                    const double t0 = now_ms();
                    run_op(net, op);
                    tm[op.kind] += now_ms() - t0;
                    const Tensor& o = net.t[op.out];
                    const double px = double(o.h) * o.w;
                    if (op.kind == kStem) mc[kStem] += 27.0 * o.c * px;
                    else if (op.kind == kDw) mc[kDw] += 9.0 * o.c * px;
                    else if (op.kind == kPw) mc[kPw] += double(net.t[op.in].c) * o.c * px;
                    else if (op.kind == kDw1d) mc[kDw1d] += 3.0 * o.c * px;
                }
            }
            const char* names[kKinds] = {"Stem", "DW3x3", "PW1x1", "Zeilen", "DW1x3"};
            std::printf("  %s:", s.name.c_str());
            for (int k = 0; k < kKinds; ++k)
                if (tm[k] > 0)
                    std::printf(" %s %.2f ms %.1f MMAC (%.2f/ms)", names[k], tm[k] / 40, mc[k] / 40 * 1e-6, (mc[k] * 1e-6) / tm[k]);
            std::printf("\n");
            continue;
        }
        std::vector<float> rows(net.t.back().w);
        std::vector<int> kinds(net.t.back().w);
        std::vector<double> ms;
        double pre = 0;
        volatile float sink = 0;
        const int warm = 20;
        for (int it = 0; it < iter + warm; ++it) {
            const double t0 = now_ms();
            preprocess(frames[it % frames.size()].data(), net.f, net.t[0]);
            const double t1 = now_ms();
            forward(net);
            sink = sink + postprocess(net.t.back(), net.nr, rows, kinds);
            const double t2 = now_ms();
            if (it >= warm) {
                ms.push_back(t2 - t1);
                pre += t1 - t0;
            }
            if (hz > 0) {
                const double wait = 1000.0 / hz - (now_ms() - t0);
                if (wait > 0) {
                    timespec ts{time_t(wait / 1000), long(std::fmod(wait, 1000.0) * 1e6)};
                    nanosleep(&ts, nullptr);
                }
            }
        }
        std::sort(ms.begin(), ms.end());
        double sum = 0;
        for (double v : ms) sum += v;
        const double mean = sum / ms.size();
        std::printf("%-5s %4dx%-4d %5d %7.1f %7.1f %7.0f %6.0f | %6.2f %6.2f %6.2f %6.2f | %8.2f  %.2f (%.1e)\n",
                    s.name.c_str(), kFullW / s.f, kCropH / s.f, net.t.back().w, net.macs * 1e-6, net.params * 1e-3,
                    net.weight_bytes() / 1024.0, net.buffer_bytes() / 1024.0, mean, ms[ms.size() / 2],
                    ms[size_t(ms.size() * 0.95)], ms.back(), net.macs * 1e-6 / mean, pre / iter, worst);
        std::fflush(stdout);
    }
    return rc;
}
