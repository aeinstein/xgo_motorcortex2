// FloorNet im Betrieb: Gewichte aus einer Binärdatei (tools/floornet/export.py), feste Topologie wie
// tools/floornet/net.py, Rechnung in floornet_kern.h. Über ctypes aus xgonav/floornet.py.
//
// Eingabe Vollbild 640x480 BGR, Ausgabe je Spalte (80) die Vollbildzeile der Bodengrenze: bestes Zeilenfach,
// mit Parabel verfeinert, einzelne Zacken auf Wunsch herausgenommen (fn_set_outlier), wie net.decode. Die Datei enthält einen Testvektor: fn_selftest rechnet ein künstliches
// Bild und vergleicht die Logits mit PyTorch.
#include <cstdint>
#include <cstdio>

#if defined(__aarch64__)

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

#include "floornet_kern.h"

using namespace fn_kern;

namespace {

constexpr int kAbi = 2;
constexpr uint32_t kVersion = 1;
constexpr int kDown = 2, kNcols = 80, kBins = 64, kHead = 32;
constexpr int kOutCols = 7, kOutBins = 2;      // wie net.OUTLIER_COLS, net.OUTLIER_BINS
constexpr float kMasked = -1e9f;               // Fach außerhalb des Fensters, wie masked_fill in net.decode

struct Stage { int c, sy, sx, n; };

struct Handle {
    Net net;
    float centers[kBins];
    std::vector<float> test;     // erwartete Logits des Testbilds: [Fach][Spalte]
    float out_px = 0.f, out_ratio = 0.3f;      // Zacken herausnehmen, 0 = aus
};

bool read_floats(FILE* f, std::vector<float>& v, size_t n) {
    v.resize(n);
    return std::fread(v.data(), sizeof(float), n, f) == n;
}

Op make(Kind k, int in, int out, int sy, int sx, bool relu) {
    Op op;
    op.kind = k; op.in = in; op.out = out; op.sy = sy; op.sx = sx; op.relu = relu;
    return op;
}

// Netz wie net.FloorNet aufbauen, Gewichte in derselben Reihenfolge wie net.fold.
bool load_net(FILE* f, Net& net) {
    net.f = kDown;
    net.nr = kBins;
    const int W = kFullW / kDown, H = kCropH / kDown;
    auto add = [&](int c, int h, int w) {
        net.t.emplace_back();
        net.t.back().init(c, h, w);
        return int(net.t.size()) - 1;
    };
    auto stride = [](int n, int st) { return (n + st - 1) / st; };
    int cur = add(3, H, W);
    bool ok = true;
    {
        const Tensor in = net.t[cur];
        const int c0 = 12, o = add(c0, stride(in.h, 2), stride(in.w, 2));
        Op op = make(kStem, cur, o, 2, 2, true);
        ok = ok && read_floats(f, op.w, size_t(c0) * in.c * 9) && read_floats(f, op.b, c0);
        net.ops.push_back(std::move(op));
        cur = o;
    }
    auto ds = [&](int c, int sy, int sx, bool residual) {
        const Tensor in = net.t[cur];
        const int src = cur, h = stride(in.h, sy), w = stride(in.w, sx);
        const int od = add(in.c, h, w);
        Op d = make(kDw, cur, od, sy, sx, false);
        ok = ok && read_floats(f, d.w, size_t(in.c) * 9);
        net.ops.push_back(std::move(d));
        const int o = add(c, h, w);
        Op p = make(kPw, od, o, 1, 1, true);
        ok = ok && read_floats(f, p.w, size_t(c) * in.c) && read_floats(f, p.b, c);
        if (residual) p.res = src;
        if (ok) pack_pw(p, in.c, c);
        net.ops.push_back(std::move(p));
        cur = o;
    };
    for (const Stage& st : {Stage{24, 2, 2, 1}, Stage{32, 2, 1, 2}, Stage{48, 2, 1, 3}})
        for (int i = 0; i < st.n; ++i) {
            if (i == 0) ds(st.c, st.sy, st.sx, false);
            else ds(st.c, 1, 1, true);
        }
    {
        const Tensor in = net.t[cur];
        const int cc = in.c * in.h, w = in.w;
        const int oc = add(cc, 1, w);
        net.ops.push_back(make(kCollapse, cur, oc, 1, 1, false));
        const int o1 = add(kHead, 1, w);
        Op p1 = make(kPw, oc, o1, 1, 1, true);
        ok = ok && read_floats(f, p1.w, size_t(kHead) * cc) && read_floats(f, p1.b, kHead);
        if (ok) pack_pw(p1, cc, kHead);
        net.ops.push_back(std::move(p1));
        const int o2 = add(kHead, 1, w);
        Op d1 = make(kDw1d, o1, o2, 1, 1, true);
        ok = ok && read_floats(f, d1.w, size_t(kHead) * 3) && read_floats(f, d1.b, kHead);
        net.ops.push_back(std::move(d1));
        const int o3 = add(kBins, 1, w);
        Op p2 = make(kPw, o2, o3, 1, 1, false);
        ok = ok && read_floats(f, p2.w, size_t(kBins) * kHead) && read_floats(f, p2.b, kBins);
        if (ok) pack_pw(p2, kHead, kBins);
        net.ops.push_back(std::move(p2));
    }
    if (ok) {
        assign_slots(net, true);
        ok = net.t.back().w == kNcols;
    }
    return ok;
}

// Bestes Fach der Spalte x unter den Fächern lo..hi, mit Parabel; Fächer außerhalb zählen als kMasked.
float peak(const Handle& h, const Tensor& out, int x, int lo, int hi, int* best_out) {
    int best = lo;
    float bv = out.row(lo, 0)[x];
    for (int c = lo + 1; c <= hi; ++c) {
        const float v = out.row(c, 0)[x];
        if (v > bv) { bv = v; best = c; }
    }
    float frac = 0.f;
    if (best > 0 && best < kBins - 1) {
        const float l = best - 1 >= lo ? out.row(best - 1, 0)[x] : kMasked;
        const float r = best + 1 <= hi ? out.row(best + 1, 0)[x] : kMasked;
        const float den = l - 2.f * bv + r;
        if (den < -1e-6f) frac = std::max(-0.5f, std::min(0.5f, 0.5f * (l - r) / den));
    }
    if (best_out) *best_out = best;
    const float pos = float(best) + frac;
    const int a = std::max(0, std::min(kBins - 1, int(std::floor(pos))));
    const int b = std::min(kBins - 1, a + 1);
    const float t = pos - float(a);
    return h.centers[a] * (1.f - t) + h.centers[b] * t;
}

// Grenzzeile je Spalte, wie net.decode: bestes Fach; mit out_px > 0 gilt für eine Spalte, die um mehr als out_px
// vom Median ihrer kOutCols Nachbarn abweicht, das beste Fach beim Medianwert, wenn dort mindestens out_ratio der
// Masse ihres eigenen Gipfels liegt.
void decode(const Handle& h, float* rows) {
    const Tensor& out = h.net.t.back();
    int best[kNcols];
    for (int x = 0; x < kNcols; ++x) rows[x] = peak(h, out, x, 0, kBins - 1, &best[x]);
    if (!(h.out_px > 0.f)) return;
    float raw[kNcols];
    std::memcpy(raw, rows, sizeof(raw));
    for (int x = 0; x < kNcols; ++x) {
        float v[kOutCols];
        for (int k = 0; k < kOutCols; ++k) v[k] = raw[std::max(0, std::min(kNcols - 1, x + k - kOutCols / 2))];
        std::nth_element(v, v + kOutCols / 2, v + kOutCols);
        const float med = v[kOutCols / 2];
        if (!(std::fabs(raw[x] - med) > h.out_px)) continue;
        int nb = 0;
        for (int c = 1; c < kBins; ++c)
            if (std::fabs(h.centers[c] - med) < std::fabs(h.centers[nb] - med)) nb = c;
        const float top = out.row(best[x], 0)[x];
        auto mass = [&](int b) {             // Summe der Wahrscheinlichkeiten um Fach b, bis auf den gemeinsamen Nenner
            float s = 0.f;
            for (int c = std::max(0, b - kOutBins); c <= std::min(kBins - 1, b + kOutBins); ++c)
                s += std::exp(out.row(c, 0)[x] - top);
            return s;
        };
        if (mass(nb) >= h.out_ratio * mass(best[x]))
            rows[x] = peak(h, out, x, std::max(0, nb - kOutBins), std::min(kBins - 1, nb + kOutBins), nullptr);
    }
}

void run(Handle& h, const uint8_t* bgr, int stride) {
    preprocess(bgr, kDown, h.net.t[0], stride);
    forward(h.net);
}

// Künstliches Testbild, dasselbe wie export.test_frame.
std::vector<uint8_t> test_frame() {
    std::vector<uint8_t> v(size_t(kFullW) * kFullH * 3);
    for (int y = 0; y < kFullH; ++y)
        for (int x = 0; x < kFullW; ++x)
            for (int c = 0; c < 3; ++c)
                v[(size_t(y) * kFullW + x) * 3 + c] = uint8_t(((x * 3 + y * 5 + c * 40) ^ ((x / 32) * (y / 24) * 17)) & 255);
    return v;
}

}  // namespace

extern "C" {

int fn_abi() { return kAbi; }

void* fn_new(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return nullptr;
    auto h = std::make_unique<Handle>();
    char magic[4];
    uint32_t head[3];
    bool ok = std::fread(magic, 1, 4, f) == 4 && std::memcmp(magic, "FNET", 4) == 0 && std::fread(head, 4, 3, f) == 3 &&
              head[0] == kVersion && head[1] == uint32_t(kNcols) && head[2] == uint32_t(kBins) &&
              std::fread(h->centers, sizeof(float), kBins, f) == size_t(kBins);
    ok = ok && load_net(f, h->net) && read_floats(f, h->test, size_t(kBins) * kNcols);
    if (ok) {                    // Datei muss genau hier enden
        char tail[5];
        ok = std::fread(tail, 1, 5, f) == 4 && std::memcmp(tail, "ENDE", 4) == 0;
    }
    std::fclose(f);
    return ok ? h.release() : nullptr;
}

void fn_free(void* h) { delete static_cast<Handle*>(h); }

// Zacken herausnehmen: px = Abweichung vom Median der Nachbarspalten in Vollbildzeilen (0 = aus), ratio = nötige
// Masse beim Medianwert gegenüber dem eigenen Gipfel.
void fn_set_outlier(void* hp, float px, float ratio) {
    Handle& h = *static_cast<Handle*>(hp);
    h.out_px = px;
    h.out_ratio = ratio;
}

// bgr: Vollbild 640x480, stride Byte je Zeile. rows: kNcols Werte. logits: optional kBins * kNcols ([Fach][Spalte]).
void fn_run(void* hp, const uint8_t* bgr, int stride, float* rows, float* logits) {
    Handle& h = *static_cast<Handle*>(hp);
    run(h, bgr, stride);
    decode(h, rows);
    if (logits) {
        const Tensor& out = h.net.t.back();
        for (int c = 0; c < kBins; ++c) std::memcpy(logits + size_t(c) * kNcols, out.row(c, 0), sizeof(float) * kNcols);
    }
}

// Nur die Auswertung, für Tests: logits ([Fach][Spalte]) ersetzen die Netzausgabe, rows wie fn_run.
void fn_decode(void* hp, const float* logits, float* rows) {
    Handle& h = *static_cast<Handle*>(hp);
    Tensor& out = h.net.t.back();
    for (int c = 0; c < kBins; ++c) std::memcpy(out.row(c, 0), logits + size_t(c) * kNcols, sizeof(float) * kNcols);
    decode(h, rows);
}

// Größter relativer Fehler der Logits auf dem Testbild gegen PyTorch.
double fn_selftest(void* hp) {
    Handle& h = *static_cast<Handle*>(hp);
    const std::vector<uint8_t> frame = test_frame();
    run(h, frame.data(), kFullW * 3);
    const Tensor& out = h.net.t.back();
    double worst = 0.0;
    for (int c = 0; c < kBins; ++c)
        for (int x = 0; x < kNcols; ++x) {
            const double want = h.test[size_t(c) * kNcols + x], got = out.row(c, 0)[x];
            if (!std::isfinite(got)) return 1e9;
            worst = std::max(worst, std::fabs(want - got) / (1.0 + std::fabs(want)));
        }
    return worst;
}

}  // extern "C"

#else  // ohne NEON gibt es den Kern nicht; xgonav/floornet.py meldet das über die ABI-Nummer

extern "C" int fn_abi() { return 0; }

#endif
