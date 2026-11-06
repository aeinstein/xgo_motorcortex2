// FloorNet: NEON-Kerne für das Spaltennetz. Gemeinsam für den Betrieb (floornet.cpp) und die Messung
// (tools/floornet/bench_kern.cpp).
//
// A72 am Rider, gemessen 2026-11-06 (1,496 GHz): FMA Q-Form Latenz 3 Takte, 1 je Takt; ab vier
// unabhängigen Akkumulatoren voll ausgelastet (4 MAC/Takt = 6 MMAC/ms). Q-Load dagegen nur etwa
// alle 2 Takte: Laden + FMA 1:1 bleibt bei 2,8 MMAC/ms. Darum hier jeden geladenen Wert mehrfach
// nutzen: 1x1 wie GEMM (ein Eingabevektor für acht Ausgangskanäle), 3x3 mit vext statt dreier Ladebefehle.
//
// Vorgaben ans Netz, die das Training einhalten muss: nach der tiefenweisen Faltung kein ReLU und
// kein Bias (der Bias steckt in dem der folgenden 1x1-Schicht); Residuum vor dem ReLU der 1x1-Schicht.
#pragma once
#if !defined(__aarch64__)
#error "Nur für aarch64 mit NEON"
#endif
#include <arm_neon.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace fn_kern {

constexpr int kSlack = 16;                 // Rand rechts: Vektorladen darf darüber hinaus lesen
constexpr int kFullW = 640, kFullH = 480;
constexpr int kCropY = 137;                // Horizont beim Balancieren ~Zeile 217, 80 Zeilen Kontext darüber
constexpr int kCropH = 343;

// Planar (Kanal, Zeile, Spalte), je Ebene ein Rand aus Nullen: Zeile -1 und h, Spalte -1 und rechts
// kSlack. Der Rand wird nie beschrieben; das ist die Null-Auffüllung von Conv2d(padding=1).
// Der Speicher liegt in Net::slots: Tensoren gleicher Größe, die nicht gleichzeitig leben, teilen
// sich einen Platz.
struct Tensor {
    int c = 0, h = 0, w = 0, st = 0;
    size_t plane = 0;
    float* base = nullptr;
    void init(int c_, int h_, int w_) {
        c = c_; h = h_; w = w_;
        st = w + 1 + kSlack;
        plane = size_t(h + 2) * st;
    }
    size_t size() const { return plane * c; }
    float* row(int ch, int y) { return base + ch * plane + size_t(y + 1) * st + 1; }
    const float* row(int ch, int y) const { return base + ch * plane + size_t(y + 1) * st + 1; }
};

enum Kind { kStem, kDw, kPw, kCollapse, kDw1d, kKinds };

struct Op {
    Kind kind;
    int in, out;
    int sy = 1, sx = 1;
    bool relu = false;
    std::vector<float> w, b;               // b leer = kein Bias
    int res = -1;                          // Residuum: dieser Tensor wird vor dem ReLU addiert (nur kPw)
    std::vector<float> pk;                 // kPw: Gewichte je Achtergruppe und Eingangskanal
    mutable std::vector<const float*> src; // kPw: Zeigerfeld, einmal angelegt
};

struct Net {
    std::vector<Tensor> t;
    std::vector<Op> ops;
    std::vector<std::vector<float>> slots;
    int f = 2, nr = 64;
    double macs = 0, params = 0;
    size_t weight_bytes() const {
        size_t n = 0;
        for (const Op& o : ops) n += (o.w.size() + o.b.size()) * sizeof(float);
        return n;
    }
    size_t buffer_bytes() const {
        size_t n = 0;
        for (const auto& s : slots) n += s.size() * sizeof(float);
        return n;
    }
};

// Tensoren auf Speicherplätze verteilen. share: nach Lebensdauer teilen (nur gleiche Höhe und
// Breite, damit der Null-Rand an derselben Stelle bleibt), sonst jeder Tensor für sich.
inline void assign_slots(Net& net, bool share) {
    const int nt = int(net.t.size()), nop = int(net.ops.size());
    std::vector<int> last(nt, -1), slot_of(nt, -1);
    for (int k = 0; k < nop; ++k) {
        const Op& op = net.ops[k];
        last[op.in] = std::max(last[op.in], k);
        last[op.out] = std::max(last[op.out], k);
        if (op.res >= 0) last[op.res] = std::max(last[op.res], k);
    }
    last[nt - 1] = nop;                    // Ausgabe bleibt bis zur Auswertung
    struct Slot { int h, w; size_t size; int busy_until; };
    std::vector<Slot> slots;
    auto take = [&](int id, int from) {
        const Tensor& x = net.t[id];
        int found = -1;
        if (share)
            for (int s = 0; s < int(slots.size()); ++s)
                if (slots[s].h == x.h && slots[s].w == x.w && slots[s].busy_until < from) { found = s; break; }
        if (found < 0) {
            slots.push_back({x.h, x.w, 0, -1});
            found = int(slots.size()) - 1;
        }
        slots[found].size = std::max(slots[found].size, x.size());
        slots[found].busy_until = last[id];
        slot_of[id] = found;
    };
    take(0, -1);
    for (int k = 0; k < nop; ++k)
        if (slot_of[net.ops[k].out] < 0) take(net.ops[k].out, k);
    net.slots.resize(slots.size());
    for (size_t s = 0; s < slots.size(); ++s) net.slots[s].assign(slots[s].size, 0.f);
    for (int i = 0; i < nt; ++i) net.t[i].base = net.slots[slot_of[i]].data();
}

// ---- schnelle Rechnung (NEON) ----------------------------------------------------------------

struct V4 { float32x4_t a, b, c, d; };

inline V4 load4(const float* p) { return {vld1q_f32(p), vld1q_f32(p + 4), vld1q_f32(p + 8), vld1q_f32(p + 12)}; }
inline void store4(float* p, const V4& v) {
    vst1q_f32(p, v.a);
    vst1q_f32(p + 4, v.b);
    vst1q_f32(p + 8, v.c);
    vst1q_f32(p + 12, v.d);
}
inline float32x4_t tap3(float32x4_t v, float32x4_t l, float32x4_t m, float32x4_t r, float32x4_t kl, float32x4_t km,
                        float32x4_t kr) {
    return vfmaq_f32(vfmaq_f32(vfmaq_f32(v, l, kl), m, km), r, kr);
}

// Drei Abgriffe einer Zeile bei Schritt 1, für 16 Spalten ab r. prev: die vier Werte vor r.
inline void row_s1(V4& v, const float* r, float32x4_t& prev, float32x4_t kl, float32x4_t km, float32x4_t kr) {
    const float32x4_t c0 = vld1q_f32(r), c1 = vld1q_f32(r + 4), c2 = vld1q_f32(r + 8), c3 = vld1q_f32(r + 12),
                      nx = vld1q_f32(r + 16);
    v.a = tap3(v.a, vextq_f32(prev, c0, 3), c0, vextq_f32(c0, c1, 1), kl, km, kr);
    v.b = tap3(v.b, vextq_f32(c0, c1, 3), c1, vextq_f32(c1, c2, 1), kl, km, kr);
    v.c = tap3(v.c, vextq_f32(c1, c2, 3), c2, vextq_f32(c2, c3, 1), kl, km, kr);
    v.d = tap3(v.d, vextq_f32(c2, c3, 3), c3, vextq_f32(c3, nx, 1), kl, km, kr);
    prev = c3;
}

// Dasselbe bei Schritt 2 (16 Ausgabespalten). vld2 ab r-1: val[0] = links (r-1, r+1, ...),
// val[1] = Mitte (r, r+2, ...); rechts = links um eins weiter.
inline void row_s2(V4& v, const float* r, float32x4_t kl, float32x4_t km, float32x4_t kr) {
    const float32x4x2_t g0 = vld2q_f32(r - 1), g1 = vld2q_f32(r + 7), g2 = vld2q_f32(r + 15), g3 = vld2q_f32(r + 23);
    v.a = tap3(v.a, g0.val[0], g0.val[1], vextq_f32(g0.val[0], g1.val[0], 1), kl, km, kr);
    v.b = tap3(v.b, g1.val[0], g1.val[1], vextq_f32(g1.val[0], g2.val[0], 1), kl, km, kr);
    v.c = tap3(v.c, g2.val[0], g2.val[1], vextq_f32(g2.val[0], g3.val[0], 1), kl, km, kr);
    v.d = tap3(v.d, g3.val[0], g3.val[1], vextq_f32(g3.val[0], vld1q_dup_f32(r + 31), 1), kl, km, kr);
}

// Rest der Zeile einzeln. Für Schritt sx: Eingabespalte = x * sx.
inline void dw_tail(float* o, const float* r0, const float* r1, const float* r2, const float* k, int x, int n, int sx,
                    bool acc) {
    for (; x < n; ++x) {
        const int i = x * sx;
        float s = acc ? o[x] : 0.f;
        s += k[0] * r0[i - 1] + k[1] * r0[i] + k[2] * r0[i + 1];
        s += k[3] * r1[i - 1] + k[4] * r1[i] + k[5] * r1[i + 1];
        s += k[6] * r2[i - 1] + k[7] * r2[i] + k[8] * r2[i + 1];
        o[x] = s;
    }
}

// Eine Ausgabezeile einer 3x3-Faltung auf einer Eingabeebene. acc: auf o aufaddieren (Stem).
inline void dw_row_s1(float* o, const float* r0, const float* r1, const float* r2, const float* k, int n, bool acc) {
    const float32x4_t k0 = vdupq_n_f32(k[0]), k1 = vdupq_n_f32(k[1]), k2 = vdupq_n_f32(k[2]), k3 = vdupq_n_f32(k[3]),
                      k4 = vdupq_n_f32(k[4]), k5 = vdupq_n_f32(k[5]), k6 = vdupq_n_f32(k[6]), k7 = vdupq_n_f32(k[7]),
                      k8 = vdupq_n_f32(k[8]);
    const float32x4_t z = vdupq_n_f32(0.f);
    float32x4_t p0 = z, p1 = z, p2 = z;    // links von Spalte 0 ist Rand (Null)
    int x = 0;
    for (; x + 16 <= n; x += 16) {
        V4 v = acc ? load4(o + x) : V4{z, z, z, z};
        row_s1(v, r0 + x, p0, k0, k1, k2);
        row_s1(v, r1 + x, p1, k3, k4, k5);
        row_s1(v, r2 + x, p2, k6, k7, k8);
        store4(o + x, v);
    }
    dw_tail(o, r0, r1, r2, k, x, n, 1, acc);
}

inline void dw_row_s2(float* o, const float* r0, const float* r1, const float* r2, const float* k, int n, bool acc) {
    const float32x4_t k0 = vdupq_n_f32(k[0]), k1 = vdupq_n_f32(k[1]), k2 = vdupq_n_f32(k[2]), k3 = vdupq_n_f32(k[3]),
                      k4 = vdupq_n_f32(k[4]), k5 = vdupq_n_f32(k[5]), k6 = vdupq_n_f32(k[6]), k7 = vdupq_n_f32(k[7]),
                      k8 = vdupq_n_f32(k[8]);
    const float32x4_t z = vdupq_n_f32(0.f);
    int x = 0;
    for (; x + 16 <= n; x += 16) {
        const int i = 2 * x;
        V4 v = acc ? load4(o + x) : V4{z, z, z, z};
        row_s2(v, r0 + i, k0, k1, k2);
        row_s2(v, r1 + i, k3, k4, k5);
        row_s2(v, r2 + i, k6, k7, k8);
        store4(o + x, v);
    }
    dw_tail(o, r0, r1, r2, k, x, n, 2, acc);
}

inline void finish_row(float* o, int n, float bias, bool relu) {
    const float32x4_t b = vdupq_n_f32(bias), z = vdupq_n_f32(0.f);
    int x = 0;
    for (; x + 4 <= n; x += 4) {
        float32x4_t v = vaddq_f32(vld1q_f32(o + x), b);
        if (relu) v = vmaxq_f32(v, z);
        vst1q_f32(o + x, v);
    }
    for (; x < n; ++x) {
        const float v = o[x] + bias;
        o[x] = relu ? std::max(v, 0.f) : v;
    }
}

inline void run_stem(const Op& op, const Tensor& in, Tensor& out) {
    for (int y = 0; y < out.h; ++y) {
        const int yi = y * op.sy;
        for (int oc = 0; oc < out.c; ++oc) {
            float* o = out.row(oc, y);
            for (int ic = 0; ic < in.c; ++ic) {
                const float* k = &op.w[size_t(oc * in.c + ic) * 9];
                const float* r0 = in.row(ic, yi - 1), *r1 = in.row(ic, yi), *r2 = in.row(ic, yi + 1);
                if (op.sx == 2) dw_row_s2(o, r0, r1, r2, k, out.w, ic > 0);
                else dw_row_s1(o, r0, r1, r2, k, out.w, ic > 0);
            }
            finish_row(o, out.w, op.b[oc], op.relu);
        }
    }
}

inline void run_dw(const Op& op, const Tensor& in, Tensor& out) {
    for (int y = 0; y < out.h; ++y) {
        const int yi = y * op.sy;
        for (int c = 0; c < out.c; ++c) {
            float* o = out.row(c, y);
            const float* k = &op.w[size_t(c) * 9];
            const float* r0 = in.row(c, yi - 1), *r1 = in.row(c, yi), *r2 = in.row(c, yi + 1);
            if (op.sx == 2) dw_row_s2(o, r0, r1, r2, k, out.w, false);
            else dw_row_s1(o, r0, r1, r2, k, out.w, false);
            if (!op.b.empty() || op.relu) finish_row(o, out.w, op.b.empty() ? 0.f : op.b[c], op.relu);
        }
    }
}

// 1x1 wie GEMM: je vier Spalten und acht Ausgangskanäle; für jeden Eingangskanal ein Ladebefehl für die
// Eingabe, zwei für die Gewichte, acht FMA.
inline void run_pw(const Op& op, const Tensor& in, Tensor& out, const Tensor* res) {
    const int n = out.w, cin = in.c, cout = out.c, groups = cout / 8;
    const float** src = op.src.data();
    const float32x4_t zero = vdupq_n_f32(0.f);
    const bool relu = op.relu;
    for (int y = 0; y < out.h; ++y) {
        for (int ic = 0; ic < cin; ++ic) src[ic] = in.row(ic, y);
        for (int g = 0; g < groups; ++g) {
            const float* pk0 = &op.pk[size_t(g) * cin * 8];
            const float* bias = &op.b[size_t(g) * 8];
            float* o[8];
            const float* rs[8] = {};
            for (int j = 0; j < 8; ++j) {
                o[j] = out.row(g * 8 + j, y);
                if (res) rs[j] = res->row(g * 8 + j, y);
            }
            const float32x4_t b0 = vld1q_f32(bias), b1 = vld1q_f32(bias + 4);
            int x = 0;
            for (; x + 4 <= n; x += 4) {
                float32x4_t v0 = vdupq_laneq_f32(b0, 0), v1 = vdupq_laneq_f32(b0, 1), v2 = vdupq_laneq_f32(b0, 2),
                            v3 = vdupq_laneq_f32(b0, 3), v4 = vdupq_laneq_f32(b1, 0), v5 = vdupq_laneq_f32(b1, 1),
                            v6 = vdupq_laneq_f32(b1, 2), v7 = vdupq_laneq_f32(b1, 3);
                if (res) {
                    v0 = vaddq_f32(v0, vld1q_f32(rs[0] + x));
                    v1 = vaddq_f32(v1, vld1q_f32(rs[1] + x));
                    v2 = vaddq_f32(v2, vld1q_f32(rs[2] + x));
                    v3 = vaddq_f32(v3, vld1q_f32(rs[3] + x));
                    v4 = vaddq_f32(v4, vld1q_f32(rs[4] + x));
                    v5 = vaddq_f32(v5, vld1q_f32(rs[5] + x));
                    v6 = vaddq_f32(v6, vld1q_f32(rs[6] + x));
                    v7 = vaddq_f32(v7, vld1q_f32(rs[7] + x));
                }
                const float* pk = pk0;
                for (int ic = 0; ic < cin; ++ic, pk += 8) {
                    const float32x4_t a = vld1q_f32(src[ic] + x), k0 = vld1q_f32(pk), k1 = vld1q_f32(pk + 4);
                    v0 = vfmaq_laneq_f32(v0, a, k0, 0);
                    v1 = vfmaq_laneq_f32(v1, a, k0, 1);
                    v2 = vfmaq_laneq_f32(v2, a, k0, 2);
                    v3 = vfmaq_laneq_f32(v3, a, k0, 3);
                    v4 = vfmaq_laneq_f32(v4, a, k1, 0);
                    v5 = vfmaq_laneq_f32(v5, a, k1, 1);
                    v6 = vfmaq_laneq_f32(v6, a, k1, 2);
                    v7 = vfmaq_laneq_f32(v7, a, k1, 3);
                }
                if (relu) {
                    v0 = vmaxq_f32(v0, zero);
                    v1 = vmaxq_f32(v1, zero);
                    v2 = vmaxq_f32(v2, zero);
                    v3 = vmaxq_f32(v3, zero);
                    v4 = vmaxq_f32(v4, zero);
                    v5 = vmaxq_f32(v5, zero);
                    v6 = vmaxq_f32(v6, zero);
                    v7 = vmaxq_f32(v7, zero);
                }
                vst1q_f32(o[0] + x, v0);
                vst1q_f32(o[1] + x, v1);
                vst1q_f32(o[2] + x, v2);
                vst1q_f32(o[3] + x, v3);
                vst1q_f32(o[4] + x, v4);
                vst1q_f32(o[5] + x, v5);
                vst1q_f32(o[6] + x, v6);
                vst1q_f32(o[7] + x, v7);
            }
            for (; x < n; ++x)
                for (int j = 0; j < 8; ++j) {
                    float s = bias[j] + (res ? rs[j][x] : 0.f);
                    for (int ic = 0; ic < cin; ++ic) s += pk0[size_t(ic) * 8 + j] * src[ic][x];
                    o[j][x] = relu ? std::max(s, 0.f) : s;
                }
        }
        // übrige Kanäle (cout nicht durch 8 teilbar) einzeln
        for (int oc = groups * 8; oc < cout; ++oc) {
            float* o = out.row(oc, y);
            const float* wt = &op.w[size_t(oc) * cin];
            if (res) std::memcpy(o, res->row(oc, y), sizeof(float) * n);
            else std::memset(o, 0, sizeof(float) * n);
            for (int ic = 0; ic < cin; ++ic) {
                const float* a = src[ic];
                const float32x4_t kk = vdupq_n_f32(wt[ic]);
                int x = 0;
                for (; x + 4 <= n; x += 4) vst1q_f32(o + x, vfmaq_f32(vld1q_f32(o + x), vld1q_f32(a + x), kk));
                for (; x < n; ++x) o[x] += wt[ic] * a[x];
            }
            finish_row(o, n, op.b[oc], relu);
        }
    }
}

inline void run_dw1d(const Op& op, const Tensor& in, Tensor& out) {
    const int n = out.w;
    for (int c = 0; c < out.c; ++c) {
        float* o = out.row(c, 0);
        const float* r = in.row(c, 0), *k = &op.w[size_t(c) * 3];
        const float32x4_t kl = vdupq_n_f32(k[0]), km = vdupq_n_f32(k[1]), kr = vdupq_n_f32(k[2]);
        int x = 0;
        for (; x + 4 <= n; x += 4) {
            float32x4_t v = vmulq_f32(vld1q_f32(r + x - 1), kl);
            v = vfmaq_f32(v, vld1q_f32(r + x), km);
            v = vfmaq_f32(v, vld1q_f32(r + x + 1), kr);
            vst1q_f32(o + x, v);
        }
        for (; x < n; ++x) o[x] = k[0] * r[x - 1] + k[1] * r[x] + k[2] * r[x + 1];
        finish_row(o, n, op.b[c], op.relu);
    }
}

inline void run_collapse(const Tensor& in, Tensor& out) {
    for (int c = 0; c < in.c; ++c)
        for (int y = 0; y < in.h; ++y)
            std::memcpy(out.row(c * in.h + y, 0), in.row(c, y), sizeof(float) * in.w);
}

inline void run_op(Net& net, const Op& op) {
    const Tensor& in = net.t[op.in];
    Tensor& out = net.t[op.out];
    switch (op.kind) {
        case kStem: run_stem(op, in, out); break;
        case kDw: run_dw(op, in, out); break;
        case kPw: run_pw(op, in, out, op.res >= 0 ? &net.t[op.res] : nullptr); break;
        case kCollapse: run_collapse(in, out); break;
        case kDw1d: run_dw1d(op, in, out); break;
        case kKinds: break;
    }
}

inline void forward(Net& net) {
    for (const Op& op : net.ops) run_op(net, op);
}

// ---- Referenz: dieselbe Rechnung als einfache Schleifen ----------------------------------------

inline void ref_op(Net& net, const Op& op) {
    const Tensor& in = net.t[op.in];
    Tensor& out = net.t[op.out];
    const Tensor* res = op.res >= 0 ? &net.t[op.res] : nullptr;
    for (int oc = 0; oc < out.c; ++oc)
        for (int y = 0; y < out.h; ++y)
            for (int x = 0; x < out.w; ++x) {
                float s = 0.f;
                if (op.kind == kCollapse) {
                    s = in.row(oc / in.h, oc % in.h)[x];
                } else {
                    if (op.kind == kStem || op.kind == kDw) {
                        const int first = op.kind == kStem ? 0 : oc, last = op.kind == kStem ? in.c : oc + 1;
                        for (int ic = first; ic < last; ++ic) {
                            const float* k = op.kind == kStem ? &op.w[size_t(oc * in.c + ic) * 9] : &op.w[size_t(oc) * 9];
                            for (int dy = -1; dy <= 1; ++dy)
                                for (int dx = -1; dx <= 1; ++dx)
                                    s += k[(dy + 1) * 3 + dx + 1] * in.row(ic, y * op.sy + dy)[x * op.sx + dx];
                        }
                    } else if (op.kind == kPw) {
                        for (int ic = 0; ic < in.c; ++ic) s += op.w[size_t(oc) * in.c + ic] * in.row(ic, y)[x];
                        if (res) s += res->row(oc, y)[x];
                    } else {             // kDw1d
                        const float* k = &op.w[size_t(oc) * 3];
                        const float* r = in.row(oc, 0);
                        s = k[0] * r[x - 1] + k[1] * r[x] + k[2] * r[x + 1];
                    }
                    if (!op.b.empty()) s += op.b[oc];
                    if (op.relu) s = std::max(s, 0.f);
                }
                out.row(oc, y)[x] = s;
            }
}

// Null-Rand noch unversehrt? (Der schnelle Weg liest darüber hinaus und darf ihn nie beschreiben.)
inline bool border_clean(const Tensor& x) {
    for (int c = 0; c < x.c; ++c) {
        for (int y = -1; y <= x.h; ++y) {
            const float* r = x.row(c, y);
            if (y == -1 || y == x.h) {
                for (int i = -1; i < x.w + kSlack; ++i)
                    if (r[i] != 0.f) return false;
            } else {
                if (r[-1] != 0.f) return false;
                for (int i = x.w; i < x.w + kSlack; ++i)
                    if (r[i] != 0.f) return false;
            }
        }
    }
    return true;
}

// Ausschnitt ab kCropY, f:1 mitteln, nach float (planar, -0.5..0.5). Frame ist BGR, interleaved.
inline void preprocess(const uint8_t* frame, int f, Tensor& in, int stride = kFullW * 3) {
    const int W = in.w, H = in.h;
    const float scale = 1.f / (255.f * f * f);
    for (int y = 0; y < H; ++y) {
        const uint8_t* rows[4];
        for (int j = 0; j < f; ++j) rows[j] = frame + size_t(kCropY + y * f + j) * stride;
        float* o0 = in.row(0, y), *o1 = in.row(1, y), *o2 = in.row(2, y);
        for (int x = 0; x < W; ++x) {
            int s0 = 0, s1 = 0, s2 = 0;
            for (int j = 0; j < f; ++j) {
                const uint8_t* p = rows[j] + size_t(x) * f * 3;
                for (int i = 0; i < f; ++i) {
                    s0 += p[3 * i];
                    s1 += p[3 * i + 1];
                    s2 += p[3 * i + 2];
                }
            }
            o0[x] = s0 * scale - 0.5f;
            o1[x] = s1 * scale - 0.5f;
            o2[x] = s2 * scale - 0.5f;
        }
    }
}

inline void pack_pw(Op& op, int cin, int cout) {
    const int groups = cout / 8;
    op.pk.resize(size_t(groups) * cin * 8);
    for (int g = 0; g < groups; ++g)
        for (int ic = 0; ic < cin; ++ic)
            for (int j = 0; j < 8; ++j)
                op.pk[(size_t(g) * cin + ic) * 8 + j] = op.w[size_t(g * 8 + j) * cin + ic];
    op.src.resize(cin);
}

}  // namespace fn_kern
