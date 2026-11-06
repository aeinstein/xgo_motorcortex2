// Prüfhilfe: die NEON-Befehle von xgonav/native/floornet_kern.h als einfache C++-Funktionen, damit der Kern
// auch ohne ARM läuft (tests/test_floornet_native.py). Langsam, nur fürs Rechenergebnis, nie für den Betrieb.
#pragma once
#include <algorithm>
#include <cmath>

struct float32x4_t { float v[4]; };
struct float32x4x2_t { float32x4_t val[2]; };

inline float32x4_t vld1q_f32(const float* p) { return {{p[0], p[1], p[2], p[3]}}; }
inline void vst1q_f32(float* p, float32x4_t a) { for (int i = 0; i < 4; ++i) p[i] = a.v[i]; }
inline float32x4_t vdupq_n_f32(float x) { return {{x, x, x, x}}; }
inline float32x4_t vld1q_dup_f32(const float* p) { return vdupq_n_f32(*p); }
inline float32x4_t vdupq_laneq_f32(float32x4_t a, int l) { return vdupq_n_f32(a.v[l]); }

inline float32x4_t vaddq_f32(float32x4_t a, float32x4_t b) {
    return {{a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2], a.v[3] + b.v[3]}};
}
inline float32x4_t vmulq_f32(float32x4_t a, float32x4_t b) {
    return {{a.v[0] * b.v[0], a.v[1] * b.v[1], a.v[2] * b.v[2], a.v[3] * b.v[3]}};
}
inline float32x4_t vmaxq_f32(float32x4_t a, float32x4_t b) {
    return {{std::max(a.v[0], b.v[0]), std::max(a.v[1], b.v[1]), std::max(a.v[2], b.v[2]), std::max(a.v[3], b.v[3])}};
}
// a + b * c, einmal gerundet wie FMLA
inline float32x4_t vfmaq_f32(float32x4_t a, float32x4_t b, float32x4_t c) {
    return {{std::fma(b.v[0], c.v[0], a.v[0]), std::fma(b.v[1], c.v[1], a.v[1]),
             std::fma(b.v[2], c.v[2], a.v[2]), std::fma(b.v[3], c.v[3], a.v[3])}};
}
inline float32x4_t vfmaq_laneq_f32(float32x4_t a, float32x4_t b, float32x4_t c, int l) {
    return vfmaq_f32(a, b, vdupq_n_f32(c.v[l]));
}
// Vier Werte ab Stelle n aus der Folge a, b
inline float32x4_t vextq_f32(float32x4_t a, float32x4_t b, int n) {
    float32x4_t r;
    for (int i = 0; i < 4; ++i) r.v[i] = i + n < 4 ? a.v[i + n] : b.v[i + n - 4];
    return r;
}
// Acht Werte abwechselnd auf zwei Register verteilen
inline float32x4x2_t vld2q_f32(const float* p) {
    return {{{{p[0], p[2], p[4], p[6]}}, {{p[1], p[3], p[5], p[7]}}}};
}
