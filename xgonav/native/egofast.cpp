// Ego-Motion-Kern für xgonav/egomotion.py (optischer Fluss + RANSAC + Drehmodell).
//
// Rechnet dasselbe wie EgoMotion.update() in Python, mit denselben OpenCV-Aufrufen und
// Parametern, aber mit EINER Bildpyramide je Bild: Python baut in jedem
// calcOpticalFlowPyrLK-Aufruf beide Pyramiden neu (die Python-Bindung nimmt keine fertigen
// Pyramiden an). Hier wird die Pyramide des aktuellen Bildes (mit Ableitungen) einmal gebaut,
// für die Rückwärtsprüfung genutzt und im nächsten Bild als Vorgänger wiederverwendet.
// CM4: Pyramide mit Ableitungen 0,76 ms, je LK-Aufruf bisher ~1,0 ms nur für Pyramiden.
//
// Schnittstelle: reines C (ctypes), Zustand (vorige Pyramide, Punkte) im Objekt.
// Bauen: make -C xgonav/native   (braucht g++, libopencv-dev, pkg-config)
// Ohne FMA-Zusammenfassung übersetzen (-ffp-contract=off), damit die Gleitkommarechnung
// wie in numpy getrennt multipliziert und addiert.
//
// Abkürzungen (namespace fast), jeweils bitgleich zu den OpenCV-Aufrufen in egomotion.py und
// nur aktiv, wenn eine Selbstprüfung beim ersten ego_new() das am Gerät bestätigt
// (ego_fast_mask()), sonst rechnet OpenCV. CM4 2026-11-05:
//   - Grau + 2:1 verkleinern in einem NEON-Durchgang (cvtColor + resize: 0,8 ms)
//   - Schärfe = Varianz des Laplace ganzzahlig (Laplacian CV_32F + meanStdDev: 0,8 ms)
//   - goodFeaturesToTrack mit eigenem Boxfilter (cornerMinEigenVal summiert 32FC3 in double
//     ohne SIMD: 6,9 von 7,8 ms; die Reihenfolge der Summen ist hier exakt nachgebaut)

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <vector>

namespace {

constexpr int kAbi = 2;   // bei Änderung der Strukturen/Funktionen erhöhen (egomotion.py prüft das)

}  // namespace

// ============================================================================================
// Abkürzungen, bitgleich zu OpenCV 4.6 (Quellen: color_rgb.simd.hpp, resize.cpp, deriv.cpp,
// mean.dispatch.cpp, corner.cpp, box_filter.simd.hpp, featureselect.cpp)
// ============================================================================================
namespace fast {

inline int refl101(int i, int n) { return i < 0 ? -i : (i >= n ? 2 * n - 2 - i : i); }

// ---- Grau + 2:1 (= cvtColor BGR2GRAY, dann resize INTER_AREA auf halbe Größe) -------------
// Grau: Y = (3735 B + 19235 G + 9798 R + 2^14) >> 15 (M = 0, OpenCV) bzw. 14-Bit-Koeffizienten
// (M = 1, falls eine HAL so rechnet). Fläche 2x2 im fast_mode: (a + b + c + d + 2) >> 2.
template <int M>
inline int gray1(const uint8_t* p) {
    return M == 0 ? (p[0] * 3735 + p[1] * 19235 + p[2] * 9798 + (1 << 14)) >> 15
                  : (p[0] * 1868 + p[1] * 9617 + p[2] * 4899 + (1 << 13)) >> 14;
}

#if defined(__aarch64__)
template <int M>
inline uint16x4_t gray4(uint16x4_t b, uint16x4_t g, uint16x4_t r) {
    uint32x4_t a;
    if (M == 0) {
        a = vmull_n_u16(b, 3735);
        a = vmlal_n_u16(a, g, 19235);
        a = vmlal_n_u16(a, r, 9798);
        return vrshrn_n_u32(a, 15);
    }
    a = vmull_n_u16(b, 1868);
    a = vmlal_n_u16(a, g, 9617);
    a = vmlal_n_u16(a, r, 4899);
    return vrshrn_n_u32(a, 14);
}

// 16 BGR-Pixel -> Summen benachbarter Paare (8 Werte)
template <int M>
inline uint16x8_t gray16_pairs(const uint8_t* p) {
    const uint8x16x3_t v = vld3q_u8(p);
    const uint16x8_t bl = vmovl_u8(vget_low_u8(v.val[0])), bh = vmovl_high_u8(v.val[0]);
    const uint16x8_t gl = vmovl_u8(vget_low_u8(v.val[1])), gh = vmovl_high_u8(v.val[1]);
    const uint16x8_t rl = vmovl_u8(vget_low_u8(v.val[2])), rh = vmovl_high_u8(v.val[2]);
    const uint16x8_t lo = vcombine_u16(gray4<M>(vget_low_u16(bl), vget_low_u16(gl), vget_low_u16(rl)),
                                       gray4<M>(vget_high_u16(bl), vget_high_u16(gl), vget_high_u16(rl)));
    const uint16x8_t hi = vcombine_u16(gray4<M>(vget_low_u16(bh), vget_low_u16(gh), vget_low_u16(rh)),
                                       gray4<M>(vget_high_u16(bh), vget_high_u16(gh), vget_high_u16(rh)));
    return vpaddq_u16(lo, hi);
}
#endif

template <int M>
void gray_half_t(const uint8_t* src, int w, int h, size_t stride, uint8_t* dst, size_t dstep) {
    const int dw = w / 2, dh = h / 2;
    for (int y = 0; y < dh; ++y) {
        const uint8_t* r0 = src + static_cast<size_t>(2 * y) * stride;
        const uint8_t* r1 = r0 + stride;
        uint8_t* d = dst + static_cast<size_t>(y) * dstep;
        int x = 0;
#if defined(__aarch64__)
        for (; x + 16 <= dw; x += 16) {
            const uint8_t* a = r0 + x * 6;
            const uint8_t* b = r1 + x * 6;
            const uint16x8_t s0 = vaddq_u16(gray16_pairs<M>(a), gray16_pairs<M>(b));
            const uint16x8_t s1 = vaddq_u16(gray16_pairs<M>(a + 48), gray16_pairs<M>(b + 48));
            vst1q_u8(d + x, vcombine_u8(vrshrn_n_u16(s0, 2), vrshrn_n_u16(s1, 2)));
        }
#endif
        for (; x < dw; ++x) {
            const uint8_t* a = r0 + x * 6;
            const uint8_t* b = r1 + x * 6;
            d[x] = static_cast<uint8_t>((gray1<M>(a) + gray1<M>(a + 3) + gray1<M>(b) + gray1<M>(b + 3) + 2) >> 2);
        }
    }
}

void gray_half(const uint8_t* src, int w, int h, size_t stride, int mode, uint8_t* dst, size_t dstep) {
    if (mode == 0)
        gray_half_t<0>(src, w, h, stride, dst, dstep);
    else
        gray_half_t<1>(src, w, h, stride, dst, dstep);
}

// ---- Schärfe: Varianz von Laplacian(ksize 1, CV_32F, BORDER_REFLECT_101) wie meanStdDev ----
// Der Laplace ist ganzzahlig (|v| <= 1020), Summe und Quadratsumme damit exakt; offen ist nur,
// ob OpenCV die Schlussformel sq*scale - m*m mit FMA übersetzt hat (variant 0..2).
void lap_sums(const uint8_t* g, int w, int h, size_t stride, int64_t& s_out, int64_t& sq_out) {
    int64_t s = 0;
    uint64_t sq = 0;
    for (int y = 0; y < h; ++y) {
        const uint8_t* c = g + static_cast<size_t>(y) * stride;
        const uint8_t* up = g + static_cast<size_t>(refl101(y - 1, h)) * stride;
        const uint8_t* dn = g + static_cast<size_t>(refl101(y + 1, h)) * stride;
        auto px = [&](int x) {
            const int v = up[x] + dn[x] + c[refl101(x - 1, w)] + c[refl101(x + 1, w)] - 4 * c[x];
            s += v;
            sq += static_cast<uint64_t>(v * v);
        };
        px(0);
        int x = 1;
#if defined(__aarch64__)
        int32x4_t vs = vdupq_n_s32(0);
        uint64x2_t vq = vdupq_n_u64(0);
        for (; x + 16 <= w - 1; x += 16) {
            const uint8x16_t l = vld1q_u8(c + x - 1), r = vld1q_u8(c + x + 1), m = vld1q_u8(c + x);
            const uint8x16_t u = vld1q_u8(up + x), d = vld1q_u8(dn + x);
            for (int half = 0; half < 2; ++half) {
                const uint16x8_t sum4 = half == 0
                    ? vaddq_u16(vaddl_u8(vget_low_u8(u), vget_low_u8(d)), vaddl_u8(vget_low_u8(l), vget_low_u8(r)))
                    : vaddq_u16(vaddl_high_u8(u, d), vaddl_high_u8(l, r));
                const uint16x8_t c4 = vshlq_n_u16(half == 0 ? vmovl_u8(vget_low_u8(m)) : vmovl_high_u8(m), 2);
                const int16x8_t v = vsubq_s16(vreinterpretq_s16_u16(sum4), vreinterpretq_s16_u16(c4));
                vs = vpadalq_s16(vs, v);
                const int32x4_t q0 = vmull_s16(vget_low_s16(v), vget_low_s16(v));
                const int32x4_t q1 = vmull_high_s16(v, v);
                vq = vpadalq_u32(vq, vreinterpretq_u32_s32(q0));
                vq = vpadalq_u32(vq, vreinterpretq_u32_s32(q1));
            }
        }
        s += vaddvq_s32(vs);
        sq += vaddvq_u64(vq);
#endif
        for (; x < w; ++x) px(x);
    }
    s_out = s;
    sq_out = static_cast<int64_t>(sq);
}

double lap_var(int64_t s, int64_t sq, int n, int variant) {
    const double scale = 1. / n;
    const double m = static_cast<double>(s) * scale;
    const double q = static_cast<double>(sq);
    double v;
    if (variant == 0)
        v = q * scale - m * m;
    else if (variant == 1)
        v = std::fma(q, scale, -(m * m));
    else
        v = std::fma(-m, m, q * scale);
    const double sd = std::sqrt(std::max(v, 0.));
    return sd * sd;
}

double sharpness(const cv::Mat& gray, int variant) {
    int64_t s, sq;
    lap_sums(gray.data, gray.cols, gray.rows, gray.step, s, sq);
    return lap_var(s, sq, gray.cols * gray.rows, variant);
}

// ---- goodFeaturesToTrack(blockSize 5, Sobel 3, Mindestabstand) ------------------------------
// cornerMinEigenVal: Dx, Dy per cv::Sobel (Skala 1/5100), Produkte in float, Boxfilter 5x5
// unnormiert in double (RowSum ksize 5: Summe von links; ColumnSum: laufende Summe ab den ersten
// vier Zeilen, s0 = SUM + neu, Ausgabe (float)s0, SUM = s0 - alt), Rand REFLECT_101, dann
// (a + c) - sqrt(fma(b, b, (a - c)^2)) wie calcMinEigenVal (NEON: v_muladd = vfmaq_f32).
void eig_cols(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& eig, int x0, int x1);

void min_eigen(const cv::Mat& gray, cv::Mat& eig) {
    const int w = gray.cols, h = gray.rows;
    double scale = static_cast<double>(1 << 2) * 5;
    scale *= 255.0;
    scale = 1.0 / scale;
    // Puffer wiederverwenden: frisch angelegt (> 128 KB, mmap) kosteten die Seitenfehler am CM4 merklich
    // (lokale Referenzen: ein Lambda fängt thread_local nicht ein, im Worker wäre es dessen Kopie)
    static thread_local cv::Mat tl_dx, tl_dy;
    cv::Mat& dx = tl_dx;
    cv::Mat& dy = tl_dy;
    // beide Ableitungen und dann die zwei Spaltenhälften parallel (OpenCV-Threads der
    // Hauptschleife); jeder Wert wird dabei genauso gerechnet wie am Stück
    cv::parallel_for_(cv::Range(0, 2), [&](const cv::Range& r) {
        for (int k = r.start; k < r.end; ++k) {
            if (k == 0)
                cv::Sobel(gray, dx, CV_32F, 1, 0, 3, scale, 0, cv::BORDER_DEFAULT);
            else
                cv::Sobel(gray, dy, CV_32F, 0, 1, 3, scale, 0, cv::BORDER_DEFAULT);
        }
    }, 2);
    eig.create(h, w, CV_32F);
    const int xm = (w / 2) & ~3;
    cv::parallel_for_(cv::Range(0, 2), [&](const cv::Range& r) {
        for (int k = r.start; k < r.end; ++k) eig_cols(dx, dy, eig, k == 0 ? 0 : xm, k == 0 ? xm : w);
    }, 2);
}

// Boxfilter + kleinster Eigenwert für die Spalten [x0, x1) (siehe min_eigen)
void eig_cols(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& eig, int x0, int x1) {
    const int w = dx.cols, h = dx.rows, nx = x1 - x0;
    if (nx <= 0) return;
    const int W3 = nx * 3;
    std::vector<float> prod((nx + 4) * 3);         // Produkte einer Zeile, 2 Pixel Rand je Seite
    std::vector<double> ring(5 * static_cast<size_t>(W3));   // Zeilensummen, Zeile y bei y mod 5
    std::vector<double> sum(W3);
    std::vector<float> cov(W3);
    auto rowsum = [&](int yy, double* __restrict out) {   // yy: Zeile im erweiterten Bild (-2 .. h+1)
        const int sy = refl101(yy, h);
        const float* px = dx.ptr<float>(sy);
        const float* py = dy.ptr<float>(sy);
        float* P = prod.data();
        auto put = [&](int xe) {                   // xe: lokal, 0 = globale Spalte x0 - 2
            const int sx = refl101(x0 - 2 + xe, w);
            const float a = px[sx], b = py[sx];
            P[xe * 3] = a * a;
            P[xe * 3 + 1] = a * b;
            P[xe * 3 + 2] = b * b;
        };
        int xe = 0;
        while (xe < nx + 4 && x0 - 2 + xe < 0) put(xe++);
#if defined(__aarch64__)
        for (; xe + 4 <= nx + 4 && x0 - 2 + xe + 4 <= w; xe += 4) {
            const int gx = x0 - 2 + xe;
            const float32x4_t a = vld1q_f32(px + gx), b = vld1q_f32(py + gx);
            float32x4x3_t v;
            v.val[0] = vmulq_f32(a, a);
            v.val[1] = vmulq_f32(a, b);
            v.val[2] = vmulq_f32(b, b);
            vst3q_f32(P + xe * 3, v);
        }
#endif
        for (; xe < nx + 4; ++xe) put(xe);
        // Summe von links wie RowSum (ksize 5): ((((S0 + S1) + S2) + S3) + S4) je Wert, in double
        const float* S = P;
        int i = 0;
#if defined(__aarch64__)
        for (; i + 4 <= W3; i += 4) {
            const float32x4_t s0 = vld1q_f32(S + i), s1 = vld1q_f32(S + i + 3), s2 = vld1q_f32(S + i + 6);
            const float32x4_t s3 = vld1q_f32(S + i + 9), s4 = vld1q_f32(S + i + 12);
            float64x2_t lo = vaddq_f64(vcvt_f64_f32(vget_low_f32(s0)), vcvt_f64_f32(vget_low_f32(s1)));
            float64x2_t hi = vaddq_f64(vcvt_high_f64_f32(s0), vcvt_high_f64_f32(s1));
            lo = vaddq_f64(lo, vcvt_f64_f32(vget_low_f32(s2)));
            hi = vaddq_f64(hi, vcvt_high_f64_f32(s2));
            lo = vaddq_f64(lo, vcvt_f64_f32(vget_low_f32(s3)));
            hi = vaddq_f64(hi, vcvt_high_f64_f32(s3));
            lo = vaddq_f64(lo, vcvt_f64_f32(vget_low_f32(s4)));
            hi = vaddq_f64(hi, vcvt_high_f64_f32(s4));
            vst1q_f64(out + i, lo);
            vst1q_f64(out + i + 2, hi);
        }
#endif
        for (; i < W3; ++i)
            out[i] = static_cast<double>(S[i]) + static_cast<double>(S[i + 3]) + static_cast<double>(S[i + 6]) +
                     static_cast<double>(S[i + 9]) + static_cast<double>(S[i + 12]);
    };
    auto ringrow = [&](int yy) { return ring.data() + static_cast<size_t>((yy + 5) % 5) * W3; };
    std::fill(sum.begin(), sum.end(), 0.0);
    for (int yy = -2; yy <= 1; ++yy) {
        double* r = ringrow(yy);
        rowsum(yy, r);
        for (int i = 0; i < W3; ++i) sum[i] += r[i];
    }
    for (int y = 0; y < h; ++y) {
        double* rp = ringrow(y + 2);      // Platz der Zeile y - 3, die nicht mehr gebraucht wird
        rowsum(y + 2, rp);
        const double* rm = ringrow(y - 2);
        double* __restrict sm = sum.data();
        float* __restrict cv3 = cov.data();
        int i = 0;
#if defined(__aarch64__)
        for (; i + 4 <= W3; i += 4) {
            const float64x2_t a = vaddq_f64(vld1q_f64(sm + i), vld1q_f64(rp + i));
            const float64x2_t b = vaddq_f64(vld1q_f64(sm + i + 2), vld1q_f64(rp + i + 2));
            vst1q_f32(cv3 + i, vcombine_f32(vcvt_f32_f64(a), vcvt_f32_f64(b)));
            vst1q_f64(sm + i, vsubq_f64(a, vld1q_f64(rm + i)));
            vst1q_f64(sm + i + 2, vsubq_f64(b, vld1q_f64(rm + i + 2)));
        }
#endif
        for (; i < W3; ++i) {
            const double s0 = sm[i] + rp[i];
            cv3[i] = static_cast<float>(s0);
            sm[i] = s0 - rm[i];
        }
        float* e = eig.ptr<float>(y) + x0;
        int x = 0;
#if defined(__aarch64__)
        const float32x4_t half = vdupq_n_f32(0.5f);
        for (; x + 4 <= nx; x += 4) {
            const float32x4x3_t v = vld3q_f32(cov.data() + x * 3);
            const float32x4_t a = vmulq_f32(v.val[0], half), c = vmulq_f32(v.val[2], half);
            const float32x4_t t0 = vsubq_f32(a, c);
            const float32x4_t t = vfmaq_f32(vmulq_f32(t0, t0), v.val[1], v.val[1]);
            vst1q_f32(e + x, vsubq_f32(vaddq_f32(a, c), vsqrtq_f32(t)));
        }
#endif
        for (; x < nx; ++x) {
            const float a = cov[x * 3] * 0.5f, b = cov[x * 3 + 1], c = cov[x * 3 + 2] * 0.5f;
            const float t0 = a - c;
            e[x] = (a + c) - std::sqrt(std::fma(b, b, t0 * t0));
        }
    }
}

// Rest von goodFeaturesToTrack (featureselect.cpp): Schwelle maxVal*quality (Maske), lokale
// Maxima 3x3 nach Schwelle, absteigend sortiert (gleiche Werte: höhere Adresse zuerst),
// Mindestabstand über ein Gitter, höchstens maxc Punkte.
void good_features(const cv::Mat& gray, const cv::Mat& mask, int maxc, double quality, double min_dist,
                   std::vector<cv::Point2f>& out) {
    out.clear();
    static thread_local cv::Mat eig;
    min_eigen(gray, eig);
    const int w = gray.cols, h = gray.rows;
    // maxVal wie minMaxLoc(eig, mask): Maximum der Pixel mit Maske != 0, ohne solche 0
    float mx = -INFINITY;
    bool any = false;
    for (int y = 0; y < h; ++y) {
        const float* e = eig.ptr<float>(y);
        const uint8_t* m = mask.empty() ? nullptr : mask.ptr<uint8_t>(y);
        for (int x = 0; x < w; ++x)
            if (!m || m[x]) {
                any = true;
                mx = std::max(mx, e[x]);
            }
    }
    const double maxv = any ? static_cast<double>(mx) : 0.0;
    const float thr = static_cast<float>(maxv * quality);
    // threshold(THRESH_TOZERO) und dilate 3x3: T = e > thr ? e : 0, D = Maximum von T im 3x3;
    // zeilenweise rollierend (T und waagrechtes Maximum H je 3 Zeilen)
    std::vector<float> tb(3 * static_cast<size_t>(w)), hb(3 * static_cast<size_t>(w), 0.f);
    auto prep = [&](int y) {
        const float* e = eig.ptr<float>(y);
        float* t = tb.data() + static_cast<size_t>(y % 3) * w;
        float* hm = hb.data() + static_cast<size_t>(y % 3) * w;
        int x = 0;
#if defined(__aarch64__)
        const float32x4_t vt = vdupq_n_f32(thr), z = vdupq_n_f32(0.f);
        for (; x + 4 <= w; x += 4) {
            const float32x4_t v = vld1q_f32(e + x);
            vst1q_f32(t + x, vbslq_f32(vcgtq_f32(v, vt), v, z));
        }
#endif
        for (; x < w; ++x) t[x] = e[x] > thr ? e[x] : 0.f;
        for (x = 1; x < w - 1; ++x) hm[x] = std::max(std::max(t[x - 1], t[x]), t[x + 1]);
    };
    std::vector<std::pair<float, int>> cand;
    cand.reserve(2048);
    if (h >= 3) {
        prep(0);
        prep(1);
    }
    for (int y = 1; y < h - 1; ++y) {
        prep(y + 1);
        const float* t = tb.data() + static_cast<size_t>(y % 3) * w;
        const float* h0 = hb.data() + static_cast<size_t>((y + 2) % 3) * w;
        const float* h1 = hb.data() + static_cast<size_t>(y % 3) * w;
        const float* h2 = hb.data() + static_cast<size_t>((y + 1) % 3) * w;
        const uint8_t* m = mask.empty() ? nullptr : mask.ptr<uint8_t>(y);
        for (int x = 1; x < w - 1; ++x) {
            const float v = t[x];
            if (v == 0 || (m && !m[x])) continue;
            if (v == std::max(std::max(h0[x], h1[x]), h2[x])) cand.emplace_back(v, y * w + x);
        }
    }
    std::sort(cand.begin(), cand.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
        return a.first > b.first || (!(a.first < b.first) && a.second > b.second);
    });
    if (min_dist >= 1) {
        const int cell = cvRound(min_dist);
        const int gw = (w + cell - 1) / cell, gh = (h + cell - 1) / cell;
        std::vector<std::vector<cv::Point2f>> grid(static_cast<size_t>(gw) * gh);
        const double md2 = min_dist * min_dist;
        for (const auto& cd : cand) {
            const int y = cd.second / w, x = cd.second - y * w;
            const int xc = x / cell, yc = y / cell;
            bool good = true;
            for (int yy = std::max(0, yc - 1); good && yy <= std::min(gh - 1, yc + 1); ++yy)
                for (int xx = std::max(0, xc - 1); good && xx <= std::min(gw - 1, xc + 1); ++xx)
                    for (const auto& q : grid[static_cast<size_t>(yy) * gw + xx]) {
                        const float ddx = x - q.x, ddy = y - q.y;
                        if (ddx * ddx + ddy * ddy < md2) {
                            good = false;
                            break;
                        }
                    }
            if (good) {
                grid[static_cast<size_t>(yc) * gw + xc].emplace_back(static_cast<float>(x), static_cast<float>(y));
                out.emplace_back(static_cast<float>(x), static_cast<float>(y));
                if (maxc > 0 && static_cast<int>(out.size()) == maxc) break;
            }
        }
    } else {
        for (const auto& cd : cand) {
            const int y = cd.second / w, x = cd.second - y * w;
            out.emplace_back(static_cast<float>(x), static_cast<float>(y));
            if (maxc > 0 && static_cast<int>(out.size()) == maxc) break;
        }
    }
}

// ---- Selbstprüfung -------------------------------------------------------------------------
struct Modes {
    int gray = -1;    // Grau-Variante, -1 = OpenCV
    int sharp = -1;   // Schlussformel der Varianz, -1 = OpenCV
    bool gftt = false;
};

// Testbilder (deterministisch): Rauschen, geglättet, Streifen (erzeugt winzige Sobel-Reste)
std::vector<cv::Mat> test_images(int w, int h, int type) {
    std::vector<cv::Mat> v;
    cv::RNG rng(0x5eed);
    cv::Mat a(h, w, type);
    rng.fill(a, cv::RNG::UNIFORM, 0, 256);
    v.push_back(a);
    cv::Mat b;
    cv::GaussianBlur(a, b, cv::Size(0, 0), 2.0);
    v.push_back(b);
    cv::Mat c(h, w, type);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w * c.channels(); ++x)
            c.ptr<uint8_t>(y)[x] = static_cast<uint8_t>(((y % 2) ? 40 : 200) + ((x / 7) % 5) * 9 + rng.uniform(0, 3));
    v.push_back(c);
    cv::Mat d;
    cv::GaussianBlur(a, d, cv::Size(0, 0), 0.8);
    cv::addWeighted(d, 0.5, c, 0.5, 0, d);
    v.push_back(d);
    return v;
}

Modes check(int pw, int ph) {
    Modes m;
    // Grau + 2:1
    const std::vector<cv::Mat> bgr = test_images(pw * 2, ph * 2, CV_8UC3);
    for (int mode = 0; mode < 2 && m.gray < 0; ++mode) {
        bool ok = true;
        for (const auto& img : bgr) {
            cv::Mat g, ref, out(ph, pw, CV_8U);
            cv::cvtColor(img, g, cv::COLOR_BGR2GRAY);
            cv::resize(g, ref, cv::Size(pw, ph), 0, 0, cv::INTER_AREA);
            gray_half(img.data, img.cols, img.rows, img.step, mode, out.data, out.step);
            ok = ok && cv::countNonZero(out != ref) == 0;
        }
        if (ok) m.gray = mode;
    }
    const std::vector<cv::Mat> gray = test_images(pw, ph, CV_8UC1);
    // Schärfe: erst die Schlussformel an Zufallsfeldern bestimmen (auf Bildern unterscheiden sich
    // die Varianten nur selten: RiderPi 19 von 150 Bildern), dann am Bild prüfen
    {
        cv::RNG rng(77);
        bool vok[3] = {true, true, true};
        for (int t = 0; t < 200; ++t) {
            const int n = 500 + 37 * t;
            cv::Mat a(1, n, CV_32F);
            int64_t s = 0, sq = 0;
            for (int i = 0; i < n; ++i) {
                const int v = rng.uniform(-1020, 1021);
                a.at<float>(i) = static_cast<float>(v);
                s += v;
                sq += static_cast<int64_t>(v) * v;
            }
            cv::Scalar mean, sd;
            cv::meanStdDev(a, mean, sd);
            for (int var = 0; var < 3; ++var) vok[var] = vok[var] && lap_var(s, sq, n, var) == sd[0] * sd[0];
        }
        for (int var = 0; var < 3 && m.sharp < 0; ++var)
            if (vok[var]) m.sharp = var;
    }
    if (m.sharp >= 0) {
        for (const auto& g : gray) {
            cv::Mat lap;
            cv::Laplacian(g, lap, CV_32F);
            cv::Scalar mean, sd;
            cv::meanStdDev(lap, mean, sd);
            if (sharpness(g, m.sharp) != sd[0] * sd[0]) m.sharp = -1;
        }
    }
    // Merkmale: Eigenwerte bitgleich, Punkte gleich (mit und ohne Maske)
    bool ok = true;
    for (const auto& g : gray) {
        cv::Mat ref, eig;
        cv::cornerMinEigenVal(g, ref, 5, 3);
        min_eigen(g, eig);
        ok = ok && std::memcmp(ref.data, eig.data, ref.total() * sizeof(float)) == 0;
        cv::Mat mask(g.size(), CV_8U, cv::Scalar(255));
        cv::rectangle(mask, cv::Point(pw / 4, ph / 4), cv::Point(pw / 2, ph / 2), cv::Scalar(0), cv::FILLED);
        cv::circle(mask, cv::Point(pw * 3 / 4, ph / 2), 20, cv::Scalar(0), cv::FILLED);
        for (const cv::Mat& mk : {cv::Mat(), mask}) {
            std::vector<cv::Point2f> a, b;
            cv::goodFeaturesToTrack(g, a, 90, 0.01, 8, mk, 5);
            good_features(g, mk, 90, 0.01, 8, b);
            ok = ok && a == b;
        }
    }
    m.gftt = ok;
    return m;
}

const Modes& modes(int pw, int ph) {
    static const Modes m = check(pw, ph);   // einmal je Prozess (Arbeitsauflösung ist fest)
    return m;
}

}  // namespace fast

extern "C" {

// muss zu _EgoParams in egomotion.py passen
struct EgoParams {
    int pw, ph, full_w, full_h;
    int lk_win, lk_levels, lk_iters, lk_fb_check;
    int max_features, min_features, min_distance, pad0;
    double s, ransac_px, quality, eps;
    double K[9], Ki[9], Hd[27];   // aus Python (numpy), damit beide Wege dieselben Zahlen nutzen
};

// muss zu _EgoOut in egomotion.py passen
struct EgoOut {
    int tracked;     // 1: Fluss + Modell bis zum Ende gerechnet (A, yaw, ... gültig)
    int inliers;
    int npts;        // Punkte für das nächste Bild
    int pad0;
    double sharp;
    double yaw, pitch, roll, sc;
    double dx, dy;
    double exp_inst; // Expansion dieses Bildes (1/s, >= 0), nur wenn tracked
    double A[9];     // prev -> cur, volle Auflösung, zeilenweise
};

}  // extern "C"

namespace {

void mul3(const double* a, const double* b, double* out) {
    double r[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r[i * 3 + j] = a[i * 3 + 0] * b[0 * 3 + j] + a[i * 3 + 1] * b[1 * 3 + j] + a[i * 3 + 2] * b[2 * 3 + j];
    std::copy(r, r + 9, out);
}

class Ego {
public:
    explicit Ego(const EgoParams& p) : p_(p), m_(fast::modes(p.pw, p.ph)) {}

    void step(const cv::Mat& bgr, double dt, const double* boxes, int nboxes, EgoOut& o) {
        o = EgoOut{};
        for (int i = 0; i < 9; ++i) o.A[i] = (i % 4 == 0) ? 1.0 : 0.0;
        // wie Python: erst grau, dann verkleinern
        cv::Mat gray;
        if (m_.gray >= 0 && bgr.cols == 2 * p_.pw && bgr.rows == 2 * p_.ph) {
            gray.create(p_.ph, p_.pw, CV_8U);
            fast::gray_half(bgr.data, bgr.cols, bgr.rows, bgr.step, m_.gray, gray.data, gray.step);
        } else {
            cv::Mat g;
            cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
            cv::resize(g, gray, cv::Size(p_.pw, p_.ph), 0, 0, cv::INTER_AREA);
        }
        if (m_.sharp >= 0) {
            o.sharp = fast::sharpness(gray, m_.sharp);
        } else {
            cv::Mat lap;
            cv::Laplacian(gray, lap, CV_32F);
            cv::Scalar mean, sd;
            cv::meanStdDev(lap, mean, sd);
            o.sharp = sd[0] * sd[0];
        }

        std::vector<cv::Mat> pyr;
        cv::buildOpticalFlowPyramid(gray, pyr, cv::Size(p_.lk_win, p_.lk_win), p_.lk_levels, true);
        if (has_prev_ && pts_.size() >= 8 && dt > 0) track(pyr, dt, o);
        replenish(gray, boxes, nboxes);
        prev_pyr_ = std::move(pyr);
        has_prev_ = true;
        o.npts = static_cast<int>(pts_.size());
    }

private:
    void track(const std::vector<cv::Mat>& pyr, double dt, EgoOut& o) {
        const cv::Size win(p_.lk_win, p_.lk_win);
        const cv::TermCriteria crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, p_.lk_iters, 0.03);
        std::vector<cv::Point2f> p1, p0r;
        std::vector<uchar> st, st2;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_pyr_, pyr, pts_, p1, st, err, win, p_.lk_levels, crit);
        const size_t n = pts_.size();
        std::vector<char> good(n);
        for (size_t i = 0; i < n; ++i) good[i] = st[i] == 1;
        if (p_.lk_fb_check) {
            cv::calcOpticalFlowPyrLK(pyr, prev_pyr_, p1, p0r, st2, err, win, p_.lk_levels, crit);
            for (size_t i = 0; i < n; ++i) {
                const float ex = pts_[i].x - p0r[i].x, ey = pts_[i].y - p0r[i].y;
                const float e2 = ex * ex + ey * ey;
                good[i] = good[i] && st2[i] == 1 && e2 < 1.0f;
            }
        }
        std::vector<cv::Point2f> a, b;
        a.reserve(n);
        b.reserve(n);
        for (size_t i = 0; i < n; ++i)
            if (good[i]) {
                a.push_back(pts_[i]);
                b.push_back(p1[i]);
            }
        if (a.size() < 8) {
            pts_ = b;
            return;
        }
        std::vector<uchar> inl_r;
        cv::Mat M = cv::estimateAffinePartial2D(a, b, inl_r, cv::RANSAC, p_.ransac_px, 200, 0.98);
        if (M.empty()) {
            pts_ = b;
            return;
        }
        std::vector<char> m(a.size());
        for (size_t i = 0; i < a.size(); ++i) m[i] = inl_r[i] != 0;
        double q[4];
        fit_rotation(a, b, m, q);

        double Hs[9];
        model_h(q[0], q[1], q[2], q[3], Hs);
        // A = S @ Hs @ Si
        const double S[9] = {p_.s, 0, 0, 0, p_.s, 0, 0, 0, 1.0};
        const double Si[9] = {1 / p_.s, 0, 0, 0, 1 / p_.s, 0, 0, 0, 1.0};
        double T[9];
        mul3(S, Hs, T);
        mul3(T, Si, o.A);
        int ninl = 0;
        for (char v : m) ninl += v ? 1 : 0;
        o.inliers = ninl;
        o.yaw = q[0];
        o.pitch = q[1];
        o.roll = q[2];
        o.sc = q[3];
        const double cx = p_.full_w / 2.0, cy = p_.full_h / 2.0;
        const double X = o.A[0] * cx + o.A[1] * cy + o.A[2];
        const double Y = o.A[3] * cx + o.A[4] * cy + o.A[5];
        const double Z = o.A[6] * cx + o.A[7] * cy + o.A[8];
        o.dx = X / Z - cx;
        o.dy = Y / Z - cy;

        std::vector<cv::Point2f> ai, bi;
        for (size_t i = 0; i < a.size(); ++i)
            if (m[i]) {
                ai.push_back(a[i]);
                bi.push_back(b[i]);
            }
        o.exp_inst = expansion(ai, bi, dt);
        o.tracked = 1;
        pts_ = bi;   // nur Inlier weiterverfolgen (Außenseiter = bewegte Objekte)
    }

    // Kameradrehung wie rot_matrix() in egomotion.py
    static void rot_matrix(double yaw, double pitch, double roll, double* R) {
        const double cy = std::cos(-yaw), sy = std::sin(-yaw);
        const double cp = std::cos(pitch), sp = std::sin(pitch);
        const double cr = std::cos(roll), sr = std::sin(roll);
        const double Ry[9] = {cy, 0, sy, 0, 1, 0, -sy, 0, cy};
        const double Rx[9] = {1, 0, 0, 0, cp, -sp, 0, sp, cp};
        const double Rz[9] = {cr, -sr, 0, sr, cr, 0, 0, 0, 1};
        double T[9];
        mul3(Ry, Rx, T);
        mul3(T, Rz, R);
    }

    // wie EgoMotion._model_h: E @ (K @ R^T @ Ki)
    void model_h(double yaw, double pitch, double roll, double sc, double* H) const {
        double R[9], Rt[9], T[9], Hr[9];
        rot_matrix(yaw, pitch, roll, R);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) Rt[i * 3 + j] = R[j * 3 + i];
        mul3(p_.K, Rt, T);
        mul3(T, p_.Ki, Hr);
        const double cx = p_.pw / 2.0, cy = p_.ph / 2.0;
        const double E[9] = {1 + sc, 0, -sc * cx, 0, 1 + sc, -sc * cy, 0, 0, 1.0};
        mul3(E, Hr, H);
    }

    // wie EgoMotion._fit_rotation: lineares LS für (yaw, pitch, roll, expansion), 2 Durchgänge
    void fit_rotation(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b,
                      std::vector<char>& m, double* q) const {
        const size_t n = a.size();
        const double eps = p_.eps;
        const double cx = p_.pw / 2.0, cy = p_.ph / 2.0;
        std::vector<double> J(n * 2 * 4);   // Zeilen x0,y0,x1,y1,... ; Spalten yaw,pitch,roll,sc
        std::vector<double> d(n * 2);
        for (size_t i = 0; i < n; ++i) {
            const double px = a[i].x, py = a[i].y;
            for (int k = 0; k < 3; ++k) {
                const double* H = p_.Hd + 9 * k;
                const double Q0 = H[0] * px + H[1] * py + H[2] * 1.0;
                const double Q1 = H[3] * px + H[4] * py + H[5] * 1.0;
                const double Q2 = H[6] * px + H[7] * py + H[8] * 1.0;
                J[(2 * i) * 4 + k] = ((Q0 / Q2) - px) / eps;
                J[(2 * i + 1) * 4 + k] = ((Q1 / Q2) - py) / eps;
            }
            J[(2 * i) * 4 + 3] = px - cx;
            J[(2 * i + 1) * 4 + 3] = py - cy;
            d[2 * i] = static_cast<double>(b[i].x - a[i].x);       // wie numpy: Differenz in float32
            d[2 * i + 1] = static_cast<double>(b[i].y - a[i].y);
        }
        q[0] = q[1] = q[2] = q[3] = 0.0;
        const double thr = std::max(p_.ransac_px, 1.0);
        const double thr2 = thr * thr;
        for (int it = 0; it < 2; ++it) {
            int cnt = 0;
            for (char v : m) cnt += v ? 1 : 0;
            if (cnt < 6) break;
            // Normalgleichungen J^T J q = J^T d über die markierten Punkte
            cv::Matx44d JtJ = cv::Matx44d::zeros();
            cv::Vec4d Jtd(0, 0, 0, 0);
            for (size_t i = 0; i < n; ++i) {
                if (!m[i]) continue;
                for (int rr = 0; rr < 2; ++rr) {
                    const double* row = &J[(2 * i + rr) * 4];
                    const double di = d[2 * i + rr];
                    for (int u = 0; u < 4; ++u) {
                        Jtd[u] += row[u] * di;
                        for (int v = 0; v < 4; ++v) JtJ(u, v) += row[u] * row[v];
                    }
                }
            }
            cv::Vec4d sol;
            if (!cv::solve(JtJ, Jtd, sol, cv::DECOMP_LU)) {
                // singulär: Ausgleich per SVD über die markierten Zeilen (wie lstsq)
                cv::Mat Jm(0, 4, CV_64F), dm(0, 1, CV_64F);
                for (size_t i = 0; i < n; ++i) {
                    if (!m[i]) continue;
                    for (int rr = 0; rr < 2; ++rr) {
                        Jm.push_back(cv::Mat(1, 4, CV_64F, const_cast<double*>(&J[(2 * i + rr) * 4])).clone());
                        dm.push_back(d[2 * i + rr]);
                    }
                }
                cv::Mat s;
                cv::solve(Jm, dm, s, cv::DECOMP_SVD);
                sol = cv::Vec4d(s.at<double>(0), s.at<double>(1), s.at<double>(2), s.at<double>(3));
            }
            for (int u = 0; u < 4; ++u) q[u] = sol[u];
            for (size_t i = 0; i < n; ++i) {
                const double* r0 = &J[(2 * i) * 4];
                const double* r1 = &J[(2 * i + 1) * 4];
                const double e0 = (r0[0] * q[0] + r0[1] * q[1] + r0[2] * q[2] + r0[3] * q[3]) - d[2 * i];
                const double e1 = (r1[0] * q[0] + r1[1] * q[1] + r1[2] * q[2] + r1[3] * q[3]) - d[2 * i + 1];
                m[i] = e0 * e0 + e1 * e1 < thr2;
            }
        }
    }

    // wie EgoMotion._expansion (ohne Glättung, die bleibt in Python)
    double expansion(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b, double dt) const {
        const float cx = static_cast<float>(p_.pw / 2.0), cy = static_cast<float>(p_.ph / 2.0);
        const float lx = static_cast<float>(p_.pw * 0.25), ly = static_cast<float>(p_.ph * 0.3);
        std::vector<cv::Point2f> ac, bc;
        for (size_t i = 0; i < a.size(); ++i)
            if (std::fabs(a[i].x - cx) < lx && std::fabs(a[i].y - cy) < ly) {
                ac.push_back(a[i]);
                bc.push_back(b[i]);
            }
        if (ac.size() < 6) return 0.0;
        cv::Mat Mc = cv::estimateAffinePartial2D(ac, bc, cv::noArray(), cv::LMEDS);
        if (Mc.empty()) return 0.0;
        const double sc = std::hypot(Mc.at<double>(0, 0), Mc.at<double>(1, 0));
        return std::max(0.0, (sc - 1.0) / dt);
    }

    // wie EgoMotion._replenish
    void replenish(const cv::Mat& gray, const double* boxes, int nboxes) {
        const size_t n = pts_.size();
        if (static_cast<int>(n) >= p_.min_features) return;
        cv::Mat mask(gray.size(), CV_8U, cv::Scalar(255));
        for (int i = 0; i < nboxes; ++i) {
            const double* bx = boxes + 4 * i;
            cv::rectangle(mask, cv::Point(static_cast<int>(bx[0] / p_.s), static_cast<int>(bx[1] / p_.s)),
                          cv::Point(static_cast<int>(bx[2] / p_.s), static_cast<int>(bx[3] / p_.s)), cv::Scalar(0),
                          cv::FILLED);
        }
        for (const auto& p : pts_)
            cv::circle(mask, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), p_.min_distance, cv::Scalar(0),
                       cv::FILLED);
        std::vector<cv::Point2f> fresh;
        if (m_.gftt)
            fast::good_features(gray, mask, p_.max_features - static_cast<int>(n), p_.quality, p_.min_distance, fresh);
        else
            cv::goodFeaturesToTrack(gray, fresh, p_.max_features - static_cast<int>(n), p_.quality, p_.min_distance,
                                    mask, 5);
        pts_.insert(pts_.end(), fresh.begin(), fresh.end());
    }

    EgoParams p_;
    const fast::Modes& m_;
    bool has_prev_ = false;
    std::vector<cv::Mat> prev_pyr_;
    std::vector<cv::Point2f> pts_;
};

}  // namespace

extern "C" {

int ego_abi(void) { return kAbi; }

void* ego_new(const EgoParams* p) {
    try {
        return new Ego(*p);
    } catch (...) {
        return nullptr;
    }
}

// Aktive Abkürzungen nach der Selbstprüfung (Arbeitsauflösung pw x ph):
// Bit 0 Grau + 2:1, Bit 1 Schärfe, Bit 2 Merkmale. -1 = Fehler.
int ego_fast_mask(int pw, int ph) {
    try {
        const fast::Modes& m = fast::modes(pw, ph);
        return (m.gray >= 0 ? 1 : 0) | (m.sharp >= 0 ? 2 : 0) | (m.gftt ? 4 : 0);
    } catch (...) {
        return -1;
    }
}

// Für Tests: die Abkürzungen einzeln (Rückgabe -1, wenn am Gerät nicht bitgleich zu OpenCV).
int xg_gray_half(const uint8_t* bgr, int w, int h, int stride, uint8_t* out) {
    const fast::Modes& m = fast::modes(w / 2, h / 2);
    if (m.gray < 0 || w % 2 || h % 2) return -1;
    fast::gray_half(bgr, w, h, static_cast<size_t>(stride), m.gray, out, static_cast<size_t>(w / 2));
    return 0;
}

int xg_sharpness(const uint8_t* g, int w, int h, int stride, double* out) {
    const fast::Modes& m = fast::modes(w, h);
    if (m.sharp < 0) return -1;
    const cv::Mat img(h, w, CV_8U, const_cast<uint8_t*>(g), static_cast<size_t>(stride));
    *out = fast::sharpness(img, m.sharp);
    return 0;
}

// Punkte als x, y (float) nach out_xy, höchstens cap; Rückgabe Anzahl oder -1.
int xg_good_features(const uint8_t* g, int w, int h, int stride, const uint8_t* mask, int maxc, double quality,
                     double min_dist, float* out_xy, int cap) {
    try {
        if (!fast::modes(w, h).gftt) return -1;
        const cv::Mat img(h, w, CV_8U, const_cast<uint8_t*>(g), static_cast<size_t>(stride));
        cv::Mat mk;
        if (mask) mk = cv::Mat(h, w, CV_8U, const_cast<uint8_t*>(mask));
        std::vector<cv::Point2f> pts;
        fast::good_features(img, mk, maxc, quality, min_dist, pts);
        const int n = std::min(cap, static_cast<int>(pts.size()));
        for (int i = 0; i < n; ++i) {
            out_xy[2 * i] = pts[i].x;
            out_xy[2 * i + 1] = pts[i].y;
        }
        return n;
    } catch (const std::exception&) {
        return -1;
    }
}

void ego_free(void* h) { delete static_cast<Ego*>(h); }

// bgr: 8-Bit BGR, Zeilenabstand stride (Bytes). boxes: nboxes x (x1, y1, x2, y2), Vollbild.
// Rückgabe 0 = ok, -1 = Fehler (Ausnahme in OpenCV; Zustand unverändert bis auf Teilschritte).
int ego_step(void* h, const uint8_t* bgr, int w, int hgt, int stride, double dt, const double* boxes, int nboxes,
             EgoOut* out) {
    try {
        const cv::Mat img(hgt, w, CV_8UC3, const_cast<uint8_t*>(bgr), static_cast<size_t>(stride));
        static_cast<Ego*>(h)->step(img, dt, boxes, nboxes, *out);
        return 0;
    } catch (const std::exception&) {
        return -1;
    }
}

}  // extern "C"
