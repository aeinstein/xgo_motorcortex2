// Kantenverfahren des Bodenmodells (FreeSpaceEstimator._edge_columns + _glare_cols) in C++.
//
// Rechnet dasselbe wie die numpy-Fassung in xgonav/freespace.py (gleiche OpenCV-Aufrufe,
// Ganzzahl-Summen, Vergleiche in double wie numpy), ohne die vielen Zwischenarrays.
// Unterschiede höchstens bei Werten exakt auf einer Schwelle (Rundung der gewichteten Summe).
// Übersetzen ohne FMA-Zusammenfassung (-ffp-contract=off, siehe Makefile).

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <limits>
#include <vector>

namespace {
constexpr int kEdgeAbi = 2;   // bei Änderung von EdgeParams/Funktionen erhöhen (freespace.py prüft das)
}

extern "C" {

// muss zu _EdgeParams in freespace.py passen
struct EdgeParams {
    int H, W, k, edge_thin;
    int use_log, edge_glare, glare_v, glare_px;
    int glare_spread, pad0, pad1, pad2;
    double step_thr, seg_thr, chroma_w, glare_dl;   // chroma_w bereits auf float32 gerundet
    uint8_t lut[256];                                // Log-Helligkeit (aus Python, gleiche Rundung)
};

int edge_abi(void) { return kEdgeAbi; }

}  // extern "C"

#if defined(__aarch64__)
#include <arm_neon.h>
namespace {
// Faktor 4, 3 Kanäle, dw Vielfaches von 8: je 32 Eingabepixel (4 Zeilen) -> 8 Ausgabepixel.
// vld3q trennt BGR, vpaddlq_u8 addiert waagrechte Paare, dann 4 Zeilen, vpaddq_u16 die zweite
// Paarstufe -> Summe über 4x4. Runden ganzzahlig: Modus 0 = rint(s/16) (halb zur geraden Zahl,
// gleich lrintf(float(s) * 0.0625f), da exakt darstellbar), Modus 1 = (s + 8) >> 4.
void area4_bgr_neon(const uint8_t* src, int stride, int dw, int dh, int round_mode, uint8_t* dst) {
    const uint16x8_t seven = vdupq_n_u16(7), eight = vdupq_n_u16(8), one = vdupq_n_u16(1);
    for (int dy = 0; dy < dh; ++dy) {
        const uint8_t* r0 = src + static_cast<size_t>(dy * 4) * stride;
        const uint8_t* r1 = r0 + stride;
        const uint8_t* r2 = r1 + stride;
        const uint8_t* r3 = r2 + stride;
        uint8_t* d = dst + static_cast<size_t>(dy) * dw * 3;
        for (int dx = 0; dx < dw; dx += 8) {
            const int o = dx * 12;
            uint16x8_t half[2][3];
            for (int h = 0; h < 2; ++h) {
                const int oo = o + h * 48;
                const uint8x16x3_t a = vld3q_u8(r0 + oo), b = vld3q_u8(r1 + oo);
                const uint8x16x3_t c = vld3q_u8(r2 + oo), e = vld3q_u8(r3 + oo);
                for (int ch = 0; ch < 3; ++ch)
                    half[h][ch] = vaddq_u16(vaddq_u16(vpaddlq_u8(a.val[ch]), vpaddlq_u8(b.val[ch])),
                                            vaddq_u16(vpaddlq_u8(c.val[ch]), vpaddlq_u8(e.val[ch])));
            }
            uint8x8x3_t out;
            for (int ch = 0; ch < 3; ++ch) {
                const uint16x8_t s = vpaddq_u16(half[0][ch], half[1][ch]);
                const uint16x8_t r = round_mode == 0
                                         ? vaddq_u16(vaddq_u16(s, seven), vandq_u16(vshrq_n_u16(s, 4), one))
                                         : vaddq_u16(s, eight);
                out.val[ch] = vmovn_u16(vshrq_n_u16(r, 4));
            }
            vst3_u8(d + dx * 3, out);
        }
    }
}
}  // namespace
#endif

extern "C" {

// Verkleinern um den ganzzahligen Faktor f (Mittel über f x f Pixel) wie cv::resize(INTER_AREA).
// OpenCV rechnet bei Faktor != 2 ohne SIMD über eine Indextabelle (CM4: 2,5 ms für 640x480 -> 160x120).
// round_mode 0: wie OpenCV generisch (float(Summe) * (1/f²), lrintf = zur geraden Zahl),
//            1: halb aufrunden ((Summe + f²/2) / f²). Welche Variante cv::resize am Gerät entspricht,
// prüft freespace.py beim Start; ist keine bitgleich, bleibt es bei cv2.resize.
int area_down(const uint8_t* src, int sw, int sh, int stride, int cn, int f, int round_mode, uint8_t* dst) {
    if (f < 1 || sw % f || sh % f || cn < 1 || cn > 4) return -1;
    const int dw = sw / f, dh = sh / f, area = f * f, rowlen = sw * cn;
#if defined(__aarch64__)
    if (f == 4 && cn == 3 && dw % 8 == 0) {
        area4_bgr_neon(src, stride, dw, dh, round_mode, dst);
        return 0;
    }
#endif
    const float scale = 1.f / static_cast<float>(area);
    std::vector<uint32_t> acc(static_cast<size_t>(rowlen));
    for (int dy = 0; dy < dh; ++dy) {
        std::fill(acc.begin(), acc.end(), 0u);
        for (int k = 0; k < f; ++k) {
            const uint8_t* s = src + static_cast<size_t>(dy * f + k) * stride;
            for (int i = 0; i < rowlen; ++i) acc[i] += s[i];
        }
        uint8_t* d = dst + static_cast<size_t>(dy) * dw * cn;
        for (int dx = 0; dx < dw; ++dx) {
            for (int ch = 0; ch < cn; ++ch) {
                uint32_t sum = 0;
                for (int j = 0; j < f; ++j) sum += acc[(dx * f + j) * cn + ch];
                int v;
                if (round_mode == 0)
                    v = static_cast<int>(std::lrintf(static_cast<float>(sum) * scale));
                else
                    v = static_cast<int>((sum + static_cast<uint32_t>(area / 2)) / static_cast<uint32_t>(area));
                d[dx * cn + ch] = static_cast<uint8_t>(std::min(std::max(v, 0), 255));
            }
        }
    }
    return 0;
}

}  // extern "C"

namespace {

// _glare_cols: Spalten, deren unterste Stufe ein Lichtschein ist
void glare_cols(const EdgeParams& p, const cv::Mat& small, const cv::Mat& l_lin, const std::vector<int>& best,
                int hz, std::vector<char>& glare) {
    const int H = p.H, W = p.W, k = p.k;
    glare.assign(W, 0);
    std::vector<int> cols;
    for (int c = 0; c < W; ++c)
        if (best[c] >= 0 && best[c] >= k) cols.push_back(c);
    if (cols.empty()) return;
    // Spaltensummen der linearen Helligkeit (nur für die betroffenen Spalten nötig)
    std::vector<char> brighter(W, 0);
    bool any_bright = false;
    for (int c : cols) {
        const int b = best[c];
        long s_above = 0, s_below = 0;
        for (int r = b - k; r < b; ++r) s_above += l_lin.at<uchar>(r, c);
        for (int r = b; r < std::min(b + k, H); ++r) s_below += l_lin.at<uchar>(r, c);
        const double la = static_cast<double>(s_above) / k;
        const double lb = static_cast<double>(s_below) / k;
        brighter[c] = (la - lb) >= p.glare_dl;
        any_bright = any_bright || brighter[c];
    }
    if (!any_bright) return;
    const int top = std::max(0, hz - 12);
    for (int c : cols) {
        const int b = best[c];
        const int r0 = std::min(top, b);
        int n_sat = 0;
        for (int r = r0; r < b; ++r) {
            const uchar* px = small.ptr<uchar>(r) + 3 * c;
            const int vmax = std::max(std::max(px[0], px[1]), px[2]);
            n_sat += vmax >= p.glare_v;
        }
        glare[c] = brighter[c] && n_sat >= p.glare_px;
    }
    bool any = false;
    for (char g : glare) any = any || g;
    if (any && p.glare_spread > 0) {
        std::vector<int> rowv(W, -1000);
        for (int c : cols) rowv[c] = best[c];
        std::vector<char> grow(W);
        for (int it = 0; it < p.glare_spread; ++it) {
            bool g_any = false;
            for (int c = 0; c < W; ++c) {
                bool g = false;
                if (c > 0) g = g || (glare[c - 1] && std::abs(rowv[c] - rowv[c - 1]) <= 3);
                if (c < W - 1) g = g || (glare[c + 1] && std::abs(rowv[c] - rowv[c + 1]) <= 3);
                grow[c] = g && brighter[c] && !glare[c];
                g_any = g_any || grow[c];
            }
            if (!g_any) break;
            for (int c = 0; c < W; ++c) glare[c] = glare[c] || grow[c];
        }
    }
    // Streifen am Bildrand verwerfen (dort bleibt es beim Hindernis); d = diff vor dem Löschen
    if (glare[0] || glare[W - 1]) {
        const std::vector<char> g0 = glare;
        if (g0[0]) {
            int end = W;
            for (int j = 0; j + 1 < W; ++j)
                if (g0[j] && !g0[j + 1]) { end = j + 1; break; }
            for (int c = 0; c < end; ++c) glare[c] = 0;
        }
        if (glare[W - 1]) {
            int start = 0;
            for (int j = W - 2; j >= 0; --j)
                if (!g0[j] && g0[j + 1]) { start = j + 1; break; }
            for (int c = start; c < W; ++c) glare[c] = 0;
        }
    }
}

}  // namespace

extern "C" {

// small: H x W x 3 BGR (unkorrigiert), floor: H x W (0 = kein Boden), srel: H x W bool oder NULL,
// need: H Mindesthöhen (Index 0 = unterste Zeile, wie _need_rows), free_out: W.
// Rückgabe 0 = ok, -1 = Fehler.
int edge_columns(const EdgeParams* pp, const uint8_t* small_p, const uint8_t* floor_p, const uint8_t* srel_p,
                 const float* need, double horizon, float* free_out, int* n_glare_out) {
    try {
        const EdgeParams& p = *pp;
        const int H = p.H, W = p.W, k = p.k;
        const cv::Mat small(H, W, CV_8UC3, const_cast<uint8_t*>(small_p));
        cv::Mat lab;
        cv::cvtColor(small, lab, cv::COLOR_BGR2Lab);
        if (p.edge_thin) {
            const cv::Mat k3 = cv::Mat::ones(3, 3, CV_8U);
            cv::morphologyEx(lab, lab, cv::MORPH_OPEN, k3);
            cv::morphologyEx(lab, lab, cv::MORPH_CLOSE, k3);
        }
        cv::Mat l_lin;
        cv::extractChannel(lab, l_lin, 0);
        // kumulative Summen je Kanal (Zeile 0 = 0), L ggf. logarithmisch
        std::vector<int32_t> cs(static_cast<size_t>(H + 1) * W * 3, 0);
        for (int r = 0; r < H; ++r) {
            const uchar* src = lab.ptr<uchar>(r);
            const int32_t* prev = &cs[static_cast<size_t>(r) * W * 3];
            int32_t* cur = &cs[static_cast<size_t>(r + 1) * W * 3];
            for (int c = 0; c < W; ++c) {
                const uchar l = p.use_log ? p.lut[src[3 * c]] : src[3 * c];
                cur[3 * c] = prev[3 * c] + l;
                cur[3 * c + 1] = prev[3 * c + 1] + src[3 * c + 1];
                cur[3 * c + 2] = prev[3 * c + 2] + src[3 * c + 2];
            }
        }
        auto CS = [&](int r, int c, int ch) -> int32_t { return cs[(static_cast<size_t>(r) * W + c) * 3 + ch]; };
        const double w1 = p.chroma_w;
        const int hz = static_cast<int>(std::ceil(horizon));
        const int r0 = std::max(hz, k), r1 = H - k;          // Kandidatenzeilen [r0, r1)
        const double span = std::max(1.0, H - horizon);
        const double step_thr = p.step_thr * k;
        const double kf = static_cast<double>(static_cast<float>(k));

        std::vector<int> best(W, -1);
        bool any_hit = false;
        // je Spalte von unten nach oben: unterste Zeile mit Stufe und abweichendem Stück darüber
        for (int c = 0; c < W; ++c) {
            for (int r = r1 - 1; r >= r0; --r) {
                double step = 0.0;
                int32_t sb[3];
                for (int ch = 0; ch < 3; ++ch) {
                    sb[ch] = CS(r + k, c, ch) - CS(r, c, ch);
                    const int32_t up = CS(r, c, ch) - CS(r - k, c, ch);
                    const double a = static_cast<double>(std::abs(up - sb[ch]));
                    step = ch == 0 ? a * 1.0 : step + a * w1;
                }
                if (!(step >= step_thr)) continue;
                const float nf = need[H - 1 - r];
                const int n = std::max(k, static_cast<int>(std::ceil(nf)));
                const int top = std::max(r - n, 0);
                const double den = static_cast<double>(static_cast<float>(r - top));
                double segd = 0.0;
                for (int ch = 0; ch < 3; ++ch) {
                    const double seg = static_cast<double>(CS(r, c, ch) - CS(top, c, ch)) / den;
                    const double a = std::fabs(seg - static_cast<double>(sb[ch]) / kf);
                    segd = ch == 0 ? a * 1.0 : segd + a * w1;
                }
                if (segd >= p.seg_thr) {
                    best[c] = r;
                    any_hit = true;
                    break;
                }
            }
        }
        std::vector<float> fr(W, 1.0f);
        std::vector<char> glare;
        if (any_hit) {
            for (int c = 0; c < W; ++c)
                if (best[c] >= 0) fr[c] = static_cast<float>(static_cast<double>(H - 1 - best[c]) / span);
            if (p.edge_glare) glare_cols(p, small, l_lin, best, hz, glare);
        }
        // Hindernis schon am unteren Rand (Farbmodell, nur messbare Pixel)
        const int nb = std::max(2, k);
        std::vector<char> bot_hit(W, 0);
        for (int c = 0; c < W; ++c) {
            int cnt = 0;
            for (int r = H - nb; r < H; ++r) {
                const size_t i = static_cast<size_t>(r) * W + c;
                cnt += floor_p[i] == 0 && (srel_p == nullptr || srel_p[i] != 0);
            }
            bot_hit[c] = static_cast<double>(cnt) / nb >= 0.6;
            if (bot_hit[c]) fr[c] = 0.0f;
            fr[c] = std::min(std::max(fr[c], 0.0f), 1.0f);
        }
        int n_glare = 0;
        if (!glare.empty()) {
            for (int c = 0; c < W; ++c) {
                glare[c] = glare[c] && !bot_hit[c];
                if (glare[c]) {
                    ++n_glare;
                    fr[c] = 1.0f;                 // vor dem Glätten: nicht auf Nachbarn übertragen
                }
            }
        }
        const cv::Mat row(1, W, CV_32F, fr.data());
        cv::Mat out(1, W, CV_32F, free_out);
        cv::erode(row, out, cv::Mat::ones(1, 5, CV_8U));
        if (n_glare)
            for (int c = 0; c < W; ++c)
                if (glare[c]) free_out[c] = std::numeric_limits<float>::quiet_NaN();
        *n_glare_out = n_glare;
        return 0;
    } catch (const std::exception&) {
        return -1;
    }
}

}  // extern "C"
