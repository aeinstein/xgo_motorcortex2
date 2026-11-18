// Entzerren des Kamerabilds (Linsenverzeichnung) in C++: dasselbe wie cv2.remap(INTER_LINEAR) mit
// Festkomma-Tabellen aus cv2.convertMaps(CV_16SC2), bitgleich, in einem Thread.
//
// Rechenweg von OpenCV (remapBilinear für 8 Bit): je Zielpixel die Quellspalte/-zeile (map1, ganzzahlig) und die
// Bruchteile in 1/32 (map2 = by * 32 + bx). Gewichte ganzzahlig mit Summe 32768:
//   w00 = 32 (32-bx)(32-by), w01 = 32 bx (32-by), w10 = 32 (32-bx) by, w11 = 32 bx by
//   Ziel = (w00 p00 + w01 p01 + w10 p10 + w11 p11 + 16384) >> 15
// Dasselbe in zwei Schritten, exakt (alle Gewichte sind Vielfache von 32):
//   t0 = (32-bx) p00 + bx p01,  t1 = (32-bx) p10 + bx p11,  Ziel = ((32-by) t0 + by t1 + 512) >> 10
// Quellpixel außerhalb des Bilds zählen 0 (BORDER_CONSTANT). camera.py prüft beim Start gegen cv2.remap und
// nimmt diese Fassung nur, wenn sie am Gerät bitgleich ist. Braucht kein OpenCV.

#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace {
constexpr int kLensAbi = 1;   // bei Änderung der Funktionen erhöhen (camera.py prüft das)

inline uint32_t tap(const uint8_t* src, int w, int h, int stride, int cn, int x, int y, int c) {
    if (x < 0 || y < 0 || x >= w || y >= h) return 0;
    return src[static_cast<size_t>(y) * stride + static_cast<size_t>(x) * cn + c];
}

// ein Zielpixel, auch am Rand
inline void eins(const uint8_t* src, int w, int h, int stride, int cn, int sx, int sy, uint32_t m, uint8_t* d) {
    const uint32_t bx = m & 31u, by = (m >> 5) & 31u, ax = 32u - bx, ay = 32u - by;
    if (sx >= 0 && sy >= 0 && sx <= w - 2 && sy <= h - 2) {
        const uint8_t* p0 = src + static_cast<size_t>(sy) * stride + static_cast<size_t>(sx) * cn;
        const uint8_t* p1 = p0 + stride;
        for (int c = 0; c < cn; ++c)
            d[c] = static_cast<uint8_t>((ay * (ax * p0[c] + bx * p0[cn + c]) + by * (ax * p1[c] + bx * p1[cn + c]) + 512u) >> 10);
        return;
    }
    for (int c = 0; c < cn; ++c) {
        const uint32_t t0 = ax * tap(src, w, h, stride, cn, sx, sy, c) + bx * tap(src, w, h, stride, cn, sx + 1, sy, c);
        const uint32_t t1 = ax * tap(src, w, h, stride, cn, sx, sy + 1, c) + bx * tap(src, w, h, stride, cn, sx + 1, sy + 1, c);
        d[c] = static_cast<uint8_t>((ay * t0 + by * t1 + 512u) >> 10);
    }
}
}  // namespace

extern "C" {

int lens_abi(void) { return kLensAbi; }

// src: h Zeilen mit stride Bytes, cn Kanäle (1..4). map1: je Zielpixel (x, y) int16, map2: je Zielpixel uint16.
// dst: dh Zeilen mit dstride Bytes, dw Pixel je Zeile. Rückgabe 0, sonst -1.
int lens_remap(const uint8_t* src, int w, int h, int stride, int cn, const int16_t* map1, const uint16_t* map2,
               uint8_t* dst, int dw, int dh, int dstride) {
    if (!src || !map1 || !map2 || !dst || w < 2 || h < 2 || cn < 1 || cn > 4 || dw < 1 || dh < 1) return -1;
#if defined(__aarch64__)
    static const int16_t kIota[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    const int16x8_t iota = vld1q_s16(kIota);
    const uint8x8_t k32 = vdup_n_u8(32), k31 = vdup_n_u8(31);
#endif
    for (int y = 0; y < dh; ++y) {
        const int16_t* m1 = map1 + static_cast<size_t>(y) * dw * 2;
        const uint16_t* m2 = map2 + static_cast<size_t>(y) * dw;
        uint8_t* d = dst + static_cast<size_t>(y) * dstride;
        int x = 0;
        while (x < dw) {
            const int sx = m1[2 * x], sy = m1[2 * x + 1];
#if defined(__aarch64__)
            // Acht Zielpixel auf einmal, wenn sie aus acht nebeneinanderliegenden Quellpixeln derselben Zeile
            // kommen (die Tabelle ist glatt: das gilt fast überall) und alle Quellpixel im Bild liegen.
            if (cn == 3 && x + 8 <= dw && sx >= 0 && sy >= 0 && sx + 8 <= w - 1 && sy <= h - 2) {
                const int16x8x2_t xy = vld2q_s16(m1 + 2 * x);
                const int16x8_t ex = veorq_s16(xy.val[0], vaddq_s16(vdupq_n_s16(static_cast<int16_t>(sx)), iota));
                const int16x8_t ey = veorq_s16(xy.val[1], vdupq_n_s16(static_cast<int16_t>(sy)));
                if (vmaxvq_u16(vreinterpretq_u16_s16(vorrq_s16(ex, ey))) == 0) {
                    const uint16x8_t mm = vld1q_u16(m2 + x);
                    const uint8x8_t bx = vand_u8(vmovn_u16(mm), k31), by = vand_u8(vmovn_u16(vshrq_n_u16(mm, 5)), k31);
                    const uint8x8_t ax = vsub_u8(k32, bx);
                    const uint16x8_t ay16 = vmovl_u8(vsub_u8(k32, by)), by16 = vmovl_u8(by);
                    const uint8_t* p0 = src + static_cast<size_t>(sy) * stride + static_cast<size_t>(sx) * 3;
                    const uint8x8x3_t a0 = vld3_u8(p0), a1 = vld3_u8(p0 + 3);
                    const uint8x8x3_t b0 = vld3_u8(p0 + stride), b1 = vld3_u8(p0 + stride + 3);
                    uint8x8x3_t r;
                    for (int c = 0; c < 3; ++c) {
                        const uint16x8_t t0 = vmlal_u8(vmull_u8(a0.val[c], ax), a1.val[c], bx);
                        const uint16x8_t t1 = vmlal_u8(vmull_u8(b0.val[c], ax), b1.val[c], bx);
                        const uint32x4_t lo = vmlal_u16(vmull_u16(vget_low_u16(t0), vget_low_u16(ay16)),
                                                        vget_low_u16(t1), vget_low_u16(by16));
                        const uint32x4_t hi = vmlal_u16(vmull_u16(vget_high_u16(t0), vget_high_u16(ay16)),
                                                        vget_high_u16(t1), vget_high_u16(by16));
                        r.val[c] = vmovn_u16(vcombine_u16(vrshrn_n_u32(lo, 10), vrshrn_n_u32(hi, 10)));
                    }
                    vst3_u8(d, r);
                    x += 8;
                    d += 24;
                    continue;
                }
            }
#endif
            eins(src, w, h, stride, cn, sx, sy, m2[x], d);
            ++x;
            d += cn;
        }
    }
    return 0;
}

}  // extern "C"
