// Stimmproben für den Modus pet (Anbindung xgonav/stimme.py, Teil von libxgofast.so). Braucht kein OpenCV.
//
//   st_mono16k  Stereo 48 kHz (int16) -> mono 16 kHz: Mittel beider Kanäle, Tiefpass 7 kHz (31 Abgriffe,
//               Hamming), jeder 3. Wert
//   st_mfcc     13 MFCC je 10 ms (25 ms Hamming, 512er FFT, 24 Mel-Bänder 100..7600 Hz, DCT-II), stille Ränder
//               (trim_db unter der lautesten Rahmenenergie) ab, Mittelwert abgezogen; dazu der Anteil stimmhafter
//               Rahmen unter den lauten (bis laut_db unter der lautesten): spektrale Flachheit 300..3000 Hz unter flat_max und Grundton 70..400 Hz
//               (Autokorrelation aus dem Leistungsspektrum, durch die des Fensters geteilt) über per_min
//   st_dtw      DTW im Band (Sakoe-Chiba, um die Diagonale), Kosten je Schritt = Summe / (na + nb)
//   st_dehne    zeitlich alpha-mal so lang bei gleicher Tonhöhe (WSOLA), mit schnellerem Abspielen danach = höher
//   st_tor      Rauschtor vor dem Filter: Rahmenenergie ab gate_db unter der lautesten auf knee_db weich zu, je
//               Abtastwert linear zwischen den Rahmenmitten, Ränder ohne Verstärkung weg
//   st_sittich  Wellensittich: schneller abspielen (linear), Schnarren (modulierte Verzögerung), Hochpass und Tiefpass
//               (RBJ-Biquads), tanh-Begrenzung, 10 ms Ein-/Ausblenden, Spitze auf `peak`
// Rechnet dasselbe wie _PyKern in stimme.py (dort numpy-FFT, deshalb gleich bis auf Rundung, nicht bitgleich).
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "xgo.h"

namespace {

const int kStimmAbi = 5;
const double kPi = 3.14159265358979323846;
const int kRateIn = 48000, kRate = 16000, kRateOut = 48000;
const int kFrame = 400, kHop = 160, kNfft = 512, kNbin = kNfft / 2 + 1, kNmel = 24, kNcep = 13;
const int kFirN = 31, kLagLo = 40, kLagHi = 228;     // Grundton 400..70 Hz
const int kWsN = 480, kWsHop = 240, kWsTol = 160;     // WSOLA: 30 ms, Takt 15 ms, Suche +-10 ms
const double kFirFc = 7000.0, kMelLo = 100.0, kMelHi = 7600.0, kFlatLo = 300.0, kFlatHi = 3000.0;

struct Stimm {
    double flat_max = 0.3, per_min = 0.5, trim = 0.063, laut = 0.1;   // trim/laut linear aus dB
    int k0 = 0, k1 = 0;
    std::vector<double> win, fb, dct, twr, twi, zr, zi;   // fb [b * kNbin + k], dct [n * kNmel + m]
    std::vector<double> rw;                               // Autokorrelation des Fensters / r0, Lag kLagLo..kLagHi
    std::vector<int> rev;
};

double mel(double f) { return 2595.0 * std::log10(1.0 + f / 700.0); }

std::vector<double> fir() {
    std::vector<double> h(kFirN);
    const double k = 2.0 * kFirFc / kRateIn;
    double s = 0.0;
    for (int j = 0; j < kFirN; j++) {
        const double m = j - (kFirN - 1) / 2.0, a = kPi * k * m;
        const double sinc = m == 0.0 ? 1.0 : std::sin(a) / a;
        h[j] = k * sinc * (0.54 - 0.46 * std::cos(2.0 * kPi * j / (kFirN - 1)));
        s += h[j];
    }
    for (double& v : h) v /= s;
    return h;
}

void fft(Stimm& o) {
    const int n = kNfft;
    double* re = o.zr.data();
    double* im = o.zi.data();
    for (int i = 0; i < n; i++) {
        const int j = o.rev[i];
        if (j > i) {
            double t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len >> 1, step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int j = 0; j < half; j++) {
                const double wr = o.twr[j * step], wi = o.twi[j * step];
                const int a = i + j, b = a + half;
                const double xr = re[b] * wr - im[b] * wi, xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr; im[b] = im[a] - xi;
                re[a] += xr; im[a] += xi;
            }
        }
    }
}

// y in-place durch einen Biquad (Direktform I, normiert)
void biquad(std::vector<double>& y, double f0, bool hp) {
    const double w0 = 2.0 * kPi * f0 / kRateOut, cw = std::cos(w0), al = std::sin(w0) / (2.0 * std::sqrt(0.5));
    double b0, b1;
    if (hp) { b0 = (1.0 + cw) / 2.0; b1 = -(1.0 + cw); }
    else { b0 = (1.0 - cw) / 2.0; b1 = 1.0 - cw; }
    const double a0 = 1.0 + al;
    const double B0 = b0 / a0, B1 = b1 / a0, B2 = b0 / a0, A1 = -2.0 * cw / a0, A2 = (1.0 - al) / a0;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    for (double& v : y) {
        const double x = v;
        const double o = B0 * x + B1 * x1 + B2 * x2 - A1 * y1 - A2 * y2;
        x2 = x1; x1 = x; y2 = y1; y1 = o;
        v = o;
    }
}

// Autokorrelation aus dem Leistungsspektrum p (kNbin Werte): r = irfft(p), hier per Vorwärts-FFT des geraden
// Spektrums (reell) / kNfft. Schreibt r[0..kNfft) nach o.zr.
void autokorr(Stimm& o, const double* p) {
    for (int k = 0; k < kNfft; k++) {
        o.zr[k] = p[k <= kNfft / 2 ? k : kNfft - k];
        o.zi[k] = 0.0;
    }
    fft(o);
    for (int k = 0; k < kNfft; k++) o.zr[k] /= kNfft;
}

}  // namespace

extern "C" {

XGO_API int st_abi() { return kStimmAbi; }

// pcm: n Stereo-Abtastwerte (L, R, ...) mit 48 kHz. out: mindestens n / 3 + 1 Werte. Rückgabe: Anzahl.
XGO_API int st_mono16k(const int16_t* pcm, int n, double* out) {
    if (n < kFirN) return 0;
    static const std::vector<double> h = fir();
    int k = 0;
    for (int m = 0; m + kFirN <= n; m += 3) {
        double s = 0.0;
        for (int j = 0; j < kFirN; j++)
            s += h[j] * (((double)pcm[2 * (m + j)] + pcm[2 * (m + j) + 1]) * 0.5);
        out[k++] = s;
    }
    return k;
}

XGO_API void* st_new(double flat_max, double per_min, double trim_db, double laut_db) {
    Stimm* o = new Stimm;
    o->flat_max = flat_max;
    o->per_min = per_min;
    o->trim = std::pow(10.0, -trim_db / 10.0);
    o->laut = std::pow(10.0, -laut_db / 10.0);
    o->k0 = (int)std::ceil(kFlatLo * kNfft / kRate);
    o->k1 = (int)std::floor(kFlatHi * kNfft / kRate);
    o->win.resize(kFrame);
    for (int i = 0; i < kFrame; i++)
        o->win[i] = 0.54 - 0.46 * std::cos(2.0 * kPi * i / (kFrame - 1));
    std::vector<double> hz(kNmel + 2);
    const double m0 = mel(kMelLo), m1 = mel(kMelHi);
    for (int i = 0; i < kNmel + 2; i++) {
        const double p = m0 + (m1 - m0) * i / (kNmel + 1);
        hz[i] = 700.0 * (std::pow(10.0, p / 2595.0) - 1.0);
    }
    o->fb.assign((size_t)kNmel * kNbin, 0.0);
    for (int b = 0; b < kNmel; b++) {
        const double lo = hz[b], ce = hz[b + 1], hi = hz[b + 2];
        for (int k = 0; k < kNbin; k++) {
            const double f = (double)k * kRate / kNfft;
            double w = 0.0;
            if (f > lo && f <= ce) w = (f - lo) / (ce - lo);
            else if (f > ce && f < hi) w = (hi - f) / (hi - ce);
            o->fb[(size_t)b * kNbin + k] = w;
        }
    }
    o->dct.resize((size_t)kNcep * kNmel);
    for (int n = 0; n < kNcep; n++)
        for (int m = 0; m < kNmel; m++)
            o->dct[(size_t)n * kNmel + m] = std::cos(kPi * (n + 1) * (m + 0.5) / kNmel);
    o->twr.resize(kNfft / 2); o->twi.resize(kNfft / 2);
    for (int i = 0; i < kNfft / 2; i++) {
        o->twr[i] = std::cos(2.0 * kPi * i / kNfft);
        o->twi[i] = -std::sin(2.0 * kPi * i / kNfft);
    }
    o->rev.resize(kNfft);
    for (int i = 0; i < kNfft; i++) {
        int r = 0;
        for (int b = 0; b < 9; b++)
            if (i & (1 << b)) r |= 1 << (8 - b);
        o->rev[i] = r;
    }
    o->zr.resize(kNfft); o->zi.resize(kNfft);
    {
        for (int i = 0; i < kNfft; i++) { o->zr[i] = i < kFrame ? o->win[i] : 0.0; o->zi[i] = 0.0; }
        fft(*o);
        std::vector<double> pw(kNbin);
        for (int k = 0; k < kNbin; k++) pw[k] = o->zr[k] * o->zr[k] + o->zi[k] * o->zi[k];
        autokorr(*o, pw.data());
        o->rw.resize(kLagHi - kLagLo + 1);
        for (int l = kLagLo; l <= kLagHi; l++) o->rw[l - kLagLo] = o->zr[l] / o->zr[0];
    }
    return o;
}

XGO_API void st_free(void* h) { delete static_cast<Stimm*>(h); }

// x: n Werte mit 16 kHz. out: max_frames x 13. voiced: Anteil stimmhafter unter den lauten Rahmen.
// Rückgabe: Rahmen nach dem Abschneiden der stillen Ränder.
XGO_API int st_mfcc(void* h, const double* x, int n, double* out, int max_frames, double* voiced) {
    Stimm* o = static_cast<Stimm*>(h);
    *voiced = 0.0;
    if (n < kFrame) return 0;
    const int m = 1 + (n - kFrame) / kHop;
    if (m > max_frames) return 0;
    std::vector<double> P((size_t)m * kNbin), e(m);
    double emax = 0.0;
    for (int f = 0; f < m; f++) {
        const int s = f * kHop;
        for (int i = 0; i < kNfft; i++) {
            if (i < kFrame) {
                const int t = s + i;
                const double y = t == 0 ? x[0] : x[t] - 0.97 * x[t - 1];
                o->zr[i] = y * o->win[i];
            } else {
                o->zr[i] = 0.0;
            }
            o->zi[i] = 0.0;
        }
        fft(*o);
        double es = 0.0;
        double* p = &P[(size_t)f * kNbin];
        for (int k = 0; k < kNbin; k++) {
            p[k] = o->zr[k] * o->zr[k] + o->zi[k] * o->zi[k];
            es += p[k];
        }
        e[f] = es;
        if (es > emax) emax = es;
    }
    if (emax <= 0.0) return 0;
    const double thr = o->trim * emax, thr_laut = o->laut * emax;
    int a = 0, b = m;
    while (a < m && e[a] < thr) a++;
    while (b > a && e[b - 1] < thr) b--;
    if (a >= b) return 0;
    const int nk = o->k1 - o->k0 + 1;
    int nlaut = 0, nvoiced = 0;
    std::vector<double> mean(kNcep, 0.0), le(kNmel);
    for (int f = a; f < b; f++) {
        const double* p = &P[(size_t)f * kNbin];
        const bool laut = e[f] >= thr_laut;
        if (laut) {
            double sl = 0.0, sm = 0.0;
            for (int k = o->k0; k <= o->k1; k++) {
                sl += std::log(p[k] + 1e-10);
                sm += p[k];
            }
            const double flat = std::exp(sl / nk) / (sm / nk + 1e-10);
            double per = 0.0;
            autokorr(*o, p);
            if (o->zr[0] > 0.0) {
                per = -std::numeric_limits<double>::infinity();
                for (int l = kLagLo; l <= kLagHi; l++) {
                    const double v = o->zr[l] / o->zr[0] / o->rw[l - kLagLo];
                    if (v > per) per = v;
                }
            }
            nlaut++;
            if (flat < o->flat_max && per > o->per_min) nvoiced++;
        }
        for (int bb = 0; bb < kNmel; bb++) {
            const double* w = &o->fb[(size_t)bb * kNbin];
            double s = 0.0;
            for (int k = 0; k < kNbin; k++) s += p[k] * w[k];
            le[bb] = std::log(s > 1e-10 ? s : 1e-10);
        }
        double* c = &out[(size_t)(f - a) * kNcep];
        for (int q = 0; q < kNcep; q++) {
            const double* d = &o->dct[(size_t)q * kNmel];
            double s = 0.0;
            for (int bb = 0; bb < kNmel; bb++) s += le[bb] * d[bb];
            c[q] = s;
            mean[q] += s;
        }
    }
    const int nf = b - a;
    for (int q = 0; q < kNcep; q++) mean[q] /= nf;
    for (int f = 0; f < nf; f++)
        for (int q = 0; q < kNcep; q++) out[(size_t)f * kNcep + q] -= mean[q];
    *voiced = nlaut ? (double)nvoiced / nlaut : 0.0;
    return nf;
}

XGO_API double st_dtw(const double* a, int na, const double* b, int nb, int dim, double band) {
    const double inf = std::numeric_limits<double>::infinity();
    if (na <= 0 || nb <= 0) return inf;
    double r = band * (na > nb ? na : nb);
    if (r < 1.0) r = 1.0;
    std::vector<double> prev(nb, inf), cur(nb, inf);
    for (int i = 0; i < na; i++) {
        const double ce = na > 1 ? (double)i * (nb - 1) / (na - 1) : 0.0;
        int j0 = (int)std::ceil(ce - r), j1 = (int)std::floor(ce + r);
        if (j0 < 0) j0 = 0;
        if (j1 > nb - 1) j1 = nb - 1;
        for (int j = 0; j < nb; j++) cur[j] = inf;
        const double* ai = a + (size_t)i * dim;
        for (int j = j0; j <= j1; j++) {
            const double* bj = b + (size_t)j * dim;
            double s = 0.0;
            for (int q = 0; q < dim; q++) {
                const double d = ai[q] - bj[q];
                s += d * d;
            }
            const double c = std::sqrt(s);
            if (i == 0 && j == 0) { cur[0] = c; continue; }
            double best = prev[j];
            if (j > 0) {
                if (prev[j - 1] < best) best = prev[j - 1];
                if (cur[j - 1] < best) best = cur[j - 1];
            }
            if (best < inf) cur[j] = best + c;
        }
        prev.swap(cur);
    }
    return prev[nb - 1] / (na + nb);
}

// x: n Werte -> out (round(n * alpha) Werte, höchstens max_out). Rückgabe: Anzahl.
XGO_API int st_dehne(const double* x, int n, double alpha, double* out, int max_out) {
    const int N = kWsN, Hs = kWsHop;
    if (n < N + Hs || alpha <= 0.0) {
        if (n > max_out) return 0;
        for (int t = 0; t < n; t++) out[t] = x[t];
        return n;
    }
    const int L = (int)std::lround(n * alpha);
    if (L > max_out) return 0;
    std::vector<double> win(N), y((size_t)L + N, 0.0);
    for (int i = 0; i < N; i++) win[i] = 0.5 - 0.5 * std::cos(2.0 * kPi * i / N);
    for (int i = 0; i < N; i++) y[i] += x[i] * win[i];
    int prev = 0;
    for (int k = 1;; k++) {
        const int nom = (int)std::lround(k * Hs / alpha);
        if (prev + Hs + N > n || nom + N > n || k * Hs + N > L + N) break;
        const double* tm = x + prev + Hs;
        const int lo = nom - kWsTol > 0 ? nom - kWsTol : 0;
        const int hi = nom + kWsTol < n - N ? nom + kWsTol : n - N;
        int p = lo;
        double best = -std::numeric_limits<double>::infinity();
        for (int j = lo; j <= hi; j++) {
            double s = 0.0;
            for (int i = 0; i < N; i++) s += tm[i] * x[j + i];
            if (s > best) { best = s; p = j; }
        }
        double* yo = &y[(size_t)k * Hs];
        for (int i = 0; i < N; i++) yo[i] += x[p + i] * win[i];
        prev = p;
    }
    for (int t = 0; t < L; t++) out[t] = y[t];
    return L;
}

// x: n Werte mit 16 kHz -> out (höchstens n). Rückgabe: Anzahl nach dem Abschneiden der Ränder.
XGO_API int st_tor(const double* x, int n, double gate_db, double knee_db, double* out) {
    if (n < kFrame) {
        for (int t = 0; t < n; t++) out[t] = x[t];
        return n;
    }
    const int m = 1 + (n - kFrame) / kHop;
    std::vector<double> g(m);
    double emax = 0.0;
    for (int f = 0; f < m; f++) {
        double s = 0.0;
        for (int i = 0; i < kFrame; i++) {
            const double v = x[f * kHop + i];
            s += v * v;
        }
        g[f] = s;
        if (s > emax) emax = s;
    }
    if (emax <= 0.0) return 0;
    const double kn = knee_db > 1e-6 ? knee_db : 1e-6;
    for (int f = 0; f < m; f++) {
        const double ed = 10.0 * std::log10((g[f] > 1e-30 ? g[f] : 1e-30) / emax);
        double v = (ed + gate_db + knee_db) / kn;
        g[f] = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    }
    int k = 0, first = -1, last = -1;
    for (int t = 0; t < n; t++) {
        double p = (t - kFrame / 2.0) / kHop;
        if (p < 0.0) p = 0.0;
        if (p > m - 1) p = m - 1;
        const int i0 = (int)std::floor(p);
        const int i1 = i0 + 1 < m ? i0 + 1 : m - 1;
        const double fr = p - i0;
        const double gs = g[i0] * (1.0 - fr) + g[i1] * fr;
        if (gs > 0.0) {
            if (first < 0) first = t;
            last = t;
        }
        out[t] = x[t] * gs;
    }
    if (first < 0) return 0;
    for (int t = first; t <= last; t++) out[k++] = out[t];
    return k;
}

// x: n Werte mit 16 kHz -> out mit 48 kHz (höchstens max_out). Rückgabe: Anzahl.
XGO_API int st_sittich(const double* x, int n, double speed, double lo, double hi, double vib_hz, double vib_ms,
                       double drive, double peak, double* out, int max_out) {
    if (n < 2 || speed <= 0.0) return 0;
    const double st = speed * kRate / kRateOut;
    const int m = (int)std::floor((n - 1) / st) + 1;
    if (m > max_out) return 0;
    std::vector<double> y(m), z(m);
    for (int k = 0; k < m; k++) {
        const double pos = k * st;
        int i0 = (int)std::floor(pos);
        if (i0 > n - 1) i0 = n - 1;
        const double fr = pos - i0;
        const int i1 = i0 + 1 < n ? i0 + 1 : n - 1;
        y[k] = x[i0] * (1.0 - fr) + x[i1] * fr;
    }
    const double D = vib_ms * 1e-3 * kRateOut;
    for (int k = 0; k < m; k++) {
        const double p = k - D * (1.0 + std::sin(2.0 * kPi * vib_hz * k / kRateOut));
        if (p < 0.0) { z[k] = 0.0; continue; }
        int q0 = (int)std::floor(p);
        if (q0 > m - 1) q0 = m - 1;
        const double qf = p - q0;
        const int q1 = q0 + 1 < m ? q0 + 1 : m - 1;
        z[k] = y[q0] * (1.0 - qf) + y[q1] * qf;
    }
    biquad(z, lo, true);
    biquad(z, hi, false);
    double mx = 0.0;
    for (double v : z) if (std::fabs(v) > mx) mx = std::fabs(v);
    if (mx <= 0.0) {
        for (int k = 0; k < m; k++) out[k] = 0.0;
        return m;
    }
    const double td = std::tanh(drive);
    for (double& v : z) v = std::tanh(drive * (v / mx)) / td;
    int nf = kRateOut / 100;
    if (nf > m / 2) nf = m / 2;
    for (int k = 0; k < nf; k++) {
        const double r = (double)k / nf;
        z[k] *= r;
        z[m - nf + k] *= (double)(nf - 1 - k) / nf;
    }
    mx = 0.0;
    for (double v : z) if (std::fabs(v) > mx) mx = std::fabs(v);
    for (int k = 0; k < m; k++) out[k] = mx > 0.0 ? z[k] * (peak / mx) : z[k];
    return m;
}

}  // extern "C"
