// Schallrichtung aus zwei Mikrofonen (Anbindung xgonav/ortung.py, Teil von libxgofast.so). Braucht kein OpenCV.
//
// Laufzeitunterschied links/rechts aus der Phase des Kreuzspektrums (GCC-PHAT, nur im Band f_lo..f_hi):
//   ort_level  Pegel eines Blocks für das Tor: Summe beider Kanäle, je `dec` Abtastwerte addiert (Tiefpass,
//              Nullstellen bei Vielfachen von rate/dec), davon die Differenz (Hochpass). Eine Addition je Wert.
//   ort_add    nur bei offenem Tor: eine komplexe FFT für beide Kanäle (z = L + jR), Kreuzspektrum aufsummieren
//   ort_solve  Kreuzspektrum auf Betrag^beta normieren und gegen 2*ns+1 Laufzeiten prüfen (Tabellen), Parabel
//              um das Maximum
// Rechnet dasselbe wie _PyKern in ortung.py (dort numpy-FFT, deshalb gleich bis auf Rundung, nicht bitgleich).
#include <cmath>
#include <cstdint>
#include <vector>

#include "xgo.h"

namespace {

const int kOrtAbi = 1;
const double kPi = 3.14159265358979323846;

struct Ort {
    int n = 0, k0 = 0, nb = 0, ns = 0, blocks = 0;
    double beta = 1.0, tau_max = 0.0;
    std::vector<double> win, twr, twi;       // Hann-Fenster, FFT-Drehfaktoren
    std::vector<int> rev;                    // Bitumkehr
    std::vector<double> zr, zi;              // FFT-Puffer
    std::vector<double> sr, si;              // aufsummiertes Kreuzspektrum im Band
    std::vector<double> stc, sts;            // cos/sin(2 pi f_k tau_i), [i * nb + k]
};

void fft(Ort& o) {
    const int n = o.n;
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

}  // namespace

extern "C" {

XGO_API int ort_abi() { return kOrtAbi; }

// n: Blocklänge (Zweierpotenz), Band f_lo..f_hi in Hz, tau_max in s, ns: Laufzeit-Schritte je Seite
XGO_API void* ort_new(int n, double rate, double f_lo, double f_hi, double tau_max, int ns, double beta) {
    if (n < 8 || (n & (n - 1)) != 0 || ns < 1 || rate <= 0 || tau_max <= 0)
        return nullptr;
    Ort* o = new Ort;
    o->n = n; o->ns = ns; o->beta = beta; o->tau_max = tau_max;
    int k0 = (int)std::ceil(f_lo * n / rate), k1 = (int)std::floor(f_hi * n / rate);
    if (k0 < 1) k0 = 1;
    if (k1 > n / 2 - 1) k1 = n / 2 - 1;
    if (k1 < k0) { delete o; return nullptr; }
    o->k0 = k0; o->nb = k1 - k0 + 1;
    o->win.resize(n); o->twr.resize(n / 2); o->twi.resize(n / 2); o->rev.resize(n);
    o->zr.resize(n); o->zi.resize(n);
    o->sr.assign(o->nb, 0.0); o->si.assign(o->nb, 0.0);
    for (int i = 0; i < n; i++)
        o->win[i] = 0.5 - 0.5 * std::cos(2.0 * kPi * i / n);
    for (int i = 0; i < n / 2; i++) {
        o->twr[i] = std::cos(2.0 * kPi * i / n);
        o->twi[i] = -std::sin(2.0 * kPi * i / n);
    }
    int bits = 0;
    while ((1 << bits) < n) bits++;
    for (int i = 0; i < n; i++) {
        int r = 0;
        for (int b = 0; b < bits; b++)
            if (i & (1 << b)) r |= 1 << (bits - 1 - b);
        o->rev[i] = r;
    }
    const int nt = 2 * ns + 1;
    o->stc.resize((size_t)nt * o->nb); o->sts.resize((size_t)nt * o->nb);
    for (int i = 0; i < nt; i++) {
        const double tau = tau_max * (i - ns) / ns;
        for (int k = 0; k < o->nb; k++) {
            const double ph = 2.0 * kPi * ((k0 + k) * rate / n) * tau;
            o->stc[(size_t)i * o->nb + k] = std::cos(ph);
            o->sts[(size_t)i * o->nb + k] = std::sin(ph);
        }
    }
    return o;
}

XGO_API void ort_free(void* h) { delete static_cast<Ort*>(h); }

XGO_API void ort_reset(void* h) {
    Ort* o = static_cast<Ort*>(h);
    for (int k = 0; k < o->nb; k++) o->sr[k] = o->si[k] = 0.0;
    o->blocks = 0;
}

// pcm: n Stereo-Abtastwerte (L, R, L, R, ...). Mittleres Quadrat der Differenz aufeinanderfolgender dec-Summen.
XGO_API double ort_level(const int16_t* pcm, int n, int dec) {
    if (dec < 1) dec = 1;
    double e = 0.0;
    long long prev = 0;
    int m = 0;
    for (int i = 0; i + dec <= n; i += dec) {
        long long s = 0;
        for (int j = 0; j < dec; j++)
            s += (long long)pcm[2 * (i + j)] + pcm[2 * (i + j) + 1];
        if (i > 0) {
            const double d = (double)(s - prev);
            e += d * d;
            m++;
        }
        prev = s;
    }
    return m ? e / m : 0.0;
}

// Einen Block (o->n Stereo-Abtastwerte) ins Kreuzspektrum L * conj(R) aufnehmen.
XGO_API void ort_add(void* h, const int16_t* pcm) {
    Ort* o = static_cast<Ort*>(h);
    const int n = o->n;
    for (int i = 0; i < n; i++) {
        o->zr[i] = o->win[i] * pcm[2 * i];
        o->zi[i] = o->win[i] * pcm[2 * i + 1];
    }
    fft(*o);
    // Z = FFT(L + jR), A = Z[k], B = Z[n-k]:  L * conj(R) = Im(A * B) / 2 + j (|A|^2 - |B|^2) / 4
    for (int k = 0; k < o->nb; k++) {
        const int a = o->k0 + k, b = n - a;
        const double ar = o->zr[a], ai = o->zi[a], br = o->zr[b], bi = o->zi[b];
        o->sr[k] += 0.5 * (ar * bi + ai * br);
        o->si[k] += 0.25 * ((ar * ar + ai * ai) - (br * br + bi * bi));
    }
    o->blocks++;
}

// out[0]: Laufzeit in s (> 0: links zuerst), out[1]: Güte 0..1 (Anteil des Bandes, der zu dieser Laufzeit passt).
// Leert den Speicher. Rückgabe: Zahl der Blöcke, 0 = nichts gesammelt.
XGO_API int ort_solve(void* h, double* out) {
    Ort* o = static_cast<Ort*>(h);
    const int nb = o->nb, nt = 2 * o->ns + 1, blocks = o->blocks;
    out[0] = out[1] = 0.0;
    double norm = 0.0;
    for (int k = 0; k < nb; k++) {
        const double m = std::sqrt(o->sr[k] * o->sr[k] + o->si[k] * o->si[k]);
        if (m > 0.0) {
            const double g = std::pow(m, -o->beta);
            o->sr[k] *= g; o->si[k] *= g;
            norm += m * g;
        }
    }
    if (blocks > 0 && norm > 0.0) {
        std::vector<double> r(nt);
        int best = 0;
        for (int i = 0; i < nt; i++) {
            const double* c = &o->stc[(size_t)i * nb];
            const double* s = &o->sts[(size_t)i * nb];
            double acc = 0.0;
            for (int k = 0; k < nb; k++)
                acc += o->sr[k] * c[k] + o->si[k] * s[k];
            r[i] = acc;
            if (acc > r[best]) best = i;
        }
        double off = 0.0, peak = r[best];
        if (best > 0 && best < nt - 1) {
            const double den = r[best - 1] - 2.0 * r[best] + r[best + 1];
            if (den < 0.0) {
                off = 0.5 * (r[best - 1] - r[best + 1]) / den;
                peak = r[best] - 0.25 * (r[best - 1] - r[best + 1]) * off;
            }
        }
        out[0] = o->tau_max * (best - o->ns + off) / o->ns;
        out[1] = peak / norm;
    }
    ort_reset(h);
    return blocks;
}

}  // extern "C"
