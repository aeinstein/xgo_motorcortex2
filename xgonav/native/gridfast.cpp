// Raumkarte für xgonav/gridmap.py: Freistrahlen und Treffer in die Log-Odds-Karte eintragen
// (zweiter Teil von GridMap.add_freespace, die Strahlen selbst rechnet weiter numpy).
//
// Rechnet dasselbe wie numpy: gleiche Gleitkommaoperationen in gleicher Reihenfolge
// (-ffp-contract=off), float32-Arithmetik wo numpy float32 rechnet. Statt np.unique ein
// Markierungsfeld (stamp): jede Zelle höchstens einmal je Beobachtung und Durchgang.
// CM4 2026-11-05: numpy 1,5 ms (Freistrahlen 1,28 + Treffer 0,24) je Beobachtung.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {

constexpr int kGridAbi = 1;   // bei Änderung der Strukturen erhöhen (gridmap.py prüft das)

}  // namespace

extern "C" {

// muss zu _GridParams in gridmap.py passen
struct GridParams {
    int n, nrays, pad0, pad1;
    double res, half, px, py;           // half = res / 2 (Abstand der Stützstellen)
    double l_show, l_free_occ, l_free, l_min, l_occ, l_max;   // l_free_occ = l_free * occ_free_factor
};

int grid_abi(void) { return kGridAbi; }

// lo: n*n float32 (Zeile = y-Zelle), stamp: n*n uint32, gen: Marke dieses Aufrufs (gen und gen + 1
// werden benutzt, der Aufrufer zählt um 2 weiter). Je Strahl j: start, free_len, dist (entlang der
// Peilung), ca/sa = cos/sin der Weltrichtung, ok = Freistrahl eintragen, hit = Treffer eintragen.
// Rückgabe: Anzahl Stützstellen der Freistrahlen.
long grid_add(const GridParams* p, float* lo, uint32_t* stamp, uint32_t gen, const double* start,
              const double* free_len, const double* dist, const double* ca, const double* sa, const uint8_t* ok,
              const uint8_t* hit) {
    const int64_t n = p->n;
    const double nh = static_cast<double>(p->n) / 2;     // wie self.n / 2
    const float show = static_cast<float>(p->l_show);    // numpy vergleicht float32 mit Python-float in float32
    long samples = 0;
    // freie Strecken: Stützstellen alle res/2 von start bis free_len (wie numpy: s = start + k * half)
    for (int j = 0; j < p->nrays; ++j) {
        if (!ok[j]) continue;
        const int64_t steps = static_cast<int64_t>(std::ceil((free_len[j] - start[j]) / p->half)) + 1;
        samples += steps;
        for (int64_t k = 0; k < steps; ++k) {
            const double s = start[j] + static_cast<double>(k) * p->half;
            const double fx = p->px + s * ca[j];
            const double fy = p->py + s * sa[j];
            const int64_t c = static_cast<int64_t>(std::floor(fx / p->res + nh));
            const int64_t r = static_cast<int64_t>(std::floor(fy / p->res + nh));
            if (r < 0 || r >= n || c < 0 || c >= n) continue;
            const int64_t lin = r * n + c;
            if (stamp[lin] == gen) continue;
            stamp[lin] = gen;
            // Hysterese: schon belegte Zellen nur abgeschwächt abbauen (numpy: float64, dann float32)
            const float cur = lo[lin];
            const double step = cur > show ? p->l_free_occ : p->l_free;
            lo[lin] = static_cast<float>(std::max(static_cast<double>(cur) + step, p->l_min));
        }
    }
    // Treffer (numpy: float32 + Python-float bleibt float32)
    const float occ = static_cast<float>(p->l_occ), lmax = static_cast<float>(p->l_max);
    const uint32_t g2 = gen + 1;
    for (int j = 0; j < p->nrays; ++j) {
        if (!hit[j]) continue;
        const double ox = p->px + dist[j] * ca[j];
        const double oy = p->py + dist[j] * sa[j];
        const int64_t c = static_cast<int64_t>(std::floor(ox / p->res + nh));
        const int64_t r = static_cast<int64_t>(std::floor(oy / p->res + nh));
        if (r < 0 || r >= n || c < 0 || c >= n) continue;
        const int64_t lin = r * n + c;
        if (stamp[lin] == g2) continue;
        stamp[lin] = g2;
        lo[lin] = std::min(lo[lin] + occ, lmax);
    }
    return samples;
}

}  // extern "C"
