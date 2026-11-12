// Raumkarte für xgonav/gridmap.py: eine Beobachtung des Bodenmodells in die Log-Odds-Karte eintragen
// (GridMap.add_freespace: Strahlen aus der Bodengrenze, Freistrahlen, Treffer samt Bestätigung aus zwei
// Abständen), dazu die Strahlen allein (GridMap._rays, auch für den Kartenabgleich).
//
// Rechnet dasselbe wie numpy: gleiche Gleitkommaoperationen in gleicher Reihenfolge
// (-ffp-contract=off), float32-Arithmetik wo numpy float32 rechnet. Statt np.unique ein
// Markierungsfeld (stamp): jede Zelle höchstens einmal je Beobachtung und Durchgang.
// CM4 2026-11-12, je Beobachtung: Strahlen 0,31 ms und Treffer 0,69 ms in numpy, Freistrahlen 0,1 ms hier.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "xgo.h"

namespace {

constexpr int kGridAbi = 2;   // bei Änderung der Strukturen erhöhen (gridmap.py prüft das)

}  // namespace

extern "C" {

// muss zu _GridParams in gridmap.py passen
struct GridParams {
    int32_t n, nrays, step, confirm_cells;          // Strahl j gehört zur Spalte j * step
    double res, half, px, py, cp, sp;               // half = res / 2; Kamerafußpunkt; cos/sin des Kartenkurses
    double l_show, l_free_occ, l_free, l_min, l_occ, l_max;   // l_free_occ = l_free * occ_free_factor
    double min_range, max_range, free_r;            // free_r = free_range_m, sonst max_range_m
    double margin0, margin_quad, confirm_dr;        // margin0 = max(res, free_margin_m); confirm_dr <= 0: aus
    GeoRow geo;
};

XGO_API int grid_abi(void) { return kGridAbi; }

// GridMap._rays ohne Peilung: je Strahl Länge bis zur Bodengrenze, Ende und Anfang des Freistrahls (alle
// entlang der Peilung, m), ok (Freistrahl vorhanden) und hit (Treffer bis max_range). cols: Bodengrenze je
// Bildspalte, kf (oder null): ab wo die Spalte messbar ist, rad: 1 / cos(Peilung) je Strahl.
XGO_API void grid_rays(const GridParams* p, const double* cols, const double* kf, const double* rad, double* dist,
                       double* free_len, double* start, uint8_t* ok, uint8_t* hit) {
    const double inf = std::numeric_limits<double>::infinity();
    const double big = std::numeric_limits<double>::max();
    for (int j = 0; j < p->nrays; ++j) {
        const int64_t k = static_cast<int64_t>(j) * p->step;
        const double v = cols[k];
        const double d = xgo::ground_dist(p->geo, v);
        double s = p->min_range;
        if (kf) {
            const double kd = xgo::ground_dist(p->geo, kf[k]);
            // np.nan_to_num(kd, nan=inf): NaN -> inf, unendlich -> größte Zahl
            const double kn = kd != kd ? inf : (kd == inf ? big : (kd == -inf ? -big : kd));
            s = kf[k] >= 0.98 ? inf : std::max(s, kn);
        }
        const bool to_horizon = v >= 0.98;
        hit[j] = !to_horizon && std::isfinite(d) && d <= p->max_range;
        // Freistrahl endet vor der Grenze: die gemessene Grenze schwankt mit der Entfernung
        const double margin = p->margin0 + p->margin_quad * d * d;
        const double fl = to_horizon ? p->free_r : (std::isfinite(d) ? std::min(d - margin, p->free_r) : xgo::kNaN);
        dist[j] = d * rad[j];
        free_len[j] = fl * rad[j];
        start[j] = s * rad[j];
        ok[j] = std::isfinite(free_len[j]) && free_len[j] > start[j];
    }
}

// GridMap.add_freespace: Strahlen rechnen und eintragen. lo: n*n float32 (Zeile = y-Zelle), stamp: n*n
// uint32, gen: Marke dieses Aufrufs (gen und gen + 1 werden benutzt, der Aufrufer zählt um 2 weiter).
// cb/sb: cos/sin der Peilung je Strahl; die Weltrichtung ist cb*cp - sb*sp, cb*sp + sb*cp. rmin/rmax (n*n
// float32) und conf (n*n uint8) oder null: Bestätigung aus zwei Abständen (GridMap._add_hits).
// Rückgabe: Anzahl Stützstellen der Freistrahlen.
XGO_API int64_t grid_observe(const GridParams* p, float* lo, uint32_t* stamp, uint32_t gen, const double* cols,
                             const double* kf, const double* rad, const double* cb, const double* sb, float* rmin,
                             float* rmax, uint8_t* conf) {
    const int nr = p->nrays;
    std::vector<double> dist(nr), free_len(nr), start(nr), ca(nr), sa(nr);
    std::vector<uint8_t> ok(nr), hit(nr);
    grid_rays(p, cols, kf, rad, dist.data(), free_len.data(), start.data(), ok.data(), hit.data());
    for (int j = 0; j < nr; ++j) {
        ca[j] = cb[j] * p->cp - sb[j] * p->sp;
        sa[j] = cb[j] * p->sp + sb[j] * p->cp;
    }
    const int64_t n = p->n;
    const double nh = static_cast<double>(p->n) / 2;     // wie self.n / 2
    const float show = static_cast<float>(p->l_show);    // numpy vergleicht float32 mit Python-float in float32
    int64_t samples = 0;
    // freie Strecken: Stützstellen alle res/2 von start bis free_len (wie numpy: s = start + k * half)
    for (int j = 0; j < nr; ++j) {
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
    // Treffer: jede Zelle einmal, mit der Entfernung des ersten Strahls, der sie trifft
    const uint32_t g2 = gen + 1;
    std::vector<int64_t> cells;
    std::vector<float> first;
    for (int j = 0; j < nr; ++j) {
        if (!hit[j]) continue;
        const double ox = p->px + dist[j] * ca[j];
        const double oy = p->py + dist[j] * sa[j];
        const int64_t c = static_cast<int64_t>(std::floor(ox / p->res + nh));
        const int64_t r = static_cast<int64_t>(std::floor(oy / p->res + nh));
        if (r < 0 || r >= n || c < 0 || c >= n) continue;
        const int64_t lin = r * n + c;
        if (stamp[lin] == g2) continue;
        stamp[lin] = g2;
        cells.push_back(lin);
        first.push_back(static_cast<float>(dist[j]));
    }
    std::vector<uint8_t> zaehlt(cells.size(), 1);
    if (conf && !cells.empty()) {
        // Bestätigt ist eine Zelle, wenn sie oder eine Nachbarzelle (bis confirm_cells, am Rand abgeschnitten)
        // aus Entfernungen getroffen wurde, die sich um confirm_dr unterscheiden. Erst alle prüfen (Stand vor
        // dieser Beobachtung), dann schreiben: so rechnet numpy.
        const int64_t kc = std::max<int64_t>(0, p->confirm_cells);
        const float dr = static_cast<float>(p->confirm_dr);
        for (size_t i = 0; i < cells.size(); ++i) {
            const int64_t r = cells[i] / n, c = cells[i] % n;
            float lo_r = first[i], hi_r = first[i];
            for (int64_t a = -kc; a <= kc; ++a) {
                const int64_t rr = std::min(std::max(r + a, int64_t{0}), n - 1);
                for (int64_t b = -kc; b <= kc; ++b) {
                    const int64_t cc = std::min(std::max(c + b, int64_t{0}), n - 1);
                    lo_r = std::min(lo_r, rmin[rr * n + cc]);
                    hi_r = std::max(hi_r, rmax[rr * n + cc]);
                }
            }
            zaehlt[i] = conf[cells[i]] || (hi_r - lo_r >= dr);
        }
        for (size_t i = 0; i < cells.size(); ++i) {
            const int64_t lin = cells[i];
            rmin[lin] = std::min(rmin[lin], first[i]);
            rmax[lin] = std::max(rmax[lin], first[i]);
            conf[lin] = zaehlt[i];
        }
    }
    // numpy: float32 + Python-float bleibt float32
    const float occ = static_cast<float>(p->l_occ), lmax = static_cast<float>(p->l_max);
    for (size_t i = 0; i < cells.size(); ++i)
        if (zaehlt[i]) lo[cells[i]] = std::min(lo[cells[i]] + occ, lmax);
    return samples;
}

}  // extern "C"
