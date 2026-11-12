// Navigation für xgonav ohne Python-Schleifen: Wegsuche und Kürzen (planner.py), Fahrweg-Schätzung
// und Frontier-Gruppen (frontier.py), die Merkkarte (localmap.py: mitdrehen, eintragen, freiräumen,
// vergessen, bestätigen, Bahnprüfung), Wandabstand der Raumkarte (gridmap.py) und Zielpunkt des
// Wegfolgers (behaviors.py).
//
// Rechnet dasselbe wie die Python-Wege dort: gleiche Gleitkommaoperationen in gleicher Reihenfolge
// (-ffp-contract=off), float32 wo numpy float32 rechnet, Summen paarweise wie np.add.reduce, Stützstellen
// wie np.rint(np.linspace()). Wurzeln ganzer Zahlen als sqrt der exakten Summe (= math.hypot, geprüft bis
// 700 x 700), hypot/cos/sin/atan2 aus derselben libm wie numpy und math. Braucht kein OpenCV.
// Während des Aufrufs ist die GIL frei (ctypes): A* in Python hielt sie bis zu 5 ms am Stück.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#if defined(_MSC_VER)
#pragma fp_contract(off)
#define XGO_API __declspec(dllexport)
#else
#define XGO_API
#endif

namespace {

constexpr int kNavAbi = 2;   // bei Änderung der Strukturen oder Signaturen erhöhen (navfast.py prüft das)
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.141592653589793;   // math.pi

// np.add.reduce über ein zusammenhängendes Feld (numpy loops_utils.h.src: pairwise_sum, Block 128)
template <typename T>
T pairwise(const T* a, int64_t n) {
    if (n < 8) {
        T res = static_cast<T>(-0.0);
        for (int64_t i = 0; i < n; ++i) res += a[i];
        return res;
    }
    if (n <= 128) {
        T r[8];
        for (int k = 0; k < 8; ++k) r[k] = a[k];
        int64_t i;
        for (i = 8; i < n - (n % 8); i += 8)
            for (int k = 0; k < 8; ++k) r[k] += a[i + k];
        T res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; ++i) res += a[i];
        return res;
    }
    int64_t n2 = n / 2;
    n2 -= n2 % 8;
    return pairwise(a, n2) + pairwise(a + n2, n - n2);
}

// np.rint(np.linspace(a, b, n)) für ganze a, b: Stützstelle k = k * step + a, die letzte genau b
struct Line {
    int64_t a, b, n;
    double step;
    Line(int64_t a_, int64_t b_, int64_t n_)
        : a(a_), b(b_), n(n_), step(n_ > 1 ? static_cast<double>(b_ - a_) / static_cast<double>(n_ - 1) : 0.0) {}
    int64_t at(int64_t k) const {
        if (n > 1 && k == n - 1) return b;
        if (a == b || n <= 1) return a;
        return static_cast<int64_t>(std::nearbyint(static_cast<double>(k) * step + static_cast<double>(a)));
    }
};

inline double int_hypot(int64_t a, int64_t b) { return std::sqrt(static_cast<double>(a * a + b * b)); }

// Python: a % b für b > 0
inline double py_mod(double a, double b) {
    double m = std::fmod(a, b);
    if (m != 0.0) {
        if (m < 0.0) m += b;
    } else {
        m = std::copysign(0.0, b);
    }
    return m;
}

struct Node {
    double f, d;
    int32_t i;
};
struct NodeAfter {   // wie der Vergleich der Python-Tupel (f, d, i): kleinstes zuerst
    bool operator()(const Node& x, const Node& y) const {
        if (x.f != y.f) return x.f > y.f;
        if (x.d != y.d) return x.d > y.d;
        return x.i > y.i;
    }
};

// planner.astar: Zellen (Index r * w + c) von s nach g; false = kein Weg
bool astar(const uint8_t* blk, const float* cst, int h, int w, int sr, int sc, int gr, int gc,
           std::vector<int32_t>& out) {
    const int32_t si = sr * w + sc, gi = gr * w + gc;
    if (blk[si] || blk[gi]) return false;
    static const int DR[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    static const int DC[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    const double diag = std::sqrt(2.0);
    const double half[8] = {0.5, 0.5, 0.5, 0.5, diag * 0.5, diag * 0.5, diag * 0.5, diag * 0.5};
    const size_t total = static_cast<size_t>(h) * static_cast<size_t>(w);
    std::vector<double> best(total, kInf);
    std::vector<int32_t> prev(total, -1);
    std::priority_queue<Node, std::vector<Node>, NodeAfter> heap;
    best[si] = 0.0;
    heap.push({int_hypot(sr - gr, sc - gc), 0.0, si});
    while (!heap.empty()) {
        const Node cur = heap.top();
        heap.pop();
        if (cur.i == gi) {
            for (int32_t i = cur.i; i >= 0; i = prev[i]) out.push_back(i);
            std::reverse(out.begin(), out.end());
            return true;
        }
        if (cur.d > best[cur.i]) continue;
        const int r = cur.i / w, c = cur.i % w;
        const double ci = cst[cur.i];
        for (int k = 0; k < 8; ++k) {
            const int rr = r + DR[k], cc = c + DC[k];
            if (rr < 0 || rr >= h || cc < 0 || cc >= w) continue;
            const int32_t j = rr * w + cc;
            if (blk[j]) continue;
            const double nd = cur.d + half[k] * (ci + static_cast<double>(cst[j]));
            if (nd < best[j]) {
                best[j] = nd;
                prev[j] = cur.i;
                heap.push({nd + int_hypot(rr - gr, cc - gc), nd, j});
            }
        }
    }
    return false;
}

// planner._line_free
bool line_free(const uint8_t* blk, int w, int ar, int ac, int br, int bc) {
    const int64_t n = static_cast<int64_t>(std::max(std::abs(br - ar), std::abs(bc - ac))) * 2 + 1;
    const Line lr(ar, br, n), lc(ac, bc, n);
    for (int64_t k = 0; k < n; ++k)
        if (blk[lr.at(k) * w + lc.at(k)]) return false;
    return true;
}

// planner.shortcut
void shortcut(const std::vector<int32_t>& cells, const uint8_t* blk, const float* cst, int w,
              std::vector<int32_t>& out) {
    const int64_t len = static_cast<int64_t>(cells.size());
    if (len < 3) {
        out = cells;
        return;
    }
    const double one = std::hypot(1.0, 0.0), diag = std::hypot(1.0, 1.0);   // np.hypot der Schritte
    std::vector<double> seg;
    std::vector<float> ln;
    out.push_back(cells[0]);
    int64_t i = 0;
    while (i < len - 1) {
        int64_t j = len - 1;
        const int ar = cells[i] / w, ac = cells[i] % w;
        while (j > i + 1) {
            const int br = cells[j] / w, bc = cells[j] % w;
            if (line_free(blk, w, ar, ac, br, bc)) {
                // Kosten entlang des Wegs: Summe d * (c[k] + c[k+1]) / 2, und sein Maximum
                seg.clear();
                float cs_max = cst[cells[i]];
                for (int64_t k = i; k < j; ++k) {
                    const int32_t p = cells[k], q = cells[k + 1];
                    const bool schraeg = (p / w != q / w) && (p % w != q % w);
                    const float sum = cst[p] + cst[q];
                    seg.push_back((schraeg ? diag : one) * static_cast<double>(sum) / 2.0);
                    cs_max = std::max(cs_max, cst[q]);
                }
                const double weg = pairwise(seg.data(), static_cast<int64_t>(seg.size()));
                // Kosten entlang der Geraden: Mittel und Maximum
                const int64_t n = static_cast<int64_t>(std::max(std::abs(br - ar), std::abs(bc - ac))) + 1;
                const Line lr(ar, br, n), lc(ac, bc, n);
                ln.clear();
                float ln_max = -std::numeric_limits<float>::infinity();
                for (int64_t k = 0; k < n; ++k) {
                    const float v = cst[lr.at(k) * w + lc.at(k)];
                    ln.push_back(v);
                    ln_max = std::max(ln_max, v);
                }
                const float mean = static_cast<float>(static_cast<double>(pairwise(ln.data(), n)) /
                                                      static_cast<double>(n));
                const double gerade = static_cast<double>(mean) * int_hypot(br - ar, bc - ac);
                if (static_cast<double>(ln_max) <= static_cast<double>(cs_max) + 1e-3 &&
                    gerade <= 1.02 * weg + 1e-3)
                    break;
            }
            --j;
        }
        out.push_back(cells[j]);
        i = j;
    }
}

}  // namespace

extern "C" {

// muss zu _FrontierParams in navfast.py passen
struct FrontierParams {
    int32_t h, w, nlab, hs, ws, step, r0, c0, n, keep, kr, kc, navoid, zone_any, pad0, pad1;
    double res, x, y, th;
    double min_cluster_m, squeeze_cluster_m, min_unknown_m2, keep_dist2;   // keep_dist2 = keep_dist_m ** 2
    double travel_mps, gain_bonus_s, gain_ref_m2, turn_dps;
};

XGO_API int nav_abi(void) { return kNavAbi; }

// planner.route ohne Weltpunkte: A* von (sr, sc) nach (gr, gc) auf blocked (h*w uint8) und cost (h*w
// float32), mit shorten != 0 danach gekürzt (planner.shortcut). out: Platz für cap Zellen (Zeile, Spalte).
// Rückgabe: Anzahl Zellen, -1 = kein Weg, -2 = out zu klein.
XGO_API int nav_route(const uint8_t* blocked, const float* cost, int h, int w, int sr, int sc, int gr, int gc,
                      int shorten, int32_t* out, int cap) {
    std::vector<int32_t> cells, kurz;
    if (!astar(blocked, cost, h, w, sr, sc, gr, gc, cells)) return -1;
    if (shorten) shortcut(cells, blocked, cost, w, kurz);
    const std::vector<int32_t>& res = shorten ? kurz : cells;
    if (static_cast<int64_t>(res.size()) > cap) return -2;
    for (size_t k = 0; k < res.size(); ++k) {
        out[2 * k] = res[k] / w;
        out[2 * k + 1] = res[k] % w;
    }
    return static_cast<int>(res.size());
}

// frontier.wavefront ab dem groben Raster: Schritte (8er-Nachbarschaft) von (sr, sc) über Zellen mit
// grob != 0, -1 = nicht erreichbar. Die Startzelle zählt immer.
XGO_API void nav_wave(const uint8_t* grob, int h, int w, int sr, int sc, int32_t* dist) {
    const int32_t total = h * w;
    std::fill(dist, dist + total, -1);
    std::vector<int32_t> queue;
    queue.reserve(static_cast<size_t>(total));
    dist[sr * w + sc] = 0;
    queue.push_back(sr * w + sc);
    for (size_t q = 0; q < queue.size(); ++q) {
        const int32_t i = queue[q];
        const int r = i / w, c = i % w;
        const int32_t d = dist[i] + 1;
        for (int dr = -1; dr <= 1; ++dr) {
            const int rr = r + dr;
            if (rr < 0 || rr >= h) continue;
            for (int dc = -1; dc <= 1; ++dc) {
                const int cc = c + dc;
                if (cc < 0 || cc >= w) continue;
                const int32_t j = rr * w + cc;
                if (grob[j] && dist[j] < 0) {
                    dist[j] = d;
                    queue.push_back(j);
                }
            }
        }
    }
}

// Frontier-Gruppen werten (Schleife über die Gruppen in frontier.next_goal). lab: h*w int32 aus
// cv2.connectedComponents (0 = keine Frontier), zone/behind/gain_cells: h*w, dist: hs*ws (nav_wave),
// avoid: navoid x (x, y, r). Je Gruppe k (1..nlab-1): code 0 = zählt nicht, 1 = Ziel mit Abstand, 2 = Ziel
// nur in einer Engstelle, 3/4 = wie 1/2, aber gesperrt (avoid); score, gain (m²), Zelle (cr, cc), kleinster
// Abstand² (Zellen) zum bisherigen Ziel kd2 und dessen Zelle (kr_o, kc_o).
XGO_API void nav_frontier(const FrontierParams* p, const int32_t* lab, const uint8_t* zone, const float* behind,
                          const int32_t* dist, const float* gain_cells, const double* avoid, int8_t* code,
                          double* score, double* gain, int32_t* cr, int32_t* cc, int64_t* kd2, int32_t* kr_o,
                          int32_t* kc_o) {
    struct Acc {
        int64_t n = 0, sr = 0, sc = 0;
        int32_t r0 = INT32_MAX, r1 = INT32_MIN, c0 = INT32_MAX, c1 = INT32_MIN;
    };
    const int nlab = p->nlab, h = p->h, w = p->w;
    std::vector<Acc> alle(nlab), weit(nlab);
    std::vector<float> bmax(nlab, -std::numeric_limits<float>::infinity());
    for (int k = 0; k < nlab; ++k) {
        code[k] = 0;
        score[k] = gain[k] = 0.0;
        cr[k] = cc[k] = kr_o[k] = kc_o[k] = 0;
        kd2[k] = INT64_MAX;
    }
    auto add = [](Acc& a, int r, int c) {
        ++a.n;
        a.sr += r;
        a.sc += c;
        a.r0 = std::min(a.r0, r);
        a.r1 = std::max(a.r1, r);
        a.c0 = std::min(a.c0, c);
        a.c1 = std::max(a.c1, c);
    };
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const int64_t i = static_cast<int64_t>(r) * w + c;
            const int k = lab[i];
            if (k <= 0) continue;
            add(alle[k], r, c);
            if (!zone[i]) add(weit[k], r, c);
            bmax[k] = std::max(bmax[k], behind[i]);
            if (p->keep) {
                const int64_t dr = r - p->kr, dc = c - p->kc, d2 = dr * dr + dc * dc;
                if (d2 < kd2[k]) {
                    kd2[k] = d2;
                    kr_o[k] = r;
                    kc_o[k] = c;
                }
            }
        }
    }
    // Stufe je Gruppe; use: 0 = nur die Zellen mit Abstand, 1 = alle
    std::vector<int8_t> stufe(nlab, -1), use(nlab, 0);
    std::vector<double> mr(nlab, 0.0), mc(nlab, 0.0), bestv(nlab, kInf);
    for (int k = 1; k < nlab; ++k) {
        if (!alle[k].n) continue;
        int st = weit[k].n ? 0 : 1;
        bool ok = false;
        if (st == 0) {
            const Acc& a = weit[k];
            ok = int_hypot(a.c1 - a.c0 + 1, a.r1 - a.r0 + 1) * p->res >= p->min_cluster_m;
            if (!ok) st = 1;                    // der Teil mit Abstand ist zu klein: als Engstelle werten
        }
        if (st == 1) {
            const Acc& a = alle[k];
            ok = int_hypot(a.c1 - a.c0 + 1, a.r1 - a.r0 + 1) * p->res >= p->squeeze_cluster_m &&
                 static_cast<double>(bmax[k]) * p->res * p->res >= p->min_unknown_m2 && p->zone_any != 0;
        }
        if (!ok && (!p->keep || static_cast<double>(kd2[k]) * p->res * p->res > p->keep_dist2)) continue;
        stufe[k] = static_cast<int8_t>(st);
        use[k] = static_cast<int8_t>(st);
        const Acc& a = st == 0 ? weit[k] : alle[k];
        mr[k] = static_cast<double>(a.sr) / static_cast<double>(a.n);
        mc[k] = static_cast<double>(a.sc) / static_cast<double>(a.n);
    }
    // Zelle am nächsten zur Mitte (erste bei Gleichstand, Reihenfolge wie np.nonzero)
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const int64_t i = static_cast<int64_t>(r) * w + c;
            const int k = lab[i];
            if (k <= 0 || stufe[k] < 0 || (use[k] == 0 && zone[i])) continue;
            const double dr = static_cast<double>(r) - mr[k], dc = static_cast<double>(c) - mc[k];
            const double v = dr * dr + dc * dc;
            if (v < bestv[k]) {
                bestv[k] = v;
                cr[k] = r;
                cc[k] = c;
            }
        }
    }
    const double nh = static_cast<double>(p->n) / 2;
    const double rad_to_deg = 180.0 / kPi;
    for (int k = 1; k < nlab; ++k) {
        if (stufe[k] < 0) continue;
        const int r = cr[k], c = cc[k];
        const double fx = (static_cast<double>(c + p->c0) + 0.5 - nh) * p->res;
        const double fy = (static_cast<double>(r + p->r0) + 0.5 - nh) * p->res;
        bool gesperrt = false;
        for (int a = 0; a < p->navoid && !gesperrt; ++a) {
            const double dx = fx - avoid[3 * a], dy = fy - avoid[3 * a + 1];
            gesperrt = std::sqrt(dx * dx + dy * dy) < avoid[3 * a + 2];
        }
        if (gesperrt) {
            code[k] = static_cast<int8_t>(3 + stufe[k]);
            continue;
        }
        const int32_t d = dist[std::min(r / p->step, p->hs - 1) * p->ws + std::min(c / p->step, p->ws - 1)];
        const double ex = fx - p->x, ey = fy - p->y;
        // grobes Raster verfehlt die Zelle: Luftlinie mal 1,5
        const double dm = d >= 0 ? static_cast<double>(static_cast<int64_t>(d) * p->step) * p->res
                                 : 1.5 * std::sqrt(ex * ex + ey * ey);
        const double g = static_cast<double>(gain_cells[static_cast<int64_t>(r) * w + c]) * p->res * p->res;
        const double drive_s = dm / p->travel_mps;
        const double dth = std::fabs(py_mod(std::atan2(ey, ex) - p->th + kPi, 2 * kPi) - kPi);
        const double anteil = g / p->gain_ref_m2;
        score[k] = p->gain_bonus_s * (anteil < 1.0 ? anteil : 1.0) - drive_s - dth * rad_to_deg / p->turn_dps;
        gain[k] = g;
        code[k] = static_cast<int8_t>(1 + stufe[k]);
    }
}

// localmap.LocalMap._thin: Punkte (n x 2) in Reihenfolge, behalten wird, wer vom zuletzt behaltenen
// mindestens sqrt(lim) entfernt ist. keep: Indizes; Rückgabe: Anzahl.
XGO_API int64_t lmap_thin(const double* pts, int64_t n, double lim, int64_t* keep) {
    if (n <= 0) return 0;
    int64_t m = 0;
    keep[m++] = 0;
    double qx = pts[0], qy = pts[1];
    for (int64_t i = 1; i < n; ++i) {
        const double dx = pts[2 * i] - qx, dy = pts[2 * i + 1] - qy;
        if (dx * dx + dy * dy >= lim) {
            keep[m++] = i;
            qx = pts[2 * i];
            qy = pts[2 * i + 1];
        }
    }
    return m;
}

// localmap.LocalMap.confirmed_points: für jeden Punkt mit need: conf = 1, wenn die Punkte mit nb näher als
// sqrt(rad2) aus mindestens confirm verschiedenen Beobachtungen (obs) stammen, sonst 0. cell: Kante der
// Suchzellen (etwas über dem Radius). Punkte ohne need bleiben unverändert.
XGO_API void lmap_confirm(const double* pts, const int64_t* obs, int64_t n, const uint8_t* need,
                          const uint8_t* nb, double rad2, double cell, int confirm, int8_t* conf) {
    // Nachbarn in Zellen einsortieren (Zählsortierung); bei unbrauchbarer Ausdehnung alle gegen alle
    std::vector<int64_t> idx;
    idx.reserve(static_cast<size_t>(n));
    double x0 = kInf, x1 = -kInf, y0 = kInf, y1 = -kInf;
    bool finite = cell > 0;
    for (int64_t i = 0; i < n; ++i) {
        if (!nb[i]) continue;
        idx.push_back(i);
        const double x = pts[2 * i], y = pts[2 * i + 1];
        finite = finite && std::isfinite(x) && std::isfinite(y);
        x0 = std::min(x0, x);
        x1 = std::max(x1, x);
        y0 = std::min(y0, y);
        y1 = std::max(y1, y);
    }
    const int64_t m = static_cast<int64_t>(idx.size());
    int64_t gx = 0, gy = 0;
    if (finite && m > 0) {
        const double fx = std::floor((x1 - x0) / cell) + 1, fy = std::floor((y1 - y0) / cell) + 1;
        if (fx * fy <= 4e6) {
            gx = static_cast<int64_t>(fx);
            gy = static_cast<int64_t>(fy);
        }
    }
    std::vector<int64_t> start, sorted;
    auto cell_of = [&](int64_t i, int64_t& ix, int64_t& iy) {
        ix = static_cast<int64_t>(std::floor((pts[2 * i] - x0) / cell));
        iy = static_cast<int64_t>(std::floor((pts[2 * i + 1] - y0) / cell));
    };
    if (gx > 0) {
        start.assign(static_cast<size_t>(gx * gy + 1), 0);
        for (int64_t i : idx) {
            int64_t ix, iy;
            cell_of(i, ix, iy);
            ++start[iy * gx + ix + 1];
        }
        for (int64_t k = 0; k < gx * gy; ++k) start[k + 1] += start[k];
        sorted.resize(static_cast<size_t>(m));
        std::vector<int64_t> pos(start.begin(), start.end() - 1);
        for (int64_t i : idx) {
            int64_t ix, iy;
            cell_of(i, ix, iy);
            sorted[pos[iy * gx + ix]++] = i;
        }
    }
    std::vector<int64_t> seen(static_cast<size_t>(std::max(confirm, 1)));
    for (int64_t i = 0; i < n; ++i) {
        if (!need[i]) continue;
        const double cx = pts[2 * i], cy = pts[2 * i + 1];
        int found = 0;
        auto visit = [&](int64_t j) {
            const double dx = cx - pts[2 * j], dy = cy - pts[2 * j + 1];
            if (!(dx * dx + dy * dy < rad2)) return;
            const int64_t o = obs[j];
            for (int k = 0; k < found; ++k)
                if (seen[k] == o) return;
            seen[found++] = o;
        };
        if (gx > 0 && std::isfinite(cx) && std::isfinite(cy)) {
            // der Punkt selbst muss nicht unter den Nachbarn sein: Zelle auf das Raster begrenzt suchen
            const double qx = std::floor((cx - x0) / cell), qy = std::floor((cy - y0) / cell);
            const double ax = std::max(qx - 1, 0.0), bx = std::min(qx + 1, static_cast<double>(gx - 1));
            const double ay = std::max(qy - 1, 0.0), by = std::min(qy + 1, static_cast<double>(gy - 1));
            if (ax <= bx && ay <= by) {
                for (int64_t iy = static_cast<int64_t>(ay); iy <= static_cast<int64_t>(by) && found < confirm; ++iy)
                    for (int64_t ix = static_cast<int64_t>(ax); ix <= static_cast<int64_t>(bx) && found < confirm; ++ix)
                        for (int64_t q = start[iy * gx + ix]; q < start[iy * gx + ix + 1] && found < confirm; ++q)
                            visit(sorted[q]);
            }
        } else {
            for (int64_t q = 0; q < m && found < confirm; ++q) visit(idx[q]);
        }
        conf[i] = found >= confirm ? 1 : 0;
    }
}

// localmap.LocalMap.arc_clearance ab den bestätigten Punkten (n x 2): freie Strecke (m) entlang des Bogens,
// inf = frei. sperrt (n uint8): Punkte, die die Bahn sperren (nur bei Rückgabe < inf gesetzt).
XGO_API double lmap_arc(const double* pts, int64_t n, double v, double w, double horizon_s, int steps,
                        double radius, double tol, uint8_t* sperrt) {
    std::vector<double> lim(static_cast<size_t>(n));
    for (int64_t i = 0; i < n; ++i) lim[i] = std::min(radius, std::hypot(pts[2 * i], pts[2 * i + 1]) - tol);
    const double dt = horizon_s / steps;
    double x = 0.0, y = 0.0, th = 0.0, s = 0.0;
    for (int k = 0; k < steps; ++k) {
        th += w * dt;
        x += v * std::cos(th) * dt;
        y += v * std::sin(th) * dt;
        s += std::fabs(v) * dt;
        bool any = false;
        for (int64_t i = 0; i < n; ++i) {
            sperrt[i] = std::hypot(pts[2 * i] - x, pts[2 * i + 1] - y) < lim[i];
            any = any || sperrt[i];
        }
        if (any) return s;
    }
    return kInf;
}

// localmap.LocalMap.move: Punkte (n x 2) um ds zurück und um den Winkel mit cos c, sin s gedreht nach out.
XGO_API void lmap_move(const double* pts, int64_t n, double ds, double c, double s, double* out) {
    for (int64_t i = 0; i < n; ++i) {
        const double px = pts[2 * i] - ds, py = pts[2 * i + 1];
        out[2 * i] = px * c - py * s;
        out[2 * i + 1] = px * s + py * c;
    }
}

// muss zu _LmapAdd in navfast.py passen
struct LmapAdd {
    int64_t n, ncols, obs_n, cap;       // alte Punkte, Bildspalten, Nummer der Beobachtung, höchstens so viele Punkte
    double sc, cx, f, cam_fwd;          // Vollbild-Pixel je Spalte, Bildspalte der Fahrtrichtung, Brennweite, Kamera vor der Achse
    double max_range, far_range;        // far_range = max_range * 1.5
    double max_age, near, now, lim;     // near: Entfernung der unteren Bildkante; lim = MIN_SPACING ** 2
};

// localmap.LocalMap.add_freespace ab den Entfernungen je Spalte (die Geometrie rechnet weiter numpy):
// alte Punkte entfernen, wo die Beobachtung freien Boden zeigt (_clear_seen), die Grenze jeder zweiten Spalte
// ausgedünnt anhängen, Altes und Fernes vergessen (_prune). cols: Grenze je Spalte (NaN = verdeckt), dist:
// ihre Entfernung, known (oder null): Entfernung, ab der die Spalte messbar ist. Schreibt nach pts_o/t_o/obs_o
// (Platz für n + ncols / 2 + 1), die Eingaben bleiben unverändert. Rückgabe: Anzahl Punkte.
XGO_API int64_t lmap_add(const LmapAdd* p, const double* pts, const double* t, const int64_t* obs,
                         const double* cols, const double* dist, const double* known, double* pts_o, double* t_o,
                         int64_t* obs_o) {
    const int64_t n = p->n, nc = p->ncols;
    int64_t m = 0;
    // Reichweite je Spalte: frei bis zum Horizont -> max_range, sonst die Grenze; NaN = verdeckt
    std::vector<double> rng(static_cast<size_t>(nc));
    for (int64_t k = 0; k < nc; ++k) {
        if (!std::isfinite(cols[k])) rng[k] = std::numeric_limits<double>::quiet_NaN();
        else if (cols[k] >= 0.98 || !std::isfinite(dist[k])) rng[k] = p->max_range;
        else rng[k] = std::min(dist[k], p->max_range);
    }
    for (int64_t i = 0; i < n; ++i) {
        // Spalte und Entfernungen gelten ab Kamera, die Punkte ab Radachse
        const double x = pts[2 * i] - p->cam_fwd, y = pts[2 * i + 1];
        bool drop = false;
        if (x > 1e-3) {
            const double col = std::floor((p->cx - p->f * y / x) / p->sc);
            if (col >= 1 && col < static_cast<double>(nc - 1)) {            // Randspalten auslassen
                const int64_t k = static_cast<int64_t>(col);
                const double r = rng[k];
                double lo = p->near;
                if (known) lo = std::max(p->near, std::isfinite(known[k]) ? known[k] : kInf);
                drop = std::isfinite(r) && x > lo && x < r - (0.1 + 0.15 * r);
            }
        }
        if (!drop) {
            pts_o[2 * m] = pts[2 * i];
            pts_o[2 * m + 1] = pts[2 * i + 1];
            t_o[m] = t[i];
            obs_o[m] = obs[i];
            ++m;
        }
    }
    // neue Punkte: jede zweite Spalte, ausgedünnt (_thin)
    bool first = true;
    double qx = 0.0, qy = 0.0;
    for (int64_t k = 0; k < nc; k += 2) {
        const double d = dist[k];
        if (!(cols[k] < 0.98) || !std::isfinite(d) || !(d <= p->max_range)) continue;
        const double x_full = (static_cast<double>(k) + 0.5) * p->sc;
        const double x = d + p->cam_fwd, y = d * (p->cx - x_full) / p->f;      // d * tan(Peilung)
        if (!first) {
            const double dx = x - qx, dy = y - qy;
            if (!(dx * dx + dy * dy >= p->lim)) continue;
        }
        first = false;
        qx = x;
        qy = y;
        pts_o[2 * m] = x;
        pts_o[2 * m + 1] = y;
        t_o[m] = p->now;
        obs_o[m] = p->obs_n;
        ++m;
    }
    // vergessen: zu alt, zu fern, mehr als cap (die ältesten zuerst)
    const int64_t von = m > p->cap ? m - p->cap : 0;
    int64_t out = 0;
    for (int64_t i = von; i < m; ++i) {
        if (!(p->now - t_o[i] < p->max_age) || !(std::hypot(pts_o[2 * i], pts_o[2 * i + 1]) < p->far_range)) continue;
        pts_o[2 * out] = pts_o[2 * i];
        pts_o[2 * out + 1] = pts_o[2 * i + 1];
        t_o[out] = t_o[i];
        obs_o[out] = obs_o[i];
        ++out;
    }
    return out;
}

// gridmap.GridMap.wall_dist: Abstand (m) von (x, y) zur Mitte der nächsten belegten Zelle (lo > l_show) im
// Fenster ±k Zellen um die Zelle (r, c), inf wenn keine näher als reach. lo: n*n float32.
XGO_API double nav_wall_dist(const float* lo, int n, double res, float l_show, double x, double y, int r, int c,
                             int k, double reach) {
    const int r0 = std::max(0, r - k), r1 = std::min(n, r + k + 1);
    const int c0 = std::max(0, c - k), c1 = std::min(n, c + k + 1);
    const double nh = static_cast<double>(n) / 2;
    double best = kInf;
    for (int rr = r0; rr < r1; ++rr) {
        const double dy = (static_cast<double>(rr) + 0.5 - nh) * res - y;
        for (int cc = c0; cc < c1; ++cc) {
            if (!(lo[static_cast<int64_t>(rr) * n + cc] > l_show)) continue;
            const double d = std::hypot((static_cast<double>(cc) + 0.5 - nh) * res - x, dy);
            if (d < best) best = d;
        }
    }
    return best < reach ? best : kInf;
}

// behaviors.PathFollower.target: Punkt rest Meter entlang des Wegs (n x 2, n >= 1) ab der Projektion von (x, y).
XGO_API void nav_path_target(const double* path, int64_t n, double x, double y, double rest, double* out) {
    if (n == 1) {
        out[0] = path[0];
        out[1] = path[1];
        return;
    }
    double best = kInf, bt = 0.0;
    int64_t bi = 0;
    for (int64_t i = 0; i < n - 1; ++i) {
        const double ax = path[2 * i], ay = path[2 * i + 1];
        const double dx = path[2 * i + 2] - ax, dy = path[2 * i + 3] - ay;
        const double l2 = dx * dx + dy * dy;
        double t = 0.0;
        if (l2 > 1e-12) {
            t = ((x - ax) * dx + (y - ay) * dy) / l2;
            t = t < 1.0 ? t : 1.0;                      // clamp wie max(0.0, min(1.0, t))
            t = t > 0.0 ? t : 0.0;
        }
        const double ex = ax + t * dx - x, ey = ay + t * dy - y;
        const double d = std::sqrt(ex * ex + ey * ey);
        if (d < best) {
            best = d;
            bi = i;
            bt = t;
        }
    }
    double t = bt;
    for (int64_t i = bi; i < n - 1; ++i, t = 0.0) {
        const double ax = path[2 * i], ay = path[2 * i + 1];
        const double dx = path[2 * i + 2] - ax, dy = path[2 * i + 3] - ay;
        const double len = std::sqrt(dx * dx + dy * dy);
        const double left = (1.0 - t) * len;
        if (left >= rest && len > 1e-9) {
            t += rest / len;
            out[0] = ax + t * dx;
            out[1] = ay + t * dy;
            return;
        }
        rest -= left;
    }
    out[0] = path[2 * (n - 1)];
    out[1] = path[2 * (n - 1) + 1];
}

}  // extern "C"
