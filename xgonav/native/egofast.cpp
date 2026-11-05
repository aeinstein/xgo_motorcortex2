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

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <vector>

namespace {

constexpr int kAbi = 1;   // bei Änderung der Strukturen erhöhen (egomotion.py prüft das)

}  // namespace

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
    explicit Ego(const EgoParams& p) : p_(p) {}

    void step(const cv::Mat& bgr, double dt, const double* boxes, int nboxes, EgoOut& o) {
        o = EgoOut{};
        for (int i = 0; i < 9; ++i) o.A[i] = (i % 4 == 0) ? 1.0 : 0.0;
        // wie Python: erst grau, dann verkleinern
        cv::Mat g, gray, lap;
        cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
        cv::resize(g, gray, cv::Size(p_.pw, p_.ph), 0, 0, cv::INTER_AREA);
        cv::Laplacian(gray, lap, CV_32F);
        cv::Scalar mean, sd;
        cv::meanStdDev(lap, mean, sd);
        o.sharp = sd[0] * sd[0];

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
        cv::goodFeaturesToTrack(gray, fresh, p_.max_features - static_cast<int>(n), p_.quality, p_.min_distance, mask,
                                5);
        pts_.insert(pts_.end(), fresh.begin(), fresh.end());
    }

    EgoParams p_;
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
