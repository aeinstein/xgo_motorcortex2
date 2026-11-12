// Gemeinsames für navfast.cpp und gridfast.cpp: Export, Gleitkomma wie numpy, Bodengeometrie.
#pragma once

#include <cmath>
#include <limits>

#if defined(_MSC_VER)
#pragma fp_contract(off)
#define XGO_API __declspec(dllexport)
#else
#define XGO_API
#endif

extern "C" {

// Bildzeile der Bodengrenze -> Entfernung, wie geometry.CamGeometry.ground_dist_np. Muss zu navfast._GeoRow
// in navfast.py passen. v = Grenze je Spalte (0 = untere Bildkante, 1 = Horizont, Auflösung des Bodenmodells).
struct GeoRow {
    double fh, span, sc;          // Höhe des Bodenmodell-Bildes, fh - Horizontzeile, Vollbild-Pixel je Zeile
    double half_h, f;             // H / 2 und Brennweite (Vollbild-Pixel)
    double tilt, pitch;           // Neigung der Kamera und aktuelles Nicken (rad)
    double cam_h, beta_min;       // Kamerahöhe (m); am/über dem Horizont (beta <= beta_min) gibt es keine Entfernung
    double y_near;                // unterste Bildzeile (H - 1)
};

}  // extern "C"

namespace xgo {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Entfernung (m, nach vorn ab Kamerafußpunkt) der Vollbild-Zeile y; NaN am/über dem Horizont.
// atan2/tan aus der libm, wie numpy und math.
inline double ground_dist_row(const GeoRow& g, double y) {
    const double beta = std::atan2(y - g.half_h, g.f) - g.tilt - g.pitch;
    const double d = g.cam_h / std::tan(beta);
    return beta <= g.beta_min ? kNaN : d;
}

// dasselbe für die Grenze v einer Spalte
inline double ground_dist(const GeoRow& g, double v) { return ground_dist_row(g, (g.fh - v * g.span) * g.sc); }

}  // namespace xgo
