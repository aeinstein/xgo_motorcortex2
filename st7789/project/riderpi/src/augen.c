/* Raster des Augengenerators (augen.h): dasselbe Modell wie tools/eyes.py draw_kind/draw_eye/render.
 *
 * tools/eyes.py zeichnet Polygone in 4-facher Auflösung und verkleinert (INTER_AREA). Hier rechnet jedes Pixel
 * den Abstand zur Form aus (Abstandsfunktionen) und nimmt als Abdeckung clamp(0,5 - d): das glättet Kanten auf
 * einem Pixel, ohne Überabtastung. Abgetastet wird bei (i + 0,375, j + 0,375), dem Mittel der vier Teilpixel, die
 * INTER_AREA in tools/eyes.py zusammenfasst. Ergebnis nicht bitgleich, aber deckungsgleich (tests/test_augen.py).
 *
 * Reihenfolge der Abdeckung wie das Übermalen in draw_kind: Auge, Pupille (schwarz), Glanzpunkte, Oberlid,
 * Unterlid (schwarz), Träne. Beide Augen: Maximum. Global (tilt, gx, gy) wie warpAffine: je Pixel rückwärts.
 */
#include "augen.h"

#include <math.h>
#include <string.h>

#define CX_L 98.0f
#define CX_R 222.0f
#define CY 118.0f
#define MIN_H 7.0f
#define ABTAST 0.375f
#define PI_F 3.14159265358979f
/* cv2 zeichnet mit LINE_AA Flächen und Striche etwas größer als ihre Geometrie: Flächen ~0,15 px je Seite
 * (deck), Striche zusammen ~0,25 px (gemessen gegen tools/eyes.py 2026-11-28; vorher Fläche -0,7 %, geschlossenes
 * Auge -5 %, Striche -3..-7 %) */
#ifndef FLAECHE_PLUS
#define FLAECHE_PLUS 0.15f
#endif
#ifndef STRICH_PLUS
#define STRICH_PLUS 0.10f
#endif

int augen_abi(void) { return AUGEN_ABI; }

void auge_neutral(auge_t *a)
{
    memset(a, 0, sizeof(*a));
    a->w = 78.0f;
    a->h = 96.0f;
    a->r = 0.36f;
    a->sc = 1.0f;
    a->tear = -1.0f;
    a->kind = ART_EYE;
}

void bild_neutral(bild_t *b)
{
    b->tilt = b->gx = b->gy = 0.0f;
    auge_neutral(&b->l);
    auge_neutral(&b->r);
}

static inline float klemm(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline float deck(float d) { return klemm(0.5f + FLAECHE_PLUS - d, 0.0f, 1.0f); }
static inline float glatt(float x)
{
    x = klemm(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

/* ---------------------------------------------------------------- Herz (Abstandsfeld, einmal berechnet) */
/* tools/eyes.py heart(): 16 sin^3 t / 32 * w, (-(13 cos t - 5 cos 2t - 2 cos 3t - cos 4t) + 2,5) / 30 * h,
 * Polygon aus 120 Punkten. Hier als vorab berechnetes Abstandsfeld der Einheitsform (w = h = 1). */
#define HZ_N 128
#define HZ_X0 (-0.62f)
#define HZ_X1 0.62f
#define HZ_Y0 (-0.44f)
#define HZ_Y1 0.77f
static float herz_feld[HZ_N * HZ_N];
static int herz_fertig = 0;

static float seg_abstand(float px, float py, float ax, float ay, float bx, float by)
{
    float vx = bx - ax, vy = by - ay, wx = px - ax, wy = py - ay;
    float l2 = vx * vx + vy * vy;
    float t = l2 > 0 ? klemm((wx * vx + wy * vy) / l2, 0.0f, 1.0f) : 0.0f;
    float dx = wx - t * vx, dy = wy - t * vy;
    return sqrtf(dx * dx + dy * dy);
}

static void herz_bauen(void)
{
    enum { N = 120 };
    float xs[N], ys[N];
    for (int i = 0; i < N; i++) {
        double t = 2.0 * M_PI * i / (N - 1);
        double s = sin(t);
        xs[i] = (float)(16.0 * s * s * s / 32.0);
        ys[i] = (float)((-(13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t)) + 2.5) / 30.0);
    }
    for (int j = 0; j < HZ_N; j++) {
        float py = HZ_Y0 + (HZ_Y1 - HZ_Y0) * j / (HZ_N - 1);
        for (int i = 0; i < HZ_N; i++) {
            float px = HZ_X0 + (HZ_X1 - HZ_X0) * i / (HZ_N - 1);
            float d = 1e9f;
            int innen = 0;
            for (int k = 0; k < N - 1; k++) {
                float e = seg_abstand(px, py, xs[k], ys[k], xs[k + 1], ys[k + 1]);
                if (e < d) d = e;
                if ((ys[k] > py) != (ys[k + 1] > py)
                    && px < xs[k] + (py - ys[k]) * (xs[k + 1] - xs[k]) / (ys[k + 1] - ys[k]))
                    innen = !innen;
            }
            herz_feld[j * HZ_N + i] = innen ? -d : d;
        }
    }
    herz_fertig = 1;
}

/* Abstand in Einheiten der Einheitsform */
static float herz_abstand(float u, float v)
{
    float fx = (u - HZ_X0) / (HZ_X1 - HZ_X0) * (HZ_N - 1);
    float fy = (v - HZ_Y0) / (HZ_Y1 - HZ_Y0) * (HZ_N - 1);
    if (fx < 0 || fy < 0 || fx > HZ_N - 1 || fy > HZ_N - 1) return 0.2f;   /* außerhalb: sicher draußen */
    int i = (int)fx, j = (int)fy;
    if (i > HZ_N - 2) i = HZ_N - 2;
    if (j > HZ_N - 2) j = HZ_N - 2;
    float ax = fx - i, ay = fy - j;
    const float *f = herz_feld + j * HZ_N + i;
    float o = f[0] + (f[1] - f[0]) * ax;
    float n = f[HZ_N] + (f[HZ_N + 1] - f[HZ_N]) * ax;
    return o + (n - o) * ay;
}

/* ---------------------------------------------------------------- Abstandsfunktionen */
static inline float sd_rrect(float x, float y, float bw, float bh, float rr)
{
    float qx = fabsf(x) - (bw - rr), qy = fabsf(y) - (bh - rr);
    float ax = qx > 0 ? qx : 0, ay = qy > 0 ? qy : 0;
    float m = qx > qy ? qx : qy;
    return sqrtf(ax * ax + ay * ay) + (m < 0 ? m : 0) - rr;
}

static inline float sd_kreis(float x, float y, float cx, float cy, float r)
{
    float dx = x - cx, dy = y - cy;
    return sqrtf(dx * dx + dy * dy) - r;
}

/* Abstand zu einer Halbellipse um (0, 0), Halbachsen rx, ry; oben = 1: Bogen bei y <= 0, sonst y >= 0 */
static float halbellipse(float x, float y, float rx, float ry, int oben)
{
    if (oben ? y <= 0 : y >= 0) {
        float k0 = sqrtf((x / rx) * (x / rx) + (y / ry) * (y / ry));
        float k1 = sqrtf((x / (rx * rx)) * (x / (rx * rx)) + (y / (ry * ry)) * (y / (ry * ry)));
        if (k1 < 1e-9f) return rx < ry ? rx : ry;
        return fabsf(k0 * (k0 - 1.0f) / k1);
    }
    float a = sqrtf((x - rx) * (x - rx) + y * y), b = sqrtf((x + rx) * (x + rx) + y * y);
    return a < b ? a : b;
}

static float sd_dreieck(float px, float py, const float *p)   /* Quilez sdTriangle, p = 3 Punkte */
{
    float e0x = p[2] - p[0], e0y = p[3] - p[1], e1x = p[4] - p[2], e1y = p[5] - p[3], e2x = p[0] - p[4], e2y = p[1] - p[5];
    float v0x = px - p[0], v0y = py - p[1], v1x = px - p[2], v1y = py - p[3], v2x = px - p[4], v2y = py - p[5];
    float t0 = klemm((v0x * e0x + v0y * e0y) / (e0x * e0x + e0y * e0y), 0, 1);
    float t1 = klemm((v1x * e1x + v1y * e1y) / (e1x * e1x + e1y * e1y), 0, 1);
    float t2 = klemm((v2x * e2x + v2y * e2y) / (e2x * e2x + e2y * e2y), 0, 1);
    float q0x = v0x - e0x * t0, q0y = v0y - e0y * t0, q1x = v1x - e1x * t1, q1y = v1y - e1y * t1;
    float q2x = v2x - e2x * t2, q2y = v2y - e2y * t2;
    float s = e0x * e2y - e0y * e2x > 0 ? 1.0f : -1.0f;
    float d0 = q0x * q0x + q0y * q0y, d1 = q1x * q1x + q1y * q1y, d2 = q2x * q2x + q2y * q2y;
    float c0 = s * (v0x * e0y - v0y * e0x), c1 = s * (v1x * e1y - v1y * e1x), c2 = s * (v2x * e2y - v2y * e2x);
    float d = d0 < d1 ? d0 : d1;
    d = d < d2 ? d : d2;
    float c = c0 < c1 ? c0 : c1;
    c = c < c2 ? c : c2;
    return -sqrtf(d) * (c > 0 ? 1.0f : -1.0f);
}

/* ---------------------------------------------------------------- Geometrie je Auge und Form */
typedef struct {
    int kind, s;
    float cx, cy, c, sn;             /* Mitte, Drehung (lokal -> Bild: x' = cx + c x - sn y, y' = cy + sn x + c y) */
    float w, h, hb, rr;              /* Auge: Breite, Höhe, Höhe mit Lidschlag, Eckradius */
    int pupil_an, glanz_an;
    float pcx, pcy, pr, g1x, g1y, g1r, g2x, g2y, g2r;
    int top_an;
    float y0, ks, kn;                /* Oberlid: y = y0 + ks x, kn = 1 / sqrt(1 + ks^2) */
    int bot_an;
    float bb, bcv, uw;               /* Unterlid: yb = bb - bcv (1 - u^2), u = x * uw */
    float hzw, hzh, hzs;             /* Herz: Größe, Abstandsmaßstab */
    float ax_, ary, ayc, halb;       /* Bogen: Halbachsen, Mitte y, halbe Strichstärke; Spirale/Zickzack ebenso */
    int oben;
    float sr, sa, sspin;             /* Spirale */
    float ch[6];                     /* Zickzack (> <) */
    float rad;                       /* Begrenzung im lokalen System */
} geo_t;

typedef struct {
    int tear_an;
    float tx, ty, trr, tri[6];
} traene_t;

static void geo_bauen(geo_t *g, const auge_t *p, int kind, int s, float blink, float sc)
{
    memset(g, 0, sizeof(*g));
    g->kind = kind;
    g->s = s;
    g->cx = (s < 0 ? CX_L : CX_R) + p->dx;
    g->cy = CY + p->dy;
    float w = p->w * sc, h = p->h * sc;
    float hb = h * (1.0f - blink);
    if (hb < MIN_H) hb = MIN_H;
    float rot = p->rot * -s * PI_F / 180.0f;
    g->c = cosf(rot);
    g->sn = sinf(rot);
    g->w = w;
    g->h = h;
    g->hb = hb;
    switch (kind) {
    case ART_EYE: {
        float m = w < hb ? w : hb;
        float rr = p->r * m / 2.0f;
        if (rr > w / 2) rr = w / 2;
        if (rr > hb / 2) rr = hb / 2;
        g->rr = rr;
        float mwh = w < h ? w : h;
        if (p->pupil * mwh > 3) {
            g->pupil_an = 1;
            g->pr = p->pupil * mwh / 2.0f;
            g->pcx = p->ppx * w / 2.0f;
            g->pcy = p->ppy * hb / 2.0f;
            if (p->hl > 0) {
                g->glanz_an = 1;
                g->g1x = g->pcx - 0.38f * g->pr;
                g->g1y = g->pcy - 0.38f * g->pr;
                g->g1r = g->pr * 0.34f * p->hl;
                g->g2x = g->pcx + 0.42f * g->pr;
                g->g2y = g->pcy + 0.40f * g->pr;
                g->g2r = g->pr * 0.16f * p->hl;
            }
        }
        if (p->top > 0 || p->tslope != 0) {
            g->top_an = 1;
            float k = tanf(p->tslope * PI_F / 180.0f);
            g->y0 = -hb / 2.0f + p->top * hb;
            g->ks = k * s;                       /* xs * s = Abstand nach außen */
            g->kn = 1.0f / sqrtf(1.0f + k * k);
        }
        if (p->bot > 0 || p->bcurve > 0) {
            g->bot_an = 1;
            g->bb = hb / 2.0f - p->bot * hb;
            g->bcv = p->bcurve * hb;
            g->uw = 2.0f / w;
        }
        g->rad = sqrtf(w * w + hb * hb) / 2.0f + 2.0f;
        break;
    }
    case ART_HEART:
        g->hzw = w * 1.15f;
        g->hzh = h * 0.95f;
        g->hzs = sqrtf(g->hzw * g->hzh);
        g->rad = sqrtf(0.6f * 0.6f * g->hzw * g->hzw + 0.77f * 0.77f * g->hzh * g->hzh) + 2.0f;
        break;
    case ART_HAPPY:                          /* _line(arc(w * 0,40, h * 0,30), 14 sc) */
        g->ax_ = w * 0.40f;
        g->ary = h * 0.30f;
        g->ayc = g->ary / 2.0f;
        g->oben = 1;
        g->halb = 7.0f * sc + STRICH_PLUS;
        g->rad = g->ax_ + g->ary + g->halb + 2.0f;
        break;
    case ART_CLOSED:                         /* _line(arc(w * 0,42, -h * 0,13) + [0, h * 0,12], 8 sc) */
        g->ax_ = w * 0.42f;
        g->ary = h * 0.13f;
        g->ayc = -g->ary / 2.0f + h * 0.12f;
        g->oben = 0;
        g->halb = 4.0f * sc + STRICH_PLUS;
        g->rad = g->ax_ + g->ary + h * 0.12f + g->halb + 2.0f;
        break;
    case ART_SPIRAL:                         /* _line(spiral(min(w, h) * 0,52, spin * -s), 7 sc) */
        g->sr = (w < h ? w : h) * 0.52f;
        g->sa = 2.6f * 2.0f * PI_F;
        g->sspin = p->spin * -s;
        g->halb = 3.5f * sc + STRICH_PLUS;
        g->rad = g->sr + g->halb + 2.0f;
        break;
    case ART_CHEV: {                         /* > <, Spitze zeigt nach innen; _line(pts, 12 sc) */
        float a = (float)-s;
        g->ch[0] = -a * w * 0.38f; g->ch[1] = -h * 0.30f;
        g->ch[2] = a * w * 0.38f;  g->ch[3] = 0.0f;
        g->ch[4] = -a * w * 0.38f; g->ch[5] = h * 0.30f;
        g->halb = 6.0f * sc + STRICH_PLUS;
        g->rad = sqrtf(0.38f * 0.38f * w * w + 0.09f * h * h) + g->halb + 2.0f;
        break;
    }
    }
}

static void traene_bauen(traene_t *t, const auge_t *p, int s, float blink, float sc)
{
    t->tear_an = p->tear >= 0;
    if (!t->tear_an) return;
    float w = p->w * sc, h = p->h * sc;
    float hb = h * (1.0f - blink);
    if (hb < MIN_H) hb = MIN_H;
    float cx = (s < 0 ? CX_L : CX_R) + p->dx, cy = CY + p->dy;
    float ph = p->tear;
    float gr = glatt(ph / 0.5f);
    float fall = (ph - 0.5f > 0 ? ph - 0.5f : 0.0f) / 0.5f;
    t->tx = cx + (float)(-s) * w * 0.22f;
    t->ty = cy + hb / 2.0f + 4.0f + 9.0f * gr + 150.0f * fall * fall;
    t->trr = 3.0f + 5.0f * gr;
    float rr = t->trr;
    t->tri[0] = t->tx - rr * 0.93f; t->tri[1] = t->ty - rr * 0.35f;
    t->tri[2] = t->tx + rr * 0.93f; t->tri[3] = t->ty - rr * 0.35f;
    t->tri[4] = t->tx;              t->tri[5] = t->ty - rr * 2.3f;
}

/* Abdeckung einer Form im Bildsystem (x, y vor dem globalen Kippen) */
static float form_deckung(const geo_t *g, float x, float y)
{
    float vx = x - g->cx, vy = y - g->cy;
    if (vx * vx + vy * vy > g->rad * g->rad) return 0.0f;
    float lx = g->c * vx + g->sn * vy, ly = -g->sn * vx + g->c * vy;
    float m;
    switch (g->kind) {
    case ART_EYE:
        m = deck(sd_rrect(lx, ly, g->w / 2.0f, g->hb / 2.0f, g->rr));
        if (g->pupil_an) {
            m *= 1.0f - deck(sd_kreis(lx, ly, g->pcx, g->pcy, g->pr));
            if (g->glanz_an) {
                float a = deck(sd_kreis(lx, ly, g->g1x, g->g1y, g->g1r));
                m = m * (1.0f - a) + a;
                a = deck(sd_kreis(lx, ly, g->g2x, g->g2y, g->g2r));
                m = m * (1.0f - a) + a;
            }
        }
        if (m <= 0.0f) return 0.0f;
        if (g->top_an) m *= 1.0f - deck((ly - (g->y0 + g->ks * lx)) * g->kn);
        if (g->bot_an) {
            float u = klemm(lx * g->uw, -1.2f, 1.2f);
            float yb = g->bb - g->bcv * (1.0f - u * u);
            float st = g->bcv * 2.0f * u * g->uw;
            m *= 1.0f - deck((yb - ly) / sqrtf(1.0f + st * st));
        }
        return m;
    case ART_HEART:
        return deck(herz_abstand(lx / g->hzw, ly / g->hzh) * g->hzs);
    case ART_HAPPY:
    case ART_CLOSED:
        return deck(halbellipse(lx, ly - g->ayc, g->ax_, g->ary, g->oben) - g->halb);
    case ART_SPIRAL: {
        float rho = sqrtf(lx * lx + ly * ly);
        float d = rho;                                       /* Anfang in der Mitte */
        float ex = g->sr * cosf(g->sa + g->sspin) - lx, ey = g->sr * sinf(g->sa + g->sspin) - ly;
        float e = sqrtf(ex * ex + ey * ey);                  /* Ende außen */
        if (e < d) d = e;
        float a0 = atan2f(ly, lx) - g->sspin;
        a0 = fmodf(a0, 2.0f * PI_F);
        if (a0 < 0) a0 += 2.0f * PI_F;
        for (float a = a0; a <= g->sa; a += 2.0f * PI_F) {
            float q = fabsf(rho - g->sr * a / g->sa);
            if (q < d) d = q;
        }
        return deck(d - g->halb);
    }
    case ART_CHEV: {
        float d1 = seg_abstand(lx, ly, g->ch[0], g->ch[1], g->ch[2], g->ch[3]);
        float d2 = seg_abstand(lx, ly, g->ch[2], g->ch[3], g->ch[4], g->ch[5]);
        return deck((d1 < d2 ? d1 : d2) - g->halb);
    }
    }
    return 0.0f;
}

static float traene_deckung(const traene_t *t, float x, float y)
{
    float a = deck(sd_kreis(x, y, t->tx, t->ty, t->trr));
    float b = deck(sd_dreieck(x, y, t->tri));
    return a + b - a * b;
}

/* Ein Auge samt Übergang (draw_eye): bis zu zwei Formen und die Träne */
typedef struct {
    geo_t a, b;
    traene_t ta, tb;
    int zwei;                        /* 0: nur a; 1: a * (1 - x) + b * x */
    float x;
    float bx0, by0, bx1, by1;        /* Begrenzung im Bildsystem vor dem globalen Kippen */
} augenform_t;

static void rand_dazu(augenform_t *f, const geo_t *g)
{
    if (g->cx - g->rad < f->bx0) f->bx0 = g->cx - g->rad;
    if (g->cx + g->rad > f->bx1) f->bx1 = g->cx + g->rad;
    if (g->cy - g->rad < f->by0) f->by0 = g->cy - g->rad;
    if (g->cy + g->rad > f->by1) f->by1 = g->cy + g->rad;
}

static void traene_rand(augenform_t *f, const traene_t *t)
{
    if (!t->tear_an) return;
    float r = t->trr + 2.0f;
    if (t->tx - r < f->bx0) f->bx0 = t->tx - r;
    if (t->tx + r > f->bx1) f->bx1 = t->tx + r;
    if (t->ty - t->trr * 2.3f - 2.0f < f->by0) f->by0 = t->ty - t->trr * 2.3f - 2.0f;
    if (t->ty + r > f->by1) f->by1 = t->ty + r;
}

static void augenform_bauen(augenform_t *f, const auge_t *p, int s)
{
    float mix = p->mix;
    f->bx0 = f->by0 = 1e9f;
    f->bx1 = f->by1 = -1e9f;
    f->zwei = 0;
    f->x = 0.0f;
    if (p->kind == ART_EYE || mix <= 0) {
        geo_bauen(&f->a, p, ART_EYE, s, p->blink, p->sc);
        traene_bauen(&f->ta, p, s, p->blink, p->sc);
    } else if (mix >= 1) {
        geo_bauen(&f->a, p, p->kind, s, p->blink, p->sc);
        traene_bauen(&f->ta, p, s, p->blink, p->sc);
    } else {
        float bl = p->blink > mix ? p->blink : mix;
        float sc = p->sc * (0.7f + 0.3f * mix);
        geo_bauen(&f->a, p, ART_EYE, s, bl, p->sc);
        traene_bauen(&f->ta, p, s, bl, p->sc);
        geo_bauen(&f->b, p, p->kind, s, p->blink, sc);
        traene_bauen(&f->tb, p, s, p->blink, sc);
        f->zwei = 1;
        f->x = glatt((mix - 0.35f) / 0.35f);
        rand_dazu(f, &f->b);
        traene_rand(f, &f->tb);
    }
    rand_dazu(f, &f->a);
    traene_rand(f, &f->ta);
}

static float augenform_deckung(const augenform_t *f, float x, float y)
{
    if (x < f->bx0 || x > f->bx1 || y < f->by0 || y > f->by1) return 0.0f;
    float a = form_deckung(&f->a, x, y);
    if (f->ta.tear_an) {
        float t = traene_deckung(&f->ta, x, y);
        a = a * (1.0f - t) + t;
    }
    if (!f->zwei) return a;
    float b = form_deckung(&f->b, x, y);
    if (f->tb.tear_an) {
        float t = traene_deckung(&f->tb, x, y);
        b = b * (1.0f - t) + t;
    }
    return a * (1.0f - f->x) + b * f->x;
}

/* ---------------------------------------------------------------- Bild */
typedef struct {
    augenform_t l, r;
    float ca, sa, gx, gy;            /* globales Kippen um die Bildmitte, dann verschieben */
    int gedreht;
    rechteck_t rect;
} szene_t;

static void szene_bauen(szene_t *z, const bild_t *b)
{
    if (!herz_fertig) herz_bauen();
    augenform_bauen(&z->l, &b->l, -1);
    augenform_bauen(&z->r, &b->r, 1);
    float t = b->tilt * PI_F / 180.0f;
    z->ca = cosf(t);
    z->sa = sinf(t);
    z->gx = b->gx;
    z->gy = b->gy;
    z->gedreht = b->tilt != 0.0f || b->gx != 0.0f || b->gy != 0.0f;
    /* Begrenzung: beide Augen, dann vorwärts durch das Kippen (cv2.getRotationMatrix2D: Winkel gegen den
     * Uhrzeigersinn, x' = a x + b y, y' = -b x + a y um die Mitte) */
    float bx0 = z->l.bx0 < z->r.bx0 ? z->l.bx0 : z->r.bx0, bx1 = z->l.bx1 > z->r.bx1 ? z->l.bx1 : z->r.bx1;
    float by0 = z->l.by0 < z->r.by0 ? z->l.by0 : z->r.by0, by1 = z->l.by1 > z->r.by1 ? z->l.by1 : z->r.by1;
    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
    for (int k = 0; k < 4; k++) {
        float px = (k & 1) ? bx1 : bx0, py = (k & 2) ? by1 : by0;
        float vx = px - AUGEN_W / 2.0f, vy = py - AUGEN_H / 2.0f;
        float qx = z->ca * vx + z->sa * vy + AUGEN_W / 2.0f + z->gx;
        float qy = -z->sa * vx + z->ca * vy + AUGEN_H / 2.0f + z->gy;
        if (qx < x0) x0 = qx;
        if (qx > x1) x1 = qx;
        if (qy < y0) y0 = qy;
        if (qy > y1) y1 = qy;
    }
    int ix0 = (int)floorf(x0) - 1, iy0 = (int)floorf(y0) - 1, ix1 = (int)ceilf(x1) + 1, iy1 = (int)ceilf(y1) + 1;
    z->rect.x0 = ix0 < 0 ? 0 : ix0;
    z->rect.y0 = iy0 < 0 ? 0 : iy0;
    z->rect.x1 = ix1 > AUGEN_W - 1 ? AUGEN_W - 1 : ix1;
    z->rect.y1 = iy1 > AUGEN_H - 1 ? AUGEN_H - 1 : iy1;
}

static inline float szene_deckung(const szene_t *z, float X, float Y)
{
    float x = X, y = Y;
    if (z->gedreht) {
        float vx = X - AUGEN_W / 2.0f - z->gx, vy = Y - AUGEN_H / 2.0f - z->gy;
        x = z->ca * vx - z->sa * vy + AUGEN_W / 2.0f;
        y = z->sa * vx + z->ca * vy + AUGEN_H / 2.0f;
    }
    float a = augenform_deckung(&z->l, x, y);
    float b = augenform_deckung(&z->r, x, y);
    return a > b ? a : b;
}

rechteck_t augen_raster(const bild_t *b, uint8_t *alpha)
{
    szene_t z;
    szene_bauen(&z, b);
    for (int j = z.rect.y0; j <= z.rect.y1; j++) {
        uint8_t *zeile = alpha + j * AUGEN_W;
        float Y = j + ABTAST;
        for (int i = z.rect.x0; i <= z.rect.x1; i++)
            zeile[i] = (uint8_t)(szene_deckung(&z, i + ABTAST, Y) * 255.0f + 0.5f);
    }
    return z.rect;
}

rechteck_t augen_raster565(const bild_t *b, const uint16_t *lut, uint16_t *bild)
{
    szene_t z;
    szene_bauen(&z, b);
    for (int j = z.rect.y0; j <= z.rect.y1; j++) {
        uint16_t *zeile = bild + j * AUGEN_W;
        float Y = j + ABTAST;
        for (int i = z.rect.x0; i <= z.rect.x1; i++)
            zeile[i] = lut[(int)(szene_deckung(&z, i + ABTAST, Y) * 255.0f + 0.5f)];
    }
    return z.rect;
}

/* Abdeckung -> RGB565 wie tools/eyes.py to_bgr + cv2 BGR2BGR565 (gerundet, dann obere Bits) */
void augen_lut(uint8_t r, uint8_t g, uint8_t b, uint16_t *lut)
{
    for (int a = 0; a < 256; a++) {
        int rr = (int)(a / 255.0f * r + 0.5f), gg = (int)(a / 255.0f * g + 0.5f), bb = (int)(a / 255.0f * b + 0.5f);
        lut[a] = (uint16_t)(((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3));
    }
}
