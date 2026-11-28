/* Die 15 Ausdrücke des Augengenerators (augen.h), Zeile für Zeile aus tools/eyes.py (EMOTIONS) übertragen:
 * je eine Funktion t (s) -> Bild. tests/test_augen.py vergleicht die Parameter mit den Python-Funktionen.
 * Ein Ausdruck ändern: hier und in tools/eyes.py gleich.
 */
#include "augen.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define ZWEI_PI (2.0f * PI_F)

/* ---------------------------------------------------------------- Hilfen wie in tools/eyes.py */
static float smooth(float x)
{
    x = x < 0 ? 0 : (x > 1 ? 1 : x);
    return x * x * (3 - 2 * x);
}

static float lerp(float a, float b, float x) { return a + (b - a) * x; }

typedef struct { float t, v; } schluessel_t;

static float track(float t, const schluessel_t *k, int n)
{
    if (t <= k[0].t) return k[0].v;
    for (int i = 0; i + 1 < n; i++)
        if (t <= k[i + 1].t) return lerp(k[i].v, k[i + 1].v, smooth((t - k[i].t) / (k[i + 1].t - k[i].t)));
    return k[n - 1].v;
}
#define TRACK(t, ...) track((t), (const schluessel_t[]){__VA_ARGS__}, sizeof((schluessel_t[]){__VA_ARGS__}) / sizeof(schluessel_t))

static float envelope(float t, float dauer, float rein, float raus)
{
    return smooth(t / rein) * (1 - smooth((t - (dauer - raus)) / raus));
}

static float blink(float t, float t0, float dauer)
{
    float x = (t - t0) / dauer;
    if (x < 0 || x > 1) return 0;
    float s = sinf(PI_F * x);
    return s * s;
}

/* pose(base, ziel, x) für ein Feld: von der Grundstellung zum Zielwert */
#define Z(p, feld, wert, x) ((p).feld = lerp((p).feld, (wert), (x)))

static void neutral2(bild_t *b) { bild_neutral(b); }

/* ---------------------------------------------------------------- Ausdrücke */
static void angry(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.35f, 0.4f);
    float zit = e * sinf(ZWEI_PI * 11 * t) * 1.6f * (t > 0.4f);
    float puls = 0.05f * (blink(t, 0.9f, 0.25f) + blink(t, 1.45f, 0.25f));
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        Z(*p, dy, 6, e); Z(*p, w, 84, e); Z(*p, h, 86, e); Z(*p, r, 0.34f, e);
        Z(*p, top, 0.30f, e); Z(*p, tslope, -22, e); Z(*p, sc, 1 + puls, e);
        Z(*p, dx, i == 0 ? 4 : -4, e);
    }
    b->gx = zit;
}

static void cute(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.4f, 0.4f);
    float hops = 0.03f * sinf(ZWEI_PI * 1.5f * t);
    float hap = smooth((t - 1.45f) / 0.15f) * (1 - smooth((t - 2.05f) / 0.15f));
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        Z(*p, w, 100, e); Z(*p, h, 100, e); Z(*p, r, 1.0f, e); Z(*p, bcurve, 0.16f, e);
        Z(*p, dy, -4 * hops / 0.03f, e); Z(*p, sc, 1.0f + hops, e);
        p->kind = ART_HAPPY;
        p->mix = hap;
        p->blink = hap * 0.6f;
    }
    b->tilt = e * 9 * sinf(ZWEI_PI * (t - 0.4f) / 1.6f) * (t > 0.4f);
    b->gy = -3 * e;
}

static void doubt(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.4f, 0.45f);
    float blick = TRACK(t, {0.3f, 0}, {0.7f, 16}, {1.4f, 16}, {1.7f, -10}, {2.2f, -10}, {2.5f, 0});
    Z(b->l, dy, -10, e); Z(b->l, h, 104, e); Z(b->l, tslope, 10, e); Z(b->l, top, 0.06f, e);
    Z(b->r, dy, 4, e); Z(b->r, top, 0.40f, e); Z(b->r, tslope, -12, e); Z(b->r, bot, 0.18f, e); Z(b->r, w, 82, e);
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        ps[i]->dx += blick * e;
        ps[i]->dy += -5 * e * (fabsf(blick) > 1);
    }
    b->tilt = -4 * e;
}

static void embarrassed(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.25f, 0.35f);
    float blick = TRACK(t, {0.0f, 0}, {0.25f, -22}, {0.5f, -22}, {0.62f, 20}, {0.82f, 20}, {0.95f, -20},
                        {1.1f, -20}, {1.25f, 0});
    float chev = smooth((t - 1.2f) / 0.15f) * (1 - smooth((t - 2.35f) / 0.15f));
    float zit = sinf(ZWEI_PI * 9 * t) * 2.0f * chev;
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        p->dx = blick * e;
        p->dy = 6 * e;
        p->sc = 1 - 0.08f * e;
        p->kind = ART_CHEV;
        p->mix = chev;
        p->blink = chev * 0.7f;
    }
    b->gx = zit;
}

static void grievance(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.4f, 0.45f);
    float wackel = sinf(ZWEI_PI * 7 * t) * e;
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        Z(*p, dy, -6, e); Z(*p, w, 90, e); Z(*p, h, 98, e); Z(*p, r, 0.75f, e); Z(*p, top, 0.14f, e);
        Z(*p, tslope, 16, e); Z(*p, bot, 0.10f, e); Z(*p, pupil, 0.70f, e); Z(*p, ppy, -0.22f, e);
        Z(*p, hl, 1.0f, e);
        p->bot += 0.035f * wackel;
        p->dx += 0.8f * wackel;
    }
    if (t >= 0.7f && t <= 2.6f) b->r.tear = (t - 0.7f) / 1.9f;
}

static void hate(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.3f, 0.45f);
    float a = TRACK(t, {0.2f, PI_F}, {1.2f, 0.0f});
    float roll = smooth((t - 0.15f) / 0.25f) * (1 - smooth((t - 1.15f) / 0.3f));
    float dx = 22 * cosf(a) * roll, dy = -26 * sinf(a) * roll - 4 * roll;
    float flach = smooth((t - 1.1f) / 0.35f);
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        p->dx = dx + 14 * flach * smooth((t - 1.7f) / 0.35f) * e;
        p->dy = dy + 6 * flach * e;
        p->top = e * (0.12f * roll + 0.40f * flach);
        p->tslope = -10 * flach * e;
        p->bot = e * 0.20f * flach;
        p->w = lerp(78, 88, e * flach);
    }
}

static void laugh(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.25f, 0.3f);
    float hupf = -9 * fabsf(sinf(ZWEI_PI * 3.2f * t)) * e;
    float kipp = 5 * sinf(ZWEI_PI * 1.6f * t) * e;
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        ps[i]->kind = ART_HAPPY;
        ps[i]->mix = e;
        ps[i]->dy = hupf;
        ps[i]->blink = 0.7f * e;
    }
    b->tilt = kipp;
}

static void like(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.3f, 0.35f);
    float tt = fmodf(t - 0.35f, 0.85f);
    if (tt < 0) tt += 0.85f;                         /* Python-Modulo */
    float schlag = 0.13f * (blink(tt, 0.0f, 0.18f) + 0.7f * blink(tt, 0.2f, 0.18f)) * (t > 0.35f);
    float m = t / 0.3f < 1 ? t / 0.3f : 1;
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        p->kind = ART_HEART;
        p->mix = e;
        p->sc = 1 + (schlag + 0.12f * sinf(PI_F * m) * (t < 0.3f)) * e;
        p->dy = -4 * e + 3 * sinf(ZWEI_PI * 0.8f * t) * e;
    }
}

static void sad(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.6f, 0.5f);
    b->l.dx = -3;
    b->r.dx = 3;
    float langsam = blink(t, 1.6f, 0.6f);
    float blick = TRACK(t, {0.6f, 0}, {1.2f, -6}, {2.4f, -6}, {2.9f, 0});
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        Z(*p, dy, 14, e); Z(*p, top, 0.28f, e); Z(*p, tslope, 22, e); Z(*p, h, 88, e);
        p->blink = langsam;
        p->dx += blick * e;
    }
}

static void shy(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.45f, 0.45f);
    float zurueck = smooth((t - 1.2f) / 0.2f) * (1 - smooth((t - 1.6f) / 0.15f));
    float weg = 1 - zurueck;
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        Z(*p, sc, 0.84f, e); Z(*p, top, 0.36f, e); Z(*p, tslope, 6, e);
        p->dx += -24 * e * weg;
        p->dy += 16 * e * weg;
        p->top -= 0.25f * e * zurueck;
        p->blink = blink(t, 2.0f, 0.3f);
    }
    b->tilt = 6 * e * weg;
}

static void sleep_(float t, float T, bild_t *b)
{
    (void)T;
    neutral2(b);
    float top = TRACK(t, {0.2f, 0}, {1.0f, 0.55f}, {1.15f, 0.12f}, {2.0f, 0.72f}, {2.15f, 0.35f}, {3.0f, 0.88f});
    float zu = smooth((t - 2.85f) / 0.25f);
    float atem = 3 * sinf(ZWEI_PI * 0.6f * (t - 3.1f)) * smooth((t - 3.0f) / 0.3f);
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        p->top = top;
        p->tslope = 10 * (t / 1.0f < 1.0f ? t / 1.0f : 1.0f);
        p->dy = 8 * smooth(t / 3.0f) + atem;
        p->kind = ART_CLOSED;
        p->mix = zu;
    }
}

static void surprised(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.12f, 0.45f);
    float knall = TRACK(t, {0.0f, 0}, {0.08f, -0.12f}, {0.22f, 0.30f}, {0.4f, 0.16f}, {0.55f, 0.20f});
    float x = smooth(t / 0.22f) * e;
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        Z(*p, w, 90, x); Z(*p, h, 90, x); Z(*p, r, 1.0f, x); Z(*p, dy, -8, x); Z(*p, dx, i == 0 ? -8 : 8, x);
        p->sc = 1 + knall * e;
        p->blink = blink(t, 1.45f, 0.16f) + blink(t, 1.68f, 0.16f);
    }
}

static void angst(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.15f, 0.4f);
    float blick = TRACK(t, {0.2f, 0}, {0.4f, -0.7f}, {0.85f, -0.7f}, {1.0f, 0.7f}, {1.45f, 0.7f}, {1.6f, -0.5f},
                        {1.95f, -0.5f}, {2.1f, 0});
    float zittern = 2.2f * sinf(ZWEI_PI * 13 * t) * e;
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        Z(*p, sc, 1.08f, e); Z(*p, top, 0.14f, e); Z(*p, tslope, 14, e); Z(*p, pupil, 0.30f, e);
        Z(*p, hl, 1.0f, e); Z(*p, dy, -4, e);
        p->ppx = blick * e;
        p->ppy = 0.1f * e;
        p->dx += zittern + 6 * blick * e;
        p->blink = blink(t, 2.25f, 0.14f);
    }
}

static void aufwachen(float t, float T, bild_t *b)
{
    (void)T;
    neutral2(b);
    float zu = TRACK(t, {0.3f, 1.0f}, {1.0f, 0.45f}, {1.25f, 0.8f}, {1.9f, 0.0f});
    float blick = TRACK(t, {2.0f, 0}, {2.3f, -0.6f}, {2.7f, -0.6f}, {3.05f, 0.6f}, {3.45f, 0.6f}, {3.75f, 0});
    b->l.dx = b->r.dx = 34.0f * blick;
    b->l.w = 78.0f * (1 - 0.10f * blick);
    b->r.w = 78.0f * (1 + 0.10f * blick);
    b->l.blink = b->r.blink = zu + blink(t, 4.0f, 0.2f);
}

static void vertigo(float t, float T, bild_t *b)
{
    neutral2(b);
    float e = envelope(t, T, 0.3f, 0.35f);
    auge_t *ps[2] = {&b->l, &b->r};
    for (int i = 0; i < 2; i++) {
        auge_t *p = ps[i];
        float ph = ZWEI_PI * 1.4f * t + i * PI_F;
        p->kind = ART_SPIRAL;
        p->mix = e;
        p->spin = 6 * t;
        p->dx = 7 * cosf(ph) * e;
        p->dy = 7 * sinf(ph) * e;
        p->sc = 1.05f;
    }
    b->tilt = 10 * sinf(ZWEI_PI * 0.9f * t) * e;
}

/* Name -> (Funktion, Dauer in s), Reihenfolge wie EMOTIONS in tools/eyes.py */
typedef struct {
    const char *name;
    void (*fn)(float, float, bild_t *);
    float dauer;
} ausdruck_t;

static const ausdruck_t AUSDRUECKE[] = {
    {"Angry", angry, 2.4f},         {"angst", angst, 2.6f},       {"aufwachen", aufwachen, 4.4f},
    {"cute", cute, 3.0f},           {"doubt", doubt, 3.0f},       {"embarrassed", embarrassed, 2.8f},
    {"grievance", grievance, 3.0f}, {"hate", hate, 2.8f},         {"laugh", laugh, 2.2f},
    {"like", like, 2.8f},           {"sad", sad, 3.0f},           {"shy", shy, 2.8f},
    {"sleep", sleep_, 4.0f},        {"surprised", surprised, 2.4f}, {"vertigo", vertigo, 2.4f},
};
#define N_AUSDRUECKE ((int)(sizeof(AUSDRUECKE) / sizeof(AUSDRUECKE[0])))

int augen_ausdruck_n(void) { return N_AUSDRUECKE; }

const char *augen_ausdruck_name(int i) { return i >= 0 && i < N_AUSDRUECKE ? AUSDRUECKE[i].name : 0; }

float augen_ausdruck_dauer(int i) { return i >= 0 && i < N_AUSDRUECKE ? AUSDRUECKE[i].dauer : 0.0f; }

int augen_ausdruck_index(const char *name)
{
    for (int i = 0; i < N_AUSDRUECKE; i++)
        if (strcmp(AUSDRUECKE[i].name, name) == 0) return i;
    return -1;
}

void augen_ausdruck_bild(int i, float t, bild_t *b)
{
    if (i < 0 || i >= N_AUSDRUECKE) {
        bild_neutral(b);
        return;
    }
    AUSDRUECKE[i].fn(t, AUSDRUECKE[i].dauer, b);
}
