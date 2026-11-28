/* Autonome Ebene des Augengenerators (augen.h) und Befehle aus /tmp/lcd0.augen.
 *
 * Die Augen leben von selbst: Lidschlag nach Zufall (Rate je Minute, 10 % Doppelblinzler, Blinzeln bei großem
 * Blicksprung), Sakkaden (Blickwechsel springt in 30–110 ms, kleine Änderungen folgen weich), Mikrosakkaden alle
 * 1–3 s, Atmen ±0,8 px. Wer die Augen steuert (xgonav, tools/emotion.sh), schickt nur Ziele:
 *
 *   an | aus                 lebende Augen dauerhaft / nichts zeichnen (letztes Bild bleibt stehen)
 *   ausdruck NAME [STÄRKE]   Ausdruck einmal (augen_ausdruck.c); bei "an" über die lebenden Augen geblendet
 *   blick X Y                -1..1, X > 0 = rechts im Bild, Y > 0 = unten
 *   nah M                    Abstand des fixierten Ziels (m, 0 = aus): unter 0,6 m rücken die Augen zusammen
 *   lid L [S]                Grundlid 0 offen .. 1 zu, Übergang in S s
 *   farbe RRGGBB|NAME [S]    Augenfarbe (Namen wie tools/eyes.py PALETTE und tuerkis), Überblendung in S s
 *   rate N | blinzeln        Lidschläge je Minute (0 = keine eigenen) / sofort einer
 *   atmen 0|1                Atmen aus/an
 *   abbrechen                laufenden Ausdruck beenden
 */
#include "augen.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define PI_F 3.14159265358979f
#define BLICK_PX 34.0f          /* Blick: das ganze Auge wandert, das Auge in Blickrichtung 10 % breiter */
#define BLICK_Y_PX 22.0f
#define VERGENZ_PX 9.0f
#define LIDSCHLAG_S 0.26f
#define VERFOLG_TAU 0.12f       /* kleine Blickänderungen */
#define SAKKADE_AB 0.15f        /* größere springen */

struct leben {
    uint32_t rng;
    int an, atmen;
    float bx, by, x, y;
    int sakkade;
    float sx0, sy0, sx1, sy1, s_dauer;
    double s_t0;
    float mx, my, mzx, mzy;
    double m_naechst;
    float nah, vg;
    float lid_ziel, lid, lid_tau;
    float rate;
    double b_naechst, b_t0;
    uint8_t f0[3], f1[3];
    double f_t0;
    float f_dauer;
    int ausdruck;
    double a_t0;
    float staerke;
    double t_last;
};

static float zufall(leben_t *l)                 /* 0..1 */
{
    uint32_t x = l->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    l->rng = x;
    return (x >> 8) / 16777216.0f;
}

static float klemm(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static float glatt(float x)
{
    x = klemm(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}
static float folge(float ist, float soll, float dt, float tau)
{
    return tau <= 0 ? soll : ist + (soll - ist) * (1.0f - expf(-dt / tau));
}

static void lidschlag_planen(leben_t *l, double jetzt)
{
    if (l->rate <= 0) {
        l->b_naechst = 1e18;
        return;
    }
    float mittel = 60.0f / l->rate;
    float dt = -logf(1.0f - zufall(l) * 0.999f) * mittel;      /* zufällig, im Mittel 60/rate s */
    if (dt < 1.2f) dt = 1.2f;
    l->b_naechst = jetzt + dt;
}

leben_t *leben_neu(uint32_t seed)
{
    leben_t *l = calloc(1, sizeof(*l));
    if (!l) return 0;
    l->rng = seed ? seed : 0x9e3779b9u;
    l->atmen = 1;
    l->lid_tau = 0.08f;
    l->rate = 15.0f;
    l->f0[0] = l->f1[0] = 0;
    l->f0[1] = l->f1[1] = 225;
    l->f0[2] = l->f1[2] = 210;
    l->f_dauer = 0.0f;
    l->ausdruck = -1;
    l->b_t0 = -10.0;
    l->b_naechst = 2.0;
    l->m_naechst = 1.0;
    return l;
}

void leben_frei(leben_t *l) { free(l); }

void leben_farbe(const leben_t *l, double jetzt, uint8_t rgb[3])
{
    float u = l->f_dauer > 0 ? klemm((float)((jetzt - l->f_t0) / l->f_dauer), 0.0f, 1.0f) : 1.0f;
    for (int k = 0; k < 3; k++) rgb[k] = (uint8_t)(l->f0[k] + (l->f1[k] - l->f0[k]) * u + 0.5f);
}

static int farbe_lesen(const char *s, uint8_t rgb[3])
{
    static const struct { const char *name; uint8_t r, g, b; } NAMEN[] = {
        {"tuerkis", 0, 225, 210}, {"rot", 255, 80, 70}, {"rosa", 255, 120, 200},
        {"gelb", 255, 200, 0},    {"gruen", 60, 230, 90}, {"lila", 170, 110, 255},
    };
    for (unsigned i = 0; i < sizeof(NAMEN) / sizeof(NAMEN[0]); i++)
        if (strcasecmp(s, NAMEN[i].name) == 0) {
            rgb[0] = NAMEN[i].r;
            rgb[1] = NAMEN[i].g;
            rgb[2] = NAMEN[i].b;
            return 1;
        }
    if (s[0] == '#') s++;
    if (strlen(s) != 6) return 0;
    char *ende;
    unsigned long v = strtoul(s, &ende, 16);
    if (*ende) return 0;
    rgb[0] = (uint8_t)(v >> 16);
    rgb[1] = (uint8_t)(v >> 8);
    rgb[2] = (uint8_t)v;
    return 1;
}

static int zahl(const char *s, float *v)
{
    if (!s) return 0;
    char *ende;
    float f = strtof(s, &ende);
    if (ende == s || *ende || !isfinite(f)) return 0;
    *v = f;
    return 1;
}

int leben_befehl(leben_t *l, const char *zeile, double jetzt)
{
    char buf[128];
    strncpy(buf, zeile, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    char *tok[4] = {0}, *sp = 0;
    int n = 0;
    for (char *t = strtok_r(buf, " \t\r\n", &sp); t && n < 4; t = strtok_r(0, " \t\r\n", &sp)) tok[n++] = t;
    if (n == 0) return 0;
    const char *c = tok[0];
    float a, b;
    if (!strcmp(c, "an")) {
        if (!l->an) lidschlag_planen(l, jetzt);
        l->an = 1;
    } else if (!strcmp(c, "aus")) {
        /* Blick, Schielen und Lid zurück in die Mitte: sonst stünden die nächsten Ausdrücke (und das nächste "an")
         * dort, wohin zuletzt geschaut wurde (am Rider 2026-11-28: nach einem Test alles zu weit links) */
        l->an = 0;
        l->bx = l->by = l->x = l->y = 0.0f;
        l->mx = l->my = l->mzx = l->mzy = 0.0f;
        l->sakkade = 0;
        l->nah = l->vg = 0.0f;
        l->lid = l->lid_ziel = 0.0f;
    } else if (!strcmp(c, "ausdruck") && n >= 2) {
        int i = augen_ausdruck_index(tok[1]);
        if (i < 0) return 0;
        l->ausdruck = i;
        l->a_t0 = jetzt;
        l->staerke = 1.0f;
        if (n >= 3 && zahl(tok[2], &a)) l->staerke = klemm(a, 0.0f, 1.0f);
    } else if (!strcmp(c, "blick") && n >= 3 && zahl(tok[1], &a) && zahl(tok[2], &b)) {
        a = klemm(a, -1.0f, 1.0f);
        b = klemm(b, -1.0f, 1.0f);
        float amp = sqrtf((a - l->x) * (a - l->x) + (b - l->y) * (b - l->y));
        if (amp > SAKKADE_AB) {
            l->sakkade = 1;
            l->sx0 = l->x;
            l->sy0 = l->y;
            l->sx1 = a;
            l->sy1 = b;
            l->s_t0 = jetzt;
            l->s_dauer = 0.03f + 0.04f * amp;
            if (amp > 0.5f && zufall(l) < 0.5f && l->ausdruck < 0) l->b_t0 = jetzt;
        }
        l->bx = a;
        l->by = b;
    } else if (!strcmp(c, "nah") && n >= 2 && zahl(tok[1], &a)) {
        l->nah = a;
    } else if (!strcmp(c, "lid") && n >= 2 && zahl(tok[1], &a)) {
        l->lid_ziel = klemm(a, 0.0f, 1.0f);
        l->lid_tau = (n >= 3 && zahl(tok[2], &b) && b > 0) ? b / 3.0f : 0.08f;
    } else if (!strcmp(c, "farbe") && n >= 2) {
        uint8_t rgb[3];
        if (!farbe_lesen(tok[1], rgb)) return 0;
        leben_farbe(l, jetzt, l->f0);
        memcpy(l->f1, rgb, 3);
        l->f_t0 = jetzt;
        l->f_dauer = (n >= 3 && zahl(tok[2], &a) && a > 0) ? a : 0.3f;
    } else if (!strcmp(c, "rate") && n >= 2 && zahl(tok[1], &a)) {
        l->rate = a < 0 ? 0 : a;
        lidschlag_planen(l, jetzt);
    } else if (!strcmp(c, "abbrechen")) {        /* laufenden Ausdruck beenden (Daemon: fremder Besitzer) */
        l->ausdruck = -1;
    } else if (!strcmp(c, "blinzeln")) {
        l->b_t0 = jetzt;
    } else if (!strcmp(c, "atmen") && n >= 2 && zahl(tok[1], &a)) {
        l->atmen = a != 0;
    } else {
        return 0;
    }
    return 1;
}

int leben_ausdruck_laeuft(const leben_t *l, double jetzt)
{
    return l->ausdruck >= 0 && jetzt < l->a_t0 + augen_ausdruck_dauer(l->ausdruck);
}

double leben_naechstes(const leben_t *l, double jetzt)
{
    (void)jetzt;
    return l->b_naechst < l->m_naechst ? l->b_naechst : l->m_naechst;
}

static float lidschlag_form(double seit)       /* schnell zu, langsamer auf */
{
    float x = (float)(seit / LIDSCHLAG_S);
    if (x < 0 || x >= 1) return 0.0f;
    return x < 0.35f ? glatt(x / 0.35f) : 1.0f - glatt((x - 0.35f) / 0.65f);
}

static void mische(auge_t *o, const auge_t *a, const auge_t *b, float w)
{
    const float *pa = &a->dx, *pb = &b->dx;
    float *po = &o->dx;
    for (int k = 0; k < (int)((&a->blink - &a->dx) + 1); k++) po[k] = pa[k] + (pb[k] - pa[k]) * w;
    o->kind = w > 0 ? b->kind : a->kind;
}

int leben_bild(leben_t *l, double jetzt, bild_t *b)
{
    float dt = (float)(jetzt - l->t_last);
    if (dt < 0 || dt > 0.2f) dt = 0.2f;
    l->t_last = jetzt;

    /* Ausdruck */
    int aus_i = l->ausdruck;
    float aus_t = 0, aus_T = 0;
    int ende = 0;
    if (aus_i >= 0) {
        aus_T = augen_ausdruck_dauer(aus_i);
        aus_t = (float)(jetzt - l->a_t0);
        if (aus_t >= aus_T) {                    /* letztes Bild genau am Ende, dann vorbei */
            aus_t = aus_T;
            ende = 1;
            l->ausdruck = -1;
        }
    }
    if (!l->an) {
        if (aus_i < 0) return 0;
        augen_ausdruck_bild(aus_i, aus_t, b);    /* ohne lebende Augen: Ausdruck wie vorgerendert */
        return 1;
    }

    /* Blick: Sakkade oder weiches Folgen */
    int bewegt = 0;
    if (l->sakkade) {
        float u = (float)((jetzt - l->s_t0) / l->s_dauer);
        if (u >= 1) {
            l->sakkade = 0;
            l->x = l->sx1;
            l->y = l->sy1;
        } else {
            float g = glatt(u);
            l->x = l->sx0 + (l->sx1 - l->sx0) * g;
            l->y = l->sy0 + (l->sy1 - l->sy0) * g;
        }
        bewegt = 1;
    }
    if (!l->sakkade) {
        float nx = folge(l->x, l->bx, dt, VERFOLG_TAU), ny = folge(l->y, l->by, dt, VERFOLG_TAU);
        if (fabsf(nx - l->x) > 1e-3f || fabsf(ny - l->y) > 1e-3f) bewegt = 1;
        l->x = nx;
        l->y = ny;
    }
    /* Mikrosakkaden: winzige Sprünge, die zurückkehren */
    if (jetzt >= l->m_naechst) {
        l->mzx = (zufall(l) - 0.5f) * 0.08f;
        l->mzy = (zufall(l) - 0.5f) * 0.06f;
        l->m_naechst = jetzt + 1.0 + 2.0 * zufall(l);
    }
    float omx = l->mx, omy = l->my;
    l->mx = folge(l->mx, l->mzx, dt, 0.02f);
    l->my = folge(l->my, l->mzy, dt, 0.02f);
    l->mzx *= expf(-dt / 0.6f);
    l->mzy *= expf(-dt / 0.6f);
    if (fabsf(l->mx - omx) > 2e-3f || fabsf(l->my - omy) > 2e-3f) bewegt = 1;
    /* Vergenz und Lid */
    float vz = (l->nah > 0 && l->nah < 0.6f) ? klemm((0.6f - l->nah) / 0.45f, 0.0f, 1.0f) : 0.0f;
    float ovg = l->vg, olid = l->lid;
    l->vg = folge(l->vg, vz, dt, 0.15f);
    l->lid = folge(l->lid, l->lid_ziel, dt, l->lid_tau);
    if (fabsf(l->vg - ovg) > 1e-3f || fabsf(l->lid - olid) > 1e-3f) bewegt = 1;
    /* Lidschlag (nicht während eines Ausdrucks, nicht bei geschlossenem Grundlid) */
    if (jetzt >= l->b_naechst) {
        if (aus_i < 0 && l->lid < 0.95f) {
            l->b_t0 = jetzt;
            if (zufall(l) < 0.1f) l->b_naechst = jetzt + 0.4;     /* Doppelblinzler */
            else lidschlag_planen(l, jetzt);
        } else {
            lidschlag_planen(l, jetzt);
        }
    }
    float bl = lidschlag_form(jetzt - l->b_t0);
    if (bl > 0) bewegt = 1;
    if (l->f_dauer > 0 && jetzt < l->f_t0 + l->f_dauer) bewegt = 1;

    /* lebendes Bild */
    bild_t lb;
    bild_neutral(&lb);
    float x = l->x + l->mx, y = l->y + l->my;
    float atem = l->atmen ? 0.8f * sinf(2.0f * PI_F * 0.25f * (float)fmod(jetzt, 4.0)) : 0.0f;
    float zu = 1.0f - (1.0f - l->lid) * (1.0f - bl);
    lb.l.dx = BLICK_PX * x + VERGENZ_PX * l->vg;
    lb.r.dx = BLICK_PX * x - VERGENZ_PX * l->vg;
    lb.l.w = 78.0f * (1.0f - 0.10f * x);        /* Auge in Blickrichtung etwas breiter (tools/eyes.py) */
    lb.r.w = 78.0f * (1.0f + 0.10f * x);
    lb.l.dy = lb.r.dy = BLICK_Y_PX * y + atem;
    lb.l.blink = lb.r.blink = zu;

    if (aus_i < 0) {
        *b = lb;
        return bewegt ? 1 : (l->atmen ? 2 : 3);
    }
    /* Ausdruck über die lebenden Augen: Blick bleibt dazu, Ein- und Ausblenden je 0,15 s */
    bild_t e;
    augen_ausdruck_bild(aus_i, aus_t, &e);
    e.l.dx += lb.l.dx;
    e.r.dx += lb.r.dx;
    e.l.dy += lb.l.dy;
    e.r.dy += lb.r.dy;
    float w = glatt(aus_t / 0.15f) * (1.0f - glatt((aus_t - (aus_T - 0.15f)) / 0.15f)) * l->staerke;
    if (ende) w = 0.0f;
    b->tilt = lb.tilt + (e.tilt - lb.tilt) * w;
    b->gx = lb.gx + (e.gx - lb.gx) * w;
    b->gy = lb.gy + (e.gy - lb.gy) * w;
    mische(&b->l, &lb.l, &e.l, w);
    mische(&b->r, &lb.r, &e.r, w);
    if (ende) lidschlag_planen(l, jetzt);
    return 1;
}
