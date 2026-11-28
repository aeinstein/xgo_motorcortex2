/* Augengenerator des Display-Daemons: Augen als Formen, live gezeichnet (docs/display-emotions.md).
 *
 * Parametermodell 1:1 aus tools/eyes.py (NEUTRAL, draw_kind, draw_eye, render), nur ohne Vorrendern:
 *   augen.c          Raster (Abstandsfunktionen, Kantenglättung analytisch statt 4-fach Überabtasten)
 *   augen_ausdruck.c die 15 Ausdrücke als Funktionen t -> Parameter (wie EMOTIONS in tools/eyes.py)
 *   augen_leben.c    autonome Ebene (Lidschlag, Sakkaden, Atmen, Federn) und Befehle aus /tmp/lcd0.augen
 *
 * Ohne Abhängigkeiten außer libm; dieselben Quellen bauen den Daemon und libxgoaugen.so (Tests, Vorschau am PC).
 * ABI-Nummer AUGEN_ABI bei geänderten Strukturen erhöhen (tests/test_augen.py prüft sie).
 */
#ifndef AUGEN_H_EINGEBUNDEN
#define AUGEN_H_EINGEBUNDEN

#include <stdint.h>

#define AUGEN_ABI 1
#define AUGEN_W 320
#define AUGEN_H 240

enum { ART_EYE = 0, ART_HEART, ART_HAPPY, ART_CLOSED, ART_SPIRAL, ART_CHEV };

/* Ein Auge, Felder wie NEUTRAL in tools/eyes.py (Bildschirm-Pixel bzw. Anteile). */
typedef struct {
    float dx, dy, w, h, r, rot, sc;
    float top, tslope, bot, bcurve;
    float pupil, ppx, ppy, hl;
    float mix, spin, tear, blink;
    int32_t kind;                       /* ART_*: Zielform, zu der über mix überblendet wird */
} auge_t;

/* Ein Bild: global (tilt in Grad, gx/gy in px) und die beiden Augen (l = links im Bild). */
typedef struct {
    float tilt, gx, gy;
    auge_t l, r;
} bild_t;

/* Begrenzungsrechteck in Bildschirm-Pixeln, einschließlich; leer: x0 > x1. */
typedef struct {
    int32_t x0, y0, x1, y1;
} rechteck_t;

/* ---- augen.c */
int augen_abi(void);
void auge_neutral(auge_t *a);
void bild_neutral(bild_t *b);
/* Abdeckung 0..255 je Pixel (320 x 240, Zeile für Zeile) nur innerhalb des zurückgegebenen Rechtecks; außerhalb
 * bleibt alpha unberührt (der Aufrufer löscht das alte Rechteck). */
rechteck_t augen_raster(const bild_t *b, uint8_t *alpha);
/* Wie augen_raster, schreibt aber gleich RGB565 (Farbtabelle lut, 256 Einträge) ins Bild 320 x 240. */
rechteck_t augen_raster565(const bild_t *b, const uint16_t *lut, uint16_t *bild);
void augen_lut(uint8_t r, uint8_t g, uint8_t b, uint16_t *lut);

/* ---- augen_ausdruck.c */
int augen_ausdruck_n(void);
const char *augen_ausdruck_name(int i);
float augen_ausdruck_dauer(int i);
int augen_ausdruck_index(const char *name);     /* -1: unbekannt */
void augen_ausdruck_bild(int i, float t, bild_t *b);

/* ---- augen_leben.c */
typedef struct leben leben_t;
leben_t *leben_neu(uint32_t seed);
void leben_frei(leben_t *l);
/* Eine Befehlszeile (ohne Zeilenende); 1 = verstanden, 0 = ignoriert. */
int leben_befehl(leben_t *l, const char *zeile, double jetzt);
/* Bild zur Zeit jetzt. Rückgabe: 0 = nichts zu zeichnen (aus), 1 = Bild, bewegt sich (nächstes in 40 ms),
 * 2 = Bild, nur Atmen (nächstes in 100 ms), 3 = Bild, steht (nächstes erst zu leben_naechstes oder bei einem
 * Befehl). "aus" setzt Blick, Schielen und Lid auf die Mitte zurück. */
int leben_bild(leben_t *l, double jetzt, bild_t *b);
int leben_ausdruck_laeuft(const leben_t *l, double jetzt);
/* Augenfarbe zur Zeit jetzt (Überblendung) */
void leben_farbe(const leben_t *l, double jetzt, uint8_t rgb[3]);
/* Nächster Zeitpunkt, an dem sich von selbst etwas ändert (Lidschlag, Mikrosakkade), für den Schlaf im Leerlauf */
double leben_naechstes(const leben_t *l, double jetzt);

#endif
