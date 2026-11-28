#include "driver_st7789_basic.h"
#include "augen.h"
//#include "shell.h"
#include <getopt.h>
#include <sys/types.h>
#include <sys/socket.h>
//#include <netinet/in.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <errno.h>
#include <grp.h>
#include <time.h>

static int framesize = 320 * 240 * 2;
static char *framebuffer;
static char *fb_device = "/tmp/lcd0";
static int running = 1;
// SIGUSR1 = Anzeige aus + Controller schlafen, SIGUSR2 = wach. Der Ruhemodus nutzt das nicht mehr (das
// Display wurde davon nicht dunkel, dort stehen jetzt die Schlafaugen); deploy/xgo-ruhe schickt nur noch SIGUSR2.
// Beide Signale sind in allen Threads blockiert und werden im Hauptthread per sigwait angenommen;
// der Mutex hält SPI-Zugriffe von Lese-Thread (Bild), Augengenerator und Hauptthread (Schlafbefehl) auseinander.
static pthread_mutex_t spi_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int asleep = 0;

// Kein Netz: wlan0 ohne IPv4 (alle NETZ_PRUEF_S geprüft) -> oben rechts ein WLAN-Symbol mit rotem Strich über jedem
// Bild; framebuffer behält das Bild ohne Symbol, beim Wechsel wird es neu gezeichnet (auch bei stehendem Bild).
#define NETZ_PRUEF_S 2
#define ICON 24
#define ICON_X (320 - ICON - 4)
#define ICON_Y 4
static uint16_t icon[ICON * ICON];
static char *out;                          // angezeigtes Bild (framebuffer + Symbol), auch Puffer für Teilfenster
static volatile int kein_netz = 0;
static int bild_da = 0;

// Augengenerator (augen.h, docs/display-emotions.md): Befehle als Textzeilen in /tmp/lcd0.augen, gezeichnet wird hier.
// Rohbilder auf /tmp/lcd0 haben Vorrang; während eines Ausdrucks hält der Daemon /tmp/lcd0.owner (wie tools/emotion.sh),
// dann pausiert xgonav seine Ausgabe, und ein neuerer Besitzer (Player, Gegensprechen) bricht den Ausdruck ab.
static const char *augen_fifo = "/tmp/lcd0.augen";
static const char *owner_datei = "/tmp/lcd0.owner";
static const char *stat_datei = "/tmp/lcd0.augen.stat";
#define ROH_PAUSE_S 0.5                    // so lange nach dem letzten Rohbild zeichnet der Generator nicht
#define BESITZ_MAX_S 10.0                  // wie emotion.active(): ältere Besitzer-Einträge zählen nicht
static pthread_mutex_t augen_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t augen_cond;
static leben_t *leben;
static int neuer_ausdruck = 0;             // unter augen_lock: Befehl "ausdruck" kam, Besitz übernehmen
static double roh_t = -1e9;                // unter spi_lock: Zeit des letzten Rohbilds
static volatile int generator_besitzt = 0; // Daemon steht in /tmp/lcd0.owner (Ausdruck läuft)
static int gen_voll = 1;                   // unter spi_lock: nächstes Generatorbild ganz zeichnen

static double jetzt_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void icon_bauen(void) {
    const uint16_t schwarz = 0x0000, weiss = 0xFFFF, rot = 0xF800;
    for (int y = 0; y < ICON; y++) {
        for (int x = 0; x < ICON; x++) {
            uint16_t c = schwarz;
            float dx = x + 0.5f - ICON / 2.0f, dy = (ICON - 3) - (y + 0.5f);   // Fußpunkt unten Mitte
            float d = sqrtf(dx * dx + dy * dy);
            if (dy > 0 && fabsf(dx) <= dy) {                                  // Fächer ±45°
                if (d < 2.5f || fabsf(d - 8) < 1.3f || fabsf(d - 13.5f) < 1.3f || fabsf(d - 19) < 1.3f) c = weiss;
            } else if (d < 2.5f) {
                c = weiss;
            }
            if (fabsf((float)x - (float)y) < 1.6f && x > 1 && x < ICON - 2) c = rot;   // Querstrich
            icon[y * ICON + x] = c;
        }
    }
}

static int wlan_hat_ip(void) {
    struct ifaddrs *ifa, *p;
    int ok = 0;
    if (getifaddrs(&ifa) != 0) return 1;           // im Zweifel kein Symbol
    for (p = ifa; p; p = p->ifa_next) {
        if (p->ifa_addr && p->ifa_addr->sa_family == AF_INET && p->ifa_name && strcmp(p->ifa_name, "wlan0") == 0) ok = 1;
    }
    freeifaddrs(ifa);
    return ok;
}

// unter spi_lock: Fenster (einschließlich) aus framebuffer, mit Symbol, ans Display. Der Treiber verlangt
// x0 < x1 und y0 < y1.
static void anzeigen_fenster(int x0, int y0, int x1, int y1) {
    if (asleep || !bild_da) return;
    if (x1 <= x0) x1 = x0 + 1;
    if (y1 <= y0) y1 = y0 + 1;
    if (x1 > 319) { x1 = 319; if (x0 >= x1) x0 = x1 - 1; }
    if (y1 > 239) { y1 = 239; if (y0 >= y1) y0 = y1 - 1; }
    const uint16_t *fb = (const uint16_t *)framebuffer;
    uint16_t *o = (uint16_t *)out;
    int bw = x1 - x0 + 1;
    for (int y = y0; y <= y1; y++) {
        memcpy(o + (y - y0) * bw, fb + y * 320 + x0, bw * 2);
        if (kein_netz && y >= ICON_Y && y < ICON_Y + ICON) {
            for (int x = x0; x <= x1; x++)
                if (x >= ICON_X && x < ICON_X + ICON) o[(y - y0) * bw + (x - x0)] = icon[(y - ICON_Y) * ICON + (x - ICON_X)];
        }
    }
    st7789_basic_draw_picture_16bits(x0, y0, x1, y1, o);
}

// unter spi_lock
static void anzeigen(void) {
    anzeigen_fenster(0, 0, 319, 239);
}

static void *netz_pruefen(void *arg) {
    (void)arg;
    while (running) {
        int neu = !wlan_hat_ip();
        if (neu != kein_netz) {
            pthread_mutex_lock(&spi_lock);
            kein_netz = neu;
            anzeigen();
            pthread_mutex_unlock(&spi_lock);
        }
        sleep(NETZ_PRUEF_S);
    }
    return NULL;
}

/**
 * @brief     signal handler
 * @param[in] signum signal number
 * @note      none
 */
static void a_sig_handler(int signum)
{
    if (SIGINT == signum)
    {
        st7789_interface_debug_print("st7789: close the server.\n");
        running = 0;
        exit(0);
    }

    return;
}

static void *read_framebuffer(void *arg){
    (void)arg;
    ssize_t bytes;
    struct stat sb;

    int rx_fd;
    uint8_t *rx;

    if (stat(fb_device, &sb) == -1) st7789_interface_debug_print("can't stat input file\n");

    rx_fd = open(fb_device, O_RDONLY);
    if (rx_fd < 0) st7789_interface_debug_print("can't create input device\n");

    rx = malloc(framesize);
    if (!rx) st7789_interface_debug_print("can't allocate rx buffer\n");

    while(running) {
        // read full frame from device

        int total_left = framesize;
        char *buffer_pointer = rx;

        while(total_left > 0) {
            bytes = read(rx_fd, buffer_pointer, total_left);

            if(bytes <= 0) {
                if(bytes < 0) {
                    st7789_interface_debug_print("device closed\n");
                    return NULL;
                }

                // no bytes so reset framepointer
                total_left = framesize;
                buffer_pointer = rx;
                usleep(asleep ? 200000 : 10000);    // sleep when no data (Ruhemodus: seltener)

            } else {
                total_left -= bytes;
                buffer_pointer += bytes;
            }
        }

        // Bild vollständig. Während eines eigenen Ausdrucks verworfen (xgonav pausiert dann ohnehin), sonst
        // gewinnt es gegen den Augengenerator; über anzeigen(), damit das WLAN-Symbol mitkommt.
        pthread_mutex_lock(&spi_lock);
        roh_t = jetzt_s();
        if (!generator_besitzt) {
            memcpy(framebuffer, rx, framesize);
            bild_da = 1;
            gen_voll = 1;
            anzeigen();     // im Ruhemodus (asleep) verworfen
        }
        pthread_mutex_unlock(&spi_lock);

        //  draw battery level
        //st7789_basic_string(250, 220, "Batt: 10%", 9, 0xFFFF, ST7789_FONT_16);
    }

    return NULL;
}

/* ---------------------------------------------------------------- Augengenerator */

// PID in /tmp/lcd0.owner (0 = leer/unlesbar) und Alter des Eintrags
static int besitzer_lesen(double *alter) {
    char buf[32];
    struct stat st;
    *alter = 1e9;
    int fd = open(owner_datei, O_RDONLY);
    if (fd < 0) return 0;
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    if (fstat(fd, &st) == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        *alter = (ts.tv_sec - st.st_mtim.tv_sec) + (ts.tv_nsec - st.st_mtim.tv_nsec) * 1e-9;
    }
    close(fd);
    if (n <= 0) return 0;
    buf[n] = 0;
    return atoi(buf);
}

static int fremder_besitzer(void) {
    double alter;
    int pid = besitzer_lesen(&alter);
    return pid != 0 && pid != getpid() && alter < BESITZ_MAX_S;
}

static void besitz_schreiben(int pid) {
    int fd = open(owner_datei, O_WRONLY | O_TRUNC);
    if (fd < 0) return;
    if (pid) {
        char buf[32];
        int n = snprintf(buf, sizeof(buf), "%d\n", pid);
        if (write(fd, buf, n) < 0) { /* egal */ }
    }
    close(fd);
}

static void besitz_abgeben(void) {
    double alter;
    if (besitzer_lesen(&alter) == getpid()) besitz_schreiben(0);
    generator_besitzt = 0;
}

static void *augen_lesen(void *arg) {
    (void)arg;
    // Lesen und selbst einen Schreiber halten (O_RDWR): kein EOF, wenn alle Schreiber schließen, und Schreiber
    // finden immer einen Leser, solange der Daemon läuft (ohne ihn: ENXIO bzw. Zeitüberschreitung beim Öffnen).
    int fd = open(augen_fifo, O_RDWR);
    if (fd < 0) {
        st7789_interface_debug_print("st7789: kein Augen-FIFO\n");
        return NULL;
    }
    char buf[1024];
    size_t n = 0;
    while (running) {
        ssize_t r = read(fd, buf + n, sizeof(buf) - 1 - n);
        if (r <= 0) {
            if (r < 0 && errno == EINTR) continue;
            usleep(100000);
            continue;
        }
        n += (size_t)r;
        char *anfang = buf, *nl;
        while ((nl = memchr(anfang, '\n', (size_t)(buf + n - anfang))) != NULL) {
            *nl = 0;
            pthread_mutex_lock(&augen_lock);
            if (leben_befehl(leben, anfang, jetzt_s()) && strncmp(anfang, "ausdruck", 8) == 0) neuer_ausdruck = 1;
            pthread_cond_signal(&augen_cond);
            pthread_mutex_unlock(&augen_lock);
            anfang = nl + 1;
        }
        n = (size_t)(buf + n - anfang);
        memmove(buf, anfang, n);
        if (n >= sizeof(buf) - 1) n = 0;            // überlange Zeile verwerfen
    }
    return NULL;
}

static int bild_gleich(const bild_t *a, const bild_t *b) {
    const float *pa = (const float *)a, *pb = (const float *)b;
    // Felder bis auf kind sind float: tilt, gx, gy, dann zweimal auge_t
    if (a->l.kind != b->l.kind || a->r.kind != b->r.kind) return 0;
    for (size_t k = 0; k < sizeof(bild_t) / sizeof(float); k++) {
        if ((const void *)(pa + k) == (const void *)&a->l.kind || (const void *)(pa + k) == (const void *)&a->r.kind) continue;
        if (fabsf(pa[k] - pb[k]) > 1e-3f) return 0;
    }
    return 1;
}

static void stat_schreiben(double sek, long bilder, double raster_us, double spi_us, double pixel) {
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "%s.tmp", stat_datei);
    FILE *f = fopen(tmp, "w");
    if (!f) return;
    fprintf(f, "bilder_s %.1f\nraster_us %.0f\nspi_us %.0f\npixel %.0f\n", bilder / sek,
            bilder ? raster_us / bilder : 0.0, bilder ? spi_us / bilder : 0.0, bilder ? pixel / bilder : 0.0);
    fclose(f);
    rename(tmp, stat_datei);
}

static void *augen_zeichnen(void *arg) {
    (void)arg;
    uint16_t *gen = calloc(320 * 240, sizeof(uint16_t));   // Bild des Generators (ohne Symbol)
    if (!gen) return NULL;
    uint16_t lut[256];
    uint8_t lut_rgb[3] = {0, 0, 0};
    int lut_ok = 0;
    rechteck_t alt = {0, 0, -1, -1};
    bild_t letztes;
    int hat_letztes = 0;
    double stat_t0 = jetzt_s(), raster_us = 0, spi_us = 0, pixel = 0;
    long bilder = 0;

    pthread_mutex_lock(&augen_lock);
    while (running) {
        double t = jetzt_s();
        if (neuer_ausdruck) {                // neuester Spieler gewinnt (wie tools/emotion.sh)
            neuer_ausdruck = 0;
            besitz_schreiben(getpid());
            generator_besitzt = 1;
            pthread_mutex_lock(&spi_lock);
            gen_voll = 1;
            pthread_mutex_unlock(&spi_lock);
        }
        if (generator_besitzt) {
            double alter;
            int pid = besitzer_lesen(&alter);
            if (pid != getpid()) {           // neuerer Besitzer: Ausdruck abbrechen
                leben_befehl(leben, "abbrechen", t);
                generator_besitzt = 0;
            }
        }
        bild_t b;
        int art = leben_bild(leben, t, &b);
        uint8_t rgb[3];
        leben_farbe(leben, t, rgb);
        int laeuft = leben_ausdruck_laeuft(leben, t);   // nach diesem Bild noch
        if (art == 0) {                      // nichts zu zeichnen: auf den nächsten Befehl warten
            if (generator_besitzt) besitz_abgeben();
            pthread_cond_wait(&augen_cond, &augen_lock);
            continue;
        }
        pthread_mutex_unlock(&augen_lock);

        // Vorrang: ohne eigenen Ausdruck zeichnet er nicht über fremde Bilder
        int zeichnen = 1;
        if (!generator_besitzt) {
            pthread_mutex_lock(&spi_lock);
            double roh = roh_t;
            pthread_mutex_unlock(&spi_lock);
            if (t - roh < ROH_PAUSE_S || fremder_besitzer()) zeichnen = 0;
        }
        if (zeichnen && !asleep) {
            if (!lut_ok || memcmp(rgb, lut_rgb, 3) != 0) {
                augen_lut(rgb[0], rgb[1], rgb[2], lut);
                memcpy(lut_rgb, rgb, 3);
                lut_ok = 1;
                hat_letztes = 0;
            }
            pthread_mutex_lock(&spi_lock);
            int voll = gen_voll;
            pthread_mutex_unlock(&spi_lock);
            if (voll || !hat_letztes || !bild_gleich(&b, &letztes)) {
                double t0 = jetzt_s();
                for (int y = alt.y0; y <= alt.y1; y++) memset(gen + y * 320 + alt.x0, 0, (size_t)(alt.x1 - alt.x0 + 1) * 2);
                rechteck_t r = augen_raster565(&b, lut, gen);
                rechteck_t f = r;
                if (alt.x0 <= alt.x1) {
                    if (alt.x0 < f.x0) f.x0 = alt.x0;
                    if (alt.y0 < f.y0) f.y0 = alt.y0;
                    if (alt.x1 > f.x1) f.x1 = alt.x1;
                    if (alt.y1 > f.y1) f.y1 = alt.y1;
                }
                alt = r;
                double t1 = jetzt_s();
                pthread_mutex_lock(&spi_lock);
                if (gen_voll) { f.x0 = 0; f.y0 = 0; f.x1 = 319; f.y1 = 239; }
                for (int y = f.y0; y <= f.y1; y++)
                    memcpy((uint16_t *)framebuffer + y * 320 + f.x0, gen + y * 320 + f.x0, (size_t)(f.x1 - f.x0 + 1) * 2);
                bild_da = 1;
                gen_voll = 0;
                anzeigen_fenster(f.x0, f.y0, f.x1, f.y1);
                pthread_mutex_unlock(&spi_lock);
                double t2 = jetzt_s();
                letztes = b;
                hat_letztes = 1;
                bilder++;
                raster_us += (t1 - t0) * 1e6;
                spi_us += (t2 - t1) * 1e6;
                pixel += (double)(r.x1 - r.x0 + 1) * (r.y1 - r.y0 + 1);
            }
        } else {
            hat_letztes = 0;                 // nach der Pause neu zeichnen
        }
        if (generator_besitzt && !laeuft) besitz_abgeben();   // letztes Bild des Ausdrucks steht
        if (t - stat_t0 >= 10.0) {
            stat_schreiben(t - stat_t0, bilder, raster_us, spi_us, pixel);
            stat_t0 = t;
            bilder = 0;
            raster_us = spi_us = pixel = 0;
        }

        // nächstes Bild: 25 fps, solange sich etwas bewegt, sonst 10 fps (Atmen); ein Befehl weckt sofort
        double schritt = art == 1 ? 0.04 : 0.1;
        double bis = t + schritt;
        struct timespec ts;
        ts.tv_sec = (time_t)bis;
        ts.tv_nsec = (long)((bis - (double)ts.tv_sec) * 1e9);
        pthread_mutex_lock(&augen_lock);
        if (!neuer_ausdruck) pthread_cond_timedwait(&augen_cond, &augen_lock, &ts);
    }
    pthread_mutex_unlock(&augen_lock);
    return NULL;
}

static void augen_starten(void) {
    struct stat sb;
    if (stat(augen_fifo, &sb) != 0 || !S_ISFIFO(sb.st_mode)) {
        unlink(augen_fifo);
        if (mkfifo(augen_fifo, 0660) != 0) {
            st7789_interface_debug_print("st7789: Augen-FIFO nicht angelegt\n");
            return;
        }
    }
    struct group *g = getgrnam("video");        // xgonav (aeinstein) und www-data sind in video
    if (g) { if (chown(augen_fifo, 0, g->gr_gid) != 0) { /* egal */ } }
    chmod(augen_fifo, 0660);
    // Namen der Ausdrücke für tools/emotion.sh (spielt nur diese über den Generator, alles andere als Bilder)
    char namen[64];
    snprintf(namen, sizeof(namen), "%s.namen", augen_fifo);
    FILE *f = fopen(namen, "w");
    if (f) {
        for (int i = 0; i < augen_ausdruck_n(); i++) fprintf(f, "%s\n", augen_ausdruck_name(i));
        fclose(f);
        chmod(namen, 0644);
    }
    leben = leben_neu((uint32_t)time(NULL));
    if (!leben) return;
    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&augen_cond, &ca);
    pthread_t lesen, zeichnen;
    pthread_create(&lesen, NULL, augen_lesen, NULL);
    pthread_create(&zeichnen, NULL, augen_zeichnen, NULL);
}

static void init_framebuffer(){
    struct stat sb;

    framebuffer = malloc(framesize);
    out = malloc(framesize);
    if(!framebuffer || !out) st7789_interface_debug_print("cant allocate framebuffer");
    icon_bauen();

    int i;
    for(i = 0; i < framesize; i++) {
        framebuffer[i] = 0;
    }

    // Wenn FIFO nicht existiert dann erstellen
    if (stat(fb_device, &sb) != 0) mkfifo(fb_device, 0777);

    st7789_basic_init();
    st7789_basic_display_on();

    pthread_t framereader;
    pthread_create (&framereader, NULL, read_framebuffer, NULL);
    pthread_t netz;
    pthread_create(&netz, NULL, netz_pruefen, NULL);
    augen_starten();
}


int main(void)
{
    daemon(0,0);
    uint8_t res;

    /* set the signal */
    signal(SIGINT, a_sig_handler);

    // SIGUSR1/2 vor dem Start des Lese-Threads blockieren (er erbt die Maske), dann hier annehmen
    sigset_t power;
    sigemptyset(&power);
    sigaddset(&power, SIGUSR1);
    sigaddset(&power, SIGUSR2);
    pthread_sigmask(SIG_BLOCK, &power, NULL);

    init_framebuffer();

    while(1) {
        int sig;
        if (sigwait(&power, &sig) != 0) {
            sleep(1);
            continue;
        }
        int want = (sig == SIGUSR1);
        pthread_mutex_lock(&spi_lock);
        if (want != asleep) {
            if (st7789_basic_sleep(want) == 0) {
                asleep = want;
                st7789_interface_debug_print(want ? "st7789: Ruhemodus, Anzeige aus\n" : "st7789: wach\n");
            } else {
                st7789_interface_debug_print("st7789: Schlafbefehl fehlgeschlagen\n");
            }
        }
        pthread_mutex_unlock(&spi_lock);
    }
}
