#include "driver_st7789_basic.h"
//#include "shell.h"
#include <getopt.h>
#include <sys/types.h>
#include <sys/socket.h>
//#include <netinet/in.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <ifaddrs.h>
#include <netinet/in.h>

static int framesize = 320 * 240 * 2;
static char *framebuffer;
static char *fb_device = "/tmp/lcd0";
static int running = 1;
// SIGUSR1 = Anzeige aus + Controller schlafen, SIGUSR2 = wach. Der Ruhemodus nutzt das nicht mehr (das
// Display wurde davon nicht dunkel, dort stehen jetzt die Schlafaugen); deploy/xgo-ruhe schickt nur noch SIGUSR2.
// Beide Signale sind in allen Threads blockiert und werden im Hauptthread per sigwait angenommen;
// der Mutex hält SPI-Zugriffe von Lese-Thread (Bild) und Hauptthread (Schlafbefehl) auseinander.
static pthread_mutex_t spi_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int asleep = 0;

// Kein Netz: wlan0 ohne IPv4 (alle NETZ_PRUEF_S geprüft) -> oben rechts ein WLAN-Symbol mit rotem Strich über jedem
// Bild; framebuffer behält das Bild ohne Symbol, beim Wechsel wird es neu gezeichnet (auch bei stehendem Bild).
#define NETZ_PRUEF_S 2
#define ICON 24
#define ICON_X (320 - ICON - 4)
#define ICON_Y 4
static uint16_t icon[ICON * ICON];
static char *out;                          // angezeigtes Bild (framebuffer + Symbol)
static volatile int kein_netz = 0;
static int bild_da = 0;

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

// unter spi_lock
static void anzeigen(void) {
    if (asleep || !bild_da) return;
    memcpy(out, framebuffer, framesize);
    if (kein_netz) {
        uint16_t *o = (uint16_t *)out;
        for (int y = 0; y < ICON; y++) memcpy(o + (ICON_Y + y) * 320 + ICON_X, icon + y * ICON, ICON * 2);
    }
    st7789_basic_draw_picture_16bits(0, 0, 319, 239, (uint16_t *)out);
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

static void *read_framebuffer(){
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

        // buffer full write to framebuffer
        memcpy(framebuffer, rx, framesize);

        //st7789_interface_debug_print("picture complete\n");
        pthread_mutex_lock(&spi_lock);
        if (!asleep) {     // im Ruhemodus Bilder verwerfen
            st7789_basic_draw_picture_16bits(0, 0, 319, 239, (uint16_t *)framebuffer);
        }
        pthread_mutex_unlock(&spi_lock);

        //  draw battery level
        //st7789_basic_string(250, 220, "Batt: 10%", 9, 0xFFFF, ST7789_FONT_16);
    }

    return NULL;
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
    pthread_create (&framereader, NULL, read_framebuffer, "");
    pthread_t netz;
    pthread_create(&netz, NULL, netz_pruefen, NULL);
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
