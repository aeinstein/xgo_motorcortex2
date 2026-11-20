/*
 * xgo-ble: Gegenstelle des BLE-Tunnels am Pi (Firmware-Patch 0025, docs/ble.md).
 *
 * Die Firmware reicht die BLE-Characteristic "PI" über die UART durch, der Treiber legt sie als Datensätze in
 * /proc/XGORider/ble (Byte 0 = Kennung: 0 Daten, 1 BLE verbunden, 2 getrennt). Dieser Dienst liest dort
 * blockierend (schläft, solange niemand über BLE spricht), setzt die Daten zu Zeilen zusammen und antwortet:
 *
 *   Anfrage:  <id> wlan status|scan|rescan
 *             <id> wlan connect <WPA|NOPASS> <0|1> <ssid> [<passwort>]     ssid/passwort prozentkodiert
 *             <id> nav <GET|POST> <pfad?query>                             an xgonav, 127.0.0.1:8088
 *   Antwort:  <id> <code> <n>\n  + n Byte Rumpf
 *             wlan: code = Exit-Code von xgo-wlan, Rumpf = dessen Ausgabe
 *             nav:  code = HTTP-Status (599 = xgonav antwortet nicht), Rumpf = Antwort
 *             400 = Anfrage nicht verstanden, 403 = Pfad nicht erlaubt (/ws, /frame, /map.png)
 *
 * Läuft als root (deploy/xgo-ble.service): /proc/XGORider/ble ist 0600, xgo-wlan braucht root. Hängt weder am
 * Netz noch an xgonav, damit das WLAN auch ohne beides gesetzt werden kann.
 *
 *   xgo-ble [--dev PFAD] [--fd N] [--wlan PROGRAMM] [--port N]
 * --fd: schon offener Deskriptor statt --dev (Test: SOCK_SEQPACKET, ein Datensatz je read).
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define LINE_MAX_LEN 1024
#define BODY_MAX 16384           /* längere Antworten werden abgeschnitten (~3,5 s über BLE) */
#define CMD_TIMEOUT_MS 120000    /* xgo-wlan connect: nmcli wartet 45 s, dazu Chip-Reset nach dem Hotspot */
#define NAV_TIMEOUT_S 10

static const char *dev_path = "/proc/XGORider/ble";
static const char *wlan_prog = "/usr/local/sbin/xgo-wlan";
static int nav_port = 8088;

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static int write_all(int fd, const char *b, size_t n) {
    while (n) {
        ssize_t w = write(fd, b, n > 2048 ? 2048 : n);   /* der Treiber nimmt höchstens 4096 je Aufruf */
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        b += w;
        n -= (size_t)w;
    }
    return 0;
}

static void reply(int fd, const char *id, int code, const char *body, size_t n) {
    char head[64];
    int h = snprintf(head, sizeof head, "%s %d %zu\n", id, code, n);

    if (write_all(fd, head, (size_t)h) == 0 && n) write_all(fd, body, n);
}

static void reply_text(int fd, const char *id, int code, const char *text) {
    reply(fd, id, code, text, strlen(text));
}

/* Prozentkodierung auflösen. Rückgabe Länge, -1 bei Fehler oder Zeichen, die in einer Zeile für
 * xgo-wlan nichts verloren haben (Zeilenende, NUL): sonst ließe sich eine weitere Eingabezeile einschleusen. */
static int pct_decode(const char *in, char *out, size_t cap) {
    size_t n = 0;

    for (; *in; in++) {
        int c = (unsigned char)*in;

        if (c == '%') {
            char hex[3] = {in[1], in[1] ? in[2] : 0, 0};
            char *end;

            if (!hex[0] || !hex[1]) return -1;
            c = (int)strtol(hex, &end, 16);
            if (*end) return -1;
            in += 2;
        }
        if (c == 0 || c == '\n' || c == '\r') return -1;
        if (n + 1 >= cap) return -1;
        out[n++] = (char)c;
    }
    out[n] = 0;
    return (int)n;
}

/* Programm mit einem Argument starten, input auf stdin, Ausgabe (stdout + stderr) einsammeln.
 * Rückgabe Exit-Code, 255 bei Start-/Zeitfehler. */
static int run(const char *prog, const char *arg, const char *input, char *out, size_t cap, size_t *got) {
    int in_pipe[2], out_pipe[2], status = 0;
    long deadline = now_ms() + CMD_TIMEOUT_MS;
    pid_t pid;

    *got = 0;
    if (pipe(in_pipe)) return 255;
    if (pipe(out_pipe)) { close(in_pipe[0]); close(in_pipe[1]); return 255; }
    pid = fork();
    if (pid < 0) {
        close(in_pipe[0]); close(in_pipe[1]); close(out_pipe[0]); close(out_pipe[1]);
        return 255;
    }
    if (pid == 0) {
        dup2(in_pipe[0], 0); dup2(out_pipe[1], 1); dup2(out_pipe[1], 2);
        close(in_pipe[0]); close(in_pipe[1]); close(out_pipe[0]); close(out_pipe[1]);
        execl(prog, prog, arg, (char *)NULL);
        dprintf(1, "%s: %s\n", prog, strerror(errno));
        _exit(127);
    }
    close(in_pipe[0]); close(out_pipe[1]);
    if (input && *input) write_all(in_pipe[1], input, strlen(input));   /* wenige Zeilen, passt in den Pipe-Puffer */
    close(in_pipe[1]);
    for (;;) {
        struct pollfd p = {out_pipe[0], POLLIN, 0};
        long left = deadline - now_ms();
        char buf[512];
        ssize_t r;

        if (left <= 0) { kill(pid, SIGKILL); break; }
        if (poll(&p, 1, (int)left) <= 0) continue;
        r = read(out_pipe[0], buf, sizeof buf);
        if (r < 0 && errno == EINTR) continue;
        if (r <= 0) break;
        if (*got + (size_t)r > cap) r = (ssize_t)(cap - *got);     /* Rest verwerfen, weiterlesen bis zum Ende */
        memcpy(out + *got, buf, (size_t)r);
        *got += (size_t)r;
    }
    close(out_pipe[0]);
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    return WIFEXITED(status) ? WEXITSTATUS(status) : 255;
}

static void do_wlan(int fd, const char *id, char *rest, char *body) {
    char *sub = strtok(rest, " ");
    size_t n;
    int code;

    if (!sub) { reply_text(fd, id, 400, "wlan status|scan|rescan|connect\n"); return; }
    if (!strcmp(sub, "status") || !strcmp(sub, "scan") || !strcmp(sub, "rescan")) {
        code = run(wlan_prog, sub, NULL, body, BODY_MAX, &n);
        reply(fd, id, code, body, n);
        return;
    }
    if (!strcmp(sub, "connect")) {
        char *typ = strtok(NULL, " "), *hidden = strtok(NULL, " "), *ssid = strtok(NULL, " "), *pw = strtok(NULL, " ");
        char s[128], p[256], input[512];

        if (!typ || !hidden || !ssid || (strcmp(typ, "WPA") && strcmp(typ, "NOPASS")) ||
            (strcmp(hidden, "0") && strcmp(hidden, "1")) || pct_decode(ssid, s, sizeof s) < 1 ||
            pct_decode(pw ? pw : "", p, sizeof p) < 0) {
            reply_text(fd, id, 400, "wlan connect <WPA|NOPASS> <0|1> <ssid%> [<passwort%>]\n");
            return;
        }
        snprintf(input, sizeof input, "%s\n%s\n%s\n%s\n", s, p, typ, hidden);
        code = run(wlan_prog, "connect", input, body, BODY_MAX, &n);
        memset(p, 0, sizeof p);
        memset(input, 0, sizeof input);
        reply(fd, id, code, body, n);
        return;
    }
    reply_text(fd, id, 400, "wlan status|scan|rescan|connect\n");
}

static int prefix(const char *path, const char *pre) {
    size_t n = strlen(pre);

    return !strncmp(path, pre, n) && (path[n] == 0 || path[n] == '?' || path[n] == '/');
}

static void do_nav(int fd, const char *id, char *rest, char *body) {
    char *method = strtok(rest, " "), *path = strtok(NULL, " ");
    char req[LINE_MAX_LEN + 128];
    struct sockaddr_in a = {0};
    struct timeval tv = {NAV_TIMEOUT_S, 0};
    size_t n = 0;
    int s, code = 599, len;
    char *p;

    if (!method || !path || (strcmp(method, "GET") && strcmp(method, "POST")) || path[0] != '/' || strtok(NULL, " ")) {
        reply_text(fd, id, 400, "nav <GET|POST> </pfad?query>\n");
        return;
    }
    for (p = path; *p; p++) if ((unsigned char)*p < 0x21 || *p == 0x7f) { reply_text(fd, id, 400, "Pfad ungültig\n"); return; }
    if (prefix(path, "/ws") || prefix(path, "/frame") || prefix(path, "/map.png")) {
        reply_text(fd, id, 403, "nicht über BLE\n");
        return;
    }
    s = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)nav_port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (s < 0 || connect(s, (struct sockaddr *)&a, sizeof a)) {
        if (s >= 0) close(s);
        reply_text(fd, id, 599, "xgonav antwortet nicht\n");
        return;
    }
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    len = snprintf(req, sizeof req, "%s %s HTTP/1.0\r\nHost: 127.0.0.1\r\nContent-Length: 0\r\nConnection: close\r\n\r\n", method, path);
    if (write_all(s, req, (size_t)len) == 0) {
        static char raw[BODY_MAX + 2048];
        size_t got = 0;
        ssize_t r;

        while (got < sizeof raw - 1 && (r = read(s, raw + got, sizeof raw - 1 - got)) != 0) {
            if (r < 0) { if (errno == EINTR) continue; break; }
            got += (size_t)r;
        }
        raw[got] = 0;
        if (!strncmp(raw, "HTTP/", 5) && (p = strchr(raw, ' '))) {
            char *b = strstr(raw, "\r\n\r\n");

            code = atoi(p + 1);
            if (code < 100 || code > 599) code = 599;
            if (b) {
                b += 4;
                n = got - (size_t)(b - raw);
                if (n > BODY_MAX) n = BODY_MAX;
                memcpy(body, b, n);
            }
        }
    }
    close(s);
    if (code == 599 && n == 0) { reply_text(fd, id, 599, "xgonav antwortet nicht\n"); return; }
    reply(fd, id, code, body, n);
}

static void handle_line(int fd, char *line) {
    static char body[BODY_MAX];
    char *id = line, *cmd, *rest;
    size_t k;

    while (*id == ' ') id++;
    if (!*id) return;
    cmd = strchr(id, ' ');
    if (!cmd || cmd - id > 16) { reply_text(fd, "0", 400, "<id> wlan ...|nav ...\n"); return; }
    *cmd++ = 0;
    while (*cmd == ' ') cmd++;
    rest = strchr(cmd, ' ');
    if (rest) *rest++ = 0; else rest = cmd + strlen(cmd);
    k = strlen(rest);
    while (k && rest[k - 1] == ' ') rest[--k] = 0;
    if (!strcmp(cmd, "wlan")) do_wlan(fd, id, rest, body);
    else if (!strcmp(cmd, "nav")) do_nav(fd, id, rest, body);
    else reply_text(fd, id, 400, "<id> wlan ...|nav ...\n");
}

/* Liest, bis der Deskriptor endet (Treiber entladen) oder ein Fehler kommt. */
static void serve(int fd) {
    static char line[LINE_MAX_LEN];
    size_t used = 0;
    int overflow = 0;

    for (;;) {
        unsigned char rec[64];
        ssize_t n = read(fd, rec, sizeof rec), i;

        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return;
        if (rec[0] != 0) {           /* BLE verbunden oder getrennt: Angefangenes verwerfen */
            used = 0;
            overflow = 0;
            continue;
        }
        for (i = 1; i < n; i++) {
            char c = (char)rec[i];

            if (c == '\n' || c == '\r') {
                if (overflow) reply_text(fd, "0", 400, "Zeile zu lang\n");
                else if (used) { line[used] = 0; handle_line(fd, line); }
                memset(line, 0, used);       /* kann ein Passwort enthalten haben */
                used = 0;
                overflow = 0;
            } else if (used < sizeof line - 1) {
                line[used++] = c;
            } else {
                overflow = 1;
            }
        }
    }
}

int main(int argc, char **argv) {
    int fixed_fd = -1, i;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i + 1 < argc) dev_path = argv[++i];
        else if (!strcmp(argv[i], "--fd") && i + 1 < argc) fixed_fd = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--wlan") && i + 1 < argc) wlan_prog = argv[++i];
        else if (!strcmp(argv[i], "--port") && i + 1 < argc) nav_port = atoi(argv[++i]);
        else {
            fprintf(stderr, "Aufruf: xgo-ble [--dev PFAD] [--fd N] [--wlan PROGRAMM] [--port N]\n");
            return 2;
        }
    }
    signal(SIGPIPE, SIG_IGN);
    if (fixed_fd >= 0) {
        serve(fixed_fd);
        return 0;
    }
    for (;;) {
        int fd = open(dev_path, O_RDWR | O_CLOEXEC);

        if (fd >= 0) {
            serve(fd);
            close(fd);
        }
        sleep(2);    /* Treiber (noch) nicht geladen oder ohne Tunnel: später wieder versuchen */
    }
}
