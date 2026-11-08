//
// Created by aeinstein on 24.01.2025.
//
// Lesepfad (2026-11): synchrone Requests mit completion statt msleep(200),
// Reader liest blockweise und läuft ohne Pausen durch. Damit sind Lesezugriffe
// in wenigen ms fertig und die IMU lässt sich mit 50-100 Hz abfragen.
//

#include "xgo-serial.h"


struct message {
    char *data;
    size_t len;
    struct list_head list;
};

static LIST_HEAD(send_queue);     // Nachrichtenschlange
static DEFINE_MUTEX(queue_lock);  // Sperre zur Synchronisation


// Funktion zum Öffnen der seriellen Schnittstelle
static int open_serial_port(void) {
    pr_info("XGORider: open %s\n", SERIAL_PORT);
    serial_file = filp_open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC, 0);

    if (IS_ERR(serial_file)) {
        pr_err("XGORider: Failed to open serial device: %ld\n", PTR_ERR(serial_file));
        return PTR_ERR(serial_file);
    }

    pr_info("Serial device opened successfully\n");

    reader_thread = kthread_run(read_loop, NULL, "xgo_serial_rx");
    if(IS_ERR(reader_thread)) {
        pr_err("XGORider: Fehler beim Starten des Kernel-Threads\n");
        return PTR_ERR(reader_thread);
    }

    return 0;
}

static void send_read_request(uint8_t addr, uint8_t len) {
    const uint8_t mode = 0x02;
    uint8_t sum_data = 255 - ((0x09 + mode + addr + len) % 256);
    unsigned char cmd[] = {0x55, 0x00, 0x09, mode, addr, len, sum_data, 0x00, 0xAA};

    if(verbose & VERBOSE_SERIAL) {
        pr_info("XGORider: read addr 0x%02X len %u\n", addr, len);
    }
    addToSendQueue(cmd, sizeof(cmd));
}

/*
 * Liest len Bytes ab addr. Blockiert bis die Antwort da ist oder timeout_ms.
 * Rückgabe: Anzahl Bytes (>= len) oder -ETIMEDOUT/-EIO/-EINTR.
 */
static int read_addr_sync(uint8_t addr, uint8_t len, uint8_t *out, unsigned int timeout_ms) {
    int ret;
    long left;

    if (len == 0 || len > MAX_DATA_LEN) return -EINVAL;
    if (IS_ERR_OR_NULL(serial_file)) return -ENODEV;

    if (mutex_lock_interruptible(&read_lock)) return -EINTR;

    spin_lock(&resp_lock);
    pending_addr = addr;
    resp_len = 0;
    reinit_completion(&resp_done);
    spin_unlock(&resp_lock);

    send_read_request(addr, len);

    left = wait_for_completion_timeout(&resp_done, msecs_to_jiffies(timeout_ms));

    spin_lock(&resp_lock);
    if (!left) {
        ret = -ETIMEDOUT;
    } else if (resp_len < len) {
        ret = -EIO;
    } else {
        memcpy(out, resp_data, len);
        ret = resp_len;
    }
    pending_addr = -1;
    spin_unlock(&resp_lock);

    if (ret == -ETIMEDOUT) {
        atomic_inc(&stat_timeouts);
        if (verbose & VERBOSE_SERIAL) pr_warn("XGORider: timeout addr 0x%02X\n", addr);
    }

    mutex_unlock(&read_lock);
    return ret;
}

static int write_serial_data(const size_t addr, char * buffer, const size_t len){
    int value_sum = 0;
    unsigned char cmd[MAX_DATA_LEN + 8];

    if (len > MAX_DATA_LEN) return -EINVAL;

    for (uint8_t i = 0; i < len; i++) {
        value_sum += (uint8_t)buffer[i];
    }

    const int mode = 0x01;
    int sum_data = ((len + 0x08) + mode + addr + value_sum) % 256;
    sum_data = 255 - sum_data;

    cmd[0] = 0x55;
    cmd[1] = 0x00;
    cmd[2] = len + 0x08;
    cmd[3] = mode;
    cmd[4] = addr;

    for (uint8_t i = 0; i < len; i++) {
        cmd[i + 0x05] = buffer[i];
    }

    cmd[len + 0x05] = sum_data;
    cmd[len + 0x06] = 0x00;
    cmd[len + 0x07] = 0xAA;

    if(verbose & VERBOSE_SERIAL) {
        pr_info("XGORider: write addr 0x%02zX len %zu\n", addr, len);
    }

    addToSendQueue(cmd, len + 8);
    return 0;
}

// ---- IMU/Rad-Strom (nur Reader-Thread) -------------------------------------
#define STREAM_TX_NS ((u64)(XGO_STREAM_LEN + 8) * 10 * NSEC_PER_SEC / 115200)   // 34 Byte bei 115200 8N1
#define STREAM_WIN_NS (2ULL * NSEC_PER_SEC)

static u16 stream_seq_last;
static bool stream_seq_ok;
static u32 stream_tus_last;
static u64 stream_tus_hi;
static s64 stream_win_min;
static u64 stream_win_start;

static u32 le32(const uint8_t *b) {
    return (u32)b[0] | ((u32)b[1] << 8) | ((u32)b[2] << 16) | ((u32)b[3] << 24);
}

/*
 * Ein Strom-Paket: Messzeit der Firmware (t_us) auf CLOCK_MONOTONIC abbilden und als IMU- und Radprobe ablegen.
 * Versatz = Empfang - Übertragungsdauer - t_us; Wartezeiten (Firmware-loop, UART, Reader) machen ihn nur größer,
 * also gilt das Minimum: sofort, wenn ein kleinerer kommt, und je ~2 s das Minimum des letzten Fensters
 * (führt die Drift der ESP-Uhr nach).
 */
static void stream_frame(const uint8_t *b, u64 t_rx) {
    u16 seq = (u16)(b[0] | (b[1] << 8));
    u32 tus = le32(b + 2);
    u64 tus64;
    s64 cand;
    struct imu_sample s;
    struct wheel_sample w;
    unsigned long flags;

    atomic_inc(&stat_stream);
    if (stream_seq_ok && (u16)(seq - stream_seq_last) != 1) atomic_inc(&stat_stream_gaps);
    stream_seq_last = seq;
    stream_seq_ok = true;

    if (stream_off_ok && tus < stream_tus_last) {
        if (stream_tus_last - tus > 0x80000000u) {
            stream_tus_hi += 1ULL << 32;              // micros() lief über (~71 min)
        } else {
            stream_tus_hi = 0;                        // ESP neu gestartet: Zeitbezug neu
            stream_off_ok = false;
        }
    }
    stream_tus_last = tus;
    tus64 = stream_tus_hi + tus;
    cand = (s64)(t_rx - STREAM_TX_NS) - (s64)(tus64 * 1000);
    if (!stream_off_ok) {
        stream_off_ns = stream_win_min = cand;
        stream_win_start = t_rx;
        stream_off_ok = true;
    } else {
        if (cand < stream_win_min) stream_win_min = cand;
        if (cand < stream_off_ns) stream_off_ns = cand;
        if (t_rx - stream_win_start >= STREAM_WIN_NS) {
            stream_off_ns = stream_win_min;
            stream_win_min = cand;
            stream_win_start = t_rx;
        }
    }

    s.t_ns = (u64)((s64)(tus64 * 1000) + stream_off_ns);
    s.roll = f32le_to_milli(b + 6);
    s.pitch = f32le_to_milli(b + 10);
    s.yaw = f32le_to_milli(b + 14);
    w.t_ns = s.t_ns;
    w.t11 = (s32)le32(b + 18);
    w.t21 = (s32)le32(b + 22);
    w.fw_ms = (u32)div_u64(tus64, 1000);

    spin_lock_irqsave(&imu_lock, flags);
    s.seq = ++imu_seq;
    imu_log[s.seq % IMU_LOG_LEN] = s;
    w.seq = ++wheel_seq;
    wheel_log[w.seq % WHEEL_LOG_LEN] = w;
    WRITE_ONCE(stream_last_rx_ns, t_rx);
    spin_unlock_irqrestore(&imu_lock, flags);
}

// Vollständiger, prüfsummenkorrekter Frame im rx_buf.
static void handle_frame(void) {
    uint8_t dlen = rx_LEN - 8;

    atomic_inc(&stat_frames);

    if (rx_TYPE == XGO_STREAM_TYPE && rx_ADDR == XGO_STREAM && dlen == XGO_STREAM_LEN) {
        stream_frame(rx_buf, ktime_get_ns());
        return;
    }

    spin_lock(&resp_lock);
    if ((int)rx_ADDR == pending_addr) {
        memcpy(resp_data, rx_buf, dlen);
        resp_len = dlen;
        pending_addr = -1;
        complete(&resp_done);
    } else {
        // z. B. verspätete Antwort nach Timeout
        atomic_inc(&stat_unsolicited);
    }
    spin_unlock(&resp_lock);

    if (verbose & VERBOSE_SERIAL) {
        pr_info("XGORider: rx addr 0x%02X type 0x%02X len %u\n", rx_ADDR, rx_TYPE, dlen);
    }
}

static void parser_reset(bool bad) {
    if (bad) atomic_inc(&stat_bad);
    rx_FLAG = 0;
    rx_COUNT = 0;
    rx_ADDR = 0;
    rx_LEN = 0;
}

// Frame: 55 00 LEN TYPE ADDR DATA[LEN-8] CHECK 00 AA
static void parse_byte(uint8_t num) {
    switch (rx_FLAG) {
    case 0:
        if (num == 0x55) rx_FLAG = 1;
        break;

    case 1:
        if (num == 0x00) rx_FLAG = 2;
        else if (num != 0x55) rx_FLAG = 0;
        break;

    case 2:
        // vorher ungeprüft: LEN > 40 schrieb über rx_data[32] hinaus
        if (num < 9 || num > MAX_DATA_LEN + 8) {
            parser_reset(true);
            break;
        }
        rx_LEN = num;
        rx_FLAG = 3;
        break;

    case 3:
        rx_TYPE = num;
        rx_FLAG = 4;
        break;

    case 4:
        rx_ADDR = num;
        rx_COUNT = 0;
        rx_FLAG = 5;
        break;

    case 5:
        rx_buf[rx_COUNT++] = num;
        if (rx_COUNT >= rx_LEN - 8) rx_FLAG = 6;
        break;

    case 6: {
        unsigned int sum = rx_LEN + rx_TYPE + rx_ADDR;
        for (uint8_t j = 0; j < rx_LEN - 8; j++) sum += rx_buf[j];
        if (num == (uint8_t)(255 - (sum % 256))) {
            rx_FLAG = 7;
        } else {
            if (verbose & VERBOSE_SERIAL) pr_warn("XGORider: wrong checksum\n");
            parser_reset(true);
        }
        break;
    }

    case 7:
        if (num == 0x00) rx_FLAG = 8;
        else parser_reset(true);
        break;

    case 8:
        if (num == 0xAA) {
            handle_frame();
            parser_reset(false);
        } else {
            parser_reset(true);
        }
        break;

    default:
        parser_reset(true);
    }
}

static int read_loop(void *data) {
    uint8_t buf[64];

    while (!kthread_should_stop()) {
        loff_t pos = 0;
        ssize_t n = kernel_read(serial_file, buf, sizeof(buf), &pos);

        if (n <= 0) {
            // mit "stty min 0 time 1" (rc.local) kehrt read nach 100 ms leer zurück
            if (n < 0 && n != -EAGAIN && n != -EINTR) msleep(10);
            continue;
        }
        for (ssize_t i = 0; i < n; i++) parse_byte(buf[i]);
    }

    return 0;
}

static void stop_read_loop(void) {
    if(!IS_ERR_OR_NULL(reader_thread)) {
        kthread_stop(reader_thread);
        reader_thread = NULL;
    }
}

// IEEE754 float (little endian, wie vom ESP32 gesendet) -> Tausendstel, ohne FPU.
static s32 f32le_to_milli(const uint8_t *b) {
    u32 bits = (u32)b[0] | ((u32)b[1] << 8) | ((u32)b[2] << 16) | ((u32)b[3] << 24);
    int sign = bits >> 31;
    int exp = (bits >> 23) & 0xff;
    u64 man = bits & 0x7fffff;
    s64 v;
    int sh;

    if (exp == 0) return 0;                 // 0 / denormal (praktisch 0)
    if (exp == 0xff) return sign ? S32_MIN : S32_MAX;   // inf/nan

    man |= 0x800000;                        // implizite 1
    man *= 1000;                            // < 2^35
    sh = exp - 150;                         // Wert = man * 2^sh
    if (sh >= 0) {
        if (sh > 27) return sign ? S32_MIN : S32_MAX;
        v = (s64)(man << sh);
    } else if (sh < -62) {
        v = 0;
    } else {
        v = (s64)((man + (1ULL << (-sh - 1))) >> -sh);   // gerundet
    }
    if (v > S32_MAX) v = S32_MAX;
    return sign ? (s32)-v : (s32)v;
}

// Funktion: Nachricht zur Sende-Warteschlange hinzufügen
static bool addToSendQueue(const char *cmd, size_t len) {
    struct message *new_msg;

    // Speicher für neue Nachricht reservieren
    new_msg = kmalloc(sizeof(*new_msg), GFP_KERNEL);
    if (!new_msg) {
        pr_err("XGORider: Speicherzuweisung für Nachricht fehlgeschlagen\n");
        return false;
    }

    new_msg->data = kmalloc(len, GFP_KERNEL);
    if (!new_msg->data) {
        pr_err("XGORider: Speicherzuweisung für Nachrichtendaten fehlgeschlagen\n");
        kfree(new_msg);
        return false;
    }

    memcpy(new_msg->data, cmd, len);
    new_msg->len = len;

    // Nachricht zur Warteschlange hinzufügen
    mutex_lock(&queue_lock);
    list_add_tail(&new_msg->list, &send_queue);
    mutex_unlock(&queue_lock);

    doQueue();
    return true;
}

// Funktion: Nachrichten aus der Sende-Warteschlange verarbeiten
static void doQueue(void) {
    struct message *msg, *tmp;
    loff_t pos = 0;

    mutex_lock(&queue_lock);

    // Alle Nachrichten in der Warteschlange durchlaufen
    list_for_each_entry_safe(msg, tmp, &send_queue, list) {
        // Nachricht senden
        kernel_write(serial_file, msg->data, msg->len, &pos);

        // Nachricht aus der Warteschlange entfernen und Speicher freigeben
        list_del(&msg->list);
        kfree(msg->data);
        kfree(msg);
    }

    mutex_unlock(&queue_lock);
}
