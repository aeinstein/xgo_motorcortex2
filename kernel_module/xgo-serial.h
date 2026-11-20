//
// Created by aeinstein on 24.01.2025.
//

#ifndef XGO_SERIAL_H
#define XGO_SERIAL_H

#include <linux/serial.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/atomic.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/wait.h>

#include "../constants.h"

#include "xgo-drv.h"

#define SERIAL_PORT "/dev/ttyAMA0" // Anpassen je nach Hardware
#define MAX_DATA_LEN 32

int16_t current_yaw = 0;    // ganze Grad wie Register 0x68: aus dem Strom (stream_frame), ohne Strom aus readYaw()

uint8_t battery = 100;
uint8_t operational = 0x01;

// ---- Parser (nur Reader-Thread) -------------------------------------------
static uint8_t rx_FLAG = 0;
static uint8_t rx_LEN = 0;
static uint8_t rx_TYPE = 0;
static uint8_t rx_ADDR = 0;
static uint8_t rx_COUNT = 0;
static uint8_t rx_buf[MAX_DATA_LEN];

// ---- Request/Response -------------------------------------------------------
// Es ist immer höchstens ein Lesezugriff offen (read_lock). Der Reader-Thread
// erkennt die Antwort an der Adresse und weckt den Wartenden (resp_done).
static DEFINE_MUTEX(read_lock);
static DEFINE_SPINLOCK(resp_lock);
static DECLARE_COMPLETION(resp_done);
static int pending_addr = -1;
static uint8_t resp_data[MAX_DATA_LEN];
static uint8_t resp_len = 0;

static atomic_t stat_frames = ATOMIC_INIT(0);
static atomic_t stat_bad = ATOMIC_INIT(0);
static atomic_t stat_timeouts = ATOMIC_INIT(0);
static atomic_t stat_unsolicited = ATOMIC_INIT(0);

// ---- IMU --------------------------------------------------------------------
// Winkel in Milligrad (Kernel ohne FPU -> Floats per Integer-Arithmetik dekodiert),
// Zeitstempel CLOCK_MONOTONIC in ns (= time.monotonic_ns() / V4L2-Puffer-Zeit).
struct imu_sample {
	u32 seq;
	u64 t_ns;
	s32 roll;
	s32 pitch;
	s32 yaw;
};

#define IMU_LOG_LEN 128
static struct imu_sample imu_log[IMU_LOG_LEN];
static u32 imu_seq = 0;
static DEFINE_SPINLOCK(imu_lock);
static unsigned int XGO_IMU_HZ = 50;   // 0 = aus
// Abfrage ohne Strom nur über 0x0d (R-1.1.6 + Firmware-Patch 0001: Winkel + Rad-Ticks + Firmware-Zeit aus einem
// Snapshot). Die früheren Wege für Firmware ohne 0x0d (0x65, 12-Byte-Block, Einzelregister, Räder 0x0c) sind seit
// 2026-11-17 entfernt: ohne 0x0d und ohne Strom gibt es keine IMU- und Radproben.
static bool combo_supported = false;

// Rad-Encoder (aus dem Strom, sonst aus 0x0d): fortlaufende Ticks, 1024 je Umdrehung,
// beide Räder vorwärts positiv (11 = links, 21 = rechts). Zeitstempel CLOCK_MONOTONIC (CM4).
struct wheel_sample {
	u32 seq;
	u64 t_ns;
	s32 t11;
	s32 t21;
	u32 fw_ms;      // Firmware-Zeit der Probe in ms (Strom: t_us / 1000, Abfrage: millis() aus 0x0d)
};
#define WHEEL_LOG_LEN 64
static struct wheel_sample wheel_log[WHEEL_LOG_LEN];
static u32 wheel_seq = 0;
static struct task_struct *imu_thread;

// IMU/Rad-Strom (Firmware-Patch 0014): die Firmware sendet ohne Anfrage aus ihrer Regelschleife, mit eigener
// Messzeit t_us. settings/imu_stream_hz > 0 schaltet ihn ein (imu_loop schreibt die Rate jede Sekunde neu, die
// Firmware schaltet nach 5 s ohne Byte vom Pi und nach Neustart ab); solange Pakete kommen, wird nicht abgefragt.
static int XGO_LED_BRIGHTNESS_SET = -1;         // zuletzt geschriebene LED-Helligkeit (n/256), -1 = nie: Firmware-Default 32
static unsigned int XGO_IMU_STREAM_HZ = 0;      // 0 = kein Strom, Abfrage über 0x0d
static atomic_t stat_stream = ATOMIC_INIT(0);
static atomic_t stat_stream_gaps = ATOMIC_INIT(0);
static u64 stream_last_rx_ns;                   // Empfang des letzten Pakets (CLOCK_MONOTONIC), 0 = nie

// Strom läuft: letztes Paket jünger als 2,5 Sendeabstände, mindestens 200 ms. Mit festen 200 ms galt er bei
// 5 Hz (liegend, Pakete alle ~200 ms) zwischen den Paketen als aus, und Abfrage-Proben mit CM4-Zeit rutschten
// zwischen die Strom-Proben (Test 2026-11-17: 20 Pakete, 31 Proben in 4 s).
static inline bool stream_active(u64 now) {
	unsigned int hz = READ_ONCE(XGO_IMU_STREAM_HZ);
	u64 last = READ_ONCE(stream_last_rx_ns), lim = 200 * NSEC_PER_MSEC;

	if (!hz || !last) return false;
	if (hz < 12) lim = div_u64(5 * (u64)NSEC_PER_SEC, 2 * hz);
	return now - last < lim;
}
static s64 stream_off_ns;                       // CLOCK_MONOTONIC - ESP-Zeit (ns), Minimum über ~2-s-Fenster
static bool stream_off_ok;

// ---- BLE-Tunnel (Firmware-Patch 0025) ----------------------------------------
// Die Firmware reicht die BLE-Characteristic "PI" durch: Rahmen Typ 0x15, Register 0x17, Nutzlast = Kennung
// (0 Daten, 1 BLE verbunden, 2 getrennt) + höchstens 23 Byte. Der Reader-Thread legt jede Nutzlast als
// Datensatz (Längenbyte + Nutzlast) in den Ring und weckt den Leser von /proc/XGORider/ble. Gegenrichtung:
// ble_tx() stückelt in Schreibbefehle auf 0x17 (Kennung 0) mit Abstand, damit der IMU-Strom Platz behält
// und der ESP32 mit dem Weitergeben über BLE nachkommt.
#define BLE_RX_SIZE 4096
#define BLE_CHUNK (XGO_TUNNEL_MAX - 1)
#define BLE_TX_GAP_US 4000
static uint8_t ble_rx[BLE_RX_SIZE];
static unsigned int ble_rx_head, ble_rx_used;
static bool ble_closing;                        // Modul wird entladen: wartende Leser kehren zurück
static DEFINE_SPINLOCK(ble_lock);
static DEFINE_MUTEX(ble_tx_lock);
static DECLARE_WAIT_QUEUE_HEAD(ble_wq);
static atomic_t stat_tunnel_rx = ATOMIC_INIT(0);
static atomic_t stat_tunnel_drop = ATOMIC_INIT(0);
static atomic_t stat_tunnel_tx = ATOMIC_INIT(0);

static struct task_struct *reader_thread;
static struct file *serial_file;

static int ble_rx_pop(uint8_t *out);
static int ble_tx(const uint8_t *data, size_t len);

static int write_serial_data(size_t addr, char *buffer, size_t len);
static int read_addr_sync(uint8_t addr, uint8_t len, uint8_t *out, unsigned int timeout_ms);
static bool addToSendQueue(const char *cmd, size_t len);
static void doQueue(void);
static int read_loop(void *data);
static void stop_read_loop(void);
static s32 f32le_to_milli(const uint8_t *b);

#endif //XGO_SERIAL_H
