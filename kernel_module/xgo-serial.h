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

#include "../constants.h"

#include "xgo-drv.h"

#define SERIAL_PORT "/dev/ttyAMA0" // Anpassen je nach Hardware
#define BAUD_RATE B115200
#define MAX_DATA_LEN 32

int16_t initial_yaw = 0;
int16_t wanted_yaw = 0;
int16_t current_yaw = 0;

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
// IMU-Lesemodus: 0 = drei Einzelregister, 1 = roll/pitch/yaw als 12-Byte-Block ab 0x62,
// 2 = 0x65 (R-1.1.6: accel + Winkel in einer 24-Byte-Antwort),
// 3 = 0x0d (R-1.1.6+Patch: Winkel + Rad-Ticks + Firmware-Zeit aus einem Snapshot)
static int imu_block_mode = 0;

// Rad-Encoder (Register 0x0c, R-1.1.6+Patch): fortlaufende Ticks, 1024 je Umdrehung,
// beide Räder vorwärts positiv (11 = links, 21 = rechts). Zeitstempel CLOCK_MONOTONIC (CM4).
struct wheel_sample {
	u32 seq;
	u64 t_ns;
	s32 t11;
	s32 t21;
	u32 fw_ms;      // Firmware-millis() des Snapshots (nur Modus 3, sonst 0)
};
#define WHEEL_LOG_LEN 64
static struct wheel_sample wheel_log[WHEEL_LOG_LEN];
static u32 wheel_seq = 0;
static bool wheels_supported = false;
static unsigned int XGO_WHEEL_HZ = 25;  // 0 = aus; tatsächlich höchstens imu_hz, als Teiler davon
static struct task_struct *imu_thread;

// IMU/Rad-Strom (Firmware-Patch 0014): die Firmware sendet ohne Anfrage aus ihrer Regelschleife, mit eigener
// Messzeit t_us. settings/imu_stream_hz > 0 schaltet ihn ein (imu_loop schreibt die Rate jede Sekunde neu, die
// Firmware schaltet nach 5 s ohne Byte vom Pi und nach Neustart ab); solange Pakete kommen, wird nicht abgefragt.
static unsigned int XGO_IMU_STREAM_HZ = 0;      // 0 = Abfrage wie bisher
static atomic_t stat_stream = ATOMIC_INIT(0);
static atomic_t stat_stream_gaps = ATOMIC_INIT(0);
static u64 stream_last_rx_ns;                   // Empfang des letzten Pakets (CLOCK_MONOTONIC), 0 = nie
static s64 stream_off_ns;                       // CLOCK_MONOTONIC - ESP-Zeit (ns), Minimum über ~2-s-Fenster
static bool stream_off_ok;

static struct task_struct *reader_thread;
static struct file *serial_file;

static int write_serial_data(size_t addr, char *buffer, size_t len);
static int read_addr_sync(uint8_t addr, uint8_t len, uint8_t *out, unsigned int timeout_ms);
static bool addToSendQueue(const char *cmd, size_t len);
static void doQueue(void);
static int read_loop(void *data);
static void stop_read_loop(void);
static s32 f32le_to_milli(const uint8_t *b);

#endif //XGO_SERIAL_H
