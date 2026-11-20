//
// Created by aeinstein on 23.01.2025.
//
#include "xgo-drv.h"
#include "xgo-gpio.h"

static void arm_motion_watchdog(void);

#include "xgo-proc.c"
#include "xgo-serial.c"

union B2I16 conv;

// Erste IPv4 der Schnittstelle XGO_IP_DEV, 0 wenn sie fehlt, nicht läuft oder keine Adresse hat.
static u32 pi_ip_read(void) {
	struct net_device *dev;
	struct in_device *in_dev;
	const struct in_ifaddr *ifa;
	u32 addr = 0;

	rcu_read_lock();
	dev = dev_get_by_name_rcu(&init_net, XGO_IP_DEV);
	if (dev && (dev->flags & IFF_UP)) {
		in_dev = __in_dev_get_rcu(dev);
		if (in_dev) {
			ifa = rcu_dereference(in_dev->ifa_list);
			if (ifa) addr = (__force u32)ifa->ifa_local;
		}
	}
	rcu_read_unlock();
	return addr;
}

// Aus loop(): Adresse bei Änderung und alle XGO_PI_IP_MS an den ESP32 (Register 0x18, 4 Byte a.b.c.d).
static void pi_ip_tick(void) {
	static u64 sent_ns;
	u64 now = ktime_get_ns();
	u32 addr = pi_ip_read();

	if (addr == XGO_PI_IP_SENT && sent_ns && now - sent_ns < (u64)XGO_PI_IP_MS * NSEC_PER_MSEC) return;
	write_serial_data(XGO_PI_IP, (char *)&addr, 4);
	XGO_PI_IP_SENT = addr;
	sent_ns = now;
}

static int loop(void *data) {
	unsigned int n = 0;
	pr_info("XGORider: loop start");

	while (!kthread_should_stop()) {
	    gpioCheck();
	    // Batterie ändert sich langsam: ~alle 2 s statt jede Runde
	    if (n++ % 10 == 0) batteryCheck();
        checkState();
        pi_ip_tick();

        // Kurs (0x68) nur ohne Strom abfragen: mit Strom setzt ihn stream_frame aus jedem Paket
        if(operational == 0x01 && !stream_active(ktime_get_ns())) readYaw();

		msleep(XGO_MS_SLEEP_ON_LOOP);
	}
	return 0;
}

/*
 * IMU-Thread: hält den Strom der Firmware am Leben (stream_tick). Solange Pakete kommen, fragt er nichts ab;
 * die Proben legt der Reader-Thread ab (stream_frame). Ohne Strom (imu_stream_hz 0, Firmware ohne Patch 0014,
 * ESP-Neustart) fragt er mit XGO_IMU_HZ das Register 0x0d ab: Roll/Nick/Gier und Rad-Ticks aus einem Snapshot,
 * Zeitstempel CLOCK_MONOTONIC (Ende der Antwort minus Übertragungsdauer).
 */
// Zeitpunkt, zu dem die Firmware den Wert erfasst hat: kurz bevor sie antwortet, also
// Ende der Antwort minus deren Übertragungsdauer (n Nutzbytes + 8 Rahmenbytes, 115200 Baud 8N1).
static u64 sample_time(u64 t_end, unsigned int n) {
	return t_end - div_u64((u64)(n + 8) * 10 * NSEC_PER_SEC, 115200);
}

static s32 s32le(const uint8_t *b) {
	return (s32)((u32)b[0] | ((u32)b[1] << 8) | ((u32)b[2] << 16) | ((u32)b[3] << 24));
}

// Modus 3 liefert die Räder gleich mit: hier abgelegt, imu_loop übernimmt sie ins wheel_log
static struct wheel_sample combo_wheels;
static bool combo_wheels_new;

static bool imu_read(struct imu_sample *s) {
	uint8_t b[24];
	u64 t;

	if (read_addr_sync(XGO_COMBO, 24, b, 20) < 24) return false;
	t = sample_time(ktime_get_ns(), 24);
	s->roll = f32le_to_milli(b);
	s->pitch = f32le_to_milli(b + 4);
	s->yaw = f32le_to_milli(b + 8);
	s->t_ns = t;
	combo_wheels.t_ns = t;
	combo_wheels.t11 = s32le(b + 12);
	combo_wheels.t21 = s32le(b + 16);
	combo_wheels.fw_ms = (u32)s32le(b + 20);
	combo_wheels_new = true;
	return true;
}

// Strom ein/aus: Rate jede Sekunde neu schreiben (Firmware schaltet nach 5 s ohne Byte vom Pi und nach
// ESP-Neustart ab), 0 einmal beim Ausschalten. true = Pakete kommen (stream_active), Abfrage nicht nötig.
static bool stream_tick(u64 now) {
	static u64 sent_ns;
	static unsigned int sent_hz;
	unsigned int shz = READ_ONCE(XGO_IMU_STREAM_HZ);
	uint8_t v;

	if (shz == 0) {
		if (sent_hz) {
			v = 0;
			write_serial_data(XGO_STREAM, (char *)&v, 1);
			sent_hz = 0;
		}
		return false;
	}
	if (shz != sent_hz || now - sent_ns >= NSEC_PER_SEC) {
		v = shz > 250 ? 250 : shz;
		write_serial_data(XGO_STREAM, (char *)&v, 1);
		sent_hz = shz;
		sent_ns = now;
	}
	return stream_active(now);
}

static int imu_loop(void *data) {
	struct imu_sample s;
	unsigned long flags;

	while (!kthread_should_stop()) {
		unsigned int hz = READ_ONCE(XGO_IMU_HZ);
		u64 start = ktime_get_ns(), period, used;

		if (stream_tick(start)) {
			msleep(20);                       // Strom läuft: Proben kommen über den Reader-Thread
			continue;
		}
		if (hz == 0 || !combo_supported) {
			msleep(200);
			continue;
		}
		period = div_u64(NSEC_PER_SEC, hz);

		if (imu_read(&s)) {
			spin_lock_irqsave(&imu_lock, flags);
			s.seq = ++imu_seq;
			imu_log[s.seq % IMU_LOG_LEN] = s;
			spin_unlock_irqrestore(&imu_lock, flags);
		}
		// Räder aus derselben Antwort
		if (combo_wheels_new) {
			combo_wheels_new = false;
			spin_lock_irqsave(&imu_lock, flags);
			combo_wheels.seq = ++wheel_seq;
			wheel_log[combo_wheels.seq % WHEEL_LOG_LEN] = combo_wheels;
			spin_unlock_irqrestore(&imu_lock, flags);
		}

		used = ktime_get_ns() - start;
		if (used < period) {
			u64 us = div_u64(period - used, 1000);
			usleep_range(us, us + 500);
		}
	}
	return 0;
}

// Kann die Firmware 0x0d (Winkel + Räder)? Geprüft wird gegen frisch gelesene Einzelregister, damit eine
// Firmware, die bei unbekanntem Register Pufferreste schickt, nicht falsch erkannt wird: Roll/Nick auf 2°
// gleich, Gier auf 10° (über ±180° hinweg; der Kurs darf sich zwischen den Abfragen etwas drehen).
static bool imu_singles(s32 *r, s32 *p, s32 *y) {
	uint8_t b[4];
	if (read_addr_sync(XGO_ROLL, 4, b, 50) < 4) return false;
	*r = f32le_to_milli(b);
	if (read_addr_sync(XGO_PITCH, 4, b, 50) < 4) return false;
	*p = f32le_to_milli(b);
	if (read_addr_sync(XGO_YAW, 4, b, 50) < 4) return false;
	*y = f32le_to_milli(b);
	return (*r | *p | *y) != 0;
}

static bool imu_same(const uint8_t *b, s32 r, s32 p, s32 y) {
	s32 dy = abs(f32le_to_milli(b + 8) - y) % 360000;
	if (dy > 180000) dy = 360000 - dy;
	return abs(f32le_to_milli(b) - r) < 2000 && abs(f32le_to_milli(b + 4) - p) < 2000 && dy < 10000;
}

static void imu_detect_combo(void) {
	uint8_t b[24];
	s32 r, p, y;

	combo_supported = imu_singles(&r, &p, &y) && read_addr_sync(XGO_COMBO, 24, b, 50) >= 24 && imu_same(b, r, p, y);
	if (combo_supported)
		pr_info("XGORider: IMU + Räder über 0x0d (24 Byte) abfragbar\n");
	else
		pr_warn("XGORider: Firmware ohne 0x0d (Patch 0001): IMU und Räder nur über den Strom (imu_stream_hz)\n");
}

// Wird bei jedem speed_x/speed_z-Write neu gestartet. Läuft er ab, ist der
// steuernde Prozess hängengeblieben/abgestürzt -> anhalten.
static void arm_motion_watchdog(void) {
	if (XGO_MOTION_TIMEOUT_MS)
		mod_delayed_work(system_wq, &motion_wd, msecs_to_jiffies(XGO_MOTION_TIMEOUT_MS));
}

static void motion_watchdog_fn(struct work_struct *work) {
	uint8_t neutral = 128;
	pr_warn("XGORider: motion watchdog fired (%u ms ohne Sollwert) -> stop\n", XGO_MOTION_TIMEOUT_MS);
	write_serial_data(XGO_VX, &neutral, 1);
	write_serial_data(XGO_VYAW, &neutral, 1);
}

static int16_t readYaw(void){
    uint8_t b[2];
    if(read_addr_sync(XGO_YAW_INT, 2, b, 50) >= 2) {
        conv.b[0] = b[1];
        conv.b[1] = b[0];
        current_yaw = conv.i;

        if(verbose & VERBOSE_MAIN) pr_info("XGORider: current yaw: %d", current_yaw);
        return current_yaw;
    }

    return 0;
}

static void batteryCheck(void) {
    uint8_t b;
    if(read_addr_sync(XGO_BATTERY, 1, &b, 50) >= 1) {
        battery = b;

        if(verbose & VERBOSE_MAIN) pr_info("XGORider: Battery: %d", battery);

        // vorher wurde shutdown_on_low_batt ignoriert
        if (XGO_SHUTDOWN_ON_LOW_BATT && battery < XGO_LOW_BATT) {
          	pr_warn("XGORider: Battery: %d, force shutdown", battery);
            orderly_poweroff(true);
        }
    }
}

static void checkState(void) {
    // Check state
    uint8_t st;
    if(read_addr_sync(XGO_STATE, 1, &st, 50) >= 1) {
        if(st != operational) {
            operational = st;
            pr_info("XGORider: State changed");

            switch(operational){
                case 0x00:
                    pr_warn("XGORider: fallen");
                    break;

                case 0x01:
                    pr_info("XGORider: balancing");
                    break;

                default:
                    pr_warn("XGORider: Zustand %d (weder gefallen noch balancierend)\n", operational);
                    break;
            }
        }
    }
}

static int __init imu_proc_init(void) {
  	pr_info("              ******       ******");
	pr_info("            **********   **********");
	pr_info("          ************* *************");
	pr_info("         ************      ************");
	pr_info("         ************ KERK ************");
	pr_info("         ************      ***********");
	pr_info("          ***************************");
	pr_info("            ***********************");
	pr_info("              *******************");
	pr_info("                ***************");
	pr_info("                  ***********");
	pr_info("                    *******");
	pr_info("                      ***");
	pr_info("                       *");
    pr_info("XGORider init");



    int ret = open_serial_port();
    if (ret) return ret;

    char fw[11] = {0};
    // Reader läuft bereits; synchroner Read statt msleep(200) + globalem rx_data
    if (read_addr_sync(XGO_FIRMWARE_VERSION, 10, (uint8_t *)fw, 500) < 10) {
        pr_err("XGORider: keine Antwort vom ESP32 (Firmware-Version)\n");
        ret = -EIO;
        goto err_serial;
    }
    pr_info("XGORider: firmware version: %.10s\n", fw);

    ret = initGPIO();
    if(ret) { ret = -EIO; goto err_serial; }

    imu_detect_combo();

    ret = createFilesystem();
    if(ret) goto err_serial;

    thread = kthread_run(loop, NULL, "xgo_main");
	if(IS_ERR(thread)) {
        pr_err("XGORider: Fehler beim Starten des Kernel-Threads\n");
        ret = PTR_ERR(thread);
        thread = NULL;
        goto err_fs;
    }

    imu_thread = kthread_run(imu_loop, NULL, "xgo_imu");
    if (IS_ERR(imu_thread)) {
        ret = PTR_ERR(imu_thread);
        imu_thread = NULL;
        goto err_main;
    }
    return 0;

err_main:
    kthread_stop(thread);
err_fs:
    destroyFilesystem();
err_serial:
    // Reader blockiert in kernel_read bis Bytes kommen oder VTIME (rc.local) abläuft
    stop_read_loop();
    filp_close(serial_file, NULL);
    return ret;
}

static void __exit imu_proc_exit(void) {
    cancel_delayed_work_sync(&motion_wd);
    // Reader zuerst stoppen, solange IMU/Main noch abfragen -> es kommen Bytes,
    // kernel_read kehrt zurück, der Thread sieht kthread_should_stop().
    stop_read_loop();

    if (imu_thread) kthread_stop(imu_thread);
  	if (thread) kthread_stop(thread);

    // Leser von /proc/XGORider/ble wecken, sonst wartet remove_proc_entry auf ihn
    WRITE_ONCE(ble_closing, true);
    wake_up_interruptible_all(&ble_wq);

    filp_close(serial_file, NULL);
    destroyFilesystem();
}

module_init(imu_proc_init);
module_exit(imu_proc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Balen");
MODULE_DESCRIPTION("XGORider IMU Procfs Kernel Module");
