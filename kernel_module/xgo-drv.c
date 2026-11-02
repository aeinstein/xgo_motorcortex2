//
// Created by aeinstein on 23.01.2025.
//
#include "xgo-drv.h"
#include "xgo-gpio.h"

static void arm_motion_watchdog(void);

#include "xgo-proc.c"
#include "xgo-serial.c"

union B2I16 conv;

static int loop(void *data) {
	unsigned int n = 0;
	pr_info("XGORider: loop start");

	while (!kthread_should_stop()) {
	    gpioCheck();
	    // Batterie ändert sich langsam: ~alle 2 s statt jede Runde
	    if (n++ % 10 == 0) batteryCheck();
        checkState();

        if(operational == 0x01) {
            readYaw();

            if(XGO_HOLD_YAW) forceyaw();

        }

		msleep(XGO_MS_SLEEP_ON_LOOP);
	}
	return 0;
}

/*
 * IMU-Thread: Roll/Pitch/Yaw (Float-Register 0x62..0x64) mit XGO_IMU_HZ.
 * Jede Probe bekommt einen CLOCK_MONOTONIC-Zeitstempel (Mitte der Pitch-Abfrage),
 * damit Userspace sie den Kamerabildern zuordnen kann.
 */
static bool imu_read(struct imu_sample *s) {
	uint8_t b[12];
	u64 t0, t1;

	if (imu_block_mode) {
		t0 = ktime_get_ns();
		if (read_addr_sync(XGO_ROLL, 12, b, 20) < 12) return false;
		t1 = ktime_get_ns();
		s->roll = f32le_to_milli(b);
		s->pitch = f32le_to_milli(b + 4);
		s->yaw = f32le_to_milli(b + 8);
	} else {
		// Pitch zuerst und mit eigenem Zeitstempel (Mitte Anfrage/Antwort): die wichtigste Größe
		t0 = ktime_get_ns();
		if (read_addr_sync(XGO_PITCH, 4, b, 20) < 4) return false;
		t1 = ktime_get_ns();
		s->pitch = f32le_to_milli(b);
		if (read_addr_sync(XGO_ROLL, 4, b, 20) < 4) return false;
		s->roll = f32le_to_milli(b);
		if (read_addr_sync(XGO_YAW, 4, b, 20) < 4) return false;
		s->yaw = f32le_to_milli(b);
	}
	s->t_ns = t0 / 2 + t1 / 2;
	return true;
}

static int imu_loop(void *data) {
	struct imu_sample s;
	unsigned long flags;

	while (!kthread_should_stop()) {
		unsigned int hz = READ_ONCE(XGO_IMU_HZ);
		u64 start = ktime_get_ns(), period, used;

		if (hz == 0) {
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

		used = ktime_get_ns() - start;
		if (used < period) {
			u64 us = div_u64(period - used, 1000);
			usleep_range(us, us + 500);
		}
	}
	return 0;
}

// Prüfen, ob die Firmware roll/pitch/yaw als einen 12-Byte-Block liefert (spart 2/3 der Buszeit)
static void imu_detect_block_mode(void) {
	uint8_t b[12];
	s32 r, p, y;

	if (read_addr_sync(XGO_ROLL, 4, b, 50) < 4) goto single;
	r = f32le_to_milli(b);
	if (read_addr_sync(XGO_PITCH, 4, b, 50) < 4) goto single;
	p = f32le_to_milli(b);
	if (read_addr_sync(XGO_YAW, 4, b, 50) < 4) goto single;
	y = f32le_to_milli(b);
	if (read_addr_sync(XGO_ROLL, 12, b, 50) < 12) goto single;
	if (abs(f32le_to_milli(b) - r) < 2000 && abs(f32le_to_milli(b + 4) - p) < 2000 &&
	    abs(f32le_to_milli(b + 8) - y) < 2000 && (r | p | y) != 0) {
		imu_block_mode = true;
		pr_info("XGORider: IMU block read (12 Byte) aktiv\n");
		return;
	}
single:
	imu_block_mode = false;
	pr_info("XGORider: IMU Einzelregister-Modus\n");
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

/*
 * Kurs halten (settings/force_yaw).
 *
 * Vorher: Soll war immer 0° statt des Startkurses, und 128 - (current - wanted) lief
 * in uint8_t über -> Dauer-Drehbefehl statt Korrektur.
 *
 * Jetzt: Soll = Kurs beim Aufstehen bzw. beim Einschalten von force_yaw. Die Firmware
 * dreht nach einem Stoppbefehl deutlich nach (Rampe/Trägheit, bis ~15°), und diese
 * Schleife läuft nur alle sleep_ms_on_loop. Deshalb keine Dauerregelung, sondern kurze
 * Impulse mit fester kleiner Drehrate und danach Ausschwingen, bevor neu gemessen wird.
 * Kommt ein speed_z-Befehl != 128 (xgonav, Fernsteuerung), wird nicht eingegriffen;
 * 1 s nach dem letzten Drehbefehl gilt der dann erreichte Kurs als neues Soll.
 * Konvention: Kurs steigt bei Linksdrehung, speed_z > 128 dreht links.
 */
#define HOLD_DEADBAND_DEG  3
#define HOLD_SPEED         8      // Abstand von 128; kleinere Befehle ignoriert die Firmware
#define HOLD_MS_PER_DEG    40     // Impulsdauer pro Grad Fehler (~25 °/s Wirkung)
#define HOLD_PULSE_MIN_MS  80
#define HOLD_PULSE_MAX_MS  400
#define HOLD_SETTLE_MS     1000
#define HOLD_USER_MS       1000

static void forceyaw(void){
	int err;
	unsigned int ms;
	uint8_t cmd;

	if (!READ_ONCE(hold_valid) ||
	    time_before(jiffies, READ_ONCE(last_user_turn) + msecs_to_jiffies(HOLD_USER_MS))) {
		wanted_yaw = current_yaw;
		WRITE_ONCE(hold_valid, true);
		hold_next = jiffies + msecs_to_jiffies(HOLD_SETTLE_MS);
		return;
	}
	if (time_before(jiffies, hold_next)) return;

	err = ((int)wanted_yaw - (int)current_yaw) % 360;   // auf [-180, 180) umbrechen
	if (err >= 180) err -= 360;
	if (err < -180) err += 360;
	if (abs(err) <= HOLD_DEADBAND_DEG) return;

	ms = clamp_t(unsigned int, abs(err) * HOLD_MS_PER_DEG, HOLD_PULSE_MIN_MS, HOLD_PULSE_MAX_MS);
	cmd = err > 0 ? 128 + HOLD_SPEED : 128 - HOLD_SPEED;
	if (verbose & VERBOSE_MAIN)
		pr_info("XGORider: hold yaw: soll %d ist %d fehler %d -> %u fuer %u ms\n",
		        wanted_yaw, current_yaw, err, cmd, ms);
	write_serial_data(XGO_VYAW, &cmd, 1);
	msleep(ms);
	cmd = 128;
	write_serial_data(XGO_VYAW, &cmd, 1);
	hold_next = jiffies + msecs_to_jiffies(HOLD_SETTLE_MS);
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

static bool read_initial_yaw() {
    initial_yaw = readYaw();
    // force_yaw hält ab jetzt den Kurs beim Aufstehen (wird im nächsten Lauf übernommen)
    WRITE_ONCE(hold_valid, false);
    pr_info("XGORider: initial yaw: %d", initial_yaw);

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
                    read_initial_yaw(); // reset initial yaw, when standup
                    break;

                default:
                    printk(KERN_ERR "XGORider: unknown %d fuck the shit docs\n", operational);
                    break;
            }
        }
    }
}

// Not working, so many tries, hope you got more luck
/*
static void modify_serial_port_settings(const char *tty_name, int baudrate){
    struct tty_struct *tty;
    struct ktermios new_termios;

//#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0)
    //tty = serial_file->private_data;
//#else
    tty = (struct tty_struct *)serial_file->private_data;
//#endif

    if (!tty) {
        printk(KERN_ERR "XGORider: TTY-Gerät konnte nicht erhalten werden.");
        return;
    }

    // Sperren des TTYs während der Modifikation
    tty_lock(tty);
    new_termios = tty->termios;

    tty_termios_encode_baud_rate(&new_termios, baudrate, baudrate);
    tty_set_termios(tty, &new_termios);
    tty_unlock(tty);

    pr_info("Einstellungen für %s erfolgreich geändert.\n", tty_name);
}*/

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

    //ret = display_init();
    //if(ret) return ret;

    imu_detect_block_mode();

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

    filp_close(serial_file, NULL);
    destroyFilesystem();
}

module_init(imu_proc_init);
module_exit(imu_proc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Balen");
MODULE_DESCRIPTION("XGORider IMU Procfs Kernel Module");
