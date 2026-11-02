//
// Created by aeinstein on 24.01.2025.
//

#include "xgo-proc.h"

static const struct proc_ops yaw_ops = {
	.proc_read = yaw_read,
};

static const struct proc_ops state_ops = {
	.proc_read = state_read,
};

static const struct proc_ops battery_ops = {
	.proc_read = battery_read,
};

static const struct proc_ops buttons_ops = {
	.proc_read = buttons_read,
};

static const struct proc_ops settings_ops = {
	.proc_read = settings_read,
    .proc_write = settings_write,
};

static const struct proc_ops translation_ops = {
    .proc_write = translation_write,
};

// /proc/XGORider/imu: "seq t_ns roll pitch yaw" (Milligrad), neueste Probe
static int imu_show(struct seq_file *m, void *v) {
	struct imu_sample smp;
	unsigned long flags;

	spin_lock_irqsave(&imu_lock, flags);
	smp = imu_log[imu_seq % IMU_LOG_LEN];
	spin_unlock_irqrestore(&imu_lock, flags);
	if (smp.seq == 0) return 0;  // noch keine Probe
	seq_printf(m, "%u %llu %d %d %d\n", smp.seq, smp.t_ns, smp.roll, smp.pitch, smp.yaw);
	return 0;
}

// /proc/XGORider/imu_log: die letzten IMU_LOG_LEN Proben, älteste zuerst
static int imu_log_show(struct seq_file *m, void *v) {
	static struct imu_sample copy[IMU_LOG_LEN];   // nicht auf den Kernel-Stack (3 KB)
	static DEFINE_MUTEX(copy_lock);
	unsigned long flags;
	u32 last, first, i;

	mutex_lock(&copy_lock);
	spin_lock_irqsave(&imu_lock, flags);
	memcpy(copy, imu_log, sizeof(copy));
	last = imu_seq;
	spin_unlock_irqrestore(&imu_lock, flags);

	first = last > IMU_LOG_LEN - 1 ? last - (IMU_LOG_LEN - 1) : 1;
	for (i = first; last && i <= last; i++) {
		struct imu_sample *p = &copy[i % IMU_LOG_LEN];
		if (p->seq != i) continue;
		seq_printf(m, "%u %llu %d %d %d\n", p->seq, p->t_ns, p->roll, p->pitch, p->yaw);
	}
	mutex_unlock(&copy_lock);
	return 0;
}

static int serial_stats_show(struct seq_file *m, void *v) {
	seq_printf(m, "frames %d\nbad %d\ntimeouts %d\nunsolicited %d\nimu_seq %u\nimu_hz %u\nimu_block %d\n",
		atomic_read(&stat_frames), atomic_read(&stat_bad), atomic_read(&stat_timeouts),
		atomic_read(&stat_unsolicited), READ_ONCE(imu_seq), XGO_IMU_HZ, imu_block_mode);
	return 0;
}

static int imu_open(struct inode *inode, struct file *file) { return single_open(file, imu_show, NULL); }
static int imu_log_open(struct inode *inode, struct file *file) { return single_open_size(file, imu_log_show, NULL, IMU_LOG_LEN * 64); }
static int serial_stats_open(struct inode *inode, struct file *file) { return single_open(file, serial_stats_show, NULL); }

static const struct proc_ops imu_ops = {
	.proc_open = imu_open, .proc_read = seq_read, .proc_lseek = seq_lseek, .proc_release = single_release,
};
static const struct proc_ops imu_log_ops = {
	.proc_open = imu_log_open, .proc_read = seq_read, .proc_lseek = seq_lseek, .proc_release = single_release,
};
static const struct proc_ops serial_stats_ops = {
	.proc_open = serial_stats_open, .proc_read = seq_read, .proc_lseek = seq_lseek, .proc_release = single_release,
};

static ssize_t buttons_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos) {
	char decimal_buffer[3];
	snprintf(decimal_buffer, sizeof(decimal_buffer), "%d", XGO_Buttons);
	return simple_read_from_buffer(user_buf, count, ppos, decimal_buffer, strlen(decimal_buffer));
}

static ssize_t yaw_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos) {
	char decimal_buffer[7];
	snprintf(decimal_buffer, sizeof(decimal_buffer), "%d", current_yaw);
	return simple_read_from_buffer(user_buf, count, ppos, decimal_buffer, strlen(decimal_buffer));
}

static ssize_t battery_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos) {
	char decimal_buffer[4];
	snprintf(decimal_buffer, sizeof(decimal_buffer), "%d", battery);
	return simple_read_from_buffer(user_buf, count, ppos, decimal_buffer, strlen(decimal_buffer));
}

static ssize_t state_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos) {
	char decimal_buffer[4];
	snprintf(decimal_buffer, sizeof(decimal_buffer), "%d", operational);
	return simple_read_from_buffer(user_buf, count, ppos, decimal_buffer, strlen(decimal_buffer));
}


static ssize_t settings_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos) {
    char buffer[10];
    size_t len;

    const char *setting_name = file->f_path.dentry->d_name.name;

    // Überprüfen, welcher Wert gelesen werden soll
    if (strcmp(setting_name, "verbose") == 0) {
    	len = snprintf(buffer, sizeof(buffer), "%d\n", verbose);

    } else if (strcmp(setting_name, "low_batt_level") == 0) {
        len = snprintf(buffer, sizeof(buffer), "%d\n", XGO_LOW_BATT);

    } else if (strcmp(setting_name, "sleep_ms_on_loop") == 0) {
    	len = snprintf(buffer, sizeof(buffer), "%d\n", XGO_MS_SLEEP_ON_LOOP);

    } else if (strcmp(setting_name, "shutdown_on_low_batt") == 0) {
      	if(XGO_SHUTDOWN_ON_LOW_BATT) len = snprintf(buffer, sizeof(buffer), "1\n");
      	else len = snprintf(buffer, sizeof(buffer), "0\n");

    } else if (strcmp(setting_name, "force_yaw") == 0) {
      	if(XGO_HOLD_YAW) len = snprintf(buffer, sizeof(buffer), "1\n");
      	else len = snprintf(buffer, sizeof(buffer), "0\n");

    } else if (strcmp(setting_name, "motion_timeout_ms") == 0) {
    	len = snprintf(buffer, sizeof(buffer), "%u\n", XGO_MOTION_TIMEOUT_MS);

    } else if (strcmp(setting_name, "imu_hz") == 0) {
    	len = snprintf(buffer, sizeof(buffer), "%u\n", XGO_IMU_HZ);

    } else {
        return -EINVAL; // Ungültiger Zugriff
    }

    return simple_read_from_buffer(user_buf, count, ppos, buffer, len);
}

static ssize_t translation_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos) {
    char buffer[8];
    const char *setting_name = file->f_path.dentry->d_name.name;
    int parsed;
    uint8_t value;

    if (count == 0 || count >= sizeof(buffer)) {
        return -EINVAL; // Eingabewert zu groß
    }

    if (copy_from_user(buffer, user_buf, count)) {
        return -EFAULT; // Fehler beim Kopieren
    }

    buffer[count] = '\0'; // Null-terminieren für Sicherheit

    // kstrtoint statt sscanf("%d") in int8_t (schrieb 4 Byte in 1-Byte-Variable)
    if (kstrtoint(strim(buffer), 10, &parsed)) {
        return -EINVAL;
    }

    if (strcmp(setting_name, "roll") == 0) {
    	// Roll-Winkel in Grad, -20..20. Die Firmware bildet 0x36 linear 0..255 -> -20..+20 ab
    	// (Mitte 128). Vorher ging "roll" auf 0x61 (Roll-Balance an/aus), negative Werte
    	// wurden abgelehnt.
    	if (parsed < -XGO_ROLL_MAX_DEG || parsed > XGO_ROLL_MAX_DEG) return -EINVAL;
    	value = (uint8_t)DIV_ROUND_CLOSEST((parsed + XGO_ROLL_MAX_DEG) * 255, 2 * XGO_ROLL_MAX_DEG);
    	write_serial_data(XGO_ROLL_ANGLE, &value, 1);
    	return count;
    }
    if (parsed < 0 || parsed > 255) {
        return -EINVAL;
    }
    value = (uint8_t)parsed;

    if (strcmp(setting_name, "speed_x") == 0) {
    	write_serial_data(XGO_VX, &value, 1);
    	arm_motion_watchdog();

    } else if(strcmp(setting_name, "speed_z") == 0) {
    	write_serial_data(XGO_VYAW, &value, 1);
    	arm_motion_watchdog();
    	// gewollte Drehung: force_yaw hält still und übernimmt danach den neuen Kurs
    	if (value != 128) WRITE_ONCE(last_user_turn, jiffies);

    } else if(strcmp(setting_name, "height") == 0) {
    	write_serial_data(XGO_BODYHEIGHT, &value, 1);

    } else if(strcmp(setting_name, "roll_balance") == 0) {
    	// 0x61 (XGO_IMU): Roll-Balance an/aus, früher unter "roll"
    	value = value ? 1 : 0;
    	write_serial_data(XGO_IMU, &value, 1);
    }

    return count;
}

static ssize_t settings_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos) {
    char buffer[10];
    const char *setting_name = file->f_path.dentry->d_name.name;

    // Eingabedaten überprüfen und kopieren
    if (count >= sizeof(buffer)) {
        return -EINVAL; // Eingabewert zu groß
    }

    if (copy_from_user(buffer, user_buf, count)) {
        return -EFAULT; // Fehler beim Kopieren
    }

    buffer[count] = '\0'; // Null-terminieren für Sicherheit

	if(verbose & VERBOSE_PROC) pr_info("XGORider: got %s len:%zu\n", buffer, count);

    // Einstellungen abhängig vom Namen ändern
    if (strcmp(setting_name, "verbose") == 0) {
        int v;
        if (kstrtoint(strim(buffer), 0, &v)) return -EINVAL;
        verbose = v;  // Bitmaske, siehe README (1 serial, 2 gpio, 4 proc, 8 main)
        pr_info("XGORider: verbose = 0x%x", verbose);

    } else if (strcmp(setting_name, "shutdown_on_low_batt") == 0) {
        if(buffer[0] != 0x30) XGO_SHUTDOWN_ON_LOW_BATT = true;
        else XGO_SHUTDOWN_ON_LOW_BATT = false;

    } else if (strcmp(setting_name, "calibration") == 0) {
    	// Beinkalibrierung (0x04): 1 = starten (Motoren los), 0 = Nullpunkte speichern.
    	// Vorher gingen die Zeichen '0'/'1' (0x30/0x31) raus und an/aus war vertauscht;
    	// die Firmware wertet nur die Bytes 0 und 1 aus.
    	int v;
    	char b;
    	if (kstrtoint(strim(buffer), 10, &v) || (v != 0 && v != 1)) return -EINVAL;
    	b = (char)v;
    	write_serial_data(XGO_CALIBRATION, &b, 1);

    } else if (strcmp(setting_name, "pitch_calibration") == 0) {
    	// Nick-Trimm (0x09): nur "1". Die Firmware hält den Rider im Stand, wartet bis er
    	// ruhig steht, und speichert die Haltekorrektur als Trimm. Rider muss balancieren.
    	char b = 1;
    	if (strcmp(strim(buffer), "1") != 0) return -EINVAL;
    	write_serial_data(XGO_PITCH_CALIBRATION, &b, 1);

    } else if (strcmp(setting_name, "force_yaw") == 0) {
        bool on = buffer[0] != 0x30;
        // beim Einschalten den aktuellen Kurs als Soll übernehmen (nicht den vom Aufstehen)
        if (on && !XGO_HOLD_YAW) WRITE_ONCE(hold_valid, false);
        XGO_HOLD_YAW = on;

 	} else if (strcmp(setting_name, "low_batt_level") == 0) {
      	// vorher: sscanf(buffer, "%d", XGO_LOW_BATT) -> Wert statt Pointer = Oops
      	u8 v;
      	if (kstrtou8(strim(buffer), 10, &v) || v > 100) return -EINVAL;
      	XGO_LOW_BATT = v;

    } else if (strcmp(setting_name, "sleep_ms_on_loop") == 0) {
    	sscanf(buffer, "%d", &XGO_MS_SLEEP_ON_LOOP);
    	pr_info("XGORider: %d ms sleep on loop\n", XGO_MS_SLEEP_ON_LOOP);

    } else if (strcmp(setting_name, "motion_timeout_ms") == 0) {
    	unsigned int v;
    	if (kstrtouint(strim(buffer), 10, &v) || v > 60000) return -EINVAL;
    	XGO_MOTION_TIMEOUT_MS = v;
    	if (v == 0) cancel_delayed_work(&motion_wd);
    	pr_info("XGORider: motion watchdog %u ms\n", v);

    } else if (strcmp(setting_name, "imu_hz") == 0) {
    	unsigned int v;
    	if (kstrtouint(strim(buffer), 10, &v) || v > 200) return -EINVAL;
    	WRITE_ONCE(XGO_IMU_HZ, v);
    	pr_info("XGORider: IMU %u Hz\n", v);

    } else {
        return -EINVAL; // Ungültiger Zugriff
    }

    if(verbose & VERBOSE_PROC) pr_info("Setting '%s' wurde auf '%s' aktualisiert\n", setting_name, buffer);

    return count;
}

static ssize_t leds_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos) {
  	if(verbose & VERBOSE_PROC) pr_info("led write %s", user_buf);
    char buffer[9];  // "#RRGGBB\n" + Null-Terminierung (vorher 8 -> buffer[8] out of bounds)
    uint8_t red, green, blue;

    // Prüfen, ob die Eingabegröße korrekt ist
    if (count < 7 || count > 8) {  // Erwartet "#RRGGBB" oder "#RRGGBB\n"
        return -EINVAL;           // Ungültige Eingabe
    }

    // Kopiere Daten aus dem Userspace in den Kernel-Puffer
    if (copy_from_user(buffer, user_buf, count)) {
        return -EFAULT;           // Fehler beim Kopieren der Daten
    }

    // Null-terminiere den String für Sicherheit
    buffer[count] = '\0';

    // Überprüfen, ob der String das erwartete Format enthält (beginnend mit '#' und gültigen hexadezimalen Zeichen)
    if (buffer[0] != '#' ||
        !isxdigit(buffer[1]) || !isxdigit(buffer[2]) ||
        !isxdigit(buffer[3]) || !isxdigit(buffer[4]) ||
        !isxdigit(buffer[5]) || !isxdigit(buffer[6])) {
        return -EINVAL;           // Ungültiges Format
    }

    // Konvertiere den Hex-String in RGB-Werte
    if (sscanf(buffer + 1, "%2hhx%2hhx%2hhx", &red, &green, &blue) != 3) {
        return -EINVAL;           // Fehler bei der Konvertierung
    }

    // Debug-Log zur Überprüfung der extrahierten RGB-Daten
    if(verbose & VERBOSE_PROC) pr_info("LED %s - RGB Werte: R=%u, G=%u, B=%u\n", file->f_path.dentry->d_name.name, red, green, blue);

    // Hier können Sie die RGB-Werte auf die spezifische LED-Ansteuerung anwenden.
    // Beispiel: LEDs aktualisieren (in Abhängigkeit vom Dateinamen)
    if (strcmp(file->f_path.dentry->d_name.name, "0") == 0) {
        leds[0][0] = red;   // Setze den LED 1 Rotwert
        leds[0][1] = green;
        leds[0][2] = blue;
        write_serial_data(XGO_LED1, leds[0], 3);

    } else if (strcmp(file->f_path.dentry->d_name.name, "1") == 0) {
        leds[1][0] = red;
        leds[1][1] = green;
        leds[1][2] = blue;
        write_serial_data(XGO_LED2, leds[1], 3);

    } else if (strcmp(file->f_path.dentry->d_name.name, "2") == 0) {
        leds[2][0] = red;
        leds[2][1] = green;
        leds[2][2] = blue;
        write_serial_data(XGO_LED3, leds[2], 3);

    } else if (strcmp(file->f_path.dentry->d_name.name, "3") == 0) {
        leds[3][0] = red;
        leds[3][1] = green;
        leds[3][2] = blue;
        write_serial_data(XGO_LED4, leds[3], 3);

    } else {
        return -EINVAL;  // Dateiname entspricht keiner bekannten LED
    }


    // Gib die Anzahl der verarbeiteten Bytes zurück
    return count;
}

static ssize_t action_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos){
	char buffer[5];
    uint8_t action;

    // vorher ohne Längenprüfung: copy_from_user(buffer[4], ..., count) -> Kernel-Stack-Overflow
    if (count == 0 || count >= sizeof(buffer)) {
        return -EINVAL;
    }

    // Kopiere Daten aus dem Userspace in den Kernel-Puffer
    if (copy_from_user(buffer, user_buf, count)) {
        return -EFAULT;           // Fehler beim Kopieren der Daten
    }
    buffer[count] = '\0';

    if (kstrtou8(strim(buffer), 10, &action)) {
        return -EINVAL;           // Fehler bei der Konvertierung
    }

    pr_info("Action: %d", action);
	write_serial_data(XGO_ACTION, &action, 1);

  return count;
}


static const struct proc_ops led_ops = {
	.proc_write = leds_write,
};

static const struct proc_ops action_ops = {
	.proc_write = action_write,
};

static int createFilesystem(){
	proc_imu = proc_mkdir(PROC_DIR, NULL);

	if (!proc_imu) {
		destroyFilesystem();
		return -ENOMEM;
	}

    // /settings
    proc_settings = proc_mkdir("settings", proc_imu);
    proc_create("verbose", 0666, proc_settings, &settings_ops);
    proc_create("low_batt_level", 0666, proc_settings, &settings_ops);
    proc_create("shutdown_on_low_batt", 0666, proc_settings, &settings_ops);
    proc_create("force_yaw", 0666, proc_settings, &settings_ops);
    proc_create("sleep_ms_on_loop", 0666, proc_settings, &settings_ops);
	proc_create("calibration", 0666, proc_settings, &settings_ops);
	proc_create("pitch_calibration", 0666, proc_settings, &settings_ops);
	proc_create("motion_timeout_ms", 0666, proc_settings, &settings_ops);
	proc_create("imu_hz", 0666, proc_settings, &settings_ops);


	proc_yaw = proc_create("yaw", 0444, proc_imu, &yaw_ops);
	proc_state = proc_create("state", 0444, proc_imu, &state_ops);
	proc_buttons = proc_create("buttons", 0444, proc_imu, &buttons_ops);
	proc_battery = proc_create("battery", 0444, proc_imu, &battery_ops);
	proc_action = proc_create("action", 0666, proc_imu, &action_ops);
    proc_speed_x = proc_create("speed_x", 0666, proc_imu, &translation_ops);
    proc_speed_z = proc_create("speed_z", 0666, proc_imu, &translation_ops);
	proc_height = proc_create("height", 0666, proc_imu, &translation_ops);
	proc_roll = proc_create("roll", 0666, proc_imu, &translation_ops);
	proc_create("roll_balance", 0666, proc_imu, &translation_ops);
	proc_create("imu", 0444, proc_imu, &imu_ops);
	proc_create("imu_log", 0444, proc_imu, &imu_log_ops);
	proc_create("serial_stats", 0444, proc_imu, &serial_stats_ops);

	if (!proc_yaw || !proc_state || !proc_battery || !proc_buttons) return -ENOMEM;


    // /leds
    proc_leds = proc_mkdir("leds", proc_imu);
	proc_led1 = proc_create("0", 0666, proc_leds, &led_ops);
	proc_led2 = proc_create("1", 0666, proc_leds, &led_ops);
	proc_led3 = proc_create("2", 0666, proc_leds, &led_ops);
	proc_led4 = proc_create("3", 0666, proc_leds, &led_ops);

	pr_info("XGORider: proc bindings created\n");
	return 0;
}

static void destroyFilesystem(){
	remove_proc_entry("yaw", proc_imu);
	remove_proc_entry("state", proc_imu);
	remove_proc_entry("buttons", proc_imu);
	remove_proc_entry("battery", proc_imu);
	remove_proc_entry("action", proc_imu);
    remove_proc_entry("speed_x", proc_imu);
	remove_proc_entry("speed_z", proc_imu);
	remove_proc_entry("height", proc_imu);
	remove_proc_entry("roll", proc_imu);
	remove_proc_entry("roll_balance", proc_imu);
	remove_proc_entry("imu", proc_imu);
	remove_proc_entry("imu_log", proc_imu);
	remove_proc_entry("serial_stats", proc_imu);

	remove_proc_entry("0", proc_leds);
	remove_proc_entry("1", proc_leds);
	remove_proc_entry("2", proc_leds);
	remove_proc_entry("3", proc_leds);
	remove_proc_entry("leds", proc_imu);

    remove_proc_entry("verbose", proc_settings);
    remove_proc_entry("low_batt_level", proc_settings);
    remove_proc_entry("shutdown_on_low_batt", proc_settings);
    remove_proc_entry("force_yaw", proc_settings);
    remove_proc_entry("sleep_ms_on_loop", proc_settings);
    remove_proc_entry("calibration", proc_settings);
    remove_proc_entry("pitch_calibration", proc_settings);
    remove_proc_entry("motion_timeout_ms", proc_settings);
    remove_proc_entry("imu_hz", proc_settings);
    remove_proc_entry("settings", proc_imu);

	remove_proc_entry(PROC_DIR, NULL);
    pr_info("XGORider: proc bindings deleted\n");
}