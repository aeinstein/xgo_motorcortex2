//
// Created by aeinstein on 24.01.2025.
//

#ifndef XGO_DRV_H
#define XGO_DRV_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>

// nur für clion on windows
//#include <stdint.h>
//#include <stdbool.h>
//#include <stdio.h>
//#include <sys/types.h>

// Enable with: echo 1 > /proc/XGORider/settings/verbose
static int verbose = 0x00;

#include "../constants.h"

#define PROC_DIR "XGORider"


uint8_t leds[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

static uint8_t XGO_Buttons = 0;

// Settings
static bool XGO_HOLD_YAW = false;
// Zustand der Kurs-Haltung (force_yaw), siehe forceyaw() in xgo-drv.c
static bool hold_valid = false;          // wanted_yaw gültig? (false -> beim nächsten Lauf übernehmen)
static unsigned long hold_next;          // jiffies: frühester nächster Korrekturimpuls
static unsigned long last_user_turn;     // jiffies: letzter speed_z-Befehl ungleich 128
static uint8_t XGO_LOW_BATT = 8;
static bool XGO_SHUTDOWN_ON_LOW_BATT = true;
static int XGO_MS_SLEEP_ON_LOOP = 200;
// Motion-Watchdog: setzt speed_x/speed_z auf 128, wenn so lange kein neuer
// Sollwert kam. 0 = aus (Default, damit der bestehende RC-Client unverändert läuft).
static unsigned int XGO_MOTION_TIMEOUT_MS = 0;
static void motion_watchdog_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(motion_wd, motion_watchdog_fn);

static struct task_struct *thread;

static bool read_initial_yaw(void);
static void batteryCheck(void);
static void checkState(void);
static int16_t readYaw(void);
static int createFilesystem(void);
static void destroyFilesystem(void);
static int16_t readYaw(void);
static void forceyaw(void);

union B2I16 {
	int16_t i;
	char    b[2];
};

#endif //XGO_DRV_H
