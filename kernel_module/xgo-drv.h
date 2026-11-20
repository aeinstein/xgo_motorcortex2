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
#include <linux/poll.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/rcupdate.h>

// Enable with: echo 1 > /proc/XGORider/settings/verbose
static int verbose = 0x00;

#include "../constants.h"

#define PROC_DIR "XGORider"


uint8_t leds[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

static uint8_t XGO_Buttons = 0;

// Settings
static uint8_t XGO_LOW_BATT = 8;
static bool XGO_SHUTDOWN_ON_LOW_BATT = true;
static int XGO_MS_SLEEP_ON_LOOP = 200;
// Motion-Watchdog: setzt speed_x/speed_z auf 128, wenn so lange kein neuer
// Sollwert kam. 0 = aus (Default, damit der bestehende RC-Client unverändert läuft).
static unsigned int XGO_MOTION_TIMEOUT_MS = 0;
static void motion_watchdog_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(motion_wd, motion_watchdog_fn);

// IPv4 des Pi an den ESP32 melden (Firmware-Patch 0025, Register 0x18): Adresse dieser Schnittstelle, alle
// XGO_PI_IP_MS neu (der ESP32 vergisst sie nach 10 s ohne Meldung) und sofort bei Änderung.
#define XGO_PI_IP_MS 3000
static char XGO_IP_DEV[IFNAMSIZ] = "wlan0";
static u32 XGO_PI_IP_SENT;                      // zuletzt gemeldete Adresse (Netz-Byte-Reihenfolge), 0 = keine

static struct task_struct *thread;

static void batteryCheck(void);
static void checkState(void);
static int16_t readYaw(void);
static int createFilesystem(void);
static void destroyFilesystem(void);

union B2I16 {
	int16_t i;
	char    b[2];
};

#endif //XGO_DRV_H
