//
// Created by aeinstein on 24.01.2025.
//

#ifndef XGO_PROC_H
#define XGO_PROC_H

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "xgo-drv.h"
#include "xgo-serial.h"

static struct proc_dir_entry *proc_imu, *proc_settings, *proc_yaw, *proc_state, *proc_buttons, *proc_battery, *proc_action,
	*proc_leds, *proc_led1, *proc_led2, *proc_led3, *proc_led4, *proc_speed_x, *proc_speed_z,
	*proc_height, *proc_roll;

static ssize_t yaw_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t battery_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t state_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t buttons_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t settings_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t settings_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t translation_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos);

#endif //XGO_PROC_H
