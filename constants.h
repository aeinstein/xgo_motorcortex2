//
// Created by aeinstein on 23.01.2025.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define XGO_BATTERY 0x01
#define XGO_STATE 0x02
#define XGO_PERFORM 0x03
#define XGO_CALIBRATION 0x04
#define XGO_UPGRADE 0x05
#define XGO_SET_ORIGIN 0x06
#define XGO_FIRMWARE_VERSION 0x07
#define XGO_RING_SWITCH 0x0a
#define XGO_BT_NAME 0x13
#define XGO_VX 0x30
#define XGO_VYAW 0x32
#define XGO_BODYHEIGHT 0x35
#define XGO_ROTATE_X_PERIODIC 0x39
#define XGO_ACTION 0x3e
#define XGO_ROLL_ANGLE 0x36      // Roll-Sollwinkel, 0..255 -> -20..+20 Grad
#define XGO_ROLL_MAX_DEG 20
#define XGO_IMU 0x61
#define XGO_ROLL 0x62
#define XGO_PITCH 0x63
#define XGO_YAW 0x64
#define XGO_ROLL_INT 0x66
#define XGO_PITCH_INT 0x67
#define XGO_YAW_INT 0x68
#define XGO_LED1 0x69
#define XGO_LED2 0x6a
#define XGO_LED3 0x6b
#define XGO_LED4 0x6c

#define GPIO_BUTTON_A 17
#define GPIO_BUTTON_B 22
#define GPIO_BUTTON_C 23
#define GPIO_BUTTON_D 24

#define GPIO_LCD_RST 27
#define GPIO_LCD_DC 25
#define GPIO_LCD_BL 0

#define GPIO_OFFSET 512

#define SPI_FREQ_MHZ 100


#define VERBOSE_SERIAL 0x01
#define VERBOSE_GPIO 0x02
#define VERBOSE_PROC 0x04
#define VERBOSE_MAIN 0x08

#endif //CONSTANTS_H
