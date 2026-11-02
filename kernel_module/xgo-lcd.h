//
// Created by aeinstein on 24.01.2025.
//

#ifndef XGO_LCD_H
#define XGO_LCD_H

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "xgo-gpio.h"

#define SPI_PORT "/dev/spidev0.0"
#define SPI_SPEED 40000000
#define I2C_SPEED 100000

#define LCD_WIDTH 320
#define LCD_HEIGHT 240

int devSpiProtocol_cmd_read(char *text);
int devSpiProtocol_cmd_write(char *text);

struct spidev_data {
	dev_t           devt;
	spinlock_t      spi_lock;
	struct spi_device   *spi;
	struct list_head    device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex        buf_lock;
	unsigned        users;
	u8          *tx_buffer;
	u8          *rx_buffer;
	u32         speed_hz;
};

static struct spi_device *spi;

int devSpiProtocol_cmd_read(char *text){
	pr_warn("devSpiProtocol_cmd_read: %s\n",text);
	return 0;
}
EXPORT_SYMBOL_GPL(devSpiProtocol_cmd_read);

int devSpiProtocol_cmd_write(char *text) {
	char ch = 0x61;
	struct spi_transfer t = {
		.tx_buf     = &ch,
		.len        = sizeof(ch),
		.bits_per_word = 8,
		.speed_hz = 4000000,
	};
	struct spi_message  m;
	pr_warn("devSpiProtocol_cmd_write: %s\n",text);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	spi_sync(spi, &m);
	//spi_write(spi, &ch, sizeof(ch));
	return 0;
}
EXPORT_SYMBOL_GPL(devSpiProtocol_cmd_write);


static struct gpio_desc *rst, *dc, *bl;
struct spi_ioc_transfer xfer[2];

int display_init(void);
void display_command(uint8_t command, char * data, uint8_t len);
int spi_init(void);
void display_reset(void);

#endif //XGO_LCD_H
