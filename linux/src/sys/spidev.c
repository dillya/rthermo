/*
 * spidev.c: SPI device helper
 *
 * Copyright (C) 2017 Alexandre Dilly <dillya@sparod.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spidev.h>


#include "spidev.h"

struct spidev {
	struct spidev_config cfg;
	int fd;
};

struct spidev *spidev_open(const char *device, struct spidev_config *cfg)
{
	struct spidev *dev;

	/* Allocate structure */
	dev = malloc(sizeof(*dev));
	if (!dev)
		return NULL;

	/* Copy configuration */
	dev->cfg = *cfg;

	/* Open SPI device */
	dev->fd = open(device, O_RDWR);
	if (dev->fd < 0)
		goto error;

	return dev;

error:
	free(dev);
	return NULL;
}

void spidev_close(struct spidev *dev)
{
	if (!dev)
		return;

	/* Close SPI device */
	close(dev->fd);

	free(dev);
}

int spidev_read(struct spidev *dev, char *buffer, size_t size)
{
	struct spi_ioc_transfer xfer;

	/* Setup RX buffer */
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = 0;
	xfer.rx_buf = (unsigned long) buffer;
	xfer.len = size;
	xfer.cs_change = dev->cfg.cs_change;
	xfer.delay_usecs = dev->cfg.delay_usecs;
	xfer.speed_hz = dev->cfg.speed_hz;
	xfer.bits_per_word = dev->cfg.bits_per_word;

	return ioctl(dev->fd, SPI_IOC_MESSAGE(1), &xfer) > 0 ? 0 : -1;
}

int spidev_write(struct spidev *dev, char *buffer, size_t size)
{
	struct spi_ioc_transfer xfer;

	/* Setup TX buffer */
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (unsigned long) buffer;
	xfer.rx_buf = 0;
	xfer.len = size;
	xfer.cs_change = dev->cfg.cs_change;
	xfer.delay_usecs = dev->cfg.delay_usecs;
	xfer.speed_hz = dev->cfg.speed_hz;
	xfer.bits_per_word = dev->cfg.bits_per_word;

	return ioctl(dev->fd, SPI_IOC_MESSAGE(1), &xfer) > 0 ? 0 : -1;
}

int spidev_transfer(struct spidev *dev,
			const char *tx_buffer, size_t tx_size,
			char *rx_buffer, size_t rx_size)
{
	struct spi_ioc_transfer xfer[2];

	/* Check size */
	if (!tx_size || !rx_size) {
		fprintf(stderr, "rx_size and tx_size must be non null\n");
		return -1;
	}

	/* Setup TX buffer */
	memset(xfer, 0, sizeof(xfer));
	xfer[0].tx_buf = (unsigned long) tx_buffer;
	xfer[0].rx_buf = 0;
	xfer[0].len = tx_size;
	xfer[0].cs_change = dev->cfg.cs_change;
	xfer[0].delay_usecs = dev->cfg.delay_usecs;
	xfer[0].speed_hz = dev->cfg.speed_hz;
	xfer[0].bits_per_word = dev->cfg.bits_per_word;

	/* Setup RX buffer */
	xfer[1].tx_buf = 0;
	xfer[1].rx_buf = (unsigned long) rx_buffer;
	xfer[1].len = rx_size;
	xfer[1].cs_change = dev->cfg.cs_change;
	xfer[1].delay_usecs = dev->cfg.delay_usecs;
	xfer[1].speed_hz = dev->cfg.speed_hz;
	xfer[1].bits_per_word = dev->cfg.bits_per_word;

	return ioctl(dev->fd, SPI_IOC_MESSAGE(2), xfer) > 0 ? 0 : -1;
}

int spidev_transfer_full(struct spidev *dev, const char *tx_buffer,
			 char *rx_buffer, size_t size)
{
	struct spi_ioc_transfer xfer;

	/* Setup TX/RX buffer */
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (unsigned long) tx_buffer;
	xfer.rx_buf = (unsigned long) rx_buffer;
	xfer.len = size;
	xfer.cs_change = dev->cfg.cs_change;
	xfer.delay_usecs = dev->cfg.delay_usecs;
	xfer.speed_hz = dev->cfg.speed_hz;
	xfer.bits_per_word = dev->cfg.bits_per_word;

	return ioctl(dev->fd, SPI_IOC_MESSAGE(1), &xfer) == 1 ? 0 : -1;
}

int spidev_read_reg(struct spidev *dev, uint8_t address, uint8_t *value)
{
	return spidev_transfer(dev, &address, 1, value, 1);
}

int spidev_write_reg(struct spidev *dev, uint8_t address, uint8_t value)
{
	uint8_t tx[2] = { address, value };
	return spidev_write(dev, tx, 2);
}

int spidev_read_reg16(struct spidev *dev, uint8_t address, uint16_t *value)
{
	uint8_t rx[2];
	int ret;
	ret = spidev_transfer(dev, &address, 1, rx, 2);
	*value = rx[0] << 8 | rx[1];
	return ret;
}

int spidev_write_reg16(struct spidev *dev, uint8_t address, uint16_t value)
{
	uint8_t tx[3] = { address, value >> 8, value };
	return spidev_write(dev, tx, 3);
}

int spidev_read16_reg(struct spidev *dev, uint16_t address, uint8_t *value)
{
	uint8_t tx[2] = { address >> 8, address };
	return spidev_transfer(dev, tx, 2, value, 1);
}

int spidev_write16_reg(struct spidev *dev, uint16_t address, uint8_t value)
{
	uint8_t tx[3] = { address >> 8, address, value };
	return spidev_write(dev, tx, 3);
}

int spidev_read16_reg16(struct spidev *dev, uint16_t address, uint16_t *value)
{
	uint8_t tx[2] = { address >> 8, address };
	uint8_t rx[2];
	int ret;
	ret = spidev_transfer(dev, tx, 2, rx, 2);
	*value = rx[0] << 8 | rx[1];
	return ret;
}

int spidev_write16_reg16(struct spidev *dev, uint16_t address, uint16_t value)
{
	uint8_t tx[4] = { address >> 8, address, value >> 8, value };
	return spidev_write(dev, tx, 4);
}
