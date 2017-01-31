/*
 * spidev.h: SPI device helper
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

#ifndef _SPIDEV_H_
#define _SPIDEV_H_

#include <stdint.h>

struct spidev;

/* SPI device configuration */
struct spidev_config {
	uint32_t speed_hz;
	uint16_t delay_usecs;
	uint8_t bits_per_word;
	uint8_t cs_change;
};

/* Open and setup SPI device */
struct spidev *spidev_open(const char *device, struct spidev_config *cfg);
void spidev_close(struct spidev *dev);

/* Simple read / write operations */
int spidev_read(struct spidev *dev, char *buffer, size_t size);
int spidev_write(struct spidev *dev, char *buffer, size_t size);

/* Transfer operation: write tx_buffer and read data to rx_buffer */
int spidev_transfer(struct spidev *dev, const char *tx_buffer, size_t tx_size,
		    char *rx_buffer, size_t rx_size);
int spidev_transfer_full(struct spidev *dev, const char *tx_buffer,
			 char *rx_buffer, size_t size);

/* Helpers for registers access through SPI */
int spidev_read_reg(struct spidev *dev, uint8_t address, uint8_t *value);
int spidev_write_reg(struct spidev *dev, uint8_t address, uint8_t value);

int spidev_read_reg16(struct spidev *dev, uint8_t address, uint16_t *value);
int spidev_write_reg16(struct spidev *dev, uint8_t address, uint16_t value);

int spidev_read16_reg(struct spidev *dev, uint16_t address, uint8_t *value);
int spidev_write16_reg(struct spidev *dev, uint16_t address, uint8_t value);

int spidev_read16_reg16(struct spidev *dev, uint16_t address, uint16_t *value);
int spidev_write16_reg16(struct spidev *dev, uint16_t address, uint16_t value);

#endif /* _SPIDEV_H_ */
