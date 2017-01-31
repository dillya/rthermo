/*
 * nrf24.c: nRF24L01(+) handler for RF communication
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
#include <stdint.h>
#include <unistd.h>
#include <errno.h>

#include "sys/spidev.h"
#include "sys/gpio.h"
#include "nrf24.h"

#define NRF24_SPI_SPEED_HZ		8000000
#define NRF24_SPI_MAX_REGISTER_LENGTH	5
#define NRF24_SPI_MAX_PAYLOAD_LENGTH	32

/* nrf24 handler */
struct nrf24 {
	/* SPI device */
	struct spidev *spi;
	/* CE GPIO */
	int ce_gpio;
	int ce_fd;
	/* IRQ GPIO */
	int irq_gpio;
	int irq_fd;
	/* nRF24 version */
	int version_p;
	int feature_activated;
	/* Setup */
	uint8_t address_width;
	/* Status */
	int power_up;
	int rx_mode;
	int rx_started;
};

struct nrf24 *nrf24_new(struct nrf24_config *cfg)
{
	struct spidev_config spi_cfg = {
		.speed_hz = NRF24_SPI_SPEED_HZ,
		.delay_usecs = 0,
		.bits_per_word = 8,
		.cs_change = 0,
	};
	struct nrf24 *nrf;
	uint8_t reg;
	int ret;

	/* Allocate structure */
	nrf = malloc(sizeof(*nrf));
	if (!nrf)
		return NULL;

	/* Setup default values */
	nrf->feature_activated = 0;
	nrf->address_width = 5;
	nrf->power_up = 0;
	nrf->rx_mode = 0;
	nrf->rx_started = 0;

	/* Open SPI device */
	nrf->spi = spidev_open(cfg->spi_dev, &spi_cfg);
	if (!nrf->spi) {
		fprintf(stderr, "Failed to open SPI device %s\n", cfg->spi_dev);
		goto spi_error;
	}

	/* Setup CE GPIO */
	nrf->ce_gpio = cfg->ce_gpio;
	ret = gpio_export(cfg->ce_gpio);
	ret |= gpio_set_direction(cfg->ce_gpio, GPIO_DIR_OUTPUT);
	nrf->ce_fd = gpio_value_open(cfg->ce_gpio);
	if (ret || nrf->ce_fd < 0) {
		fprintf(stderr, "Failed to setup CE GPIO (%u)\n", cfg->ce_gpio);
		goto ce_error;
	}
	gpio_value_write(nrf->ce_fd, 0);

	/* Setup IRQ GPIO */
	nrf->irq_gpio = cfg->irq_gpio;
	if (cfg->irq_gpio >= 0) {
		ret = gpio_export(cfg->irq_gpio);
		ret |= gpio_irq_setup(cfg->irq_gpio, GPIO_IRQ_MODE_FALLING);
		nrf->irq_fd = gpio_value_open(cfg->irq_gpio);
		if (ret || nrf->irq_fd < 0) {
			fprintf(stderr, "Failed to setup IRQ GPIO (%u)\n",
				cfg->irq_gpio);
			goto irq_error;
		}
	} else
		nrf->irq_fd = -1;

	/* Reset chip
	 *  1. Power down
	 *  2. Clear DR and DS flags
	 *  3. Flush TX / RX buffers
	 */
	if (spidev_write_reg(nrf->spi, NRF24_REG_CONFIG | 0x20, 0x08) ||
	    spidev_write_reg(nrf->spi, NRF24_REG_STATUS | 0x20, 0x70) ||
	    spidev_write_reg(nrf->spi, 0xe1, 0xff) ||
	    spidev_write_reg(nrf->spi, 0xe2, 0xff)) {
		fprintf(stderr, "Failed to reset RF chip\n");
		goto irq_error;
	}

	/* Get nRF24 version
	 * Read RF_SETUP regsiter and find revision:
	 *  - 0x0f for a nRF24L01
	 *  - 0x07 for a nRF24L01+
	 */
	if (spidev_write_reg(nrf->spi, NRF24_REG_RF_SETUP, 0x0f) ||
	    spidev_read_reg(nrf->spi, NRF24_REG_RF_SETUP, &reg)) {
		fprintf(stderr, "Failed to read RF_SETUP register\n");
		goto irq_error;
	}
	nrf->version_p = reg == 0x0e ? 1 : 0;

	return nrf;

version_error:
	if (nrf->irq_fd != -1)
		gpio_value_close(nrf->irq_fd);
irq_error:
	gpio_value_close(nrf->ce_fd);
ce_error:
	spidev_close(nrf->spi);
spi_error:
	free(nrf);
	return NULL;
}

void nrf24_free(struct nrf24 *nrf)
{
	if (!nrf)
		return;

	/* Power down chip */
	spidev_write_reg(nrf->spi, NRF24_REG_CONFIG | 0x20, 0x08);

	/* Release IRQ GPIO */
	if (nrf->irq_fd >= 0) {
		gpio_value_close(nrf->irq_fd);
		gpio_irq_setup(nrf->irq_gpio, GPIO_IRQ_MODE_NONE);
		gpio_unexport(nrf->irq_gpio);
	}

	/* Release CE GPIO */
	gpio_value_write(nrf->ce_fd, 0);
	gpio_value_close(nrf->ce_fd);
	gpio_unexport(nrf->ce_gpio);

	/* Close SPI device */
	if (nrf->spi)
		spidev_close(nrf->spi);

	free(nrf);
}

/* Register access helpers */
static inline int nrf24_read_register(struct nrf24 *nrf, uint8_t address,
				      uint8_t *value)
{
	return spidev_read_reg(nrf->spi, address, value);
}

static inline int nrf24_read_register_n(struct nrf24 *nrf, uint8_t address,
					uint8_t *buffer, size_t len)
{
	return spidev_transfer(nrf->spi, &address, 1, buffer, len);
}

static inline int nrf24_write_register(struct nrf24 *nrf, uint8_t address,
				       uint8_t value)
{
	return spidev_write_reg(nrf->spi, address | 0x20, value);
}

static inline int nrf24_update_register(struct nrf24 *nrf, uint8_t address,
				       uint8_t value, uint8_t mask)
{
	uint8_t val;
	int ret;

	/* Read value */
	ret = spidev_read_reg(nrf->spi, address, &val);
	if (ret)
		return ret;

	/* Write update value */
	return spidev_write_reg(nrf->spi, address | 0x20,
				(val & ~mask) | (value & mask));
}

static inline int nrf24_write_register_n(struct nrf24 *nrf, uint8_t address,
					 const uint8_t *buffer, size_t len)
{
	uint8_t tx[NRF24_SPI_MAX_REGISTER_LENGTH + 1];

	/* Check buffer length */
	if (len > NRF24_SPI_MAX_REGISTER_LENGTH)
		return -1;

	/* Setup TX buffer */
	tx[0] = address | 0x20;
	memcpy(&tx[1], buffer, len);

	return spidev_write(nrf->spi, tx, len + 1);
}

static int nrf24_read_register_full(struct nrf24 *nrf, uint8_t address,
				    uint8_t *value, uint8_t *status)
{
	uint8_t tx[2] = { address, 0xFF };
	uint8_t rx[2];

	/* Read register with status */
	if (spidev_transfer_full(nrf->spi, tx, rx, 2))
		return -1;

	/* Set values */
	if (status)
		*status = rx[0];
	if (value)
		*value = rx[1];

	return 0;
}

static int nrf24_read_register_full_n(struct nrf24 *nrf, uint8_t address,
				      uint8_t *buffer, size_t len,
				      uint8_t *status)
{
	uint8_t tx[NRF24_SPI_MAX_REGISTER_LENGTH + 1];
	uint8_t rx[NRF24_SPI_MAX_REGISTER_LENGTH + 1];

	/* Check buffer length */
	if (len > NRF24_SPI_MAX_REGISTER_LENGTH)
		return -1;

	/* Setup TX buffer */
	tx[0] = address;
	memset(&tx[1], 0xFF, len);

	/* Read register buffer with status */
	if (spidev_transfer_full(nrf->spi, tx, rx, len))
		return -1;

	/* Set values */
	if (status)
		*status = rx[0];
	memcpy(buffer, &rx[1], len);

	return 0;
}

static int nrf24_write_register_full(struct nrf24 *nrf, uint8_t address,
				     uint8_t value, uint8_t *status)
{
	uint8_t tx[2] = { address | 0x20, value };
	uint8_t rx[2];

	/* Write register with status read */
	if (spidev_transfer_full(nrf->spi, tx, rx, 2))
		return -1;

	/* Set status */
	if (status)
		*status = rx[0];

	return 0;
}

static int nrf24_write_register_full_n(struct nrf24 *nrf, uint8_t address,
				       const uint8_t *buffer, size_t len,
				       uint8_t *status)
{
	uint8_t tx[NRF24_SPI_MAX_REGISTER_LENGTH + 1];
	uint8_t rx[NRF24_SPI_MAX_REGISTER_LENGTH + 1];

	/* Check buffer length */
	if (len > NRF24_SPI_MAX_REGISTER_LENGTH)
		return -1;

	/* Setup TX buffer */
	tx[0] = address | 0x20;
	memcpy(&tx[1], buffer, len);

	/* Write register buffer with status read */
	if (spidev_transfer_full(nrf->spi, tx, rx, len))
		return -1;

	/* Set values */
	if (status)
		*status = rx[0];

	return 0;
}

static inline int nrf24_activate(struct nrf24 *nrf)
{
	return spidev_write_reg(nrf->spi, 0x50, 0x73);
}

/* nRF24L01 setup */
int nrf24_set_auto_acknowledge(struct nrf24 *nrf, enum nrf24_rx_aa auto_ack)
{
	return nrf24_write_register(nrf, NRF24_REG_EN_AA, auto_ack & 0x3f);
}

int nrf24_get_auto_acknowledge(struct nrf24 *nrf, enum nrf24_rx_aa *auto_ack)
{
	uint8_t val;
	if (nrf24_read_register(nrf, NRF24_REG_EN_AA, &val))
		return -1;
	*auto_ack = val;
	return 0;
}

int nrf24_enable_auto_acknowledge(struct nrf24 *nrf, int pipe)
{
	/* Check RX pipe */
	if (pipe > 5) {
		fprintf(stderr, "Bad pipe: 0 to 5 only");
		return -1;
	}

	/* Enable auto acknowledgment */
	return nrf24_update_register(nrf, NRF24_REG_EN_AA, 1 << pipe,
				     1 << pipe);
}

int nrf24_disable_auto_acknowledge(struct nrf24 *nrf, int pipe)
{
	uint8_t val;

	/* Check RX pipe */
	if (pipe > 5) {
		fprintf(stderr, "Bad pipe: 0 to 5 only");
		return -1;
	}

	/* Disable auto acknowledgment */
	return nrf24_update_register(nrf, NRF24_REG_EN_AA, 0, 1 << pipe);
}

int nrf24_set_rx_en(struct nrf24 *nrf, enum nrf24_rx_en en_rx)
{
	return nrf24_write_register(nrf, NRF24_REG_EN_RXADDR, en_rx & 0x3f);
}

int nrf24_get_rx_en(struct nrf24 *nrf, enum nrf24_rx_en *en_rx)
{
	uint8_t val;
	if (nrf24_read_register(nrf, NRF24_REG_EN_RXADDR, &val))
		return -1;
	*en_rx = val;
	return 0;
}

int nrf24_enable_rx(struct nrf24 *nrf, int pipe)
{
	uint8_t val;

	/* Check RX pipe */
	if (pipe > 5) {
		fprintf(stderr, "Bad pipe: 0 to 5 only");
		return -1;
	}

	/* Enable pipe */
	return nrf24_update_register(nrf, NRF24_REG_EN_RXADDR, 1 << pipe,
				     1 << pipe);
}

int nrf24_disable_rx(struct nrf24 *nrf, int pipe)
{
	uint8_t val;

	/* Check RX pipe */
	if (pipe > 5) {
		fprintf(stderr, "Bad pipe: 0 to 5 only");
		return -1;
	}

	/* Disable pipe */
	return nrf24_update_register(nrf, NRF24_REG_EN_RXADDR, 0, 1 << pipe);
}

int nrf24_set_auto_retransmission(struct nrf24 *nrf, uint16_t delay_us,
				  uint8_t count)
{
	/* Check parameters */
	if (delay_us > 4000) {
		fprintf(stderr, "Bad auto retransmission delay: 250us to 4ms");
		return -1;
	}
	if (count > 15) {
		fprintf(stderr, "Bad auto retransmission count: 0 to 15");
		return -1;
	}

	/* Setup auto retransmission */
	return nrf24_write_register(nrf, NRF24_REG_SETUP_RETR,
				   (((delay_us - 1) / 250) << 4) |
				   (count & 0x0F));
}

int nrf24_get_auto_retransmission(struct nrf24 *nrf, uint16_t *delay_us,
				  uint8_t *count)
{
	uint8_t val;

	/* Get auto retransmission setup */
	if (nrf24_read_register(nrf, NRF24_REG_SETUP_RETR, &val));
		return -1;
	if (delay_us)
		*delay_us = (((val >> 4) & 0x0F) + 1) * 250;
	if (count)
		*count = val & 0x0F;
	return 0;
}

int nrf24_enable_auto_retransmission(struct nrf24 *nrf, uint8_t count)
{
	uint8_t val;

	/* Enable auto retransmission */
	return nrf24_update_register(nrf, NRF24_REG_SETUP_RETR, count, 0x0F);
}

int nrf24_disable_auto_retransmission(struct nrf24 *nrf)
{
	uint8_t val;

	/* Disable auto retransmission */
	return nrf24_update_register(nrf, NRF24_REG_SETUP_RETR, 0, 0x0F);
}

int nrf24_set_rf_channel(struct nrf24 *nrf, uint8_t channel)
{
	/* Check channel */
	if (channel > 127) {
		fprintf(stderr, "Bad RF channel: 0 to 127 MHz");
		return -1;
	}
	return nrf24_write_register(nrf, NRF24_REG_RF_CH, channel & 0x7F);
}

int nrf24_get_rf_channel(struct nrf24 *nrf, uint8_t *channel)
{
	return nrf24_read_register(nrf, NRF24_REG_RF_CH, channel);
}

int nrf24_set_rf_setup(struct nrf24 *nrf, enum nrf24_rf_bitrate bitrate,
		       enum nrf24_rf_power power, int lna_gain)
{
	/* Check values */
	if (bitrate != NRF24_RF_BITRATE_250K &&
	    bitrate != NRF24_RF_BITRATE_1M && bitrate != NRF24_RF_BITRATE_2M) {
		fprintf(stderr, "Bad RF bitrate: 250kbps, 1Mbps or 2Mbps");
		return -1;
	}
	if (!nrf->version_p && bitrate == NRF24_RF_BITRATE_250K) {
		fprintf(stderr, "Bad RF bitrate: 250kbps is not supported!");
		return -1;
	}
	if (power > 3) {
		fprintf(stderr, "Bad RF power: -18dBm to 0dBm");
		return -1;
	}

	/* Set RF setup */
	return nrf24_write_register(nrf, NRF24_REG_RF_SETUP, bitrate |
				    (power << 1) | (lna_gain & 0x01));
}

int nrf24_get_rf_setup(struct nrf24 *nrf, enum nrf24_rf_bitrate *bitrate,
		       enum nrf24_rf_power *power, int *lna_gain)
{
	uint8_t val;

	/* Set RF setup */
	if (nrf24_read_register(nrf, NRF24_REG_RF_SETUP, &val))
		return -1;
	if (bitrate)
		*bitrate = val & 0x28;
	if (power)
		*power = (val >> 1) & 0x3;
	if (lna_gain)
		*lna_gain = val & 0x01;
	return 0;
}

int nrf24_set_rf_bitrate(struct nrf24 *nrf, enum nrf24_rf_bitrate bitrate)
{
	/* Check values */
	if (bitrate != NRF24_RF_BITRATE_250K &&
	    bitrate != NRF24_RF_BITRATE_1M && bitrate != NRF24_RF_BITRATE_2M) {
		fprintf(stderr, "Bad RF bitrate: 250kbps, 1Mbps or 2Mbps");
		return -1;
	}
	if (!nrf->version_p && bitrate == NRF24_RF_BITRATE_250K) {
		fprintf(stderr, "Bad RF bitrate: 250kbps is not supported!");
		return -1;
	}

	/* Set RF bitrate */
	return nrf24_update_register(nrf, NRF24_REG_RF_SETUP, bitrate, 0x028);
}

int nrf24_set_rf_power(struct nrf24 *nrf, enum nrf24_rf_power power)
{
	/* Check values */
	if (power > 3) {
		fprintf(stderr, "Bad RF power: -18dBm to 0dBm");
		return -1;
	}

	/* Set RF power */
	return nrf24_update_register(nrf, NRF24_REG_RF_SETUP, power << 1, 0x06);
}

int nrf24_set_rf_lna_gain(struct nrf24 *nrf, int lna_gain)
{
	/* Set LNA gain */
	return nrf24_update_register(nrf, NRF24_REG_RF_SETUP, lna_gain, 0x01);
}

int nrf24_set_address_width(struct nrf24 *nrf, uint8_t width)
{
	/* Check address width */
	if (width < 3 || width > 5) {
		fprintf(stderr, "Bad address width: 3 to 5 bytes");
		return -1;
	}

	/* Set address width */
	nrf->address_width = width;
	return nrf24_write_register(nrf, NRF24_REG_SETUP_AW, width - 2);
}

int nrf24_get_address_width(struct nrf24 *nrf, uint8_t *width)
{
	/* Get address width */
	if (nrf24_read_register(nrf, NRF24_REG_SETUP_AW, width))
		return -1;
	*width += 2;
	return 0;
}

int nrf24_set_tx_address(struct nrf24 *nrf, const uint8_t *address)
{
	return nrf24_write_register_n(nrf, NRF24_REG_TX_ADDR, address,
				      nrf->address_width);
}

int nrf24_get_tx_address(struct nrf24 *nrf, uint8_t *address)
{
	return nrf24_read_register_n(nrf, NRF24_REG_TX_ADDR, address,
				     nrf->address_width);
}

int nrf24_set_rx_address(struct nrf24 *nrf, int pipe, const uint8_t *address)
{
	if (pipe > 1)
		return nrf24_write_register(nrf, NRF24_REG_RX_ADDR_P0 + pipe,
					    *address);
	return nrf24_write_register_n(nrf, NRF24_REG_RX_ADDR_P0 + pipe, address,
				      nrf->address_width);
}

int nrf24_get_rx_address(struct nrf24 *nrf, int pipe, uint8_t *address)
{
	return nrf24_read_register_n(nrf, NRF24_REG_RX_ADDR_P0 + pipe, address,
				     nrf->address_width);
}

int nrf24_set_rx_addresses(struct nrf24 *nrf, uint8_t addresses[][5])
{
	int ret = 0, i;

	/* Write all RX addresses */
	for (i = 0; i < 6; i++)
		ret |= nrf24_set_rx_address(nrf, i, addresses[i]);

	return ret;
}

int nrf24_get_rx_addresses(struct nrf24 *nrf, uint8_t addresses[][5])
{
	int ret = 0, i;

	/* Read all RX addresses */
	for (i = 0; i < 6; i++)
		ret |= nrf24_get_rx_address(nrf, i, addresses[i]);

	return ret;
}

int nrf24_set_rx_payload_width(struct nrf24 *nrf, int pipe, uint8_t width)
{
	/* Check payload width */
	if (width > 32) {
		fprintf(stderr, "Bad payload width: 0 to 32 bytes");
		return -1;
	}

	return nrf24_write_register(nrf, NRF24_REG_RX_PW_P0 + pipe,
				    width & 0x3f);
}

int nrf24_get_rx_payload_width(struct nrf24 *nrf, int pipe, uint8_t *width)
{
	return nrf24_read_register(nrf, NRF24_REG_RX_PW_P0 + pipe, width);
}

int nrf24_set_rx_payload_widths(struct nrf24 *nrf, const uint8_t *width)
{
	int ret = 0, i;

	/* Set all payload width */
	for (i = 0; i < 6; i++)
		ret |= nrf24_set_rx_payload_width(nrf, i, width[i]);

	return ret;
}

int nrf24_get_rx_payload_widths(struct nrf24 *nrf, uint8_t *width)
{
	int ret = 0, i;

	/* Fet all payload width */
	for (i = 0; i < 6; i++)
		ret |= nrf24_get_rx_payload_width(nrf, i, &width[i]);

	return ret;
}

static int nrf24_activate_feature(struct nrf24 *nrf)
{
	uint8_t val;

	/* Already activated */
	if (nrf->feature_activated)
		return 0;

	/* Read value of FEATURE */
	if (nrf24_read_register(nrf, NRF24_REG_FEATURE, &val)) {
		fprintf(stderr, "Failed to read FEATURE register");
		return -1;
	}

	/* Feature is already activated */
	if (val)
		goto activated;

	/* Try to write in FEATURE register */
	if (nrf24_write_register(nrf, NRF24_REG_FEATURE, 0x04) ||
	    nrf24_read_register(nrf, NRF24_REG_FEATURE, &val) ||
	    nrf24_write_register(nrf, NRF24_REG_FEATURE, 0x00)) {
		fprintf(stderr, "Failed to check FEATURE register activation");
		return -1;
	}

	/* Feature is already activated */
	if (val == 0x04)
		goto activated;

	/* Activate feature */
	if (nrf24_activate(nrf)) {
		fprintf(stderr, "Failed to activate FEATURE");
		return -1;
	}

activated:
	nrf->feature_activated = 1;
	return 0;
}

int nrf24_enable_dynamic_payload(struct nrf24 *nrf)
{
	/* Activate feature */
	nrf24_activate_feature(nrf);

	return nrf24_update_register(nrf, NRF24_REG_FEATURE, 0x04, 0x04);
}

int nrf24_disable_dynamic_payload(struct nrf24 *nrf)
{
	/* Activate feature */
	nrf24_activate_feature(nrf);

	return nrf24_update_register(nrf, NRF24_REG_FEATURE, 0, 0x04);
}

int nrf24_set_dynamic_payload(struct nrf24 *nrf, enum nrf24_dpl_en dpl_en)
{
	/* Activate feature */
	nrf24_activate_feature(nrf);

	/* Set dynamic payload pipes */
	if (nrf24_write_register(nrf, NRF24_REG_DYNPD, dpl_en))
		return -1;

	/* Update FEATURE register */
	if (dpl_en)
		return nrf24_enable_dynamic_payload(nrf);
	return nrf24_disable_dynamic_payload(nrf);
}

int nrf24_get_dynamic_payload(struct nrf24 *nrf, enum nrf24_dpl_en *dpl_en)
{
	uint8_t val;

	/* Activate feature */
	nrf24_activate_feature(nrf);

	/* Get dynamic payload pipes */
	if (nrf24_read_register(nrf, NRF24_REG_DYNPD, &val))
		return -1;
	*dpl_en = val;
	return 0;
}

int nrf24_set_config(struct nrf24 *nrf, int power_up, int rx_mode, int crc_en,
		     enum nrf24_crc_mode crc_mode, enum nrf24_mask_irq mask_irq)
{
	/* Setup config */
	if (nrf24_write_register(nrf, NRF24_REG_CONFIG, mask_irq |
				 (crc_en ? 0x08 : 0) | crc_mode |
				 (power_up ? 0x02 : 0) | (rx_mode ? 0x01 : 0)))
		return -1;

	/* Save configuration */
	nrf->power_up = power_up;
	nrf->rx_mode = rx_mode;

	/* Wait 1.5ms before nRF24 awake */
	if (power_up)
		usleep(1500);
	return 0;
}

int nrf24_get_config(struct nrf24 *nrf, int *power_up, int *rx_mode,
		     int *crc_en, enum nrf24_crc_mode *crc_mode,
		     enum nrf24_mask_irq *mask_irq)
{
	uint8_t val;

	/* Get configuration */
	if (nrf24_read_register(nrf, NRF24_REG_CONFIG, &val))
		return -1;
	if (power_up)
		*power_up = (val >> 1) & 0x01;
	if (rx_mode)
		*rx_mode = val & 0x01;
	if (crc_en)
		*crc_en = (val >> 3) & 0x01;
	if (crc_mode)
		*crc_mode = val & 0x04;
	if (mask_irq)
		*mask_irq = val & 0x70;
	return 0;
}

int nrf24_power_up(struct nrf24 *nrf, int rx_mode)
{
	/* Power UP */
	if (nrf24_update_register(nrf, NRF24_REG_CONFIG,
				  rx_mode ? 0x03 : 0x02, 0x03))
		return -1;

	/* Save state */
	nrf->power_up = 1;
	nrf->rx_mode = rx_mode;

	/* Wait 1.5ms before nRF24 awake */
	usleep(1500);
	return 0;
}

int nrf24_power_down(struct nrf24 *nrf)
{
	nrf->power_up = 0;
	return nrf24_update_register(nrf, NRF24_REG_CONFIG, 0, 0x02);
}

int nrf24_set_rx_mode(struct nrf24 *nrf)
{
	return nrf24_update_register(nrf, NRF24_REG_CONFIG, 0x01, 0x01);
}

int nrf24_set_tx_mode(struct nrf24 *nrf)
{
	return nrf24_update_register(nrf, NRF24_REG_CONFIG, 0, 0x01);
}

int nrf24_get_mode(struct nrf24 *nrf, int *is_rx)
{
	uint8_t val;

	/* Get RX/TX mode */
	if (nrf24_read_register(nrf, NRF24_REG_CONFIG, &val))
		return -1;
	*is_rx = val & 0x01;
	return 0;
}

int nrf24_set_crc(struct nrf24 *nrf, int crc_en, enum nrf24_crc_mode crc_mode)
{
	return nrf24_update_register(nrf, NRF24_REG_CONFIG,
				     (crc_en ? 0x08 : 0) | crc_mode, 0x0c);
}

int nrf24_enable_crc(struct nrf24 *nrf)
{
	return nrf24_update_register(nrf, NRF24_REG_CONFIG, 0x08, 0x08);
}

int nrf24_disable_crc(struct nrf24 *nrf)
{
	return nrf24_update_register(nrf, NRF24_REG_CONFIG, 0, 0x08);
}

int nrf24_set_irq(struct nrf24 *nrf, enum nrf24_mask_irq mask_irq)
{
	return nrf24_update_register(nrf, NRF24_REG_CONFIG, mask_irq, 0x70);
}

int nrf24_setup_full(struct nrf24 *nrf, struct nrf24_setup *stp)
{
	int ret;

	/* Setup RX pipes */
	ret = nrf24_set_auto_acknowledge(nrf, stp->rx_auto_ack);
	ret |= nrf24_set_rx_en(nrf, stp->rx_enable);

	/* Setup auto retransmission */
	ret |= nrf24_set_auto_retransmission(nrf, stp->auto_retr_delay_us,
					     stp->auto_retr_count);

	/* RF setup */
	ret |= nrf24_set_rf_channel(nrf, stp->rf_channel);
	ret |= nrf24_set_rf_setup(nrf, stp->rf_bitrate, stp->rf_power,
				  stp->rf_lna_gain);

	/* Setup addresses */
	ret |= nrf24_set_address_width(nrf, stp->address_width);
	ret |= nrf24_set_tx_address(nrf, stp->tx_address);
	ret |= nrf24_set_rx_addresses(nrf, stp->rx_addresses);

	/* Setup payload width */
	ret |= nrf24_set_rx_payload_widths(nrf, stp->rx_payload_widths);

	/* Setup dynamic payload */
	ret |= nrf24_set_dynamic_payload(nrf, stp->rx_dynamic_payload);

	/* Setup CRC / RX mode and Power UP / DOWN */
	ret |= nrf24_set_config(nrf, stp->power_up, stp->rx_mode, stp->crc_en,
				 stp->crc_mode, stp->mask_irq);

	return ret;
}

/* Operations */
int nrf24_rx_start(struct nrf24 *nrf)
{
	/* Already started */
	if (nrf->rx_started) {
		fprintf(stderr, "RX mode is already started\n");
		return -1;
	}

	/* Not in RX mode */
	if (!nrf->power_up || !nrf->rx_mode) {
		fprintf(stderr, "nRF24 is not powered on in RX mode\n");
		return -1;
	}

	/* Set CE to high */
	gpio_value_write(nrf->ce_fd, 1);
	usleep(130);
	nrf->rx_started = 1;
	return 0;
}

int nrf24_rx_stop(struct nrf24 *nrf)
{
	/* RX is not started */
	if (!nrf->power_up || !nrf->rx_mode || !nrf->rx_started) {
		fprintf(stderr, "RX mode is not started\n");
		return -1;
	}

	/* Set CE to low */
	nrf->rx_started = 0;
	gpio_value_write(nrf->ce_fd, 0);
	return 0;
}

int nrf24_get_status(struct nrf24 *nrf, uint8_t *status)
{
	return nrf24_read_register(nrf, NRF24_REG_STATUS, status);
}

int nrf24_clear_irq(struct nrf24 *nrf, uint8_t irq)
{
	return nrf24_write_register(nrf, NRF24_REG_STATUS, irq & 0x70);
}

int nrf24_get_fifo_status(struct nrf24 *nrf, uint8_t *status)
{
	return nrf24_read_register(nrf, NRF24_REG_FIFO_STATUS, status);
}

int nrf24_rx_read(struct nrf24 *nrf, uint8_t *buffer, size_t size)
{
	uint8_t cmd = 0x61;
	int ret;

	/* Chack read size */
	if (size < 1 || size > 32) {
		fprintf(stderr, "Payload must be 1 to 32 bytes length\n");
		return -1;
	}

	/* Read payload from RX FIFO */
	ret = spidev_transfer(nrf->spi, &cmd, 1, buffer, size);

	/* Clear STATUS and IRQ */
	ret |= nrf24_write_register(nrf, NRF24_REG_STATUS, 0x40);

	return ret;
}

int nrf24_tx_write(struct nrf24 *nrf, uint8_t *buffer, size_t size,
		   int transmit)
{
	uint8_t tx[NRF24_SPI_MAX_PAYLOAD_LENGTH + 1];
	int ret;

	/* Chack write size */
	if (size < 1 || size > 32) {
		fprintf(stderr, "Payload must be 1 to 32 bytes length\n");
		return -1;
	}

	/* Setup command */
	tx[0] = 0xA0;
	memcpy(&tx[1], buffer, size);

	/* Add payload to TX FIFO */
	ret = spidev_write(nrf->spi, tx, size + 1);
	if (ret || !transmit)
		return ret;

	/* Send payload immediately */
	gpio_value_write(nrf->ce_fd, 1);
	usleep(100);
	gpio_value_write(nrf->ce_fd, 0);

	return 0;
}

int nrf24_get_dynamic_payload_size(struct nrf24 *nrf, uint8_t *size)
{
	uint8_t cmd = 0x60;

	return spidev_transfer(nrf->spi, &cmd, 1, size, 1);
}

int nrf24_tx_flush(struct nrf24 *nrf)
{
	return spidev_write_reg(nrf->spi, 0xe1, 0xff);
}

int nrf24_rx_flush(struct nrf24 *nrf)
{
	return spidev_write_reg(nrf->spi, 0xe2, 0xff);
}

/* Wait operations */
int nrf24_rx_wait(struct nrf24 *nrf, long timeout_ms, uint8_t *pipe)
{
	uint8_t status;
	int ret;

	/* Check if FIFO is empty */
	ret = nrf24_get_fifo_status(nrf, &status);
	if (ret)
		return ret;
	if (!(status & 0x01)) {
		/* Get RX pipe */
		if (pipe) {
			ret = nrf24_get_status(nrf, &status);
			if (ret)
				return ret;
			*pipe = (status >> 1) & 0x07;
		}
		return 0;
	}

	/* Wait for next RX payload */
	if (nrf->irq_fd < 0) {
		while (timeout_ms < 0 || timeout_ms--) {
			/* Read status register */
			ret = nrf24_get_status(nrf, &status);
			if (ret)
				return ret;

			/* New data available */
			if (status & NRF24_MASK_IRQ_RX_DR)
				break;

			/* Wait 1ms before next check */
			if (usleep(1000) == EINTR)
				return NRF24_WAIT_INTERRUPT;
		}

	} else {
		/* Wait for IRQ */
		ret = gpio_irq_wait(nrf->irq_fd, NULL, timeout_ms);
		if (ret) {
			if (ret == GPIO_IRQ_TIMEOUT)
				return NRF24_WAIT_TIMEOUT;
			if (ret == EINTR)
				return NRF24_WAIT_INTERRUPT;
			return ret;
		}

		/* Read status register */
		ret = nrf24_get_status(nrf, &status);
		if (ret)
			return ret;
	}

	/* Set pipe */
	if (pipe)
		*pipe = (status >> 1) & 0x07;
	return 0;
}

int nrf24_tx_wait(struct nrf24 *nrf, long timeout_ms)
{
	uint8_t status;
	int ret;

	/* Wait for end of transmission or max retransmit */
	if (nrf->irq_fd < 0) {
		while (timeout_ms < 0 || timeout_ms--) {
			/* Read status register */
			ret = nrf24_get_status(nrf, &status);
			if (ret)
				return ret;

			/* Check TX status */
			if (status &
			    NRF24_MASK_IRQ_MAX_RT | NRF24_MASK_IRQ_TX_DS)
				break;

			/* Wait 1ms before next check */
			if (usleep(1000) == EINTR)
				return NRF24_WAIT_INTERRUPT;
		}
	} else {
		/* Wait for IRQ */
		ret = gpio_irq_wait(nrf->irq_fd, NULL, timeout_ms);
		if (ret) {
			if (ret == GPIO_IRQ_TIMEOUT)
				return NRF24_WAIT_TIMEOUT;
			if (ret == EINTR)
				return NRF24_WAIT_INTERRUPT;
			return ret;
		}

		/* Read status register */
		ret = nrf24_get_status(nrf, &status);
		if (ret)
			return ret;
	}

	/* Check TX status */
	return status & NRF24_MASK_IRQ_MAX_RT ? NRF24_WAIT_MAX_RETRY : 0;
}

/* Debug functions */
void nrf24_print_version(struct nrf24 *nrf)
{
	fprintf(stderr, " + nRF24 version: nRF24L01%s\n",
		nrf->version_p ? "+" : "");
}

void nrf24_print_setup(struct nrf24 *nrf)
{
	uint8_t rx[2][5];
	uint8_t tx[5];
	uint8_t val;

	/* Read addresses */
	nrf24_get_rx_address(nrf, 0, rx[0]);
	nrf24_get_rx_address(nrf, 1, rx[1]);
	nrf24_get_tx_address(nrf, tx);

	/*Print nRF24L01 configuration */
	fprintf(stderr, " + nRF24L01 setup:\n"
		"     - CONFIG: 0x%02x\n"
		"     - EN_AA: 0x%02x\n"
		"     - EN_RXADDR: 0x%02x\n"
		"     - SETUP_AW: 0x%02x\n"
		"     - SETUP_RETR: 0x%02x\n"
		"     - RF_CH: 0x%02x\n"
		"     - RF_SETUP: 0x%02x\n"
		"     - STATUS: 0x%02x\n"
		"     - RX_ADDR_P0-1: 0x%02x%02x%02x%02x%02x "
			"0x%02x%02x%02x%02x%02x\n"
		"     - RX_ADDR_P2-5: 0x%02x 0x%02x 0x%02x 0x%02x\n"
		"     - TX_ADDR: 0x%02x%02x%02x%02x%02x\n"
		"     - RX_PW_P0-5: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n"
		"     - FIFO_STATUS: 0x%02x\n"
		"     - DYNPD: 0x%02x\n"
		"     - FEATURE: 0x%02x\n",
		nrf24_read_register(nrf, NRF24_REG_CONFIG, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_EN_AA, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_EN_RXADDR, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_SETUP_AW, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_SETUP_RETR, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RF_CH, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RF_SETUP, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_STATUS, &val) ? 0 : val,
		rx[0][0], rx[0][1], rx[0][2], rx[0][3], rx[0][4],
		rx[1][0], rx[1][1], rx[1][2], rx[1][3], rx[1][4],
		nrf24_read_register(nrf, NRF24_REG_RX_ADDR_P2, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_ADDR_P3, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_ADDR_P4, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_ADDR_P5, &val) ? 0 : val,
		tx[0], tx[1], tx[2], tx[3], tx[4],
		nrf24_read_register(nrf, NRF24_REG_RX_PW_P0, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_PW_P1, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_PW_P2, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_PW_P3, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_PW_P4, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_RX_PW_P5, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_FIFO_STATUS, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_DYNPD, &val) ? 0 : val,
		nrf24_read_register(nrf, NRF24_REG_FEATURE, &val) ? 0 : val);
}
