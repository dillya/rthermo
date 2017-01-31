/*
 * nrf24.h: nRF24L01(+) handler for RF communication
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

#ifndef _NRF24_H_
#define _NRF24_H_

/* nRF24L01 configuration */
struct nrf24_config {
	char *spi_dev;
	int irq_gpio;
	int ce_gpio;
};

/* CRC mode */
enum nrf24_crc_mode {
	NRF24_CRC_MODE_1BYTE = 0x00,
	NRF24_CRC_MODE_2BYTES = 0x04
};

/* Mask interrupt */
enum nrf24_mask_irq {
	NRF24_MASK_IRQ_MAX_RT = 0x10,
	NRF24_MASK_IRQ_TX_DS = 0x20,
	NRF24_MASK_IRQ_RX_DR = 0x40
};

/* Auto Acknowledgment on RX pipes */
enum nrf24_rx_aa {
	NRF24_RX_AUTO_ACK_P0 = (1 << 0),
	NRF24_RX_AUTO_ACK_P1 = (1 << 1),
	NRF24_RX_AUTO_ACK_P2 = (1 << 2),
	NRF24_RX_AUTO_ACK_P3 = (1 << 3),
	NRF24_RX_AUTO_ACK_P4 = (1 << 4),
	NRF24_RX_AUTO_ACK_P5 = (1 << 5),
	NRF24_RX_AUTO_ACK_ALL = 0x3F
};

/* Enable RX pipes */
enum nrf24_rx_en {
	NRF24_RX_ENABLE_P0 = (1 << 0),
	NRF24_RX_ENABLE_P1 = (1 << 1),
	NRF24_RX_ENABLE_P2 = (1 << 2),
	NRF24_RX_ENABLE_P3 = (1 << 3),
	NRF24_RX_ENABLE_P4 = (1 << 4),
	NRF24_RX_ENABLE_P5 = (1 << 5),
	NRF24_RX_ENABLE_ALL = 0x3F
};

/* RF bitrate */
enum nrf24_rf_bitrate {
	NRF24_RF_BITRATE_250K = 0x20,
	NRF24_RF_BITRATE_1M = 0x00,
	NRF24_RF_BITRATE_2M = 0x08
};

/* RF power (in -dBm) */
enum nrf24_rf_power {
	NRF24_RF_POWER_18_DBM = 0,
	NRF24_RF_POWER_12_DBM = 1,
	NRF24_RF_POWER_6_DBM = 2,
	NRF24_RF_POWER_0_DBM = 3
};

/* Enable RX dynamic payload pipes */
enum nrf24_dpl_en {
	NRF24_DPL_ENABLE_P0 = (1 << 0),
	NRF24_DPL_ENABLE_P1 = (1 << 1),
	NRF24_DPL_ENABLE_P2 = (1 << 2),
	NRF24_DPL_ENABLE_P3 = (1 << 3),
	NRF24_DPL_ENABLE_P4 = (1 << 4),
	NRF24_DPL_ENABLE_P5 = (1 << 5),
	NRF24_DPL_ENABLE_ALL = 0x3F
};

struct nrf24_setup {
	/* General configuration */
	int power_up;
	int rx_mode;
	int crc_en;
	enum nrf24_crc_mode crc_mode;
	enum nrf24_mask_irq mask_irq;
	/* RX configuration */
	enum nrf24_rx_aa rx_auto_ack;
	enum nrf24_rx_en rx_enable;
	/* Auto retransmit configuration */
	uint16_t auto_retr_delay_us;
	uint8_t auto_retr_count;
	/* RF configuration */
	uint8_t rf_channel;
	enum nrf24_rf_bitrate rf_bitrate;
	enum nrf24_rf_power rf_power;
	uint8_t rf_lna_gain;
	/* Address configuration */
	uint8_t address_width;
	uint8_t rx_addresses[6][5];
	uint8_t tx_address[5];
	/* Payload width configuration */
	uint8_t rx_payload_widths[6];
	enum nrf24_dpl_en rx_dynamic_payload;
};

/* nrf24 handler */
struct nrf24;

/* Constructor / Desctructor */
struct nrf24 *nrf24_new(struct nrf24_config *cfg);
void nrf24_free(struct nrf24 *nrf);

/* nRF24L01 setup */
int nrf24_set_auto_acknowledge(struct nrf24 *nrf, enum nrf24_rx_aa auto_ack);
int nrf24_get_auto_acknowledge(struct nrf24 *nrf, enum nrf24_rx_aa *auto_ack);
int nrf24_enable_auto_acknowledge(struct nrf24 *nrf, int pipe);
int nrf24_disable_auto_acknowledge(struct nrf24 *nrf, int pipe);

int nrf24_set_rx_en(struct nrf24 *nrf, enum nrf24_rx_en en_rx);
int nrf24_get_rx_en(struct nrf24 *nrf, enum nrf24_rx_en *en_rx);
int nrf24_enable_rx(struct nrf24 *nrf, int pipe);
int nrf24_disable_rx(struct nrf24 *nrf, int pipe);

int nrf24_set_auto_retransmission(struct nrf24 *nrf, uint16_t delay_us,
				  uint8_t count);
int nrf24_get_auto_retransmission(struct nrf24 *nrf, uint16_t *delay_us,
				  uint8_t *count);
int nrf24_enable_auto_retransmission(struct nrf24 *nrf, uint8_t count);
int nrf24_disable_auto_retransmission(struct nrf24 *nrf);

int nrf24_set_rf_channel(struct nrf24 *nrf, uint8_t channel);
int nrf24_get_rf_channel(struct nrf24 *nrf, uint8_t *channel);

int nrf24_set_rf_setup(struct nrf24 *nrf, enum nrf24_rf_bitrate bitrate,
		       enum nrf24_rf_power power, int lna_gain);
int nrf24_get_rf_setup(struct nrf24 *nrf, enum nrf24_rf_bitrate *bitrate,
		       enum nrf24_rf_power *power, int *lna_gain);
int nrf24_set_rf_bitrate(struct nrf24 *nrf, enum nrf24_rf_bitrate bitrate);
#define nrf24_get_rf_bitrate(nrf,bitrate) \
	nrf24_get_rf_setup(nrf, bitrate, NULL, NULL)
int nrf24_set_rf_power(struct nrf24 *nrf, enum nrf24_rf_power power);
#define nrf24_get_rf_power(nrf,power) \
	nrf24_get_rf_setup(nrf, NULL, power, NULL)
int nrf24_set_rf_lna_gain(struct nrf24 *nrf, int lna_gain);
#define nrf24_get_rf_lna_gain(nrf,lna_gain) \
	nrf24_get_rf_setup(nrf, NULL, NULL, lna_gain)
#define nrf24_enable_rf_lna_gain(nrf) nrf24_set_lna_gain(nrf, 1)
#define nrf24_disable_rf_lna_gain(nrf) nrf24_set_lna_gain(nrf, 0)

int nrf24_set_address_width(struct nrf24 *nrf, uint8_t width);
int nrf24_get_address_width(struct nrf24 *nrf, uint8_t *width);

int nrf24_set_tx_address(struct nrf24 *nrf, const uint8_t *address);
int nrf24_get_tx_address(struct nrf24 *nrf, uint8_t *address);
int nrf24_set_rx_address(struct nrf24 *nrf, int pipe, const uint8_t *address);
int nrf24_get_rx_address(struct nrf24 *nrf, int pipe, uint8_t *address);
int nrf24_set_rx_addresses(struct nrf24 *nrf, uint8_t addresses[][5]);
int nrf24_get_rx_addresses(struct nrf24 *nrf, uint8_t addresses[][5]);

int nrf24_set_rx_payload_width(struct nrf24 *nrf, int pipe, uint8_t width);
int nrf24_get_rx_payload_width(struct nrf24 *nrf, int pipe, uint8_t *width);
int nrf24_set_rx_payload_widths(struct nrf24 *nrf, const uint8_t *width);
int nrf24_get_rx_payload_widths(struct nrf24 *nrf, uint8_t *width);

int nrf24_enable_dynamic_payload(struct nrf24 *nrf);
int nrf24_disable_dynamic_payload(struct nrf24 *nrf);
int nrf24_set_dynamic_payload(struct nrf24 *nrf, enum nrf24_dpl_en dpl_en);
int nrf24_get_dynamic_payload(struct nrf24 *nrf, enum nrf24_dpl_en *dpl_en);

int nrf24_set_config(struct nrf24 *nrf, int power_up, int rx_mode, int crc_en,
		     enum nrf24_crc_mode crc_mode,
		     enum nrf24_mask_irq mask_irq);
int nrf24_get_config(struct nrf24 *nrf, int *power_up, int *rx_mode,
		     int *crc_en, enum nrf24_crc_mode *crc_mode,
		     enum nrf24_mask_irq *mask_irq);

int nrf24_power_up(struct nrf24 *nrf, int rx_mode);
int nrf24_power_down(struct nrf24 *nrf);
#define nrf24_get_power(nrf,power_up,rx_mode) \
	nrf24_get_config(nrf, power_up, rx_mode, NULL, NULL, NULL)

int nrf24_set_rx_mode(struct nrf24 *nrf);
int nrf24_set_tx_mode(struct nrf24 *nrf);
int nrf24_get_mode(struct nrf24 *nrf, int *is_rx);

int nrf24_set_crc(struct nrf24 *nrf, int crc_en, enum nrf24_crc_mode crc_mode);
#define nrf24_get_crc(nrf,crc_en,crc_mode) \
	nrf24_get_config(nrf, NULL, NULL, crc_en, crc_mode NULL)
int nrf24_enable_crc(struct nrf24 *nrf);
int nrf24_disable_crc(struct nrf24 *nrf);

int nrf24_set_irq(struct nrf24 *nrf, enum nrf24_mask_irq mask_irq);
#define nrf24_get_irq(nrf,mask) \
	nrf24_get_config(nrf, NULL, NULL, NULL, NULL, mask)

int nrf24_setup_full(struct nrf24 *nrf, struct nrf24_setup *stp);

/* Operations */
int nrf24_rx_start(struct nrf24 *nrf);
int nrf24_rx_stop(struct nrf24 *nrf);

int nrf24_get_status(struct nrf24 *nrf, uint8_t *status);
int nrf24_clear_irq(struct nrf24 *nrf, uint8_t irq);

int nrf24_get_fifo_status(struct nrf24 *nrf, uint8_t *status);

int nrf24_rx_read(struct nrf24 *nrf, uint8_t *buffer, size_t size);
int nrf24_tx_write(struct nrf24 *nrf, uint8_t *buffer, size_t size,
		   int transmit);
int nrf24_get_dynamic_payload_size(struct nrf24 *nrf, uint8_t *size);

int nrf24_tx_flush(struct nrf24 *nrf);
int nrf24_rx_flush(struct nrf24 *nrf);

/* Wait operations */
#define NRF24_WAIT_TIMEOUT	1
#define NRF24_WAIT_INTERRUPT	2
#define NRF24_WAIT_MAX_RETRY	3
int nrf24_rx_wait(struct nrf24 *nrf, long timeout_ms, uint8_t *pipe);
int nrf24_tx_wait(struct nrf24 *nrf, long timeout_ms);

/* Debug */
void nrf24_print_version(struct nrf24 *nrf);
void nrf24_print_setup(struct nrf24 *nrf);

/* nRF24L01 registers */
#define NRF24_REG_CONFIG	0x00
#define NRF24_REG_EN_AA		0x01
#define NRF24_REG_EN_RXADDR	0x02
#define NRF24_REG_SETUP_AW	0x03
#define NRF24_REG_SETUP_RETR	0x04
#define NRF24_REG_RF_CH		0x05
#define NRF24_REG_RF_SETUP	0x06
#define NRF24_REG_STATUS	0x07
#define NRF24_REG_OBSERVE_TX	0x08
#define NRF24_REG_CD		0x09
#define NRF24_REG_RX_ADDR_P0	0x0A
#define NRF24_REG_RX_ADDR_P1	0x0B
#define NRF24_REG_RX_ADDR_P2	0x0C
#define NRF24_REG_RX_ADDR_P3	0x0D
#define NRF24_REG_RX_ADDR_P4	0x0E
#define NRF24_REG_RX_ADDR_P5	0x0F
#define NRF24_REG_TX_ADDR	0x10
#define NRF24_REG_RX_PW_P0	0x11
#define NRF24_REG_RX_PW_P1	0x12
#define NRF24_REG_RX_PW_P2	0x13
#define NRF24_REG_RX_PW_P3	0x14
#define NRF24_REG_RX_PW_P4	0x15
#define NRF24_REG_RX_PW_P5	0x16
#define NRF24_REG_FIFO_STATUS	0x17
#define NRF24_REG_DYNPD		0x1C
#define NRF24_REG_FEATURE	0x1D

#endif /* _NRF24_H_ */
