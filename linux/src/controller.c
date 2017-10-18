/*
 * controller.c: RThermo main controller implementation
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
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <sys/time.h>

#include "controller.h"

struct remote_device {
	/* Device status */
	int used;
	int status;
	/* Device settings */
	char *name;
	uint8_t address[RTHERMO_ADDRESS_WIDTH];
	uint8_t serial[RTHERMO_ADDRESS_WIDTH];
	uint32_t caps;
	int enabled;
	double target;
	int control;
	int local;
};

struct controller {
	/* Thread handler */
	pthread_t thread;
	pthread_mutex_t mutex;
	int stop_thread;
	/* nRF24l01(+) module handler */
	struct nrf24 *nrf;
	struct nrf24_setup nrf_setup;
	/* Main controller / remote device settings */
	struct db *db;
	/* Main controller settings */
	char *name;
	uint8_t address[RTHERMO_ADDRESS_WIDTH];
	uint8_t serial[RTHERMO_ADDRESS_WIDTH];
	uint32_t caps;
	double target;
	/* Remote devices list */
	struct remote_device devs[RTHERMO_MAX_DEVICES];
	enum nrf24_rx_en devs_rx_en;
	unsigned long devs_count;
	int pairing;
	/* Debug / verbosity */
	int verbose;
};

/* Default nRF24L01(+) setup for RThermo */
static struct nrf24_setup default_nrf_setup = {
	/* General configuration */
	.power_up = 1,
	.rx_mode = 1,
#if RTHERMO_CRC_WIDTH == 0
	.crc_en = 0,
#else
	.crc_en = 1,
#endif
#if RTHERMO_CRC_WIDTH == 2
	.crc_mode = NRF24_CRC_MODE_2BYTES,
#else
	.crc_mode = NRF24_CRC_MODE_1BYTE,
#endif
	.mask_irq = 0,
	/* RX configuration */
#if RTHERMO_AUTO_ACK == 1
	.rx_auto_ack = NRF24_RX_AUTO_ACK_ALL,
#else
	.rx_auto_ack = 0,
#endif
	.rx_enable = NRF24_RX_ENABLE_P0,
	/* Auto retransmit configuration */
	.auto_retr_delay_us = RTHERMO_RETRY_DELAY,
	.auto_retr_count = RTHERMO_RETRY_COUNT,
	/* RF configuration */
	.rf_channel = RTHERMO_RF_CHANNEL,
#if RTHERMO_RF_BITRATE == 250
	.rf_bitrate = NRF24_RF_BITRATE_250K,
#elif RTHERMO_RF_BITRATE == 1000
	.rf_bitrate = NRF24_RF_BITRATE_1M,
#else
	.rf_bitrate = NRF24_RF_BITRATE_2M,
#endif
#if RTHERMO_RF_POWER == -18
	.rf_power = NRF24_RF_POWER_18_DBM,
#elif RTHERMO_RF_POWER == -12
	.rf_power = NRF24_RF_POWER_12_DBM,
#elif RTHERMO_RF_POWER == -6
	.rf_power = NRF24_RF_POWER_6_DBM,
#else
	.rf_power = NRF24_RF_POWER_0_DBM,
#endif
	.rf_lna_gain = 1,
	/* Address configuration */
	.address_width = RTHERMO_ADDRESS_WIDTH,
	/* Payload width configuration */
	.rx_payload_widths = {
		RTHERMO_PAYLOAD_WIDTH,
		RTHERMO_PAYLOAD_WIDTH,
		RTHERMO_PAYLOAD_WIDTH,
		RTHERMO_PAYLOAD_WIDTH,
		RTHERMO_PAYLOAD_WIDTH,
		RTHERMO_PAYLOAD_WIDTH
	},
#if RTHERMO_EN_DPL == 1
	.rx_dynamic_payload = NRF24_DPL_ENABLE_ALL,
#else
	.rx_dynamic_payload = 0,
#endif
};

static void print_array(const char *name, const uint8_t *array, size_t len);
#define print_address(name,address) \
	print_array(name, address, RTHERMO_ADDRESS_WIDTH)
#define print_serial(name,serial) \
	print_array(name, serial, RTHERMO_SERIAL_WIDTH)

/* Create a new main controller instance */
struct controller *controller_new(struct nrf24 *nrf, struct db *db, int verbose)
{
	struct controller *ctrl;
	int i;

	/* Allocate private structure */
	ctrl = malloc(sizeof(*ctrl));
	if (!ctrl)
		return NULL;

	/* Initialize handle */
	pthread_mutex_init(&ctrl->mutex, 0);
	ctrl->stop_thread = 1;
	ctrl->db = db;
	ctrl->nrf = nrf;
	ctrl->nrf_setup = default_nrf_setup;
	ctrl->verbose = verbose;
	memset(ctrl->devs, 0, sizeof(ctrl->devs));
	ctrl->devs_count = 0;
	ctrl->pairing = 0;

	/* Load settings from database */
	if (db_get_main_config(db, &ctrl->name, ctrl->address, ctrl->serial,
			       &ctrl->caps, &ctrl->target)) {
		fprintf(stderr, "Failed to load main controller settings\n");
		goto error;
	}

	/* Load remote devices configuration */
	for (i = 0; i < RTHERMO_MAX_DEVICES; i++) {
		struct remote_device *dev = &ctrl->devs[i];
		int ret;

		/* Load device configuration */
		if (db_get_config(ctrl->db, i + 1, &dev->name, dev->address,
				  dev->serial, &dev->caps, &dev->enabled,
				  &dev->target, &dev->control, &dev->local))
			continue;

		/* Set device as used */
		dev->used = 1;
		ctrl->devs_count++;
	}

	return ctrl;
error:
	pthread_mutex_destroy(&ctrl->mutex);
	free(ctrl);
	return NULL;
}

void controller_free(struct controller *ctrl)
{
	if (!ctrl)
		return;

	/* Stop thread first */
	if (!ctrl->stop_thread)
		controller_stop(ctrl);

	/* Free data */
	if (ctrl->name)
		free(ctrl->name);
	pthread_mutex_destroy(&ctrl->mutex);
	free(ctrl);
}

/* Start / stop main controller */
static void *controller_thread(void *user_data);
int controller_start(struct controller *ctrl)
{
	int ret;

	/* Thread is already started */
	if (!ctrl->stop_thread)
		return -1;

	/* Create a new thread */
	ctrl->stop_thread = 0;
	ret = pthread_create(&ctrl->thread, 0, controller_thread, ctrl);
	if (ret) {
		fprintf(stderr, "Failed to create main controller thread\n");
		ctrl->stop_thread = 1;
		return ret;
	}

	return 0;
}

int controller_stop(struct controller *ctrl)
{
	/* Thread is already stopped */
	if (ctrl->stop_thread)
		return -1;

	/* Stop thread and wait it returns */
	ctrl->stop_thread = 1;
	pthread_kill(ctrl->thread, SIGUSR1);
	return pthread_join(ctrl->thread, NULL);
}

/* Target temperature control */
int controller_set_target(struct controller *ctrl, double target)
{
	/* Update target temperature */
	pthread_mutex_lock(&ctrl->mutex);
	ctrl->target = target;
	pthread_mutex_unlock(&ctrl->mutex);

	return 0;
}

double controller_get_target(struct controller *ctrl)
{
	double target;

	/* Get current target temperature */
	pthread_mutex_lock(&ctrl->mutex);
	target = ctrl->target;
	pthread_mutex_unlock(&ctrl->mutex);

	return target;
}

/* Pairing mode control */
int controller_enable_pairing_mode(struct controller *ctrl)
{
	if (ctrl->pairing || ctrl->devs_count == RTHERMO_MAX_DEVICES)
		return -1;

	/* Enable pairing mode */
	ctrl->pairing = 1;
	pthread_kill(ctrl->thread, SIGUSR1);

	return 0;
}

int controller_disable_pairing_mode(struct controller *ctrl)
{
	if (!ctrl->pairing || !ctrl->devs_count)
		return -1;

	/* Disable pairing mode */
	ctrl->pairing = 0;
	pthread_kill(ctrl->thread, SIGUSR1);

	return 0;
}

int controller_is_pairing_mode(struct controller *ctrl)
{
	return ctrl->pairing;
}

/* Device handling */
static int controller_find_device(struct controller *ctrl, uint8_t *address,
				  uint8_t *serial)
{
	int i;

	/* Find next unused device */
	for (i = 0; i < RTHERMO_MAX_DEVICES; i++) {
		struct remote_device *dev = &ctrl->devs[i];
		if (!memcmp(dev->address, address, RTHERMO_ADDRESS_WIDTH) &&
		    !memcmp(dev->serial, serial, RTHERMO_SERIAL_WIDTH))
			return i + 1;
	}

	return -1;
}

static int controller_find_next_free_device(struct controller *ctrl)
{
	int i;

	/* Find next unused device */
	for (i = 0; i < RTHERMO_MAX_DEVICES; i++) {
		if (!ctrl->devs[i].used)
			return i + 1;
	}

	return -1;
}

static int controller_update_device(struct controller *ctrl, int dev,
				    const char *name, const uint8_t *address,
				    const uint8_t *serial, const uint32_t *caps,
				    const int *enabled, const double *target,
				    const int *control, const int *local)
{
	struct remote_device *d;

	/* Check device */
	if (dev <= 0 || dev > RTHERMO_MAX_DEVICES) {
		fprintf(stderr, "Failed to update device %d (bad idx)\n", dev);
		return -1;
	}

	/* Update structure */
	d = &ctrl->devs[dev - 1];
	if (d->name)
		free(d->name);
	d->name = name ? strdup(name) : NULL;
	if (address)
		memcpy(d->address, address, RTHERMO_ADDRESS_WIDTH);
	if (serial)
		memcpy(d->serial, serial, RTHERMO_SERIAL_WIDTH);
	if (caps)
		d->caps = *caps;
	if (enabled)
		d->enabled = *enabled;
	if (target)
		d->target = *target;
	if (control)
		d->control = *control;
	if (local)
		d->local = *local;
	d->used = 1;

	/* Update device configuration in database */
	return db_update_config(ctrl->db, dev, name, address, serial, caps,
				enabled, target, control, local);
}

static int controller_add_device(struct controller *ctrl, int dev,
				 const uint8_t *address, const uint8_t *serial,
				 const uint32_t caps)
{
	int control = caps & RTHERMO_DEVICE_CAPS_HEATER_CONTROL ? 1 : 0;
	int local = caps & RTHERMO_DEVICE_CAPS_LOCAL_TEMPERATURE ? 1 : 0;
	double target = 19.0;
	int enabled = 1;

	return controller_update_device(ctrl, dev, "No name", address, serial,
					&caps, &enabled, &target, &control,
					&local);
}

static int controller_remove_device(struct controller *ctrl, int dev)
{
	return 0;
}

/* Thread entry / main loop */
static int controller_set_normal_mode_rx(struct controller *ctrl)
{
	int ret;

	/* Switch to RX mode:
	 *  - restore RX address 0 with device address to listen
	 *  - enable RX addresses for status requests
	 *  - set RX mode
	 *  - start listening requests on RX
	 */
	ret = nrf24_clear_irq(ctrl->nrf, 0xFF);
	ret |= nrf24_set_rx_address(ctrl->nrf, 0, ctrl->devs[0].address);
	ret |= nrf24_set_rx_en(ctrl->nrf, ctrl->devs_rx_en);
	ret |= nrf24_set_rx_mode(ctrl->nrf);
	ret |= nrf24_rx_start(ctrl->nrf);
	if (ret) {
		fprintf(stderr, "Failed to switch in RX mode");
		return ret;
	}

	return 0;
}

static int controller_set_normal_mode_tx(struct controller *ctrl)
{
	int ret;

	/* Swicth to TX mode:
	 *  - stop RX mode
	 *  - set TX address into RX address 0 for auto acknowledgment
	 *  - set TX mode
	 *  - enable RX address 0 to receive status response acknowledge
	 */
	ret = nrf24_rx_stop(ctrl->nrf);
	ret |= nrf24_set_rx_address(ctrl->nrf, 0, ctrl->address);
	ret |= nrf24_set_tx_mode(ctrl->nrf);
	ret |= nrf24_set_rx_en(ctrl->nrf, NRF24_RX_ENABLE_P0);
	if (ret) {
		fprintf(stderr, "Failed to switch in TX mode");
		return ret;
	}

	return 0;
}

static int controller_set_normal_mode(struct controller *ctrl, int rx_mode)
{
	int ret, i;

	/* Load main controller address */
	ret = nrf24_set_tx_address(ctrl->nrf, ctrl->address);
	if (ret) {
		fprintf(stderr, "Failed to set main controller address\n");
		return ret;
	}

	/* Load remote devices addresses */
	ctrl->devs_rx_en = 0;
	for (i = 0; i < RTHERMO_MAX_DEVICES; i++) {
		/* Disable is disabled */
		if (!ctrl->devs[i].used)
			continue;

		/* Set RX address */
		ret = nrf24_set_rx_address(ctrl->nrf, i, ctrl->devs[i].address);
		if (ret) {
			fprintf(stderr, "Failed to set address %d\n", i);
			return ret;
		}

		/* Enable RX pipe for device */
		if (ctrl->devs[i].enabled)
			ctrl->devs_rx_en |= (1 << i);
	}

	/* Switch to RX / TX mode */
	if (rx_mode)
		return controller_set_normal_mode_rx(ctrl);
	return controller_set_normal_mode_tx(ctrl);
}

static int controller_set_pairing_mode_rx(struct controller *ctrl)
{
	int ret;

	/* Switch to RX mode:
	 *  - enable RX address 1 to receive pairing requests
	 *  - set RX mode
	 *  - start listening requests on RX
	 */
	ret = nrf24_clear_irq(ctrl->nrf, 0xFF);
	ret |= nrf24_set_rx_en(ctrl->nrf, NRF24_RX_ENABLE_P1);
	ret |= nrf24_set_rx_mode(ctrl->nrf);
	ret |= nrf24_rx_start(ctrl->nrf);
	if (ret) {
		fprintf(stderr, "Failed to switch in RX pairing mode");
		return ret;
	}

	return 0;
}

static int controller_set_pairing_mode_tx(struct controller *ctrl)
{
	int ret;

	/* Swicth to TX mode:
	 *  - stop RX mode
	 *  - set TX mode
	 *  - enable RX address 0 to receive pairing response acknowledge
	 */
	ret = nrf24_rx_stop(ctrl->nrf);
	ret |= nrf24_set_tx_mode(ctrl->nrf);
	ret |= nrf24_set_rx_en(ctrl->nrf, NRF24_RX_ENABLE_P0);
	if (ret) {
		fprintf(stderr, "Failed to switch in TX pairing mode");
		return ret;
	}

	return 0;
}

static int controller_set_pairing_mode(struct controller *ctrl, int rx_mode)
{
	static const uint8_t rx_address[RTHERMO_ADDRESS_WIDTH] =
						    RTHERMO_PAIR_DEVICE_ADDRESS;
	static const uint8_t tx_address[RTHERMO_ADDRESS_WIDTH] =
						      RTHERMO_PAIR_MAIN_ADDRESS;
	int ret;

	/* Load pairing addresses:
	 *  - set RX address 0 with TX address for auto acknowledgment
	 *  - set RX address 1 with TX device address for pairing requests
	 */
	ret = nrf24_set_tx_address(ctrl->nrf, tx_address);
	ret |= nrf24_set_rx_address(ctrl->nrf, 0, tx_address);
	ret |= nrf24_set_rx_address(ctrl->nrf, 1, rx_address);
	if (ret) {
		fprintf(stderr, "Failed to set pairing addresses");
		return ret;
	}

	/* Switch to RX / TX mode */
	if (rx_mode)
		return controller_set_pairing_mode_rx(ctrl);
	return controller_set_pairing_mode_tx(ctrl);
}

static int controller_wait_tx(struct controller *ctrl, long timeout_ms)
{
	struct timeval last, now;
	int ret;

	/* Get start time */
	if (gettimeofday(&last, NULL))
		return -1;

	/* Wait until end of timeout or response ack */
	while (timeout_ms > 0) {
		/* Wait for ack */
		ret = nrf24_tx_wait(ctrl->nrf, 1000);
		if (ret < 0) {
			/* An error had occured */
			return ret;
		}

		/* Check if wait has been interrupted */
		if (ret == NRF24_WAIT_INTERRUPT) {
			/* Get current time */
			if (gettimeofday(&now, NULL))
				return -1;
			timeout_ms -=
				  ((now.tv_sec * 1000) + (now.tv_usec / 1000)) -
				 ((last.tv_sec * 1000) + (last.tv_usec / 1000));
			last = now;
			continue;
		}

		/* End of wait */
		break;
	}

	return ret;
}

static void *controller_thread(void *user_data)
{
	struct controller *ctrl = user_data;
	struct nrf24 *nrf = ctrl->nrf;
	uint8_t buffer[32];
	struct rthermo_cmd_req *req = (struct rthermo_cmd_req *) buffer;
	struct rthermo_cmd_resp resp;
	struct rthermo_cmd_pair_req *pair_req = &req->pair;
	struct rthermo_cmd_status_req *status_req = &req->status;
	uint8_t address[RTHERMO_ADDRESS_WIDTH];
	uint8_t pipe, size;
	double temperature;
	int pairing = 0;
	int next = 0;
	int ret;

	/* Copy main controller address for device address generation */
	memcpy(address, ctrl->address, RTHERMO_ADDRESS_WIDTH);

	/* Setup nRF24L01(+) */
	nrf24_setup_full(nrf, &ctrl->nrf_setup);

	/* Switch to pairing mode when no devices are registered */
	if (!ctrl->devs_count) {
		controller_set_pairing_mode(ctrl, 1);
		ctrl->pairing = pairing = 1;
	} else
		controller_set_normal_mode(ctrl, 1);

	/* Start main loop */
	if (ctrl->verbose)
		fprintf(stderr, "\nStart main controller loop in %s mode...\n",
			pairing ? "paring": "normal");

	/* Main controller loop */
	while (!ctrl->stop_thread) {
		/* Switch modes */
		if (pairing != ctrl->pairing) {
			if (ctrl->pairing)
				controller_set_pairing_mode(ctrl, 1);
			else
				controller_set_normal_mode(ctrl, 1);
			nrf24_rx_flush(ctrl->nrf);
			pairing = ctrl->pairing;
			if (ctrl->verbose) {
				fprintf(stderr, "Switch to %s mode\n",
					pairing ? "pairing" : "normal");
			}
		}

		/* Wait for next request */
		ret = nrf24_rx_wait(ctrl->nrf, -1, &pipe);
		if (ret) {
			/* An error occured */
			if (ret < 0)
				break;

			/* Timeout or signal has been received */
			if (ctrl->verbose)
				fprintf(stderr, "Timeout on RX\n");
			continue;
		}

		/* New payload */
		if (ctrl->verbose)
			fprintf(stderr, "New request on RX (pipe %d)\n", pipe);

		/* Read payload size */
		if (nrf24_get_dynamic_payload_size(ctrl->nrf, &size))
			continue;

		/* Read payload */
		if (nrf24_rx_read(ctrl->nrf, buffer, size))
			continue;

		/* Check mode */
		if (ctrl->pairing) {
			/* Check pairing request command */
			if (size != RTHERMO_CMD_PAIR_REQ_LENGTH ||
			    req->cmd != RTHERMO_CMD_PAIR_REQ)
				continue;

			/* Print pairing request */
			if (ctrl->verbose) {
				fprintf(stderr, "New pairing request:\n"
					" - id: %d\n",
					req->id);
				print_address(" - address: ",
					      pair_req->address);
				print_serial(" - serial: ", pair_req->serial);
				fprintf(stderr, " - caps: 0x%08x\n",
					pair_req->caps);
			}

			/* Check if device already exists */
			if (controller_find_device(ctrl, pair_req->address,
						   pair_req->serial) > 0) {
				if (ctrl->verbose)
					fprintf(stderr, "Device already "
						"paired\n");
				continue;
			}

			/* Switch to TX mode for pairing response */
			controller_set_pairing_mode_tx(ctrl);

			/* Find next free device slot */
			next = controller_find_next_free_device(ctrl);

			/* Generate device address */
			address[0] = ctrl->address[0] + next;

			/* Prepare pairing response */
			resp.cmd = RTHERMO_CMD_PAIR_RESP;
			resp.id = req->id;
			memcpy(resp.pair.address, address,
			       RTHERMO_ADDRESS_WIDTH);
			memcpy(resp.pair.main_address, ctrl->address,
			       RTHERMO_ADDRESS_WIDTH);

			/* Print pairing response */
			if (ctrl->verbose) {
				fprintf(stderr, "Send pairing response:\n");
				print_address(" - address: ", address);
			}

			/* Send response */
			if (nrf24_tx_write(ctrl->nrf, (uint8_t *) &resp,
					   RTHERMO_CMD_PAIR_RESP_LENGTH, 1)) {
				fprintf(stderr, "Failed to send pairing "
					"response\n");
				controller_set_pairing_mode_rx(ctrl);
				continue;
			}

			/* Wait for acknowledge */
			ret = controller_wait_tx(ctrl, 1000);
			if (ret) {
				/* An error occured */
				if (ret < 0)
					break;

				/* No acknowledge */
				fprintf(stderr, "No pairing response "
					"acknowledge received\n");
					controller_set_pairing_mode_rx(ctrl);
					continue;
			}

			/* Add device */
			if (controller_add_device(ctrl, next, resp.pair.address,
						  pair_req->serial,
						  pair_req->caps))
				fprintf(stderr, "Failed to add device\n");

			/* Print success */
			if (ctrl->verbose) {
				fprintf(stderr, "New device added:\n"
					" - dev: %d\n", next);
				print_address(" - address: ", address);
				print_serial(" - serial: ", pair_req->serial);
				fprintf(stderr, " - caps: 0x%08x\n",
					pair_req->caps);
			}

			/* Switch to RX mode */
			if (ctrl->devs_count == RTHERMO_MAX_DEVICES) {
				controller_set_normal_mode(ctrl, 1);
				ctrl->pairing = 0;
				if (ctrl->verbose)
					fprintf(stderr, "All devices are "
						"registered: switch to normal "
						"mode\n");
			} else
				controller_set_pairing_mode_rx(ctrl);
		} else {
			/* Check status request command */
			if (pipe >= RTHERMO_MAX_DEVICES ||
			    size != RTHERMO_CMD_STATUS_REQ_LENGTH ||
			    req->cmd != RTHERMO_CMD_STATUS_REQ)
				continue;

			/* Print status request */
			if (ctrl->verbose) {
				fprintf(stderr, "New status request:\n");
				print_serial(" - serial: ", status_req->serial);
				fprintf(stderr, " - id: %d\n"
					" - status: 0x%08x\n"
					" - temperature: 0x%08x\n"
					" - battery: 0x%02x\n",
					req->id, status_req->status,
					status_req->temperature,
					status_req->battery);
			}

			/* Check serial in request */
			if (memcmp(status_req->serial, ctrl->devs[pipe].serial,
				   RTHERMO_SERIAL_WIDTH))
				continue;

			/* Temperature control */
			temperature = rthermo_temperature_uint32_to_double(
						       status_req->temperature);
			/* TODO */
			ctrl->devs[pipe].status = 0;

			/* Add values to database */
			db_set_temperature(ctrl->db, pipe + 1, time(NULL),
					   temperature,
					   ctrl->devs[pipe].status);

			/* Switch to TX mode for status response */
			controller_set_normal_mode_tx(ctrl);

			/* Prepare pairing response */
			resp.cmd = RTHERMO_CMD_STATUS_RESP;
			resp.id = req->id;
			resp.status.status = ctrl->devs[pipe].status;

			/* Print status response */
			if (ctrl->verbose)
				fprintf(stderr, "Send status response:\n"
					" - status: 0x%08x\n",
					resp.status.status);

			/* Send response */
			if (nrf24_tx_write(ctrl->nrf, (uint8_t *) &resp,
					   RTHERMO_CMD_STATUS_RESP_LENGTH, 1)) {
				fprintf(stderr, "Failed to send status "
					"response\n");
				controller_set_normal_mode_rx(ctrl);
				continue;
			}

			/* Wait for acknowledge */
			ret = controller_wait_tx(ctrl, 1000);
			if (ret) {
				/* An error occured */
				if (ret < 0)
					break;

				/* No acknowledge */
				fprintf(stderr, "No status response acknowledge"
					" received\n");
			}

			/* Switch to RX mode */
			controller_set_normal_mode_rx(ctrl);
		}
	}

	return NULL;
}

/* Debug / verbose */
static void print_array(const char *name, const uint8_t *array, size_t len)
{
	int i;

	fprintf(stderr, name);
	fprintf(stderr, "%02x", array[0]);
	for (i = 1; i < len; i++)
		fprintf(stderr, ":%02x", array[i]);
	fprintf(stderr, "\n");
}

void controller_print_configuration(struct controller *ctrl)
{
	int i;

	/* Lock configuration access */
	pthread_mutex_lock(&ctrl->mutex);

	/* Dislay main controller configuration */
	fprintf(stderr, "Loaded database configuration from: %s\n",
		db_get_file(ctrl->db));
	fprintf(stderr, "Main controler configuration:\n");
	print_address(" - Main controller address: ", ctrl->address);
	print_serial(" - Main controller serial: ", ctrl->serial);
	fprintf(stderr, " - Device count: %d\n", ctrl->devs_count);
	fprintf(stderr, " - Next free device ID: %d\n",
		controller_find_next_free_device(ctrl));

	/* Display devices configuration */
	fprintf(stderr, "Devices configuration:\n");
	for (i = 0; i < RTHERMO_MAX_DEVICES; i++) {
		struct remote_device *dev = &ctrl->devs[i];

		/* Display device configuration */
		if (dev->used) {
			fprintf(stderr, "     + device %d:\n", i + 1);
			print_address("         - address: ", dev->address);
			print_serial("         - serial: ", dev->serial);
			fprintf(stderr, "         - caps: 0x%08x\n", dev->caps);
		} else
			fprintf(stderr, "     + device %d: not used\n", i + 1);
	}

	/* Unlock configuration access */
	pthread_mutex_unlock(&ctrl->mutex);
}

void controller_print_nrf24_configuration(struct controller *ctrl)
{
	/* Print nRF24L01(+) configuration */
	fprintf(stderr, "RF configuration:\n");
	nrf24_print_version(ctrl->nrf);
	nrf24_print_setup(ctrl->nrf);
}
