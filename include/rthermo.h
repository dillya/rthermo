/*
 * rthermo.h: RThermo RF protocol definitions
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

#ifndef _RTHERMO_H_
#define _RTHERMO_H_

#include <stdint.h>

/******************************************************************************
 *
 * RThermo: Remote Thermostat basde on nRF24L01+ wireless tranciever
 *
 * RThermo is composed of a main controller and up to 5 remote control devices:
 *  - main controller: it monitors local temperature and remote temperature
 *                     through data send periodically by associated devices.
 *                     It regulates house temperature globally or independently
 *                     by responding to status request send by devices with a
 *                     heaters activation flag.
 *                     So, main controller only respond to requests send by
 *                     devices.
 *  - devices: they measures local temperature and control heater activation.
 *             A status request is sent periodically to main controller with
 *             measured temperature and status response is expected in order to
 *             enable or disable heater.
 *             When device is not requesting status, it sleeps in order to save
 *             power.
 *             If it is connected to a battery, its level is also send in status
 *             requests to notify main controller of a low battery level status.
 *
 * Below, an example of a temperature control in a house with a boiler for
 * heating of most room of house and an extra electric heater for attic not
 * warmed by boiler:
 *       __________________________
 *      |       (Living room)      |  _______________________________
 *      |  _____________________   | |           (Basement)          |
 *      | |                     |  | |   __________                  |
 *      | |                     |  | |  |          |                 |
 *      | |                     | <===> | Device 0 | -> House Bolier |
 *      | |                     |  | |  |__________|                 |
 *      | |                     |  | |_______________________________|
 *      | |   Main controller   |  |  __________________________________
 *      | |     (T° sensor)     |  | |              (Attic)             |
 *      | |                     |  | |   __________                     |
 *      | |                     |  | |  |          |                    |
 *      | |                     | <===> | Device 1 | -> Electric heater |
 *      | |_____________________|  | |  |__________| <- Local T° sensor |
 *      |    v           v         | |__________________________________|
 *      |   Wifi   Electronic UI   |
 *      |__________________________|
 *
 ******************************************************************************/
#define RTHERMO_VERSION		"1.0.0"
#define RTHERMO_VERSION_MAJOR		1
#define RTHERMO_VERSION_MINOR		0
#define RTHERMO_VERSION_REVISION	0

/******************************************************************************
 * nRF settings
 *
 * This section contains all nRF24L01+ settings for RF and protocol. Default
 * registers are also provided for direct configuration (without any framework).
 *
 ******************************************************************************/

/* Basic settings
 *  - CRC width set to 2 bytes
 *  - Enable Auto Acknowlegment
 *  - Address width set to 5 bytes
 *  - Retransmit delay set to 1.5ms
 *  - Retransmit count set to 5
 *  - Payload width set to 32
 */
#define RTHERMO_CRC_WIDTH	2
#define RTHERMO_AUTO_ACK	1
#define RTHERMO_ADDRESS_WIDTH	5
#define RTHERMO_RETRY_DELAY	1500
#define RTHERMO_RETRY_COUNT	15
#define RTHERMO_PAYLOAD_WIDTH	32

/* RF setup
 *  - RF channel set to 40MHz (= 0x28)
 *  - Continuous carrier transmit disabled
 *  - Bitrate set to 250kbps
 *  - TX output power set to 0dBm
 */
#define RTHERMO_RF_CHANNEL	40
#define RTHERMO_RF_CONTINUOUS	0
#define RTHERMO_RF_BITRATE	1000
#define RTHERMO_RF_POWER	0

/* Features
 *  - Enable dynamic payload width (only for pipeline 0)
 *  - Disable payload width ACK
 *  - Disable W_TX_PAYLOAD_NOACK command
 */
#define RTHERMO_EN_DPL		1
#define RTHERMO_EN_ACK_PAY	0
#define RTHERMO_EN_DYN_ACK	0

/* Default register setup */
#define RTHERMO_REG_CONFIG	0x0C
#define RTHERMO_REG_EN_AA	0x3F
#define RTHERMO_REG_SETUP_AW	0x03
#define RTHERMO_REG_SETUP_RETR	0x5F
#define RTHERMO_REG_RF_CH	0x28
#define RTHERMO_REG_RF_SETUP	0x06
#define RTHERMO_REG_DYNPD	0x3F
#define RTHERMO_REG_FEATURE	0x04

/******************************************************************************
 * Pairing mode (used with PAIR commands)
 *
 * Device association with main controller is done with PAIR command: since
 * nRF24L01+ is limited for multiple RX addresses (same 32 MSB and different 8
 * LSB), a pairing process is done:
 *  1. main controller and device set PAIR addresses to communitcate,
 *  2. device send PAIR request to main controller with its current address and
 *     its capabilities,
 *  3. main controller check if address is already used and attribute a new one
 *     if necessary,
 *  4. device save new address and uses it for next requests.
 *
 ******************************************************************************/
#define RTHERMO_PAIR_MAIN_ADDRESS	{ 0x51, 0x69, 0x90, 0xad, 0xfd }
#define RTHERMO_PAIR_DEVICE_ADDRESS	{ 0xbe, 0x58, 0xcd, 0x61, 0xaf }

/* Device serial number */
#define RTHERMO_SERIAL_WIDTH	6

/* Max associated devices (= max simultaneous RX addresses) */
#define RTHERMO_MAX_DEVICES	6

/******************************************************************************
 * Device capabilities
 *
 * Remote devices can embed one or more capabilities like heater control, local
 * temperature measurement, ...
 *  - HEATER_CONTROL: device can enable / disable a heater
 *  - LOCAL_TEMPERATURE: device can measure local temperature
 *  - USE_BATTERY: device is connected on a battery
 *
 ******************************************************************************/
#define RTHERMO_DEVICE_CAPS_HEATER_CONTROL	(1 << 0)
#define RTHERMO_DEVICE_CAPS_LOCAL_TEMPERATURE	(1 << 1)
#define RTHERMO_DEVICE_CAPS_USE_BATTERY		(1 << 2)

/******************************************************************************
 * Protocol commands
 *
 * Two type of commands are available:
 *  - request (= req): a request is send from device to main controller
 *  - response (= resp): a response is send back from main controller to device
 *                       after it received a request from a device.
 *
 * Commands code are 16bits length (2bytes). They are composed of a magic word
 * to prevent receiving fake messages from other possible protocols on same RF
 * channel.
 * Note: only 15 commands can be coded with the current magic words.
 * Command code is followed by a unique ID which must be incremented at each
 * request in order to identify easily each request / response.
 *
 ******************************************************************************/
#define RTHERMO_CMD_MAGIC_REQ		0x7A50
#define RTHERMO_CMD_MAGIC_RESP		0xC740
#define RTHERMO_CMD_MAGIC_MASK		0xFFF0
#define RTHERMO_CMD_LENGTH		3

/* Disable structure padding */
#ifndef __SDCC
#pragma pack(push,1)
#endif

/* PING command: check if main is available
 *    Request: no data
 *    Response: struct rthermo_cmd_ping_resp
 *       - time: current timestamp in s of main device
 */
#define RTHERMO_CMD_PING_REQ		RTHERMO_CMD_MAGIC_REQ + 1
#define RTHERMO_CMD_PING_RESP		RTHERMO_CMD_MAGIC_RESP + 1

struct rthermo_cmd_ping_resp {
	uint32_t time;
};

#define RTHERMO_CMD_PING_REQ_LENGTH \
	RTHERMO_CMD_LENGTH
#define RTHERMO_CMD_PING_RESP_LENGTH \
	RTHERMO_CMD_LENGTH + sizeof(struct rthermo_cmd_ping_resp)

/* PAIR command: pairing handshake with main device
 *    Request: struct rthermo_cmd_pair_req
 *       - address: current address of device
 *       - caps: device capabilities (see RTHERMO_DEVICE_CAPS_ for details)
 *    Response: struct rthermo_cmd_pair_resp
 *       - address: new address assigned to device
 *       - main_address: address of main controller
 */
#define RTHERMO_CMD_PAIR_REQ		RTHERMO_CMD_MAGIC_REQ + 2
#define RTHERMO_CMD_PAIR_RESP		RTHERMO_CMD_MAGIC_RESP + 2

struct rthermo_cmd_pair_req {
	uint8_t serial[RTHERMO_SERIAL_WIDTH];
	uint8_t address[RTHERMO_ADDRESS_WIDTH];
	uint32_t caps;
};

struct rthermo_cmd_pair_resp {
	uint8_t address[RTHERMO_ADDRESS_WIDTH];
	uint8_t main_address[RTHERMO_ADDRESS_WIDTH];
};

#define RTHERMO_CMD_PAIR_REQ_LENGTH \
	RTHERMO_CMD_LENGTH + sizeof(struct rthermo_cmd_pair_req)
#define RTHERMO_CMD_PAIR_RESP_LENGTH \
	RTHERMO_CMD_LENGTH + sizeof(struct rthermo_cmd_pair_resp)

/* STATUS command: get new status of activation
 *    Request: struct rthermo_cmd_status_req
 *       - status: current status of device
 *       - battery: battery level of device
 *    Response: struct rthermo_cmd_status_resp
 *       - status: new status for device
 */
#define RTHERMO_CMD_STATUS_REQ		RTHERMO_CMD_MAGIC_REQ + 3
#define RTHERMO_CMD_STATUS_RESP		RTHERMO_CMD_MAGIC_RESP + 3

struct rthermo_cmd_status_req {
	uint8_t serial[RTHERMO_SERIAL_WIDTH];
	uint32_t status;
	uint32_t temperature;
	uint8_t battery;
};

struct rthermo_cmd_status_resp {
	uint32_t status;
};

#define RTHERMO_CMD_STATUS_REQ_LENGTH \
	RTHERMO_CMD_LENGTH + sizeof(struct rthermo_cmd_status_req)
#define RTHERMO_CMD_STATUS_RESP_LENGTH \
	RTHERMO_CMD_LENGTH + sizeof(struct rthermo_cmd_status_resp)

/* Global command */
struct rthermo_cmd_req {
	uint16_t cmd;
	uint8_t id;
	union {
		struct rthermo_cmd_pair_req pair;
		struct rthermo_cmd_status_req status;
	};
};

struct rthermo_cmd_resp {
	uint16_t cmd;
	uint8_t id;
	union {
		struct rthermo_cmd_ping_resp ping;
		struct rthermo_cmd_pair_resp pair;
		struct rthermo_cmd_status_resp status;
	};
};

/* Helpers to check commands code */
#define rthermo_cmd_check_req(req) \
	((req.cmd & RTHERMO_CMD_MAGIC_MASK) != RTHERMO_CMD_MAGIC_REQ)
#define rthermo_cmd_check_resp(resp) \
	((resp.cmd & RTHERMO_CMD_MAGIC_MASK) != RTHERMO_CMD_MAGIC_RESP)

/* Helper to convert temperature
 * Temperature is stored on 32bits with 16bits MSB for integer and 16bits LSB
 * for decimal.
 */
#define rthermo_temperature_uint32_to_double(temp) \
	((temp >> 16) + ((temp & 0x0FFFF) / 0x10000))
#define rthermo_temperature_double_to_uint32(temp) \
	(((temp * 0x10000) & 0x0FFFF) | ((uint32_t)temp << 16 ))

/* Enable structure padding */
#ifndef __SDCC
#pragma pack(pop)
#endif

#endif /* _RTHERMO_H_ */
