/*
 * config.h: RThermo device based in nRF24LE1 SoC configuration
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

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "pwr_clk_mgmt.h"
#include "gpio.h"

/* Enable debug
 *  - Setup UART TX pin at 9600 baud
 *  - Print some debug messages
 */
#define DEBUG

/* Device GPIOs
 *  - DEVICE_PAIRING_GPIO: GPIO on which pairing button is connected
 *  - DEVICE_PAIRING_WAKEUP: GPIO WakeUp configuration (should be same pin as
 *                           DEVICE_PAIRING_GPIO, see pwr_clk_mgmt.h for defs).
 *                           If not defined, pairing detection is done on next
 *                           next CPU wakeup from RTC.
 *  - DEVICE_HEATER_ON_GPIO: GPIO on which relay coil for ON position is
 *                           connected
 *  - DEVICE_HEATER_OFF_GPIO: GPIO on which relay coil for OFF position is
 *                            connected
 */
#define DEVICE_PAIRING_GPIO	GPIO_PIN_ID_P0_0
#define DEVICE_PAIRING_WAKEUP \
	PWR_CLK_MGMT_WAKEUP_CONFIG_OPTION_INPUT_P0_0_ENABLE
#define DEVICE_HEATER_ON_GPIO	GPIO_PIN_ID_P0_1
#define DEVICE_HEATER_OFF_GPIO	GPIO_PIN_ID_P0_2

/* Device capabilities (only support heater control) */
#define DEVICE_CAPS	RTHERMO_DEVICE_CAPS_HEATER_CONTROL

/* TX / RX setup
 *  - PAIRING_WAIT_RESPONSE_MS: time to wait for a pairing response
 *  - STATUS_WAIT_RESPONSE_MS: time to wait for a status response
 */
#define PAIRING_WAIT_RESPONSE_MS	1000
#define STATUS_WAIT_RESPONSE_MS		1000

/* RTC setup for power save
 *  - RTC_PERIOD_MS: period of RTC timer wake up (max 2s)
 *  - RTC_SLEEP_MS: CPU sleep duration before executing main loop
 */
#define RTC_PERIOD_MS	2000
#define RTC_SLEEP_MS	60000
#define RTC_VALUE	(RTC_PERIOD_MS * 65535 / 2000)
#define RTC_COUNT	(RTC_SLEEP_MS / RTC_PERIOD_MS)

#endif /* _CONFIG_H_ */
