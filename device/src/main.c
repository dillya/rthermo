/*
 * main.c: RThermo device based in nRF24LE1 SoC
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
#include <string.h>

#include "reg24le1.h"
#include "interrupt.h"
#include "uart.h"
#include "rf.h"
#include "delay.h"
#include "rtc2.h"
#include "memory.h"
#include "rng.h"

#include "rthermo.h"
#include "config.h"

/* Retentive memory area (stored IRAM)
 * All variables defined in this area won't be reset after system wake up
 *  - rtc_started: flag which indicates if RTC2 timer is started: if not, it
 *                 means that pairing has not been done yet so we switch in
 *                 deep sleep mode to wait for user pairing request. When
 *                 pairing has been done, we send periodically a STATUS request
 *                 and then switch to a Memory retention, timer on mode to wait
 *                 next user pairing request or next STATUS reuqest trigger.
 *  - rtc_count: remaining RTC wake up before main loop execution
 *  - serial: serial number of device
 *  - main_address: RF address of main controller
 *  - device_address: RF address assigned for this device
 *  - status: current status of the device
 *  - id: ID for next request
 */
__data bool rtc_started;
__data uint8_t rtc_count;
__data uint8_t serial[RTHERMO_SERIAL_WIDTH];
__data uint8_t main_address[RTHERMO_ADDRESS_WIDTH];
__data uint8_t device_address[RTHERMO_ADDRESS_WIDTH];
__data uint32_t status;
__data uint8_t id;

/* Pair addresses */
static uint8_t pair_tx_address[RTHERMO_ADDRESS_WIDTH] =
						    RTHERMO_PAIR_DEVICE_ADDRESS;
static uint8_t pair_rx_address[RTHERMO_ADDRESS_WIDTH] =
						      RTHERMO_PAIR_MAIN_ADDRESS;

/* Flag used to stop RF wait for messages */
static bool stop_rf_rx_wait;

/* Get previous power down mode */
#ifndef pwr_clk_mgmt_prev_pwr_down_mode
#define pwr_clk_mgmt_prev_pwr_down_mode(pwrdwn_reg_val) \
	(pwrdwn_reg_val & 0x07)
#define pwr_clk_mgmt_was_prev_pwr_down_power_off(pwrdwn_reg_val) \
	(pwr_clk_mgmt_prev_pwr_down_mode(pwrdwn_reg_val) == 0 ? true : false)
#define pwr_clk_mgmt_was_prev_pwr_down_deep_sleep(pwrdwn_reg_val) \
	(pwr_clk_mgmt_prev_pwr_down_mode(pwrdwn_reg_val) == 1 ? true : false)
#define pwr_clk_mgmt_was_prev_pwr_down_memory_ret_tmr_off(pwrdwn_reg_val) \
	(pwr_clk_mgmt_prev_pwr_down_mode(pwrdwn_reg_val) == 2 ? true : false)
#define pwr_clk_mgmt_was_prev_pwr_down_memory_ret_tmr_on(pwrdwn_reg_val) \
	(pwr_clk_mgmt_prev_pwr_down_mode(pwrdwn_reg_val) == 3 ? true : false)
#define pwr_clk_mgmt_was_prev_pwr_down_register_ret(pwrdwn_reg_val) \
	(pwr_clk_mgmt_prev_pwr_down_mode(pwrdwn_reg_val) == 4 ? true : false)
#define pwr_clk_mgmt_was_prev_pwr_down_standby(pwrdwn_reg_val) \
	(pwr_clk_mgmt_prev_pwr_down_mode(pwrdwn_reg_val) == 7 ? true : false)
#endif

#ifdef DEBUG
static bool debug_initialized = 0;

static void debug_init()
{
	/* Init UART for debug (9600 baudrate, with only TX) */
	gpio_pin_configure(GPIO_PIN_ID_FUNC_TXD,
	   GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
	   GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
	   GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
	uart_configure_manual_baud_calc(UART_CONFIG_OPTION_MODE_1_UART_8_BIT |
			     UART_CONFIG_OPTION_CLOCK_FOR_MODES_1_3_USE_BR_GEN |
			     UART_CONFIG_OPTION_BIT_SMOD_SET, 972);
	debug_initialized = 1;
}

static void debug_flush()
{
	/* Wait UART completion */
	if (debug_initialized)
		interrupt_wait_for_uart_tx();
}

static int debug_printf(const char *fmt, ...)
{
	va_list va;
	int ret;

	/* Init debug */
	if (!debug_initialized)
		debug_init();

	/* Print */
	va_start(va, fmt);
	ret = vprintf(fmt, va);
	va_end(va);

	return ret;
}

static void debug_print_array(const char *name, const uint8_t *data,
			      uint16_t len)
{
	/* Init debug */
	if (!debug_initialized)
		debug_init();

	/* Print array */
	printf("%s: ", name);
	while (len--)
		printf("%02x ", *data++);
	printf("\r\n");
}
#else
#define debug_init()
#define debug_flush()
#define debug_printf (void)
#define debug_print_array (void)
#endif

interrupt_isr_rfirq()
{
}

interrupt_isr_rtc2()
{
	stop_rf_rx_wait = true;
}

/* Power down setup */
static void pwr_disable_pins()
{
	uint8_t reg, i;

	/* Set all pins direction to input */
	P0DIR = 0xFF;
	P1DIR = 0xFF;

	/* Disable digital input buffer
	 * If retention latches are closed before switch to lower power mode,
	 * this configuration is very important in order to reduce power
	 * consumption and get values of the datasheet "26.1 Power consumption"
	 * section.
	 */
	reg = GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_OFF | PXCON_IN_OUT;
	for (i = 0; i < 8; i++, reg++)
		if (DEVICE_PAIRING_GPIO != i)
			P0CON = reg;;
	reg = GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_OFF | PXCON_IN_OUT;
	for (i = 8; i < 16; i++, reg++)
		if (DEVICE_PAIRING_GPIO != i)
			P1CON = reg;
}

/* RTC setup */
static void rtc_init()
{
	/* CLKLF and RTC2 already set */
	if (rtc_started)
		return;

	/* Configure RTC with its period and enable auto reload */
	rtc2_configure(RTC2_CONFIG_OPTION_ENABLE |
		       RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ |
		       RTC2_CONFIG_OPTION_DO_NOT_CAPTURE_ON_RFIRQ, RTC_VALUE);

	/* Initialize RTC cycle counter */
	rtc_count = RTC_COUNT;
	rtc_started = true;
}

/* Serial number generation */
static void serial_generate()
{
	uint8_t i;

	/* Setup random number generator */
	rng_configure(RNG_CONFIG_CORRECTOR_ENABLE | RNG_CONFIG_OPTION_RUN);

	/* Fill serial array */
	for (i = 0; i < RTHERMO_SERIAL_WIDTH; i++)
		serial[i] = rng_get_next_byte();

	/* Stop generator */
	rng_stop();

	/* Save serial number to flash */
	memory_flash_erase_page(MEMORY_FLASH_NV_STD_END_FIRST_PAGE_NUM);
	memory_flash_write_bytes(MEMORY_FLASH_NV_STD_END_START_ADDRESS,
				 RTHERMO_SERIAL_WIDTH, serial);
}

/* RF setup */
static inline void rf_write_reg(uint8_t reg, uint8_t val)
{
	rf_write_register(reg, &val, 1);
}

static inline void rf_setup()
{
	/* By default, RF is configured in TX mode */
	rf_spi_configure_enable();
	rf_write_reg(RF_EN_RXADDR, 0x01);
	rf_write_reg(RF_SETUP_AW, RTHERMO_REG_SETUP_AW);
	rf_write_reg(RF_SETUP_RETR, RTHERMO_REG_SETUP_RETR);
	rf_write_reg(RF_RF_CH, RTHERMO_REG_RF_CH);
	rf_write_reg(RF_RF_SETUP, RTHERMO_REG_RF_SETUP);
	rf_write_reg(RF_DYNPD, RTHERMO_REG_DYNPD);
	rf_write_reg(RF_FEATURE, RTHERMO_REG_FEATURE);
	rf_power_down_param(RTHERMO_REG_CONFIG);
}

static inline void rf_setup_addresses(uint8_t *tx, uint8_t *rx)
{
	/* RX address 0 is used for auto acknowledgment (in TX mode) and RX
	 * address 1 is used for RX mode.
	 */
	rf_set_tx_addr(tx, RTHERMO_ADDRESS_WIDTH);
	rf_set_rx_addr(tx, RTHERMO_ADDRESS_WIDTH, 0);
	rf_set_rx_addr(rx, RTHERMO_ADDRESS_WIDTH, 1);
}

static inline void rf_setup_rx_mode()
{
	/* In RX mode, use RX address 1 to receive data from main controller */
	rf_irq_clear_all();
	rf_write_reg(RF_EN_RXADDR, 0x02);
	rf_set_as_rx(true);
}

static inline void rf_setup_tx_mode()
{
	/* In TX mode, use RX address 0 for auto acknowledgment */
	rf_set_as_tx();
	rf_write_reg(RF_EN_RXADDR, 0x01);
	rf_irq_clear_all();
}

static void rf_start_rx_wait(uint16_t timeout_ms)
{
	uint16_t value = (32768 * (uint32_t) timeout_ms) / 1000;

	/* Reset wait flag */
	stop_rf_rx_wait = false;

	/* Setup RTC value to timeout */
	if (!rtc_started)
		rtc2_configure(RTC2_CONFIG_OPTION_ENABLE |
		       RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ |
		       RTC2_CONFIG_OPTION_DO_NOT_CAPTURE_ON_RFIRQ, value);
	else
		rtc2_set_compare_val(value);

	/* Enable RTC2 interrupt */
	interrupt_clear_tick();
	interrupt_control_rtc2_enable();
}

static void rf_stop_rx_wait()
{
	/* Disable RTC2 interrupt */
	interrupt_control_rtc2_disable();

	/* Restore RTC */
	if (rtc_started)
		rtc2_set_compare_val(RTC_VALUE);
	else
		rtc2_configure(RTC2_CONFIG_OPTION_DISABLE, 0);
}

/* Main entry function */
void main(void)
{
	struct rthermo_cmd_resp resp;
	struct rthermo_cmd_req req;
	bool resp_received = false;
	uint8_t pwrdwn = PWRDWN;
	uint8_t size;

	/* Setup GPIO
	 *  - DEVICE_PAIRING_GPIO: set as input with internal pull down resistor
	 *  - DEVICE_HEATER_ON_GPIOO: set as output
	 *  - DEVICE_HEATER_OFF_GPIOO: set as output (if different from ON GPIO)
	 */
	gpio_pin_configure(DEVICE_PAIRING_GPIO,
	    GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
	    GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_DOWN_RESISTOR);
	gpio_pin_configure(DEVICE_HEATER_ON_GPIO,
	   GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
	   GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR |
	   GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
#if (DEVICE_HEATER_OFF_GPIO != DEVICE_HEATER_ON_GPIO)
	gpio_pin_configure(DEVICE_HEATER_OFF_GPIO,
	   GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
	   GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR |
	   GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
#endif

	/* Open wakeup on pin setup latches
	 * Note: GPIOs must be set before opening retention latches in order to
	 *       keep configuration from previous wake.
	 */
	pwr_clk_mgmt_open_retention_latches();

	/* First boot (power up or reset)
	 * Load serial and addresses from Flash
	 */
	if (pwr_clk_mgmt_was_prev_pwr_down_power_off(pwrdwn)) {
		/* Init variables in retention area */
		rtc_started = false;
		id = 0;

		/* Enable internal 32.768 kHz RC clock needed by RTC */
		pwr_clk_mgmt_clklf_configure(
			     PWR_CLK_MGMT_CLKLF_CONFIG_OPTION_CLK_SRC_RCOSC32K);

		/* Load serial number
		 * The serial number is stored in first RTHERMO_SERIAL_WIDTH
		 * bytes of second page of NV flash section (the serial number
		 * is generated at first boot after a flash).
		 */
		memory_flash_read_bytes(MEMORY_FLASH_NV_STD_END_START_ADDRESS,
					RTHERMO_SERIAL_WIDTH, serial);
		if (serial[0] == 0xFF)
			serial_generate();

		/* Load main controller and device addresses
		 * The both addresses are stored in first
		 * (RTHERMO_ADDRESS_WIDTH * 2) bytes of first page of extended
		 * NV flash section.
		 */
		memory_flash_read_bytes(MEMORY_FLASH_NV_EXT_END_START_ADDRESS,
					RTHERMO_ADDRESS_WIDTH, device_address);
		memory_flash_read_bytes(MEMORY_FLASH_NV_EXT_END_START_ADDRESS +
				        RTHERMO_ADDRESS_WIDTH,
				        RTHERMO_ADDRESS_WIDTH, main_address);

		/* Print settings */
		debug_printf("=========================\r\n");
		debug_printf("= RThermo device v" RTHERMO_VERSION " =\r\n");
		debug_printf("=========================\r\n");
		debug_print_array("Serial", serial, RTHERMO_SERIAL_WIDTH);
		debug_print_array("Device address", device_address,
				  RTHERMO_ADDRESS_WIDTH);
		debug_print_array("Controller address", main_address,
				  RTHERMO_ADDRESS_WIDTH);
		debug_printf("\r\n");

		/* No device and/or main controller addresses set: sleep
		 * until next pairing request.
		 */
		if (device_address[0] == 0xFF ||
		    main_address[0] == 0xFF)
			goto sleep;

		/* Start RTC for periodic wake up of CPU: we use RTC2
		 * timer to generate a wake up event every RTC_PERIOD_MS
		 * ms. At  each wake up, we decrement a counter
		 * (rtc_count) to generate a slower clock used to
		 * trigger a STATUS request every RTC_SLEEP_MS ms.
		 */
		rtc_init();
	}

	/* Wait for RTC sleep duration */
	if (pwrdwn & 0x40) {
		if (--rtc_count)
			goto sleep;
		rtc_count = RTC_COUNT;
	}

	/* Setup RF module */
	rf_setup();

	/* Enable interrupts
	 *  - RF IRQ: used to wait TX / RX events on RF
	 */
	interrupt_control_global_enable();
	interrupt_control_rfirq_enable();

	/* Check if pairing has been requested */
	if ((pwrdwn & 0x80)) {
		/* False pairing request */
		if (!gpio_pin_val_read(DEVICE_PAIRING_GPIO))
			goto sleep;

		/* Setup pairing addresses */
		rf_setup_addresses(pair_tx_address, pair_rx_address);
		rf_power_up_param(false, RTHERMO_REG_CONFIG);

		/* Prepare PAIR request */
		req.cmd = RTHERMO_CMD_PAIR_REQ;
		req.id = id++;
		memcpy(req.pair.serial, serial, RTHERMO_SERIAL_WIDTH);
		memcpy(req.pair.address, device_address, RTHERMO_ADDRESS_WIDTH);
		req.pair.caps = DEVICE_CAPS;

		/* Send PAIR request to main controller */
		rf_write_tx_payload((uint8_t *) &req,
				    RTHERMO_CMD_PAIR_REQ_LENGTH, true);

		/* Wait acknowledgment from main controller */
		pwr_clk_mgmt_enter_pwr_mode_standby();
		if (rf_irq_max_rt_active()) {
			debug_printf("ERROR: no pairing acknowledgment\r\n");
			goto sleep;
		}

		/* Switch to RX mode */
		rf_setup_rx_mode();

		/* Wait for response */
		rf_start_rx_wait(PAIRING_WAIT_RESPONSE_MS);
		while (!stop_rf_rx_wait) {
			/* Wait for timer or response */
			pwr_clk_mgmt_enter_pwr_mode_standby();

			/* Get RX payloads */
			while (!rf_rx_fifo_is_empty()) {
				/* Read payload size */
				rf_read_rx_payload_width(&size);
				if (size != RTHERMO_CMD_PAIR_RESP_LENGTH) {
					debug_printf(
					       "bad pairing response size\r\n");
					continue;
				}

				/* Read payload */
				rf_read_rx_payload((uint8_t *) &resp, size);

				/* Check message */
				if (resp.cmd == RTHERMO_CMD_PAIR_RESP &&
				    resp.id == req.id) {
					resp_received = true;
					break;
				}
				debug_printf("bad pairing response\r\n");
			}
			rf_irq_clear_all();

			/* Response has been received */
			if (resp_received)
				break;
		}
		rf_stop_rx_wait();

		/* Failed to get a response */
		if (stop_rf_rx_wait && !resp_received) {
			debug_printf("ERROR: no pairing response\r\n");
			goto sleep;
		}

		/* Check address */
		if (resp.pair.address[0] == 0x00 ||
		    resp.pair.address[0] == 0xFF ||
		    resp.pair.main_address[0] == 0x00 ||
		    resp.pair.main_address[0] == 0xFF) {
			debug_printf("ERROR: bad pairing address\r\n");
			goto sleep;
		}

		/* Switch back to TX mode */
		rf_setup_tx_mode();
		resp_received = false;

		/* Copy addresses */
		memcpy(device_address, resp.pair.address,
		       RTHERMO_ADDRESS_WIDTH);
		memcpy(main_address, resp.pair.main_address,
		       RTHERMO_ADDRESS_WIDTH);

		/* Save addresses to flash */
		memory_flash_erase_page(MEMORY_FLASH_NV_EXT_END_FIRST_PAGE_NUM);
		memory_flash_write_bytes(MEMORY_FLASH_NV_EXT_END_START_ADDRESS,
					RTHERMO_ADDRESS_WIDTH, device_address);
		memory_flash_write_bytes(MEMORY_FLASH_NV_EXT_END_START_ADDRESS +
					 RTHERMO_ADDRESS_WIDTH,
					 RTHERMO_ADDRESS_WIDTH, main_address);

		/* Print new addresses */
		debug_print_array("New Device address", device_address,
				  RTHERMO_ADDRESS_WIDTH);
		debug_print_array("New Controller address", main_address,
				  RTHERMO_ADDRESS_WIDTH);
		debug_printf("\r\n");
		debug_flush();

		/* Start RTC (see previous call for more details) */
		rtc_init();
	}

	/* Setup addresses */
	rf_setup_addresses(device_address, main_address);
	rf_power_up_param(false, RTHERMO_REG_CONFIG);

	/* Prepare STATUS request */
	req.cmd = RTHERMO_CMD_STATUS_REQ;
	req.id = id++;
	memcpy(req.status.serial, serial, RTHERMO_SERIAL_WIDTH);
	req.status.status = status;
	req.status.temperature = 0;
	req.status.battery = ~0;

	/* Send STATUS request to main controller */
	rf_write_tx_payload((uint8_t *) &req, RTHERMO_CMD_STATUS_REQ_LENGTH,
			    true);

	/* Wait acknowledgment from main controller */
	pwr_clk_mgmt_enter_pwr_mode_standby();
	if (rf_irq_max_rt_active()) {
		debug_printf("ERROR: no status acknowledgment\r\n");
		goto sleep;
	}
	rf_irq_clear_all();

	/* Switch to RX mode */
	rf_setup_rx_mode();

	/* Wait for response */
	rf_start_rx_wait(STATUS_WAIT_RESPONSE_MS);
	while (!stop_rf_rx_wait) {
		/* Wait for timer or response */
		pwr_clk_mgmt_enter_pwr_mode_standby();

		/* Get RX payloads */
		while (!rf_rx_fifo_is_empty()) {
			/* Read payload size */
			rf_read_rx_payload_width(&size);
			if (size != RTHERMO_CMD_STATUS_RESP_LENGTH) {
				debug_printf("bad status response size\r\n");
				continue;
			}

			/* Read payload */
			rf_read_rx_payload((uint8_t *) &resp, size);

			/* Check message */
			if (resp.cmd == RTHERMO_CMD_STATUS_RESP &&
			    resp.id == req.id) {
				resp_received = true;
				break;
			}
			debug_printf("bad status response\r\n");
		}
		rf_irq_clear_all();

		/* Response has been received */
		if (resp_received)
			break;
	}
	rf_stop_rx_wait();

	/* Failed to get a response */
	if (stop_rf_rx_wait && !resp_received) {
		debug_printf("ERROR: no status response\r\n");
		goto sleep;
	}

	/* Set new status */
	/* TODO */
	status = resp.status.status;

	/* Print status */
	debug_printf("Status: 0x%04x%04x\r\n", (uint16_t) (status >> 16),
		     (uint16_t) status);

sleep:
	/* Flush debug output */
	debug_flush();

	/* Setup wakeup source for pairing GPIO */
	pwr_clk_mgmt_wakeup_pins_configure(DEVICE_PAIRING_WAKEUP);
	pwr_disable_pins();

	/* Switch to Memory retention, timer on mode: program ends here and will
	 * be restarted on next RTC wake up tick.
	 * Note: we close retention latches to keep wakeup pins configuration.
	 */
	pwr_clk_mgmt_close_retention_latches();
	while(1)
		pwr_clk_mgmt_enter_pwr_mode_memory_ret_tmr_on();
}

