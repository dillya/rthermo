/*
 * config_file.c: configuration file handler
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

#include <string.h>
#include <libconfig.h>

#include "config_file.h"

int config_file_load(const char *name, struct config *config)
{
	config_t cfg;
	config_setting_t *set;
	const char *str;
	int val;

	/* Reset configuration */
	memset(config, 0, sizeof(*config));

	/* Initialize config parser */
	config_init(&cfg);

	/* Parse configuration file */
	if (!config_read_file(&cfg, name)) {
		fprintf(stderr, "[config] failed to parse %s: %d - %s\n",
			name, config_error_line(&cfg), config_error_text(&cfg));
		config_destroy(&cfg);
		return -1;
	}

	/* Get database settings */
	set = config_lookup(&cfg, "db");
	if (set) {
		const char *str = NULL;

		/* Get file */
		if (config_setting_lookup_string(set, "file", &str) && str)
			config->db.file = strdup(str);
	}
	if (!config->db.file)
		config->db.file = strdup(DEFAULT_DB_FILE);

	/* Get HTTP settings */
	set = config_lookup(&cfg, "httpd");
	if (set) {
		if (config_setting_lookup_int(set, "port", &val))
			config->httpd.port = val;
	} else {
		config->httpd.port = DEFAULT_HTTP_PORT;
	}

	/* Get nRF24L01 settings */
	set = config_lookup(&cfg, "nrf");
	if (set) {
		const char *str = NULL;

		/* Get SPI device */
		if (config_setting_lookup_string(set, "spidev", &str) && str) {
			config->nrf.spi_dev = strdup(str);
		} else {
			fprintf(stderr, "No SPI device found!!\n");
			goto error;
		}

		/* Get GPIOs */
		if (!config_setting_lookup_int(set, "irq_gpio",
					       &config->nrf.irq_gpio))
			config->nrf.irq_gpio = -1;
		if (!config_setting_lookup_int(set, "ce_gpio",
					      &config->nrf.ce_gpio)) {
			fprintf(stderr, "No CE GPIO found!!\n");
			goto error;
		}
	} else {
		fprintf(stderr, "No nRF24L01 settings found!!\n");
		goto error;
	}

	/* Destroy config parser */
	config_destroy(&cfg);

	return 0;

error:
	config_destroy(&cfg);
	return -1;
}
