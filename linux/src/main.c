/*
 * main.c: RThermo main controller / device for Linux
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
#include <getopt.h>
#include <unistd.h>
#include <signal.h>

#include "rthermo.h"

#include "config_file.h"
#include "controller.h"
#include "httpd.h"
#include "nrf24.h"
#include "db.h"

static int main_loop_stop = 0;

static void signal_handler(int sig)
{
	main_loop_stop = 1;
}

static void print_usage(const char *name)
{
	fprintf(stderr, "Usage: %s [OPTIONS]\n"
		"\n"
		"Supported options:\n"
		" -c --config FILE      Set configuration file to use "
			"(default: %s)\n"
		" -d --daemon           Run as daemon\n"
		" -h --help             Print this help\n"
		" -v --verbose          Enable verbose output\n",
		name, DEFAULT_CONFIG_FILE);
}

int main(int argc, char *argv[])
{
	char *config_file = NULL;
	struct controller *ctrl;
	struct config cfg;
	struct nrf24 *nrf;
	struct db *db;
	int daemonize = 0;
	int verbose = 0;
	int ret;
	int i;

	/* Parse options */
	while (1) {
		static const char *short_options = "c:dhv";
		static struct option long_options[] = {
			{"config",      required_argument, 0, 'c'},
			{"daemon",      no_argument,       0, 'd'},
			{"help",        no_argument,       0, 'h'},
			{"verbose",     no_argument,       0, 'v'},
			{0, 0, 0, 0}
		};
		int c, option_index = 0;

		/* Get next argument */
		c = getopt_long(argc, argv, short_options, long_options,
				&option_index);
		if (c == EOF)
			break;

		switch (c) {
			case 'c':
				config_file = optarg;
				break;
			case 'd':
				daemonize = 1;
				break;
			case 'h':
				print_usage(argv[0]);
				return EXIT_SUCCESS;
			case 'v':
				verbose = 1;
				break;
			default:
				print_usage(argv[0]);
				return EXIT_FAILURE;
		}
	}

	/* Set default values */
	if (!config_file)
		config_file = DEFAULT_CONFIG_FILE;

	/* Daemonize */
	if (daemonize && daemon (1, 0)) {
		fprintf(stderr, "Failed to daemonize!");
		return EXIT_FAILURE;
	}

	/* Load configuration */
	if (config_file_load(config_file, &cfg))
		return EXIT_FAILURE;

	/* Display configuration */
	if (verbose) {
		fprintf(stderr, "Loaded configuration: (from %s)\n"
			" + Database:\n"
			"     - File: %s\n"
			" + HTTP server:\n"
			"     - port: %u\n",
			config_file, cfg.db.file, cfg.httpd.port);
		fprintf(stderr, " + nRF24L01 pinout :\n"
			"     - SPI device: %s\n"
			"     - IRQ GPIO: %d\n"
			"     - CE GPIO: %d\n",
			cfg.nrf.spi_dev, cfg.nrf.irq_gpio, cfg.nrf.ce_gpio);
	}

	/* Open database */
	db = db_open(&cfg.db, NULL, NULL);
	if (!db)
		return EXIT_FAILURE;

	/* Create a new nRF24L01 client */
	nrf = nrf24_new(&cfg.nrf);
	if (nrf) {
		/* Create a new controller */
		ctrl = controller_new(nrf, db, verbose);
		if (!ctrl)
			return EXIT_FAILURE;

		/* Start controller */
		controller_start(ctrl);

		/* Print debug stuff of controller */
		if (verbose) {
			controller_print_configuration(ctrl);
			controller_print_nrf24_configuration(ctrl);
		}
	} else {
		/* nRF24L01 not available */
		fprintf(stderr, "Warning: RF module is not available!\n");
	}

	/* Start HTTP server */
	if (httpd_start(&cfg.httpd))
		return EXIT_FAILURE;

	/* Install signal handler */
	signal(SIGINT, signal_handler);

	/* Start main loop */
	if (verbose)
		fprintf(stderr, "\nStart main loop...\n");

	/* Main loop */
	while (!main_loop_stop) {
		sleep(1);
	}

	/* Stop HTTP server */
	httpd_stop();

	/* Relase nRF24L01 */
	if (nrf) {
		/* Stop and free controller */
		controller_stop(ctrl);
		controller_free(ctrl);

		/* Free nRF24L01 client */
		nrf24_free(nrf);
	}

	/* Close database */
	db_close(db);

	return 0;
}
