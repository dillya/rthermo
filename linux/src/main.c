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

static int main_loop_stop = 0;

static void signal_handler(int sig)
{
	main_loop_stop = 1;
}

static void print_usage(const char *name)
{
	fprintf(stderr, "Usage: %s\n"
		"\n"
		"Supported options:\n"
		" -d --daemon           Run as daemon\n"
		" -h --help             Print this help\n"
		" -v --verbose          Enable verbose output\n",
		name);
}

int main(int argc, char *argv[])
{
	int daemonize = 0;
	int verbose = 0;

	/* Parse options */
	while (1) {
		static const char *short_options = "dhv";
		static struct option long_options[] = {
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

	/* Daemonize */
	if (daemonize && daemon (1, 0)) {
		fprintf(stderr, "Failed to daemonize!");
		return EXIT_FAILURE;
	}

	/* Install signal handler */
	signal(SIGINT, signal_handler);

	/* Start main loop */
	if (verbose)
		fprintf(stderr, "\nStart main loop...\n");

	/* Main loop */
	while (!main_loop_stop) {
		sleep(1);
	}

	return 0;
}
