/*
 * httpd.c: HTTP server
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

#include "httpd.h"

/* Global variables */
static int started = 0;

int httpd_start(struct httpd_config *cfg)
{
	/* HTTP server already started */
	if (started)
		return -1;

	/* Start HTTP server */
	started = 1;

	return 0;
}

void httpd_stop()
{
	if (!started)
		return;
	started = 0;
}
