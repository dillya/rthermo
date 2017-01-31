/*
 * httpd.h: HTTP server
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

#ifndef _HTTPD_H_
#define _HTTPD_H_

#include <stdint.h>

/* HTTP server configuration */
struct httpd_config {
	uint16_t port;
};

/* Start / stop HTTP server */
int httpd_start(struct httpd_config *cfg);
void httpd_stop();

#endif /* _HTTPD_H_ */
