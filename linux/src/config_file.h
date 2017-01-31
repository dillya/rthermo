/*
 * config_file.h: configuration file handler
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

#ifndef _CONFIG_FILE_H_
#define _CONFIG_FILE_H_

#include <stdint.h>

#include "rthermo.h"

#include "httpd.h"
#include "nrf24.h"
#include "db.h"

//#define DEFAULT_CONFIG_FILE	"/etc/rthermo.cfg"
#define DEFAULT_CONFIG_FILE	"configs/rpi3.cfg"
#define DEFAULT_DB_FILE		"/etc/rthermo.db"
#define DEFAULT_HTTP_PORT	8080

/* Global configuration */
struct config {
	struct db_config db;
	struct httpd_config httpd;
	struct nrf24_config nrf;
};

/* Load configuration from a configuration file */
int config_file_load(const char *name, struct config *cfg);

#endif /* _CONFIG_FILE_H_ */
