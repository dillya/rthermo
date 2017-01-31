/*
 * db.h: Database handler for Temperature loging and settings storage
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

#ifndef _DB_H_
#define _DB_H_

#include <stdint.h>

/* Database configuration */
struct db_config {
	char *file;
};

/* Database handler */
struct db;

/* Callback for temperature grab */
typedef int *(db_temperature_cb)(unsigned int dev, uint64_t ts, double value,
				 uint32_t control);

/* Open / close database file */
struct db *db_open(struct db_config *cfg, uint8_t *address, uint8_t *serial);
void db_close(struct db *db);

/* Database utility */
const char *db_get_file(struct db *db);

/* Device managment */
int db_get_config(struct db *db, unsigned int dev, char **name,
		  uint8_t *address, uint8_t *serial, uint32_t *caps,
		  int *enabled, double *target, int *control, int *local);
#define db_get_name(db,dev,name) \
	db_get_config(db,dev,name,NULL,NULL,NULL,NULL,NULL,NULL,NULL)
#define db_get_address(db,dev,address) \
	db_get_config(db,dev,NULL,address,NULL,NULL,NULL,NULL,NULL,NULL)
#define db_get_serial(db,dev,serial) \
	db_get_config(db,dev,NULL,NULL,serial,NULL,NULL,NULL,NULL,NULL)
#define db_get_caps(db,dev,caps) \
	db_get_config(db,dev,NULL,NULL,NULL,caps,NULL,NULL,NULL,NULL)
#define db_get_enabled(db,dev,enabled) \
	db_get_config(db,dev,NULL,NULL,NULL,NULL,enabled,NULL,NULL,NULL)
#define db_get_target(db,dev,target) \
	db_get_config(db,dev,NULL,NULL,NULL,NULL,NULL,target,NULL,NULL)
#define db_get_control(db,dev,control) \
	db_get_config(db,dev,NULL,NULL,NULL,NULL,NULL,NULL,control,NULL)
#define db_get_local(db,dev,local) \
	db_get_config(db,dev,NULL,NULL,NULL,NULL,NULL,NULL,NULL,local)

#define db_get_main_config(db,name,address,serial,caps,target) \
	db_get_config(db,0,name,address,serial,caps,NULL,target,NULL,NULL)

int db_update_config(struct db *db, unsigned int dev, const char *name,
		     const uint8_t *address, const uint8_t *serial,
		     const uint32_t *caps, const int *enabled,
		     const double *target, const int *control,
		     const int *local);
#define db_update_name(db,dev,name) \
	db_update_config(db,dev,name,NULL,NULL,NULL,NULL,NULL,NULL,NULL)
#define db_update_address(db,dev,address) \
	db_update_config(db,dev,NULL,address,NULL,NULL,NULL,NULL,NULL,NULL)
#define db_update_serial(db,dev,serial) \
	db_update_config(db,dev,NULL,NULL,serial,NULL,NULL,NULL,NULL,NULL)
#define db_update_caps(db,dev,caps) \
	db_update_config(db,dev,NULL,NULL,NULL,caps,NULL,NULL,NULL,NULL)
#define db_update_enabled(db,dev,enabled) \
	db_update_config(db,dev,NULL,NULL,NULL,NULL,enabled,NULL,NULL,NULL)
#define db_update_target(db,dev,target) \
	db_update_config(db,dev,NULL,NULL,NULL,NULL,NULL,target,NULL,NULL)
#define db_update_control(db,dev,control) \
	db_update_config(db,dev,NULL,NULL,NULL,NULL,NULL,NULL,control,NULL)
#define db_update_local(db,dev,local) \
	db_update_config(db,dev,NULL,NULL,NULL,NULL,NULL,NULL,NULL,local)

#define db_update_main_config(db,name,address,serial,caps,target) \
	db_update_config(db,0,name,address,serial,caps,NULL,target,NULL,NULL)

int db_count_device(struct db *db);
int db_find_next_free_device(struct db *db);
int db_remove_device(struct db *db, unsigned int dev);

/* Temperature managment */
int db_get_temperature(struct db *db, unsigned int dev, uint64_t start_ts,
		       uint64_t end_ts, db_temperature_cb cb, void *data);
int db_set_temperature(struct db *db, unsigned int dev, uint64_t ts,
		       double value, uint32_t status);

#endif /* _DB_H_ */
