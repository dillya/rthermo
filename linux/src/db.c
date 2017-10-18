/*
 * db.c: Database handler for Temperature loging and settings storage
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
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <sqlite3.h>

#include "rthermo.h"
#include "db.h"

/* Default values */
#define DB_DEFAULT_MAIN_NAME	"Home"
#define DB_DEFAULT_MAIN_TARGET	19.0
#define DB_DEFAULT_CAPS		RTHERMO_DEVICE_CAPS_LOCAL_TEMPERATURE

/* Tables schema
 *
 * Configuration table: (config)
 *  - id: main controller (= 0)
 *        device ID (= 1 - 6)
 *  - name: controller / device name
 *  - address: controller / device address
 *  - serial: controller / device serial
 *  - caps: controller / device capabilities
 *  - enabled: device is enabled
 *  - target: local controller / device target temperature
 *  - control: control heater activation
 *  - local: use local temperature value for control
 *
 * Temperature table: (temperature)
 *  - id: main controller (= 0)
 *        device ID (= 1 - 6)
 *  - ts: timestamp of value
 *  - value: temperature value
 *  - status: status on controller / device
 */
#define DB_CREATE \
  "CREATE TABLE config (" \
  "        'id'         INTEGER PRIMARY KEY," \
  "        'name'       INTEGER," \
  "        'address'    BLOB," \
  "        'serial'     BLOB," \
  "        'caps'       INTEGER," \
  "        'enabled'    INTEGER," \
  "        'target'     REAL," \
  "        'control'    INTEGER," \
  "        'local'      INTEGER" \
  ");" \
  "CREATE TABLE temperature (" \
  "        'id'         INTEGER," \
  "        'ts'         INTEGER," \
  "        'value'      REAL," \
  "        'status'     INTEGER" \
  ");" \

struct db {
	char *file;
	sqlite3 *db;
};

struct db *db_open(struct db_config *cfg, uint8_t *address, uint8_t *serial)
{
	struct db *db;
	int ret;

	/* Allocate structure */
	db = malloc(sizeof(*db));
	if (!db)
		return NULL;

	/* Open database file */
	if (sqlite3_open(cfg->file, &db->db)) {
		fprintf(stderr, "Failed to open databse file\n");
		free(db);
		return NULL;
	}

	/* Fille structure */
	db->file = strdup(cfg->file);

	/* Create tables in database */
	ret = sqlite3_exec (db->db, DB_CREATE, NULL, NULL, NULL);

	/* Insert or get main controller address */
	if (!ret) {
		uint8_t main_address[RTHERMO_ADDRESS_WIDTH];
		uint8_t main_serial[RTHERMO_SERIAL_WIDTH];
		double target = DB_DEFAULT_MAIN_TARGET;
		char *name = DB_DEFAULT_MAIN_NAME;
		uint32_t caps = DB_DEFAULT_CAPS;
		int i;

		/* Generate a random main controller address and serial */
		srand(time(NULL));
		for (i = 0; i < RTHERMO_ADDRESS_WIDTH; i++)
			main_address[i] = rand() / (RAND_MAX / 255);
		for (i = 0; i < RTHERMO_SERIAL_WIDTH; i++)
			main_serial[i] = rand() / (RAND_MAX / 255);

		/* Store main controller address */
		ret = db_update_main_config(db, name, main_address, main_serial,
					    &caps, &target);
		if (ret) {
			fprintf(stderr, "Failed to init database!\n");
			goto error;
		}

		/* Copy address and serial */
		if (address)
			memcpy(address, main_address, RTHERMO_ADDRESS_WIDTH);
		if (serial)
			memcpy(serial, main_serial, RTHERMO_SERIAL_WIDTH);
	} else if (db_get_main_config(db, NULL, address, serial, NULL, NULL))
			goto error;

	return db;

error:
	sqlite3_close(db->db);
	free(db->file);
	free(db);
	return db;
}

void db_close(struct db *db)
{
	if (!db)
		return;

	/* Close database */
	sqlite3_close(db->db);
	free(db->file);
	free(db);
}

/* Database utility */
const char *db_get_file(struct db *db)
{
	return db->file;
}

/* Device managment */
int db_get_config(struct db *db, unsigned int dev, char **name,
		  uint8_t *address, uint8_t *serial, uint32_t *caps,
		  int *enabled, double *target, int *control, int *local)
{
	const unsigned char *text;
	sqlite3_stmt *req;
	int ret;

	/* Check device */
	if (dev > RTHERMO_MAX_DEVICES + 1)
		return -1;

	/* Prepare SQL request to get device configuration */
	ret = sqlite3_prepare_v2(db->db, "SELECT * FROM config WHERE id = ?",
				 -1, &req, 0);
	if (ret != SQLITE_OK)
		return -1;

	/* Set device index */
	sqlite3_bind_int(req, 1, dev);

	/* Get configuration */
	ret = sqlite3_step(req);
	if (ret != SQLITE_ROW) {
		sqlite3_finalize(req);
		return -1;
	}

	/* Copy values */
	if (name) {
		text = sqlite3_column_text(req, 1);
		if (text)
			*name = strdup(text);
	}
	if (address)
		memcpy(address, sqlite3_column_blob(req, 2),
		       RTHERMO_ADDRESS_WIDTH);
	if (serial)
		memcpy(serial, sqlite3_column_blob(req, 3),
		       RTHERMO_SERIAL_WIDTH);
	if (caps)
		*caps = sqlite3_column_int(req, 4);
	if (enabled)
		*enabled = sqlite3_column_int(req, 5);
	if (target)
		*target = sqlite3_column_double(req, 6);
	if (control)
		*control = sqlite3_column_int(req, 7);
	if (local)
		*local = sqlite3_column_int(req, 8);

	/* End of request */
	sqlite3_finalize(req);

	return 0;
}

int db_update_config(struct db *db, unsigned int dev, const char *name,
		     const uint8_t *address, const uint8_t *serial,
		     const uint32_t *caps, const int *enabled,
		     const double *target, const int *control, const int *local)
{
	sqlite3_stmt *req;
	char sql[512];
	int idx = 1;
	int ret;

	/* Check device */
	if (dev > RTHERMO_MAX_DEVICES + 1)
		return -1;

	/* Generate first SQL */
	snprintf(sql, 512, "INSERT OR REPLACE INTO config (id%s%s%s%s%s%s%s%s) "
		 "VALUES (?%s%s%s%s%s%s%s%s)",
		 name ? ",name" : "",
		 address ? ",address" : "",
		 serial ? ",serial" : "",
		 caps ? ",caps" : "",
		 enabled ? ",enabled" : "",
		 target ? ",target" : "",
		 control ? ",control" : "",
		 local ? ",local" : "",
		 name ? ",?" : "",
		 address ? ",?" : "",
		 serial ? ",?" : "",
		 caps ? ",?" : "",
		 enabled ? ",?" : "",
		 target ? ",?" : "",
		 control ? ",?" : "",
		 local ? ",?" : "");

	/* Prepare SQL request to update device configuration */
	ret = sqlite3_prepare_v2(db->db, sql, -1, &req, 0);
	if (ret != SQLITE_OK)
		return -1;

	/* Feed SQL request */
	sqlite3_bind_int(req, idx++, dev);
	if (name)
		sqlite3_bind_text(req, idx++, name, -1, SQLITE_STATIC);
	if (address)
		sqlite3_bind_blob(req, idx++, address, RTHERMO_ADDRESS_WIDTH,
				  SQLITE_STATIC);
	if (serial)
		sqlite3_bind_blob(req, idx++, serial, RTHERMO_SERIAL_WIDTH,
				  SQLITE_STATIC);
	if (caps)
		sqlite3_bind_int(req, idx++, *caps);
	if (enabled)
		sqlite3_bind_int(req, idx++, *enabled);
	if (target)
		sqlite3_bind_double(req, idx++, *target);
	if (control)
		sqlite3_bind_int(req, idx++, *control);
	if (local)
		sqlite3_bind_int(req, idx++, *local);

	/* Send SQL request */
	ret = sqlite3_step(req);
	sqlite3_finalize(req);

	return (ret == SQLITE_DONE) ? 0 : -1;
}

int db_count_device(struct db *db)
{
	sqlite3_stmt *req;
	int count = -1;

	/* Prepare SQL request to count devices */
	if (sqlite3_prepare_v2(db->db, "SELECT COUNT(*) FROM config", -1, &req, 0))
		return -1;

	/* Get count */
	if (sqlite3_step(req) == SQLITE_ROW)
		count = sqlite3_column_int(req, 0) - 1;
	sqlite3_finalize(req);

	return count;
}

int db_find_next_free_device(struct db *db)
{
	sqlite3_stmt *req;
	int id = 0, last_id = 1;

	/* Prepare SQL request to count devices */
	if (sqlite3_prepare_v2(db->db, "SELECT id FROM config WHERE id > 0 "
			       "ORDER BY id ASC", -1, &req, 0))
		return -1;

	/* Find first free ID */
	while (sqlite3_step(req) == SQLITE_ROW) {
		id = sqlite3_column_int(req, 0);
		if (last_id < id)
			break;
		last_id++;
	}
	sqlite3_finalize(req);

	return last_id != id && last_id < RTHERMO_MAX_DEVICES ? last_id : -1;
}

int db_remove_device(struct db *db, unsigned int dev)
{
	sqlite3_stmt *req;
	int ret;

	/* Check device */
	if (!dev || dev > RTHERMO_MAX_DEVICES + 1)
		return -1;

	/* Prepare SQL request to remove device */
	ret = sqlite3_prepare_v2(db->db, "DELETE FROM config WHERE id = ?",
				 -1, &req, 0);
	if (ret != SQLITE_OK)
		return -1;

	/* Set device for SQL request */
	sqlite3_bind_int(req, 1, dev);

	/* Send SQL request */
	ret = sqlite3_step(req);
	sqlite3_finalize(req);

	return (ret == SQLITE_DONE) ? 0 : -1;
}

/* Temperature managment */
int db_get_temperature(struct db *db, unsigned int dev, uint64_t start_ts,
		       uint64_t end_ts, db_temperature_cb cb, void *data)
{
	sqlite3_stmt *req;
	int ret;

	/* Check callback and device */
	if (!cb || dev > RTHERMO_MAX_DEVICES + 1)
		return -1;

	/* Prepare SQL request to get temperature range */
	ret = sqlite3_prepare_v2(db->db, "SELECT ts,value,status FROM temperature "
				 "WHERE id = ? AND ts >= ? AND < ?",
				 -1, &req, 0);
	if (ret != SQLITE_OK)
		return -1;

	/* Set device index and timestamp range */
	sqlite3_bind_int(req, 1, dev);
	sqlite3_bind_int64(req, 2, start_ts);
	sqlite3_bind_int64(req, 3, end_ts);

	/* Get temperature range */
	while ((ret = sqlite3_step(req)) == SQLITE_ROW) {
		if (cb(dev, sqlite3_column_int64(req, 0),
		       sqlite3_column_double(req, 1),
		       sqlite3_column_int(req, 2)))
			break;
	}

	/* End of request */
	sqlite3_finalize(req);

	return (ret == SQLITE_DONE) ? 0 : -1;
}

int db_set_temperature(struct db *db, unsigned int dev, uint64_t ts,
		       double value, uint32_t status)
{
	sqlite3_stmt *req;
	int ret;

	/* Prepare SQL request */
	ret = sqlite3_prepare_v2(db->db, "INSERT INTO temperature "
				 "(ts,value,status) VALUES (?,?,?)",
				 -1, &req, 0);
	if (ret != SQLITE_OK)
		return -1;

	/* Bind values for request */
	sqlite3_bind_int64(req, 1, ts);
	sqlite3_bind_double(req, 2, value);
	sqlite3_bind_int(req, 3, status);

	/* Send SQL request */
	ret = sqlite3_step(req);
	sqlite3_finalize(req);

	return (ret == SQLITE_DONE) ? 0 : -1;
}
