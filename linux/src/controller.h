/*
 * controller.h: RThermo main controller implementation
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

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "rthermo.h"
#include "nrf24.h"
#include "db.h"

/* Main controller handle */
struct controller;

/* Create a new main controller instance */
struct controller *controller_new(struct nrf24 *nrf, struct db *db,
				  int verbose);
void controller_free(struct controller *ctrl);

/* Start / stop main controller */
int controller_start(struct controller *ctrl);
int controller_stop(struct controller *ctrl);

/* Pairing mode control */
int controller_enable_pairing_mode(struct controller *ctrl);
int controller_disable_pairing_mode(struct controller *ctrl);
int controller_is_pairing_mode(struct controller *ctrl);

/* Target temperature control */
int controller_set_target(struct controller *ctrl, double target);
double controller_get_target(struct controller *ctrl);

/* Device control */
int controller_enable_device(struct controller *ctrl, unsigned int dev);
int controller_disable_device(struct controller *ctrl, unsigned int dev);
int controller_set_device_target(struct controller *ctrl, unsigned int dev,
				 double target);
double controller_get_device_target(struct controller *ctrl, unsigned int dev);

/* Debug / verbose */
void controller_print_configuration(struct controller *ctrl);
void controller_print_nrf24_configuration(struct controller *ctrl);

#endif /* _CONTROLLER_H_ */
