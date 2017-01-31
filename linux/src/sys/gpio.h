/*
 * gpio.h: GPIO class sysfs helper
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

#ifndef _GPIO_H_
#define _GPIO_H_

enum gpio_direction {
	GPIO_DIR_INPUT = 0,      /* Set as input */
	GPIO_DIR_OUTPUT          /* Set as output */
};

enum gpio_irq_mode {
	GPIO_IRQ_MODE_NONE = 0,  /* IRQ handling is disabled */
	GPIO_IRQ_MODE_RISING,    /* Trigger IRQ on rising edge */
	GPIO_IRQ_MODE_FALLING,   /* Trigger IRQ on falling edge */
	GPIO_IRQ_MODE_BOTH       /* Trigger IRQ on both edges */
};

/* Export / Unexport GPIO to user space
 *
 * In order to use GPIOs under Linux user space, developer must export GPIO
 * from Kernel space before changing direction, reading or writting values, ...
 */
int gpio_export(int gpio);
int gpio_unexport(int gpio);
int gpio_is_exported(int gpio);

/* Setup GPIO direction
 *
 * This function allows to set direction of GPIO by setting dir with:
 *  - GPIO_DIR_INPUT to set as input,
 *  - GPIO_DIR_OUTPUT to set as output.
 */
int gpio_set_direction(int gpio, enum gpio_direction dir);

/* Read / write operation
 *
 * First user must open a GPIO value with gpio_value_open(). It returns a file
 * descriptor which can be passed to three other functions.
 *
 * To read/write a value from/to, user can respectively use gpio_value_read()
 * and gpio_value_wrtie().
 *
 * When all operations on GPIO are finished, the file descriptor must be closed
 * with gpio_value_close().
 */
int gpio_value_open(int gpio);
int gpio_value_read(int fd, int *value);
int gpio_value_write(int fd, int value);
void gpio_value_close(int fd);

/* Helper for read / write operation
 *
 * This two functions allows to respectively do a read or a write to a GPIO in
 * only one call. It can be used when operations are not periodic.
 */
int gpio_read(int gpio, int *value);
int gpio_write(int gpio, int value);

/* GPIO IRQ handling
 *
 * To set IRQ triggering, user can use gpio_irq_setup(), to which a mode is
 * passed:
 *  - GPIO_IRQ_MODE_NONE to disable IRQ,
 *  - GPIO_IRQ_MODE_RISING to trigger IRQ on rising edge,
 *  - GPIO_IRQ_MODE_FALLING to trigger IRQ on falling edge,
 *  - GPIO_IRQ_MODE_BOTH to rigger IRQ on both edges.
 *
 * When IRQ is set, user can wait IRQ by doing a call to poll(2) or select(2)
 * with the file descriptor returned by gpio_value_open().
 *
 * To wait next IRQ, user can call gpio_irq_wait() which returns when an IRQ is
 * triggered, if a timeout has been reached (return GPIO_IRQ_TIMEOUT) or if an
 * error has occured (return negative value).
 * The timeout is in ms. If its value is 0, the function returns immediatly, if
 * its value is negative, it waits forever the IRQ.
 */
#define GPIO_IRQ_TIMEOUT	1
int gpio_irq_setup(int gpio, enum gpio_irq_mode mode);
int gpio_irq_wait(int fd, int *value, int timeout);

#endif /* _GPIO_H_ */
