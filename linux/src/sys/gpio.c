/*
 * gpio.c: GPIO class sysfs helper
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
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "gpio.h"

#define EXPORT_SIZE	16
#define GPIO_SIZE	64

int gpio_export(int gpio)
{
	char val[EXPORT_SIZE];
	ssize_t len;
	int fd;

	/* GPIO is already exported */
	if (gpio_is_exported(gpio))
		return 0;

	/* Open export */
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd < 0)
		return fd;

	/* Write GPIO number */
	len = snprintf(val, EXPORT_SIZE, "%u", gpio);
	len = write(fd, val, len);

	/* Close export */
	close(fd);

	/* Wait until GPIO is available */
	usleep(100000);

	return len > 0 ? 0 : len;
}

int gpio_unexport(int gpio)
{
	char val[EXPORT_SIZE];
	ssize_t len;
	int fd;

	/* Open unexport */
	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd < 0)
		return fd;

	/* Write GPIO number */
	len = snprintf(val, EXPORT_SIZE, "%u", gpio);
	len = write(fd, val, len);
	close(fd);

	return len > 0 ? 0 : len;
}

int gpio_is_exported(int gpio)
{
	struct stat st;
	char path[GPIO_SIZE];

	/* Check if GPIO is exported */
	snprintf(path, GPIO_SIZE, "/sys/class/gpio/gpio%u", gpio);
	return !stat(path, &st);
}

int gpio_set_direction(int gpio, enum gpio_direction dir)
{
	const char *val = (dir == GPIO_DIR_OUTPUT) ? "out" : "in";
	char path[GPIO_SIZE];
	ssize_t len;
	int fd;

	/* Open GPIO direction */
	snprintf(path, GPIO_SIZE, "/sys/class/gpio/gpio%u/direction", gpio);
	fd = open(path, O_WRONLY);
	if (fd < 0)
		return fd;

	/* Set direction */
	len = write(fd, val, strlen(val));
	close(fd);

	return len > 0 ? 0 : len;
}

int gpio_value_open(int gpio)
{
	char path[GPIO_SIZE];

	/* Open GPIO value */
	snprintf(path, GPIO_SIZE, "/sys/class/gpio/gpio%u/value", gpio);
	return open(path, O_RDWR);
}

int gpio_value_read(int fd, int *value)
{
	char val[2];
	ssize_t len;

	/* Read GPIO value */
	lseek(fd, SEEK_SET, 0);
	len = read(fd, val, 2);
	*value = (*val == '1') ? 1 : 0;

	return len > 0 ? 0 : len;
}

int gpio_value_write(int fd, int value)
{
	const char *val = value ? "1" : "0";
	ssize_t len;

	/* Write GPIO value */
	len = write(fd, val, 2);

	return len > 0 ? 0 : len;
}

void gpio_value_close(int fd)
{
	if (fd > 0)
		close(fd);
}

int gpio_read(int gpio, int *value)
{
	char val[2];
	ssize_t len;
	int fd;

	/* Open GPIO value */
	fd = gpio_value_open(gpio);
	if (fd < 0)
		return fd;

	/* Read GPIO value */
	len = read(fd, val, 2);
	*value = (*val == '1') ? 1 : 0;
	close(fd);

	return len > 0 ? 0 : len;
}

int gpio_write(int gpio, int value)
{
	const char *val = value ? "1" : "0";
	char path[GPIO_SIZE];
	ssize_t len;
	int fd;

	/* Open GPIO value */
	fd = gpio_value_open(gpio);
	if (fd < 0)
		return fd;

	/* Set value */
	len = write(fd, val, 2);
	close(fd);

	return len > 0 ? 0 : len;
}

int gpio_irq_setup(int gpio, enum gpio_irq_mode mode)
{
	char path[GPIO_SIZE];
	const char *val;
	ssize_t len;
	int fd, ret;

	/* Setup pin as input */
	ret = gpio_set_direction(gpio, GPIO_DIR_INPUT);
	if (ret)
		return ret;

	/* Open GPIO edge */
	snprintf(path, GPIO_SIZE, "/sys/class/gpio/gpio%u/edge", gpio);
	fd = open(path, O_RDWR);
	if (fd < 0)
		return fd;

	/* Select mmode */
	switch (mode) {
		case GPIO_IRQ_MODE_NONE:
			val = "none";
			break;
		case GPIO_IRQ_MODE_RISING:
			val = "rising";
			break;
		case GPIO_IRQ_MODE_FALLING:
			val = "falling";
			break;
		case GPIO_IRQ_MODE_BOTH:
			val = "both";
			break;
		default:
			return -1;
	}

	/* Set edge */
	len = write(fd, val, strlen(val));
	close(fd);

	return len > 0 ? 0 : len;
}

int gpio_irq_wait(int fd, int *value, int timeout)
{
	struct pollfd fds;
	int val, ret;

	/* Setup events */
	fds.fd = fd;
	fds.events = POLLPRI | POLLERR;

	/* Wait for newt interrupt */
	ret = poll(&fds, 1, timeout);
	if (ret <= 0) {
		if (!ret)
			return GPIO_IRQ_TIMEOUT;
		return ret;
	}

	/* Read value */
	if (fds.revents & POLLPRI) {
		ret = gpio_value_read(fd, &val);
		if (value)
			*value = val;
		return ret;
	}

	return -1;
}
