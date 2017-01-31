bin_PROGRAMS = rthermo

rthermo_SOURCES = \
	sys/spidev.c \
	sys/gpio.c \
	config_file.c \
	controller.c \
	httpd.c \
	nrf24.c \
	db.c \
	main.c

rthermo_CFLAGS = \
	$(SQLITE_CFLAGS) \
	$(LIBCONFIG_CFLAGS) \
	-I$(top_srcdir)/include

rthermo_LDADD = \
	$(SQLITE_LIBS) \
	$(LIBCONFIG_LIBS) \
	-lpthread

noinst_HEADERS = \
	sys/spidev.h \
	sys/gpio.h \
	config_file.h \
	controller.h \
	httpd.h \
	nrf24.h \
	db.h
