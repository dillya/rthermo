dnl Autoconf settings for RThermo
AC_PREREQ([2.69])

dnl Initialize Autoconf for RThermo
AC_INIT([RThermo], [1.0.0], [http://www.sparod.com/rthermo/], [rthermo])

dnl Initialize Automake
AM_INIT_AUTOMAKE([subdir-objects])
AM_SILENT_RULES([yes])

dnl Set up macro directory
AC_CONFIG_MACRO_DIR([m4])

dnl Set source directory
AC_CONFIG_SRCDIR([src/main.c])

dnl Set output for config header
AC_CONFIG_HEADERS([config.h])

dnl Check for programs
AC_PROG_CC
AM_PROG_CC_C_O

dnl Check for header files
AC_HEADER_STDC

AC_CONFIG_FILES(
Makefile
include/Makefile
src/Makefile
)
AC_OUTPUT
