#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.71])
AC_INIT([wanhive-pug], [0.0.1], [info@wanhive.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([foreign subdir-objects])
AC_CONFIG_SRCDIR([src/wanhive-pug.h])
AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_MACRO_DIRS([m4])


# Library version
# https://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
WH_LIB_VERSION='0:0:0'
AC_SUBST([WH_LIB_VERSION])
AC_DEFINE_UNQUOTED([WH_LIB_VERSION], ["${WH_LIB_VERSION}"], [Library's version number])

# Checks for programs.
AC_PROG_CXX([g++])
AC_PROG_CC([gcc])
## https://www.gnu.org/software/autoconf-archive/ax_cxx_compile_stdcxx.html
AX_CXX_COMPILE_STDCXX(17, [noext], [mandatory])

# Checks for libraries.
LT_INIT
AC_CHECK_LIB([i2c], [i2c_smbus_process_call], , [AC_MSG_ERROR([libi2c not found.])])
# nm <lib>.so | grep <method_name>
AC_CHECK_LIB([wanhive], [_ZNK7wanhive9Exception4whatEv], , [AC_MSG_ERROR([libwanhive not found.])])

# Checks for header files.
AC_CHECK_HEADERS([sys/ioctl.h termios.h unistd.h], , [AC_MSG_ERROR([Header files missing.])])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_CHECK_FUNCS([memset])

AC_CONFIG_FILES([Makefile docs/Makefile src/Makefile])
AC_OUTPUT
