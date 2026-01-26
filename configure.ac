#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.71])
AC_INIT([wanhive-pug], [1.0.0], [info@wanhive.com], , [www.wanhive.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])
AC_CONFIG_SRCDIR([src/wanhive-pug.h])
AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_MACRO_DIRS([m4])
PKG_INSTALLDIR

# Release year
INCEPTION_YEAR='2026'
AC_SUBST([INCEPTION_YEAR])
AC_DEFINE_UNQUOTED([INCEPTION_YEAR], ["${INCEPTION_YEAR}"], [Release year])

# Library version
# https://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
WH_LIB_VERSION='0:0:0'
AC_SUBST([WH_LIB_VERSION])
AC_DEFINE_UNQUOTED([WH_LIB_VERSION], ["${WH_LIB_VERSION}"], [Library version])

# Checks for programs.
AC_PROG_CXX([g++])
AC_PROG_CC([gcc])
## https://www.gnu.org/software/autoconf-archive/ax_cxx_compile_stdcxx.html
AX_CXX_COMPILE_STDCXX(17, [noext], [mandatory])
AM_PROG_AR

# Checks for libraries.
LT_INIT
AC_CHECK_LIB([gps], [gps_stream], , [AC_MSG_ERROR([libgps not found.])])
AC_CHECK_LIB([i2c], [i2c_smbus_process_call], , [AC_MSG_ERROR([libi2c not found.])])
AC_CHECK_LIB([jpeg], [jpeg_std_error], , [AC_MSG_ERROR([libjpeg not found.])])
AC_CHECK_LIB([m], [sqrt], , [AC_MSG_ERROR([libm not found.])])

# Check for packages.
WHUB_VERSION=17.0.0
AC_SUBST(WHUB_VERSION)
LIBCAM_VERSION=0.2.0
AC_SUBST(LIBCAM_VERSION)

PKG_CHECK_MODULES([WHUB], [wanhive >= $WHUB_VERSION], , [AC_MSG_ERROR([libwanhive not found.])])
PKG_CHECK_MODULES([LIBCAM], [libcamera >= $LIBCAM_VERSION], , [AC_MSG_ERROR([libcamera not found.])])

# Checks for header files.
AC_CHECK_HEADERS([sys/ioctl.h termios.h unistd.h], , [AC_MSG_ERROR([Header files missing.])])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_INT8_T
AC_TYPE_SIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_MMAP
AC_CHECK_FUNCS([memset munmap pow select sqrt])

# Doxygen configuration file generation
AC_CHECK_PROG([HAVE_DOXYGEN], [doxygen], [yes], [no])
AS_IF([test "x$HAVE_DOXYGEN" = "xyes"], [
		AC_CONFIG_FILES([Doxyfile])
		AC_CHECK_PROG([HAVE_DOT], [dot], [YES], [NO])
		AC_SUBST([HAVE_DOT])
	], [AC_MSG_WARN([Doxygen not found.])])
AM_CONDITIONAL([WH_HAVE_DOXYGEN], [test "x$HAVE_DOXYGEN" = "xyes"])

AC_CONFIG_FILES([Makefile docs/Makefile src/Makefile src/wanhive-pug.pc])
AC_OUTPUT
