#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([NTPC-comm],[0.0.0.1],[morpheby@gmail.com])

AC_SUBST([VERSION_4D], ["0.0.0.1"])
AC_DEFINE([PACKAGE_VERSION_NUM], [0,0,0,1],
	[Define to the version number (acceptable by rc or windres) of this package.])
AC_DEFINE([VERSION_NUM], PACKAGE_VERSION_NUM,
	[Version number (acceptable by rc or windres) of this package.])
AC_DEFINE([PACKAGE_IS_RELEASE], [0], [Define to 1 if package is for release.])

AC_DEFINE([PACKAGE_AUTHOR], ["Morpheby"], [Author of the package])

AC_DEFINE([PACKAGE_COPYRIGHT], ["Copyright (C) 2012 " PACKAGE_AUTHOR ". Licensed under GNU GPL"], [Copyright])

AC_ARG_ENABLE([debug],
[  --enable-debug    Turn on debugging],
[case "${enableval}" in
  yes)
	AC_DEFINE([DEBUG], [], ["Use for building with debug information"])
  ;;
  no) ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-debug]) ;;
esac],[])

AC_CANONICAL_TARGET
if [[ ${target_cpu} == x86_64 ]]; then
	cxx_target=-m64
else
	cxx_target=-m32
fi

if [[ ${target_os} == mingw32 ]]; then
	cxx_target=${cxx_target} -mconsole -municode
fi

AC_CONFIG_SRCDIR([src/main.cpp])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_AUX_DIR([.])
AM_INIT_AUTOMAKE

case $target_os in
  mingw32) COMMHANDLER_SYSTEM='CommHandler-windows.$(OBJEXT)' ;;
  *)       COMMHANDLER_SYSTEM='CommHandler-unix.$(OBJEXT)' ;;
esac
AC_SUBST([COMMHANDLER_SYSTEM])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_RANLIB
AC_PROG_MKDIR_P
AC_PROG_LN_S
AC_PROG_SED
AM_PROG_AR
AC_PROG_INSTALL

CFLAGS="${CFLAGS} ${cxx_target}"
LDFLAGS="${LDFLAGS} ${cxx_target}"


# Checks for libraries.

# there shuold be checks for uxtheme, gdi32 and ole32,
# but neither of them has cdecl functions....

# Checks for header files.
AC_CHECK_HEADERS([memory.h stdlib.h string.h tchar.h \
	windows.h windowsx.h termios.h fcntl.h sys/ioctl.h \
	errno.h])

# MinGW commctrl.h may be non-self-compilable and
# demands inclusion of windows.h
AC_CHECK_HEADERS([commctrl.h uxtheme.h], [], [], [
	#if HAVE_WINDOWS_H
	#	include <windows.h>
	#endif
	])

# Check for c++11 compatible compiler
AC_LANG([C++])
AX_CXX_COMPILE_STDCXX_11([], \
                         [
                          AC_MSG_ERROR([
****************************************************************
**************** Only C++11 is supported ***********************
****************************************************************
]) ])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T
AC_CHECK_TYPES([ptrdiff_t])

# Checks for library functions.
#AC_FUNC_MALLOC
# malloc test doesn't check if we are cross-compling, and thus may actually fail...
AC_CHECK_FUNCS([memset open close tcgetattr read write cfsetispeed cfmakeraw strerror])




AC_CONFIG_FILES([	Makefile
					src/Makefile
				])
AC_OUTPUT
