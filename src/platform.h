/*
 * platform.h
 *
 *  Created on: 23.08.2013
 *      Author: morpheby
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

#include "config.h"

#if defined(_WIN32) || defined(__MINGW32__)

#define WINVER 0x0501
#define WIN32_IE 0x600
#define WINDOWS_ISOLATION_AWARE_ENABLED 1

#ifdef DEBUG
#define _DEBUG
#endif

#define UINCODE
#define _UNICODE

#define WIN32_LEAN_AND_MEAN

#define COMM_USE_WINDOWS_BACKEND

#endif /* WIN32 */

#if defined(__APPLE__) && defined(__MACH__)

/* CMSPAR is not supported on OS X */
#ifndef NO_SPACEMARK_PARITY
#define NO_SPACEMARK_PARITY
#endif

#endif /* APPLE */


#if defined(__linux__)

#ifdef NO_SPACEMARK_PARITY
#undef NO_SPACEMARK_PARITY
#endif

#endif /* LINUX */

#endif /* PLATFORM_H_ */
