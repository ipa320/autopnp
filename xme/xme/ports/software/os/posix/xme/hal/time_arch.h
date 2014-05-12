/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: time_arch.h 2321 2013-02-01 18:21:50Z geisinger $
 */

/**
 * \file
 *         Time abstraction (architecture specific part: POSIX implementation).
 */

#ifndef XME_HAL_TIME_ARCH_H
#define XME_HAL_TIME_ARCH_H

#ifndef XME_HAL_TIME_H
	#error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_TIME_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <time.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
// Documented in xme/hal/time.h
typedef struct timespec xme_hal_time_timeHandle_t;

#endif // #ifndef XME_HAL_TIME_ARCH_H
