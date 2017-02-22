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
 * $Id: safeString_arch.c 4880 2013-08-30 13:07:39Z camek $
 */

/**
 * \file
 *         Safe string handling abstraction (architecture specific part:
 *         POSIX implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/safeString.h"

#include <stdarg.h>
#include <stdio.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
int
xme_hal_safeString_scprintf
(
	const char* format,
	...
)
{
	va_list argp;
	int result;

	va_start(argp, format);
	result = xme_fallback_vsnprintf(NULL, 0, format, argp);
	va_end(argp);

	return result;
}

int xme_hal_safeString_snprintf
(
	char* buffer,
	size_t sizeInBytes,
	const char* format,
	...
)
{
	va_list args;
	int result;

	va_start(args, format);
	result = xme_fallback_vsnprintf(buffer, sizeInBytes, format, args);
	va_end(args);

	return result;
}
