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
 * $Id: safeString_arch.h 6285 2014-01-09 18:16:42Z gulati $
 */

/**
 * \file
 *         Safe string handling abstraction (architecture specific part:
 *         Posix implementation).
 */

#ifndef XME_HAL_SAFESTRING_ARCH_H
#define XME_HAL_SAFESTRING_ARCH_H

#ifndef XME_HAL_SAFESTRING_H
	#error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_SAFESTRING_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <string.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
static INLINE
int
xme_hal_safeString_vscprintf
(
	const char* format,
	va_list args
)
{
	return xme_fallback_vsnprintf(NULL, 0, format, args);
}

static INLINE
int
xme_hal_safeString_vsnprintf
(
	char* buffer,
	size_t sizeInBytes,
	const char* format,
	va_list args
)
{
	return xme_fallback_vsnprintf(buffer, sizeInBytes, format, args);
}

static INLINE
char*
xme_hal_safeString_strncpy
(
	char* strDest,
	const char* strSource,
	size_t sizeInBytes
)
{
	char* result;

	// "strDest may be NULL if sizeInBytes is zero"
	XME_CHECK(0 != sizeInBytes, strDest);

	result = xme_fallback_strncpy(strDest, strSource, sizeInBytes);

	// Make sure the string is NULL-terminated, possibly truncating it
	strDest[sizeInBytes-1] = 0;

	return result;
}

static INLINE
char*
xme_hal_safeString_strncat
(
	char* strDest,
	const char* strSource,
	size_t sizeInBytes
)
{
	size_t length;

	// "strDest may be NULL if sizeInBytes is zero"
	XME_CHECK(0 != sizeInBytes, strDest);

	// Determine current length of string
	length = xme_fallback_strnlen(strDest, sizeInBytes);

	// A return value of sizeInBytes indicates that the string
	// contained in strDest is longer than the given size or
	// it is not NULL-terminated. Hence, there is no room left
	// to append strSource.
	if (sizeInBytes == length)
	{
		// Truncate the string
		strDest[sizeInBytes-1] = 0;
		return strDest;
	}
	else
	{
		// Copy only the characters that fit the remaining space
		// in the buffer, excluding the terminating NULL character.
		return xme_fallback_strncat(strDest, strSource, sizeInBytes-(length+1));
	}
}

static INLINE
size_t
xme_hal_safeString_strnlen
(
	const char *str,
	size_t maxlen
)
{
	return xme_fallback_strnlen(str, maxlen);
}

#endif // #ifndef XME_HAL_SAFESTRING_ARCH_H
