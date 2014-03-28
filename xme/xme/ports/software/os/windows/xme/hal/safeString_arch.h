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
 *         Windows implementation).
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
    return xme_fallback__vscprintf(format, args);
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
    return xme_fallback__vsnprintf_s(buffer, sizeInBytes, _TRUNCATE, format, args);
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
    // "strDest may be NULL if sizeInBytes is zero"
    XME_CHECK(0 != sizeInBytes, strDest);

    // From MSDN (<http://msdn.microsoft.com/en-us/library/5dae5d43%28v=VS.80%29.aspx>):
    // "Note that unlike strncpy, if count is greater than the length of strSource,
    // the destination string is NOT padded with null characters up to length count."
    // Since there is no way to efficiently find out the number of copied bytes from
    // the function call below, we designed this API in a way that it does not
    // guarantee the excessive characters in strDest to be zeroed out. Hence, the
    // behavior of xme_hal_safeString_strncpy() differs from strncpy() in this respect!

    // Discard error indicator
    xme_fallback_strncpy_s(strDest, sizeInBytes, strSource, _TRUNCATE);

    return strDest;
}

static INLINE
char*
xme_hal_safeString_strncat
(
    char *strDest,
    const char *strSource,
    size_t sizeInBytes
)
{
    // "strDest may be NULL if sizeInBytes is zero"
    XME_CHECK(0 != sizeInBytes, strDest);

    // Discard error indicator
    xme_fallback_strncat_s(strDest, sizeInBytes, strSource, _TRUNCATE);

    return strDest;
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
