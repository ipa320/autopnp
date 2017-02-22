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
 * $Id: safeString_arch.c 4971 2013-09-04 13:24:38Z ruiz $
 */

/**
 * \file
 *         Safe string handling abstraction (architecture specific part:
 *         Windows implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/safeString.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#ifndef _TRUNCATE
#define _TRUNCATE ((size_t)-1) ///< the truncate size.
#endif

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
    int formattedMsgLen;
    va_list argp;

    va_start(argp, format);
    formattedMsgLen = xme_fallback__vscprintf(format, argp);
    va_end(argp);

    return formattedMsgLen;
}

int
xme_hal_safeString_snprintf
(
    char* buffer,
    size_t sizeInBytes,
    const char* format,
    ...
)
{
    size_t formattedMsgLen;
    va_list argp;
    int result;

    va_start(argp, format);
    formattedMsgLen = xme_fallback__vscprintf(format, argp);
    va_end(argp);

    if ((formattedMsgLen + 1) > sizeInBytes)
    {
        // Buffer size is not sufficient to hold formatted string and terminating null character.
        // Put the truncated formatted string (with place for the terminating null character)
        // into the buffer and return the length that the fully formatted string would have had
        // (excluding the terminating null character).
        if (NULL != buffer)
        {
            va_start(argp, format);
            xme_fallback__vsnprintf_s(buffer, sizeInBytes, _TRUNCATE, format, argp);
            va_end(argp);
        }
        return formattedMsgLen;
    }
    else
    {
        // Buffer size is sufficient, sprintf_s() behavior complies to xme_hal_safeString_snprintf()
        // specification.
        va_start(argp, format);
        result = xme_fallback_vsprintf_s(buffer, sizeInBytes, format, argp);
        va_end(argp);
        return result;
    }
}
