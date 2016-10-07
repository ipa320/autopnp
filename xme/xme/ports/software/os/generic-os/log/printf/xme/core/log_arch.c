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
 * $Id: log_arch.c 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Logging system abstraction (architecture specific part:
 *         generic embedded implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/log.h"

#include "uprintf.h"

#include <stdarg.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

/**
 * \function _xme_log_console
 *
 * \brief  Outputs a log message to the console.
 *
 * \param  acronym (Short) prefix for the log message, typically indicating a
 *         component type or other filter criteria.
 * \param  severity Log message severity.
 * \param  message Log message string literal or const char* pointer. The string
 *         can include special format characters as known from the printf()
 *         function.
 * \param  args Optional parameters for formatting the log message string.
 */
static void
FORMAT_FUNCTION(3, 4)
_xme_log_console
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    va_list args
);

static void
_xme_log_console
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    va_list args
)
{
    XME_CHECK(severity >= minimumLogSeverity, XME_CHECK_RVAL_VOID);

    if (NULL != acronym)
    {
        printf("[%s] ", acronym);
    }

    printf
    (
        (XME_LOG_WARNING == severity) ? "Warning: " : (
        (XME_LOG_ERROR == severity) ? "Error: " : (
        (XME_LOG_FATAL == severity) ? "Fatal: " : (
        (XME_LOG_VERBOSE == severity) ? "Verbose: " : (
        (XME_LOG_DEBUG == severity) ? "Debug: " : ""))))
    );

    vprintf(message, args);
}

void
xme_log_handler
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    ...
)
{
    va_list args;

    if (xme_core_log_logCallback)
    {
        xme_core_log_logCallback(severity, message);
    }
    else
    {
        va_start(args, message);
        _xme_log_console(acronym, severity, message, args);
        va_end(args);
    }
}
