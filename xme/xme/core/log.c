/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: log.c 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Logging system abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/log.h"

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
/**
 * \var xme_core_log_minimumLogSeverityLevel
 *
 * \brief Minumum log severity level.
 */
#ifdef XME_LOG_CONSOLE_MIN_SEVERITY
xme_log_severity_t xme_core_log_minimumLogSeverityLevel = XME_LOG_CONSOLE_MIN_SEVERITY;
#else // #ifdef XME_LOG_CONSOLE_MIN_SEVERITY
xme_log_severity_t xme_core_log_minimumLogSeverityLevel = XME_LOG_NOTE; // The default log level.
#endif // #ifdef XME_LOG_CONSOLE_MIN_SEVERITY

// Documented in log.h
xme_core_log_logCallback_t xme_core_log_logCallback;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_log_severity_t
xme_core_log_getMinimumLogSeverityLevel(void)
{
    return xme_core_log_minimumLogSeverityLevel;
}

void
xme_core_log_setMinimumLogSeverityLevel
(
    xme_log_severity_t minSeverity
)
{
    xme_core_log_minimumLogSeverityLevel = minSeverity;
}
