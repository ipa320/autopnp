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
 * $Id: log_arch.c 4830 2013-08-27 16:50:50Z geisinger $
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

void
xme_log_handler
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    ...
)
{
    if (xme_core_log_logCallback)
    {
        xme_core_log_logCallback(severity, message);
    }
}
