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
 * $Id: cmdLine_arch.c 6269 2014-01-08 16:00:03Z geisinger $
 */

/**
 * \file
 *         Command line parsing abstraction (architecture specific part:
 *         generic-OS implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/cmdLine.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_cmdLine_init
(
    int argc,
    char* argv[],
    const char* const options,
    char printError
)
{
    // Nothing to do
    XME_UNUSED_PARAMETER(argc);
    XME_UNUSED_PARAMETER(argv);
    XME_UNUSED_PARAMETER(options);
    XME_UNUSED_PARAMETER(printError);

    return XME_STATUS_SUCCESS;
}

int
xme_hal_cmdLine_getArgs
(
    char** outArgv[]
)
{
    if (NULL != outArgv)
    {
        *outArgv = NULL;
    }
    return 0;
}

int
xme_hal_cmdLine_getNextOption(void)
{
    // Always no options
    return -1;
}

const char*
xme_hal_cmdLine_getOptionArgument(void)
{
    // Always empty argument
    return NULL;
}

int
xme_hal_cmdLine_getIntegerOptionArgument
(
    int minValue,
    int maxValue,
    int defaultValue
)
{
    // Always default value
    XME_UNUSED_PARAMETER(minValue);
    XME_UNUSED_PARAMETER(maxValue);
    
    return defaultValue;
}

void
xme_hal_cmdLine_fini(void)
{
    // Nothing to do
}
