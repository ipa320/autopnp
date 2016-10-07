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
 * $Id: cmdLine_arch.c 7664 2014-03-04 08:47:41Z geisinger $
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

#include "getopt.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_hal_cmdLine_config_t
 *
 * \brief Structure for representing the internal state of the xme_hal_cmdLine
 *        component.
 */
typedef struct
{
    char initialized; ///< Whether the command line parsing abstraction has been initialized.
    int argc; ///< Number of command line arguments as passed to xme_hal_cmdLine_init().
    char** argv; ///< Address of the vector of command line arguments as passed to xme_hal_cmdLine_init().
    const char* options; ///< Address of a string that specifies the names of command line options to parse.
    char printError; ///< Whether messages should be printed to stderr in case of command line parsing errors.
} xme_hal_cmdLine_config_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \var xme_hal_cmdLine_config
 *
 * \brief Static variable containing the internal state of the xme_hal_cmdLine
 *        component.
 */
static xme_hal_cmdLine_config_t xme_hal_cmdLine_config;

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
    XME_ASSERT(NULL != argv);
    XME_ASSERT(argc >= 1);
    XME_CHECK(!xme_hal_cmdLine_config.initialized, XME_STATUS_INVALID_CONFIGURATION);

    xme_hal_cmdLine_config.initialized = 1;
    xme_hal_cmdLine_config.argc = argc;
    xme_hal_cmdLine_config.argv = argv;
    xme_hal_cmdLine_config.options = options;
    xme_hal_cmdLine_config.printError = printError;

    return XME_STATUS_SUCCESS;
}

int
xme_hal_cmdLine_getArgs
(
    char** outArgv[]
)
{
    // This returns the address of the command line argument vector as passed
    // to xme_hal_cmdLine_init(). Specifically, we do not copy the command
    // line arguments here.

    if (NULL != outArgv)
    {
        *outArgv = xme_hal_cmdLine_config.argv;
    }
    return xme_hal_cmdLine_config.argc;
}

int
xme_hal_cmdLine_getNextOption(void)
{
    XME_CHECK(xme_hal_cmdLine_config.initialized, -1);

    return getopt(xme_hal_cmdLine_config.argc, xme_hal_cmdLine_config.argv, (char*) xme_hal_cmdLine_config.options);
}

const char*
xme_hal_cmdLine_getOptionArgument(void)
{
    XME_ASSERT_RVAL(xme_hal_cmdLine_config.initialized, NULL);

    return optarg;
}

int
xme_hal_cmdLine_getIntegerOptionArgument
(
    int minValue,
    int maxValue,
    int defaultValue
)
{
    int value;
    int rv;

    XME_ASSERT_RVAL(xme_hal_cmdLine_config.initialized, defaultValue);

    // No argument means we use the default value
    XME_CHECK(NULL != optarg, defaultValue);

    // Make sure we got an integer argument
#ifdef _MSC_VER
    rv = sscanf_s(optarg, "%d", &value);
#else
    rv = sscanf(optarg, "%d", &value);
#endif
    if (rv != 1)
    {
        if (xme_hal_cmdLine_config.printError)
        {
            fprintf
            (
                stderr, "%s: integer argument required -- %s\n",
                xme_hal_cmdLine_config.argv[0], xme_hal_cmdLine_config.argv[optind - 1]
            );
        }
        return defaultValue;
    }

    // Make sure the integer argument is within the desired range
    if (value < minValue || maxValue < value)
    {
        if (xme_hal_cmdLine_config.printError)
        {
            fprintf
            (
                stderr, "%s: argument out of integer range -- %c %s\n",
                xme_hal_cmdLine_config.argv[0], optopt, xme_hal_cmdLine_config.argv[optind - 1]
            );
        }
        
        return defaultValue;
    }

    return value;
}

void
xme_hal_cmdLine_fini(void)
{
    XME_ASSERT_NORVAL(xme_hal_cmdLine_config.initialized);

    xme_hal_cmdLine_config.initialized = 0;
    xme_hal_cmdLine_config.argc = 0;
    xme_hal_cmdLine_config.argv = NULL;
    xme_hal_cmdLine_config.options = NULL;
    xme_hal_cmdLine_config.printError = 0;
}
