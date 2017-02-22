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
 * $$
 */

/**
 * \file
 * \brief Command line parsing abstraction.
 */

#ifndef XME_HAL_CMDLINE_H
#define XME_HAL_CMDLINE_H

/**
 * \defgroup hal_cmdLine Command line parsing abstraction
 * @{
 *
 * \brief Command line parsing abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdbool.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the command line parsing abstraction with the given
 *        argument vector.
 *
 * \details argc and argv are the command line arguments typically passed to
 *          the program's main() function. If an argument equals to "--",
 *          the remaining arguments are not considered for argument parsing.
 *
 * \note This HAL component expects the memory pointer to by the argv and
 *       options arguments to be valid throughout the lifetime of the HAL
 *       module. In case the memory is temporarily allocated, be sure to
 *       call xme_hal_cmdLine_fini() before freeing the memory.
 *
 * \param[in] argc Number of arguments in the argument vector.
 *            Must be greater or equal to 1.
 * \param[in] argv Argument vector as initially passed to the executable.
 *            Must not be NULL. The element after the last element in the
 *            array (i.e., argv[argc]) must be NULL.
 * \param[in] options String that specifies the option characters that are
 *            valid for this program. An option character in this string can be
 *            followed by a colon (":") to indicate that it takes a required
 *            argument. If an option character is followed by two colons
 *            ("::"), its argument is optional.
 * \param[in] printError If nonzero, indicates to print error messages to the
 *            standard error stream if an unknown option character or an option
 *            with a missing required argument is encountered.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the HAL module has already been
 *         initialized.
 */
xme_status_t
xme_hal_cmdLine_init
(
    int argc,
    char* argv[],
    const char* const options,
    char printError
);

/**
 * \brief Copies the address of the command line arguments initially passed to
 *        xme_hal_cmdLine_init() to the given variable.
 *
 * \details If xme_hal_cmdLine_init() has not been called yet or was called
 *          with the argv parameter set to NULL, the variable pointed to by
 *          outArgv is set to NULL.
 *
 * \note    If outArgv is NULL, the function still returns the number of
 *          command line arguments (argc) as passed to xme_hal_cmdLine_init().
 *
 * \param[out] outArgv Address of a variable to initialize with the address of
 *             the memory that contains the command line arguments.
 *
 * \return Returns the number of command line arguments (argc) initially passed
 *         to xme_hal_cmdLine_init() or zero if xme_hal_cmdLine_init() has not
 *         been called yet.
 */
int
xme_hal_cmdLine_getArgs
(
    char** outArgv[]
);

/**
 * \brief Returns the identifier of the next option from the command line.
 *
 * \return Next option argument from the argument list specified by the argv
 *         and argc arguments. If an option character is found in argv that
 *         was not included in options, or a missing option argument, this
 *         function returns '?'.
 *         In this case, a call to xme_hal_cmdLine_getActualOption() can be
 *         issued to retrieve the actual option character.
 *         If the first character of options is a colon (':'), then this
 *         function returns ':' instead of '?' to indicate a missing option
 *         argument.
 */
int
xme_hal_cmdLine_getNextOption(void);

/**
 * \brief Returns the argument of the option previously parsed from the
 *        command line.
 *
 * \return Value of the most recently parsed option argument.
 */
const char*
xme_hal_cmdLine_getOptionArgument(void);

/**
 * \brief Returns the argument of the option previously parsed from the
 *        command line as an integer value.
 *
 * \details If printError was set in the previous call to
 *          xme_hal_cmdLine_init(), prints a message to the standard
 *          error stream in case parsing fails.
 *
 * \param[in] minValue Minimum allowed value, inclusive.
 * \param[in] maxValue Maximum allowed value, inclusive.
 * \param[in] defaultValue Default value to use in case of an error.
 *            Should be in the interval [minValue, maxValue].
 *
 * \return Value of the most recently parsed option argument as an integer
 *         value, clamped to the given range.
 */
int
xme_hal_cmdLine_getIntegerOptionArgument
(
    int minValue,
    int maxValue,
    int defaultValue
);

/**
 * \brief Frees resources occupied by the command line parsing abstraction.
 */
void
xme_hal_cmdLine_fini(void);

/**
 * @}
 */

XME_EXTERN_C_END

#endif // #ifndef XME_HAL_CMDLINE_H
