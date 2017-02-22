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
 * $Id: safeString.h 7483 2014-02-18 16:14:01Z wiesmueller $
 */

/**
 * \file
 * \brief Safe string handling abstraction.
 */

#ifndef XME_HAL_SAFESTRING_H
#define XME_HAL_SAFESTRING_H

/**
 * \defgroup hal_safeString Safe String Handling abstraction
 * @{
 *
 * \brief This component handles safe string operation.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdarg.h>
#include <stdio.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Returns the number of characters that would be generated if the
 *        string was to be formatted according to the specified format string
 *        using the given list of parameters.
 *        The value returned does not include the terminating null character.
 *
 * \param[in] format Format string as used in printf().
 * \param[in] ... Additional parameters as per format.
 *
 * \return On success, returns the total number of characters that would be
 *         generated if the string was to be formatted according to the
 *         specified format string, not including the terminating null
 *         character. On error, returns a negative number.
 */
int
FORMAT_FUNCTION(1, 2)
xme_hal_safeString_scprintf
(
    const char* format,
    ...
);

/**
 * \brief Returns the number of characters that would be generated if the
 *        string was to be formatted according to the specified format string
 *        using the given list of arguments.
 *        The value returned does not include the terminating null character.
 *
 * \param[in] format Format string as used in printf().
 * \param[in] args Additional parameters as an argument list as per format.
 *
 * \return On success, returns the total number of characters that would be
 *         generated if the string was to be formatted according to the
 *         specified format string, not including the terminating null
 *         character. On error, returns a negative number.
 */
static
int
FORMAT_FUNCTION(1, 0)
xme_hal_safeString_vscprintf
(
    const char* format,
    va_list args
);

/**
 * \brief Formats output data according to the given format string using the
 *        given list of parameters, possibly truncating the output string if
 *        the buffer is not large enough.
 *
 * \details If the number of bytes required to represent the result of
 *          formatting the string including the terminating null character is
 *          at most sizeOfBuffer, the function copies the format result
 *          including the terminating null character to buffer.
 *
 *          If the number of bytes required to represent the result of
 *          formatting the string including the terminating null character is
 *          larger than sizeOfBuffer, then as much of strSource as will fit
 *          into buffer is copied while still leaving room for the terminating
 *          null character which is always appended.
 *
 *          Example usage for determining the length of a formatted string:
 *          \code{.c}
 *          int len;
 *          int i = 42;
 *          len = xme_hal_safeString_snprintf(NULL, 0, "example %d", i);
 *          if (len < 0)
 *          {
 *            // No information gained
 *          }
 *          else
 *          {
 *            // len contains the length of the formatted string
 *          }
 *          \endcode
 *
 * \param[in,out] buffer Destination buffer. May be NULL if sizeInBytes is
 *                zero.
 * \param[in] sizeInBytes The size of the destination buffer, in bytes.
 * \param[in] format Format string as used in printf().
 * \param[in] ... Additional parameters as per format.
 *
 * \return On success, returns the total number of characters written, not
 *         including the terminating null character. On error, returns a
 *         negative number. On platforms where this is supported, the function
 *         might return a value larger than sizeInBytes to indicate that the
 *         string was truncated. In this case, buffer contains the truncated
 *         string and the return value corresponds to the number of characters
 *         excluding the terminating null character that would have been
 *         printed if the buffer would have been large enough.
 */
int
FORMAT_FUNCTION(3, 4)
xme_hal_safeString_snprintf
(
    char* buffer,
    size_t sizeInBytes,
    const char* format,
    ...
);

/**
 * \brief Formats output data according to the given format string using the
 *        given list of arguments, possibly truncating the output string if
 *        the buffer is not large enough.
 *
 * \deatils If the number of bytes required to represent the result of
 *          formatting the string including the terminating null character is
 *          at most sizeOfBuffer, the function copies the format result
 *          including the terminating null character to buffer.
 *
 *          If the number of bytes required to represent the result of
 *          formatting the string including the terminating null character is
 *          larger than sizeOfBuffer, then as much of strSource as will fit
 *          into buffer is copied while still leaving room for the terminating
 *          null character which is always appended.
 *
 *          Example usage for determining the length of a formatted string:
 *          \code{.c}
 *          int len;
 *          int i = 42;
 *          len = xme_hal_safeString_vsnprintf(NULL, 0, "example %d", args);
 *          if (len < 0)
 *          {
 *            // No information gained
 *          }
 *          else
 *          {
 *            // len contains the length of the formatted string
 *          }
 *          \endcode
 *
 * \param[in,out] buffer Destination buffer. May be NULL if sizeInBytes is
 *                zero.
 * \param[in] sizeInBytes The size of the destination buffer, in bytes.
 * \param[in] format Format string as used in printf().
 * \param[in] args Additional parameters as per format.
 *
 * \return On success, returns the total number of characters written, not
 *         including the terminating null character. On error, returns a
 *         negative number. On platforms where this is supported, the function
 *         might return a value larger than sizeInBytes to indicate that the
 *         string was truncated. In this case, buffer contains the truncated
 *         string and the return value corresponds to the number of characters
 *         excluding the terminating null character that would have been
 *         printed if the buffer would have been large enough.
 */
static
int
FORMAT_FUNCTION(3, 0)
xme_hal_safeString_vsnprintf
(
    char* buffer,
    size_t sizeInBytes,
    const char* format,
    va_list args
);

/**
 * \brief Copies character of string strSource to strDest, possibly truncating
 *        the string if the destination buffer is not large enough.
 *
 * \details Unlike the strncpy() function from the standard library, the resulting
 *          string is guaranteed to be NULL-terminated if sizeInBytes is larger
 *          than zero.
 *
 *          If the array pointed to by strSource is a string that is shorter
 *          than sizeInBytes bytes, the function copies all bytes of the
 *          string to strDest including the terminating null character.
 *
 *          If the array pointed to by strSource is a string that is longer
 *          than sizeInBytes bytes, then as much of strSource as will fit into
 *          strDest is copied while still leaving room for the
 *          terminating null character which is always appended.
 *
 * \note Unlike strncpy(), if sizeInBytes is greater than the length of
 *       strSource, strDest is NOT guaranteed to be padded with NULL
 *       characters up to its size. If you rely on this functionality,
 *       zero out strDest before calling this function or manually
 *       determine the length of the source string and zero out the
 *       respective part.
 *
 * \note If copying takes place between objects that overlap, the result
 *       is undefined.
 *
 * \param[in,out] strDest Destination string. May be NULL if sizeInBytes is
 *                zero.
 * \param[in] strSource Source string.
 * \param[in] sizeInBytes The size of the destination string buffer, in bytes.
 *
 * \return Returns strDest.
 */
static
char*
xme_hal_safeString_strncpy
(
    char* strDest,
    const char* strSource,
    size_t sizeInBytes
);

/**
 * \brief  Appends characters from string strSource to string strDest, possibly
 *         truncating the string if the destination buffer is not large enough.
 *
 * \details The resulting string is guaranteed to be NULL-terminated if
 *          sizeInBytes is larger than zero.
 *
 *          If the length of the string pointed to by strSource plus the length
 *          of the string pointed to by strDest is shorter than sizeInBytes
 *          bytes, the function appends all bytes of the string to the end of
 *          strDest including the terminating null character.
 *
 *          If the length of the string pointed to by strSource plus the length
 *          of the string pointed to by strDest is at least sizeInBytes bytes,
 *          then as much of strSource as will fit into the remaining space in
 *          strDest is copied while still leaving room for the terminating null
 *          character which is always appended.
 *
 * \note If copying takes place between objects that overlap, the result
 *       is undefined.
 *
 * \param[in,out] strDest Destination string. May be NULL if sizeInBytes is
 *                zero.
 * \param strSource Source string.
 * \param sizeInBytes The size of the destination string buffer, in bytes
 *        (including the space already used).
 *
 * \return Returns strDest.
 */
static
char*
xme_hal_safeString_strncat
(
    char *strDest,
    const char *strSource,
    size_t sizeInBytes
);

/**
 * \brief Returns  the  number  of  bytes in the string pointed to by s
 *        excluding the terminating null bye ('\0'), but at most maxlen. In
 *        doing this, it looks only at the first maxlen bytes at s and never
 *        beyond s + maxlen.
 *
 * \param[in] str String for which to determine the length.
 * \param[in] maxlen Maximum length to check.
 *
 * \return Length of string, if that is less than maxlen, or maxlen if there
 *         is no null byte ('\0') among the first maxlen bytes pointed to by str.
 */
static
size_t
xme_hal_safeString_strnlen
(
	const char *str,
	size_t maxlen
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/safeString_arch.h"

/**
 * @}
 */


#endif // #ifndef XME_HAL_SAFESTRING_H
