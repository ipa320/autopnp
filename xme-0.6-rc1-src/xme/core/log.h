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
 * $Id: log.h 4830 2013-08-27 16:50:50Z geisinger $
 */

/**
 * \file
 *         Logging system abstraction.
 */

#ifndef XME_CORE_LOG_H
#define XME_CORE_LOG_H

/**
 * \defgroup core_log Logging system abstraction.
 * @{
 *
 * \brief General logging system abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum  xme_log_severity_t
 *
 * \brief Log message severity.
 */
typedef enum
{
    XME_LOG_DEBUG   =  -4, ///< Debug message for developers. Ignored in release builds!
    XME_LOG_VERBOSE =  -2, ///< Verbose message.
    XME_LOG_NOTE    =   0, ///< Note. Messages of this severity and higher severity will be visible by default.
    XME_LOG_WARNING =   2, ///< Warning.
    XME_LOG_ERROR   =   4, ///< Error message.
    XME_LOG_FATAL   =   6, ///< Fatal error message.
    XME_LOG_ALWAYS  = 127, ///< Message that should always be output.
}
xme_log_severity_t;

/**
 * \typedef xme_core_log_logCallback_t
 *
 * \brief  Callback function for sending log messages.
 *
 * \param  severity Log message severity.
 * \param  message Log message string (no further formatting done).
 */
typedef void (*xme_core_log_logCallback_t)
(
    xme_log_severity_t severity,
    const char* message
);

/**
 * \typedef xme_core_log_logHandler_t
 *
 * \brief  Handler function for log messages.
 *
 * \param  acronym (Short) prefix for the log message, typically indicating a
 *         component type or other filter criteria.
 * \param  severity Log message severity.
 * \param  message Log message string (may contain format strings).
 * \param  ... Format arguments.
 */
typedef void (*xme_core_log_logHandler_t)
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    ...
);

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * Callback function used by the platform specific implementations of XME_LOG
 * (XME_LOG_ARCH definition in log_arch.h) to send log messages.
 * The core logger component will set this to the actual function during its
 * initialization. When not using the core logger you can point the callback
 * to a custom implementation.
 * When NULL a fallback logging mechanism is used. The used fallback depends on
 * the platform specific implementation.
 */
extern xme_core_log_logCallback_t xme_core_log_logCallback;

XME_EXTERN_C_END

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def    XME_LOG_CONSOLE_MIN_SEVERITY
 *
 * \brief  Minimum severity for console logging in case the xme_core_logger
 *         component is not used.
 */
#ifndef XME_LOG_CONSOLE_MIN_SEVERITY
    #define XME_LOG_CONSOLE_MIN_SEVERITY XME_LOG_NOTE
#endif

/**
 * \def    XME_LOG
 *
 * \brief  Issues a log message.
 *
 *         Depending on the platform specific implementation long log messages
 *         might be truncated, or message formatting might be unavailable.
 *
 * \param  severity Log message severity. Note that messages of severity
 *         XME_LOG_DEBUG (and below) are ignored in release builds.
 * \param  message Log message string literal or const char* pointer. The string
 *         can include special format characters as known from the printf()
 *         function.
 * \param  ... Optional parameters for formatting the log message string.
 */
#ifdef NDEBUG
#define XME_LOG(severity, message, ...) \
    /* Release build, ignore messages with severity XME_LOG_DEBUG altogether */ \
    do \
    { \
        if (severity > XME_LOG_DEBUG) \
        { \
            xme_log_handler(NULL, severity, message, ##__VA_ARGS__); \
        } \
    } while (0)
#else // #ifdef NDEBUG
#define XME_LOG(severity, message, ...) \
    /* Debug build, accept all log levels according to XME_LOG_CONSOLE_MIN_SEVERITY */ \
    do \
    { \
        xme_log_handler(NULL, severity, message, ##__VA_ARGS__); \
    } while (0)
#endif // #ifdef NDEBUG

/**
 * \def    XME_LOG_IF
 *
 * \brief  Issues a log message if the given condition holds.
 *
 *         Depending on the platform specific implementation long log messages
 *         might be truncated, or message formatting might be unavailable.
 *
 * \param  condition Condition to check. Should evaluate to a boolean value.
 * \param  severity Log message severity.
 * \param  message Log message string literal or const char* pointer. The string
 *         can include special format characters as known from the printf()
 *         function.
 * \param  ... Optional parameters for formatting the log message string.
 */
#define XME_LOG_IF(condition, severity, message, ...) \
    do { \
        if (condition) \
        { \
            XME_LOG(severity, message, ##__VA_ARGS__); \
        } \
    } while (0)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

/**
 * \brief  Platform specific implementation of XME_LOG.
 *
 * \details Formats log message and calls log callback.
 *          Depending on the platform specific implementation long log messages
 *          might be truncated, or message formatting might be unavailable.
 *
 * \note   Components should never call this function directly.
 *         Usee the XME_LOG() macro instead.
 *
 * \see    XME_LOG()
 *
 * \param  acronym (Short) prefix for the log message, typically indicating a
 *         component type or other filter criteria.
 * \param  severity Log message severity.
 * \param  message Log message string literal or const char* pointer. The string
 *         can include special format characters as known from the printf()
 *         function.
 * \param  ... Optional parameters for formatting the log message string.
 */
void
FORMAT_FUNCTION(3, 4)
xme_log_handler
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    ...
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/core/log_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_CORE_LOG_H
