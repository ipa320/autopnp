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
 * $Id: logUtils.h 5044 2013-09-11 16:57:31Z geisinger $
 */

/**
 * \file
 *         Logging utility functions.
 */

#ifndef XME_CORE_LOGUTILS_H
#define XME_CORE_LOGUTILS_H

/**
 * \ingroup core_log
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/core/log.h"

#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_log_componentID
 *
 * \brief  Unique indentifier for a component type or instance.
 */
typedef uint32_t xme_core_log_componentID;

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def    XME_LOG_C
 *
 * \brief  Issues a log message from a specific component.
 *
 * The component parameter allows to specify the identifier of the
 * component type or instance from which the log message is issued.
 * This can be used for component specific filtering of log messages.
 *
 * Depending on the platform specific implementation long log messages might
 * be truncated, or message formatting might be unavailable.
 *
 * \param[in] component Identifier of the component type where the log message
 *            originates from.
 * \param[in] severity Log message severity. Note that messages of severity
 *            XME_LOG_DEBUG (and below) are ignored in release builds.
 * \param[in] message Log message string literal or const char* pointer. The string
 *            can include special format characters as known from the printf()
 *            function.
 * \param[in] ... Optional parameters for formatting the log message string.
 */
#ifdef NDEBUG
#define XME_LOG_C(component, severity, message, ...) \
    /* Release build, ignore messages with severity XME_LOG_DEBUG altogether */ \
    do \
    { \
        if (severity >= xme_core_log_getComponentLogLevel(component) && severity > XME_LOG_DEBUG) \
        { \
            xme_log_handler(xme_core_log_getComponentAcronym(component), severity, message, ##__VA_ARGS__); \
        } \
    } while (0)
#else // #ifdef NDEBUG
#define XME_LOG_C(component, severity, message, ...) \
    /* Debug build, accept all log levels according to XME_LOG_CONSOLE_MIN_SEVERITY */ \
    do \
    { \
        if (severity >= xme_core_log_getComponentLogLevel(component)) \
        { \
            xme_log_handler(xme_core_log_getComponentAcronym(component), severity, message, ##__VA_ARGS__); \
        } \
    } while (0)
#endif // #ifdef NDEBUG

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Registers the given component type or instance at the logging
 *         subsystem.
 *
 * \param[in] component Identifier of the component type or instance to
 *            register.
 * \param[in] componentAcronym Human-readable (short) name of the component.
 *            Typically used as a prefix in log messages.
 * \param[in] defaultComponentTypeMinSeverity Default log level for this
 *            component.
 *
 * \retval XME_STATUS_SUCCESS if the component was successfully registered.
 * \retval XME_STATUS_OUT_OF_RESOURCES if there were not enough resources
 *         to register the component.
 */
xme_status_t
xme_core_log_registerComponent
(
    xme_core_log_componentID component,
    const char* componentAcronym,
    xme_log_severity_t defaultComponentTypeMinSeverity
);

/**
 * \brief  Retrieves the current log level of the given component type or
 *         instance.
 *
 * The log level specifies the minimum severity of log messages that should be
 * processed. All log messages with less severity from the respective component
 * are ignored.
 *
 * \param[in] component Identifier of the component for which to retrieve the
 *            log level.
 *
 * \return Returns the current log level (i.e., minimum severity) of the given
 *         component if it has been previously registered using
 *         xme_core_log_registerComponentType() and
 *         XME_LOG_CONSOLE_MIN_SEVERITY otherwise.
 */
xme_log_severity_t
xme_core_log_getComponentLogLevel
(
    xme_core_log_componentID component
);

/**
 * \brief  Retrieves the acronym of the given component type or instance.
 *
 * \param[in] component Identifier of the component for which to retrieve the
 *            acronym.
 *
 * \return Returns the acronym of the given component if it has been previously
 *         registered using xme_core_log_registerComponentType() and NULL
 *         otherwise.
 */
const char*
xme_core_log_getComponentAcronym
(
    xme_core_log_componentID component
);

/**
 * \brief  Sets the log level for the given component type or instance.
 *
 * The log level specifies the minimum severity of log messages that should be
 * processed. All log messages with less severity from the respective component
 * will be subsequently ignored.
 *
 * \param[in] component Identifier of the component for which to set the log
 *            level.
 * \param[in] componentMinSeverity New minimum severity for the respective
 *            component.
 *
 * \retval  XME_STATUS_SUCCESS if the log level has been successfully changed.
 * \retval  XME_STATUS_NO_SUCH_VALUE if no component with the given identifier
 *          has been registered.
 */
xme_status_t
xme_core_log_setComponentLogLevel
(
    xme_core_log_componentID component,
    xme_log_severity_t componentMinSeverity
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_LOGUTILS_H
