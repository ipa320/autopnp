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
 * $Id: logUtils.h 7794 2014-03-12 17:13:22Z geisinger $
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
 * \typedef xme_core_log_componentID_t
 *
 * \brief  Unique indentifier for a component type or instance.
 */
typedef uint32_t xme_core_log_componentID_t;

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

/**
 * \def XME_LOG_C_IF
 *
 * \brief Issues a log message if the given condition holds.
 *
 * \details Depending on the platform specific implementation long log messages
 *          might be truncated, or message formatting might be unavailable.
 *
 * \note If you do not want to specify a component identifier or component type
 *       for the log message, use the XME_LOG_IF() macro instead.
 *
 * \param[in] condition Condition to check. Should evaluate to a boolean value.
 * \param[in] component Component type or instance identifier previously
 *            registered in the logging subsystem using
 *            xme_core_log_registerComponent().
 * \param[in] severity Log message severity.
 * \param[in] message Log message string literal or const char* pointer. The string
 *            can include special format characters as known from the printf()
 *            function.
 * \param[in] ... Optional parameters for formatting the log message string.
 */
#define XME_LOG_C_IF(condition, component, severity, message, ...) \
    do { \
        if (condition) \
        { \
            XME_LOG_C(component, severity, message, ##__VA_ARGS__); \
        } \
    } while (0)

/**
 * \def XME_CHECK_MSG_C
 *
 * \ingroup defines
 *
 * \brief Outputs a log message for a given component type or instance
 *        and returns from the current function with the given return value
 *        if the given condition does not hold.
 *
 * \details This macro helps to enforce a paradigm most functions is XME
 *          follow: they first check the arguments given to them and return
 *          an error code if a parameter does not meet their expectations.
 *          See the documentation of the defines group for usage examples.
 *
 * \note If you need to do cleanup tasks in case the condition does not
 *       hold, use the XME_CHECK_MSG_C_REC() macro instead. If you do not
 *       want to specify a component identifier or component type for the
 *       log message, use the XME_CHECK_MSG_REC() or XME_CHECK_MSG() macro
 *       instead. If you do not want to output a log message when the
 *       condition does not hold, use the XME_CHECK() or XME_CHECK_REC()
 *       macro instead.
 *
 * \param[in] condition Condition to check. Should evaluate to a boolean value.
 * \param[in] rval Return value to pass to the caller of the current function
 *            in case the condition does not hold. If the current function does
 *            not have a return value, specify XME_CHECK_RVAL_VOID for this
 *            parameter.
 * \param[in] component Component type or instance identifier previously
 *            registered in the logging subsystem using
 *            xme_core_log_registerComponent().
 * \param[in] severity Log message severity.
 * \param[in] message Log message string. The string can include special format
 *            characters as known from the printf() function.
 * \param[in] ... Optional parameters for formatting the log message string.
 */
#define XME_CHECK_MSG_C(condition, rval, component, severity, message, ...) \
    do { \
        if (!(condition)) \
        { \
            XME_LOG_C(component, severity, message, ##__VA_ARGS__); \
            return rval; \
        } \
    } while (0)

/**
 * \def XME_CHECK_MSG_C_REC
 *
 * \ingroup defines
 *
 * \brief Outputs a log message for a given component type or instance,
 *        performs the given recovery (cleanup) operations and returns from
 *        the current function with the given return value if the given
 *        condition does not hold.
 *
 * \details This macro helps to enforce a paradigm most functions is XME
 *          follow: they first check the arguments given to them and return
 *          an error code if a parameter does not meet their expectations.
 *          See the documentation of the defines group for usage examples.
 *
 * \note If you do not need to do cleanup tasks in case the condition does
 *       not hold, use the XME_CHECK_MSG() or XME_CHECK_MSG_C() macro
 *       instead. If you do not want to specify a component identifier or
 *       component type for the log message, use the XME_CHECK_MSG_REC()
 *       or XME_CHECK_MSG() macro instead. If you do not want to output
 *       a log message when the condition does not hold, use the XME_CHECK()
 *       or XME_CHECK_REC() macro instead.
 *
 * \param[in] condition Condition to check. Should evaluate to a boolean value.
 * \param[in] rval Return value to pass to the caller of the current function
 *            in case the condition does not hold. If the current function does
 *            not have a return value, specify XME_CHECK_RVAL_VOID for this
 *            parameter.
 * \param[in] recovery Recovery (cleanup) operations to perform in case the
 *            condition does not hold.
 * \param[in] component Component type or instance identifier previously
 *            registered in the logging subsystem using
 *            xme_core_log_registerComponent().
 * \param[in] severity Log message severity.
 * \param[in] message Log message string. The string can include special format
 *            characters as known from the printf() function.
 * \param[in] ... Optional parameters for formatting the log message string.
 */
#define XME_CHECK_MSG_C_REC(condition, rval, recovery, component, severity, message, ...) \
    do { \
        if (!(condition)) \
        { \
            XME_LOG_C(component, severity, message, ##__VA_ARGS__); \
            do { recovery; } while (0); \
            return rval; \
        } \
    } while (0)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the logging utility functions abstraction.
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_log_logUtils_init(void);

/**
 * \brief Frees all resources associated with the logging utility functions
 *        abstraction.
 */
void
xme_core_log_logUtils_fini(void);

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
 * \retval XME_STATUS_ALREADY_EXIST if the given component type or instance
 *         identifier has already been registered.
 * \retval XME_STATUS_OUT_OF_RESOURCES if there were not enough resources
 *         to register the component.
 */
xme_status_t
xme_core_log_registerComponent
(
    xme_core_log_componentID_t component,
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
    xme_core_log_componentID_t component
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
    xme_core_log_componentID_t component
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
    xme_core_log_componentID_t component,
    xme_log_severity_t componentMinSeverity
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_LOGUTILS_H
