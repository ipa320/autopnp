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
 * $Id: log_arch.c 4945 2013-09-04 07:37:46Z ruiz $
 */

/**
 * \file
 *         Logging system abstraction (architecture specific part: generic OS
 *                                     based implementation).
 */

/**
 * \addtogroup core_log 
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/log.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

#ifdef _WIN32
    #include <Windows.h>
#endif // #ifdef _WIN32

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def    XME_LOG_CONSOLE_COLOR_TAG_ERROR
 *
 * \brief  Escape sequence used to color error text on the text console.
 */
/**
 * \def    XME_LOG_CONSOLE_COLOR_TAG_FATAL
 *
 * \brief  Escape sequence used to color error text on the text console.
 */
#if defined(XME_ENABLE_CONSOLE_COLORS) && defined(_WIN32) // Windows
    #define XME_LOG_CONSOLE_COLOR_TAG_WARNING _xme_log_console_setTextColor(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY)  ///< the console color for warnings (windows platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_ERROR _xme_log_console_setTextColor(FOREGROUND_RED | FOREGROUND_INTENSITY)               ///< the console color for errors (windows platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_FATAL _xme_log_console_setTextColor(FOREGROUND_RED | FOREGROUND_INTENSITY)               ///< the console color for fatal errors (windows platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_VERBOSE _xme_log_console_setTextColor(FOREGROUND_GREEN | FOREGROUND_BLUE)                ///< the console color for verbose mode (windows platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_DEBUG _xme_log_console_setTextColor(FOREGROUND_RED | FOREGROUND_BLUE)                    ///< the console color for debug (windows platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_NOTE _xme_log_console_setTextColor(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE)  ///< the console color for notes (windows platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_RESET _xme_log_console_setTextColor(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE) ///< the console color for reset (windows platform)
#elif defined(XME_ENABLE_CONSOLE_COLORS) && defined(__GNUC__) // Linux
    #define XME_LOG_CONSOLE_COLOR_TAG_WARNING "\33[1;33m" ///< the console color for warnings (linux platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_ERROR "\33[1;31m"   ///< the console color for errors (linux platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_FATAL "\33[1;31m"   ///< the console color for fatal errors (liunx platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_VERBOSE "\33[36m"   ///< the console color for verbose mode (linux platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_DEBUG "\33[35m"     ///< the console color for debug (linux platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_NOTE "\33[0m"       ///< the console color for notes (linux platform).
    #define XME_LOG_CONSOLE_COLOR_TAG_RESET "\33[0m"      ///< the console color for reset (linux platform)
#else // neither Windows nor Linux or colored console output disabled
    #define XME_LOG_CONSOLE_COLOR_TAG_WARNING "" ///< the console color for warnings. 
    #define XME_LOG_CONSOLE_COLOR_TAG_ERROR ""   ///< the console color for errors.
    #define XME_LOG_CONSOLE_COLOR_TAG_FATAL ""   ///< the console color for fatal errors. 
    #define XME_LOG_CONSOLE_COLOR_TAG_VERBOSE "" ///< the console color for verbose mode. 
    #define XME_LOG_CONSOLE_COLOR_TAG_DEBUG ""   ///< the console color for debug. 
    #define XME_LOG_CONSOLE_COLOR_TAG_NOTE ""    ///< the console color for notes. 
    #define XME_LOG_CONSOLE_COLOR_TAG_RESET ""   ///< the console color for reset. 
#endif

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
#ifdef _WIN32
static
const char*
_xme_log_console_setTextColor(WORD color); ///< sets the text color. 
#endif // #ifdef _WIN32

/**
 * \brief displays a message with the provided arguments. 
 *
 * \param acronym (Short) prefix for the log message, typically indicating a
 *        component type or other filter criteria.
 * \param severity the severity level. 
 * \param message the message to display.
 * \param args the arguments associated to the message. 
 */
static
void
_xme_log_console_args
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    va_list args
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
#ifdef _WIN32
static
const char*
_xme_log_console_setTextColor(WORD color)
{
    static HANDLE _xme_log_console_consoleHandle = 0;

    if (!_xme_log_console_consoleHandle)
    {
        _xme_log_console_consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
    }

    SetConsoleTextAttribute(_xme_log_console_consoleHandle, color);

    return "";
}
#endif // #ifdef _WIN32

static
void
_xme_log_console_args
(
    const char* acronym,
    xme_log_severity_t severity,
    const char* message,
    va_list args
)
{
    if (severity < XME_LOG_CONSOLE_MIN_SEVERITY)
    {
        return;
    }

    (void)xme_fallback_printf
    (
        (NULL != acronym) ? "%s[%s] %s" : "%s%s%s",

        (XME_LOG_WARNING == severity) ? XME_LOG_CONSOLE_COLOR_TAG_WARNING : (
        (XME_LOG_ERROR == severity) ?  XME_LOG_CONSOLE_COLOR_TAG_ERROR : (
        (XME_LOG_FATAL == severity) ?  XME_LOG_CONSOLE_COLOR_TAG_FATAL : (
        (XME_LOG_VERBOSE == severity) ?  XME_LOG_CONSOLE_COLOR_TAG_VERBOSE : (
        (XME_LOG_DEBUG == severity) ?  XME_LOG_CONSOLE_COLOR_TAG_DEBUG :
        XME_LOG_CONSOLE_COLOR_TAG_NOTE)))),

        (NULL != acronym) ? acronym : "",

        (XME_LOG_WARNING == severity) ?  "Warning: " : (
        (XME_LOG_ERROR == severity) ?  "Error: " : (
        (XME_LOG_FATAL == severity) ?  "Fatal: " : (
        (XME_LOG_VERBOSE == severity) ?  "Verbose: " : (
        (XME_LOG_DEBUG == severity) ?  "Debug: " :  ""))))
    );

    xme_fallback_vprintf(message, args);

#ifdef XME_ENABLE_CONSOLE_COLORS
    // puts() can not be used, because it automatically
    // appends a newline character to the output.
    xme_fallback_fputs(XME_LOG_CONSOLE_COLOR_TAG_RESET, stdout);
#else // #ifdef XME_ENABLE_CONSOLE_COLORS
    // The XME_LOG_CONSOLE_COLOR_TAG_RESET define won't be used
    // at all in this case. To prevent the compiler from complaining
    // about this, "use" the define without affecting compilation.
    #ifdef XME_LOG_CONSOLE_COLOR_TAG_RESET
    #endif // #ifdef XME_LOG_CONSOLE_COLOR_TAG_RESET
#endif // #ifdef XME_ENABLE_CONSOLE_COLORS
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
    va_start(args, message);

    if (NULL != xme_core_log_logCallback)
    {
        int len = xme_hal_safeString_vscprintf(message, args);
        if (len < 0)
        {
            xme_core_log_logCallback(severity, "[error formatting log message]");
        }
        else
        {
            char* strBuffer;
            unsigned int formattedMsgLen = (unsigned int) len;

            if (NULL != acronym)
            {
                // Three additional characters due to format "[%s] "
                formattedMsgLen += strlen(acronym) + 3;
            }

            // Terminating null character not included in formattedMsgLen
            formattedMsgLen++;

            // FIXME: Use a statically allocated string buffer for decently sized strings and only allocate on demand. But what about thread safety?
            strBuffer = (char*)xme_hal_mem_alloc((uint16_t) formattedMsgLen);

            if (NULL != strBuffer)
            {
                unsigned int printed = 0;
                if (NULL != acronym)
                {
                    int result = xme_hal_safeString_snprintf(strBuffer, (size_t) formattedMsgLen, "[%s] ", acronym);
                    if (result > 0)
                    {
                        printed = (unsigned int) result;

                        // On some platforms, xme_hal_safeString_snprintf() might
                        // return a value larger than formattedMsgLen to indicate
                        // that the buffer was too small. This should never be
                        // the case.
                        XME_ASSERT_NORVAL(printed <= formattedMsgLen);
                    }
                }

                xme_hal_safeString_vsnprintf
                (
                    (char*) (((uintptr_t) strBuffer) + ((uintptr_t) printed)),
                    (size_t) (formattedMsgLen - printed),
                    message,
                    args
                );

                xme_core_log_logCallback(severity, strBuffer);
                xme_hal_mem_free(strBuffer);
            }
            else
            {
                xme_core_log_logCallback(severity, "[not enough memory for log message]");
            }
        }
    }
    else
    {
        _xme_log_console_args(acronym, severity, message, args);
    }

    va_end(args);
}

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
extern xme_core_log_logCallback_t xme_core_log_logCallback;

/**
 * @}
 */
