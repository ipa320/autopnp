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
 * $Id: defines.h 7792 2014-03-12 17:07:33Z geisinger $
 */

/**
 * \file
 *         Generic definition and defines.
 */

#ifndef XME_DEFINES_H
#define XME_DEFINES_H

/*! \mainpage Welcome to CHROMOSOME!
 *
 * \section intro_sec Installation and Use
 *
 * This document is the API documentation of CHROMOSOME.
 * Please consult the separate CHROMOSOME Tutorial PDF file (available from http://chromosome.fortiss.org/) for an introduction and usage guide.
 */

/**
 * \defgroup defines Generic defines
 *
 * \brief Utility defines and macros.
 *
 * \details Usage examples of the XME_CHECK*() family of macros:
 *          \code
 *          void errorHandler(void);
 *          void cleanup(void);
 *          void cleanupSomeMore(void);
 *
 *          xme_status_t
 *          myFunc(int param)
 *          {
 *              // Return XME_STATUS_INVALID_PARAMETER if param > 10
 *              XME_CHECK(param <= 10, XME_STATUS_INVALID_PARAMETER);
 *
 *              // Call errorHandler() and return
 *              // XME_STATUS_INVALID_PARAMETER if param > 10
 *              XME_CHECK_REC(param <= 10, XME_STATUS_INVALID_PARAMETER, errorHandler());
 *
 *              // Do extensive cleanup and return
 *              // XME_STATUS_INVALID_PARAMETER if param > 10
 *              XME_CHECK_REC
 *              (
 *                  param <= 10,
 *                  XME_STATUS_INVALID_PARAMETER,
 *                  {
 *                      errorHandler();
 *                      cleanup();
 *                      cleanupSomeMore();
 *                  }
 *              );
 *
 *              // Output a message and continue if param > 10
 *              XME_CHECK_MSG
 *              (
 *                  param <= 10,
 *                  XME_STATUS_INVALID_PARAMETER,
 *                  XME_LOG_ERROR,
 *                  "param was expected to be <= 10, but it is %d!\n",
 *                  param
 *              );
 *
 *              // Output a message, do extensive cleanup and return
 *              // XME_STATUS_INVALID_PARAMETER if param > 10
 *              XME_CHECK_MSG_REC
 *              (
 *                  param <= 10,
 *                  XME_STATUS_INVALID_PARAMETER,
 *                  {
 *                      errorHandler();
 *                      cleanup();
 *                      cleanupSomeMore();
 *                  },
 *                  XME_LOG_ERROR,
 *                  "param was expected to be <= 10, but it is %d!\n",
 *                  param
 *              );
 *
 *              return XME_STATUS_SUCCESS;
 *          }
 *          \endcode
 */

/**
 * \def XME_ASSERT(condition)
 *
 * \brief Asserts that a condition holds in debug mode and aborts if it does
 *        not.
 *
 * \details Typical usage scenarios of this macro include:
 *          \code
 *          xme_status_t
 *          func(void)
 *          {
 *              void* q;
 *              void* p = someFunc();
 *              XME_ASSERT(NULL != p); // will return XME_CORE_STATUS_UNEXPECTED on assertion failure
 *              XME_ASSERT(NULL != someFunc()); // will trigger runtime warning about side effects
 *              XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != someFunc())); // disable side effects warning
 *              XME_ASSERT(NULL != (q = p)); // will trigger runtime warning about side effects
 *              XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != (q = p))); // disable side effects warning
 *              return XME_CORE_STATUS_SUCCESS;
 *          }
 *          \endcode
 *
 * \note Since this macro will only have an effect in debug mode (i.e., when
 *       NDEBUG is not set), the condition must not yield any side effects
 *       relevant to non-debug code. To avoid side effects being present in
 *       the condition, a heuristic check has been implemented that detects
 *       the (potential) presence of side effects during runtime and issues
 *       a warning using XME_LOG().
 *       See xme_assert_checkForSideEffects() for details and on how to
 *       prevent the check for intended side effects.
 *
 * \param[in] condition Condition to assert. See note.
 *
 * \return Returns void if the condition holds. Returns from the current
 *         function with the return value XME_CORE_STATUS_UNEXPECTED if the
 *         condition does not hold. This requires the return type of that
 *         function to be xme_status_t. Use the XME_ASSERT_RVAL() or
 *         XME_ASSERT_NORVAL() macros instead if that function has a different
 *         return type or returns void, respectively.
 *         This behavior is used to make the effect of XME_ASSERT consistent on
 *         platforms that do not allow a debugger to break program execution if
 *         the condition does not hold.
 *         In this case, an error check in the code calling the function where
 *         the assertion is triggered can handle the XME_CORE_STATUS_UNEXPECTED
 *         error code.
 */

/**
 * \def    XME_ASSERT_RVAL(condition, rval)
 *
 * \brief Asserts that a condition holds in debug mode and aborts if it does
 *        not.
 *
 * \details Typical usage scenarios of this macro include:
 *          \code
 *          int
 *          func(void)
 *          {
 *              void* q;
 *              void* p = someFunc();
 *              XME_ASSERT_RVAL(NULL != p, 42); // will return 42 on assertion failure
 *              XME_ASSERT_RVAL(NULL != someFunc(), 42); // will trigger runtime warning about side effects
 *              XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(NULL != someFunc()), 42); // disable side effects warning
 *              XME_ASSERT_RVAL(NULL != (q = p), 42); // will trigger runtime warning about side effects
 *              XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(NULL != (q = p)), 42); // disable side effects warning
 *              return XME_CORE_STATUS_SUCCESS;
 *          }
 *          \endcode
 *
 * \note Since this macro will only have an effect in debug mode (i.e., when
 *       NDEBUG is not set), the condition must not yield any side effects
 *       relevant to non-debug code. To avoid side effects being present in
 *       the condition, a heuristic check has been implemented that detects
 *       the (potential) presence of side effects during runtime and issues
 *       a warning using XME_LOG().
 *       See xme_assert_checkForSideEffects() for details and on how to
 *       prevent the check for intended side effects.
 *
 * \param[in] condition Condition to assert. See note.
 * \param[in] rval Value to return from the function calling this macro in case
 *            the condition does not hold. See return value.
 *
 * \return Returns void if the condition holds. Returns from the current
 *         function with the return value rval if the condition does not hold.
 *         Use the XME_ASSERT() or XME_ASSERT_NORVAL() macros instead if that
 *         function has a return type of xme_status_t or returns void,
 *         respectively.
 *         This behavior is used to make the effect of XME_ASSERT consistent on
 *         platforms that do not allow a debugger to break program execution
 *         if the condition does not hold.
 *         In this case, an error check in the code calling the function where
 *         the assertion is triggered can handle the return value.
 */

/**
 * \def XME_ASSERT_NORVAL(condition)
 *
 * \brief Asserts that a condition holds in debug mode and aborts if it does
 *         not.
 *
 * \details Typical usage scenarios of this macro include:
 *          \code
 *          void
 *          func(void)
 *          {
 *              void* q;
 *              void* p = someFunc();
 *              XME_ASSERT_NORVAL(NULL != p); // will return on assertion failure
 *              XME_ASSERT_NORVAL(NULL != someFunc()); // will trigger runtime warning about side effects
 *              XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(NULL != someFunc())); // disable side effects warning
 *              XME_ASSERT_NORVAL(NULL != (q = p)); // will trigger runtime warning about side effects
 *              XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(NULL != (q = p))); // disable side effects warning
 *              return XME_CORE_STATUS_SUCCESS;
 *          }
 *          \endcode
 *
 * \note Since this macro will only have an effect in debug mode (i.e., when
 *       NDEBUG is not set), the condition must not yield any side effects
 *       relevant to non-debug code. To avoid side effects being present in
 *       the condition, a heuristic check has been implemented that detects
 *       the (potential) presence of side effects during runtime and issues
 *       a warning using XME_LOG().
 *       See xme_assert_checkForSideEffects() for details and on how to
 *       prevent the check for intended side effects.
 *
 * \param[in] condition Condition to assert. See note.
 *
 * \return Returns void if the condition holds. Returns from the current
 *         function with a void return value if the condition does not hold.
 *         Use the XME_ASSERT() or XME_ASSERT_RVAL() macros instead if that
 *         function has a return type of xme_status_t or an arbitrary
 *         other return type, respectively.
 *         This behavior is used to make the effect of assertions consistent on
 *         platforms that do not allow a debugger to break program execution
 *         if the condition does not hold.
 */

/**
 * \def XME_ASSERT_NO_SIDE_EFFECTS(condition)
 *
 * \brief Disable "potential side effects" warning for conditions of
 *        XME_ASSERT*() macros calls on platforms that support checking
 *        for unintended side effects.
 *
 * \note This concept may not be supported on all platforms!
 *
 * \details Normally, a warning is issued during runtime if a call to an
 *          XME_ASSERT*() macro contains a function call, an assignment,
 *          an increment or a decrement.
 *          The reason for this is that those side effects will not be
 *          present in release builds (because assertions become
 *          no-operations in release builds) and hence could break the
 *          functionality. In situations where it is safe to call a
 *          specific function or use one of the operators mentioned above,
 *          surround the condition with the XME_ASSERT_NO_SIDE_EFFECTS()
 *          macro to disable the warning.
 *
 * Typical usage scenarios of this macro include:
 * \code
 * void
 * func(void)
 * {
 *     // Subtle error! doSomething() will not be called in release builds!
 *     // -> Triggers warning "[...] with potential side effects (function call)"
 *     XME_ASSERT_NORVAL(XME_CORE_STATUS_SUCCESS == doSomething());
 *
 *     // Correct way to call doSomething() and check the result
 *     // (the extra scope might be required to define the 'result' variable):
 *     {
 *         xme_status_t result = doSomething();
 *         XME_ASSERT_NORVAL(XME_CORE_STATUS_SUCCESS == result);
 *     }
 *
 *     // Or, if you are sure that doSomething() has no side effects that
 *     // should also be present in release builds:
 *     // -> Silences warning
 *     XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_CORE_STATUS_SUCCESS == doSomething()));
 * }
 * \endcode
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
// Include options controlled by FindXME.cmake which generates this include
// file into a subdirectory of the binary folder.
#include "xme/xme_opt.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/**
 * \def XME_MAX_SYSTEM_VALUE
 *
 * \brief Maximum value of most XME enumerations.
 *
 * \warning Keep consistent with xme_maxSystemValue_t!
 */
#define XME_MAX_SYSTEM_VALUE (0xFFFFU)

//----------------------------------------------------------------------------//
//     Inline macros                                                          //
//----------------------------------------------------------------------------//

/**
 * \def INLINE
 *
 * \brief Suggests inline-compilation of function in a platform-independent
 *        way.
 */
#if defined(_MSC_VER)
    #define INLINE __inline // The "inline" keyword is only available in C++
#elif defined(__GNUC__)
    #define INLINE inline
#else
    #define INLINE /* Do not try to inline, it might not be supported */
#endif

//----------------------------------------------------------------------------//
//     Designated initialization of struct fields                             //
//----------------------------------------------------------------------------//

/*
 * The following definitions HAVE_DESIGNATED_INITIALIZERS() and SFINIT()
 * are inspired by a StackOverflow port under the cc-wiki-sa 3.0 license
 * (see http://creativecommons.org/licenses/by-sa/3.0/
 *  and http://blog.stackoverflow.com/2009/06/attribution-required/).
 *
 * Original question:
 *  - http://stackoverflow.com/questions/5440611/
 *
 * The following authors contributed to this question:
 *  - cordic: http://stackoverflow.com/users/1823136/
 *  - Stephan: http://stackoverflow.com/users/363573/
 *  - DinGODzilla: http://stackoverflow.com/users/64062/
 *
 * The following code until the "----- END cc-by-sa 3.0 -----" marker may be
 * licensed under the license of CHROMOSOME or the cc-wiki-sa 3.0 license.
 */

/**
 * \def HAVE_DESIGNATED_INITIALIZERS
 *
 * \brief Define that is set depending on whether the compiler supports
 *        designated initialization.
 */
#if _MSC_VER > 0 && _MSC_VER <= 1700 // 1600 is MSVC 10.0, 1700 is MSVC 11.0
    #undef HAVE_DESIGNATED_INITIALIZERS // MSVC 10.0 and earlier do not support designed initialization
#elif _MSC_VER > 1700
    #error Please evaluate whether designated initializers are supported in this version of the Microsoft C/C++ compiler!
#else
    #define HAVE_DESIGNATED_INITIALIZERS // Assume designed initialization is supported
#endif

/**
 * \def    SFINIT
 *
 * \brief  Initialize a struct field.
 *         In case designated initialization is supported by the compiler
 *         (i.e., ::HAVE_DESIGNATED_INITIALIZERS is defined), this feature
 *         will be used.
 *
 * \note   For compatibility with compilers that do not support designated
 *         initialization of struct fields, ensure that the order of
 *         initializations is the same as the order of struct fields.
 *
 * \details Example 1:
 *          <code>
 *          typedef struct s_t
 *          {
 *              int a;
 *              char b;
 *          } s_t;
 *
 *          // Correct:
 *          s_t s1a =
 *          {
 *              SFINIT(a, 1234),
 *              SFINIT(b, 56)
 *          };
 *          s_t s1b =
 *          {
 *              SFINIT(a, 0),
 *              SFINIT(b, 56)
 *          };
 *          s_t s1c =
 *          {
 *              SFINIT(a, 1234); // will initialize 'b' with zero
 *          };
 *
 *          // Wrong:
 *          s_t s2a =
 *          {
 *              SFINIT(b, 56); // will initialize 'a' with 56 and 'b' with zero
 *                             // when designated initialization is not available!
 *          };
 *          </code>
 *
 *          Example 2:
 *          <code>
 *          typedef struct s2_t
 *          {
 *              struct
 *              {
 *                  int x;
 *                  int y;
 *              } subStruct;
 *          } s2_t;
 *
 *          s2_t s2 =
 *          {
 *              SFINIT(subStruct,
 *              {
 *                  SFINIT(x, 0),
 *                  SFINIT(y, 1)
 *              })
 *          };
 *          </code>
 *
 * \param  name Name of the struct field to initialize.
 * \param  ... Value of the struct field.
 */
#ifdef HAVE_DESIGNATED_INITIALIZERS
#define SFINIT(name, ...) .name = __VA_ARGS__
#else
#define SFINIT(name, ...) __VA_ARGS__
#endif

/* ----- END cc-by-sa 3.0 ----- */

//----------------------------------------------------------------------------//
//     extern "C" macros                                                      //
//----------------------------------------------------------------------------//

#if defined(__cplusplus) //&& !defined(__CDT_PARSER__)
/**
 * \def XME_EXTERN_C_FORCE
 *
 * \brief When defined, forces all functions to be declared with C linkage.
 *
 * \note By default, C linkage is not enforced if the Eclipse CDT parser is
 *       processing this file as part of a pre-compile syntax check, because
 *       this causes (wrong) syntax errors. This fixes issue #1806.
 */
#define XME_EXTERN_C_FORCE
#endif // #if defined(__cplusplus) && !defined(__CDT_PARSER__)

/**
 * \def    XME_EXTERN_C
 *
 * \brief  Equals 'extern "C"' if C linkage is to be used and nothing otherwise.
 */
#ifdef XME_EXTERN_C_FORCE
#define XME_EXTERN_C extern "C"
#else // #ifdef XME_EXTERN_C_FORCE
#define XME_EXTERN_C /* nothing */
#endif // #ifdef XME_EXTERN_C_FORCE

/**
 * \def    XME_EXTERN_C_BEGIN
 *
 * \brief  Begins an 'extern "C"' block if C linkage is to be used.
 */
#ifdef XME_EXTERN_C_FORCE
#define XME_EXTERN_C_BEGIN extern "C" \
{
#else // #ifdef XME_EXTERN_C_FORCE
#define XME_EXTERN_C_BEGIN /* nothing */
#endif // #ifdef XME_EXTERN_C_FORCE

/**
 * \def XME_EXTERN_C_END
 *
 * \brief Ends an 'extern "C"' block if C linkage is to be used.
 *
 * \note Visual Studio 2010's IntelliSense has a strange behavior (bug?)
 *       that will produce a lot of false positive messages stating "this
 *       declaration has no storage class or type specifier" when the
 *       XME_EXTERN_C_END macro is used. The only workaround I found was
 *       to add an actual declaration to the macro. Since this issue is
 *       most likely to be seen during development, I'm only putting it
 *       there when NDEBUG is not defined.
 */
#ifdef XME_EXTERN_C_FORCE
#ifdef NDEBUG
#define XME_EXTERN_C_END \
    /* This might produce VS2010 IntelliSense errors "this declaration */ \
    /* has no storage class or type specifier", which can be safely    */ \
    /* ignored.                                                        */ \
}
#else // #ifdef NDEBUG
#define XME_EXTERN_C_END \
    /* Need to have a dummy declaration here   */ \
    /* to work around VS2010 IntelliSense bug: */ \
    extern char xme_extern_c_dummy_variable; \
}
#endif // #ifdef NDEBUG
#else // #ifdef XME_EXTERN_C_FORCE
#define XME_EXTERN_C_END /* nothing */
#endif // #ifdef XME_EXTERN_C_FORCE

/**
 * \typedef xme_status_t
 *
 * \brief Core status codes.
 */
typedef enum
{
    XME_STATUS_SUCCESS = 0, ///< The operation completed successfully. This must be defined as zero.
    XME_STATUS_UNEXPECTED, ///< Error: Assertion failed. This value can be returned by any API function in debug mode if an assertion failed even if it is not documented. It should only be used by the ::XME_ASSERT() family of functions in order to indicate that no  assertion handling is installed.
    XME_STATUS_NO_SUCH_VALUE, ///< Error: No such value.
    XME_STATUS_INTERNAL_ERROR, ///< Error: Unspecified internal error.
    XME_STATUS_INVALID_PARAMETER, ///< Error: Invalid parameter. At least one of the given parameters is set to an invalid or unsupported value.
    XME_STATUS_INVALID_CONFIGURATION, ///< Error: Invalid configuration. This means that all parameters were set to proper values, but the given combination of configuration parameters is not allowed at this point in time.
    XME_STATUS_INVALID_HANDLE, ///< Error: Invalid handle. The specified handle is not valid (any more).
    XME_STATUS_PERMISSION_DENIED, ///< Error: Permission denied.
    XME_STATUS_UNSUPPORTED, ///< Error: Requested function is not supported on this platform.
    XME_STATUS_NOT_FOUND, ///< Error: Object not found.
    XME_STATUS_OUT_OF_RESOURCES, ///< Error: Out of resources.
    XME_STATUS_ALREADY_EXIST, ///< Error: Object already exists.
    XME_STATUS_TEMPORARY_FAILURE, ///< Error: Temporary failure, try again later.
    XME_STATUS_TIMEOUT, ///< Error: Request timed out.
    XME_STATUS_CONNECTION_REFUSED, ///< Error: Connection refused.
    XME_STATUS_WOULD_BLOCK, ///< Error: Operation would block.
    XME_STATUS_INTERRUPTED, ///< Error: Operation interrupted.
    XME_STATUS_ABORTED, ///< Error: Operation aborted.
    XME_STATUS_BUFFER_TOO_SMALL ///< Error: Buffer too small.
}
xme_status_t;

//----------------------------------------------------------------------------//
//     Unused parameters in functions                                         //
//----------------------------------------------------------------------------//

/**
 * \def XME_UNUSED_PARAMETER
 *
 * \ingroup defines
 *
 * \brief Declares the given parameter as unused within the current function.
 *
 * \details This macro can be used to declare a parameter that is passed to a
 *          function, but not used within its body as unused. Depending on the
 *          compiler and its warning level, a warning or compiler error might
 *          otherwise be reported.
 *
 * \param[in] param The name of the parameter to declare unused.
 */
#define XME_UNUSED_PARAMETER(param) (void)(param)

//----------------------------------------------------------------------------//
//     Compiler-specific features                                             //
//----------------------------------------------------------------------------//

/**
 * \def FORMAT_FUNCTION
 *
 * \ingroup defines
 *
 * \brief Declares a function as being a format function similar to printf().
 *        Some compilers (e.g., GCC) support additional compile-time
 *        checks for the parameters in this case.
 *
 * \details Usage example:
 *         <code>
 *         int
 *         FORMAT_FUNCTION(2, 3)
 *         logMessage(int severity, const char * format, ...);
 *
 *         void f(void)
 *         {
 *             // This will trigger GCC warning: format �%d� expects argument
 *             // of type �int�, but argument 2 has type �const char*� [-Wformat]
 *             logMessage(1, "This is %d!\n", "wrong");
 *         }
 *         </code>
 *
 * \param[in] formatParamIndex One-based index of the format string parameter of
 *            the respective function.
 * \param[in] firstArgParamIndex One-based index of the first format argument of
 *            the respective function.
 */
#ifdef __GNUC__
#define FORMAT_FUNCTION(formatParamIndex, firstArgParamIndex) \
    /* This extension allows GCC to check the format string */ \
    /*__attribute__((format(printf, formatParamIndex, firstArgParamIndex)))*/ /* TODO: Temporarily disabled until we have a suitable abstraction for formatting, see Issue #2412 */
#else // #ifdef __GNUC__
#define FORMAT_FUNCTION(formatParamIndex, firstArgParamIndex) /* not supported */
#endif // #ifdef __GNUC__

/**
 * \def    SCAN_FUNCTION
 *
 * \ingroup defines
 *
 * \brief  Declares a function as being a scan function similar to scanf().
 *         Some compilers (e.g., GCC) support additional compile-time
 *         checks for the parameters in this case.
 *
 * \details Usage example:
 *         <code>
 *         int
 *         SCAN_FUNCTION(2, 3)
 *         myScan(int param, const char * format, ...);
 *
 *         void f(void)
 *         {
 *             // This will trigger GCC warning: format �%d� expects argument of
 *             // type �int*�, but argument 3 has type �char*� [-Wformat]
 *             char c;
 *             myScan(1, "This is wrong %d!\n", &c);
 *         }
 *         </code>
 *
 * \param  formatParamIndex One-based index of the format string parameter of
 *         the respective function.
 * \param  firstOutParamIndex One-based index of the first output argument of
 *         the respective function.
 */
#ifdef __GNUC__
#define SCAN_FUNCTION(formatParamIndex, firstOutParamIndex) \
    /* This extension allows GCC to check the format string */ \
    /*__attribute__((format(scanf, formatParamIndex, firstOutParamIndex)))*/  /* TODO: Temporarily disabled until we have a suitable abstraction for formatting, see Issue #2412 */
#else // #ifdef __GNUC__
#define SCAN_FUNCTION(formatParamIndex, firstOutParamIndex) /* not supported */
#endif // #ifdef __GNUC__

//----------------------------------------------------------------------------//
//     Exit macro                                                             //
//----------------------------------------------------------------------------//

/**
 * \def XME_EXIT
 *
 * \ingroup defines
 *
 * \brief Exits the program execution.
 *
 * \param[in] message Log message string. The string can include special format
 *            characters as known from the printf() function.
 * \param[in] value Value send as exit code.
 */
#define XME_EXIT(value, message, ...) \
    do { \
        XME_LOG(XME_LOG_FATAL, message, ##__VA_ARGS__); \
        exit(value); \
    } while (0)

//----------------------------------------------------------------------------//
//     Check macros                                                           //
//----------------------------------------------------------------------------//

/**
 * \def XME_CHECK_RVAL_VOID
 *
 * \ingroup defines
 *
 * \brief Used in the XME_CHECK*() family of macros to specify that the
 *        respective function does not have a return value.
 */
#define XME_CHECK_RVAL_VOID /* empty parameter */

/**
 * \def XME_CHECK
 *
 * \ingroup defines
 *
 * \brief Returns from the current function with the given return value if the
 *        given condition does not hold.
 *
 * \details This macro helps to enforce a paradigm most functions is XME
 *          follow: they first check the arguments given to them and return
 *          an error code if a parameter does not meet their expectations.
 *          See the documentation of the defines group for usage examples.
 *
 * \note If you need to do cleanup tasks in case the condition does not
 *       hold, use the XME_CHECK_REC() macro instead. If you want to output
 *       a log message when the condition does not hold, use the
 *       XME_CHECK_MSG(), XME_CHECK_MSG_C(), XME_CHECK_MSG_REC() or
 *       XME_CHECK_MSG_C_REC() macro instead.
 *
 * \param[in] condition Condition to check. Should evaluate to a boolean value.
 * \param[in] rval Return value to pass to the caller of the current function
 *            in case the condition does not hold. If the current function does
 *            not have a return value, specify XME_CHECK_RVAL_VOID for this
 *            parameter.
 */
#define XME_CHECK(condition, rval) \
    do { \
        if (!(condition)) \
        { \
            return rval; \
        } \
    } while (0)

/**
 * \def XME_CHECK_REC
 *
 * \ingroup defines
 *
 * \brief Performs the given recovery (cleanup) operations and returns from
 *        the current function with the given return value if the given
 *        condition does not hold.
 *
 * \details This macro helps to enforce a paradigm most functions is XME
 *          follow: they first check the arguments given to them and return
 *          an error code if a parameter does not meet their expectations.
 *          See the documentation of the defines group for usage examples.
 *
 * \note If you do not need to do cleanup tasks in case the condition does
 *       not hold, use the XME_CHECK() macro instead. If you want to output
 *       a log message when the condition does not hold, use the
 *       XME_CHECK_MSG(), XME_CHECK_MSG_C(), XME_CHECK_MSG_REC() or
 *       XME_CHECK_MSG_C_REC() macro instead.
 *
 * \param[in] condition Condition to check. Should evaluate to a boolean value.
 * \param[in] rval Return value to pass to the caller of the current function
 *            in case the condition does not hold. If the current function does
 *            not have a return value, specify XME_CHECK_RVAL_VOID for this
 *            parameter.
 * \param[in] recovery Recovery (cleanup) operations to perform in case the
 *            condition does not hold.
 */
#define XME_CHECK_REC(condition, rval, recovery) \
    do { \
        if (!(condition)) \
        { \
            do { recovery; } while (0); \
            return rval; \
        } \
    } while (0)

/**
 * \def XME_CHECK_MSG
 *
 * \ingroup defines
 *
 * \brief Outputs a log message and returns from the current function with
 *        the given return value if the given condition does not hold.
 *
 * \details This macro helps to enforce a paradigm most functions is XME
 *          follow: they first check the arguments given to them and return
 *          an error code if a parameter does not meet their expectations.
 *          See the documentation of the defines group for usage examples.
 *
 * \note If you need to do cleanup tasks in case the condition does not
 *       hold, use the XME_CHECK_MSG_REC() macro instead. If you want to
 *       specify a component identifier or component type in order to
 *       automatically prepend the associated component (instance) acronym
 *       in front of the log message, use the XME_CHECK_MSG_C() macro
 *       instead. If you do not want to output a log message when the
 *       condition does not hold, use the XME_CHECK() or XME_CHECK_REC()
 *       macro instead.
 *
 * \param[in] condition Condition to check. Should evaluate to a boolean value.
 * \param[in] rval Return value to pass to the caller of the current function
 *            in case the condition does not hold. If the current function does
 *            not have a return value, specify XME_CHECK_RVAL_VOID for this
 *            parameter.
 * \param[in] severity Log message severity.
 * \param[in] message Log message string. The string can include special format
 *            characters as known from the printf() function.
 * \param[in] ... Optional parameters for formatting the log message string.
 */
#define XME_CHECK_MSG(condition, rval, severity, message, ...) \
    do { \
        if (!(condition)) \
        { \
            XME_LOG(severity, message, ##__VA_ARGS__); \
            return rval; \
        } \
    } while (0)

/**
 * \def    XME_CHECK_MSG_REC
 *
 * \ingroup defines
 *
 * \brief Outputs a log message, performs the given recovery (cleanup)
 *        operations and returns from the current function with the given
 *        return value if the given condition does not hold.
 *
 * \details This macro helps to enforce a paradigm most functions is XME
 *          follow: they first check the arguments given to them and return
 *          an error code if a parameter does not meet their expectations.
 *          See the documentation of the defines group for usage examples.
 *
 * \note   If you do not need to do cleanup tasks in case the condition does
 *         not hold, use the XME_CHECK_MSG() or XME_CHECK_MSG_C() macro
 *         instead. If you want to specify a component identifier or
 *         component type in order to automatically prepend the associated
 *         component (instance) acronym in front of the log message, use the
 *         XME_CHECK_MSG_C_REC() macro instead. If you do not want to output
 *         a log message when the condition does not hold, use the XME_CHECK()
 *         or XME_CHECK_REC() macro instead.
 *
 * \param[in] condition Condition to check. Should evaluate to a boolean value.
 * \param[in] rval Return value to pass to the caller of the current function
 *            in case the condition does not hold. If the current function does
 *            not have a return value, specify XME_CHECK_RVAL_VOID for this
 *            parameter.
 * \param[in] recovery Recovery (cleanup) operations to perform in case the
 *            condition does not hold.
 * \param[in] severity Log message severity.
 * \param[in] message Log message string. The string can include special format
 *            characters as known from the printf() function.
 * \param[in] ... Optional parameters for formatting the log message string.
 */
#define XME_CHECK_MSG_REC(condition, rval, recovery, severity, message, ...) \
    do { \
        if (!(condition)) \
        { \
            XME_LOG(severity, message, ##__VA_ARGS__); \
            do { recovery; } while (0); \
            return rval; \
        } \
    } while (0)

//----------------------------------------------------------------------------//
//     Misc macros                                                            //
//----------------------------------------------------------------------------//

/**
 * \def XME_NOT_IMPLEMENTED
 *
 * \ingroup defines
 *
 * \brief Makes the application exit with an error message about an
 *        unimplemented function.
 *
 * \details The error message being printed contains the name of the calling
 *          function in order to simplify tracing. When this macro is executed,
 *          the application terminates with a return code of EXIT_FAILURE using
 *          exit().
 *
 * \warning The program context never returns from this macro.
 */
#define XME_NOT_IMPLEMENTED() \
    do { \
        XME_LOG(XME_LOG_FATAL, "%s() not implemented!\n", __FUNCTION__); \
        exit(EXIT_FAILURE); \
    } while (0)

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_maxSystemValue_t
 *
 * \brief Type that can at least represent values between zero and XME_MAX_SYSTEM_VALUE.
 *
 * \warning Keep consistent with XME_MAX_SYSTEM_VALUE!
 */
typedef unsigned short xme_maxSystemValue_t;

/**
 * \typedef xme_hal_time_timeInterval_t
 *
 * \brief  A time interval in nanoseconds.
 */
typedef unsigned long long int xme_hal_time_timeInterval_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

#ifdef XME_ASSERT_NONFATAL_MODE
/**
 * \var    _xme_assert_assertionFailureCounter
 *
 * \brief  Counts the number of assertion failures that have happened until now.
 *         This can be used to check whether an expected assertion failure is
 *         actually thrown.
 *
 * \note   This is an internal variable. Do not use it directly.
 *         Instead, use one of the macros named below.
 *
 * \note   Since "normal" assertion failures terminate the program, this variable
 *         is only defined if XME_ASSERT_NONFATAL_MODE is defined, which makes
 *         assertion failures "non-fatal".
 *
 * \see    ::EXPECT_XME_ASSERTION_FAILURE()
 * \see    ::EXPECT_XME_ASSERTION_FAILURES()
 * \see    ::ASSERT_XME_ASSERTION_FAILURE()
 * \see    ::ASSERT_XME_ASSERTION_FAILURES()
 * \see    ::EXPECT_NO_XME_ASSERTION_FAILURES()
 * \see    ::ASSERT_NO_XME_ASSERTION_FAILURES()
 */
extern unsigned int _xme_assert_assertionFailureCounter;
#endif // #ifdef XME_ASSERT_NONFATAL_MODE

#if defined(DEBUG) || defined(DOXYGEN)
/**
 * \var    _xme_assert_noSideEffects
 *
 * \brief  Static variable whose name will be printed in non-fatal assertion
 *         failure messages instead of a cryptic number.
 *
 * \see    XME_ASSERT_NO_SIDE_EFFECTS()
 */
static const int _xme_assert_noSideEffects = 1;
#endif // #if defined(DEBUG) || defined(DOXYGEN)

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/core/defines_arch.h"

#include "xme/core/deprecated.h"

#endif // #ifndef XME_DEFINES_H
