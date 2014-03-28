/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: defines_arch.h 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Generic definition and defines (architecture specific part:
 *         generic OS based implementation).
 *
 */

#ifndef XME_DEFINES_ARCH_H
#define XME_DEFINES_ARCH_H

#ifndef XME_DEFINES_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_DEFINES_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#ifdef __cplusplus
    #include <cassert>
#else
    #include <assert.h>
#endif

#ifdef DEBUG
    #ifdef __cplusplus
        #include <exception>
        #include <sstream>
        #include <string>
    #endif // #ifdef __cplusplus
#endif // #ifdef DEBUG

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

// Some of the do/while loops in the following macros will lead to the code
// for recovery and rval to be never executed. However, the compiler will
// perform a syntax check, which is what we are after here. The compiler will
// optimize the code by removing the unreachable statements anyway.
#if defined(DEBUG) || defined(DOXYGEN)

    // Debug mode
    #ifdef __cplusplus

        // C++ Debug mode:
        // In C++ Debug mode, failed assertions will throw an exception.
        class AssertionFailedException : public std::exception
        {
        public:
            AssertionFailedException(const char* expression, const char* file, long int line)
            {
                std::stringstream s;
                s << "Assertion failed: " << expression << " at " << file << ':' << line;
                _expression = s.str();
            }

            virtual ~AssertionFailedException(void) throw () {}

            virtual const char* what(void) const throw ()
            {
                return _expression.c_str();
            }
        protected:
            std::string _expression;
        };

        #ifdef XME_ASSERT_NONFATAL_MODE
            #define _XME_ASSERT_HANDLER(condition) ((!!(condition)) || (throw AssertionFailedException(#condition, __FILE__, _xme_assert_assertionFailureCounter++ ? __LINE__ : __LINE__), 0))
        #else // #ifdef XME_ASSERT_NONFATAL_MODE
            #define _XME_ASSERT_HANDLER(condition) ((!!(condition)) || (throw AssertionFailedException(#condition, __FILE__, __LINE__), 0))
        #endif // #ifdef XME_ASSERT_NONFATAL_MODE

    #else // #ifdef __cplusplus
        // Non-C++ Debug mode:
        // In non-C++ Debug mode, failed assertions will by default force abort() to be called,
        // effectively terminating the program. If nonfatal assertion mode is selected (see below),
        // a message will be printed to the console instead (for use in unit tests).
        // Depending on the platform, more debugging options might be offered.
        #ifdef XME_ASSERT_NONFATAL_MODE
            // Nonfatal model will only print a message to the console
            #define _XME_ASSERT_HANDLER(condition) ((!!(condition)) || ((void) xme_fallback_printf("Assertion failed (nonfatal mode): %s, file %s, line %d\n", #condition, __FILE__, _xme_assert_assertionFailureCounter++ ? __LINE__ : __LINE__), 0))
        #else // #ifdef XME_ASSERT_NONFATAL_MODE
            // Fatal mode terminates the program
            #define _XME_ASSERT_HANDLER(condition) ((!(condition))?(xme_print_stacktrace(), assert(condition), 1):(1))
        #endif // #ifdef XME_ASSERT_NONFATAL_MODE

    #endif // #ifdef __cplusplus

    #define XME_ASSERT(condition) do { (void) xme_assert_checkForSideEffects("XME_ASSERT", #condition, __FILE__, __LINE__, 1); if (!_XME_ASSERT_HANDLER(condition)) { return XME_STATUS_UNEXPECTED; } } while (0)
    #define XME_ASSERT_RVAL(condition, rval) do { (void) xme_assert_checkForSideEffects("XME_ASSERT_RVAL", #condition, __FILE__, __LINE__, 1); if (!_XME_ASSERT_HANDLER(condition)) { return rval; } } while (0)
    #define XME_ASSERT_NORVAL(condition) do { (void) xme_assert_checkForSideEffects("XME_ASSERT_NORVAL", #condition, __FILE__, __LINE__, 1); if (!_XME_ASSERT_HANDLER(condition)) { return; } } while (0)

#else // #if defined(DEBUG) || defined(DOXYGEN)

    // Release mode:
    // In release mode, all assertions will effectively be NOPs, but we
    // still want to let the compiler check the syntactical correctness
    // of all parameters to avoid surprises when compiling in Debug mode.
    #define _XME_ASSERT_HANDLER(condition) ((void) (condition), 1)
    #define XME_ASSERT(condition) do { while (0) { (void) (condition); return XME_STATUS_UNEXPECTED; } } while (0)
    #define XME_ASSERT_RVAL(condition, rval) do { while (0) { (void) (condition); return rval; } } while (0)
    #define XME_ASSERT_NORVAL(condition) do { while (0) { (void) (condition); return; } } while (0)

#endif // #if defined(DEBUG) || defined(DOXYGEN)

// The name of this define has to match what is expected in xme_assert_checkForSideEffects()!
#if defined(DEBUG) || defined(DOXYGEN)
    #define XME_ASSERT_NO_SIDE_EFFECTS(condition) (_xme_assert_noSideEffects && (condition))
#else // #if defined(DEBUG) || defined(DOXYGEN)
    #define XME_ASSERT_NO_SIDE_EFFECTS(condition) (1 && (condition))
#endif // #if defined(DEBUG) || defined(DOXYGEN)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

#ifndef NDEBUG
/**
 * \brief  Uses a heuristic to check for side effects within conditions of
 *         assertion macros.
 *
 *         It is dangerous for conditions of assertion macros like XME_ASSERT(),
 *         XME_ASSERT_RVAL() and XME_ASSERT_NORVAL() to contain side effects,
 *         because those side effects will not be present in release builds,
 *         where those macros are effectively NOPs.
 *
 *         The heuristic detects the following situations as statements with
 *         (potential) side effects:
 *          - A C identifier that is followed by an opening bracket (i.e., a
 *            function or macro call).
 *          - An assignment operator ('=', '+=', '-=', '*=', '/=', or '%=',
 *            but not '==').
 *          - An increment or decrement operator ('++' or '--', prefix or
 *            postfix).
 *         In case a side effect is detected, a nonzero value is returned and
 *         a warning may be issued depending on the emitWarning parameter.
 *
 *         Note that this heuristic does not cover all situations where side
 *         effects might occur, and it might yield false positives.
 *         See notes on how to acknowledge false positives.
 *
 * \note   To prevent detection of side effects for a specific condition (for
 *         example in case of known false positives or in situations where it
 *         is known that the side effects are not required in release builds),
 *         make sure that the condition is enclosed by the macro
 *         XME_ASSERT_NO_SIDE_EFFECTS().
 *         See the examples section of the XME_ASSERT_NO_SIDE_EFFECTS() macro
 *         for details.
 *
 *     odo   This is a runtime check for side effects.
 *         Of course it would be better to use static code analysis to detect
 *         such a circumstance. Until we have such mechanisms in place, this
 *         is a quick way to avoid common problems with assertion macros.
 *
 * \param  macro Name of the macro that called this function (for debug message
 *         output).
 * \param  condition String representation of the condition to check for
 *         potential side effects.
 * \param  file Name of the file that contains the assertion macro (for debug
 *         message output).
 * \param  line Line within the file that contains the assertion macro (for
 *         debug message output).
 * \param  emitWarning If set to a nonzero value, the function will emit a
 *         warning if condition contains statements with potential side effects.
 *
 * \return Returns zero if no side effects could be detected in condition (as
 *         defined above) or condition contains the XME_ASSERT_NO_SIDE_EFFECTS()
 *         macro to mask potential side effects and a nonzero value otherwise.
 *
 * \see    XME_ASSERT(), XME_ASSERT_RVAL(), XME_ASSERT_NORVAL(),
 *         XME_ASSERT_NO_SIDE_EFFECTS()
 */
int
xme_assert_checkForSideEffects(const char* macro, const char* condition, const char* file, const unsigned long line, int emitWarning);

void
xme_print_stacktrace(void);
#endif // #ifndef NDEBUG

XME_EXTERN_C_END

#endif // #ifndef XME_DEFINES_ARCH_H
