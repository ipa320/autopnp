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
 * $Id: defines_arch.h 5011 2013-09-09 16:06:10Z geisinger $
 */

/**
 * \file
 *         Generic definition and defines (architecture specific part:
 *         generic embedded implementation).
 */

#ifndef XME_DEFINES_ARCH_H
#define XME_DEFINES_ARCH_H

#ifndef XME_DEFINES_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_DEFINES_H

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

// Some of the do/while loops in the following macros will lead to the code
// for recovery and rval to be never executed. However, the compiler will
// perform a syntax check, which is what we are after here. The compiler will
// optimize the code by removing the unreachable statements anyway.
#if defined(DEBUG) || defined(DOXYGEN)

    // Debug mode:
    // In debug mode, assertions will by default halt the system so it can be
    // inspected with a debugger. If nonfatal assertion mode is selected (see
    // below), a message will be printed to the console instead (for use in
    // unit tests).
    #ifdef XME_ASSERT_NONFATAL_MODE
        #define _XME_ASSERT_HANDLER(condition) ((!!(condition)) || ((void) xme_fallback_printf("Assertion failed (nonfatal mode): %s, file %s, line %d\n", #condition, __FILE__, xme_assert_assertionFailureCounter++ ? __LINE__ : __LINE__), 0))
    #else // #ifdef XME_ASSERT_NONFATAL_MODE
        #define _XME_ASSERT_HANDLER(condition) ((!!(condition)) || xme_assert_endlessLoop())
    #endif // #ifdef XME_ASSERT_NONFATAL_MODE

    #define XME_ASSERT(condition) do { if (!_XME_ASSERT_HANDLER(condition)) { return XME_STATUS_UNEXPECTED; } } while (0)
    #define XME_ASSERT_RVAL(condition, rval) do { if (!_XME_ASSERT_HANDLER(condition)) { return rval; } } while (0)
    #define XME_ASSERT_NORVAL(condition) do { if (!_XME_ASSERT_HANDLER(condition)) { return; } } while (0)

#else // #if defined(DEBUG) || defined(DOXYGEN)

    // Release mode:
    // In release mode, all assertions will effectively be NOPs, but we
    // still want to let the compiler check the syntax of all parameters.
    #define _XME_ASSERT_HANDLER(condition) ((void) (condition), 1)
    #define XME_ASSERT(condition) do { while (0) { (void) (condition); return XME_STATUS_UNEXPECTED; } } while (0)
    #define XME_ASSERT_RVAL(condition, rval) do { while (0) { (void) (condition); return rval; } } while (0)
    #define XME_ASSERT_NORVAL(condition) do { while (0) { (void) (condition); return; } } while (0)

#endif // #if defined(DEBUG) || defined(DOXYGEN)

#define XME_ASSERT_NO_SIDE_EFFECTS(condition) (condition)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

#ifndef NDEBUG
/**
 * \brief  Goes to an endless loop, which is typically used in reaction to
 *         severe problems like assertion failures.
 *
 * \return Always return zero.
 */
int
xme_assert_endlessLoop(void);
#endif // #ifndef NDEBUG

XME_EXTERN_C_END

#endif // #ifndef XME_DEFINES_ARCH_H
