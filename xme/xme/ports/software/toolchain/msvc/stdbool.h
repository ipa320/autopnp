/**
 * Copyright (c) 2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: stdbool.h 6263 2014-01-08 13:42:49Z geisinger $
 *
 * File:
 *         <stdbool.h> for Microsoft Visual C++.
 */

#ifndef STDBOOL_H_
#define STDBOOL_H_

/**
 * Notice    - Modified and re-released under Apache 2 license by fortiss GmbH
 *             for use in CHROMOSOME. Original author and license below.
 *
 * stdbool.h - ISO C99 Boolean type
 * Author    - Bill Chatfield
 * E-mail    - bill underscore chatfield at yahoo dot com
 * Copyright - You are free to use for any purpose except illegal acts
 * Warrenty  - None: don't blame me if it breaks something
 *
 * In ISO C99, stdbool.h is a standard header and _Bool is a keyword, but
 * some compilers don't offer these yet. This header file is an
 * implementation of the standard ISO C99 stdbool.h header file. It checks
 * for various compiler versions and defines things that are missing in
 * those versions.
 *
 * The GNU and Watcom compilers include a stdbool.h, but the Microsoft
 * compilers do not.
 *
 * See http://predef.sourceforge.net/precomp.html for compile macros.
 */

/**
 * Microsoft C/C++ version 14.00.50727.762, which comes with Visual C++ 2005,
 * version 15.00.30729.01, which comes with Visual C++ 2008, and
 * version 16.0.40219.1, which comes with Visual C++ 2010, and
 * version 17.0.60610.1, which comes with Visual C++ 2012, do not
 * define _Bool.
 */
#ifdef _MSC_VER
#if !defined(__cplusplus) && _MSC_VER <= 1700

// bool is not available in C89 mode, but "true" and "false" might have been
// declared by header files included before this one (e.g., "rtwtypes.h"),
// so we need to be careful here.
#if defined(true) && defined(false)
typedef enum
{
    _Bool_promotes_to_int = -1,
    _Bool_false = 0,
    _Bool_true = 1
} _Bool;
#else // #if defined(true) && defined(false)
typedef enum
{
    _Bool_promotes_to_int = -1,
    false = 0,
    true = 1
} _Bool;
#endif // #if defined(true) && defined(false)

#else // #if !defined(__cplusplus) && _MSC_VER <= 1700

// bool is available in C++ mode
typedef bool _Bool;

#endif // #if !defined(__cplusplus) && _MSC_VER <= 1700
#endif // #ifdef _MSC_VER

/**
 * Define the Boolean macros only if they are not already defined.
 * In C++ mode, we assume that these constants are defined.
 */
#if !defined(__bool_true_false_are_defined) && !defined(__cplusplus)
#if !defined(bool)
#define bool _Bool
#endif // #if !defined(bool)
#if !defined(false)
#define false 0
#endif // #if !defined(false)
#if !defined(true)
#define true 1
#endif // #if !defined(true)
#define __bool_true_false_are_defined 1
#endif // #if !defined(__bool_true_false_are_defined) && !defined(__cplusplus)

#endif // #ifndef STDBOOL_H_
