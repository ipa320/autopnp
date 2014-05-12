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
 * $Id: math_arch.c 4598 2013-08-07 14:28:43Z ruiz $
 */

/**
 * \file
 *         Math functions (architecture specific part: x86).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/math_arch.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
#ifdef _MSC_VER
#pragma warning(disable: 4100)

__declspec(naked) uint32_t __fastcall
xme_hal_math_ceilPowerOfTwo(uint32_t x)
{
    __asm
    {
        xor eax,eax
        dec ecx
        bsr ecx,ecx
        cmovz ecx,eax
        setnz al
        inc eax
        shl eax,cl
        ret
    }
}

__declspec(naked) uint32_t __fastcall
xme_hal_math_floorPowerOfTwo(uint32_t x) {
    __asm
    {
        xor eax,eax
        bsr ecx,ecx
        setnz al
        shl eax,cl
        ret
    }
}

#elif defined (__GNUC__) /* #ifdef _MSC_VER */
uint32_t
#if defined(__i386__) && ((__GNUC__ >= 4) || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4))
__attribute__((fastcall))
#endif
xme_hal_math_ceilPowerOfTwo(uint32_t x)
{
    unsigned eax;
    __asm__
    (
        "xor %%eax,%%eax\n    "
        "dec %%ecx\n    "
        "bsr %%ecx,%%ecx\n    "
        "cmovz %%ecx,%%eax\n    "
        "setnz %%al\n    "
        "inc %%eax\n    "
        "shl %%cl,%%eax\n    "
        : "=a" (eax)
        : "c" (x)
    );
    return eax;
}

uint32_t
#if defined(__i386__) && ((__GNUC__ >= 4) || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4))
__attribute__((fastcall))
#endif
xme_hal_math_floorPowerOfTwo(uint32_t x)
{
    unsigned eax;
    __asm__
    (
        "xor %%eax,%%eax\n    "
        "bsr %%ecx,%%ecx\n    "
        "setnz %%al\n    "
        "shl %%cl,%%eax\n    "
        : "=a" (eax)
        : "c" (x)
    );
    return eax;
}
#else /* #elif defined (__GNUC__) */
#error "Architecture specific implementation of math functions could not be found. Create a port, or use the generic implementation in arch/gen_c."
#endif /* #elif defined (__GNUC__) */
