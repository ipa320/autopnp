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
 * $Id: math_arch.h 3345 2013-05-17 12:07:58Z geisinger $
 */

/**
 * \file
 *         Math functions (architecture specific part: x86).
 */

#ifndef XME_HAL_MATH_ARCH_H
#define XME_HAL_MATH_ARCH_H

/**
 * \ingroup hal_math
 *  @{
 *
 * \defgroup hal_math_x86 Math functions (x86 architecture)
 *  @{
 *
 * \brief  Math functions.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

#ifdef _MSC_VER
/**
 * \brief  Returns the least power of 2 greater than or equal to the given value.
 *
 * \note   Note that for x=0 and for x>2147483648 this returns 0!
 *         Implemented after http://www.flipcode.com/archives/Some_More_Power_Of_2_Utility_Functions.shtml.
 *
 * \param  x Unsigned input number.
 * \return Least power of 2 greater than or equal to the given value.
 */
/*__declspec(naked)*/ uint32_t __fastcall
xme_hal_math_ceilPowerOfTwo(uint32_t x);

/**
 * \brief  Returns the greatest power of 2 less than or equal to the given value.
 *
 * \note   Note that for x=0 this returns 0!
 *         Implemented after http://www.flipcode.com/archives/Some_More_Power_Of_2_Utility_Functions.shtml.
 *
 * \param  x Unsigned input number.
 * \return Greatest power of 2 less than or equal to the given value.
 */
/*__declspec(naked)*/ uint32_t __fastcall
xme_hal_math_floorPowerOfTwo(uint32_t x);

#elif defined (__GNUC__) /* #ifdef _MSC_VER */
uint32_t
#if defined(__i386__) && ((__GNUC__ >= 4) || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4))
__attribute__((fastcall))
#endif
xme_hal_math_ceilPowerOfTwo(uint32_t x);
/**
 * \brief  Returns the greatest power of 2 less than or equal to the given value.
 *
 * \note   Note that for x=0 this returns 0!
 *         Implemented after http://www.flipcode.com/archives/Some_More_Power_Of_2_Utility_Functions.shtml.
 *
 * \param  x Unsigned input number.
 * \return Greatest power of 2 less than or equal to the given value.
 */

uint32_t
#if defined(__i386__) && ((__GNUC__ >= 4) || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4))
__attribute__((fastcall))
#endif
xme_hal_math_floorPowerOfTwo(uint32_t x);

#else /* #elif defined (__GNUC__) */
#error "Architecture specific implementation of math functions could not be found. Create a port, or use the generic implementation in arch/gen_c."
#endif /* #elif defined (__GNUC__) */

XME_EXTERN_C_END

/**@} // end of specific hal_math_x86 implementation */
/**@} // end of specific hal_math in group */

#endif /* #ifndef XME_HAL_MATH_ARCH_H */
