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
 * $Id: math.h 2885 2013-04-11 18:01:06Z geisinger $
 */

/** 
 * \file
 * \brief Math functions.
 */

#ifndef XME_HAL_MATH_H
#define XME_HAL_MATH_H

/** 
 * \defgroup hal_math Math functions
 * @{ 
 *
 * \brief  Helping Math functions. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/math_arch.h"
#include <math.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/**
 * \def    XME_HAL_MATH_IS_POWER_OF_2
 *
 * \brief  Returns whether a number is a power of two.
 *
 * \param  number Number to test.
 * \param  bits Number of bits to respect during the check.
 *         Must be one of 8, 16, 32, and 64.
 * \return Returns true if the given number is a power of two and false otherwise.
 */
#define XME_HAL_MATH_IS_POWER_OF_2(number, bits) ((int##bits##_t)(number) && (((int##bits##_t)(number) & ((int##bits##_t)(number)-1)) == 0))

/**
 * \def    XME_HAL_MATH_MIN
 *
 * \brief  Returns the minimum value of of two given values.
 *
 * \param  a First number.
 * \param  b Second number.
 * \return Returns a if a is smaller than b and b otherwise.
 */
#define XME_HAL_MATH_MIN(a, b) ((a)<(b)?(a):(b))

/**
 * \def    XME_HAL_MATH_MAX
 *
 * \brief  Returns the maximum value of of two given values.
 *
 * \param  a First number.
 * \param  b Second number.
 * \return Returns a if a is larger than b and b otherwise.
 */
#define XME_HAL_MATH_MAX(a, b) ((a)>(b)?(a):(b))

/**
 * @}
 */


#endif // #ifndef XME_HAL_MATH_H
