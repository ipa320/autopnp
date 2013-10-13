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
 * $Id: math_arch.c 3345 2013-05-17 12:07:58Z geisinger $
 */

/**
 * \file
 *         Math functions (architecture specific part: generic C stub).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/math_arch.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
uint32_t
xme_hal_math_ceilPowerOfTwo(uint32_t x)
{
	if ((0 < x) && (x <= 0x80000000))
	{
		uint32_t mask = 0x80000000;
		do
		{
			if (0 != (x & mask)) {
				return (x == mask) ? mask : mask << 1;
			}
			mask >>= 1;
		} while (1);
	}

    return 0;
}

uint32_t
xme_hal_math_floorPowerOfTwo(uint32_t x)
{
	if (0 < x)
	{
		uint32_t mask = 0x80000000;
		do
		{
			if (0 != (x & mask)) {
				return mask;
			}
			mask >>= 1;
		} while (1);
	}

	return 0;
}
