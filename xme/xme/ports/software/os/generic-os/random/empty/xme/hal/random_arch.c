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
 * $Id: random_arch.c 3345 2013-05-17 12:07:58Z geisinger $
 */

/**
 * \file
 *         Random number generator abstraction (architecture specific part:
 *         generic "embedded" implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/random.h"

#include <time.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_random_init()
{
	// Register the calling thread, which should be the "main thread",
	// if such concept exists on this platform
	{
	    xme_status_t result = xme_hal_random_registerThread();
		XME_ASSERT(XME_STATUS_SUCCESS == result);
	}

	return XME_STATUS_SUCCESS;
}

void
xme_hal_random_fini()
{
	// Deregister the calling thread, which should be the "main thread",
	// if such concept exists on this platform
	xme_hal_random_deregisterThread();
}

xme_status_t
xme_hal_random_registerThread()
{
	// Nothing to do

	return XME_STATUS_SUCCESS;
}

void
xme_hal_random_deregisterThread()
{
	// Nothing to do
}

uint16_t
xme_hal_random_rand()
{
	// Nothing to do

	return 0;
}

uint16_t
xme_hal_random_randRange(uint16_t min, uint16_t max)
{
	XME_ASSERT_RVAL(max >= min, max);

	return min;
}
