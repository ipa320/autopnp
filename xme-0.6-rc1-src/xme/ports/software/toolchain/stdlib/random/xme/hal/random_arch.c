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
 * $Id: random_arch.c 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         Random number generator abstraction (architecture specific part:
 *         generic OS based implementation).
 */

/**
 * \addtogroup hal_random 
 * @{
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
xme_hal_random_init(void)
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
xme_hal_random_fini(void)
{
    // Deregister the calling thread, which should be the "main thread",
    // if such concept exists on this platform
    xme_hal_random_deregisterThread();
}

xme_status_t
xme_hal_random_registerThread(void)
{
    // Initialize thread-specific random seed
    srand((unsigned int)time(NULL));

    return XME_STATUS_SUCCESS;
}

void
xme_hal_random_deregisterThread(void)
{
    // Nothing to do
}

uint16_t
xme_hal_random_rand(void)
{
#if RAND_MAX < 0xFFFF
    // According to documentation, "RAND_MAX is granted to be at least 32767".
    // If it is between 32767 and 65535, we have to generate a "true" 16 bit
    // random number by randomly choosing the most significant bit.
    return (uint16_t) (rand() | (rand() << 15));
#else
    return (uint16_t) rand();
#endif
}

uint16_t
xme_hal_random_randRange(uint16_t min, uint16_t max)
{
    uint16_t r;

    XME_ASSERT_RVAL(max >= min, max);
    XME_CHECK(max != min, max);

#if RAND_MAX < 0xFFFF
    // According to documentation, "RAND_MAX is granted to be at least 32767".
    // But max-min could be larger than XME_HAL_RANDOM_RAND_MAX, causing problems.
    // For example, if max is 65535, min is 0 and RAND_MAX is 0x7FFF, then this
    // function would only return even numbers, which is obviously bad.
    // Hence the following "workaround" is used in case the most significant bit is
    // not affected by rand().
    r = (uint16_t) (rand() | (rand() << 15));
#else
    r = (uint16_t) rand();
#endif

    // The shift by 0.5f is necessary to compensate for the effects introduced when
    // casting to uint16_t: in this case, the decimal places are simply thrown away.
    // Hence, the lower value receives more hits and the highest value receives less
    // hits. This is compensated by shifting the whole range of values up by 0.5f
    // before casting.
    return (uint16_t) (min + (max-min) * ((float)r/XME_HAL_RANDOM_RAND_MAX) + 0.5f);
}

/**
 * @}
 */
