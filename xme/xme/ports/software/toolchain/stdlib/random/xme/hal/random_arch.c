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
 * $Id: random_arch.c 7661 2014-03-03 18:26:04Z geisinger $
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

#ifdef WIN32
#include <Windows.h>
#else // #ifdef WIN32
#ifdef linux
#include <sys/types.h>
#include <unistd.h>
#if _REENTRANT
#include <pthread.h>
#endif // #if _REENTRANT
#endif // #ifdef linux
#endif // #ifdef WIN32

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
// Adapted from http://burtleburtle.net/bob/hash/doobs.html
// License:
// ----------
// By Bob Jenkins, 1996.  bob_jenkins@burtleburtle.net.  You may use this
// code any way you wish, private, educational, or commercial.  It's free.
// ----------
// See also http://web.archive.org/web/20070111091013/http://www.concentric.net/~Ttwang/tech/inthash.htm
// (archived version of http://www.concentric.net/~Ttwang/tech/inthash.htm)
/**
 * \brief Mix 3 32-bit values reversibly.
 *
 * \details For every delta with one or two bits set, and the deltas of all
 *          three high bits or all three low bits, whether the original value
 *          of a, b, c is almost all zero or is uniformly distributed,
 *           - If mix() is run forward or backward, at least 32 bits in a, b, c
 *             have at least 1/4 probability of changing.
 *           - If mix() is run forward, every bit of c will change between 1/3
 *             and 2/3 of the time (well, 22/100 and 78/100 for some 2-bit
 *             deltas).
 *          mix96Bits() was built out of 36 single-cycle latency instructions
 *          in a structure that could supported 2x parallelism.
 *
 * \param[in] a First 32-bit value to mix.
 * \param[in] b Second 32-bit value to mix.
 * \param[in] c Third 32-bit value to mix.
 *
 * \return Mixed 32-bit value.
 */
int mix96Bits(int a, int b, int c);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
int mix96Bits(int a, int b, int c)
{
  a -= b; a -= c; a ^= (c >> 13);
  b -= c; b -= a; b ^= (a <<  8);
  c -= a; c -= b; c ^= (b >> 13);
  a -= b; a -= c; a ^= (c >> 12);
  b -= c; b -= a; b ^= (a << 16);
  c -= a; c -= b; c ^= (b >>  5);
  a -= b; a -= c; a ^= (c >>  3);
  b -= c; b -= a; b ^= (a << 10);
  c -= a; c -= b; c ^= (b >> 15);
  return c;
}

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
    int tm, cl, pid = 0, tid = 0;
    tm = (int) time(NULL);
    cl = (int) clock();
#ifdef WIN32
    pid = (int) GetCurrentProcessId();
    tid = (int) GetCurrentThreadId();
#else // #ifdef WIN32
#ifdef linux
    pid = (int) getpid();
#if _REENTRANT
    tid = (int) pthread_self();
#endif // #if _REENTRANT
#endif // #ifdef linux
#endif // #ifdef WIN32

    srand(mix96Bits(tm, cl, pid + tid));

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
