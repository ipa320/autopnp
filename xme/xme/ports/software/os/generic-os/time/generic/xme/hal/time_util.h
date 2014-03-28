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
 * $Id: time_util.h 5029 2013-09-10 13:29:16Z geisinger $
 */

/**
 * \file
 *         Time abstraction (generic OS-based utils).
 */

#ifndef XME_HAL_TIME_UTIL_H
#define XME_HAL_TIME_UTIL_H

#ifndef XME_HAL_TIME_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_util" suffix) instead.
#endif // #ifndef XME_HAL_TIME_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Gets the time interval in seconds. 
 *
 * \param timeInterval the input time interval.
 *
 * \return the corresponding timeInterval expressed in seconds. 
 */
static INLINE
uint32_t
xme_hal_time_timeIntervalInSeconds
(
    xme_hal_time_timeInterval_t timeInterval
)
{
    return (uint32_t)(timeInterval/1000000000);
}

/**
 * \brief Gets the time interval in milliseconds. 
 *
 * \param timeInterval the input time interval.
 *
 * \return the corresponding timeInterval expressed in milliseconds. 
 */
static INLINE
uint64_t
xme_hal_time_timeIntervalInMilliseconds
(
    xme_hal_time_timeInterval_t timeInterval
)
{
    return (uint32_t)(timeInterval/1000000);
}

/**
 * \brief Gets the time interval in microseconds. 
 *
 * \param timeInterval the input time interval.
 *
 * \return the corresponding timeInterval expressed in microseconds. 
 */
static INLINE
uint64_t
xme_hal_time_timeIntervalInMicroseconds
(
    xme_hal_time_timeInterval_t timeInterval
)
{
    return (uint32_t)(timeInterval/1000);
}

/**
 * \brief Gets the time interval from an incoming time interval expressed in seconds. 
 *
 * \param timeInterval_s the input time interval expressed in seconds.
 *
 * \return the corresponding timeInterval expressed in nanoseconds. 
 */
static INLINE
xme_hal_time_timeInterval_t
xme_hal_time_timeIntervalFromSeconds
(
    uint32_t timeInterval_s
)
{
    return ((xme_hal_time_timeInterval_t)timeInterval_s) * 1000000000;
}

/**
 * \brief Gets the time interval from an incoming time interval expressed in milliseconds. 
 *
 * \param timeInterval_ms the input time interval expressed in milliseconds.
 *
 * \return the corresponding timeInterval expressed in nanoseconds. 
 */
static INLINE
xme_hal_time_timeInterval_t
xme_hal_time_timeIntervalFromMilliseconds
(
    uint64_t timeInterval_ms
)
{
    return ((xme_hal_time_timeInterval_t)timeInterval_ms) * 1000000;
}

/**
 * \brief Gets the time interval from an incoming time interval expressed in microseconds. 
 *
 * \param timeInterval_us the input time interval expressed in microseconds.
 *
 * \return the corresponding timeInterval expressed in nanoseconds. 
 */
static INLINE
xme_hal_time_timeInterval_t
xme_hal_time_timeIntervalFromMicroseconds
(
    uint64_t timeInterval_us
)
{
    return ((xme_hal_time_timeInterval_t)timeInterval_us) * 1000;
}

XME_EXTERN_C_END

#endif // #ifndef XME_HAL_TIME_UTIL_H
