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
 * $Id: time_arch.c 3090 2013-04-26 14:33:46Z geisinger $
 */

/**
 * \file
 *         Time abstraction (architecture specific part: POSIX implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/time.h"

#include <limits.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// Begin from http://www.velocityreviews.com/forums/t681985-the-maximum-value-held-by-a-time_t.html.

#define BUILD_ASSERT__(EXPR) sizeof(struct { unsigned int build_assert_failed : (EXPR) ? 1 : -1; })
#define BUILD_ASSERT_DECL(EXPR) extern int (*build_assert(void))[BUILD_ASSERT__(EXPR)]

#define TYPE_IS_INTEGER(TYPE) ((TYPE)1.5 == (TYPE)1)
#define TYPE_IS_SIGNED(TYPE) ((TYPE)0 > (TYPE)-1)
#define TYPE_VALUE_BITS(TYPE) (sizeof(TYPE) * CHAR_BIT - TYPE_IS_SIGNED(TYPE))

//#define TYPE_MINIMUM(TYPE) (TYPE_IS_SIGNED(TYPE) ? ~(TYPE)0 << TYPE_VALUE_BITS(TYPE) : 0)
#define TYPE_MAXIMUM(TYPE) (TYPE_IS_SIGNED(TYPE) ? ~(~(TYPE)0 << TYPE_VALUE_BITS(TYPE)) : (TYPE)-1)

BUILD_ASSERT_DECL(TYPE_IS_INTEGER(time_t)); // C standard allows floating-point time_t, but we don't support it.
BUILD_ASSERT_DECL(TYPE_IS_SIGNED(time_t)); // Be notified when we come across an unsigned time_t platform (just in case).

#define TIME_MAX TYPE_MAXIMUM(time_t)
//#define TIME_MIN TYPE_MINIMUM(time_t)

// End from http://www.velocityreviews.com/forums/t681985-the-maximum-value-held-by-a-time_t.html.

/******************************************************************************/
/***   Constants                                                            ***/
/******************************************************************************/
const xme_hal_time_timeHandle_t XME_HAL_TIME_INVALID_TIME_HANDLE =
{
	TIME_MAX, // tv_sec
	LONG_MAX // tv_nsec
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
bool
xme_hal_time_isValidTimeHandle
(
	xme_hal_time_timeHandle_t timeHandle
)
{
	return (XME_HAL_TIME_INVALID_TIME_HANDLE.tv_sec != timeHandle.tv_sec ||
		XME_HAL_TIME_INVALID_TIME_HANDLE.tv_nsec != timeHandle.tv_nsec);
}

xme_hal_time_timeHandle_t
xme_hal_time_getCurrentTime(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (xme_hal_time_timeHandle_t)ts;
}

int8_t
xme_hal_time_compareTime
(
	xme_hal_time_timeHandle_t time1,
	xme_hal_time_timeHandle_t time2
)
{
	XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(time1)), -1);
	XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(time2)), -1);

	if (time1.tv_sec == time2.tv_sec)
	{
		if (time1.tv_nsec < time2.tv_nsec)
		{
			return -1;
		}
		else if (time1.tv_nsec > time2.tv_nsec)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else if (time1.tv_sec < time2.tv_sec)
	{
		return -1;
	}
	else // if (time1.tv_sec > time2.tv_sec)
	{
		return 1;
	}
}

// Inspired by tv_util of libgpl:
// http://www.geonius.com/software/libgpl/tv_util.html
xme_hal_time_timeHandle_t
xme_hal_time_offsetTime
(
	xme_hal_time_timeHandle_t time,
	xme_hal_time_offsetOperation_t operation,
	xme_hal_time_timeInterval_t offset
)
{
	uint32_t intervalSeconds;
	long intervalNanoseconds;

	XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(time)), XME_HAL_TIME_INVALID_TIME_HANDLE);
	XME_ASSERT_RVAL(time.tv_nsec >= 0L, time);
	XME_ASSERT_RVAL(time.tv_nsec < 1000000000L, time);

	intervalSeconds = xme_hal_time_timeIntervalInSeconds(offset);
	intervalNanoseconds = offset % 1000000000ULL;

	switch (operation)
	{
		case XME_HAL_TIME_OFFSET_OPERATION_ADD:
			time.tv_sec += intervalSeconds;
			time.tv_nsec += intervalNanoseconds; // this is guaranteed to not overflow
			if (time.tv_nsec >= 1000000000LL)
			{
				time.tv_sec++;
				time.tv_nsec -= 1000000000LL;
			}
			break;

		case XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT:
			if ((time.tv_sec < (long)intervalSeconds) || ((time.tv_sec == (long)intervalSeconds) && (time.tv_nsec <= intervalNanoseconds)))
			{
				time.tv_sec = time.tv_nsec = 0;
			}
			else
			{
				time.tv_sec -= intervalSeconds;

				if (time.tv_nsec < intervalNanoseconds)
				{
					time.tv_nsec += 1000000000LL - intervalNanoseconds;
					time.tv_sec--;
				}
				else
				{
					time.tv_nsec -= intervalNanoseconds;
				}
			}
			break;

		default:
			XME_ASSERT_RVAL
			(
				XME_HAL_TIME_OFFSET_OPERATION_ADD == operation ||
				XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT == operation,
				time
			);
	}

	return time;
}

xme_hal_time_timeInterval_t
xme_hal_time_getTimeInterval
(
	xme_hal_time_timeHandle_t* startTime,
	bool reset
)
{
	xme_hal_time_timeInterval_t interval;
	xme_hal_time_timeHandle_t now;

	XME_ASSERT(NULL != startTime);
	XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(*startTime)), 0);

	now = xme_hal_time_getCurrentTime();
	interval = xme_hal_time_getTimeIntervalBetween(*startTime, now);

	if (reset)
	{
		*startTime = now;
	}

	return interval;
}

xme_hal_time_timeInterval_t
xme_hal_time_getTimeIntervalBetween
(
	xme_hal_time_timeHandle_t startTime,
	xme_hal_time_timeHandle_t stopTime
)
{
	XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(startTime)), 0);
	XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(stopTime)), 0);

	return ((xme_hal_time_timeInterval_t)(stopTime.tv_sec - startTime.tv_sec)) * 1000000000ULL + (stopTime.tv_nsec - startTime.tv_nsec);
}

void
xme_hal_time_destroyTimeHandle
(
	xme_hal_time_timeHandle_t timeHandle
)
{
	// Nothing to do
	XME_UNUSED_PARAMETER(timeHandle);
}

xme_hal_time_timeHandle_t
xme_hal_time_handleFromAbsoluteTime
(
	struct tm* absoluteTime
)
{
	struct timespec result;
	time_t time;

	XME_ASSERT_RVAL(NULL != absoluteTime, XME_HAL_TIME_INVALID_TIME_HANDLE);

	time = mktime(absoluteTime);
	
	XME_CHECK(-1 != time, XME_HAL_TIME_INVALID_TIME_HANDLE);

	result.tv_sec = (long)time;
	result.tv_nsec = 0;

	return result;
}

xme_status_t
xme_hal_time_absoluteTimeFromHandle
(
	xme_hal_time_timeHandle_t timeHandle,
	struct tm* absoluteTime
)
{
	struct timeval* tv = (struct timeval*) &timeHandle;
	struct tm* tempTime;
	time_t time = (time_t) tv->tv_sec;

	XME_ASSERT(NULL != absoluteTime);

	tempTime = localtime(&time);
	xme_fallback_memcpy(absoluteTime, tempTime, sizeof(struct tm));

	return XME_STATUS_SUCCESS;
}

size_t
xme_hal_time_formatTime
(
	char* buffer,
	size_t sizeInBytes,
	const char* format,
	xme_hal_time_timeHandle_t timeHandle
)
{
	struct timeval* tv;
	struct tm* tempTime;
	time_t time;

	tv = (struct timeval*) &timeHandle;
	time = tv->tv_sec;

	tempTime = localtime(&time);

	return strftime(buffer, sizeInBytes, format, tempTime);
}
