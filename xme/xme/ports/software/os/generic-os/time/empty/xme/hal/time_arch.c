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
 * $Id: time_arch.c 3090 2013-04-26 14:33:46Z geisinger $
 */

/**
 * \file
 *         Time abstraction (architecture specific part: FreeRTOS
 *         implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Constants                                                            ***/
/******************************************************************************/
const xme_hal_time_timeHandle_t XME_HAL_TIME_INVALID_TIME_HANDLE = UINT32_MAX;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
bool
xme_hal_time_isValidTimeHandle
(
	xme_hal_time_timeHandle_t timeHandle
)
{
	return (XME_HAL_TIME_INVALID_TIME_HANDLE != timeHandle);
}

xme_hal_time_timeHandle_t
xme_hal_time_getCurrentTime(void)
{
	return (xme_hal_time_timeHandle_t)0;
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

	return (time1 < time2) ? -1 : (time1 > time2 ? 1 : 0);
}

xme_hal_time_timeHandle_t
xme_hal_time_offsetTime
(
	xme_hal_time_timeHandle_t time,
	xme_hal_time_offsetOperation_t operation,
	xme_hal_time_timeInterval_t offset
)
{
	XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_time_isValidTimeHandle(time)), XME_HAL_TIME_INVALID_TIME_HANDLE);

	switch (operation)
	{
		case XME_HAL_TIME_OFFSET_OPERATION_ADD:
			time += offset;
			break;

		case XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT:
			if (time >= offset)
			{
				time -= offset;
			}
			else
			{
				time = 0;
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

	return stopTime - startTime;
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
	XME_UNUSED_PARAMETER(absoluteTime);

	return (xme_hal_time_timeHandle_t)0;
}

xme_status_t
xme_hal_time_absoluteTimeFromHandle
(
	xme_hal_time_timeHandle_t timeHandle,
	struct tm* absoluteTime
)
{
	XME_UNUSED_PARAMETER(timeHandle);
	XME_UNUSED_PARAMETER(absoluteTime);

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
	XME_UNUSED_PARAMETER(buffer);
	XME_UNUSED_PARAMETER(sizeInBytes);
	XME_UNUSED_PARAMETER(format);
	XME_UNUSED_PARAMETER(timeHandle);

	return 0;
}
