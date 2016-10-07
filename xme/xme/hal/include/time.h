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
 * $Id: time.h 5163 2013-09-25 10:10:46Z camek $
 */

/**
 * \file
 * \brief Time abstraction.
 */

#ifndef XME_HAL_TIME_H
#define XME_HAL_TIME_H

/**
 * \defgroup hal_time Time Abstraction
 * @{
 *
 * \brief This component defines a set of operation to work with time.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_hal_time_timeHandle_t
 * \brief  Handle for a point in time.
 *
 * \note   This representation of time is platform specific.
 *         It is not meant to be sent as part of a topic.
 *         Use the respective functions from this HAL component in order to
 *         convert the time handle into a literal value with clearly defined
 *         semantics before attempting to send it to another component.
 */
 // Defined in xme/hal/time_arch.h for the respective platform

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/time_arch.h"

/******************************************************************************/
/***   Constants                                                            ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \var  XME_HAL_TIME_INVALID_TIME_HANDLE
 * \brief  Invalid time handle.
 */
// Value defined in xme/hal/time_arch.c for the respective platform
extern const xme_hal_time_timeHandle_t XME_HAL_TIME_INVALID_TIME_HANDLE;

XME_EXTERN_C_END

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_hal_time_offsetOperation_t
 * \brief Defines the different operation allowed for a time offset.
 */
typedef enum
{
    XME_HAL_TIME_OFFSET_OPERATION_ADD = 0, ///< Shift the given point in time into the future by adding the offset interval.
    XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT = 1 ///< Shift the given point in time into the past by subtracting the offset interval.
}
xme_hal_time_offsetOperation_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Returns whether the given time handle is valid.
 *
 * \details A time handle is invalid if it is set to the value
 *         XME_HAL_TIME_INVALID_TIME_HANDLE.
 *
 * \param  timeHandle Time handle to check validity for.
 *
 * \return Returns true if the given time handle is valid and false otherwise.
 */
extern bool
xme_hal_time_isValidTimeHandle
(
    xme_hal_time_timeHandle_t timeHandle
);

/**
 * \brief  Retrieves a handle for the current point in time.
 *
 * \note   The returned time handle must be destroyed in a call to
 *         ::xme_hal_time_destroyTimeHandle() when it is not used any more.
 *
 * \return Returns a handle for the current point in time.
 */
extern xme_hal_time_timeHandle_t
xme_hal_time_getCurrentTime(void);

/**
 * \brief  Compares two points in time.
 *
 * \note   If one of the given time handles is invalid, the behavior is
 *         undefined.
 *
 * \param  time1 First point in time.
 * \param  time2 Second point in time.
 *
 * \return Returns a value smaller than zero if time1 corresponds to an
 *         earlier point in time than time2. Returns a value larger than
 *         zero if time1 corresponds to a later point in time than time2.
 *         Returns zero if time1 and time2 correspond to the same point
 *         in time (with respect to the granularity of the time system).
 */
extern int8_t
xme_hal_time_compareTime
(
    xme_hal_time_timeHandle_t time1,
    xme_hal_time_timeHandle_t time2
);

/**
 * \brief  Offsets the given point in time by the given interval.
 *
 * \note   Use the functions xme_hal_time_timeIntervalFromSeconds(),
 *         xme_hal_time_timeIntervalFromMilliseconds(),
 *         xme_hal_time_timeIntervalFromMicroseconds() and
 *         xme_hal_time_timeIntervalFromNanoseconds() to retrieve the value
 *         to pass to offset from a discrete time interval value.
 *
 * \note   If the given time handle is invalid, the behavior is undefined.
 *
 * \param  time Point in time to offset.
 * \param  operation Operation to perform. \see xme_hal_time_offsetOperation
 * \param  offset Interval to offset the given time.
 *
 * \return Returns a handle for the point in time with offset applied.
 */
extern xme_hal_time_timeHandle_t
xme_hal_time_offsetTime
(
    xme_hal_time_timeHandle_t time,
    xme_hal_time_offsetOperation_t operation,
    xme_hal_time_timeInterval_t offset
);

/**
 * \brief  Returns the time interval that has passed between the given start
 *         and stop time (represented by time handles).
 *
 * \note   Use the functions xme_hal_time_timeIntervalInSeconds(),
 *         xme_hal_time_timeIntervalInMilliseconds(),
 *         xme_hal_time_timeIntervalInMicroseconds() and
 *         xme_hal_time_timeIntervalInNanoseconds() on the returned interval
 *         representation to retrieve a discrete value for the time interval.
 *
 * \details In case start time corresponds to a later point in time than
 *         stopTime, the result may be undefined. If you are not sure about
 *         the order, invoke xme_hal_time_compareTime() first.
 *
 * \note   If one of the given time handles is invalid, the behavior is
 *         undefined.
 *
 * \param  startTime Start time.
 * \param  stopTime Stop time.
 * \return the number of miliseconds between start time and stop time.
 */
extern xme_hal_time_timeInterval_t
xme_hal_time_getTimeIntervalBetween
(
    xme_hal_time_timeHandle_t startTime,
    xme_hal_time_timeHandle_t stopTime
);

/**
 * \brief  Returns the time interval that has passed between the given start
 *         time (represented by a time handle) and the current point in time.
 *
 * \note   Use the functions xme_hal_time_timeIntervalInSeconds(),
 *         xme_hal_time_timeIntervalInMilliseconds(),
 *         xme_hal_time_timeIntervalInMicroseconds() and
 *         xme_hal_time_timeIntervalInNanoseconds() on the returned interval
 *         representation to retrieve a discrete value for the time interval.
 *
 * \param  startTime Address of a time handle that corresponds to the point
 *         in time that marks the start of the time interval to measure.
 *         If this parameter is NULL or the value it is pointing to corresponds
 *         to an invalid time handle, the behavior is undefined.
 * \param  reset If set to true, the function will replace the value in
 *         startTime with the current time such that subsequent calls to
 *         xme_hal_time_getTimeInterval() will yield the time interval between
 *         the most recent execution of xme_hal_time_getTimeInterval() and the
 *         current time. If set to true, startTime will not be modified.
 *
 * \return Returns the time interval that have passed between the point in time
 *         specified by startTime and now.
 */
extern xme_hal_time_timeInterval_t
xme_hal_time_getTimeInterval
(
    xme_hal_time_timeHandle_t* startTime,
    bool reset
);

/**
 * \brief  Frees all resources associated with the given time handle.
 *
 * \param  timeHandle Time handle to destroy.
 */
extern void
xme_hal_time_destroyTimeHandle
(
    xme_hal_time_timeHandle_t timeHandle
);

/**
 * \brief  Creates a time handle representing the given absolute point
 *         in time.
 *
 * \note   The input valuehas the granularity of seconds. Hence, the
 *         resulting time handle will not contain any fractions of a second.
 *
 * \param  absoluteTime Calendar date and time of the absolute point in
 *         time to use.
 *
 * \return On success, returns a valid time handle corresponding to the
 *         given absolute point in time. On error, returns
 *         XME_HAL_TIME_INVALID_TIME_HANDLE.
 */
extern xme_hal_time_timeHandle_t
xme_hal_time_handleFromAbsoluteTime
(
    struct tm* absoluteTime
);

/**
 * \brief  Retrieves the absolute point in time corresponding to the
 *         given handle.
 *
 * \note   The resulting time value has the granularity of seconds.
 *         Fractions of a second are not represented.
 *
 * \param  timeHandle Time handle with the time information.
 * \param  absoluteTime Calendar date and time of the absolute point in
 *         time derived from the time handle.
 *
 * \retval XME_STATUS_SUCCESS if the time value was successfully stored
 *         in absoluteTime.
 * \retval XME_STATUS_INVALID_HANDLE if the given time handle was invalid.
 */
extern xme_status_t
xme_hal_time_absoluteTimeFromHandle
(
    xme_hal_time_timeHandle_t timeHandle,
    struct tm* absoluteTime
);

/**
 * \brief  Creates a string representation of an absolute point in time
 *         according to specified format arguments.
 *
 * \details The syntax of the format arguments is consistent with the strftime()
 *          function from the C standard library. Compliant platforms have to
 *          implement at least the following format specifiers: %d, %H, %m, %M,
 *          %S, %x, %X, %y, %Y, %%.
 *
 * \details On systems that do not offer absolute time functionality (e.g.,
 *          systems without a realtime clock), the timestamp might be relative
 *          to the reset time of the system or it may be all zeroes. Please
 *          consult the platform documentation for more information.
 *
 * \param  buffer Pointer to the destination array where the resulting
 *         C string is copied.
 * \param  sizeInBytes Maximum number of characters to be copied to ptr, including
 *         the terminating null-character.
 * \param  format C string containing any combination of regular
 *         characters and special format specifiers. These format
 *         specifiers are replaced by the function to the corresponding
 *         values to represent the time specified in timeHandle.
 *         They all begin with a percentage (%) sign. Please consult
 *         the documentation of the C standard library function
 *         strftime() for detailed information. Notice that as stated
 *         above, certain constraints might apply to the available
 *         format arguments.
 * \param  timeHandle the time handle. 
 *
 * \return If the resulting C string fits in less than maxsize
 *         characters including the terminating null-character, the
 *         total number of characters copied to ptr (not including the
 *         terminating null-character) is returned. Otherwise, zero is
 *         returned and the contents of the array are indeterminate.
 */
extern size_t
xme_hal_time_formatTime
(
    char* buffer,
    size_t sizeInBytes,
    const char* format,
    xme_hal_time_timeHandle_t timeHandle
);

/**
 * \brief  Converts a time interval in nanoseconds to a value in seconds,
 *         discarding the fractional part.
 *
 * \param  timeInterval Time interval to convert to seconds.
 *
 * \return Returns the seconds portion of the given time interval,
 *         discarding the fractional part.
 */
static uint32_t
xme_hal_time_timeIntervalInSeconds
(
    xme_hal_time_timeInterval_t timeInterval
);

/**
 * \brief  Converts a time interval in nanoseconds to a value in milliseconds,
 *         discarding the fractional part.
 *
 * \param  timeInterval Time interval to convert to seconds.
 *
 * \return Returns the milliseconds portion of the given time interval,
 *         discarding the fractional part.
 */
static uint64_t
xme_hal_time_timeIntervalInMilliseconds
(
    xme_hal_time_timeInterval_t timeInterval
);

/**
 * \brief  Converts a time interval in nanoseconds to a value in microseconds,
 *         discarding the fractional part.
 *
 * \param  timeInterval Time interval to convert to seconds.
 *
 * \return Returns the microseconds portion of the given time interval,
 *         discarding the fractional part.
 */
static uint64_t
xme_hal_time_timeIntervalInMicroseconds
(
    xme_hal_time_timeInterval_t timeInterval
);

/**
 * \brief  Converts a time interval specified in seconds into the
 *         corresponding nanosecond value.
 *
 * \param  timeInterval_s Time interval in seconds to convert.
 *
 * \return Returns the time interval in nanoseconds corresponding to the
 *         given time interval.
 */
static xme_hal_time_timeInterval_t
xme_hal_time_timeIntervalFromSeconds
(
    uint32_t timeInterval_s
);

/**
 * \brief  Converts a time interval specified in milliseconds into the
 *         corresponding nanosecond value.
 *
 * \param  timeInterval_ms Time interval in milliseconds to convert.
 *
 * \return Returns the time interval in nanoseconds corresponding to the
 *         given time interval.
 */
static xme_hal_time_timeInterval_t
xme_hal_time_timeIntervalFromMilliseconds
(
    uint64_t timeInterval_ms
);

/**
 * \brief  Converts a time interval specified in microseconds into the
 *         corresponding nanosecond value.
 *
 * \param  timeInterval_us Time interval in microseconds to convert.
 *
 * \return Returns the time interval in nanoseconds corresponding to the
 *         given time interval.
 */
static xme_hal_time_timeInterval_t
xme_hal_time_timeIntervalFromMicroseconds
(
    uint64_t timeInterval_us
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/time_util.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_TIME_H
