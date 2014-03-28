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
 * $Id: interfaceTestTime.cpp 4598 2013-08-07 14:28:43Z ruiz $
 */

/**
 * \file
 *         Time interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/sleep.h"
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class TimeInterfaceTest: public ::testing::Test
{
protected:
    TimeInterfaceTest()
    : timeHandle1(XME_HAL_TIME_INVALID_TIME_HANDLE)
    , timeHandle2(XME_HAL_TIME_INVALID_TIME_HANDLE)
    , timeInterval(0)
    {
        absoluteTime.tm_sec = 0;
        absoluteTime.tm_min = 0;
        absoluteTime.tm_hour = 0;
        absoluteTime.tm_mday = 0;
        absoluteTime.tm_mon = 0;
        absoluteTime.tm_year = 0;
        absoluteTime.tm_wday = 0;
        absoluteTime.tm_yday = 0;
        absoluteTime.tm_isdst = -1;

        someTime.tm_sec = 56;
        someTime.tm_min = 34;
        someTime.tm_hour = 12;
        someTime.tm_mday = 30;
        someTime.tm_mon = 3;
        someTime.tm_year = 2013-1900;
        someTime.tm_wday = 0;
        someTime.tm_yday = 0;
        someTime.tm_isdst = 1;
    }

    virtual ~TimeInterfaceTest()
    {
        if (xme_hal_time_isValidTimeHandle(timeHandle1))
        {
            xme_hal_time_destroyTimeHandle(timeHandle1);
        }

        if (xme_hal_time_isValidTimeHandle(timeHandle2))
        {
            xme_hal_time_destroyTimeHandle(timeHandle2);
        }
    }

    xme_hal_time_timeHandle_t timeHandle1;
    xme_hal_time_timeHandle_t timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    struct tm absoluteTime;
    struct tm someTime;
    char buf[512];
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     TimeInterfaceTest                                                      //
//----------------------------------------------------------------------------//

TEST_F(TimeInterfaceTest, conversionBetweenTimeIntervalAndSeconds)
{
    EXPECT_EQ(0ULL,    xme_hal_time_timeIntervalInSeconds(            0ULL));
    EXPECT_EQ(0ULL,    xme_hal_time_timeIntervalInSeconds(    999999999ULL));
    EXPECT_EQ(1ULL,    xme_hal_time_timeIntervalInSeconds(   1000000000ULL));
    EXPECT_EQ(1ULL,    xme_hal_time_timeIntervalInSeconds(   1999999999ULL));
    EXPECT_EQ(2ULL,    xme_hal_time_timeIntervalInSeconds(   2000000000ULL));
    EXPECT_EQ(9992ULL, xme_hal_time_timeIntervalInSeconds(9992000000000ULL));
}

TEST_F(TimeInterfaceTest, conversionBetweenTimeIntervalAndMilliseconds)
{
    EXPECT_EQ(0ULL,    xme_hal_time_timeIntervalInMilliseconds(         0ULL));
    EXPECT_EQ(0ULL,    xme_hal_time_timeIntervalInMilliseconds(    999999ULL));
    EXPECT_EQ(1ULL,    xme_hal_time_timeIntervalInMilliseconds(   1000000ULL));
    EXPECT_EQ(1ULL,    xme_hal_time_timeIntervalInMilliseconds(   1999999ULL));
    EXPECT_EQ(2ULL,    xme_hal_time_timeIntervalInMilliseconds(   2000000ULL));
    EXPECT_EQ(9992ULL, xme_hal_time_timeIntervalInMilliseconds(9992000000ULL));
}

TEST_F(TimeInterfaceTest, conversionBetweenTimeIntervalAndMicroseconds)
{
    EXPECT_EQ(0ULL,    xme_hal_time_timeIntervalInMicroseconds(      0ULL));
    EXPECT_EQ(0ULL,    xme_hal_time_timeIntervalInMicroseconds(    999ULL));
    EXPECT_EQ(1ULL,    xme_hal_time_timeIntervalInMicroseconds(   1000ULL));
    EXPECT_EQ(1ULL,    xme_hal_time_timeIntervalInMicroseconds(   1999ULL));
    EXPECT_EQ(2ULL,    xme_hal_time_timeIntervalInMicroseconds(   2000ULL));
    EXPECT_EQ(9992ULL, xme_hal_time_timeIntervalInMicroseconds(9992000ULL));
}

TEST_F(TimeInterfaceTest, getCurrentTime)
{
    timeHandle1 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
}

TEST_F(TimeInterfaceTest, getCurrentTimeGranularity)
{
    timeHandle1 = xme_hal_time_getCurrentTime();
    xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
    timeHandle2 = xme_hal_time_getCurrentTime();

    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));

    EXPECT_GT(0, xme_hal_time_compareTime(timeHandle1, timeHandle2));
    EXPECT_EQ(0, xme_hal_time_compareTime(timeHandle1, timeHandle1));
    EXPECT_EQ(0, xme_hal_time_compareTime(timeHandle2, timeHandle2));
    EXPECT_LT(0, xme_hal_time_compareTime(timeHandle2, timeHandle1));
}

TEST_F(TimeInterfaceTest, offsetTimeAddZeroMilliseconds)
{
    timeHandle1 = xme_hal_time_getCurrentTime();
    timeHandle2 = xme_hal_time_offsetTime(timeHandle1, XME_HAL_TIME_OFFSET_OPERATION_ADD, xme_hal_time_timeIntervalFromMilliseconds(0));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    EXPECT_EQ(0, xme_hal_time_compareTime(timeHandle1, timeHandle2));
}

TEST_F(TimeInterfaceTest, offsetTimeAddOneMillisecond)
{
    timeHandle1 = xme_hal_time_getCurrentTime();
    timeHandle2 = xme_hal_time_offsetTime(timeHandle1, XME_HAL_TIME_OFFSET_OPERATION_ADD, xme_hal_time_timeIntervalFromMilliseconds(1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    EXPECT_GT(0, xme_hal_time_compareTime(timeHandle1, timeHandle2));
}

TEST_F(TimeInterfaceTest, offsetTimeSubtractZeroMilliseconds)
{
    timeHandle1 = xme_hal_time_getCurrentTime();
    timeHandle2 = xme_hal_time_offsetTime(timeHandle1, XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT, xme_hal_time_timeIntervalFromMilliseconds(0));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    EXPECT_EQ(0, xme_hal_time_compareTime(timeHandle1, timeHandle2));
}

TEST_F(TimeInterfaceTest, offsetTimeSubtractOneMillisecond)
{
    timeHandle1 = xme_hal_time_getCurrentTime();
    timeHandle2 = xme_hal_time_offsetTime(timeHandle1, XME_HAL_TIME_OFFSET_OPERATION_SUBTRACT, xme_hal_time_timeIntervalFromMilliseconds(1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    EXPECT_LT(0, xme_hal_time_compareTime(timeHandle1, timeHandle2));
}

TEST_F(TimeInterfaceTest, intervals)
{
    size_t i;

    // On some platforms, sleeping will wait a shorter time than specified
    // because of low system tick resolution. The minOffset and maxOffset
    // variables try to define some reasonable values to compensate for this.
    const unsigned int intervals_ms[] = {10, 25, 100, 1000};
    const int minOffset_ms = -10;
    const int maxOffset_ms = 20;

    for (i = 0; i < sizeof(intervals_ms) / sizeof(unsigned int); ++i)
    {
        timeHandle1 = xme_hal_time_getCurrentTime();
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i]));
        timeHandle2 = xme_hal_time_getCurrentTime();

        EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
        EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));

        EXPECT_LE(xme_hal_time_compareTime(timeHandle1, timeHandle2), 0);

        timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
        EXPECT_GE(timeInterval, xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i] + minOffset_ms));
        EXPECT_LE(timeInterval, xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i] + maxOffset_ms));
        printf
        (
            "xme_hal_time_getIntervalBetween(): %ums <= %lums <= %ums\n",
            intervals_ms[i] + minOffset_ms,
            (long)xme_hal_time_timeIntervalInMilliseconds(timeInterval),
            intervals_ms[i] + maxOffset_ms
        );

        xme_hal_time_destroyTimeHandle(timeHandle1);
        xme_hal_time_destroyTimeHandle(timeHandle2);

        timeHandle1 = xme_hal_time_getCurrentTime();
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i]));
        timeInterval = xme_hal_time_getTimeInterval(&timeHandle1, true);
        EXPECT_GE(timeInterval, xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i] + minOffset_ms));
        EXPECT_LE(timeInterval, xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i] + maxOffset_ms));
        printf
        (
            "xme_hal_time_getInterval() w/ reset: %ums <= %lums <= %ums\n",
            intervals_ms[i] + minOffset_ms,
            (long)xme_hal_time_timeIntervalInMilliseconds(timeInterval),
            intervals_ms[i] + maxOffset_ms
        );

        xme_hal_time_destroyTimeHandle(timeHandle1);

        timeHandle1 = xme_hal_time_getCurrentTime();
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i]));
        xme_hal_time_getTimeInterval(&timeHandle1, false);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(intervals_ms[i]));
        timeInterval = xme_hal_time_getTimeInterval(&timeHandle1, false);
        EXPECT_GE(timeInterval, xme_hal_time_timeIntervalFromMilliseconds(2 * (intervals_ms[i] + minOffset_ms)));
        EXPECT_LE(timeInterval, xme_hal_time_timeIntervalFromMilliseconds(2 * (intervals_ms[i] + maxOffset_ms)));
        printf
        (
            "xme_hal_time_getInterval() w/o reset: %ums <= %lums <= %ums\n",
            2*(intervals_ms[i] + minOffset_ms),
            (long)xme_hal_time_timeIntervalInMilliseconds(timeInterval),
            2*(intervals_ms[i] + maxOffset_ms)
        );

        xme_hal_time_destroyTimeHandle(timeHandle1);
    }
}

TEST_F(TimeInterfaceTest, convertSomeTime)
{
    timeHandle1 = xme_hal_time_handleFromAbsoluteTime(&someTime);
    xme_hal_time_absoluteTimeFromHandle(timeHandle1, &absoluteTime);
    EXPECT_EQ(someTime.tm_sec, absoluteTime.tm_sec);
    EXPECT_EQ(someTime.tm_min, absoluteTime.tm_min);
    EXPECT_EQ(someTime.tm_hour, absoluteTime.tm_hour);
    EXPECT_EQ(someTime.tm_mday, absoluteTime.tm_mday);
    EXPECT_EQ(someTime.tm_mon, absoluteTime.tm_mon);
    EXPECT_EQ(someTime.tm_year, absoluteTime.tm_year);
    EXPECT_EQ(someTime.tm_isdst, absoluteTime.tm_isdst);
}

TEST_F(TimeInterfaceTest, convertAbsoluteTime)
{
    timeHandle1 = xme_hal_time_getCurrentTime();
    xme_hal_time_absoluteTimeFromHandle(timeHandle1, &absoluteTime);

    EXPECT_LE(0, absoluteTime.tm_sec);
    EXPECT_GE(61, absoluteTime.tm_sec); // cosideration of leap seconds

    EXPECT_LE(0, absoluteTime.tm_min);
    EXPECT_GE(59, absoluteTime.tm_min);

    EXPECT_LE(0, absoluteTime.tm_hour);
    EXPECT_GE(23, absoluteTime.tm_hour);

    EXPECT_LE(1, absoluteTime.tm_mday);
    EXPECT_GE(31, absoluteTime.tm_mday);

    EXPECT_LE(0, absoluteTime.tm_mon);
    EXPECT_GE(11, absoluteTime.tm_mon);

    EXPECT_LE(0, absoluteTime.tm_wday);
    EXPECT_GE(6, absoluteTime.tm_wday);

    EXPECT_LE(0, absoluteTime.tm_yday);
    EXPECT_GE(365, absoluteTime.tm_yday); // consideration of leap years
}

TEST_F(TimeInterfaceTest, formatSomeTimeC89)
{
    timeHandle1 = xme_hal_time_handleFromAbsoluteTime(&someTime);

    // %c and %Z omitted, because they yield a locale-dependent value
    xme_hal_time_formatTime(buf, sizeof(buf), "%a|%A|%b|%B|%d|%H|%I|%j|%m|%M|%p|%S|%U|%w|%W|%x|%X|%y|%Y|%%", timeHandle1);
    EXPECT_STREQ("Tue|Tuesday|Apr|April|30|12|12|120|04|34|PM|56|17|2|17|04/30/13|12:34:56|13|2013|%", buf);

    xme_hal_time_formatTime(buf, sizeof(buf), "%c", timeHandle1);
    EXPECT_STRNE("", buf);

    xme_hal_time_formatTime(buf, sizeof(buf), "%Z", timeHandle1);
    EXPECT_STRNE("", buf);
}

#if __STDC_VERSION__ >= 199901L
TEST_F(TimeInterfaceTest, formatSomeTimeC99)
{
    // Test format arguments that are only valid in C99 mode

    timeHandle1 = xme_hal_time_handleFromAbsoluteTime(&someTime);

    // %z omitted, because it yields a locale-dependent value
    xme_hal_time_formatTime(buf, sizeof(buf), "%C|%D|%e|%F|%g|%G|%h|%n|%r|%R|%t|%T|%u|%V", timeHandle1);
    EXPECT_STREQ("20|04/30/13|30|2013-04-30|13|2013|Apr|\n|12:34:56 PM|12:34|    |12:34:56|2|18", buf);

    xme_hal_time_formatTime(buf, sizeof(buf), "%Ec|%EC|%Ex|%EX|%Ey|%EY", timeHandle1);
    EXPECT_STREQ("Tue Apr 30 12:34:56 2013|20|04/30/13|12:34:56|13|2013", buf);

    xme_hal_time_formatTime(buf, sizeof(buf), "%Od|%Oe|%OH|%OI|%Om|%OM|%OS|%Ou|%OU|%OV|%Ow|%OW|%Oy", timeHandle1);
    EXPECT_STREQ("30|30|12|12|04|34|56|2|17|18|2|17|13", buf);
}
#endif

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
