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
 * $Id: smokeTestTime.cpp 3090 2013-04-26 14:33:46Z geisinger $
 */

/**
 * \file
 *         Time smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class TimeSmokeTest: public ::testing::Test
{
protected:
    TimeSmokeTest()
    : timeHandle(XME_HAL_TIME_INVALID_TIME_HANDLE)
    {
        someTime.tm_sec = 56;
        someTime.tm_min = 34;
        someTime.tm_hour = 12;
        someTime.tm_mday = 30;
        someTime.tm_mon = 3;
        someTime.tm_year = 2013-1900;
        someTime.tm_wday = 0;
        someTime.tm_yday = 0;
        someTime.tm_isdst = -1;
    }

    virtual ~TimeSmokeTest()
    {
        if (xme_hal_time_isValidTimeHandle(timeHandle))
        {
            xme_hal_time_destroyTimeHandle(timeHandle);
        }
    }

    struct tm someTime;
    xme_hal_time_timeHandle_t timeHandle;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     TimeSmokeTest                                                          //
//----------------------------------------------------------------------------//

TEST_F(TimeSmokeTest, invalidTimeHandles)
{
    EXPECT_FALSE(xme_hal_time_isValidTimeHandle(timeHandle));
}

TEST_F(TimeSmokeTest, simpleConversion)
{
    EXPECT_EQ(xme_hal_time_timeIntervalInSeconds(0ULL),      0UL);
    EXPECT_EQ(xme_hal_time_timeIntervalInMilliseconds(0ULL), 0ULL);
    EXPECT_EQ(xme_hal_time_timeIntervalInMicroseconds(0ULL), 0ULL);
}

TEST_F(TimeSmokeTest, simpleAbsoluteTime)
{
    timeHandle = xme_hal_time_handleFromAbsoluteTime(&someTime);
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
