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
 * $Id: smokeTestRandom.cpp 2895 2013-04-12 18:04:27Z geisinger $
 */

/**
 * \file
 *         Random number generator smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/random.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class RandomSmokeTest: public ::testing::Test
{
protected:
    RandomSmokeTest()
    : minRand(0U)
    , maxRand(XME_HAL_RANDOM_RAND_MAX)
    {
    }

    virtual ~RandomSmokeTest()
    {
    }

    uint16_t minRand;
    uint16_t maxRand;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     RandomSmokeTest                                                        //
//----------------------------------------------------------------------------//

TEST_F(RandomSmokeTest, initializeUninitialized)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_random_init());
    xme_hal_random_fini();
}

TEST_F(RandomSmokeTest, initializeUninitializedAndRegisterThread)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_random_init());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_random_registerThread());
    xme_hal_random_deregisterThread();
    xme_hal_random_fini();
}

TEST_F(RandomSmokeTest, randMax)
{
    // According to documentation, XME_HAL_RANDOM_RAND_MAX must be at least this large
    EXPECT_GE(XME_HAL_RANDOM_RAND_MAX, 32767);
}

TEST_F(RandomSmokeTest, randRangeOverOneElement)
{
    EXPECT_EQ(minRand, xme_hal_random_randRange(minRand, minRand));
    EXPECT_EQ(minRand+1, xme_hal_random_randRange(minRand+1, minRand+1));
    EXPECT_EQ(maxRand-1, xme_hal_random_randRange(maxRand-1, maxRand-1));
    EXPECT_EQ(maxRand, xme_hal_random_randRange(maxRand, maxRand));
}

TEST_F(RandomSmokeTest, randRangeOverTwoElements)
{
    EXPECT_LE(minRand, xme_hal_random_randRange(minRand, minRand+1));
    EXPECT_LE(minRand, xme_hal_random_randRange(minRand, minRand+1));

    EXPECT_LE(minRand+1, xme_hal_random_randRange(minRand+1, minRand+2));
    EXPECT_LE(minRand+1, xme_hal_random_randRange(minRand+1, minRand+2));
    EXPECT_GE(minRand+2, xme_hal_random_randRange(minRand+1, minRand+2));
    EXPECT_GE(minRand+2, xme_hal_random_randRange(minRand+1, minRand+2));

    EXPECT_LE(maxRand-2, xme_hal_random_randRange(maxRand-2, maxRand-1));
    EXPECT_LE(maxRand-2, xme_hal_random_randRange(maxRand-2, maxRand-1));
    EXPECT_GE(maxRand-1, xme_hal_random_randRange(maxRand-2, maxRand-1));
    EXPECT_GE(maxRand-1, xme_hal_random_randRange(maxRand-2, maxRand-1));

    EXPECT_LE(maxRand-1, xme_hal_random_randRange(maxRand-1, maxRand));
    EXPECT_LE(maxRand-1, xme_hal_random_randRange(maxRand-1, maxRand));
    EXPECT_GE(maxRand, xme_hal_random_randRange(maxRand-1, maxRand));
    EXPECT_GE(maxRand, xme_hal_random_randRange(maxRand-1, maxRand));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
