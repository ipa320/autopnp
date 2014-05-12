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
 * $Id: integrationTestRandom.cpp 2895 2013-04-12 18:04:27Z geisinger $
 */

/**
 * \file
 *         Random number generator integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/random.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class RandomIntegrationTest: public ::testing::Test
{
protected:
    RandomIntegrationTest()
    : numRounds(256U)
    {
        xme_hal_random_init();
        xme_hal_random_registerThread();
    }

    virtual ~RandomIntegrationTest()
    {
        xme_hal_random_deregisterThread();
        xme_hal_random_fini();
    }

    const uint32_t numRounds;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     RandomIntegrationTest                                                  //
//----------------------------------------------------------------------------//

TEST_F(RandomIntegrationTest, randRangeRandom)
{
    for (uint64_t i = 0; i < numRounds; i++)
    {
        uint16_t min = xme_hal_random_rand();
        uint16_t max = xme_hal_random_rand();

        if (min <= max)
        {
            uint16_t rand = xme_hal_random_randRange(min, max);
            EXPECT_LE(rand, max);
            EXPECT_GE(rand, min);
        }
        else
        {
            // For max < min, the return value is undefined
        }
    }
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
