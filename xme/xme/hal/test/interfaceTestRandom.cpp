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
 * $Id: interfaceTestRandom.cpp 2895 2013-04-12 18:04:27Z geisinger $
 */

/**
 * \file
 *         Random number generator interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/random.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class RandomInterfaceTest: public ::testing::Test
{
protected:
    RandomInterfaceTest()
    : numRounds(256U)
    , numSamples((double)numRounds*65535U)
    {
        xme_hal_random_init();
        xme_hal_random_registerThread();

        // Zero memory
        for (uint64_t i=0; i<sizeof(distribution)/sizeof(distribution[0]); i++)
        {
            distribution[i] = 0;
        }
    }

    virtual ~RandomInterfaceTest()
    {
        xme_hal_random_deregisterThread();
        xme_hal_random_fini();
    }

    const uint32_t numRounds;

    double numSamples;
    uint16_t distribution[((uint16_t)-1)+1];
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     RandomInterfaceTest                                                    //
//----------------------------------------------------------------------------//

TEST_F(RandomInterfaceTest, initializeInitialized)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_random_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_random_registerThread());
}

TEST_F(RandomInterfaceTest, rand)
{
    // Test range and distribution of xme_hal_random_rand()

    const double safetyFactor = 2.0;
    double sum = 0;

    uint16_t minExpected = ((uint16_t)(numSamples / ((double)XME_HAL_RANDOM_RAND_MAX+1) / safetyFactor));
    uint16_t maxExpected = ((uint16_t)(numSamples / ((double)XME_HAL_RANDOM_RAND_MAX+1) * safetyFactor));

    for (uint64_t i = 0; i <= (double)numRounds*65536U; i++)
    {
        uint16_t r = xme_hal_random_rand();

        // Prevent overflow
        if (distribution[r] < 0xFFFF)
        {
            distribution[r]++;
        }

        sum += r;
    }

    printf("xme_hal_random_rand() test:\n");
    printf("- Minimum expected value per bin: %d (should be >> 0 for the test to be effective)\n", minExpected);
    printf("- Maximum expected value per bin: %d (must be <= 65535 for the test to be effective)\n", maxExpected);

    for (uint64_t i = 0; i <= XME_HAL_RANDOM_RAND_MAX; i++)
    {
        // In a truly uniform distribution, each item would be incremented
        // numSamples/(XME_HAL_RANDOM_RAND_MAX+1) times. These are safety bounds
        // that should always be true, given a somehow uniform distribution.
        EXPECT_GE(distribution[i], minExpected);
        EXPECT_LE(distribution[i], maxExpected);
    }

    // Check whether mean over all random number
    // is almost the mean of the interval
    {
        double mean = sum / (XME_HAL_RANDOM_RAND_MAX+1) / numSamples;
        EXPECT_LT(0.499, mean);
        EXPECT_LT(mean, 0.501);
    }
}

TEST_F(RandomInterfaceTest, randRange)
{
    // Test range and distribution of xme_hal_random_randRange()

    const double safetyFactor = 2.5;
    double biasedSum = 0;

    uint16_t min;
    uint16_t max;

    for (;;)
    {
        min = xme_hal_random_rand();
        max = xme_hal_random_rand();

        // Edge case for testing integrity of the check below:
        //min = 0;
        //max = (uint16_t)(numRounds*safetyFactor-1);

        // Swap min and max if max < min
        if (max < min)
        {
            uint16_t temp = min;
            min = max;
            max = temp;
        }

        // For these tests to work out without overflows in the safe bounds,
        // we have to assert that the interval is larger than numRounds*safetyFactor.
        if (max-min+1 < numRounds*safetyFactor)
        {
            printf("Chosen interval [%d, %d] for xme_hal_rand_randomRange() is too small (%d value(s)), starting over.\n", min, max, max-min+1);
            continue;
        }

        break;
    }

    ASSERT_LE((uint32_t)(numSamples / ((double)max-min+1) * safetyFactor), 65535U);

    uint16_t minExpected = (uint16_t)(numSamples / ((double)max-min+1) / safetyFactor);
    uint16_t maxExpected = (uint16_t)(numSamples / ((double)max-min+1) * safetyFactor);

    for (uint64_t i = 0; i <= numSamples; i++)
    {
        uint16_t r = xme_hal_random_randRange(min, max);

        // Prevent overflow
        if (distribution[r] < 0xFFFF)
        {
            distribution[r]++;
        }

        biasedSum += r - min;
    }

    printf("xme_hal_random_randRange() test:\n");
    printf("- Minimum expected value per bin: %d (should be >> 0 for the test to be effective)\n", minExpected);
    printf("- Maximum expected value per bin: %d (must be <= 65535 for the test to be effective)\n", maxExpected);

    for (uint64_t i = 0U; i <= 0xFFFF; i++)
    {
        if (i < min || i > max)
        {
            EXPECT_EQ(0, distribution[i]);
        }
        else
        {
            // In a truly uniform distribution, each item would be incremented
            // numSamples/(max-min+1) times. These are safety bounds that should
            // always be true, given a somehow uniform distribution.
            EXPECT_GE(distribution[i], numSamples/(max-min+1) / safetyFactor);
            EXPECT_LE(distribution[i], numSamples/(max-min+1) * safetyFactor);
        }
    }

    // Check whether mean over all random number
    // is almost the mean of the interval
    {
        double mean = biasedSum / (max-min+1) / numSamples;
        EXPECT_LT(0.499, mean);
        EXPECT_LT(mean, 0.501);
    }
}

TEST_F(RandomInterfaceTest, randRangeOverOneElement)
{
    // Test xme_hal_random_rand_range() with min == max

    for (uint64_t i = 0U; i <= XME_HAL_RANDOM_RAND_MAX; i++)
    {
        EXPECT_EQ(i, xme_hal_random_randRange((uint16_t)i, (uint16_t)i));
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
