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
 * $Id: smokeTestNetworkTopologyCalculator.cpp 4406 2013-07-30 11:39:37Z geisinger $
 */

/**
 * \file
 *         Network Topology Calculator interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/directory/include/networkTopologyCalculator.h"



/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class NetworkTopologyCalculatorSmokeTest: public ::testing::Test
{
protected:
    // constructor
    NetworkTopologyCalculatorSmokeTest()
    {
        // do nothing
    }

    virtual ~NetworkTopologyCalculatorSmokeTest()
    {
    }

    // SetUp before the first test case
    virtual void SetUpTestCase()
    {
        //do nothing
    }

    virtual void TearDownTestCase()
    {
        xme_core_directory_networkTopologyCalculator_fini();
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST(NetworkTopologyCalculatorSmokeTest, initFunctionWithNULLParameter)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_init(NULL));
    xme_core_directory_networkTopologyCalculator_fini();
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
