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
 * $Id: smokeTestBroker.cpp 4597 2013-08-07 14:18:28Z ruiz $
 */

/**
 * \file
 *         Broker test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/broker/include/broker.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class BrokerSmokeTest: public ::testing::Test
{
protected:
    // constructor
    BrokerSmokeTest()
    {
        // do nothing
    }

    virtual ~BrokerSmokeTest()
    {
    }

    // SetUp before the first test case
    virtual void SetUpTestCase()
    {
        //xme_core_broker_init(NULL);
    }

    virtual void TearDownTestCase()
    {
        xme_core_broker_fini();
    }
};



TEST(BrokerSmokeTest, InitFunctionWithNULLParameter)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
    xme_core_broker_fini();
}

TEST(BrokerSmokeTest, InitFunctionWithWrongInitStruct)
{
    int foo = 6;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(&foo));
    xme_core_broker_fini();
}

TEST(BrokerSmokeTest, InitFunctionWithCorrectInitStruct)
{
    xme_core_broker_initStruct_t foo = {(xme_core_component_t) 1};
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(&foo));
    xme_core_broker_fini();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
