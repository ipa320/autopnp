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
 * $Id: smokeTestDemarshaler.cpp 4779 2013-08-26 13:28:25Z wiesmueller $
 */

/**
 * \file
 *         Demarshaler waypoint test.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "demarshaler.h"
#include "deMarshalerTestTopic.h"
#include "deMarshalerTestTopicData.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/net.h"

#include "xme/wp/waypoint.h"

class DemarshalerSmokeTest : public testing::Test
{
protected:
    DemarshalerSmokeTest()
    {
    }

    ~DemarshalerSmokeTest()
    {
        // Do nothing
    }
};

TEST_F(DemarshalerSmokeTest, DemarshalerInitAndFini)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_init());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_fini());
}

TEST_F(DemarshalerSmokeTest, DemarshalerFiniBeforeInit)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_fini());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_init());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_fini());
}

TEST_F(DemarshalerSmokeTest, DemarshalerRunInvalidInstanceId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_init());
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_wp_marshal_demarshaler_run(XME_WP_WAYPOINT_INSTANCEID_INVALID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_fini());
}

TEST_F(DemarshalerSmokeTest, DemarshalerRunMaxInstanceId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_init());
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_wp_marshal_demarshaler_run(XME_WP_WAYPOINT_INSTANCEID_MAX));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_fini());
}

int main(int argc, char **argv)
{
    int retval;
    
    ::testing::InitGoogleTest(&argc, argv);
    retval = RUN_ALL_TESTS();
    
    return retval;
}
