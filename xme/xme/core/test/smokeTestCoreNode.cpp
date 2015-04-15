/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: smokeTestCoreNode.cpp 6322 2014-01-14 17:37:46Z geisinger $
 */

/**
 * \file
 *         XME core node smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/node.h"

#include "xme/core/testUtils.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class CoreNodeSmokeTest: public ::testing::Test
{
protected:
    CoreNodeSmokeTest()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_init());
    }

    virtual ~CoreNodeSmokeTest()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(xme_core_node_fini());
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     CoreNodeSmokeTest                                                      //
//----------------------------------------------------------------------------//

TEST_F(CoreNodeSmokeTest, getCurrentNodeIDInitiallyInvalid)
{
    EXPECT_EQ(XME_CORE_NODE_INVALID_NODE_ID, xme_core_node_getCurrentNodeId());
}

TEST_F(CoreNodeSmokeTest, setAndGetCurrentNodeID)
{
    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 42);
    EXPECT_EQ((xme_core_node_nodeId_t) 42, xme_core_node_getCurrentNodeId());
}

TEST_F(CoreNodeSmokeTest, getNodeNameInitiallyEmpty)
{
    char buffer[32] = { 0 };
    
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_getNodeName(buffer, sizeof(buffer)));
    EXPECT_STREQ("", buffer);
}

TEST_F(CoreNodeSmokeTest, setAndGetNodeName)
{
    char buffer[32] = { 0 };
    
    xme_core_node_setNodeName("testNode");
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_getNodeName(buffer, sizeof(buffer)));
    EXPECT_STREQ("testNode", buffer);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
