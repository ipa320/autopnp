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
 * $Id: interfaceTestCoreNode.cpp 6335 2014-01-15 09:11:42Z geisinger $
 */

/**
 * \file
 *         XME core node interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/node.h"

#include "xme/core/testUtils.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class CoreNodeInterfaceTest: public ::testing::Test
{
protected:
    CoreNodeInterfaceTest()
    : nodeID1((xme_core_node_nodeId_t) 42)
    , nodeID2((xme_core_node_nodeId_t) 43)
    {
    }

    virtual ~CoreNodeInterfaceTest()
    {
    }

    xme_core_node_nodeId_t nodeID1;
    xme_core_node_nodeId_t nodeID2;
};

class CoreNodeInterfaceTestInitialized: public ::testing::Test
{
protected:
    CoreNodeInterfaceTestInitialized()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_init());
    }

    virtual ~CoreNodeInterfaceTestInitialized()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(xme_core_node_fini());
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     CoreNodeInterfaceTest                                                  //
//----------------------------------------------------------------------------//

TEST_F(CoreNodeInterfaceTest, getCurrentNodeIDUninitialized)
{
    // FIXME: This should throw an assertion failure
    EXPECT_EQ(XME_CORE_NODE_INVALID_NODE_ID, xme_core_node_getCurrentNodeId());
}

TEST_F(CoreNodeInterfaceTest, setCurrentNodeIDUninitialized)
{
    // FIXME: This should throw an assertion failure
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_setCurrentNodeId(nodeID1));
}

TEST_F(CoreNodeInterfaceTest, setNodeNameUninitialized)
{
    // FIXME: This should throw an assertion failure
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_setNodeName("testNode"));
}

TEST_F(CoreNodeInterfaceTest, getNodeNameUninitialized)
{
    char buffer[32] = { 0 };

    // FIXME: This should throw an assertion failure
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_getNodeName(buffer, sizeof(buffer)));
}

//----------------------------------------------------------------------------//
//     CoreNodeInterfaceTestInitialized                                       //
//----------------------------------------------------------------------------//

TEST_F(CoreNodeInterfaceTest, setCurrentNodeIDWithInvalidNodeID)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_setCurrentNodeId(XME_CORE_NODE_INVALID_NODE_ID));
}

TEST_F(CoreNodeInterfaceTest, retainLastValidNodeID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_setCurrentNodeId(nodeID1));
    
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_setCurrentNodeId(XME_CORE_NODE_INVALID_NODE_ID));
    EXPECT_EQ(nodeID1, xme_core_node_getCurrentNodeId());
    
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_setCurrentNodeId(nodeID2));
    EXPECT_EQ(nodeID2, xme_core_node_getCurrentNodeId());
}

TEST_F(CoreNodeInterfaceTest, setNodeNameWithNullArgument)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_setNodeName(NULL));
}

TEST_F(CoreNodeInterfaceTest, setNodeNameWithEmptyArgument)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_setNodeName(""));
}

TEST_F(CoreNodeInterfaceTest, setNodeNameWithTooLongString)
{
    const size_t nodeNameSize = sizeof(((xme_core_node_nodeData_t*)0)->nodeName);
    char buffer[nodeNameSize+16];

    // Adapt this when needed
    ASSERT_EQ(256U, nodeNameSize);

    // Construct a string that is too long and shorten it until it works
    xme_hal_mem_set(buffer, 'a', sizeof(buffer));

    buffer[sizeof(buffer)-1] = 0;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_setNodeName(buffer));

    buffer[sizeof(buffer)-10] = 0;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_setNodeName(buffer));

    buffer[sizeof(buffer)-16] = 0;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_setNodeName(buffer));

    buffer[sizeof(buffer)-17] = 0;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_setNodeName(buffer));

    buffer[sizeof(buffer)-nodeNameSize] = 0;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_setNodeName(buffer));
}

TEST_F(CoreNodeInterfaceTest, getNodeNameWithTooSmallBuffer)
{
    char buffer[32] = { 0 };

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_setNodeName("testNode"));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_getNodeName(buffer, 0));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_getNodeName(buffer, 1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_getNodeName(buffer, 5));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_getNodeName(buffer, 8));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_getNodeName(buffer, 9));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_getNodeName(buffer, 10));
}

TEST_F(CoreNodeInterfaceTest, retainLastValidNodeName)
{
    char buffer1[32] = { 0 };
    char buffer2[32] = { 0 };

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_setNodeName("testNode"));
    
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_node_setNodeName(NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_getNodeName(buffer1, sizeof(buffer1)));
    EXPECT_STREQ("testNode", buffer1);
    
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_setNodeName("anotherName"));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_getNodeName(buffer2, sizeof(buffer2)));
    EXPECT_STREQ("anotherName", buffer2);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
