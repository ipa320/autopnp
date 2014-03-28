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
 * $Id: smokeTestNode.cpp 6310 2014-01-14 13:59:32Z geisinger $
 */

/**
 * \file
 *         Node Iterator smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/directory/include/node.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class NodeSmokeTest: public ::testing::Test
{
protected:
    NodeSmokeTest()
    {
    }

    virtual ~NodeSmokeTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NodeSmokeTest                                                          //
//----------------------------------------------------------------------------//

TEST_F(NodeSmokeTest, initIteratorOnNonInitedNodeTableWithValidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
}

TEST_F(NodeSmokeTest, initIteratorOnNonInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INVALID_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
}

TEST_F(NodeSmokeTest, initIteratorOnNonInitedNodeTableWithNonExistingInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, (xme_core_directory_node_interfaceTypes_t) 2, &iterator));
    EXPECT_EQ(NULL, iterator);
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeIdOnNonInitedNodeTableWithValidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeIdOnNonInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INVALID_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeIdOnNonInitedNodeTableWithNonExistingInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, (xme_core_directory_node_interfaceTypes_t) 2, &iterator));
    EXPECT_EQ(NULL, iterator);
}

TEST_F(NodeSmokeTest, initIteratorOnInitedNodeTableWithValidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
    xme_core_directory_node_fini();
}

TEST_F(NodeSmokeTest, initIteratorOnInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INVALID_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
    xme_core_directory_node_fini();
}

TEST_F(NodeSmokeTest, initIteratorOnInitedNodeTableWithNonExistingInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, (xme_core_directory_node_interfaceTypes_t) 2, &iterator));
    EXPECT_EQ(NULL, iterator);
    xme_core_directory_node_fini();
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeOnInitedNodeTableWithValidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
    xme_core_directory_node_fini();
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeOnInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INVALID_INTERFACE, &iterator));
    EXPECT_EQ(NULL, iterator);
    xme_core_directory_node_fini();
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeOnInitedNodeTableWithNonExistingInterface)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, (xme_core_directory_node_interfaceTypes_t) 2, &iterator));
    EXPECT_EQ(NULL, iterator);
    xme_core_directory_node_fini();
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
