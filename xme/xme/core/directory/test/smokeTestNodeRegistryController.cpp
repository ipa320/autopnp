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
 * $Id: smokeTestNodeRegistryController.cpp 6304 2014-01-14 09:08:52Z gulati $
 */

/**
 * \file
 *         Node Iterator smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/directory/include/nodeRegistryController.h"

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
    typedef enum
    {
       XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1 = 100,
       XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_2
    }
    moreInterfaces;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NoodeInterfaceTest                                                     //
//----------------------------------------------------------------------------//

TEST_F(NodeSmokeTest, initIteratorOnNonInitedNodeTableWithValidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &iterator));
    EXPECT_TRUE(NULL==iterator);
}

TEST_F(NodeSmokeTest, initIteratorOnNonInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator));
    EXPECT_TRUE(NULL==iterator);
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeIdOnNonInitedNodeTableWithValidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &iterator));
    EXPECT_TRUE(NULL==iterator);
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeIdOnNonInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator));
    EXPECT_TRUE(NULL==iterator);
}

TEST_F(NodeSmokeTest, initIteratorOnInitedNodeTableWithValidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &iterator));
    EXPECT_TRUE(NULL==iterator);
    xme_core_directory_nodeRegistryController_fini();
}

TEST_F(NodeSmokeTest, initIteratorOnInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator));
    EXPECT_TRUE(NULL==iterator);
    xme_core_directory_nodeRegistryController_fini();
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeOnInitedNodeTableWithValidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &iterator));
    EXPECT_TRUE(NULL==iterator);
    xme_core_directory_nodeRegistryController_fini();
}

TEST_F(NodeSmokeTest, initIteratorWithInvalidNodeOnInitedNodeTableWithInvalidInterface)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator(XME_CORE_NODE_INVALID_NODE_ID, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator));
    EXPECT_TRUE(NULL==iterator);
    xme_core_directory_nodeRegistryController_fini();
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
