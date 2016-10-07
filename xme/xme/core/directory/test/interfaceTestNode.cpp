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
 * $Id: interfaceTestNode.cpp 6310 2014-01-14 13:59:32Z geisinger $
 */

/**
 * \file
 *         Node Iterator interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/directory/include/node.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class NodeInterfaceTest: public ::testing::Test
{
protected:
    NodeInterfaceTest()
    : node1((xme_core_node_nodeId_t)1)
    , node2((xme_core_node_nodeId_t)2)
    {

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_init());

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_registerNode(node1, "node1"));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_registerNode(node2, "node2"));

        // Add interfaces
        nodeIntf1_1.interfaceType = XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE;
        nodeIntf1_1.interfaceAddress.ipAddress.ip = "192.168.2.1";
        nodeIntf1_1.interfaceAddress.ipAddress.port = 32211;
        nodeIntf1_2.interfaceType = XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE;
        nodeIntf1_2.interfaceAddress.ipAddress.ip = "192.168.2.2";
        nodeIntf1_2.interfaceAddress.ipAddress.port = 32212;
        nodeIntf1_3.interfaceType = (xme_core_directory_node_interfaceTypes_t) XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1;
        nodeIntf1_3.interfaceAddress.ipAddress.ip = "BlueToothAddress1";
        nodeIntf1_3.interfaceAddress.ipAddress.port = 0;
        nodeIntf1_4.interfaceType = XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE;
        nodeIntf1_4.interfaceAddress.ipAddress.ip = "192.168.2.4";
        nodeIntf1_4.interfaceAddress.ipAddress.port = 32214;
        nodeIntf1_5.interfaceType = (xme_core_directory_node_interfaceTypes_t) XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1;
        nodeIntf1_5.interfaceAddress.ipAddress.ip = "BlueToothAddress2";
        nodeIntf1_5.interfaceAddress.ipAddress.port = 0;

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_addInterface(node1, nodeIntf1_1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_addInterface(node1, nodeIntf1_2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_addInterface(node1, nodeIntf1_3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_addInterface(node1, nodeIntf1_4));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_addInterface(node1, nodeIntf1_5));

    }

    virtual ~NodeInterfaceTest()
    {
        xme_core_directory_node_fini();
    }

    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;

    xme_core_directory_node_nodeInterface_t nodeIntf1_1;
    xme_core_directory_node_nodeInterface_t nodeIntf1_2;
    xme_core_directory_node_nodeInterface_t nodeIntf1_3;
    xme_core_directory_node_nodeInterface_t nodeIntf1_4;
    xme_core_directory_node_nodeInterface_t nodeIntf1_5;

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
//     NodeInterfaceTest                                                      //
//----------------------------------------------------------------------------//

TEST_F(NodeInterfaceTest, EthernetInterfacesIterator)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, &iterator));

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    xme_core_directory_node_nodeInterface_t *temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, temp->interfaceType);
    EXPECT_STREQ("192.168.2.1", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(32211u, temp->interfaceAddress.ipAddress.port);

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, temp->interfaceType);
    EXPECT_STREQ("192.168.2.2", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(32212u, temp->interfaceAddress.ipAddress.port);

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, temp->interfaceType);
    EXPECT_STREQ("192.168.2.4", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(32214u, temp->interfaceAddress.ipAddress.port);

    EXPECT_FALSE(xme_core_directory_node_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeInterfaceTest, AllInterfacesIterator)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INVALID_INTERFACE, &iterator));

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    xme_core_directory_node_nodeInterface_t *temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, temp->interfaceType);
    EXPECT_STREQ("192.168.2.1", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(32211u, temp->interfaceAddress.ipAddress.port);

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, temp->interfaceType);
    EXPECT_STREQ("192.168.2.2", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(32212u, temp->interfaceAddress.ipAddress.port);

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ((xme_core_directory_node_interfaceTypes_t)XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1, temp->interfaceType);
    EXPECT_STREQ("BlueToothAddress1", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(0u, temp->interfaceAddress.ipAddress.port);

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_CORE_DIRECTORY_NCC_INTERFACETYPE_ETHERNET_INTERFACE, temp->interfaceType);
    EXPECT_STREQ("192.168.2.4", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(32214u, temp->interfaceAddress.ipAddress.port);

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ((xme_core_directory_node_interfaceTypes_t)XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1, temp->interfaceType);
    EXPECT_STREQ("BlueToothAddress2", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(0u, temp->interfaceAddress.ipAddress.port);

    EXPECT_FALSE(xme_core_directory_node_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeInterfaceTest, NonExistantInterfacesIterator)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, (xme_core_directory_node_interfaceTypes_t)XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_2, &iterator));

    EXPECT_FALSE(xme_core_directory_node_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeInterfaceTest, BlueToothInterfacesIterator)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, (xme_core_directory_node_interfaceTypes_t)XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1, &iterator));

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    xme_core_directory_node_nodeInterface_t *temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ((xme_core_directory_node_interfaceTypes_t)XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1, temp->interfaceType);
    EXPECT_STREQ("BlueToothAddress1", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(0u, temp->interfaceAddress.ipAddress.port);

    EXPECT_TRUE(xme_core_directory_node_hasNextInterface(iterator));
    temp = xme_core_directory_node_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ((xme_core_directory_node_interfaceTypes_t)XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INTERFACE_1, temp->interfaceType);
    EXPECT_STREQ("BlueToothAddress2", temp->interfaceAddress.ipAddress.ip);
    EXPECT_EQ(0u, temp->interfaceAddress.ipAddress.port);

    EXPECT_FALSE(xme_core_directory_node_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeInterfaceTest, EmptyInterfacesTableIterator)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)2, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INVALID_INTERFACE, &iterator));

    EXPECT_FALSE(xme_core_directory_node_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_node_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeInterfaceTest, NodeDoesNotExistIterator)
{
    xme_core_directory_node_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_node_initNodeInterfaceIterator((xme_core_node_nodeId_t)3, XME_CORE_DIRECTORY_NCC_INTERFACETYPE_INVALID_INTERFACE, &iterator));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_node_finiNodeInterfaceIterator(iterator));
}
/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
