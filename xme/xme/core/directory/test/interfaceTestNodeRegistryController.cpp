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
 * $Id: interfaceTestNodeRegistryController.cpp 6623 2014-02-05 10:43:23Z wiesmueller $
 */

/**
 * \file
 *         Node Iterator interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include "xme/core/directory/include/nodeRegistryController.h"

#include "xme/hal/include/net.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/random.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class NodeRegistryControllerSimpleInterfaceTest: public ::testing::Test
{
protected:
    NodeRegistryControllerSimpleInterfaceTest()
    : node1((xme_core_node_nodeId_t)1)
    , node2((xme_core_node_nodeId_t)2)
    , node3((xme_core_node_nodeId_t)3)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        guid1 = (xme_core_node_guid_t) xme_hal_random_rand();
        guid2 = guid1 + 1;
    }

    virtual ~NodeRegistryControllerSimpleInterfaceTest()
    {
        xme_core_directory_nodeRegistryController_fini();
    }

    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;
    xme_core_node_nodeId_t node3;

    xme_com_interface_address_t nodeIntf1_1;
    xme_com_interface_address_t nodeIntf1_2;
    xme_com_interface_address_t nodeIntf1_3;
    xme_com_interface_address_t nodeIntf1_4;
    xme_com_interface_address_t nodeIntf1_5;

    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;
};

class NodeRegistryControllerAddInterfaceTest: public ::testing::Test
{
protected:
    NodeRegistryControllerAddInterfaceTest()
    : node1((xme_core_node_nodeId_t)1)
    , node2((xme_core_node_nodeId_t)2)
    , ip(0)
    , size(64)
    , port(0)
    {
        ip = (char*) xme_hal_mem_alloc(size);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        guid1 = (xme_core_node_guid_t) xme_hal_random_rand();
        guid2 = (xme_core_node_guid_t) xme_hal_random_rand();

        nodeIntf1 = (xme_com_interface_address_t*) xme_hal_mem_alloc(sizeof(xme_com_interface_address_t));
        nodeIntf2 = (xme_com_interface_address_t*) xme_hal_mem_alloc(sizeof(xme_com_interface_address_t));

        nodeIntf = (xme_com_interface_address_t*) xme_hal_mem_alloc(sizeof(xme_com_interface_address_t));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32212", nodeIntf2));
    }

    virtual ~NodeRegistryControllerAddInterfaceTest()
    {
        xme_core_directory_nodeRegistryController_fini();
    }

    xme_status_t
    xme_com_interface_ipv6StringToGenericAddress
    (
        const char* ip6,
        xme_com_interface_address_t *interfaceAddress
    )
    {
        xme_status_t status = xme_com_interface_ipv4StringToGenericAddress(ip6, interfaceAddress);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        interfaceAddress->addressType = XME_COM_INTERFACE_ADDRESS_TYPE_IPV6;

        return XME_STATUS_SUCCESS;
    }

    xme_status_t
    xme_com_interface_genericAddressToIPv6String
    (
        xme_com_interface_address_t *interfaceAddress,
        char* const ip6,
        uint8_t size6
    )
    {
        uint16_t port6 = 0;
        XME_CHECK( NULL != ip6 && NULL != interfaceAddress && (uint16_t)XME_COM_INTERFACE_ADDRESS_TYPE_IPV6 == interfaceAddress->addressType, XME_STATUS_INVALID_PARAMETER);
        port6 = xme_hal_net_ntohs(((uint16_t)(interfaceAddress->data.ipv6.port[1]) << 8) | interfaceAddress->data.ipv6.port[0]);
        xme_hal_safeString_snprintf( ip, size6, "%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u:%u",
            interfaceAddress->data.ipv6.ip[0],interfaceAddress->data.ipv6.ip[1],interfaceAddress->data.ipv6.ip[2],interfaceAddress->data.ipv6.ip[3], 
            interfaceAddress->data.ipv6.ip[4],interfaceAddress->data.ipv6.ip[5],interfaceAddress->data.ipv6.ip[6],interfaceAddress->data.ipv6.ip[7],
            interfaceAddress->data.ipv6.ip[8],interfaceAddress->data.ipv6.ip[9],interfaceAddress->data.ipv6.ip[10],interfaceAddress->data.ipv6.ip[11],
            interfaceAddress->data.ipv6.ip[12],interfaceAddress->data.ipv6.ip[13],interfaceAddress->data.ipv6.ip[14],interfaceAddress->data.ipv6.ip[15], port6);
    
        return XME_STATUS_SUCCESS;
    }


    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;

    xme_com_interface_address_t* nodeIntf1;
    xme_com_interface_address_t* nodeIntf2;

    xme_com_interface_address_t* nodeIntf;

    char* ip;
    uint8_t size;
    uint16_t port;
    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;

    typedef enum 
    {
        XME_COM_INTERFACE_ADDRESS_TYPE_INTERFACE1 = 100,
        XME_COM_INTERFACE_ADDRESS_TYPE_INTERFACE2
    }
    moreInterfaces;
};

class NodeRegistryControllerInterfaceTest: public ::testing::Test
{
protected:
    NodeRegistryControllerInterfaceTest()
    : node1((xme_core_node_nodeId_t)1)
    , node2((xme_core_node_nodeId_t)2)
    , ip(0)
    , size(64)
    , port(0)
    {
        ip = (char*) xme_hal_mem_alloc(size);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        guid1 = (xme_core_node_guid_t) xme_hal_random_rand();
        guid2 = (xme_core_node_guid_t) xme_hal_random_rand();

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", guid1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "node2", guid2));

        // Add interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1_1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.2:32212", &nodeIntf1_2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv6StringToGenericAddress("192.168.2.3:32213", &nodeIntf1_3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.4:32214", &nodeIntf1_4));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv6StringToGenericAddress("192.168.2.5:32215", &nodeIntf1_5));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1_1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1_2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1_3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1_4));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1_5));

    }

    virtual ~NodeRegistryControllerInterfaceTest()
    {
        xme_core_directory_nodeRegistryController_fini();
    }

    xme_status_t
    xme_com_interface_ipv6StringToGenericAddress
    (
        const char* ip6,
        xme_com_interface_address_t *interfaceAddress
    )
    {
        xme_status_t status = xme_com_interface_ipv4StringToGenericAddress(ip6, interfaceAddress);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        interfaceAddress->addressType = XME_COM_INTERFACE_ADDRESS_TYPE_IPV6;

        return XME_STATUS_SUCCESS;
    }

    xme_status_t
    xme_com_interface_genericAddressToIPv6String
    (
        xme_com_interface_address_t *interfaceAddress,
        char* const ip6,
        uint8_t size6
    )
    {
        uint16_t port6 = 0;
        XME_CHECK( NULL != ip6 && NULL != interfaceAddress && (uint16_t)XME_COM_INTERFACE_ADDRESS_TYPE_IPV6 == interfaceAddress->addressType, XME_STATUS_INVALID_PARAMETER);
        port6 = xme_hal_net_ntohs(((uint16_t)(interfaceAddress->data.ipv6.port[1]) << 8) | interfaceAddress->data.ipv6.port[0]);
        xme_hal_safeString_snprintf( ip, size6, "%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u.%u:%u",
            interfaceAddress->data.ipv6.ip[0],interfaceAddress->data.ipv6.ip[1],interfaceAddress->data.ipv6.ip[2],interfaceAddress->data.ipv6.ip[3], 
            interfaceAddress->data.ipv6.ip[4],interfaceAddress->data.ipv6.ip[5],interfaceAddress->data.ipv6.ip[6],interfaceAddress->data.ipv6.ip[7],
            interfaceAddress->data.ipv6.ip[8],interfaceAddress->data.ipv6.ip[9],interfaceAddress->data.ipv6.ip[10],interfaceAddress->data.ipv6.ip[11],
            interfaceAddress->data.ipv6.ip[12],interfaceAddress->data.ipv6.ip[13],interfaceAddress->data.ipv6.ip[14],interfaceAddress->data.ipv6.ip[15], port6);
    
        return XME_STATUS_SUCCESS;
    }


    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;

    xme_com_interface_address_t nodeIntf1_1;
    xme_com_interface_address_t nodeIntf1_2;
    xme_com_interface_address_t nodeIntf1_3;
    xme_com_interface_address_t nodeIntf1_4;
    xme_com_interface_address_t nodeIntf1_5;

    char* ip;
    uint8_t size;
    uint16_t port;
    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;

    typedef enum 
    {
        XME_COM_INTERFACE_ADDRESS_TYPE_INTERFACE1 = 100,
        XME_COM_INTERFACE_ADDRESS_TYPE_INTERFACE2
    }
    moreInterfaces;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NodeInterfaceTest                                                     //
//----------------------------------------------------------------------------//


TEST_F(NodeRegistryControllerSimpleInterfaceTest, SimpleNodeRegistrationWithInvalidNodeId)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_registerNode(XME_CORE_NODE_INVALID_NODE_ID, NULL, 0u));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, NodeRegistrationWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_registerNode(XME_CORE_NODE_INVALID_NODE_ID, "node0", 5u));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_registerNode(node1, NULL, 5u));  
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_registerNode(node2, "node2", 0u));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, NodeRegistrationWithValidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, NodeRegistrationTwice)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "node2", 6u));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, NodeRegistrationWithDifferentNodeIdButSameGUID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
    // TODO: Check if this case is possible. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "node2", 5u));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, CheckIfRegisteredGUIDWithoutNodeRegistration)
{
    xme_core_node_nodeId_t nodeID;

    ASSERT_FALSE(xme_core_directory_nodeRegistryController_isNodeGuidRegistered(0u, NULL));
    ASSERT_FALSE(xme_core_directory_nodeRegistryController_isNodeGuidRegistered(5u, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
    ASSERT_TRUE(xme_core_directory_nodeRegistryController_isNodeGuidRegistered(5u, NULL));
    ASSERT_TRUE(xme_core_directory_nodeRegistryController_isNodeGuidRegistered(5u, &nodeID));
    EXPECT_EQ(node1, nodeID);
    ASSERT_FALSE(xme_core_directory_nodeRegistryController_isNodeGuidRegistered(6u, NULL));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, GetNodeIdFromGUIDWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getNodeIdFromGUID(0u, NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getNodeIdFromGUID(5u, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getNodeIdFromGUID(5u, &node1));
    ASSERT_EQ(XME_CORE_NODE_INVALID_NODE_ID, node1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "node2", 5u));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getNodeIdFromGUID(5u, NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getNodeIdFromGUID(0u, &node2));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, GetNodeIdFromGUIDWithValidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_getNodeIdFromGUID(5u, &node1));
    ASSERT_EQ(XME_CORE_NODE_LOCAL_NODE_ID, node1);
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getNodeIdFromGUID(6u, &node1));
    ASSERT_EQ(XME_CORE_NODE_INVALID_NODE_ID, node1);
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, GetGUIDFromNodeIdWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(XME_CORE_NODE_INVALID_NODE_ID, NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(node1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(node1, &guid1));
    ASSERT_EQ(0u, guid1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(XME_CORE_NODE_INVALID_NODE_ID, NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(node1, NULL));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, GetGUIDFromNodeIdWithValidParameters)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(node1, &guid1));
    ASSERT_EQ(0U, guid1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 5u));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(node1, &guid1));
    ASSERT_EQ(5U, guid1);
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getGUIDFromNodeId(node2, &guid1));
    ASSERT_EQ(0U, guid1);
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, getNodeName)
{
    char buffer[10];
    char smallBuffer[5];
    const char* const node1Name = "node1a";
    const char* const node2Name = "node2ab";
    const char* const node3Name = "node3abc";

    // Check invalid parameters
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getNodeName(XME_CORE_NODE_INVALID_NODE_ID, NULL, 0));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getNodeName(node1, NULL, 0));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getNodeName(XME_CORE_NODE_INVALID_NODE_ID, buffer, 0));
    
    // Check for non-registered node
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getNodeName(node1, buffer, sizeof(buffer)));

    // Register some nodes for further tests
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, node1Name, 7u));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, node2Name, 8u));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node3, node3Name, 9u));

    // Get and check name of all three nodes
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_getNodeName(node1, buffer, sizeof(buffer)));
    EXPECT_EQ(0, strcmp(buffer, node1Name));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_getNodeName(node2, buffer, sizeof(buffer)));
    EXPECT_EQ(0, strcmp(buffer, node2Name));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_getNodeName(node3, buffer, sizeof(buffer)));
    EXPECT_EQ(0, strcmp(buffer, node3Name));

    // Test truncation when buffer is too small
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_getNodeName(node1, smallBuffer, sizeof(smallBuffer)));
    EXPECT_EQ(0, strcmp(smallBuffer, "node"));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, nodeIDIteratorInterface)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_initNodeIDIterator(NULL));

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextNodeID(NULL));

    EXPECT_EQ(XME_CORE_NODE_INVALID_NODE_ID, xme_core_directory_nodeRegistryController_nextNodeID(NULL));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_finiNodeIDIterator(NULL));
}

TEST_F(NodeRegistryControllerSimpleInterfaceTest, nodeIDIterator)
{
    xme_core_directory_nodeRegistryController_nodeIDIterator_t iterator;

    // No nodes registered
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeIDIterator(&iterator));

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeIDIterator(&iterator));

    // -------------

    // Register three nodes for testing the iterator
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", 6u));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "node2", 6u));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node3, "node3", 6u));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeIDIterator(&iterator)); // Iterator should work again after fini + init

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator)); // Repeated calls of hasNext must not change iterator state
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_EQ(node1, xme_core_directory_nodeRegistryController_nextNodeID(&iterator));
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_EQ(node2, xme_core_directory_nodeRegistryController_nextNodeID(&iterator));
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_EQ(node3, xme_core_directory_nodeRegistryController_nextNodeID(&iterator));
    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeIDIterator(&iterator));

    // -------------

    // Check again if iterator is working after fini + init
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeIDIterator(&iterator));

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_EQ(node1, xme_core_directory_nodeRegistryController_nextNodeID(&iterator));
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_EQ(node2, xme_core_directory_nodeRegistryController_nextNodeID(&iterator));
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));
    EXPECT_EQ(node3, xme_core_directory_nodeRegistryController_nextNodeID(&iterator));
    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextNodeID(&iterator));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeIDIterator(&iterator));
}

//////////////////////////////////////////////////////////////////////////////////

TEST_F(NodeRegistryControllerAddInterfaceTest, AddInterfaceWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_addInterface(XME_CORE_NODE_INVALID_NODE_ID, *nodeIntf1));
}

TEST_F(NodeRegistryControllerAddInterfaceTest, AddInterfaceWithValidParameters)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_addInterface(node1, *nodeIntf1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", guid1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, *nodeIntf1));
}

TEST_F(NodeRegistryControllerAddInterfaceTest, AddInterfaceAddingTwiceTheInterface)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_addInterface(node1, *nodeIntf1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", guid1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, *nodeIntf1));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_directory_nodeRegistryController_addInterface(node1, *nodeIntf1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, *nodeIntf2));
}

TEST_F(NodeRegistryControllerAddInterfaceTest, GetInterfaceWitInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getInterface(XME_CORE_NODE_INVALID_NODE_ID, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &nodeIntf));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &nodeIntf));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, NULL));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", guid1));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getInterface(XME_CORE_NODE_INVALID_NODE_ID, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &nodeIntf));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &nodeIntf));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, NULL));
}

TEST_F(NodeRegistryControllerAddInterfaceTest, GetInterfaceWitValidParameters)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &nodeIntf));
    
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node1", guid1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &nodeIntf));
    
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, *nodeIntf1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &nodeIntf));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_getInterface(node1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV6, &nodeIntf));
}

//////////////////////////////////////////////////////////////////////////////////

TEST_F(NodeRegistryControllerInterfaceTest, EthernetInterfacesIterator)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &iterator));

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    xme_com_interface_address_t *temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL != temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(temp, ip, size));
    EXPECT_STREQ("192.168.2.1:32211", ip);

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL != temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(temp, ip, size));
    EXPECT_STREQ("192.168.2.2:32212", ip);

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL != temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(temp, ip, size));
    EXPECT_STREQ("192.168.2.4:32214", ip);

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeRegistryControllerInterfaceTest, AllInterfacesIterator)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator));

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    xme_com_interface_address_t *temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL != temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(temp, ip, size));
    EXPECT_STREQ("192.168.2.1:32211", ip);

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL != temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(temp, ip, size));
    EXPECT_STREQ("192.168.2.2:32212", ip);

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv6String(temp, ip, size));
    EXPECT_STREQ("0.0.0.0.0.0.0.0.0.0.255.255.192.168.2.3:32213", ip);

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(temp, ip, size));
    EXPECT_STREQ("192.168.2.4:32214", ip);

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    // Calling hasNextInterface twice should not shift the iterator (Issue #3743)
    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv6String(temp, ip, size));
    EXPECT_STREQ("0.0.0.0.0.0.0.0.0.0.255.255.192.168.2.5:32215", ip);

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));

    // Calling nextInterface even after hasNextInterface has returned false should return NULL
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL == temp);

    // hasNextInterface should always return false once it has returned false
    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));

    // Calling nextInterface even after hasNextInterface has returned false should nor wrap around
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL == temp);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeRegistryControllerInterfaceTest, NonExistantInterfacesIterator)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, (xme_com_interface_addressType_t)XME_COM_INTERFACE_ADDRESS_TYPE_INTERFACE2, &iterator));

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeRegistryControllerInterfaceTest, IPv6InterfacesIterator)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)1, XME_COM_INTERFACE_ADDRESS_TYPE_IPV6, &iterator));

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    xme_com_interface_address_t *temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv6String(temp, ip, size));
    EXPECT_STREQ("0.0.0.0.0.0.0.0.0.0.255.255.192.168.2.3:32213", ip);

    EXPECT_TRUE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    temp = xme_core_directory_nodeRegistryController_nextInterface(iterator);
    EXPECT_TRUE(NULL!=temp);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv6String(temp, ip, size));
    EXPECT_STREQ("0.0.0.0.0.0.0.0.0.0.255.255.192.168.2.5:32215", ip);

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeRegistryControllerInterfaceTest, EmptyInterfacesTableIterator)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)2, XME_COM_INTERFACE_ADDRESS_TYPE_IPV6, &iterator));

    EXPECT_FALSE(xme_core_directory_nodeRegistryController_hasNextInterface(iterator));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator));
}

TEST_F(NodeRegistryControllerInterfaceTest, NodeDoesNotExistIterator)
{
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator=NULL;
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_nodeRegistryController_initNodeInterfaceIterator((xme_core_node_nodeId_t)3, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator));
}
/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
