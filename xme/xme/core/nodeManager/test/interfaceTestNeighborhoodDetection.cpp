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
 * $Id: interfaceTestNeighborhoodDetection.cpp 4995 2013-09-05 18:43:44Z ruiz $
 */

/**
 * \file
 *         Neighborhood Detection interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/nodeManager/include/neighborhoodDetection.h"
#include "xme/core/directory/include/nodeRegistryController.h"
#include "xme/core/directory/include/networkTopologyCalculator.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/net.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class NeighborhoodDetectionInterfaceTest: public ::testing::Test
{
protected:

    /*
     * \brief The interface definition of the node.
     */
    xme_com_interface_address_t nodeIntf1;
    xme_com_interface_address_t nodeIntf2;
    xme_com_interface_address_t nodeIntf3;
    xme_com_interface_address_t nodeIntf4first;
    xme_com_interface_address_t nodeIntf4second;
    xme_com_interface_address_t nodeIntf5;
    xme_com_interface_address_t nodeIntf6;

    /*
     * \brief The node IDs.
     */
    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;
    xme_core_node_nodeId_t node3;
    xme_core_node_nodeId_t node4;
    xme_core_node_nodeId_t node5;
    xme_core_node_nodeId_t node6;

    // constructor
    NeighborhoodDetectionInterfaceTest()
    {
        /*
         * Build the different data structures used for testing.
         * The graph looks as follows, where a directed edge
         * indicates that the sink node received a hello-packet from the source node:
         *                   __________
         *                  v          \
         *                  1 <----> 3  |
         *                  ^       ^^  |
         *                  |  ____/ |  |
         *                  | /      |  |
         *                  vv       v  |
         *   5 <---> 4.2=4=4.1 <---> 2 <´
         *   ^       ^
         *   |       |
         *    \      v
         *     `---> 6
         *
         * Notice that node 4 has two network interfaces, 4.1 and 4.2.
         */

        node1 = (xme_core_node_nodeId_t) 1;
        node2 = (xme_core_node_nodeId_t) 2;
        node3 = (xme_core_node_nodeId_t) 3;
        node4 = (xme_core_node_nodeId_t) 4;
        node5 = (xme_core_node_nodeId_t) 5;
        node6 = (xme_core_node_nodeId_t) 6;

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.2:32211", &nodeIntf2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.3:32211", &nodeIntf3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.41:32211", &nodeIntf4first));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.42:32211", &nodeIntf4second));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.5:32211", &nodeIntf5));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.6:32211", &nodeIntf6));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "first node", (xme_core_node_guid_t)1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "second node", (xme_core_node_guid_t)2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node3, "third node", (xme_core_node_guid_t)3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node4, "fourth node", (xme_core_node_guid_t)4));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node5, "fifth node", (xme_core_node_guid_t)5));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node6, "sixth node", (xme_core_node_guid_t)6));
        // Add network interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node2, nodeIntf2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node3, nodeIntf3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node4, nodeIntf4first));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node4, nodeIntf4second));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node5, nodeIntf5));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node6, nodeIntf6));
    }

    virtual ~NeighborhoodDetectionInterfaceTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NeighborhoodDetectionInterfaceTest                                     //
//----------------------------------------------------------------------------//
TEST_F(NeighborhoodDetectionInterfaceTest, processAnnouncementsWithoutInitializing)
{
    /*
     * \brief Data structures for the announcements of the nodes.
     */
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode2;

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf1));
}

TEST_F(NeighborhoodDetectionInterfaceTest, forwardNeighborhoodInformationWithoutInitializing)
{
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t info;
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeManager_neighborhoodDetection_forwardNeighborhoodInformation(&info));
}

TEST_F(NeighborhoodDetectionInterfaceTest, sendAnnouncementWithoutInitializing)
{
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeManager_neighborhoodDetection_sendAnnouncement());
}

TEST_F(NeighborhoodDetectionInterfaceTest, processAnnouncementsNodeWithOneInterface)
{
    /*
     * \brief Data structures for the announcements of the nodes.
     */
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode2;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode3;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode4;

    char ipString[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    uint8_t size = XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE;

    xme_core_nodeManager_neighborhoodDetection_neighborsList_t neighbors;
    uint32_t ip = 0;
    uint16_t port = 0;

    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node1));

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf1));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(1, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        EXPECT_EQ(2, currentNeighbor->senderNodeID);
        EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
        EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
        EXPECT_STREQ("192.168.2.2:32211", ipString);
        EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
        EXPECT_STREQ("192.168.2.1:32211", ipString);
        EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node3
    announcementNode3.sendingInterfaceID = this->nodeIntf3;
    announcementNode3.sendingNodeID = this->node3;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode3, &this->nodeIntf1));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(2, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2 and node 3
    int i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node4
    announcementNode4.sendingInterfaceID = this->nodeIntf4first;
    announcementNode4.sendingNodeID = this->node4;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode4, &this->nodeIntf1));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(3, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2, node 3 and node 4
    i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 2)
        {
            EXPECT_EQ(4, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node2 again
    //This shouldn't change anything
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf1));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(3, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2, node 3, node 4, node 5 and node 6
    i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 2)
        {
            EXPECT_EQ(4, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}

TEST_F(NeighborhoodDetectionInterfaceTest, processAnnouncementsNodeWithTwoInterfaces)
{
    /*
     * \brief Data structures for the announcements of the nodes.
     */
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode1;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode2;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode3;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode5;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode6;

    char ipString[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    uint8_t size = XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE;

    xme_core_nodeManager_neighborhoodDetection_neighborsList_t neighbors;
    uint32_t ip = 0;
    uint16_t port = 0;

    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node4));

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf4first));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(1, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        EXPECT_EQ(2, currentNeighbor->senderNodeID);
        EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
        EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
        EXPECT_STREQ("192.168.2.2:32211", ipString);
        EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
        EXPECT_STREQ("192.168.2.41:32211", ipString);
        EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node3
    announcementNode3.sendingInterfaceID = this->nodeIntf3;
    announcementNode3.sendingNodeID = this->node3;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode3, &this->nodeIntf4first));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(2, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2 and node 3
    int i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node1
    announcementNode1.sendingInterfaceID = this->nodeIntf1;
    announcementNode1.sendingNodeID = this->node1;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode1, &this->nodeIntf4first));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(3, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2, node 3 and node 1
    i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 2)
        {
            EXPECT_EQ(1, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node5
    announcementNode5.sendingInterfaceID = this->nodeIntf5;
    announcementNode5.sendingNodeID = this->node5;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode5, &this->nodeIntf4second));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(4, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2, node 3, node 1 and node 5
    i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 2)
        {
            EXPECT_EQ(1, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 3)
        {
            EXPECT_EQ(5, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.5:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node6
    announcementNode6.sendingInterfaceID = this->nodeIntf6;
    announcementNode6.sendingNodeID = this->node6;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode6, &this->nodeIntf4second));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(5, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2, node 3, node 1, node 5 and node 6
    i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 2)
        {
            EXPECT_EQ(1, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 3)
        {
            EXPECT_EQ(5, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.5:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 4)
        {
            EXPECT_EQ(6, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.6:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Receive announcement from node2 again
    //This shouldn't change anything
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf4first));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(5, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2, node 3, node 1, node 5 and node 6
    i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 2)
        {
            EXPECT_EQ(1, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 3)
        {
            EXPECT_EQ(5, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.5:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 4)
        {
            EXPECT_EQ(6, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.6:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}

TEST_F(NeighborhoodDetectionInterfaceTest, processAnnouncementsInvalidNodeId)
{
    /*
     * \brief Data structures for the announcements of the nodes.
     */
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode2;

    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node1));

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = XME_CORE_NODE_INVALID_NODE_ID;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf1));

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}

TEST_F(NeighborhoodDetectionInterfaceTest, processAnnouncementsInvalidAddressType)
{
    /*
     * \brief Data structures for the announcements of the nodes.
     */
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode2;

    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node1));

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingInterfaceID.addressType = XME_COM_INTERFACE_ADDRESS_TYPE_INVALID;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf1));

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    xme_com_interface_address_t interfaceAddress;
    interfaceAddress.addressType = XME_COM_INTERFACE_ADDRESS_TYPE_INVALID;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &interfaceAddress));

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}

TEST_F(NeighborhoodDetectionInterfaceTest, updateNeighborhoodInformation)
{
    /*
     * \brief Data structures for the announcements of the nodes.
     */
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode2;

    char ipString[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    uint8_t size = XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE;

    xme_core_nodeManager_neighborhoodDetection_neighborsList_t neighbors;
    uint32_t ip = 0;
    uint16_t port = 0;

    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node1));

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf1));
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(1, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 2
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        EXPECT_EQ(2, currentNeighbor->senderNodeID);
        EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
        EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
        EXPECT_STREQ("192.168.2.2:32211", ipString);
        EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
        EXPECT_STREQ("192.168.2.1:32211", ipString);
        EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    //Check that the announcement is still in the list after updating the information
    for(int i = 1; i < XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL; i++)
    {
        xme_core_nodeManager_neighborhoodDetection_updateNeighborhoodInformation();
        neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
        EXPECT_EQ(1, xme_hal_singlyLinkedList_getItemCount(&neighbors));
        //The item in the list should be the announcment of node 2
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL - i, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    }

    //Check that the announcement is removed from the list after updating the information
    xme_core_nodeManager_neighborhoodDetection_updateNeighborhoodInformation();
    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(0, xme_hal_singlyLinkedList_getItemCount(&neighbors));

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}

TEST_F(NeighborhoodDetectionInterfaceTest, forwardNeighborhoodInformation)
{
    /*
     * \brief Data structures for the announcements of the nodes.
     */
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode1;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode2;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode3;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode5;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t announcementNode6;

    char ipString[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    uint8_t size = XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE;

    xme_core_nodeManager_neighborhoodDetection_neighborsList_t neighbors;
    uint32_t ip = 0;
    uint16_t port = 0;

    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node4));

    //Receive announcement from node1
    announcementNode1.sendingInterfaceID = this->nodeIntf1;
    announcementNode1.sendingNodeID = this->node1;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode1, &this->nodeIntf4first));

    //Receive announcement from node2
    announcementNode2.sendingInterfaceID = this->nodeIntf2;
    announcementNode2.sendingNodeID = this->node2;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode2, &this->nodeIntf4first));

    //Receive announcement from node3
    announcementNode3.sendingInterfaceID = this->nodeIntf3;
    announcementNode3.sendingNodeID = this->node3;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode3, &this->nodeIntf4first));

    //Receive announcement from node5
    announcementNode5.sendingInterfaceID = this->nodeIntf5;
    announcementNode5.sendingNodeID = this->node5;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode5, &this->nodeIntf4second));

    //Receive announcement from node6
    announcementNode6.sendingInterfaceID = this->nodeIntf6;
    announcementNode6.sendingNodeID = this->node6;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements(&announcementNode6, &this->nodeIntf4second));

    neighbors = xme_core_nodeManager_neighborhoodDetection_getNeighborList();
    EXPECT_EQ(5, xme_hal_singlyLinkedList_getItemCount(&neighbors));
    //The item in the list should be the announcment of node 1, node 2, node 3, node 5 and node 6
    int i = 0;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighbors, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentNeighbor)
    {
        if(i == 0)
        {
            EXPECT_EQ(1, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 1)
        {
            EXPECT_EQ(2, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 2)
        {
            EXPECT_EQ(3, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 3)
        {
            EXPECT_EQ(5, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.5:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else if(i == 4)
        {
            EXPECT_EQ(6, currentNeighbor->senderNodeID);
            EXPECT_EQ(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, currentNeighbor->lastSeenBeforeTicks);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, currentNeighbor->senderInterfaceID.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->senderInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->senderInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.6:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&currentNeighbor->receivingInterfaceID, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&currentNeighbor->receivingInterfaceID, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            i++;
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t info;
    info.neighbors = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * NEIGHBOR_COUNT);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_forwardNeighborhoodInformation(&info));
    EXPECT_EQ(this->node4, info.nodeId);
    EXPECT_EQ(5, info.neighborsCount);
    EXPECT_FALSE(info.overflow);
    for(i = 0; i < info.neighborsCount; i++)
    {
        if(i == 0)
        {
            EXPECT_EQ(this->node1, info.neighbors[i].senderNodeId);
            EXPECT_EQ(this->node4, info.neighbors[i].receiverNodeId);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, info.neighbors[i].senderInterfaceId.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].senderInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].senderInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.1:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].receiverInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].receiverInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        }
        else if(i == 1)
        {
            EXPECT_EQ(this->node2, info.neighbors[i].senderNodeId);
            EXPECT_EQ(this->node4, info.neighbors[i].receiverNodeId);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, info.neighbors[i].senderInterfaceId.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].senderInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].senderInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.2:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].receiverInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].receiverInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        }
        else if(i == 2)
        {
            EXPECT_EQ(this->node3, info.neighbors[i].senderNodeId);
            EXPECT_EQ(this->node4, info.neighbors[i].receiverNodeId);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, info.neighbors[i].senderInterfaceId.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].senderInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].senderInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.3:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].receiverInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].receiverInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.41:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        }
        else if(i == 3)
        {
            EXPECT_EQ(this->node5, info.neighbors[i].senderNodeId);
            EXPECT_EQ(this->node4, info.neighbors[i].receiverNodeId);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, info.neighbors[i].senderInterfaceId.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].senderInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].senderInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.5:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].receiverInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].receiverInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        }
        else if(i == 4)
        {
            EXPECT_EQ(this->node6, info.neighbors[i].senderNodeId);
            EXPECT_EQ(this->node4, info.neighbors[i].receiverNodeId);
            EXPECT_EQ(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, info.neighbors[i].senderInterfaceId.addressType);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].senderInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].senderInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.6:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&info.neighbors[i].receiverInterfaceId, ipString, size));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&info.neighbors[i].receiverInterfaceId, &ip, &port));
            EXPECT_STREQ("192.168.2.42:32211", ipString);
            EXPECT_EQ(32211U, xme_hal_net_ntohs(port));
        }
        else
        {
            ASSERT_TRUE(false);
        }
    }

    xme_hal_mem_free(info.neighbors);

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}

TEST_F(NeighborhoodDetectionInterfaceTest, sendAnnouncement)
{
    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sharedPtr_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_sendAnnouncement());

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_sharedPtr_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}

/*TEST_F(NeighborhoodDetectionInterfaceTest, receiveFromNetwork)
{
    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sharedPtr_init());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&this->node1));
    while(1)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_sendAnnouncement());
        receiveFromNetwork();
    }

    xme_core_nodeManager_neighborhoodDetection_fini();
    xme_hal_sharedPtr_fini();
    xme_hal_net_fini();
    xme_hal_sync_fini();
}*/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
