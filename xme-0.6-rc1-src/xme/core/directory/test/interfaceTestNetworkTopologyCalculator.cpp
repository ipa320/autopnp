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
 * $Id: interfaceTestNetworkTopologyCalculator.cpp 4570 2013-08-06 10:57:02Z gulati $
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
#include "xme/core/directory/include/nodeRegistryController.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class NetworkTopologyCalculatorInterfaceTest: public ::testing::Test
{
protected:

    /*
     * \brief Pointer to the neighbors of the node, contains a list of the neighbors.
     *        Each neighbor item contains the sender ID and the sender interface ID
     */
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode1;
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode2;
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode3;
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode4;
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode5;
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode6;
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode4Updated;
    xme_core_directory_networkTopologyCalculator_neighbors_t *neighborListPointerNode3Updated;

    /*
     * \brief Data structure containing the neighborhood information of one node.
     *        Contains the node ID of the node, the list of neighbors it has, whether
     *        there was an overflow while storing the neighbors or not and the neighbor count.
     */
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode1;
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode2;
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode3;
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode4;
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode5;
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode6;
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode3Updated;
    xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t neighborInformationNode4Updated;

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

    char ip[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    uint8_t size;
    uint16_t port;

    NetworkTopologyCalculatorInterfaceTest()
    : size(sizeof(ip))
    , port(32211u)
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

        neighborListPointerNode1 = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 3);
        neighborListPointerNode2 = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 3);
        neighborListPointerNode3 = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 3);
        neighborListPointerNode4 = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 5);
        neighborListPointerNode5 = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 2);
        neighborListPointerNode6 = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 2);
        neighborListPointerNode4Updated = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 3);
        neighborListPointerNode3Updated = (xme_core_directory_networkTopologyCalculator_neighbors_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_neighbors_t) * 3);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.2:32211", &nodeIntf2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.3:32211", &nodeIntf3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.41:32211", &nodeIntf4first));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.42:32211", &nodeIntf4second));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.5:32211", &nodeIntf5));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.6:32211", &nodeIntf6));

        //data structure for node 1:
        xme_core_directory_networkTopologyCalculator_neighbors_t item;
        item.senderNodeId = (xme_core_node_nodeId_t) 2;
        item.senderInterfaceId = nodeIntf2;
        item.receiverNodeId = (xme_core_node_nodeId_t) 1;
        item.receiverInterfaceId = nodeIntf1;
        neighborListPointerNode1[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 3;
        item.senderInterfaceId = nodeIntf3;
        neighborListPointerNode1[1] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 4;
        item.senderInterfaceId = nodeIntf4first;
        neighborListPointerNode1[2] = item;

        neighborInformationNode1.nodeId = (xme_core_node_nodeId_t) 1;
        neighborInformationNode1.neighbors = neighborListPointerNode1;
        neighborInformationNode1.neighborsCount = 3;
        neighborInformationNode1.overflow = false;

        //data structure for node2:
        item.senderNodeId = (xme_core_node_nodeId_t) 1;
        item.senderInterfaceId = nodeIntf1;
        item.receiverNodeId = (xme_core_node_nodeId_t) 2;
        item.receiverInterfaceId = nodeIntf2;
        neighborListPointerNode2[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 3;
        item.senderInterfaceId = nodeIntf3;
        neighborListPointerNode2[1] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 4;
        item.senderInterfaceId = nodeIntf4first;
        neighborListPointerNode2[2] = item;

        neighborInformationNode2.nodeId = (xme_core_node_nodeId_t) 2;
        neighborInformationNode2.neighbors = neighborListPointerNode2;
        neighborInformationNode2.neighborsCount = 3;
        neighborInformationNode2.overflow = false;

        //data structure for node 3:
        item.senderNodeId = (xme_core_node_nodeId_t) 1;
        item.senderInterfaceId = nodeIntf1;
        item.receiverNodeId = (xme_core_node_nodeId_t) 3;
        item.receiverInterfaceId = nodeIntf3;
        neighborListPointerNode3[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 2;
        item.senderInterfaceId = nodeIntf2;
        neighborListPointerNode3[1] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 4;
        item.senderInterfaceId = nodeIntf4first;
        neighborListPointerNode3[2] = item;

        neighborInformationNode3.nodeId = (xme_core_node_nodeId_t) 3;
        neighborInformationNode3.neighbors = neighborListPointerNode3;
        neighborInformationNode3.neighborsCount = 3;
        neighborInformationNode3.overflow = false;

        //data structure for node 4:
        item.senderNodeId = (xme_core_node_nodeId_t) 1;
        item.senderInterfaceId = nodeIntf1;
        item.receiverNodeId = (xme_core_node_nodeId_t) 4;
        item.receiverInterfaceId = nodeIntf4first;
        neighborListPointerNode4[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 2;
        item.senderInterfaceId = nodeIntf2;
        neighborListPointerNode4[1] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 3;
        item.senderInterfaceId = nodeIntf3;
        neighborListPointerNode4[2] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 5;
        item.senderInterfaceId = nodeIntf5;
        item.receiverInterfaceId = nodeIntf4second;
        neighborListPointerNode4[3] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 6;
        item.senderInterfaceId = nodeIntf6;
        neighborListPointerNode4[4] = item;

        neighborInformationNode4.nodeId = (xme_core_node_nodeId_t) 4;
        neighborInformationNode4.neighbors = neighborListPointerNode4;
        neighborInformationNode4.neighborsCount = 5;
        neighborInformationNode4.overflow = false;

        //data structure for node 5:
        item.senderNodeId = (xme_core_node_nodeId_t) 4;
        item.senderInterfaceId = nodeIntf4second;
        item.receiverNodeId = (xme_core_node_nodeId_t) 5;
        item.receiverInterfaceId = nodeIntf5;
        neighborListPointerNode5[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 6;
        item.senderInterfaceId = nodeIntf6;
        neighborListPointerNode5[1] = item;

        neighborInformationNode5.nodeId = (xme_core_node_nodeId_t) 5;
        neighborInformationNode5.neighbors = neighborListPointerNode5;
        neighborInformationNode5.neighborsCount = 2;
        neighborInformationNode5.overflow = false;

        //data structure for node 6:
        item.senderNodeId = (xme_core_node_nodeId_t) 4;
        item.senderInterfaceId = nodeIntf4second;
        item.receiverNodeId = (xme_core_node_nodeId_t) 6;
        item.receiverInterfaceId = nodeIntf6;
        neighborListPointerNode6[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 5;
        item.senderInterfaceId = nodeIntf5;
        neighborListPointerNode6[1] = item;

        neighborInformationNode6.nodeId = (xme_core_node_nodeId_t) 6;
        neighborInformationNode6.neighbors = neighborListPointerNode6;
        neighborInformationNode6.neighborsCount = 2;
        neighborInformationNode6.overflow = false;

        //updated data structure for node 4:
        item.senderNodeId = (xme_core_node_nodeId_t) 1;
        item.senderInterfaceId = nodeIntf1;
        item.receiverNodeId = (xme_core_node_nodeId_t) 4;
        item.receiverInterfaceId = nodeIntf4first;
        neighborListPointerNode4Updated[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 3;
        item.senderInterfaceId = nodeIntf3;
        neighborListPointerNode4Updated[1] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 5;
        item.senderInterfaceId = nodeIntf5;
        item.receiverInterfaceId = nodeIntf4second;
        neighborListPointerNode4Updated[2] = item;

        neighborInformationNode4Updated.nodeId = (xme_core_node_nodeId_t) 4;
        neighborInformationNode4Updated.neighbors = neighborListPointerNode4Updated;
        neighborInformationNode4Updated.neighborsCount = 3;
        neighborInformationNode4Updated.overflow = false;

        //updated data structure for node 3:
        item.senderNodeId = (xme_core_node_nodeId_t) 1;
        item.senderInterfaceId = nodeIntf1;
        item.receiverNodeId = (xme_core_node_nodeId_t) 3;
        item.receiverInterfaceId = nodeIntf3;
        neighborListPointerNode3Updated[0] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 2;
        item.senderInterfaceId = nodeIntf2;
        neighborListPointerNode3Updated[1] = item;
        item.senderNodeId = (xme_core_node_nodeId_t) 4;
        item.senderInterfaceId = nodeIntf4first;
        neighborListPointerNode3Updated[2] = item;

        neighborInformationNode3Updated.nodeId = (xme_core_node_nodeId_t) 3;
        neighborInformationNode3Updated.neighbors = neighborListPointerNode3Updated;
        neighborInformationNode3Updated.neighborsCount = 3;
        neighborInformationNode3Updated.overflow = true;
    }

    void NetworkTopologyCalculatorCheckDataNode1(xme_com_interface_address_t* interfaceId)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(interfaceId, ip, size));
        EXPECT_STREQ(ip, "192.168.2.1:32211");
    }
    void NetworkTopologyCalculatorCheckDataNode2(xme_com_interface_address_t* interfaceId)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(interfaceId, ip, size));
        EXPECT_STREQ(ip, "192.168.2.2:32211");
    }
    void NetworkTopologyCalculatorCheckDataNode3(xme_com_interface_address_t* interfaceId)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(interfaceId, ip, size));
        EXPECT_STREQ(ip, "192.168.2.3:32211");
    }
    void NetworkTopologyCalculatorCheckDataNode4first(xme_com_interface_address_t* interfaceId)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(interfaceId, ip, size));
        EXPECT_STREQ(ip, "192.168.2.41:32211");
    }
    void NetworkTopologyCalculatorCheckDataNode4second(xme_com_interface_address_t* interfaceId)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(interfaceId, ip, size));
        EXPECT_STREQ(ip, "192.168.2.42:32211");
    }
    void NetworkTopologyCalculatorCheckDataNode5(xme_com_interface_address_t* interfaceId)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(interfaceId, ip, size));
        EXPECT_STREQ(ip, "192.168.2.5:32211");
    }
    void NetworkTopologyCalculatorCheckDataNode6(xme_com_interface_address_t* interfaceId)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(interfaceId, ip, size));
        EXPECT_STREQ(ip, "192.168.2.6:32211");
    }

    void NetworkTopologyCalculatorCheckFrom1To2(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode1(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode2(&edgeData->receiverInterfaceId);
        EXPECT_EQ(1, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(2, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom2To1(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode2(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode1(&edgeData->receiverInterfaceId);
        EXPECT_EQ(2, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(1, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom1To3(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode1(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode3(&edgeData->receiverInterfaceId);
        EXPECT_EQ(1, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(3, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom3To1(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode3(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode1(&edgeData->receiverInterfaceId);
        EXPECT_EQ(3, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(1, xme_hal_graph_getSinkVertex(currentGraph,* e));
    }
    void NetworkTopologyCalculatorCheckFrom1To4(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode1(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode4first(&edgeData->receiverInterfaceId);
        EXPECT_EQ(1, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(4, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom4To1(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode4first(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode1(&edgeData->receiverInterfaceId);
        EXPECT_EQ(4, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(1, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom2To3(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode2(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode3(&edgeData->receiverInterfaceId);
        EXPECT_EQ(2, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(3, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom3To2(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode3(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode2(&edgeData->receiverInterfaceId);
        EXPECT_EQ(3, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(2, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom2To4(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode2(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode4first(&edgeData->receiverInterfaceId);
        EXPECT_EQ(2, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(4, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom4To2(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode4first(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode2(&edgeData->receiverInterfaceId);
        EXPECT_EQ(4, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(2, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom3To4(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode3(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode4first(&edgeData->receiverInterfaceId);
        EXPECT_EQ(3, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(4, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom4To3(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode4first(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode3(&edgeData->receiverInterfaceId);
        EXPECT_EQ(4, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(3, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom5To4(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode5(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode4second(&edgeData->receiverInterfaceId);
        EXPECT_EQ(5, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(4, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom4To5(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode4second(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode5(&edgeData->receiverInterfaceId);
        EXPECT_EQ(4, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(5, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom4To6(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode4second(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode6(&edgeData->receiverInterfaceId);
        EXPECT_EQ(4, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(6, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom6To4(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode6(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode4second(&edgeData->receiverInterfaceId);
        EXPECT_EQ(6, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(4, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom5To6(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode5(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode6(&edgeData->receiverInterfaceId);
        EXPECT_EQ(5, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(6, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }
    void NetworkTopologyCalculatorCheckFrom6To5(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_edgeId_t* e)
    {
        xme_core_directory_networkTopologyCalculator_edgeData_t* edgeData;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(currentGraph, *e, (void**) &edgeData));
        NetworkTopologyCalculatorCheckDataNode6(&edgeData->senderInterfaceId);
        NetworkTopologyCalculatorCheckDataNode5(&edgeData->receiverInterfaceId);
        EXPECT_EQ(6, xme_hal_graph_getSourceVertex(currentGraph, *e));
        EXPECT_EQ(5, xme_hal_graph_getSinkVertex(currentGraph, *e));
    }

    void NetworkTopologyCalculatorCheckIncomingEdgesNode1(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect three incoming edges
        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom2To1(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom3To1(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To1(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckOutgoingEdgesNode1(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect three outgoing edges
        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom1To2(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom1To3(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom1To4(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckIncomingEdgesNode2(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect three incoming edges
        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom1To2(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom3To2(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To2(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckOutgoingEdgesNode2(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect three outgoing edges
        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom2To1(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom2To3(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom2To4(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckIncomingEdgesNode3(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect three incoming edges
        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom1To3(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom2To3(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To3(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckOutgoingEdgesNode3(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect three outgoing edges
        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom3To1(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom3To2(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom3To4(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckIncomingEdgesNode4(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect five incoming edges
        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom1To4(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom2To4(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom3To4(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom5To4(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom6To4(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckOutgoingEdgesNode4(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect five outgoing edges
        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To1(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To2(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To3(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To5(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To6(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckIncomingEdgesNode5(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect two incoming edges
        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To5(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom6To5(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckOutgoingEdgesNode5(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect two outgoing edges
        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom5To4(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom5To6(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckIncomingEdgesNode6(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect two incoming edges
        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom4To6(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
        e = xme_hal_graph_nextIncomingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom5To6(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(currentGraph, *v));
    }
    void NetworkTopologyCalculatorCheckOutgoingEdgesNode6(xme_hal_graph_graph_t* currentGraph, xme_hal_graph_vertexId_t* v)
    {
        xme_hal_graph_edgeId_t e;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(currentGraph, *v));
        //Expect two outgoing edges
        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom6To4(currentGraph, &e);

        EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
        e = xme_hal_graph_nextOutgoingEdge(currentGraph, *v);
        NetworkTopologyCalculatorCheckFrom6To5(currentGraph, &e);

        EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(currentGraph, *v));
    }

    virtual ~NetworkTopologyCalculatorInterfaceTest()
    {
        xme_hal_mem_free(neighborListPointerNode1);
        xme_hal_mem_free(neighborListPointerNode2);
        xme_hal_mem_free(neighborListPointerNode3);
        xme_hal_mem_free(neighborListPointerNode4);
        xme_hal_mem_free(neighborListPointerNode5);
        xme_hal_mem_free(neighborListPointerNode6);
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NetworkTopologyCalculatorInterfaceTest                                 //
//----------------------------------------------------------------------------//
TEST_F(NetworkTopologyCalculatorInterfaceTest, networkRepresentation)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_init(NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode1));
    xme_hal_graph_graph_t currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(4, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&currentGraph));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    int i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_hal_graph_edgeId_t e;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS
            EXPECT_EQ(NUMBER_OF_TICKS, data->lastSeenBeforeTicks);
            //Expect zero outgoing edges
            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS -1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To1(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom3To1(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To1(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode2));
    currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(4, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&currentGraph));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_hal_graph_edgeId_t e;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To2(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS
            EXPECT_EQ(NUMBER_OF_TICKS, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To1(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode2(&currentGraph, &v);
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom3To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom3To2(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To2(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode3));
    currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(4, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(9, xme_hal_graph_getEdgeCount(&currentGraph));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_hal_graph_edgeId_t e;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To2(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode2(&currentGraph, &v);
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom3To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom3To2(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode3(&currentGraph, &v);
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To2(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode4));
    currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(6, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(14, xme_hal_graph_getEdgeCount(&currentGraph));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_hal_graph_edgeId_t e;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode1(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode2(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode2(&currentGraph, &v);
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode3(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode3(&currentGraph, &v);
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To2(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect five incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode4(&currentGraph, &v);
        }
        else if(data->nodeId == 5)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To4(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else if(data->nodeId == 6)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom6To4(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode5));
    currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(6, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(16, xme_hal_graph_getEdgeCount(&currentGraph));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_hal_graph_edgeId_t e;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 4
            EXPECT_EQ(NUMBER_OF_TICKS - 4, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode1(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode2(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode2(&currentGraph, &v);
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode3(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode3(&currentGraph, &v);
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect four outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To2(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To3(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To5(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect five incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode4(&currentGraph, &v);
        }
        else if(data->nodeId == 5)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS
            EXPECT_EQ(NUMBER_OF_TICKS, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To4(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect two incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode5(&currentGraph, &v);
        }
        else if(data->nodeId == 6)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode6(&currentGraph, &v);

            //Expect zero incoming edges
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode6));
    currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(6, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(18, xme_hal_graph_getEdgeCount(&currentGraph));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 5
            EXPECT_EQ(NUMBER_OF_TICKS - 5, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode1(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 4
            EXPECT_EQ(NUMBER_OF_TICKS - 4, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode2(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode2(&currentGraph, &v);
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 4
            EXPECT_EQ(NUMBER_OF_TICKS - 4, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode3(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode3(&currentGraph, &v);
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 4
            EXPECT_EQ(NUMBER_OF_TICKS - 4, data->lastSeenBeforeTicks);
            //Expect 5 outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode4(&currentGraph, &v);

            //Expect five incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode4(&currentGraph, &v);
        }
        else if(data->nodeId == 5)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode5(&currentGraph, &v);

            //Expect two incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode5(&currentGraph, &v);
        }
        else if(data->nodeId == 6)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 1
            EXPECT_EQ(NUMBER_OF_TICKS - 1, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode6(&currentGraph, &v);

            //Expect two incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode6(&currentGraph, &v);
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode4Updated));
    currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(6, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(16, xme_hal_graph_getEdgeCount(&currentGraph));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_hal_graph_edgeId_t e;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 6
            EXPECT_EQ(NUMBER_OF_TICKS - 6, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode1(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 5
            EXPECT_EQ(NUMBER_OF_TICKS - 5, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode2(&currentGraph, &v);
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 5
            EXPECT_EQ(NUMBER_OF_TICKS - 5, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode3(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode3(&currentGraph, &v);
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect five outgoing edge
            NetworkTopologyCalculatorCheckOutgoingEdgesNode4(&currentGraph, &v);

            //Expect three incoming edges
            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
            e = xme_hal_graph_nextIncomingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To4(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
            e = xme_hal_graph_nextIncomingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom3To4(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
            e = xme_hal_graph_nextIncomingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To4(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else if(data->nodeId == 5)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            //Notice: now information for node 4 come after information for node 6,
            //        because node 4 has been recently updated!
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To6(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To4(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect two incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode5(&currentGraph, &v);
        }
        else if(data->nodeId == 6)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 2
            EXPECT_EQ(NUMBER_OF_TICKS - 2, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom6To5(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect two incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode6(&currentGraph, &v);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_networkTopologyCalculator_updateNetwork(&this->neighborInformationNode3Updated));
    currentGraph = xme_core_directory_networkTopologyCalculator_getNetworkGraph();
    EXPECT_EQ(6, xme_hal_graph_getVertexCount(&currentGraph));
    EXPECT_EQ(16, xme_hal_graph_getEdgeCount(&currentGraph));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&currentGraph));
    i = 0;
    while(xme_hal_graph_hasNextVertex(&currentGraph))
    {
        i++;
        xme_hal_graph_vertexId_t v;
        xme_hal_graph_edgeId_t e;
        xme_core_directory_networkTopologyCalculator_vertexData_t* data;

        v = xme_hal_graph_nextVertex(&currentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&currentGraph, v, (void**) &data));
        EXPECT_EQ(i, (int) data->nodeId);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&currentGraph, v));

        //Check if the edge data of the node is correct
        if(data->nodeId == 1)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 7
            EXPECT_EQ(NUMBER_OF_TICKS - 7, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To2(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To4(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode1(&currentGraph, &v);
        }
        else if(data->nodeId == 2)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 6
            EXPECT_EQ(NUMBER_OF_TICKS - 6, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom2To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode2(&currentGraph, &v);
        }
        else if(data->nodeId == 3)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 5
            EXPECT_EQ(NUMBER_OF_TICKS - 5, data->lastSeenBeforeTicks);
            //Expect three outgoing edges
            NetworkTopologyCalculatorCheckOutgoingEdgesNode3(&currentGraph, &v);

            //Expect three incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode3(&currentGraph, &v);
        }
        else if(data->nodeId == 4)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 4
            EXPECT_EQ(NUMBER_OF_TICKS - 4, data->lastSeenBeforeTicks);
            //Expect five outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To1(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To2(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To5(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To6(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom4To3(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect three incoming edges
            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
            e = xme_hal_graph_nextIncomingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom1To4(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
            e = xme_hal_graph_nextIncomingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom3To4(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
            e = xme_hal_graph_nextIncomingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To4(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&currentGraph, v));
        }
        else if(data->nodeId == 5)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect two outgoing edges
            //Notice: now information for node 4 come after information for node 6,
            //        because node 4 has been recently updated!
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To6(&currentGraph, &e);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom5To4(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect two incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode5(&currentGraph, &v);
        }
        else if(data->nodeId == 6)
        {
            //Expect last seen before ticks to be NUMBER_OF_TICKS - 3
            EXPECT_EQ(NUMBER_OF_TICKS - 3, data->lastSeenBeforeTicks);
            //Expect one outgoing edge
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));
            e = xme_hal_graph_nextOutgoingEdge(&currentGraph, v);
            NetworkTopologyCalculatorCheckFrom6To5(&currentGraph, &e);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&currentGraph, v));

            //Expect two incoming edges
            NetworkTopologyCalculatorCheckIncomingEdgesNode6(&currentGraph, &v);
        }
        else
        {
            ASSERT_TRUE(false);
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&currentGraph, v));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&currentGraph, v));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&currentGraph));
}


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
