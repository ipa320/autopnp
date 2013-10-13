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
 * $Id: interfaceTestNetworkConfigurationCalculator.cpp 5117 2013-09-19 10:13:40Z wiesmueller $
 */

/**
 * \file
 *         Network Configuration Calculator interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/networkConfigurationCalculator.h"

#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/com/interface.h"
#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class NetworkConfigurationCalculatorInterfaceTest: public ::testing::Test
{
protected:
    NetworkConfigurationCalculatorInterfaceTest()
    : node1((xme_core_node_nodeId_t)1)
    , node2((xme_core_node_nodeId_t)2)
    , cont1((xme_core_container_t)1)
    , cont2((xme_core_container_t)2)
    , comp1((xme_core_component_t)1)
    , comp2((xme_core_component_t)2)
    , comp3((xme_core_component_t)3)
    , port1((xme_core_dataManager_dataPacketId_t)1)
    , port2((xme_core_dataManager_dataPacketId_t)2)
    , topic(XME_CORE_TOPIC(42))
    , chan1((xme_core_channelId_t)1)
    , chan2((xme_core_channelId_t)2)
    {
        /*
         * Builds the following graph of logical routes:
         * 
         *       .-------> vs1
         *      /  LR e1
         *    vp
         *      \  LR e2
         *       '-------> vs2
         *
         *
         *  .-----------.-------.-------.-------.
         *  | attribute |  vp   |  vs1  |  vs2  |
         *  |-----------|-------|-------|-------|
         *  | topic     | topic | topic | topic |
         *  | node      | node1 | node2 | node1 |
         *  | container | cont1 | cont2 | cont1 |
         *  | component | comp1 | comp2 | comp3 |
         *  | port      | port1 | port1 | port2 |
         *  '-----------'-------'-------'-------'
         * 
         *  .-----------.-------.-------.
         *  | attribute |  e1   |  e2   |
         *  |-----------|-------|-------|
         *  | topic     | topic | topic |
         *  | channel   | chan1 | chan2 |
         *  '-----------'-------'-------'
         * 
         *  Key:
         *  vp  = publication vertex
         *  vs1 = subscription component port vertex 1
         *  vs2 = subscription component port vertex 2
         *  e1  = logical route edge 1
         *  e2  = logical route edge 2
         *  mX  - marshaling waypoint for route X
         *  sX  - UDP send waypoint for route X
         *  rX  - UDP receive waypoint for route X
         *  dX  - demarshaling waypoint for route X
         *  LR  - logical route edge
         *  MC  - memcopy edge
         *  UDP - UDP link edge
         *
         * After the Network Configuration Calculator has calculated
         * the waypoints, the following graph is expected:
         *
         *      MC      MC       UDP      MC       MC
         *      .--> m1 ---> sA ----> rA ---> dA ---> vs1
         *     /                 
         *    vp
         *     \                 MC
         *      '-----------------------------------> vs2
         */

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "vp and vs2 node", (xme_core_node_guid_t)1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "vs1 node", (xme_core_node_guid_t)2));

        // Add network interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.2:32211", &nodeIntf2));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node2, nodeIntf2));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&logicalRoutes));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&deploymentGraph));

        // Add vp
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vpd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vpd,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            NULL // announcement should not be needed
        );
        vpd.vertexId = xme_hal_graph_addVertex(&logicalRoutes, &vpd);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vpd.vertexId);

        // Add vs1
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vs1d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vs1d,
            comp2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            NULL // announcement should not be needed
        );
        vs1d.vertexId = xme_hal_graph_addVertex(&logicalRoutes, &vs1d);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs1d.vertexId);

        // Add vs2
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vs2d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vs2d,
            comp3,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            NULL // announcement should not be needed
        );
        vs2d.vertexId = xme_hal_graph_addVertex(&logicalRoutes, &vs2d);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs2d.vertexId);

        // Add e1
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e1d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e1d,
            chan1,
            false
        );
        e1d.edgeId = xme_hal_graph_addEdge(&logicalRoutes, vpd.vertexId, vs1d.vertexId, &e1d);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1d.edgeId);

        // Add e2
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e2d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e2d,
            chan2,
            false
        );
        e2d.edgeId = xme_hal_graph_addEdge(&logicalRoutes, vpd.vertexId, vs2d.vertexId, &e2d);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2d.edgeId);
    }

    virtual ~NetworkConfigurationCalculatorInterfaceTest()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&deploymentGraph));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&logicalRoutes));

        xme_core_directory_nodeRegistryController_fini();
    }

    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;

    xme_com_interface_address_t nodeIntf1;
    xme_com_interface_address_t nodeIntf2;

    xme_core_container_t cont1;
    xme_core_container_t cont2;

    xme_core_component_t comp1;
    xme_core_component_t comp2;
    xme_core_component_t comp3;

    xme_core_dataManager_dataPacketId_t port1;
    xme_core_dataManager_dataPacketId_t port2;

    xme_core_topic_t topic;

    xme_core_channelId_t chan1;
    xme_core_channelId_t chan2;

    xme_core_pnp_dataLinkGraph_vertexData_t vpd;
    xme_core_pnp_dataLinkGraph_vertexData_t vs1d;
    xme_core_pnp_dataLinkGraph_vertexData_t vs2d;

    xme_core_pnp_dataLinkGraph_edgeData_t e1d;
    xme_core_pnp_dataLinkGraph_edgeData_t e2d;

    xme_hal_graph_graph_t logicalRoutes;
    xme_hal_graph_graph_t deploymentGraph;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NetworkConfigurationCalculatorInterfaceTest                            //
//----------------------------------------------------------------------------//

TEST_F(NetworkConfigurationCalculatorInterfaceTest, waypointTransform)
{
    xme_hal_graph_vertexId_t vp, vs1, vs2;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_ncc_getPhysicalRoutes(&logicalRoutes, &deploymentGraph));

    EXPECT_EQ(7, xme_hal_graph_getVertexCount(&deploymentGraph));
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&deploymentGraph));

    vp = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &vpd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(vpd.vertexId, vp);

    vs1 = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &vs1d, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs1);

    vs2 = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &vs2d, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&deploymentGraph));
    while (xme_hal_graph_hasNextVertex(&deploymentGraph))
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;

        v = xme_hal_graph_nextVertex(&deploymentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&deploymentGraph, v, (void**) &vd));

        if (vp == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd->vertexData.componentPortVertex.portType);

            // Expect four outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vp, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vp, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vs2, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect zero incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vs1 == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp2, vd->vertexData.componentPortVertex.componentId); 

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect two incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vs1, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vs2 == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp3, vd->vertexData.componentPortVertex.componentId);

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect two incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vp, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vs2, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vp, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);
            EXPECT_EQ((topic >>  8) & 0xFF, ed->edgeData.udpLinkEdge.key[0]);
            EXPECT_EQ((topic >>  0) & 0xFF, ed->edgeData.udpLinkEdge.key[1]);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);
            EXPECT_EQ((topic >>  8) & 0xFF, ed->edgeData.udpLinkEdge.key[0]);
            EXPECT_EQ((topic >>  0) & 0xFF, ed->edgeData.udpLinkEdge.key[1]);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vs1, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));

            EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
            EXPECT_EQ(topic, ed->topicId);

            vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
            EXPECT_EQ(vd->vertexId, vi);

            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
    }
    xme_hal_graph_finiVertexIterator(&deploymentGraph);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
