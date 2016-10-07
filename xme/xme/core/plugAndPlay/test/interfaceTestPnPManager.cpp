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
 * $Id: interfaceTestPnPManager.cpp 7694 2014-03-06 15:56:24Z wiesmueller $
 */

/**
 * \file
 *         Plug and Play Manager integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"
#include "xme/core/plugAndPlay/include/plugAndPlayManagerInternalMethods.h"
#include "xme/core/plugAndPlay/include/plugAndPlayManagerInternalTypes.h"

#include "xme/defines.h"
#include "xme/core/node.h"
#include "xme/core/testUtils.h"
#include "xme/core/topic.h"
#include "xme/core/manifestRepository/include/manifestRepository.h"
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"
#include "xme/core/plugAndPlay/include/networkConfigurationCalculator.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

#include <string.h>

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PnPManagerInterfaceTestSplitGraph: public xme::testing::Test
{
protected:
    PnPManagerInterfaceTestSplitGraph()
    {
        udpKey[0] = 1;
        udpKey[1] = 2;
        udpKey[2] = 3;
        udpKey[3] = 4;
    }

    virtual ~PnPManagerInterfaceTestSplitGraph()
    {
    }

    virtual void AssertionCheckedSetUp()
    { 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

        /*
         * Builds the following graph in graph descriptor "nccGraph" (see key below):
         * 
         *       MC       MC      UDP      MC       MC
         *       .--> mA ---> sA ----> rA ---> dA ---.
         *      /                                     \
         *    cA ---------------------------------.    |
         *                       LR               _\| \|/
         *                                           cC
         *                       LR               '/| /|\
         *    cB ---------------------------------'    |
         *      \                                     /
         *       '--> mB ---> sB ----> rB ---> dB ---'
         *       MC       MC      UDP      MC       MC
         * 
         *  .-----------.-------.-------.-------.
         *  | attribute |  cA   |  cB   |  cC   |
         *  |-----------|-------|-------|-------|
         *  | topic     | topic | topic | topic |
         *  | node      | node1 | node2 | node3 |
         *  | container | cont1 | cont2 | cont3 |
         *  | component | comp1 | comp2 | comp3 |
         *  | port      | dpid1 | dpid2 | dpid3 |
         *  '-----------'-------'-------'-------'
         * 
         *  .-----------.---------------.---------------.
         *  | attribute |  LR cA -> cC  |  LR cB -> cC  |
         *  |-----------|---------------|---------------|
         *  | topic     |     topic     |     topic     |
         *  | channel   |     chan1     |     chan2     |
         *  '-----------'---------------'---------------'
         * 
         *  Key:
         *  cX  - component X
         *  mX  - marshaling waypoint for component X
         *  sX  - UDP send waypoint for component X
         *  rX  - UDP receive waypoint for component X
         *  dX  - demarshaling waypoint for component X
         *  LR  - logical route edge
         *  MC  - memcopy edge
         *  UDP - UDP link edge
         */

        xme_com_interface_ipv4StringToGenericAddress("192.168.0.1:8080", &addr);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  GRAPH: Part A                                                                                        ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraph));

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentA,
            //node1,
            //cont1,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            //dpid1,
            NULL
        );

        // Populate the marshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &marshalerA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );

        // Populate the UPD Send waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpSendA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
        (
            &udpSendA,
            udpKey,
            &addr
        );

        // Create vertices

        componentA.vertexId = xme_hal_graph_addVertex(&nccGraph, &componentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentA.vertexId);

        marshalerA.vertexId = xme_hal_graph_addVertex(&nccGraph, &marshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, marshalerA.vertexId);

        udpSendA.vertexId = xme_hal_graph_addVertex(&nccGraph, &udpSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpSendA.vertexId);

        // Populate the edge data

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToMarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToMarshalerA,
            topic,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &marshalerToUDPSendA,
            topic,
            channelID1
        );

        // Create edges

        componentToMarshalerA.edgeId = xme_hal_graph_addEdge(&nccGraph, componentA.vertexId, marshalerA.vertexId, &componentToMarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToMarshalerA.edgeId);

        marshalerToUDPSendA.edgeId = xme_hal_graph_addEdge(&nccGraph, marshalerA.vertexId, udpSendA.vertexId, &marshalerToUDPSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, marshalerToUDPSendA.edgeId);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  GRAPH: Part C-A                                                                                      ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Populate the UPD Receive waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpReceiveCfromA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
        (
            &udpReceiveCfromA,
            udpKey,
            &addr
        );

        // Populate the demarshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &demarshalerCfromA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData(&demarshalerCfromA, 1u);

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentC,
            //node3,
            //cont3,
            comp3,
            (xme_core_componentType_t)comp3,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            //dpid3,
            NULL
        );

        // Create vertices

        udpReceiveCfromA.vertexId = xme_hal_graph_addVertex(&nccGraph, &udpReceiveCfromA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveCfromA.vertexId);

        demarshalerCfromA.vertexId = xme_hal_graph_addVertex(&nccGraph, &demarshalerCfromA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerCfromA.vertexId);

        componentC.vertexId = xme_hal_graph_addVertex(&nccGraph, &componentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentC.vertexId);

        // Populate the edge data

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &logicalRouteAtoC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &logicalRouteAtoC,
            topic,
            chan1,
            false
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpSendToUDPReceiveAtoC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
        (
            &udpSendToUDPReceiveAtoC,
            topic,
            udpKey
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerCfromA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &udpReceiveToDemarshalerCfromA,
            topic, 
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentCfromA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &demarshalerToComponentCfromA,
            topic,
            channelID1
        );

        // Create edges

        logicalRouteAtoC.edgeId = xme_hal_graph_addEdge(&nccGraph, componentA.vertexId, componentC.vertexId, &logicalRouteAtoC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, logicalRouteAtoC.edgeId);

        udpSendToUDPReceiveAtoC.edgeId = xme_hal_graph_addEdge(&nccGraph, udpSendA.vertexId, udpReceiveCfromA.vertexId, &udpSendToUDPReceiveAtoC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpSendToUDPReceiveAtoC.edgeId);

        udpReceiveToDemarshalerCfromA.edgeId = xme_hal_graph_addEdge(&nccGraph, udpReceiveCfromA.vertexId, demarshalerCfromA.vertexId, &udpReceiveToDemarshalerCfromA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerCfromA.edgeId);

        demarshalerToComponentCfromA.edgeId = xme_hal_graph_addEdge(&nccGraph, demarshalerCfromA.vertexId, componentC.vertexId, &demarshalerToComponentCfromA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentCfromA.edgeId);


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  GRAPH: Part B                                                                                        ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentB,
            //node2,
            //cont2,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            //dpid2,
            NULL
        );

        // Populate the marshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &marshalerB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData(&marshalerB, 1u);

        // Populate the UPD Send waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpSendB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
        (
            &udpSendB,
            udpKey,
            &addr
        );

        // Create vertices

        componentB.vertexId = xme_hal_graph_addVertex(&nccGraph, &componentB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentB.vertexId);

        marshalerB.vertexId = xme_hal_graph_addVertex(&nccGraph, &marshalerB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, marshalerB.vertexId);

        udpSendB.vertexId = xme_hal_graph_addVertex(&nccGraph, &udpSendB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpSendB.vertexId);

        // Populate the edge data

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToMarshalerB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToMarshalerB,
            topic,
            channelID2
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &marshalerToUDPSendB,
            topic,
            channelID2
        );

        // Create edges

        componentToMarshalerB.edgeId = xme_hal_graph_addEdge(&nccGraph, componentB.vertexId, marshalerB.vertexId, &componentToMarshalerB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToMarshalerB.edgeId);

        marshalerToUDPSendB.edgeId = xme_hal_graph_addEdge(&nccGraph, marshalerB.vertexId, udpSendB.vertexId, &marshalerToUDPSendB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, marshalerToUDPSendB.edgeId);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  GRAPH: Part C-B                                                                                      ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Populate the UPD Receive waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpReceiveCfromB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
        (
            &udpReceiveCfromB,
            udpKey,
            &addr
        );

        // Populate the demarshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &demarshalerCfromB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData(&demarshalerCfromB, 1u);

        // Create vertices

        udpReceiveCfromB.vertexId = xme_hal_graph_addVertex(&nccGraph, &udpReceiveCfromB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveCfromB.vertexId);

        demarshalerCfromB.vertexId = xme_hal_graph_addVertex(&nccGraph, &demarshalerCfromB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerCfromB.vertexId);

        // Populate the edge data

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &logicalRouteBtoC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &logicalRouteBtoC,
            topic,
            chan2,
            false
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpSendToUDPReceiveBtoC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
        (
            &udpSendToUDPReceiveBtoC,
            topic,
            udpKey
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerCfromB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &udpReceiveToDemarshalerCfromB,
            topic,
            channelID2
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentCfromB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &demarshalerToComponentCfromB,
            topic,
            channelID2
        );

        // Create edges

        logicalRouteBtoC.edgeId = xme_hal_graph_addEdge(&nccGraph, componentB.vertexId, componentC.vertexId, &logicalRouteBtoC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, logicalRouteBtoC.edgeId);

        udpSendToUDPReceiveBtoC.edgeId = xme_hal_graph_addEdge(&nccGraph, udpSendB.vertexId, udpReceiveCfromB.vertexId, &udpSendToUDPReceiveBtoC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpSendToUDPReceiveBtoC.edgeId);

        udpReceiveToDemarshalerCfromB.edgeId = xme_hal_graph_addEdge(&nccGraph, udpReceiveCfromB.vertexId, demarshalerCfromB.vertexId, &udpReceiveToDemarshalerCfromB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerCfromB.edgeId);

        demarshalerToComponentCfromB.edgeId = xme_hal_graph_addEdge(&nccGraph, demarshalerCfromB.vertexId, componentC.vertexId, &demarshalerToComponentCfromB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentCfromB.edgeId);

        EXPECT_EQ(11, xme_hal_graph_getVertexCount(&nccGraph));
        EXPECT_EQ(12, xme_hal_graph_getEdgeCount(&nccGraph));
    }

    virtual void AssertionCheckedTearDown()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraph));
        xme_core_pnp_pnpManager_fini();
    }

    static int edgeCompareCallbackMatchLogicalRouteEdges
    (
        void* edgeData1,
        void* edgeData2
    )
    {
        xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) edgeData1;

        XME_ASSERT_RVAL(NULL == edgeData2, 1);

        return (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType) ? 0 : 1;
    }

    static const xme_core_topic_t topic = XME_CORE_TOPIC(42);
    static const xme_core_channelId_t channelID1 = (xme_core_channelId_t) 1U;
    static const xme_core_channelId_t channelID2 = (xme_core_channelId_t) 2U;

    static const xme_core_node_nodeId_t node1 = (xme_core_node_nodeId_t) 1001;
    static const xme_core_node_nodeId_t node2 = (xme_core_node_nodeId_t) 1002;
    static const xme_core_node_nodeId_t node3 = (xme_core_node_nodeId_t) 1003;

    static const xme_core_container_t cont1 = (xme_core_container_t) 101;
    static const xme_core_container_t cont2 = (xme_core_container_t) 102;
    static const xme_core_container_t cont3 = (xme_core_container_t) 103;

    static const xme_core_component_t comp1 = (xme_core_component_t) 11;
    static const xme_core_component_t comp2 = (xme_core_component_t) 11;
    static const xme_core_component_t comp3 = (xme_core_component_t) 11;

    static const xme_core_channelId_t chan1 = (xme_core_channelId_t) 21;
    static const xme_core_channelId_t chan2 = (xme_core_channelId_t) 22;

    static const xme_core_dataManager_dataPacketId_t dpid1 = (xme_core_dataManager_dataPacketId_t) 31;
    static const xme_core_dataManager_dataPacketId_t dpid2 = (xme_core_dataManager_dataPacketId_t) 32;
    static const xme_core_dataManager_dataPacketId_t dpid3 = (xme_core_dataManager_dataPacketId_t) 33;

    xme_core_pnp_ncc_physicalRoutes_t nccGraph;

    xme_com_interface_address_t addr;

    // Component A
    xme_core_pnp_dataLinkGraph_vertexData_t componentA;
    xme_core_pnp_dataLinkGraph_vertexData_t marshalerA;
    xme_core_pnp_dataLinkGraph_vertexData_t udpSendA;

    xme_core_pnp_dataLinkGraph_edgeData_t componentToMarshalerA;
    xme_core_pnp_dataLinkGraph_edgeData_t marshalerToUDPSendA;
    xme_core_pnp_dataLinkGraph_edgeData_t udpSendToUDPReceiveAtoC;

    // Component B
    xme_core_pnp_dataLinkGraph_vertexData_t componentB;
    xme_core_pnp_dataLinkGraph_vertexData_t marshalerB;
    xme_core_pnp_dataLinkGraph_vertexData_t udpSendB;

    xme_core_pnp_dataLinkGraph_edgeData_t componentToMarshalerB;
    xme_core_pnp_dataLinkGraph_edgeData_t marshalerToUDPSendB;
    xme_core_pnp_dataLinkGraph_edgeData_t udpSendToUDPReceiveBtoC;

    // Component C
    xme_core_pnp_dataLinkGraph_vertexData_t udpReceiveCfromA;
    xme_core_pnp_dataLinkGraph_vertexData_t udpReceiveCfromB;
    xme_core_pnp_dataLinkGraph_vertexData_t demarshalerCfromA;
    xme_core_pnp_dataLinkGraph_vertexData_t demarshalerCfromB;
    xme_core_pnp_dataLinkGraph_vertexData_t componentC;

    xme_core_pnp_dataLinkGraph_edgeData_t logicalRouteAtoC;
    xme_core_pnp_dataLinkGraph_edgeData_t logicalRouteBtoC;
    xme_core_pnp_dataLinkGraph_edgeData_t udpReceiveToDemarshalerCfromA;
    xme_core_pnp_dataLinkGraph_edgeData_t udpReceiveToDemarshalerCfromB;
    xme_core_pnp_dataLinkGraph_edgeData_t demarshalerToComponentCfromA;
    xme_core_pnp_dataLinkGraph_edgeData_t demarshalerToComponentCfromB;

    uint8_t udpKey[4];
};

class PnPManagerInterfaceTestBase: public xme::testing::Test
{
protected:
    PnPManagerInterfaceTestBase()
    {
    }

    virtual ~PnPManagerInterfaceTestBase()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

        createComponentTypeManifests();
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_pnp_pnpManager_fini();
        xme_core_nodeMgr_compRep_fini();
        xme_core_pnp_lrm_fini();
        xme_core_manifestRepository_fini();
        xme_core_directory_attribute_fini();
    }

private:
    void
    createComponentTypeManifests(void)
    {
        xme_core_componentManifest_t componentManifest;
    
        // Create and add component type manifest for component 'sensor' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_SENSOR));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    
        // Create and add component type manifest for component 'monitor' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_MONITOR));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }

        // Create and add component type manifest for component 'sensor (B)' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_SENSOR_B));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }

        // Create and add component type manifest for component 'sensor (KB)' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_SENSOR_KB));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }

        // Create and add component type manifest for component '�monitor (KB)' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_MONITOR_KB));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }

    xme_status_t
    createComponentTypeManifest
    (
        xme_core_componentManifest_t* componentManifest,
        xme_core_componentType_t componentType
    )
    {
        uint32_t functionArrayLength;
        uint32_t portArrayLength;
    
        XME_CHECK
        (
            NULL != componentManifest,
            XME_STATUS_INVALID_PARAMETER
        );
    
        // Initialize the structure with all zeros
        xme_hal_mem_set(componentManifest, 0U, sizeof(xme_core_componentManifest_t));

        componentManifest->componentType = componentType;

        functionArrayLength = sizeof(componentManifest->functionManifests) / sizeof(componentManifest->functionManifests[0]);

        // Function 'printSensorValue'
        {
            if (0 >= functionArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == functionArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more functions (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        functionArrayLength
                    );
                }
            }
            else
            {
                xme_core_functionManifest_t* functionManifest;
            
                functionManifest = &componentManifest->functionManifests[0];
                functionManifest->functionId = (xme_core_component_functionId_t)1;
                functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(200);
                functionManifest->alphaCurve.alphaCurve = 0;
                functionManifest->completion = true;
            }
        }

        portArrayLength = sizeof(componentManifest->portManifests) / sizeof(componentManifest->portManifests[0]);

        // Subscription 'sensorValueIn'
        {
            if (0 >= portArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == portArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more ports (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        portArrayLength
                    );
                }
            }
            else
            {
                xme_core_componentPortManifest_t* portManifest;
            
                portManifest = &componentManifest->portManifests[0];
                portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                portManifest->queueSize = 1u;

                switch (componentType)
                {
                    case XME_CORE_COMPONENT_TYPE_MONITOR:
                    case XME_CORE_COMPONENT_TYPE_MONITOR_KB:
                        portManifest->lowerConnectionBound = 0;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                        break;
                    case XME_CORE_COMPONENT_TYPE_SENSOR:
                    case XME_CORE_COMPONENT_TYPE_SENSOR_B:
                    case XME_CORE_COMPONENT_TYPE_SENSOR_KB:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                        break;
                    default:
                        XME_CHECK_MSG(false, XME_STATUS_UNSUPPORTED, XME_LOG_ERROR, "Unhandled component type (0x%08X)!\n", componentType);
                }

                portManifest->topic = XME_CORE_TOPIC(4098);

                switch (componentType)
                {
                    case XME_CORE_COMPONENT_TYPE_MONITOR_KB:
                        portManifest->attrSet = xme_core_directory_attribute_createAttributeSet();
                        XME_CHECK(XME_STATUS_SUCCESS ==
                            xme_core_directory_attribute_addPredefinedAttributeFilter
                            (
                                portManifest->attrSet,
                                (xme_core_attribute_key_t) 1,
                                "KB",
                                1U,
                                0U,
                                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                                false
                            ),
                            XME_STATUS_INTERNAL_ERROR
                        );
                        break;
                    case XME_CORE_COMPONENT_TYPE_SENSOR_B:
                        portManifest->attrSet = xme_core_directory_attribute_createAttributeSet();
                        XME_CHECK(XME_STATUS_SUCCESS ==
                            xme_core_directory_attribute_addPredefinedAttributeDefinition
                            (
                                portManifest->attrSet,
                                (xme_core_attribute_key_t) 1,
                                "B",
                                1U,
                                0U,
                                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                                false
                            ),
                            XME_STATUS_INTERNAL_ERROR
                        );
                        break;
                    case XME_CORE_COMPONENT_TYPE_SENSOR_KB:
                        portManifest->attrSet = xme_core_directory_attribute_createAttributeSet();
                        XME_CHECK(XME_STATUS_SUCCESS ==
                            xme_core_directory_attribute_addPredefinedAttributeDefinition
                            (
                                portManifest->attrSet,
                                (xme_core_attribute_key_t) 1,
                                "KB",
                                1U,
                                0U,
                                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                                false
                            ),
                            XME_STATUS_INTERNAL_ERROR
                        );
                        break;
                    case XME_CORE_COMPONENT_TYPE_MONITOR:
                    case XME_CORE_COMPONENT_TYPE_SENSOR:
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        break;
                    default:
                        XME_ASSERT(false);
                }
            }
        }
    
        return XME_STATUS_SUCCESS;
    }
};

class PnPManagerInterfaceTestTransformSubgraph: public PnPManagerInterfaceTestBase
{
protected:
    PnPManagerInterfaceTestTransformSubgraph()
    {
        udpKey[0] = 1;
        udpKey[1] = 2;
        udpKey[2] = 3;
        udpKey[3] = 4;
    }

    virtual ~PnPManagerInterfaceTestTransformSubgraph()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        PnPManagerInterfaceTestBase::AssertionCheckedSetUp();

        // Builds the following graph in graph descriptor "nccGraphA" (see key below):
        //
        //       MC      MC
        //   cA ---> mA ---> sA

        // Builds the following graph in graph descriptor "nccGraphB" (see key below):
        //
        //       MC
        //   cB ---> cC

        // Builds the following graph in graph descriptor "nccGraphC" (see key below):
        //
        //       MC      MC
        //   rC ---> dC ---> cC

        // Key:
        // cX  - component X
        // mX  - marshaling waypoint for component X
        // sX  - UDP send waypoint for component X
        // rX  - UDP receive waypoint for component X
        // dX  - demarshaling waypoint for component X
        // LR  - logical route edge
        // MC  - memcopy edge

        xme_com_interface_ipv4StringToGenericAddress("192.168.0.1:8080", &addr);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  SUBGRAPH A                                                                                           ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraphA));
        
        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentA,
            //node1,
            //cont1,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            //dpid1,
            NULL
        );

        // Populate the marshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &marshalerA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData(&marshalerA, 1u);

        // Populate the UPD Send waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpSendA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
        (
            &udpSendA,
            udpKey,
            &addr
        );

        componentA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &componentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentA.vertexId);

        marshalerA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &marshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, marshalerA.vertexId);

        udpSendA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &udpSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpSendA.vertexId);

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToMarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToMarshalerA,
            topic,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &marshalerToUDPSendA,
            topic,
            channelID1
        );

        componentToMarshalerA.edgeId = xme_hal_graph_addEdge(&nccGraphA, componentA.vertexId, marshalerA.vertexId, &componentToMarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToMarshalerA.edgeId);

        marshalerToUDPSendA.edgeId = xme_hal_graph_addEdge(&nccGraphA, marshalerA.vertexId, udpSendA.vertexId, &marshalerToUDPSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, marshalerToUDPSendA.edgeId);

        EXPECT_EQ(3, xme_hal_graph_getVertexCount(&nccGraphA));
        EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&nccGraphA));

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  SUBGRAPH B                                                                                           ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraphB));
        
        // Populate the source component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentA,
            //node1,
            //cont1,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            //dpid1,
            NULL
        );

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentB,
            //node1,
            //cont1,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            //dpid2,
            NULL
        );

        componentA.vertexId = xme_hal_graph_addVertex(&nccGraphB, &componentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentA.vertexId);

        componentB.vertexId = xme_hal_graph_addVertex(&nccGraphB, &componentB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentB.vertexId);

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentAToComponentB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentAToComponentB,
            topic,
            channelID2
        );

        componentAToComponentB.edgeId = xme_hal_graph_addEdge(&nccGraphB, componentA.vertexId, componentB.vertexId, &componentAToComponentB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentAToComponentB.edgeId);

        EXPECT_EQ(2, xme_hal_graph_getVertexCount(&nccGraphB));
        EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&nccGraphB));

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  SUBGRAPH C                                                                                           ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraphC));
        
        // Populate the UPD Receive waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpReceiveC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );

        xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
        (
            &udpReceiveC,
            udpKey,
            &addr
        );

        // Populate the demarshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &demarshalerC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );
        xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData(&demarshalerC, 1u);

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node3,
            topic
        );

        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentC,
            //node3,
            //cont3,
            comp3,
            (xme_core_componentType_t)comp3,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            //dpid3,
            NULL
        );

        udpReceiveC.vertexId = xme_hal_graph_addVertex(&nccGraphC, &udpReceiveC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveC.vertexId);

        demarshalerC.vertexId = xme_hal_graph_addVertex(&nccGraphC, &demarshalerC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerC.vertexId);

        componentC.vertexId = xme_hal_graph_addVertex(&nccGraphC, &componentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentC.vertexId);

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &udpReceiveToDemarshalerC,
            topic,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &demarshalerToComponentC,
            topic,
            channelID1
        );

        udpReceiveToDemarshalerC.edgeId = xme_hal_graph_addEdge(&nccGraphC, udpReceiveC.vertexId, demarshalerC.vertexId, &udpReceiveToDemarshalerC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerC.edgeId);

        demarshalerToComponentC.edgeId = xme_hal_graph_addEdge(&nccGraphC, demarshalerC.vertexId, componentC.vertexId, &demarshalerToComponentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentC.edgeId);

        EXPECT_EQ(3, xme_hal_graph_getVertexCount(&nccGraphA));
        EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&nccGraphA));
    }

    virtual void AssertionCheckedTearDown()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphA));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphB));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphC));

        PnPManagerInterfaceTestBase::AssertionCheckedTearDown();
    }

    static const xme_core_topic_t topic = XME_CORE_TOPIC(42);
    static const xme_core_channelId_t channelID1 = (xme_core_channelId_t) 1U;
    static const xme_core_channelId_t channelID2 = (xme_core_channelId_t) 2U;

    static const xme_core_node_nodeId_t node1 = (xme_core_node_nodeId_t) 1001;
    static const xme_core_node_nodeId_t node3 = (xme_core_node_nodeId_t) 1003;

    static const xme_core_container_t cont1 = (xme_core_container_t) 101;
    static const xme_core_container_t cont3 = (xme_core_container_t) 103;

    static const xme_core_component_t comp1 = (xme_core_component_t) 11;
    static const xme_core_component_t comp2 = (xme_core_component_t) 11;
    static const xme_core_component_t comp3 = (xme_core_component_t) 11;

    static const xme_core_dataManager_dataPacketId_t dpid1 = (xme_core_dataManager_dataPacketId_t) 31;
    static const xme_core_dataManager_dataPacketId_t dpid2 = (xme_core_dataManager_dataPacketId_t) 32;
    static const xme_core_dataManager_dataPacketId_t dpid3 = (xme_core_dataManager_dataPacketId_t) 33;

    xme_core_pnp_ncc_physicalRoutes_t nccGraphA; // remote publish route: componentA, marshaler and udp send.
    xme_core_pnp_ncc_physicalRoutes_t nccGraphB; // local publish-subscribe route: componentA - componentB
    xme_core_pnp_ncc_physicalRoutes_t nccGraphC; // remove subscribe route: upd receive, demarshaler, componentC. 

    xme_com_interface_address_t addr;

    // Component A
    xme_core_pnp_dataLinkGraph_vertexData_t componentA;
    xme_core_pnp_dataLinkGraph_vertexData_t marshalerA;
    xme_core_pnp_dataLinkGraph_vertexData_t udpSendA;

    xme_core_pnp_dataLinkGraph_edgeData_t componentToMarshalerA;
    xme_core_pnp_dataLinkGraph_edgeData_t marshalerToUDPSendA;

    // Component B
    xme_core_pnp_dataLinkGraph_vertexData_t componentB;

    // Component C
    xme_core_pnp_dataLinkGraph_vertexData_t componentC;

    uint8_t udpKey[4];

    // subgraph A: direct component memcpy
    xme_core_pnp_dataLinkGraph_edgeData_t componentAToComponentB;

    // subgraph C: separate subgraph for receiving data
    xme_core_pnp_dataLinkGraph_vertexData_t udpReceiveC;
    xme_core_pnp_dataLinkGraph_vertexData_t demarshalerC;

    xme_core_pnp_dataLinkGraph_edgeData_t udpReceiveToDemarshalerC;
    xme_core_pnp_dataLinkGraph_edgeData_t demarshalerToComponentC;

};

class PnPManagerInterfaceTestChannelRRWaypointForTwoClientsAndOneServerTests: public PnPManagerInterfaceTestBase
{
protected:
    PnPManagerInterfaceTestChannelRRWaypointForTwoClientsAndOneServerTests()
    {
        // UDP Key for Requests
        udpKeyRq[0] = 1;
        udpKeyRq[1] = 2;
        udpKeyRq[2] = 3;
        udpKeyRq[3] = 4;

        // UDP Key for Responses
        udpKeyRs[0] = 5;
        udpKeyRs[1] = 6;
        udpKeyRs[2] = 7;
        udpKeyRs[3] = 8;
    }

    virtual ~PnPManagerInterfaceTestChannelRRWaypointForTwoClientsAndOneServerTests()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        PnPManagerInterfaceTestBase::AssertionCheckedSetUp();

        // Builds the following graph in graph descriptor "nccGraphA" (see key below):
        //
        //       MC       MC      MC
        //   cA ---> ciA ---> mA ---> sA (request)
        //     A
        //     \------- dA <--------- rA (response)
        //           MC          MC

        // Builds the following graph in graph descriptor "nccGraphBC" (see key below):
        //
        //       MC      MC        MC       MC      MC
        //   rC ---> dC ----.    .---> csC ---> mC ---> sC
        // (remote request) _\| /    (remote response)
        //                   _cC
        //       MC      MC  /| \  MC       MC
        //   cB ---> ciB ---'    '---> csC ---> cB
        //    (local request)        (local response)
        //
        // Key:
        // cX  - component X
        // ciX  - channel injector waypoint for component X
        // mX  - marshaling waypoint for component X
        // sX  - UDP send waypoint for component X
        // rX  - UDP receive waypoint for component X
        // dX  - demarshaling waypoint for component X
        // csX  - channel selector waypoint for component X
        // LR  - logical route edge
        // MC  - memcopy edge

        xme_com_interface_ipv4StringToGenericAddress("192.168.0.1:8080", &addr);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  SUBGRAPH A                                                                                           ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraphA));
        
        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentA,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the channel injector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &injectorA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelInjectorVertexData
        (
            &injectorA,
            (xme_core_channelId_t) 1,
            1u
        );

        // Populate the marshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &marshalerA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData(&marshalerA, 1u);

        // Populate the UPD Send waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpSendA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
        (
            &udpSendA,
            udpKeyRq,
            &addr
        );

        // Populate the UPD Receive waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpReceiveA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRs
        );

        xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
        (
            &udpReceiveA,
            udpKeyRs,
            &addr
        );

        // Populate the demarshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &demarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData(&demarshalerA, 1u);

        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentA,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        componentA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &componentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentA.vertexId);

        injectorA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &injectorA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, injectorA.vertexId);

        marshalerA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &marshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, marshalerA.vertexId);

        udpSendA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &udpSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpSendA.vertexId);

        udpReceiveA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &udpReceiveA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveA.vertexId);

        demarshalerA.vertexId = xme_hal_graph_addVertex(&nccGraphA, &demarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerA.vertexId);

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToInjectorA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToInjectorA,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &injectorToMarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &injectorToMarshalerA,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &marshalerToUDPSendA,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &udpReceiveToDemarshalerA,
            topicRs,
            channelID3
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &demarshalerToComponentA,
            topicRs,
            channelID3
        );

        componentToInjectorA.edgeId = xme_hal_graph_addEdge(&nccGraphA, componentA.vertexId, injectorA.vertexId, &componentToInjectorA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToInjectorA.edgeId);

        injectorToMarshalerA.edgeId = xme_hal_graph_addEdge(&nccGraphA, injectorA.vertexId, marshalerA.vertexId, &injectorToMarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, injectorToMarshalerA.edgeId);

        marshalerToUDPSendA.edgeId = xme_hal_graph_addEdge(&nccGraphA, marshalerA.vertexId, udpSendA.vertexId, &marshalerToUDPSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, marshalerToUDPSendA.edgeId);

        udpReceiveToDemarshalerA.edgeId = xme_hal_graph_addEdge(&nccGraphA, udpReceiveA.vertexId, demarshalerA.vertexId, &udpReceiveToDemarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerA.edgeId);

        demarshalerToComponentA.edgeId = xme_hal_graph_addEdge(&nccGraphA, demarshalerA.vertexId, componentA.vertexId, &demarshalerToComponentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentA.edgeId);

        EXPECT_EQ(6, xme_hal_graph_getVertexCount(&nccGraphA));
        EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&nccGraphA));

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  SUBGRAPH C                                                                                           ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraphBC));
        
        // Populate the UPD Receive waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpReceiveC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
        (
            &udpReceiveC,
            udpKeyRq,
            &addr
        );

        // Populate the demarshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &demarshalerC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData(&demarshalerC, 1u);

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentC,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentC,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            NULL
        );

        // Populate the channel selector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &selectorCA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelSelectorVertexData
        (
            &selectorCA,
            (xme_core_channelId_t) 1,
            (xme_core_channelId_t) 3,
            1u
        );

        // Populate the marshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &marshalerC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData(&marshalerC, 1u);

        // Populate the UPD Send waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpSendC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
        (
            &udpSendC,
            udpKeyRs,
            &addr
        );


        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentB,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the channel injector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &injectorB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelInjectorVertexData
        (
            &injectorB,
            (xme_core_channelId_t) 2,
            1u
        );

        // Populate the channel selector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &selectorB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelSelectorVertexData
        (
            &selectorB,
            (xme_core_channelId_t) 2,
            (xme_core_channelId_t) 4,
            1u
        );

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentB,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        udpReceiveC.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &udpReceiveC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveC.vertexId);

        demarshalerC.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &demarshalerC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerC.vertexId);

        componentC.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &componentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentC.vertexId);

        selectorCA.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &selectorCA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, selectorCA.vertexId);

        marshalerC.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &marshalerC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, marshalerC.vertexId);

        udpSendC.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &udpSendC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpSendC.vertexId);

        componentB.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &componentB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentB.vertexId);

        injectorB.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &injectorB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, injectorB.vertexId);

        selectorB.vertexId = xme_hal_graph_addVertex(&nccGraphBC, &selectorB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, selectorB.vertexId);

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &udpReceiveToDemarshalerC,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &demarshalerToComponentC,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToSelectorCA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToSelectorCA,
            topicRs,
            channelID3
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &selectorToMarshalerC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &selectorToMarshalerC,
            topicRs,
            channelID3
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &marshalerToUDPSendC,
            topicRs,
            channelID3
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToInjectorB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToInjectorB,
            topicRq,
            channelID2
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &injectorToComponentBC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &injectorToComponentBC,
            topicRq,
            channelID2
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToSelectorB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToSelectorB,
            topicRs,
            channelID4
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &selectorToComponentB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &selectorToComponentB,
            topicRs,
            channelID4
        );

        udpReceiveToDemarshalerC.edgeId = xme_hal_graph_addEdge(&nccGraphBC, udpReceiveC.vertexId, demarshalerC.vertexId, &udpReceiveToDemarshalerC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerC.edgeId);

        demarshalerToComponentC.edgeId = xme_hal_graph_addEdge(&nccGraphBC, demarshalerC.vertexId, componentC.vertexId, &demarshalerToComponentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentC.edgeId);

        componentToSelectorCA.edgeId = xme_hal_graph_addEdge(&nccGraphBC, componentC.vertexId, selectorCA.vertexId, &componentToSelectorCA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToSelectorCA.edgeId);

        selectorToMarshalerC.edgeId = xme_hal_graph_addEdge(&nccGraphBC, selectorCA.vertexId, marshalerC.vertexId, &selectorToMarshalerC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, selectorToMarshalerC.edgeId);

        marshalerToUDPSendC.edgeId = xme_hal_graph_addEdge(&nccGraphBC, marshalerC.vertexId, udpSendC.vertexId, &marshalerToUDPSendC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, marshalerToUDPSendC.edgeId);

        componentToInjectorB.edgeId = xme_hal_graph_addEdge(&nccGraphBC, componentB.vertexId, injectorB.vertexId, &componentToInjectorB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToInjectorB.edgeId);

        injectorToComponentBC.edgeId = xme_hal_graph_addEdge(&nccGraphBC, injectorB.vertexId, componentC.vertexId, &injectorToComponentBC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, injectorToComponentBC.edgeId);

        componentToSelectorB.edgeId = xme_hal_graph_addEdge(&nccGraphBC, componentC.vertexId, selectorB.vertexId, &componentToSelectorB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToSelectorB.edgeId);

        selectorToComponentB.edgeId = xme_hal_graph_addEdge(&nccGraphBC, selectorB.vertexId, componentB.vertexId, &selectorToComponentB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, selectorToComponentB.edgeId);

        EXPECT_EQ(9, xme_hal_graph_getVertexCount(&nccGraphBC));
        EXPECT_EQ(9, xme_hal_graph_getEdgeCount(&nccGraphBC));
    }

    virtual void AssertionCheckedTearDown()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphA));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphBC));

        PnPManagerInterfaceTestBase::AssertionCheckedTearDown();
    }

    static const xme_core_topic_t topicRq = XME_CORE_TOPIC(42);
    static const xme_core_topic_t topicRs = XME_CORE_TOPIC(43);
    static const xme_core_channelId_t channelID1 = (xme_core_channelId_t) 1U;
    static const xme_core_channelId_t channelID2 = (xme_core_channelId_t) 2U;
    static const xme_core_channelId_t channelID3 = (xme_core_channelId_t) 3U;
    static const xme_core_channelId_t channelID4 = (xme_core_channelId_t) 4U;

    static const xme_core_node_nodeId_t node1 = (xme_core_node_nodeId_t) 1001;
    static const xme_core_node_nodeId_t node2 = (xme_core_node_nodeId_t) 1002;

    static const xme_core_component_t comp1 = (xme_core_component_t) 11;
    static const xme_core_component_t comp2 = (xme_core_component_t) 12;

    static const xme_core_dataManager_dataPacketId_t dpid1 = (xme_core_dataManager_dataPacketId_t) 31;
    static const xme_core_dataManager_dataPacketId_t dpid2 = (xme_core_dataManager_dataPacketId_t) 32;
    static const xme_core_dataManager_dataPacketId_t dpid3 = (xme_core_dataManager_dataPacketId_t) 33;

    xme_core_pnp_ncc_physicalRoutes_t nccGraphA; // remote publish route: componentA, marshaler and udp send.
    xme_core_pnp_ncc_physicalRoutes_t nccGraphBC; // local publish-subscribe route: componentA - componentB

    xme_com_interface_address_t addr;

    // Component A
    xme_core_pnp_dataLinkGraph_vertexData_t componentA;
    xme_core_pnp_dataLinkGraph_vertexData_t injectorA;
    xme_core_pnp_dataLinkGraph_vertexData_t marshalerA;
    xme_core_pnp_dataLinkGraph_vertexData_t udpSendA;
    xme_core_pnp_dataLinkGraph_vertexData_t udpReceiveA;
    xme_core_pnp_dataLinkGraph_vertexData_t demarshalerA;

    xme_core_pnp_dataLinkGraph_edgeData_t componentToInjectorA;
    xme_core_pnp_dataLinkGraph_edgeData_t injectorToMarshalerA;
    xme_core_pnp_dataLinkGraph_edgeData_t marshalerToUDPSendA;
    xme_core_pnp_dataLinkGraph_edgeData_t udpReceiveToDemarshalerA;
    xme_core_pnp_dataLinkGraph_edgeData_t demarshalerToComponentA;

    // Component B
    xme_core_pnp_dataLinkGraph_vertexData_t componentB;
    xme_core_pnp_dataLinkGraph_vertexData_t injectorB;
    xme_core_pnp_dataLinkGraph_vertexData_t selectorB;

    // Edges for local request/response.
    xme_core_pnp_dataLinkGraph_edgeData_t componentToInjectorB;
    xme_core_pnp_dataLinkGraph_edgeData_t injectorToComponentBC;
    xme_core_pnp_dataLinkGraph_edgeData_t componentToSelectorB;
    xme_core_pnp_dataLinkGraph_edgeData_t selectorToComponentB;

    // Component C
    xme_core_pnp_dataLinkGraph_vertexData_t componentC;
    xme_core_pnp_dataLinkGraph_vertexData_t udpReceiveC;
    xme_core_pnp_dataLinkGraph_vertexData_t demarshalerC;
    xme_core_pnp_dataLinkGraph_vertexData_t selectorCA;
    xme_core_pnp_dataLinkGraph_vertexData_t marshalerC;
    xme_core_pnp_dataLinkGraph_vertexData_t udpSendC;
    
    // Edges for remote request/response.
    xme_core_pnp_dataLinkGraph_edgeData_t udpReceiveToDemarshalerC;
    xme_core_pnp_dataLinkGraph_edgeData_t demarshalerToComponentC;
    xme_core_pnp_dataLinkGraph_edgeData_t componentToSelectorCA;
    xme_core_pnp_dataLinkGraph_edgeData_t selectorToMarshalerC;
    xme_core_pnp_dataLinkGraph_edgeData_t marshalerToUDPSendC;

    uint8_t udpKeyRq[4];
    uint8_t udpKeyRs[4];
};

class PnPManagerInterfaceTestChannelRRWaypointForOneClientAndTwoServersTests: public PnPManagerInterfaceTestBase
{
protected:
    PnPManagerInterfaceTestChannelRRWaypointForOneClientAndTwoServersTests()
    {
        // UDP Key for Requests
        udpKeyRq[0] = 1;
        udpKeyRq[1] = 2;
        udpKeyRq[2] = 3;
        udpKeyRq[3] = 4;

        // UDP Key for Responses
        udpKeyRs[0] = 5;
        udpKeyRs[1] = 6;
        udpKeyRs[2] = 7;
        udpKeyRs[3] = 8;
    }

    virtual ~PnPManagerInterfaceTestChannelRRWaypointForOneClientAndTwoServersTests()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        PnPManagerInterfaceTestBase::AssertionCheckedSetUp();

        // Builds the following graph in graph descriptor "nccGraphAC" (see key below):
        //
        //
        //                         MC        MC      MC
        //                        .--> ciAB ---> mA ---> sA
        //                       /     (remote request)
        //                      / MC      MC
        //                   cA <--- dA <--- rA
        // (local response) /   \.
        //                csC    ciAC
        //                  \    /  (local request)
        //                    cC

        // Builds the following graph in graph descriptor "nccGraphB" (see key below):
        //
        //
        //       MC       MC      MC
        //   cB ---> csB ---> dB ---> rB (remote request)
        //     A
        //     \------- dB <--------- rB (remote request)
        //           MC          MC
        //
        // Key:
        // cX  - component X
        // ciX  - channel injector waypoint for component X
        // mX  - marshaling waypoint for component X
        // sX  - UDP send waypoint for component X
        // rX  - UDP receive waypoint for component X
        // dX  - demarshaling waypoint for component X
        // csX  - channel selector waypoint for component X
        // LR  - logical route edge
        // MC  - memcopy edge

        xme_com_interface_ipv4StringToGenericAddress("192.168.0.1:8080", &addr);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  SUBGRAPH A & B                                                                                           ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraphAC));
        
        // Populate the component data
        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentA,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the channel injector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &injectorAB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelInjectorVertexData
        (
            &injectorAB,
            (xme_core_channelId_t) 1,
            1u
        );

        // Populate the marshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &marshalerA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData(&marshalerA, 1u);

        // Populate the UPD Send waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpSendA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
        (
            &udpSendA,
            udpKeyRq,
            &addr
        );

        // Populate the UPD Receive waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpReceiveA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRs
        );

        xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
        (
            &udpReceiveA,
            udpKeyRs,
            &addr
        );

        // Populate the demarshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &demarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData(&demarshalerA, 1u);

        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentA,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentA,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the channel injector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &injectorAC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelInjectorVertexData
        (
            &injectorAC,
            (xme_core_channelId_t) 2,
            1u
        );

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentC,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node1,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentC,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            1U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the channel selector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &selectorC,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelSelectorVertexData
        (
            &selectorC,
            (xme_core_channelId_t) 2,
            (xme_core_channelId_t) 4,
            1u
        );


        componentA.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &componentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentA.vertexId);

        injectorAB.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &injectorAB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, injectorAB.vertexId);

        marshalerA.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &marshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, marshalerA.vertexId);

        udpSendA.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &udpSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpSendA.vertexId);

        udpReceiveA.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &udpReceiveA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveA.vertexId);

        demarshalerA.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &demarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerA.vertexId);

        injectorAC.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &injectorAC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, injectorAC.vertexId);

        componentC.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &componentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentC.vertexId);

        selectorC.vertexId = xme_hal_graph_addVertex(&nccGraphAC, &selectorC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, selectorC.vertexId);

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToInjectorAB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToInjectorAB,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &injectorToMarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &injectorToMarshalerA,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &marshalerToUDPSendA,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &udpReceiveToDemarshalerA,
            topicRs,
            channelID3
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &demarshalerToComponentA,
            topicRs,
            channelID3
        );

        // Local request/response (A->C)
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToInjectorAC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToInjectorAC,
            topicRq,
            channelID2
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &injectorToComponentC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &injectorToComponentC,
            topicRq,
            channelID2
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToSelectorC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToSelectorC,
            topicRs,
            channelID4
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &selectorToComponentA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &selectorToComponentA,
            topicRs,
            channelID4
        );

        // Remote request/response edges
        componentToInjectorAB.edgeId = xme_hal_graph_addEdge(&nccGraphAC, componentA.vertexId, injectorAB.vertexId, &componentToInjectorAB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToInjectorAB.edgeId);

        injectorToMarshalerA.edgeId = xme_hal_graph_addEdge(&nccGraphAC, injectorAB.vertexId, marshalerA.vertexId, &injectorToMarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, injectorToMarshalerA.edgeId);

        marshalerToUDPSendA.edgeId = xme_hal_graph_addEdge(&nccGraphAC, marshalerA.vertexId, udpSendA.vertexId, &marshalerToUDPSendA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, marshalerToUDPSendA.edgeId);

        udpReceiveToDemarshalerA.edgeId = xme_hal_graph_addEdge(&nccGraphAC, udpReceiveA.vertexId, demarshalerA.vertexId, &udpReceiveToDemarshalerA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerA.edgeId);

        demarshalerToComponentA.edgeId = xme_hal_graph_addEdge(&nccGraphAC, demarshalerA.vertexId, componentA.vertexId, &demarshalerToComponentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentA.edgeId);

        // Local request/response edges
        componentToInjectorAC.edgeId = xme_hal_graph_addEdge(&nccGraphAC, componentA.vertexId, injectorAC.vertexId, &componentToInjectorAC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToInjectorAC.edgeId);

        injectorToComponentC.edgeId = xme_hal_graph_addEdge(&nccGraphAC, injectorAC.vertexId, componentC.vertexId, &injectorToComponentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, injectorToComponentC.edgeId);

        componentToSelectorC.edgeId = xme_hal_graph_addEdge(&nccGraphAC, componentC.vertexId, selectorC.vertexId, &componentToSelectorC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToSelectorC.edgeId);

        selectorToComponentA.edgeId = xme_hal_graph_addEdge(&nccGraphAC, selectorC.vertexId, componentA.vertexId, &selectorToComponentA);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, selectorToComponentA.edgeId);

        EXPECT_EQ(9, xme_hal_graph_getVertexCount(&nccGraphAC));
        EXPECT_EQ(9, xme_hal_graph_getEdgeCount(&nccGraphAC));

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///  SUBGRAPH B                                                                                           ///
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Initialize the graph
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&nccGraphB));
        
        // Populate the UPD Receive waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpReceiveB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
        (
            &udpReceiveB,
            udpKeyRq,
            &addr
        );

        // Populate the demarshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &demarshalerB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );
        xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData(&demarshalerB, 1u);

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRq
        );

        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentB,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL
        );

        // Populate the component data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &componentB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &componentB,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            NULL
        );

        // Populate the channel selector waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &selectorB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );

        xme_core_pnp_dataLinkGraph_initWaypointChannelSelectorVertexData
        (
            &selectorB,
            (xme_core_channelId_t) 1,
            (xme_core_channelId_t) 3,
            1u
        );

        // Populate the marshaler waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &marshalerB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData(&marshalerB, 1u);

        // Populate the UPD Send waypoint data
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &udpSendB,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
            XME_HAL_GRAPH_INVALID_VERTEX_ID,
            node2,
            topicRs
        );
        xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
        (
            &udpSendB,
            udpKeyRs,
            &addr
        );

        udpReceiveB.vertexId = xme_hal_graph_addVertex(&nccGraphB, &udpReceiveB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveB.vertexId);

        demarshalerB.vertexId = xme_hal_graph_addVertex(&nccGraphB, &demarshalerB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerB.vertexId);

        componentB.vertexId = xme_hal_graph_addVertex(&nccGraphB, &componentB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentB.vertexId);

        selectorB.vertexId = xme_hal_graph_addVertex(&nccGraphB, &selectorB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, selectorB.vertexId);

        marshalerB.vertexId = xme_hal_graph_addVertex(&nccGraphB, &marshalerB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, marshalerB.vertexId);

        udpSendB.vertexId = xme_hal_graph_addVertex(&nccGraphB, &udpSendB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpSendB.vertexId);

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &udpReceiveToDemarshalerB,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &demarshalerToComponentB,
            topicRq,
            channelID1
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &componentToSelectorB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &componentToSelectorB,
            topicRs,
            channelID3
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &selectorToMarshalerB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &selectorToMarshalerB,
            topicRs,
            channelID3
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID
        );
        xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
        (
            &marshalerToUDPSendB,
            topicRs,
            channelID3
        );

        udpReceiveToDemarshalerB.edgeId = xme_hal_graph_addEdge(&nccGraphB, udpReceiveB.vertexId, demarshalerB.vertexId, &udpReceiveToDemarshalerB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerB.edgeId);

        demarshalerToComponentB.edgeId = xme_hal_graph_addEdge(&nccGraphB, demarshalerB.vertexId, componentB.vertexId, &demarshalerToComponentB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentB.edgeId);

        componentToSelectorB.edgeId = xme_hal_graph_addEdge(&nccGraphB, componentB.vertexId, selectorB.vertexId, &componentToSelectorB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, componentToSelectorB.edgeId);

        selectorToMarshalerB.edgeId = xme_hal_graph_addEdge(&nccGraphB, selectorB.vertexId, marshalerB.vertexId, &selectorToMarshalerB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, selectorToMarshalerB.edgeId);

        marshalerToUDPSendB.edgeId = xme_hal_graph_addEdge(&nccGraphB, marshalerB.vertexId, udpSendB.vertexId, &marshalerToUDPSendB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, marshalerToUDPSendB.edgeId);

        EXPECT_EQ(6, xme_hal_graph_getVertexCount(&nccGraphB));
        EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&nccGraphB));
    }

    virtual void AssertionCheckedTearDown()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphAC));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphB));

        PnPManagerInterfaceTestBase::AssertionCheckedTearDown();
    }

    static const xme_core_topic_t topicRq = XME_CORE_TOPIC(42);
    static const xme_core_topic_t topicRs = XME_CORE_TOPIC(43);
    static const xme_core_channelId_t channelID1 = (xme_core_channelId_t) 1U;
    static const xme_core_channelId_t channelID2 = (xme_core_channelId_t) 2U;
    static const xme_core_channelId_t channelID3 = (xme_core_channelId_t) 3U;
    static const xme_core_channelId_t channelID4 = (xme_core_channelId_t) 4U;

    static const xme_core_node_nodeId_t node1 = (xme_core_node_nodeId_t) 1001;
    static const xme_core_node_nodeId_t node2 = (xme_core_node_nodeId_t) 1002;

    static const xme_core_component_t comp1 = (xme_core_component_t) 11;
    static const xme_core_component_t comp2 = (xme_core_component_t) 12;

    static const xme_core_dataManager_dataPacketId_t dpid1 = (xme_core_dataManager_dataPacketId_t) 31;
    static const xme_core_dataManager_dataPacketId_t dpid2 = (xme_core_dataManager_dataPacketId_t) 32;
    static const xme_core_dataManager_dataPacketId_t dpid3 = (xme_core_dataManager_dataPacketId_t) 33;

    xme_core_pnp_ncc_physicalRoutes_t nccGraphAC; // remote publish route: componentA, marshaler and udp send.
    xme_core_pnp_ncc_physicalRoutes_t nccGraphB; // local publish-subscribe route: componentA - componentB

    xme_com_interface_address_t addr;

    // Component A (remote request/response)
    xme_core_pnp_dataLinkGraph_vertexData_t componentA;
    xme_core_pnp_dataLinkGraph_vertexData_t injectorAB;
    xme_core_pnp_dataLinkGraph_vertexData_t marshalerA;
    xme_core_pnp_dataLinkGraph_vertexData_t udpSendA;
    xme_core_pnp_dataLinkGraph_vertexData_t udpReceiveA;
    xme_core_pnp_dataLinkGraph_vertexData_t demarshalerA;

    xme_core_pnp_dataLinkGraph_edgeData_t componentToInjectorAB;
    xme_core_pnp_dataLinkGraph_edgeData_t injectorToMarshalerA;
    xme_core_pnp_dataLinkGraph_edgeData_t marshalerToUDPSendA;
    xme_core_pnp_dataLinkGraph_edgeData_t udpReceiveToDemarshalerA;
    xme_core_pnp_dataLinkGraph_edgeData_t demarshalerToComponentA;

    // Component B
    xme_core_pnp_dataLinkGraph_vertexData_t componentB;
    xme_core_pnp_dataLinkGraph_vertexData_t udpReceiveB;
    xme_core_pnp_dataLinkGraph_vertexData_t demarshalerB;
    xme_core_pnp_dataLinkGraph_vertexData_t selectorB;
    xme_core_pnp_dataLinkGraph_vertexData_t marshalerB;
    xme_core_pnp_dataLinkGraph_vertexData_t udpSendB;
    
    // Edges for remote request/response.
    xme_core_pnp_dataLinkGraph_edgeData_t udpReceiveToDemarshalerB;
    xme_core_pnp_dataLinkGraph_edgeData_t demarshalerToComponentB;
    xme_core_pnp_dataLinkGraph_edgeData_t componentToSelectorB;
    xme_core_pnp_dataLinkGraph_edgeData_t selectorToMarshalerB;
    xme_core_pnp_dataLinkGraph_edgeData_t marshalerToUDPSendB;

    // Component C (local request/response from/to A)
    xme_core_pnp_dataLinkGraph_vertexData_t injectorAC;
    xme_core_pnp_dataLinkGraph_vertexData_t componentC;
    xme_core_pnp_dataLinkGraph_vertexData_t selectorC;

    // Edges for local request/response.
    xme_core_pnp_dataLinkGraph_edgeData_t componentToInjectorAC;
    xme_core_pnp_dataLinkGraph_edgeData_t injectorToComponentC;
    xme_core_pnp_dataLinkGraph_edgeData_t componentToSelectorC;
    xme_core_pnp_dataLinkGraph_edgeData_t selectorToComponentA;

    uint8_t udpKeyRq[4];
    uint8_t udpKeyRs[4];
};

class PnPManagerInterfaceTestLoginTest: public xme::testing::Test
{
protected:
    PnPManagerInterfaceTestLoginTest()
    {
    }

    virtual ~PnPManagerInterfaceTestLoginTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        xme_core_pnp_pnpManager_init(NULL);
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_pnp_pnpManager_fini();
    }
};

class PnPManagerInterfaceTestEstablishingLogicalRouteTests: public PnPManagerInterfaceTestBase
{
protected:
    PnPManagerInterfaceTestEstablishingLogicalRouteTests()
    : localNodeId((xme_core_node_nodeId_t)1)
    , nodeId2((xme_core_node_nodeId_t)2)
    , nodeId3((xme_core_node_nodeId_t)3)
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        PnPManagerInterfaceTestBase::AssertionCheckedSetUp();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(localNodeId, "localNodeId", (xme_core_node_guid_t)1));
        {
            xme_com_interface_address_t nodeInterface;

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &nodeInterface));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(localNodeId, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(localNodeId));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId2, "nodeId2", (xme_core_node_guid_t)2));
        {
            xme_com_interface_address_t nodeInterface;

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33222", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId2, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId2));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId3, "nodeId3", (xme_core_node_guid_t)3));
        {
            xme_com_interface_address_t nodeInterface;

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33223", &nodeInterface));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId3, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId3));
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_directory_nodeRegistryController_fini();
        
        PnPManagerInterfaceTestBase::AssertionCheckedTearDown();
    }

    xme_core_node_nodeId_t localNodeId;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeId3;
};

void
createComponentManifestForPnPManagerTest(void);

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     PnPManagerInterfaceTestSplitGraph                                      //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerInterfaceTestSplitGraph, splitGraph)
{
    xme_core_pnp_pnpManager_physicalGraphList_t graphList;

    XME_HAL_SINGLYLINKEDLIST_INIT(graphList);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_splitGraph(&nccGraph, &graphList));

    EXPECT_EQ(3, xme_hal_singlyLinkedList_getItemCount(&graphList));

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(graphList, xme_core_pnp_pnpManager_physicalGraphListItem_t, graphListItem);
    {
        ASSERT_TRUE(NULL != graphListItem);
        EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, graphListItem->nodeId);

        // Register vertex and edge comparison callback functions
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(graphListItem->splitGraph, &xme_core_pnp_dataLinkGraph_vertexCompareIdenticalVertexData));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(graphListItem->splitGraph, &xme_core_pnp_dataLinkGraph_edgeCompareIdenticalEdgeData));

        switch ((xme_maxSystemValue_t)graphListItem->nodeId)
        {
            case (xme_maxSystemValue_t)node1:
            {
                xme_hal_graph_vertexId_t vi1, vi2, vi3;
                xme_hal_graph_edgeId_t ei1, ei2;

                EXPECT_EQ(3, xme_hal_graph_getVertexCount(graphListItem->splitGraph));
                EXPECT_EQ(2, xme_hal_graph_getEdgeCount(graphListItem->splitGraph));

                // Find componentA vertex
                vi1 = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &componentA, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

                // Find marshalerA vertex
                vi2 = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &marshalerA, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

                // Find udpSendA vertex
                vi3 = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &udpSendA, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

                // Find componentToMarshalerA edge
                ei1 = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi1, vi2, &componentToMarshalerA, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

                // Find marshalerToUDPSendA edge
                ei2 = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi2, vi3, &marshalerToUDPSendA, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

                break;
            }

            case (xme_maxSystemValue_t)node2:
            {
                xme_hal_graph_vertexId_t vi1, vi2, vi3;
                xme_hal_graph_edgeId_t ei1, ei2;

                EXPECT_EQ(3, xme_hal_graph_getVertexCount(graphListItem->splitGraph));
                EXPECT_EQ(2, xme_hal_graph_getEdgeCount(graphListItem->splitGraph));

                // Find componentB vertex
                vi1 = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &componentB, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

                // Find marshalerB vertex
                vi2 = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &marshalerB, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

                // Find udpSendB vertex
                vi3 = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &udpSendB, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

                // Find componentToMarshalerB edge
                ei1 = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi1, vi2, &componentToMarshalerB, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

                // Find marshalerToUDPSendB edge
                ei2 = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi2, vi3, &marshalerToUDPSendB, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

                break;
            }

            case (xme_maxSystemValue_t)node3:
            {
                xme_hal_graph_vertexId_t vi1a, vi1b, vi2a, vi2b, vi3;
                xme_hal_graph_edgeId_t ei1a, ei1b, ei2a, ei2b;

                EXPECT_EQ(5, xme_hal_graph_getVertexCount(graphListItem->splitGraph));
                EXPECT_EQ(4, xme_hal_graph_getEdgeCount(graphListItem->splitGraph));

                // Find udpReceiveCfromA vertex
                vi1a = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &udpReceiveCfromA, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1a);

                // Find demarshalerCfromA vertex
                vi1b = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &demarshalerCfromA, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1b);

                // Find udpReceiveCfromB vertex
                vi2a = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &udpReceiveCfromB, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2a);

                // Find demarshalerCfromB vertex
                vi2b = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &demarshalerCfromB, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                // Since demarshaler waypoints are currently not distinguishable w.r.t. their data, we have to ensure that we pick two disjoint vertices
                if (vi1b == vi2b) vi2b = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &demarshalerCfromB, vi2b);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2b);

                // Find componentC vertex
                vi3 = xme_hal_graph_getNextVertexWithDataComparison(graphListItem->splitGraph, &componentC, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

                // Find udpReceiveToDemarshalerCfromA edge
                ei1a = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi1a, vi1b, &udpReceiveToDemarshalerCfromA, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1a);

                // Find demarshalerToComponentCfromA edge
                ei1b = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi1b, vi3, &demarshalerToComponentCfromA, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1b);

                // Find udpReceiveToDemarshalerCfromB edge
                ei2a = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi2a, vi2b, &udpReceiveToDemarshalerCfromB, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2a);

                // Find demarshalerToComponentCfromB edge
                ei2b = xme_hal_graph_getNextEdgeBetweenWithDataComparison(graphListItem->splitGraph, vi2b, vi3, &demarshalerToComponentCfromB, XME_HAL_GRAPH_INVALID_EDGE_ID);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2b);

                break;
            }

            default:
            {
                // Error
                EXPECT_TRUE(false);
            }
        }

        // Common tests
        {
            xme_hal_graph_edgeId_t ei;

            // Look for logical link edges; there should be none in the graph
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(graphListItem->splitGraph, &edgeCompareCallbackMatchLogicalRouteEdges));
            ei = xme_hal_graph_getNextEdgeWithDataComparison(graphListItem->splitGraph, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID);
            EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, ei);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    xme_hal_singlyLinkedList_fini(&graphList);
}

//----------------------------------------------------------------------------//
//     PnPManagerInterfaceTestTransformSubgraph                               //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerInterfaceTestTransformSubgraph, processGraphWithRemotePublisherSubgraph)
{
    xme_core_nodeMgr_compRep_componentHandle_t comp1Handle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_topic_pnpManager_runtime_graph_model_t runtimeGraph;

    builder = xme_core_nodeMgr_compRep_createBuilder(node1, XME_CORE_COMPONENT_TYPE_SENSOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 0u, 11u);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, xme_hal_time_timeIntervalFromMilliseconds(21ull));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &comp1Handle));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphA, node1, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD));

    // Test resulting runtime graph
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(&runtimeGraph));
    EXPECT_EQ(11u, runtimeGraph.vertex[0].portData[0].queueSize);
    EXPECT_EQ(xme_hal_time_timeIntervalFromMilliseconds(21ull), runtimeGraph.vertex[0].functionData[0].executionPeriod);
    // TODO: Test resulting runtime graph more
}

TEST_F(PnPManagerInterfaceTestTransformSubgraph, processGraphWithLocalMemCopySubgraph)
{
    xme_core_nodeMgr_compRep_componentHandle_t comp1Handle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    builder = xme_core_nodeMgr_compRep_createBuilder(node1, XME_CORE_COMPONENT_TYPE_SENSOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &comp1Handle));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphB, node1, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD));
    // TODO: Test resulting data structure
}

TEST_F(PnPManagerInterfaceTestTransformSubgraph, processGraphWithRemoveSubscriberSubgraph)
{
    xme_core_nodeMgr_compRep_componentHandle_t comp1Handle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    builder = xme_core_nodeMgr_compRep_createBuilder(node3, XME_CORE_COMPONENT_TYPE_SENSOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &comp1Handle));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphC, node3, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD));
    // TODO: Test resulting data structure
}

TEST_F(PnPManagerInterfaceTestTransformSubgraph, announcePortsWithNullParameters)
{
    xme_core_componentManifest_t* componentManifest;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_CORE_COMPONENT_TYPE_INVALID));

    componentManifest = (xme_core_componentManifest_t*)xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_findComponentManifest(XME_CORE_COMPONENT_TYPE_SENSOR, componentManifest));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_CORE_COMPONENT_TYPE_INVALID));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(XME_CORE_NODE_INVALID_NODE_ID, comp1, (xme_core_componentType_t)comp1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(node1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_CORE_COMPONENT_TYPE_INVALID));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(node1, comp1, XME_CORE_COMPONENT_TYPE_INVALID));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(node1, XME_CORE_COMPONENT_TYPE_INVALID, (xme_core_componentType_t)comp1));
}

//----------------------------------------------------------------------------//
//   PnPManagerInterfaceTest: registerNode/deregisterNode/isNodeRegistered    //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerInterfaceTestLoginTest, registerNodeWithInvalidNode)
{
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered(XME_CORE_NODE_INVALID_NODE_ID));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_registerNode(XME_CORE_NODE_INVALID_NODE_ID));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered(XME_CORE_NODE_INVALID_NODE_ID));
}

TEST_F(PnPManagerInterfaceTestLoginTest, deregisterNodeWithInvalidNode)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_deregisterNode(XME_CORE_NODE_INVALID_NODE_ID));
}

TEST_F(PnPManagerInterfaceTestLoginTest, deregisterNodeWithNonExistingNode)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)1));
}

TEST_F(PnPManagerInterfaceTestLoginTest, deregisterNodeWithExistingNode)
{
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
}

TEST_F(PnPManagerInterfaceTestLoginTest, deregisterNodeWithExistingNodeAndMoreNodes)
{
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)3));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)3));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)3));
}

TEST_F(PnPManagerInterfaceTestLoginTest, deregisterNodeWithExistingNodeAndMoreNodesReverse)
{
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)3));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)3));
}

TEST_F(PnPManagerInterfaceTestLoginTest, registerNodeWithDifferentNodes)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)4));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));

    // Try it again, to obtain the alredy exists. 
    EXPECT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
}

TEST_F(PnPManagerInterfaceTestLoginTest, registerNodeWithDifferentNodesAndDeregisterLater)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    // Try it again, to obtain the alredy exists. 
    EXPECT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
}

///////////////////////////////////////////////////////////////////////////////////
/// PnPManager: Testing the establishment of logical routes                     ///
///////////////////////////////////////////////////////////////////////////////////

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, createAnnouncementsAndReadRuntimeGraph)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, createAnnouncementsWithAttributesAndReadRuntimeGraph)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR_KB, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR_B, NULL, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR_KB, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, testSetAllLogicalRoutesAsEstablished)
{
    // No announcements. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR_KB, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, testStatusNotFoundInGetNextRuntimeGraph)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
}

TEST_F(PnPManagerInterfaceTestChannelRRWaypointForTwoClientsAndOneServerTests, TestRequestResponseForGraphWithRemoteClient)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));
    uint8_t i;
    xme_core_nodeMgr_compRep_componentHandle_t comp1Handle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    builder = xme_core_nodeMgr_compRep_createBuilder(node1, XME_CORE_COMPONENT_TYPE_SENSOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &comp1Handle));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphA, node1, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));

    EXPECT_EQ((xme_core_node_nodeId_t) 1001, runtimeGraph->nodeId);

    i = 0U;
    // Component A (index=0)
    EXPECT_EQ((xme_core_component_t) 11, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_SENSOR, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("19|", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Injector A (index=1)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Marshaller A (index=2)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // UDP Send A (index=3)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // UDP Receive A (index=4)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);    
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // Demarshaller A (index=5)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Invalid (index=6)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE, runtimeGraph->vertex[i].vertexType);

    i = 0U;
    // From ComponentA to Channel Injector A (index = 0)
    EXPECT_EQ(1, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(2, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Injector A to Marshaler A (index = 1)
    EXPECT_EQ(2, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(3, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Marshaler A to UDP Send A (index = 2)
    EXPECT_EQ(3, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(4, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From UDP Receive A to Demarshaler A (index = 3)
    EXPECT_EQ(5, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(6, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Demarshaler A to Component A (index = 4)
    EXPECT_EQ(6, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(1, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // Invalid (index=5)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE, runtimeGraph->edge[i].edgeType);
}

TEST_F(PnPManagerInterfaceTestChannelRRWaypointForTwoClientsAndOneServerTests, TestRequestResponseForGraphWithLocalClientAndRemoteAndLocalServer)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));
    uint8_t i;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    builder = xme_core_nodeMgr_compRep_createBuilder(node2, XME_CORE_COMPONENT_TYPE_SENSOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));

    builder = xme_core_nodeMgr_compRep_createBuilder(node2, XME_CORE_COMPONENT_TYPE_MONITOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphBC, node2, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));

    EXPECT_EQ((xme_core_node_nodeId_t) 1002, runtimeGraph->nodeId);

    i = 0U;
    // UDP Receive C (index=0)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);    
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // Demarshaler C (index=1)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Component C (index=2)
    EXPECT_EQ((xme_core_component_t) 12, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_MONITOR, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("18|", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Selector C for A (index=3)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1|3|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Marshaller C (index=4)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // UDP Send C (index=5)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // Component B (index=6)
    EXPECT_EQ((xme_core_component_t) 11, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_SENSOR, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("19|", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Injector B (index=7)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("2|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Selector C for B (index=8)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("2|4|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Invalid (index=9)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE, runtimeGraph->vertex[i].vertexType);

    // EDGE CHECKING
    i = 0U;
    // From UDP Receive C to Demarshaler C (index = 0)
    EXPECT_EQ(1, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(2, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Demarshaler C to Component C (index = 1)
    EXPECT_EQ(2, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(3, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Component C to Channel Selector C for A (index = 2)
    EXPECT_EQ(3, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(4, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Component C to Channel Selector B (index = 3)
    EXPECT_EQ(3, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(9, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|4", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Selector C for A to Marshaler C (index = 4)
    EXPECT_EQ(4, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(5, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Marshaler A to UDP Send C (index = 5)
    EXPECT_EQ(5, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(6, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Component B to Channel Injector B (index = 6)
    EXPECT_EQ(7, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(8, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|2", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Injector B to Component C (index = 7)
    EXPECT_EQ(8, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(3, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|2", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Selector B to Component B (index = 8)
    EXPECT_EQ(9, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(7, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|4", runtimeGraph->edge[i].edgeData));
    i++;

    // Invalid (index=9)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE, runtimeGraph->edge[i].edgeType);
}

TEST_F(PnPManagerInterfaceTestChannelRRWaypointForOneClientAndTwoServersTests, TestRequestResponseForGraphWithLocalAndRemoveClientAndLocalServer)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));
    uint8_t i;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    builder = xme_core_nodeMgr_compRep_createBuilder(node1, XME_CORE_COMPONENT_TYPE_MONITOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));

    builder = xme_core_nodeMgr_compRep_createBuilder(node1, XME_CORE_COMPONENT_TYPE_SENSOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphAC, node1, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));

    EXPECT_EQ((xme_core_node_nodeId_t) 1001, runtimeGraph->nodeId);

    i = 0U;
    // Component A (index=0)
    EXPECT_EQ((xme_core_component_t) 11, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_MONITOR, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("19|", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Injector A for B (index=1)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Marshaller A (index=2)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // UDP Send A (index=3)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // UDP Receive A from B (index=4)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);    
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // Demarshaler A from B (index=5)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Injector A for C (index=6)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("2|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Component C (index=7)
    EXPECT_EQ((xme_core_component_t) 12, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_SENSOR, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("16|", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Selector B for A (index=8)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("2|4|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Invalid (index=9)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE, runtimeGraph->vertex[i].vertexType);

    // EDGE CHECKING
    i = 0U;
    // From Component A to Channel Injector A for B (index = 0)
    EXPECT_EQ(1, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(2, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Component A to Channel Injector A to C (index = 5)
    EXPECT_EQ(1, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(7, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|2", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Injector A for B to Marshaller A (index = 1)
    EXPECT_EQ(2, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(3, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Marshaller A to UDP Send A (index = 2)
    EXPECT_EQ(3, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(4, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From UDP Receive A from B to Demarshaller A from B (index = 3)
    EXPECT_EQ(5, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(6, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Demarshaller A from B to Component A (index = 4)
    EXPECT_EQ(6, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(1, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Injector A to C to Component C (index = 6)
    EXPECT_EQ(7, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(8, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|2", runtimeGraph->edge[i].edgeData));
    i++;

    // From Component C to Channel Selector C to A (index = 7)
    EXPECT_EQ(8, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(9, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|4", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Selector C to A to Component A (index = 8)
    EXPECT_EQ(9, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(1, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|4", runtimeGraph->edge[i].edgeData));
    i++;

    // Invalid (index=9)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE, runtimeGraph->edge[i].edgeType);
}

TEST_F(PnPManagerInterfaceTestChannelRRWaypointForOneClientAndTwoServersTests, TestRequestResponseForGraphWithRemoteServer)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));
    uint8_t i;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    builder = xme_core_nodeMgr_compRep_createBuilder(node2, XME_CORE_COMPONENT_TYPE_SENSOR);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphB, node2, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));

    EXPECT_EQ((xme_core_node_nodeId_t) 1002, runtimeGraph->nodeId);

    i = 0U;
    // UDP Receive B (index=0)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);    
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // Demarshaller B (index=1)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Component B (index=2)
    EXPECT_EQ((xme_core_component_t) 11, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_SENSOR, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("18|", runtimeGraph->vertex[i].vertexData));
    i++;

    // Channel Selector B (index=3)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1|3|1", runtimeGraph->vertex[i].vertexData));
    i++;

    // Marshaller B (index=4)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER, runtimeGraph->vertex[i].vertexType);
    EXPECT_EQ(0, strcmp("1", runtimeGraph->vertex[i].vertexData));
    i++;

    // UDP Send B (index=5)
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, runtimeGraph->vertex[i].componentId);
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, runtimeGraph->vertex[i].componentType);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND, runtimeGraph->vertex[i].vertexType);
    // TODO: Check vertexData
    i++;

    // Invalid (index=6)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE, runtimeGraph->vertex[i].vertexType);

    i = 0U;
    // From UDP Receive B to Demarshaler B (index = 0)
    EXPECT_EQ(1, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(2, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Demarshaler B to Component B (index = 1)
    EXPECT_EQ(2, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(3, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("42|5|1", runtimeGraph->edge[i].edgeData));
    i++;

    // From Component B to Channel Selector B (index = 2)
    EXPECT_EQ(3, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(4, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Channel Selector B to Marshaler B (index = 3)
    EXPECT_EQ(4, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(5, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // From Marshaler A to UDP Send B (index = 4)
    EXPECT_EQ(5, runtimeGraph->edge[i].srcVertexIndex);
    EXPECT_EQ(6, runtimeGraph->edge[i].sinkVertexIndex);
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, runtimeGraph->edge[i].edgeType);
    EXPECT_EQ(0, strcmp("43|5|3", runtimeGraph->edge[i].edgeData));
    i++;

    // Invalid (index=5)
    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE, runtimeGraph->edge[i].edgeType);
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, LogoutProcessForOneNode)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

    // Unannounce the whole node. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceNode(localNodeId));

    while (xme_core_pnp_pnpManager_hasNewRuntimeGraphs())
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));

        EXPECT_TRUE(NULL != runtimeGraph);

        switch((xme_maxSystemValue_t) runtimeGraph->nodeId)
        {
            case 1:
                ASSERT_EQ(XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_LOGOUT, (xme_core_topic_pnpManagerRuntimeGraphModelAction_e) runtimeGraph->action);
                break;
                // NOTE: We do not check here the content of the graph. This should be made in a separate test. 
            case 2:
            case 3:
                ASSERT_EQ(XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_REMOVE, (xme_core_topic_pnpManagerRuntimeGraphModelAction_e) runtimeGraph->action);
                break;
            default:
                ASSERT_TRUE(false); // <- this assertion should never be reached in this unit test. 
        }
    }

    // Finally, remove all logical routes associated to that node. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeNodeInstances(localNodeId));
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, LogoutProcessForInvalidNode)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_unannounceNode(XME_CORE_NODE_INVALID_NODE_ID));
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, LogoutProcessForNonRegisteredNode)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_unannounceNode(localNodeId));
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, LogoutProcessForNonExistingLogicalRoutes)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));

    // Check that there are no routes associated to the local node, but still unannouncement takes place (XME_STATUS_SUCCESS). 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceNode(localNodeId));

    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, LogoutProcessForNonMatchingLogicalRoutes)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    createComponentManifestForPnPManagerTest();

    // Create two matching topics... 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    // ... and one non-matching topic. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_USER, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    // Get runtime graphs and establish logical routes (2 runtime graphs). 
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

    // Check that there are no routes associated to the node3, but still unannouncement takes place (XME_STATUS_SUCCESS). 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceNode(nodeId3));

    // There should be no runtime graphs (remove/logout)
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, LogoutProcessForNonEstablishedLogicalRoutes)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    // Create two matching topics... 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    // Get runtime graphs and establish logical routes (3 runtime graphs). 
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    // Remember: we got the runtime graphs (to empty the runtime graphs list), but not establish routes. 

    // Check that there are no routes associated to the local node, but still unannouncement takes place (XME_STATUS_SUCCESS). 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceNode(localNodeId));

    // No new runtime graphs expected. 
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, RemoveLogicalRoutesForInvalidNode)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_removeNodeInstances(XME_CORE_NODE_INVALID_NODE_ID));
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, RemoveLogicalRoutesForNonExistingRoutes)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_removeNodeInstances(localNodeId));
}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, RemoveLogicalRoutesForNonEstablishedRoutes)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    createComponentManifestForPnPManagerTest();

    // localNode and node2 have a logical route. For node3 there are no routes. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, NULL, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, NULL, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_USER, NULL, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    // Note: We do not establish the route between localNode and node2. 

    // Try to remove logical routes associated to localNode (non-established route) and node3 (non-existing route). 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeNodeInstances(localNodeId));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeNodeInstances(nodeId3));
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_removeNodeInstances((xme_core_node_nodeId_t)4));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

void
createComponentManifestForPnPManagerTest(void)
{
    xme_core_componentManifest_t* componentManifest;

    componentManifest = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));

    componentManifest->componentType = XME_CORE_COMPONENT_TYPE_USER;
    (void) xme_hal_safeString_strncpy(componentManifest->name, "testComponent", sizeof(componentManifest->name));

    //Function
    componentManifest->functionManifests[0].functionId = (xme_core_component_functionId_t) 1;
    (void) xme_hal_safeString_strncpy(componentManifest->functionManifests[0].name, "testFunction", sizeof(componentManifest->functionManifests[0].name));
    componentManifest->functionManifests[0].wcet = 100000000;
    componentManifest->functionManifests[0].alphaCurve.alphaCurve = 0;
    componentManifest->functionManifests[0].completion = true;

    //Port
    componentManifest->portManifests[0].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
    componentManifest->portManifests[0].topic = XME_CORE_TOPIC_USER; 
    componentManifest->portManifests[0].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
    componentManifest->portManifests[0].lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
    componentManifest->portManifests[0].upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_manifestRepository_addComponentManifest(componentManifest, false), XME_CHECK_RVAL_VOID);
}
