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
 * $Id: interfaceTestPnPManager.cpp 5220 2013-09-30 09:34:21Z ruiz $
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
#include "xme/core/plugAndPlay/include/networkConfigurationCalculator.h"

#include "xme/defines.h"
#include "xme/core/node.h"
#include "xme/core/topic.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PnPManagerInterfaceTestSplitGraph: public ::testing::Test
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

    virtual void SetUp()
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
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
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
            "192.168.0.1:8080"
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
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
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
            "192.168.0.1:8080"
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
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
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
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &logicalRouteAtoC,
            chan1,
            false
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpSendToUDPReceiveAtoC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );
        xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
        (
            &udpSendToUDPReceiveAtoC,
            udpKey
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerCfromA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentCfromA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
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
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
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
            "192.168.0.1:8080"
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
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
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
            "192.168.0.1:8080"
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

        // Populate the component data
#if 0 // NOT NEEDED. ALREADY CREATED BEFORE.
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
            node3,
            cont3,
            comp3,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            dpid3,
            NULL
        );
#endif

        // Create vertices

        udpReceiveCfromB.vertexId = xme_hal_graph_addVertex(&nccGraph, &udpReceiveCfromB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, udpReceiveCfromB.vertexId);

        demarshalerCfromB.vertexId = xme_hal_graph_addVertex(&nccGraph, &demarshalerCfromB);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, demarshalerCfromB.vertexId);

#if 0 // NOT NEEDED. ALREADY CREATED BEFORE.
        componentC.vertexId = xme_hal_graph_addVertex(&nccGraph, &componentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, componentC.vertexId);
#endif

        // Populate the edge data

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &logicalRouteBtoC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &logicalRouteBtoC,
            chan2,
            false
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpSendToUDPReceiveBtoC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );
        xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
        (
            &udpSendToUDPReceiveBtoC,
            udpKey
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &udpReceiveToDemarshalerCfromB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentCfromB,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
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

    virtual void TearDown()
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

class PnPManagerInterfaceTestTransformSubgraph: public ::testing::Test
{
protected:
    PnPManagerInterfaceTestTransformSubgraph()
    {
        udpKey[0] = 1;
        udpKey[1] = 2;
        udpKey[2] = 3;
        udpKey[3] = 4;

        xme_core_pnp_lrm_init(NULL);
    }

    virtual ~PnPManagerInterfaceTestTransformSubgraph()
    {
        xme_core_pnp_lrm_fini();
    }

    virtual void SetUp()
    { 
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
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
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
            "192.168.0.1:8080"
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
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &marshalerToUDPSendA,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
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
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
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
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
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
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
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
            "192.168.0.1:8080"
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
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
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
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );

        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &demarshalerToComponentC,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
            XME_HAL_GRAPH_INVALID_EDGE_ID,
            topic
        );

        udpReceiveToDemarshalerC.edgeId = xme_hal_graph_addEdge(&nccGraphC, udpReceiveC.vertexId, demarshalerC.vertexId, &udpReceiveToDemarshalerC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, udpReceiveToDemarshalerC.edgeId);

        demarshalerToComponentC.edgeId = xme_hal_graph_addEdge(&nccGraphC, demarshalerC.vertexId, componentC.vertexId, &demarshalerToComponentC);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, demarshalerToComponentC.edgeId);

        EXPECT_EQ(3, xme_hal_graph_getVertexCount(&nccGraphA));
        EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&nccGraphA));
    }

    virtual void TearDown()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphA));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphB));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&nccGraphC));
    }

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

        // Create and add component type manifest for component '´monitor (KB)' to manifest repository
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

                switch (componentType)
                {
                    case XME_CORE_COMPONENT_TYPE_MONITOR:
                    case XME_CORE_COMPONENT_TYPE_MONITOR_KB:
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

    static const xme_core_topic_t topic = XME_CORE_TOPIC(42);

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

class PnPManagerInterfaceTestLoginTests: public ::testing::Test
{
protected:
    PnPManagerInterfaceTestLoginTests()
    {
        xme_core_pnp_pnpManager_init(NULL);
    }

    virtual ~PnPManagerInterfaceTestLoginTests()
    {
        xme_core_pnp_pnpManager_fini();
    }
};

class PnPManagerInterfaceTestEstablishingLogicalRouteTests: public ::testing::Test
{
protected:
    PnPManagerInterfaceTestEstablishingLogicalRouteTests()
    : localNodeId((xme_core_node_nodeId_t)1)
    , nodeId2((xme_core_node_nodeId_t)2)
    , nodeId3((xme_core_node_nodeId_t)3)
    {
        xme_core_pnp_pnpManager_init(NULL);
        xme_core_pnp_lrm_init(NULL);

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

    virtual ~PnPManagerInterfaceTestEstablishingLogicalRouteTests()
    {
        xme_core_pnp_pnpManager_fini();
        xme_core_directory_nodeRegistryController_fini();
        xme_core_pnp_lrm_fini();

    }

    virtual void SetUp()
    { 
    }

    virtual void TearDown()
    {
    }

    xme_core_node_nodeId_t localNodeId;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeId3;
};

/******************************************************************************/
/***   Helper functions                                                     ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

//// Ensure that this function gets exported to other translation units
//extern uint16_t
//xme_core_topicUtil_getTopicSize
//(
//    xme_core_topic_t topicId
//);

uint16_t
xme_core_topicUtil_getTopicSize
(
    xme_core_topic_t topicId
)
{
    XME_UNUSED_PARAMETER(topicId);

    return 5;
}

XME_EXTERN_C_END

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     PnPManagerInterfaceTestSplitGraph                                    //
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

    XME_HAL_SINGLYLINKEDLIST_FINI(graphList);
}

//----------------------------------------------------------------------------//
//     PnPManagerInterfaceTestTransformSubgraph                             //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerInterfaceTestTransformSubgraph, processGraphWithRemotePublisherSubgraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createInstance(node1, comp1, XME_CORE_COMPONENT_TYPE_SENSOR)); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphA, node1));
    // TODO: Test resulting data structure
}

TEST_F(PnPManagerInterfaceTestTransformSubgraph, processGraphWithLocalMemCopySubgraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createInstance(node1, comp1, XME_CORE_COMPONENT_TYPE_SENSOR)); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphB, node1));
    // TODO: Test resulting data structure
}

TEST_F(PnPManagerInterfaceTestTransformSubgraph, processGraphWithRemoveSubscriberSubgraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createInstance(node3, comp1, XME_CORE_COMPONENT_TYPE_SENSOR)); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_processGraph(&nccGraphC, node3));
    // TODO: Test resulting data structure
}

TEST_F(PnPManagerInterfaceTestTransformSubgraph, announcePortsWithNullParameters)
{
    xme_core_componentManifest_t* componentManifest;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(NULL, XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(NULL, node1, comp1));

    createComponentTypeManifests();
    componentManifest = (xme_core_componentManifest_t*)xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_findComponentManifest(XME_CORE_COMPONENT_TYPE_SENSOR, componentManifest));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(componentManifest, XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(componentManifest, XME_CORE_NODE_INVALID_NODE_ID, comp1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announcePorts(componentManifest, node1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announcePorts(componentManifest, node1, comp1));
}


TEST_F(PnPManagerInterfaceTestTransformSubgraph, instanceFullTestParameters)
{
    xme_core_pnp_pnpManager_fini();
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

    // Create instance
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createInstance(node1, comp1, XME_CORE_COMPONENT_TYPE_SENSOR));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_createInstance(XME_CORE_NODE_INVALID_NODE_ID, comp1, XME_CORE_COMPONENT_TYPE_SENSOR));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_createInstance(node1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_CORE_COMPONENT_TYPE_SENSOR));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_createInstance(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_CORE_COMPONENT_TYPE_SENSOR));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_createInstance(node1, comp1, XME_CORE_COMPONENT_TYPE_INVALID));

    // Update instance
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstance(node1, comp1, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_REQUESTED));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstance(node1, comp1, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_PREPARED));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstance(node1, comp1, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_SUBMITTED));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstance(node1, comp1, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_RUNNING));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstance(node1, comp1, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_FAILURE));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstance(node1, comp1, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_INVALID_STATUS));

    // RemoveInstance
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_removeInstance(XME_CORE_NODE_INVALID_NODE_ID, comp1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_removeInstance(node1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_removeInstance(XME_CORE_NODE_INVALID_NODE_ID, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeInstance(node1, comp1));
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_removeInstance(node1, comp1));
}

TEST_F(PnPManagerInterfaceTestTransformSubgraph, testGetComponentType)
{
    xme_core_componentType_t componentType;

    xme_core_pnp_pnpManager_fini();
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

    // Get the component type. 
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(XME_CORE_NODE_INVALID_NODE_ID, comp1, &componentType));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(node1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, &componentType));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(node1, comp1, NULL));
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getComponentType(node1, comp1, &componentType));

    // Create instance
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createInstance(node1, comp1, XME_CORE_COMPONENT_TYPE_SENSOR));

    // Get the component type. 
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(XME_CORE_NODE_INVALID_NODE_ID, comp1, &componentType));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(node1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, &componentType));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(node1, comp1, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getComponentType(node1, comp1, &componentType));
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_SENSOR, componentType);

    // RemoveInstance
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeInstance(node1, comp1));

    // Get the component type. 
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(XME_CORE_NODE_INVALID_NODE_ID, comp1, &componentType));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(node1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, &componentType));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getComponentType(node1, comp1, NULL));
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getComponentType(node1, comp1, &componentType));
}


//----------------------------------------------------------------------------//
//   PnPManagerInterfaceTest: registerNode/deregisterNode/isNodeRegistered    //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerInterfaceTestLoginTests, registerNodeWithInvalidNode)
{
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered(XME_CORE_NODE_INVALID_NODE_ID));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_registerNode(XME_CORE_NODE_INVALID_NODE_ID));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered(XME_CORE_NODE_INVALID_NODE_ID));
}

TEST_F(PnPManagerInterfaceTestLoginTests, deregisterNodeWithInvalidNode)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_deregisterNode(XME_CORE_NODE_INVALID_NODE_ID));
}

TEST_F(PnPManagerInterfaceTestLoginTests, deregisterNodeWithNonExistingNode)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)1));
}

TEST_F(PnPManagerInterfaceTestLoginTests, deregisterNodeWithExistingNode)
{
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)1));
    EXPECT_TRUE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_deregisterNode((xme_core_node_nodeId_t)1));
    EXPECT_FALSE(xme_core_pnp_pnpManager_isNodeRegistered((xme_core_node_nodeId_t)1));
}

TEST_F(PnPManagerInterfaceTestLoginTests, deregisterNodeWithExistingNodeAndMoreNodes)
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

TEST_F(PnPManagerInterfaceTestLoginTests, deregisterNodeWithExistingNodeAndMoreNodesReverse)
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

TEST_F(PnPManagerInterfaceTestLoginTests, registerNodeWithDifferentNodes)
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

TEST_F(PnPManagerInterfaceTestLoginTests, registerNodeWithDifferentNodesAndDeregisterLater)
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

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

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

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

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

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR_KB, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR_B, nodeId2, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_SENSOR_KB, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR, nodeId3, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));

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

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_instantiateComponentOnNode(XME_CORE_COMPONENT_TYPE_MONITOR_KB, localNodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());

}

TEST_F(PnPManagerInterfaceTestEstablishingLogicalRouteTests, testStatusNotFoundInGetNextRuntimeGraph)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* runtimeGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraph(runtimeGraph));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
