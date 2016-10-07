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
 * $Id: interfaceTestNetworkConfigurationCalculator.cpp 7700 2014-03-06 19:19:54Z ruiz $
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

#include "xme/core/manifestRepository/include/manifestRepository.h"
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"
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
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&logicalRoutes));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&deploymentGraph));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());
    }

    virtual ~NetworkConfigurationCalculatorInterfaceTest()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_manifestRepository_fini();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&deploymentGraph));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&logicalRoutes));

        xme_core_directory_nodeRegistryController_fini();
    }

    xme_hal_graph_graph_t logicalRoutes;
    xme_hal_graph_graph_t deploymentGraph;
};

class NetworkConfigurationCalculatorInterfaceTestForIntraNodeDCC: public NetworkConfigurationCalculatorInterfaceTest
{
protected:
    NetworkConfigurationCalculatorInterfaceTestForIntraNodeDCC()
    : node1((xme_core_node_nodeId_t) 1)
    , comp1((xme_core_component_t) 1)
    , comp2((xme_core_component_t) 2)
    , comp3((xme_core_component_t) 3)
    , topic((xme_core_topic_t) 42)
    , chan1((xme_core_channelId_t) 1)
    , chan2((xme_core_channelId_t) 2)
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
         *  | node      | node1 | node1 | node1 |
         *  | component | comp1 | comp2 | comp3 |
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
         *        MC
         *      .----> vs1
         *     /
         *    vp
         *     \  MC
         *      '----> vs2
         */

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "vp, vs1 and vs2 node", (xme_core_node_guid_t) 1));

        // Add network interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1));

        // Add vp
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vpd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vpd),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vpd,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vpd.vertexId);

        // Add vs1
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vs1d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vs1d),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vs1d,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs1d.vertexId);

        // Add vs2
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vs2d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vs2d),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vs2d,
            comp3,
            (xme_core_componentType_t)comp3,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs2d.vertexId);

        // Add e1
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e1d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, vpd.vertexId, vs1d.vertexId, &e1d)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e1d,
            topic,
            chan1,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1d.edgeId);

        // Add e2
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e2d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, vpd.vertexId, vs2d.vertexId, &e2d)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e2d,
            topic,
            chan2,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2d.edgeId);

        xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Logical Routes", &logicalRoutes);
    }

    virtual ~NetworkConfigurationCalculatorInterfaceTestForIntraNodeDCC()
    {
    }

    xme_core_node_nodeId_t node1;

    xme_com_interface_address_t nodeIntf1;

    xme_core_component_t comp1;
    xme_core_component_t comp2;
    xme_core_component_t comp3;

    xme_core_topic_t topic;

    xme_core_channelId_t chan1;
    xme_core_channelId_t chan2;

    xme_core_pnp_dataLinkGraph_vertexData_t vpd;
    xme_core_pnp_dataLinkGraph_vertexData_t vs1d;
    xme_core_pnp_dataLinkGraph_vertexData_t vs2d;

    xme_core_pnp_dataLinkGraph_edgeData_t e1d;
    xme_core_pnp_dataLinkGraph_edgeData_t e2d;
};

class NetworkConfigurationCalculatorInterfaceTestForQueueSizeCalculation: public NetworkConfigurationCalculatorInterfaceTest
{
protected:
    NetworkConfigurationCalculatorInterfaceTestForQueueSizeCalculation()
    : node1((xme_core_node_nodeId_t) 1)
    , comp1((xme_core_component_t) 1)
    , comp2((xme_core_component_t) 2)
    , comp3((xme_core_component_t) 3)
    , topic((xme_core_topic_t) 42)
    , chan1((xme_core_channelId_t) 1)
    , chan2((xme_core_channelId_t) 2)
    {
        /*
         * Builds the following graph of logical routes:
         *
         *    vp1 ----.
         *             \
         *              vs
         *             /
         *    vp2 ----'
         *
         *
         *  .-----------.-------.-------.-------.
         *  | attribute |  vp1  |  vp2  |  vs   |
         *  |-----------|-------|-------|-------|
         *  | topic     | topic | topic | topic |
         *  | node      | node1 | node1 | node1 |
         *  | component | comp1 | comp2 | comp3 |
         *  '-----------'-------'-------'-------'
         * 
         *  Key:
         *  vp1  = publication component port vertex 1 (compID = comp1, compType = comp1)
         *  vp2  = publication component port vertex 2 (compID = comp2, compType = comp2)
         *  vs = subscription component port vertex (compID = comp3, compType = comp3)
         */

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "node with vp1, vp2 and vs", (xme_core_node_guid_t) 1));

        // Add network interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1));

        // Add vp1
        {
            // Add manifest for vp1
            xme_core_componentManifest_t componentManifest;
            xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = (xme_core_componentType_t)comp1;
            componentManifest.portManifests[0].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
            componentManifest.portManifests[0].queueSize = 2u;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));

            // Register vp1 in component repository
            xme_core_nodeMgr_compRep_componentBuilder_t* builder = xme_core_nodeMgr_compRep_createBuilder(node1, (xme_core_componentType_t)comp1);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
            EXPECT_NE((void*)NULL, builder);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));
        }
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vp1d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vp1d),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vp1d,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vp1d.vertexId);

        // Add vp2
        {
            // Add manifest for vp2
            xme_core_componentManifest_t componentManifest;
            xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = (xme_core_componentType_t)comp2;
            componentManifest.portManifests[0].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
            componentManifest.portManifests[0].queueSize = 3u;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));

            // Register vp2 in component repository
            xme_core_nodeMgr_compRep_componentBuilder_t* builder = xme_core_nodeMgr_compRep_createBuilder(node1, (xme_core_componentType_t)comp2);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp2);
            EXPECT_NE((void*)NULL, builder);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));
        }
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vp2d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vp2d),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vp2d,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vp2d.vertexId);

        // Add vs
        {
            // Add manifest for vs
            xme_core_componentManifest_t componentManifest;
            xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = (xme_core_componentType_t)comp3;
            componentManifest.portManifests[0].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
            componentManifest.portManifests[0].queueSize = 1u; // This should be overridden by the NCC to 5
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));

            // Register vs in component repository
            xme_core_nodeMgr_compRep_componentBuilder_t* builder = xme_core_nodeMgr_compRep_createBuilder(node1, (xme_core_componentType_t)comp3);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp3);
            EXPECT_NE((void*)NULL, builder);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));
        }
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vsd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vsd),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vsd,
            comp3,
            (xme_core_componentType_t)comp3,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vsd.vertexId);

        // Add e1
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e1d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, vp1d.vertexId, vsd.vertexId, &e1d)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e1d,
            topic,
            chan1,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1d.edgeId);

        // Add e2
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e2d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, vp2d.vertexId, vsd.vertexId, &e2d)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e2d,
            topic,
            chan2,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2d.edgeId);

        xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Logical Routes", &logicalRoutes);
    }

    virtual ~NetworkConfigurationCalculatorInterfaceTestForQueueSizeCalculation()
    {
    }

    xme_core_node_nodeId_t node1;

    xme_com_interface_address_t nodeIntf1;

    xme_core_component_t comp1;
    xme_core_component_t comp2;
    xme_core_component_t comp3;

    xme_core_topic_t topic;

    xme_core_channelId_t chan1;
    xme_core_channelId_t chan2;

    xme_core_pnp_dataLinkGraph_vertexData_t vp1d;
    xme_core_pnp_dataLinkGraph_vertexData_t vp2d;
    xme_core_pnp_dataLinkGraph_vertexData_t vsd;

    xme_core_pnp_dataLinkGraph_edgeData_t e1d;
    xme_core_pnp_dataLinkGraph_edgeData_t e2d;
};

class NetworkConfigurationCalculatorInterfaceTestForInterNodeDCC: public NetworkConfigurationCalculatorInterfaceTest
{
protected:
    NetworkConfigurationCalculatorInterfaceTestForInterNodeDCC()
    : node1((xme_core_node_nodeId_t) 1)
    , node2((xme_core_node_nodeId_t) 2)
    , comp1((xme_core_component_t) 1)
    , comp2((xme_core_component_t) 2)
    , comp3((xme_core_component_t) 3)
    , topic((xme_core_topic_t) 42)
    , chan1((xme_core_channelId_t) 1)
    , chan2((xme_core_channelId_t) 2)
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
         *  | component | comp1 | comp2 | comp3 |
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

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "vp and vs2 node", (xme_core_node_guid_t) 1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "vs1 node", (xme_core_node_guid_t) 2));

        // Add network interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.2:32211", &nodeIntf2));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node2, nodeIntf2));

        // Add vp
        {
            // Add manifest for vp
            xme_core_componentManifest_t componentManifest;
            xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = (xme_core_componentType_t)comp1;
            componentManifest.portManifests[0].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
            componentManifest.portManifests[0].queueSize = 42u;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));

            // Register vp in component repository
            xme_core_nodeMgr_compRep_componentBuilder_t* builder = xme_core_nodeMgr_compRep_createBuilder(node1, (xme_core_componentType_t)comp1);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp1);
            EXPECT_NE((void*)NULL, builder);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));
        }
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vpd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vpd),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vpd,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vpd.vertexId);

        // Add vs1
        {
            // Add manifest for vs1
            xme_core_componentManifest_t componentManifest;
            xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = (xme_core_componentType_t)comp2;
            componentManifest.portManifests[0].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
            componentManifest.portManifests[0].queueSize = 12u;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));

            // Register vs1 in component repository
            xme_core_nodeMgr_compRep_componentBuilder_t* builder = xme_core_nodeMgr_compRep_createBuilder(node2, (xme_core_componentType_t)comp2);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, comp2);
            EXPECT_NE((void*)NULL, builder);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));
        }
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vs1d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vs1d),
            node2,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vs1d,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs1d.vertexId);

        // Add vs2
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &vs2d,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &vs2d),
            node1,
            topic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &vs2d,
            comp3,
            (xme_core_componentType_t)comp3,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs2d.vertexId);

        // Add e1
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e1d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, vpd.vertexId, vs1d.vertexId, &e1d)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e1d,
            topic,
            chan1,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1d.edgeId);

        // Add e2
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &e2d,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, vpd.vertexId, vs2d.vertexId, &e2d)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &e2d,
            topic,
            chan2,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2d.edgeId);

        xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Logical Routes", &logicalRoutes);
    }

    virtual ~NetworkConfigurationCalculatorInterfaceTestForInterNodeDCC()
    {
    }

    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;

    xme_com_interface_address_t nodeIntf1;
    xme_com_interface_address_t nodeIntf2;

    xme_core_component_t comp1;
    xme_core_component_t comp2;
    xme_core_component_t comp3;

    xme_core_topic_t topic;

    xme_core_channelId_t chan1;
    xme_core_channelId_t chan2;

    xme_core_pnp_dataLinkGraph_vertexData_t vpd;
    xme_core_pnp_dataLinkGraph_vertexData_t vs1d;
    xme_core_pnp_dataLinkGraph_vertexData_t vs2d;

    xme_core_pnp_dataLinkGraph_edgeData_t e1d;
    xme_core_pnp_dataLinkGraph_edgeData_t e2d;
};

class NetworkConfigurationCalculatorInterfaceTestForIntraNodeRR: public NetworkConfigurationCalculatorInterfaceTest
{
protected:
    NetworkConfigurationCalculatorInterfaceTestForIntraNodeRR()
    : node1((xme_core_node_nodeId_t) 1)
    , comp1((xme_core_component_t) 1)
    , comp2((xme_core_component_t) 2)
    , requestTopic((xme_core_topic_t) 42)
    , responseTopic((xme_core_topic_t) 43)
    , chan1((xme_core_channelId_t) 1)
    , chan2((xme_core_channelId_t) 2)
    {
        /*
         * Builds the following graph of logical routes:
         *
         *        LR rq
         *    RqS -----> RqH
         *    | |        |¦| CM
         *    RsH <----- RsS
         * comp1  LR rs   comp2
         *
         *
         *  .-----------.-------.-------.-------.-------.
         *  | attribute |  RqS  |  RqH  |  RsS  |  RsH  |
         *  |-----------|-------|-------|-------|-------|
         *  | topic     |  req  |  req  |  rsp  |  rsp  |
         *  | node      | node1 | node1 | node1 | node1 |
         *  | component | comp1 | comp2 | comp2 | comp1 |
         *  '-----------'-------'-------'-------'-------'
         * 
         *  .-----------.-------.-------.
         *  | attribute |  rq   |  rs   |
         *  |-----------|-------|-------|
         *  | topic     |  req  |  rsp  |
         *  | channel   | chan1 | chan2 |
         *  '-----------'-------'-------'
         * 
         *  Key:
         *  RqS = request sender component port vertex
         *  RqH = request handler component port vertex
         *  RsS = response sender component port vertex
         *  RsH = response handler component port vertex
         *  rq  = logical route edge for request
         *  rs  = logical route edge for response
         *  cm  = channel mapping edge for request/response
         *  ciX = channel injector waypoint for route X
         *  csX = channel selector waypoint for route X
         *  LR  - logical route edge
         *  MC  - memcopy edge
         *  CM  = channel mapping edge
         *
         * After the Network Configuration Calculator has calculated
         * the waypoints, the following graph is expected:
         *
         *                ,-- config: ch1
         *                |
         *          MC         MC
         *    RqS -----> ci1 -----> RqH
         *    | |                   | |
         *    RsH <----- cs2 <----- RsS
         * comp1    MC    |    MC    comp2
         * node1          |          node1
         *                '-- config: ch1 -> ch2
         */

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "RqS, RqH, RsS and RsH node", (xme_core_node_guid_t) 1));

        // Add network interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1));

        // Add RqS
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rqsd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rqsd),
            node1,
            requestTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rqsd,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rqsd.vertexId);

        // Add RqH
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rqhd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rqhd),
            node1,
            requestTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rqhd,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rqhd.vertexId);

        // Add RsS
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rssd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rssd),
            node1,
            responseTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rssd,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rssd.vertexId);

        // Add RsH
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rshd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rshd),
            node1,
            responseTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rshd,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rshd.vertexId);

        // Add rq
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &rqd,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, rqsd.vertexId, rqhd.vertexId, &rqd)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &rqd,
            requestTopic,
            chan1,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, rqd.edgeId);

        // Add cm
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &cmd,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING,
            xme_hal_graph_addEdge(&logicalRoutes, rqhd.vertexId, rssd.vertexId, &cmd)
        );
        xme_core_pnp_dataLinkGraph_initChannelMappingEdgeData
        (
            &cmd,
            chan1,
            chan2
        );

        // Add rs
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &rsd,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, rssd.vertexId, rshd.vertexId, &rsd)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &rsd,
            responseTopic,
            chan2,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, rsd.edgeId);

        xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Logical Routes", &logicalRoutes);
    }

    virtual ~NetworkConfigurationCalculatorInterfaceTestForIntraNodeRR()
    {
    }

    xme_core_node_nodeId_t node1;

    xme_com_interface_address_t nodeIntf1;

    xme_core_component_t comp1;
    xme_core_component_t comp2;

    xme_core_topic_t requestTopic;
    xme_core_topic_t responseTopic;

    xme_core_channelId_t chan1;
    xme_core_channelId_t chan2;

    xme_core_pnp_dataLinkGraph_vertexData_t rqsd;
    xme_core_pnp_dataLinkGraph_vertexData_t rqhd;
    xme_core_pnp_dataLinkGraph_vertexData_t rssd;
    xme_core_pnp_dataLinkGraph_vertexData_t rshd;

    xme_core_pnp_dataLinkGraph_edgeData_t rqd;
    xme_core_pnp_dataLinkGraph_edgeData_t rsd;
    xme_core_pnp_dataLinkGraph_edgeData_t cmd;
};

class NetworkConfigurationCalculatorInterfaceTestForInterNodeRR: public NetworkConfigurationCalculatorInterfaceTest
{
protected:
    NetworkConfigurationCalculatorInterfaceTestForInterNodeRR()
    : node1((xme_core_node_nodeId_t) 1)
    , node2((xme_core_node_nodeId_t) 2)
    , comp1((xme_core_component_t) 1)
    , comp2((xme_core_component_t) 2)
    , requestTopic((xme_core_topic_t) 42)
    , responseTopic((xme_core_topic_t) 43)
    , chan1((xme_core_channelId_t) 1)
    , chan2((xme_core_channelId_t) 2)
    {
        /*
         * Builds the following graph of logical routes:
         *
         *        LR rq
         *    RqS -----> RqH
         *    | |        |¦| CM
         *    RsH <----- RsS
         * comp1  LR rs   comp2
         *
         *
         *  .-----------.-------.-------.-------.-------.
         *  | attribute |  RqS  |  RqH  |  RsS  |  RsH  |
         *  |-----------|-------|-------|-------|-------|
         *  | topic     |  req  |  req  |  rsp  |  rsp  |
         *  | node      | node1 | node2 | node2 | node1 |
         *  | component | comp1 | comp2 | comp2 | comp1 |
         *  '-----------'-------'-------'-------'-------'
         * 
         *  .-----------.-------.-------.
         *  | attribute |  rq   |  rs   |
         *  |-----------|-------|-------|
         *  | topic     |  req  |  rsp  |
         *  | channel   | chan1 | chan2 |
         *  '-----------'-------'-------'
         * 
         *  Key:
         *  RqS = request sender component port vertex
         *  RqH = request handler component port vertex
         *  RsS = response sender component port vertex
         *  RsH = response handler component port vertex
         *  rq  = logical route edge for request
         *  rs  = logical route edge for response
         *  cm  = channel mapping edge for request/response
         *  ciX = channel injector waypoint for route X
         *  csX = channel selector waypoint for route X
         *  LR  - logical route edge
         *  MC  - memcopy edge
         *  CM  = channel mapping edge
         *
         * After the Network Configuration Calculator has calculated
         * the waypoints, the following graph is expected:
         *
         *                ,-- config: ch1
         *                |
         *          MC         MC
         *    RqS -----> ci1 -----> RqH
         *    | |                   | |
         *    RsH <----- cs2 <----- RsS
         * comp1    MC    |    MC    comp2
         * node1          |          node2
         *                '-- config: ch1 -> ch2
         */

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node1, "RqS and RsH node", (xme_core_node_guid_t) 1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node2, "RqH and RsS node", (xme_core_node_guid_t) 2));

        // Add network interfaces
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.2:32211", &nodeIntf2));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node1, nodeIntf1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node2, nodeIntf2));

        // Add RqS
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rqsd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rqsd),
            node1,
            requestTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rqsd,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rqsd.vertexId);

        // Add RqH
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rqhd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rqhd),
            node2,
            requestTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rqhd,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rqhd.vertexId);

        // Add RsS
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rssd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rssd),
            node2,
            responseTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rssd,
            comp2,
            (xme_core_componentType_t)comp2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rssd.vertexId);

        // Add RsH
        xme_core_pnp_dataLinkGraph_initVertexData
        (
            &rshd,
            XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT,
            xme_hal_graph_addVertex(&logicalRoutes, &rshd),
            node1,
            responseTopic
        );
        xme_core_pnp_dataLinkGraph_initComponentPortVertexData
        (
            &rshd,
            comp1,
            (xme_core_componentType_t)comp1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            NULL // announcement should not be needed
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, rshd.vertexId);

        // Add rq
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &rqd,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, rqsd.vertexId, rqhd.vertexId, &rqd)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &rqd,
            requestTopic,
            chan1,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, rqd.edgeId);

        // Add cm
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &cmd,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING,
            xme_hal_graph_addEdge(&logicalRoutes, rqhd.vertexId, rssd.vertexId, &cmd)
        );
        xme_core_pnp_dataLinkGraph_initChannelMappingEdgeData
        (
            &cmd,
            chan1,
            chan2
        );

        // Add rs
        xme_core_pnp_dataLinkGraph_initEdgeData
        (
            &rsd,
            XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE,
            xme_hal_graph_addEdge(&logicalRoutes, rssd.vertexId, rshd.vertexId, &rsd)
        );
        xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
        (
            &rsd,
            responseTopic,
            chan2,
            false
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, rsd.edgeId);

        xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Logical Routes", &logicalRoutes);
    }

    virtual ~NetworkConfigurationCalculatorInterfaceTestForInterNodeRR()
    {
    }

    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;

    xme_com_interface_address_t nodeIntf1;
    xme_com_interface_address_t nodeIntf2;

    xme_core_component_t comp1;
    xme_core_component_t comp2;

    xme_core_topic_t requestTopic;
    xme_core_topic_t responseTopic;

    xme_core_channelId_t chan1;
    xme_core_channelId_t chan2;

    xme_core_pnp_dataLinkGraph_vertexData_t rqsd;
    xme_core_pnp_dataLinkGraph_vertexData_t rqhd;
    xme_core_pnp_dataLinkGraph_vertexData_t rssd;
    xme_core_pnp_dataLinkGraph_vertexData_t rshd;

    xme_core_pnp_dataLinkGraph_edgeData_t rqd;
    xme_core_pnp_dataLinkGraph_edgeData_t rsd;
    xme_core_pnp_dataLinkGraph_edgeData_t cmd;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NetworkConfigurationCalculatorInterfaceTestForIntraNodeDCC             //
//----------------------------------------------------------------------------//

TEST_F(NetworkConfigurationCalculatorInterfaceTestForIntraNodeDCC, waypointTransform)
{
    xme_hal_graph_vertexId_t vp, vs1, vs2;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_ncc_getPhysicalRoutes(&logicalRoutes, &deploymentGraph));

    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Physical Routes", &deploymentGraph);

    EXPECT_EQ(3, xme_hal_graph_getVertexCount(&deploymentGraph));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&deploymentGraph));

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
        xme_core_pnp_dataLinkGraph_vertexData_t* vd = NULL;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed = NULL;

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
            EXPECT_EQ(comp1, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect two outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs1, vi);

                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs2, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect zero incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vs1 == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp2, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs1, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
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
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs2, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else
        {
            // Unxpected vertex
            EXPECT_TRUE(false);
        }
    }
    xme_hal_graph_finiVertexIterator(&deploymentGraph);
}

//----------------------------------------------------------------------------//
//     NetworkConfigurationCalculatorInterfaceTestForQueueSizeCalculation     //
//----------------------------------------------------------------------------//

TEST_F(NetworkConfigurationCalculatorInterfaceTestForQueueSizeCalculation, waypointTransform)
{
    xme_hal_graph_vertexId_t vp1, vp2, vs;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_ncc_getPhysicalRoutes(&logicalRoutes, &deploymentGraph));

    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Physical Routes", &deploymentGraph);

    EXPECT_EQ(3, xme_hal_graph_getVertexCount(&deploymentGraph));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&deploymentGraph));

    vp1 = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &vp1d, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(vp1d.vertexId, vp1);

    vp2 = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &vp2d, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vp2);

    vs = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &vsd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vs);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&deploymentGraph));
    while (xme_hal_graph_hasNextVertex(&deploymentGraph))
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd = NULL;

        v = xme_hal_graph_nextVertex(&deploymentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&deploymentGraph, v, (void**) &vd));

        if (vs == vd->vertexId)
        {
            xme_core_nodeMgr_compRep_componentHandle_t componentHandle;
            xme_core_nodeMgr_compRep_portHandle_t portHandle;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp3, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_getComponentInstance(vd->nodeId, vd->vertexData.componentPortVertex.componentId, &componentHandle));
            ASSERT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, componentHandle);
            portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, 0u);
            ASSERT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, portHandle);
            EXPECT_EQ(5u, xme_core_nodeMgr_compRep_getQueueSize(portHandle));
        }
    }
    xme_hal_graph_finiVertexIterator(&deploymentGraph);
}

//----------------------------------------------------------------------------//
//     NetworkConfigurationCalculatorInterfaceTestForInterNodeDCC             //
//----------------------------------------------------------------------------//

TEST_F(NetworkConfigurationCalculatorInterfaceTestForInterNodeDCC, waypointTransform)
{
    xme_hal_graph_vertexId_t vp, vs1, vs2;
    uint8_t udpSendKey[XME_WP_UDP_HEADER_KEY_LENGTH] = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t udpReceiveKey[XME_WP_UDP_HEADER_KEY_LENGTH] = { 0xFE, 0xDC, 0xBA, 0x98 };

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_ncc_getPhysicalRoutes(&logicalRoutes, &deploymentGraph));

    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Physical Routes", &deploymentGraph);

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
        xme_core_pnp_dataLinkGraph_vertexData_t* vd = NULL;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed = NULL;

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
            EXPECT_EQ(comp1, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect two outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs2, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect zero incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
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
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs1, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
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
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs2, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect queue size
            EXPECT_EQ(42u, vd->vertexData.waypointMarshalerVertex.inputPortQueueSize);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vp, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);
                (void) xme_hal_mem_copy(&udpSendKey, ed->edgeData.udpLinkEdge.key, XME_WP_UDP_HEADER_KEY_LENGTH);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);
                (void) xme_hal_mem_copy(&udpReceiveKey, ed->edgeData.udpLinkEdge.key, XME_WP_UDP_HEADER_KEY_LENGTH);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(topic, vd->topicId);

            // Expect queue size
            EXPECT_EQ(12u, vd->vertexData.waypointDemarshalerVertex.inputPortQueueSize);

            // Expect one outgoing edge
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vs1, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(topic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);
            }
            EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else
        {
            // Unxpected vertex
            EXPECT_TRUE(false);
        }
    }
    xme_hal_graph_finiVertexIterator(&deploymentGraph);

    EXPECT_EQ(0, xme_hal_mem_compare(&udpSendKey, &udpReceiveKey, XME_WP_UDP_HEADER_KEY_LENGTH));
}

//----------------------------------------------------------------------------//
//     NetworkConfigurationCalculatorInterfaceTestForIntraNodeRR              //
//----------------------------------------------------------------------------//

TEST_F(NetworkConfigurationCalculatorInterfaceTestForIntraNodeRR, waypointTransform)
{
    xme_hal_graph_vertexId_t vrqs, vrqh, vrss, vrsh;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_ncc_getPhysicalRoutes(&logicalRoutes, &deploymentGraph));

    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Physical Routes", &deploymentGraph);

    EXPECT_EQ(6, xme_hal_graph_getVertexCount(&deploymentGraph));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&deploymentGraph));

    vrqs = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rqsd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrqs);

    vrqh = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rqhd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrqh);

    vrss = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rssd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrss);

    vrsh = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rshd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrsh);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&deploymentGraph));
    while (xme_hal_graph_hasNextVertex(&deploymentGraph))
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd = NULL;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed = NULL;

        v = xme_hal_graph_nextVertex(&deploymentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&deploymentGraph, v, (void**) &vd));

        if (vrqs == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(requestTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp1, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrqs, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // channel injector waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect zero incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vrqh == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(requestTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp2, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect no outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // channel injector waypoint

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrqh, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vrss == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(responseTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp2, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(1U, vd->vertexData.componentPortVertex.portIndex);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrss, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // channel selector waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect no incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vrsh == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(responseTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp1, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(1U, vd->vertexData.componentPortVertex.portIndex);

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // channel selector waypoint

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrsh, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(requestTopic, vd->topicId);
            EXPECT_EQ(chan1, vd->vertexData.waypointChannelInjectorVertex.injectedChannelId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrqh, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrqs, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(responseTopic, vd->topicId);
            EXPECT_EQ(chan1, vd->vertexData.waypointChannelSelectorVertex.sourceChannelId);
            EXPECT_EQ(chan2, vd->vertexData.waypointChannelSelectorVertex.destinationChannelId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrsh, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrss, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else
        {
            // Unxpected vertex
            EXPECT_TRUE(false);
        }
    }
    xme_hal_graph_finiVertexIterator(&deploymentGraph);
}

//----------------------------------------------------------------------------//
//     NetworkConfigurationCalculatorInterfaceTestForInterNodeRR              //
//----------------------------------------------------------------------------//

TEST_F(NetworkConfigurationCalculatorInterfaceTestForInterNodeRR, waypointTransform)
{
    xme_hal_graph_vertexId_t vrqs, vrqh, vrss, vrsh;
    uint8_t requestUDPSendKey[XME_WP_UDP_HEADER_KEY_LENGTH] = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t requestUDPReceiveKey[XME_WP_UDP_HEADER_KEY_LENGTH] = { 0xFE, 0xDC, 0xBA, 0x98 };
    uint8_t responseUDPSendKey[XME_WP_UDP_HEADER_KEY_LENGTH] = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t responseUDPReceiveKey[XME_WP_UDP_HEADER_KEY_LENGTH] = { 0xFE, 0xDC, 0xBA, 0x98 };

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_ncc_getPhysicalRoutes(&logicalRoutes, &deploymentGraph));

    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_ALWAYS, "Physical Routes", &deploymentGraph);

    EXPECT_EQ(14, xme_hal_graph_getVertexCount(&deploymentGraph));
    EXPECT_EQ(12, xme_hal_graph_getEdgeCount(&deploymentGraph));

    vrqs = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rqsd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrqs);

    vrqh = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rqhd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrqh);

    vrss = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rssd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrss);

    vrsh = xme_hal_graph_getNextVertexWithDataComparison(&deploymentGraph, &rshd, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vrsh);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&deploymentGraph));
    while (xme_hal_graph_hasNextVertex(&deploymentGraph))
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd = NULL;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed = NULL;

        v = xme_hal_graph_nextVertex(&deploymentGraph);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&deploymentGraph, v, (void**) &vd));

        if (vrqs == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(requestTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp1, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrqs, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // channel injector waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect zero incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vrqh == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(requestTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp2, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(0U, vd->vertexData.componentPortVertex.portIndex);

            // Expect no outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // demarshaler waypoint

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrqh, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vrss == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(responseTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp2, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(1U, vd->vertexData.componentPortVertex.portIndex);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrss, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // channel selector waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect no incoming edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (vrsh == vd->vertexId)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(responseTopic, vd->topicId);
            EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER, vd->vertexData.componentPortVertex.portType);
            EXPECT_EQ(comp1, vd->vertexData.componentPortVertex.componentId);
            EXPECT_EQ(1U, vd->vertexData.componentPortVertex.portIndex);

            // Expect zero outgoing edges
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // demarshaler waypoint

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrsh, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node1, vd->nodeId);
            EXPECT_EQ(requestTopic, vd->topicId);
            EXPECT_EQ(chan1, vd->vertexData.waypointChannelInjectorVertex.injectedChannelId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // marshaler waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(requestTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrqs, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            ASSERT_TRUE(requestTopic == vd->topicId || responseTopic == vd->topicId);
            EXPECT_EQ((requestTopic == vd->topicId) ? node1 : node2, vd->nodeId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // UDP send waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // channel injector waypoint

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            ASSERT_TRUE(requestTopic == vd->topicId || responseTopic == vd->topicId);
            EXPECT_EQ((requestTopic == vd->topicId) ? node1 : node2, vd->nodeId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.udpLinkEdge.topicId);
                (void) xme_hal_mem_copy((requestTopic == vd->topicId) ? &requestUDPSendKey : &responseUDPSendKey, ed->edgeData.udpLinkEdge.key, XME_WP_UDP_HEADER_KEY_LENGTH);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // UDP receive waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // marshaler waypoint

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            ASSERT_TRUE(requestTopic == vd->topicId || responseTopic == vd->topicId);
            EXPECT_EQ((requestTopic == vd->topicId) ? node2 : node1, vd->nodeId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // demarshaler waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.udpLinkEdge.topicId);
                (void) xme_hal_mem_copy((requestTopic == vd->topicId) ? &requestUDPReceiveKey : &responseUDPReceiveKey, ed->edgeData.udpLinkEdge.key, XME_WP_UDP_HEADER_KEY_LENGTH);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // UDP send

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            ASSERT_TRUE(requestTopic == vd->topicId || responseTopic == vd->topicId);
            EXPECT_EQ((requestTopic == vd->topicId) ? node2 : node1, vd->nodeId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ((requestTopic == vd->topicId) ? vrqh : vrsh, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(vd->topicId, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // UDP receive waypoint

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR == vd->vertexType)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_EQ(node2, vd->nodeId);
            EXPECT_EQ(responseTopic, vd->topicId);
            EXPECT_EQ(chan1, vd->vertexData.waypointChannelSelectorVertex.sourceChannelId);
            EXPECT_EQ(chan2, vd->vertexData.waypointChannelSelectorVertex.destinationChannelId);

            // Expect one outgoing edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextOutgoingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi); // marshaler waypoint

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&deploymentGraph, vd->vertexId));

            // Expect one incoming edge
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
            {
                EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&deploymentGraph, xme_hal_graph_nextIncomingEdge(&deploymentGraph, vd->vertexId), (void**) &ed));
                EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ed->edgeType);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ed->edgeId);
                EXPECT_EQ(responseTopic, ed->edgeData.memCopyEdge.topicId);

                vi = xme_hal_graph_getSourceVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vrss, vi);

                vi = xme_hal_graph_getSinkVertex(&deploymentGraph, ed->edgeId);
                EXPECT_EQ(vd->vertexId, vi);

                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&deploymentGraph, vd->vertexId));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&deploymentGraph, vd->vertexId));
        }
        else
        {
            // Unxpected vertex
            EXPECT_TRUE(false);
        }
    }
    xme_hal_graph_finiVertexIterator(&deploymentGraph);

    EXPECT_EQ(0, xme_hal_mem_compare(&requestUDPSendKey, &requestUDPReceiveKey, XME_WP_UDP_HEADER_KEY_LENGTH));
    EXPECT_EQ(0, xme_hal_mem_compare(&responseUDPSendKey, &responseUDPReceiveKey, XME_WP_UDP_HEADER_KEY_LENGTH));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
