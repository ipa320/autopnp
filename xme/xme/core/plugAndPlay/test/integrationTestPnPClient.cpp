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
 * $Id: integrationTestPnPClient.cpp 7802 2014-03-13 09:04:01Z geisinger $
 */

/**
 * \file
 *         Plug and Play Manager integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "xme/defines.h"

#include "xme/com/interface.h"

#include "xme/core/manifestRepository/include/manifestRepository.h"
#include "xme/core/node.h"
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpClientInterface.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"
#include "xme/core/plugAndPlay/test/mocks.h"
#include "xme/core/testUtils.h"
#include "xme/core/topic.h"
#include "xme/core/topicData.h"
#include "xme/hal/include/random.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

#include <inttypes.h>
#include <gtest/gtest.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

xme_status_t
componentWrapperReceivePort
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t componentInternalPortId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(componentInternalPortId);

    xme_core_pnp_test_componentWrapperReceivePortCallCount++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
componentInit
(
    void* const config,
    const char* initializationString
)
{
    XME_UNUSED_PARAMETER(config);
    XME_UNUSED_PARAMETER(initializationString);

    xme_core_pnp_test_componentInitCallCount++;

    return xme_core_pnp_test_componentInitStatus;
}

XME_EXTERN_C_END

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
/**
 * \class PnPClientIntegrationTestCommon
 *
 * \brief Common setUp and tearDown functions for initializing and finalizing
 *        required XME components, plus some utility functions for sub-classes.
 */
class PnPClientIntegrationTestCommon: public xme::testing::Test
{
protected:

    xme_core_componentType_t componentTypeSensor;
    xme_core_componentType_t componentTypeMonitor;

    xme_core_topic_t topic;

    typedef struct
    {
        uint32_t value;
    } topic_t;

    PnPClientIntegrationTestCommon()
    {
        topic = XME_CORE_TOPIC(6000);
        componentTypeSensor = (xme_core_componentType_t)5001u;
        componentTypeMonitor= (xme_core_componentType_t)5002u;
    }

    virtual void AssertionCheckedSetUp()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_pnp_pnpClient_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();
        xme_core_directory_attribute_fini();
    }

    void
    registerTopicsAndAttributes(void)
    {
        xme_core_attribute_descriptor_list_t attributeDescriptorList;
        xme_core_attribute_descriptor_t attributeDescriptors[1];

        attributeDescriptors[0].key = (xme_core_attribute_key_t) 0;
        attributeDescriptors[0].size = (size_t) 0;

        attributeDescriptorList.length = 1;
        attributeDescriptorList.element = attributeDescriptors;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_registerAttributeDescriptorList
        (
            topic,
            &attributeDescriptorList,
            false
        ));
    }

    void
    addComponentTypeSensorManifest(void)
    {
        xme_core_componentManifest_t componentManifest;
    
        // Create and add component type manifest for component 'sensor' to manifest repository
        // Sensor has a single function with a single required output port
        {
            xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));

            componentManifest.componentType = componentTypeSensor;
            componentManifest.componentWrapperReceivePort = &componentWrapperReceivePort;
            componentManifest.componentInit = &componentInit;

            {
                xme_core_functionManifest_t* functionManifest = &(componentManifest.functionManifests[0]);
                
                functionManifest->functionId = (xme_core_component_functionId_t)1;
                functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(200ull);
                functionManifest->completion = true;
                functionManifest->requiredPortIndicesLength = 1;
                functionManifest->requiredPortIndices[0] = 0;
                functionManifest->optionalPortIndicesLength = 0;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[0]);
            
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                portManifest->topic = topic;
                portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                portManifest->queueSize = 1u;
            }

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }
    
    void
    addComponentTypeMonitorManifest(void)
    {
        xme_core_componentManifest_t componentManifest;

        // Create and add component type manifest for component 'monitor' to manifest repository
        // Monitor has a single function with a single required input port (queue size 12)
        {
            xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));

            componentManifest.componentType = componentTypeMonitor;
            componentManifest.componentWrapperReceivePort = &componentWrapperReceivePort;
            componentManifest.componentInit = &componentInit;

            {
                xme_core_functionManifest_t* functionManifest = &(componentManifest.functionManifests[0]);
                
                functionManifest->functionId = (xme_core_component_functionId_t)1;
                functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(200ull);
                functionManifest->completion = true;
                functionManifest->requiredPortIndicesLength = 1;
                functionManifest->requiredPortIndices[0] = 0;
                functionManifest->optionalPortIndicesLength = 0;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[0]);
            
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                portManifest->lowerConnectionBound = 0;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                portManifest->topic = topic;
                portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                portManifest->queueSize = 12u;
            }

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }
};

/**
 * \class PnPClientIntegrationTestMultiNode
 *
 * \brief Three registered nodes and components instantiated on these nodes
 *        for testing complex runtime graph processing.
 */
class PnPClientIntegrationTestMultiNode: public PnPClientIntegrationTestCommon
{
protected:
    xme_core_component_t invalidComponentId;

    xme_core_node_nodeId_t invalidNodeId;
    xme_core_node_nodeId_t localNodeId;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeId3;

    xme_core_componentType_t invalidComponentType;

    xme_core_nodeMgr_compRep_componentHandle_t componentHandle1;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle2;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle3;

    xme_core_topic_pnpManager_runtime_graph_model_t outGraph;

    PnPClientIntegrationTestMultiNode()
    : invalidComponentId(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT)
    , invalidNodeId(XME_CORE_NODE_INVALID_NODE_ID)
    , localNodeId((xme_core_node_nodeId_t)1)
    , nodeId2((xme_core_node_nodeId_t)2)
    , nodeId3((xme_core_node_nodeId_t)3)
    , invalidComponentType(XME_CORE_COMPONENT_TYPE_INVALID)
    {
    }

    virtual ~PnPClientIntegrationTestMultiNode()
    {
    }

    virtual void AssertionCheckedSetUp()
    { 
        PnPClientIntegrationTestCommon::AssertionCheckedSetUp();

        xme_core_pnp_test_majorCycleLength = xme_hal_time_timeIntervalFromMilliseconds(10000ull);

        registerNodes();
        registerTopicsAndAttributes();
        addComponentTypeSensorManifest();
        addComponentTypeMonitorManifest();
        createComponents();
    }

    virtual void AssertionCheckedTearDown()
    {
        PnPClientIntegrationTestCommon::AssertionCheckedTearDown();
    }

    void
    createComponents(void)
    {
        xme_core_nodeMgr_compRep_componentBuilder_t* builder;

        builder = xme_core_nodeMgr_compRep_createBuilder(localNodeId, componentTypeSensor);
        ASSERT_NE((void*)NULL, builder);
        xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, (xme_core_component_t)1); // Usually this is done in announceComponentInstanceManifest, but we skip this here
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle1));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_plugInNewComponent(componentHandle1));
        
        builder = xme_core_nodeMgr_compRep_createBuilder(nodeId2, componentTypeMonitor);
        ASSERT_NE((void*)NULL, builder);
        xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, (xme_core_component_t)2);
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle2));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_plugInNewComponent(componentHandle2));

        builder = xme_core_nodeMgr_compRep_createBuilder(nodeId3, componentTypeSensor);
        ASSERT_NE((void*)NULL, builder);
        xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, (xme_core_component_t)3);
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle3));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_plugInNewComponent(componentHandle3));
    }

    void
    registerNodes(void)
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(localNodeId, "localNodeId", (xme_core_node_guid_t)1));
        {
            xme_com_interface_address_t nodeInterface;

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &nodeInterface));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(localNodeId, nodeInterface));
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(localNodeId));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId2, "nodeId2", (xme_core_node_guid_t)2));
        {
            xme_com_interface_address_t nodeInterface;

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33222", &nodeInterface));

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId2, nodeInterface));
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId2));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId3, "nodeId3", (xme_core_node_guid_t)3));
        {
            xme_com_interface_address_t nodeInterface;

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33223", &nodeInterface));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId3, nodeInterface));
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId3));
    }
};

/**
 * \class PnPClientIntegrationTestLocalNode
 *
 * \brief One monitor will already be built and set as announced
 *        (but not created or registered).
 *        By that we simulate that the pnpClient send an instance manifest
 *        for that component.
 *
 *        We also prepare a runtime graph with two sensors and
 *        one monitor (which refers to the existing one) for the local node.
 */
class PnPClientIntegrationTestLocalNode: public PnPClientIntegrationTestCommon
{
protected:
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle1;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle2;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle3;

    xme_core_component_t componentID1;
    xme_core_component_t componentID2;
    xme_core_component_t componentID3;

    xme_hal_time_timeInterval_t component1Function0ExecutionPeriod;
    xme_hal_time_timeInterval_t component2Function0ExecutionPeriod;
    xme_hal_time_timeInterval_t component3Function0ExecutionPeriod;

    uint8_t component1Port0QueueSize;
    uint8_t component2Port0QueueSize;
    uint8_t component3Port0QueueSize;

    xme_core_topic_pnpManager_runtime_graph_model_t runtimeGraphModel;

    PnPClientIntegrationTestLocalNode()
    {
    }

    virtual ~PnPClientIntegrationTestLocalNode()
    {
    }

    virtual void AssertionCheckedSetUp()
    { 
        PnPClientIntegrationTestCommon::AssertionCheckedSetUp();

        xme_core_pnp_test_majorCycleLength = xme_hal_time_timeIntervalFromMilliseconds(10000ull);

        xme_core_node_setCurrentNodeId(XME_CORE_NODE_LOCAL_NODE_ID);

        resetPeriodDividers();
        registerTopicsAndAttributes();
        addComponentTypeMonitorManifest();
        addComponentTypeSensorManifest();
        createRuntimeGraph();
        createComponents(); // Must be called after createRuntimeGraph()
    }

    virtual void AssertionCheckedTearDown()
    {
        PnPClientIntegrationTestCommon::AssertionCheckedTearDown();
    }

    void
    createComponents()
    {
        xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
        xme_core_nodeMgr_compRep_componentHandle_t monitorHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;

        // The node will already have an announced entry for the monitor
        // We need to make sure that it matches the information from the runtime graph
        builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentTypeMonitor);
        ASSERT_NE((void*)NULL, builder);
        xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 0u, component3Port0QueueSize);
        xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, (xme_core_nodeMgr_compRep_componentHandle_t)1);
        xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, component3Function0ExecutionPeriod);
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &monitorHandle));
    
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(monitorHandle));

        ASSERT_EQ((xme_core_nodeMgr_compRep_componentHandle_t)1, monitorHandle) << "If this assertion fails you might need to update the foreign handle in the test\n.";

        runtimeGraphModel.vertex[2].componentHandle = monitorHandle;
    }

    void
    resetPeriodDividers(void)
    {
        xme_hal_mem_set(xme_core_pnp_test_periodDividers, 0u, sizeof(xme_core_pnp_test_periodDividers));
    }

    void
    createRuntimeGraph(void)
    {
        componentID1 = (xme_core_component_t)8001;
        componentID2 = (xme_core_component_t)8002;
        componentID3 = (xme_core_component_t)8003;

        component1Function0ExecutionPeriod = xme_core_pnp_test_majorCycleLength - 1ull;
        component2Function0ExecutionPeriod = 2ull * xme_core_pnp_test_majorCycleLength;
        component3Function0ExecutionPeriod = 3ull * xme_core_pnp_test_majorCycleLength - 1;

        component1Port0QueueSize = 2u;
        component2Port0QueueSize = 3u;
        component3Port0QueueSize = 4u;

        xme_hal_mem_set(&runtimeGraphModel, 0u, sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

        runtimeGraphModel.action = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD;
        runtimeGraphModel.nodeId = XME_CORE_NODE_LOCAL_NODE_ID;
        
        {
            uint8_t i = 0;

            runtimeGraphModel.vertex[i].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
            runtimeGraphModel.vertex[i].componentId = componentID1;
            runtimeGraphModel.vertex[i].componentType = componentTypeSensor;
            runtimeGraphModel.vertex[i].functionData[0].executionPeriod = component1Function0ExecutionPeriod;
            runtimeGraphModel.vertex[i].portData[0].queueSize = component1Port0QueueSize;
            xme_hal_safeString_strncpy(runtimeGraphModel.vertex[i].vertexData, "", sizeof(runtimeGraphModel.vertex[i].vertexData));

            ASSERT_NE(0,
                xme_hal_safeString_snprintf
                (
                    runtimeGraphModel.vertex[i].vertexData,
                    sizeof(runtimeGraphModel.vertex[i].vertexData),
                    "%u|%s",
                    XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    ""
                )
            );
        }

        {
            uint8_t i = 1;

            runtimeGraphModel.vertex[i].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
            runtimeGraphModel.vertex[i].componentId = componentID2;
            runtimeGraphModel.vertex[i].componentType = componentTypeSensor;
            runtimeGraphModel.vertex[i].functionData[0].executionPeriod = component2Function0ExecutionPeriod;
            runtimeGraphModel.vertex[i].portData[0].queueSize = component2Port0QueueSize;

            ASSERT_NE(0,
                xme_hal_safeString_snprintf
                (
                    runtimeGraphModel.vertex[i].vertexData,
                    sizeof(runtimeGraphModel.vertex[i].vertexData),
                    "%u|%s",
                    XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    ""
                )
            );
        }

        {
            uint8_t i = 2;

            runtimeGraphModel.vertex[i].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
            runtimeGraphModel.vertex[i].componentId = componentID3;
            runtimeGraphModel.vertex[i].componentType = componentTypeMonitor;
            runtimeGraphModel.vertex[i].functionData[0].executionPeriod = component3Function0ExecutionPeriod;
            runtimeGraphModel.vertex[i].portData[0].queueSize = component3Port0QueueSize;
            xme_hal_safeString_strncpy(runtimeGraphModel.vertex[i].vertexData, "", sizeof(runtimeGraphModel.vertex[i].vertexData));

            ASSERT_NE(0,
                xme_hal_safeString_snprintf
                (
                    runtimeGraphModel.vertex[i].vertexData,
                    sizeof(runtimeGraphModel.vertex[i].vertexData),
                    "%u|%s",
                    XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    ""
                ));
        }

        {
            uint8_t i = 0;

            runtimeGraphModel.edge[i].edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;
            runtimeGraphModel.edge[i].srcVertexIndex = 1u;
            runtimeGraphModel.edge[i].sinkVertexIndex = 3u;
            ASSERT_NE(0,
                xme_hal_safeString_snprintf
                (
                    runtimeGraphModel.edge[i].edgeData,
                    sizeof(runtimeGraphModel.edge[i].edgeData),
                    "%"PRIu32"|%"PRIu16"|%"PRIu32,
                    topic,
                    sizeof(topic_t),
                    (xme_core_channelId_t)1000
                ));
        }

        {
            uint8_t i = 1;

            runtimeGraphModel.edge[i].edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;
            runtimeGraphModel.edge[i].srcVertexIndex = 2u;
            runtimeGraphModel.edge[i].sinkVertexIndex = 3u;
            ASSERT_NE(0,
                xme_hal_safeString_snprintf
                (
                    runtimeGraphModel.edge[i].edgeData,
                    sizeof(runtimeGraphModel.edge[i].edgeData),
                    "%"PRIu32"|%"PRIu16"|%"PRIu32,
                    topic,
                    sizeof(topic_t),
                    (xme_core_channelId_t)1001
                ));
        }
    }
};

class PnPClientPlugOutIntegrationTest: public xme::testing::Test
{
    // 
    // Initial Configuration
    // --------------------------------------------------------------------------------------------------------
    // | Node | ComponentID | PortType   | ComponentType | Topic    | TargetComponentIDs | SourceComponentIDs |
    // --------------------------------------------------------------------------------------------------------
    // |    1 |           1 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C4 |                    |
    // |    1 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C6 |
    // --------------------------------------------------------------------------------------------------------
    // |    2 |           3 | PUBLISHER  |   CT_USER + 3 | USER + 2 |               N3C5 |                    |
    // |    2 |           4 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C6 |
    // --------------------------------------------------------------------------------------------------------
    // |    3 |           5 | SUBSCRIBER |   CT_USER + 4 | USER + 2 |                    |               N2C3 |
    // |    3 |           6 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C4 |                    |
    // --------------------------------------------------------------------------------------------------------
    // 
    // Operations: 
    // - Remove component N3C5 (Subscriber of topic 2). 
    //
    // --------------------------------------------------------------------------------------------------------
    // | Node | ComponentID | PortType   | ComponentType | Topic    | TargetComponentIDs | SourceComponentIDs |
    // --------------------------------------------------------------------------------------------------------
    // |    1 |           1 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C4 |                    |
    // |    1 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C6 |
    // --------------------------------------------------------------------------------------------------------
    // |    2 |           3 | PUBLISHER  |   CT_USER + 3 | USER + 2 |               N3C5 |                    | <-- remove graph
    // |    2 |           4 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C6 |
    // --------------------------------------------------------------------------------------------------------
    // |  *3* |         *5* | SUBSCRIBER |   CT_USER + 4 | USER + 2 |                    |               N2C3 | <-- remove graph
    // |    3 |           6 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C4 |                    |
    // --------------------------------------------------------------------------------------------------------
    //
    // - Remove component N3C5 (Publisher of topic 1). 
    //
    // --------------------------------------------------------------------------------------------------------
    // | Node | ComponentID | PortType   | ComponentType | Topic    | TargetComponentIDs | SourceComponentIDs |
    // --------------------------------------------------------------------------------------------------------
    // |    1 |           1 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C4 |                    |
    // |    1 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C6 | <-- remove graph (1 link to N3C2)
    // --------------------------------------------------------------------------------------------------------
    // |    - |           - |      -     |             - |        - |                  - |                  - |
    // |    2 |           4 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C6 | <-- remove graph (1 link to N3C2)
    // --------------------------------------------------------------------------------------------------------
    // |    - |           - |      -     |             - |        - |                  - |                  - |
    // |  *3* |         *6* | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C4 |                    | <-- remove graph (2 links)
    // --------------------------------------------------------------------------------------------------------
    
#define COMPONENTTYPE1 (XME_CORE_COMPONENT_TYPE_USER + 1U)
#define COMPONENTTYPE2 (XME_CORE_COMPONENT_TYPE_USER + 2U)
#define COMPONENTTYPE3 (XME_CORE_COMPONENT_TYPE_USER + 3U)
#define COMPONENTTYPE4 (XME_CORE_COMPONENT_TYPE_USER + 4U)

protected:
    PnPClientPlugOutIntegrationTest()
    {
        // Declaration of three nodes. 
        invalidNodeId = (xme_core_node_nodeId_t) XME_CORE_NODE_INVALID_NODE_ID;
        localNodeId = (xme_core_node_nodeId_t) XME_CORE_NODE_LOCAL_NODE_ID;
        nodeId2 = (xme_core_node_nodeId_t) 2U;
        nodeId3 = (xme_core_node_nodeId_t) 3U;

        // Declaration of the two user topics. 
        userTopic1 = XME_CORE_TOPIC((uint16_t)XME_CORE_TOPIC_USER + 1U);
        userTopic2 = XME_CORE_TOPIC((uint16_t)XME_CORE_TOPIC_USER + 2U);

        // Declaration of components
        invalidComponentId= (xme_core_component_t) XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;
        componentId1 = (xme_core_component_t) 1U;
        componentId2 = (xme_core_component_t) 2U;
        componentId3 = (xme_core_component_t) 3U;
        componentId4 = (xme_core_component_t) 4U;
        componentId5 = (xme_core_component_t) 5U;
        componentId6 = (xme_core_component_t) 6U;

        // Declaration of component types (one subscriber and publisher per topic). 
        invalidComponentType = (xme_core_componentType_t) XME_CORE_COMPONENT_TYPE_INVALID;
        componentTypePublisherTopic1 = (xme_core_componentType_t) COMPONENTTYPE1;
        componentTypeSubscriberTopic1 = (xme_core_componentType_t) COMPONENTTYPE2;
        componentTypePublisherTopic2 = (xme_core_componentType_t) COMPONENTTYPE3;
        componentTypeSubscriberTopic2 = (xme_core_componentType_t) COMPONENTTYPE4;

        functionIdentifier = 1U;
    }

    virtual ~PnPClientPlugOutIntegrationTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    { 
        // Init all needed components: manifest repository, logical route manager, plug and play manager,
        // node registry controller and component repository. 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());

        // Generate three different generated IDs for register nodes. 
        guid1 = (xme_core_node_guid_t) xme_hal_random_rand();
        guid2 = (xme_core_node_guid_t) (((uint64_t)guid1) - 1);
        guid3 = (xme_core_node_guid_t) (((uint64_t)guid1) + 1);

        // Register the topics and attributes. 
        registerTopicsAndAttributes(userTopic1);
        registerTopicsAndAttributes(userTopic2);

        // Register the nodes
        // Node 1 (Local Node)
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(localNodeId, "localNodeId", guid1));
        {
            xme_com_interface_address_t nodeInterface;
            
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &nodeInterface));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(localNodeId, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(localNodeId));

        // Node 2
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId2, "nodeId2", guid2));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33222", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId2, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId2));

        // Node 3
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId3, "nodeId3", guid3));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33223", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId3, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId3));

        // Reserve memory for the runtime graph. 
        outGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));
        ASSERT_NE((void*)NULL, outGraph);

        // Create component manifests. 
        createComponentTypeManifests();

        // Announce manifests. 
        announceManifests();

        // Consume announcements runtime graphs. 
        consumeRuntimeGraphs();

        // Establish all the routes. 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();
        xme_core_directory_attribute_fini();

    }

    void
    createComponentTypeManifests(void)
    {
        xme_core_componentManifest_t componentManifest;
    
        // Create and add component type manifest for component 'publisher of topic 1' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypePublisherTopic1));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    
        // Create and add component type manifest for component 'subscriber of topic 1' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypeSubscriberTopic1));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }

        // Create and add component type manifest for component 'publisher of topic 2' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypePublisherTopic2));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    
        // Create and add component type manifest for component 'subscriber of topic 2' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypeSubscriberTopic2));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }

    void
    announceManifests(void)
    {
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

        // Announce components in local node.
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(localNodeId, componentId1, componentTypePublisherTopic1));
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(localNodeId, componentId2, componentTypeSubscriberTopic1));

        // Announce components in node 2.
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId2, componentId3, componentTypePublisherTopic2));
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId2, componentId4, componentTypeSubscriberTopic1));

        // Announce components in node 3. 
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId3, componentId5, componentTypeSubscriberTopic2));
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId3, componentId6, componentTypePublisherTopic1));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateConfiguration());
    }

    xme_status_t
    announceNewComponentInPnPManager
    (
        xme_core_node_nodeId_t nodeID,
        xme_core_component_t componentID,
        xme_core_componentType_t componentType
    )
    {
        xme_status_t status = XME_STATUS_INTERNAL_ERROR;
        xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
        xme_core_nodeMgr_compRep_componentHandle_t componentHandle;

        XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);

        builder = xme_core_nodeMgr_compRep_createBuilder(nodeID, componentType);
        XME_CHECK(NULL != builder, XME_STATUS_INTERNAL_ERROR);
        xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID);
        xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, componentID);
        status = xme_core_nodeMgr_compRep_build(builder, &componentHandle);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        return xme_core_pnp_pnpManager_announceNewComponentOnNode(componentHandle);
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
        componentManifest->componentWrapperReceivePort = &componentWrapperReceivePort;
        componentManifest->componentInit = &componentInit;

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
                functionManifest->functionId = (xme_core_component_functionId_t) functionIdentifier++;
                functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(200);
                functionManifest->alphaCurve.alphaCurve = 0;
                functionManifest->completion = true;
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT; i ++)
                {
                    componentManifest->functionManifests[i].functionId = (xme_core_component_functionId_t)0;
                    componentManifest->functionManifests[i].wcet = xme_hal_time_timeIntervalFromMilliseconds(0);
                    componentManifest->functionManifests[i].alphaCurve.alphaCurve = 0;
                    componentManifest->functionManifests[i].completion = true;
                    componentManifest->functionManifests[i].requiredPortIndicesLength = 1;
                    componentManifest->functionManifests[i].requiredPortIndices[0] = 0;
                    componentManifest->functionManifests[i].optionalPortIndicesLength = 0;
                }
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
                switch(componentType)
                {
                    case (xme_core_componentType_t) COMPONENTTYPE1:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                        portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic1;
                        portManifest->queueSize = 1u;
                        break;
                    case (xme_core_componentType_t) COMPONENTTYPE2:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                        portManifest->lowerConnectionBound = 0;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic1;
                        portManifest->queueSize = 1u;
                        break;
                    case (xme_core_componentType_t) COMPONENTTYPE3:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                        portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic2;
                        portManifest->queueSize = 1u;
                        break;
                    case (xme_core_componentType_t) COMPONENTTYPE4:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                        portManifest->lowerConnectionBound = 0;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic2;
                        portManifest->queueSize = 1u;
                        break;
                    default:
                        break;
                }
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT; i ++)
                {
                    componentManifest->portManifests[i].portType = XME_CORE_COMPONENT_PORTTYPE_INVALID;
                    componentManifest->portManifests[i].topic = XME_CORE_TOPIC_INVALID_TOPIC;
                    componentManifest->portManifests[i].lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    componentManifest->portManifests[i].upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    componentManifest->portManifests[i].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                }
            }
        }
    
        return XME_STATUS_SUCCESS;
    }

    void
    registerTopicsAndAttributes
    (
        xme_core_topic_t topicID
    )
    {
        xme_core_attribute_descriptor_list_t attributeDescriptorList;
        xme_core_attribute_descriptor_t attributeDescriptors[1];

        attributeDescriptors[0].key = (xme_core_attribute_key_t) 0;
        attributeDescriptors[0].size = (size_t) 0;

        attributeDescriptorList.length = 1;
        attributeDescriptorList.element = attributeDescriptors;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_registerAttributeDescriptorList
        (
            topicID,
            &attributeDescriptorList,
            false
        ));
    }

    void
    consumeRuntimeGraphs(void)
    {
        uint8_t i = 0U;

        // Consume all runtime graphs. 
        while (xme_core_pnp_pnpManager_hasNewRuntimeGraphs())
        {
            char isShutdownDesired;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(outGraph));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
            i++;
        }

        EXPECT_EQ(3U, i);
    }

    xme_core_component_t invalidComponentId;
    xme_core_component_t componentId1;
    xme_core_component_t componentId2;
    xme_core_component_t componentId3;
    xme_core_component_t componentId4;
    xme_core_component_t componentId5;
    xme_core_component_t componentId6;

    xme_core_node_nodeId_t invalidNodeId;
    xme_core_node_nodeId_t localNodeId;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeId3;

    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;
    xme_core_node_guid_t guid3;

    xme_core_componentType_t invalidComponentType;
    xme_core_componentType_t componentTypePublisherTopic1;
    xme_core_componentType_t componentTypeSubscriberTopic1;
    xme_core_componentType_t componentTypePublisherTopic2;
    xme_core_componentType_t componentTypeSubscriberTopic2;

    xme_core_topic_t userTopic1;
    xme_core_topic_t userTopic2;

    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph;

    uint8_t functionIdentifier;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

TEST_F(PnPClientIntegrationTestMultiNode, processGraph)
{
    char isShutdownDesired;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpClient_processGraph(NULL, &isShutdownDesired));

    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, &outGraph)); // N1 publication for N2.

    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION,
        xme_core_pnp_pnpClient_processGraph(&outGraph, &isShutdownDesired));

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 1);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(&outGraph, &isShutdownDesired));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, &outGraph)); // N1 publication for N2.

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 2);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(&outGraph, &isShutdownDesired));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, &outGraph)); // N1 publication for N2.

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 3);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(&outGraph, &isShutdownDesired));

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 1);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(&outGraph, &isShutdownDesired));
}

TEST_F(PnPClientIntegrationTestMultiNode, logoutAPI)
{
    xme_hal_singlyLinkedList_t(10) tmpList;
    XME_HAL_SINGLYLINKEDLIST_INIT(tmpList);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_logoutNode((xme_core_node_nodeId_t)10u));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_logoutNode((xme_core_node_nodeId_t)2u));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_logoutNode((xme_core_node_nodeId_t)20u));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_getLoggedoutNodes(&tmpList));
    EXPECT_EQ(10u, (uint16_t)(uintptr_t)xme_hal_singlyLinkedList_itemFromIndex((void*)&(tmpList), 0));
    EXPECT_EQ(2u, (uint16_t)(uintptr_t)xme_hal_singlyLinkedList_itemFromIndex((void*)&(tmpList), 1));
    EXPECT_EQ(20u, (uint16_t)(uintptr_t)xme_hal_singlyLinkedList_itemFromIndex((void*)&(tmpList), 2));
    XME_HAL_SINGLYLINKEDLIST_FINI(tmpList);
}

TEST_F(PnPClientIntegrationTestLocalNode, processGraphSensorMonitorOnLocalNode)
{
    char isShutdownDesired;

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(&runtimeGraphModel, &isShutdownDesired));

    // Check if port/function-instance data was processed correctly
    xme_core_nodeMgr_compRep_componentIteratorInit();
    while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
    {
        xme_core_nodeMgr_compRep_componentHandle_t componentHandle =
            xme_core_nodeMgr_compRep_componentIteratorNext();
        xme_core_component_t componentID =
            xme_core_nodeMgr_compRep_getComponentID(componentHandle);
        xme_core_nodeMgr_compRep_portHandle_t portHandle =
                xme_core_nodeMgr_compRep_getPort(componentHandle, 0u); // Every component has only a single port
        xme_core_nodeMgr_compRep_portHandle_t functionHandle =
                xme_core_nodeMgr_compRep_getFunction(componentHandle, 0u); // Every component has only a single port

        if (componentID1 == componentID)
        {
            EXPECT_EQ(component1Port0QueueSize, xme_core_nodeMgr_compRep_getQueueSize(portHandle));
            EXPECT_EQ(component1Function0ExecutionPeriod, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle));
        }
        else if (componentID2 == componentID)
        {
            EXPECT_EQ(component2Port0QueueSize, xme_core_nodeMgr_compRep_getQueueSize(portHandle));
            EXPECT_EQ(component2Function0ExecutionPeriod, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle));
        }
        else if (componentID3 == componentID)
        {
            EXPECT_EQ(component3Port0QueueSize, xme_core_nodeMgr_compRep_getQueueSize(portHandle));
            EXPECT_EQ(component3Function0ExecutionPeriod, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle));
        }
        else
        {
            EXPECT_FALSE(true) << "Unknown component with ID " << componentID << ". Please add a check for this type in the unit test.";
        }
    }
    xme_core_nodeMgr_compRep_componentIteratorFini();

    // Check if the period dividers have been calculated correctly
    {
        uint16_t i = 0u;

        for (i = 0u; i < sizeof(xme_core_pnp_test_periodDividers) / sizeof(xme_core_pnp_test_periodDividers[0]); i++)
        {
            if (0u == xme_core_pnp_test_periodDividers[i].componentID) { break; }
            
            if (componentID1 == xme_core_pnp_test_periodDividers[i].componentID)
            {
                EXPECT_EQ(0u, xme_core_pnp_test_periodDividers[i].periodDivider);
            }
            else if (componentID2 == xme_core_pnp_test_periodDividers[i].componentID)
            {
                EXPECT_EQ(2u, xme_core_pnp_test_periodDividers[i].periodDivider);
            }
            else if (componentID3 == xme_core_pnp_test_periodDividers[i].componentID)
            {
                EXPECT_EQ(2u, xme_core_pnp_test_periodDividers[i].periodDivider);
            }
            else
            {
                ASSERT_FALSE(true) << "Unknown component with ID " << xme_core_pnp_test_periodDividers[i].componentID << ". Please add a check for this type in the unit test.";
            }
        }
    }
}

TEST_F(PnPClientIntegrationTestLocalNode, checkComponentEntryReuse)
{
    char isShutdownDesired;
    uint32_t componentCount = 0u;

    xme_core_nodeMgr_compRep_componentIteratorInit();
    while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
    {
        (void)xme_core_nodeMgr_compRep_componentIteratorNext();

        componentCount++;
    }
    xme_core_nodeMgr_compRep_componentIteratorFini();

    EXPECT_EQ(1u, componentCount);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(&runtimeGraphModel, &isShutdownDesired));

    componentCount = 0u;
    xme_core_nodeMgr_compRep_componentIteratorInit();
    while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
    {
        (void)xme_core_nodeMgr_compRep_componentIteratorNext();

        componentCount++;
    }
    xme_core_nodeMgr_compRep_componentIteratorFini();

    EXPECT_EQ(3u, componentCount); // Three components in runtime graph, which means we reused one existing entry
}

TEST_F(PnPClientIntegrationTestLocalNode, instanceManifestConstruction)
{
    // Set queue size and execution period of a component and check if the
    // resulting instance manifest is set correctly
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle;
    xme_core_topic_pnp_componentInstanceManifest_t instanceManfiest;

    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentTypeMonitor);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, xme_hal_time_timeIntervalFromMilliseconds(111ull));
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 0u, 11u);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_plugInNewComponent(componentHandle));

    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentTypeSensor);
    ASSERT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, xme_hal_time_timeIntervalFromMilliseconds(222ull));
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 0u, 22u);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_plugInNewComponent(componentHandle));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_getManifest(&instanceManfiest, false));

    {
        uint16_t i = 0u;
        bool monitorChecked = false;
        bool sensorChecked = false;

        for (i = 0u; i < sizeof(instanceManfiest.components) / sizeof(instanceManfiest.components[0]); i++)
        {
            if (instanceManfiest.components[i].componentType == componentTypeMonitor)
            {
                EXPECT_FALSE(monitorChecked);
                EXPECT_EQ(xme_hal_time_timeIntervalFromMilliseconds(111ull), instanceManfiest.components[i].functionData[0].executionPeriod);
                EXPECT_EQ(11u, instanceManfiest.components[i].portData[0].queueSize);
                monitorChecked = true;
            }
            else if (instanceManfiest.components[i].componentType == componentTypeSensor)
            {
                EXPECT_FALSE(sensorChecked);
                EXPECT_EQ(xme_hal_time_timeIntervalFromMilliseconds(222ull), instanceManfiest.components[i].functionData[0].executionPeriod);
                EXPECT_EQ(22u, instanceManfiest.components[i].portData[0].queueSize);
                sensorChecked = true;
            }
        }

        EXPECT_TRUE(monitorChecked);
        EXPECT_TRUE(sensorChecked);
    }
}

TEST_F(PnPClientPlugOutIntegrationTest, RemoveComponentTest_Issue3953)
{
    char isShutdownDesired = false;

    // Remove first component from node 3 (componentID=5). 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceComponentOnNode(nodeId3, componentId5));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph));

    // PnPClient: process remove graph from node 2. 
    xme_core_node_setCurrentNodeId(localNodeId);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191
    xme_core_node_setCurrentNodeId(nodeId2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    xme_core_node_setCurrentNodeId(nodeId3);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191

    // FIXME: Issue #3953. Add checks to the status of local node, node2 and node3

    // Subgraph for node 3. 
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph));

    // PnPClient: process remove graph from node 3. 
    xme_core_node_setCurrentNodeId(localNodeId);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191
    xme_core_node_setCurrentNodeId(nodeId2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191
    xme_core_node_setCurrentNodeId(nodeId3);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));

    // FIXME: Issue #3953. Add checks to the status of local node, node2 and node3

    // Remove the instance from PnPManager. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeComponentInstance(nodeId3, componentId5));

    // Remove second component from node 3 (componentID=6). 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceComponentOnNode(nodeId3, componentId6));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph));

    // PnPClient: process remove graph from local node. 
    xme_core_node_setCurrentNodeId(localNodeId);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    xme_core_node_setCurrentNodeId(nodeId2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191
    xme_core_node_setCurrentNodeId(nodeId3);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191

    // FIXME: Issue #3953. Add checks to the status of local node, node2 and node3

    // Obtain the runtime graph for node 2. 
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph));
    
    // PnPClient: process remove graph from node 2.
    xme_core_node_setCurrentNodeId(localNodeId);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191
    xme_core_node_setCurrentNodeId(nodeId2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    xme_core_node_setCurrentNodeId(nodeId3);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191

    // FIXME: Issue #3953. Add checks to the status of local node, node2 and node3

    // Obtain the runtime graph for node 3. 
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph));
    
    // PnPClient: process remove graph from node 3.
    xme_core_node_setCurrentNodeId(localNodeId);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191
    xme_core_node_setCurrentNodeId(nodeId2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));
    //ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired)); #Issue 4191
    xme_core_node_setCurrentNodeId(nodeId3);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_processGraph(outGraph, &isShutdownDesired));

    // FIXME: Issue #3953. Add checks to the status of local node, node2 and node3

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    // Remove the instance. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeComponentInstance(nodeId3, componentId6));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
