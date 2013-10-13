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
 * $Id: integrationTestPnPClient.cpp 5188 2013-09-26 12:39:27Z camek $
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
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "xme/defines.h"
#include "xme/com/interface.h"
#include "xme/core/node.h"
#include "xme/core/topic.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

#include "xme/core/topicData.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PnPClientIntegrationTest: public ::testing::Test
{
protected:
    PnPClientIntegrationTest()
    : invalidComponentId(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT)
    , componentId1((xme_core_component_t)1)
    , componentId2((xme_core_component_t)2)
    , componentId3((xme_core_component_t)3)
    , componentId4((xme_core_component_t)4)
    , componentId5((xme_core_component_t)5)
    , invalidNodeId(XME_CORE_NODE_INVALID_NODE_ID)
    , localNodeId((xme_core_node_nodeId_t)1)
    , nodeId2((xme_core_node_nodeId_t)2)
    , nodeId3((xme_core_node_nodeId_t)3)
    , invalidComponentType(XME_CORE_COMPONENT_TYPE_INVALID)
    , componentTypeSensor(XME_CORE_COMPONENT_TYPE_SENSOR)
    , componentTypeMonitor(XME_CORE_COMPONENT_TYPE_MONITOR)
    {
    }

    virtual ~PnPClientIntegrationTest()
    {
    }

    virtual void SetUp()
    { 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
        createComponentTypeManifests();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

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

        outGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

        ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, invalidComponentId));
        ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, nodeId2, invalidComponentId));
        ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId3, invalidComponentId));

        ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_pnp_pnpClient_init());
    }

    virtual void TearDown()
    {
        xme_core_pnp_pnpClient_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();
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
                        "%s:%d Component type 'monitor' defines more functions (%d) than can be stored in the manifest data structure (%d).\n",
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

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT; i ++)
                {
                    componentManifest->functionManifests[i].functionId = (xme_core_component_functionId_t)0;
                    componentManifest->functionManifests[i].wcet = xme_hal_time_timeIntervalFromMilliseconds(0);
                    componentManifest->functionManifests[i].alphaCurve.alphaCurve = 0;
                    componentManifest->functionManifests[i].completion = true;
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
                        "%s:%d Component type 'monitor' defines more ports (%d) than can be stored in the manifest data structure (%d).\n",
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
                if (componentType == XME_CORE_COMPONENT_TYPE_MONITOR) // monitor
                {
                    portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                } 
                else
                {
                    portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                }
                portManifest->topic = XME_CORE_TOPIC(4098); 
                portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT; i ++)
                {
                    componentManifest->portManifests[i].portType = XME_CORE_COMPONENT_PORTTYPE_INVALID;
                    componentManifest->portManifests[i].topic = XME_CORE_TOPIC_INVALID_TOPIC;
                    componentManifest->portManifests[i].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                }
            }
        }
    
        return XME_STATUS_SUCCESS;
    }

    xme_core_component_t invalidComponentId;
    xme_core_component_t componentId1;
    xme_core_component_t componentId2;
    xme_core_component_t componentId3;
    xme_core_component_t componentId4;
    xme_core_component_t componentId5;

    xme_core_node_nodeId_t invalidNodeId;
    xme_core_node_nodeId_t localNodeId;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeId3;

    xme_core_componentType_t invalidComponentType;
    xme_core_componentType_t componentTypeSensor;
    xme_core_componentType_t componentTypeMonitor;

    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph;
};

/******************************************************************************/
/***   Helper functions                                                     ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

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
//     PnPManager: instantiateComponentOnNode                                 //
//----------------------------------------------------------------------------//

TEST_F(PnPClientIntegrationTest, processGraphInvalidParameter)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpClient_processGraph(NULL));

    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph)); // N1 publication for N2.

    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION,
        xme_core_pnp_pnpClient_processGraph(outGraph));

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 1);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(outGraph));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph)); // N1 publication for N2.

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 2);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(outGraph));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph)); // N1 publication for N2.

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 3);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(outGraph));

    xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t) 1);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpClient_processGraph(outGraph));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
