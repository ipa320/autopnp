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
 * $Id: integrationTestPnPManager.cpp 5220 2013-09-30 09:34:21Z ruiz $
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
#include "xme/core/plugAndPlay/include/networkConfigurationCalculator.h"

#include "xme/defines.h"
#include "xme/core/node.h"
#include "xme/core/topic.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/random.h"
#include "xme/hal/include/time.h"

#include "xme/core/topicData.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PnPManagerIntegrationTest: public ::testing::Test
{
protected:
    PnPManagerIntegrationTest()
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

    virtual ~PnPManagerIntegrationTest()
    {
    }

    virtual void SetUp()
    { 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        guid1 = (xme_core_node_guid_t) xme_hal_random_rand();
        guid2 = (xme_core_node_guid_t) (((uint64_t)guid1) - 1);
        guid3 = (xme_core_node_guid_t) (((uint64_t)guid1) + 1);

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(localNodeId, "localNodeId", guid1));
        {
            xme_com_interface_address_t nodeInterface;
            
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &nodeInterface));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(localNodeId, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(localNodeId));


        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId2, "nodeId2", guid2));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33222", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId2, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId2));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId3, "nodeId3", guid3));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33223", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId3, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId3));

        outGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

        createComponentTypeManifests();

    }

    virtual void TearDown()
    {
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

    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;
    xme_core_node_guid_t guid3;

    xme_core_componentType_t invalidComponentType;
    xme_core_componentType_t componentTypeSensor;
    xme_core_componentType_t componentTypeMonitor;

    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph;
};

class PnPManagerLoginIntegrationTest: public ::testing::Test
{
    protected:
    // constructor
    PnPManagerLoginIntegrationTest()
        :nodeId1((xme_core_node_nodeId_t) 1)
        ,nodeId2((xme_core_node_nodeId_t) 2)
        ,nodeIdInvalid(XME_CORE_NODE_INVALID_NODE_ID)
        ,guid0(0)
        ,guid1(512)
        ,guid2(1024)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createSensorComponentTypeManifest());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createMonitorComponentTypeManifest());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());


        pnpLoginResponse1 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse2 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse3 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse4 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));

        pnpLoginResponse1->nodeId = nodeId1;
        pnpLoginResponse2->nodeId = nodeId1;
        pnpLoginResponse3->nodeId = nodeId2;
        pnpLoginResponse4->nodeId = XME_CORE_NODE_INVALID_NODE_ID;

        ip = "192.168.0.1:33331";
        port = 33331u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse1->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse1->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.2:33332";
        port = 33332u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse2->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse2->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.3:33333";
        port = 33333u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse3->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse3->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.4:33334";
        port = 33334u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse4->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse4->portAddress, &portNetworkByteOrder, 2);

        loginResponse = (xme_core_topic_login_loginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginResponse_t));
    }

    virtual ~PnPManagerLoginIntegrationTest()
    {
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();
    }

    xme_core_node_nodeId_t nodeId1;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeIdInvalid;

    xme_core_node_guid_t guid0;
    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;

    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse1;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse2;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse3;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse4;

    xme_core_topic_login_loginResponse_t* loginResponse;
    
    const char* ip;
    uint16_t port;
    uint32_t ipNetworkByteOrder;
    uint16_t portNetworkByteOrder;
    xme_com_interface_address_t interfaceAddress;
};

class PnPManagerIntegrationTestComponentInstanceManifestTests: public ::testing::Test
{
protected:
    PnPManagerIntegrationTestComponentInstanceManifestTests()
    {
        xme_core_pnp_pnpManager_init(NULL);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createSensorComponentTypeManifest());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createMonitorComponentTypeManifest());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        componentInstanceManifest1 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));
        componentInstanceManifest2 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));
        componentInstanceManifest3 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));
        componentInstanceManifest4 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));

        // Component Instance Manifest 1 (invalid node)
        componentInstanceManifest1->nodeId = XME_CORE_NODE_INVALID_NODE_ID;

        // Component Instance Manifest 2 (only node established)
        componentInstanceManifest2->nodeId = (xme_core_node_nodeId_t) 2;

        // Component Instance Manifest 3 (with one component)
        componentInstanceManifest3->nodeId = (xme_core_node_nodeId_t) 3;
        componentInstanceManifest3->components[0].componentId = (xme_core_component_t) 1;
        componentInstanceManifest3->components[0].componentType = XME_CORE_COMPONENT_TYPE_SENSOR;
        
        // Component Instance Manifest 4 (with two components)
        componentInstanceManifest4->nodeId = (xme_core_node_nodeId_t) 4;
        componentInstanceManifest4->components[0].componentId = (xme_core_component_t) 1;
        componentInstanceManifest4->components[0].componentType = XME_CORE_COMPONENT_TYPE_SENSOR;
        componentInstanceManifest4->components[1].componentId = (xme_core_component_t) 2;
        componentInstanceManifest4->components[1].componentType = XME_CORE_COMPONENT_TYPE_MONITOR;

        createComponentTypeManifests();

        invalidInterfaceAddress.addressType = XME_COM_INTERFACE_ADDRESS_TYPE_INVALID;

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &validInterfaceAddress));
    }

    virtual ~PnPManagerIntegrationTestComponentInstanceManifestTests()
    {
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();

        xme_hal_mem_free(componentInstanceManifest1);
        xme_hal_mem_free(componentInstanceManifest2);
        xme_hal_mem_free(componentInstanceManifest3);
        xme_hal_mem_free(componentInstanceManifest4);
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



    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest1;
    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest2;
    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest3;
    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest4;

    xme_com_interface_address_t invalidInterfaceAddress;
    xme_com_interface_address_t validInterfaceAddress;
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

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, invalidNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, invalidNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, invalidNodeId, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeValidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithTwoSensorsOfSameTypeOnTheSameNodeAndValidComponentId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId2));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithTwoSensorsOfSameTypeOnTheSameNodeAndInvalidComponentId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, invalidComponentId));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInTwoNodesAndInvalidComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId3, invalidComponentId));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInTwoNodesAndValidComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId3, componentId2));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInTwoNodesAndSameComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId3, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInSameNodeAndSameComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId2, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInLocalNodeAndSameComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithoutManifests)
{
    xme_core_manifestRepository_fini();
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, localNodeId, componentId1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, localNodeId, componentId1));
}

//----------------------------------------------------------------------------//
//     PnPManager: hasNextRuntimeGraph + getNextRuntimeGraph                  //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerIntegrationTest, hasNextRuntimeGraphTestPopulationWithComponentIdEstablished)
{
    // One sensor, two monitors. 
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, componentId1));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, nodeId2, componentId2));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, nodeId3, componentId3));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
}

TEST_F(PnPManagerIntegrationTest, hasNextRuntimeGraphTestPopulationWithComponentIdNotEstablished)
{
    // One monitor, two sensors
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, nodeId2, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId3, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
}

TEST_F(PnPManagerIntegrationTest, getNextRuntimeGraphTwoSensorsTwoMonitors)
{
    // One monitor, two sensors
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, nodeId3, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(invalidNodeId, outGraph));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph)); // N1 publication for N2 and N3.
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph)); // no more expected.

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph)); // N2 subscription for N1 & N2, N2 publication for N2 and N3.
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph)); // no more expected. 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph)); // N3 subscription for N1 & N2.
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph)); // no more expected.
}

//----------------------------------------------------------------------------//
//     PnPManager: updateInstanceFromRuntimeGraph                             //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerIntegrationTest, updateInstanceFromRuntimeGraphTestingOfSeveralSituations)
{
    // One monitor, two sensors
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, nodeId2, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(invalidNodeId, outGraph));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph)); // N1 publication for N2.

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_INTERNAL_ERROR));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_INTERNAL_ERROR));

    xme_core_pnp_pnpManager_fini();
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_INTERNAL_ERROR));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_INTERNAL_ERROR));
}

//----------------------------------------------------------------------------//
//     PnPManagerIntegrationTest: Processing Component Instance Manifest      //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processNullComponentInstanceManifest)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announceComponentInstanceManifest(NULL));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processInvalidNodeComponentInstanceManifest)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest1));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processValidNodeComponentInstanceManifestWithoutComponents)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest2));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processValidNodeComponentInstanceManifestWithOneComponentInstantiated)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest3));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processValidNodeComponentInstanceManifestWithTwoComponentsInstantiated)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest4));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest4));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processSeveralComponentInstanceManifests)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest2)); // without components
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest3)); // with one component
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest4)); // with two components
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode((xme_core_node_nodeId_t)2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode((xme_core_node_nodeId_t)3)); // the components are already running, so no new graphs to send. 
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode((xme_core_node_nodeId_t)4)); // the components are already running, so no new graphs to send. 
    // TODO: Check if this is still valid when announcing new component instance manifests. 
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, RegisterInterfaceAddressWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_registerInterfaceAddress(XME_CORE_NODE_INVALID_NODE_ID, invalidInterfaceAddress));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_registerInterfaceAddress(XME_CORE_NODE_INVALID_NODE_ID, validInterfaceAddress));
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)1, invalidInterfaceAddress));
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)2, validInterfaceAddress));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, RegisterInterfaceAddressWithoutNodeRegistrationAndWithNodeRegistration)
{
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)1, validInterfaceAddress));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode((xme_core_node_nodeId_t)1, "nodeId1", (xme_core_node_guid_t)1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)1, validInterfaceAddress));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
